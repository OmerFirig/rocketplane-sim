//! 6-DOF rigid-body simulation: quaternion attitude, RK4 integration,
//! variable mass / CG / inertia, landing gear, RCS, SAS, Dryden turbulence
//! and stagnation-point aerodynamic heating.
//!
//! World frame: +X east, +Y up, −Z north, flat ground at y = 0.

use crate::aero::{self, AeroInput, AeroOut, Controls};
use crate::aircraft::{mass_properties, AirframeConfig, GearLeg, MassProps, Strip};
use crate::atmosphere::{self, Air, G0, STEFAN_BOLTZMANN};
use crate::rocket::{EngineConfig, EngineState, NozzleDesign, RcsThruster};
use macroquad::math::{DQuat, DVec3};

const DEG: f64 = std::f64::consts::PI / 180.0;

#[derive(Clone, Copy, Debug, Default)]
pub struct PilotInput {
    pub pitch: f64, // +1 = nose up
    pub roll: f64,  // +1 = roll right
    pub yaw: f64,   // +1 = nose right
    pub throttle: f64,
    pub brake: f64,
    pub trim: f64,  // stabilator trim, rad
}

#[derive(Clone, Copy, Debug)]
struct Rb {
    pos: DVec3,
    vel: DVec3,
    att: DQuat,
    omega: DVec3,
}

#[derive(Clone, Copy)]
struct RbDot {
    dpos: DVec3,
    dvel: DVec3,
    datt: DQuat,
    domega: DVec3,
}

impl Rb {
    fn add(&self, d: &RbDot, h: f64) -> Rb {
        Rb {
            pos: self.pos + d.dpos * h,
            vel: self.vel + d.dvel * h,
            att: (self.att + d.datt * h).normalize(),
            omega: self.omega + d.domega * h,
        }
    }
}

#[derive(Clone, Debug, PartialEq)]
pub enum Status {
    Flying,
    Crashed(String),
}

#[derive(Clone, Copy, Debug, Default)]
pub struct Environment {
    pub wind_speed: f64,
    pub wind_from_deg: f64, // meteorological: direction the wind blows FROM
    pub turbulence: u8,     // 0 off, 1 light, 2 moderate, 3 severe
    pub dt_isa: f64,
}

struct Rng(u64);
impl Rng {
    fn next(&mut self) -> f64 {
        self.0 ^= self.0 << 13;
        self.0 ^= self.0 >> 7;
        self.0 ^= self.0 << 17;
        (self.0 >> 11) as f64 / (1u64 << 53) as f64
    }
    fn normal(&mut self) -> f64 {
        let u1 = self.next().max(1e-12);
        let u2 = self.next();
        (-2.0 * u1.ln()).sqrt() * (2.0 * std::f64::consts::PI * u2).cos()
    }
}

pub struct Sim {
    pub cfg: AirframeConfig,
    pub eng_cfg: EngineConfig,
    pub nozzle: NozzleDesign,
    pub strips: Vec<Strip>,
    pub gear: Vec<GearLeg>,
    pub hard: Vec<DVec3>,
    pub env: Environment,

    pub pos: DVec3,
    pub vel: DVec3,
    pub att: DQuat,
    pub omega: DVec3,

    pub fuel: f64,
    pub ox: f64,
    pub rcs_prop: f64,
    pub mp: MassProps,
    pub engine: EngineState,
    pub input: PilotInput,
    pub ctrl: Controls,
    pub gimbal: (f64, f64),
    pub flap_cmd: f64,
    pub gear_cmd: bool,
    pub speedbrake_cmd: bool,
    pub rcs_on: bool,
    pub sas_on: bool,
    pub rcs_firing: [i8; 3], // pitch, yaw, roll: −1/0/+1

    downwash: f64,
    gust: DVec3,
    rng: Rng,

    pub time: f64,
    pub air: Air,
    pub aero: AeroOut,
    pub thrust_body: DVec3,
    pub nz: f64,
    pub ny: f64,
    pub gear_stroke: Vec<f64>,
    pub on_ground: bool,
    pub wind_total: DVec3,

    pub stag_temp: f64,
    pub heat_flux: f64,
    pub skin_temp: f64,

    pub status: Status,
    pub max_mach: f64,
    pub max_alt: f64,
    pub max_q: f64,
    pub max_nz: f64,
    pub max_skin: f64,
    pub dv_spent: f64,
}

impl Sim {
    pub fn new(cfg: AirframeConfig, eng_cfg: EngineConfig, env: Environment) -> Sim {
        let nozzle = eng_cfg.design();
        let strips = cfg.build_strips();
        let gear = cfg.gear();
        let hard = cfg.hard_points();
        let ngear = gear.len();
        let mut s = Sim {
            mp: mass_properties(&cfg, 0.0, 0.0, 0.0),
            cfg, eng_cfg, nozzle, strips, gear, hard, env,
            pos: DVec3::ZERO,
            vel: DVec3::ZERO,
            att: DQuat::IDENTITY,
            omega: DVec3::ZERO,
            fuel: 0.0, ox: 0.0, rcs_prop: 0.0,
            engine: EngineState::default(),
            input: PilotInput::default(),
            ctrl: Controls::default(),
            gimbal: (0.0, 0.0),
            flap_cmd: 0.0,
            gear_cmd: true,
            speedbrake_cmd: false,
            rcs_on: false,
            sas_on: true,
            rcs_firing: [0; 3],
            downwash: 0.0,
            gust: DVec3::ZERO,
            rng: Rng(0x9E37_79B9_7F4A_7C15),
            time: 0.0,
            air: atmosphere::isa(0.0, 0.0),
            aero: AeroOut::default(),
            thrust_body: DVec3::ZERO,
            nz: 1.0, ny: 0.0,
            gear_stroke: vec![0.0; ngear],
            on_ground: false,
            wind_total: DVec3::ZERO,
            stag_temp: 288.0, heat_flux: 0.0, skin_temp: 288.0,
            status: Status::Flying,
            max_mach: 0.0, max_alt: 0.0, max_q: 0.0, max_nz: 0.0, max_skin: 0.0, dv_spent: 0.0,
        };
        s.refuel();
        s
    }

    /// Fill the tanks at the propellant's O/F ratio, limited by tank volume.
    pub fn refuel(&mut self) {
        let p = self.eng_cfg.propellant();
        let load = self.cfg.prop_load;
        if p.solid {
            // the grain fills both tank bays with ~78 % volumetric loading
            self.fuel = (self.cfg.fuel_volume + self.cfg.ox_volume) * p.rho_fuel * 0.78 * load;
            self.ox = 0.0;
        } else {
            let fuel_max = self.cfg.fuel_volume * p.rho_fuel;
            let fuel_by_ox = self.cfg.ox_volume * p.rho_ox / p.of_ratio;
            self.fuel = fuel_max.min(fuel_by_ox) * load;
            self.ox = self.fuel * p.of_ratio;
        }
        self.rcs_prop = self.cfg.rcs_capacity;
        self.engine.flameout_reason = None;
        self.mp = mass_properties(&self.cfg, self.fuel, self.ox, self.rcs_prop);
    }

    /// Rebuild derived data after the sandbox panel changed a parameter.
    pub fn reconfigure(&mut self) {
        let old_cg = self.mp.cg;
        self.nozzle = self.eng_cfg.design();
        self.strips = self.cfg.build_strips();
        self.gear = self.cfg.gear();
        self.hard = self.cfg.hard_points();
        self.mp = mass_properties(&self.cfg, self.fuel, self.ox, self.rcs_prop);
        self.pos += self.att * (self.mp.cg - old_cg);
    }

    pub fn propellant_mass(&self) -> f64 {
        self.fuel + self.ox
    }

    /// World position of a body-frame point.
    pub fn point_world(&self, p: DVec3) -> DVec3 {
        self.pos + self.att * (p - self.mp.cg)
    }

    pub fn euler(&self) -> (f64, f64, f64) {
        let fwd = self.att * DVec3::X;
        let up = self.att * DVec3::Y;
        let pitch = fwd.y.clamp(-1.0, 1.0).asin();
        let heading = fwd.x.atan2(-fwd.z).rem_euclid(2.0 * std::f64::consts::PI);
        // roll: angle of the body-up vector around the forward axis
        let right_flat = DVec3::new(-fwd.z, 0.0, fwd.x).normalize_or_zero();
        let up_flat = right_flat.cross(fwd).normalize_or_zero();
        let roll = up.dot(right_flat).atan2(up.dot(up_flat));
        (pitch, roll, heading)
    }

    pub fn toggle_engine(&mut self) {
        if self.engine.running {
            if self.eng_cfg.propellant().solid {
                return; // a solid motor cannot be shut down
            }
            self.engine.running = false;
        } else if self.propellant_mass() > 1.0 {
            self.engine.running = true;
            self.engine.flameout_reason = None;
        }
    }

    fn wind_mean(&self) -> DVec3 {
        let from = self.env.wind_from_deg * DEG;
        // blows from heading `from` toward the opposite direction
        let to = DVec3::new(-from.sin(), 0.0, from.cos());
        // boundary-layer wind profile (1/7 power law below 300 m)
        let h = self.pos.y.max(0.5);
        let shear = (h / 10.0).powf(1.0 / 7.0).min(1.7);
        to * self.env.wind_speed * shear
    }

    fn update_turbulence(&mut self, dt: f64) {
        let sigma0 = [0.0, 1.5, 3.0, 6.5][self.env.turbulence.min(3) as usize];
        if sigma0 == 0.0 {
            self.gust = DVec3::ZERO;
            return;
        }
        let h = self.pos.y.max(10.0);
        let sigma = sigma0 * (1.0 - 0.85 * aero::smoothstep(2000.0, 15000.0, h));
        // Dryden scale lengths (MIL-F-8785C), low-altitude model in feet
        let (lu, lw) = if h < 305.0 {
            let hf = h / 0.3048;
            (hf / (0.177 + 0.000823 * hf).powf(1.2) * 0.3048, h)
        } else {
            (533.0, 533.0)
        };
        let v = self.vel.length().max(5.0);
        let filt = |x: f64, l: f64, rng: &mut Rng| {
            let a = (-v * dt / l).exp();
            x * a + sigma * (1.0 - a * a).sqrt() * rng.normal()
        };
        self.gust = DVec3::new(
            filt(self.gust.x, lu, &mut self.rng),
            filt(self.gust.y, lw, &mut self.rng),
            filt(self.gust.z, lu, &mut self.rng),
        );
    }

    fn update_actuators(&mut self, dt: f64) {
        let q = self.aero.qbar.max(1.0);
        let sched = (15_000.0 / q).clamp(0.15, 3.0);
        let (mut p, mut r, mut y) = (self.input.pitch, self.input.roll, self.input.yaw);
        if self.sas_on {
            p -= 0.9 * self.omega.z * sched;
            r -= 0.35 * self.omega.x * sched;
            y += 1.2 * self.omega.y * sched;
        }
        let (p, r, y) = (p.clamp(-1.0, 1.0), r.clamp(-1.0, 1.0), y.clamp(-1.0, 1.0));

        let sym = self.input.trim - p * 22.0 * DEG;
        let diff = r * 12.0 * DEG;
        let lim = 35.0 * DEG;
        let rate = |cur: f64, tgt: f64, r: f64| cur + (tgt - cur).clamp(-r * dt, r * dt);
        self.ctrl.stab_left = rate(self.ctrl.stab_left, (sym + diff).clamp(-lim, lim), 30.0 * DEG);
        self.ctrl.stab_right = rate(self.ctrl.stab_right, (sym - diff).clamp(-lim, lim), 30.0 * DEG);
        self.ctrl.rudder = rate(self.ctrl.rudder, y * 15.0 * DEG, 30.0 * DEG);
        self.ctrl.flap = rate(self.ctrl.flap, self.flap_cmd, 6.0 * DEG);
        self.ctrl.gear_down = rate(self.ctrl.gear_down, if self.gear_cmd { 1.0 } else { 0.0 }, 0.25);
        self.ctrl.speedbrake = rate(self.ctrl.speedbrake, if self.speedbrake_cmd { 1.0 } else { 0.0 }, 0.6);

        let gmax = self.eng_cfg.gimbal_max_deg * DEG;
        self.gimbal = (
            rate(self.gimbal.0, p * gmax, 20.0 * DEG),
            rate(self.gimbal.1, y * gmax, 20.0 * DEG),
        );

        let fire = |c: f64| if c > 0.15 { 1 } else if c < -0.15 { -1 } else { 0 };
        self.rcs_firing = if self.rcs_on && self.rcs_prop > 0.0 { [fire(p), fire(y), fire(r)] } else { [0; 3] };
    }

    /// Forces/moments other than aerodynamics that are constant over a step.
    fn propulsive(&self) -> (DVec3, DVec3) {
        let mut f = DVec3::ZERO;
        let mut m = DVec3::ZERO;
        let t = self.engine.out.thrust;
        if t > 0.0 {
            let (gp, gy) = self.gimbal;
            let dir = DVec3::new(1.0, -gp.sin(), -gy.sin()).normalize();
            let ft = dir * t;
            f += ft;
            m += (self.cfg.nozzle_pivot - self.mp.cg).cross(ft);
        }
        let rcs = RcsThruster::H2O2;
        let ft = rcs.thrust(self.air.pressure);
        let nose = DVec3::new(self.cfg.nose_x - 1.2, 0.0, 0.0) - self.mp.cg;
        let tip = self.cfg.wing.span() * 0.45;
        let [rp, ry, rr] = self.rcs_firing;
        if rp != 0 {
            let fv = DVec3::Y * (ft * rp as f64);
            f += fv;
            m += nose.cross(fv);
        }
        if ry != 0 {
            let fv = DVec3::Z * (ft * ry as f64);
            f += fv;
            m += nose.cross(fv);
        }
        if rr != 0 {
            // pure couple from the two opposed wing-tip thrusters
            m += DVec3::X * (2.0 * ft * tip * rr as f64);
        }
        (f, m)
    }

    fn derivs(&mut self, s: &Rb, prop: (DVec3, DVec3), record: bool) -> RbDot {
        let inv = s.att.inverse();
        let wind = self.wind_total;
        let v_air_body = inv * (s.vel - wind);

        let mut height = s.pos.y + self.cfg.wing.y_root - self.mp.cg.y;
        height = height.max(0.05);
        let aero_out = aero::evaluate(&AeroInput {
            cfg: &self.cfg,
            strips: &self.strips,
            air: &self.air,
            v_air: v_air_body,
            omega: s.omega,
            cg: self.mp.cg,
            ctrl: self.ctrl,
            downwash: self.downwash,
            height,
            engine_on: self.engine.out.thrust > 1.0,
            nozzle_exit_area: self.nozzle.exit_area,
            record,
        });

        let mut f_body = aero_out.force + prop.0;
        let mut m_body = aero_out.moment + prop.1;

        // landing gear
        let mut on_ground = false;
        if self.ctrl.gear_down > 0.98 {
            for (i, g) in self.gear.iter().enumerate() {
                let r_b = g.attach - self.mp.cg;
                let p = s.pos + s.att * r_b;
                let pen = -p.y;
                if record {
                    self.gear_stroke[i] = pen.max(0.0);
                }
                if pen <= 0.0 {
                    continue;
                }
                on_ground = true;
                let v_pt = s.vel + s.att * s.omega.cross(r_b);
                let n = (g.k * pen - g.c * v_pt.y).max(0.0);
                let fwd_b = if g.steerable {
                    let steer = self.input.yaw * 0.3 * (1.0 - aero::smoothstep(10.0, 40.0, s.vel.length()));
                    DVec3::new(steer.cos(), 0.0, steer.sin())
                } else {
                    DVec3::X
                };
                let mut fwd = s.att * fwd_b;
                fwd.y = 0.0;
                let fwd = fwd.normalize_or_zero();
                let lat = DVec3::new(-fwd.z, 0.0, fwd.x);
                let v_long = v_pt.dot(fwd);
                let v_lat = v_pt.dot(lat);
                let mu_long = 0.015 + if g.braked { 0.55 * self.input.brake } else { 0.0 };
                let f_w = DVec3::Y * n
                    - fwd * (mu_long * n * (v_long / 0.4).tanh())
                    - lat * (0.8 * n * (v_lat / 0.2).tanh());
                let f_b = inv * f_w;
                f_body += f_b;
                m_body += r_b.cross(f_b);
            }
        }

        let g = atmosphere::gravity(s.pos.y);
        let accel_world = s.att * (f_body / self.mp.mass) - DVec3::Y * g;

        let i = self.mp.inertia;
        let h = i * s.omega;
        let domega = self.mp.inv_inertia * (m_body - s.omega.cross(h));
        let w = DQuat::from_xyzw(s.omega.x, s.omega.y, s.omega.z, 0.0);

        if record {
            let spec = f_body / self.mp.mass;
            self.nz = spec.y / G0;
            self.ny = spec.z / G0;
            self.on_ground = on_ground;
            self.aero = aero_out;
        }
        RbDot { dpos: s.vel, dvel: accel_world, datt: s.att * w * 0.5, domega }
    }

    pub fn step(&mut self, dt: f64) {
        if self.status != Status::Flying {
            return;
        }
        self.time += dt;
        self.update_actuators(dt);
        self.air = atmosphere::isa(self.pos.y, self.env.dt_isa);
        self.update_turbulence(dt);
        self.wind_total = self.wind_mean() + self.gust;

        // engine & propellant flow
        self.engine.throttle = self.input.throttle;
        self.engine.step(&self.eng_cfg, &self.nozzle, self.air.pressure, dt);
        let used = self.engine.out.mdot * dt;
        if used > 0.0 {
            let p = self.eng_cfg.propellant();
            if p.solid {
                self.fuel -= used;
            } else {
                self.fuel -= used / (1.0 + p.of_ratio);
                self.ox -= used * p.of_ratio / (1.0 + p.of_ratio);
            }
            self.dv_spent += self.engine.out.thrust / self.mp.mass * dt;
            if self.fuel <= 0.0 || (!p.solid && self.ox <= 0.0) {
                self.fuel = self.fuel.max(0.0);
                self.ox = self.ox.max(0.0);
                if self.engine.running {
                    self.engine.running = false;
                    self.engine.flameout_reason = Some("PROPELLANT DEPLETION");
                }
            }
        }
        let rcs_count = self.rcs_firing.iter().filter(|&&f| f != 0).count() as f64;
        self.rcs_prop = (self.rcs_prop - rcs_count * RcsThruster::H2O2.mdot() * dt).max(0.0);

        // variable mass: keep the body reference point fixed while the CG moves
        let old_cg = self.mp.cg;
        self.mp = mass_properties(&self.cfg, self.fuel, self.ox, self.rcs_prop);
        self.pos += self.att * (self.mp.cg - old_cg);

        self.thrust_body = if self.engine.out.thrust > 0.0 {
            DVec3::new(1.0, -self.gimbal.0.sin(), -self.gimbal.1.sin()).normalize() * self.engine.out.thrust
        } else {
            DVec3::ZERO
        };
        let prop = self.propulsive();

        // RK4
        let s0 = Rb { pos: self.pos, vel: self.vel, att: self.att, omega: self.omega };
        let k1 = self.derivs(&s0, prop, true);
        let k2 = self.derivs(&s0.add(&k1, dt * 0.5), prop, false);
        let k3 = self.derivs(&s0.add(&k2, dt * 0.5), prop, false);
        let k4 = self.derivs(&s0.add(&k3, dt), prop, false);
        let comb = |a: DVec3, b: DVec3, c: DVec3, d: DVec3| (a + b * 2.0 + c * 2.0 + d) * (dt / 6.0);
        self.pos += comb(k1.dpos, k2.dpos, k3.dpos, k4.dpos);
        self.vel += comb(k1.dvel, k2.dvel, k3.dvel, k4.dvel);
        self.omega += comb(k1.domega, k2.domega, k3.domega, k4.domega);
        let datt = (k1.datt + k2.datt * 2.0 + k3.datt * 2.0 + k4.datt) * (dt / 6.0);
        self.att = (self.att + datt).normalize();

        // downwash reaches the tail after the convection time l_t / V
        let a = &self.aero;
        let dw_target = self.cfg.downwash_k * 2.0 * a.cl_wing / (std::f64::consts::PI * self.cfg.wing.aspect)
            * (1.0 - 0.6 * aero::smoothstep(0.95, 1.4, a.mach));
        let lt = self.cfg.wing.x_root_le - self.cfg.htail.x_root_le;
        let tau = lt / a.tas.max(5.0);
        self.downwash += (dw_target - self.downwash) * (dt / tau).min(1.0);

        self.update_heating(dt);
        self.check_limits();
    }

    fn update_heating(&mut self, dt: f64) {
        let v = self.aero.tas;
        let m = self.aero.mach;
        let t_inf = self.air.temperature;
        self.stag_temp = t_inf * (1.0 + 0.2 * m * m);
        // Sutton–Graves stagnation-point convective flux, nose radius 0.3 m
        let rn = 0.3;
        let q_cold = 1.7415e-4 * (self.air.density / rn).sqrt() * v * v * v;
        let drive = (self.stag_temp - self.skin_temp) / (self.stag_temp - t_inf).max(1.0);
        let q_conv = if self.stag_temp > t_inf + 1.0 { q_cold * drive } else { 0.0 };
        let q_nat = 10.0 * (t_inf - self.skin_temp);
        let q_rad = 0.8 * STEFAN_BOLTZMANN * (self.skin_temp.powi(4) - t_inf.min(250.0).powi(4));
        self.heat_flux = q_conv;
        // Inconel-X skin: ρ 8250 kg/m³, c 460 J/kg·K, 2 mm thick
        let c_areal = 8250.0 * 460.0 * 0.002;
        self.skin_temp += (q_conv + q_nat - q_rad) / c_areal * dt;
    }

    fn check_limits(&mut self) {
        let a = &self.aero;
        self.max_mach = self.max_mach.max(a.mach);
        self.max_alt = self.max_alt.max(self.pos.y);
        self.max_q = self.max_q.max(a.qbar);
        self.max_nz = self.max_nz.max(self.nz.abs());
        self.max_skin = self.max_skin.max(self.skin_temp);
        let qbar = a.qbar;

        let sink = -self.vel.y;
        let why = if self.hard.iter().any(|hp| self.point_world(*hp).y < 0.0) {
            Some(format!("GROUND IMPACT  (sink rate {:.1} m/s, speed {:.0} m/s)", sink, self.vel.length()))
        } else if self.ctrl.gear_down > 0.98
            && self.gear.iter().zip(&self.gear_stroke).any(|(g, &st)| st > g.max_stroke + 0.08)
        {
            Some(format!("GEAR COLLAPSE  (sink rate {:.1} m/s)", sink))
        } else if self.nz > 11.0 || self.nz < -5.5 {
            Some(format!("STRUCTURAL FAILURE: load factor {:.1} g", self.nz))
        } else if qbar > 160_000.0 {
            Some(format!("STRUCTURAL FAILURE: dynamic pressure {:.0} kPa", qbar / 1000.0))
        } else if self.skin_temp > 1500.0 {
            Some(format!("THERMAL FAILURE: skin {:.0} K", self.skin_temp))
        } else {
            None
        };
        if let Some(why) = why {
            self.crash(why);
        }
    }

    fn crash(&mut self, why: String) {
        self.status = Status::Crashed(why);
        self.engine.running = false;
    }

    /// Quasi-steady aerodynamics at a given α and symmetric stabilator, with
    /// the downwash iterated to equilibrium. Returns (lift, pitch moment, CL_wing).
    pub fn eval_static(&self, air: &Air, speed: f64, alpha: f64, stab: f64) -> (f64, f64, f64) {
        let v = DVec3::new(alpha.cos(), -alpha.sin(), 0.0) * speed;
        let mut c = self.ctrl;
        c.stab_left = stab;
        c.stab_right = stab;
        c.rudder = 0.0;
        let mut cl = 0.0;
        let mut out = AeroOut::default();
        for _ in 0..5 {
            let dw = self.cfg.downwash_k * 2.0 * cl / (std::f64::consts::PI * self.cfg.wing.aspect)
                * (1.0 - 0.6 * aero::smoothstep(0.95, 1.4, speed / air.speed_of_sound));
            out = aero::evaluate(&AeroInput {
                cfg: &self.cfg, strips: &self.strips, air, v_air: v, omega: DVec3::ZERO,
                cg: self.mp.cg, ctrl: c, downwash: dw, height: 1e4, engine_on: false,
                nozzle_exit_area: self.nozzle.exit_area, record: false,
            });
            cl = out.cl_wing;
        }
        (out.lift, out.moment.z, cl)
    }

    /// Static margin (fraction of MAC; positive = stable) and neutral point x.
    pub fn static_margin(&self, alt: f64, mach: f64) -> (f64, f64) {
        let air = atmosphere::isa(alt, self.env.dt_isa);
        let v = mach * air.speed_of_sound;
        let (l1, m1, _) = self.eval_static(&air, v, 1.0 * DEG, 0.0);
        let (l2, m2, _) = self.eval_static(&air, v, 3.0 * DEG, 0.0);
        let dx = (m2 - m1) / (l2 - l1); // x_ac − x_cg
        let mac = self.cfg.wing.mac();
        (-dx / mac, self.mp.cg.x + dx)
    }

    /// Solve for the angle of attack and stabilator trim that give 1 g level
    /// flight (lift = weight, pitching moment = 0) at the current state.
    pub fn trim_level(&mut self) {
        let speed = self.vel.length();
        if speed < 30.0 {
            return;
        }
        let (_, _, heading) = self.euler();
        let air = atmosphere::isa(self.pos.y, self.env.dt_isa);
        let weight = self.mp.mass * atmosphere::gravity(self.pos.y);
        let eval = |sim: &Sim, alpha: f64, stab: f64| sim.eval_static(&air, speed, alpha, stab);
        let mut alpha = 3.0 * DEG;
        let mut stab = 0.0;
        for _ in 0..30 {
            // inner: stabilator for zero pitching moment (bisection)
            let (mut lo, mut hi) = (-30.0 * DEG, 20.0 * DEG);
            for _ in 0..40 {
                let mid = 0.5 * (lo + hi);
                let (_, m, _) = eval(self, alpha, mid);
                // more stab (LE up) → more tail lift → nose-down moment (−z)
                if m > 0.0 { lo = mid } else { hi = mid }
            }
            stab = 0.5 * (lo + hi);
            let (l, _, _) = eval(self, alpha, stab);
            let (l2, _, _) = eval(self, alpha + 0.2 * DEG, stab);
            let slope = (l2 - l) / (0.2 * DEG);
            if slope.abs() < 1.0 {
                break;
            }
            let da = ((weight - l) / slope).clamp(-3.0 * DEG, 3.0 * DEG);
            alpha = (alpha + da).clamp(-10.0 * DEG, 25.0 * DEG);
            if da.abs() < 1e-5 {
                break;
            }
        }
        let (_, _, cl) = eval(self, alpha, stab);
        self.downwash = self.cfg.downwash_k * 2.0 * cl / (std::f64::consts::PI * self.cfg.wing.aspect);
        self.input.trim = stab;
        self.ctrl.stab_left = stab;
        self.ctrl.stab_right = stab;
        // level flight: velocity horizontal, nose pitched up by alpha
        let vdir = DVec3::new(heading.sin(), 0.0, -heading.cos());
        self.vel = vdir * speed;
        self.att = DQuat::from_rotation_y(std::f64::consts::FRAC_PI_2 - heading) * DQuat::from_rotation_z(alpha);
    }
}

#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum Scenario {
    Runway,
    AirLaunch,
    Hypersonic,
    HighAltitude,
    Approach,
}

impl Scenario {
    pub const ALL: [Scenario; 5] = [
        Scenario::Runway, Scenario::AirLaunch, Scenario::Hypersonic, Scenario::HighAltitude, Scenario::Approach,
    ];
    pub fn name(&self) -> &'static str {
        match self {
            Scenario::Runway => "Runway start",
            Scenario::AirLaunch => "B-52 air launch, 13.7 km, M0.8",
            Scenario::Hypersonic => "Hypersonic run, 30 km, M5",
            Scenario::HighAltitude => "Space shot, 60 km climbing 35°",
            Scenario::Approach => "Unpowered approach, 4 km, 15 km out",
        }
    }
}

pub fn spawn(sim: &mut Sim, sc: Scenario) {
    sim.status = Status::Flying;
    sim.omega = DVec3::ZERO;
    sim.engine = EngineState::default();
    sim.input = PilotInput::default();
    sim.ctrl = Controls::default();
    sim.time = 0.0;
    sim.gust = DVec3::ZERO;
    sim.skin_temp = 288.0;
    sim.downwash = 0.0;
    sim.dv_spent = 0.0;
    sim.max_mach = 0.0;
    sim.max_alt = 0.0;
    sim.max_q = 0.0;
    sim.max_nz = 0.0;
    sim.max_skin = 0.0;
    sim.flap_cmd = 0.0;
    sim.speedbrake_cmd = false;
    sim.refuel();
    let north = DQuat::from_rotation_y(std::f64::consts::FRAC_PI_2);
    let set_flight = |sim: &mut Sim, alt: f64, speed: f64, heading_pos: DVec3| {
        sim.gear_cmd = false;
        sim.ctrl.gear_down = 0.0;
        sim.pos = heading_pos + DVec3::Y * alt;
        sim.att = north;
        sim.vel = DVec3::new(0.0, 0.0, -speed);
        sim.trim_level();
    };
    match sc {
        Scenario::Runway => {
            sim.gear_cmd = true;
            sim.ctrl.gear_down = 1.0;
            sim.att = north;
            sim.vel = DVec3::ZERO;
            sim.pos = DVec3::new(0.0, 1.6 + sim.mp.cg.y - 0.12, -150.0);
            sim.input.brake = 0.0;
            sim.flap_cmd = 20.0 * DEG;
            sim.ctrl.flap = sim.flap_cmd;
        }
        Scenario::AirLaunch => {
            let a = atmosphere::isa(13_700.0, sim.env.dt_isa).speed_of_sound;
            set_flight(sim, 13_700.0, 0.8 * a, DVec3::new(0.0, 0.0, 60_000.0));
        }
        Scenario::Hypersonic => {
            sim.fuel *= 0.5;
            sim.ox *= 0.5;
            sim.mp = mass_properties(&sim.cfg, sim.fuel, sim.ox, sim.rcs_prop);
            let a = atmosphere::isa(30_000.0, sim.env.dt_isa).speed_of_sound;
            set_flight(sim, 30_000.0, 5.0 * a, DVec3::new(0.0, 0.0, 250_000.0));
        }
        Scenario::HighAltitude => {
            sim.fuel = 0.0;
            sim.ox = 0.0;
            sim.mp = mass_properties(&sim.cfg, sim.fuel, sim.ox, sim.rcs_prop);
            sim.gear_cmd = false;
            sim.ctrl.gear_down = 0.0;
            sim.rcs_on = true;
            sim.pos = DVec3::new(0.0, 60_000.0, 200_000.0);
            let g = 35.0 * DEG;
            sim.att = north * DQuat::from_rotation_z(g);
            sim.vel = DVec3::new(0.0, g.sin(), -g.cos()) * 1500.0;
        }
        Scenario::Approach => {
            sim.fuel = 0.0;
            sim.ox = 0.0;
            sim.mp = mass_properties(&sim.cfg, sim.fuel, sim.ox, sim.rcs_prop);
            set_flight(sim, 4_000.0, 170.0, DVec3::new(0.0, 0.0, 15_000.0));
        }
    }
    sim.air = atmosphere::isa(sim.pos.y, sim.env.dt_isa);
}
