mod aero;
mod aircraft;
mod atmosphere;
mod hud;
mod render;
mod rocket;
mod selftest;
mod sim;

use aircraft::AirframeConfig;
use hud::{kv, kvc, panel, Row, Ui, ALERT, DIM, FG, GOOD, HEAD, WARN};
use macroquad::prelude::*;
use render::View;
use rocket::{EngineConfig, PROPELLANTS};
use sim::{spawn, Environment, Scenario, Sim, Status};
use std::io::Write as _;

const DT: f64 = 1.0 / 1000.0;
const DEG: f64 = std::f64::consts::PI / 180.0;

struct Param {
    name: &'static str,
    unit: &'static str,
    step: f64,
    min: f64,
    max: f64,
    get: fn(&Sim) -> f64,
    set: fn(&mut Sim, f64),
}

fn params() -> Vec<Param> {
    macro_rules! p {
        ($n:expr, $u:expr, $st:expr, $lo:expr, $hi:expr, |$s:ident| $g:expr, |$s2:ident, $v:ident| $body:expr) => {
            Param { name: $n, unit: $u, step: $st, min: $lo, max: $hi, get: |$s: &Sim| $g, set: |$s2: &mut Sim, $v: f64| { $body; } }
        };
    }
    vec![
        p!("Propellant", "", 1.0, 0.0, (PROPELLANTS.len() - 1) as f64, |s| s.eng_cfg.prop as f64,
            |s, v| if !s.engine.running { s.eng_cfg.prop = v as usize; s.reconfigure(); s.refuel(); }),
        p!("Chamber pressure", "MPa", 0.2, 0.5, 25.0, |s| s.eng_cfg.pc_nominal / 1e6, |s, v| { s.eng_cfg.pc_nominal = v * 1e6; s.reconfigure() }),
        p!("Nozzle area ratio", "", 0.5, 1.5, 200.0, |s| s.eng_cfg.area_ratio, |s, v| { s.eng_cfg.area_ratio = v; s.reconfigure() }),
        p!("Throat diameter", "m", 0.005, 0.03, 0.8, |s| s.eng_cfg.throat_diam, |s, v| { s.eng_cfg.throat_diam = v; s.reconfigure() }),
        p!("Nozzle half-angle", "°", 1.0, 0.0, 35.0, |s| s.eng_cfg.half_angle_deg, |s, v| { s.eng_cfg.half_angle_deg = v; s.reconfigure() }),
        p!("Min throttle", "%", 5.0, 5.0, 100.0, |s| s.eng_cfg.min_throttle * 100.0, |s, v| s.eng_cfg.min_throttle = v / 100.0),
        p!("TVC gimbal limit", "°", 0.5, 0.0, 12.0, |s| s.eng_cfg.gimbal_max_deg, |s, v| s.eng_cfg.gimbal_max_deg = v),
        p!("Propellant load", "%", 5.0, 0.0, 100.0, |s| s.cfg.prop_load * 100.0,
            |s, v| if !s.engine.running { s.cfg.prop_load = v / 100.0; s.refuel(); s.reconfigure(); }),
        p!("Dry mass", "kg", 100.0, 1500.0, 30000.0, |s| s.cfg.dry_mass, |s, v| { s.cfg.dry_mass = v; s.reconfigure() }),
        p!("Dry CG (ballast)", "m", 0.05, -2.0, 1.5, |s| s.cfg.dry_cg.x, |s, v| { s.cfg.dry_cg.x = v; s.reconfigure() }),
        p!("Wing area", "m²", 0.5, 6.0, 60.0, |s| s.cfg.wing.area, |s, v| { s.cfg.wing.area = v; s.reconfigure() }),
        p!("Wing aspect ratio", "", 0.1, 0.8, 12.0, |s| s.cfg.wing.aspect, |s, v| { s.cfg.wing.aspect = v; s.reconfigure() }),
        p!("Wing LE sweep", "°", 1.0, 0.0, 75.0, |s| s.cfg.wing.sweep_le_deg, |s, v| { s.cfg.wing.sweep_le_deg = v; s.reconfigure() }),
        p!("Wing t/c", "%", 0.5, 2.0, 18.0, |s| s.cfg.wing.tc * 100.0, |s, v| { s.cfg.wing.tc = v / 100.0; s.reconfigure() }),
        p!("Wing dihedral", "°", 1.0, -15.0, 15.0, |s| s.cfg.wing.dihedral_deg, |s, v| { s.cfg.wing.dihedral_deg = v; s.reconfigure() }),
        p!("Wing position (root LE)", "m", 0.05, -2.0, 4.0, |s| s.cfg.wing.x_root_le, |s, v| { s.cfg.wing.x_root_le = v; s.reconfigure() }),
        p!("H-tail area", "m²", 0.2, 0.5, 15.0, |s| s.cfg.htail.area, |s, v| { s.cfg.htail.area = v; s.reconfigure() }),
        p!("V-tail area", "m²", 0.2, 0.5, 12.0, |s| s.cfg.vtail.area, |s, v| { s.cfg.vtail.area = v; s.reconfigure() }),
        p!("Wind speed", "m/s", 1.0, 0.0, 50.0, |s| s.env.wind_speed, |s, v| s.env.wind_speed = v),
        p!("Wind from", "°", 10.0, 0.0, 350.0, |s| s.env.wind_from_deg, |s, v| s.env.wind_from_deg = v),
        p!("Turbulence (Dryden)", "lvl", 1.0, 0.0, 3.0, |s| s.env.turbulence as f64, |s, v| s.env.turbulence = v as u8),
        p!("ISA temperature offset", "K", 1.0, -40.0, 40.0, |s| s.env.dt_isa, |s, v| s.env.dt_isa = v),
    ]
}

struct App {
    sim: Sim,
    view: View,
    scenario: Scenario,
    paused: bool,
    time_scale: f64,
    acc: f64,
    hist: hud::History,
    show_help: bool,
    show_panel: bool,
    sel: usize,
    params: Vec<Param>,
    static_margin: (f64, f64),
    sm_timer: f32,
    log: Option<std::io::BufWriter<std::fs::File>>,
    log_name: String,
    log_last: f64,
    last_mouse: Vec2,
    msg: Option<(String, f32)>,
}

impl App {
    fn new() -> App {
        let mut sim = Sim::new(AirframeConfig::default(), EngineConfig::default(), Environment::default());
        let scenario = Scenario::AirLaunch;
        spawn(&mut sim, scenario);
        App {
            sim,
            view: View::new(),
            scenario,
            paused: false,
            time_scale: 1.0,
            acc: 0.0,
            hist: hud::History::new(),
            show_help: true,
            show_panel: false,
            sel: 0,
            params: params(),
            static_margin: (0.0, 0.0),
            sm_timer: 0.0,
            log: None,
            log_name: String::new(),
            log_last: -1.0,
            last_mouse: Vec2::ZERO,
            msg: None,
        }
    }

    fn restart(&mut self, sc: Scenario) {
        self.scenario = sc;
        spawn(&mut self.sim, sc);
        self.hist.clear();
        self.acc = 0.0;
        self.log_last = -1.0;
        self.flash(format!("Scenario: {}", sc.name()));
    }

    fn flash(&mut self, s: String) {
        self.msg = Some((s, 3.0));
    }

    fn input(&mut self, dt: f32) {
        let s = &mut self.sim;
        let k = |c: KeyCode| if is_key_down(c) { 1.0 } else { 0.0 };
        let ramp = |cur: f64, tgt: f64, rate: f64| cur + (tgt - cur).clamp(-rate * dt as f64, rate * dt as f64);
        let (tp, tr, ty) = (k(KeyCode::S) - k(KeyCode::W), k(KeyCode::D) - k(KeyCode::A), k(KeyCode::E) - k(KeyCode::Q));
        s.input.pitch = ramp(s.input.pitch, tp, if tp == 0.0 { 6.0 } else { 2.5 });
        s.input.roll = ramp(s.input.roll, tr, if tr == 0.0 { 6.0 } else { 3.0 });
        s.input.yaw = ramp(s.input.yaw, ty, if ty == 0.0 { 6.0 } else { 3.0 });
        s.input.brake = k(KeyCode::B);
        if is_key_down(KeyCode::LeftShift) {
            s.input.throttle = (s.input.throttle + 0.5 * dt as f64).min(1.0);
        }
        if is_key_down(KeyCode::LeftControl) {
            s.input.throttle = (s.input.throttle - 0.5 * dt as f64).max(0.0);
        }
        if is_key_pressed(KeyCode::Z) {
            s.input.throttle = 1.0;
        }
        if is_key_pressed(KeyCode::Space) {
            s.toggle_engine();
        }
        if is_key_pressed(KeyCode::F) {
            s.flap_cmd = if s.flap_cmd < 10.0 * DEG { 20.0 * DEG } else if s.flap_cmd < 30.0 * DEG { 40.0 * DEG } else { 0.0 };
        }
        if is_key_pressed(KeyCode::G) {
            s.gear_cmd = !s.gear_cmd;
        }
        if is_key_pressed(KeyCode::X) {
            s.speedbrake_cmd = !s.speedbrake_cmd;
        }
        if is_key_pressed(KeyCode::R) {
            s.rcs_on = !s.rcs_on;
        }
        if is_key_pressed(KeyCode::T) {
            s.sas_on = !s.sas_on;
        }
        if is_key_pressed(KeyCode::K) && !s.on_ground {
            s.trim_level();
        }
        if !self.show_panel {
            if is_key_pressed(KeyCode::Up) || (is_key_down(KeyCode::Up) && is_key_down(KeyCode::RightShift)) {
                s.input.trim += 0.25 * DEG;
            }
            if is_key_pressed(KeyCode::Down) || (is_key_down(KeyCode::Down) && is_key_down(KeyCode::RightShift)) {
                s.input.trim -= 0.25 * DEG;
            }
            s.input.trim = s.input.trim.clamp(-30.0 * DEG, 20.0 * DEG);
        }

        if is_key_pressed(KeyCode::V) {
            self.view.show_forces = !self.view.show_forces;
        }
        if is_key_pressed(KeyCode::C) {
            self.view.mode = self.view.mode.next();
            self.view.look_yaw = 0.0;
            self.view.look_pitch = 0.0;
            let m = self.view.mode.name();
            self.flash(format!("Camera: {}", m));
        }
        if is_key_pressed(KeyCode::H) || is_key_pressed(KeyCode::F1) {
            self.show_help = !self.show_help;
        }
        if is_key_pressed(KeyCode::P) {
            self.paused = !self.paused;
        }
        if is_key_pressed(KeyCode::RightBracket) {
            self.time_scale = (self.time_scale * 2.0).min(16.0);
        }
        if is_key_pressed(KeyCode::LeftBracket) {
            self.time_scale = (self.time_scale * 0.5).max(0.0625);
        }
        if is_key_pressed(KeyCode::Tab) {
            self.show_panel = !self.show_panel;
        }
        if is_key_pressed(KeyCode::L) {
            self.toggle_log();
        }
        let keys = [KeyCode::Key1, KeyCode::Key2, KeyCode::Key3, KeyCode::Key4, KeyCode::Key5];
        for (i, key) in keys.iter().enumerate() {
            if is_key_pressed(*key) {
                self.restart(Scenario::ALL[i]);
            }
        }
        if is_key_pressed(KeyCode::Backspace) {
            self.restart(self.scenario);
        }

        if self.show_panel {
            let n = self.params.len();
            if is_key_pressed(KeyCode::Down) {
                self.sel = (self.sel + 1) % n;
            }
            if is_key_pressed(KeyCode::Up) {
                self.sel = (self.sel + n - 1) % n;
            }
            let mult = if is_key_down(KeyCode::RightShift) { 10.0 } else { 1.0 };
            let dir = if is_key_pressed(KeyCode::Right) { 1.0 } else if is_key_pressed(KeyCode::Left) { -1.0 } else { 0.0 };
            if dir != 0.0 {
                let p = &self.params[self.sel];
                let v = (p.get)(&self.sim) + dir * p.step * mult;
                let v = if p.name == "Propellant" || p.name == "Wind from" {
                    // wrap around
                    let span = p.max - p.min + p.step;
                    p.min + (v - p.min).rem_euclid(span)
                } else {
                    v.clamp(p.min, p.max)
                };
                (p.set)(&mut self.sim, v);
            }
        }

        // mouse camera: drag with the left (or right) button, middle button recentres
        let mp: Vec2 = mouse_position().into();
        if is_mouse_button_down(MouseButton::Left) || is_mouse_button_down(MouseButton::Right) {
            let d = mp - self.last_mouse;
            let v = &mut self.view;
            if v.mode == render::CamMode::Orbit {
                v.orbit_yaw += d.x * 0.008;
                v.orbit_pitch = (v.orbit_pitch + d.y * 0.008).clamp(-1.5, 1.5);
            } else {
                v.look_yaw += d.x * 0.008;
                v.look_pitch = (v.look_pitch + d.y * 0.008).clamp(-1.45, 1.45);
            }
        }
        if is_mouse_button_pressed(MouseButton::Middle) {
            self.view.look_yaw = 0.0;
            self.view.look_pitch = 0.0;
        }
        self.last_mouse = mp;
        let wheel = mouse_wheel().1;
        if wheel != 0.0 {
            self.view.orbit_dist = (self.view.orbit_dist * if wheel > 0.0 { 0.9 } else { 1.1 }).clamp(8.0, 400.0);
        }
    }

    fn toggle_log(&mut self) {
        if self.log.take().is_some() {
            let n = self.log_name.clone();
            self.flash(format!("Telemetry saved: {}", n));
            return;
        }
        let stamp = std::time::SystemTime::now().duration_since(std::time::UNIX_EPOCH).map(|d| d.as_secs()).unwrap_or(0);
        self.log_name = format!("flight_{}.csv", stamp);
        match std::fs::File::create(&self.log_name) {
            Ok(f) => {
                let mut w = std::io::BufWriter::new(f);
                let _ = writeln!(w, "t_s,x_m,alt_m,z_m,tas_ms,mach,alpha_deg,beta_deg,pitch_deg,roll_deg,heading_deg,nz_g,q_pa,lift_n,drag_n,thrust_n,mdot_kgs,isp_s,pc_pa,pe_pa,pa_pa,mass_kg,cg_x_m,fuel_kg,ox_kg,rho,temp_k,skin_k,heat_flux_wm2,stab_l_deg,stab_r_deg,rudder_deg");
                self.log = Some(w);
                let n = self.log_name.clone();
                self.flash(format!("Logging telemetry to {}", n));
            }
            Err(e) => self.flash(format!("Log failed: {}", e)),
        }
    }

    fn write_log(&mut self) {
        let s = &self.sim;
        if s.time - self.log_last < 0.05 {
            return;
        }
        self.log_last = s.time;
        if let Some(w) = &mut self.log {
            let (p, r, h) = s.euler();
            let a = &s.aero;
            let e = &s.engine.out;
            let _ = writeln!(
                w,
                "{:.3},{:.2},{:.2},{:.2},{:.3},{:.4},{:.3},{:.3},{:.3},{:.3},{:.3},{:.3},{:.1},{:.1},{:.1},{:.1},{:.3},{:.2},{:.0},{:.1},{:.2},{:.1},{:.4},{:.1},{:.1},{:.6},{:.2},{:.1},{:.0},{:.3},{:.3},{:.3}",
                s.time, s.pos.x, s.pos.y, s.pos.z, a.tas, a.mach, a.alpha.to_degrees(), a.beta.to_degrees(),
                p.to_degrees(), r.to_degrees(), h.to_degrees(), s.nz, a.qbar, a.lift, a.drag, e.thrust, e.mdot, e.isp,
                e.pc, e.pe, s.air.pressure, s.mp.mass, s.mp.cg.x, s.fuel, s.ox, s.air.density, s.air.temperature,
                s.skin_temp, s.heat_flux, s.ctrl.stab_left.to_degrees(), s.ctrl.stab_right.to_degrees(), s.ctrl.rudder.to_degrees()
            );
        }
    }

    fn update(&mut self, dt: f32) {
        if !self.paused {
            self.acc += dt as f64 * self.time_scale;
            let mut n = 0;
            while self.acc >= DT && n < 40_000 {
                self.sim.step(DT);
                self.acc -= DT;
                n += 1;
            }
            if n >= 40_000 {
                self.acc = 0.0;
            }
        }
        let s = &self.sim;
        let a = &s.aero;
        self.hist.push(s.time, [s.pos.y / 1000.0, a.mach, a.qbar / 1000.0, s.nz, s.skin_temp]);
        self.sm_timer -= dt;
        if self.sm_timer <= 0.0 && a.mach > 0.08 {
            self.sm_timer = 0.4;
            self.static_margin = s.static_margin(s.pos.y, a.mach);
        }
        if self.log.is_some() {
            self.write_log();
        }
        if let Some((_, t)) = &mut self.msg {
            *t -= dt;
            if *t <= 0.0 {
                self.msg = None;
            }
        }
    }

    fn warnings(&self) -> Vec<(String, Color)> {
        let s = &self.sim;
        let a = &s.aero;
        let mut w = Vec::new();
        let eas = a.tas * (s.air.density / atmosphere::RHO0).sqrt();
        if a.stall_frac > 0.3 && !s.on_ground {
            w.push(("STALL".to_string(), ALERT));
        }
        if s.nz > 7.33 || s.nz < -3.0 {
            w.push((format!("OVER-G {:.1}", s.nz), ALERT));
        }
        if a.qbar > 110_000.0 {
            w.push(("MAX Q".to_string(), ALERT));
        }
        if s.skin_temp > 920.0 {
            w.push(("SKIN OVERTEMP".to_string(), if s.skin_temp > 1250.0 { ALERT } else { WARN }));
        }
        if s.engine.out.separated && s.engine.out.thrust > 0.0 {
            w.push(("NOZZLE FLOW SEPARATION".to_string(), WARN));
        }
        if let Some(r) = s.engine.flameout_reason {
            w.push((format!("FLAMEOUT: {}", r), WARN));
        }
        let total = s.fuel + s.ox;
        if s.engine.running && total < 0.1 * (s.cfg.ox_volume * 1000.0) {
            w.push(("LOW PROPELLANT".to_string(), WARN));
        }
        if s.ctrl.gear_down > 0.1 && eas > 140.0 {
            w.push(("GEAR OVERSPEED".to_string(), WARN));
        }
        if s.ctrl.flap > 2.0 * DEG && eas > 160.0 {
            w.push(("FLAP OVERSPEED".to_string(), WARN));
        }
        if s.rcs_on && s.rcs_prop <= 0.0 {
            w.push(("RCS EMPTY".to_string(), WARN));
        }
        if a.qbar < 1500.0 && s.pos.y > 25_000.0 && !s.rcs_on {
            w.push(("LOW Q: AERO CONTROLS WEAK, ENABLE RCS (R)".to_string(), WARN));
        }
        if !s.on_ground && s.pos.y < 400.0 && s.vel.y < -25.0 {
            w.push(("SINK RATE, PULL UP".to_string(), ALERT));
        }
        if !s.on_ground && s.pos.y < 150.0 && s.ctrl.gear_down < 0.5 && s.vel.y < 0.0 {
            w.push(("GEAR UP".to_string(), ALERT));
        }
        w
    }
}

fn fmt_alt(m: f64) -> String {
    format!("{:.0} m  ({:.0} ft)", m, m / 0.3048)
}

fn draw_ui(app: &App, ui: &Ui) {
    let s = &app.sim;
    let a = &s.aero;
    let e = &s.engine.out;
    let (pitch, roll, heading) = s.euler();
    let sw = screen_width();
    let sh = screen_height();
    let eas = a.tas * (s.air.density / atmosphere::RHO0).sqrt();
    let cas = atmosphere::calibrated_airspeed(atmosphere::impact_pressure(a.mach, s.air.pressure));
    let gamma = (s.vel.y / s.vel.length().max(1e-6)).clamp(-1.0, 1.0).asin();
    let cl = a.lift / (a.qbar.max(1e-6) * s.cfg.wing.area);
    let cd = a.drag / (a.qbar.max(1e-6) * s.cfg.wing.area);

    // ---------------- air data ----------------
    let mut rows = vec![
        Row::Head("AIR DATA".into()),
        kv("Altitude", fmt_alt(s.pos.y)),
        kv("Vertical speed", format!("{:+.1} m/s", s.vel.y)),
        kv("TAS", format!("{:.1} m/s  {:.0} kt", a.tas, a.tas * 1.943_84)),
        kv("EAS / CAS", format!("{:.0} / {:.0} kt", eas * 1.943_84, cas * 1.943_84)),
        kvc("Mach", format!("{:.3}", a.mach), if a.mach > 1.0 { HEAD } else { FG }),
        kv("Ground speed", format!("{:.1} m/s", (s.vel.x * s.vel.x + s.vel.z * s.vel.z).sqrt())),
        kvc("Dynamic pressure", format!("{:.2} kPa", a.qbar / 1000.0), if a.qbar > 110e3 { ALERT } else { FG }),
        kvc("α / β", format!("{:+.2}° / {:+.2}°", a.alpha.to_degrees(), a.beta.to_degrees()), if a.stall_frac > 0.3 { ALERT } else { FG }),
        kv("Flight path γ", format!("{:+.2}°", gamma.to_degrees())),
        kvc("Load factor nz / ny", format!("{:+.2} / {:+.2} g", s.nz, s.ny), if s.nz > 7.33 || s.nz < -3.0 { ALERT } else { FG }),
        kv("Body rates p q r", format!("{:+.1} {:+.1} {:+.1} °/s", s.omega.x.to_degrees(), s.omega.z.to_degrees(), -s.omega.y.to_degrees())),
        Row::Head("AERODYNAMICS".into()),
        kv("Lift / Drag", format!("{:.1} / {:.1} kN", a.lift / 1e3, a.drag / 1e3)),
        kv("CL / CD", format!("{:.3} / {:.4}", cl, cd)),
        kv("L/D", format!("{:.2}", a.lift / a.drag.max(1e-6))),
        kv("CD friction+form", format!("{:.4}", a.cd0)),
        kv("CD wave", format!("{:.4}", a.cd_wave)),
        kv("CD induced+sep", format!("{:.4}", a.cd_induced)),
        kv("CD base", format!("{:.4}", a.cd_base)),
        kv("Reynolds (MAC)", format!("{:.2e}", a.re_mac)),
        kvc("Stalled area", format!("{:.0} %", a.stall_frac * 100.0), if a.stall_frac > 0.3 { ALERT } else { FG }),
        kvc("Static margin", format!("{:+.1} % MAC", app.static_margin.0 * 100.0),
            if app.static_margin.0 < 0.0 { ALERT } else if app.static_margin.0 < 0.05 { WARN } else { FG }),
        kv("Downwash/stall model", format!("{}", if a.mach > 1.0 { "Ackeret" } else { "Helmbold/PG" })),
        Row::Head("ATMOSPHERE (ISA 1976)".into()),
        kv("T / p", format!("{:.1} K / {:.0} Pa", s.air.temperature, s.air.pressure)),
        kv("ρ / a", format!("{:.4} kg/m³ / {:.0} m/s", s.air.density, s.air.speed_of_sound)),
        kv("μ (Sutherland)", format!("{:.3e} Pa·s", s.air.viscosity)),
        kv("Wind + gust", format!("{:.1} m/s", s.wind_total.length())),
        kv("Gravity", format!("{:.4} m/s²", atmosphere::gravity(s.pos.y))),
    ];
    if app.show_panel {
        rows.truncate(12);
    }
    let compact = sw < 1300.0;
    let lw = if compact { 300.0 } else { 330.0 };
    let left_h = panel(ui, 10.0, 10.0, lw, &rows);

    // ---------------- propulsion ----------------
    let p = s.eng_cfg.propellant();
    let state = if s.engine.running {
        ("RUNNING", GOOD)
    } else if s.engine.out.thrust > 1.0 {
        ("SHUTDOWN", WARN)
    } else {
        ("OFF", DIM)
    };
    let pa = s.air.pressure;
    let exp = if e.thrust <= 0.0 {
        ("-", DIM)
    } else if e.separated {
        ("SEPARATED", ALERT)
    } else if e.pe > pa * 1.1 {
        ("UNDER-expanded", HEAD)
    } else if e.pe < pa * 0.9 {
        ("OVER-expanded", WARN)
    } else {
        ("ideal", GOOD)
    };
    let m_dry = s.mp.mass - s.fuel - s.ox;
    let isp_ref = if e.isp > 1.0 { e.isp } else { s.eng_cfg.performance(&s.nozzle, s.eng_cfg.pc_nominal, pa).isp };
    let dv = isp_ref * atmosphere::G0 * (s.mp.mass / m_dry).ln();
    let mdot_nom = s.eng_cfg.pc_nominal * s.nozzle.throat_area / s.nozzle.cstar;
    let burn_left = (s.fuel + s.ox) / mdot_nom.max(1e-6);
    let full_fuel = if p.solid { (s.cfg.fuel_volume + s.cfg.ox_volume) * p.rho_fuel * 0.78 } else {
        (s.cfg.fuel_volume * p.rho_fuel).min(s.cfg.ox_volume * p.rho_ox / p.of_ratio) * (1.0 + p.of_ratio)
    };
    let rows = vec![
        Row::Head("ROCKET ENGINE".into()),
        kv("Propellant", p.name.to_string()),
        kvc("State", state.0.to_string(), state.1),
        kv("Throttle cmd", if p.solid { "solid: fixed".into() } else { format!("{:.0} % (min {:.0})", s.input.throttle * 100.0, s.eng_cfg.min_throttle * 100.0) }),
        kv("Chamber pressure", format!("{:.2} MPa", e.pc / 1e6)),
        kv("Thrust", format!("{:.1} kN", e.thrust / 1e3)),
        kv("Mass flow ṁ", format!("{:.1} kg/s", e.mdot)),
        kv("Isp", format!("{:.1} s", e.isp)),
        kv("Exhaust v_e", format!("{:.0} m/s", e.ve)),
        kv("c* / C_F", format!("{:.0} m/s / {:.3}", s.nozzle.cstar, e.cf)),
        kv("Exit Mach", format!("{:.2}", e.exit_mach.max(s.nozzle.exit_mach * (e.thrust > 0.0) as u8 as f64))),
        kv("p_exit / p_amb", format!("{:.1} / {:.1} kPa", e.pe / 1e3, pa / 1e3)),
        kvc("Nozzle", exp.0.to_string(), exp.1),
        kv("Thrust / weight", format!("{:.2}", e.thrust / (s.mp.mass * atmosphere::gravity(s.pos.y)))),
        kv("Gimbal p / y", format!("{:+.1}° / {:+.1}°", s.gimbal.0.to_degrees(), s.gimbal.1.to_degrees())),
        Row::Head("PROPELLANT & MASS".into()),
        Row::Bar(format!("Fuel {:.0} kg", s.fuel), (s.fuel / (full_fuel / (1.0 + p.of_ratio)).max(1.0)) as f32, Color::new(0.9, 0.7, 0.2, 1.0)),
        Row::Bar(format!("Oxid {:.0} kg", s.ox), (s.ox / (full_fuel * p.of_ratio / (1.0 + p.of_ratio)).max(1.0)) as f32, Color::new(0.3, 0.7, 1.0, 1.0)),
        Row::Bar(format!("H2O2 {:.0} kg", s.rcs_prop), (s.rcs_prop / s.cfg.rcs_capacity) as f32, Color::new(0.8, 0.8, 0.9, 1.0)),
        kv("Burn time left (100%)", format!("{:.1} s", burn_left)),
        kv("Δv left (Tsiolkovsky)", format!("{:.0} m/s", dv)),
        kv("Δv spent", format!("{:.0} m/s", s.dv_spent)),
        kv("Mass", format!("{:.0} kg", s.mp.mass)),
        kv("CG x", format!("{:+.3} m", s.mp.cg.x)),
        kv("Ixx Iyy Izz", format!("{:.0} {:.0} {:.0}", s.mp.inertia.x_axis.x / 1e3, s.mp.inertia.y_axis.y / 1e3, s.mp.inertia.z_axis.z / 1e3) + " t·m²"),
        Row::Head("AEROTHERMAL".into()),
        kv("Stagnation temp", format!("{:.0} K", s.stag_temp)),
        kv("Nose heat flux", format!("{:.1} kW/m²", s.heat_flux / 1e3)),
        Row::Bar(format!("Skin {:.0} K", s.skin_temp), ((s.skin_temp - 200.0) / 1300.0) as f32,
            if s.skin_temp > 920.0 { ALERT } else { Color::new(1.0, 0.5, 0.2, 1.0) }),
    ];
    let rw = if compact { 310.0 } else { 340.0 };
    panel(ui, sw - rw - 10.0, 10.0, rw, &rows);

    // ---------------- flight state, ADI ----------------
    let r = 95.0;
    let cx = sw * 0.5;
    let cy = sh - r - 45.0;
    hud::adi(ui, cx, cy, r, pitch as f32, roll as f32, a.alpha as f32, a.beta as f32, heading as f32);
    let deg = |x: f64| x.to_degrees();
    let rows = vec![
        Row::Head("CONTROLS".into()),
        kv("Stab L / R", format!("{:+.1}° / {:+.1}°", deg(s.ctrl.stab_left), deg(s.ctrl.stab_right))),
        kv("Rudder", format!("{:+.1}°", deg(s.ctrl.rudder))),
        kv("Pitch trim", format!("{:+.2}°", deg(s.input.trim))),
        kv("Flaps", format!("{:.0}°", deg(s.ctrl.flap))),
        kvc("Gear", if s.ctrl.gear_down > 0.99 { "DOWN".into() } else if s.ctrl.gear_down < 0.01 { "UP".into() } else { "transit".into() },
            if s.ctrl.gear_down > 0.99 { GOOD } else { FG }),
        kv("Speed brake", format!("{:.0} %", s.ctrl.speedbrake * 100.0)),
        kvc("Brakes", if s.input.brake > 0.0 { "ON".into() } else { "off".into() }, if s.input.brake > 0.0 { WARN } else { FG }),
        kvc("SAS (rate damper)", if s.sas_on { "ON".into() } else { "off".into() }, if s.sas_on { GOOD } else { WARN }),
        kvc("RCS", if s.rcs_on { "ON".into() } else { "off".into() }, if s.rcs_on { GOOD } else { FG }),
        kv("Pitch / roll", format!("{:+.1}° / {:+.1}°", deg(pitch), deg(roll))),
    ];
    let ch = rows.len() as f32 * ui.lh() + 10.0;
    if !app.show_panel {
        panel(ui, 10.0, sh - ch - 10.0, if compact { 230.0 } else { 250.0 }, &rows);
    }
    let gw = if compact { (sw - cx - r - 30.0).clamp(160.0, 300.0) } else { 300.0 };
    app.hist.draw(ui, sw - gw - 10.0, sh - 5.0 * 50.0 - 16.0, gw, 50.0);

    // ---------------- top status & warnings ----------------
    let status = format!(
        "T+{:.1}s   x{}{}   cam: {}   {}{}   [H] help  [Tab] sandbox",
        s.time,
        app.time_scale,
        if app.paused { "  PAUSED" } else { "" },
        app.view.mode.name(),
        app.scenario.name(),
        if app.log.is_some() { "   ● REC" } else { "" }
    );
    let w = ui.width(&status, ui.size);
    draw_rectangle(sw * 0.5 - w * 0.5 - 8.0, 6.0, w + 16.0, ui.lh() + 6.0, hud::BG);
    ui.text(&status, sw * 0.5 - w * 0.5, 6.0 + ui.lh(), FG);
    let mut wy = 50.0;
    for (txt, col) in app.warnings() {
        let w = ui.width(&txt, 20);
        draw_rectangle(sw * 0.5 - w * 0.5 - 10.0, wy - 20.0, w + 20.0, 28.0, Color::new(0.0, 0.0, 0.0, 0.6));
        draw_rectangle_lines(sw * 0.5 - w * 0.5 - 10.0, wy - 20.0, w + 20.0, 28.0, 2.0, col);
        ui.text_sized(&txt, sw * 0.5 - w * 0.5, wy, 20, col);
        wy += 34.0;
    }
    if let Some((m, _)) = &app.msg {
        let w = ui.width(m, 18);
        ui.text_sized(m, sw * 0.5 - w * 0.5, wy + 10.0, 18, HEAD);
    }

    if let Status::Crashed(why) = &s.status {
        let big = "AIRCRAFT LOST";
        let w1 = ui.width(big, 44);
        let w2 = ui.width(why, 22);
        draw_rectangle(0.0, sh * 0.38, sw, 130.0, Color::new(0.2, 0.0, 0.0, 0.75));
        ui.text_sized(big, sw * 0.5 - w1 * 0.5, sh * 0.38 + 55.0, 44, ALERT);
        ui.text_sized(why, sw * 0.5 - w2 * 0.5, sh * 0.38 + 90.0, 22, FG);
        let st = format!("max Mach {:.2} · max alt {:.0} m · max q {:.0} kPa · max |nz| {:.1} · max skin {:.0} K   —   Backspace: retry, 1-5: scenarios",
            s.max_mach, s.max_alt, s.max_q / 1e3, s.max_nz, s.max_skin);
        let w3 = ui.width(&st, 15);
        ui.text_sized(&st, sw * 0.5 - w3 * 0.5, sh * 0.38 + 118.0, 15, DIM);
    } else if s.on_ground && s.aero.tas < 0.5 && s.time > 5.0 && s.max_alt > 50.0 {
        let t = format!("WHEELS STOPPED — max Mach {:.2}, max altitude {:.0} m", s.max_mach, s.max_alt);
        let w = ui.width(&t, 22);
        ui.text_sized(&t, sw * 0.5 - w * 0.5, sh * 0.35, 22, GOOD);
    }

    if app.show_panel {
        draw_sandbox(app, ui, left_h + 20.0);
    } else if app.show_help {
        draw_help(ui);
    }
}

fn draw_sandbox(app: &App, ui: &Ui, y: f32) {
    let s = &app.sim;
    let mut rows = vec![Row::Head("SANDBOX  ↑↓ select  ←→ change  (RShift ×10)".into())];
    for (i, p) in app.params.iter().enumerate() {
        let v = (p.get)(s);
        let val = match p.name {
            "Propellant" => PROPELLANTS[v as usize].name.to_string(),
            "Turbulence (Dryden)" => ["off", "light", "moderate", "severe"][v as usize].to_string(),
            _ => format!("{:.3} {}", v, p.unit).replace(".000 ", " "),
        };
        let mark = if i == app.sel { "▶ " } else { "  " };
        rows.push(Row::Kv(format!("{}{}", mark, p.name), val, if i == app.sel { WARN } else { FG }));
    }
    // derived engine performance for the current configuration
    let ec = &s.eng_cfg;
    let d = &s.nozzle;
    let sl = ec.performance(d, ec.pc_nominal, atmosphere::P0);
    let vac = ec.performance(d, ec.pc_nominal, 0.0);
    let pe_design = ec.design_ambient_pressure(d);
    // altitude where the nozzle is perfectly expanded (bisection on ISA pressure)
    let (mut lo, mut hi) = (-1000.0, 90_000.0);
    for _ in 0..50 {
        let mid = 0.5 * (lo + hi);
        if atmosphere::isa(mid, 0.0).pressure > pe_design { lo = mid } else { hi = mid }
    }
    let m_full = s.mp.mass;
    let mprop = s.fuel + s.ox;
    rows.push(Row::Head("DERIVED (ideal rocket, frozen flow)".into()));
    rows.push(kv("c* / exit Mach", format!("{:.0} m/s / {:.2}", d.cstar, d.exit_mach)));
    rows.push(kv("Throat / exit diam.", format!("{:.3} / {:.3} m", ec.throat_diam, (d.exit_area * 4.0 / std::f64::consts::PI).sqrt())));
    rows.push(kvc("Thrust SL / vac", format!("{:.1} / {:.1} kN", sl.thrust / 1e3, vac.thrust / 1e3), if sl.separated { WARN } else { FG }));
    rows.push(kv("Isp SL / vac", format!("{:.1} / {:.1} s", sl.isp, vac.isp)));
    rows.push(kv("ṁ / divergence λ", format!("{:.1} kg/s / {:.4}", vac.mdot, d.lambda)));
    rows.push(kv("Optimum expansion alt.", format!("{:.0} m", 0.5 * (lo + hi))));
    rows.push(kv("Burn time", format!("{:.1} s", mprop / vac.mdot.max(1e-6))));
    rows.push(kv("Δv vac (Tsiolkovsky)", format!("{:.0} m/s", vac.isp * atmosphere::G0 * (m_full / (m_full - mprop)).ln())));
    rows.push(kv("T/W at SL, now", format!("{:.2}", sl.thrust / (m_full * atmosphere::G0))));
    rows.push(kvc("Static margin (now)", format!("{:+.1} % MAC", app.static_margin.0 * 100.0),
        if app.static_margin.0 < 0.0 { ALERT } else { FG }));
    if s.engine.running {
        rows.push(Row::Line("engine running: propellant/load changes locked".into(), WARN));
    }
    panel(ui, 10.0, y, 420.0, &rows);
}

fn draw_help(ui: &Ui) {
    let lines = [
        "W/S pitch  A/D roll  Q/E yaw          ↑/↓ pitch trim   K auto-trim level",
        "Space ignite / shut down engine       Shift/Ctrl throttle   Z full",
        "F flaps 0/20/40   G gear   X speed brakes   B wheel brakes",
        "R reaction control (H2O2 RCS)   T stability augmentation (SAS)",
        "V force vectors (lift green, stall yellow, drag red, thrust orange, NP blue, CG yellow)",
        "C camera   left-drag: rotate view  wheel: zoom  middle: recentre   P pause  [ ] warp  L CSV",
        "1 runway  2 B-52 drop  3 hypersonic  4 space shot  5 dead-stick approach   Backspace restart",
        "Tab sandbox: propellant, nozzle, chamber pressure, geometry, mass, wind, ISA ΔT",
        "H hide this help",
    ];
    let lh = ui.lh();
    let w = 700.0;
    let h = lines.len() as f32 * lh + 14.0;
    let x = screen_width() * 0.5 - w * 0.5;
    let y = screen_height() * 0.5 - h * 0.5 - 60.0;
    draw_rectangle(x, y, w, h, hud::BG);
    for (i, l) in lines.iter().enumerate() {
        ui.text(l, x + 12.0, y + lh * (i as f32 + 1.0), if i == 0 { HEAD } else { FG });
    }
}

async fn app_main() {
    let font = load_ttf_font("/usr/share/fonts/TTF/DejaVuSansMono.ttf").await.ok();
    let mut ui = Ui { font, size: 14 };
    let mut app = App::new();
    // optional start-up flags: --scenario N  --ignite  --forces  --cam N  --nohelp
    let args: Vec<String> = std::env::args().collect();
    let arg_n = |name: &str| args.iter().position(|a| a == name).and_then(|i| args.get(i + 1)).and_then(|v| v.parse::<usize>().ok());
    if let Some(n) = arg_n("--scenario") {
        app.restart(Scenario::ALL[(n.max(1) - 1).min(Scenario::ALL.len() - 1)]);
    }
    if args.iter().any(|a| a == "--ignite") {
        app.sim.input.throttle = 1.0;
        app.sim.toggle_engine();
    }
    app.view.show_forces = args.iter().any(|a| a == "--forces");
    app.show_help = !args.iter().any(|a| a == "--nohelp");
    for _ in 0..arg_n("--cam").unwrap_or(0) {
        app.view.mode = app.view.mode.next();
    }
    loop {
        let dt = get_frame_time().min(0.05);
        app.input(dt);
        app.update(dt);
        render::draw_world(&app.sim, &mut app.view, dt, app.static_margin.1);
        set_default_camera();
        ui.size = if screen_width() < 1300.0 || screen_height() < 900.0 { 12 } else { 14 };
        draw_ui(&app, &ui);
        next_frame().await;
    }
}

fn main() {
    if std::env::args().any(|a| a == "--selftest") {
        selftest::run();
        return;
    }
    let conf = Conf {
        window_title: "Rocketplane — 6-DOF aerodynamics & rocket propulsion sandbox".to_string(),
        window_width: 1600,
        window_height: 940,
        sample_count: 4,
        window_resizable: true,
        ..Default::default()
    };
    macroquad::Window::from_config(conf, app_main());
}
