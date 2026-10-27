//! Ideal-rocket (quasi-1D isentropic nozzle) liquid / solid engine model.
//!
//! Chamber gas is treated as a calorically perfect gas with properties taken
//! from NASA CEA equilibrium runs at the listed mixture ratio. From these the
//! model derives c*, choked mass flow, exit Mach (area–Mach relation), exit
//! pressure and exhaust velocity, then thrust
//!     F = λ·ṁ·v_e + (p_e − p_a)·A_e
//! including nozzle divergence loss λ and Summerfield flow separation in an
//! over-expanded nozzle. Hydrogen-peroxide monopropellant RCS is modelled too.

use crate::atmosphere::{G0, R_UNIVERSAL};

#[derive(Clone, Copy, Debug)]
pub struct Propellant {
    pub name: &'static str,
    pub of_ratio: f64,       // oxidizer / fuel mass ratio
    pub chamber_temp: f64,   // K (adiabatic flame temperature)
    pub gamma: f64,          // ratio of specific heats of combustion products
    pub molar_mass: f64,     // kg/mol of combustion products
    pub rho_fuel: f64,       // kg/m³
    pub rho_ox: f64,         // kg/m³
    pub solid: bool,         // solid motors can neither throttle nor shut down
}

pub const PROPELLANTS: [Propellant; 6] = [
    Propellant { name: "LOX / Ammonia (XLR99)", of_ratio: 1.25, chamber_temp: 2_950.0, gamma: 1.23, molar_mass: 0.0195, rho_fuel: 682.0, rho_ox: 1141.0, solid: false },
    Propellant { name: "LOX / RP-1",            of_ratio: 2.56, chamber_temp: 3_670.0, gamma: 1.22, molar_mass: 0.0233, rho_fuel: 810.0, rho_ox: 1141.0, solid: false },
    Propellant { name: "LOX / LH2",             of_ratio: 6.00, chamber_temp: 3_500.0, gamma: 1.21, molar_mass: 0.0136, rho_fuel: 70.8,  rho_ox: 1141.0, solid: false },
    Propellant { name: "LOX / Methane",         of_ratio: 3.40, chamber_temp: 3_550.0, gamma: 1.21, molar_mass: 0.0213, rho_fuel: 422.0, rho_ox: 1141.0, solid: false },
    Propellant { name: "N2O4 / UDMH",           of_ratio: 2.60, chamber_temp: 3_400.0, gamma: 1.24, molar_mass: 0.0236, rho_fuel: 791.0, rho_ox: 1440.0, solid: false },
    Propellant { name: "APCP solid (AP/Al/HTPB)", of_ratio: 0.0, chamber_temp: 3_300.0, gamma: 1.18, molar_mass: 0.0290, rho_fuel: 1750.0, rho_ox: 1750.0, solid: true },
];

#[derive(Clone, Debug)]
pub struct EngineConfig {
    pub prop: usize,
    pub pc_nominal: f64,    // Pa, chamber pressure at 100 % throttle
    pub throat_diam: f64,   // m
    pub area_ratio: f64,    // Ae / At
    pub half_angle_deg: f64, // conical divergence half-angle (0 = ideal bell)
    pub eta_cstar: f64,     // combustion efficiency
    pub min_throttle: f64,  // below this the chamber goes unstable
    pub gimbal_max_deg: f64,
    pub spool_tau: f64,     // s, chamber pressure time constant
}

impl Default for EngineConfig {
    fn default() -> Self {
        // Sized to resemble the Reaction Motors XLR99 (~250 kN at altitude).
        EngineConfig {
            prop: 0,
            pc_nominal: 4.2e6,
            throat_diam: 0.215,
            area_ratio: 9.8,
            half_angle_deg: 15.0,
            eta_cstar: 0.97,
            min_throttle: 0.5,
            gimbal_max_deg: 0.0,
            spool_tau: 0.18,
        }
    }
}

/// Values that only depend on the configuration, cached between changes.
#[derive(Clone, Copy, Debug, Default)]
pub struct NozzleDesign {
    pub throat_area: f64,
    pub exit_area: f64,
    pub exit_mach: f64,
    pub pe_over_pc: f64,
    pub cstar: f64,        // real c* (with η)
    pub ve_ideal: f64,     // full-expansion exhaust velocity to design exit
    pub lambda: f64,       // divergence factor
}

#[derive(Clone, Copy, Debug, Default)]
pub struct EngineOutput {
    pub thrust: f64,
    pub mdot: f64,
    pub isp: f64,
    pub ve: f64,
    pub pe: f64,
    pub pc: f64,
    pub cf: f64,
    pub exit_mach: f64,
    pub separated: bool,
}

pub fn exit_mach_from_area_ratio(eps: f64, g: f64) -> f64 {
    // A/A* is monotonically increasing for M>1: bisection is robust.
    let (mut lo, mut hi) = (1.0 + 1e-9, 60.0);
    for _ in 0..100 {
        let mid = 0.5 * (lo + hi);
        if area_ratio_from_mach(mid, g) < eps { lo = mid } else { hi = mid }
    }
    0.5 * (lo + hi)
}

pub fn area_ratio_from_mach(m: f64, g: f64) -> f64 {
    let t = 2.0 / (g + 1.0) * (1.0 + 0.5 * (g - 1.0) * m * m);
    t.powf((g + 1.0) / (2.0 * (g - 1.0))) / m
}

fn pressure_ratio(m: f64, g: f64) -> f64 {
    (1.0 + 0.5 * (g - 1.0) * m * m).powf(-g / (g - 1.0))
}

fn mach_from_pressure_ratio(pr: f64, g: f64) -> f64 {
    ((pr.powf(-(g - 1.0) / g) - 1.0) * 2.0 / (g - 1.0)).max(0.0).sqrt()
}

impl EngineConfig {
    pub fn propellant(&self) -> &'static Propellant {
        &PROPELLANTS[self.prop]
    }

    pub fn design(&self) -> NozzleDesign {
        let p = self.propellant();
        let g = p.gamma;
        let r_spec = R_UNIVERSAL / p.molar_mass;
        let at = std::f64::consts::PI * 0.25 * self.throat_diam * self.throat_diam;
        let big_gamma = g.sqrt() * (2.0 / (g + 1.0)).powf((g + 1.0) / (2.0 * (g - 1.0)));
        let cstar = self.eta_cstar * (r_spec * p.chamber_temp).sqrt() / big_gamma;
        let me = exit_mach_from_area_ratio(self.area_ratio, g);
        let pe_pc = pressure_ratio(me, g);
        let ve = (2.0 * g / (g - 1.0) * r_spec * p.chamber_temp * (1.0 - pe_pc.powf((g - 1.0) / g))).sqrt()
            * self.eta_cstar.sqrt();
        let a = self.half_angle_deg.to_radians();
        NozzleDesign {
            throat_area: at,
            exit_area: at * self.area_ratio,
            exit_mach: me,
            pe_over_pc: pe_pc,
            cstar,
            ve_ideal: ve,
            lambda: 0.5 * (1.0 + a.cos()),
        }
    }

    /// Steady-state performance at a given chamber pressure and ambient pressure.
    pub fn performance(&self, d: &NozzleDesign, pc: f64, pa: f64) -> EngineOutput {
        if pc <= 1.0 {
            return EngineOutput { exit_mach: d.exit_mach, ..Default::default() };
        }
        let p = self.propellant();
        let g = p.gamma;
        let r_spec = R_UNIVERSAL / p.molar_mass;
        let mdot = pc * d.throat_area / d.cstar;
        let pe = pc * d.pe_over_pc;

        // Summerfield criterion: the boundary layer separates where the wall
        // pressure drops below ~0.4·pa. Beyond that point the wall sees roughly
        // ambient pressure, so the nozzle behaves like a shorter one.
        let (ve, p_exit, a_exit, me, separated) = if pe < 0.4 * pa && pa > 0.0 {
            let p_sep = (0.4 * pa).min(pc * 0.5);
            let m_sep = mach_from_pressure_ratio(p_sep / pc, g).max(1.0);
            let eps_sep = area_ratio_from_mach(m_sep, g).min(self.area_ratio);
            let ve = (2.0 * g / (g - 1.0) * r_spec * p.chamber_temp
                * (1.0 - (p_sep / pc).powf((g - 1.0) / g))).sqrt() * self.eta_cstar.sqrt();
            (ve, p_sep, d.throat_area * eps_sep, m_sep, true)
        } else {
            (d.ve_ideal, pe, d.exit_area, d.exit_mach, false)
        };

        let thrust = (d.lambda * mdot * ve + (p_exit - pa) * a_exit).max(0.0);
        EngineOutput {
            thrust,
            mdot,
            isp: thrust / (mdot * G0),
            ve,
            pe: p_exit,
            pc,
            cf: thrust / (pc * d.throat_area),
            exit_mach: me,
            separated,
        }
    }

    /// Ambient pressure at which the nozzle is perfectly expanded (pe = pa).
    pub fn design_ambient_pressure(&self, d: &NozzleDesign) -> f64 {
        self.pc_nominal * d.pe_over_pc
    }
}

/// Runtime engine state: chamber pressure follows the throttle with a
/// first-order lag (feed system + combustion response).
#[derive(Clone, Debug, Default)]
pub struct EngineState {
    pub running: bool,
    pub throttle: f64,     // 0..1 commanded
    pub pc: f64,
    pub out: EngineOutput,
    pub flameout_reason: Option<&'static str>,
    pub burn_time: f64,
}

impl EngineState {
    pub fn pc_command(&self, cfg: &EngineConfig) -> f64 {
        if !self.running {
            return 0.0;
        }
        let tl = if cfg.propellant().solid { 1.0 } else { self.throttle.clamp(cfg.min_throttle, 1.0) };
        cfg.pc_nominal * tl
    }

    pub fn step(&mut self, cfg: &EngineConfig, d: &NozzleDesign, pa: f64, dt: f64) {
        let target = self.pc_command(cfg);
        let k = (dt / cfg.spool_tau).min(1.0);
        // shutdown tails off faster than the start transient
        let k = if target < self.pc { (k * 1.6).min(1.0) } else { k };
        self.pc += (target - self.pc) * k;
        if self.pc < 1000.0 && !self.running {
            self.pc = 0.0;
        }
        self.out = cfg.performance(d, self.pc, pa);
        if self.running {
            self.burn_time += dt;
        }
    }
}

/// Hydrogen-peroxide (90 %) monopropellant reaction control thruster.
#[derive(Clone, Copy, Debug)]
pub struct RcsThruster {
    pub f_vac: f64,      // N
    pub isp_vac: f64,    // s
    pub exit_area: f64,  // m²
}

impl RcsThruster {
    pub const H2O2: RcsThruster = RcsThruster { f_vac: 500.0, isp_vac: 160.0, exit_area: 0.0012 };
    pub fn thrust(&self, pa: f64) -> f64 {
        (self.f_vac - pa * self.exit_area).max(0.0)
    }
    pub fn mdot(&self) -> f64 {
        self.f_vac / (self.isp_vac * G0)
    }
}
