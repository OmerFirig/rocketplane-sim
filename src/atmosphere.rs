//! U.S. Standard Atmosphere 1976 (ISA), 0–86 km, isothermal extension above.
//!
//! Hydrostatic equation integrated per layer on geopotential altitude,
//! ideal-gas density, speed of sound sqrt(γRT) and Sutherland viscosity.

pub const R_AIR: f64 = 287.052_87; // J/(kg·K), specific gas constant of dry air
pub const GAMMA_AIR: f64 = 1.4;
pub const G0: f64 = 9.806_65; // m/s², standard gravity
pub const R_UNIVERSAL: f64 = 8.314_462_618; // J/(mol·K)
pub const R_EARTH: f64 = 6_371_000.0; // mean Earth radius for gravity
const R_GEOPOT: f64 = 6_356_766.0; // ISA geopotential reference radius
pub const P0: f64 = 101_325.0;
pub const RHO0: f64 = 1.225;
pub const A0: f64 = 340.294;
pub const STEFAN_BOLTZMANN: f64 = 5.670_374e-8;

/// (base geopotential altitude [m], base temperature [K], lapse rate [K/m])
const LAYERS: [(f64, f64, f64); 8] = [
    (0.0, 288.15, -0.0065),    // troposphere
    (11_000.0, 216.65, 0.0),   // tropopause
    (20_000.0, 216.65, 0.001), // stratosphere 1
    (32_000.0, 228.65, 0.0028), // stratosphere 2
    (47_000.0, 270.65, 0.0),   // stratopause
    (51_000.0, 270.65, -0.0028), // mesosphere 1
    (71_000.0, 214.65, -0.002), // mesosphere 2
    (84_852.0, 186.946, 0.0),  // mesopause (isothermal extension)
];

#[derive(Clone, Copy, Debug, Default)]
pub struct Air {
    pub temperature: f64,       // K
    pub pressure: f64,          // Pa
    pub density: f64,           // kg/m³
    pub speed_of_sound: f64,    // m/s
    pub viscosity: f64,         // Pa·s (dynamic)
}

/// Sutherland's law for air.
pub fn sutherland(t: f64) -> f64 {
    1.458e-6 * t.powf(1.5) / (t + 110.4)
}

/// Standard atmosphere at geometric altitude `z` (m). `dt_isa` shifts the whole
/// temperature profile (ISA+ΔT day) while the pressure profile is kept standard,
/// so a hot day gives lower density at the same altitude, as it does in reality.
pub fn isa(z: f64, dt_isa: f64) -> Air {
    let z = z.max(-1000.0);
    let h = R_GEOPOT * z / (R_GEOPOT + z);
    let g_r = G0 / R_AIR;

    let mut p_base = P0;
    let mut layer = 0;
    for i in 0..LAYERS.len() - 1 {
        let (hb, tb, lr) = LAYERS[i];
        let h_top = LAYERS[i + 1].0;
        if h < h_top {
            break;
        }
        p_base = layer_pressure(p_base, tb, lr, h_top - hb, g_r);
        layer = i + 1;
    }
    let (hb, tb, lr) = LAYERS[layer];
    let t_std = tb + lr * (h - hb);
    let p = layer_pressure(p_base, tb, lr, h - hb, g_r);

    let t = (t_std + dt_isa).max(100.0);
    let rho = p / (R_AIR * t);
    Air {
        temperature: t,
        pressure: p,
        density: rho,
        speed_of_sound: (GAMMA_AIR * R_AIR * t).sqrt(),
        viscosity: sutherland(t),
    }
}

fn layer_pressure(p_base: f64, tb: f64, lr: f64, dh: f64, g_r: f64) -> f64 {
    if lr.abs() < 1e-12 {
        p_base * (-g_r * dh / tb).exp()
    } else {
        p_base * (tb / (tb + lr * dh)).powf(g_r / lr)
    }
}

/// Newtonian gravity falling with altitude (inverse-square law).
pub fn gravity(z: f64) -> f64 {
    let r = R_EARTH / (R_EARTH + z.max(0.0));
    G0 * r * r
}

/// Impact pressure qc a pitot tube would read: isentropic below M1, Rayleigh
/// pitot formula (normal shock in front of the probe) above.
pub fn impact_pressure(mach: f64, p: f64) -> f64 {
    let g = GAMMA_AIR;
    if mach < 1.0 {
        p * ((1.0 + 0.5 * (g - 1.0) * mach * mach).powf(g / (g - 1.0)) - 1.0)
    } else {
        let m2 = mach * mach;
        let a = (0.5 * (g + 1.0) * m2).powf(g / (g - 1.0));
        let b = ((g + 1.0) / (2.0 * g * m2 - (g - 1.0))).powf(1.0 / (g - 1.0));
        p * (a * b - 1.0)
    }
}

/// Calibrated airspeed from impact pressure (sea-level standard calibration).
pub fn calibrated_airspeed(qc: f64) -> f64 {
    let x = qc / P0 + 1.0;
    let sub = A0 * (5.0 * (x.powf(2.0 / 7.0) - 1.0)).sqrt();
    if sub <= A0 {
        return sub;
    }
    // Supersonic calibration: solve qc/P0 + 1 = f(Vc/a0) by fixed-point iteration.
    let mut m = sub / A0;
    for _ in 0..30 {
        let m2 = m * m;
        let f = 166.921_58 * m2 * m2 * m2 * m / (7.0 * m2 - 1.0).powf(2.5);
        m *= (x / f).powf(0.15);
    }
    m * A0
}
