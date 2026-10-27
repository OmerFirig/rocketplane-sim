//! Airframe geometry, lifting-surface discretisation and mass properties.
//!
//! Body axes: +X forward, +Y up, +Z right (right-handed). The body origin is a
//! fixed fuselage reference station; the centre of gravity moves as propellant
//! burns. Default numbers are modelled on the North American X-15.

use macroquad::math::{DMat3, DVec3};

#[derive(Clone, Copy, PartialEq, Eq, Debug)]
pub enum Group {
    Wing,
    HTail,
    VTail,
}

#[derive(Clone, Copy, PartialEq, Eq, Debug)]
pub enum Ctrl {
    None,
    Flap,
    StabLeft,
    StabRight,
    Rudder,
}

#[derive(Clone, Debug)]
pub struct SurfaceParams {
    pub area: f64,         // m² (both panels for wing / h-tail)
    pub aspect: f64,
    pub taper: f64,
    pub sweep_le_deg: f64,
    pub tc: f64,           // thickness / chord
    pub dihedral_deg: f64,
    pub x_root_le: f64,    // body x of root leading edge
    pub y_root: f64,
    pub wave_k: f64,       // Ackeret thickness factor: 16/3 biconvex, 4 diamond/wedge
    pub te_bluntness: f64, // trailing-edge base height / chord (wedge sections)
}

impl SurfaceParams {
    pub fn span(&self) -> f64 {
        (self.area * self.aspect).sqrt()
    }
    /// Trapezoid root chord; for a single fin `span()` is its height.
    pub fn root_chord(&self) -> f64 {
        2.0 * self.area / (self.span() * (1.0 + self.taper))
    }
    pub fn mac(&self) -> f64 {
        let l = self.taper;
        2.0 / 3.0 * self.root_chord() * (1.0 + l + l * l) / (1.0 + l)
    }
    /// Sweep of the line at chord fraction `x` (0 = LE, 0.5 = half chord).
    pub fn sweep_at(&self, x: f64) -> f64 {
        let t = self.sweep_le_deg.to_radians().tan()
            - 4.0 / self.aspect * x * (1.0 - self.taper) / (1.0 + self.taper);
        t.atan()
    }
}

#[derive(Clone, Debug)]
pub struct Strip {
    pub group: Group,
    pub ctrl: Ctrl,
    pub le: DVec3,        // leading-edge midpoint of the strip (body)
    pub chord_dir: DVec3, // unit, forward
    pub normal: DVec3,    // unit, "lift-positive" side
    pub span_dir: DVec3,  // chord × normal
    pub chord: f64,
    pub width: f64,
    pub area: f64,
    pub tau: f64,         // control effectiveness dα/dδ
    pub cm_delta: f64,    // ΔCm(c/4) per rad of control deflection
}

#[derive(Clone, Debug)]
pub struct GearLeg {
    pub attach: DVec3, // wheel contact point at full extension (body)
    pub k: f64,        // N/m
    pub c: f64,        // N·s/m
    pub max_stroke: f64,
    pub steerable: bool,
    pub braked: bool,
}

#[derive(Clone, Debug)]
pub struct AirframeConfig {
    pub wing: SurfaceParams,
    pub htail: SurfaceParams,
    pub vtail: SurfaceParams,
    pub fus_length: f64,
    pub fus_diam: f64,
    pub base_diam: f64,
    pub nose_x: f64,
    pub dry_mass: f64,
    pub dry_cg: DVec3,
    pub dry_inertia: DVec3, // (Ixx roll, Iyy yaw, Izz pitch) about dry CG, kg·m²
    pub ox_tank: DVec3,
    pub fuel_tank: DVec3,
    pub ox_volume: f64,
    pub fuel_volume: f64,
    pub tank_length: f64,
    pub rcs_tank: DVec3,
    pub rcs_capacity: f64,
    pub prop_load: f64,     // 0..1 fill fraction
    pub flap_span: f64,     // fraction of semispan (inboard)
    pub flap_chord: f64,    // cf / c
    pub speedbrake_area: f64,
    pub gear_cda: f64,
    pub nozzle_pivot: DVec3,
    pub downwash_k: f64,
    pub strips_per_side: usize,
}

impl Default for AirframeConfig {
    fn default() -> Self {
        AirframeConfig {
            wing: SurfaceParams {
                area: 18.6, aspect: 2.5, taper: 0.2, sweep_le_deg: 25.6, tc: 0.05,
                dihedral_deg: 0.0, x_root_le: 1.1, y_root: -0.1, wave_k: 16.0 / 3.0, te_bluntness: 0.0,
            },
            htail: SurfaceParams {
                area: 4.8, aspect: 2.8, taper: 0.3, sweep_le_deg: 45.0, tc: 0.045,
                dihedral_deg: -15.0, x_root_le: -4.6, y_root: -0.15, wave_k: 16.0 / 3.0, te_bluntness: 0.0,
            },
            vtail: SurfaceParams {
                // 10° wedge with blunt trailing edge, as on the X-15
                area: 4.0, aspect: 1.25, taper: 0.55, sweep_le_deg: 30.0, tc: 0.09,
                dihedral_deg: 90.0, x_root_le: -4.2, y_root: 0.6, wave_k: 4.0, te_bluntness: 0.09,
            },
            fus_length: 15.5,
            fus_diam: 1.4,
            base_diam: 1.3,
            nose_x: 8.0,
            dry_mass: 6620.0,
            dry_cg: DVec3::new(-0.45, 0.0, 0.0),
            dry_inertia: DVec3::new(5_300.0, 109_000.0, 106_000.0),
            ox_tank: DVec3::new(0.9, 0.0, 0.0),
            fuel_tank: DVec3::new(-1.7, 0.0, 0.0),
            ox_volume: 3.80,
            fuel_volume: 5.47,
            tank_length: 3.0,
            rcs_tank: DVec3::new(0.0, 0.2, 0.0),
            rcs_capacity: 160.0,
            prop_load: 1.0,
            flap_span: 0.45,
            flap_chord: 0.25,
            speedbrake_area: 1.6,
            gear_cda: 0.9,
            nozzle_pivot: DVec3::new(-7.0, 0.0, 0.0),
            downwash_k: 0.55,
            strips_per_side: 6,
        }
    }
}

/// Thin-airfoil flap theory: lift effectiveness τ and ΔCm about c/4.
fn flap_theory(cf_c: f64) -> (f64, f64) {
    let th = (2.0 * cf_c - 1.0).clamp(-1.0, 1.0).acos();
    let tau = 1.0 - (th - th.sin()) / std::f64::consts::PI;
    let dcm = -0.5 * th.sin() * (1.0 - th.cos());
    (tau, dcm)
}

impl AirframeConfig {
    pub fn build_strips(&self) -> Vec<Strip> {
        let mut out = Vec::new();
        let n = self.strips_per_side.max(2);
        let (flap_tau, flap_cm) = flap_theory(self.flap_chord);

        // wing and horizontal tail: mirrored panels
        for (group, p, nstrip) in [(Group::Wing, &self.wing, n), (Group::HTail, &self.htail, (n / 2).max(2))] {
            let semi = p.span() * 0.5;
            let cr = p.root_chord();
            let tan_le = p.sweep_le_deg.to_radians().tan();
            let dih = p.dihedral_deg.to_radians();
            for right in [false, true] {
                let side = if right { 1.0 } else { -1.0 };
                for i in 0..nstrip {
                    let e0 = i as f64 / nstrip as f64;
                    let e1 = (i + 1) as f64 / nstrip as f64;
                    let ec = 0.5 * (e0 + e1);
                    let c = cr * (1.0 - (1.0 - p.taper) * ec);
                    let width = semi * (e1 - e0);
                    let area = 0.5 * (cr * (1.0 - (1.0 - p.taper) * e0) + cr * (1.0 - (1.0 - p.taper) * e1)) * width;
                    let s = ec * semi;
                    let le = DVec3::new(p.x_root_le - s * tan_le, p.y_root + s * dih.sin(), side * s * dih.cos());
                    let normal = DVec3::new(0.0, dih.cos(), -side * dih.sin());
                    let chord_dir = DVec3::X;
                    let (ctrl, tau, cm_delta) = match group {
                        Group::Wing if ec <= self.flap_span => (Ctrl::Flap, flap_tau, flap_cm),
                        Group::Wing => (Ctrl::None, 0.0, 0.0),
                        _ => (if right { Ctrl::StabRight } else { Ctrl::StabLeft }, 1.0, 0.0),
                    };
                    out.push(Strip {
                        group, ctrl, le, chord_dir, normal,
                        span_dir: chord_dir.cross(normal),
                        chord: c, width, area, tau, cm_delta,
                    });
                }
            }
        }

        // vertical tail: single panel pointing up, upper 60 % all-moving rudder
        let p = &self.vtail;
        let h = p.span();
        let cr = p.root_chord();
        let tan_le = p.sweep_le_deg.to_radians().tan();
        let nv = (n / 2).max(3);
        for i in 0..nv {
            let e0 = i as f64 / nv as f64;
            let e1 = (i + 1) as f64 / nv as f64;
            let ec = 0.5 * (e0 + e1);
            let c = cr * (1.0 - (1.0 - p.taper) * ec);
            let width = h * (e1 - e0);
            let area = c * width;
            let s = ec * h;
            let normal = DVec3::new(0.0, 0.0, -1.0);
            let moving = ec > 0.4;
            out.push(Strip {
                group: Group::VTail,
                ctrl: if moving { Ctrl::Rudder } else { Ctrl::None },
                le: DVec3::new(p.x_root_le - s * tan_le, p.y_root + s, 0.0),
                chord_dir: DVec3::X,
                normal,
                span_dir: DVec3::X.cross(normal),
                chord: c, width, area,
                tau: if moving { 1.0 } else { 0.0 },
                cm_delta: 0.0,
            });
        }
        out
    }

    pub fn gear(&self) -> Vec<GearLeg> {
        let mk = |x: f64, z: f64, k: f64, c: f64, steer: bool| GearLeg {
            attach: DVec3::new(x, -1.6, z), k, c, max_stroke: 0.40, steerable: steer, braked: !steer,
        };
        vec![
            mk(5.0, 0.0, 1.6e5, 2.2e4, true),
            mk(-1.8, -1.5, 3.2e5, 4.0e4, false),
            mk(-1.8, 1.5, 3.2e5, 4.0e4, false),
        ]
    }

    /// Points that must never touch the ground (crash detection).
    pub fn hard_points(&self) -> Vec<DVec3> {
        let w = &self.wing;
        let ws = w.span() * 0.5;
        let wt = w.x_root_le - ws * w.sweep_le_deg.to_radians().tan();
        let t = &self.htail;
        let ts = t.span() * 0.5;
        let dih = t.dihedral_deg.to_radians();
        let tx = t.x_root_le - ts * t.sweep_le_deg.to_radians().tan();
        let r = self.fus_diam * 0.5;
        let tail_x = self.nose_x - self.fus_length;
        vec![
            DVec3::new(self.nose_x, 0.0, 0.0),
            DVec3::new(self.nose_x - 2.0, -r, 0.0),
            DVec3::new(0.0, -r, 0.0),
            DVec3::new(tail_x, -r, 0.0),
            DVec3::new(wt, w.y_root, ws),
            DVec3::new(wt, w.y_root, -ws),
            DVec3::new(tx, t.y_root + ts * dih.sin(), ts * dih.cos()),
            DVec3::new(tx, t.y_root + ts * dih.sin(), -ts * dih.cos()),
            DVec3::new(self.vtail.x_root_le, self.vtail.y_root + self.vtail.span(), 0.0),
        ]
    }

    pub fn wet_area_fuselage(&self) -> f64 {
        std::f64::consts::PI * self.fus_diam * self.fus_length * 0.85
    }
    pub fn frontal_area(&self) -> f64 {
        std::f64::consts::PI * 0.25 * self.fus_diam * self.fus_diam
    }
    pub fn base_area(&self) -> f64 {
        std::f64::consts::PI * 0.25 * self.base_diam * self.base_diam
    }
}

#[derive(Clone, Copy, Debug)]
pub struct MassProps {
    pub mass: f64,
    pub cg: DVec3,
    pub inertia: DMat3,
    pub inv_inertia: DMat3,
}

fn point_inertia(m: f64, d: DVec3) -> DMat3 {
    // m (|d|² I − d dᵀ), the parallel-axis tensor of a point mass
    let d2 = d.length_squared();
    DMat3::from_cols(
        DVec3::new(d2 - d.x * d.x, -d.y * d.x, -d.z * d.x),
        DVec3::new(-d.x * d.y, d2 - d.y * d.y, -d.z * d.y),
        DVec3::new(-d.x * d.z, -d.y * d.z, d2 - d.z * d.z),
    ) * m
}

/// Mass, CG and full inertia tensor from dry structure + current propellant.
pub fn mass_properties(cfg: &AirframeConfig, fuel: f64, ox: f64, rcs: f64) -> MassProps {
    let parts = [
        (cfg.dry_mass, cfg.dry_cg),
        (fuel, cfg.fuel_tank),
        (ox, cfg.ox_tank),
        (rcs, cfg.rcs_tank),
    ];
    let mass: f64 = parts.iter().map(|p| p.0).sum();
    let cg = parts.iter().map(|p| p.1 * p.0).fold(DVec3::ZERO, |a, b| a + b) / mass;

    let mut inertia = DMat3::from_diagonal(cfg.dry_inertia) + point_inertia(cfg.dry_mass, cfg.dry_cg - cg);
    for &(m, pos) in &parts[1..] {
        if m <= 0.0 {
            continue;
        }
        // liquid slug approximated as a slender cylinder along X
        let slug = m * cfg.tank_length * cfg.tank_length / 12.0;
        inertia += DMat3::from_diagonal(DVec3::new(m * 0.25 * 0.25, slug, slug)) + point_inertia(m, pos - cg);
    }
    MassProps { mass, cg, inertia, inv_inertia: inertia.inverse() }
}
