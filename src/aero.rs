//! Aerodynamic model: blade-element / strip theory over every lifting surface
//! plus a slender-body fuselage, valid from low subsonic to hypersonic speeds.
//!
//! Per strip the local relative wind is v_cg + ω×r − wind, so rotational
//! damping, dihedral effect, adverse sideslip and asymmetric stall emerge from
//! geometry instead of tabulated derivatives.
//!
//! Lift slope:   Helmbold/DATCOM (Prandtl–Glauert β) subsonic,
//!               Ackeret linear supersonic theory 4/√(M²−1) with tip loss,
//!               smooth transonic bridge.
//! Stall:        sigmoid blend to flat-plate Newtonian normal force.
//! Friction:     Blasius laminar / Schlichting turbulent Cf with transition
//!               at Re 5·10⁵ and Van Driest-style compressibility correction.
//! Form factor:  Raymer.
//! Wave drag:    Korn critical Mach + Lock 4th-power drag rise, Ackeret
//!               thickness drag supersonic, Sears–Haack area-rule fuselage.
//! Induced drag: CL²/(π e A) with Wieselsberger ground effect; supersonic
//!               drag-due-to-lift β·CL²/4.
//! Fuselage:     slender-body potential normal force + Allen–Perkins viscous
//!               crossflow, blunt-base drag with plume fill when the engine burns.

use crate::aircraft::{AirframeConfig, Ctrl, Group, Strip};
use crate::atmosphere::{Air, GAMMA_AIR};
use macroquad::math::DVec3;
use std::f64::consts::PI;

pub fn smoothstep(e0: f64, e1: f64, x: f64) -> f64 {
    let t = ((x - e0) / (e1 - e0)).clamp(0.0, 1.0);
    t * t * (3.0 - 2.0 * t)
}

/// Mean skin-friction coefficient of a flat plate, with transition.
pub fn skin_friction(re: f64, mach: f64) -> f64 {
    let re = re.max(1e3);
    let lam = 1.328 / re.sqrt();
    let turb = 0.455 / re.log10().powf(2.58) / (1.0 + 0.144 * mach * mach).powf(0.65);
    // Prandtl-Schlichting mixed laminar/turbulent with transition at 5e5
    let mixed = (turb - 1700.0 / re).max(lam);
    if re < 5e5 { lam } else { mixed }
}

/// 3D lift-curve slope per radian of a finite wing.
pub fn lift_slope(aspect: f64, sweep_half: f64, mach: f64) -> f64 {
    let sub = |m: f64| {
        let b2 = (1.0 - m * m).max(0.04);
        let k = 0.95; // section lift slope / 2π
        let t = sweep_half.tan();
        2.0 * PI * aspect
            / (2.0 + (4.0 + aspect * aspect * b2 / (k * k) * (1.0 + t * t / b2)).sqrt())
    };
    let sup = |m: f64| {
        let beta = (m * m - 1.0).sqrt();
        let tip = (1.0 - 1.0 / (2.0 * aspect * beta)).clamp(0.45, 1.0);
        4.0 / beta * tip
    };
    if mach <= 0.9 {
        sub(mach)
    } else if mach >= 1.25 {
        sup(mach)
    } else {
        // transonic: blend (linear theories both diverge here)
        let w = smoothstep(0.9, 1.25, mach);
        let peak = sub(0.9).max(sup(1.25)) * 1.05;
        let mid = 1.0 - (2.0 * w - 1.0).abs();
        (1.0 - w) * sub(0.9) + w * sup(1.25) + mid * (peak - 0.5 * (sub(0.9) + sup(1.25))) * 0.5
    }
}

/// Wave drag (thickness only) of a thin section: Korn/Lock transonic,
/// Ackeret supersonic. `mn` is the Mach number normal to the sweep line.
pub fn section_wave_drag(mn: f64, tc: f64, cl: f64, wave_k: f64) -> f64 {
    let ka = 0.87; // Korn technology factor (conventional section)
    let mdd = ka - tc - cl.abs() / 10.0;
    let mcrit = mdd - (0.1f64 / 80.0).powf(1.0 / 3.0);
    let ackeret = |m: f64| wave_k * tc * tc / (m * m - 1.0).max(0.04).sqrt();
    // Lock's law only holds for the onset of drag rise; cap it at the
    // transonic peak, which sits a little above the low-supersonic value.
    let peak = 1.3 * ackeret(1.15);
    let lock = |m: f64| if m > mcrit { (20.0 * (m - mcrit).powi(4)).min(peak) } else { 0.0 };
    if mn <= 1.0 {
        lock(mn)
    } else if mn >= 1.15 {
        ackeret(mn)
    } else {
        let w = smoothstep(1.0, 1.15, mn);
        (1.0 - w) * lock(1.0).max(peak) + w * ackeret(mn)
    }
}

/// Raymer form factor for a wing section.
fn wing_form_factor(tc: f64, mach: f64, sweep_tmax: f64) -> f64 {
    let xc = 0.4; // chordwise position of max thickness
    (1.0 + 0.6 / xc * tc + 100.0 * tc.powi(4)) * 1.34 * mach.max(0.05).powf(0.18) * sweep_tmax.cos().powf(0.28)
}

/// Oswald span efficiency (Raymer).
pub fn oswald(aspect: f64, sweep_le: f64) -> f64 {
    let straight = 1.78 * (1.0 - 0.045 * aspect.powf(0.68)) - 0.64;
    let swept = 4.61 * (1.0 - 0.045 * aspect.powf(0.68)) * sweep_le.cos().powf(0.15) - 3.1;
    let w = smoothstep(0.0, 30f64.to_radians(), sweep_le);
    ((1.0 - w) * straight + w * swept).clamp(0.55, 0.92)
}

#[derive(Clone, Copy, Debug, Default)]
pub struct Controls {
    pub stab_left: f64,  // rad, + = leading edge up (more tail lift)
    pub stab_right: f64,
    pub rudder: f64,     // rad, + = force toward −Z (nose right)
    pub flap: f64,       // rad, + = trailing edge down
    pub speedbrake: f64, // 0..1
    pub gear_down: f64,  // 0..1
}

#[derive(Clone, Copy, Debug, Default)]
pub struct StripViz {
    pub pos: DVec3,
    pub lift: DVec3,
    pub drag: DVec3,
    pub stalled: f64,
}

#[derive(Clone, Debug, Default)]
pub struct AeroOut {
    pub force: DVec3,  // body
    pub moment: DVec3, // body, about CG
    pub lift: f64,
    pub drag: f64,
    pub side: f64,
    pub cl_wing: f64,
    pub alpha: f64,
    pub beta: f64,
    pub mach: f64,
    pub qbar: f64,
    pub tas: f64,
    pub re_mac: f64,
    pub cd0: f64,
    pub cd_wave: f64,
    pub cd_induced: f64,
    pub cd_base: f64,
    pub stall_frac: f64,
    pub strips: Vec<StripViz>,
}

pub struct AeroInput<'a> {
    pub cfg: &'a AirframeConfig,
    pub strips: &'a [Strip],
    pub air: &'a Air,
    pub v_air: DVec3,   // body-frame velocity of CG relative to the air mass
    pub omega: DVec3,   // body rates
    pub cg: DVec3,
    pub ctrl: Controls,
    pub downwash: f64,  // rad at the horizontal tail
    pub height: f64,    // wing height above ground (m)
    pub engine_on: bool,
    pub nozzle_exit_area: f64,
    pub record: bool,
}

struct SurfaceCache {
    cla: f64,
    alpha_stall: f64,
    e: f64,
    aspect_eff: f64,
    sweep_le: f64,
    sweep_tmax: f64,
    tc: f64,
    wave_k: f64,
    te_blunt: f64,
    span: f64,
}

pub fn evaluate(inp: &AeroInput) -> AeroOut {
    let cfg = inp.cfg;
    let air = inp.air;
    let mut out = AeroOut::default();
    let speed = inp.v_air.length();
    out.tas = speed;
    out.mach = speed / air.speed_of_sound;
    out.qbar = 0.5 * air.density * speed * speed;
    if speed > 1e-3 {
        out.alpha = (-inp.v_air.y).atan2(inp.v_air.x);
        out.beta = (inp.v_air.z / speed).clamp(-1.0, 1.0).asin();
    }
    out.re_mac = air.density * speed * cfg.wing.mac() / air.viscosity;

    let mach = out.mach;
    let supersonic_w = smoothstep(0.95, 1.25, mach);
    // aerodynamic centre: quarter chord subsonic, half chord supersonic
    let ac_frac = 0.25 + 0.25 * smoothstep(0.85, 1.4, mach);

    let cache = |p: &crate::aircraft::SurfaceParams, aspect_eff: f64| {
        let sweep_half = p.sweep_at(0.5);
        let a = lift_slope(aspect_eff, sweep_half, mach);
        // low aspect ratio wings stall later (vortex lift)
        let alpha_stall = (11.0 + 24.0 / aspect_eff.max(0.5)).min(32.0).to_radians();
        SurfaceCache {
            cla: a,
            alpha_stall,
            e: oswald(aspect_eff, p.sweep_le_deg.to_radians()),
            aspect_eff,
            sweep_le: p.sweep_le_deg.to_radians(),
            sweep_tmax: p.sweep_at(0.4),
            tc: p.tc,
            wave_k: p.wave_k,
            te_blunt: p.te_bluntness,
            span: p.span(),
        }
    };
    let wing_c = cache(&cfg.wing, cfg.wing.aspect);
    let ht_c = cache(&cfg.htail, cfg.htail.aspect);
    // fuselage and h-tail act as an end plate on the fin
    let vt_c = cache(&cfg.vtail, cfg.vtail.aspect * 1.55);

    // Wieselsberger ground effect on induced drag
    let hb = (inp.height / wing_c.span).max(0.01);
    let ge = (16.0 * hb).powi(2) / (1.0 + (16.0 * hb).powi(2));

    let mut force = DVec3::ZERO;
    let mut moment = DVec3::ZERO;
    let mut wing_lift_sum = 0.0;
    let mut wing_area_sum = 0.0;
    let mut stall_acc = 0.0;
    let mut cd0_acc = 0.0;
    let mut cdw_acc = 0.0;
    let mut cdi_acc = 0.0;

    for s in inp.strips {
        let sc = match s.group {
            Group::Wing => &wing_c,
            Group::HTail => &ht_c,
            Group::VTail => &vt_c,
        };
        let ac = s.le - s.chord_dir * (ac_frac * s.chord);
        let r = ac - inp.cg;
        let v = inp.v_air + inp.omega.cross(r);
        let vp = v - s.span_dir * v.dot(s.span_dir);
        let vmag = vp.length();
        if vmag < 0.5 {
            continue;
        }
        let q = 0.5 * air.density * vmag * vmag;
        let m_local = vmag / air.speed_of_sound;
        let re = air.density * vmag * s.chord / air.viscosity;

        let delta = match s.ctrl {
            Ctrl::None => 0.0,
            Ctrl::Flap => inp.ctrl.flap,
            Ctrl::StabLeft => inp.ctrl.stab_left,
            Ctrl::StabRight => inp.ctrl.stab_right,
            Ctrl::Rudder => inp.ctrl.rudder,
        };
        // large flap deflections lose effectiveness (flow separation on the flap)
        let tau = if s.ctrl == Ctrl::Flap {
            s.tau * (1.0 - 0.35 * smoothstep(10f64.to_radians(), 40f64.to_radians(), delta.abs()))
        } else {
            s.tau
        };

        let mut alpha = (-vp.dot(s.normal)).atan2(vp.dot(s.chord_dir));
        if s.group == Group::HTail {
            alpha -= inp.downwash;
        }
        let alpha_eff = alpha + tau * delta;
        // wrap to (-π, π] so reversed flow behaves like a flat plate
        let alpha_eff = (alpha_eff + PI).rem_euclid(2.0 * PI) - PI;

        // attached-flow lift with sigmoid transition to flat-plate (post-stall)
        let a_s = sc.alpha_stall - 0.4 * (tau * delta).max(0.0) * (1.0 - supersonic_w);
        let m_sig = 18.0;
        let sigma = {
            let e1 = (-m_sig * (alpha_eff - a_s)).exp();
            let e2 = (m_sig * (alpha_eff + a_s)).exp();
            ((1.0 + e1 + e2) / ((1.0 + e1) * (1.0 + e2))).clamp(0.0, 1.0)
        };
        let cl_lin = sc.cla * alpha_eff;
        let cn_fp = 1.25 + 0.6 * supersonic_w; // flat-plate normal force coefficient
        let cl_fp = cn_fp * alpha_eff.sin() * alpha_eff.cos();
        let cl = (1.0 - sigma) * cl_lin + sigma * cl_fp;

        // drag build-up
        let cf = skin_friction(re, m_local);
        let cd0 = cf * wing_form_factor(sc.tc, m_local, sc.sweep_tmax) * 2.04;
        let mn = m_local * sc.sweep_le.cos();
        let cdw = section_wave_drag(mn, sc.tc, cl, sc.wave_k) * sc.sweep_le.cos().powi(2);
        let cd_te = if sc.te_blunt > 0.0 {
            // base pressure behind a blunt trailing edge: −Cp_base ≈ 0.13 subsonic,
            // tends toward 0.7 × vacuum limit 2/(γM²) supersonic
            let cpb = if m_local < 1.0 { 0.13 + 0.12 * m_local * m_local } else {
                (0.25 / m_local).min(0.7 * 2.0 / (GAMMA_AIR * m_local * m_local))
            };
            cpb * sc.te_blunt
        } else {
            0.0
        };
        let k_sub = ge / (PI * sc.e * sc.aspect_eff);
        let beta_s = (m_local * m_local - 1.0).max(0.0).sqrt();
        let k_sup = beta_s / 4.0;
        let k_ind = (1.0 - supersonic_w) * k_sub + supersonic_w * k_sup.max(k_sub);
        let cdi = (1.0 - sigma) * k_ind * cl * cl;
        let cd_fp = sigma * cn_fp * alpha_eff.sin().powi(2);
        let cd_flap = if s.ctrl == Ctrl::Flap { 1.7 * cfg.flap_chord.powf(1.38) * delta.sin().powi(2) } else { 0.0 };
        let cd = cd0 + cdw + cd_te + cdi + cd_fp + cd_flap;

        let drag_dir = -vp / vmag;
        let lift_dir = s.span_dir.cross(vp).normalize_or_zero();
        let lift = lift_dir * (q * s.area * cl);
        let drag = drag_dir * (q * s.area * cd);
        // control-surface camber pitching moment about the local span axis
        let cm = s.cm_delta * delta * (1.0 - supersonic_w);
        let m_pitch = s.span_dir * (q * s.area * s.chord * cm);

        force += lift + drag;
        moment += r.cross(lift + drag) + m_pitch;

        let w = s.area / cfg.wing.area;
        stall_acc += sigma * s.area;
        cd0_acc += cd0 * w * q / out.qbar.max(1e-6);
        cdw_acc += (cdw + cd_te) * w * q / out.qbar.max(1e-6);
        cdi_acc += (cdi + cd_fp + cd_flap) * w * q / out.qbar.max(1e-6);
        if s.group == Group::Wing {
            wing_lift_sum += cl * s.area;
            wing_area_sum += s.area;
        }
        if inp.record {
            out.strips.push(StripViz { pos: ac, lift, drag, stalled: sigma });
        }
    }
    out.cl_wing = if wing_area_sum > 0.0 { wing_lift_sum / wing_area_sum } else { 0.0 };
    let total_area: f64 = inp.strips.iter().map(|s| s.area).sum();
    out.stall_frac = stall_acc / total_area.max(1e-9);

    // ---------------- fuselage (body of revolution) ----------------
    if speed > 0.5 {
        let q = out.qbar;
        let a_ref = cfg.frontal_area();
        let ux = inp.v_air.x / speed;
        let alpha_c = ux.clamp(-1.0, 1.0).acos(); // total angle of attack
        let fineness = cfg.fus_length / cfg.fus_diam;

        let re_l = air.density * speed * cfg.fus_length / air.viscosity;
        let cf = skin_friction(re_l, mach);
        let ff = 1.0 + 60.0 / fineness.powi(3) + fineness / 400.0;
        let cd_fric = cf * ff * cfg.wet_area_fuselage() / a_ref;

        // Sears–Haack minimum wave drag × empirical shape factor E_WD
        let sh = 9.0 * PI / 2.0 * (a_ref / cfg.fus_length).powi(2) / a_ref;
        let e_wd = 2.0;
        let decay = (1.0 - 0.06 * (mach - 1.2).max(0.0)).max(0.55);
        let cd_wave = sh * e_wd * smoothstep(0.8, 1.05, mach) * decay;

        // blunt base: the rocket plume fills the base when the engine burns
        let cpb = if mach < 1.0 { 0.12 + 0.13 * mach * mach } else {
            (0.25 / mach).min(0.7 * 2.0 / (GAMMA_AIR * mach * mach))
        };
        let base_open = if inp.engine_on {
            0.25 * (cfg.base_area() - inp.nozzle_exit_area).max(0.0)
        } else {
            cfg.base_area()
        };
        let cd_base = cpb * base_open / a_ref;

        let cd_axial = (cd_fric + cd_wave + cd_base) * alpha_c.cos().powi(2);
        let drag = -inp.v_air / speed * (q * a_ref * cd_axial);
        let x_mid = cfg.nose_x - 0.5 * cfg.fus_length;
        let r_mid = DVec3::new(x_mid, 0.0, 0.0) - inp.cg;
        force += drag;
        moment += r_mid.cross(drag);

        // crossflow: slender-body potential + viscous (Allen & Perkins)
        let v_perp = DVec3::new(0.0, inp.v_air.y, inp.v_air.z);
        if v_perp.length() > 1e-4 {
            let n_dir = -v_perp.normalize();
            let mc = mach * alpha_c.sin();
            let cdc = 1.2 + 0.5 * smoothstep(0.4, 1.0, mc) - 0.2 * smoothstep(1.5, 4.0, mc);
            let eta = 0.68; // finite-length crossflow factor for L/D ≈ 11
            let a_plan = cfg.fus_diam * cfg.fus_length * 0.92;
            let cn_pot = (cfg.base_area() / a_ref) * (2.0 * alpha_c).sin() * (alpha_c / 2.0).cos();
            let cn_visc = eta * cdc * (a_plan / a_ref) * alpha_c.sin().powi(2);
            let f_pot = n_dir * (q * a_ref * cn_pot);
            let f_visc = n_dir * (q * a_ref * cn_visc);
            let r_pot = DVec3::new(cfg.nose_x - 2.2, 0.0, 0.0) - inp.cg;
            let r_visc = DVec3::new(x_mid + 0.6, 0.0, 0.0) - inp.cg;
            force += f_pot + f_visc;
            moment += r_pot.cross(f_pot) + r_visc.cross(f_visc);
        }

        // fuselage pitch/yaw damping from crossflow due to rotation
        let l = cfg.fus_length;
        let damp = -0.5 * air.density * speed * 1.2 * cfg.fus_diam * l.powi(3) / 24.0;
        moment += DVec3::new(0.0, inp.omega.y * damp, inp.omega.z * damp);

        // speed brakes and landing gear: bluff-body drag
        let bluff = 1.2 * cfg.speedbrake_area * inp.ctrl.speedbrake + cfg.gear_cda * inp.ctrl.gear_down;
        let f_bluff = -inp.v_air / speed * (q * bluff);
        let r_sb = DVec3::new(-5.5, 0.0, 0.0) - inp.cg;
        force += f_bluff;
        moment += r_sb.cross(f_bluff);

        let ws = cfg.wing.area;
        cd0_acc += (cd_fric * a_ref + bluff) / ws;
        cdw_acc += cd_wave * a_ref / ws;
        out.cd_base = cd_base * a_ref / ws;
    }

    out.cd0 = cd0_acc;
    out.cd_wave = cdw_acc;
    out.cd_induced = cdi_acc;
    out.force = force;
    out.moment = moment;

    // wind-axis decomposition for the HUD
    if speed > 0.5 {
        let vhat = inp.v_air / speed;
        out.drag = -force.dot(vhat);
        let side_ax = (DVec3::Z - vhat * vhat.z).normalize_or_zero();
        let up_ax = side_ax.cross(vhat);
        out.lift = force.dot(up_ax);
        out.side = force.dot(side_ax);
    }
    out
}
