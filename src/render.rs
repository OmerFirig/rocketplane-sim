//! 3D rendering. Everything is drawn camera-relative around the aircraft CG
//! (f64 world → f32 offsets) and in two depth passes: a far pass for terrain
//! and scenery and a near pass for the aircraft, plume and force vectors.

use crate::aircraft::{Ctrl, Group};
use crate::sim::Sim;
use macroquad::math::{DQuat, DVec3};
use macroquad::models::{draw_mesh, Mesh, Vertex};
use macroquad::prelude::*;

#[derive(Clone, Copy, PartialEq, Eq, Debug)]
pub enum CamMode {
    Chase,
    Orbit,
    Cockpit,
    Tower,
}

impl CamMode {
    pub fn next(self) -> CamMode {
        match self {
            CamMode::Chase => CamMode::Orbit,
            CamMode::Orbit => CamMode::Cockpit,
            CamMode::Cockpit => CamMode::Tower,
            CamMode::Tower => CamMode::Chase,
        }
    }
    pub fn name(self) -> &'static str {
        match self {
            CamMode::Chase => "chase",
            CamMode::Orbit => "orbit",
            CamMode::Cockpit => "cockpit",
            CamMode::Tower => "tower",
        }
    }
}

pub struct View {
    pub mode: CamMode,
    pub orbit_yaw: f32,
    pub orbit_pitch: f32,
    pub orbit_dist: f32,
    pub chase_dir: Vec3,
    /// mouse-look offsets for the chase and cockpit cameras (rad)
    pub look_yaw: f32,
    pub look_pitch: f32,
    pub show_forces: bool,
    pub mountains: Vec<(DVec3, f32, f32)>,
    pub flicker: f32,
}

impl View {
    pub fn new() -> View {
        // deterministic ring of mountains for horizon reference
        let mut mountains = Vec::new();
        let mut seed = 12345u32;
        let mut rnd = || {
            seed = seed.wrapping_mul(1_103_515_245).wrapping_add(12345);
            (seed >> 8) as f32 / (1u32 << 24) as f32
        };
        for _ in 0..90 {
            let ang = rnd() * std::f32::consts::TAU;
            let r = 25_000.0 + rnd() * 120_000.0;
            let h = 300.0 + rnd() * 1500.0;
            let w = h * (2.0 + rnd() * 3.0);
            mountains.push((DVec3::new((ang.cos() * r) as f64, 0.0, (ang.sin() * r) as f64), h, w));
        }
        View {
            mode: CamMode::Chase,
            orbit_yaw: 0.6,
            orbit_pitch: 0.25,
            orbit_dist: 32.0,
            chase_dir: vec3(0.0, 0.0, 1.0),
            look_yaw: 0.0,
            look_pitch: 0.0,
            show_forces: false,
            mountains,
            flicker: 0.0,
        }
    }
}

fn f(v: DVec3) -> Vec3 {
    vec3(v.x as f32, v.y as f32, v.z as f32)
}

struct MeshBuilder {
    verts: Vec<Vertex>,
    idx: Vec<u16>,
    rot: DQuat,
    origin: DVec3,
    sun: Vec3,
}

impl MeshBuilder {
    fn new(rot: DQuat, origin: DVec3) -> Self {
        MeshBuilder { verts: Vec::new(), idx: Vec::new(), rot, origin, sun: vec3(0.4, 0.8, 0.3).normalize() }
    }
    fn xf(&self, p: DVec3) -> Vec3 {
        f(self.rot * (p - self.origin))
    }
    fn flush(&mut self) {
        if !self.idx.is_empty() {
            let mesh = Mesh { vertices: std::mem::take(&mut self.verts), indices: std::mem::take(&mut self.idx), texture: None };
            draw_mesh(&mesh);
        }
    }
    /// Triangle in body coordinates, flat-shaded (two-sided).
    fn tri(&mut self, a: DVec3, b: DVec3, c: DVec3, col: Color) {
        if self.verts.len() > 60_000 {
            self.flush();
        }
        let (pa, pb, pc) = (self.xf(a), self.xf(b), self.xf(c));
        let n = (pb - pa).cross(pc - pa).normalize_or_zero();
        let shade = 0.38 + 0.62 * n.dot(self.sun).abs();
        let col = Color::new(col.r * shade, col.g * shade, col.b * shade, col.a);
        let base = self.verts.len() as u16;
        for p in [pa, pb, pc] {
            self.verts.push(Vertex::new(p.x, p.y, p.z, 0.0, 0.0, col));
        }
        self.idx.extend_from_slice(&[base, base + 1, base + 2]);
    }
    fn quad(&mut self, a: DVec3, b: DVec3, c: DVec3, d: DVec3, col: Color) {
        self.tri(a, b, c, col);
        self.tri(a, c, d, col);
    }
    /// Unlit triangle already in render space (for plumes).
    fn tri_raw(&mut self, a: Vec3, b: Vec3, c: Vec3, ca: Color, cb: Color, cc: Color) {
        let base = self.verts.len() as u16;
        self.verts.push(Vertex::new(a.x, a.y, a.z, 0.0, 0.0, ca));
        self.verts.push(Vertex::new(b.x, b.y, b.z, 0.0, 0.0, cb));
        self.verts.push(Vertex::new(c.x, c.y, c.z, 0.0, 0.0, cc));
        self.idx.extend_from_slice(&[base, base + 1, base + 2]);
    }
}

fn rotate_about(p: DVec3, pivot: DVec3, axis: DVec3, ang: f64) -> DVec3 {
    pivot + DQuat::from_axis_angle(axis.normalize(), ang) * (p - pivot)
}

/// Blackbody-ish tint of the Inconel skin as it heats up.
fn skin_color(temp: f64) -> Color {
    let base = Color::new(0.20, 0.20, 0.22, 1.0);
    let t = ((temp - 650.0) / 900.0).clamp(0.0, 1.0) as f32;
    Color::new(base.r + t * 0.9, base.g + t * t * 0.35, base.b + t * t * t * 0.1, 1.0)
}

fn camera_setup(sim: &Sim, view: &mut View, dt: f32) -> (Vec3, Vec3, Vec3) {
    let fwd = f(sim.att * DVec3::X);
    let up = f(sim.att * DVec3::Y);
    match view.mode {
        CamMode::Chase => {
            let v = f(sim.vel);
            let want = if v.length() > 20.0 { -v.normalize() * 0.5 - fwd * 0.5 } else { -fwd };
            let want = want.normalize_or_zero();
            let k = (dt * 3.0).min(1.0);
            view.chase_dir = (view.chase_dir + (want - view.chase_dir) * k).normalize_or_zero();
            let base = view.chase_dir * 28.0 + vec3(0.0, 6.0, 0.0);
            // rotate the trailing position around the aircraft by the mouse-look offsets
            let dist = base.length() * view.orbit_dist / 32.0;
            let yaw = base.z.atan2(base.x) + view.look_yaw;
            let el = ((base.y / base.length()).asin() + view.look_pitch).clamp(-1.45, 1.45);
            let pos = vec3(yaw.cos() * el.cos(), el.sin(), yaw.sin() * el.cos()) * dist;
            (pos, fwd * 6.0, Vec3::Y)
        }
        CamMode::Orbit => {
            let (cy, sy) = (view.orbit_yaw.cos(), view.orbit_yaw.sin());
            let (cp, sp) = (view.orbit_pitch.cos(), view.orbit_pitch.sin());
            let pos = vec3(cy * cp, sp, sy * cp) * view.orbit_dist;
            (pos, Vec3::ZERO, Vec3::Y)
        }
        CamMode::Cockpit => {
            let eye = f(sim.att * (DVec3::new(sim.cfg.nose_x - 3.4, 1.05, 0.0) - sim.mp.cg));
            // look around inside the cockpit: yaw about body-up, pitch toward body-up
            let right = f(sim.att * DVec3::Z);
            let (y, p) = (view.look_yaw, -view.look_pitch.clamp(-1.4, 1.4));
            let dir = (fwd * y.cos() + right * y.sin()) * p.cos() + up * p.sin();
            (eye, eye + dir * 100.0, up)
        }
        CamMode::Tower => {
            let tower = DVec3::new(250.0, 30.0, -1200.0);
            let rel = f(tower - sim.pos);
            (rel, Vec3::ZERO, Vec3::Y)
        }
    }
}

fn sky_color(alt: f64) -> Color {
    let r = (crate::atmosphere::isa(alt, 0.0).density / 1.225).powf(0.35) as f32;
    Color::new(0.02 + 0.50 * r, 0.03 + 0.70 * r, 0.08 + 0.87 * r, 1.0)
}

pub fn draw_world(sim: &Sim, view: &mut View, dt: f32, x_np: f64) {
    clear_background(sky_color(sim.pos.y));
    let (eye, target, up) = camera_setup(sim, view, dt);
    let eye_world = sim.pos + DVec3::new(eye.x as f64, eye.y as f64, eye.z as f64);
    let cam_h = eye_world.y.max(0.5) as f32;
    let fovy = if view.mode == CamMode::Tower {
        // tower camera zooms on the aircraft
        (40.0 / eye.length().max(40.0)).clamp(0.01, 1.0) * 1.2
    } else {
        1.05
    };

    // ---------- far pass: ground and scenery ----------
    set_camera(&Camera3D {
        position: eye,
        target,
        up,
        fovy,
        z_near: (cam_h * 0.25).clamp(0.3, 5_000.0),
        z_far: 4.0e6,
        ..Default::default()
    });
    let o = sim.pos;
    let g = -o.y as f32;
    draw_plane(vec3(0.0, g - 0.5, 0.0), vec2(2.0e6, 2.0e6), None, Color::new(0.74, 0.66, 0.50, 1.0));

    // moving ground grid (spacing scales with altitude for speed cues)
    let alt = o.y.max(1.0);
    let spacing = 10f64.powf((alt / 2.0).log10().ceil().max(1.7));
    let n = 40;
    let cx = (o.x / spacing).round() * spacing;
    let cz = (o.z / spacing).round() * spacing;
    let ext = spacing * n as f64;
    let gc = Color::new(0.62, 0.55, 0.42, 1.0);
    for i in -n..=n {
        let x = (cx + i as f64 * spacing - o.x) as f32;
        let z = (cz + i as f64 * spacing - o.z) as f32;
        draw_line_3d(vec3(x, g - 0.3, (cz - ext - o.z) as f32), vec3(x, g - 0.3, (cz + ext - o.z) as f32), gc);
        draw_line_3d(vec3((cx - ext - o.x) as f32, g - 0.3, z), vec3((cx + ext - o.x) as f32, g - 0.3, z), gc);
    }

    // runway 3000 × 60 m along −Z (north)
    let rw = |p: DVec3| f(p - o);
    draw_plane(rw(DVec3::new(0.0, 0.05, -1500.0)), vec2(30.0, 1500.0), None, Color::new(0.25, 0.25, 0.27, 1.0));
    for i in 0..50 {
        let z = -30.0 - i as f64 * 60.0;
        draw_plane(rw(DVec3::new(0.0, 0.1, z)), vec2(0.6, 15.0), None, WHITE);
    }
    for k in 0..8 {
        let x = -25.0 + k as f64 * 7.0;
        for zc in [-15.0, -2985.0] {
            draw_plane(rw(DVec3::new(x + 2.0, 0.1, zc)), vec2(1.2, 12.0), None, WHITE);
        }
    }
    // hangars and tower
    for (i, x) in [180.0, 240.0, 300.0].iter().enumerate() {
        draw_cube(rw(DVec3::new(*x, 9.0, -600.0 - i as f64 * 80.0)), vec3(50.0, 18.0, 60.0), None, Color::new(0.55, 0.56, 0.6, 1.0));
    }
    draw_cube(rw(DVec3::new(250.0, 15.0, -1200.0)), vec3(8.0, 30.0, 8.0), None, Color::new(0.8, 0.8, 0.82, 1.0));
    // distance markers every 500 m alongside
    for i in 0..7 {
        draw_cube(rw(DVec3::new(-45.0, 1.0, -(i as f64) * 500.0)), vec3(2.0, 2.0, 2.0), None, Color::new(0.9, 0.4, 0.1, 1.0));
    }
    // mountains
    let mut mb = MeshBuilder::new(DQuat::IDENTITY, o);
    for (c, h, w) in &view.mountains {
        let (h, w) = (*h as f64, *w as f64);
        let top = *c + DVec3::new(0.0, h, 0.0);
        let col = Color::new(0.55, 0.47, 0.40, 1.0);
        let p = [
            *c + DVec3::new(w, -1.0, 0.0),
            *c + DVec3::new(0.0, -1.0, w * 0.8),
            *c + DVec3::new(-w * 0.9, -1.0, 0.0),
            *c + DVec3::new(0.0, -1.0, -w),
        ];
        for i in 0..4 {
            mb.tri(p[i], p[(i + 1) % 4], top, col);
        }
    }
    mb.flush();

    // depth-only clear, then the near pass
    unsafe {
        let mut gl = get_internal_gl();
        gl.flush();
        gl.quad_context.clear(None, Some(1.0), None);
    }
    set_camera(&Camera3D {
        position: eye,
        target,
        up,
        fovy,
        z_near: 0.1,
        z_far: 20_000.0,
        ..Default::default()
    });
    draw_aircraft(sim, view);
    if view.show_forces {
        draw_forces(sim);
        draw_np_marker(sim, x_np);
    }
}

fn draw_aircraft(sim: &Sim, view: &mut View) {
    let cfg = &sim.cfg;
    let mut mb = MeshBuilder::new(sim.att, sim.mp.cg);
    let skin = skin_color(sim.skin_temp);

    // fuselage: elliptical cross-sections lofted along X
    let seg = 14;
    let stations: Vec<(f64, f64)> = {
        let l = cfg.fus_length;
        let nose = cfg.nose_x;
        let r = cfg.fus_diam * 0.5;
        let mut v = vec![];
        for i in 0..=20 {
            let t = i as f64 / 20.0;
            let x = nose - t * l;
            let nose_len = 3.8;
            let d = nose - x;
            let rr = if d < nose_len {
                // tangent ogive
                let s = d / nose_len;
                r * (s * (2.0 - s)).sqrt().max(0.02)
            } else {
                r * (1.0 - 0.07 * ((d - nose_len) / (l - nose_len)))
            };
            v.push((x, rr));
        }
        v
    };
    let ring = |x: f64, r: f64, k: usize| {
        let a = k as f64 / seg as f64 * std::f64::consts::TAU;
        // side tunnels make the X-15 fuselage wider than tall
        DVec3::new(x, r * a.sin(), r * 1.22 * a.cos())
    };
    for w in stations.windows(2) {
        let (x0, r0) = w[0];
        let (x1, r1) = w[1];
        for k in 0..seg {
            mb.quad(ring(x0, r0, k), ring(x0, r0, k + 1), ring(x1, r1, k + 1), ring(x1, r1, k), skin);
        }
    }
    // base
    let (xb, rb) = *stations.last().unwrap();
    for k in 0..seg {
        mb.tri(DVec3::new(xb, 0.0, 0.0), ring(xb, rb, k), ring(xb, rb, k + 1), Color::new(0.1, 0.1, 0.1, 1.0));
    }
    // canopy
    let cx = cfg.nose_x - 3.6;
    let cc = Color::new(0.15, 0.25, 0.35, 1.0);
    let c0 = DVec3::new(cx + 1.2, 0.55, 0.0);
    let c1 = DVec3::new(cx - 0.9, 0.62, 0.0);
    let ct = DVec3::new(cx - 0.2, 0.92, 0.0);
    mb.tri(c0, ct, DVec3::new(cx - 0.2, 0.6, 0.38), cc);
    mb.tri(c0, ct, DVec3::new(cx - 0.2, 0.6, -0.38), cc);
    mb.tri(c1, ct, DVec3::new(cx - 0.2, 0.6, 0.38), cc);
    mb.tri(c1, ct, DVec3::new(cx - 0.2, 0.6, -0.38), cc);

    // lifting surfaces drawn per strip, so deflections animate per panel
    for s in &sim.strips {
        let half = s.span_dir * (s.width * 0.5);
        let le_a = s.le - half;
        let le_b = s.le + half;
        let te_a = le_a - s.chord_dir * s.chord;
        let te_b = le_b - s.chord_dir * s.chord;
        let tc = match s.group { Group::Wing => cfg.wing.tc, Group::HTail => cfg.htail.tc, Group::VTail => cfg.vtail.tc };
        let th = s.normal * (tc * s.chord * 0.5);
        let (delta, hinge) = match s.ctrl {
            Ctrl::StabLeft => (sim.ctrl.stab_left, 0.45),
            Ctrl::StabRight => (sim.ctrl.stab_right, 0.45),
            Ctrl::Rudder => (sim.ctrl.rudder, 0.4),
            Ctrl::Flap => (sim.ctrl.flap, 1.0 - cfg.flap_chord),
            Ctrl::None => (0.0, 1.0),
        };
        let col = match s.group {
            Group::Wing => skin,
            _ => Color::new(skin.r * 0.9, skin.g * 0.9, skin.b * 0.95, 1.0),
        };
        let mid_a = le_a - s.chord_dir * (s.chord * 0.4);
        let mid_b = le_b - s.chord_dir * (s.chord * 0.4);
        let hinge_a = le_a - s.chord_dir * (s.chord * hinge);
        let rot = |p: DVec3| {
            let fwd = (p - hinge_a).dot(s.chord_dir);
            if s.ctrl == Ctrl::Flap && fwd > -1e-6 {
                p // flap: only the part aft of the hinge moves
            } else if s.ctrl == Ctrl::None {
                p
            } else {
                rotate_about(p, hinge_a, s.span_dir, delta)
            }
        };
        if s.ctrl == Ctrl::Flap {
            let hinge_b = le_b - s.chord_dir * (s.chord * hinge);
            mb.quad(le_a, le_b, mid_b + th, mid_a + th, col);
            mb.quad(le_a, le_b, mid_b - th, mid_a - th, col);
            mb.quad(mid_a + th, mid_b + th, hinge_b + th * 0.6, hinge_a + th * 0.6, col);
            mb.quad(mid_a - th, mid_b - th, hinge_b - th * 0.6, hinge_a - th * 0.6, col);
            let fc = Color::new(0.35, 0.33, 0.30, 1.0);
            mb.quad(hinge_a, hinge_b, rot(te_b), rot(te_a), fc);
        } else {
            mb.quad(rot(le_a), rot(le_b), rot(mid_b + th), rot(mid_a + th), col);
            mb.quad(rot(le_a), rot(le_b), rot(mid_b - th), rot(mid_a - th), col);
            mb.quad(rot(mid_a + th), rot(mid_b + th), rot(te_b), rot(te_a), col);
            mb.quad(rot(mid_a - th), rot(mid_b - th), rot(te_b), rot(te_a), col);
        }
    }

    // speed brakes on the aft fuselage sides
    let sb = sim.ctrl.speedbrake * 0.9;
    for side in [-1.0, 1.0] {
        let hinge = DVec3::new(-5.0, 0.0, side * 0.85);
        let tip = DVec3::new(-5.0 - 1.2 * (sb).cos(), 0.0, side * (0.85 + 1.2 * sb.sin()));
        mb.quad(hinge + DVec3::Y * 0.35, hinge - DVec3::Y * 0.35, tip - DVec3::Y * 0.35, tip + DVec3::Y * 0.35,
            Color::new(0.45, 0.45, 0.48, 1.0));
    }

    // nozzle bell
    let re = (sim.nozzle.exit_area / std::f64::consts::PI).sqrt();
    let rt = re / sim.eng_cfg.area_ratio.sqrt();
    let xe = cfg.nose_x - cfg.fus_length - 0.4;
    let xt = xe + 1.2;
    let nc = Color::new(0.35, 0.3, 0.28, 1.0);
    for k in 0..seg {
        let a0 = k as f64 / seg as f64 * std::f64::consts::TAU;
        let a1 = (k + 1) as f64 / seg as f64 * std::f64::consts::TAU;
        let p = |x: f64, r: f64, a: f64| DVec3::new(x, r * a.sin(), r * a.cos());
        mb.quad(p(xt, rt, a0), p(xt, rt, a1), p(xe, re, a1), p(xe, re, a0), nc);
    }

    // landing gear
    if sim.ctrl.gear_down > 0.02 {
        let ext = sim.ctrl.gear_down;
        for (i, g) in sim.gear.iter().enumerate() {
            let stroke = sim.gear_stroke.get(i).copied().unwrap_or(0.0).min(g.max_stroke);
            let top = DVec3::new(g.attach.x, -0.5, g.attach.z * 0.6);
            let bottom = g.attach + DVec3::Y * stroke;
            let bottom = top + (bottom - top) * ext;
            let w = 0.12;
            let c = Color::new(0.75, 0.75, 0.78, 1.0);
            mb.quad(top + DVec3::X * w, top - DVec3::X * w, bottom - DVec3::X * w, bottom + DVec3::X * w, c);
            let wr = 0.35;
            let wc = Color::new(0.05, 0.05, 0.05, 1.0);
            for zoff in [-0.12, 0.12] {
                let cen = bottom + DVec3::new(0.0, wr, zoff);
                for k in 0..10 {
                    let a0 = k as f64 / 10.0 * std::f64::consts::TAU;
                    let a1 = (k + 1) as f64 / 10.0 * std::f64::consts::TAU;
                    mb.tri(cen, cen + DVec3::new(a0.cos(), a0.sin(), 0.0) * wr, cen + DVec3::new(a1.cos(), a1.sin(), 0.0) * wr, wc);
                }
            }
        }
    }
    mb.flush();

    draw_plume(sim, view, xe, re);
    draw_rcs(sim);
}

fn plume_color(prop: usize) -> (Color, Color) {
    match prop {
        0 => (Color::new(0.75, 0.55, 1.0, 0.75), Color::new(0.9, 0.4, 0.8, 0.0)), // ammonia: pinkish-violet
        1 => (Color::new(1.0, 0.85, 0.45, 0.85), Color::new(1.0, 0.35, 0.05, 0.0)), // kerosene: sooty orange
        2 => (Color::new(0.55, 0.7, 1.0, 0.45), Color::new(0.4, 0.5, 1.0, 0.0)),   // hydrogen: faint blue
        3 => (Color::new(0.45, 0.65, 1.0, 0.75), Color::new(0.7, 0.4, 0.9, 0.0)),  // methane: blue
        4 => (Color::new(1.0, 0.7, 0.4, 0.7), Color::new(0.9, 0.5, 0.3, 0.0)),     // hypergolic
        _ => (Color::new(1.0, 0.95, 0.85, 0.95), Color::new(1.0, 0.6, 0.3, 0.0)), // solid: bright, aluminium
    }
}

fn draw_plume(sim: &Sim, view: &mut View, xe: f64, re: f64) {
    let out = &sim.engine.out;
    if out.thrust < 100.0 {
        return;
    }
    view.flicker = (view.flicker + 0.37) % 1.0;
    let pa = sim.air.pressure.max(1.0);
    let ratio = (out.pe / pa).max(0.05);
    let frac = (out.pc / sim.eng_cfg.pc_nominal).clamp(0.0, 1.2);
    let de = 2.0 * re;
    // under-expanded plumes balloon as ambient pressure falls; over-expanded neck down
    let spread = (0.3 * ratio.ln()).clamp(-0.25, 3.5);
    let len = de * (5.0 + 10.0 * frac) * (1.0 + 0.4 * spread.max(0.0)) * (0.95 + 0.1 * view.flicker as f64);
    let (core, tail) = plume_color(sim.eng_cfg.prop);
    let mut mb = MeshBuilder::new(sim.att, sim.mp.cg);
    let seg = 16;
    let rings = 10;
    let radius = |t: f64| {
        let r_end = re * (1.0 + spread * 1.5).max(0.5) + 0.25 * de * t;
        re + (r_end - re) * (t * 2.5).min(1.0) * (1.0 - 0.3 * t)
    };
    let (rot, org) = (sim.att, sim.mp.cg);
    let body = |x: f64, r: f64, a: f64| f(rot * (DVec3::new(x, r * a.sin(), r * a.cos()) - org));
    for i in 0..rings {
        let t0 = i as f64 / rings as f64;
        let t1 = (i + 1) as f64 / rings as f64;
        let (x0, x1) = (xe - len * t0, xe - len * t1);
        let (r0, r1) = (radius(t0), radius(t1));
        let c0 = mix(core, tail, t0 as f32);
        let c1 = mix(core, tail, t1 as f32);
        for k in 0..seg {
            let a0 = k as f64 / seg as f64 * std::f64::consts::TAU;
            let a1 = (k + 1) as f64 / seg as f64 * std::f64::consts::TAU;
            let (p00, p01, p10, p11) = (body(x0, r0, a0), body(x0, r0, a1), body(x1, r1, a0), body(x1, r1, a1));
            mb.tri_raw(p00, p01, p11, c0, c0, c1);
            mb.tri_raw(p00, p11, p10, c0, c1, c1);
        }
    }
    // shock diamonds: cell spacing ≈ 1.22·De·√(M²−1) (Prandtl/Pack), visible near ideal expansion
    if ratio > 0.25 && ratio < 4.0 && sim.air.density > 0.05 {
        let spacing = 1.22 * de * (out.exit_mach * out.exit_mach - 1.0).max(0.1).sqrt();
        for k in 1..6 {
            let x = xe - spacing * k as f64;
            if xe - x > len * 0.8 {
                break;
            }
            let fade = 1.0 - k as f32 / 6.0;
            let c = Color::new(1.0, 0.95, 0.85, 0.8 * fade);
            let r = re * 0.45;
            for j in 0..seg {
                let a0 = j as f64 / seg as f64 * std::f64::consts::TAU;
                let a1 = (j + 1) as f64 / seg as f64 * std::f64::consts::TAU;
                let ctr = body(x, 0.0, 0.0);
                mb.tri_raw(ctr, body(x + r * 0.6, r, a0), body(x + r * 0.6, r, a1), c, c, c);
                mb.tri_raw(ctr, body(x - r * 0.6, r, a0), body(x - r * 0.6, r, a1), c, c, c);
            }
        }
    }
    mb.flush();
}

fn mix(a: Color, b: Color, t: f32) -> Color {
    Color::new(a.r + (b.r - a.r) * t, a.g + (b.g - a.g) * t, a.b + (b.b - a.b) * t, a.a + (b.a - a.a) * t)
}

fn draw_rcs(sim: &Sim) {
    let [p, y, r] = sim.rcs_firing;
    let puff = Color::new(0.95, 0.95, 1.0, 0.8);
    let jet = |at: DVec3, dir: DVec3| {
        let a = f(sim.att * (at - sim.mp.cg));
        let b = f(sim.att * (at + dir * 1.6 - sim.mp.cg));
        draw_line_3d(a, b, puff);
        draw_sphere(b, 0.18, None, Color::new(1.0, 1.0, 1.0, 0.35));
    };
    let nose = DVec3::new(sim.cfg.nose_x - 1.2, 0.0, 0.0);
    // the exhaust leaves opposite to the force it produces
    if p != 0 {
        jet(nose, -DVec3::Y * p as f64);
    }
    if y != 0 {
        jet(nose, -DVec3::Z * y as f64);
    }
    if r != 0 {
        let tip = sim.cfg.wing.span() * 0.45;
        jet(DVec3::new(-0.5, 0.0, tip), DVec3::Y * r as f64);
        jet(DVec3::new(-0.5, 0.0, -tip), -DVec3::Y * r as f64);
    }
}

fn draw_forces(sim: &Sim) {
    let w = sim.mp.mass * crate::atmosphere::G0;
    let scale = 3.0 / w * 20.0; // a strip carrying 1/20 of the weight draws 3 m
    let to = |p: DVec3| f(sim.att * (p - sim.mp.cg));
    for s in &sim.aero.strips {
        let p = to(s.pos);
        let l = f(sim.att * (s.lift * scale));
        let d = f(sim.att * (s.drag * scale * 3.0));
        let lc = if s.stalled > 0.5 { Color::new(1.0, 0.8, 0.0, 1.0) } else { Color::new(0.2, 1.0, 0.3, 1.0) };
        draw_line_3d(p, p + l, lc);
        draw_line_3d(p, p + d, Color::new(1.0, 0.25, 0.2, 1.0));
    }
    // thrust (orange), drawn at 1/4 scale so it fits
    let pivot = to(sim.cfg.nozzle_pivot);
    draw_line_3d(pivot, pivot + f(sim.att * (sim.thrust_body * scale * 0.25)), Color::new(1.0, 0.6, 0.1, 1.0));
    // weight (magenta) and air-relative velocity direction (cyan)
    draw_line_3d(Vec3::ZERO, vec3(0.0, -6.0, 0.0), Color::new(1.0, 0.3, 1.0, 1.0));
    let v = sim.vel - sim.wind_total;
    if v.length() > 1.0 {
        draw_line_3d(Vec3::ZERO, f(v.normalize() * 15.0), Color::new(0.3, 0.9, 1.0, 1.0));
    }
    draw_sphere(Vec3::ZERO, 0.25, None, YELLOW);
}

fn draw_np_marker(sim: &Sim, x_np: f64) {
    let p = f(sim.att * (DVec3::new(x_np, 0.0, 0.0) - sim.mp.cg));
    draw_sphere(p, 0.22, None, Color::new(0.3, 0.5, 1.0, 1.0));
}
