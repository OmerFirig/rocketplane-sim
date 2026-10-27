//! 2D instrument overlay: data panels, attitude indicator, strip charts.

use macroquad::prelude::*;
use std::collections::VecDeque;

pub const BG: Color = Color::new(0.0, 0.0, 0.0, 0.58);
pub const FG: Color = Color::new(0.85, 0.95, 0.88, 1.0);
pub const DIM: Color = Color::new(0.55, 0.65, 0.6, 1.0);
pub const HEAD: Color = Color::new(0.45, 0.85, 1.0, 1.0);
pub const WARN: Color = Color::new(1.0, 0.75, 0.15, 1.0);
pub const ALERT: Color = Color::new(1.0, 0.3, 0.25, 1.0);
pub const GOOD: Color = Color::new(0.4, 1.0, 0.5, 1.0);

pub struct Ui {
    pub font: Option<Font>,
    pub size: u16,
}

impl Ui {
    pub fn text(&self, s: &str, x: f32, y: f32, c: Color) {
        self.text_sized(s, x, y, self.size, c);
    }
    pub fn text_sized(&self, s: &str, x: f32, y: f32, size: u16, c: Color) {
        draw_text_ex(s, x, y, TextParams { font: self.font.as_ref(), font_size: size, color: c, ..Default::default() });
    }
    pub fn width(&self, s: &str, size: u16) -> f32 {
        measure_text(s, self.font.as_ref(), size, 1.0).width
    }
    pub fn lh(&self) -> f32 {
        self.size as f32 * 1.22
    }
}

/// A row is either a section header, a key/value pair or a free line.
pub enum Row {
    Head(String),
    Kv(String, String, Color),
    Line(String, Color),
    Bar(String, f32, Color), // label, fill 0..1
}

pub fn kv(k: &str, v: String) -> Row {
    Row::Kv(k.to_string(), v, FG)
}
pub fn kvc(k: &str, v: String, c: Color) -> Row {
    Row::Kv(k.to_string(), v, c)
}

/// Draw a panel and return its height.
pub fn panel(ui: &Ui, x: f32, y: f32, w: f32, rows: &[Row]) -> f32 {
    let lh = ui.lh();
    let h = rows.len() as f32 * lh + 10.0;
    draw_rectangle(x, y, w, h, BG);
    let mut cy = y + lh;
    for r in rows {
        match r {
            Row::Head(s) => ui.text(s, x + 8.0, cy, HEAD),
            Row::Kv(k, v, c) => {
                ui.text(k, x + 8.0, cy, DIM);
                let vw = ui.width(v, ui.size);
                ui.text(v, x + w - 8.0 - vw, cy, *c);
            }
            Row::Line(s, c) => ui.text(s, x + 8.0, cy, *c),
            Row::Bar(label, frac, c) => {
                ui.text(label, x + 8.0, cy, DIM);
                let bx = x + w * 0.45;
                let bw = w * 0.55 - 10.0;
                draw_rectangle_lines(bx, cy - lh * 0.7, bw, lh * 0.7, 1.0, DIM);
                draw_rectangle(bx + 1.0, cy - lh * 0.7 + 1.0, (bw - 2.0) * frac.clamp(0.0, 1.0), lh * 0.7 - 2.0, *c);
            }
        }
        cy += lh;
    }
    h
}

/// Attitude director indicator with pitch ladder, roll pointer and a
/// velocity-vector marker placed by α and β.
#[allow(clippy::too_many_arguments)]
pub fn adi(ui: &Ui, cx: f32, cy: f32, r: f32, pitch: f32, roll: f32, alpha: f32, beta: f32, heading: f32) {
    let ppd = r / 35.0; // pixels per degree of pitch
    let c0 = vec2(cx, cy);
    draw_circle(cx, cy, r, Color::new(0.18, 0.45, 0.85, 1.0));

    // world "up" on screen (y grows downward); banking right turns the
    // horizon counter-clockwise
    let (s, c) = roll.sin_cos();
    let up = vec2(-s, -c);
    let along = vec2(c, -s);
    let pd = pitch.to_degrees();
    let center = c0 - up * (pd * ppd);
    let below = |p: Vec2| (p - center).dot(up) < 0.0;

    // ground = circle ∩ half-plane (convex) → triangle fan
    let segs = 72;
    let pt = |i: usize| {
        let a = i as f32 / segs as f32 * std::f32::consts::TAU;
        vec2(cx + r * a.cos(), cy + r * a.sin())
    };
    let mut poly: Vec<Vec2> = Vec::new();
    for i in 0..segs {
        let (p0, p1) = (pt(i), pt(i + 1));
        if below(p0) {
            poly.push(p0);
        }
        if below(p0) != below(p1) {
            let d0 = (p0 - center).dot(up);
            let d1 = (p1 - center).dot(up);
            poly.push(p0 + (p1 - p0) * (d0 / (d0 - d1)));
        }
    }
    let gnd = Color::new(0.55, 0.36, 0.18, 1.0);
    for i in 1..poly.len().saturating_sub(1) {
        draw_triangle(poly[0], poly[i], poly[i + 1], gnd);
    }

    for k in (-90..=90).step_by(10) {
        let off = (pd - k as f32) * ppd;
        if off.abs() > r * 0.8 {
            continue;
        }
        let mid = c0 - up * off;
        let half = if k == 0 { r * 0.97 } else { r * 0.28 };
        let (a, b) = (mid - along * half, mid + along * half);
        draw_line(a.x, a.y, b.x, b.y, if k == 0 { 2.0 } else { 1.2 }, WHITE);
        if k != 0 {
            ui.text_sized(&format!("{}", k), b.x + 3.0, b.y + 4.0, 12, WHITE);
        }
    }

    // velocity vector: below the nose by α, beside it by β (body frame)
    let fpm = c0 + vec2(beta.to_degrees() * ppd, alpha.to_degrees() * ppd);
    if (fpm - c0).length() < r * 0.9 {
        draw_circle_lines(fpm.x, fpm.y, 6.0, 2.0, GOOD);
        draw_line(fpm.x - 14.0, fpm.y, fpm.x - 6.0, fpm.y, 2.0, GOOD);
        draw_line(fpm.x + 6.0, fpm.y, fpm.x + 14.0, fpm.y, 2.0, GOOD);
        draw_line(fpm.x, fpm.y - 6.0, fpm.x, fpm.y - 12.0, 2.0, GOOD);
    }

    let y = Color::new(1.0, 0.85, 0.1, 1.0);
    draw_line(cx - r * 0.45, cy, cx - r * 0.12, cy, 3.0, y);
    draw_line(cx + r * 0.12, cy, cx + r * 0.45, cy, 3.0, y);
    draw_line(cx - r * 0.12, cy, cx, cy + r * 0.08, 3.0, y);
    draw_line(cx + r * 0.12, cy, cx, cy + r * 0.08, 3.0, y);

    let tip = c0 + up * r;
    draw_triangle(tip, tip - up * 12.0 + along * 7.0, tip - up * 12.0 - along * 7.0, WHITE);
    for d in [-60.0f32, -45.0, -30.0, -20.0, -10.0, 0.0, 10.0, 20.0, 30.0, 45.0, 60.0] {
        let a = d.to_radians();
        let p0 = vec2(cx + a.sin() * r, cy - a.cos() * r);
        let p1 = vec2(cx + a.sin() * (r + 8.0), cy - a.cos() * (r + 8.0));
        draw_line(p0.x, p0.y, p1.x, p1.y, 1.5, WHITE);
    }
    draw_circle_lines(cx, cy, r, 2.0, Color::new(0.8, 0.8, 0.8, 1.0));
    let hs = format!("HDG {:03.0}°", heading.to_degrees());
    ui.text(&hs, cx - ui.width(&hs, ui.size) * 0.5, cy + r + 22.0, FG);
}

pub struct History {
    pub t: VecDeque<f64>,
    pub series: Vec<(&'static str, VecDeque<f64>, Color)>,
    pub last: f64,
    pub window: f64,
}

impl History {
    pub fn new() -> History {
        History {
            t: VecDeque::new(),
            series: vec![
                ("altitude km", VecDeque::new(), Color::new(0.4, 0.8, 1.0, 1.0)),
                ("Mach", VecDeque::new(), Color::new(1.0, 0.8, 0.3, 1.0)),
                ("q kPa", VecDeque::new(), Color::new(1.0, 0.45, 0.4, 1.0)),
                ("nz g", VecDeque::new(), Color::new(0.5, 1.0, 0.5, 1.0)),
                ("skin K", VecDeque::new(), Color::new(1.0, 0.5, 0.9, 1.0)),
            ],
            last: -1.0,
            window: 120.0,
        }
    }
    pub fn clear(&mut self) {
        self.t.clear();
        for s in &mut self.series {
            s.1.clear();
        }
        self.last = -1.0;
    }
    pub fn push(&mut self, t: f64, vals: [f64; 5]) {
        if t - self.last < 0.1 {
            return;
        }
        self.last = t;
        self.t.push_back(t);
        for (s, v) in self.series.iter_mut().zip(vals) {
            s.1.push_back(v);
        }
        while let Some(&t0) = self.t.front() {
            if t - t0 > self.window {
                self.t.pop_front();
                for s in &mut self.series {
                    s.1.pop_front();
                }
            } else {
                break;
            }
        }
    }
    pub fn draw(&self, ui: &Ui, x: f32, y: f32, w: f32, h_each: f32) {
        let n = self.series.len() as f32;
        draw_rectangle(x, y, w, h_each * n + 6.0, BG);
        if self.t.len() < 2 {
            return;
        }
        let t0 = *self.t.front().unwrap();
        let t1 = *self.t.back().unwrap();
        let span = (t1 - t0).max(1.0);
        for (i, (name, data, col)) in self.series.iter().enumerate() {
            let top = y + 4.0 + i as f32 * h_each;
            let (mut lo, mut hi) = (f64::MAX, f64::MIN);
            for &v in data {
                lo = lo.min(v);
                hi = hi.max(v);
            }
            if (hi - lo).abs() < 1e-6 {
                hi = lo + 1.0;
            }
            let ph = h_each - 6.0;
            draw_line(x + 4.0, top + ph, x + w - 4.0, top + ph, 1.0, Color::new(1.0, 1.0, 1.0, 0.15));
            let mut prev: Option<Vec2> = None;
            for (t, v) in self.t.iter().zip(data) {
                let px = x + 4.0 + ((t - t0) / span) as f32 * (w - 8.0);
                let py = top + ph - ((v - lo) / (hi - lo)) as f32 * ph;
                if let Some(p) = prev {
                    draw_line(p.x, p.y, px, py, 1.5, *col);
                }
                prev = Some(vec2(px, py));
            }
            let last = data.back().copied().unwrap_or(0.0);
            ui.text_sized(&format!("{} {:.2}  [{:.1}…{:.1}]", name, last, lo, hi), x + 6.0, top + 12.0, 12, *col);
        }
    }
}
