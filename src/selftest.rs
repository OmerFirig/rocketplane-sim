//! `cargo run --release -- --selftest`: headless checks of the physics models
//! against reference values, plus a few open-loop flights.

use crate::aero;
use crate::aircraft::AirframeConfig;
use crate::atmosphere::{self, isa};
use crate::rocket::{EngineConfig, PROPELLANTS};
use crate::sim::{spawn, Environment, Scenario, Sim};

pub fn run() {
    println!("== ISA 1976 (reference: ρ 1.2250 / 0.36392 / 0.08803 / 0.018410 / 0.0010269 / 0.0000157 kg/m³)");
    for z in [0.0, 11_000.0, 20_000.0, 30_000.0, 50_000.0, 80_000.0] {
        let a = isa(z, 0.0);
        println!(
            "  z={:>6.0} m  T={:7.2} K  p={:10.2} Pa  ρ={:.7} kg/m³  a={:6.1} m/s  μ={:.3e}",
            z, a.temperature, a.pressure, a.density, a.speed_of_sound, a.viscosity
        );
    }

    println!("\n== Engines (Pc nominal, ε as configured)");
    for i in 0..PROPELLANTS.len() {
        let e = EngineConfig { prop: i, ..EngineConfig::default() };
        let d = e.design();
        let sl = e.performance(&d, e.pc_nominal, atmosphere::P0);
        let vac = e.performance(&d, e.pc_nominal, 0.0);
        println!(
            "  {:<26} c*={:6.0}  Me={:4.2}  pe={:6.0} Pa  F_sl={:6.1} kN Isp_sl={:5.1}s{}  F_vac={:6.1} kN Isp_vac={:5.1}s  ṁ={:5.1} kg/s",
            PROPELLANTS[i].name, d.cstar, d.exit_mach, vac.pe, sl.thrust / 1e3, sl.isp,
            if sl.separated { "(sep)" } else { "     " }, vac.thrust / 1e3, vac.isp, vac.mdot
        );
    }

    println!("\n== Wing lift-curve slope vs Mach (A=2.5) and section wave drag (t/c 5 %)");
    let w = AirframeConfig::default().wing;
    for m in [0.2, 0.6, 0.8, 0.9, 1.0, 1.1, 1.3, 2.0, 3.0, 5.0] {
        println!(
            "  M={:3.1}  CLα={:5.3}/rad  CDwave={:.5}  Cf(Re=3e7)={:.5}",
            m, aero::lift_slope(w.aspect, w.sweep_at(0.5), m),
            aero::section_wave_drag(m * w.sweep_le_deg.to_radians().cos(), w.tc, 0.0, w.wave_k),
            aero::skin_friction(3e7, m)
        );
    }

    let env = Environment::default();
    let sim = Sim::new(AirframeConfig::default(), EngineConfig::default(), env);
    println!("\n== Static margin, full tanks (CG x {:.2} m, MAC {:.2} m)", sim.mp.cg.x, sim.cfg.wing.mac());
    for m in [0.3, 0.8, 0.95, 1.2, 2.0, 3.0, 5.0, 6.5] {
        let (sm, xnp) = sim.static_margin(15_000.0, m);
        println!("  M={:3.1}  SM={:6.1} % MAC  x_np={:6.2} m", m, sm * 100.0, xnp);
    }
    for sc in [Scenario::AirLaunch, Scenario::Approach, Scenario::Hypersonic] {
        let mut sim = Sim::new(AirframeConfig::default(), EngineConfig::default(), env);
        spawn(&mut sim, sc);
        println!(
            "\n== {}  mass {:.0} kg  CG x {:.2} m  trim stab {:.2}°",
            sc.name(), sim.mp.mass, sim.mp.cg.x, sim.input.trim.to_degrees()
        );
        fly(&mut sim, 30.0, |_s, _t| {});
    }

    let mut sim = Sim::new(AirframeConfig::default(), EngineConfig::default(), env);
    spawn(&mut sim, Scenario::AirLaunch);
    println!("\n== Air launch, full throttle, pull to 2 g then hold 25° climb");
    sim.toggle_engine();
    sim.input.throttle = 1.0;
    fly(&mut sim, 120.0, |s, _t| {
        let (pitch, _, _) = s.euler();
        s.input.pitch = ((25f64.to_radians() - pitch) * 3.0 - s.omega.z * 1.5).clamp(-1.0, 1.0);
        s.input.roll = (-s.euler().1 * 2.0).clamp(-1.0, 1.0);
    });

    let mut sim = Sim::new(AirframeConfig::default(), EngineConfig::default(), env);
    spawn(&mut sim, Scenario::HighAltitude);
    println!("\n== Space shot: ballistic arc with RCS + SAS, then hold 15° α for re-entry");
    fly(&mut sim, 240.0, |s, _t| {
        if s.vel.y < 0.0 && s.pos.y < 70_000.0 {
            s.input.pitch = ((15f64.to_radians() - s.aero.alpha) * 4.0 - s.omega.z * 2.0).clamp(-1.0, 1.0);
        }
    });

    let solid = EngineConfig { prop: 5, ..EngineConfig::default() };
    let mut sim = Sim::new(AirframeConfig::default(), solid, env);
    spawn(&mut sim, Scenario::AirLaunch);
    println!("\n== Solid motor air launch: grain {:.0} kg, cannot be shut down", sim.fuel);
    sim.toggle_engine();
    sim.toggle_engine();
    println!("  after shutdown attempt: running = {}", sim.engine.running);

    let mut sim = Sim::new(AirframeConfig::default(), EngineConfig::default(), env);
    spawn(&mut sim, Scenario::Runway);
    println!("\n== Runway takeoff");
    sim.toggle_engine();
    sim.input.throttle = 0.6;
    fly(&mut sim, 20.0, |s, _t| {
        s.input.pitch = if s.aero.tas > 125.0 { ((12f64.to_radians() - s.euler().0) * 4.0).clamp(-1.0, 1.0) } else { 0.0 };
    });
}

fn fly(sim: &mut Sim, secs: f64, mut ctl: impl FnMut(&mut Sim, f64)) {
    let dt = 1.0 / 1000.0;
    let steps = (secs / dt) as usize;
    for i in 0..=steps {
        ctl(sim, i as f64 * dt);
        if i % (steps / 10).max(1) == 0 {
            let (p, r, h) = sim.euler();
            let a = &sim.aero;
            println!(
                "  t={:6.1} alt={:8.0} M={:5.2} TAS={:6.0} α={:6.2}° θ={:6.1}° φ={:6.1}° ψ={:5.0}° nz={:5.2} q={:6.1}kPa F={:5.0}kN m={:6.0} L/D={:5.2} Tskin={:5.0}K {}",
                sim.time, sim.pos.y, a.mach, a.tas, a.alpha.to_degrees(), p.to_degrees(), r.to_degrees(),
                h.to_degrees(), sim.nz, a.qbar / 1e3, sim.engine.out.thrust / 1e3, sim.mp.mass,
                a.lift / a.drag.max(1e-6), sim.skin_temp,
                if let crate::sim::Status::Crashed(w) = &sim.status { w.as_str() } else { "" }
            );
        }
        sim.step(dt);
    }
}
