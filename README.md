# Rocketplane — rocket plane physics sandbox (Rust)

A 6-degree-of-freedom simulation of an X-15-style rocket-powered aircraft. No
precomputed derivative lookup tables: every force is computed each step from
fluid mechanics and rocket thermodynamics equations.

```sh
cargo run --release                 # sim
cargo run --release -- --selftest   # physics validation output (no window)
cargo run --release -- --scenario 2 --ignite --forces --cam 1 --nohelp
```

Built with [macroquad](https://github.com/not-fl3/macroquad) 0.4.

### Command-line flags

| Flag | Effect |
|---|---|
| `--scenario N` | 1 runway, 2 B-52 drop, 3 hypersonic, 4 space shot, 5 unpowered approach |
| `--ignite` | Start with the engine lit at full throttle |
| `--forces` | Start with force vectors shown |
| `--cam N` | Advance the camera mode N times (0 chase, 1 orbit, 2 cockpit, 3 tower) |
| `--nohelp` | Hide the help overlay |
| `--selftest` | Run the headless physics validation and exit |

## Physics models

| Area | Model |
|---|---|
| Atmosphere | U.S. Standard Atmosphere 1976, 0–86 km, 8 layers, geopotential altitude; ideal-gas density, `a = √(γRT)`, Sutherland viscosity; ISA+ΔT days |
| Gravity | `g = g₀ (R/(R+h))²` |
| Rigid body | 6-DOF, quaternion attitude, RK4 (1 kHz), Euler equations `Iω̇ = M − ω×Iω`, full inertia tensor |
| Variable mass | Fuel/oxidizer tanks as separate point masses + liquid-cylinder inertia; CG and inertia shift during the burn |
| Lift | Strip theory: every wing/tail segment sees its own local airflow (`v + ω×r − wind`). Subsonic Helmbold/DATCOM + Prandtl-Glauert, supersonic Ackeret `4/√(M²−1)` + tip loss, transonic bridge |
| Aerodynamic center | ¼ chord subsonic, ½ chord supersonic — supersonic "tuck" and static-margin growth emerge on their own |
| Stall | Sigmoid blend to flat-plate Newtonian normal force; thin wings stall later; asymmetric stall is possible |
| Skin friction | Blasius laminar / Schlichting turbulent Cf, transition at Re 5·10⁵, compressibility correction, Raymer form factor |
| Wave drag | Korn critical Mach + Lock 4th-power law (transonic, capped at the transonic peak), Ackeret thickness drag (supersonic), Sears-Haack fuselage |
| Induced drag | `CL²/(πeA)`, Raymer Oswald efficiency, Wieselsberger ground effect; `β·CL²/4` supersonic |
| Fuselage | Slender-body potential normal force + Allen-Perkins viscous crossflow; blunt base drag, filled by the exhaust plume while the engine burns |
| Wing-tail interaction | Downwash reaches the tail with an `l_t/V` lag (Cmα̇ damping emerges naturally) |
| Control surfaces | Thin-airfoil flap theory: `τ = 1 − (θ − sin θ)/π`, flap Cm; X-15-style differential horizontal stabilizer (roll) |
| Rocket engine | Ideal rocket: c*, choked mass flow at the throat, exit Mach from the area-Mach relation, `F = λṁvₑ + (pₑ − pₐ)Aₑ`, combustion efficiency, conical nozzle divergence loss, chamber-pressure lag |
| Nozzle flow separation | Summerfield criterion (`pₑ < 0.4 pₐ`): the effective area ratio shrinks in an over-expanded nozzle |
| Propellants | LOX/Ammonia (XLR99), LOX/RP-1, LOX/LH2, LOX/Methane, N2O4/UDMH, APCP solid (no throttling, no shutdown). Tank volume is fixed: choosing LH2 puts far less mass in the same tank |
| RCS | 90% H2O2 monopropellant thrusters, thrust depends on ambient pressure |
| Heating | Stagnation temperature, Sutton-Graves convective heat flux, hot-wall correction, Inconel-X skin heat capacity and radiative cooling |
| Wind | 1/7 power-law boundary-layer profile + Dryden turbulence (MIL-F-8785C scale lengths) |
| Ground | Spring-damper landing gear, rolling/brake/side friction, nose-wheel steering, gear collapse and hull-strike detection |
| Limits | Over-g, dynamic pressure, skin temperature → structural/thermal damage |

## Validation (`--selftest`)

- Atmospheric density matches reference tables to four significant digits
  (0, 20, 30, 50, 80 km).
- XLR99 engine: 262 kN vacuum thrust, Isp 291 s (real XLR99: 262 kN, 276 s).
- Static margin: about 10 % MAC subsonic, about 37 % supersonic, dropping again
  at hypersonic speeds (the same trend known from the X-15).
- B-52 drop + full-throttle climb: passes Mach 6, reaches about 60 km.
- Space shot: apogee 98.5 km, 4.3 g on re-entry, skin 892 K.
- The solid motor cannot be shut down (correct behaviour).

## Controls

| Key | Action |
|---|---|
| W/S, A/D, Q/E | Pitch, roll, yaw |
| ↑/↓ | Pitch trim · **K** auto level trim |
| Space | Ignite / shut down engine |
| Shift / Ctrl, Z | Throttle up / down, full throttle |
| F, G, X, B | Flaps 0/20/40, landing gear, airbrake, wheel brakes |
| R, T | RCS, SAS (stability augmentation) |
| V | Force vectors (lift green/stalled yellow, drag red, thrust orange, NP blue) |
| C | Camera: chase / orbit / cockpit / tower |
| Left click + drag | Rotate camera (chase, orbit, look around in cockpit) · wheel: zoom · middle click: recenter |
| P, [ ] | Pause, time acceleration |
| L | CSV telemetry log |
| 1–5, Backspace | Scenarios: runway, B-52 drop, hypersonic, space shot, unpowered approach; restart |
| Tab | Sandbox panel: propellant, chamber pressure, nozzle area ratio, throat diameter, wing area/aspect ratio/sweep/thickness, mass, CG, wind, turbulence, ISA ΔT |

## Source layout

| File | Contents |
|---|---|
| `src/atmosphere.rs` | ISA 1976, Sutherland viscosity, gravity vs. altitude, pitot/CAS (subsonic isentropic, supersonic Rayleigh pitot) |
| `src/rocket.rs` | Ideal rocket engine, propellant table, nozzle separation, H2O2 RCS thruster |
| `src/aircraft.rs` | Geometry (wing, horizontal and vertical tail), strip discretisation, flap theory, landing gear, contact points, mass/CG/inertia |
| `src/aero.rs` | Aerodynamic model |
| `src/sim.rs` | 6-DOF integration, variable mass, gear, RCS, SAS, turbulence, heating, limits, trim solver, static margin, scenarios |
| `src/render.rs` | 3D rendering with camera-relative positioning (no f64 → f32 precision loss), moving control surfaces, heated skin glow, propellant-dependent exhaust colour, shock diamonds |
| `src/hud.rs` | Info panels, attitude indicator, 120 s telemetry graphs |
| `src/main.rs` | App loop, input, sandbox parameters, warnings, CSV logging, layout |
| `src/selftest.rs` | Headless physics validation |

## Known simplifications

Flat (non-rotating) Earth; frozen-flow ideal gas with constant γ (not a real CEA
equilibrium calculation); linear theory instead of Newtonian in the hypersonic
regime; no wing-body interference and no shock position on the wing; single-point
skin temperature.

Shock waves are not solved as a flow field (no CFD). Their effects — wave drag,
aerodynamic-center shift, pitot shock — come from closed-form results of shock
theory.

## Roadmap

1. Shock-expansion theory (oblique shock + Prandtl-Meyer, Rankine-Hugoniot,
   attached/detached shock) and Newtonian flow at hypersonic speeds.
2. Live 2D Euler flow solver around the wing section, to watch shocks form.
3. Unsteady aerodynamics (Wagner/Theodorsen) and dynamic stall.
4. Visual Mach cone / condensation cloud.

## License

[MIT](LICENSE)
