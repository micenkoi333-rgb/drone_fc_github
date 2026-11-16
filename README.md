# drone_fc — tilt-rotor VTOL flight physics simulator

A 6-DOF flight physics simulator for small electric **tilt-rotor VTOL aircraft**: four propellers on tilting nacelles that lift the aircraft
like a quadcopter, then rotate forward so the same propellers drive it as an airplane. The goal is to evaluate a design *before* it is built —
thrust margins, energy use, stability, hover↔cruise transitions, wind tolerance and failure cases — and to develop the flight controller
against a model that behaves like real hardware (noisy sensors, delays, servo backlash, battery sag).

Written in **C++17** (header-only core, no dependencies). The 3D GUI uses Dear ImGui + ImPlot + GLFW (OpenGL). Python is used only for
offline table generation and neural-network training.

Two aircraft are included:

| | `aircraft/qp5.ini` — QP-5 TiltQuad v3 (default) | `aircraft/tiltrotor.ini` — earlier concept |
|---|---|---|
| Mass | 2.37 kg | 2.28 kg |
| Wing | 1.0 m span, tapered with twist, SD7037 | 1.6 × 0.28 m rectangular, Clark Y |
| Propulsion | 2 × 2216 + 10×7″ (front, tractor), 2 × 2212 + 9×7″ (rear, pusher) | 4 × 10×4.5″ |
| Tail | horizontal stabilizer + two fixed fins, yaw by differential thrust | V-tail with ruddervators |
| Battery | 4S3P Li-ion 21700 | 6S2P Li-ion 21700 |

Any aircraft is described by a single INI file (geometry, mass items, motors, propellers, battery, controller gains); the simulator derives
mass properties, inertia, aerodynamic tables and trim from it.

## Physics model

**Rigid body.** 6-DOF equations of motion integrated with RK4 at 1 kHz; mass, centre of gravity and inertia tensor computed from individual
mass items; landing gear contact model.

**Aerodynamics.**
- Wing split into spanwise strips (blade-element / strip theory) with full-range airfoil tables (−180…180°, Selig-style post-stall).
- Nonlinear lifting-line solution for induced angles (handles taper and twist), downwash at the tail via Biot–Savart.
- Dynamic stall per strip, wing ground effect, fuselage drag, control surfaces.
- Propeller slipstream over the wing and tail; rotor-to-rotor wake interaction.

**Propellers and nacelles.**
- Thrust and torque from UIUC-style CT/CP(J) tables, H-force in oblique inflow, reaction and gyroscopic torques along each nacelle axis.
- Rotor ground effect and vortex-ring state when the disk faces down; drag of stopped propellers; nacelle blockage for pushers.
- Tilt servos with rate limits, travel limits and backlash; frame flexibility.

**Power.** Brushless motor + ESC model (with active braking), Thevenin battery model with temperature, implicit bus-voltage solution —
current, voltage sag and remaining energy come out of the same model.

**Environment.** ISA atmosphere, wind shear with terrain roughness, Dryden turbulence with gust gradients, thermals.

**Sensors.** Either "truth + noise and delays", or raw sensors: two IMUs with vibration and failures, GPS with latency and velocity bias,
barometer, magnetometer, airspeed, nacelle position feedback — feeding a 15-state EKF.

## Flight controller

ArduPilot-QuadPlane-style architecture: attitude/rate loops, position control, TECS and L1 for airplane flight, blended transitions,
Q_ASSIST and weathervaning. Tilt-rotor specific: forces are distributed by tilting the nacelles instead of pitching the body, yaw in hover by
differential nacelle tilt, braking by tilting nacelles backwards. Failsafes: lost GPS / airspeed / link, low battery, motor failure, jammed
nacelle actuator. Five manual flight modes (keyboard or gamepad).

An optional on-board "AI co-processor" (`ai/`) runs small neural networks next to the controller: an external-force model, a learned
station-keeping policy, an aerobatics trainer, and health diagnostics (prop damage, actuator backlash, CG shift, battery).

## Validation

- **~480 automated tests** (`tests/`): physics invariants, aerodynamic tables, trim, flight modes, failures, wind, Monte-Carlo runs.
- QP-5 model vs. the airframe's design documentation:

| | Design doc | Simulator |
|---|---|---|
| Mass | 2.370 kg | 2.370 kg |
| Ixx / Iyy / Izz, kg·m² | 0.039 / 0.207 / 0.244 | 0.040 / 0.198 / 0.237 |
| Stall speed | 11.7 m/s | 11.6 m/s |
| Max thrust front / rear | 1422 / 1016 g | 1400 / 1014 g |
| Drag at 14.6 / 18 / 22 m/s | 2.66 / 2.58 / 2.94 N | 2.58 / 2.61 / 3.08 N |
| L/D at 14.6 m/s | 8.8 | 9.0 |
| Hover power | 352 W | 444 W |
| Cruise power at 14.6 m/s | 78 W | 99 W |

Aerodynamics agrees within 5%. Power differs by ~25% because the propeller data are generic; it will be recalibrated from thrust-stand
measurements (see `docs/QP5.md`).

- A 1 km QP-5 mission (take-off, transition, cruise, hover, return, landing) completes in calm air and in 3–12 m/s wind from all four
  directions, landing within 5 m of home.

## Build and run (Windows, Visual Studio)

```bat
build.bat            :: console simulator, tests and GUI (drone_gui.exe)
build_gui.bat        :: GUI only
run_tests.bat        :: all test groups in parallel on all CPU cores
drone_sim.exe aircraft          :: mass properties, CG, inertia, checks
drone_sim.exe qp trim           :: trim table (airplane) and transition schedule
drone_sim.exe qp modes [V]      :: eigenvalues of the flight modes
drone_sim.exe qp mission        :: mission energy estimate
```

Run from the project root. The core and tests also compile with g++ (`g++ -std=c++17 -O2 -I. tests/test_main.cpp -lpthread`).
The active aircraft is set in `aircraft/active.txt` or in the GUI (Parameters → Aircraft).

Speed: a 120 s flight takes ~9 s on one core (~13× real time); batch runs (mission check in 17 wind conditions, tests, AI training)
use all cores.

## GUI (`drone_gui.exe`)

3D view with tilting nacelles and spinning disks, several cameras, PFD, map and live plots; mission and environment settings,
fault injection, manual flight, parameter editor, flight reports (TXT, CSV, interactive HTML), "check mission in different winds" on all cores.

## Known limitations

- Propeller and motor data are generic (UIUC-style), not measured — the biggest source of error in power and endurance.
- SD7037 airfoil table is an approximation; landing gear, frame stiffness and servo speed are estimates.
- Oblique-inflow propeller model is axial + H-force, no blade-resolved aerodynamics.

## Repository layout

```
aircraft/   aircraft definitions (INI) and airfoil tables (JSON)
sim/        physics: aerodynamics, propellers, battery, sensors, environment, rigid body
fc/         flight controller: modes, control loops, EKF, failsafes, logging
ai/         on-board neural networks and diagnostics
gui/        ImGui / OpenGL front end
tools/      airfoil table generator, AI training, ArduPilot SITL bridge
tests/      automated tests
docs/       design notes (in Ukrainian)
third_party/ GLFW, Dear ImGui, ImPlot
```

Documentation in `docs/` and the original README (`README_uk.md`) are in Ukrainian.
