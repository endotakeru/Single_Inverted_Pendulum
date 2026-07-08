# Single Inverted Pendulum — Simulation + Hardware Control

An educational control-systems project: a cart on a rail balances a rod with a
mass on the end (an inverted pendulum) using PD feedback. It spans the full
stack — a **Python simulation** to design and understand the controller, an
**Arduino controller** for the real hardware rig, the **CAD/3D-printed**
mechanical design, and the **KiCad schematic** for the electronics.

## What the system is

```
            o   <- end bob (mass m1)
            |
            |   <- rod (mass m2, length l), angle theta from vertical
         [ cart ]  <- slides on a rail
   =========================  <- rail
            |
        cart force / motion   --->
```

* **State vector (used throughout):** `X = [x, x_dot, theta, theta_dot]`
  * `x` cart position [m], `x_dot` cart velocity [m/s]
  * `theta` pendulum angle from vertical [rad] (0 = upright), `theta_dot` [rad/s]
* **The actuator moves the CART, not the pendulum.** The pendulum is never pushed
  directly — it only reacts to the cart sliding underneath it. Balancing means
  moving the cart so it stays under the falling pendulum.

## Project structure

```
Single_Inverted_Pendulum/
├── Python_Simulation/             # the Python simulation (design + study)
│   ├── main.py                    #   runs the whole simulation start to finish
│   ├── params.py                  #   constants, simulation settings, PD gains
│   ├── dynamics.py                #   nonlinear equations of motion (the plant)
│   ├── linear_model.py            #   linearised state-space A, B, C, D matrices + open-loop poles
│   ├── pd_controller.py           #   PD control law (angle loop + position loop)
│   ├── simulation.py              #   time-domain run with scipy solve_ivp
│   ├── plot_results.py            #   plots: position, angle, velocities, force
│   └── requirements.txt           #   pinned Python dependencies
├── Arduino_Control/
│   ├── arduino_IP/arduino_IP.ino  # main closed-loop PD controller for the physical rig
│   └── arduino_Calibration/       # staged bring-up sketches, run before arduino_IP.ino
│       ├── SensorCalibration/     #   zero the P3022 angle offset
│       ├── MotorTest/             #   verify stepper direction/speed/microstepping
│       ├── SensorReader/          #   raw sensor readout for debugging
│       └── FinalHardwareTest/     #   combined sensor + motor check before closing the loop
├── CAD/                           # Fusion 360 STEP files
│   ├── Assemblies/                #   Master_Assembly, Platform_Assembly, Extended_Rot_Sensor
│   ├── Custom/                    #   designed parts (platform, bearing slider, pendulum stick, spacers, ...)
│   └── Vendor/                    #   off-the-shelf parts (bearings, motor, shaft, T-slot rail, ...)
├── STL(3d_printing)/              # printable STLs for all custom/3D-printed parts
├── IP_Circuit_Schematics/         # KiCad project (Arduino + DRV8825 + P3022 wiring)
├── Dashboard_Simulation/
│   └── index.html                 # standalone browser version of the PD simulator
├── Single_IP_Project_Report.pdf   # full written project report
├── Hand_calculation.pdf           # hand-derived Lagrangian equations of motion
└── README.md                      # this file
```

---

## Part 1 — Python simulation (`Python_Simulation/`)

Design and study the controller in software first.

```bash
cd Python_Simulation
pip install -r requirements.txt
python main.py
```

Useful options:

```bash
python main.py                 # tuned balancing gains + plots (default)
python main.py --gains spec    # the spec gains (Kp_theta = 1) -> falls over & trips
python main.py --sweep         # sweep theta0 = -15..+15 deg; all angles overlaid (theta & x panels)
```

In the simulation the controller output is a **force** on the cart. Two PD loops
are **summed into one cart force**:

```
F = (Kp_theta*theta + Kd_theta*d theta/dt)   # angle loop (upright)
  + (Kp_x*x         + Kd_x*d x/dt)            # position loop (centre)
```

The default `params.py` gains (`Kp_theta = 1`) are intentionally too weak to
balance; `Params.balanced()` / `--gains preset` (`Kp_theta = 9.17`, `Kd_theta = 0.75`)
do balance. `linear_model.py` builds the linearised state-space matrices and
reports the open-loop poles (one at +7.19 rad/s — confirming the plant is
open-loop unstable). `Dashboard_Simulation/index.html` is the same simulator
in a browser — just open the file.

## Part 2 — Arduino controller (`Arduino_Control/`)

The closed-loop controller for the real rig lives in
`arduino_IP/arduino_IP.ino`.

- Board: **Arduino Uno R3** (powered over USB).
- Motor: **NEMA 17 stepper** driven by a **DRV8825 STEP/DIR driver** on a
  **separate 12–24 V supply**; the cart is moved by a GT2 belt/pulley.
- Sensor: **P3022** analog angle sensor read on **A0**.
- It reads the angle, runs a PD law, and converts the PD output into a stepper
  **direction + step frequency** (bigger error → step faster).

> **Important difference from the simulation:** the sim's actuator is a *force*;
> the stepper is a *velocity* command. So the simulation gains do **not** transfer
> directly — they are included in the sketch only as a reference. **Tune the
> hardware gains carefully**, starting small.

Before running `arduino_IP.ino`, bring the hardware up in stages using the
sketches in `arduino_Calibration/`:

1. **SensorCalibration** — zero the P3022 offset so upright reads 0°.
2. **MotorTest** — verify stepper direction/speed/microstepping, independent of the sensor.
3. **SensorReader** — raw sensor readout for debugging.
4. **FinalHardwareTest** — combined sensor + motor check before trusting the closed loop.

Open the sketch in the Arduino IDE, select *Arduino Uno*, and upload. Watch the
Serial Monitor at **115200 baud**.

## Part 3 — Mechanical design (`CAD/`, `STL(3d_printing)/`)

Designed in Fusion 360 as two STEP assemblies (`CAD/Assemblies/`), built from
custom parts (`CAD/Custom/`) and off-the-shelf vendor parts (`CAD/Vendor/`).
The cart rides a 0.540 m T-slot rail (usable half-travel ≈ 0.194–0.214 m)
driven by a belt/pulley; the pendulum pivots in a single vertical plane.
Printable STLs for every custom/3D-printed part (platform, bearing slider,
bearing housing, pendulum stick, T-slot holders, spacers, drill guides) are in
`STL(3d_printing)/`.

## Part 4 — Electronics (`IP_Circuit_Schematics/`)

KiCad project wiring the Arduino Uno R3, the DRV8825 driver, and the P3022
sensor, plus the RC noise filter on the sensor line and the smoothing
capacitor on the motor supply. **Before opening in KiCad, double check the
DRV8825 `RST`/`SLP` pins are tied to +5V, not GND** — they're active-low and
must be logic-high for the driver to run.

## Part 5 — Reports

- `Single_IP_Project_Report.pdf` — full write-up: modeling, control design,
  software/hardware implementation, results, and appendix.
- `Hand_calculation.pdf` — the hand-derived Lagrangian equations of motion.

---

## Notes / assumptions

* **Mass convention:** `m1` = end bob (point mass at the tip), `m2` = the rod —
  matches `B = l(m1 + m2/2)` and `C = l²(m1 + m2/3)`.
* **Default Arduino wiring** (unless changed): P3022 OUT→A0, STEP→D2, DIR→D3,
  ENABLE→D4, all grounds common, motor power from the external 12–24 V supply.
* **Controller naming:** the Python and Arduino control classes are named PD
  (`pd_controller.py`, `CartPolePD`) — there is no integral term in either
  implementation.
