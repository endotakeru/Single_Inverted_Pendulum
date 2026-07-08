# Single Inverted Pendulum — Simulation + Hardware Control

An educational control-systems project: a cart on a rail balances a rod with a
mass on the end (an inverted pendulum) using PID feedback. It has two halves —
a **Python simulation** to design and understand the controller, and an
**Arduino controller** for the real hardware rig.

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
single_inverted_pendulum/
├── python_simulation/        # the Python simulation (design + study)
│   ├── main.py               #   runs the whole simulation start to finish
│   ├── params.py             #   constants, simulation settings, PID gains
│   ├── dynamics.py           #   nonlinear equations of motion (the plant)
│   ├── linear_model.py       #   linearised state-space A, B, C, D matrices
│   ├── pid_controller.py     #   PID control law (angle loop + position loop)
│   ├── simulation.py         #   time-domain run with scipy solve_ivp
│   └── plot_results.py       #   plots: position, angle, velocities, force
├── arduino_control/
│   └── arduino_control.ino   # Arduino Uno sketch for the physical rig
├── hardware_notes/
│   └── wiring_notes.md       # wiring diagram + safety checklist
├── index.html                # standalone browser version of the simulator
└── README.md                 # this file
```

---

## Part 1 — Python simulation (`python_simulation/`)

Design and study the controller in software first.

```bash
pip install numpy scipy matplotlib
cd python_simulation
python main.py
```

Useful options:

```bash
python main.py                 # tuned balancing gains + plots (default)
python main.py --gains spec    # the spec gains (Kp_theta = 1) -> falls over & trips
python main.py --sweep         # sweep theta0 = -15..+15 deg; all angles overlaid (theta & x panels)
```

In the simulation the controller output is a **force** on the cart. Two PID loops
are **summed into one cart force**:

```
F = (Kp_theta*theta + Ki_theta*∫theta + Kd_theta*d theta/dt)   # angle loop (upright)
  + (Kp_x*x         + Ki_x*∫x         + Kd_x*d x/dt)            # position loop (centre)
```

The default `params.py` gains (`Kp_theta = 1`) are intentionally too weak to
balance; `Params.balanced()` / `--gains preset` (`Kp_theta = 24`, `Kd_theta = 2.6`)
do balance. (`index.html` is the same simulator in a browser — just open the file.)

## Part 2 — Arduino controller (`arduino_control/arduino_control.ino`)

A clean **starting** controller for the real rig, to be tuned on the bench.

- Board: **Arduino Uno R3** (powered over USB).
- Motor: **NEMA 17 stepper** driven by an **external STEP/DIR driver** on a
  **separate 12–24 V supply**; the cart is moved by the stepper.
- Sensor: **P3022** analog angle sensor read on **A0**.
- It reads the angle, runs a PID, and converts the PID output into a stepper
  **direction + step frequency** (bigger error → step faster).

> **Important difference from the simulation:** the sim's actuator is a *force*;
> the stepper is a *velocity* command. So the simulation gains do **not** transfer
> directly — they are included in the sketch only as a reference. **Tune the
> hardware gains carefully**, starting small.

Open `arduino_control/arduino_control.ino` in the Arduino IDE, select
*Arduino Uno*, and upload. Watch the Serial Monitor at **115200 baud**.

## Part 3 — Hardware notes (`hardware_notes/wiring_notes.md`)

Full wiring tables (P3022, stepper driver, motor supply), an explanation of the
shared-ground and "don't power the motor from the Arduino" rules, and a safety
checklist (current limit, no mains on breadboards, don't hot-unplug the motor,
limit switches, emergency stop). **Read it before powering the motor.**

---

## Notes / assumptions

* **Mass convention:** `m1` = end bob (point mass at the tip), `m2` = the rod —
  matches `B = l(m1 + m2/2)` and `C = l²(m1 + m2/3)`.
* **Default Arduino wiring** (unless changed): P3022 OUT→A0, STEP→D2, DIR→D3,
  ENABLE→D4, all grounds common, motor power from the external 12–24 V supply.
* The Python files were **moved** (not deleted) from the project root into
  `python_simulation/` to match this layout; they import each other and run the
  same as before from inside that folder.
