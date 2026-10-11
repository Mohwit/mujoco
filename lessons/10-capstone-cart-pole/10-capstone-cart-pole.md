# Capstone 1: Balance a cart-pole

> **The interactive lesson is in `lessons/index.html` (Capstone 1, slot 10).** It has the three-controller picture, the pose and fall experiments, the mass-matrix explorer, the controller comparison, the predict boxes and the cart-pole sandbox. These notes are the short written version.

## Goal

Build a model you have not seen, a pole balanced on a sliding cart, and keep it upright with a controller. Nothing here is new MuJoCo: every milestone uses Lessons 1 to 9. The only new idea is that the controller must watch the cart as well as the pole (full-state feedback).

Analogy: balancing a broom on your palm. You move your hand under the broom, and you must also not walk off the table.

## The model

```xml
<option timestep="0.002" gravity="0 0 -9.81"/>
<worldbody>
  <geom name="rail" type="box" size="1.2 0.01 0.01" pos="0 0 0.5" contype="0" conaffinity="0"/>
  <body name="cart" pos="0 0 0.5">
    <joint name="slider" type="slide" axis="1 0 0" range="-1 1"/>
    <geom name="cart_box" type="box" size="0.1 0.075 0.05" mass="1" contype="0" conaffinity="0"/>
    <body name="pole" pos="0 0 0">
      <joint name="hinge" type="hinge" axis="0 1 0"/>
      <geom name="pole_rod" type="capsule" fromto="0 0 0 0 0 0.6" size="0.02" contype="0" conaffinity="0"/>
      <site name="tip" pos="0 0 0.6"/>
    </body>
  </body>
</worldbody>
<actuator>
  <motor name="push" joint="slider" gear="20" ctrlrange="-1 1"/>
</actuator>
<sensor>
  <jointpos name="cart_x" joint="slider"/>    <jointpos name="pole_angle" joint="hinge"/>
  <jointvel name="cart_v" joint="slider"/>    <jointvel name="pole_spin" joint="hinge"/>
  <framepos name="tip_pos" objtype="site" objname="tip"/>
</sensor>
```

| Number | Reason |
|---|---|
| timestep 0.002, gravity -9.81 | Course defaults (Lesson 3) |
| slider range -1..1 m | A 2 m table-top track; past it the joint limit acts as an end stop |
| cart 1 kg, box 20 x 15 x 10 cm | A small trolley, round mass |
| pole capsule 0.6 m up, radius 0.02 m | Points up, so lean 0 = upright and gravity makes it fall (inverted pendulum). Default density 1000 kg/m³ gives 0.7875 kg |
| contype 0, conaffinity 0 | No contacts (Lesson 7): ncon = 0 always |
| gear 20, ctrlrange -1..1 | At most 20 N; the balance tests use at most 11.21 N, so no saturation |

Sign convention: hinge axis +y, so a **positive lean tips the top towards +x**. Positive ctrl pushes the cart towards +x.

## Run

```bash
cd /Users/mohwit/Desktop/mujoco && .venv/bin/python lessons/10-capstone-cart-pole/10_capstone_cart_pole.py
cd /Users/mohwit/Desktop/mujoco && .venv/bin/python lessons/10-capstone-cart-pole/10_check_browser_copy.py   # needs node
```

## Milestone 1: build and predict (Lessons 1, 2, 4, 5, 6, 7)

Predict first, then compare: nbody = 3 (world, cart, pole), njnt = 2, nq = 2, nv = 2, nu = 1, ngeom = 3, nsite = 1, nsensor = 5, nsensordata = 7.

- `qpos = [x (m), lean (rad)]`, `qvel = [x speed (m/s), lean speed (rad/s)]`.
- `sensordata[0:4] = [cart_x, pole_angle, cart_v, pole_spin]`, `sensordata[4:7]` = tip position, world frame. At x = 0, lean = 0.1: `[0.0, 0.1, 0.0, 0.0, 0.0599, 0.0, 1.097]`.
- Two joints, one motor: the pole can only be moved by moving the cart.

## Milestone 2: let go (Lesson 3)

| start lean (rad) | passes 0.5 rad at (s) | cart x then (m) |
|---|---|---|
| 0.05 | 0.510 | -0.0574 |
| 0.1 | 0.392 | -0.0506 |

The cart moves **opposite** to the fall: no outside sideways force, so the centre of mass stays put (0.013195 m at the start, 0.013174 m at 0.392 s, 0.012839 m at 0.6 s; the drift is integrator step error).

## Milestone 3: mass matrix and equations of motion (Lessons 3, 8)

`mj_fullM` at lean 0: `[[1.787493, 0.236248], [0.236248, 0.096741]]`; at lean 0.3 the off-diagonal is 0.225696.

```text
[ 1.787493          0.236248 cos θ ] [qacc0]   [ 20 ctrl + 0.236248 sin θ · θ̇² ]
[ 0.236248 cos θ    0.096741       ] [qacc1] = [ 2.317591 sin θ                ]
```

- M00 = cart + pole mass; M11 = 0.025867 + 0.787493 x 0.3² = pole inertia about the hinge; M01 = pole mass x 0.3 m x cos(lean) is the **coupling**: pushing the cart tips the pole (the tablecloth trick).
- Hand solution vs MuJoCo: lean 0.1 at rest, ctrl 0: `[-0.462220, +3.514795]` (diff 8.9e-16). A moving state with ctrl 0.4: `[+5.156987, -4.951526]` (diff 3.6e-15). Upright, ctrl 0.5: `[+8.260626, -20.172924]`: the pole tips left when the cart is pushed right.
- The web page uses exactly these equations with semi-implicit Euler; `10_check_browser_copy.py` shows it matches MuJoCo to 8.9e-16 over six runs.

## Milestone 4: controllers (Lessons 5, 8)

```text
ctrl = k_offset * x + k_lean * lean + k_cart_speed * x_speed + k_lean_speed * lean_speed     (force = 20 ctrl N)
```

| Knob | LQR value | In newtons |
|---|---|---|
| cart offset | 0.9827 per m | 19.654 N/m |
| lean | 4.1404 per rad | 82.808 N/rad |
| cart speed | 0.8076 per m/s | 16.152 N/(m/s) |
| lean speed | 0.7561 per rad/s | 15.123 N/(rad/s) |

- **Lean-only PD** (cart knobs 0): pole upright from 0.488 s, but the cart rolls on at 0.2284 m/s and hits the 1 m stop at 4.398 s, which knocks the pole over.
- **Four knobs**, from 0.1 rad: settle (|x| < 0.01 m and |lean| < 0.01 rad) 1.482 s, max travel +0.0962 m, peak ctrl 0.4140 (8.28 N). From 0.2 rad: 1.922 s, 0.1962 m, 0.8281.
- **Push** (`qfrc_applied[1] = +1.0 N m` on the hinge for 0.1 s): lean range -0.1369 to +0.0285 rad, travel 0.2202 m, peak ctrl 0.5604, settle 2.050 s.
- **Surprising sign**: from x = +0.5 m the cart first moves away, to +0.5425 m, to tip the pole towards home.
- **Sign check**: negate all four knobs and the pole is spinning (|lean| = 12.311 rad) after 1 s.
- Gains: linearise near upright (`qacc = Kq [x, θ] + Ku ctrl`, uncontrolled lean grows like exp(5.9476 t)), write one Euler step as `s' = A s + B ctrl` (matches `mjd_transitionFD` to 4.5e-14), and iterate the discrete Riccati equation with cost x² + θ² + ctrl² (2390 iterations, numpy only). LQR returns K = -knobs. Largest growth factor per step 0.995763: the slowest wobble halves every 0.327 s.

## Milestone 5: Gymnasium environment (Lesson 9)

`CartPoleBalanceEnv`: obs (4,) = `sensordata[:4]` (refreshed with `mj_forward` after stepping), action (1,) = ctrl in [-1, 1], frame_skip 10 (0.02 s), reward +1 per step, terminated when |lean| > 0.2 rad or |x| > 0.9 m, truncated after 500 steps (10 s). `check_env` passes with 2 warnings (infinite observation bounds).

| seed | zero | random | lean-only | four knobs |
|---|---|---|---|---|
| 0 | 21 | 30 | 387 | 500 |
| 1 | 16 | 28 | 500 | 500 |
| 2 | 22 | 11 | 500 | 500 |
| 3 | 23 | 9 | 500 | 500 |
| 4 | 35 | 8 | 500 | 500 |
| mean | 23.4 | 17.2 | 477.4 | 500.0 |

## Checkpoint

1. A second pole on top (new hinge, no new motor): nq, nv, nu, and the mass-matrix size?
2. Lean +0.1 rad, at rest, four knobs: ctrl and force at step 0?
3. Why does lean-only PD lose the cart?
4. Cart at +0.5 m, pole upright: which way does the controller push first, and why?
5. Why end an episode at |x| > 0.9 m rather than at the 1 m joint limit?

Next: Lesson 11, Building Bigger Models.
