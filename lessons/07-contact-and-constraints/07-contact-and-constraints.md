# Lesson 07: Contact and constraints

> **The interactive lesson is in `lessons/index.html` (Lesson 7).** It covers the same ideas with pictures you can drive, predict boxes, and a ramp sandbox that replays numbers recorded from MuJoCo.

## Goal

Predict which geom pairs can collide, read the contacts MuJoCo finds, and fix sliding, sinking or pass-through problems with `friction`, `condim`, `solref` and `margin`.

## Mental model

A contact is the floor pushing back. Friction is the sideways grip, and it has a limit.

MuJoCo answers three questions every step:

1. **Who may touch?** Two geoms can collide only if `(contype1 & conaffinity2) || (contype2 & conaffinity1)` is non-zero, and they are not a parent-child body pair.
2. **Where do they touch?** Every touching pair goes into `data.contact`, and `data.ncon` counts them.
3. **How hard do they push and grip?** `mj_contactForce` gives the push in the contact frame. Sliding grip is at most `mu * N` (Coulomb's rule).

Model or data:

- `friction`, `condim`, `margin`, `contype`, `conaffinity`, `solref` are written in the XML. They live in `model.geom_*` arrays.
- `data.ncon` and `data.contact` are rebuilt every step.

## Runnable example

```bash
uv run --python .venv/bin/python lessons/07-contact-and-constraints/07_contact_and_constraints.py
```

The sandbox data comes from a helper that records box-on-ramp runs:

```bash
uv run --python .venv/bin/python lessons/07-contact-and-constraints/record_ramp_grid.py
```

## What the script shows (real output, MuJoCo 3.15.0)

### A. Collision filtering

| geom | contype | conaffinity | with the floor (1/1) | after 1 s, centre z (world) |
|---|---|---|---|---|
| red | 1 | 1 | 1 & 1 = 1, can collide | 0.0996 m, resting |
| green | 2 | 2 | 1 & 2 = 0 and 2 & 1 = 0, ignored | -4.3952 m, fell through |
| blue | 0 | 1 | 1 & 1 = 1, can collide (one direction is enough) | 0.0996 m, resting |

`data.ncon = 2`, pairs `floor-red` and `floor-blue`. Two capsules overlapping at a hinge (parent and child) give `ncon = 0` by default and `ncon = 2` with `<flag filterparent="disable"/>`.

### B. Resting contact (1 kg box, 0.2 m cube)

```text
data.ncon = 4   (one contact per box corner touching the floor)
  contact 0: geom1=floor geom2=box  dist=-0.000108 m  pos=(-0.100, -0.100, -0.000054) m  normal=frame[0:3]=(0, 0, 1)  normal force=2.4525 N
  ...
sum of normal forces = 9.8100 N   vs m*g = 9.8100 N
```

- `dist` is in metres; negative means overlap. `pos` is in the world frame, midway between the surfaces.
- The first row of `frame` is the contact normal, pointing from `geom1` to `geom2`.
- Penetration depends on `solref` time constant: 0.005 s gives -0.007 mm, 0.02 s (default) gives -0.108 mm, 0.05 s gives -0.491 mm.

### C. Friction on a ramp

Threshold angle = atan(mu): 11.31 deg for mu = 0.2, 26.57 deg for mu = 0.5, 38.66 deg for mu = 0.8.

At 20 deg, mu = 0.5, `mj_contactForce` summed over 4 corners gives N = 9.2184 N (= m g cos 20) and friction 3.3552 N (= m g sin 20). The limit mu N = 4.6092 N is larger, so the box sticks.

Downhill distance after 1 s (mm), S = slides:

| angle | tan | mu = 0.2 | mu = 0.5 | mu = 0.8 | ideal, mu = 0.5 |
|---|---|---|---|---|---|
| 10 | 0.1763 | 0.9 | 0.6 | 0.7 | 0.0 |
| 15 | 0.2679 | 322.6 S | 1.0 | 1.1 | 0.0 |
| 20 | 0.3640 | 757.3 S | 1.9 | 1.7 | 0.0 |
| 25 | 0.4663 | 1186.2 S | 5.6 | 2.3 | 0.0 |
| 26 | 0.4877 | 1271.0 S | 6.7 | 2.6 | 0.0 |
| 27 | 0.5095 | 1355.4 S | 53.0 S | 2.8 | 41.6 |
| 30 | 0.5774 | 1606.8 S | 332.4 S | 4.7 | 328.6 |
| 35 | 0.7002 | 2013.8 S | 803.6 S | 9.8 | 804.4 |
| 40 | 0.8391 | 2405.9 S | 1271.5 S | 154.3 S | 1274.1 |

Honest fine print: MuJoCo's soft contacts creep a few mm per second below the threshold and let go slightly early (26.5 deg moves 22.34 mm). `<option cone="elliptic" impratio="10"/>` reduces this: 26.5 deg moves 0.22 mm.

condim on a 10 deg ramp, mu = 0.5: `condim="1"` slides 853.4 mm (frictionless ideal 851.7 mm), `condim="3"` moves 0.6 mm.

### D. Margin

- Ball radius 0.1 m with a 0.03 m air gap: floor margin 0 lists no contact; margin 0.05 lists one with `dist = +0.030 m`.
- With margin 0.05 the ball rests at z = 0.1496 m instead of 0.0996 m: a margin pushes early, so things float by about the margin.
- A ball (radius 0.02 m) thrown at 10 m/s at a 1 cm plate passes through with margin 0 (z = -4.9309 m after 0.5 s) and stops with plate margin 0.05 (z = 0.0746 m).

## Maths

- Normal force on a ramp: N = m g cos(angle).
- Pull along the ramp: m g sin(angle).
- Sticks while m g sin(angle) <= mu m g cos(angle), so tan(angle) <= mu. Threshold = atan(mu).
- Above it, a = g (sin(angle) - mu cos(angle)). At 30 deg, mu = 0.5: a = 0.6571 m/s^2, d = a t^2 / 2 = 328.6 mm in 1 s. MuJoCo: 332.4 mm.

## Exercise: predict first

1. Floor 1/1, ball contype 2, conaffinity 3. Do they collide?
2. A 1 kg box rests on 4 corners. What is the sum of `f6[0]`?
3. mu = 0.8. At what angle should the box start sliding? What setting reduces creep?
4. A contact has `dist = +0.02 m`. Are the geoms touching? Why is it listed?
5. A box slides on a 10 deg ramp with mu = 0.5. What do you check first?

Change one setting at a time and rerun the script.

## Checkpoint

Report your five answers. Then, in the sandbox, find the angle where the box starts sliding for mu = 0.3 and explain what condim 1 changes.

## Debugging rules

- Falls through: check `contype`/`conaffinity` first, then parent-child filtering, then thin geoms, margin and timestep.
- Sinks too far: smaller `solref` time constant (keep it at least twice the timestep).
- Slides when it should stick: raise `friction`, make sure `condim` is not 1, try `cone="elliptic"` with a larger `impratio`.
