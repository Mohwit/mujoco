# Lesson 04: Inspection and visualization

> **The interactive lesson is in `lessons/index.html` (Lesson 4).** It has pictures you can drive, predict boxes and the Model Inspector sandbox. These notes are the short written version.

## Goal

Find any part of a model by name or id, read where it is in the world (`xpos`, `xquat`, `xipos`, `site_xpos`), set up a camera and take a picture without a window, and read what a model looks like from its XML alone.

## The model

A two-link arm in the x-z plane (x right, z up, +y into the screen), plus a loose box:

- `upper_arm` at `pos="0 0 1"`, hinge `shoulder` about `0 1 0`, capsule 0.5 m, radius 0.04 m
- `forearm` at `pos="0.5 0 0"` in the upper arm, hinge `elbow` about `0 1 0`, capsule 0.4 m, radius 0.03 m, plus a 1 kg cube `weight` at the tip
- site `hand` at the forearm tip
- `box`: 0.2 m cube with a free joint `box_free`, at `(1.2, 0, 0.1)`, turned 30 degrees about z
- floor plane, a light, cameras `side` (fixed) and `track` (`mode="trackcom"`, inside the forearm)

## Mental model

Names are for people, ids are for arrays. Like a phone contacts list: you look up "Mum", the phone dials the number.

```text
name "forearm" --mj_name2id(model, mjOBJ_BODY, ...)--> id 2 --> data.xpos[2], data.xquat[2], data.xipos[2]
```

Every kind of part has its own list. Body 0 is always `world`.

| Array | Shape | Meaning | Unit, frame |
|---|---|---|---|
| `data.xpos` | (nbody, 3) | body frame origin | m, world |
| `data.xquat` | (nbody, 4) | body orientation `[w, x, y, z]` | none, world |
| `data.xmat` | (nbody, 9) | same orientation as a 3x3 matrix; columns are the body's axes | none, world |
| `data.xipos` | (nbody, 3) | body centre of mass | m, world |
| `model.body_ipos` | (nbody, 3) | centre of mass in the body's own frame | m, body |
| `data.site_xpos` | (nsite, 3) | site position | m, world |
| `data.cam_xpos` | (ncam, 3) | camera position | m, world |
| `model.jnt_qposadr` / `jnt_dofadr` | (njnt,) | first qpos / qvel index of each joint | index |

All `x...` arrays are computed by `mj_forward` (or `mj_step`). After you change `qpos`, they are stale until the next `mj_forward`.

A quaternion is "an axis to spin around plus how far": `w = cos(angle/2)`, `(x, y, z) = sin(angle/2) * axis`. A turn about y by angle a is `[cos(a/2), 0, sin(a/2), 0]`.

A free joint uses 7 qpos numbers (x, y, z, then a quaternion) but 6 qvel numbers (linear velocity in world, angular velocity in the body frame). So `nq = 9` but `nv = 8` here.

## Runnable example

```bash
uv run --python .venv/bin/python lessons/04-inspection-and-visualization/04_inspection_and_visualization.py
```

Key output:

```text
body    nbody = 4   0:world  1:upper_arm  2:forearm  3:box
joint   njnt  = 3   0:shoulder  1:elbow  2:box_free
geom    ngeom = 5   0:floor  1:upper_geom  2:forearm_geom  3:weight  4:box_geom
site    nsite = 1   0:hand
camera  ncam  = 2   0:side  1:track
body 2 forearm   xpos=( 0.5000,  0.0000,  1.0000) m  xquat=[ 1.0000,  0.0000,  0.0000,  0.0000]  xipos=( 0.7891,  0.0000,  1.0000) m
forearm body_ipos (centre of mass in its own frame) = ( 0.2891,  0.0000,  0.0000) m
site_xpos(hand) before mj_forward = ( 0.9000,  0.0000,  1.0000) m  (stale: still the zero pose)
site_xpos(hand) after  mj_forward = ( 0.5458,  0.0000,  0.3749) m (world)
forearm   xpos=( 0.4388,  0.0000,  0.7603) m  xquat=[ 0.7961,  0.0000,  0.6052,  0.0000]  xipos=( 0.5161,  0.0000,  0.4817) m
nq = 9 (numbers in qpos)   nv = 8 (numbers in qvel)
joint 2 box_free type=free  qposadr=2 (7 numbers: qpos[2:9])  dofadr=2 (6 numbers: qvel[2:8])
camera 1 track  mode=trackcom  cam_xpos at pose (0.5, 0.8) = ( 0.5170, -1.2000,  0.6817) m
```

It also saves four 320 x 240 PNGs in this folder (`cam_side_zero.png`, `cam_side_bent.png`, `cam_track_zero.png`, `cam_track_bent.png`) with `mujoco.Renderer`. If no OpenGL context is available, it prints a message and skips them.

Helper checks:

- `check_lesson4.py`: exercise answers (typo gives -1, the hand at shoulder 0 / elbow pi/2, a hinge after a free joint, the cart-pole XML exercise).
- `check_fk.py`: runs the sandbox's JavaScript forward kinematics in node and compares with MuJoCo at 8 poses (worst difference 2.2e-16).

## Forward kinematics by hand (pose 0.5, 0.8 rad)

```text
elbow = (0.5 cos 0.5, 0, 1 - 0.5 sin 0.5)       = (0.4388, 0, 0.7603)
hand  = elbow + 0.4 (cos 1.3, 0, -sin 1.3)       = (0.5458, 0, 0.3749)
forearm xquat = [cos 0.65, 0, sin 0.65, 0]       = [0.7961, 0, 0.6052, 0]
```

## Exercise: predict first

1. What does `mj_name2id(model, mjOBJ_JOINT, "elbow")` return? And with `mjOBJ_BODY`?
2. Why are the forearm's `xpos` and `xipos` different?
3. Shoulder 0, elbow pi/2: where is the hand? What is the forearm's `xquat`?
4. You set `qpos[0] = 1.0` and print `site_xpos` straight away. Is it right?
5. Read the cart-pole XML in the lesson and give nbody, njnt, nq, nv and the tip site position, without running it.

## Checkpoint

Report your answers to the five questions. Do not mark this lesson complete until you can find a part by name, read its world position after `mj_forward`, and explain why `nq` and `nv` differ here.
