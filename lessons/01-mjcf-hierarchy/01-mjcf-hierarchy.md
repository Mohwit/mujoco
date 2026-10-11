# Lesson 1: MuJoCo model hierarchy

> **New to this?** Open the beginner-friendly, animated explainer first: `01-mjcf-hierarchy-explained.html` (in this folder). It covers the same ideas in plain language, then gradually adds the official terms and optional maths.

## Goal

Understand how MJCF XML becomes a MuJoCo model.

## Mental model

MuJoCo separates three things:

- **`MjModel`**: the fixed design of the simulation.
- **`MjData`**: the changing simulation state.
- **MJCF/XML**: the description of bodies, joints, and geometry.

The lesson model has this structure:

```text
world body
└── pendulum body
    ├── hinge joint
    ├── rod geom
    └── bob body
        └── weight geom
```

Here, `pendulum` is the parent body. It contains the hinge and rod, and it owns the `bob` child body. The complete physical pendulum assembly is therefore the `pendulum` body plus its child `bob` body and their geoms.

`bob` is not another complete pendulum. It is a child body frame placed at the rod endpoint. Its `weight` geom gives that body its visible mass shape.

The `world` body is always present. The model therefore contains three body frames:

```text
world       # fixed root frame
pendulum    # parent body with the hinge and rod
bob         # child body at the rod endpoint
```

The two visible shapes are geoms:

```text
rod         # capsule geom attached to pendulum
weight      # sphere geom attached to bob
```

A body is a frame in the hierarchy. A geom is the visible and physical shape attached to a body. A body can contain other bodies.

The model contains one hinge joint. Therefore it has one position value and one velocity value:

```text
qpos: 1
qvel: 1
```

The hinge angle is stored in:

```python
data.qpos[0]
```

## Learning artifacts

Open the interactive explorer in a browser:

```text
01-mjcf-hierarchy.html
```

It lets you change body height, rod length, and joint angle. It also shows which model counts stay unchanged.

View the static hierarchy diagram:

```text
01-mjcf-hierarchy.svg
```

## Runnable example

Run:

```bash
uv run --python .venv/bin/python lessons/01-mjcf-hierarchy/01_mjcf_hierarchy.py
```

Source file:

```text
lessons/01-mjcf-hierarchy/01_mjcf_hierarchy.py
```

The important operations are:

```python
model = mujoco.MjModel.from_xml_string(XML)
data = mujoco.MjData(model)

data.qpos[0] = 0.7
mujoco.mj_forward(model, data)
mujoco.mj_step(model, data)
```

- `mj_forward()` computes derived values from the current state.
- `mj_step()` advances the physics by one timestep.

## Expected output

The exact final position can vary slightly by MuJoCo version. The structure should look like this:

```text
Model counts:
  bodies: 3
  joints: 1
  geoms:  2
  qpos:   1
  qvel:   1

Bodies:
  0: world
  1: pendulum
  2: bob

Geoms:
  0: rod
  1: weight

Joint state before stepping: [0.]
Joint state after one step:  [approximately 0.7]
```

## Meaning of the model counts

| Value | Meaning in this model |
|---|---|
| `bodies: 3` | Three body frames: `world`, `pendulum`, and `bob`. |
| `joints: 1` | One hinge joint controls the pendulum angle. |
| `geoms: 2` | Two physical shapes: the rod capsule and the weight sphere. |
| `qpos: 1` | One position coordinate: the hinge angle. |
| `qvel: 1` | One velocity coordinate: the hinge angular velocity. |

A body is a frame in the hierarchy. A geom is a shape attached to a body. `qpos` and `qvel` describe the current joint state, not the number of bodies or geoms.

## Exercise

Make one change at a time. Predict the result before running the script.

### Experiment A: change the body position

Change:

```xml
<body name="pendulum" pos="0 0 1">
```

to:

```xml
<body name="pendulum" pos="0 0 2">
```

Predict:

1. How does the pendulum's world position change?
2. Does the joint angle change?
3. Does `model.nq` change?

### Experiment B: change the rod length

Change:

```xml
fromto="0 0 0 0 0 -1"
```

to:

```xml
fromto="0 0 0 0 0 -2"
```

Predict:

1. How does the rod change visually?
2. Does the bob body move automatically?
3. Does `model.nq` change?

## Checkpoint

Record the following in your response or in a personal notes file:

- Your predictions.
- The output after Experiment A.
- The output after Experiment B.
- Which predictions were correct.

Do not mark this lesson complete until both experiments are run and explained.

## Next lesson

MJCF frames, body positions, joint axes, and coordinate systems.
