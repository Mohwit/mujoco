# Lesson 03: State and stepping

> **New to this?** Open the beginner-friendly, animated explainer first: `03-state-and-stepping-explained.html` (in this folder). It covers the same ideas in plain language, then gradually adds the official terms and optional maths.

## Goal

Understand how MuJoCo stores mutable state in `MjData` and changes it one timestep at a time.

## Mental model

`MjModel` is the fixed machine. `MjData` is the current snapshot.

For a one-joint pendulum:

- `data.qpos[0]` is the hinge angle.
- `data.qvel[0]` is the hinge angular velocity.
- `data.qacc[0]` is the hinge angular acceleration.
- `data.time` is simulation time.

`mj_forward(model, data)` computes derived values from the current state. It does not advance time.

`mj_step(model, data)` advances the simulation by one timestep. It computes forces and accelerations, integrates the state, and increases `data.time`.

```text
current qpos, qvel
        │
        ├── mj_forward → derived positions and velocities
        │
        └── mj_step → qacc → new qvel → new qpos → time + timestep
```

A reset restores the state. It does not rebuild the model.

## Runnable example

Run:

```bash
uv run --python .venv/bin/python lessons/03-state-and-stepping/03_state_and_stepping.py
```

The example sets an initial angle, calls `mj_forward()`, then calls `mj_step()` several times.

## Inspectable output

Compare the first two rows:

```text
before stepping: time=0.000 qpos=0.700 qvel=0.000
step 1:          time=0.002 qpos=0.700 qvel=...
```

The first `mj_forward()` computes a valid current state at time zero. Only `mj_step()` advances time.

## Exercise: predict first

1. If you call `mj_forward()` three times without `mj_step()`, does `data.time` change?
2. If `data.qpos[0] = 0.7` and `data.qvel[0] = 0`, what should happen to `qpos` after one step under gravity?
3. What happens to `data.time` after ten steps if the timestep is `0.002` seconds?
4. Add `mujoco.mj_resetData(model, data)` after the loop. Which values return to their initial state?

Change one line at a time. Print `time`, `qpos`, and `qvel` before and after each operation.

## Checkpoint

Report:

- Whether `mj_forward()` changes time.
- The time after ten steps.
- How `qpos` and `qvel` changed.
- Which values reset after `mj_resetData()`.

Do not mark this lesson complete until you can explain the difference between model constants and mutable data.
