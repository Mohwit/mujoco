# Lesson 02: Frames, positions, and joint axes

> **New to this?** Open the beginner-friendly, animated explainer first: `02-frames-and-axes-explained.html` (in this folder). It covers the same ideas in plain language, then gradually adds the official terms and optional maths.

## Goal

Understand which coordinates belong to a body frame, which coordinates are world coordinates, and how a joint axis defines motion.

## Mental model

Every MuJoCo body creates a coordinate frame.

- `pos` on a `<body>` places that body frame relative to its parent frame.
- A geom's `pos` is local to the body that owns it.
- A joint's `axis` is also expressed in its body's local frame.
- MuJoCo combines the hierarchy to compute world coordinates such as `body_xpos` and `geom_xpos`.

A child body does not start at world zero. Its local origin starts at its `pos` relative to the parent. This is why nested bodies are useful: move the parent frame and its children move with it.

### Local versus world coordinates

```text
world frame
└── base body at (0, 0, 1)
    └── link body at (0, 0, 0.6) in base coordinates
        └── geom at (0, 0, -0.8) in link coordinates
```

The link body position is not `(0, 0, 0.6)` in world coordinates unless the parent has no rotation and its origin is at world zero. In this example, the parent offset is part of the final world position.

### Joint axes

For a hinge joint, `axis="0 1 0"` means rotation around the body's local y axis. The axis is not a direction to the target. It is the line around which the joint rotates.

## MJCF example

```xml
<body name="base" pos="0 0 1">
  <body name="link" pos="0 0 0.6">
    <joint name="hinge" type="hinge" axis="0 1 0"/>
    <geom name="link_geom" type="capsule" fromto="0 0 0  0 0 -0.8" size="0.08"/>
  </body>
</body>
```

The capsule's `fromto` endpoints are local to `link`. The world position and orientation are computed after MuJoCo applies the body hierarchy and joint state.

## Runnable example

Run:

```bash
uv run --python .venv/bin/python lessons/02-frames-and-axes/02_frames_and_axes.py
```

The script prints local inputs and world-space outputs. `mj_forward()` is important here because it propagates the current `qpos` through the model before we inspect derived positions.

## Inspectable output

Look for these relationships:

```text
base local pos:       [0. 0. 1.]
link local pos:       [0.  0.  0.6]
link world position:  [0.  0.  1.6]
```

The exact geom position changes when the joint angle changes. The number of bodies and joints does not.

## Exercise: predict first

Before running the script, write your predictions:

1. If `base pos="0 0 1"` becomes `base pos="0 0 2"`, what happens to the link world position?
2. If the joint axis changes from `0 1 0` to `1 0 0`, which rotation direction changes?
3. If the link body's local `pos="0 0 0.6"` becomes `0 0 1.2`, does the link move in world space?

Then change one value at a time and run the script again.

## Checkpoint

Report:

- Your three predictions.
- The printed `link world position` before and after changing the base position.
- Which axis the joint used and how you know.

Do not mark this lesson complete until you can explain why local coordinates and world coordinates are different.
