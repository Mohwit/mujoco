# Lesson 11: Building bigger models

> **The interactive lesson is in `lessons/index.html` (Lesson 11).** It has the class tree, the step-by-step lookup ladder, the keyframe picker, the shelf generator and the style-sheet sandbox. These notes are the short written version.

## Goal

Write big models without repeating yourself: default classes, keyframes, `<include>`, and editing or generating models from Python with `MjSpec`.

This is the first lesson of Part 2. Lessons 11 to 15 each build one piece of the warehouse robot in Capstone 2 (lesson 16).

Prerequisites: Lesson 4 (the two-link arm, names, `site_xpos`) and Lesson 7 (`contype`, `conaffinity`, `condim`, friction).

## Files

| File | What it is |
|---|---|
| `11_building_bigger_models.py` | The lesson script (source of every number below) |
| `11_scene.xml`, `11_arm.xml`, `11_shelf.xml` | The same model split over three files with `<include>` |
| `11_check_browser.py` | Checks the page's JavaScript against MuJoCo (resolver for 64 toggle combinations, forward kinematics, keyframes) |

Run: `cd /Users/mohwit/Desktop/mujoco && .venv/bin/python lessons/11-building-bigger-models/11_building_bigger_models.py`

## Model

The lesson 4 arm (shoulder at (0, 0, 1) m, upper arm 0.5 m, forearm 0.4 m, 0.1 m cube `weight` at the hand, site `hand`) plus a shelf at x = 0.75 m with four fixed boxes and one loose red box (free joint). nbody 5, ngeom 12, njnt 3, nq 9, nv 8, nkey 4.

## Mental model: default classes are a style sheet

```xml
<default>                                   <!-- main -->
  <geom rgba="0.75 0.78 0.76 1"/>
  <default class="robot">
    <geom rgba="0.1 0.7 0.6 1" contype="0" conaffinity="0"/>
    <joint axis="0 1 0" damping="0.5"/>
    <default class="finger">
      <geom friction="2 0.05 0.01" condim="4" contype="1" conaffinity="1"/>
    </default>
  </default>
  <default class="furniture"> <geom type="box" friction="0.5 0.005 0.001" rgba="0.55 0.35 0.15 1"/> </default>
  <default class="object"> <geom type="box" size="0.05 0.05 0.05" friction="0.6 0.01 0.001" rgba="0.6 0.45 0.25 1"/> </default>
</default>
```

For every attribute the compiler takes the first level that sets it:

1. explicit, on the element
2. its class: `class="..."`, else the nearest body `childclass`, else `main`
3. parent classes, up to `main`
4. `main`
5. MuJoCo's built-in value (friction 1 0.005 0.0001, condim 3, contype 1, conaffinity 1, rgba 0.5 0.5 0.5 1)

The compiled `MjModel` keeps only numbers (`geom_friction` (12, 3), `geom_rgba` (12, 4), `geom_contype` (12,)...). It does not remember classes.

| geom | rgba | friction | contype |
|---|---|---|---|
| `floor` (main) | 0.75 0.78 0.76 1, main | 1 0.005 0.0001, built-in | 1, built-in |
| `upper_geom` (robot, childclass) | 0.1 0.7 0.6 1, class robot | built-in | 0, class robot |
| `forearm_geom` (robot) | 0.9 0.5 0.2 1, explicit | built-in | 0, class robot |
| `weight` (finger, own class) | 0.1 0.7 0.6 1, parent class robot | 2 0.05 0.01, class finger | 1, class finger |
| `box_b` (object, inside childclass furniture) | 0.6 0.45 0.25 1, class object | 0.6 0.01 0.001, class object | 1, built-in |
| `red_box_geom` (object) | 0.85 0.15 0.1 1, explicit | 0.6 0.01 0.001, class object | 1, built-in |

17 values typed inside `<default>` reach the elements 43 times (39 geom attributes + 4 joint attributes).

## Keyframes are save points

```xml
<keyframe>
  <key name="home"    qpos="-1.0 2.0   0.75 0 0.55 1 0 0 0"/>
  <key name="lesson4" qpos="0.5 0.8    0.75 0 0.55 1 0 0 0"/>
  <key name="reach"   qpos="-0.12 1.0  0.75 0 0.55 1 0 0 0"/>
  <key name="swing"   qpos="-1.0 2.0   0.75 0 0.55 1 0 0 0" qvel="3 0  0 0 0 0 0 0" time="2"/>
</keyframe>
```

`model.key_qpos` has shape (4, 9) = (nkey, nq). Load with `mujoco.mj_resetDataKeyframe(model, data, k)`, then `mj_forward`.

| state | qpos[0:2] (rad) | hand `site_xpos` (m, world) | time (s) | qvel[0] (rad/s) |
|---|---|---|---|---|
| `mj_resetData` (qpos0) | 0, 0 | (0.9000, 0, 1.0000) | 0 | 0 |
| home | -1.0, 2.0 | (0.4863, 0, 1.0841) | 0 | 0 |
| lesson4 | 0.5, 0.8 | (0.5458, 0, 0.3749) | 0 | 0 |
| reach | -0.12, 1.0 | (0.7513, 0, 0.7516) | 0 | 0 |
| swing | -1.0, 2.0 | (0.4863, 0, 1.0841) | 2.0 | 3.0 |

By hand for reach: x = 0.5 cos(-0.12) + 0.4 cos(0.88) = 0.4964 + 0.2549 = 0.7513 m; z = 1 + 0.0599 - 0.3083 = 0.7516 m, 0.1516 m above the red box's top (0.60 m).

Gotcha: 10 qpos numbers for nq = 9 is an error (`keyframe 'lesson4': invalid qpos size, expected 9, got 10`). Too few is silently filled from `qpos0` in MuJoCo 3.15.

## `<include>`

`11_scene.xml` (36 lines) includes `11_arm.xml` (13 lines) and `11_shelf.xml` (16 lines) inside `<worldbody>`. Loaded with `MjModel.from_xml_path`, it compiles to the same counts and identical arrays as the one-file XML. Including the same file twice is refused: `File '11_arm.xml' already included`.

## MjSpec: the editable blueprint

- `spec = mujoco.MjSpec.from_string(XML)`; `spec.compile()` returns a new `MjModel`.
- `spec.geom("red_box_geom").size = [0.04, 0.04, 0.04]`; compile again: the new model's red box is 0.512 kg (1000 kg/m^3 x 0.08^3 m^3); the old model is still 1.0 kg. Make a new `MjData`, or use `spec.recompile(model, data)` to keep the state (time 2.020 s, qpos[0] -0.9366 rad carried over).
- Defaults are copied when an element is created: after `spec.find_default("finger").geom.friction = [1, 0.05, 0.01]`, the existing `weight` keeps 2 0.05 0.01; a geom added afterwards gets 1 0.05 0.01.
- Generate: `s.add_default("furniture", s.default)`, then `add_body` / `add_geom(furn, ...)` in a loop. n shelves with k boxes: nbody = 1 + n, ngeom = 1 + n (3 + k) (3 x 4 gives 4 and 22; checked for n 1-6, k 1-4). `s.to_xml()` prints the XML.
- `cells.attach(MjSpec.from_string(XML), frame=frame, prefix="cell0_")` twice: nbody 9, ngeom 25, nq 18, nkey 8. Use a fresh child spec for each copy.

## Real world: the Capstone 2 warehouse robot

| class | writes | used by | why |
|---|---|---|---|
| robot | friction 0.8, condim 3, contype 0, conaffinity 0; joint damping 2, armature 0.01 | 7 geoms, 6 joints | chassis panels never collide; only wheels touch the floor |
| wheel | friction 1.5, condim 6 | 4 wheels | traction; turning and rolling friction |
| finger | friction 2.0, condim 4 | 2 finger geoms, grip_L, grip_R | squeezed box does not slip or spin |
| object | friction 0.6, condim 3 | red box (0.25 kg), blue cylinder | things to pick up |
| furniture | friction 0.5, condim 3 | 25 geoms | walls, shelves, table, crates, pylons |

Read the compiled arrays: 5 of the 7 robot geoms write `contype="1"` explicitly (only chassis_body and top_plate compile with 0), and every arm joint writes its own damping. Keyframes `home` and `pre_grasp`, `key_qpos` (2, 32) = 7 chassis + 4 wheels + 7 arm/gripper + 7 + 7 objects.

## Exercise: predict first

1. `box_a` has `class="object"` inside a body with `childclass="furniture"`. Friction?
2. A new `<geom name="bolt" type="sphere" size="0.01"/>` inside the forearm, no class. contype, rgba, friction?
3. A key with 10 qpos numbers when nq = 9? With 2?
4. You changed a geom size in a spec, but the simulation did not change. Why?

Answers (from the script): 0.6 0.01 0.001 from object; contype 0, rgba 0.1 0.7 0.6 1 (robot), friction 1 0.005 0.0001 (built-in); error / filled from qpos0; you must compile again and use the new model with a new data (or `recompile`).

## Checkpoint

Report the friction, contype and rgba of `weight` and the level each came from, the shape of `key_qpos`, and what `spec.compile()` returns.
