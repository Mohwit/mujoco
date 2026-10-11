# Lesson 06: Sensors and observation

> **The interactive lesson is in `lessons/index.html` (Lesson 6).** It has the animated dashboard, the stale-sensor lab, the accelerometer picture, the angle-wrapping circle, and the sensor sandbox. These notes are the written summary.

## Goal

Declare sensors in MJCF, read their values from `data.sensordata` with the right slice, and build an observation vector for a control policy.

## Mental model

A real robot cannot read MuJoCo's memory. It only knows what its sensors report. Think of a car dashboard: each gauge measures one thing, and all gauges sit on one panel.

- Each `<sensor>` child in the XML is one gauge.
- `data.sensordata` is the panel: one flat array with every reading, in XML order.
- Sensor `i` owns `sensordata[sensor_adr[i] : sensor_adr[i] + sensor_dim[i]]`.
- `data.sensor("name").data` reads one sensor's slice by name.

The model is the course pendulum (lesson 3) with the lesson 5 motor (`name="motor"`, `gear="100"`, `ctrlrange="-1 1"`), plus a site `tip` at the rod end (`pos="0 0 -1"` in the pendulum frame).

| id | Sensor | Measures | dim | Frame | Unit | Slice |
|---|---|---|---|---|---|---|
| 0 | `jointpos angle` | hinge angle | 1 | joint | rad | `[0]` |
| 1 | `jointvel spin` | hinge spin speed | 1 | joint | rad/s | `[1]` |
| 2 | `actuatorfrc motor_force` | `actuator_force`, before the gear (= ctrl here) | 1 | actuator | ctrl units (x100 = N m at the hinge) | `[2]` |
| 3 | `framepos tip_pos` | tip position | 3 | world | m | `[3:6]` |
| 4 | `accelerometer tip_acc` | acceleration minus gravity | 3 | tip site | m/s^2 | `[6:9]` |

So `nsensor = 5` but `nsensordata = 9`, and the tip z is at index 5.

## Runnable example

```bash
uv run --python .venv/bin/python lessons/06-sensors-and-observation/06_sensors_and_observation.py
```

Helper that checks the page's in-browser maths against MuJoCo:

```bash
uv run --python .venv/bin/python lessons/06-sensors-and-observation/check_browser_match.py
```

## Inspectable output

```text
nsensor      (number of sensors)          5
nsensordata  (length of data.sensordata)  9
id  name          type                  adr  dim  noise
3   tip_pos       mjSENS_FRAMEPOS       3    3    0.00
4   tip_acc       mjSENS_ACCELEROMETER  6    3    0.20
```

Key facts the script shows:

1. **At rest** the accelerometer reads `(0, 0, 9.81)` m/s^2 in the tip site frame. It measures real acceleration minus gravity: `0 - (-9.81) = +9.81`, like a phone lying on a table.
2. **Stale values.** Setting `qpos[0] = 0.7` does not change `sensordata`. Only `mj_forward` or `mj_step` fill it. After `mj_forward`, `sensordata[0] = 0.70000 = qpos[0]` and `sensordata[3:6] = (-0.64422, 0, 0.23516) = site_xpos` of the tip.
3. **After `mj_step` the sensors are one step behind.** `mj_step` computes sensors first, then integrates: `qpos[0] = 0.69996` but the angle sensor still says `0.70000`.
4. **actuatorfrc is before the gear.** With `ctrl = 0.5`, `motor_force = 0.500` while `qfrc_actuator = 50.0` N m at the hinge.
5. **Bottom of the swing** at `qvel = 2.0` rad/s: `tip_acc z = 9.81 + 2.0^2 x 1.0 = 13.81` m/s^2 (centripetal term).
6. **Noise is stored, not applied.** MuJoCo 3.15 keeps `noise` in `model.sensor_noise`, but three `mj_forward` calls give identical readings. Add noise yourself with a seeded generator (`np.random.default_rng(0)`).

## Observation vector

```python
obs = np.array([np.cos(q), np.sin(q), qd])   # shape (3,), float64, q and qd from the sensors
```

Use `cos` and `sin` because the angle wraps: 0 and 2 pi rad are the same pose. Optionally append `motor_force` (already in -1..1).

Rollout from rest with `ctrl = 0.5` (50 N m at the hinge):

```text
 time   angle(rad)   obs = [cos(angle), sin(angle), spin rad/s]    motor_force  qfrc_actuator
 0.000   0.00000    [ 1.00000,  0.00000,  0.00000]     0.50         50.0 N m
 0.500   0.59386    [ 0.82879,  0.55957,  1.67319]     0.50         50.0 N m
```

## Exercise: predict first

1. Add `<gyro site="tip"/>` as the last sensor. Predict `nsensor`, `nsensordata` and its `sensor_adr`. (Answer: 6, 12, 9.)
2. Set `data.qpos[0] = 1.0` and read the angle sensor without `mj_forward`. What do you get?
3. Set `qpos = 0`, `qvel = 3.0` and call `mj_forward`. Predict `tip_acc` z. (Answer: 18.81 m/s^2.)
4. Change `noise="0.01"` to `noise="1.0"` on `angle`. Does `sensordata[0]` change?

## Checkpoint

Report:

- The length of `sensordata` and the slice each sensor owns.
- What the accelerometer reads at rest, and why it is +9.81 rather than -9.81.
- Why `sensordata` can disagree with `qpos` right after `mj_step`.
- Your observation vector and why it uses cos and sin.
