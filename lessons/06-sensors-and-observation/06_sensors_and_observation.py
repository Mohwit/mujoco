"""Lesson 06: declare sensors in MJCF, read sensordata, build an observation vector.

Run:  uv run --python .venv/bin/python lessons/06-sensors-and-observation/06_sensors_and_observation.py
"""

import mujoco
import numpy as np

# The course pendulum from lessons 3 and 5 (1 m capsule, hinge about y, lesson 5 motor "motor"),
# plus a site at the rod's end and five sensors.
XML = r"""
<mujoco model="sensor_pendulum">
  <option timestep="0.002" gravity="0 0 -9.81"/>
  <worldbody>
    <body name="pendulum" pos="0 0 1">
      <joint name="hinge" type="hinge" axis="0 1 0"/>
      <geom type="capsule" fromto="0 0 0 0 0 -1" size="0.08"/>
      <site name="tip" pos="0 0 -1" size="0.02"/>
    </body>
  </worldbody>
  <actuator>
    <motor name="motor" joint="hinge" gear="100" ctrlrange="-1 1" ctrllimited="true"/>
  </actuator>
  <sensor>
    <jointpos      name="angle"        joint="hinge"   noise="0.01"/>
    <jointvel      name="spin"         joint="hinge"   noise="0.05"/>
    <actuatorfrc   name="motor_force"  actuator="motor"/>
    <framepos      name="tip_pos"      objtype="site" objname="tip"/>
    <accelerometer name="tip_acc"      site="tip"      noise="0.2"/>
  </sensor>
</mujoco>
"""

# What each sensordata element means (frame and unit).
LABELS = [
    ("angle", "hinge angle", "joint", "rad"),
    ("spin", "hinge angular velocity", "joint", "rad/s"),
    ("motor_force", "actuator_force, before gear", "actuator", "ctrl units"),
    ("tip_pos", "tip x", "world", "m"),
    ("tip_pos", "tip y", "world", "m"),
    ("tip_pos", "tip z", "world", "m"),
    ("tip_acc", "accel x", "tip site", "m/s^2"),
    ("tip_acc", "accel y", "tip site", "m/s^2"),
    ("tip_acc", "accel z", "tip site", "m/s^2"),
]


def print_sensordata(data: mujoco.MjData) -> None:
    for i, (name, meaning, frame, unit) in enumerate(LABELS):
        print(f"  sensordata[{i}]  {name:<13} {meaning:<27} ({frame:<8}) {data.sensordata[i]: .5f} {unit}")


def observation(data: mujoco.MjData) -> np.ndarray:
    """Policy input: [cos(angle), sin(angle), spin], shape (3,)."""
    q = data.sensor("angle").data[0]
    qd = data.sensor("spin").data[0]
    return np.array([np.cos(q), np.sin(q), qd], dtype=np.float64)


def main() -> None:
    model = mujoco.MjModel.from_xml_string(XML)
    data = mujoco.MjData(model)

    print("== 1. What the model declares (MjModel, fixed) ==")
    print(f"nsensor      (number of sensors)          {model.nsensor}")
    print(f"nsensordata  (length of data.sensordata)  {model.nsensordata}")
    print("id  name          type                  adr  dim  noise")
    for i in range(model.nsensor):
        name = mujoco.mj_id2name(model, mujoco.mjtObj.mjOBJ_SENSOR, i)
        typ = mujoco.mjtSensor(model.sensor_type[i]).name
        print(f"{i:<3} {name:<13} {typ:<21} {model.sensor_adr[i]:<4} {model.sensor_dim[i]:<4} {model.sensor_noise[i]:.2f}")

    print("\n== 2. Hanging at rest: qpos = 0, qvel = 0, ctrl = 0, after mj_forward ==")
    mujoco.mj_forward(model, data)
    print_sensordata(data)

    print("\n== 3. Stale values: sensordata only changes in mj_forward / mj_step ==")
    data.qpos[0] = 0.7
    print(f"set qpos[0] = 0.7, no mj_forward:  angle sensor = {data.sensor('angle').data[0]:.5f} rad  (stale)")
    mujoco.mj_forward(model, data)
    print(f"after mj_forward:                  angle sensor = {data.sensor('angle').data[0]:.5f} rad")
    print_sensordata(data)
    print(f"qpos[0]           = {data.qpos[0]:.5f} rad   (same as sensordata[0])")
    print(f"site_xpos['tip']  = {np.array2string(data.site('tip').xpos, precision=5)} m world (same as sensordata[3:6])")
    print(f"qacc[0]           = {data.qacc[0]:.5f} rad/s^2")
    data.ctrl[0] = 0.5
    mujoco.mj_forward(model, data)
    print(f"ctrl = 0.5: motor_force sensor = {data.sensor('motor_force').data[0]:.3f} (= actuator_force, before gear),"
          f" qfrc_actuator = {data.qfrc_actuator[0]:.1f} N m at the hinge (gear 100)")
    data.ctrl[0] = 0.0
    mujoco.mj_forward(model, data)
    mujoco.mj_step(model, data)
    print("after one mj_step:")
    print(f"  qpos[0] = {data.qpos[0]:.5f} rad, angle sensor = {data.sensor('angle').data[0]:.5f} rad"
          "  (sensors describe the START of the step)")

    print("\n== 4. Accelerometer at the bottom of the swing: qpos = 0, qvel = 2.0 rad/s ==")
    mujoco.mj_resetData(model, data)
    data.qvel[0] = 2.0
    mujoco.mj_forward(model, data)
    print(f"tip_acc (tip site frame) = {np.array2string(data.sensor('tip_acc').data, precision=5)} m/s^2")
    print(f"expected z: g + qvel^2 * L = 9.81 + 2.0^2 * 1.0 = {9.81 + 2.0**2:.5f}")

    print("\n== 5. Noise: MuJoCo stores it, it does not add it ==")
    print(f"model.sensor_noise = {model.sensor_noise}  (standard deviations)")
    mujoco.mj_resetData(model, data)
    reads = []
    for _ in range(3):
        mujoco.mj_forward(model, data)
        reads.append(data.sensor("tip_acc").data[2])
    print(f"tip_acc z, three mj_forward calls at rest: {[f'{r:.5f}' for r in reads]}  (identical: no noise added)")
    rng = np.random.default_rng(0)
    noise_std = np.repeat(model.sensor_noise, model.sensor_dim)  # one std per sensordata element
    noisy = data.sensordata + rng.normal(0.0, 1.0, model.nsensordata) * noise_std
    print("noisy copy, added in Python with seed 0 (index: clean -> noisy):")
    for i in range(model.nsensordata):
        print(f"  [{i}] {LABELS[i][0]:<13} {data.sensordata[i]: .4f} -> {noisy[i]: .4f}")

    print("\n== 6. Observation vector during a rollout: from rest, ctrl = 0.5 (50 N m at the hinge) for 0.5 s ==")
    mujoco.mj_resetData(model, data)
    data.ctrl[0] = 0.5
    mujoco.mj_forward(model, data)
    obs = observation(data)
    print(f"obs shape {obs.shape}, dtype {obs.dtype}")
    print(" time   angle(rad)   obs = [cos(angle), sin(angle), spin rad/s]    motor_force  qfrc_actuator")
    steps_per_print = 50  # 50 steps x 0.002 s = 0.1 s
    for k in range(6):
        if k > 0:
            for _ in range(steps_per_print):
                mujoco.mj_step(model, data)
            mujoco.mj_forward(model, data)  # refresh sensors for the current state
        obs = observation(data)
        print(f" {data.time:.3f}  {data.sensor('angle').data[0]: .5f}    "
              f"[{obs[0]: .5f}, {obs[1]: .5f}, {obs[2]: .5f}]     {data.sensor('motor_force').data[0]:.2f}         {data.qfrc_actuator[0]:.1f} N m")

    print("\n== 7. Why cos and sin: angles that wrap ==")
    for q in (0.0, 2 * np.pi, np.pi, -np.pi):
        c, sn = np.round(np.cos(q), 5) + 0.0, np.round(np.sin(q), 5) + 0.0  # + 0.0 turns -0.0 into 0.0
        print(f"angle {q: .5f} rad -> cos {c: .5f}, sin {sn: .5f}")
    obs4 = np.concatenate([obs, data.sensor("motor_force").data])
    print(f"optional 4th element, motor_force (already in -1..1): obs4 = {np.array2string(obs4, precision=5)}")


if __name__ == "__main__":
    main()
