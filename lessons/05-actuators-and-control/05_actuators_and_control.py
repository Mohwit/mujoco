"""Lesson 05: add an actuator to the course pendulum, set ctrl, predict the torque.

Three versions of the same pendulum differ in ONE thing: the actuator.
  motor    : torque = gear * ctrl                    (a hand pushing with a set strength)
  position : torque = kp * (ctrl - qpos)             (a spring pulling toward a target angle)
  velocity : torque = kv * (ctrl - qvel)             (cruise control for spin speed)
"""

import numpy as np
import mujoco

BASE_XML = r"""
<mujoco model="actuator_demo">
  <option timestep="0.002" gravity="0 0 -9.81"/>
  <worldbody>
    <body name="pendulum" pos="0 0 1">
      <joint name="hinge" type="hinge" axis="0 1 0"/>
      <geom type="capsule" fromto="0 0 0 0 0 -1" size="0.08"/>
    </body>
  </worldbody>
  <actuator>
    {actuator}
  </actuator>
</mujoco>
"""

# gear 100 + ctrlrange -1..1: ctrl is a "throttle" from -100 % to +100 %, max 100 N m.
MOTOR = '<motor name="motor" joint="hinge" gear="100" ctrlrange="-1 1" ctrllimited="true"/>'
# ctrl is a target angle; a quarter turn either side is plenty for this demo.
POSITION = '<position name="servo" joint="hinge" kp="500" ctrlrange="-1.57 1.57" ctrllimited="true"/>'
# ctrl is a target spin speed in rad/s.
VELOCITY = '<velocity name="cruise" joint="hinge" kv="200" ctrlrange="-2 2" ctrllimited="true"/>'


def build(actuator: str) -> tuple[mujoco.MjModel, mujoco.MjData]:
    model = mujoco.MjModel.from_xml_string(BASE_XML.format(actuator=actuator))
    return model, mujoco.MjData(model)


def header(title: str) -> None:
    print(f"\n=== {title} ===")


def main() -> None:
    model, data = build(MOTOR)
    mujoco.mj_forward(model, data)

    header("1. The pendulum (read from MuJoCo, not typed in)")
    bid = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_BODY, "pendulum")
    m = model.body_mass[bid]
    d = -model.body_ipos[bid][2]  # CoM is straight below the hinge in the body frame
    g = -model.opt.gravity[2]
    M = np.zeros((model.nv, model.nv))
    mujoco.mj_fullM(model, data, M)
    I = M[0, 0]
    mgd = m * g * d
    print(f"body_mass[{bid}]           (pendulum mass, kg):            {m:.5f}")
    print(f"body_ipos[{bid}]           (CoM in body frame, m):         {model.body_ipos[bid]}")
    print(f"CoM distance d            (hinge to CoM, m):              {d:.3f}")
    print(f"body_inertia[{bid}]        (spin about own CoM x,y,z, kg m^2): {model.body_inertia[bid]}")
    print(f"m*d^2                     (parallel-axis part, kg m^2):   {m * d * d:.5f}")
    print(f"I = mass matrix [0,0]     (inertia about hinge, kg m^2):  {I:.5f}")
    print(f"m*g*d                     (max gravity torque, N m):      {mgd:.5f}")

    header("2. The motor actuator: names, ids, arrays")
    aid = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_ACTUATOR, "motor")
    print(f"model.nu                  (number of actuators):          {model.nu}")
    print(f"mj_name2id(ACTUATOR,'motor') (actuator id):               {aid}")
    print(f"actuator_trnid[{aid},0]      (joint id it drives):           {model.actuator_trnid[aid, 0]}")
    print(f"actuator_gear[{aid},0]       (torque per unit ctrl, N m):    {model.actuator_gear[aid, 0]:.1f}")
    print(f"actuator_ctrlrange[{aid}]    (allowed ctrl):                 {model.actuator_ctrlrange[aid]}")
    print(f"model.actuator('motor').gear[0]  (same, by name):           {model.actuator('motor').gear[0]:.1f}")
    print(f"data.ctrl                 (shape {data.ctrl.shape}, the command):     {data.ctrl}")

    header("3. Predict the torque: ctrl = 0.5, gear = 100")
    data.ctrl[0] = 0.5
    mujoco.mj_forward(model, data)
    print(f"data.ctrl[0]              (command, unitless):            {data.ctrl[0]:.3f}")
    print(f"data.actuator_force[0]    (actuator output, before gear): {data.actuator_force[0]:.3f}")
    print(f"data.qfrc_actuator[0]     (torque on hinge, N m):         {data.qfrc_actuator[0]:.3f}")

    header("4. Which ctrl holds the pendulum still at 0.5 rad?")
    theta = 0.5
    tau_g = mgd * np.sin(theta)
    ctrl_hold = tau_g / model.actuator_gear[aid, 0]
    print(f"gravity torque m*g*d*sin(0.5) (N m):                      {tau_g:.5f}")
    print(f"ctrl needed = gravity torque / gear:                      {ctrl_hold:.5f}")
    mujoco.mj_resetData(model, data)
    data.qpos[0] = theta
    data.ctrl[0] = ctrl_hold
    mujoco.mj_forward(model, data)
    print(f"qacc at start             (rad/s^2, ~0 means balanced):   {data.qacc[0]:.2e}")
    for _ in range(500):
        mujoco.mj_step(model, data)
    print(f"after 1 s: time={data.time:.3f} s  qpos={data.qpos[0]:.5f} rad  qvel={data.qvel[0]:.2e} rad/s")
    data.ctrl[0] = 0.0
    for _ in range(500):
        mujoco.mj_step(model, data)
    print(f"ctrl=0 for 1 more s: qpos={data.qpos[0]:.5f} rad  (no muscle: it swings)")

    header("5. Clamping by ctrlrange")
    for c in (0.8, 1.5, -3.0):
        mujoco.mj_resetData(model, data)
        data.ctrl[0] = c
        mujoco.mj_forward(model, data)
        print(f"data.ctrl[0]={c:+.2f}  actuator_force={data.actuator_force[0]:+.3f}  "
              f"qfrc_actuator={data.qfrc_actuator[0]:+.3f} N m")

    header("6. Model is fixed, data is mutable")
    mujoco.mj_resetData(model, data)
    data.ctrl[0] = 0.5
    data.actuator_force[0] = 999.0
    mujoco.mj_step(model, data)
    print(f"wrote actuator_force=999, after mj_step it is:            {data.actuator_force[0]:.3f}  (recomputed)")
    model.actuator_gear[aid, 0] = 200.0
    mujoco.mj_forward(model, data)
    print(f"model.actuator_gear set to 200, qfrc_actuator:            {data.qfrc_actuator[0]:.3f} N m (machine redesigned)")
    mujoco.mj_resetData(model, data)
    print(f"after mj_resetData: ctrl={data.ctrl[0]:.1f}, gear still {model.actuator_gear[aid, 0]:.0f}  (reset touches data only)")
    model.actuator_gear[aid, 0] = 100.0

    header("7. Three actuator types, one formula each (gain and bias from the model)")
    for name, act, q, v in (("motor", MOTOR, 0.2, 0.3), ("servo", POSITION, 0.2, 0.3), ("cruise", VELOCITY, 0.2, 0.3)):
        mdl, dat = build(act)
        dat.qpos[0], dat.qvel[0], dat.ctrl[0] = q, v, 0.5
        mujoco.mj_forward(mdl, dat)
        print(f"{name:<7} gainprm[0]={mdl.actuator_gainprm[0, 0]:6.1f}  biasprm[0:3]={mdl.actuator_biasprm[0, :3]}  "
              f"ctrl=0.5 qpos={q} qvel={v} -> actuator_force={dat.actuator_force[0]:8.3f}  qfrc={dat.qfrc_actuator[0]:8.3f} N m")

    header("8. Same start (hanging still), ctrl = 0.5 for each, angle after 1 s")
    for name, act in (("motor", MOTOR), ("servo", POSITION), ("cruise", VELOCITY)):
        mdl, dat = build(act)
        dat.ctrl[0] = 0.5
        for _ in range(500):
            mujoco.mj_step(mdl, dat)
        print(f"{name:<7} t={dat.time:.3f} s  qpos={dat.qpos[0]: .5f} rad  qvel={dat.qvel[0]: .5f} rad/s  "
              f"actuator_force={dat.actuator_force[0]: .3f}")
    print(f"integrator: {mujoco.mjtIntegrator(model.opt.integrator).name}")


if __name__ == "__main__":
    main()
