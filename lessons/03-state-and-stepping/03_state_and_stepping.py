"""Lesson 03: inspect MuJoCo state before and after stepping."""

import mujoco

XML = r"""
<mujoco model="state_demo">
  <option timestep="0.002" gravity="0 0 -9.81"/>
  <worldbody>
    <body name="pendulum" pos="0 0 1">
      <joint name="hinge" type="hinge" axis="0 1 0"/>
      <geom type="capsule" fromto="0 0 0 0 0 -1" size="0.08"/>
    </body>
  </worldbody>
</mujoco>
"""


def show(label: str, data: mujoco.MjData) -> None:
    print(
        f"{label:<18} time={data.time:.3f} "
        f"qpos={data.qpos[0]: .5f} qvel={data.qvel[0]: .5f} "
        f"qacc={data.qacc[0]: .5f}"
    )


def main() -> None:
    model = mujoco.MjModel.from_xml_string(XML)
    data = mujoco.MjData(model)

    data.qpos[0] = 0.7
    data.qvel[0] = 0.0
    mujoco.mj_forward(model, data)
    show("after forward", data)

    for step in range(1, 11):
        mujoco.mj_step(model, data)
        show(f"step {step}", data)

    mujoco.mj_resetData(model, data)
    show("after reset", data)
    print(f"model timestep     {model.opt.timestep:.3f}")
    print(f"model nq / nv       {model.nq} / {model.nv}")


if __name__ == "__main__":
    main()
