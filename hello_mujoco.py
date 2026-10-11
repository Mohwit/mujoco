import time

import mujoco
import mujoco.viewer


XML = """
<mujoco model="pendulum">
  <option timestep="0.002" gravity="0 0 -9.81"/>
  <worldbody>
    <body name="pendulum" pos="0 0 1">
      <joint name="hinge" type="hinge" axis="0 1 0"/>
      <geom name="rod" type="capsule" fromto="0 0 0 0 0 -1" size="0.05" rgba="0.2 0.6 0.9 1"/>
      <body name="bob" pos="0 0 -1">
        <geom type="sphere" size="0.12" rgba="0.9 0.3 0.2 1"/>
      </body>
    </body>
  </worldbody>
</mujoco>
"""


def main() -> None:
    model = mujoco.MjModel.from_xml_string(XML)
    data = mujoco.MjData(model)

    # Give the pendulum an initial angle so the motion is visible.
    data.qpos[0] = 0.7
    mujoco.mj_forward(model, data)

    # This opens MuJoCo's native viewer. Close the window to exit.
    with mujoco.viewer.launch_passive(model, data) as viewer:
        while viewer.is_running():
            mujoco.mj_step(model, data)
            viewer.sync()
            time.sleep(model.opt.timestep)


if __name__ == "__main__":
    main()
