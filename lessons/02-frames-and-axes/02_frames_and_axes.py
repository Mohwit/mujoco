"""Lesson 02: inspect local frames, world positions, and joint axes."""

import mujoco
import numpy as np

XML = r"""
<mujoco model="frames_and_axes">
  <option gravity="0 0 -9.81"/>
  <worldbody>
    <body name="base" pos="0 0 1">
      <geom name="base_geom" type="box" size="0.18 0.18 0.08"/>
      <body name="link" pos="0 0 0.6">
        <joint name="hinge" type="hinge" axis="0 1 0"/>
        <geom name="link_geom" type="capsule" fromto="0 0 0 0 0 -0.8" size="0.08"/>
      </body>
    </body>
  </worldbody>
</mujoco>
"""


def row(label: str, value: np.ndarray) -> None:
    print(f"{label:<24} {np.array2string(value, precision=3, suppress_small=True)}")


def main() -> None:
    model = mujoco.MjModel.from_xml_string(XML)
    data = mujoco.MjData(model)

    base_id = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_BODY, "base")
    link_id = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_BODY, "link")
    joint_id = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_JOINT, "hinge")
    geom_id = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_GEOM, "link_geom")

    print("Local inputs")
    row("base body pos:", model.body_pos[base_id])
    row("link body pos:", model.body_pos[link_id])
    row("hinge axis:", model.jnt_axis[joint_id])
    print()

    for angle in (0.0, 0.7):
        data.qpos[model.jnt_qposadr[joint_id]] = angle
        mujoco.mj_forward(model, data)
        print(f"State with hinge angle = {angle:.2f} rad")
        row("link world position:", data.xpos[link_id])
        link_world_z = data.xmat[link_id].reshape(3, 3)[:, 2]
        row("link world axis z:", link_world_z)
        row("geom world position:", data.geom_xpos[geom_id])
        print()

    print("Counts")
    print(f"bodies: {model.nbody}")
    print(f"joints: {model.njnt}")
    print(f"qpos:   {model.nq}")
    print(f"qvel:   {model.nv}")


if __name__ == "__main__":
    main()
