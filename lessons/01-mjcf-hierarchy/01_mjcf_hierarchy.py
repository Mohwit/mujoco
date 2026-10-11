"""Lesson 1: inspect the hierarchy of a small MuJoCo model."""

import mujoco


XML = """
<mujoco model="hierarchy_demo">
  <option gravity="0 0 -9.81" />
  <worldbody>
    <body name="pendulum" pos="0 0 1">
      <joint name="hinge" type="hinge" axis="0 1 0" />
      <geom name="rod" type="capsule"
            fromto="0 0 0 0 0 -1" size="0.05" />
      <body name="bob" pos="0 0 -1">
        <geom name="weight" type="sphere" size="0.12" />
      </body>
    </body>
  </worldbody>
</mujoco>
"""


def name(model: mujoco.MjModel, object_type: mujoco.mjtObj, index: int) -> str:
    return mujoco.mj_id2name(model, object_type, index) or "<unnamed>"


def main() -> None:
    model = mujoco.MjModel.from_xml_string(XML)
    data = mujoco.MjData(model)

    print("Model counts:")
    print(f"  bodies: {model.nbody}")
    print(f"  joints: {model.njnt}")
    print(f"  geoms:  {model.ngeom}")
    print(f"  qpos:   {model.nq}")
    print(f"  qvel:   {model.nv}")

    print("\nBodies:")
    for body_id in range(model.nbody):
        print(f"  {body_id}: {name(model, mujoco.mjtObj.mjOBJ_BODY, body_id)}")

    print("\nGeoms:")
    for geom_id in range(model.ngeom):
        print(f"  {geom_id}: {name(model, mujoco.mjtObj.mjOBJ_GEOM, geom_id)}")

    print("\nJoint state before stepping:", data.qpos.copy())
    data.qpos[0] = 0.7
    mujoco.mj_forward(model, data)
    mujoco.mj_step(model, data)
    print("Joint state after one step: ", data.qpos.copy())


if __name__ == "__main__":
    main()
