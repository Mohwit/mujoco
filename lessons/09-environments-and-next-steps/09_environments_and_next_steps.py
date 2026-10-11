"""Lesson 09: wrap the course pendulum in a Gymnasium environment.

The game: a pendulum starts hanging at a random small angle. The motor must lift it
to the target angle (1.0 rad, the lesson 8 target) and hold it there still.

Run (headless, no viewer needed):
    uv run --python .venv/bin/python lessons/09-environments-and-next-steps/09_environments_and_next_steps.py
"""

import re
import warnings

import gymnasium as gym
import mujoco
import numpy as np
from gymnasium import spaces

# Course pendulum (lesson 3) plus the course motor (lesson 5): torque = gear * ctrl,
# gear 100, ctrl clamped to -1..1, so the motor gives at most 100 N*m.
XML = r"""
<mujoco model="pendulum_env">
  <option timestep="0.002" gravity="0 0 -9.81"/>
  <worldbody>
    <body name="pendulum" pos="0 0 1">
      <joint name="hinge" type="hinge" axis="0 1 0"/>
      <geom type="capsule" fromto="0 0 0 0 0 -1" size="0.08"/>
    </body>
  </worldbody>
  <actuator>
    <motor name="motor" joint="hinge" gear="100" ctrlrange="-1 1" ctrllimited="true"/>
  </actuator>
</mujoco>
"""


class PendulumHoldEnv(gym.Env):
    """Gymnasium wrapper around the course pendulum.

    observation (3,) float32: [cos q, sin q, qvel]  (q = hinge angle, rad; qvel in rad/s)
    action      (1,) float32: motor command ctrl, -1..1 (written to data.ctrl[0]; torque = 100 * ctrl N*m)
    reward      -(angle_error**2 + 0.1 * qvel**2 + 0.01 * action**2), once per env step
    terminated  never (there is no "game over" state in this task)
    truncated   True after max_steps env steps (the clock ran out)
    """

    metadata = {"render_modes": []}

    def __init__(self, frame_skip=10, max_steps=250, target=1.0, start_range=0.5):
        self.model = mujoco.MjModel.from_xml_string(XML)   # the blueprint, fixed
        self.data = mujoco.MjData(self.model)               # the state, changes every step
        self.frame_skip = frame_skip       # physics steps per env step
        self.max_steps = max_steps         # env steps per episode
        self.target = target               # target hinge angle, rad
        self.start_range = start_range     # reset angle drawn from [-start_range, +start_range] rad
        lo, hi = self.model.actuator_ctrlrange[0]
        self.action_space = spaces.Box(low=lo, high=hi, shape=(1,), dtype=np.float32)
        high = np.array([1.0, 1.0, np.inf], dtype=np.float32)
        self.observation_space = spaces.Box(low=-high, high=high, dtype=np.float32)
        self.steps = 0

    def _obs(self):
        q, qd = self.data.qpos[0], self.data.qvel[0]
        return np.array([np.cos(q), np.sin(q), qd], dtype=np.float32)

    def reset(self, *, seed=None, options=None):
        super().reset(seed=seed)                        # seeds self.np_random
        mujoco.mj_resetData(self.model, self.data)      # time 0, qpos = qpos0, qvel 0, ctrl 0
        self.data.qpos[0] = self.np_random.uniform(-self.start_range, self.start_range)
        mujoco.mj_forward(self.model, self.data)        # recompute derived quantities
        self.steps = 0
        return self._obs(), {"angle": float(self.data.qpos[0])}

    def step(self, action):
        a = float(np.clip(action[0], self.action_space.low[0], self.action_space.high[0]))
        self.data.ctrl[0] = a                           # motor command; torque = gear * ctrl = 100 * a N*m
        for _ in range(self.frame_skip):                # hold the action for frame_skip ticks
            mujoco.mj_step(self.model, self.data)
        self.steps += 1
        err = self.data.qpos[0] - self.target           # rad
        qd = self.data.qvel[0]                           # rad/s
        reward = -(err**2 + 0.1 * qd**2 + 0.01 * a**2)
        terminated = False
        truncated = self.steps >= self.max_steps
        info = {"time": self.data.time, "angle": float(self.data.qpos[0])}
        return self._obs(), float(reward), terminated, truncated, info


# ---------- three policies: obs -> action ----------
KP, KD = 400.0, 40.0    # PD gains from lesson 8: N*m per rad, N*m per (rad/s)
GEAR = 100.0            # N*m per unit ctrl (lesson 5 motor)
_m = mujoco.MjModel.from_xml_string(XML)
MGD = _m.body_mass[1] * -_m.opt.gravity[2] * -_m.body_ipos[1][2]   # m*g*d, N*m (d = 0.5 m)


def zero_policy(obs, rng):
    return np.array([0.0], dtype=np.float32)


def random_policy(obs, rng):
    return rng.sample()     # rng is the env's action_space, seeded


def pd_policy(obs, rng, target=1.0, gravity_comp=False):
    q = np.arctan2(obs[1], obs[0])      # recover the angle from [cos q, sin q]
    tau = KP * (target - q) - KD * obs[2]          # N*m, lesson 8
    if gravity_comp:
        tau += MGD * np.sin(q)                     # N*m, cancel gravity's pull
    return np.array([np.clip(tau / GEAR, -1.0, 1.0)], dtype=np.float32)   # N*m -> ctrl


def pd_grav_policy(obs, rng):
    return pd_policy(obs, rng, gravity_comp=True)


POLICIES = {"zero": zero_policy, "random": random_policy, "PD": pd_policy, "PD+grav": pd_grav_policy}


def run_episode(env, policy, seed):
    obs, info = env.reset(seed=seed)
    env.action_space.seed(seed)
    ret, n = 0.0, 0
    while True:
        obs, r, term, trunc, info = env.step(policy(obs, env.action_space))
        ret += r
        n += 1
        if term or trunc:
            return ret, n, info


def main():
    env = PendulumHoldEnv()
    m = env.model
    h = m.opt.timestep
    print("== 1. The game and its clock ==")
    print(f"model.opt.timestep (physics tick)       : {h:.3f} s")
    print(f"frame_skip (physics ticks per env step) : {env.frame_skip}")
    print(f"env step (control period)               : {h * env.frame_skip:.3f} s  -> {1 / (h * env.frame_skip):.0f} actions per second")
    print(f"max_steps per episode                   : {env.max_steps}  -> {env.max_steps * h * env.frame_skip:.1f} s of simulated time")
    print(f"observation_space                       : {env.observation_space}")
    print(f"action_space (ctrl, -1..1)              : {env.action_space}")
    print(f"motor gear (N*m per unit ctrl)          : {m.actuator_gear[0, 0]:.0f}  -> max torque {m.actuator_gear[0, 0] * m.actuator_ctrlrange[0, 1]:.0f} N*m")
    mujoco.mj_forward(m, env.data)
    full_m = np.zeros((m.nv, m.nv))
    mujoco.mj_fullM(m, env.data, full_m)     # joint-space inertia, 1x1 for one hinge
    print(f"pendulum mass, inertia about hinge      : {m.body_mass[1]:.2f} kg, I = {full_m[0, 0]:.4f} kg*m^2")
    print(f"target angle                            : {env.target:.1f} rad")
    print(f"torque to HOLD the target (m g d sin)   : {MGD * np.sin(env.target):.2f} N*m  = ctrl {MGD * np.sin(env.target) / GEAR:.4f}")

    print("\n== 2. Gymnasium's own checker ==")
    from gymnasium.utils.env_checker import check_env
    with warnings.catch_warnings(record=True) as w:
        warnings.simplefilter("always")
        check_env(PendulumHoldEnv(), skip_render_check=True)
    print(f"check_env: passed, {len(w)} warning(s)")
    for x in w:
        msg = re.sub(r"\x1b\[[0-9;]*m", "", str(x.message)).splitlines()[0]
        print("   warning:", msg[:100])

    print("\n== 3. One short episode by hand (seed 0, PD policy, max_steps=5) ==")
    short = PendulumHoldEnv(max_steps=5)
    obs, info = short.reset(seed=0)
    print(f"reset(seed=0): time={short.data.time:.3f} s  qpos={short.data.qpos[0]:.5f} rad  obs=[cos q, sin q, qvel]={np.round(obs, 5).tolist()}")
    ret = 0.0
    while True:
        a = pd_policy(obs, None)
        obs, r, term, trunc, info = short.step(a)
        ret += r
        q, qd = short.data.qpos[0], short.data.qvel[0]
        print(f"step {short.steps}: time={short.data.time:.3f} s  ctrl={a[0]: .5f} ({a[0] * GEAR: .2f} N*m)  qpos={q: .5f} rad  qvel={qd: .5f} rad/s"
              f"  reward={r: .5f}  return={ret: .5f}  terminated={term} truncated={trunc}")
        print(f"        reward terms: (qpos-target)^2={(q - short.target)**2:.5f}  0.1*qvel^2={0.1 * qd**2:.5f}  0.01*ctrl^2={0.01 * a[0]**2:.5f}")
        if term or trunc:
            break

    print("\n== 4. Returns over 5 seeds (frame_skip 10, 250 steps = 5.0 s) ==")
    seeds = range(5)
    print("seed  start angle (rad)    zero     random       PD   PD+grav")
    totals = {k: [] for k in POLICIES}
    for s in seeds:
        row = []
        for name, pol in POLICIES.items():
            r, n, info = run_episode(env, pol, s)
            totals[name].append(r)
            row.append(r)
        _, info0 = env.reset(seed=s)
        print(f"{s:>4}  {info0['angle']: .5f}          " + "  ".join(f"{x:8.3f}" for x in row))
    print("mean                     " + "  ".join(f"{np.mean(totals[k]):8.3f}" for k in POLICIES))
    print("(PD: tau = 400*(1.0 - q) - 40*qvel N*m; PD+grav adds m*g*d*sin(q); ctrl = clip(tau/100, -1, 1))")

    print("\n== 5. Same 5 s, frame_skip 20 (so only 125 env steps) ==")
    env20 = PendulumHoldEnv(frame_skip=20, max_steps=125)
    print("seed   zero (fs10)  zero (fs20)   PD+grav (fs10)  PD+grav (fs20)")
    for s in seeds:
        z10, _, _ = run_episode(env, zero_policy, s)
        z20, _, _ = run_episode(env20, zero_policy, s)
        p10, _, _ = run_episode(env, pd_grav_policy, s)
        p20, _, i20 = run_episode(env20, pd_grav_policy, s)
        print(f"{s:>4}  {z10:10.3f}  {z20:10.3f}     {p10:10.3f}      {p20:10.3f}")
    print(f"env20 final time after one episode: {i20['time']:.3f} s")

    print("\n== 6. Final state, seed 0, after 5.0 s ==")
    for name, pol in POLICIES.items():
        r, n, info = run_episode(env, pol, 0)
        print(f"{name:>7}: qpos={env.data.qpos[0]: .5f} rad  qvel={env.data.qvel[0]: .5f} rad/s  ctrl={env.data.ctrl[0]: .5f}  time={info['time']:.3f} s  steps={n}")

    print("\n== 7. Gymnasium's built-in MuJoCo environments in this venv ==")
    print(f"'InvertedPendulum-v5' registered: {'InvertedPendulum-v5' in gym.registry}")
    try:
        e = gym.make("InvertedPendulum-v5")
        print("gym.make('InvertedPendulum-v5'): OK", e.observation_space, e.action_space)
    except Exception as ex:  # report truthfully; do not install anything
        print(f"gym.make('InvertedPendulum-v5'): fails with {type(ex).__name__}: {ex}")


if __name__ == "__main__":
    main()
