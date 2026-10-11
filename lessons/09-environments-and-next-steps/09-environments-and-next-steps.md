# Lesson 09: Environments and next steps

> **Interactive lesson:** open `lessons/index.html` and choose **Lesson 9** (Environments and Next Steps). It covers the same ideas in plain language, with a game loop you can drive one step at a time, an episode sandbox, and optional maths.

## Goal

Wrap the course pendulum in a Gymnasium environment with `reset`, `step`, an observation and a reward. Then know where to go next: reinforcement learning, trajectory optimisation, or sim-to-real.

## Mental model

An environment is a video-game console around the simulation.

| Gym word | Console analogy | MuJoCo calls inside |
|---|---|---|
| `env.reset(seed)` | new game | `mj_resetData`, set `qpos[0]` from `self.np_random`, `mj_forward` |
| `env.step(action)` | one button press | `data.ctrl[0] = action`, then `mj_step` x `frame_skip` |
| observation | the screen | `[cos qpos[0], sin qpos[0], qvel[0]]`, shape (3,) |
| reward | the score for this moment | computed from `qpos`, `qvel`, `ctrl` |
| return | the final score | sum of the rewards in one episode |
| terminated | game over | never true here (nothing can crash) |
| truncated | time up | true after 250 env steps (5.0 s) |
| policy | the player | zero, random, PD, PD + gravity, or a neural network |

The player never touches `MjData`. It only sends an action and receives an observation and a reward. That fixed doorway is what lets you swap in any controller or learning algorithm.

## The game

The course pendulum (lesson 3) with the lesson 5 motor:

```xml
<motor name="motor" joint="hinge" gear="100" ctrlrange="-1 1" ctrllimited="true"/>
```

- Start: hanging, tilted by a random angle in [-0.5, 0.5] rad (seeded).
- Task: lift to **1.0 rad** (the lesson 8 target) and hold.
- Holding 1.0 rad needs `m g d sin(1.0) = 91.84 N m`, which is ctrl 0.9184. The motor's maximum is 100 N m, so it can, with 8% to spare.
- `action_space = Box(-1, 1, (1,))` (ctrl). Torque = 100 x ctrl N m.
- `observation_space = Box(3,)`: cos q, sin q in [-1, 1], qvel unbounded (rad/s).
- Reward per step: `-((qpos - 1.0)^2 + 0.1 * qvel^2 + 0.01 * ctrl^2)`.
  - angle error: being away from the target costs the most;
  - speed: swinging fast through the target is not "holding";
  - effort: pushing costs a little (holding costs 0.0084 per step).

## Two clocks

`model.opt.timestep = 0.002 s` is the physics tick. `frame_skip = 10` holds each action for 10 ticks, so one env step is 0.020 s: a control rate of 50 actions per second. An episode of 250 env steps is 2,500 `mj_step` calls, 5.0 s.

Returns only compare fairly at the same frame_skip and episode length. With frame_skip 20 and 125 steps (still 5 s), the zero player's return for seed 0 halves from -256.548 to -128.276. Multiply each reward by the step time to compare: both give -5.131.

## Runnable example

```bash
uv run --python .venv/bin/python lessons/09-environments-and-next-steps/09_environments_and_next_steps.py
```

Key output (MuJoCo 3.15.0):

```text
action_space (ctrl, -1..1)              : Box(-1.0, 1.0, (1,), float32)
torque to HOLD the target (m g d sin)   : 91.84 N*m  = ctrl 0.9184
check_env: passed, 2 warning(s)          # both: observation bound is infinite (advice only)

step 1: time=0.020 s  ctrl= 1.00000 ( 100.00 N*m)  qpos= 0.13934 rad  qvel= 0.21589 rad/s  reward=-0.75540  return=-0.75540
...
step 5: time=0.100 s  ctrl= 1.00000 ( 100.00 N*m)  qpos= 0.19143 rad  qvel= 1.05595 rad/s  reward=-0.77530  return=-3.81846  truncated=True

seed  start angle (rad)    zero     random       PD   PD+grav
   0   0.13696          -256.548  -364.607   -28.020   -24.566
   1   0.01182          -250.117  -274.149   -34.684   -31.981
   2  -0.23839          -264.999  -326.232   -51.432   -50.051
   3  -0.41435          -296.021  -318.863   -66.057   -65.403
   4   0.44306          -313.484  -272.742   -16.571   -11.111
mean                     -276.234  -311.319   -39.353   -36.622
```

- PD is lesson 8's `tau = 400 (1.0 - q) - 40 qvel` N m, sent as `ctrl = clip(tau / 100, -1, 1)`. It sags: it rests at 0.80359 rad with ctrl 0.78566.
- PD + gravity adds `m g d sin(q)` and rests at 0.99999 rad with ctrl 0.91848.
- Random is worse than doing nothing on average (not on every seed).

Gymnasium's own `InvertedPendulum-v5` is registered in this venv, but `gym.make` fails with `ModuleNotFoundError: No module named 'imageio'`. It needs the `gymnasium[mujoco]` extra, which is not installed. Nothing was installed for this lesson.

## Helper scripts

- `09_check_browser_physics.py` runs the page's JavaScript pendulum (from `_fragments/script.js`) and the real env side by side: zero, PD and PD + gravity, seeds 0-4, frame_skip 10 and 20, every step. Largest differences: qpos 8.2e-8 rad, return 3.0e-6. PASS at 4 decimals.
- `09_try_sac.py` (optional, about 1 minute on a laptop CPU) trains Stable-Baselines3 SAC on this env for 20,000 env steps (80 episodes). One run here: mean return over seeds 0-4 SAC -24.253, PD + gravity -36.622, zero -276.234. Exact numbers can vary between machines.

## Where next

| Road | What it is | Pick it when |
|---|---|---|
| Reinforcement learning (Stable-Baselines3 PPO, SAC) | An algorithm plays the env many times and tunes a neural-network policy to raise the return. | You cannot write a good controller by hand (for example a contact-rich grasp). |
| GPU scale (MJX, MuJoCo Playground) | MJX is MuJoCo in JAX, so many copies step in parallel on a GPU or TPU; Playground is a set of environments and training recipes built on it. | Training needs many millions of steps. (Not installed in this venv.) |
| Trajectory optimisation and MPC (MuJoCo MPC) | Use the model to plan the best actions a short time ahead, apply the first, replan every step. | The model is accurate and you want behaviour without training. |
| Sim-to-real: system identification, domain randomisation | Measure the real robot and tune the XML to match; vary uncertain values on every reset so the policy copes with a range. | You will run the controller on hardware. |

Capstone 2 (`lessons/16-capstone-warehouse-robot/`, lesson 16 in `lessons/index.html`) is this pattern at full size: `WarehouseGraspEnv` resets with random parking and box errors (domain randomisation), observes 19 numbers in the chassis frame, acts with 4 numbers in [-1, 1], runs 25 physics steps per action (20 Hz) for 100 steps, and is trained with SAC by `train_grasp_rl.py`. Its notes describe two real cases of reward hacking.

## Exercise: predict first

1. `timestep = 0.002`, `frame_skip = 10`. How many actions per simulated second? How many `mj_step` calls per 250-step episode?
2. You call `env.reset(seed=3)` twice. Same start angle? Why?
3. Which MuJoCo calls does `reset` make, in which order, and why the last one?
4. The PD player sees `obs = [0.99064, 0.13653, 0.0]`. What ctrl does it send?
5. Why is `terminated` always False here while `truncated` becomes True?

## Checkpoint

Report:

- the control rate and the number of physics steps per episode;
- which player had the best mean return, and why plain PD scores slightly worse than PD + gravity;
- what happens to a return when frame_skip doubles at the same episode time;
- which "where next" road you would take for your own project, and why.
