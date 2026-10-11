# Lesson 15: Learning a Policy

The interactive lesson is in `lessons/index.html` (Lesson 15). These are the written notes.

**Goal:** let a learning algorithm (SAC) play the lesson 9 pendulum game until it writes its own controller, read its learning curve, compare it with the PD controller, and see that it learns exactly what the reward pays for, loopholes included.

## Run

```bash
cd /Users/mohwit/Desktop/mujoco
.venv/bin/python lessons/15-learning-a-policy/15_train.py              # slow part, about 7 min on CPU
.venv/bin/python lessons/15-learning-a-policy/15_learning_a_policy.py  # a few seconds
.venv/bin/python lessons/15-learning-a-policy/15_make_page_data.py     # copies recorded rollouts into the page fragment
```

`15_train.py` imports `PendulumHoldEnv` from lesson 9 (it does not copy it), trains SAC with fixed seeds, evaluates at checkpoints 0, 1k, 2k, 5k, 10k, 20k env steps on seeds 0-4 with exploration off, and writes `results/curves.json` plus the trained policy network `results/sac_base_seed0_actor.pt` (0.3 MB). The lesson script reloads that network, replays it in MuJoCo and checks it reproduces the stored return (-24.253, same: True).

## 1. Plain words

Training a dog with treats: it learns what you reward, not what you meant. Reinforcement learning (RL) is trial and error with a score. We write no controller, only the reward.

## 2. RL words, mapped onto lesson 9

| RL word | In this lesson |
|---|---|
| environment | `PendulumHoldEnv` (lesson 9): obs `[cos q, sin q, qvel]` shape (3,), action ctrl shape (1,) in -1..1 (100 x ctrl N*m) |
| policy (actor) | neural network 3 -> 256 -> 256 -> 1 action (tanh to -1..1), 67,330 numbers |
| critic | 2 networks, 4 inputs (obs + action) -> 256 -> 256 -> 1, 67,329 numbers each: a learned guess of the future score |
| exploration | SAC adds randomness while training; evaluation uses `deterministic=True` |
| replay buffer | diary of past steps, room for 1,000,000; batches of 256; first 100 steps random |
| discount gamma | 0.99 per env step (0.02 s) |

Discount with real numbers: a reward 100 steps (2.0 s) away counts 0.99^100 = 0.3660. The look-ahead is 1/(1-0.99) = 100 steps = 2.0 s. The holding cost -0.0084 per step, 100 steps away, is worth -0.00307 now.

## 3. Learning curve (lesson 9 reward, mean return over seeds 0-4)

| env steps | episodes | train seed 0 | seed 1 | seed 2 | mean |
|---|---|---|---|---|---|
| 0 | 0 | -223.841 | -202.278 | -405.000 | -277.039 |
| 1,000 | 4 | -227.747 | -208.121 | -219.635 | -218.501 |
| 2,000 | 8 | -134.030 | -122.514 | -119.223 | -125.256 |
| 5,000 | 20 | -30.825 | -28.827 | -29.053 | -29.568 |
| 10,000 | 40 | -24.605 | -24.568 | -25.112 | -24.762 |
| 20,000 | 80 | -24.253 | -24.420 | -24.095 | -24.256 |

References on the same seeds: zero -276.234, random -311.319, PD + gravity -36.622. One 20,000-step run took about 41 s on this machine (Apple M5, CPU, one thread); all ten training runs plus evaluations took 432 s.

Untrained SAC is not random noise: it is a fixed arbitrary rule. Depending on its starting weights it scored better than random (-223.841, -202.278) or worse (-405.000).

## 4. SAC against PD + gravity

SAC wins the score, -24.253 against -36.622, on all five seeds and on unseen seeds 100-104 (-19.854 against -29.169). Most of the gap comes from the first second (-21.419 against -31.412): PD + gravity overshoots (2.8 swings through the band 1.0 +- 0.05 rad per episode), SAC arrives without overshoot (0.0) and with smoother commands (mean step-to-step change 0.0070 against 0.0138).

But PD + gravity ends exactly at 1.0000 rad while SAC holds at 0.9186 rad (that sag wastes 1.14 points over the last 4 s). PD needs no training and changes target with one number; SAC only knows target 1.0. On a simple, well-modelled task a hand-written controller is nearly as good and far simpler. RL earns its keep where you cannot write the controller, such as the grasp in Capstone 2.

## 5. Reward design, one change at a time (20,000 steps, train seed 0)

| variant | reward per step | own return | PD+grav, same reward | lesson 9 return | mean abs action | final error (rad) |
|---|---|---|---|---|---|---|
| lesson 9 | -(err^2 + 0.1 qvel^2 + 0.01 a^2) | -24.253 | -36.622 | -24.253 | 0.809 | 0.081 |
| no action term | -(err^2 + 0.1 qvel^2) | -22.362 | -34.615 | -24.079 | 0.813 | 0.073 |
| action x100 | -(err^2 + 0.1 qvel^2 + 1.0 a^2) | -140.010 | -235.351 | -78.400 | 0.489 | 0.507 |
| sparse | +1 if abs(err) < 0.05 rad | 225.400 | 167.800 | -26.554 | 0.894 | 0.023 |

* Removing the action term changed almost nothing (no jitter appeared): the term was tiny (0.0084 per step at the target).
* Action term x100: the cheapest hold angle under that reward is 0.4993 rad (script search); the learner ends seed 0 at 0.493 rad. It found the trade-off we wrote.
* Sparse: own return 0, 0, 0, 3.4, 198.4, 225.4 at the checkpoints. Nothing to learn from until exploration stumbles into the band.
* Only compare returns under the same reward. Use a fixed yardstick (the lesson 9 return) to compare behaviour.

## 6. Reward hacking (real, reproducible)

Reward "approach" = `max(0, |err_prev| - |err|)`: pays for getting closer, moving away is free. SAC learned to swing back and forth through the target (seed 0 between -1.312 and 1.370 rad) with all three training seeds: own return 8.785 / 10.522 / 9.925 against PD + gravity's 1.572 (an honest go-and-stay policy can earn about the mean start error, 1.012). Scored with the lesson 9 reward: -811.976 / -1394.211 / -1014.966.

Fix "signed" = `|err_prev| - |err|`: a swing away and back nets zero. Retrained: no swings, own return 0.853, lesson 9 return -28.634, final error 0.159 rad (less precise than the lesson 9 reward).

Capstone 2 tells two more true stories from the warehouse grasp env: touch-sensor grasp bonus collected by pressing on the shelf, and an episode-ending success bonus that taught the robot to hover below the finish line.

## 7. Where this goes

Capstone 2's `WarehouseGraspEnv` trains the same SAC on a 19-number chassis-frame observation and a 4-number action (move the grasp point, open or close), while a classical controller drives the base and analytic IK moves the arm.

## Checkpoint questions

1. gamma 0.99, step 0.02 s: worth of a reward 1.0 s away? (0.99^50 = 0.6050)
2. Why evaluate with `deterministic=True`? (with noise: -26.981 instead of -24.253)
3. New target 0.5 rad: what changes for PD + gravity, and for SAC?
4. The action x100 learner holds at 0.49 rad. Broken? (No: cheapest point is 0.4993 rad.)
5. Spot the loophole in "+1 for every step the tip moves up".
