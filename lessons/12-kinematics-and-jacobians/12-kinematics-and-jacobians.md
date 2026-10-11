# Lesson 12: Kinematics and Jacobians

> **The interactive lesson is in `lessons/index.html` (Lesson 12).** It has the chain-of-frames picture, the Jacobian arrows, the singular-pose ellipse, analytic IK with both elbow solutions, a step-by-step damped least squares solver and the kinematics sandbox. These notes are the short written version.

## Goal

Go from joint angles to the hand position (forward kinematics), learn how fast the hand moves per joint speed (the Jacobian, `mj_jacSite`), and go back from a hand position to joint angles (inverse kinematics), with the two-link formula and with damped least squares.

Prerequisites: Lesson 4 (names, `xpos`, `site_xpos`, this arm), Lesson 3 (`mj_forward`, semi-implicit Euler), Lesson 5 (position actuators).

## Model

The Lesson 4 two-link arm, unchanged; the loose box and the cameras are dropped because kinematics never looks at them.

```xml
<body name="upper_arm" pos="0 0 1">
  <joint name="shoulder" type="hinge" axis="0 1 0"/>
  <geom type="capsule" fromto="0 0 0 0.5 0 0" size="0.04"/>
  <body name="forearm" pos="0.5 0 0">
    <joint name="elbow" type="hinge" axis="0 1 0"/>
    <geom type="capsule" fromto="0 0 0 0.4 0 0" size="0.03"/>
    <geom name="weight" type="box" pos="0.4 0 0" size="0.05 0.05 0.05"/>
    <site name="hand" pos="0.4 0 0" size="0.02"/>
  </body>
</body>
```

Shoulder at world (0, 0, 1) m, L1 = 0.5 m, L2 = 0.4 m. `qpos[0]` = shoulder, `qpos[1]` = elbow (rad); nq = nv = 2.

**Sign convention.** Both hinges turn about +y, which points into the screen in the side view. A positive turn about +y moves +x towards -z, so positive angles tip the arm down (clockwise on screen). MuJoCo: `qpos = (+0.1, 0)` puts the hand at z = 0.9101 m; `(-0.1, 0)` at z = 1.0899 m.

## Mental model

```text
forward:  qpos --mj_forward--> site_xpos           one answer
velocity: qvel --jacp (3 x nv)--> hand velocity    v = jacp @ qvel (world, m/s)
inverse:  target --law of cosines (2 links) or DLS (any arm)--> qpos   0, 1, 2 or many answers
```

- Forward kinematics is a chain of frames: hand = (0, 1) + R(q1)[L1, 0] + R(q1 + q2)[L2, 0], with R(a)[L, 0] = (L cos a, -L sin a) in world (x, z).
- `mj_jacSite(model, data, jacp, jacr, sid)` fills two 3 x nv arrays (world frame): hand velocity per rad/s of each joint (m/rad) and hand-frame spin per rad/s. Call `mj_forward` first.
- Column j of a hinge is axis x (hand - joint): its length is the joint-to-hand distance.
- Singular pose: the arm straight (elbow = 0). det J = -L1 L2 sin q2 = 0; the hand cannot move along the arm.
- Analytic IK: cos q2 = (r² - L1² - L2²) / (2 L1 L2); q1 = atan2(w, u) - atan2(L2 sin q2, L1 + L2 cos q2), with u = x out and w = 1 - z down. |cos q2| > 1 means out of reach (r > 0.9 m or r < 0.1 m).
- Damped least squares: repeat dq = Jᵀ (J Jᵀ + λ² I)⁻¹ (target - site_xpos), q += dq, `mj_forward`. Works for any arm.
- To follow a moving target, run IK each control step and send the angles as `ctrl` to position actuators (Lesson 5).

## Runnable example

```bash
.venv/bin/python lessons/12-kinematics-and-jacobians/12_kinematics_and_jacobians.py
```

Check that the web page's kinematics matches MuJoCo (needs `node`):

```bash
.venv/bin/python lessons/12-kinematics-and-jacobians/check_browser_match.py
```

It prints `worst difference: 1.5e-10 -> PASS (4 decimals)` (FK, 2 x 2 Jacobian, IK and six full DLS runs).

## Key numbers (from the script)

| Quantity | Value |
|---|---|
| hand at (0.5, 0.8) | (0.5458, 0, 0.3749) m, by hand and `site_xpos` agree to 1.1e-16 m |
| jacp at (0.5, 0.8), rows x, y, z | [[-0.6251, -0.3854], [0, 0], [-0.5458, -0.1070]] m/rad |
| jacr | [[0, 0], [1, 1], [0, 0]] |
| column lengths | shoulder 0.8299 m/rad (= shoulder-to-hand distance), elbow 0.4000 m/rad |
| J qvel, qvel = (0.1, 0.2) | (-0.1396, 0, -0.0760) m/s; finite difference agrees to 7.2e-11 |
| one `mj_step` from qvel (0.1, 0.2) | hand moved at (-0.1388, 0, -0.0935) m/s = J times the NEW qvel (0.147747, 0.120372) |
| straight arm, shoulder 0.1 rad/s | (0, 0, -0.0900) m/s |
| det J at elbow 0.8 / 0.1 / 0.01 / 0 | -0.14347 / -0.01997 / -0.00200 / 0 (m/rad)²; condition number 5.7 / 48.5 / 485 / infinite |
| IK for the cup (0.6, 0, 0.6) | r = 0.7211 m, cos q2 = 0.2750; elbow up (0.0255, 1.2922), elbow down (1.1505, -1.2922) rad |
| reach test at shoulder height | r = 0.6: 2 solutions; 0.9: 1; 1.0: 0 (too far); 0.1: 1 (folded); 0.05: 0 (too close) |
| DLS, cup, start (0, 0), λ = 0.05 | 7 iterations to < 1e-6 m, ends at the elbow-up answer |
| DLS from (0, 0.001), λ = 0 / 0.01 / 0.05 / 0.2 / 0.5 | first step 1476.27 / 0.73 / 0.41 / 0.39 / 0.32 rad; 9 / 5 / 7 / 18 / 72 iterations |
| DLS, unreachable (1.2, 0, 1.0), λ = 0.2 | settles straight, error 0.3000 m (best possible) |
| same DLS on a 3-link arm | 9 iterations, q = (-0.2237, 1.2315, 1.0924) rad |

## Common mistakes

- Reading `site_xpos` or calling `mj_jacSite` after changing `qpos` without `mj_forward`: you get the old pose.
- Forgetting the sign: positive angles tip this arm down.
- Plain J⁻¹ e (λ = 0) near a singular pose: steps of hundreds of radians. Use λ of a few centimetres.
- With the full 3-row `jacp` of a planar arm, λ = 0 fails outright: row y is always zero, so J Jᵀ is singular.

## Exercise: predict first

1. Where is the hand for qpos = (1.0, -0.5)?
2. What shape is `jacp` here, and what does row z, column 0 mean?
3. The elbow alone turns at 0.1 rad/s. Does the hand speed depend on the pose?
4. A target is 0.95 m from the shoulder. What do analytic IK and DLS (λ = 0.2) do?
5. Your IK steps are hundreds of radians. Cause and fix?

## Checkpoint

Report your answers. Then, in the sandbox, drag the target outside the ring in analytic and in DLS mode, and explain the difference.
