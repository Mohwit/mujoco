"""Lesson 12: forward kinematics, the Jacobian, and inverse kinematics (analytic and damped least squares).

Run:  .venv/bin/python lessons/12-kinematics-and-jacobians/12_kinematics_and_jacobians.py
Headless: prints only.
"""

import math

import mujoco
import numpy as np

# The lesson 4 two-link arm, unchanged, in the world x-z plane (side view: x right, z up, +y into the screen).
#   shoulder at world (0, 0, 1)   upper arm 0.50 m (L1)   forearm 0.40 m to the `hand` site (L2)
# The loose box and the two cameras of lesson 4 are dropped: kinematics never looks at them.
XML = r"""
<mujoco model="two_link_arm">
  <option timestep="0.002" gravity="0 0 -9.81"/>
  <worldbody>
    <light name="sun" pos="0.5 -1.5 3" dir="0 0.4 -1"/>
    <geom name="floor" type="plane" size="2 2 0.05" rgba="0.75 0.78 0.76 1"/>
    <body name="upper_arm" pos="0 0 1">
      <joint name="shoulder" type="hinge" axis="0 1 0"/>
      <geom name="upper_geom" type="capsule" fromto="0 0 0 0.5 0 0" size="0.04" rgba="0.1 0.7 0.6 1"/>
      <body name="forearm" pos="0.5 0 0">
        <joint name="elbow" type="hinge" axis="0 1 0"/>
        <geom name="forearm_geom" type="capsule" fromto="0 0 0 0.4 0 0" size="0.03" rgba="0.9 0.5 0.2 1"/>
        <geom name="weight" type="box" pos="0.4 0 0" size="0.05 0.05 0.05" rgba="0.5 0.4 0.8 1"/>
        <site name="hand" pos="0.4 0 0" size="0.02"/>
      </body>
    </body>
  </worldbody>
</mujoco>
"""

# Same arm plus a 0.2 m third link (a "wrist"): used only to show that numerical IK needs no new formula.
XML3 = XML.replace(
    '<site name="hand" pos="0.4 0 0" size="0.02"/>',
    '<body name="wrist" pos="0.4 0 0"><joint name="wrist" type="hinge" axis="0 1 0"/>'
    '<geom type="capsule" fromto="0 0 0 0.2 0 0" size="0.02"/><site name="hand" pos="0.2 0 0" size="0.02"/></body>',
)

L1, L2 = 0.5, 0.4
BASE = np.array([0.0, 0.0, 1.0])
TARGET = np.array([0.6, 0.0, 0.6])   # a "cup" 0.4 m below the shoulder and 0.6 m out


def v3(a) -> str:
    return "(" + ", ".join(f"{x: .4f}" for x in a) + ")"


def fk_by_hand(q1: float, q2: float) -> np.ndarray:
    """Chain of frames: rotate by q1, walk L1; rotate by q1 + q2 more, walk L2. Rotation about +y sends +x to -z."""
    return BASE + np.array([L1 * math.cos(q1), 0, -L1 * math.sin(q1)]) + np.array([L2 * math.cos(q1 + q2), 0, -L2 * math.sin(q1 + q2)])


def hand_at(model, data, sid, q) -> np.ndarray:
    data.qpos[:] = q
    mujoco.mj_forward(model, data)
    return data.site_xpos[sid].copy()


def jac(model, data, sid):
    jacp = np.zeros((3, model.nv))
    jacr = np.zeros((3, model.nv))
    mujoco.mj_jacSite(model, data, jacp, jacr, sid)
    return jacp, jacr


def analytic_ik(target):
    """Two-link IK in the x-z plane (law of cosines). Returns [] (unreachable) or a list of (name, q1, q2)."""
    u = target[0] - BASE[0]           # metres out from the shoulder (world x)
    w = -(target[2] - BASE[2])        # metres DOWN from the shoulder (positive angles tip the arm down)
    r2 = u * u + w * w
    c2 = (r2 - L1 * L1 - L2 * L2) / (2 * L1 * L2)
    if abs(c2) > 1 + 1e-12:
        return []
    c2 = max(-1.0, min(1.0, c2))
    sols = []
    for sgn in (+1, -1):
        q2 = sgn * math.acos(c2)
        q1 = math.atan2(w, u) - math.atan2(L2 * math.sin(q2), L1 + L2 * math.cos(q2))
        sols.append((q1, q2))
        if c2 in (1.0, -1.0):          # straight or folded: both branches are the same pose
            break
    return sols


def dls_ik(model, data, sid, target, q0, lam, iters=50, tol=1e-6, use=(0, 1, 2)):
    """Damped least squares: dq = J^T (J J^T + lam^2 I)^-1 e, repeated. Works for any arm: only needs J and e.

    use: which world rows (x=0, y=1, z=2) to match. All three is the general case. For this planar arm,
    row y of J is always zero, so lam = 0 needs use=(0, 2), or J J^T has a zero row and cannot be inverted.
    """
    q = np.array(q0, dtype=float)
    use = list(use)
    rows = []
    for k in range(iters + 1):
        p = hand_at(model, data, sid, q)
        e = (target - p)[use]                            # world frame, metres
        jacp = jac(model, data, sid)[0][use]
        rows.append((k, q.copy(), p.copy(), np.linalg.norm(e)))
        if np.linalg.norm(e) < tol:
            break
        A = jacp @ jacp.T + lam * lam * np.eye(len(use))
        dq = jacp.T @ np.linalg.solve(A, e)               # radians
        q = q + dq
    return rows


def main() -> None:
    np.set_printoptions(precision=4, suppress=True)
    model = mujoco.MjModel.from_xml_string(XML)
    data = mujoco.MjData(model)
    sid = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_SITE, "hand")
    print(f"model: nq = {model.nq}, nv = {model.nv}, site 'hand' id = {sid}, L1 = {L1} m, L2 = {L2} m, shoulder at {v3(BASE)} m (world)")

    print("\n== 1. Sign convention: a positive hinge angle about +y turns +x towards -z ==")
    for q in [(0.0, 0.0), (0.1, 0.0), (-0.1, 0.0)]:
        print(f"qpos = ({q[0]:+.1f}, {q[1]:+.1f}) rad -> hand site_xpos = {v3(hand_at(model, data, sid, q))} m (world)")
    print("so +shoulder tips the arm DOWN (clockwise in the side view, because +y points into the screen)")

    print("\n== 2. Forward kinematics as a chain of frames ==")
    print("hand = base + R(q1) [L1, 0] + R(q1 + q2) [L2, 0]   with R(a) [L, 0] = (L cos a, -L sin a) in (x, z)")
    for q in [(0.0, 0.0), (0.5, 0.8), (1.0, -0.5), (-0.4, 1.2), (0.3, 2.0)]:
        mj = hand_at(model, data, sid, q)
        bh = fk_by_hand(*q)
        elbow = data.xpos[model.body("forearm").id]
        print(f"q = ({q[0]:+.2f}, {q[1]:+.2f}) rad: elbow xpos = {v3(elbow)}  hand by hand = {v3(bh)}  "
              f"MuJoCo site_xpos = {v3(mj)}  |diff| = {np.abs(bh - mj).max():.1e} m")
    q1, q2 = 0.5, 0.8
    print(f"pose (0.5, 0.8) step by step: R(0.5)[0.5,0] = ({L1*math.cos(q1):.4f}, {-L1*math.sin(q1):.4f}),"
          f" R(1.3)[0.4,0] = ({L2*math.cos(q1+q2):.4f}, {-L2*math.sin(q1+q2):.4f}),"
          f" sum + (0, 1) = ({L1*math.cos(q1)+L2*math.cos(q1+q2):.4f}, {1-L1*math.sin(q1)-L2*math.sin(q1+q2):.4f})")

    print("\n== 3. The Jacobian at pose (0.5, 0.8) ==")
    q = np.array([0.5, 0.8])
    p0 = hand_at(model, data, sid, q)
    jacp, jacr = jac(model, data, sid)
    print(f"jacp (3 x nv = 3 x 2, rows world x, y, z; columns shoulder, elbow; m/rad):\n{jacp}")
    print(f"jacr (3 x 2, rad/s of the forearm per rad/s of each joint):\n{jacr}")
    s1, c1, s12, c12 = math.sin(q1), math.cos(q1), math.sin(q1 + q2), math.cos(q1 + q2)
    Jf = np.array([[-L1 * s1 - L2 * s12, -L2 * s12], [-L1 * c1 - L2 * c12, -L2 * c12]])
    print(f"planar 2x2 by formula [[-L1 s1 - L2 s12, -L2 s12], [-L1 c1 - L2 c12, -L2 c12]] =\n{Jf}")
    print(f"max |formula - jacp rows x,z| = {np.abs(Jf - jacp[[0, 2]]).max():.1e}")
    elbow = data.xpos[model.body("forearm").id].copy()
    r1, r2 = p0 - BASE, p0 - elbow
    print(f"column = (joint axis) x (hand - joint): shoulder col = (0,1,0) x {v3(r1)} = {v3(np.cross([0, 1, 0], r1))};"
          f" elbow col = (0,1,0) x {v3(r2)} = {v3(np.cross([0, 1, 0], r2))}")
    print(f"column lengths: shoulder {np.linalg.norm(jacp[:, 0]):.4f} m/rad (= shoulder-to-hand distance {np.linalg.norm(r1):.4f} m),"
          f" elbow {np.linalg.norm(jacp[:, 1]):.4f} m/rad (= L2)")
    for qv in [(0.1, 0.0), (0.0, 0.1), (0.1, 0.2)]:
        qv = np.array(qv)
        v = jacp @ qv
        eps = 1e-6
        fd = (hand_at(model, data, sid, q + eps * qv) - hand_at(model, data, sid, q - eps * qv)) / (2 * eps)
        print(f"qvel = ({qv[0]:.1f}, {qv[1]:.1f}) rad/s -> v_hand = J qvel = {v3(v)} m/s, speed {np.linalg.norm(v):.4f} m/s;"
              f" finite difference = {v3(fd)}  |diff| = {np.abs(v - fd).max():.1e}")
    # straight arm: 0.1 rad/s at the shoulder
    hand_at(model, data, sid, (0.0, 0.0))
    js, _ = jac(model, data, sid)
    print(f"straight arm, shoulder 0.1 rad/s: v_hand = {v3(js @ [0.1, 0])} m/s (0.9 m x 0.1 rad/s); elbow 0.1 rad/s: {v3(js @ [0, 0.1])} m/s")

    # check against a real mj_step
    data.qpos[:] = q
    data.qvel[:] = [0.1, 0.2]
    mujoco.mj_forward(model, data)
    pb = data.site_xpos[sid].copy()
    vb = jac(model, data, sid)[0] @ data.qvel
    mujoco.mj_step(model, data)
    mujoco.mj_forward(model, data)
    pa = data.site_xpos[sid].copy()
    va = jac(model, data, sid)[0] @ data.qvel
    h = model.opt.timestep
    res = np.zeros(6)
    mujoco.mj_objectVelocity(model, data, mujoco.mjtObj.mjOBJ_SITE, sid, res, 0)
    print(f"one mj_step (h = {h} s, gravity on) from qvel = (0.1, 0.2): (site_xpos after - before) / h = {v3((pa - pb) / h)} m/s")
    print(f"   J qvel before step = {v3(vb)}   J qvel after step = {v3(va)}   mj_objectVelocity (linear, world) after = {v3(res[3:])} m/s")
    print(f"   qvel after step = ({data.qvel[0]:.6f}, {data.qvel[1]:.6f}) rad/s (gravity changed it during the step)")

    print("\n== 4. The singular pose: arm straight (elbow = 0) ==")
    for e in [0.8, 0.3, 0.1, 0.01, 0.0]:
        hand_at(model, data, sid, (0.0, e))
        J2 = jac(model, data, sid)[0][[0, 2]]
        sv = np.linalg.svd(J2, compute_uv=False)
        cond = sv[0] / sv[1] if sv[1] > 1e-15 else float("inf")
        print(f"q = (0, {e:.2f}): J2 = [[{J2[0,0]: .4f}, {J2[0,1]: .4f}], [{J2[1,0]: .4f}, {J2[1,1]: .4f}]]  det = {np.linalg.det(J2): .5f} (m/rad)^2"
              f" (-L1 L2 sin q2 = {-L1*L2*math.sin(e): .5f})  smallest stretch {sv[1]:.5f} m/rad  condition number {cond:.1f}")
    print("at q = (0, 0) row x is all zeros: no joint speed moves the hand along the arm (x); both columns point along z")
    hand_at(model, data, sid, (0.0, 0.01))
    J2 = jac(model, data, sid)[0][[0, 2]]
    need = np.linalg.solve(J2, [0.001, 0.0])
    print(f"at q = (0, 0.01): to move the hand outward (+x) at 1 mm/s the joints need qvel = ({need[0]:.4f}, {need[1]:.4f}) rad/s, size {np.linalg.norm(need):.4f} rad/s")

    print("\n== 5. Analytic inverse kinematics (law of cosines) ==")
    tgt = TARGET
    u, w = tgt[0] - BASE[0], -(tgt[2] - BASE[2])
    r = math.hypot(u, w)
    c2 = (r * r - L1 * L1 - L2 * L2) / (2 * L1 * L2)
    print(f"target {v3(tgt)} m (world): out u = {u:.4f} m, down w = {w:.4f} m, distance r = {r:.4f} m")
    print(f"cos q2 = (r^2 - L1^2 - L2^2) / (2 L1 L2) = ({r*r:.4f} - 0.25 - 0.16) / 0.4 = {c2:.4f} -> q2 = +/-{math.acos(c2):.4f} rad")
    print(f"atan2(w, u) = {math.atan2(w, u):.4f} rad (direction to the target, below horizontal)")
    sols = analytic_ik(tgt)
    elz = [hand_at(model, data, sid, qs) is None or data.xpos[model.body("forearm").id][2] for qs in sols]
    for (q1s, q2s), ez in zip(sols, elz):
        p = hand_at(model, data, sid, (q1s, q2s))
        el = data.xpos[model.body("forearm").id]
        corr = math.atan2(L2 * math.sin(q2s), L1 + L2 * math.cos(q2s))
        print(f"  q2 = {q2s:+.4f}: correction atan2(L2 sin q2, L1 + L2 cos q2) = {corr:+.4f}, q1 = {math.atan2(w, u):.4f} - ({corr:+.4f}) = {q1s:+.4f} rad;"
              f" elbow at {v3(el)} -> {'elbow UP' if ez == max(elz) else 'elbow DOWN'};"
              f" check site_xpos = {v3(p)}  error {np.linalg.norm(p - tgt):.1e} m")
    print("reach test (targets at shoulder height, distance r from the shoulder):")
    for rr in [0.6, 0.9, 1.0, 0.05, 0.1]:
        t = BASE + np.array([rr, 0, 0])
        sols = analytic_ik(t)
        why = "too far (r > L1 + L2 = 0.9)" if rr > 0.9 else ("too close (r < L1 - L2 = 0.1)" if rr < 0.1 else "")
        desc = "; ".join(f"({a:+.4f}, {b:+.4f}) -> hand {v3(hand_at(model, data, sid, (a, b)))}" for a, b in sols)
        print(f"  r = {rr:.2f} m: {len(sols)} solution(s) {desc} {why}")

    print("\n== 6. Numerical IK: damped least squares (DLS) ==")
    print("dq = J^T (J J^T + lambda^2 I)^-1 e,  e = target - hand (world, m),  q <- q + dq, repeat")
    rows = dls_ik(model, data, sid, TARGET, (0.0, 0.0), 0.05)
    print(f"target {v3(TARGET)} m, start q = (0, 0) (straight arm, singular!), lambda = 0.05 m")
    print(" iter   q1 (rad)  q2 (rad)   hand x (m)  hand z (m)   |error| (m)")
    for k, qq, p, err in rows:
        print(f" {k:4d}   {qq[0]: .4f}   {qq[1]: .4f}    {p[0]: .4f}     {p[2]: .4f}     {err:.6f}")
    print(f"converged to q = ({rows[-1][1][0]:.4f}, {rows[-1][1][1]:.4f}) = the elbow-up analytic answer")

    try:
        dls_ik(model, data, sid, TARGET, (0.3, 0.6), 0.0, iters=1)
    except np.linalg.LinAlgError as exc:
        print(f"\nlambda = 0 with all 3 rows: LinAlgError '{exc}' (row y of J is always 0, so J J^T is singular)")
    print("lambda near the singular pose: start q = (0, 0.001), same target, rows x and z only (lambda = 0 is plain J^-1 e)")
    print(" lambda   first |dq| (rad)   iterations to 1e-6 m   final q")
    for lam in [0.0, 0.01, 0.05, 0.2, 0.5]:
        rws = dls_ik(model, data, sid, TARGET, (0.0, 0.001), lam, iters=200, use=(0, 2))
        dq1 = np.linalg.norm(rws[1][1] - rws[0][1]) if len(rws) > 1 else 0
        ok = rws[-1][3] < 1e-6
        print(f"  {lam:4.2f}   {dq1:12.4f}       {len(rws) - 1 if ok else 'not in 200':>10}            ({rws[-1][1][0]: .4f}, {rws[-1][1][1]: .4f})")
        if lam == 0.0:
            qf = rws[-1][1]
            wrap = [math.remainder(a, 2 * math.pi) for a in qf]
            turns = [round((a - b) / (2 * math.pi)) for a, b in zip(qf, wrap)]
            print(f"         (that is ({wrap[0]:.4f}, {wrap[1]:.4f}) plus {turns[0]} and {turns[1]} whole turns: right pose, absurd joint motion)")
            print(f"         (first step {dq1:.1f} rad = {dq1 / (2 * math.pi):.1f} full turns)")

    print("\nunreachable target (1.2, 0, 1.0) (1.2 m from the shoulder), start (0.3, 0.6): |error| (m) every 10 iterations")
    for lam in [0.05, 0.2]:
        rws = dls_ik(model, data, sid, np.array([1.2, 0, 1.0]), (0.3, 0.6), lam, iters=60, tol=0)
        errs = "  ".join(f"{r[3]:.4f}" for r in rws[::10])
        print(f"  lambda {lam:.2f}: {errs}   final q = ({rws[-1][1][0]: .4f}, {rws[-1][1][1]: .4f}) hand = {v3(rws[-1][2])}")
    print("  best possible: arm straight towards the target, error 1.2 - 0.9 = 0.3 m")
    rws = dls_ik(model, data, sid, np.array([0.95, 0, 1.0]), (0.3, 0.6), 0.2, iters=60, tol=0)
    print(f"  target (0.95, 0, 1.0), lambda 0.20, 60 iterations: q = ({rws[-1][1][0]: .4f}, {rws[-1][1][1]: .4f}), |error| = {rws[-1][3]:.4f} m;"
          f" analytic IK: {len(analytic_ik(np.array([0.95, 0, 1.0])))} solutions")

    print("\nsame DLS function on a THREE-link arm (wrist 0.2 m added), no new formula:")
    m3 = mujoco.MjModel.from_xml_string(XML3)
    d3 = mujoco.MjData(m3)
    s3 = mujoco.mj_name2id(m3, mujoco.mjtObj.mjOBJ_SITE, "hand")
    rws = dls_ik(m3, d3, s3, TARGET, (0.0, 0.0, 0.0), 0.05)
    print(f"  nv = {m3.nv}, jacp shape {jac(m3, d3, s3)[0].shape}; iterations {len(rws) - 1}; final q = {np.array2string(rws[-1][1], precision=4)}"
          f" rad; hand = {v3(rws[-1][2])}; |error| = {rws[-1][3]:.1e} m")


if __name__ == "__main__":
    main()
