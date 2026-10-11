// ======== LESSON 12: KINEMATICS AND JACOBIANS (shared core, sandbox 12, explainer) ========
(() => {
// CORE-BEGIN
// Same arm as 12_kinematics_and_jacobians.py (the lesson 4 two-link arm). World x-z plane, side view.
// Shoulder at world (0, 0, 1), L1 = 0.5 m, L2 = 0.4 m to the hand site. Hinges about +y:
// a positive angle turns +x towards -z, so positive angles tip the arm DOWN (clockwise on screen).
// Checked against MuJoCo 3.15.0 (mj_forward, mj_jacSite, the script's IK and DLS) by check_browser_match.py.
const L12 = { L1: 0.5, L2: 0.4, BX: 0, BZ: 1 };
function l12FK(q1, q2) {                                   // world (x, z) of elbow and hand, m
  const ex = L12.BX + L12.L1 * Math.cos(q1), ez = L12.BZ - L12.L1 * Math.sin(q1);
  return { elbow: [ex, ez], hand: [ex + L12.L2 * Math.cos(q1 + q2), ez - L12.L2 * Math.sin(q1 + q2)] };
}
function l12Jac(q1, q2) {                                  // rows world x and z of mj_jacSite's jacp, m/rad
  const s1 = Math.sin(q1), c1 = Math.cos(q1), s12 = Math.sin(q1 + q2), c12 = Math.cos(q1 + q2);
  return [[-L12.L1 * s1 - L12.L2 * s12, -L12.L2 * s12], [-L12.L1 * c1 - L12.L2 * c12, -L12.L2 * c12]];
}
function l12IK(x, z) {                                     // analytic two-link IK, law of cosines
  const u = x - L12.BX, w = -(z - L12.BZ), r = Math.hypot(u, w), L1 = L12.L1, L2 = L12.L2;
  let c2 = (r * r - L1 * L1 - L2 * L2) / (2 * L1 * L2);
  const out = { r, c2, sols: [], why: '' };
  if (Math.abs(c2) > 1 + 1e-12) { out.why = r > L1 + L2 ? 'too far' : 'too close'; return out; }
  c2 = Math.max(-1, Math.min(1, c2));
  for (const sg of [1, -1]) {
    const q2 = sg * Math.acos(c2), q1 = Math.atan2(w, u) - Math.atan2(L2 * Math.sin(q2), L1 + L2 * Math.cos(q2));
    out.sols.push([q1, q2]);
    if (c2 === 1 || c2 === -1) break;                      // straight or folded: one pose only
  }
  return out;
}
function l12DLS(q, tx, tz, lam) {                          // one step: dq = J^T (J J^T + lam^2 I)^-1 e
  const h = l12FK(q[0], q[1]).hand, e = [tx - h[0], tz - h[1]], J = l12Jac(q[0], q[1]);
  const a = J[0][0], b = J[0][1], c = J[1][0], d = J[1][1], l2 = lam * lam;
  const A11 = a * a + b * b + l2, A12 = a * c + b * d, A22 = c * c + d * d + l2, det = A11 * A22 - A12 * A12;
  if (Math.abs(det) < 1e-30) return { q: q.slice(), dq: [0, 0], err: Math.hypot(e[0], e[1]), singular: true };
  const y0 = (A22 * e[0] - A12 * e[1]) / det, y1 = (-A12 * e[0] + A11 * e[1]) / det;
  const dq = [a * y0 + c * y1, b * y0 + d * y1];
  return { q: [q[0] + dq[0], q[1] + dq[1]], dq, err: Math.hypot(e[0], e[1]), singular: false };
}
// CORE-END

const $ = id => document.getElementById(id);
const f4 = v => (Math.abs(v) < 5e-5 ? 0 : v).toFixed(4);
const p4 = v => { const s = f4(v); return s[0] === '-' ? s : ' ' + s; };
const errOf = (q, tx, tz) => { const h = l12FK(q[0], q[1]).hand; return Math.hypot(tx - h[0], tz - h[1]); };
const ROWHEAD = ' iter   q1 (rad)  q2 (rad)   hand x (m)  hand z (m)   |error| (m)';
const row = (k, q, tx, tz) => { const h = l12FK(q[0], q[1]).hand; return String(k).padStart(5) + '   ' + p4(q[0]) + '   ' + p4(q[1]) + '    ' + p4(h[0]) + '     ' + p4(h[1]) + '     ' + errOf(q, tx, tz).toFixed(6); };
function sv2(J) {                                          // singular values of a 2x2 (largest, smallest)
  const a = J[0][0], b = J[0][1], c = J[1][0], d = J[1][1];
  const p = a * a + b * b + c * c + d * d, det = a * d - b * c, disc = Math.sqrt(Math.max(0, p * p / 4 - det * det));
  return [Math.sqrt(p / 2 + disc), Math.sqrt(Math.max(0, p / 2 - disc))];
}

// ---------------- Sandbox 12 (SVG) ----------------
(() => {
  const svg = $('s12Svg'); if (!svg) return;
  const S = 180, X = x => 200 + S * x, Y = z => 405 - S * z;
  const st = { q: [0, 0], tx: 0.6, tz: 0.6, mode: 'ana', up: true, path: [], it: 0, timer: null };
  const tx = $('s12Tx'), tz = $('s12Tz'), lam = $('s12Lam');
  const setLine = (id, x1, z1, x2, z2) => { const e = $(id); e.setAttribute('x1', X(x1)); e.setAttribute('y1', Y(z1)); e.setAttribute('x2', X(x2)); e.setAttribute('y2', Y(z2)); };
  function stop() { if (st.timer) { clearTimeout(st.timer); st.timer = null; } $('s12Solve').textContent = 'Solve'; }
  function solveAnalytic() {
    const r = l12IK(st.tx, st.tz), u = st.tx, w = -(st.tz - 1);
    if (r.sols.length) { st.q = (st.up || r.sols.length === 1) ? r.sols[0].slice() : r.sols[1].slice(); return 'reachable: ' + r.sols.length + (r.sols.length === 1 ? ' solution' : ' solutions'); }
    st.q = [Math.atan2(w, u), r.why === 'too far' ? 0 : Math.PI];
    return 'unreachable, ' + r.why + (r.why === 'too far' ? ' (r > 0.9 m): arm points at it' : ' (r < 0.1 m): arm folds towards it');
  }
  function render(status) {
    const k = l12FK(st.q[0], st.q[1]), J = l12Jac(st.q[0], st.q[1]), h = k.hand;
    setLine('s12Up', 0, 1, k.elbow[0], k.elbow[1]); setLine('s12Fo', k.elbow[0], k.elbow[1], h[0], h[1]);
    $('s12Elbow').setAttribute('cx', X(k.elbow[0])); $('s12Elbow').setAttribute('cy', Y(k.elbow[1]));
    $('s12Hand').setAttribute('cx', X(h[0])); $('s12Hand').setAttribute('cy', Y(h[1]));
    $('s12Tgt').setAttribute('transform', `translate(${X(st.tx)},${Y(st.tz)})`);
    const show = $('s12Cols').checked;                    // arrow = hand velocity if that joint turns at 0.5 rad/s
    ['s12C1', 's12C2'].forEach((id, j) => { $(id).style.display = show ? '' : 'none'; setLine(id, h[0], h[1], h[0] + 0.5 * J[0][j], h[1] + 0.5 * J[1][j]); });
    $('s12Path').setAttribute('points', st.path.map(p => X(p[0]).toFixed(1) + ',' + Y(p[1]).toFixed(1)).join(' '));
    const e = errOf(st.q, st.tx, st.tz), det = J[0][0] * J[1][1] - J[0][1] * J[1][0];
    $('s12Q').textContent = f4(st.q[0]) + ', ' + f4(st.q[1]);
    $('s12H').textContent = f4(h[0]) + ', ' + f4(h[1]);
    $('s12E').textContent = e.toFixed(6);
    $('s12J').textContent = '[[' + p4(J[0][0]) + ', ' + p4(J[0][1]) + '], [' + p4(J[1][0]) + ', ' + p4(J[1][1]) + ']]';
    $('s12Det').textContent = (Math.abs(det) < 5e-6 ? 0 : det).toFixed(5);
    $('s12It').textContent = st.mode === 'dls' ? String(st.it) : '-';
    if (status !== undefined) $('s12Status').textContent = status;
    $('s12TxVal').textContent = st.tx.toFixed(2); $('s12TzVal').textContent = st.tz.toFixed(2); $('s12LamVal').textContent = (+lam.value).toFixed(2);
  }
  function targetChanged() {
    stop();
    if (st.mode === 'ana') { st.path = []; render(solveAnalytic()); }
    else { st.path = [l12FK(st.q[0], st.q[1]).hand]; st.it = 0; render('target moved: press Step or Solve'); }
  }
  function stepOnce() {
    const r = l12DLS(st.q, st.tx, st.tz, +lam.value);
    if (r.singular) { render('lambda = 0 at a singular pose: J J^T cannot be inverted'); return false; }
    st.q = r.q; st.it++; st.path.push(l12FK(st.q[0], st.q[1]).hand);
    const e = errOf(st.q, st.tx, st.tz);
    render(e < 1e-6 ? 'converged: error < 1e-6 m' : 'step size |dq| = ' + Math.hypot(r.dq[0], r.dq[1]).toFixed(4) + ' rad');
    return e >= 1e-6;
  }
  function setMode(m) {
    stop(); st.mode = m;
    document.querySelectorAll('[data-s12mode]').forEach(b => { const on = b.dataset.s12mode === m; b.classList.toggle('btn-primary', on); b.setAttribute('aria-pressed', on); });
    $('s12AnaCtl').style.display = m === 'ana' ? '' : 'none'; $('s12DlsCtl').style.display = m === 'dls' ? '' : 'none';
    if (m === 'dls') { st.it = 0; st.path = [l12FK(st.q[0], st.q[1]).hand]; render('DLS: press Step one iteration or Solve'); } else targetChanged();
  }
  document.querySelectorAll('[data-s12mode]').forEach(b => b.addEventListener('click', () => setMode(b.dataset.s12mode)));
  $('s12Elb').addEventListener('click', () => { st.up = !st.up; $('s12Elb').textContent = st.up ? 'Elbow up (q2 > 0)' : 'Elbow down (q2 < 0)'; $('s12Elb').setAttribute('aria-pressed', !st.up); targetChanged(); });
  $('s12Step').addEventListener('click', () => { stop(); stepOnce(); });
  $('s12Solve').addEventListener('click', () => {
    if (st.timer) { stop(); return; }
    if (RM) { let n = 0; while (n++ < 200 && stepOnce()); return; }
    $('s12Solve').textContent = 'Stop';
    const tick = () => { st.timer = null; if (stepOnce() && st.it < 200) { st.timer = setTimeout(tick, 220); $('s12Solve').textContent = 'Stop'; } else stop(); };
    tick();
  });
  $('s12Reset').addEventListener('click', () => { stop(); st.q = [0, 0]; st.it = 0; st.path = [l12FK(0, 0).hand]; render('arm reset to straight (0, 0): a singular pose'); });
  tx.addEventListener('input', () => { st.tx = +tx.value; targetChanged(); });
  tz.addEventListener('input', () => { st.tz = +tz.value; targetChanged(); });
  lam.addEventListener('input', () => render());
  $('s12Cols').addEventListener('change', () => render());
  // drag or click the target
  let drag = false;
  function fromPointer(ev) {
    const pt = svg.createSVGPoint(); pt.x = ev.clientX; pt.y = ev.clientY;
    const p = pt.matrixTransform(svg.getScreenCTM().inverse());
    st.tx = Math.max(-1.05, Math.min(1.05, Math.round((p.x - 200) / S * 100) / 100));
    st.tz = Math.max(0, Math.min(2, Math.round((405 - p.y) / S * 100) / 100));
    tx.value = st.tx; tz.value = st.tz; targetChanged();
  }
  svg.addEventListener('pointerdown', ev => { drag = true; svg.setPointerCapture(ev.pointerId); fromPointer(ev); });
  svg.addEventListener('pointermove', ev => { if (drag) fromPointer(ev); });
  svg.addEventListener('pointerup', () => { drag = false; });
  svg.addEventListener('pointercancel', () => { drag = false; });
  setMode('ana');
})();

// ======== LESSON 12 EXPLAINER ========
(() => {
  // Only redraw this lesson's canvases while it is on screen (hidden canvases have zero size).
  const addEventListener = (type, fn, opts) => window.addEventListener(type, e => { if (document.getElementById('lesson12').classList.contains('active')) fn(e); }, opts);

  // World (x, z) metres -> canvas pixels, same scale on both axes, z up.
  function view(g, x0, x1, z0, z1) {
    const s = Math.min(g.w / (x1 - x0), g.h / (z1 - z0)), ox = (g.w - s * (x1 - x0)) / 2, oz = (g.h - s * (z1 - z0)) / 2;
    return { s, X: x => ox + (x - x0) * s, Y: z => g.h - oz - (z - z0) * s };
  }
  function scene(c, V, o = {}) {
    if (o.ring) {                                           // reachable workspace: 0.1 m <= r <= 0.9 m from the shoulder
      c.fillStyle = css('--teal'); c.globalAlpha = 0.07; c.beginPath(); c.arc(V.X(0), V.Y(1), 0.9 * V.s, 0, 7); c.arc(V.X(0), V.Y(1), 0.1 * V.s, 0, 7, true); c.fill(); c.globalAlpha = 1;
      c.setLineDash([4, 4]); c.strokeStyle = css('--dim'); c.lineWidth = 1; c.beginPath(); c.arc(V.X(0), V.Y(1), 0.9 * V.s, 0, 7); c.stroke(); c.beginPath(); c.arc(V.X(0), V.Y(1), 0.1 * V.s, 0, 7); c.stroke(); c.setLineDash([]);
    }
    c.strokeStyle = css('--grid'); c.lineWidth = 1; c.beginPath(); c.moveTo(0, V.Y(0)); c.lineTo(c.canvas.clientWidth, V.Y(0)); c.stroke();
    c.setLineDash([3, 4]); c.strokeStyle = css('--dim'); c.beginPath(); c.moveTo(V.X(0), V.Y(0)); c.lineTo(V.X(0), V.Y(1)); c.stroke(); c.setLineDash([]);
    arrow(c, V.X(0), V.Y(0), V.X(0.25), V.Y(0), css('--x'), 1.5, 6); arrow(c, V.X(0), V.Y(0), V.X(0), V.Y(0.25), css('--z'), 1.5, 6);
    text(c, 'x', V.X(0.28), V.Y(0), css('--x'), 11, 'left'); text(c, 'z', V.X(0), V.Y(0.3), css('--z'), 11, 'center');
    if (!o.noLabel) text(c, 'world origin', V.X(-0.04), V.Y(0) - 8, css('--dim'), 10, 'right', 500);
  }
  function arm(c, V, q, o = {}) {
    const k = l12FK(q[0], q[1]), w = o.thin ? 0.6 : 1;
    c.globalAlpha = o.alpha || 1; c.lineCap = 'round';
    c.strokeStyle = o.col1 || css('--teal'); c.lineWidth = 0.08 * V.s * w; c.beginPath(); c.moveTo(V.X(0), V.Y(1)); c.lineTo(V.X(k.elbow[0]), V.Y(k.elbow[1])); c.stroke();
    c.strokeStyle = o.col2 || css('--orange'); c.lineWidth = 0.06 * V.s * w; c.beginPath(); c.moveTo(V.X(k.elbow[0]), V.Y(k.elbow[1])); c.lineTo(V.X(k.hand[0]), V.Y(k.hand[1])); c.stroke();
    c.fillStyle = css('--bright'); [[0, 1], k.elbow].forEach(p => { c.beginPath(); c.arc(V.X(p[0]), V.Y(p[1]), 3.5, 0, 7); c.fill(); });
    c.fillStyle = css('--red'); c.beginPath(); c.arc(V.X(k.hand[0]), V.Y(k.hand[1]), 4.5, 0, 7); c.fill();
    c.globalAlpha = 1; return k;
  }
  function cross(c, V, x, z, col, lab) {
    const px = V.X(x), py = V.Y(z); c.strokeStyle = col; c.lineWidth = 2;
    c.beginPath(); c.arc(px, py, 7, 0, 7); c.moveTo(px - 11, py); c.lineTo(px + 11, py); c.moveTo(px, py - 11); c.lineTo(px, py + 11); c.stroke();
    if (lab) { const right = px + 14 + lab.length * 6.8 < c.canvas.clientWidth; text(c, lab, right ? px + 12 : px - 12, py - 12, col, 11, right ? 'left' : 'right'); }
  }
  const slider = (id, out, fmtv) => { const el = $(id); const upd = () => { $(out).textContent = fmtv(+el.value); }; upd(); return el; };
  const setRo = (id, v) => { const el = $(id); if (el.textContent !== v) { el.textContent = v; flash(el.closest('.ro')); } };

  /* ---------- Section 1: try to touch the cup by hand ---------- */
  (() => {
    const cv = $('l12cvIdea'); let g = fit(cv, 0.8); const CUP = [0.6, 0.6]; let ghosts = false;
    const a = $('l12IdQ1'), b = $('l12IdQ2');
    function draw() {
      const { ctx: c, w, h } = g; c.clearRect(0, 0, w, h); const V = view(g, -1.0, 1.25, -0.1, 2.0);
      scene(c, V, { ring: false }); cross(c, V, CUP[0], CUP[1], css('--yellow'), 'cup (0.6, 0.6)');
      if (ghosts) l12IK(CUP[0], CUP[1]).sols.forEach(s => arm(c, V, s, { alpha: 0.3, thin: true }));
      const q = [+a.value, +b.value], k = arm(c, V, q);
      $('l12oIdQ1').textContent = (+a.value).toFixed(2) + ' rad'; $('l12oIdQ2').textContent = (+b.value).toFixed(2) + ' rad';
      setRo('l12rIdX', f4(k.hand[0])); setRo('l12rIdZ', f4(k.hand[1])); setRo('l12rIdD', Math.hypot(k.hand[0] - CUP[0], k.hand[1] - CUP[1]).toFixed(4));
      text(c, 'side view, +y into screen', 8, 12, css('--dim'), 10, 'left', 500);
    }
    [a, b].forEach(el => el.addEventListener('input', draw));
    $('l12IdShow').addEventListener('click', () => { ghosts = !ghosts; $('l12IdShow').textContent = ghosts ? 'Hide the two answers' : 'Show the two answers'; draw(); });
    $('l12IdUp').addEventListener('click', () => { const s = l12IK(...CUP).sols[0]; a.value = s[0]; b.value = s[1]; draw(); });
    $('l12IdDown').addEventListener('click', () => { const s = l12IK(...CUP).sols[1]; a.value = s[0]; b.value = s[1]; draw(); });
    addEventListener('resize', () => { g = fit(cv, 0.8); draw(); }); addEventListener('themechange', draw);
    draw();
  })();

  /* ---------- Section 2: forward kinematics as a chain of frames ---------- */
  (() => {
    const cv = $('l12cvChain'); let g = fit(cv, 0.85); const a = $('l12ChQ1'), b = $('l12ChQ2');
    function draw() {
      const { ctx: c, w, h } = g; c.clearRect(0, 0, w, h); const V = view(g, -0.95, 1.1, -0.05, 1.95);
      scene(c, V); const q1 = +a.value, q2 = +b.value, k = l12FK(q1, q2);
      arm(c, V, [q1, q2], { alpha: 0.25 });
      // dashed references: horizontal at the shoulder, extension of the upper arm at the elbow
      c.setLineDash([4, 4]); c.strokeStyle = css('--dim'); c.lineWidth = 1; c.beginPath();
      c.moveTo(V.X(0), V.Y(1)); c.lineTo(V.X(0.35), V.Y(1));
      c.moveTo(V.X(k.elbow[0]), V.Y(k.elbow[1])); c.lineTo(V.X(k.elbow[0] + 0.25 * Math.cos(q1)), V.Y(k.elbow[1] - 0.25 * Math.sin(q1))); c.stroke(); c.setLineDash([]);
      arrow(c, V.X(0), V.Y(1), V.X(k.elbow[0]), V.Y(k.elbow[1]), css('--teal'), 3, 9);
      arrow(c, V.X(k.elbow[0]), V.Y(k.elbow[1]), V.X(k.hand[0]), V.Y(k.hand[1]), css('--orange'), 3, 9);
      // angle arcs (canvas angles grow clockwise, the same way positive hinge angles turn here)
      c.strokeStyle = css('--yellow'); c.lineWidth = 1.5;
      c.beginPath(); c.arc(V.X(0), V.Y(1), 0.12 * V.s, Math.min(0, q1), Math.max(0, q1)); c.stroke();
      c.beginPath(); c.arc(V.X(k.elbow[0]), V.Y(k.elbow[1]), 0.1 * V.s, Math.min(q1, q1 + q2), Math.max(q1, q1 + q2)); c.stroke();
      text(c, 'q1', V.X(0.15 * Math.cos(q1 / 2)), V.Y(1 - 0.15 * Math.sin(q1 / 2)) + 2, css('--yellow'), 11, 'left');
      text(c, 'q2', V.X(k.elbow[0] + 0.14 * Math.cos(q1 + q2 / 2)), V.Y(k.elbow[1] - 0.14 * Math.sin(q1 + q2 / 2)), css('--yellow'), 11, 'left');
      cross(c, V, k.hand[0], k.hand[1], css('--red'), 'hand (' + f4(k.hand[0]) + ', ' + f4(k.hand[1]) + ')');
      $('l12oChQ1').textContent = q1.toFixed(2) + ' rad'; $('l12oChQ2').textContent = q2.toFixed(2) + ' rad';
      const c1 = 0.5 * Math.cos(q1), s1 = -0.5 * Math.sin(q1), c12 = 0.4 * Math.cos(q1 + q2), s12 = -0.4 * Math.sin(q1 + q2);
      $('l12ChEq').textContent =
        'link 1 = R(' + q1.toFixed(2) + ') [0.5, 0] = (' + p4(c1) + ', ' + p4(s1) + ')\n' +
        'link 2 = R(' + (q1 + q2).toFixed(2) + ') [0.4, 0] = (' + p4(c12) + ', ' + p4(s12) + ')\n' +
        'hand = (0, 1) + link 1 + link 2\n     = (' + p4(k.hand[0]) + ', ' + p4(k.hand[1]) + ') m, world (x, z)';
    }
    [a, b].forEach(el => el.addEventListener('input', draw));
    document.querySelectorAll('.l12chpre').forEach(bt => bt.addEventListener('click', () => { const [x, y] = bt.dataset.q.split(',').map(Number); a.value = x; b.value = y; draw(); }));
    addEventListener('resize', () => { g = fit(cv, 0.85); draw(); }); addEventListener('themechange', draw);
    draw();
  })();

  /* ---------- Section 3: the Jacobian, column by column ---------- */
  (() => {
    const cv = $('l12cvJac'); let g = fit(cv, 0.85);
    const a = $('l12JQ1'), b = $('l12JQ2'), va = $('l12JV1'), vb = $('l12JV2');
    function draw() {
      const { ctx: c, w, h } = g; c.clearRect(0, 0, w, h); const V = view(g, -0.35, 1.15, -0.05, 1.45);
      scene(c, V, { noLabel: true }); const q = [+a.value, +b.value], qd = [+va.value, +vb.value], k = arm(c, V, q), J = l12Jac(q[0], q[1]), H = k.hand;
      // faint: column j x 0.5 = hand velocity if joint j alone turns at 0.5 rad/s (drawn as metres moved in 1 s)
      for (let j = 0; j < 2; j++) {
        const col = j ? css('--orange') : css('--teal');
        const tx = H[0] + 0.5 * J[0][j], tz = H[1] + 0.5 * J[1][j];
        c.globalAlpha = 0.45; arrow(c, V.X(H[0]), V.Y(H[1]), V.X(tx), V.Y(tz), col, 1.5, 7); c.globalAlpha = 1;
        const dx = V.X(tx) - V.X(H[0]), dy = V.Y(tz) - V.Y(H[1]), n = Math.hypot(dx, dy) || 1;
        text(c, 'col ' + (j + 1), V.X(tx) + 16 * dx / n, V.Y(tz) + 16 * dy / n, col, 10, 'center');
      }
      const v = [J[0][0] * qd[0] + J[0][1] * qd[1], J[1][0] * qd[0] + J[1][1] * qd[1]];
      arrow(c, V.X(H[0]), V.Y(H[1]), V.X(H[0] + v[0]), V.Y(H[1] + v[1]), css('--purple'), 3, 9);
      $('l12oJQ1').textContent = q[0].toFixed(2) + ' rad'; $('l12oJQ2').textContent = q[1].toFixed(2) + ' rad';
      $('l12oJV1').textContent = qd[0].toFixed(2) + ' rad/s'; $('l12oJV2').textContent = qd[1].toFixed(2) + ' rad/s';
      $('l12JEq').textContent =
        'J (m/rad)               qvel (rad/s)  v_hand (m/s)\n' +
        '[ ' + p4(J[0][0]) + '  ' + p4(J[0][1]) + ' ]   [ ' + p4(qd[0]) + ' ]   [ ' + p4(v[0]) + ' ]  x\n' +
        '[ ' + p4(J[1][0]) + '  ' + p4(J[1][1]) + ' ] x [ ' + p4(qd[1]) + ' ] = [ ' + p4(v[1]) + ' ]  z';
      setRo('l12rJS', Math.hypot(v[0], v[1]).toFixed(4)); setRo('l12rJC1', Math.hypot(J[0][0], J[1][0]).toFixed(4)); setRo('l12rJR', Math.hypot(H[0], H[1] - 1).toFixed(4));
    }
    [a, b, va, vb].forEach(el => el.addEventListener('input', draw));
    document.querySelectorAll('.l12jpre').forEach(bt => bt.addEventListener('click', () => { const v = bt.dataset.v.split(',').map(Number); a.value = v[0]; b.value = v[1]; va.value = v[2]; vb.value = v[3]; draw(); }));
    addEventListener('resize', () => { g = fit(cv, 0.85); draw(); }); addEventListener('themechange', draw);
    draw();
  })();

  /* ---------- Section 4: the singular pose ---------- */
  (() => {
    const cv = $('l12cvSing'); let g = fit(cv, 0.7); const e2 = $('l12SgQ2');
    function draw() {
      const { ctx: c, w, h } = g; c.clearRect(0, 0, w, h); const V = view(g, -0.2, 1.35, 0.55, 1.45);
      scene(c, V); const q = [0, +e2.value], k = arm(c, V, q), J = l12Jac(q[0], q[1]), H = k.hand;
      // every hand velocity the joints can make with joint speeds of size 0.3 rad/s: an ellipse
      c.strokeStyle = css('--purple'); c.fillStyle = css('--purple'); c.lineWidth = 2; c.beginPath();
      for (let i = 0; i <= 120; i++) { const t = i / 120 * 2 * Math.PI, d = [0.3 * Math.cos(t), 0.3 * Math.sin(t)]; const x = H[0] + J[0][0] * d[0] + J[0][1] * d[1], z = H[1] + J[1][0] * d[0] + J[1][1] * d[1]; i ? c.lineTo(V.X(x), V.Y(z)) : c.moveTo(V.X(x), V.Y(z)); }
      c.globalAlpha = 0.12; c.fill(); c.globalAlpha = 1; c.stroke();
      // the arm direction: the hand would need to move along this to reach further
      const ux = Math.cos(q[0]), uz = -Math.sin(q[0]);
      arrow(c, V.X(H[0]), V.Y(H[1]), V.X(H[0] + 0.25 * ux), V.Y(H[1] + 0.25 * uz), css('--yellow'), 1.5, 7);
      text(c, 'outward', V.X(H[0] + 0.25 * ux), V.Y(H[1] + 0.25 * uz) - 12, css('--yellow'), 10, 'center');
      const s = sv2(J), det = J[0][0] * J[1][1] - J[0][1] * J[1][0];
      $('l12oSgQ2').textContent = (+e2.value).toFixed(2) + ' rad';
      setRo('l12rSgDet', (Math.abs(det) < 5e-6 ? 0 : det).toFixed(5)); setRo('l12rSgMin', s[1].toFixed(5));
      setRo('l12rSgCond', s[1] < 1e-12 ? 'infinite' : (s[0] / s[1]).toFixed(1)); setRo('l12rSgVx', f4(J[0][0] * 0.3) + ', ' + f4(J[0][1] * 0.3));
    }
    e2.addEventListener('input', draw);
    document.querySelectorAll('.l12sgpre').forEach(bt => bt.addEventListener('click', () => { e2.value = bt.dataset.q; draw(); }));
    addEventListener('resize', () => { g = fit(cv, 0.7); draw(); }); addEventListener('themechange', draw);
    draw();
  })();

  /* ---------- Section 5: analytic IK, both answers ---------- */
  (() => {
    const cv = $('l12cvIk'); let g = fit(cv, 0.85); const tx = $('l12IkX'), tz = $('l12IkZ');
    let V = null;
    function draw() {
      const { ctx: c, w, h } = g; c.clearRect(0, 0, w, h); V = view(g, -1.05, 1.25, -0.05, 2.0);
      scene(c, V, { ring: true }); const x = +tx.value, z = +tz.value, r = l12IK(x, z);
      if (r.sols.length) {
        if (r.sols[1]) arm(c, V, r.sols[1], { col1: css('--purple'), col2: css('--purple'), alpha: 0.55, thin: true });
        arm(c, V, r.sols[0]);
      } else {
        c.setLineDash([5, 4]); c.strokeStyle = css('--red'); c.lineWidth = 1.5; c.beginPath(); c.moveTo(V.X(0), V.Y(1)); c.lineTo(V.X(x), V.Y(z)); c.stroke(); c.setLineDash([]);
      }
      cross(c, V, x, z, css('--yellow'), 'target');
      $('l12oIkX').textContent = x.toFixed(2) + ' m'; $('l12oIkZ').textContent = z.toFixed(2) + ' m';
      setRo('l12rIkR', r.r.toFixed(4)); setRo('l12rIkC', r.c2.toFixed(4));
      setRo('l12rIkUp', r.sols[0] ? f4(r.sols[0][0]) + ', ' + f4(r.sols[0][1]) : 'none');
      setRo('l12rIkDn', r.sols.length === 2 ? f4(r.sols[1][0]) + ', ' + f4(r.sols[1][1]) : (r.sols.length ? 'same pose' : 'none'));
      setRo('l12rIkN', r.sols.length + (r.why ? ' (' + r.why + ')' : ''));
    }
    [tx, tz].forEach(el => el.addEventListener('input', draw));
    document.querySelectorAll('.l12ikpre').forEach(bt => bt.addEventListener('click', () => { const v = bt.dataset.t.split(',').map(Number); tx.value = v[0]; tz.value = v[1]; draw(); }));
    cv.addEventListener('pointerdown', ev => {
      if (!V) return; const rc = cv.getBoundingClientRect(), px = ev.clientX - rc.left, py = ev.clientY - rc.top;
      const x0 = V.X(0), y1 = V.Y(0);
      tx.value = Math.round((px - x0) / V.s * 100) / 100; tz.value = Math.round((y1 - py) / V.s * 100) / 100; draw();
    });
    addEventListener('resize', () => { g = fit(cv, 0.85); draw(); }); addEventListener('themechange', draw);
    draw();
  })();

  /* ---------- Section 6: damped least squares, step by step ---------- */
  (() => {
    const cv = $('l12cvDls'); let g = fit(cv, 0.85); const lam = $('l12DLam'), log = $('l12DLog');
    const st = { q: [0, 0], start: [0, 0], t: [0.6, 0.6], it: 0, path: [], lines: [], timer: null };
    function stop() { if (st.timer) { clearTimeout(st.timer); st.timer = null; } $('l12DSolve').textContent = 'Solve (up to 60 steps)'; }
    function reset() {
      stop(); st.q = st.start.slice(); st.it = 0; st.path = [l12FK(st.q[0], st.q[1]).hand];
      st.lines = ['target (' + st.t[0].toFixed(1) + ', ' + st.t[1].toFixed(1) + ') m, start q = (' + st.start.join(', ') + '), lambda = ' + (+lam.value).toFixed(2) + ' m', ROWHEAD, row(0, st.q, ...st.t)];
      draw();
    }
    function step() {
      const r = l12DLS(st.q, st.t[0], st.t[1], +lam.value);
      if (r.singular) { st.lines.push('lambda = 0 and J J^T has det 0: cannot invert, no step'); draw(); return false; }
      st.q = r.q; st.it++; st.path.push(l12FK(st.q[0], st.q[1]).hand); st.lines.push(row(st.it, st.q, ...st.t));
      $('l12rDdq').textContent = Math.hypot(r.dq[0], r.dq[1]).toFixed(4); flash($('l12rDdq').closest('.ro'));
      const e = errOf(st.q, ...st.t); draw(); return e >= 1e-6 && st.it < 60;
    }
    function draw() {
      const { ctx: c, w, h } = g; c.clearRect(0, 0, w, h); const V = view(g, -1.0, 1.3, -0.05, 2.0);
      scene(c, V, { ring: true }); arm(c, V, st.q);
      c.strokeStyle = css('--purple'); c.fillStyle = css('--purple'); c.lineWidth = 1.5; c.beginPath();
      st.path.forEach((p, i) => { const x = Math.max(-5, Math.min(5, p[0])), z = Math.max(-5, Math.min(5, p[1])); i ? c.lineTo(V.X(x), V.Y(z)) : c.moveTo(V.X(x), V.Y(z)); }); c.stroke();
      st.path.forEach((p, i) => { c.beginPath(); c.arc(V.X(p[0]), V.Y(p[1]), 3, 0, 7); c.fill(); if (i < 8) text(c, String(i), V.X(p[0]) + 6, V.Y(p[1]) - 8, css('--purple'), 10, 'left'); });
      cross(c, V, st.t[0], st.t[1], css('--yellow'), 'target');
      log.textContent = st.lines.join('\n'); log.scrollTop = 1e9;
      $('l12oDLam').textContent = (+lam.value).toFixed(2) + ' m';
      $('l12rDit').textContent = String(st.it); $('l12rDerr').textContent = errOf(st.q, ...st.t).toFixed(6);
      $('l12rDq').textContent = f4(st.q[0]) + ', ' + f4(st.q[1]);
    }
    $('l12DStep').addEventListener('click', () => { stop(); step(); });
    $('l12DSolve').addEventListener('click', () => {
      if (st.timer) { stop(); return; }
      if (RM) { while (step()); return; }
      const tick = () => { st.timer = null; if (step()) { st.timer = setTimeout(tick, 350); $('l12DSolve').textContent = 'Stop'; } else stop(); };
      tick();
    });
    $('l12DReset').addEventListener('click', reset);
    lam.addEventListener('input', reset);
    document.querySelectorAll('.l12dstart').forEach(bt => bt.addEventListener('click', () => { st.start = bt.dataset.q.split(',').map(Number); reset(); }));
    document.querySelectorAll('.l12dtgt').forEach(bt => bt.addEventListener('click', () => { st.t = bt.dataset.t.split(',').map(Number); reset(); }));
    addEventListener('resize', () => { g = fit(cv, 0.85); draw(); }); addEventListener('themechange', draw);
    reset();
  })();
})();
})();
