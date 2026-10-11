// ======== SANDBOX 5: ACTUATOR PLAYGROUND ========
// Same model as 05_actuators_and_control.py: course pendulum + one actuator, Euler, h = 0.002 s.
// I and m*g*d are read from MuJoCo (mj_fullM and body_mass). MuJoCo's Euler integrator treats only
// joint damping implicitly, so the kp / kv terms below are explicit forces from the current state.
// 05_check_browser_match.py compares this exact stepping rule with mj_step: max diff 1.8e-12 over 10 s.
(() => {
  const H = 0.002, I = 7.876394677499037, MGD = 109.14043675018316;
  const $ = id => document.getElementById(id);
  const TYPES = {
    motor:    { lo: -1, hi: 1, c0: 0.5, cStep: 0.005, p0: 100, pMax: 200, pStep: 5, btn: 's5TypeMotor',
                text: 'motor: torque = gear x ctrl', cLab: 'ctrl (strength, -1 to 1)', pLab: 'gear (N m per unit ctrl)', cU: '(strength)', fU: '(before gear)' },
    position: { lo: -1.57, hi: 1.57, c0: 0.5, cStep: 0.01, p0: 500, pMax: 2000, pStep: 10, btn: 's5TypePos',
                text: 'position: torque = kp x (ctrl - qpos)', cLab: 'ctrl (target angle, rad)', pLab: 'kp (N m per rad)', cU: 'rad (target)', fU: 'N m (gear 1)' },
    velocity: { lo: -2, hi: 2, c0: 0.5, cStep: 0.01, p0: 200, pMax: 1000, pStep: 10, btn: 's5TypeVel',
                text: 'velocity: torque = kv x (ctrl - qvel)', cLab: 'ctrl (target speed, rad/s)', pLab: 'kv (N m per rad/s)', cU: 'rad/s (target)', fU: 'N m (gear 1)' },
  };
  let type = 'motor';
  const s = { q: 0, v: 0, t: 0, f: 0, tau: 0 };
  let playing = false, last = 0, acc = 0, raf = 0;

  // force = gain * clip(ctrl) + bias(q, v); torque on hinge = gear * force (gear is 1 for position / velocity)
  function forces(q, v) {
    const T = TYPES[type], ctrl = +$('s5Ctrl').value, par = +$('s5Par').value;
    const c = Math.min(T.hi, Math.max(T.lo, ctrl));
    if (type === 'motor') return [c, par * c];
    const f = type === 'position' ? par * (c - q) : par * (c - v);
    return [f, f];
  }
  function step() {
    const [f, tau] = forces(s.q, s.v);
    s.f = f; s.tau = tau;
    const a = (tau - MGD * Math.sin(s.q)) / I;
    s.v += H * a; s.q += H * s.v; s.t += H;
  }
  function arcPath(r, phi, span) {
    if (Math.abs(span) < 0.03) return '';
    const a0 = phi - span / 2, a1 = phi + span / 2;
    const p = a => (200 + r * Math.cos(a)).toFixed(1) + ' ' + (120 + r * Math.sin(a)).toFixed(1);
    return `M ${p(a0)} A ${r} ${r} 0 0 ${span > 0 ? 1 : 0} ${p(a1)}`;
  }
  function render() {
    const T = TYPES[type];
    $('s5Pendulum').setAttribute('transform', `translate(200, 120) rotate(${(s.q * 180 / Math.PI).toFixed(3)})`);
    // screen angle of the rod (y down): pi/2 + q; a positive torque increases q (clockwise on screen)
    const phi = Math.PI / 2 + s.q, tg = -MGD * Math.sin(s.q);
    const span = tau => Math.max(-2.2, Math.min(2.2, tau / 120 * 1.6));
    $('s5TauArc').setAttribute('d', arcPath(150, phi, span(s.tau)));
    $('s5GravArc').setAttribute('d', arcPath(175, phi, span(tg)));
    const tgt = $('s5Target');
    if (type === 'position') { tgt.style.display = ''; tgt.setAttribute('transform', `rotate(${(Math.min(T.hi, Math.max(T.lo, +$('s5Ctrl').value)) * 180 / Math.PI).toFixed(3)}, 200, 120)`); }
    else tgt.style.display = 'none';
    $('s5TypeText').textContent = T.text;
    $('s5TimeText').textContent = `time: ${s.t.toFixed(3)} s`;
    $('s5Time').textContent = s.t.toFixed(3);
    $('s5RCtrl').textContent = (+$('s5Ctrl').value).toFixed(4);
    $('s5RForce').textContent = s.f.toFixed(4);
    $('s5RTau').textContent = s.tau.toFixed(4);
    $('s5Qpos').textContent = s.q.toFixed(4);
    $('s5Qvel').textContent = s.v.toFixed(4);
  }
  // Before any step (and after a slider move at rest) show what mj_forward would report.
  function preview() { const [f, tau] = forces(s.q, s.v); s.f = f; s.tau = tau; render(); }
  function stop() { playing = false; cancelAnimationFrame(raf); $('s5Play').textContent = 'Play'; }
  function loop(ts) {
    if (!playing) return;
    const dt = Math.min(0.05, (ts - last) / 1000); last = ts; acc += dt;
    while (acc >= H) { step(); acc -= H; }
    render();
    if (s.t >= 20 || !isFinite(s.q)) { stop(); return; }
    raf = requestAnimationFrame(loop);
  }
  $('s5Play').addEventListener('click', () => {
    if (playing) { stop(); return; }
    if (RM) { for (let i = 0; i < 500; i++) step(); render(); return; } // reduced motion: jump 1 s
    if (s.t >= 20) reset();
    playing = true; $('s5Play').textContent = 'Pause'; last = performance.now(); acc = 0; raf = requestAnimationFrame(loop);
  });
  $('s5Step').addEventListener('click', () => { step(); render(); });
  $('s5Step100').addEventListener('click', () => { for (let i = 0; i < 100; i++) step(); render(); });
  function reset() { stop(); Object.assign(s, { q: 0, v: 0, t: 0 }); preview(); }
  $('s5Reset').addEventListener('click', reset);
  function syncLabels() {
    const T = TYPES[type];
    $('s5CtrlVal').textContent = (+$('s5Ctrl').value).toFixed(type === 'motor' ? 3 : 2);
    $('s5ParVal').textContent = $('s5Par').value;
    $('s5CtrlLab').textContent = T.cLab; $('s5ParLab').textContent = T.pLab;
    $('s5RCtrlU').textContent = T.cU; $('s5RForceU').textContent = T.fU;
  }
  function setType(k) {
    type = k; const T = TYPES[k], c = $('s5Ctrl'), p = $('s5Par');
    Object.keys(TYPES).forEach(n => { const b = $(TYPES[n].btn); b.classList.toggle('btn-primary', n === k); b.setAttribute('aria-pressed', String(n === k)); });
    c.min = T.lo; c.max = T.hi; c.step = T.cStep; c.value = T.c0;
    p.min = 0; p.max = T.pMax; p.step = T.pStep; p.value = T.p0;
    syncLabels(); reset();
  }
  Object.keys(TYPES).forEach(k => $(TYPES[k].btn).addEventListener('click', () => setType(k)));
  $('s5Ctrl').addEventListener('input', () => { syncLabels(); if (!playing) preview(); });
  $('s5Par').addEventListener('input', () => { syncLabels(); if (!playing) preview(); });
  syncLabels(); preview();
})();

// ======== LESSON 5 EXPLAINER ========
(() => {
  // Only redraw this lesson's canvases while it is on screen (hidden canvases have zero size).
  const addEventListener = (type, fn, opts) => window.addEventListener(type, e => { if (document.getElementById('lesson5').classList.contains('active')) fn(e); }, opts);
// ---------- Lesson 5 page logic ----------
// Same model as 05_actuators_and_control.py. I = 7.87639 kg m^2 (mj_fullM), m g d = 109.14044 N m.
const H = 0.002, I = 7.876394677499037, MGD = 109.14043675018316;
const $ = id => document.getElementById(id);
const setRo = (id, txt) => { const el = $(id); const v = el.querySelector('.val'); if (v.textContent !== txt) { v.textContent = txt; flash(el); } };
const sgn = (v, d) => (v >= 0 ? '+' : '') + v.toFixed(d);

// Draw the pendulum (1 m rod) hanging from P. +q swings the end to the left (hinge axis +y into the screen).
function drawPend(c, P, L, q, col, o = {}) {
  c.setLineDash([4, 5]); c.strokeStyle = css('--dim'); c.lineWidth = 1; c.beginPath(); c.moveTo(P[0], P[1]); c.lineTo(P[0], P[1] + L * 1.08); c.stroke(); c.setLineDash([]);
  if (o.target !== undefined) {
    c.setLineDash([6, 5]); c.strokeStyle = css('--purple'); c.lineWidth = 1.5; c.beginPath(); c.moveTo(...P); c.lineTo(P[0] - Math.sin(o.target) * L * 1.05, P[1] + Math.cos(o.target) * L * 1.05); c.stroke(); c.setLineDash([]);
  }
  const end = [P[0] - Math.sin(q) * L, P[1] + Math.cos(q) * L];
  c.strokeStyle = col; c.lineWidth = Math.max(4, Math.min(0.16 * L, 18)); c.lineCap = 'round'; c.beginPath(); c.moveTo(...P); c.lineTo(...end); c.stroke();
  const com = [P[0] - Math.sin(q) * L / 2, P[1] + Math.cos(q) * L / 2];
  c.fillStyle = css('--bright'); c.beginPath(); c.arc(...com, 4, 0, 7); c.fill();
  c.fillStyle = css('--purple'); c.beginPath(); c.arc(...P, 5, 0, 7); c.fill();
  return { com, end };
}
// Curved torque arrow around the pivot, centred on the rod direction. Positive torque increases q.
function torqueArc(c, P, r, q, tau, col, width = 3) {
  const span = Math.max(-2.2, Math.min(2.2, tau / 120 * 1.6));
  if (Math.abs(span) < 0.03) return;
  const phi = Math.PI / 2 + q, a0 = phi - span / 2, a1 = phi + span / 2;
  c.strokeStyle = col; c.lineWidth = width; c.beginPath(); c.arc(P[0], P[1], r, Math.min(a0, a1), Math.max(a0, a1)); c.stroke();
  const ex = P[0] + r * Math.cos(a1), ey = P[1] + r * Math.sin(a1), dir = Math.sign(span);
  const tx = -Math.sin(a1) * dir, ty = Math.cos(a1) * dir; // tangent in the direction of travel
  arrow(c, ex - tx * 6, ey - ty * 6, ex + tx * 4, ey + ty * 4, col, width, 10);
}

/* ---------- Section 1: tug of war (no time passes) ---------- */
(() => {
  const cv = $('l5cvA'); let g = fit(cv, 0.9);
  function draw() {
    const q = +$('l5aQ').value, tm = +$('l5aT').value, tg = -MGD * Math.sin(q), net = tm + tg;
    $('l5oAq').textContent = q.toFixed(2) + ' rad'; $('l5oAt').textContent = tm.toFixed(1) + ' N m';
    setRo('l5aG', sgn(tg, 2)); setRo('l5aM', sgn(tm, 2)); setRo('l5aN', sgn(net, 2));
    $('l5aV').textContent = Math.abs(net) < 0.5 ? 'Balanced (within 0.5 N m): the rod would stay still.' : net > 0 ? 'The muscle wins: the rod would start turning up (qpos grows).' : 'Gravity wins: the rod would start falling back down.';
    const { ctx: c, w, h } = g; c.clearRect(0, 0, w, h);
    const L = h * 0.55, P = [w * 0.6, h * 0.16];
    drawPend(c, P, L, q, css('--teal'));
    torqueArc(c, P, L * 0.72, q, tm, css('--teal'), 4);
    torqueArc(c, P, L * 0.92, q, tg, css('--orange'), 3);
    text(c, 'muscle ' + tm.toFixed(1) + ' N m', 10, h - 40, css('--teal'), 11);
    text(c, 'gravity ' + tg.toFixed(1) + ' N m', 10, h - 22, css('--orange'), 11);
    text(c, 'qpos = ' + q.toFixed(2) + ' rad', 10, 14, css('--yellow'), 11);
  }
  $('l5aQ').addEventListener('input', draw); $('l5aT').addEventListener('input', draw);
  addEventListener('resize', () => { g = fit(cv, 0.9); draw(); }); addEventListener('themechange', draw);
  draw();
})();

/* ---------- Section 3: ctrl -> clip -> gear -> torque ---------- */
(() => {
  const cv = $('l5cvB'); let g = fit(cv, 0.42);
  function draw() {
    const ctrl = +$('l5bC').value, gear = +$('l5bG').value, f = Math.min(1, Math.max(-1, ctrl)), tau = gear * f;
    $('l5oBc').textContent = ctrl.toFixed(2); $('l5oBg').textContent = gear.toFixed(0);
    setRo('l5bRc', ctrl.toFixed(3)); setRo('l5bRf', f.toFixed(3)); setRo('l5bRq', tau.toFixed(3));
    const { ctx: c, w, h } = g; c.clearRect(0, 0, w, h);
    // number line for ctrl, with the allowed band
    const X = v => 30 + (v + 2) / 4 * (w - 60), y0 = h * 0.24;
    c.fillStyle = css('--grid'); c.fillRect(X(-1), y0 - 10, X(1) - X(-1), 20);
    c.strokeStyle = css('--dim'); c.lineWidth = 1; c.beginPath(); c.moveTo(X(-2), y0); c.lineTo(X(2), y0); c.stroke();
    [-2, -1, 0, 1, 2].forEach(v => { c.beginPath(); c.moveTo(X(v), y0 - 5); c.lineTo(X(v), y0 + 5); c.stroke(); text(c, String(v), X(v), y0 + 18, css('--dim'), 10, 'center'); });
    text(c, 'ctrlrange -1 to 1', X(0), y0 - 20, css('--purple'), 11, 'center');
    c.fillStyle = css('--blue'); c.beginPath(); c.arc(X(ctrl), y0, 6, 0, 7); c.fill();
    if (f !== ctrl) { arrow(c, X(ctrl), y0 + 2, X(f) + (ctrl > 0 ? 8 : -8), y0 + 2, css('--orange'), 2, 7); text(c, 'clipped', (X(ctrl) + X(f)) / 2, y0 - 12, css('--orange'), 10, 'center'); }
    c.fillStyle = css('--teal'); c.beginPath(); c.arc(X(f), y0, 4, 0, 7); c.fill();
    // pipeline boxes
    const by = h * 0.68, bw = (w - 60) / 3, labels = [['data.ctrl', ctrl.toFixed(3), css('--blue')], ['actuator_force', f.toFixed(3), css('--teal')], ['qfrc_actuator', tau.toFixed(1) + ' N m', css('--orange')]];
    labels.forEach(([lab, val, col], i) => {
      const x = 10 + i * (bw + 20);
      c.strokeStyle = col; c.lineWidth = 1.5; c.strokeRect(x, by - 24, bw, 48);
      text(c, lab, x + bw / 2, by - 10, css('--dim'), 10, 'center'); text(c, val, x + bw / 2, by + 10, col, 13, 'center', 700);
      if (i < 2) { arrow(c, x + bw + 2, by, x + bw + 18, by, css('--dim'), 1.5, 6); text(c, i ? 'x ' + gear : 'clip', x + bw + 10, by + 34, css('--dim'), 10, 'center'); }
    });
  }
  $('l5bC').addEventListener('input', draw); $('l5bG').addEventListener('input', draw);
  addEventListener('resize', () => { g = fit(cv, 0.42); draw(); }); addEventListener('themechange', draw);
  draw();
})();

/* ---------- Section 4: hold still at 0.5 rad (motor, gear 100) ---------- */
(() => {
  const cv = $('l5cvC'); let g = fit(cv, 1.05);
  const Q0 = 0.5, TEND = 3, GEAR = 100;
  let s, hist, playing = false, last = 0, acc = 0, raf = 0;
  const ctrl = () => Math.min(1, Math.max(-1, +$('l5cC').value));
  function reset() { s = { q: Q0, v: 0, t: 0 }; hist = [[0, Q0]]; draw(); }
  function step() { const tau = GEAR * ctrl(), a = (tau - MGD * Math.sin(s.q)) / I; s.v += H * a; s.q += H * s.v; s.t += H; if (Math.round(s.t / H) % 5 === 0) hist.push([s.t, s.q]); }
  function draw() {
    $('l5oCc').textContent = (+$('l5cC').value).toFixed(5);
    const tau = GEAR * ctrl();
    setRo('l5cT', s.t.toFixed(3)); setRo('l5cQ', s.q.toFixed(5)); setRo('l5cV', s.v.toFixed(5)); setRo('l5cF', tau.toFixed(3));
    const { ctx: c, w, h } = g; c.clearRect(0, 0, w, h);
    const L = h * 0.42, P = [w * 0.55, h * 0.1];
    drawPend(c, P, L, s.q, css('--teal'), { target: Q0 });
    torqueArc(c, P, L * 0.72, s.q, tau, css('--teal'), 4);
    torqueArc(c, P, L * 0.92, s.q, -MGD * Math.sin(s.q), css('--orange'), 3);
    // qpos trace over the run
    const y0 = h * 0.62, ph = h * 0.34, X = t => 40 + t / TEND * (w - 50), Y = q => y0 + ph - (q + 0.5) / 2 * ph;
    c.strokeStyle = css('--grid'); c.lineWidth = 1; c.strokeRect(40, y0, w - 50, ph);
    c.setLineDash([4, 4]); c.strokeStyle = css('--purple'); c.beginPath(); c.moveTo(40, Y(Q0)); c.lineTo(w - 10, Y(Q0)); c.stroke(); c.setLineDash([]);
    [-0.5, 0.5, 1.5].forEach(v => text(c, v.toFixed(1), 34, Y(v), css('--dim'), 9, 'right'));
    text(c, 'qpos (rad) over ' + TEND + ' s', 44, y0 + 10, css('--dim'), 10);
    c.strokeStyle = css('--teal'); c.lineWidth = 2; c.beginPath();
    hist.forEach(([t, q], i) => { const y = Math.max(y0, Math.min(y0 + ph, Y(q))); i ? c.lineTo(X(t), y) : c.moveTo(X(t), y); }); c.stroke();
    text(c, 'gear x ctrl = ' + tau.toFixed(2) + ' N m', 10, 14, css('--teal'), 11);
    text(c, 'gravity = ' + (-MGD * Math.sin(s.q)).toFixed(2) + ' N m', 10, 30, css('--orange'), 11);
  }
  function stop() { playing = false; cancelAnimationFrame(raf); $('l5cPlay').textContent = 'Run 3 s'; }
  function loop(ts) {
    if (!playing) return;
    const dt = Math.min(0.05, (ts - last) / 1000); last = ts; acc += dt;
    while (acc >= H && s.t < TEND - 1e-9) { step(); acc -= H; }
    draw();
    if (s.t >= TEND - 1e-9) { stop(); return; }
    raf = requestAnimationFrame(loop);
  }
  $('l5cPlay').addEventListener('click', () => {
    if (playing) { stop(); return; }
    if (s.t >= TEND - 1e-9) reset();
    if (RM) { while (s.t < TEND - 1e-9) step(); draw(); return; }
    playing = true; $('l5cPlay').textContent = 'Pause'; last = performance.now(); acc = 0; raf = requestAnimationFrame(loop);
  });
  $('l5cHold').addEventListener('click', () => { $('l5cC').value = 0.52325; if (!playing) reset(); });
  $('l5cReset').addEventListener('click', () => { stop(); reset(); });
  $('l5cC').addEventListener('input', () => { if (!playing) draw(); else $('l5oCc').textContent = (+$('l5cC').value).toFixed(5); });
  addEventListener('resize', () => { g = fit(cv, 1.05); draw(); }); addEventListener('themechange', draw);
  reset();
})();

/* ---------- Section 5: three actuator types, same start, ctrl = 0.5 ---------- */
(() => {
  const cv = $('l5cvD'), ce = $('l5cvE'); let g = fit(cv, 0.42), ge = fit(ce, 0.38);
  const CTRL = 0.5, KINDS = [
    ['motor', '--teal', (q, v) => 100 * CTRL, 'l5dM'],
    ['servo', '--purple', (q, v) => 500 * (CTRL - q), 'l5dP'],
    ['cruise', '--blue', (q, v) => 200 * (CTRL - v), 'l5dV'],
  ];
  let st, hist, t, tEnd = 1, playing = false, last = 0, acc = 0, raf = 0;
  function reset() { st = KINDS.map(() => ({ q: 0, v: 0 })); hist = [[0, 0, 0, 0]]; t = 0; draw(); }
  function step() {
    KINDS.forEach(([, , tauFn], i) => { const s = st[i], a = (tauFn(s.q, s.v) - MGD * Math.sin(s.q)) / I; s.v += H * a; s.q += H * s.v; });
    t += H; if (Math.round(t / H) % 5 === 0) hist.push([t, st[0].q, st[1].q, st[2].q]);
  }
  function draw() {
    KINDS.forEach(([, , , id], i) => setRo(id, st[i].q.toFixed(5))); setRo('l5dT', t.toFixed(3));
    const { ctx: c, w, h } = g; c.clearRect(0, 0, w, h);
    const L = h * 0.42;
    KINDS.forEach(([name, col], i) => {
      const P = [w * (i + 0.5) / 3, h * 0.14];
      drawPend(c, P, L, st[i].q, css(col), i === 1 ? { target: CTRL } : {});
      text(c, name, P[0], h - 12, css(col), 11, 'center');
    });
    const { ctx: e, w: W, h: Hh } = ge; e.clearRect(0, 0, W, Hh);
    const span = Math.max(1, tEnd), X = tt => 36 + tt / span * (W - 46), Y = q => Hh - 16 - (q + 0.2) / 1.6 * (Hh - 26);
    e.strokeStyle = css('--grid'); e.lineWidth = 1;
    [0, 0.5, 1].forEach(v => { e.beginPath(); e.moveTo(36, Y(v)); e.lineTo(W - 10, Y(v)); e.stroke(); text(e, v.toFixed(1), 30, Y(v), css('--dim'), 9, 'right'); });
    for (let k = 0; k <= span; k += span > 2 ? 2 : 0.5) text(e, k + ' s', X(k), Hh - 6, css('--dim'), 9, 'center');
    text(e, 'qpos (rad)', 40, 10, css('--dim'), 10);
    KINDS.forEach(([, col], i) => { e.strokeStyle = css(col); e.lineWidth = 2; e.beginPath(); hist.forEach((r, j) => { j ? e.lineTo(X(r[0]), Y(r[i + 1])) : e.moveTo(X(r[0]), Y(r[i + 1])); }); e.stroke(); });
  }
  function stop() { playing = false; cancelAnimationFrame(raf); }
  function loop(ts) {
    if (!playing) return;
    const dt = Math.min(0.05, (ts - last) / 1000); last = ts; acc += dt;
    while (acc >= H && t < tEnd - 1e-9) { step(); acc -= H; }
    draw();
    if (t >= tEnd - 1e-9) { stop(); return; }
    raf = requestAnimationFrame(loop);
  }
  function run(T) {
    stop(); tEnd = T; reset();
    if (RM) { while (t < tEnd - 1e-9) step(); draw(); return; }
    playing = true; last = performance.now(); acc = 0; raf = requestAnimationFrame(loop);
  }
  $('l5dRun1').addEventListener('click', () => run(1));
  $('l5dRun10').addEventListener('click', () => run(10));
  $('l5dReset').addEventListener('click', () => { stop(); tEnd = 1; reset(); });
  addEventListener('resize', () => { g = fit(cv, 0.42); ge = fit(ce, 0.38); draw(); }); addEventListener('themechange', draw);
  reset();
})();

})();
