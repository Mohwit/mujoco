// ======== LESSON 8: PD CONTROLLER (shared core, sandbox 8, explainer) ========
(() => {
// SIM-CORE-BEGIN
// Same model as 08_controllers_and_experiments.py: course pendulum + the lesson 5 motor (gear 100, ctrlrange -1 1).
// I and m*g*d are read from MuJoCo 3.15.0 (mj_fullM and body_mass). One step below is exactly what
// MuJoCo's Euler integrator does for this model: the controller reads (qpos, qvel) and computes tau in N m,
// ctrl = tau / gear clipped to [-1, 1], torque = gear * ctrl, qacc = (torque - m g d sin q) / I,
// then qvel += h*qacc and qpos += h*qvel (with the NEW qvel).
const L8 = { I: 7.876394677499037, MGD: 109.14043675018316, GEAR: 100, BAND: 0.02 };
function l8Run(kp, kd, target, h, gc, tEnd, limited = true) {
  const n = Math.round(tEnd / h), t = [0], q = [0], v = [0], u = [], f = [];
  let qq = 0, vv = 0;
  for (let k = 0; k < n; k++) {
    let tau = kp * (target - qq) - kd * vv;              // 1. read state, 2. compute tau (N m)
    if (gc) tau += L8.MGD * Math.sin(qq);
    let c = tau / L8.GEAR;                                // ...and turn it into ctrl
    if (limited) c = Math.max(-1, Math.min(1, c));        // ctrlrange -1 1
    const trq = L8.GEAR * c;                              // torque the motor applies
    const a = (trq - L8.MGD * Math.sin(qq)) / L8.I;       // 3. mj_step: qacc from forces
    vv += h * a; qq += h * vv;                            // semi-implicit Euler
    u.push(tau); f.push(trq); t.push((k + 1) * h); q.push(qq); v.push(vv);
    if (!isFinite(qq) || Math.abs(qq) > 100) break;       // exploded
  }
  return { t, q, v, u, f };
}
function l8QStar(kp, target, gc) {
  if (gc) return target;
  let lo = 0, hi = target;
  for (let i = 0; i < 80; i++) { const m = 0.5 * (lo + hi); if (kp * (target - m) - L8.MGD * Math.sin(m) > 0) lo = m; else hi = m; }
  return 0.5 * (lo + hi);
}
function l8Metrics(r, kp, target, gc) {
  const qs = l8QStar(kp, target, gc), { t, q, f } = r;
  let rise = null, qmax = -Infinity, lastOut = -1, peak = 0;
  for (let i = 0; i < q.length; i++) {
    if (rise === null && q[i] >= 0.9 * qs) rise = t[i];
    if (q[i] > qmax) qmax = q[i];
    if (Math.abs(q[i] - qs) > L8.BAND * target) lastOut = i;
  }
  f.forEach(x => { if (Math.abs(x) > peak) peak = Math.abs(x); });
  const settle = lastOut < 0 ? 0 : (lastOut === q.length - 1 ? null : t[lastOut + 1]);
  return { qs, rise, overshoot: Math.max(0, (qmax - qs) / qs * 100), settle, ssErr: target - q[q.length - 1], peak, qmax };
}
// SIM-CORE-END

const $ = id => document.getElementById(id);
const f3 = x => x === null ? 'not settled' : x.toFixed(3);
const z = x => Math.abs(x) < 5e-7 ? 0 : x;
// growth of a wobble per step for I*qacc = -kp*e - kd*qvel under semi-implicit Euler (largest |eigenvalue|)
function l8Growth(kp, kd, h) {
  const tr = 2 - h * h * kp / L8.I - h * kd / L8.I, det = 1 - h * kd / L8.I, disc = tr * tr - 4 * det;
  if (disc < 0) return Math.sqrt(Math.abs(det));
  const s = Math.sqrt(disc); return Math.max(Math.abs((tr + s) / 2), Math.abs((tr - s) / 2));
}

// Draw one plot band: series = [{x:[], y:[], col, w, dots}], with optional horizontal guide lines.
function plot(c, box, xr, yr, series, guides, ylab) {
  const [x0, y0, w, h] = box, X = x => x0 + (x - xr[0]) / (xr[1] - xr[0]) * w, Y = y => y0 + h - (y - yr[0]) / (yr[1] - yr[0]) * h;
  c.strokeStyle = css('--grid'); c.lineWidth = 1; c.strokeRect(x0, y0, w, h);
  const step = xr[1] > 6 ? 2 : (xr[1] > 2 ? 1 : 0.5);
  for (let x = xr[0]; x <= xr[1] + 1e-9; x += step) { c.beginPath(); c.moveTo(X(x), y0); c.lineTo(X(x), y0 + h); c.stroke(); text(c, x + ' s', X(x), y0 + h + 10, css('--dim'), 9, 'center'); }
  text(c, yr[1].toFixed(yr[1] >= 10 ? 0 : 1), x0 - 4, y0 + 6, css('--dim'), 9, 'right');
  text(c, yr[0].toFixed(yr[0] <= -10 ? 0 : 1), x0 - 4, y0 + h - 6, css('--dim'), 9, 'right');
  if (ylab) text(c, ylab, x0 + 6, y0 + 10, css('--dim'), 10);
  (guides || []).forEach(gd => { c.setLineDash([5, 4]); c.strokeStyle = gd.col; c.lineWidth = 1.5; c.beginPath(); c.moveTo(x0, Y(gd.y)); c.lineTo(x0 + w, Y(gd.y)); c.stroke(); c.setLineDash([]); if (gd.label) text(c, gd.label, x0 + w - 4, Y(gd.y) - 8, gd.col, 10, 'right'); });
  c.save(); c.beginPath(); c.rect(x0, y0 - 2, w, h + 4); c.clip();
  series.forEach(s => {
    c.strokeStyle = s.col; c.lineWidth = s.w || 2; c.beginPath();
    const n = s.n === undefined ? s.x.length : s.n, stride = Math.max(1, Math.floor(n / 1500));
    for (let i = 0; i < n; i += stride) { const yy = Math.max(-1e4, Math.min(1e4, Y(s.y[i]))); i ? c.lineTo(X(s.x[i]), yy) : c.moveTo(X(s.x[i]), yy); }
    c.stroke();
    if (s.dots && n <= 400) { c.fillStyle = s.col; for (let i = 0; i < n; i++) { c.beginPath(); c.arc(X(s.x[i]), Y(s.y[i]), 2.2, 0, 7); c.fill(); } }
  });
  c.restore();
  return { X, Y };
}

// Pendulum seen from the side: +q swings the hanging end toward -x (left), as in lesson 3.
function pend(c, P, L, q, target, col) {
  if (target !== null) { c.setLineDash([5, 4]); c.strokeStyle = css('--yellow'); c.lineWidth = 1.5; c.beginPath(); c.moveTo(...P); c.lineTo(P[0] - Math.sin(target) * L * 1.1, P[1] + Math.cos(target) * L * 1.1); c.stroke(); c.setLineDash([]); text(c, 'target', P[0] - Math.sin(target) * L * 1.15, P[1] + Math.cos(target) * L * 1.15, css('--yellow'), 10, 'center'); }
  c.setLineDash([3, 5]); c.strokeStyle = css('--dim'); c.lineWidth = 1; c.beginPath(); c.moveTo(...P); c.lineTo(P[0], P[1] + L * 1.05); c.stroke(); c.setLineDash([]);
  c.strokeStyle = col || css('--teal'); c.lineWidth = Math.max(6, L * 0.14); c.lineCap = 'round'; c.beginPath(); c.moveTo(...P); c.lineTo(P[0] - Math.sin(q) * L, P[1] + Math.cos(q) * L); c.stroke();
  c.fillStyle = css('--purple'); c.beginPath(); c.arc(...P, 5, 0, 7); c.fill();
}

// ======== SANDBOX 8 ========
(() => {
  const ids = ['s8Kp', 's8Kd', 's8Target', 's8H'];
  const DEF = { s8Kp: 400, s8Kd: 40, s8Target: 1.0, s8H: 0.002 };
  const SX = 200, SY = 64, SL = 120, PX0 = 40, PY0 = 250, PW = 345, PH = 170;
  let token = 0;
  const val = id => +$(id).value;
  function labels() {
    $('s8KpVal').textContent = val('s8Kp').toFixed(0); $('s8KdVal').textContent = val('s8Kd').toFixed(0);
    $('s8TargetVal').textContent = val('s8Target').toFixed(2); $('s8HVal').textContent = val('s8H').toFixed(3);
  }
  function setPend(q) {
    $('s8Pend').setAttribute('transform', `translate(${SX}, ${SY}) rotate(${(q * 180 / Math.PI).toFixed(3)})`);
  }
  function setTargetLine(tg) {
    $('s8TargetLine').setAttribute('x2', (SX - Math.sin(tg) * SL * 1.15).toFixed(1));
    $('s8TargetLine').setAttribute('y2', (SY + Math.cos(tg) * SL * 1.15).toFixed(1));
  }
  function ymap(y, ymax) { return PY0 + PH - Math.max(-0.2, Math.min(ymax, y)) / ymax * PH; }
  function run(instant) {
    labels();
    const kp = val('s8Kp'), kd = val('s8Kd'), tg = val('s8Target'), h = val('s8H'), gc = $('s8Gc').checked;
    const r = l8Run(kp, kd, tg, h, gc, 3), m = l8Metrics(r, kp, tg, gc);
    const ymax = Math.max(2.5, tg * 1.25);
    setTargetLine(tg);
    $('s8TargetY').setAttribute('y1', ymap(tg, ymax)); $('s8TargetY').setAttribute('y2', ymap(tg, ymax));
    $('s8QsY').setAttribute('y1', ymap(m.qs, ymax)); $('s8QsY').setAttribute('y2', ymap(m.qs, ymax));
    $('s8YMax').textContent = ymax.toFixed(1) + ' rad';
    const pts = r.t.map((t, i) => `${(PX0 + t / 3 * PW).toFixed(1)},${ymap(r.q[i], ymax).toFixed(1)}`);
    const blew = r.q.some(x => !isFinite(x) || Math.abs(x) > 6);
    $('s8Over').textContent = m.overshoot.toFixed(1); $('s8Settle').textContent = f3(m.settle);
    $('s8Err').textContent = isFinite(m.ssErr) ? z(m.ssErr).toFixed(4) : 'NaN'; $('s8Peak').textContent = m.peak.toFixed(1);
    $('s8Rise').textContent = m.rise === null ? 'never' : m.rise.toFixed(3); $('s8Qs').textContent = m.qs.toFixed(4);
    $('s8Grow').textContent = gc ? l8Growth(kp, kd, h).toFixed(4) : 'gravity comp off';
    $('s8Note').textContent = blew ? 'the angle left the plot: unstable' : (m.peak >= L8.GEAR - 1e-9 ? 'the motor hit its 100 N m limit (ctrl = 1)' : '');
    const my = ++token, N = pts.length;
    if (RM || instant === true) { $('s8Curve').setAttribute('points', pts.join(' ')); setPend(r.q[N - 1]); $('s8Clock').textContent = 't = ' + r.t[N - 1].toFixed(2) + ' s'; return; }
    const t0 = performance.now();
    (function frame(now) {
      if (my !== token) return;
      const tt = Math.max(0, Math.min(3, (now - t0) / 1000)), k = Math.min(N - 1, Math.round(tt / h));
      $('s8Curve').setAttribute('points', pts.slice(0, k + 1).join(' '));
      setPend(r.q[k]); $('s8Clock').textContent = 't = ' + r.t[k].toFixed(2) + ' s';
      if (tt < 3) requestAnimationFrame(frame);
    })(t0);
  }
  ids.forEach(id => $(id).addEventListener('input', labels));
  $('s8Run').addEventListener('click', () => run(false));
  $('s8Reset').addEventListener('click', () => { token++; ids.forEach(id => { $(id).value = DEF[id]; }); $('s8Gc').checked = false; run(true); });
  run(true);
})();

// ======== LESSON 8 EXPLAINER ========
(() => {
  // Only redraw this lesson's canvases while it is on screen (hidden canvases have zero size).
  const addEventListener = (type, fn, opts) => window.addEventListener(type, e => { if (document.getElementById('lesson8').classList.contains('active')) fn(e); }, opts);

  /* ---------- Section 1: three controllers chase the target ---------- */
  (() => {
    const cv = $('l8cvIdea'); let g = fit(cv, 0.62);
    const MODES = { l8IdeaP: [400, 0, false, 'P only'], l8IdeaPD: [400, 112, false, 'P + D'], l8IdeaPDG: [400, 112, true, 'P + D + gravity help'] };
    let r = null, mode = null, k = 0, token = 0;
    function draw() {
      const { ctx: c, w, h } = g; c.clearRect(0, 0, w, h);
      const L = Math.min(h * 0.55, w * 0.17), P = [w * 0.25, h * 0.12];
      pend(c, P, L, r ? r.q[k] : 0, 1.0);
      const box = [w * 0.47, 16, w * 0.5, h - 48];
      const series = r ? [{ x: r.t, y: r.q, col: css('--teal'), n: k + 1 }] : [];
      plot(c, box, [0, 3], [0, 2], series, [{ y: 1, col: css('--yellow'), label: 'target 1.0 rad' }], 'angle (rad)');
      if (r) {
        text(c, MODES[mode][3], 10, h - 34, css('--bright'), 12);
        text(c, 't = ' + r.t[k].toFixed(2) + ' s   angle = ' + r.q[k].toFixed(3) + ' rad   motor = ' + r.f[Math.min(k, r.f.length - 1)].toFixed(1) + ' N m', 10, h - 14, css('--dim'), 11);
      } else text(c, 'press a button below', 10, h - 14, css('--dim'), 11);
    }
    Object.keys(MODES).forEach(id => $(id).addEventListener('click', () => {
      const [kp, kd, gc] = MODES[id]; mode = id; r = l8Run(kp, kd, 1.0, 0.002, gc, 3); const my = ++token;
      if (RM) { k = r.t.length - 1; draw(); return; }
      const t0 = performance.now();
      (function frame(now) { if (my !== token) return; const tt = Math.max(0, Math.min(3, (now - t0) / 1000)); k = Math.round(tt / 0.002); draw(); if (tt < 3) requestAnimationFrame(frame); })(t0);
    }));
    addEventListener('resize', () => { g = fit(cv, 0.62); draw(); }); addEventListener('themechange', draw);
    draw();
  })();

  /* ---------- Section 2: one turn of the control loop, with real numbers ---------- */
  (() => {
    const items = [...document.querySelectorAll('#l8LoopList li')];
    const r = l8Run(400, 40, 1.0, 0.002, false, 0.01);
    let turn = 0, stage = 0;
    function render() {
      items.forEach((li, i) => { li.style.opacity = (i < stage) ? 1 : 0.45; li.style.fontWeight = i === stage - 1 ? '600' : ''; });
      const q = r.q[turn], v = r.v[turn], u = r.u[turn], L = [`Turn ${turn}, time = ${r.t[turn].toFixed(3)} s`];
      if (stage >= 1) L.push(`1. read:     qpos[0] = ${q.toFixed(6)} rad,  qvel[0] = ${v.toFixed(6)} rad/s`);
      if (stage >= 2) L.push(`2. compute:  tau = 400 x (1.0 - ${q.toFixed(6)}) - 40 x ${v.toFixed(6)} = ${u.toFixed(4)} N m\n             ctrl[0] = clip(tau / 100, -1, 1) = clip(${(u / 100).toFixed(4)}) = ${Math.max(-1, Math.min(1, u / 100)).toFixed(4)}`);
      if (stage >= 3) L.push(`3. mj_step:  torque = 100 x ctrl = ${r.f[turn].toFixed(4)} N m\n             qpos[0] -> ${r.q[turn + 1].toFixed(6)} rad,  qvel[0] -> ${r.v[turn + 1].toFixed(6)} rad/s\n             time -> ${r.t[turn + 1].toFixed(3)} s`);
      if (stage === 0) L.push('Press "Next stage".');
      $('l8LoopEq').textContent = L.join('\n');
      $('l8LoopNext').textContent = stage >= 3 ? 'Next turn' : 'Next stage';
    }
    $('l8LoopNext').addEventListener('click', () => { if (stage >= 3) { if (turn < 2) { turn++; stage = 0; } else { turn = 0; stage = 0; } } else stage++; render(); });
    $('l8LoopReset').addEventListener('click', () => { turn = 0; stage = 0; render(); });
    render();
  })();

  /* ---------- Section 3: step-response explorer (kp, kd) ---------- */
  (() => {
    const cv = $('l8cvStep'); let g = fit(cv, 0.55);
    function draw() {
      const kp = +$('l8Kp').value, kd = +$('l8Kd').value;
      $('l8oKp').textContent = kp.toFixed(0) + ' N m/rad'; $('l8oKd').textContent = kd.toFixed(0) + ' N m s/rad';
      const r = l8Run(kp, kd, 1.0, 0.002, false, 3), m = l8Metrics(r, kp, 1.0, false);
      const { ctx: c, w, h } = g; c.clearRect(0, 0, w, h);
      const box = [40, 14, w - 54, h - 44];
      const { X, Y } = plot(c, box, [0, 3], [0, 2], [{ x: r.t, y: r.q, col: css('--teal') }],
        [{ y: 1, col: css('--yellow'), label: 'target 1.0 rad' }, { y: m.qs, col: css('--dim'), label: 'balance q* ' + m.qs.toFixed(3) }], 'angle (rad)');
      c.globalAlpha = 0.18; c.fillStyle = css('--purple'); c.fillRect(box[0], Y(m.qs + 0.02), box[2], Y(m.qs - 0.02) - Y(m.qs + 0.02)); c.globalAlpha = 1;
      if (m.overshoot > 0.05) { const i = r.q.indexOf(m.qmax); c.fillStyle = css('--orange'); c.beginPath(); c.arc(X(r.t[i]), Y(m.qmax), 5, 0, 7); c.fill(); text(c, 'overshoot ' + m.overshoot.toFixed(1) + ' %', X(r.t[i]) + 8, Y(m.qmax) - 10, css('--orange'), 11); }
      if (m.settle !== null) { c.strokeStyle = css('--purple'); c.lineWidth = 2; c.beginPath(); c.moveTo(X(m.settle), box[1]); c.lineTo(X(m.settle), box[1] + box[3]); c.stroke(); text(c, 'settled ' + m.settle.toFixed(3) + ' s', X(m.settle) + 6, box[1] + box[3] - 12, css('--purple'), 11); }
      const kc = 2 * Math.sqrt(kp * L8.I);
      $('l8rZeta').textContent = (kd / kc).toFixed(3); $('l8rRise').textContent = m.rise === null ? 'never' : m.rise.toFixed(3);
      $('l8rOver').textContent = m.overshoot.toFixed(1); $('l8rSettle').textContent = f3(m.settle);
      $('l8rErr').textContent = m.ssErr.toFixed(4); $('l8rSat').textContent = (r.u.filter(x => Math.abs(x) >= L8.GEAR).length * 0.002).toFixed(3); $('l8rCrit').textContent = kc.toFixed(2);
    }
    ['l8Kp', 'l8Kd'].forEach(id => $(id).addEventListener('input', draw));
    document.querySelectorAll('.l8kdpre').forEach(b => b.addEventListener('click', () => { $('l8Kp').value = 400; $('l8Kd').value = b.dataset.kd; draw(); }));
    addEventListener('resize', () => { g = fit(cv, 0.55); draw(); }); addEventListener('themechange', draw);
    draw();
  })();

  /* ---------- Section 4: steady-state error and gravity compensation ---------- */
  (() => {
    const cv = $('l8cvSS'); let g = fit(cv, 0.5);
    function draw() {
      const kp = +$('l8SKp').value; $('l8oSKp').textContent = kp.toFixed(0) + ' N m/rad';
      const a = l8Run(kp, 112, 1.0, 0.002, false, 3), b = l8Run(kp, 112, 1.0, 0.002, true, 3), qs = l8QStar(kp, 1.0, false);
      const { ctx: c, w, h } = g; c.clearRect(0, 0, w, h);
      plot(c, [40, 14, w - 54, h - 44], [0, 3], [0, 1.4], [{ x: a.t, y: a.q, col: css('--orange') }, { x: b.t, y: b.q, col: css('--teal') }],
        [{ y: 1, col: css('--yellow'), label: 'target 1.0 rad' }, { y: qs, col: css('--dim'), label: 'q* ' + qs.toFixed(3) }], 'angle (rad)');
      text(c, 'PD + gravity comp', w - 20, h - 46, css('--teal'), 11, 'right'); text(c, 'PD only', w - 20, h - 62, css('--orange'), 11, 'right');
      $('l8sGuess').textContent = (L8.MGD * Math.sin(1) / kp).toFixed(4); $('l8sBal').textContent = (1 - qs).toFixed(4);
      $('l8sPD').textContent = (1 - a.q[a.q.length - 1]).toFixed(4); $('l8sGC').textContent = z(1 - b.q[b.q.length - 1]).toFixed(6);
    }
    $('l8SKp').addEventListener('input', draw);
    document.querySelectorAll('.l8kppre').forEach(b => b.addEventListener('click', () => { $('l8SKp').value = b.dataset.kp; draw(); }));
    addEventListener('resize', () => { g = fit(cv, 0.5); draw(); }); addEventListener('themechange', draw);
    draw();
  })();

  /* ---------- Section 5: timestep and stability ---------- */
  (() => {
    const cv = $('l8cvDt'); let g = fit(cv, 0.62), H = 0.002;
    const ref = l8Run(400, 400, 1.0, 0.002, true, 10);
    function draw() {
      $('l8oH').textContent = H.toFixed(3) + ' s';
      const lim = !$('l8NoLim').checked, r = l8Run(400, 400, 1.0, H, true, 10, lim), { ctx: c, w, h } = g; c.clearRect(0, 0, w, h);
      const top = [44, 14, w - 58, (h - 70) * 0.62], bot = [44, top[1] + top[3] + 26, w - 58, (h - 70) * 0.38];
      plot(c, top, [0, 10], [0, 1.6], [{ x: ref.t, y: ref.q, col: css('--dim'), w: 2 }, { x: r.t, y: r.q, col: css('--teal'), dots: true }], [{ y: 1, col: css('--yellow'), label: 'target' }], 'angle (rad)');
      plot(c, bot, [0, 10], [-120, 170], [{ x: r.t.slice(1), y: r.f, col: css('--orange'), w: 1.5 }], lim ? [{ y: 100, col: css('--red') }, { y: -100, col: css('--red') }] : [], 'motor torque (N m)');
      const test = 400 * H * H / L8.I + 2 * 400 * H / L8.I, gr = l8Growth(400, 400, H);
      let lo = Infinity, hi = -Infinity, off = null;
      r.f.forEach((x, i) => { if (r.t[i + 1] > 8 + 1e-9) { lo = Math.min(lo, x); hi = Math.max(hi, x); } });
      r.q.forEach((x, i) => { if (off === null && Math.abs(x - 1) > 1) off = r.t[i]; });
      const ended = r.t[r.t.length - 1] < 10 - 1e-9, swing = ended ? Infinity : hi - lo;
      const verdict = off !== null ? 'EXPLODES' : (swing < 1 ? 'stable' : 'BUZZING');
      $('l8dN').textContent = Math.round(10 / H); $('l8dKd').textContent = (400 * H / L8.I).toFixed(4); $('l8dKp').textContent = (400 * H * H / L8.I).toFixed(5);
      $('l8dTest').textContent = test.toFixed(3); $('l8dG').textContent = gr.toFixed(4);
      $('l8dSw').textContent = isFinite(swing) ? swing.toFixed(2) : '-'; $('l8dQ').textContent = ended ? 'left the chart' : r.q[r.q.length - 1].toFixed(4);
      $('l8dV').textContent = verdict; $('l8dV').style.color = verdict === 'stable' ? 'var(--teal)' : 'var(--red)';
      const msg = verdict === 'stable' ? 'stable: wobbles shrink' : (verdict === 'BUZZING' ? 'buzzing: motor slams between its limits' : 'no limit: wobble passes 1 rad at ' + off.toFixed(2) + ' s');
      text(c, msg, top[0] + top[2] - 6, top[1] + top[3] - 12, verdict === 'stable' ? css('--teal') : css('--red'), 11, 'right');
    }
    const toX = v => Math.round(v * 1000);
    $('l8H').addEventListener('input', () => { H = +$('l8H').value / 1000; draw(); });
    $('l8NoLim').addEventListener('change', draw);
    document.querySelectorAll('.l8hpre').forEach(b => b.addEventListener('click', () => { H = +b.dataset.h; $('l8H').value = toX(H); draw(); }));
    addEventListener('resize', () => { g = fit(cv, 0.62); draw(); }); addEventListener('themechange', draw);
    draw();
  })();

  /* ---------- Section 7: plot the logged run (what a CSV plotter would show) ---------- */
  (() => {
    const cv = $('l8cvLog'); let g = fit(cv, 0.6);
    const r = l8Run(400, 40, 1.0, 0.002, false, 3);
    function draw() {
      const { ctx: c, w, h } = g; c.clearRect(0, 0, w, h);
      const top = [44, 14, w - 58, (h - 64) * 0.55], bot = [44, top[1] + top[3] + 24, w - 58, (h - 64) * 0.45];
      plot(c, top, [0, 3], [0, 1.6], [{ x: r.t, y: r.q, col: css('--teal') }], [{ y: 1, col: css('--yellow'), label: 'target_rad' }], 'qpos_rad');
      plot(c, bot, [0, 3], [-1.3, 1.7], [{ x: r.t.slice(0, -1), y: r.f.map(x => x / L8.GEAR), col: css('--orange') }], [{ y: 1, col: css('--red'), label: 'ctrlrange +1' }, { y: -1, col: css('--red'), label: '-1' }], 'ctrl');
    }
    addEventListener('resize', () => { g = fit(cv, 0.6); draw(); }); addEventListener('themechange', draw);
    draw();
  })();
})();
})();
