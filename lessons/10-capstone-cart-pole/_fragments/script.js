// ======== CAPSTONE 1 (LESSON 10): CART-POLE (shared core, sandbox 10, explainer) ========
(() => {
// SIM-CORE-BEGIN
// Same model as 10_capstone_cart_pole.py. The four masses below are read from MuJoCo 3.15.0 (mj_fullM at theta = 0
// and body_mass). One call of c10Step is exactly one MuJoCo Euler step for this model (no damping, no contacts):
// ctrl from the state before the step, clipped to [-1, 1]; force on the cart = 20 * ctrl N; optional torque on the
// hinge (qfrc_applied[1]); solve the 2 x 2 mass matrix for qacc; qvel += h*qacc; qpos += h*qvel (the NEW qvel).
const C10 = {
  H: 0.002, GEAR: 20, RAIL: 1.0, LP: 0.6,
  M00: 1.7874925584998416,   // cart + pole mass, kg
  MPL: 0.2362477675499525,   // pole mass x CoM distance (0.3 m), kg m
  I: 0.09674128264079906,    // pole inertia about the hinge, kg m^2
  MGL: 2.3175905996650337,   // pole mass x g x CoM distance, N m
  MP: 0.7874925584998416,    // pole mass, kg
  // LQR knobs [cart offset /m, lean /rad, cart speed /(m/s), lean speed /(rad/s)]: ctrl = k . [x, th, xd, thd]
  LQR: [0.9826863975287924, 4.140379609173346, 0.8076247143640032, 0.7561296253666459]
};
function c10Step(s, k, push) {
  const u = k ? k[0] * s[0] + k[1] * s[1] + k[2] * s[2] + k[3] * s[3] : 0;   // ctrl asked
  const c = Math.max(-1, Math.min(1, u));                                    // ctrlrange -1 1
  const cs = Math.cos(s[1]), sn = Math.sin(s[1]);
  const a = C10.M00, b = C10.MPL * cs, d = C10.I;                            // mass matrix M(theta)
  const r0 = C10.GEAR * c + C10.MPL * sn * s[3] * s[3];                       // cart: motor + spin term
  const r1 = C10.MGL * sn + (push || 0);                                      // hinge: gravity + push
  const det = a * d - b * b;
  const ax = (r0 * d - b * r1) / det, ath = (a * r1 - b * r0) / det;          // qacc = M^-1 rhs
  s[2] += C10.H * ax; s[3] += C10.H * ath;                                    // speeds first
  s[0] += C10.H * s[2]; s[1] += C10.H * s[3];                                 // then positions
  return u;
}
// Run from rest at tilt th0. push = [hinge torque N m, duration s] from t = 0. Stops at the rail if stopRail.
function c10Run(k, th0, T, push, stopRail) {
  const n = Math.round(T / C10.H), s = [0, th0, 0, 0];
  const t = [0], x = [0], th = [th0], xd = [0], u = [];
  let rail = null;
  for (let i = 0; i < n; i++) {
    const p = push && i * C10.H < push[1] - 1e-9 ? push[0] : 0;
    u.push(c10Step(s, k, p));
    t.push((i + 1) * C10.H); x.push(s[0]); th.push(s[1]); xd.push(s[2]);
    if (rail === null && Math.abs(s[0]) >= C10.RAIL) { rail = (i + 1) * C10.H; if (stopRail) break; }
  }
  return { t, x, th, xd, u, rail };
}
// SIM-CORE-END

const $ = id => document.getElementById(id);
function c10Metrics(r) {
  let last = -1, xmax = 0, umax = 0;
  for (let i = 0; i < r.t.length; i++) {
    if (Math.abs(r.x[i]) > 0.01 || Math.abs(r.th[i]) > 0.01) last = i;
    if (Math.abs(r.x[i]) > Math.abs(xmax)) xmax = r.x[i];
  }
  r.u.forEach(v => { if (Math.abs(v) > umax) umax = Math.abs(v); });
  const settle = last < 0 ? 0 : (last === r.t.length - 1 ? null : r.t[last + 1]);
  return { settle, xmax, umax };
}

// Cart-pole seen from the side (+x to the right, +z up). S = px per metre; (cx, cy) = rail centre on screen.
function c10Draw(c, cx, cy, S, x, th, o) {
  o = o || {};
  const dim = css('--dim'), X = cx + x * S;
  c.strokeStyle = css('--grid'); c.lineWidth = 3; c.beginPath(); c.moveTo(cx - 1.2 * S, cy); c.lineTo(cx + 1.2 * S, cy); c.stroke();
  c.strokeStyle = css('--red'); c.lineWidth = 3;
  [-1, 1].forEach(e => { c.beginPath(); c.moveTo(cx + e * S, cy - 9); c.lineTo(cx + e * S, cy + 9); c.stroke(); });
  if (o.labels !== false) { text(c, '-1 m', cx - S, cy + 18, dim, 10, 'center'); text(c, '+1 m', cx + S, cy + 18, dim, 10, 'center'); text(c, '0', cx, cy + 18, dim, 10, 'center'); }
  c.strokeStyle = dim; c.lineWidth = 1; c.beginPath(); c.moveTo(cx, cy - 4); c.lineTo(cx, cy + 4); c.stroke();
  if (o.com !== undefined) { c.fillStyle = css('--purple'); c.beginPath(); c.moveTo(cx + o.com * S, cy + 6); c.lineTo(cx + o.com * S - 6, cy + 14); c.lineTo(cx + o.com * S + 6, cy + 14); c.closePath(); c.fill(); }
  const cw = Math.max(0.2 * S, 18), ch = Math.max(0.1 * S, 10);
  c.fillStyle = css('--blue'); c.globalAlpha = 0.85; c.fillRect(X - cw / 2, cy - ch / 2, cw, ch); c.globalAlpha = 1;
  const L = C10.LP * S, tx = X + Math.sin(th) * L, ty = cy - Math.cos(th) * L;
  c.setLineDash([3, 5]); c.strokeStyle = dim; c.lineWidth = 1; c.beginPath(); c.moveTo(X, cy); c.lineTo(X, cy - L * 1.08); c.stroke(); c.setLineDash([]);
  c.strokeStyle = o.col || css('--teal'); c.lineWidth = Math.max(4, 0.04 * S); c.lineCap = 'round'; c.beginPath(); c.moveTo(X, cy); c.lineTo(tx, ty); c.stroke();
  c.fillStyle = css('--purple'); c.beginPath(); c.arc(X, cy, 4, 0, 7); c.fill();
  if (o.force) { const fl = o.force * 2.4; if (Math.abs(fl) > 2) arrow(c, X, cy, X + fl, cy, css('--orange'), 3, 8); }
  if (o.push) arrow(c, tx - Math.sign(o.push) * 34, ty, tx - Math.sign(o.push) * 6, ty, css('--purple'), 3, 8);
  return { X, tx, ty };
}
// Simple time plot. series: [{t, y, col, n}], guides: [{y, col, label}]
function c10Plot(c, box, tr, yr, series, guides, ylab) {
  const [x0, y0, w, h] = box, X = t => x0 + (t - tr[0]) / (tr[1] - tr[0]) * w, Y = y => y0 + h - (y - yr[0]) / (yr[1] - yr[0]) * h;
  c.strokeStyle = css('--grid'); c.lineWidth = 1; c.strokeRect(x0, y0, w, h);
  const st = tr[1] - tr[0] > 3 ? 1 : 0.25;
  for (let t = tr[0]; t <= tr[1] + 1e-9; t += st) { c.beginPath(); c.moveTo(X(t), y0); c.lineTo(X(t), y0 + h); c.stroke(); text(c, (+t.toFixed(2)) + ' s', X(t), y0 + h + 10, css('--dim'), 9, 'center'); }
  text(c, String(yr[1]), x0 - 4, y0 + 6, css('--dim'), 9, 'right'); text(c, String(yr[0]), x0 - 4, y0 + h - 6, css('--dim'), 9, 'right');
  if (ylab) text(c, ylab, x0 + 6, y0 + 10, css('--dim'), 10);
  (guides || []).forEach(gd => { c.setLineDash([5, 4]); c.strokeStyle = gd.col; c.lineWidth = 1.2; c.beginPath(); c.moveTo(x0, Y(gd.y)); c.lineTo(x0 + w, Y(gd.y)); c.stroke(); c.setLineDash([]); if (gd.label) text(c, gd.label, x0 + w - 4, Y(gd.y) - 8, gd.col, 10, 'right'); });
  c.save(); c.beginPath(); c.rect(x0, y0 - 1, w, h + 2); c.clip();
  series.forEach(s => {
    const n = s.n === undefined ? s.t.length : s.n, stride = Math.max(1, Math.floor(n / 800));
    c.strokeStyle = s.col; c.lineWidth = 2; c.beginPath();
    for (let i = 0; i < n; i += stride) { const yy = Y(Math.max(yr[0] - 1, Math.min(yr[1] + 1, s.y[i]))); i ? c.lineTo(X(s.t[i]), yy) : c.moveTo(X(s.t[i]), yy); }
    c.stroke();
  });
  c.restore();
  return { X, Y };
}

// ======== SANDBOX 10 ========
(() => {
  const KN = ['s10KPos', 's10KLean', 's10KVel', 's10KSpin'], S = 160, CX = 200, CY = 200;
  const PX0 = 40, PY0 = 268, PW = 345, PH = 150, WIN = 6;
  let k = C10.LQR.slice(), st = [0, 0.1, 0, 0], t = 0, steps = 0, pushLeft = 0, pushTau = 0, lastU = 0, ended = '', playing = false, raf = 0;
  let hist = [];
  const on = () => $('s10On').checked;
  function knobLabels() { KN.forEach((id, i) => { $(id + 'Val').textContent = k[i].toFixed(4); }); }
  function setStatus(msg, col) { $('s10Status').textContent = msg; $('s10Status').setAttribute('fill', col || 'var(--text-dim)'); $('s10State').textContent = msg; }
  function render() {
    const X = CX + st[0] * S;
    $('s10Cart').setAttribute('transform', `translate(${X.toFixed(2)}, ${CY})`);
    $('s10Pole').setAttribute('transform', `rotate(${(st[1] * 180 / Math.PI).toFixed(3)})`);
    const f = Math.max(-1, Math.min(1, lastU)) * C10.GEAR * 2.4;
    const fa = $('s10Force');
    if (Math.abs(f) > 2) { fa.style.display = ''; fa.setAttribute('x2', f.toFixed(1)); } else fa.style.display = 'none';
    const pa = $('s10PushArrow');
    if (pushLeft > 0) { pa.style.display = ''; const sx = -Math.sign(pushTau) * 30; pa.setAttribute('x1', sx); pa.setAttribute('x2', -Math.sign(pushTau) * 6); } else pa.style.display = 'none';
    $('s10Clock').textContent = `t = ${t.toFixed(3)} s`;
    $('s10X').textContent = st[0].toFixed(4); $('s10Th').textContent = st[1].toFixed(4);
    $('s10Xd').textContent = st[2].toFixed(4); $('s10Thd').textContent = st[3].toFixed(4);
    $('s10U').textContent = lastU.toFixed(4) + (Math.abs(lastU) > 1 ? ' (clipped to ' + Math.sign(lastU).toFixed(0) + ')' : '');
    $('s10T').textContent = t.toFixed(3);
    const t0 = Math.max(0, t - WIN), pts = th => hist.filter(p => p[0] >= t0).map(p => `${(PX0 + (p[0] - t0) / WIN * PW).toFixed(1)},${(PY0 + PH / 2 - Math.max(-1, Math.min(1, th(p))) * PH / 2).toFixed(1)}`).join(' ');
    $('s10TrTh').setAttribute('points', pts(p => p[2] / 0.3));
    $('s10TrX').setAttribute('points', pts(p => p[1] / 1.0));
    $('s10TLeft').textContent = t0.toFixed(1) + ' s'; $('s10TRight').textContent = (t0 + WIN).toFixed(1) + ' s';
  }
  function stepOnce() {
    if (ended) return false;
    const p = pushLeft > 0 ? pushTau : 0;
    lastU = c10Step(st, on() ? k : null, p);
    if (pushLeft > 0) pushLeft--;
    steps++; t = steps * C10.H;
    if (steps % 5 === 0) hist.push([t, st[0], st[1]]);
    if (Math.abs(st[0]) >= C10.RAIL) { ended = 'rail'; setStatus('cart hit the rail stop at ' + t.toFixed(3) + ' s: stopped', 'var(--red)'); return false; }
    if (Math.abs(st[1]) > Math.PI / 2) { ended = 'fell'; setStatus('pole fell past horizontal at ' + t.toFixed(3) + ' s: stopped', 'var(--red)'); return false; }
    if (Math.abs(st[1]) > 0.2) setStatus('leaning more than 0.2 rad (the env would end here)', 'var(--orange)');
    else setStatus(on() ? 'balancing' : 'controller off', on() ? 'var(--teal)' : 'var(--text-dim)');
    return true;
  }
  function stop() { playing = false; cancelAnimationFrame(raf); $('s10Play').textContent = 'Play'; }
  function play() {
    if (playing) { stop(); return; }
    if (ended) reset();
    if (RM) { for (let i = 0; i < 1000; i++) if (!stepOnce()) break; render(); return; }
    playing = true; $('s10Play').textContent = 'Pause';
    let last = performance.now();
    (function frame(now) {
      if (!playing) return;
      const n = Math.min(40, Math.max(1, Math.round((now - last) / 1000 / C10.H))); last = now;
      for (let i = 0; i < n; i++) if (!stepOnce()) { stop(); break; }
      render();
      if (playing) raf = requestAnimationFrame(frame);
    })(last);
  }
  function reset() {
    stop();
    st = [0, +$('s10Tilt').value, 0, 0]; t = 0; steps = 0; pushLeft = 0; lastU = 0; ended = ''; hist = [[0, 0, st[1]]];
    setStatus(on() ? 'ready: press Play' : 'controller off: press Play', 'var(--text-dim)');
    render();
  }
  function push(dir) { if (ended) return; pushTau = dir * 1.0; pushLeft = 50; render(); if (!playing) play(); }
  KN.forEach((id, i) => $(id).addEventListener('input', () => { k[i] = +$(id).value; knobLabels(); }));
  $('s10Gains').addEventListener('click', () => { k = C10.LQR.slice(); KN.forEach((id, i) => { $(id).value = k[i]; }); knobLabels(); });
  $('s10Tilt').addEventListener('input', () => { $('s10TiltVal').textContent = (+$('s10Tilt').value).toFixed(2); if (steps === 0) reset(); });
  $('s10On').addEventListener('change', () => { if (steps === 0) reset(); });
  $('s10Play').addEventListener('click', play);
  $('s10Step').addEventListener('click', () => { stop(); stepOnce(); render(); });
  $('s10Reset').addEventListener('click', reset);
  $('s10PushL').addEventListener('click', () => push(-1));
  $('s10PushR').addEventListener('click', () => push(1));
  KN.forEach((id, i) => { $(id).value = k[i]; });
  knobLabels(); reset();
})();

// ======== LESSON 10 EXPLAINER ========
(() => {
  // Only redraw this lesson's canvases while it is on screen (hidden canvases have zero size).
  const addEventListener = (type, fn, opts) => window.addEventListener(type, e => { if (document.getElementById('lesson10').classList.contains('active')) fn(e); }, opts);
  const LEAN_ONLY = [0, C10.LQR[1], 0, C10.LQR[3]];

  // Animate a recorded run over its real duration; draw(k) is called with the current sample index.
  function animate(r, draw, tokenRef) {
    const my = ++tokenRef.v, N = r.t.length, T = r.t[N - 1];
    if (RM) { draw(N - 1); return; }
    const t0 = performance.now();
    (function frame(now) {
      if (my !== tokenRef.v) return;
      const tt = Math.min(T, (now - t0) / 1000); draw(Math.min(N - 1, Math.round(tt / C10.H)));
      if (tt < T) requestAnimationFrame(frame);
    })(t0);
  }

  /* ---------- Section 1: three ways to treat the broom ---------- */
  (() => {
    const cv = $('l10cvIdea'); let g = fit(cv, 0.7);
    const MODES = { l10IdeaFree: [null, 1.0, 'no control: let go'], l10IdeaLean: [LEAN_ONLY, 6, 'watch the lean only'], l10IdeaFull: [C10.LQR, 6, 'watch the lean and the cart'] };
    let r = null, mode = null, k = 0; const tok = { v: 0 };
    function draw() {
      const { ctx: c, w, h } = g; c.clearRect(0, 0, w, h);
      const S = Math.min(w / 2.7, h * 0.55), cy = h * 0.42;
      c10Draw(c, w / 2, cy, S, r ? r.x[k] : 0, r ? r.th[k] : 0.1, { force: r && MODES[mode][0] ? Math.max(-1, Math.min(1, r.u[Math.min(k, r.u.length - 1)])) * C10.GEAR : 0 });
      const box = [44, cy + 30, w - 58, h - cy - 52];
      const T = r ? r.t[r.t.length - 1] : 6;
      c10Plot(c, box, [0, Math.max(1, Math.ceil(T))], [-1, 1], r ? [{ t: r.t, y: r.x, col: css('--orange'), n: k + 1 }, { t: r.t, y: r.th, col: css('--teal'), n: k + 1 }] : [], [], 'teal: lean (rad)   orange: cart x (m)');
      if (r) text(c, MODES[mode][2] + '   t = ' + r.t[k].toFixed(2) + ' s   x = ' + r.x[k].toFixed(3) + ' m   lean = ' + r.th[k].toFixed(3) + ' rad' + (r.rail !== null && r.t[k] >= r.rail ? '   RAIL HIT' : ''), 10, 14, css('--bright'), 11);
      else text(c, 'start: lean 0.1 rad, at rest. Press a button.', 10, 14, css('--dim'), 11);
    }
    Object.keys(MODES).forEach(id => $(id).addEventListener('click', () => {
      const [kk, T] = MODES[id]; mode = id; r = c10Run(kk, 0.1, T, null, true);
      animate(r, i => { k = i; draw(); }, tok);
    }));
    addEventListener('resize', () => { g = fit(cv, 0.7); draw(); }); addEventListener('themechange', draw);
    draw();
  })();

  /* ---------- Section 2 (M1): pose the model, read qpos and sensordata ---------- */
  (() => {
    const cv = $('l10cvPose'); let g = fit(cv, 0.62);
    function draw() {
      const x = +$('l10PX').value, th = +$('l10PT').value;
      $('l10oPX').textContent = x.toFixed(2) + ' m'; $('l10oPT').textContent = th.toFixed(2) + ' rad';
      const { ctx: c, w, h } = g; c.clearRect(0, 0, w, h);
      const S = Math.min(w / 2.7, h * 0.9), cy = h * 0.8;
      const p = c10Draw(c, w / 2, cy, S, x, th);
      // world frame at the rail centre (the rail is 0.5 m above the world origin)
      arrow(c, w / 2, cy, w / 2 + 34, cy, css('--x'), 2, 6); text(c, 'x', w / 2 + 40, cy, css('--x'), 11);
      arrow(c, w / 2, cy, w / 2, cy - 34, css('--z'), 2, 6); text(c, 'z', w / 2 + 8, cy - 36, css('--z'), 11);
      c.fillStyle = css('--yellow'); c.beginPath(); c.arc(p.tx, p.ty, 5, 0, 7); c.fill();
      const tipx = x + C10.LP * Math.sin(th), tipz = 0.5 + C10.LP * Math.cos(th);
      text(c, 'tip site', p.tx + 8, p.ty - 10, css('--yellow'), 11);
      text(c, 'slider: qpos[0] = ' + x.toFixed(2) + ' m', 10, 14, css('--blue'), 11);
      text(c, 'hinge: qpos[1] = ' + th.toFixed(2) + ' rad', 10, 30, css('--teal'), 11);
      $('l10PoseSens').textContent = `sensordata, shape (7,)\n[0]   cart_x      ${x.toFixed(4)} m\n[1]   pole_angle  ${th.toFixed(4)} rad\n` +
        `[2]   cart_v      0 m/s\n[3]   pole_spin   0 rad/s\n[4:7] tip_pos     (${tipx.toFixed(4)}, 0, ${tipz.toFixed(4)}) m, world`;
    }
    ['l10PX', 'l10PT'].forEach(id => $(id).addEventListener('input', draw));
    addEventListener('resize', () => { g = fit(cv, 0.62); draw(); }); addEventListener('themechange', draw);
    draw();
  })();

  /* ---------- Section 3 (M2): let go and time the fall ---------- */
  (() => {
    const cv = $('l10cvFall'); let g = fit(cv, 0.72);
    let r = null, k = 0, cross = -1; const tok = { v: 0 };
    const com = i => (1.0 * r.x[i] + C10.MP * (r.x[i] + 0.3 * Math.sin(r.th[i]))) / C10.M00;
    function draw() {
      const { ctx: c, w, h } = g; c.clearRect(0, 0, w, h);
      const S = Math.min(w / 2.7, h * 0.55), cy = h * 0.42;
      c10Draw(c, w / 2, cy, S, r ? r.x[k] : 0, r ? r.th[k] : 0.1, { com: r ? com(k) : undefined });
      const box = [44, cy + 30, w - 58, h - cy - 52];
      const { X, Y } = c10Plot(c, box, [0, 0.6], [-1.5, 1.5], r ? [{ t: r.t, y: r.th, col: css('--teal'), n: k + 1 }, { t: r.t, y: r.x.map(v => v * 10), col: css('--orange'), n: k + 1 }] : [],
        [{ y: 0.5, col: css('--yellow'), label: '0.5 rad' }], 'teal: lean (rad)   orange: cart x (x10, m)');
      if (r && cross >= 0 && k >= cross) {
        c.strokeStyle = css('--purple'); c.lineWidth = 1.5; c.beginPath(); c.moveTo(X(r.t[cross]), box[1]); c.lineTo(X(r.t[cross]), box[1] + box[3]); c.stroke();
        text(c, r.t[cross].toFixed(3) + ' s', X(r.t[cross]) + 6, box[1] + 22, css('--purple'), 11);
      }
      text(c, 'purple triangle: centre of mass of cart + pole', 10, 14, css('--purple'), 10);
      if (r) {
        $('l10fT').textContent = cross >= 0 && k >= cross ? r.t[cross].toFixed(3) : '-';
        $('l10fX').textContent = cross >= 0 && k >= cross ? r.x[cross].toFixed(4) : r.x[k].toFixed(4);
        $('l10fCom').textContent = com(k).toFixed(4);
      }
    }
    function go(th0) {
      r = c10Run(null, th0, 0.6, null, false); cross = r.th.findIndex(v => Math.abs(v) > 0.5);
      animate(r, i => { k = i; draw(); }, tok);
    }
    $('l10Fall05').addEventListener('click', () => go(0.05)); $('l10Fall10').addEventListener('click', () => go(0.1));
    addEventListener('resize', () => { g = fit(cv, 0.72); draw(); }); addEventListener('themechange', draw);
    draw();
  })();

  /* ---------- Section 4 (M3): the mass matrix couples cart and pole ---------- */
  (() => {
    const cv = $('l10cvMass'); let g = fit(cv, 0.48);
    function draw() {
      const th = +$('l10MT').value, u = +$('l10MU').value;
      $('l10oMT').textContent = th.toFixed(2) + ' rad'; $('l10oMU').textContent = u.toFixed(2) + ' (' + (u * C10.GEAR).toFixed(1) + ' N)';
      const a = C10.M00, b = C10.MPL * Math.cos(th), d = C10.I, r0 = C10.GEAR * u, r1 = C10.MGL * Math.sin(th), det = a * d - b * b;
      const ax = (r0 * d - b * r1) / det, ath = (a * r1 - b * r0) / det;
      const { ctx: c, w, h } = g; c.clearRect(0, 0, w, h);
      const S = Math.min(w / 2.5, h * 1.15), cy = h * 0.8;
      const p = c10Draw(c, w / 2, cy, S, 0, th, { force: u * C10.GEAR, labels: false });
      // tip acceleration from the hinge alone: 0.6 * qacc[1], at right angles to the pole
      const sc = 2.2, tl = C10.LP * ath * sc;
      if (Math.abs(tl) > 2) arrow(c, p.tx, p.ty, p.tx + Math.cos(th) * tl, p.ty + Math.sin(th) * tl, css('--purple'), 2.5, 7);
      if (Math.abs(ax * sc * 4) > 2) arrow(c, w / 2, cy + 22, w / 2 + ax * sc * 4, cy + 22, css('--blue'), 2.5, 7);
      text(c, 'orange: force   blue: qacc[0]   purple: qacc[1] at the tip', 10, 14, css('--dim'), 10);
      $('l10mM01').textContent = b.toFixed(6); $('l10mDet').textContent = det.toFixed(6);
      $('l10mA0').textContent = fmt(ax, 6); $('l10mA1').textContent = fmt(ath, 6);
    }
    ['l10MT', 'l10MU'].forEach(id => $(id).addEventListener('input', draw));
    document.querySelectorAll('.l10mpre').forEach(bt => bt.addEventListener('click', () => { $('l10MT').value = bt.dataset.th; $('l10MU').value = bt.dataset.u; draw(); }));
    addEventListener('resize', () => { g = fit(cv, 0.48); draw(); }); addEventListener('themechange', draw);
    draw();
  })();

  /* ---------- Section 5 (M4): lean-only vs all four knobs, and a push ---------- */
  (() => {
    const cv = $('l10cvCtrl'); let g = fit(cv, 0.75);
    const MODES = {
      l10CLean: [LEAN_ONLY, 0.1, null, 'lean-only PD'],
      l10CFull: [C10.LQR, 0.1, null, 'all four knobs (LQR)'],
      l10CPush: [C10.LQR, 0.0, [1.0, 0.1], 'all four knobs, push +1.0 N m for 0.1 s']
    };
    let r = null, mode = null, k = 0; const tok = { v: 0 };
    function draw() {
      const { ctx: c, w, h } = g; c.clearRect(0, 0, w, h);
      const S = Math.min(w / 2.7, h * 0.5), cy = h * 0.36;
      const pushing = r && MODES[mode][2] && r.t[k] < 0.1 - 1e-9;
      c10Draw(c, w / 2, cy, S, r ? r.x[k] : 0, r ? r.th[k] : 0.1, { force: r ? Math.max(-1, Math.min(1, r.u[Math.min(k, r.u.length - 1)])) * C10.GEAR : 0, push: pushing ? 1 : 0 });
      const box = [44, cy + 30, w - 58, h - cy - 52];
      c10Plot(c, box, [0, 6], [-1.2, 1.2], r ? [{ t: r.t, y: r.x, col: css('--orange'), n: k + 1 }, { t: r.t, y: r.th.map(v => v * 4), col: css('--teal'), n: k + 1 }] : [],
        [{ y: 1, col: css('--red'), label: 'rail stop +1 m' }, { y: -1, col: css('--red') }], 'orange: cart x (m)   teal: lean (x4, rad)');
      if (r) {
        text(c, MODES[mode][3] + '   t = ' + r.t[k].toFixed(2) + ' s' + (r.rail !== null && r.t[k] >= r.rail ? '   RAIL HIT at ' + r.rail.toFixed(3) + ' s' : ''), 10, 14, css('--bright'), 11);
        if (k === r.t.length - 1) {
          const m = c10Metrics(r);
          $('l10cSettle').textContent = r.rail !== null ? 'never' : (m.settle === null ? 'not settled' : m.settle.toFixed(3));
          $('l10cX').textContent = fmt(m.xmax, 4); $('l10cU').textContent = m.umax.toFixed(4) + ' (' + (m.umax * C10.GEAR).toFixed(2) + ' N)';
          $('l10cRail').textContent = r.rail !== null ? r.rail.toFixed(3) + ' s' : 'no';
        }
      } else text(c, 'press a button below', 10, 14, css('--dim'), 11);
    }
    Object.keys(MODES).forEach(id => $(id).addEventListener('click', () => {
      const [kk, th0, push] = MODES[id]; mode = id; r = c10Run(kk, th0, 6, push, true);
      ['l10cSettle', 'l10cX', 'l10cU', 'l10cRail'].forEach(i => { $(i).textContent = '-'; });
      animate(r, i => { k = i; draw(); }, tok);
    }));
    addEventListener('resize', () => { g = fit(cv, 0.75); draw(); }); addEventListener('themechange', draw);
    draw();
  })();
})();
})();
