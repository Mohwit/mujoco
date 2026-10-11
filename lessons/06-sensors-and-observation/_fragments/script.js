// ======== SANDBOX 6: SENSOR DASHBOARD ========
(() => {
  // Same model as 06_sensors_and_observation.py. Constants read from MuJoCo 3.15 by check_browser_match.py:
  // m g d = 109.140436750183 N m, I about the hinge = 7.87639467749904 kg m^2.
  // sense() + the Euler update below match MuJoCo's sensordata, qpos and qvel to 1e-13 over 5000 steps.
  const MGD = 109.140436750183, INV_I = 0.126961641835542, G = 9.81, H = 0.002, GEAR = 100; // lesson 5 motor: gear 100, ctrlrange -1..1
  const sense = (q, v, u) => {
    const a = (-MGD * Math.sin(q) + GEAR * u) * INV_I;
    return { a, s: [q, v, u, -Math.sin(q), 0, 1 - Math.cos(q), -a - G * Math.sin(q), 0, G * Math.cos(q) + v * v] };
  };
  const SENS = [
    { name: 'angle', type: 'jointpos', adr: 0, dim: 1, unit: 'rad', frame: 'joint', col: '--teal' },
    { name: 'spin', type: 'jointvel', adr: 1, dim: 1, unit: 'rad/s', frame: 'joint', col: '--orange' },
    { name: 'motor_force', type: 'actuatorfrc', adr: 2, dim: 1, unit: 'ctrl units', frame: 'actuator', col: '--purple' },
    { name: 'tip_pos', type: 'framepos', adr: 3, dim: 3, unit: 'm', frame: 'world', col: '--blue' },
    { name: 'tip_acc', type: 'accelerometer', adr: 6, dim: 3, unit: 'm/s²', frame: 'tip site', col: '--yellow' },
  ];
  const CELL = ['angle', 'spin', 'motor', 'tip x', 'tip y', 'tip z', 'acc x', 'acc y', 'acc z'];
  const OWNER = [0, 1, 2, 3, 3, 3, 4, 4, 4];
  const NOISE = [0.01, 0.05, 0, 0, 0, 0, 0.2, 0.2, 0.2]; // model.sensor_noise repeated by sensor_dim
  const $ = id => document.getElementById(id);
  const f4 = v => (Math.abs(v) < 5e-5 ? 0 : v).toFixed(4);
  const gauss = () => { let u = 0, w = 0; while (u === 0) u = Math.random(); while (w === 0) w = Math.random(); return Math.sqrt(-2 * Math.log(u)) * Math.cos(2 * Math.PI * w); };
  const PX = 200, PY = 140, L = 115, NS = 'http://www.w3.org/2000/svg';

  const st = { q: 0.7, v: 0, t: 0, u: 0, sd: null, shown: null };
  let hi = -1, noise = false, playing = false, last = 0, acc = 0, tPlay = 0;

  // Build the nine sensordata cells once.
  const strip = $('s6Strip'), cells = [], vals = [], cw = 376 / 9;
  const mk = (tag, attrs, txt) => { const e = document.createElementNS(NS, tag); Object.entries(attrs).forEach(([k, v]) => e.setAttribute(k, v)); if (txt !== undefined) e.textContent = txt; strip.appendChild(e); return e; };
  for (let i = 0; i < 9; i++) {
    const x = 12 + i * cw;
    mk('text', { x: x + cw / 2, y: 298, 'text-anchor': 'middle', fill: 'var(--text-dim)', 'font-size': 10, 'font-family': 'var(--mono)' }, '[' + i + ']');
    cells.push(mk('rect', { x: x + 1.5, y: 304, width: cw - 3, height: 56, rx: 4, fill: `var(${SENS[OWNER[i]].col})`, 'fill-opacity': 0.12, stroke: 'var(--border)', 'stroke-width': 1 }));
    vals.push(mk('text', { x: x + cw / 2, y: 337, 'text-anchor': 'middle', fill: 'var(--text-bright)', 'font-size': 10, 'font-family': 'var(--mono)' }, ''));
    mk('text', { x: x + cw / 2, y: 374, 'text-anchor': 'middle', fill: `var(${SENS[OWNER[i]].col})`, 'font-size': 9, 'font-family': 'var(--mono)' }, CELL[i]);
  }
  SENS.forEach(s => {
    const x0 = 12 + s.adr * cw + 4, x1 = 12 + (s.adr + s.dim) * cw - 4;
    mk('path', { d: `M${x0},384 L${x0},390 L${x1},390 L${x1},384`, fill: 'none', stroke: `var(${s.col})`, 'stroke-width': 1.5 });
    if (s.dim > 1) mk('text', { x: (x0 + x1) / 2, y: 404, 'text-anchor': 'middle', fill: `var(${s.col})`, 'font-size': 10, 'font-family': 'var(--mono)' }, s.name);
  });
  // Readout rows.
  $('s6Rows').innerHTML = CELL.map((c, i) => `<span id="s6Row${i}"><span class="rk">[${i}] ${SENS[OWNER[i]].name}${SENS[OWNER[i]].dim > 1 ? ' ' + 'xyz'[i - SENS[OWNER[i]].adr] : ''}:</span> <span class="${OWNER[i] % 2 ? 'rv-o' : 'rv'}" id="s6V${i}"></span> <span class="rk">${SENS[OWNER[i]].unit}</span></span><br>`).join('');

  function refreshShown() { st.shown = st.sd.map((x, i) => noise ? x + NOISE[i] * gauss() : x); }
  function forward() { st.sd = sense(st.q, st.v, st.u).s; refreshShown(); }
  function step() { const r = sense(st.q, st.v, st.u); st.sd = r.s; st.v += H * r.a; st.q += H * st.v; st.t += H; }

  function render() {
    $('s6Pendulum').setAttribute('transform', `translate(${PX}, ${PY}) rotate(${st.q * 180 / Math.PI})`);
    const tx = PX - Math.sin(st.q) * L, ty = PY + Math.cos(st.q) * L;
    const lab = $('s6TipLabel'); lab.setAttribute('x', tx + (Math.sin(st.q) > 0 ? -60 : 10)); lab.setAttribute('y', ty + 4);
    $('s6TimeText').textContent = `time: ${st.t.toFixed(3)} s`;
    $('s6CtrlText').textContent = `ctrl: ${st.u >= 0 ? '+' : ''}${st.u.toFixed(2)} (hinge torque ${st.u >= 0 ? '+' : ''}${(GEAR * st.u).toFixed(0)} N m)`;
    // torque arc around the pivot: positive ctrl turns the rod so the tip swings toward -x
    const span = Math.abs(st.u) * 1.4, r = 32, a0 = -Math.PI / 2, a1 = a0 + Math.sign(st.u) * span;
    $('s6Torque').setAttribute('d', Math.abs(st.u) < 0.01 ? '' : `M${PX + r * Math.cos(a0)},${PY + r * Math.sin(a0)} A${r},${r} 0 0 ${st.u > 0 ? 1 : 0} ${PX + r * Math.cos(a1)},${PY + r * Math.sin(a1)}`);
    for (let i = 0; i < 9; i++) {
      const on = hi === OWNER[i];
      cells[i].setAttribute('fill-opacity', on ? 0.4 : 0.12);
      cells[i].setAttribute('stroke', on ? `var(${SENS[OWNER[i]].col})` : 'var(--border)');
      cells[i].setAttribute('stroke-width', on ? 2.5 : 1);
      vals[i].textContent = (Math.abs(st.shown[i]) < 5e-4 ? 0 : st.shown[i]).toFixed(2);
      $('s6V' + i).textContent = f4(st.shown[i]);
      $('s6Row' + i).style.background = on ? 'var(--teal-wash)' : ''; $('s6Row' + i).style.fontWeight = on ? '700' : '';
    }
    const s = SENS[hi];
    $('s6HiText').textContent = s ? `${s.name}: ${s.type}, sensordata[${s.adr}${s.dim > 1 ? ':' + (s.adr + s.dim) : ''}], ${s.frame} frame, ${s.unit}` : 'Press "Highlight next sensor" to light up one slice';
    $('s6Time').textContent = st.t.toFixed(3); $('s6Qpos').textContent = f4(st.q);
    const q = st.shown[0];
    $('s6Obs').textContent = `[${f4(Math.cos(q))}, ${f4(Math.sin(q))}, ${f4(st.shown[1])}]`;
  }
  function stepN(n) { for (let i = 0; i < n; i++) step(); refreshShown(); render(); }
  function setPlay(on) { playing = on; $('s6Play').textContent = on ? 'Pause' : 'Play'; }
  function loop(ts) {
    if (!playing) return;
    const dt = Math.min(0.05, (ts - last) / 1000); last = ts; acc += dt; let n = 0;
    while (acc >= H) { step(); acc -= H; n++; }
    if (n) { refreshShown(); render(); }
    if (st.t - tPlay > 30) { setPlay(false); return; } // stop after 30 s of simulated time
    requestAnimationFrame(loop);
  }
  $('s6Play').addEventListener('click', () => {
    if (RM) { stepN(250); return; } // reduced motion: jump 0.5 s per press, no animation loop
    setPlay(!playing);
    if (playing) { last = performance.now(); acc = 0; tPlay = st.t; requestAnimationFrame(loop); }
  });
  $('s6Step').addEventListener('click', () => stepN(1));
  $('s6Step50').addEventListener('click', () => stepN(50));
  $('s6Reset').addEventListener('click', () => { setPlay(false); Object.assign(st, { q: 0.7, v: 0, t: 0 }); forward(); render(); });
  $('s6Ctrl').addEventListener('input', () => { st.u = +$('s6Ctrl').value; $('s6CtrlVal').textContent = st.u.toFixed(2); render(); });
  $('s6Hi').addEventListener('click', () => { hi = hi >= 4 ? -1 : hi + 1; render(); });
  $('s6Noise').addEventListener('click', () => {
    noise = !noise; const b = $('s6Noise'); b.textContent = noise ? 'Noise: on (added by this page)' : 'Noise: off'; b.setAttribute('aria-pressed', String(noise));
    $('s6NoiseNote').textContent = noise ? 'Noise on: this page adds Gaussian noise with the sensor_noise std values. MuJoCo would not.' : 'Noise off: values are exactly what MuJoCo puts in sensordata.';
    refreshShown(); render();
  });
  forward(); render();
})();

// ======== LESSON 6 EXPLAINER ========
(() => {
  // Only redraw this lesson's canvases while it is on screen (hidden canvases have zero size).
  const addEventListener = (type, fn, opts) => window.addEventListener(type, e => { if (document.getElementById('lesson6').classList.contains('active')) fn(e); }, opts);
  // Same model and constants as sandbox 6 (verified against MuJoCo 3.15 by check_browser_match.py).
  const MGD = 109.140436750183, INV_I = 0.126961641835542, G = 9.81, H = 0.002, GEAR = 100; // lesson 5 motor: gear 100, ctrlrange -1..1
  const sense = (q, v, u) => {
    const a = (-MGD * Math.sin(q) + GEAR * u) * INV_I;
    return { a, s: [q, v, u, -Math.sin(q), 0, 1 - Math.cos(q), -a - G * Math.sin(q), 0, G * Math.cos(q) + v * v] };
  };
  const SENS = [
    { name: 'angle', adr: 0, dim: 1, unit: 'rad', col: '--teal' },
    { name: 'spin', adr: 1, dim: 1, unit: 'rad/s', col: '--orange' },
    { name: 'motor_force', adr: 2, dim: 1, unit: 'ctrl units', col: '--purple' },
    { name: 'tip_pos', adr: 3, dim: 3, unit: 'm', col: '--blue' },
    { name: 'tip_acc', adr: 6, dim: 3, unit: 'm/s²', col: '--yellow' },
  ];
  const CELL = ['angle', 'spin', 'motor', 'tip x', 'tip y', 'tip z', 'acc x', 'acc y', 'acc z'];
  const OWNER = [0, 1, 2, 3, 3, 3, 4, 4, 4];
  const $ = id => document.getElementById(id);
  const z = (v, eps) => (Math.abs(v) < eps ? 0 : v);
  const f5 = v => z(v, 5e-6).toFixed(5);
  const f2 = v => z(v, 5e-3).toFixed(2);
  // A rod hanging from pivot P, length s px; +q swings the tip toward -x (left on screen).
  function rod(c, P, s, q, col, w, dashed) {
    c.save(); if (dashed) c.setLineDash([6, 5]);
    c.strokeStyle = col; c.lineWidth = w; c.lineCap = 'round';
    c.beginPath(); c.moveTo(P[0], P[1]); c.lineTo(P[0] - Math.sin(q) * s, P[1] + Math.cos(q) * s); c.stroke(); c.restore();
    return [P[0] - Math.sin(q) * s, P[1] + Math.cos(q) * s];
  }
  function rrect(c, x, y, w, h, r) { c.beginPath(); c.moveTo(x + r, y); c.arcTo(x + w, y, x + w, y + h, r); c.arcTo(x + w, y + h, x, y + h, r); c.arcTo(x, y + h, x, y, r); c.arcTo(x, y, x + w, y, r); c.closePath(); }

  /* ---------- Section 1: animated dashboard ---------- */
  (() => {
    const cv = $('l6cvDash'); let g = fit(cv, cv.clientWidth < 520 ? 1.2 : 0.66);
    const st = { q: 0.7, v: 0, t: 0 };
    let sd = sense(0.7, 0, 0).s, hi = 0, pinned = false, playing = false, last = 0, acc = 0, tHi = 0, tPlay = 0;
    const aspect = () => (cv.clientWidth < 520 ? 1.2 : 0.66); // narrow screens stack the pendulum above the gauges
    function draw() {
      const { ctx: c, w, h } = g; c.clearRect(0, 0, w, h);
      const narrow = w < 520, fs = narrow ? 10 : Math.max(9, Math.min(12, w / 52));
      // layout: [pendulum box], gauges (x, y, width, row pitch, row height), strip (y, height)
      const L = narrow
        ? { P: [w * 0.5, h * 0.04], Lp: h * 0.2, info: [8, h * 0.06], gx: 8, gw: w - 16, gy: h * 0.29, gp: h * 0.072, gh: h * 0.06, y0: h * 0.665, chh: h * 0.24 * 0.55 }
        : { P: [w * 0.16, h * 0.1], Lp: Math.min(h * 0.42, w * 0.2), info: [8, h * 0.62], gx: w * 0.34, gw: w * 0.64, gy: h * 0.035, gp: h * 0.13, gh: h * 0.105, y0: h * 0.72, chh: h * 0.18 };
      const P = L.P, Lp = L.Lp, gx = L.gx, gw = L.gw, gh = L.gh, sx0 = 8, cw = (w - 16) / 9, y0 = L.y0, chh = L.chh;
      // pendulum
      c.setLineDash([4, 5]); c.strokeStyle = css('--dim'); c.lineWidth = 1; c.beginPath(); c.moveTo(P[0], P[1]); c.lineTo(P[0], P[1] + Lp * 1.1); c.stroke(); c.setLineDash([]);
      const tip = rod(c, P, Lp, st.q, css('--teal'), Math.max(6, Lp * 0.12));
      c.fillStyle = css('--purple'); c.beginPath(); c.arc(P[0], P[1], 5, 0, 7); c.fill();
      c.fillStyle = css('--blue'); c.beginPath(); c.arc(tip[0], tip[1], 5, 0, 7); c.fill();
      text(c, 't = ' + st.t.toFixed(2) + ' s', L.info[0], L.info[1], css('--dim'), fs);
      text(c, 'ctrl = 0', L.info[0], L.info[1] + fs + 4, css('--dim'), fs);
      // gauges
      SENS.forEach((s, k) => {
        const gy = L.gy + k * L.gp, on = k === hi, col = css(s.col);
        c.globalAlpha = on ? 0.18 : 0.06; c.fillStyle = col; rrect(c, gx, gy, gw, gh, 6); c.fill(); c.globalAlpha = 1;
        c.strokeStyle = on ? col : css('--border'); c.lineWidth = on ? 2.5 : 1; rrect(c, gx, gy, gw, gh, 6); c.stroke();
        text(c, s.name, gx + 8, gy + gh / 2, col, fs, 'left', 700);
        const v = sd.slice(s.adr, s.adr + s.dim).map(f2);
        text(c, (s.dim > 1 ? '(' + v.join(', ') + ')' : v[0]) + ' ' + s.unit, gx + gw - 8, gy + gh / 2, on ? css('--bright') : css('--dim'), fs, 'right');
        if (on && !narrow) { // connector from the gauge to its slice of sensordata
          const cx = sx0 + (s.adr + s.dim / 2) * cw, ex = narrow ? cx : gx - 10, ey = narrow ? gy + gh : gy + gh / 2;
          c.setLineDash([4, 4]); c.strokeStyle = col; c.lineWidth = 1.5; c.beginPath(); c.moveTo(narrow ? cx : gx, ey); c.lineTo(ex, ey); c.lineTo(cx, y0 - 4); c.stroke(); c.setLineDash([]);
        }
      });
      // the sensordata strip
      for (let i = 0; i < 9; i++) {
        const x = sx0 + i * cw, s = SENS[OWNER[i]], on = OWNER[i] === hi, col = css(s.col);
        c.globalAlpha = on ? 0.35 : 0.08; c.fillStyle = col; rrect(c, x + 1.5, y0, cw - 3, chh, 4); c.fill(); c.globalAlpha = 1;
        c.strokeStyle = on ? col : css('--border'); c.lineWidth = on ? 2.5 : 1; rrect(c, x + 1.5, y0, cw - 3, chh, 4); c.stroke();
        text(c, '[' + i + ']', x + cw / 2, y0 + chh * 0.2, css('--dim'), fs - 1, 'center');
        text(c, f2(sd[i]), x + cw / 2, y0 + chh * 0.52, css('--bright'), narrow ? 8.5 : fs - 1, 'center');
        text(c, CELL[i], x + cw / 2, y0 + chh * 0.82, col, narrow ? 8 : fs - 2, 'center');
      }
      const s = SENS[hi], yb = y0 + chh + (h - y0 - chh) / 2;
      text(c, 'data.sensordata, shape (9,)', 8, narrow ? yb - 8 : yb, css('--dim'), fs);
      text(c, s.name + ' owns sensordata[' + s.adr + (s.dim > 1 ? ':' + (s.adr + s.dim) : '') + ']', narrow ? 8 : w - 8, narrow ? yb + 10 : yb, css(s.col), fs, narrow ? 'left' : 'right', 700);
    }
    function step() { const r = sense(st.q, st.v, 0); sd = r.s; st.v += H * r.a; st.q += H * st.v; st.t += H; }
    function setPlay(on) { playing = on; $('l6dPlay').textContent = on ? 'Pause' : 'Play'; }
    function loop(ts) {
      if (!playing) return;
      const dt = Math.min(0.05, (ts - last) / 1000); last = ts; acc += dt; tHi += dt;
      while (acc >= H) { step(); acc -= H; }
      if (!pinned && tHi > 1.2) { tHi = 0; hi = (hi + 1) % 5; }
      draw();
      if (st.t - tPlay > 20) { setPlay(false); return; } // finished: 20 s of swinging
      requestAnimationFrame(loop);
    }
    $('l6dPlay').addEventListener('click', () => {
      if (RM) { for (let i = 0; i < 150; i++) step(); if (!pinned) hi = (hi + 1) % 5; draw(); return; } // reduced motion: one jump per press
      setPlay(!playing); if (playing) { last = performance.now(); tPlay = st.t; requestAnimationFrame(loop); }
    });
    $('l6dReset').addEventListener('click', () => { setPlay(false); Object.assign(st, { q: 0.7, v: 0, t: 0 }); sd = sense(0.7, 0, 0).s; draw(); });
    document.querySelectorAll('.l6dPick').forEach(b => b.addEventListener('click', () => { const k = +b.dataset.k; pinned = k >= 0; if (pinned) hi = k; tHi = 0; draw(); }));
    addEventListener('resize', () => { g = fit(cv, aspect()); draw(); }); addEventListener('themechange', draw);
    draw();
  })();

  /* ---------- Section 3: sensors update only in mj_forward / mj_step ---------- */
  (() => {
    const cv = $('l6cvRead'); let g = fit(cv, 0.9);
    const st = { q: 0, v: 0, t: 0 };
    let sd = new Array(9).fill(0), siteZ = 0, mode = 'new'; // fresh MjData: sensordata and site_xpos are all zeros
    const log = [], logEl = $('l6rLog');
    function addLog(s) { log.push(s); if (log.length > 120) log.shift(); logEl.textContent = log.join('\n'); logEl.scrollTop = 1e9; }
    function forward() { sd = sense(st.q, st.v, 0).s; siteZ = 1 - Math.cos(st.q); mode = 'fresh'; }
    function step() { const r = sense(st.q, st.v, 0); sd = r.s; siteZ = 1 - Math.cos(st.q); st.v += H * r.a; st.q += H * st.v; st.t += H; mode = 'stepped'; }
    const MSG = {
      new: 'NOT COMPUTED YET: a new MjData holds zeros. Call mj_forward().',
      reset: 'CLEARED: mj_resetData set sensordata to zeros. Call mj_forward().',
      set: 'STALE: you changed qpos, sensors still describe the old pose.',
      stepped: 'ONE STEP BEHIND: after mj_step, sensors describe the start of the step.',
      fresh: 'FRESH: sensors match qpos.',
    };
    function show(changed) {
      $('l6rQ').querySelector('.val').textContent = f5(st.q);
      $('l6rS0').querySelector('.val').textContent = f5(sd[0]);
      $('l6rX').querySelector('.val').textContent = f5(siteZ);
      $('l6rS5').querySelector('.val').textContent = f5(sd[5]);
      $('l6rS8').querySelector('.val').textContent = f5(sd[8]);
      const m = $('l6rStale'); m.textContent = MSG[mode]; m.style.color = mode === 'fresh' ? 'var(--teal)' : 'var(--orange)';
      (changed || []).forEach(id => flash($(id)));
      const { ctx: c, w, h } = g; c.clearRect(0, 0, w, h);
      const P = [w / 2, h * 0.4], s = h * 0.36;
      c.setLineDash([4, 5]); c.strokeStyle = css('--dim'); c.lineWidth = 1; c.beginPath(); c.moveTo(P[0], P[1]); c.lineTo(P[0], P[1] + s * 1.1); c.stroke(); c.setLineDash([]);
      rod(c, P, s, st.q, css('--teal'), Math.max(8, s * 0.13));
      rod(c, P, s * 1.05, sd[0], css('--orange'), 2.5, true);
      c.fillStyle = css('--purple'); c.beginPath(); c.arc(P[0], P[1], 5, 0, 7); c.fill();
      text(c, 'truth: qpos[0] = ' + f5(st.q) + ' rad', 10, 16, css('--teal'), 11);
      text(c, '- - sensor: sensordata[0] = ' + f5(sd[0]) + ' rad', 10, 34, css('--orange'), 11);
    }
    const all = ['l6rQ', 'l6rS0', 'l6rX', 'l6rS5', 'l6rS8'];
    $('l6rFwd').addEventListener('click', () => { forward(); addLog('mujoco.mj_forward(model, data)   # sensordata[0] = ' + f5(sd[0]) + ', sensordata[8] = ' + f5(sd[8])); show(['l6rS0', 'l6rX', 'l6rS5', 'l6rS8']); });
    $('l6rStep').addEventListener('click', () => { step(); addLog('mujoco.mj_step(model, data)      # qpos[0] = ' + f5(st.q) + ', sensordata[0] = ' + f5(sd[0])); show(all); });
    $('l6rReset').addEventListener('click', () => { Object.assign(st, { q: 0, v: 0, t: 0 }); sd = new Array(9).fill(0); siteZ = 0; mode = 'reset'; $('l6rQs').value = 0; $('l6rOq').textContent = '0.00 rad'; addLog('mujoco.mj_resetData(model, data) # everything back to zeros'); show(all); });
    $('l6rQs').addEventListener('input', () => { st.q = +$('l6rQs').value; st.v = 0; mode = 'set'; $('l6rOq').textContent = st.q.toFixed(2) + ' rad'; show(['l6rQ']); });
    $('l6rQs').addEventListener('change', () => addLog('data.qpos[0] = ' + st.q.toFixed(2) + '; data.qvel[0] = 0   # sensordata NOT updated yet'));
    addEventListener('resize', () => { g = fit(cv, 0.9); show(); }); addEventListener('themechange', () => show());
    addLog('data = mujoco.MjData(model)       # sensordata starts as 9 zeros');
    show();
  })();

  /* ---------- Section 4: accelerometer ---------- */
  (() => {
    const cv = $('l6cvAcc'); let g = fit(cv, 0.95);
    let q = 0, v = 0;
    function draw() {
      const r = sense(q, v, 0), ax = r.s[6], az = r.s[8];
      $('l6aOq').textContent = q.toFixed(2) + ' rad'; $('l6aOv').textContent = v.toFixed(2) + ' rad/s';
      $('l6aX').querySelector('.val').textContent = f5(ax); $('l6aZ').querySelector('.val').textContent = f5(az); $('l6aA').querySelector('.val').textContent = f5(r.a);
      const { ctx: c, w, h } = g; c.clearRect(0, 0, w, h);
      const P = [w / 2, h * 0.5], s = h * 0.3;
      c.setLineDash([4, 5]); c.strokeStyle = css('--dim'); c.lineWidth = 1; c.beginPath(); c.moveTo(P[0], P[1]); c.lineTo(P[0], P[1] + s * 1.1); c.stroke(); c.setLineDash([]);
      const tip = rod(c, P, s, q, css('--teal'), 6);
      c.fillStyle = css('--purple'); c.beginPath(); c.arc(P[0], P[1], 5, 0, 7); c.fill();
      // tip site frame: x axis (cos q, sin q) on screen, z axis (sin q, -cos q) on screen (back up the rod)
      const xd = [Math.cos(q), Math.sin(q)], zd = [Math.sin(q), -Math.cos(q)], ax3 = h * 0.1;
      arrow(c, tip[0], tip[1], tip[0] + xd[0] * ax3, tip[1] + xd[1] * ax3, css('--x'), 2, 7);
      arrow(c, tip[0], tip[1], tip[0] + zd[0] * ax3, tip[1] + zd[1] * ax3, css('--z'), 2, 7);
      text(c, 'x', tip[0] + xd[0] * (ax3 + 9), tip[1] + xd[1] * (ax3 + 9), css('--x'), 11, 'center');
      text(c, 'z', tip[0] + zd[0] * (ax3 + 9), tip[1] + zd[1] * (ax3 + 9), css('--z'), 11, 'center');
      // reading vector, drawn in the world but built from site-frame components
      const k = h * 0.026, rx = (ax * xd[0] + az * zd[0]) * k, ry = (ax * xd[1] + az * zd[1]) * k, len = Math.hypot(rx, ry), cap = h * 0.42, sc = len > cap ? cap / len : 1;
      if (len > 2) arrow(c, tip[0], tip[1], tip[0] + rx * sc, tip[1] + ry * sc, css('--yellow'), 3, 10);
      c.fillStyle = css('--blue'); c.beginPath(); c.arc(tip[0], tip[1], 4.5, 0, 7); c.fill();
      text(c, 'tip_acc (tip site frame)', 10, 16, css('--yellow'), 11);
      text(c, '= (' + f2(ax) + ', 0, ' + f2(az) + ') m/s²', 10, 32, css('--yellow'), 11);
      text(c, 'reading = real acceleration - gravity', 10, h - 12, css('--dim'), 10);
    }
    $('l6aQ').addEventListener('input', () => { q = +$('l6aQ').value; draw(); });
    $('l6aV').addEventListener('input', () => { v = +$('l6aV').value; draw(); });
    document.querySelectorAll('.l6aPre').forEach(b => b.addEventListener('click', () => { q = +b.dataset.q; v = +b.dataset.v; $('l6aQ').value = q; $('l6aV').value = v; draw(); ['l6aX', 'l6aZ'].forEach(id => flash($(id))); }));
    addEventListener('resize', () => { g = fit(cv, 0.95); draw(); }); addEventListener('themechange', draw);
    draw();
  })();

  /* ---------- Section 5: angle wrapping, number line vs circle ---------- */
  (() => {
    const cv = $('l6cvWrap'); let g = fit(cv, 0.85);
    let q = 0;
    function draw() {
      $('l6wO').textContent = q.toFixed(2) + ' rad';
      $('l6wC').querySelector('.val').textContent = f5(Math.cos(q)); $('l6wS').querySelector('.val').textContent = f5(Math.sin(q));
      const { ctx: c, w, h } = g; c.clearRect(0, 0, w, h);
      // number line from -7 to 7 rad
      const ly = h * 0.12, X = t => 20 + (t + 7) / 14 * (w - 40);
      text(c, 'raw angle (what NOT to give the policy)', 10, h * 0.035, css('--dim'), 10);
      c.strokeStyle = css('--grid'); c.lineWidth = 2; c.beginPath(); c.moveTo(X(-7), ly); c.lineTo(X(7), ly); c.stroke();
      [[-2, '-2π'], [-1, '-π'], [0, '0'], [1, 'π'], [2, '2π']].forEach(([k, lab]) => { const x = X(k * Math.PI); c.strokeStyle = css('--dim'); c.lineWidth = 1; c.beginPath(); c.moveTo(x, ly - 6); c.lineTo(x, ly + 6); c.stroke(); text(c, lab, x, ly + 16, css('--dim'), 10, 'center'); });
      c.fillStyle = css('--orange'); c.beginPath(); c.arc(X(q), ly, 6, 0, 7); c.fill();
      // circle: (cos q, sin q)
      const C = [w / 2, h * 0.63], R = Math.min(w * 0.3, h * 0.23);
      text(c, 'obs[0:2] = (cos θ, sin θ): a point on a circle', 10, h * 0.255, css('--dim'), 10);
      c.strokeStyle = css('--grid'); c.lineWidth = 1; c.beginPath(); c.moveTo(C[0] - R - 14, C[1]); c.lineTo(C[0] + R + 14, C[1]); c.moveTo(C[0], C[1] - R - 14); c.lineTo(C[0], C[1] + R + 14); c.stroke();
      text(c, 'cos', C[0] + R + 16, C[1] - 10, css('--dim'), 10, 'right'); text(c, 'sin', C[0] + 6, C[1] - R - 6, css('--dim'), 10);
      c.strokeStyle = css('--teal'); c.lineWidth = 2; c.beginPath(); c.arc(C[0], C[1], R, 0, 7); c.stroke();
      const px = C[0] + Math.cos(q) * R, py = C[1] - Math.sin(q) * R;
      c.strokeStyle = css('--orange'); c.lineWidth = 1.5; c.beginPath(); c.moveTo(C[0], C[1]); c.lineTo(px, py); c.stroke();
      c.fillStyle = css('--orange'); c.beginPath(); c.arc(px, py, 7, 0, 7); c.fill();
      text(c, '(' + f2(Math.cos(q)) + ', ' + f2(Math.sin(q)) + ')', C[0], C[1] + R + 26, css('--orange'), 12, 'center', 700);
    }
    $('l6wQ').addEventListener('input', () => { q = +$('l6wQ').value; draw(); });
    document.querySelectorAll('.l6wPre').forEach(b => b.addEventListener('click', () => { q = +b.dataset.q; $('l6wQ').value = q; draw(); ['l6wC', 'l6wS'].forEach(id => flash($(id))); }));
    addEventListener('resize', () => { g = fit(cv, 0.85); draw(); }); addEventListener('themechange', draw);
    draw();
  })();
})();
