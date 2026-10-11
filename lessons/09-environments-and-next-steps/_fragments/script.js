// ======== SANDBOX 9 + LESSON 9 (one outer IIFE so both share the pendulum env core) ========
(() => {
// @@CORE9-START
// Same model and env as 09_environments_and_next_steps.py (PendulumHoldEnv).
// Motor from lesson 5: torque = gear * ctrl = 100 * ctrl N*m, ctrl clipped to -1..1.
// qacc = (100 * ctrl - m g d sin q) / I, then MuJoCo's Euler step: v += h*qacc, q += h*v.
// m g d = 109.1404 N*m and I = 7.8764 kg*m^2 were read from MuJoCo (qfrc_bias, mass matrix).
// Checked against MuJoCo 3.15.0 by 09_check_browser_physics.py (agrees to better than 4 decimals).
const L9 = (() => {
  const H = 0.002, MGD = 109.14043675018314, INV_I = 1 / 7.876394677499038;
  const GEAR = 100, LO = -1, HI = 1, KP = 400, KD = 40, TARGET = 1.0;
  // Start angles from env.reset(seed) in Python (numpy's generator, recorded, rad).
  const START = [0.1369616873214543, 0.011821624700256717, -0.2383878657506836, -0.41435083285637564, 0.44305610557236763];
  const f32 = Math.fround, clip = x => Math.max(LO, Math.min(HI, x));
  function make(fs = 10, maxSteps = 250) { return { fs, maxSteps, q: 0, v: 0, t: 0, steps: 0, ret: 0, a: 0, r: 0, term: false, trunc: false, seed: 0, rng: null }; }
  // mulberry32: the page's own seeded random numbers (NOT numpy's generator).
  function prng(seed) { let s = (seed * 2654435761 + 12345) >>> 0; return () => { s = (s + 0x6D2B79F5) >>> 0; let t = s; t = Math.imul(t ^ (t >>> 15), t | 1); t ^= t + Math.imul(t ^ (t >>> 7), t | 61); return ((t ^ (t >>> 14)) >>> 0) / 4294967296; }; }
  function obs(e) { return [f32(Math.cos(e.q)), f32(Math.sin(e.q)), f32(e.v)]; }
  function reset(e, seed) { Object.assign(e, { q: START[seed], v: 0, t: 0, steps: 0, ret: 0, a: 0, r: 0, term: false, trunc: false, seed, rng: prng(seed) }); return obs(e); }
  function tick(e) { const acc = (GEAR * e.a - MGD * Math.sin(e.q)) * INV_I; e.v += H * acc; e.q += H * e.v; e.t += H; }
  function step(e, action) {
    e.a = clip(action);
    for (let i = 0; i < e.fs; i++) tick(e);
    e.steps++;
    const err = e.q - TARGET;
    e.r = -(err * err + 0.1 * e.v * e.v + 0.01 * e.a * e.a);
    e.ret += e.r; e.term = false; e.trunc = e.steps >= e.maxSteps;
    return obs(e);
  }
  const policies = {
    zero: () => 0,
    random: (o, e) => f32(LO + (HI - LO) * e.rng()),
    // lesson 8 PD in N*m, then divided by the gear to get ctrl
    pd: o => f32(clip((KP * (TARGET - Math.atan2(o[1], o[0])) - KD * o[2]) / GEAR)),
    // the same, plus gravity compensation m g d sin q
    pdg: o => { const q = Math.atan2(o[1], o[0]); return f32(clip((KP * (TARGET - q) - KD * o[2] + MGD * Math.sin(q)) / GEAR)); },
  };
  function episode(policy, seed, fs = 10, maxSteps = 250, rec) {
    const e = make(fs, maxSteps); let o = reset(e, seed);
    while (!e.trunc && !e.term) { o = step(e, policies[policy](o, e)); if (rec) rec.push([e.t, e.q, e.v, e.a, e.r]); }
    return e;
  }
  return { H, GEAR, KP, KD, TARGET, START, make, reset, step, obs, tick, policies, episode, clip };
})();
// @@CORE9-END

const fx = (v, d) => { const s = (Math.abs(v) < 0.5 * Math.pow(10, -d) ? 0 : v).toFixed(d); return s.startsWith('-') ? s : ' ' + s; };
const POLNAME = { zero: 'zero', random: 'random', pd: 'PD', pdg: 'PD + gravity' };

// ======== SANDBOX 9: EPISODE PLAYER ========
(() => {
  const $ = id => document.getElementById(id);
  if (!$('sandbox9')) return;
  const env = L9.make();
  let pol = 'pdg', seed = 0, o = L9.reset(env, 0), timer = null;
  function render() {
    $('s9Pend').setAttribute('transform', `translate(200, 120) rotate(${env.q * 180 / Math.PI})`);
    const arr = $('s9Torque'), L = env.a * 60;
    if (Math.abs(L) > 1) { arr.style.display = ''; arr.setAttribute('x2', -L); } else arr.style.display = 'none';
    $('s9TimeText').textContent = `time: ${env.t.toFixed(3)} s`;
    $('s9StepText').textContent = `env step: ${env.steps} / ${env.maxSteps}`;
    $('s9PolText').textContent = `policy: ${POLNAME[pol]}, seed ${seed}`;
    $('s9Steps').textContent = env.steps;
    $('s9Time').textContent = env.t.toFixed(3);
    $('s9Obs').textContent = `[${fx(o[0], 4)}, ${fx(o[1], 4)}, ${fx(o[2], 4)}]`;
    $('s9Act').textContent = fx(env.a, 4);
    $('s9Tau').textContent = fx(env.a * L9.GEAR, 2);
    $('s9Rew').textContent = fx(env.r, 5);
    $('s9Ret').textContent = fx(env.ret, 3);
    $('s9Done').textContent = `${env.term} / ${env.trunc}`;
    $('s9Run').textContent = timer ? 'Pause' : 'Run episode';
    $('s9Step').disabled = env.trunc;
    document.querySelectorAll('#sandbox9 [data-s9pol]').forEach(b => b.classList.toggle('btn-primary', b.dataset.s9pol === pol));
    document.querySelectorAll('#sandbox9 [data-s9seed]').forEach(b => b.classList.toggle('btn-primary', +b.dataset.s9seed === seed));
  }
  function stop() { if (timer) { clearInterval(timer); timer = null; } }
  function doStep() { if (env.trunc) return; o = L9.step(env, L9.policies[pol](o, env)); }
  function reset() { stop(); o = L9.reset(env, seed); render(); }
  $('s9Reset').addEventListener('click', reset);
  $('s9Step').addEventListener('click', () => { stop(); doStep(); render(); });
  $('s9Run').addEventListener('click', () => {
    if (timer) { stop(); render(); return; }
    if (env.trunc) o = L9.reset(env, seed);
    if (RM) { while (!env.trunc) doStep(); render(); return; }
    timer = setInterval(() => { doStep(); doStep(); if (env.trunc) stop(); render(); }, 40); // 2 env steps per 40 ms = real time
    render();
  });
  document.querySelectorAll('#sandbox9 [data-s9pol]').forEach(b => b.addEventListener('click', () => { pol = b.dataset.s9pol; reset(); }));
  document.querySelectorAll('#sandbox9 [data-s9seed]').forEach(b => b.addEventListener('click', () => { seed = +b.dataset.s9seed; reset(); }));
  render();
})();

// ======== LESSON 9 EXPLAINER ========
(() => {
  // Only redraw this lesson's canvases while it is on screen (hidden canvases have zero size).
  const addEventListener = (type, fn, opts) => window.addEventListener(type, e => { if (document.getElementById('lesson9').classList.contains('active')) fn(e); }, opts);
  const $ = id => document.getElementById(id);
  if (!$('lesson9')) return;

  function drawPend(c, cx, cy, L, q, col) {
    c.setLineDash([3, 4]); c.strokeStyle = css('--dim'); c.lineWidth = 1; c.beginPath(); c.moveTo(cx, cy); c.lineTo(cx, cy + L * 1.1); c.stroke();
    c.strokeStyle = css('--yellow'); c.beginPath(); c.moveTo(cx, cy); c.lineTo(cx - Math.sin(L9.TARGET) * L * 1.1, cy + Math.cos(L9.TARGET) * L * 1.1); c.stroke(); c.setLineDash([]);
    text(c, 'target', cx - Math.sin(L9.TARGET) * L * 1.15, cy + Math.cos(L9.TARGET) * L * 1.15 + 8, css('--yellow'), 10, 'center');
    c.strokeStyle = col || css('--teal'); c.lineWidth = Math.max(4, L * 0.14); c.lineCap = 'round';
    c.beginPath(); c.moveTo(cx, cy); c.lineTo(cx - Math.sin(q) * L, cy + Math.cos(q) * L); c.stroke();
    c.fillStyle = css('--purple'); c.beginPath(); c.arc(cx, cy, 4, 0, 7); c.fill();
  }

  /* ---------- Section 1: the game loop, one step at a time ---------- */
  (() => {
    const cv = $('l9cvLoop'), asp = () => (cv.clientWidth < 480 ? 0.95 : 0.62); let g = fit(cv, asp());
    const env = L9.make();
    let pol = 'pdg', o = L9.reset(env, 0), phase = 0, prog = 0, ticks = 0, busy = false, playing = false, raf = 0;
    // phase: 0 idle, 1 action travels to env, 2 physics ticks, 3 obs + reward travel back
    function draw() {
      const { ctx: c, w, h } = g; c.clearRect(0, 0, w, h);
      const nar = w < 480, fs = nar ? 10 : 11, ax = w * 0.05, aw = w * 0.3, ex = w * 0.55, ew = w * 0.4, by = h * (nar ? 0.17 : 0.22), bh = h * (nar ? 0.52 : 0.56);
      c.lineWidth = 1.5;
      c.strokeStyle = css('--blue'); c.strokeRect(ax, by, aw, bh);
      c.strokeStyle = css('--teal'); c.strokeRect(ex, by, ew, bh);
      text(c, 'agent (policy)', ax + aw / 2, by + 14, css('--blue'), 12, 'center');
      text(c, POLNAME[pol], ax + aw / 2, by + 32, css('--bright'), 13, 'center');
      text(c, 'obs -> action', ax + aw / 2, by + bh / 2 + 6, css('--dim'), fs, 'center');
      text(c, 'environment', ex + ew / 2, by + 14, css('--teal'), 12, 'center');
      text(c, (nar ? 'ticks ' : 'MuJoCo ticks ') + ticks + ' / 10', ex + ew / 2, by + bh - 12, phase === 2 ? css('--orange') : css('--dim'), fs, 'center');
      drawPend(c, ex + ew / 2 + (nar ? ew * 0.15 : 0), by + 30, bh * 0.5, env.q);
      // arrows
      const yTop = by - h * (nar ? 0.07 : 0.1), yBot = by + bh + h * (nar ? 0.07 : 0.1);
      c.strokeStyle = css('--border'); c.lineWidth = 1.5;
      arrow(c, ax + aw / 2, by, ax + aw / 2, yTop, css('--border'), 1.5, 0); c.beginPath(); c.moveTo(ax + aw / 2, yTop); c.lineTo(ex + ew / 2, yTop); c.stroke(); arrow(c, ex + ew / 2, yTop, ex + ew / 2, by, css('--border'), 1.5, 8);
      arrow(c, ex + ew / 2, by + bh, ex + ew / 2, yBot, css('--border'), 1.5, 0); c.beginPath(); c.moveTo(ex + ew / 2, yBot); c.lineTo(ax + aw / 2, yBot); c.stroke(); arrow(c, ax + aw / 2, yBot, ax + aw / 2, by + bh, css('--border'), 1.5, 8);
      const mx = (ax + aw / 2 + ex + ew / 2) / 2;
      text(c, 'action = ctrl ' + fx(env.a, 3), mx, yTop - 11, css('--orange'), 12, 'center');
      text(c, 'obs = [' + fx(o[0], 3) + ', ' + fx(o[1], 3) + ', ' + fx(o[2], 3) + ']', mx, yBot + 12, css('--teal'), fs, 'center');
      text(c, 'reward = ' + fx(env.r, 4), mx, yBot + 27, css('--teal'), fs, 'center');
      // moving token
      if (phase === 1 || phase === 3) {
        const pts = phase === 1 ? [[ax + aw / 2, by], [ax + aw / 2, yTop], [ex + ew / 2, yTop], [ex + ew / 2, by]] : [[ex + ew / 2, by + bh], [ex + ew / 2, yBot], [ax + aw / 2, yBot], [ax + aw / 2, by + bh]];
        const segs = [], tot = pts.slice(1).reduce((s, p, i) => { const d = Math.hypot(p[0] - pts[i][0], p[1] - pts[i][1]); segs.push(d); return s + d; }, 0);
        let d = prog * tot, i = 0; while (i < segs.length - 1 && d > segs[i]) { d -= segs[i]; i++; }
        const f = Math.min(1, d / segs[i]), x = pts[i][0] + (pts[i + 1][0] - pts[i][0]) * f, y = pts[i][1] + (pts[i + 1][1] - pts[i][1]) * f;
        c.fillStyle = phase === 1 ? css('--orange') : css('--teal'); c.beginPath(); c.arc(x, y, 7, 0, 7); c.fill();
      }
      text(c, 'env step ' + env.steps + '   time ' + env.t.toFixed(3) + ' s', 8, h - 8, css('--dim'), fs);
    }
    function show() {
      $('l9lStep').querySelector('.val').textContent = env.steps;
      $('l9lTime').querySelector('.val').textContent = env.t.toFixed(3);
      $('l9lAct').querySelector('.val').textContent = fx(env.a, 4);
      $('l9lTau').querySelector('.val').textContent = fx(env.a * L9.GEAR, 2);
      $('l9lRew').querySelector('.val').textContent = fx(env.r, 5);
      $('l9lRet').querySelector('.val').textContent = fx(env.ret, 4);
      $('l9lDone').querySelector('.val').textContent = env.term + ' / ' + env.trunc;
      $('l9lStepBtn').disabled = busy || env.trunc;
      $('l9lPlay').textContent = playing ? 'Pause' : 'Play the episode';
      document.querySelectorAll('#lesson9 [data-l9pol]').forEach(b => b.classList.toggle('primary', b.dataset.l9pol === pol));
      draw();
    }
    function instantStep() { const a = L9.policies[pol](o, env); o = L9.step(env, a); ticks = 10; }
    function animStep(done) {
      busy = true; const a = L9.clip(L9.policies[pol](o, env)); env.a = a; phase = 1; prog = 0; ticks = 0; let last = performance.now();
      const save = { q: env.q, v: env.v, t: env.t };
      function frame(ts) {
        const dt = Math.min(0.05, (ts - last) / 1000); last = ts;
        if (phase === 1) { prog += dt / 0.45; if (prog >= 1) { phase = 2; prog = 0; } }
        else if (phase === 2) { prog += dt / 0.5; const want = Math.min(10, Math.floor(prog * 10)); while (ticks < want) { L9.tick(env); ticks++; } if (ticks >= 10) { phase = 3; prog = 0; Object.assign(env, save); o = L9.step(env, a); } }
        else if (phase === 3) { prog += dt / 0.45; if (prog >= 1) { phase = 0; busy = false; show(); flash($('l9lRew')); flash($('l9lRet')); if (done) done(); return; } }
        show(); raf = requestAnimationFrame(frame);
      }
      raf = requestAnimationFrame(frame);
    }
    function step() { if (busy || env.trunc) return; if (RM) { instantStep(); show(); flash($('l9lRew')); return; } animStep(); show(); }
    function play() {
      if (playing) { playing = false; show(); return; }
      if (env.trunc) { o = L9.reset(env, 0); ticks = 0; }
      playing = true; let last = performance.now(), acc = 0;
      if (RM) { while (!env.trunc) instantStep(); playing = false; show(); return; }
      function loop(ts) { if (!playing) return; acc += Math.min(0.05, (ts - last) / 1000); last = ts; while (acc >= 0.02 && !env.trunc) { instantStep(); acc -= 0.02; } if (env.trunc) playing = false; show(); if (playing) requestAnimationFrame(loop); }
      requestAnimationFrame(loop); show();
    }
    function reset() { playing = false; cancelAnimationFrame(raf); busy = false; phase = 0; ticks = 0; o = L9.reset(env, 0); show(); }
    $('l9lStepBtn').onclick = step;
    $('l9lReset').onclick = reset;
    $('l9lPlay').onclick = () => { if (!busy) play(); };
    document.querySelectorAll('#lesson9 [data-l9pol]').forEach(b => b.onclick = () => { pol = b.dataset.l9pol; reset(); });
    addEventListener('resize', () => { g = fit(cv, asp()); draw(); });
    addEventListener('themechange', draw);
    show();
  })();

  /* ---------- Section 3: control rate vs physics rate ---------- */
  (() => {
    const cv = $('l9cvRate'); let g = fit(cv, 0.5);
    let fs = 10;
    function draw() {
      const { ctx: c, w, h } = g; c.clearRect(0, 0, w, h);
      const n = Math.round(5 / (L9.H * fs)), rec = [];
      L9.episode('random', 0, fs, n, rec);
      const ez = L9.episode('zero', 0, fs, n), ep = L9.episode('pdg', 0, fs, n);
      const T = 0.2, X = t => 44 + t / T * (w - 60);
      // physics ticks row
      text(c, 'physics ticks: one mj_step every 0.002 s', 44, 14, css('--dim'), 11);
      for (let k = 0; k <= T / L9.H + 1e-9; k++) { c.fillStyle = css('--dim'); c.fillRect(X(k * L9.H) - 0.5, 24, 1, 8); }
      // action held (zero-order hold) as steps
      const top = 62, span = h * 0.24, Ya = a => top + (1 - a) / 2 * span;
      text(c, 'action held between env steps (random player, page PRNG)', 44, 46, css('--orange'), 11);
      c.strokeStyle = css('--grid'); c.lineWidth = 1; c.beginPath(); c.moveTo(44, Ya(0)); c.lineTo(w - 16, Ya(0)); c.stroke();
      text(c, '+1', 6, Ya(1), css('--dim'), 9); text(c, '0', 6, Ya(0), css('--dim'), 9); text(c, '-1', 6, Ya(-1), css('--dim'), 9);
      c.strokeStyle = css('--orange'); c.lineWidth = 2; c.beginPath();
      let prevT = 0;
      rec.forEach(([t, , , a], i) => { if (prevT >= T - 1e-9) return; const t1 = Math.min(t, T); if (i === 0) c.moveTo(X(0), Ya(a)); else c.lineTo(X(prevT), Ya(a)); c.lineTo(X(t1), Ya(a)); prevT = t; });
      c.stroke();
      // reward dots
      const base = h - 24, rs = h - top - span - 60, Yr = r => base + Math.max(-1.2, r) / 1.2 * rs;
      text(c, 'one reward per env step (lower = more points lost)', 44, base - rs - 14, css('--teal'), 11);
      c.strokeStyle = css('--grid'); c.beginPath(); c.moveTo(44, base); c.lineTo(w - 16, base); c.stroke();
      c.fillStyle = css('--teal'); rec.forEach(([t, , , , r]) => { if (t > T + 1e-9) return; c.beginPath(); c.arc(X(t), Yr(r), 3.5, 0, 7); c.fill(); });
      for (let k = 0; k <= 4; k++) text(c, (k * 0.05).toFixed(2) + ' s', X(k * 0.05), h - 8, css('--dim'), 10, 'center');
      $('l9rN').querySelector('.val').textContent = n;
      $('l9rDt').querySelector('.val').textContent = (L9.H * fs).toFixed(3);
      $('l9rZ').querySelector('.val').textContent = ez.ret.toFixed(3);
      $('l9rP').querySelector('.val').textContent = ep.ret.toFixed(3);
      document.querySelectorAll('#lesson9 [data-l9fs]').forEach(b => b.classList.toggle('primary', +b.dataset.l9fs === fs));
    }
    document.querySelectorAll('#lesson9 [data-l9fs]').forEach(b => b.onclick = () => { fs = +b.dataset.l9fs; draw(); ['l9rN', 'l9rDt', 'l9rZ', 'l9rP'].forEach(id => flash($(id))); });
    addEventListener('resize', () => { g = fit(cv, 0.5); draw(); });
    addEventListener('themechange', draw);
    draw();
  })();

  /* ---------- Section 4: four players, one seed ---------- */
  (() => {
    const cv = $('l9cvPol'); let g = fit(cv, 0.55);
    const P = ['zero', 'random', 'pd', 'pdg'], COL = { zero: '--dim', random: '--purple', pd: '--orange', pdg: '--teal' }, RO = { zero: 'l9pZ', random: 'l9pR', pd: 'l9pP', pdg: 'l9pG' };
    let seed = 0, shown = 250, playing = false;
    const runs = {};
    function compute() { P.forEach(p => { const rec = []; L9.episode(p, seed, 10, 250, rec); runs[p] = rec; }); }
    function draw() {
      const { ctx: c, w, h } = g; c.clearRect(0, 0, w, h);
      let qm = 1.3; P.forEach(p => runs[p].forEach(r => { qm = Math.max(qm, Math.abs(r[1]) * 1.05); })); qm = Math.min(qm, 4);
      const X = t => 44 + t / 5 * (w - 60), Y = q => h / 2 - q / qm * (h / 2 - 22);
      c.strokeStyle = css('--grid'); c.lineWidth = 1;
      for (let t = 0; t <= 5; t++) { c.beginPath(); c.moveTo(X(t), 14); c.lineTo(X(t), h - 18); c.stroke(); text(c, t + ' s', X(t), h - 7, css('--dim'), 10, 'center'); }
      c.beginPath(); c.moveTo(44, Y(0)); c.lineTo(w - 16, Y(0)); c.stroke();
      c.setLineDash([5, 4]); c.strokeStyle = css('--yellow'); c.beginPath(); c.moveTo(44, Y(L9.TARGET)); c.lineTo(w - 16, Y(L9.TARGET)); c.stroke(); c.setLineDash([]);
      text(c, 'target 1.0 rad', w - 18, Y(L9.TARGET) - 9, css('--yellow'), 10, 'right');
      text(c, '+' + qm.toFixed(1), 4, Y(qm) + 4, css('--dim'), 9); text(c, '-' + qm.toFixed(1), 4, Y(-qm) - 4, css('--dim'), 9); text(c, 'qpos', 4, h / 2 - 8, css('--dim'), 10); text(c, 'rad', 4, h / 2 + 8, css('--dim'), 9);
      c.save(); c.beginPath(); c.rect(44, 10, w - 56, h - 26); c.clip();
      P.forEach(p => {
        c.strokeStyle = css(COL[p]); c.lineWidth = p === 'pdg' ? 2.5 : 1.6; c.beginPath(); c.moveTo(X(0), Y(L9.START[seed]));
        runs[p].slice(0, shown).forEach(([t, q]) => c.lineTo(X(t), Y(q))); c.stroke();
      });
      c.restore();
      P.forEach((p, k) => text(c, POLNAME[p], w - 16, h - 30 - (3 - k) * 14, css(COL[p]), 11, 'right'));
      P.forEach(p => { $(RO[p]).querySelector('.val').textContent = runs[p].slice(0, shown).reduce((s, x) => s + x[4], 0).toFixed(3); });
      $('l9pS').querySelector('.val').textContent = L9.START[seed].toFixed(5);
      document.querySelectorAll('#lesson9 [data-l9seed]').forEach(b => b.classList.toggle('primary', +b.dataset.l9seed === seed));
      $('l9pPlay').textContent = playing ? 'Pause' : 'Replay over 5 s';
    }
    function play() {
      if (playing) { playing = false; draw(); return; }
      if (RM) { shown = 250; draw(); return; }
      playing = true; shown = 0; let last = performance.now(), acc = 0;
      function loop(ts) { if (!playing) return; acc += Math.min(0.05, (ts - last) / 1000); last = ts; shown = Math.min(250, Math.floor(acc / 0.02)); if (shown >= 250) playing = false; draw(); if (playing) requestAnimationFrame(loop); }
      requestAnimationFrame(loop);
    }
    document.querySelectorAll('#lesson9 [data-l9seed]').forEach(b => b.onclick = () => { seed = +b.dataset.l9seed; playing = false; shown = 250; compute(); draw(); ['l9pZ', 'l9pR', 'l9pP', 'l9pG', 'l9pS'].forEach(id => flash($(id))); });
    $('l9pPlay').onclick = play;
    addEventListener('resize', () => { g = fit(cv, 0.55); draw(); });
    addEventListener('themechange', draw);
    compute(); draw();
  })();
})();
})();
