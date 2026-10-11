// ======== SANDBOX 4: MODEL INSPECTOR ========
// Same model as 04_inspection_and_visualization.py: base at z = 1, upper arm 0.5 m, forearm 0.4 m,
// both hinges about +y, a 1 kg block at the forearm tip, hand site at the tip, a loose box at x = 1.2.
// Forward kinematics below matches MuJoCo's xpos / xquat / xipos / site_xpos / cam_xpos to 4 decimals
// (checked by lessons/04-inspection-and-visualization/check_fk.py, which runs this exact block in node).
(() => {
  // FK-BEGIN
  const L4FK = (() => {
    const L1 = 0.5, L2 = 0.4, Z0 = 1.0;
    const C1 = 0.25, C2 = 0.2891237521251078;          // model.body_ipos x of upper_arm, forearm (m)
    const TRACK = [0.0008762478748922, -1.2, 0.2];     // track camera offset from the forearm's centre of mass (m)
    const BOXQ = [Math.cos(Math.PI / 12), 0, 0, Math.sin(Math.PI / 12)]; // 30 degrees about z
    return (q1, q2) => {
      const c1 = Math.cos(q1), s1 = Math.sin(q1), c12 = Math.cos(q1 + q2), s12 = Math.sin(q1 + q2);
      const elbow = [L1 * c1, 0, Z0 - L1 * s1];
      const fipos = [elbow[0] + C2 * c12, 0, elbow[2] - C2 * s12];
      return {
        xpos: [[0, 0, 0], [0, 0, Z0], elbow, [1.2, 0, 0.1]],
        xquat: [[1, 0, 0, 0], [Math.cos(q1 / 2), 0, Math.sin(q1 / 2), 0], [Math.cos((q1 + q2) / 2), 0, Math.sin((q1 + q2) / 2), 0], BOXQ],
        xipos: [[0, 0, 0], [C1 * c1, 0, Z0 - C1 * s1], fipos, [1.2, 0, 0.1]],
        hand: [elbow[0] + L2 * c12, 0, elbow[2] - L2 * s12],
        cam: [[0.6, -3.0, 0.7], [fipos[0] + TRACK[0], TRACK[1], fipos[2] + TRACK[2]]],
      };
    };
  })();
  // FK-END
  const $ = id => document.getElementById(id);
  const n4 = v => (Math.abs(v) < 5e-5 ? 0 : v).toFixed(4);
  const vec = a => '(' + a.map(n4).join(', ') + ')';
  const quat = a => '[' + a.map(n4).join(', ') + ']';
  const NAMES = ['world', 'upper_arm', 'forearm', 'box'];
  const CAMS = ['side (fixed)', 'track (trackcom)'];
  const S = 160, X = x => 165 + S * x, Y = z => 420 - S * z;
  let sel = 2, cam = 0;

  function update() {
    const q1 = +$('s4Q1').value, q2 = +$('s4Q2').value, k = L4FK(q1, q2);
    $('s4Q1Val').textContent = q1.toFixed(2); $('s4Q2Val').textContent = q2.toFixed(2);
    const e = k.xpos[2], h = k.hand;
    const set = (id, at) => { const el = $(id); Object.entries(at).forEach(([a, v]) => el.setAttribute(a, v)); };
    set('s4UpperRod', { x1: X(0), y1: Y(1), x2: X(e[0]), y2: Y(e[2]) });
    set('s4ForeRod', { x1: X(e[0]), y1: Y(e[2]), x2: X(h[0]), y2: Y(h[2]) });
    // positive angle about +y turns x toward -z: on screen that is clockwise
    set('s4Weight', { transform: `translate(${X(h[0])},${Y(h[2])}) rotate(${(q1 + q2) * 180 / Math.PI})` });
    set('s4O1', { cx: X(0), cy: Y(1) }); set('s4O2', { cx: X(e[0]), cy: Y(e[2]) });
    set('s4C1', { transform: `translate(${X(k.xipos[1][0])},${Y(k.xipos[1][2])})` });
    set('s4C2', { transform: `translate(${X(k.xipos[2][0])},${Y(k.xipos[2][2])})` });
    set('s4Hand', { cx: X(h[0]), cy: Y(h[2]) }); set('s4HandLbl', { x: X(h[0]) + 8, y: Y(h[2]) + 16 });
    set('s4CamSide', { transform: `translate(${X(k.cam[0][0])},${Y(k.cam[0][2])})` });
    set('s4CamTrack', { transform: `translate(${X(k.cam[1][0])},${Y(k.cam[1][2])})` });
    ['s4CamSide', 's4CamTrack'].forEach((id, i) => $(id).querySelectorAll('rect, text').forEach(el => {
      const col = i === cam ? 'var(--teal)' : 'var(--text-dim)';
      el.setAttribute(el.tagName === 'rect' ? 'stroke' : 'fill', col);
    }));
    // selected body: ring + its own frame arrows, turned by its orientation
    const p = k.xpos[sel], ang = sel === 1 ? q1 : sel === 2 ? q1 + q2 : 0;
    set('s4Sel', { cx: X(p[0]), cy: Y(p[2]) });
    set('s4Frame', { transform: `translate(${X(p[0])},${Y(p[2])}) rotate(${ang * 180 / Math.PI})` });
    $('s4Name').textContent = NAMES[sel]; $('s4Id').textContent = sel;
    $('s4Xpos').textContent = vec(k.xpos[sel]); $('s4Xquat').textContent = quat(k.xquat[sel]);
    $('s4Xipos').textContent = vec(k.xipos[sel]); $('s4HandPos').textContent = vec(h);
    $('s4CamName').textContent = CAMS[cam]; $('s4CamPos').textContent = vec(k.cam[cam]);
  }
  function pick(attr, v) {
    document.querySelectorAll(`#sandbox4 [${attr}]`).forEach(b => {
      const on = +b.getAttribute(attr) === v;
      b.classList.toggle('btn-primary', on); b.setAttribute('aria-pressed', String(on));
    });
  }
  const pose = (a, b) => { $('s4Q1').value = a; $('s4Q2').value = b; update(); };
  ['s4Q1', 's4Q2'].forEach(id => $(id).addEventListener('input', update));
  $('s4Script').addEventListener('click', () => pose(0.5, 0.8));
  $('s4Zero').addEventListener('click', () => pose(0, 0));
  $('s4Q3').addEventListener('click', () => pose(0, 1.57));
  document.querySelectorAll('#sandbox4 [data-s4body]').forEach(b => b.addEventListener('click', () => { sel = +b.dataset.s4body; pick('data-s4body', sel); update(); }));
  document.querySelectorAll('#sandbox4 [data-s4cam]').forEach(b => b.addEventListener('click', () => { cam = +b.dataset.s4cam; pick('data-s4cam', cam); update(); }));
  document.querySelectorAll('#sandbox4 svg [data-body]').forEach(g => g.addEventListener('click', () => { sel = +g.dataset.body; pick('data-s4body', sel); update(); }));
  update();
})();

// ======== LESSON 4 EXPLAINER ========
(() => {
  // Only redraw this lesson's canvases while it is on screen (hidden canvases have zero size).
  const addEventListener = (type, fn, opts) => window.addEventListener(type, e => { if (document.getElementById('lesson4').classList.contains('active')) fn(e); }, opts);
  const $ = id => document.getElementById(id);
  const n4 = v => (Math.abs(v) < 5e-5 ? 0 : v).toFixed(4);
  const vec = a => '(' + a.map(n4).join(', ') + ')';
  // Same arm as the script. C2 = forearm centre of mass along the forearm (model.body_ipos, m).
  const C2 = 0.2891237521251078;
  function arm(q1, q2) {
    const c1 = Math.cos(q1), s1 = Math.sin(q1), c12 = Math.cos(q1 + q2), s12 = Math.sin(q1 + q2);
    const e = [0.5 * c1, 1 - 0.5 * s1];
    return { sh: [0, 1], e, h: [e[0] + 0.4 * c12, e[1] - 0.4 * s12], i1: [0.25 * c1, 1 - 0.25 * s1], i2: [e[0] + C2 * c12, e[1] - C2 * s12] };
  }
  // world (x, z) in metres -> canvas pixels, fitted to the box [x0, x1] x [z0, z1]
  function view(w, h, x0, x1, z0, z1) {
    const s = Math.min((w - 20) / (x1 - x0), (h - 20) / (z1 - z0));
    const ox = (w - s * (x1 - x0)) / 2 - s * x0, oz = (h + s * (z1 - z0)) / 2 + s * z0;
    return { s, P: (x, z) => [ox + s * x, oz - s * z] };
  }
  function seg(c, a, b, col, wd) { c.strokeStyle = col; c.lineWidth = wd; c.lineCap = 'round'; c.beginPath(); c.moveTo(...a); c.lineTo(...b); c.stroke(); }
  function dot(c, p, r, col) { c.fillStyle = col; c.beginPath(); c.arc(p[0], p[1], r, 0, 7); c.fill(); }
  function comMark(c, p, col) { c.strokeStyle = col; c.lineWidth = 1.5; c.beginPath(); c.arc(p[0], p[1], 6, 0, 7); c.moveTo(p[0] - 6, p[1]); c.lineTo(p[0] + 6, p[1]); c.moveTo(p[0], p[1] - 6); c.lineTo(p[0], p[1] + 6); c.stroke(); }
  function frameArrows(c, p, ang, len) {
    // body x axis in world = (cos a, -sin a) in (x, z); z axis = (sin a, cos a); canvas y points down
    arrow(c, p[0], p[1], p[0] + len * Math.cos(ang), p[1] + len * Math.sin(ang), css('--x'), 2, 7);
    arrow(c, p[0], p[1], p[0] + len * Math.sin(ang), p[1] - len * Math.cos(ang), css('--z'), 2, 7);
  }
  // draw the whole model; hl lets one part stand out
  function scene(c, V, q1, q2, hl) {
    const A = arm(q1, q2), P = V.P, s = V.s, dim = hl ? 0.3 : 1;
    const on = (kind, id) => hl && hl.kind === kind && hl.id === id;
    c.globalAlpha = 1;
    seg(c, P(-0.3, 0), P(1.5, 0), css('--grid'), 2);
    text(c, 'floor, z = 0', P(-0.28, 0)[0], P(0, 0)[1] + 10, css('--dim'), 10);
    c.globalAlpha = on('body', 0) || on('geom', 0) ? 1 : 0.7;
    arrow(c, ...P(0, 0), ...P(0.22, 0), css('--x'), 2, 7); arrow(c, ...P(0, 0), ...P(0, 0.22), css('--z'), 2, 7);
    if (!hl) text(c, 'world', P(0.03, 0)[0], P(0, 0.06)[1], css('--dim'), 10);
    // box silhouette: 0.2 m cube turned 30 degrees about z is 0.273 m wide from the side
    c.globalAlpha = on('body', 3) || on('geom', 4) || on('joint', 2) ? 1 : dim;
    const b0 = P(1.2 - 0.1366, 0.2), b1 = P(1.2 + 0.1366, 0);
    c.fillStyle = css('--blue'); c.globalAlpha *= 0.5; c.fillRect(b0[0], b0[1], b1[0] - b0[0], b1[1] - b0[1]);
    c.globalAlpha = on('body', 3) || on('geom', 4) || on('joint', 2) ? 1 : dim;
    c.strokeStyle = css('--blue'); c.lineWidth = 1.5; c.strokeRect(b0[0], b0[1], b1[0] - b0[0], b1[1] - b0[1]);
    c.globalAlpha = on('body', 1) || on('geom', 1) ? 1 : dim;
    seg(c, P(...A.sh), P(...A.e), css('--teal'), Math.max(4, 0.08 * s));
    c.globalAlpha = on('body', 2) || on('geom', 2) ? 1 : dim;
    seg(c, P(...A.e), P(...A.h), css('--orange'), Math.max(3, 0.06 * s));
    c.globalAlpha = on('body', 2) || on('geom', 3) ? 1 : dim;
    const hp = P(...A.h), half = 0.05 * s;
    c.save(); c.translate(...hp); c.rotate(q1 + q2); c.fillStyle = css('--purple'); c.fillRect(-half, -half, 2 * half, 2 * half); c.restore();
    c.globalAlpha = on('site', 0) ? 1 : dim; dot(c, hp, 4, css('--red'));
    c.globalAlpha = on('joint', 0) ? 1 : dim; dot(c, P(...A.sh), 5, css('--bright'));
    c.globalAlpha = on('joint', 1) ? 1 : dim; dot(c, P(...A.e), 4.5, css('--bright'));
    c.globalAlpha = 1;
    return A;
  }

  /* ---------- Section 1: names and ids ---------- */
  (() => {
    const cv = $('l4cvA'); let g = fit(cv, 0.8);
    // Lists and zero-pose numbers printed by 04_inspection_and_visualization.py
    const L = {
      body: { obj: 'mjOBJ_BODY', items: [['world', [0, 0], 'data.xpos[0] = (0.0000, 0.0000, 0.0000) m, world\nBody 0 is always the world. It never moves.'],
        ['upper_arm', [0, 1], 'data.xpos[1] = (0.0000, 0.0000, 1.0000) m, world\nThe shoulder: from pos="0 0 1".'],
        ['forearm', [0.5, 1], 'data.xpos[2] = (0.5000, 0.0000, 1.0000) m, world\nThe elbow: 0.5 m along the upper arm.'],
        ['box', [1.2, 0.1], 'data.xpos[3] = (1.2000, 0.0000, 0.1000) m, world\nResting on the floor (half size 0.1 m).']] },
      joint: { obj: 'mjOBJ_JOINT', items: [['shoulder', [0, 1], 'model.jnt_qposadr[0] = 0\nso its angle lives in data.qpos[0] (rad).'],
        ['elbow', [0.5, 1], 'model.jnt_qposadr[1] = 1\nso its angle lives in data.qpos[1] (rad).'],
        ['box_free', [1.2, 0.1], 'model.jnt_qposadr[2] = 2\nso it owns data.qpos[2:9]: 7 numbers.']] },
      geom: { obj: 'mjOBJ_GEOM', items: [['floor', [0.6, 0], 'Belongs to body 0 (world).\nA flat plane at z = 0.'],
        ['upper_geom', [0.25, 1], 'Belongs to body 1 (upper_arm).\nCapsule, 0.5 m long, radius 0.04 m.'],
        ['forearm_geom', [0.7, 1], 'Belongs to body 2 (forearm).\nCapsule, 0.4 m long, radius 0.03 m.'],
        ['weight', [0.9, 1], 'Also belongs to body 2 (forearm).\n0.1 m cube, 1 kg. One body, two geoms.'],
        ['box_geom', [1.2, 0.1], 'Belongs to body 3 (box).\n0.2 m cube.']] },
      site: { obj: 'mjOBJ_SITE', items: [['hand', [0.9, 1], 'data.site_xpos[0] = (0.9000, 0.0000, 1.0000) m, world\nA marker at the forearm tip. No mass.']] },
      camera: { obj: 'mjOBJ_CAMERA', items: [['side', [0.6, 0.7], 'data.cam_xpos[0] = (0.6000, -3.0000, 0.7000) m\nFixed, 3 m in front of the arm.'],
        ['track', [0.79, 1.2], 'data.cam_xpos[1] = (0.7900, -1.2000, 1.2000) m\nmode trackcom: follows the forearm.']] },
    };
    const MAP = { body: 'body', joint: 'joint', geom: 'geom', site: 'site', camera: 'camera' };
    let kind = 'body', id = 2;
    function draw() {
      const c = g.ctx; c.clearRect(0, 0, g.w, g.h);
      const V = view(g.w, g.h, -0.35, 1.5, -0.12, 1.35);
      scene(c, V, 0, 0, kind === 'camera' ? { kind: 'none' } : { kind: MAP[kind], id });
      L[kind].items.forEach(([nm, p], i) => {
        const q = V.P(...p), lab = i + ':' + nm, isOn = i === id;
        if (kind === 'camera') { c.strokeStyle = isOn ? css('--teal') : css('--dim'); c.lineWidth = 1.5; c.strokeRect(q[0] - 7, q[1] - 5, 14, 10); }
        const dy = kind === 'joint' || kind === 'body' ? -16 - (i % 2) * 14 : 18 + (i % 2) * 14;
        text(c, lab, q[0], q[1] + dy, isOn ? css('--teal') : css('--dim'), isOn ? 12 : 10, 'center', isOn ? 700 : 500);
      });
      const [nm] = L[kind].items[id];
      $('l4aOut').textContent = `mujoco.mj_name2id(model, mujoco.mjtObj.${L[kind].obj}, "${nm}")\n  = ${id}\n\n${L[kind].items[id][2]}`;
    }
    function names() {
      const box = $('l4aNames'); box.textContent = '';
      L[kind].items.forEach(([nm], i) => {
        const b = document.createElement('button'); b.type = 'button'; b.textContent = nm; if (i === id) b.className = 'primary';
        b.addEventListener('click', () => { if (!b.isConnected) return; id = i; box.querySelectorAll('button').forEach((x, j) => x.className = j === i ? 'primary' : ''); draw(); flash($('l4aOut')); });
        box.appendChild(b);
      });
    }
    document.querySelectorAll('#l4aKinds button').forEach(b => b.addEventListener('click', () => {
      kind = b.dataset.kind; id = kind === 'body' ? 2 : 0;
      document.querySelectorAll('#l4aKinds button').forEach(x => x.className = x === b ? 'primary' : '');
      names(); draw();
    }));
    addEventListener('resize', () => { g = fit(cv, 0.8); draw(); }); addEventListener('themechange', draw);
    names(); draw();
  })();

  /* ---------- Section 3: xpos, xipos, site_xpos ---------- */
  (() => {
    const cv = $('l4cvB'); let g = fit(cv, 0.85);
    function draw(changed) {
      const q1 = +$('l4bQ1').value, q2 = +$('l4bQ2').value, c = g.ctx;
      $('l4bO1').textContent = q1.toFixed(2) + ' rad'; $('l4bO2').textContent = q2.toFixed(2) + ' rad';
      c.clearRect(0, 0, g.w, g.h);
      const V = view(g.w, g.h, -0.95, 1.4, -0.05, 1.95), P = V.P;
      const A = scene(c, V, q1, q2, null);
      frameArrows(c, P(...A.sh), q1, 26); frameArrows(c, P(...A.e), q1 + q2, 26);
      dot(c, P(...A.sh), 5, css('--bright')); dot(c, P(...A.e), 5, css('--bright'));
      comMark(c, P(...A.i1), css('--yellow')); comMark(c, P(...A.i2), css('--yellow'));
      text(c, 'upper_arm xpos', P(...A.sh)[0] - 8, P(...A.sh)[1] - 14, css('--bright'), 10, 'right');
      text(c, 'forearm xpos', P(...A.e)[0] + 10, P(...A.e)[1] - 14, css('--bright'), 10);
      text(c, 'xipos', P(...A.i2)[0] + 10, P(...A.i2)[1] + 2, css('--yellow'), 10);
      text(c, 'hand', P(...A.h)[0] + 10, P(...A.h)[1] + 14, css('--red'), 10);
      text(c, 'world frame, metres', 10, 12, css('--dim'), 10);
      const set = (id, v) => { $(id).querySelector('.val').textContent = v; };
      set('l4bX', vec([A.e[0], 0, A.e[1]])); set('l4bI', vec([A.i2[0], 0, A.i2[1]])); set('l4bH', vec([A.h[0], 0, A.h[1]]));
      if (changed) ['l4bX', 'l4bI', 'l4bH'].forEach(id => flash($(id)));
    }
    const pose = (a, b) => { $('l4bQ1').value = a; $('l4bQ2').value = b; draw(true); };
    $('l4bQ1').addEventListener('input', () => draw(true)); $('l4bQ2').addEventListener('input', () => draw(true));
    $('l4bScript').addEventListener('click', () => pose(0.5, 0.8)); $('l4bZero').addEventListener('click', () => pose(0, 0));
    addEventListener('resize', () => { g = fit(cv, 0.85); draw(); }); addEventListener('themechange', () => draw());
    draw();
  })();

  /* ---------- Section 4: quaternion about y ---------- */
  (() => {
    const cv = $('l4cvQ'); let g = fit(cv, 0.8);
    function draw(changed) {
      const a = +$('l4qA').value, c = g.ctx, W = g.w, H = g.h;
      $('l4qO').textContent = a.toFixed(2) + ' rad';
      const q = [Math.cos(a / 2), 0, Math.sin(a / 2), 0];
      c.clearRect(0, 0, W, H);
      // left: the turned frame
      const cx = W * 0.27, cy = H * 0.47, R = Math.min(W * 0.17, H * 0.3);
      c.setLineDash([4, 4]); seg(c, [cx, cy], [cx + R, cy], css('--grid'), 1); seg(c, [cx, cy], [cx, cy - R], css('--grid'), 1); c.setLineDash([]);
      frameArrows(c, [cx, cy], a, R);
      text(c, 'x', cx + (R + 12) * Math.cos(a), cy + (R + 12) * Math.sin(a), css('--x'), 12, 'center');
      text(c, 'z', cx + (R + 12) * Math.sin(a), cy - (R + 12) * Math.cos(a), css('--z'), 12, 'center');
      // the skewer: +y points into the screen (circle with a cross)
      c.strokeStyle = css('--y'); c.lineWidth = 2; c.beginPath(); c.arc(cx, cy, 8, 0, 7); c.moveTo(cx - 5.5, cy - 5.5); c.lineTo(cx + 5.5, cy + 5.5); c.moveTo(cx + 5.5, cy - 5.5); c.lineTo(cx - 5.5, cy + 5.5); c.stroke();
      c.strokeStyle = css('--yellow'); c.lineWidth = 2; c.beginPath(); c.arc(cx, cy, R * 0.45, Math.min(0, a), Math.max(0, a)); c.stroke();
      text(c, 'a = ' + a.toFixed(2) + ' rad', cx, Math.min(H - 12, cy + R + 26), css('--yellow'), 11, 'center');
      text(c, 'skewer: +y, into the screen', 8, 12, css('--y'), 10);
      // right: four bars
      const bx = W * 0.6, bw = W * 0.34, top = 24, rowH = (H - 48) / 4, labels = ['w', 'x', 'y', 'z'], cols = [css('--teal'), css('--x'), css('--y'), css('--z')];
      labels.forEach((L, i) => {
        const y0 = top + i * rowH + rowH / 2, mid = bx + bw / 2;
        seg(c, [bx, y0], [bx + bw, y0], css('--grid'), 1); seg(c, [mid, y0 - 8], [mid, y0 + 8], css('--dim'), 1);
        c.fillStyle = cols[i]; const len = q[i] * bw / 2; c.fillRect(Math.min(mid, mid + len), y0 - 7, Math.abs(len), 14);
        text(c, L, bx - 12, y0, cols[i], 12, 'center', 700);
        text(c, n4(q[i]), mid + (q[i] >= 0 ? -6 : 6), y0 - 16, css('--bright'), 11, q[i] >= 0 ? 'right' : 'left');
      });
      text(c, '-1', bx, H - 12, css('--dim'), 9, 'center'); text(c, '0', bx + bw / 2, H - 12, css('--dim'), 9, 'center'); text(c, '+1', bx + bw, H - 12, css('--dim'), 9, 'center');
      $('l4qQ').querySelector('.val').textContent = '[' + q.map(n4).join(', ') + ']';
      $('l4qM').querySelector('.val').textContent = vec([Math.cos(a), 0, -Math.sin(a)]);
      if (changed) { flash($('l4qQ')); flash($('l4qM')); }
    }
    const setA = v => { $('l4qA').value = v; draw(true); };
    $('l4qA').addEventListener('input', () => draw(true));
    $('l4qNone').addEventListener('click', () => setA(0)); $('l4qUp').addEventListener('click', () => setA(0.5)); $('l4qFore').addEventListener('click', () => setA(1.3));
    addEventListener('resize', () => { g = fit(cv, 0.8); draw(); }); addEventListener('themechange', () => draw());
    draw();
  })();

  /* ---------- Section 5: qpos and qvel seats ---------- */
  (() => {
    const cv = $('l4cvR'), asp = () => (cv.clientWidth < 520 ? 0.62 : 0.42); let g = fit(cv, asp());
    // qpos at pose (0.5, 0.8) as printed by the script; qvel is all zero
    const QPOS = [0.5, 0.8, 1.2, 0, 0.1, 0.9659, 0, 0, 0.2588];
    const QL = ['angle', 'angle', 'x', 'y', 'z', 'w', 'qx', 'qy', 'qz'], VL = ['spin', 'spin', 'vx', 'vy', 'vz', 'wx', 'wy', 'wz'];
    const OWN_Q = [0, 1, 2, 2, 2, 2, 2, 2, 2], OWN_V = [0, 1, 2, 2, 2, 2, 2, 2];
    const INFO = [
      'joint 0 "shoulder", type hinge\nmodel.jnt_qposadr[0] = 0, so qpos[0:1]: 1 number, the angle (rad)\nmodel.jnt_dofadr[0] = 0, so qvel[0:1]: 1 number, the spin speed (rad/s)',
      'joint 1 "elbow", type hinge\nmodel.jnt_qposadr[1] = 1, so qpos[1:2]: 1 number, the angle (rad)\nmodel.jnt_dofadr[1] = 1, so qvel[1:2]: 1 number, the spin speed (rad/s)',
      'joint 2 "box_free", type free\nmodel.jnt_qposadr[2] = 2, so qpos[2:9]: 7 numbers\n  qpos[2:5] = (1.2000, 0.0000, 0.1000)  box position, world, m\n  qpos[5:9] = [0.9659, 0.0000, 0.0000, 0.2588]  box quaternion [w, x, y, z]\nmodel.jnt_dofadr[2] = 2, so qvel[2:8]: 6 numbers\n  qvel[2:5] linear velocity (world, m/s), qvel[5:8] angular velocity (box frame, rad/s)',
    ];
    let j = 2;
    function draw() {
      const c = g.ctx, W = g.w, H = g.h, cols = [css('--teal'), css('--orange'), css('--blue')];
      c.clearRect(0, 0, W, H);
      const left = 52, cw = (W - left - 8) / 9, ch = Math.min(34, H * 0.2);
      const row = (y, vals, labs, own, name, n) => {
        text(c, name, 4, y + ch / 2, css('--bright'), 11, 'left', 700);
        for (let i = 0; i < n; i++) {
          const x = left + i * cw, mine = own[i] === j;
          c.globalAlpha = mine ? 1 : 0.3; c.fillStyle = cols[own[i]]; c.globalAlpha *= 0.25; c.fillRect(x + 1, y, cw - 2, ch);
          c.globalAlpha = mine ? 1 : 0.35; c.strokeStyle = cols[own[i]]; c.lineWidth = mine ? 2 : 1; c.strokeRect(x + 1, y, cw - 2, ch);
          text(c, vals[i], x + cw / 2, y + ch / 2, css('--bright'), Math.min(11, cw / 4.2), 'center');
          text(c, '[' + i + ']', x + cw / 2, y - 8, css('--dim'), 9, 'center');
          text(c, labs[i], x + cw / 2, y + ch + 10, cols[own[i]], Math.min(9, cw / 3.6), 'center');
          c.globalAlpha = 1;
        }
      };
      const y1 = 22, y2 = y1 + ch + 44;
      row(y1, QPOS.map(v => v.toFixed(4).replace(/0+$/, '').replace(/\.$/, '.0')), QL, OWN_Q, 'qpos', 9);
      row(y2, new Array(8).fill('0'), VL, OWN_V, 'qvel', 8);
      text(c, 'nq = 9, nv = 8', W - 6, H - 8, css('--dim'), 10, 'right');
      $('l4rOut').textContent = INFO[j];
    }
    document.querySelectorAll('#l4rJ button').forEach(b => b.addEventListener('click', () => {
      j = +b.dataset.j; document.querySelectorAll('#l4rJ button').forEach(x => x.className = x === b ? 'primary' : ''); draw(); flash($('l4rOut'));
    }));
    addEventListener('resize', () => { g = fit(cv, asp()); draw(); }); addEventListener('themechange', draw);
    draw();
  })();
})();
