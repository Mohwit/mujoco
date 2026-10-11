// ======== SANDBOX 11 + LESSON 11 (one outer IIFE so both share the model core) ========
(() => {
// CORE11-BEGIN
// Same model as 11_building_bigger_models.py (and 11_scene.xml + 11_arm.xml + 11_shelf.xml).
// The resolver does what MuJoCo's compiler does for each geom attribute: look on the element,
// then its class, then that class's parent ... then main, then MuJoCo's built-in value.
// Checked against the compiled MjModel for all 64 toggle combinations by 11_check_browser.py.
const L11 = (() => {
  const BUILTIN = { type: 'sphere', size: [0, 0, 0], rgba: [0.5, 0.5, 0.5, 1], friction: [1, 0.005, 0.0001], condim: 3, contype: 1, conaffinity: 1 };
  const ATTRS = ['type', 'size', 'rgba', 'friction', 'condim', 'contype', 'conaffinity'];
  const CLASSES = {
    main: { parent: null, geom: { rgba: [0.75, 0.78, 0.76, 1] } },
    robot: { parent: 'main', geom: { rgba: [0.1, 0.7, 0.6, 1], contype: 0, conaffinity: 0 } },
    finger: { parent: 'robot', geom: { friction: [2, 0.05, 0.01], condim: 4, contype: 1, conaffinity: 1 } },
    furniture: { parent: 'main', geom: { type: 'box', friction: [0.5, 0.005, 0.001], rgba: [0.55, 0.35, 0.15, 1] } },
    object: { parent: 'main', geom: { type: 'box', size: [0.05, 0.05, 0.05], friction: [0.6, 0.01, 0.001], rgba: [0.6, 0.45, 0.25, 1] } },
  };
  // cls = the class MuJoCo uses (own class="...", else nearest childclass, else main).
  // exp = attributes written on the element itself. Capsule sizes are the compiled (radius, half-length, 0) from fromto.
  // c = centre in the world (m) for the fixed geoms; the arm and red box are placed by the keyframe.
  const GEOMS = [
    { name: 'floor', cls: 'main', via: 'no class', exp: { type: 'plane', size: [2, 2, 0.05] } },
    { name: 'upper_geom', cls: 'robot', via: 'childclass', exp: { type: 'capsule', size: [0.04, 0.25, 0] }, fromto: true },
    { name: 'forearm_geom', cls: 'robot', via: 'childclass', exp: { type: 'capsule', size: [0.03, 0.2, 0], rgba: [0.9, 0.5, 0.2, 1] }, fromto: true },
    { name: 'weight', cls: 'finger', via: 'own class', exp: { type: 'box', size: [0.05, 0.05, 0.05] } },
    { name: 'board', cls: 'furniture', via: 'childclass', exp: { size: [0.15, 0.35, 0.02] }, c: [0.75, 0, 0.48] },
    { name: 'post_left', cls: 'furniture', via: 'childclass', exp: { size: [0.15, 0.02, 0.23] }, c: [0.75, 0.33, 0.23] },
    { name: 'post_right', cls: 'furniture', via: 'childclass', exp: { size: [0.15, 0.02, 0.23] }, c: [0.75, -0.33, 0.23] },
    { name: 'box_a', cls: 'object', via: 'own class', exp: {}, c: [0.75, 0.24, 0.55] },
    { name: 'box_b', cls: 'object', via: 'own class', exp: {}, c: [0.75, 0.12, 0.55] },
    { name: 'box_c', cls: 'object', via: 'own class', exp: {}, c: [0.75, -0.12, 0.55] },
    { name: 'box_d', cls: 'object', via: 'own class', exp: {}, c: [0.75, -0.24, 0.55] },
    { name: 'red_box_geom', cls: 'object', via: 'own class', exp: { rgba: [0.85, 0.15, 0.1, 1] } },
  ];
  // The six class settings the sandbox can flip: [class, attributes, new value(s)].
  const TOGGLES = [
    { cls: 'main', set: { rgba: [0.2, 0.4, 0.9, 1] }, label: 'main rgba' },
    { cls: 'robot', set: { rgba: [0.2, 0.4, 0.9, 1] }, label: 'robot rgba' },
    { cls: 'robot', set: { contype: 1, conaffinity: 1 }, label: 'robot contype and conaffinity' },
    { cls: 'finger', set: { friction: [1, 0.05, 0.01] }, label: 'finger friction' },
    { cls: 'furniture', set: { friction: [1, 0.005, 0.001] }, label: 'furniture friction' },
    { cls: 'object', set: { friction: [0.3, 0.01, 0.001] }, label: 'object friction' },
  ];
  // Keyframes from the XML (qpos has nq = 9 numbers: shoulder, elbow, red box x y z, red box quaternion w x y z).
  const BOX = [0.75, 0, 0.55, 1, 0, 0, 0];
  const KEYS = {
    qpos0: { qpos: [0, 0, ...BOX], qvel0: 0, time: 0 },
    home: { qpos: [-1.0, 2.0, ...BOX], qvel0: 0, time: 0 },
    lesson4: { qpos: [0.5, 0.8, ...BOX], qvel0: 0, time: 0 },
    reach: { qpos: [-0.12, 1.0, ...BOX], qvel0: 0, time: 0 },
    swing: { qpos: [-1.0, 2.0, ...BOX], qvel0: 3, time: 2 },
  };
  function classes(on = []) {
    const out = JSON.parse(JSON.stringify(CLASSES));
    TOGGLES.forEach((t, i) => { if (on[i]) Object.assign(out[t.cls].geom, JSON.parse(JSON.stringify(t.set))); });
    return out;
  }
  function chain(cls, C = CLASSES) { const out = []; while (cls) { out.push(cls); cls = C[cls].parent; } return out; }
  function level(i, c) { return c === 'main' ? 'main (global)' : (i === 0 ? 'class ' + c : 'parent class ' + c); }
  // Resolve every attribute of one geom: { attr: { v: value, lvl: where it came from } }.
  function resolve(name, on = []) {
    const C = classes(on), g = GEOMS.find(x => x.name === name), out = {};
    ATTRS.forEach(a => {
      if (a in g.exp) { out[a] = { v: g.exp[a], lvl: 'explicit' }; return; }
      const ch = chain(g.cls, C);
      for (let i = 0; i < ch.length; i++) if (a in C[ch[i]].geom) { out[a] = { v: C[ch[i]].geom[a], lvl: level(i, ch[i]) }; return; }
      out[a] = { v: BUILTIN[a], lvl: 'built-in' };
    });
    return out;
  }
  // The search, one rung at a time, for the ladder picture.
  function rungs(name, a, on = []) {
    const C = classes(on), g = GEOMS.find(x => x.name === name);
    const out = [{ lvl: 'explicit', where: 'on the <geom> itself', has: a in g.exp, v: g.exp[a] }];
    chain(g.cls, C).forEach((c, i) => out.push({ lvl: level(i, c), where: '<default class="' + c + '">', has: a in C[c].geom, v: C[c].geom[a] }));
    out.push({ lvl: 'built-in', where: 'MuJoCo\'s own value', has: true, v: BUILTIN[a] });
    return out;
  }
  // Forward kinematics of the lesson 4 arm (shoulder at world (0, 0, 1), hinges about +y).
  function fk(q1, q2) {
    const th = q1 + q2;
    const elbow = [0.5 * Math.cos(q1), 0, 1 - 0.5 * Math.sin(q1)];
    const hand = [elbow[0] + 0.4 * Math.cos(th), 0, elbow[2] - 0.4 * Math.sin(th)];
    return { elbow, hand, th, R: [Math.cos(th), Math.sin(th), -Math.sin(th), Math.cos(th)] }; // forearm xmat entries xx, xz, zx, zz
  }
  return { BUILTIN, ATTRS, CLASSES, GEOMS, TOGGLES, KEYS, classes, chain, resolve, rungs, fk };
})();
// CORE11-END

const show = v => Array.isArray(v) ? v.map(x => String(x)).join(' ') : String(v);
// rgba values of this model -> page colour variables (the page never hard-codes a colour)
const COLOR = { '0.75 0.78 0.76 1': '--dim', '0.1 0.7 0.6 1': '--teal', '0.2 0.4 0.9 1': '--blue', '0.9 0.5 0.2 1': '--orange',
  '0.55 0.35 0.15 1': '--yellow', '0.6 0.45 0.25 1': '--purple', '0.85 0.15 0.1 1': '--red' };
const colVar = rgba => COLOR[show(rgba)] || '--dim';

// ======== SANDBOX 11: DEFAULT-CLASS RESOLVER + KEYFRAME PICKER ========
(() => {
  const $ = id => document.getElementById(id);
  if (!$('sandbox11')) return;
  const NS = 'http://www.w3.org/2000/svg', S = 220, OX = 75, OY = 400;
  const P = (x, y, z) => [OX + S * (x + 0.8 * y), OY - S * (z + 0.45 * y)];   // oblique view: +y drawn up and to the right
  const VIEW = [0.8, -1, 0.45];                                                 // direction towards the viewer
  const on = [false, false, false, false, false, false];
  let key = 'reach', sel = 'weight', changed = new Set();

  function el(tag, attrs, parent) { const e = document.createElementNS(NS, tag); for (const k in attrs) e.setAttribute(k, attrs[k]); parent.appendChild(e); return e; }
  // a box with centre c, half sizes h, turned th about +y; draws only the faces that face the viewer
  function cuboid(g, c, h, th, col, name) {
    const ct = Math.cos(th), st = Math.sin(th);
    const W = (lx, ly, lz) => [c[0] + ct * lx + st * lz, c[1] + ly, c[2] - st * lx + ct * lz];
    const faces = [[0, 1], [0, -1], [1, 1], [1, -1], [2, 1], [2, -1]];
    faces.forEach(([ax, s]) => {
      const n = [0, 0, 0]; n[ax] = s;
      const nw = [ct * n[0] + st * n[2], n[1], -st * n[0] + ct * n[2]];
      const dot = nw[0] * VIEW[0] + nw[1] * VIEW[1] + nw[2] * VIEW[2];
      if (dot <= 0) return;
      const o = [0, 1, 2].filter(i => i !== ax), pts = [];
      [[-1, -1], [1, -1], [1, 1], [-1, 1]].forEach(([a, b]) => { const l = [0, 0, 0]; l[ax] = s * h[ax]; l[o[0]] = a * h[o[0]]; l[o[1]] = b * h[o[1]]; pts.push(P(...W(...l))); });
      el('polygon', { points: pts.map(p => p.map(v => v.toFixed(1)).join(',')).join(' '), fill: `var(${col})`, 'fill-opacity': ax === 1 ? 0.55 : (ax === 2 ? 0.8 : 0.35), stroke: `var(${col})`, 'stroke-width': 1, 'data-geom': name, style: 'cursor:pointer' }, g);
    });
  }
  function outline(g, pts, dashed) {
    el('polygon', { points: pts.map(p => p.map(v => v.toFixed(1)).join(',')).join(' '), fill: 'none', stroke: 'var(--text-bright)', 'stroke-width': dashed ? 1.5 : 3, 'stroke-dasharray': dashed ? '4 3' : 'none', 'pointer-events': 'none' }, g);
  }
  function hull(c, h) { // projected outline of an axis-aligned box (front face + top + right): six-point silhouette
    const [x, y, z] = c, [a, b, d] = h;
    return [P(x - a, y - b, z - d), P(x + a, y - b, z - d), P(x + a, y + b, z - d), P(x + a, y + b, z + d), P(x - a, y + b, z + d), P(x - a, y - b, z + d)];
  }

  function render() {
    const scene = $('s11Scene'); scene.textContent = '';
    const K = L11.KEYS[key], f = L11.fk(K.qpos[0], K.qpos[1]), R = {};
    L11.GEOMS.forEach(g => { R[g.name] = L11.resolve(g.name, on); });
    // floor: a strip of the plane z = 0
    const fl = [P(-0.05, -0.25, 0), P(0.95, -0.25, 0), P(0.95, 0.35, 0), P(-0.05, 0.35, 0)];
    el('polygon', { points: fl.map(p => p.join(',')).join(' '), fill: `var(${colVar(R.floor.rgba.v)})`, 'fill-opacity': 0.18, stroke: `var(${colVar(R.floor.rgba.v)})`, 'stroke-width': 1.5, 'data-geom': 'floor', style: 'cursor:pointer' }, scene);
    if (sel === 'floor') outline(scene, fl, true);
    if (changed.has('floor')) outline(scene, fl, false);
    // stand under the shoulder
    const s0 = P(0, 0, 0), s1 = P(0, 0, 1);
    el('line', { x1: s0[0], y1: s0[1], x2: s1[0], y2: s1[1], stroke: 'var(--text-dim)', 'stroke-opacity': 0.4, 'stroke-width': 3, 'stroke-dasharray': '3 4' }, scene);
    // everything else, far (+y) to near (-y)
    const items = [];
    L11.GEOMS.forEach((g, i) => {
      if (g.name === 'floor') return;
      let c = g.c, y = g.c ? g.c[1] : 0;
      if (g.name === 'red_box_geom') c = K.qpos.slice(2, 5);
      items.push({ g, c, y, i });
    });
    const PRI = { board: 0, red_box_geom: 1, upper_geom: 2, forearm_geom: 3, weight: 4 };
    items.sort((a, b) => (b.y - a.y) || ((PRI[a.g.name] ?? 5) - (PRI[b.g.name] ?? 5)) || (a.i - b.i));
    items.forEach(({ g, c }) => {
      const r = R[g.name], col = colVar(r.rgba.v);
      if (g.name === 'upper_geom' || g.name === 'forearm_geom') {
        const a = g.name === 'upper_geom' ? P(0, 0, 1) : P(...f.elbow), b = g.name === 'upper_geom' ? P(...f.elbow) : P(...f.hand);
        const w = 2 * r.size.v[0] * S;
        if (sel === g.name || changed.has(g.name)) el('line', { x1: a[0], y1: a[1], x2: b[0], y2: b[1], stroke: 'var(--text-bright)', 'stroke-width': w + (sel === g.name ? 5 : 7), 'stroke-linecap': 'round', 'stroke-dasharray': changed.has(g.name) ? 'none' : '4 3', 'pointer-events': 'none' }, scene);
        el('line', { x1: a[0], y1: a[1], x2: b[0], y2: b[1], stroke: `var(${col})`, 'stroke-width': w, 'stroke-linecap': 'round', 'data-geom': g.name, style: 'cursor:pointer' }, scene);
        return;
      }
      const h = r.size.v, th = g.name === 'weight' ? f.th : 0, cc = g.name === 'weight' ? f.hand : c;
      cuboid(scene, cc, h, th, col, g.name);
      if (g.name !== 'weight') { if (sel === g.name) outline(scene, hull(cc, h), true); if (changed.has(g.name)) outline(scene, hull(cc, h), false); }
      else if (sel === g.name || changed.has(g.name)) { const p = P(...cc); el('circle', { cx: p[0], cy: p[1], r: 0.075 * S + 3, fill: 'none', stroke: 'var(--text-bright)', 'stroke-width': changed.has(g.name) ? 3 : 1.5, 'stroke-dasharray': changed.has(g.name) ? 'none' : '4 3', 'pointer-events': 'none' }, scene); }
    });
    // joints and the hand site
    [P(0, 0, 1), P(...f.elbow)].forEach(p => el('circle', { cx: p[0], cy: p[1], r: 4, fill: 'var(--text-bright)', 'pointer-events': 'none' }, scene));
    const hp = P(...f.hand);
    el('circle', { cx: hp[0], cy: hp[1], r: 3.5, fill: 'var(--red)', stroke: 'var(--text-bright)', 'stroke-width': 1, 'pointer-events': 'none' }, scene);
    if (K.qvel0) { // swing: shoulder turning at +3 rad/s; hand velocity (world x, z) = d(hand)/d(q1) * qvel[0], drawn as 0.12 s of travel
      const v = [-0.5 * Math.sin(K.qpos[0]) - 0.4 * Math.sin(f.th), -0.5 * Math.cos(K.qpos[0]) - 0.4 * Math.cos(f.th)].map(x => x * K.qvel0);
      const tip = P(f.hand[0] + 0.12 * v[0], 0, f.hand[2] + 0.12 * v[1]);
      el('line', { x1: hp[0], y1: hp[1], x2: tip[0], y2: tip[1], stroke: 'var(--orange)', 'stroke-width': 2.5, 'pointer-events': 'none' }, scene);
      el('circle', { cx: tip[0], cy: tip[1], r: 3, fill: 'var(--orange)', 'pointer-events': 'none' }, scene);
    }
    readouts(R, f, K);
  }

  function readouts(R, f, K) {
    $('s11KeyText').textContent = key === 'qpos0' ? 'mj_resetData: qpos0' : 'key: ' + key;
    $('s11SelText').textContent = 'geom: ' + sel;
    $('s11Q').textContent = K.qpos[0].toFixed(4) + ', ' + K.qpos[1].toFixed(4);
    $('s11Hand').textContent = '(' + f.hand.map(v => v.toFixed(4)).join(', ') + ')';
    $('s11V').textContent = K.qvel0.toFixed(1); $('s11T').textContent = K.time.toFixed(1);
    $('s11Box').textContent = '(' + K.qpos.slice(2, 5).map(v => v.toFixed(2)).join(', ') + ')';
    const g = L11.GEOMS.find(x => x.name === sel), r = R[sel], id = L11.GEOMS.indexOf(g);
    let h = `<span class="rk">geom ${id}</span> <span class="rv">${sel}</span> <span class="rk">uses class</span> <span class="rv">${g.cls}</span> <span class="rk">(${g.via})</span><br>`;
    h += `<span class="rk">search order: element, ${L11.chain(g.cls).join(', ')}, built-in</span><br>`;
    L11.ATTRS.forEach(a => {
      const note = a === 'size' && g.fromto ? ' (from fromto)' : '';
      h += `<span class="rk">${a}:</span> <span class="rv">${show(r[a].v)}</span>${note ? '<span class="rk">' + note + '</span>' : ''} <span class="rk">from</span> <span class="rv-o">${r[a].lvl}</span><br>`;
    });
    $('s11GeomOut').innerHTML = h;
    const users = {}; L11.GEOMS.forEach(x => (users[x.cls] = users[x.cls] || []).push(x.name));
    const ch = L11.chain(g.cls);
    const line = (c, pad, extra) => `${'&nbsp;'.repeat(pad)}<span class="${ch.includes(c) ? 'rv-o' : 'rk'}">${c}</span><span class="rk">: ${(users[c] || []).join(', ')}${extra || ''}</span><br>`;
    $('s11TreeOut').innerHTML = '<span class="rk">class tree (orange = the picked geom\'s search path):</span><br>' + line('main', 0, ' (global default)') + line('robot', 2, '; joints shoulder, elbow') + line('finger', 4) + line('furniture', 2) + line('object', 2);
  }

  function toggle(i) {
    const before = {}; L11.GEOMS.forEach(g => { before[g.name] = L11.resolve(g.name, on); });
    on[i] = !on[i];
    const t = L11.TOGGLES[i], attrs = Object.keys(t.set);
    changed = new Set(); const kept = [];
    L11.GEOMS.forEach(g => {
      const after = L11.resolve(g.name, on);
      const diff = attrs.some(a => show(after[a].v) !== show(before[g.name][a].v));
      if (diff) changed.add(g.name);
      else if (L11.chain(g.cls).includes(t.cls)) kept.push(`${g.name} (${attrs[0]} from ${after[attrs[0]].lvl})`);
    });
    const C0 = L11.CLASSES[t.cls].geom, now = on[i] ? t.set : C0;
    const was = on[i] ? C0 : t.set;
    let h = `<span class="rk">${t.label}:</span> <span class="rv">${attrs.map(a => show(was[a])).join(' / ')}</span> <span class="rk">to</span> <span class="rv-o">${attrs.map(a => show(now[a])).join(' / ')}</span><br>`;
    h += `<span class="rk">geoms that changed:</span> <span class="rv">${[...changed].join(', ') || 'none'}</span><br>`;
    h += `<span class="rk">use this class but kept their value:</span> <span class="rv">${kept.join(', ') || 'none'}</span>`;
    $('s11ChangeOut').innerHTML = h;
    document.querySelectorAll('[data-s11tog]').forEach(b => { const k = +b.dataset.s11tog; b.classList.toggle('btn-primary', on[k]); b.setAttribute('aria-pressed', String(on[k])); });
    flash($('s11ChangeOut')); render();
  }
  function pick(name) {
    sel = name;
    document.querySelectorAll('[data-s11geom]').forEach(b => { const a = b.dataset.s11geom === sel; b.classList.toggle('btn-primary', a); b.setAttribute('aria-pressed', String(a)); });
    render(); flash($('s11GeomOut'));
  }
  document.querySelectorAll('[data-s11key]').forEach(b => b.addEventListener('click', () => {
    key = b.dataset.s11key;
    document.querySelectorAll('[data-s11key]').forEach(x => { const a = x === b; x.classList.toggle('btn-primary', a); x.setAttribute('aria-pressed', String(a)); });
    render(); flash($('s11KeyOut'));
  }));
  document.querySelectorAll('[data-s11geom]').forEach(b => b.addEventListener('click', () => pick(b.dataset.s11geom)));
  document.querySelectorAll('[data-s11tog]').forEach(b => b.addEventListener('click', () => toggle(+b.dataset.s11tog)));
  $('s11Scene').addEventListener('click', e => { const t = e.target.closest('[data-geom]'); if (t) pick(t.dataset.geom); });
  render();
})();

// ======== LESSON 11 EXPLAINER ========
(() => {
  // Only redraw this lesson's canvases while it is on screen (hidden canvases have zero size).
  const addEventListener = (type, fn, opts) => window.addEventListener(type, e => { if (document.getElementById('lesson11').classList.contains('active')) fn(e); }, opts);
  const $ = id => document.getElementById(id);
  if (!$('lesson11')) return;
  const users = {}; L11.GEOMS.forEach(g => (users[g.cls] = users[g.cls] || []).push(g.name));
  const pressed = (sel, attr, val) => document.querySelectorAll(sel).forEach(b => { const a = b.dataset[attr] === val; b.classList.toggle('primary', a); b.setAttribute('aria-pressed', String(a)); });
  function box(c, x, y, w, h, stroke, fill, lw = 1.5) { c.fillStyle = fill; c.strokeStyle = stroke; c.lineWidth = lw; c.beginPath(); c.rect(x, y, w, h); if (fill !== 'none') c.fill(); c.stroke(); }

  /* ---------- Section 1: the class tree ---------- */
  (() => {
    const cv = $('l11cvTree'); let g = fit(cv, 0.66), pick = 'robot';
    // what each class writes (from the XML)
    const SETS = { main: 'rgba grey', robot: 'rgba teal, contype 0, conaffinity 0; joints: axis y, damping 0.5', finger: 'friction 2, condim 4, contype 1, conaffinity 1', furniture: 'type box, friction 0.5, rgba wood', object: 'type box, size 0.05, friction 0.6, rgba card' };
    const POS = { main: [0.5, 0.1], robot: [0.18, 0.38], finger: [0.18, 0.74], furniture: [0.5, 0.38], object: [0.82, 0.38] };
    function draw() {
      const { ctx: c, w, h } = g; c.clearRect(0, 0, w, h);
      const fs = Math.max(9, Math.min(12, w / 52));
      const inPath = L11.chain(pick);
      [['main', 'robot'], ['robot', 'finger'], ['main', 'furniture'], ['main', 'object']].forEach(([a, b]) => {
        const on = inPath.includes(a) && inPath.includes(b);
        c.strokeStyle = on ? css('--orange') : css('--dim'); c.lineWidth = on ? 2.5 : 1;
        c.beginPath(); c.moveTo(POS[a][0] * w, POS[a][1] * h + 14); c.lineTo(POS[b][0] * w, POS[b][1] * h - 14); c.stroke();
      });
      Object.keys(POS).forEach(k => {
        const [x, y] = [POS[k][0] * w, POS[k][1] * h], bw = Math.min(w * 0.3, 150), sel = k === pick, path = inPath.includes(k);
        box(c, x - bw / 2, y - 14, bw, 28, sel ? css('--teal') : (path ? css('--orange') : css('--dim')), sel ? css('--teal-dim') : 'none', sel ? 2.5 : 1.2);
        text(c, k === 'main' ? 'main (global)' : k, x, y, sel ? css('--bright') : css('--dim'), fs + 1, 'center', 700);
        const mem = users[k] || [];
        if (k === 'main') mem.forEach(m => text(c, m, x + bw / 2 + 8, y, sel ? css('--teal') : css('--dim'), fs, 'left', sel ? 700 : 500));
        else mem.forEach((m, i) => text(c, m, x, y + 26 + i * (fs + 4), sel ? css('--teal') : css('--dim'), fs, 'center', sel ? 700 : 500));
      });
      $('l11TreeSets').textContent = (pick === 'main' ? 'main (global)' : pick) + ' writes: ' + SETS[pick] + '. Used by: ' + (users[pick] || []).join(', ') + '.';
    }
    document.querySelectorAll('.l11tree').forEach(b => b.addEventListener('click', () => { pick = b.dataset.cls; pressed('.l11tree', 'cls', pick); draw(); }));
    pressed('.l11tree', 'cls', pick);
    addEventListener('resize', () => { g = fit(cv, 0.66); draw(); });
    addEventListener('themechange', draw);
    draw();
  })();

  /* ---------- Section 2: the lookup ladder ---------- */
  (() => {
    const cv = $('l11cvLadder'); let g = fit(cv, 0.56), geom = 'weight', attr = 'friction', shown = 1;
    function draw() {
      const { ctx: c, w, h } = g; c.clearRect(0, 0, w, h);
      const R = L11.rungs(geom, attr), hit = R.findIndex(r => r.has), fs = Math.max(9, Math.min(12, w / 50));
      const rh = Math.min(42, (h - 40) / R.length - 8), x0 = 16, bw = w - 32;
      text(c, `${geom}.${attr}: checked top to bottom, first hit wins`, x0, 14, css('--dim'), fs, 'left', 600);
      R.forEach((r, i) => {
        const y = 32 + i * (rh + 8), vis = i < shown, found = i === hit && vis, skipped = vis && i > hit;
        const col = found ? css('--teal') : (skipped ? css('--grid') : (vis ? css('--orange') : css('--grid')));
        box(c, x0, y, bw, rh, col, found ? css('--teal-dim') : 'none', found ? 2.5 : 1.2);
        text(c, `${i + 1}. ${r.lvl}`, x0 + 10, y + rh / 2 - (rh > 30 ? 7 : 0), vis ? css('--bright') : css('--dim'), fs, 'left', 700);
        if (rh > 30) text(c, r.where, x0 + 10, y + rh / 2 + 9, css('--dim'), fs - 1, 'left', 500);
        let msg = '?';
        if (vis) msg = skipped ? 'never checked' : (r.has ? attr + ' = ' + show(r.v) + (found ? '   <- used' : '') : 'not set here, look further');
        text(c, msg, x0 + bw - 10, y + rh / 2, found ? css('--teal') : (vis && !skipped ? css('--orange') : css('--dim')), fs, 'right', found ? 700 : 600);
      });
      $('l11LadNext').disabled = shown >= R.length;
    }
    function reset() { shown = 1; draw(); }
    document.querySelectorAll('.l11ladg').forEach(b => b.addEventListener('click', () => { geom = b.dataset.g; pressed('.l11ladg', 'g', geom); reset(); }));
    document.querySelectorAll('.l11lada').forEach(b => b.addEventListener('click', () => { attr = b.dataset.a; pressed('.l11lada', 'a', attr); reset(); }));
    $('l11LadNext').addEventListener('click', () => { shown++; draw(); });
    $('l11LadAll').addEventListener('click', () => { shown = 99; draw(); });
    pressed('.l11ladg', 'g', geom); pressed('.l11lada', 'a', attr);
    addEventListener('resize', () => { g = fit(cv, 0.56); draw(); });
    addEventListener('themechange', draw);
    draw();
  })();

  /* ---------- Section 3: keyframes pose the arm ---------- */
  (() => {
    const cv = $('l11cvKeys'); let g = fit(cv, 0.95), key = 'reach';
    const NAMES = ['qpos0', 'home', 'lesson4', 'reach', 'swing'];
    function draw() {
      const { ctx: c, w, h } = g; c.clearRect(0, 0, w, h);
      const S = Math.min(w / 1.3, h / 1.62), X = x => 0.22 * S + x * S, Z = z => h - 0.08 * S - z * S, fs = Math.max(9, Math.min(12, w / 50));
      // floor, stand, shelf (side view: board, posts, red box)
      c.strokeStyle = css('--dim'); c.lineWidth = 1.5; c.beginPath(); c.moveTo(0, Z(0)); c.lineTo(w, Z(0)); c.stroke();
      c.setLineDash([3, 4]); c.beginPath(); c.moveTo(X(0), Z(0)); c.lineTo(X(0), Z(1)); c.stroke(); c.setLineDash([]);
      box(c, X(0.6), Z(0.5), 0.3 * S, 0.04 * S, css('--yellow'), css('--grid'));
      box(c, X(0.6), Z(0.46), 0.3 * S, 0.46 * S, css('--yellow'), 'none', 1);
      box(c, X(0.7), Z(0.6), 0.1 * S, 0.1 * S, css('--red'), css('--grid'));
      text(c, 'shelf', X(0.75), Z(0.25), css('--yellow'), fs, 'center');
      const arm = (q1, q2, col, lw, alpha) => {
        const f = L11.fk(q1, q2); c.globalAlpha = alpha; c.lineCap = 'round';
        c.strokeStyle = col; c.lineWidth = lw; c.beginPath(); c.moveTo(X(0), Z(1)); c.lineTo(X(f.elbow[0]), Z(f.elbow[2])); c.lineTo(X(f.hand[0]), Z(f.hand[2])); c.stroke();
        c.globalAlpha = 1; return f;
      };
      NAMES.forEach(k => { if (k !== key) arm(L11.KEYS[k].qpos[0], L11.KEYS[k].qpos[1], css('--dim'), 3, 0.35); });
      const K = L11.KEYS[key], f = arm(K.qpos[0], K.qpos[1], css('--teal'), Math.max(5, 0.08 * S), 1);
      c.fillStyle = css('--red'); c.beginPath(); c.arc(X(f.hand[0]), Z(f.hand[2]), 5, 0, 7); c.fill();
      if (K.qvel0) { const v = [-0.5 * Math.sin(K.qpos[0]) - 0.4 * Math.sin(f.th), 0.5 * Math.cos(K.qpos[0]) + 0.4 * Math.cos(f.th)].map(x => x * K.qvel0 * 0.06 * S);
        arrow(c, X(f.hand[0]), Z(f.hand[2]), X(f.hand[0]) + v[0], Z(f.hand[2]) + v[1], css('--orange'), 2.5, 8); text(c, 'qvel[0] = 3 rad/s', 8, h - 12, css('--orange'), fs, 'left'); }
      text(c, 'side view: world x right, z up', 8, 12, css('--dim'), fs, 'left', 500);
      $('l11kQ').querySelector('.val').textContent = K.qpos[0].toFixed(2) + ', ' + K.qpos[1].toFixed(2);
      $('l11kX').querySelector('.val').textContent = f.hand[0].toFixed(4);
      $('l11kZ').querySelector('.val').textContent = f.hand[2].toFixed(4);
      $('l11kV').querySelector('.val').textContent = K.qvel0.toFixed(1);
      $('l11kT').querySelector('.val').textContent = K.time.toFixed(1);
    }
    document.querySelectorAll('.l11key').forEach(b => b.addEventListener('click', () => { key = b.dataset.k; pressed('.l11key', 'k', key); draw(); ['l11kQ', 'l11kX', 'l11kZ', 'l11kV', 'l11kT'].forEach(id => flash($(id))); }));
    pressed('.l11key', 'k', key);
    addEventListener('resize', () => { g = fit(cv, 0.95); draw(); });
    addEventListener('themechange', draw);
    draw();
  })();

  /* ---------- Section 5: a row of shelves generated by a loop ---------- */
  (() => {
    const cv = $('l11cvRow'), asp = () => (cv.clientWidth < 500 ? 0.34 : 0.24); let g = fit(cv, asp());
    function draw() {
      const { ctx: c, w, h } = g; c.clearRect(0, 0, w, h);
      const n = +$('l11RowN').value, k = +$('l11RowK').value, fs = Math.max(9, Math.min(12, w / 50));
      $('l11oRowN').textContent = n; $('l11oRowK').textContent = k;
      // top view: y to the right, x up. shelves at y = 0.8 i, each 0.3 m deep (x) and 0.7 m wide (y)
      const S = Math.min((w - 30) / 4.7, (h - 44) / 0.3), Y = y => 15 + (y + 0.35) * S, X = x => 26 + (0.9 - x) * S;
      for (let i = 0; i < n; i++) {
        const yc = 0.8 * i;
        box(c, Y(yc - 0.35), X(0.9), 0.7 * S, 0.3 * S, css('--yellow'), 'none', 1.5);
        text(c, 'shelf' + i, Y(yc), X(0.9) - 12, css('--yellow'), fs, 'center');
        for (let j = 0; j < k; j++) {
          const y = k > 1 ? -0.24 + 0.48 * j / (k - 1) : 0;
          box(c, Y(yc + y - 0.05), X(0.8), 0.1 * S, 0.1 * S, css('--purple'), css('--grid'));
        }
      }
      text(c, 'top view: each loop pass adds 1 body and 3 + k geoms', 8, h - 8, css('--dim'), fs, 'left', 500);
      const nb = 1 + n, ng = 1 + n * (3 + k);
      $('l11rB').querySelector('.val').textContent = nb; $('l11rG').querySelector('.val').textContent = ng;
      $('l11rC').querySelector('.val').textContent = n * (3 + k);
    }
    ['l11RowN', 'l11RowK'].forEach(id => $(id).addEventListener('input', () => { draw(); ['l11rB', 'l11rG', 'l11rC'].forEach(x => flash($(x))); }));
    addEventListener('resize', () => { g = fit(cv, asp()); draw(); });
    addEventListener('themechange', draw);
    draw();
  })();
})();
})();
