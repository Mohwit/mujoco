// ======== LESSON 7: CONTACT AND CONSTRAINTS (sandbox + explainer) ========
(() => {
  // Box-on-ramp runs recorded from MuJoCo 3.15.0 by lessons/07-contact-and-constraints/record_ramp_grid.py.
  // 1 kg box, 0.2 m cube, timestep 0.002 s, released from rest. Downhill distance every 0.1 s for 1 s,
  // in units of 0.1 mm. c3[angleIndex][muIndex] uses condim 3; c1[angleIndex] uses condim 1 (no friction).
  // angles 0, 2.5, ..., 45 deg; mus 0.1, 0.2, ..., 1.0.
  const L7 = {"angles":[0.0,2.5,5.0,7.5,10.0,12.5,15.0,17.5,20.0,22.5,25.0,27.5,30.0,32.5,35.0,37.5,40.0,42.5,45.0],"mus":[0.1,0.2,0.3,0.4,0.5,0.6,0.7,0.8,0.9,1.0],"c3":[[[0,0,0,0,0,0,0,0,0,0,0],[0,0,0,0,0,0,0,0,0,0,0],[0,0,0,0,0,0,0,0,0,0,0],[0,0,0,0,0,0,0,0,0,0,0],[0,0,0,0,0,0,0,0,0,0,0],[0,0,0,0,0,0,0,0,0,0,0],[0,0,0,0,0,0,0,0,0,0,0],[0,0,0,0,0,0,0,0,0,0,0],[0,0,0,0,0,0,0,0,0,0,0],[0,0,0,0,0,0,0,0,0,0,0]],[[0,0,0,0,1,1,1,1,1,1,1],[0,0,0,0,1,1,1,1,1,1,1],[0,0,0,0,1,1,1,1,1,1,1],[0,0,0,0,1,1,1,1,1,1,1],[0,0,0,0,1,1,1,1,1,1,1],[0,0,0,1,1,1,1,1,1,1,2],[0,0,0,1,1,1,1,1,1,2,2],[0,0,0,1,1,1,1,1,2,2,2],[0,0,1,1,1,1,1,2,2,2,2],[0,0,1,1,1,1,1,2,2,2,2]],[[0,0,1,1,2,2,3,3,3,4,4],[0,0,0,1,1,1,1,2,2,2,2],[0,0,0,1,1,1,1,2,2,2,2],[0,0,1,1,1,1,2,2,2,2,3],[0,0,1,1,1,1,2,2,2,3,3],[0,0,1,1,1,2,2,2,2,3,3],[0,0,1,1,1,2,2,2,3,3,3],[0,0,1,1,1,2,2,3,3,3,4],[0,0,1,1,2,2,2,3,3,4,4],[0,1,1,1,2,2,3,3,4,4,5]],[[0,16,62,139,248,386,556,756,988,1250,1542],[0,0,1,2,2,3,3,4,4,5,5],[0,0,1,1,1,2,2,3,3,3,4],[0,0,1,1,2,2,2,3,3,4,4],[0,0,1,1,2,2,3,3,3,4,4],[0,0,1,1,2,2,3,3,4,4,5],[0,1,1,2,2,3,3,4,4,5,5],[0,1,1,2,2,3,3,4,4,5,6],[0,1,1,2,2,3,4,4,5,6,6],[0,1,1,2,3,3,4,5,5,6,7]],[[0,38,149,334,593,925,1332,1812,2366,2993,3694],[0,1,2,3,3,4,5,6,7,8,9],[0,1,1,2,3,3,4,4,5,6,6],[0,1,1,2,2,3,3,4,4,5,5],[0,1,1,2,2,3,3,4,4,5,6],[0,1,1,2,2,3,4,4,5,5,6],[0,1,1,2,3,3,4,5,5,6,7],[0,1,2,2,3,4,4,5,6,7,7],[0,1,2,2,3,4,5,6,7,7,8],[0,1,2,3,4,5,5,6,7,8,9]],[[0,59,235,528,937,1463,2105,2864,3739,4731,5839],[0,12,51,112,193,295,418,561,726,911,1117],[0,1,2,3,4,5,6,7,8,9,10],[0,1,2,2,3,4,5,5,6,7,8],[0,1,1,2,3,4,4,5,6,7,7],[0,1,2,2,3,4,5,5,6,7,8],[0,1,2,3,3,4,5,6,7,8,8],[0,1,2,3,4,5,6,6,7,8,9],[0,1,2,3,4,5,6,7,8,9,10],[0,1,2,3,5,6,7,8,9,10,11]],[[0,81,322,721,1280,1997,2874,3910,5105,6460,7973],[0,33,130,292,518,808,1163,1582,2066,2613,3226],[0,2,5,7,9,12,15,17,20,22,25],[0,2,3,4,5,6,7,9,10,11,12],[0,1,2,3,4,5,6,7,8,9,10],[0,1,2,3,4,5,6,7,8,9,10],[0,2,3,4,5,6,7,8,9,10,11],[0,2,3,4,5,6,7,8,9,10,11],[0,2,3,4,5,7,8,9,10,11,13],[0,2,3,5,6,7,9,10,11,13,14]],[[0,103,407,913,1620,2528,3638,4949,6462,8176,10092],[0,55,218,489,867,1354,1948,2650,3460,4378,5404],[0,9,37,81,139,212,299,400,515,645,789],[0,2,4,5,7,8,10,11,13,15,16],[0,2,3,5,6,7,9,10,12,13,14],[0,2,3,4,6,7,8,9,10,12,13],[0,2,3,4,6,7,8,10,11,12,14],[0,2,3,5,6,7,9,10,11,13,14],[0,2,3,5,6,8,9,11,12,13,15],[0,2,4,5,7,8,10,11,13,15,16]],[[0,124,492,1102,1956,3054,4395,5979,7806,9877,12191],[0,77,305,685,1215,1897,2730,3714,4849,6135,7573],[0,31,121,270,479,746,1072,1457,1902,2404,2966],[0,3,7,11,15,19,23,27,31,34,38],[0,2,4,6,8,10,11,13,15,17,19],[0,2,4,5,7,9,10,12,14,15,17],[0,2,4,5,7,9,10,12,13,15,16],[0,2,4,6,7,9,11,12,14,16,17],[0,3,4,6,8,10,11,13,15,17,18],[0,3,5,6,8,10,12,14,16,18,19]],[[0,145,575,1290,2290,3574,5143,6997,9136,11559,14267],[0,99,392,879,1561,2436,3506,4770,6228,7880,9727],[0,53,209,469,832,1299,1869,2543,3320,4202,5186],[0,10,39,82,138,207,288,383,490,610,743],[0,3,8,12,16,20,23,28,32,36,39],[0,3,5,7,9,11,13,15,17,19,22],[0,3,5,7,8,10,12,14,16,18,20],[0,3,5,7,9,11,13,15,17,19,20],[0,3,5,7,9,11,13,16,18,20,22],[0,3,5,8,10,12,14,17,19,21,23]],[[0,166,658,1475,2618,4087,5882,8002,10447,13219,16316],[0,121,478,1073,1904,2972,4275,5817,7595,9609,11862],[0,75,299,670,1189,1855,2669,3632,4743,6000,7407],[0,31,120,268,476,743,1068,1451,1896,2397,2958],[0,6,11,17,23,28,34,39,44,51,56],[0,4,8,12,16,21,24,29,33,37,41],[0,2,5,7,10,12,15,17,20,22,24],[0,2,5,7,9,12,14,16,19,21,23],[0,3,5,8,10,12,15,17,20,22,25],[0,3,6,8,11,13,16,19,21,24,26]],[[0,187,739,1658,2942,4593,6609,8992,11740,14854,18334],[0,142,563,1263,2243,3500,5037,6854,8947,11322,13974],[0,98,388,869,1543,2408,3466,4715,6154,7789,9614],[0,54,212,475,843,1317,1894,2578,3362,4257,5254],[0,13,47,98,167,255,359,482,623,782,959],[0,6,12,18,24,30,36,42,48,53,59],[0,4,9,13,17,22,26,30,35,39,43],[0,4,7,10,12,15,18,21,24,27,30],[0,4,7,9,12,15,18,20,23,26,29],[0,4,7,10,13,16,19,22,25,28,31]],[[0,207,819,1837,3261,5090,7325,9965,13010,16462,20319],[0,164,649,1454,2581,4027,5795,7882,10290,13020,16068],[0,122,479,1072,1901,2965,4265,5800,7572,9580,11815],[0,79,310,690,1219,1902,2737,3720,4854,6136,7567],[0,37,140,308,543,842,1206,1638,2137,2698,3324],[0,8,16,24,31,40,47,56,64,71,80],[0,6,13,19,26,32,38,45,51,58,64],[0,4,9,14,19,23,28,33,37,42,47],[0,4,7,11,15,18,22,26,29,33,37],[0,4,7,11,14,17,21,24,27,31,34]],[[0,227,898,2013,3573,5576,8025,10918,14254,18036,22261],[0,184,730,1638,2907,4537,6531,8884,11598,14675,18115],[0,142,563,1262,2242,3500,5034,6850,8944,11316,13968],[0,100,396,887,1574,2458,3541,4813,6288,7955,9821],[0,58,229,512,909,1423,2046,2785,3634,4596,5681],[0,20,71,153,264,408,581,784,1018,1287,1578],[0,8,17,26,35,44,53,62,71,80,89],[0,6,13,20,27,34,41,48,55,62,69],[0,5,10,16,21,27,32,37,43,48,54],[0,5,10,14,19,24,28,33,38,42,47]],[[0,246,974,2185,3878,6053,8711,11850,15472,19577,24163],[0,205,812,1820,3231,5043,7257,9872,12893,16313,20138],[0,164,649,1456,2584,4035,5807,7900,10314,13049,16106],[0,123,487,1092,1937,3022,4349,5924,7732,9776,12075],[0,82,324,725,1295,2019,2896,3947,5152,6525,8036],[0,41,162,366,643,1011,1448,1976,2577,3254,4035],[0,11,25,41,57,75,93,112,130,149,168],[0,9,19,29,38,49,58,68,78,88,98],[0,7,14,21,28,35,42,49,56,64,71],[0,6,13,20,27,33,40,47,53,60,67]],[[0,265,1049,2353,4176,6518,9379,12760,16661,21080,26020],[0,225,892,2000,3550,5541,7972,10847,14163,17921,22120],[0,185,734,1645,2923,4561,6564,8932,11665,14760,18214],[0,145,577,1294,2298,3585,5159,7016,9162,11588,14296],[0,106,419,940,1673,2602,3752,5106,6658,8424,10370],[0,66,262,588,1044,1637,2350,3196,4175,5285,6523],[0,29,111,248,432,669,963,1300,1702,2150,2648],[0,13,26,40,54,68,82,96,110,124,138],[0,10,20,29,39,48,58,68,78,87,97],[0,9,18,27,35,44,53,61,70,79,88]],[[0,283,1122,2516,4466,6970,10031,13647,17818,22545,27826],[0,245,970,2176,3860,6027,8672,11796,15407,19490,24059],[0,206,817,1834,3255,5084,7312,9952,12994,16435,20290],[0,168,664,1494,2649,4140,5958,8106,10577,13383,16519],[0,130,514,1151,2046,3190,4589,6229,8117,10287,12715],[0,91,361,815,1436,2258,3225,4378,5745,7279,8993],[0,54,213,476,843,1308,1893,2565,3352,4250,5235],[0,22,74,156,266,405,573,774,1000,1258,1543],[0,12,26,40,55,69,84,100,115,130,145],[0,10,21,31,42,53,63,74,85,96,106]],[[0,301,1192,2675,4747,7410,10663,14507,18941,23965,29580],[0,264,1047,2347,4165,6501,9353,12726,16621,21026,25951],[0,227,900,2018,3581,5590,8050,10937,14282,18071,22302],[0,190,753,1689,2998,4678,6740,9167,11973,15135,18675],[0,153,607,1361,2421,3774,5430,7385,9653,12220,15062],[0,116,460,1036,1839,2855,4113,5609,7337,9274,11460],[0,80,316,707,1255,1954,2788,3827,5008,6325,7777],[0,45,175,387,681,1061,1534,2084,2711,3418,4158],[0,22,58,105,163,235,318,414,519,639,767],[0,13,25,38,51,63,76,89,101,114,127]],[[0,318,1261,2828,5019,7835,11275,15339,20027,25340,31277],[0,283,1121,2514,4461,6963,10021,13635,17800,22521,27797],[0,248,981,2198,3903,6090,8759,11927,15568,19703,24314],[0,212,840,1883,3342,5216,7509,10220,13350,16862,20829],[0,177,700,1562,2782,4337,6265,8505,11109,14032,17336],[0,141,558,1254,2230,3471,4999,6773,8840,11226,13782],[0,106,412,931,1666,2600,3720,5059,6602,8376,10347],[0,71,278,621,1114,1737,2499,3407,4413,5589,6874],[0,41,150,328,584,895,1292,1746,2277,2873,3484],[0,17,37,63,99,158,271,509,1035,2082,3005]]],"c1":[[0,0,0,0,0,0,0,0,0,0,0],[0,22,86,194,344,537,773,1051,1373,1737,2144],[0,44,173,387,687,1073,1544,2101,2743,3470,4284],[0,65,259,580,1029,1607,2313,3146,4108,5197,6415],[0,87,344,772,1370,2138,3077,4185,5465,6914,8534],[0,108,429,962,1707,2665,3835,5217,6811,8618,10638],[0,129,513,1150,2041,3186,4585,6238,8145,10306,12720],[0,150,596,1336,2372,3702,5328,7248,9463,11974,14779],[0,171,678,1520,2698,4211,6060,8244,10764,13619,16810],[0,191,758,1701,3018,4711,6780,9224,12043,15238,18808],[0,211,837,1878,3333,5203,7487,10186,13300,16828,20771],[0,231,915,2052,3642,5685,8181,11130,14531,18386,22694],[0,250,991,2222,3944,6156,8858,12052,15735,19909,24574],[0,269,1065,2388,4238,6615,9519,12951,16909,21395,26407],[0,287,1137,2549,4524,7062,10162,13825,18051,22839,28190],[0,305,1206,2705,4801,7495,10785,14673,19158,24240,29919],[0,322,1274,2857,5070,7914,11388,15493,20229,25595,31592],[0,338,1339,3002,5329,8318,11969,16284,21261,26901,33204],[0,354,1401,3142,5577,8706,12528,17044,22253,28156,34753]]};
  const G7 = 9.81, M7 = 1, DEG = Math.PI / 180;
  // Replay: linear interpolation between the 0.1 s samples recorded from MuJoCo. Returns metres.
  const traj = (ai, mi, c1) => (c1 ? L7.c1[ai] : L7.c3[ai][mi]);
  const replayAt = (tr, t) => { const x = Math.max(0, Math.min(t, 1)) * 10, k = Math.min(9, Math.floor(x)), f = x - k; return (tr[k] + (tr[k + 1] - tr[k]) * f) / 10000; };
  // Ideal Coulomb rule (rigid contact), from rest: d = a t^2 / 2 with a = g (sin - mu cos), or 0 if that is negative.
  const idealAt = (deg, mu, c1, t) => { const a = G7 * (Math.sin(deg * DEG) - (c1 ? 0 : mu) * Math.cos(deg * DEG)); return 0.5 * Math.max(a, 0) * t * t; };

  // ======== SANDBOX 7: RAMP ========
  (() => {
    const $ = id => document.getElementById(id);
    if (!$('sandbox7')) return;
    const S = 90, L = 3.8, S0 = 0.15, BX = 380, BY = 400; // px per m, ramp length (m), start offset (m), bottom corner
    let c1 = false, t = 0, playing = false, last = 0;
    const ai = () => Math.round(+$('s7Angle').value / 2.5), mi = () => Math.round(+$('s7Mu').value * 10) - 1;
    const deg = () => L7.angles[ai()], mu = () => L7.mus[mi()];
    function place(el, s, th) {
      const xt = BX - L * Math.cos(th) * S, yt = BY - L * Math.sin(th) * S;
      const x = xt + s * Math.cos(th) * S, y = yt + s * Math.sin(th) * S;
      el.setAttribute('transform', `translate(${x.toFixed(2)}, ${y.toFixed(2)}) rotate(${(th / DEG).toFixed(2)})`);
    }
    function render() {
      const d = deg(), m = mu(), th = d * DEG, tr = traj(ai(), mi(), c1);
      const xt = BX - L * Math.cos(th) * S, yt = BY - L * Math.sin(th) * S;
      $('s7Ramp').setAttribute('points', `${xt.toFixed(1)},${yt.toFixed(1)} ${BX},${BY} ${xt.toFixed(1)},${BY}`);
      const r = 50, ex = BX - r * Math.cos(th), ey = BY - r * Math.sin(th);
      $('s7Arc').setAttribute('d', d > 0 ? `M ${BX - r} ${BY} A ${r} ${r} 0 0 1 ${ex.toFixed(1)} ${ey.toFixed(1)}` : '');
      $('s7ArcText').textContent = d.toFixed(1) + ' deg';
      $('s7ArcText').setAttribute('x', BX - r - 6);
      const dm = replayAt(tr, t), di = idealAt(d, m, c1, t);
      place($('s7Box'), S0 + dm, th); place($('s7Ghost'), S0 + di, th);
      const end = tr[10] / 10000, endI = idealAt(d, m, c1, 1);
      const predSlide = c1 ? d > 0 : Math.tan(th) > m + 1e-12;
      $('s7AngleVal').textContent = d.toFixed(1); $('s7MuVal').textContent = m.toFixed(1);
      $('s7RA').textContent = d.toFixed(1); $('s7RM').textContent = c1 ? m.toFixed(2) + ' (ignored, condim 1)' : m.toFixed(2);
      $('s7RT').textContent = Math.tan(th).toFixed(4);
      $('s7RN').textContent = (M7 * G7 * Math.cos(th)).toFixed(4);
      $('s7RP').textContent = predSlide ? 'slides' : 'sticks';
      $('s7RR').textContent = (end * 1000).toFixed(1) + ' mm, ' + (end > 0.01 ? 'slides' : 'sticks');
      $('s7RI').textContent = (endI * 1000).toFixed(1) + ' mm';
      $('s7RD').textContent = (dm * 1000).toFixed(1);
      $('s7TimeText').textContent = `time: ${t.toFixed(2)} s`;
      $('s7Verdict').textContent = c1 ? 'condim 1: push only, no grip' : `threshold atan(mu) = ${(Math.atan(m) / DEG).toFixed(2)} deg`;
    }
    function stop() { playing = false; $('s7Play').textContent = 'Play 1 s'; }
    function loop(ts) {
      if (!playing) return;
      t = Math.min(1, t + Math.min(0.05, (ts - last) / 1000)); last = ts; render();
      if (t >= 1) { stop(); return; }
      requestAnimationFrame(loop);
    }
    $('s7Play').addEventListener('click', () => {
      if (playing) { stop(); return; }
      if (t >= 1) t = 0;
      if (RM) { t = 1; render(); return; }
      playing = true; $('s7Play').textContent = 'Pause'; last = performance.now(); requestAnimationFrame(loop);
    });
    $('s7Reset').addEventListener('click', () => { stop(); t = 0; render(); });
    ['s7Angle', 's7Mu'].forEach(id => $(id).addEventListener('input', () => { stop(); t = 0; render(); }));
    function setC(v) {
      c1 = v; stop(); t = 0;
      $('s7C1').classList.toggle('btn-primary', v); $('s7C3').classList.toggle('btn-primary', !v);
      $('s7C1').setAttribute('aria-pressed', String(v)); $('s7C3').setAttribute('aria-pressed', String(!v));
      render();
    }
    $('s7C3').addEventListener('click', () => setC(false));
    $('s7C1').addEventListener('click', () => setC(true));
    render();
  })();

  // ======== LESSON 7 EXPLAINER ========
  (() => {
    // Only redraw this lesson's canvases while it is on screen (hidden canvases have zero size).
    const addEventListener = (type, fn, opts) => window.addEventListener(type, e => { if (document.getElementById('lesson7').classList.contains('active')) fn(e); }, opts);
    const $ = id => document.getElementById(id);
    if (!$('lesson7')) return;
    const setRo = (id, v) => { const el = $(id); const val = el.querySelector('.val'); if (val.textContent !== v) { val.textContent = v; flash(el); } };

    // Draw a ramp that falls to the right, from top-left (x0, y0); length len px; angle th.
    function rampPath(c, x0, y0, len, th) {
      const x1 = x0 + len * Math.cos(th), y1 = y0 + len * Math.sin(th);
      c.fillStyle = css('--surface'); c.strokeStyle = css('--teal'); c.lineWidth = 1.5;
      c.beginPath(); c.moveTo(x0, y0); c.lineTo(x1, y1); c.lineTo(x0, y1); c.closePath(); c.fill(); c.stroke();
      return [x1, y1];
    }
    // Box of side b px sitting on the ramp at distance s px from the top corner.
    function boxOnRamp(c, x0, y0, th, s, b, col) {
      c.save(); c.translate(x0 + s * Math.cos(th), y0 + s * Math.sin(th)); c.rotate(th);
      c.fillStyle = col; c.globalAlpha = 0.85; c.fillRect(-b / 2, -b, b, b); c.globalAlpha = 1;
      c.strokeStyle = col; c.lineWidth = 1; c.strokeRect(-b / 2, -b, b, b); c.restore();
    }

    /* ---------- Section 1: sandpaper versus ice ---------- */
    (() => {
      const cv = $('l7cvA'); let g = fit(cv, 0.5);
      const TH = 20 * DEG, LEN = 1.6, S0 = 0.15, trS = L7.c3[8][4], trI = L7.c3[8][0]; // 20 deg; mu 0.5 and 0.1
      let t = 0, playing = false, last = 0;
      function draw() {
        const { ctx: c, w, h } = g; c.clearRect(0, 0, w, h);
        const half = w / 2, sc = (half - 36) / (LEN * Math.cos(TH));
        [[trS, 'sandpaper, mu = 0.5', css('--orange')], [trI, 'ice, mu = 0.1', css('--blue')]].forEach(([tr, lab, col], k) => {
          const x0 = k * half + 18, y0 = h * 0.3;
          rampPath(c, x0, y0, LEN * sc, TH);
          const d = replayAt(tr, t);
          boxOnRamp(c, x0, y0, TH, (S0 + d) * sc, 0.2 * sc, col);
          text(c, lab, x0, 16, col, 12);
          text(c, 'moved ' + (d * 1000).toFixed(1) + ' mm', x0, 34, css('--dim'), 11);
        });
        text(c, '20 deg', w - 10, h - 10, css('--yellow'), 11, 'right');
        setRo('l7aT', t.toFixed(2)); setRo('l7aS', (replayAt(trS, t) * 1000).toFixed(1)); setRo('l7aI', (replayAt(trI, t) * 1000).toFixed(1));
      }
      function loop(ts) { if (!playing) return; t = Math.min(1, t + Math.min(0.05, (ts - last) / 1000)); last = ts; draw(); if (t >= 1) { playing = false; return; } requestAnimationFrame(loop); }
      $('l7aGo').addEventListener('click', () => { if (playing) return; t = 0; if (RM) { t = 1; draw(); return; } playing = true; last = performance.now(); requestAnimationFrame(loop); });
      $('l7aReset').addEventListener('click', () => { playing = false; t = 0; draw(); });
      addEventListener('resize', () => { g = fit(cv, 0.5); draw(); }); addEventListener('themechange', draw);
      draw();
    })();

    /* ---------- Section 2: collision filtering ---------- */
    (() => {
      const cv = $('l7cvB'); let g = fit(cv, 0.5);
      const names = ['floor', 'red', 'green', 'blue'], cols = ['--dim', '--red', '--y', '--blue'];
      const SCRIPT = [[1, 1], [1, 1], [2, 2], [0, 1]];
      const bits = SCRIPT.map(r => r.slice());
      const body = $('l7fRows');
      body.innerHTML = names.map((n, i) => `<tr><td><code>${n}</code></td>` + ['t1', 't2', 'a1', 'a2'].map(k =>
        `<td><label class="check" style="margin:0"><input type="checkbox" id="l7f${i}${k}" aria-label="${n} ${k[0] === 't' ? 'contype' : 'conaffinity'} bit ${k[1]}"></label></td>`).join('') + '</tr>').join('');
      const can = (i, j) => ((bits[i][0] & bits[j][1]) | (bits[j][0] & bits[i][1])) !== 0;
      function sync() {
        bits.forEach((b, i) => { $(`l7f${i}t1`).checked = !!(b[0] & 1); $(`l7f${i}t2`).checked = !!(b[0] & 2); $(`l7f${i}a1`).checked = !!(b[1] & 1); $(`l7f${i}a2`).checked = !!(b[1] & 2); });
      }
      function read() {
        bits.forEach((b, i) => { b[0] = ($(`l7f${i}t1`).checked ? 1 : 0) + ($(`l7f${i}t2`).checked ? 2 : 0); b[1] = ($(`l7f${i}a1`).checked ? 1 : 0) + ($(`l7f${i}a2`).checked ? 2 : 0); });
      }
      function draw() {
        const { ctx: c, w, h } = g; c.clearRect(0, 0, w, h);
        const fy = h * 0.66, r = Math.min(w / 14, h / 9), sy = h * 0.2;
        c.strokeStyle = css('--teal'); c.lineWidth = 3; c.beginPath(); c.moveTo(10, fy); c.lineTo(w - 10, fy); c.stroke();
        text(c, `floor: contype=${bits[0][0]} conaffinity=${bits[0][1]}`, 10, 14, css('--teal'), 11);
        text(c, 'dashed = start (z = 0.5 m), numbers = contype/conaffinity', 10, 30, css('--dim'), 10);
        for (let k = 1; k <= 3; k++) {
          const x = w * (k / 4), col = css(cols[k]), hit = can(0, k);
          // start position (dashed) and final position (the rule's prediction)
          c.setLineDash([3, 3]); c.strokeStyle = col; c.lineWidth = 1; c.beginPath(); c.arc(x, sy + r, r, 0, 7); c.stroke(); c.setLineDash([]);
          text(c, names[k] + ' ' + bits[k][0] + '/' + bits[k][1], x, sy + r, col, 11, 'center');
          const y = hit ? fy - r : h - r * 0.5;
          if (!hit) arrow(c, x, sy + 2 * r + 4, x, h - r * 1.6, col, 1.5, 7);
          c.globalAlpha = hit ? 0.9 : 0.35; c.fillStyle = col; c.beginPath(); c.arc(x, y, r, 0, 7); c.fill(); c.globalAlpha = 1;
          text(c, hit ? 'lands' : 'falls through', hit ? x : x + 8, fy + 14, hit ? css('--bright') : col, 11, hit ? 'center' : 'left');
        }
        const lines = [];
        for (let i = 0; i < 4; i++) for (let j = i + 1; j < 4; j++) {
          const a = bits[i][0] & bits[j][1], b = bits[j][0] & bits[i][1];
          lines.push(`${(names[i] + '-' + names[j]).padEnd(12)} ${bits[i][0]}&${bits[j][1]}=${a}  ${bits[j][0]}&${bits[i][1]}=${b}  -> ${a || b ? 'can collide' : 'ignored'}`);
        }
        $('l7fOut').textContent = lines.join('\n');
      }
      body.addEventListener('change', () => { read(); draw(); });
      $('l7fScript').addEventListener('click', () => { SCRIPT.forEach((r, i) => { bits[i][0] = r[0]; bits[i][1] = r[1]; }); sync(); draw(); });
      $('l7fAll').addEventListener('click', () => { bits.forEach(b => { b[0] = 1; b[1] = 1; }); sync(); draw(); });
      addEventListener('resize', () => { g = fit(cv, 0.5); draw(); }); addEventListener('themechange', draw);
      sync(); draw();
    })();

    /* ---------- Section 3: resting box, contact forces and penetration ---------- */
    (() => {
      const cv = $('l7cvC'); let g = fit(cv, 0.45);
      const SR = [['0.005 1', -0.007], ['0.02 1', -0.108], ['0.05 1', -0.491]]; // dist in mm, from the script
      let k = 1;
      function draw() {
        const { ctx: c, w, h } = g; c.clearRect(0, 0, w, h);
        const fy = h * 0.66, b = h * 0.44, cx = w * 0.26;
        c.strokeStyle = css('--teal'); c.lineWidth = 2; c.beginPath(); c.moveTo(8, fy); c.lineTo(w * 0.56, fy); c.stroke();
        text(c, 'floor (geom 0)', w * 0.555, fy - 9, css('--teal'), 10, 'right');
        c.fillStyle = css('--surface'); c.strokeStyle = css('--orange'); c.lineWidth = 1.5; c.fillRect(cx - b / 2, fy - b, b, b); c.strokeRect(cx - b / 2, fy - b, b, b);
        text(c, 'box (geom 1), 1 kg', cx, fy - b - 10, css('--orange'), 10, 'center');
        arrow(c, cx, fy - b * 0.62, cx, fy - b * 0.1, css('--orange'), 3, 9);
        text(c, 'm g = 9.81 N', cx + 6, fy - b * 0.72, css('--orange'), 11);
        [cx - b / 2, cx + b / 2].forEach(x => arrow(c, x, fy + h * 0.2, x, fy + 2, css('--blue'), 2.5, 8));
        text(c, 'f6[0] = 2.4525 N at each of 4 corners (2 hidden behind)', 10, h - 10, css('--blue'), 10);
        // zoom window on one corner
        const zx = w * 0.62, zy = h * 0.08, zw = w * 0.35, zh = h * 0.8, ppmm = zh * 0.6 / 0.5; // 0.5 mm spans 60% of the window
        c.strokeStyle = css('--border'); c.lineWidth = 1; c.strokeRect(zx, zy, zw, zh);
        text(c, 'zoom: window is 0.83 mm tall', zx + 6, zy + 11, css('--dim'), 10);
        const sy = zy + zh * 0.35, pen = -SR[k][1] * ppmm;
        c.fillStyle = css('--surface'); c.fillRect(zx + 1, sy, zw - 2, zh * 0.65 - 1);
        c.strokeStyle = css('--teal'); c.lineWidth = 2; c.beginPath(); c.moveTo(zx, sy); c.lineTo(zx + zw, sy); c.stroke();
        c.strokeStyle = css('--orange'); c.lineWidth = 2; c.beginPath(); c.moveTo(zx + 8, zy + 24); c.lineTo(zx + zw * 0.5, zy + 24); c.lineTo(zx + zw * 0.5, sy + pen); c.lineTo(zx + 8, sy + pen); c.stroke();
        c.globalAlpha = 0.4; c.fillStyle = css('--red'); c.fillRect(zx + 8, sy, zw * 0.5 - 8, pen); c.globalAlpha = 1;
        text(c, 'box corner', zx + 12, zy + 36, css('--orange'), 10);
        text(c, 'floor surface', zx + zw - 4, sy - 8, css('--teal'), 10, 'right');
        text(c, 'solref ' + SR[k][0], zx + zw * 0.76, sy + 18, css('--dim'), 10, 'center');
        text(c, 'dist = ' + SR[k][1].toFixed(3) + ' mm', zx + zw * 0.76, sy + 36, css('--red'), 11, 'center');
      }
      document.querySelectorAll('#lesson7 .l7sr').forEach(btn => btn.addEventListener('click', () => {
        k = +btn.dataset.k;
        document.querySelectorAll('#lesson7 .l7sr').forEach(x => x.classList.toggle('primary', x === btn));
        setRo('l7cD', SR[k][1].toFixed(3)); draw();
      }));
      addEventListener('resize', () => { g = fit(cv, 0.45); draw(); }); addEventListener('themechange', draw);
      draw();
    })();

    /* ---------- Section 4: free-body diagram on the ramp (ideal rule) ---------- */
    (() => {
      const cv = $('l7cvD'); let g = fit(cv, 0.5);
      function draw() {
        const d = +$('l7dA').value, mu = +$('l7dM').value, th = d * DEG;
        const N = M7 * G7 * Math.cos(th), need = M7 * G7 * Math.sin(th), lim = mu * N, slide = need > lim + 1e-12;
        $('l7dAo').textContent = d.toFixed(1) + ' deg'; $('l7dMo').textContent = mu.toFixed(2);
        setRo('l7dN', N.toFixed(4)); setRo('l7dF', need.toFixed(4)); setRo('l7dL', lim.toFixed(4)); setRo('l7dTh', (Math.atan(mu) / DEG).toFixed(2));
        setRo('l7dV', slide ? 'slides' : 'sticks');
        const { ctx: c, w, h } = g; c.clearRect(0, 0, w, h);
        const x0 = w * 0.04, y0 = h * 0.3, len = Math.min(w * 0.92 / Math.max(Math.cos(th), 0.01), h * 0.66 / Math.max(Math.sin(th), 1e-3));
        const [x1, y1] = rampPath(c, x0, y0, len, th);
        if (d > 0) { c.strokeStyle = css('--yellow'); c.lineWidth = 1.5; c.beginPath(); c.arc(x1, y1, 46, Math.PI, Math.PI + th); c.stroke(); }
        text(c, d.toFixed(1) + ' deg', x1 - 52, y1 - 10, css('--yellow'), 11, 'right');
        const b = h * 0.2, s = len * 0.42;
        c.save(); c.translate(x0 + s * Math.cos(th), y0 + s * Math.sin(th)); c.rotate(th);
        c.fillStyle = css('--surface'); c.fillRect(-b / 2, -b, b, b); c.strokeStyle = css('--bright'); c.lineWidth = 1.5; c.strokeRect(-b / 2, -b, b, b); c.restore();
        const nx = Math.sin(th), ny = -Math.cos(th);                       // normal (screen), out of the ramp
        const ux = -Math.cos(th), uy = -Math.sin(th);                      // up the slope (screen)
        const px = x0 + s * Math.cos(th), py = y0 + s * Math.sin(th);
        const cx = px + nx * b / 2, cy = py + ny * b / 2;                  // box centre
        const k = h * 0.028;                                               // px per newton
        c.fillStyle = css('--bright'); c.beginPath(); c.arc(cx, cy, 3, 0, 7); c.fill();
        arrow(c, cx, cy, cx, cy + G7 * k, css('--orange'), 3, 9);
        text(c, 'weight ' + (M7 * G7).toFixed(2) + ' N', cx + 8, cy + G7 * k, css('--orange'), 11);
        arrow(c, cx, cy, cx + nx * N * k, cy + ny * N * k, css('--blue'), 3, 9);
        text(c, 'N ' + N.toFixed(2) + ' N', cx + nx * N * k + 8, cy + ny * N * k, css('--blue'), 11);
        const f = Math.min(need, lim);
        if (f > 0.01) { arrow(c, cx, cy, cx + ux * f * k, cy + uy * f * k, css('--teal'), 3, 9); text(c, 'friction ' + f.toFixed(2) + ' N', cx + ux * f * k - 6, cy + uy * f * k - 10, css('--teal'), 11, 'right'); }
        if (slide) {
          const ex = need - lim; c.setLineDash([5, 4]);
          arrow(c, cx, cy, cx - ux * ex * k, cy - uy * ex * k, css('--red'), 2, 8); c.setLineDash([]);
          text(c, 'unbalanced ' + ex.toFixed(2) + ' N', cx - ux * ex * k + 8, cy - uy * ex * k + 4, css('--red'), 11);
        }
        text(c, slide ? 'ideal rule: slides (tan > mu)' : 'ideal rule: sticks (tan <= mu)', 10, 14, slide ? css('--red') : css('--teal'), 12);
        text(c, 'tan(angle) = ' + Math.tan(th).toFixed(4) + '   mu = ' + mu.toFixed(2), 10, 32, css('--dim'), 11);
      }
      ['l7dA', 'l7dM'].forEach(id => $(id).addEventListener('input', draw));
      addEventListener('resize', () => { g = fit(cv, 0.5); draw(); }); addEventListener('themechange', draw);
      draw();
    })();

    /* ---------- Section 5: margin ---------- */
    (() => {
      const cv = $('l7cvE'); let g = fit(cv, 0.8);
      // From part D of the script: [margin, ncon at start, dist at start (m), resting centre z after 2 s (m)]
      const MG = [[0, 0, null, 0.0996], [0.05, 1, 0.030, 0.1496]];
      let k = 0, settled = false;
      function draw() {
        const [mg, n, dist, zr] = MG[k], z = settled ? zr : 0.13;
        setRo('l7mN', String(n)); setRo('l7mD', dist === null ? 'none' : '+' + dist.toFixed(3)); setRo('l7mZ', z.toFixed(4));
        const { ctx: c, w, h } = g; c.clearRect(0, 0, w, h);
        const fy = h * 0.84, sc = (h * 0.6) / 0.3, cx = w * 0.36;   // 0.3 m of height spans 60% of the view
        if (mg > 0) {
          c.fillStyle = css('--teal'); c.globalAlpha = 0.15; c.fillRect(8, fy - mg * sc, w - 16, mg * sc); c.globalAlpha = 1;
          c.setLineDash([5, 4]); c.strokeStyle = css('--teal'); c.lineWidth = 1; c.beginPath(); c.moveTo(8, fy - mg * sc); c.lineTo(w - 8, fy - mg * sc); c.stroke(); c.setLineDash([]);
          text(c, 'band = margin 0.05 m', w - 12, fy + 12, css('--teal'), 10, 'right');
        }
        c.strokeStyle = css('--teal'); c.lineWidth = 3; c.beginPath(); c.moveTo(8, fy); c.lineTo(w - 8, fy); c.stroke();
        text(c, 'floor z = 0', 12, fy + 12, css('--teal'), 10);
        const r = 0.1 * sc, by = fy - z * sc;
        c.fillStyle = css('--orange'); c.globalAlpha = 0.85; c.beginPath(); c.arc(cx, by, r, 0, 7); c.fill(); c.globalAlpha = 1;
        const gap = z - 0.1;
        c.strokeStyle = css('--yellow'); c.lineWidth = 1.5; c.beginPath(); c.moveTo(cx + r + 14, by + r); c.lineTo(cx + r + 14, fy); c.stroke();
        text(c, 'gap ' + (gap * 100).toFixed(2) + ' cm', cx + r + 20, (by + r + fy) / 2, css('--yellow'), 11);
        text(c, 'ball centre z = ' + z.toFixed(4) + ' m', 10, 14, css('--bright'), 11);
        text(c, settled ? 'after 2 s (MuJoCo)' : 'at start: centre z = 0.13 m', 10, 30, css('--dim'), 10);
        if (n && !settled) text(c, 'listed in data.contact: dist = +0.030 m', 10, 46, css('--teal'), 10);
      }
      document.querySelectorAll('#lesson7 .l7mg').forEach(btn => btn.addEventListener('click', () => {
        k = +btn.dataset.k; settled = false;
        document.querySelectorAll('#lesson7 .l7mg').forEach(x => x.classList.toggle('primary', x === btn)); draw();
      }));
      $('l7mDrop').addEventListener('click', () => { settled = true; draw(); });
      addEventListener('resize', () => { g = fit(cv, 0.8); draw(); }); addEventListener('themechange', draw);
      draw();
    })();
  })();
})();
