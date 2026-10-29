/* SafePath – the "brain": mapping, planning, porter missions, dashboard UI */
(function () {
  'use strict';
  const { GridMap, firstRun, DX, DY, HEAD_DEG, FREE, OCC, UNK, StopError, sleep } = SP;
  const $ = s => document.querySelector(s);

  /* ============================ settings ============================ */
  const SETTINGS_VER = 2;
  const DEFAULTS = {
    ver: SETTINGS_VER,
    cmPerSec: 18, degPerSec: 110, driveSpd: 140, turnSpd: 150, safeCm: 25,
    cellCm: 30, limitM: 6, dynTTL: 30, angles: '15,50,90,130,165',
    maxTrust: 150, trips: 3, dwell: 3
  };
  const FIELDS = [
    ['cmPerSec', 'Drive speed (cm/s)', 'from calibration'],
    ['degPerSec', 'Turn speed (°/s)', 'from calibration'],
    ['driveSpd', 'Drive power (110–255)'],
    ['turnSpd', 'Turn power (110–255)'],
    ['safeCm', 'Emergency stop (cm)', 'robot brakes itself'],
    ['cellCm', 'Map cell size (cm)', 'changing clears map'],
    ['limitM', 'Max mapping range (m)'],
    ['dynTTL', 'Forget temp. obstacle (s)'],
    ['angles', 'Scan angles (°)', 'servo positions', 'text']
  ];
  let S = loadSettings();
  let settingsReset = false;
  function loadSettings() {
    try {
      const st = JSON.parse(localStorage.getItem('safepath.settings') || '{}');
      if (st.ver !== SETTINGS_VER) {                  // older version: slower speeds & new safety distance
        for (const k of ['cmPerSec', 'degPerSec', 'driveSpd', 'turnSpd', 'safeCm', 'maxTrust']) delete st[k];
        settingsReset = Object.keys(st).length > 0;
        st.ver = SETTINGS_VER;
      }
      return Object.assign({}, DEFAULTS, st);
    } catch (e) { return Object.assign({}, DEFAULTS); }
  }
  function saveSettings() { try { localStorage.setItem('safepath.settings', JSON.stringify(S)); } catch (e) { /* ignore */ } }

  /* ============================== state ============================= */
  let robot = null, connected = false;
  let map = new GridMap(81, S.cellCm);
  let pose = homePose();
  let startCell = null, endCell = null, mapped = false;
  let trail = [], plannedPath = null, targetCell = null, lastRays = null;
  let phase = 'IDLE', prevPhase = 'IDLE', running = false, abortFlag = false, paused = false;
  let tool = null, view = null, calElapsed = 0, badPolls = 0;
  const blacklist = new Map();
  const stats = { scans: 0, dist: 0, deliveries: 0, obstacles: 0, replans: 0 };
  const tele = { front: -1, back: -1, servo: 90, act: 'idle' };

  const tScale = () => window.SP_TIME_SCALE || 1;        // only used by automated tests
  const cal = () => ({ cmps: +S.cmPerSec, degps: +S.degPerSec });
  const msPerCell = () => S.cellCm / cal().cmps * 1000;
  const ms90 = () => Math.round(90 / cal().degps * 1000);
  const anglesArr = () => String(S.angles).split(',').map(v => parseInt(v, 10)).filter(v => v >= 0 && v <= 180).slice(0, 16);
  const same = (a, b) => a && b && a.x === b.x && a.y === b.y;
  const fmtCm = d => (d > 0 ? d.toFixed(0) + ' cm' : '—');

  /* =============================== log ============================== */
  function log(msg, type = '') {
    const el = document.createElement('div');
    const t = new Date();
    el.innerHTML = `<time>${t.toTimeString().slice(0, 8)}</time>`;
    const span = document.createElement('span');
    span.className = type; span.textContent = msg;
    el.appendChild(span);
    const box = $('#log');
    box.appendChild(el);
    while (box.children.length > 300) box.removeChild(box.firstChild);
    box.scrollTop = box.scrollHeight;
  }

  /* ========================= run / pause / stop ===================== */
  async function gate() {
    while (paused && !abortFlag) await sleep(150);
    if (abortFlag) throw new StopError();
  }
  async function appWait(ms) {
    const end = Date.now() + ms / tScale();
    while (Date.now() < end) { await gate(); await sleep(Math.min(100, Math.max(1, end - Date.now()))); }
  }
  function setPhase(p, text) {
    phase = p;
    const el = $('#phaseLabel');
    el.textContent = p; el.className = 'phase ' + p;
    if (text !== undefined) $('#phaseText').textContent = text;
    refreshButtons();
  }
  async function runTask(fn) {
    if (running || !connected) return;
    running = true; abortFlag = false; paused = false;
    refreshButtons();
    try { await fn(); }
    catch (e) {
      if (e instanceof StopError) { setPhase('STOPPED', 'Stopped by operator – robot halted.'); log('Stopped by operator', 'warn'); }
      else { setPhase('STOPPED', e.message); log('Error: ' + e.message, 'err'); try { await robot.stop(); } catch (_) { /* ignore */ } }
    } finally {
      running = false; paused = false; targetCell = null; plannedPath = null;
      tele.act = 'idle'; updateUI();
    }
  }
  let lastUptime = 0;
  function check(s) {
    if (s.result === 'stopped') throw new StopError();
    if (s.result === 'lost') log('Wi-Fi hiccup – the robot paused itself; continuing', 'warn');
    if (typeof s.uptime === 'number') {
      if (s.uptime < lastUptime) log('The robot restarted (power dip?) – check battery charge and the capacitor', 'err');
      lastUptime = s.uptime;
    }
    return s;
  }

  /* ========================= robot primitives ======================= */
  const poseCm = () => ({ x: pose.cx, y: pose.cy });
  function homePose() { const c = map.center(map.home); return { x: map.home.x, y: map.home.y, h: 0, cx: c.x, cy: c.y }; }
  const headingDeg = () => HEAD_DEG[pose.h];

  /* ---- Staying accurate without wheel sensors ----
     Moves and turns are timed, so small errors build up. The walls are used to cancel them:
     1. Heading: the servo measures the wall ahead at equal angles left and right (±25°, ±35°).
        Square to the wall, both sides read the same; a difference gives the skew angle,
        which is fixed with a small corrective turn.
     2. Position: when a wall ahead (or behind) is already on the map, the measured distance
        tells where the robot is along that line, and the position is nudged to match.
     3. At every START / END stop the robot re-checks its position along both axes. */
  const ALIGN_ANGLES = [55, 65, 90, 115, 125];
  const mapAngles = () => [...new Set(anglesArr().concat(ALIGN_ANGLES))].sort((a, b) => a - b).slice(0, 16);
  const rc = s => { check(s); applyTele(s); return s; };

  // returns how many degrees the robot is rotated LEFT of square (0 = unknown / square)
  function wallSkew(readings) {
    const g = a => { const r = readings.find(x => x[0] === a); return r ? r[1] : -1; };
    const D = g(90);
    if (!(D > 25 && D < 130)) return 0;
    const est = [];
    for (const k of [25, 35]) {
      const l = g(90 + k), r = g(90 - k);
      if (!(l > D && r > D && l < D * 2 && r < D * 2)) continue;     // both beams must hit the same wall
      const al = Math.acos(Math.min(1, D / l)) * 180 / Math.PI;
      const ar = Math.acos(Math.min(1, D / r)) * 180 / Math.PI;
      est.push((al - ar) / 2);
    }
    if (est.length < 2 || Math.abs(est[0] - est[1]) > 3) return 0;   // both beam pairs must agree
    const delta = (est[0] + est[1]) / 2;
    return Math.abs(delta) < 2.5 || Math.abs(delta) > 15 ? 0 : delta;
  }

  // Correct position along the facing axis: compare the straight-ahead / straight-back
  // distances with where the map says those walls are (average of earlier echoes).
  function snapAxis(front, back) {
    const cs = S.cellCm, fixes = [];
    for (const [d, dir] of [[front, pose.h], [back, (pose.h + 2) % 4]]) {
      if (!(d > 12 && d < 150)) continue;
      const axisX = DX[dir] !== 0;
      const hx = pose.cx + DX[dir] * d, hy = pose.cy + DY[dir] * d;
      let bestErr = null;
      for (let k = -1; k <= 1; k++) {
        const x = Math.floor(hx / cs) + (axisX ? k : 0), y = Math.floor(hy / cs) + (axisX ? 0 : k);
        const m = map.hitMean(x, y, 2);
        if (!m) continue;
        const err = axisX ? m.x - hx : m.y - hy;
        if (bestErr === null || Math.abs(err) < Math.abs(bestErr)) bestErr = err;
      }
      if (bestErr !== null && Math.abs(bestErr) < cs * 0.6) fixes.push({ axisX, err: bestErr });
    }
    if (!fixes.length) return;
    const ox = pose.cx, oy = pose.cy;
    for (const f of fixes) {
      const w = 0.7 / fixes.filter(g => g.axisX === f.axisX).length;
      if (f.axisX) pose.cx += f.err * w; else pose.cy += f.err * w;
    }
    if (!syncCell()) { pose.cx = ox; pose.cy = oy; }
  }

  // keep the grid cell in step with the cm position; refuses to move into a wall cell
  function syncCell() {
    const cs = S.cellCm, nx = Math.floor(pose.cx / cs), ny = Math.floor(pose.cy / cs);
    if (nx === pose.x && ny === pose.y) return true;
    if (map.state(nx, ny) === OCC && !map.weak(nx, ny)) return false;
    pose.x = nx; pose.y = ny;
    map.markVisited(nx, ny);
    trail.push({ x: nx, y: ny });
    return true;
  }

  async function correctHeading(delta) {
    const ms = Math.round(Math.abs(delta) / cal().degps * 1000);
    if (ms < 20) return;
    tele.act = 'squaring up'; updateUI();
    rc(await robot.turn(delta > 0 ? 'R' : 'L', ms, S.turnSpd));
  }

  // quick look at the wall ahead: square up to it and fix position along it
  async function alignAhead() {
    const r = rc(await robot.scan(ALIGN_ANGLES));
    const readings = r.scan || [];
    const g = a => { const x = readings.find(q => q[0] === a); return x ? x[1] : -1; };
    const delta = wallSkew(readings);
    if (!delta) snapAxis(g(90), -1);
    if (delta) await correctHeading(delta);
    stats.scans++;
    updateUI();
  }

  // integrate=false: only use the scan to correct drift (missions – people are not walls)
  async function scanHere(opts = {}) {
    const integrate = opts.integrate !== false;
    await gate();
    tele.act = integrate ? 'scanning' : 'checking position'; updateUI();
    const r = rc(await robot.scan(integrate ? mapAngles() : ALIGN_ANGLES));
    const readings = r.scan || [];
    const delta = wallSkew(readings);
    const thBase = headingDeg() - delta;                     // true facing while scanning
    const g = a => { const x = readings.find(q => q[0] === a); return x ? x[1] : -1; };
    if (!delta) snapAxis(g(90), r.sback);
    const p = poseCm(), rays = [];
    for (const [a, d] of readings) {
      const th = thBase - (a - 90);                          // servo > 90 = left
      if (integrate) map.integrate(p.x, p.y, th, d, { maxTrust: S.maxTrust, record: a === 90 && !delta });
      rays.push({ th, d });
    }
    if (integrate) map.integrate(p.x, p.y, thBase + 180, r.sback, { maxTrust: S.maxTrust, record: !delta });
    rays.push({ th: thBase + 180, d: r.sback });
    lastRays = { x: p.x, y: p.y, rays };
    map.markVisited(pose.x, pose.y);
    stats.scans++;
    if (delta) await correctHeading(delta);
    updateUI();
  }

  async function turnTo(d) {
    let diff = (d - pose.h + 4) % 4, turned = false;
    while (diff !== 0) {
      await gate();
      // the robot's nose swings out when it spins - make room first if a wall is close in front
      const st = await robot.status(); applyTele(st);
      if (st.front > 0 && st.front < 22 && !(st.back > 0 && st.back < 25)) await backUp(14);
      else if (st.back > 0 && st.back < 12 && !(st.front > 0 && st.front < 30)) await nudgeForward(10);
      const right = diff !== 3;
      tele.act = right ? 'turning right' : 'turning left'; updateUI();
      rc(await robot.turn(right ? 'R' : 'L', ms90(), S.turnSpd));
      pose.h = (pose.h + (right ? 1 : 3)) % 4;
      diff = (d - pose.h + 4) % 4;
      turned = true;
      updateUI();
      await appWait(150);
    }
    return turned;
  }

  // Drive to the centre of the cell n steps ahead (or behind, driving in reverse with the
  // back sensor watching). learn=true also adds the front/back readings to the map.
  async function driveCells(n, opts = {}) {
    await gate();
    const back = !!opts.back, dirH = back ? (pose.h + 2) % 4 : pose.h;
    const cs = S.cellCm, alongX = DX[dirH] !== 0, sgn = alongX ? DX[dirH] : DY[dirH];
    const cur = alongX ? pose.cx : pose.cy;
    const target = ((alongX ? pose.x : pose.y) + sgn * n + 0.5) * cs;
    const distCm = (target - cur) * sgn;
    const ms = Math.round(distCm / cal().cmps * 1000);
    tele.act = `${back ? 'reverse' : 'forward'} ${n} cell${n > 1 ? 's' : ''}`; updateUI();
    const s = rc(await robot.drive(back ? 'B' : 'F', ms, S.driveSpd, S.safeCm));
    const done = s.result === 'done' ? distCm
      : s.result === 'stuck' ? 0
        : Math.max(0, Math.min(distCm, s.elapsed / 1000 * cal().cmps));
    const c0 = alongX ? pose.x : pose.y;
    let steps = Math.max(0, (Math.floor((cur + sgn * done) / cs) - c0) * sgn);
    for (let i = 0; i < steps; i++) {                     // walk cell by cell so the trail is marked
      const nx = pose.x + DX[dirH], ny = pose.y + DY[dirH];
      if (map.state(nx, ny) === OCC && !map.weak(nx, ny)) { steps = i; break; }
      pose.x = nx; pose.y = ny; map.markVisited(nx, ny); trail.push({ x: nx, y: ny });
    }
    const cellNow = c0 + sgn * steps;                     // keep the cm position inside the cell we are in
    const pos = Math.min((cellNow + 1) * cs - 1, Math.max(cellNow * cs + 1, cur + sgn * done));
    if (alongX) pose.cx = pos; else pose.cy = pos;
    stats.dist += Math.abs(pos - cur);
    snapAxis(s.front, s.back);
    if (opts.learn) {                                     // both sensors feed the map after every move
      const p = poseCm();
      if (s.front > 0) map.integrate(p.x, p.y, headingDeg(), s.front, { maxTrust: S.maxTrust });
      if (s.back > 0) map.integrate(p.x, p.y, headingDeg() + 180, s.back, { maxTrust: S.maxTrust });
    }
    if (s.result === 'stuck') {
      log('Wheels turning but robot not moving – something low/thin is in the way', 'warn');
      const ax = pose.x + DX[dirH], ay = pose.y + DY[dirH];
      if (map.inB(ax, ay) && !map.visited[map.i(ax, ay)]) map.L[map.i(ax, ay)] = 4;   // remember it as an obstacle
      if (back) await nudgeForward(8); else await backUp(10);
    }
    updateUI();
    return { blocked: s.result === 'blocked' || s.result === 'stuck', stuck: s.result === 'stuck', reverse: back, moved: steps, front: back ? s.back : s.front };
  }

  async function nudgeForward(cm) {
    const ms = Math.round(cm / cal().cmps * 1000);
    const s = rc(await robot.drive('F', ms, S.driveSpd, Math.min(S.safeCm, 15)));
    const done = s.result === 'done' ? cm : Math.min(cm, s.elapsed / 1000 * cal().cmps);
    const ox = pose.cx, oy = pose.cy;
    pose.cx += DX[pose.h] * done; pose.cy += DY[pose.h] * done;
    if (!syncCell()) { pose.cx = ox; pose.cy = oy; }
  }

  async function backUp(cm) {
    const ms = Math.round(cm / cal().cmps * 1000);
    tele.act = 'backing up'; updateUI();
    const s = rc(await robot.drive('B', ms, S.driveSpd, Math.min(S.safeCm, 15)));
    const done = s.result === 'done' ? cm : Math.min(cm, s.elapsed / 1000 * cal().cmps);
    const ox = pose.cx, oy = pose.cy;
    pose.cx -= DX[pose.h] * done; pose.cy -= DY[pose.h] * done;
    if (!syncCell()) { pose.cx = ox; pose.cy = oy; }
  }

  async function executeRun(run, align, learn) {
    // target is behind us: reverse (back sensor watches) instead of spinning 180°
    if ((run.dir - pose.h + 4) % 4 === 2) return driveCells(Math.min(run.n, 2), { back: true, learn });
    const turned = await turnTo(run.dir);
    if (turned || align) {                                 // square up to the wall ahead before driving
      const st = await robot.status();
      if (st.front > 25 && st.front < 130) await alignAhead();
    }
    return driveCells(run.n, { learn });
  }

  /* ============================= MAPPING ============================ */
  function bump(c, by = 1) { const k = map.i(c.x, c.y); blacklist.set(k, (blacklist.get(k) || 0) + by); }

  async function doMapping() {
    setPhase('MAPPING', 'Exploring – the robot scans, moves to the nearest unexplored edge, and repeats.');
    log('Mapping started', 'ok');
    blacklist.clear();
    map.markVisited(pose.x, pose.y);
    await scanHere();
    const limit = Math.round(S.limitM * 100 / S.cellCm);
    let steps = 0;
    for (; steps < 400; steps++) {
      await gate();
      const f = map.findFrontier(pose, blacklist, map.home, limit);
      if (!f) { log('No unexplored reachable area left – map complete', 'ok'); break; }
      targetCell = f;
      if (same(f, pose)) {                               // unknown is right beside us: look at it
        bump(f);
        const d = map.unknownDir(pose);
        if (d < 0) { bump(f, 9); continue; }
        await turnTo(d); await scanHere();
        continue;
      }
      const path = map.plan(pose, f, pose.h, false, true);
      if (!path) { bump(f, 9); continue; }
      plannedPath = path; updateUI();
      const r = await executeRun(firstRun(path, 2), false, true);
      if (r.moved === 0) {
        bump(f);
        if (r.blocked && !r.reverse && !r.stuck) {      // something is right ahead
          const ax = pose.x + DX[pose.h], ay = pose.y + DY[pose.h];
          if (map.inB(ax, ay) && !map.visited[map.i(ax, ay)]) map.L[map.i(ax, ay)] = Math.min(4, map.L[map.i(ax, ay)] + 1.5);
        }
      }
      await scanHere();
    }
    if (steps >= 400) log('Step limit reached – finishing mapping', 'warn');
    targetCell = null; plannedPath = null;
    mapped = true;
    if (!startCell) startCell = { x: map.home.x, y: map.home.y };
    if (!endCell) autoEnd();
    const st = map.stats();
    log(`Map: ${st.areaM2.toFixed(1)} m² free space, ${st.occ} obstacle cells`, 'ok');
    setPhase('RETURNING', 'Mapping done – returning to the home position.');
    await navigateTo(map.home);
    await turnTo(0);
    setPhase('READY', 'Robot is home. Check start/end on the map, then start the shuttle.');
    log('Robot is back home – ready for missions', 'ok');
  }

  /* ========================== NAVIGATION ============================ */
  // Look along both axes (front+back, then left+right) to pin down x and y against the map
  async function relocalize() {
    tele.act = 'checking position'; updateUI();
    await scanHere({ integrate: false });
    await turnTo((pose.h + 1) % 4);
    await scanHere({ integrate: false });
  }

  // Follow the plan; if something blocks the way, wait for it to move, else detour around it.
  async function navigateTo(goal) {
    let noPath = 0, moves = 0, stuck = 0;
    while (!same(pose, goal)) {
      await gate();
      if (++moves > 300) throw new Error('Could not reach the goal (too many moves)');
      if (moves % 40 === 0) { log('Taking a while – re-checking position against the walls', 'warn'); await relocalize(); }
      const path = map.plan(pose, goal, pose.h, true);
      if (!path) {
        plannedPath = null; updateUI();
        if (noPath === 0) log('No clear route right now – waiting for the way to open…', 'warn');
        if (++noPath > 20) throw new Error('Goal unreachable – the route stayed blocked');
        await appWait(3000);
        continue;
      }
      noPath = 0;
      plannedPath = path; updateUI();
      const r = await executeRun(firstRun(path, 4), true);
      if (!r.blocked) { stuck = 0; continue; }

      if (r.stuck || r.front > S.safeCm + 12 || r.front < 0) {          // stopped but nothing in front: side contact / drift
        if (++stuck > 6) throw new Error('Robot keeps getting stuck – place it at home and click "Robot is at home"');
        if (stuck % 2 === 0) await relocalize();
        log('Unexpected stop (probably brushed something at the side) – backing up and trying another way', 'warn');
        // we can't see sideways while driving, so keep away from everything around the spot
        const h = pose.h, L = (h + 3) % 4, R = (h + 1) % 4, ax = pose.x + DX[h], ay = pose.y + DY[h];
        const around = [[ax, ay], [ax + DX[L], ay + DY[L]], [ax + DX[R], ay + DY[R]], [pose.x + DX[L], pose.y + DY[L]], [pose.x + DX[R], pose.y + DY[R]]];
        await backUp(S.cellCm * 0.4);
        for (const [x, y] of around) if (!(x === goal.x && y === goal.y) && !(x === pose.x && y === pose.y)) map.addDyn(x, y, 2 * S.dynTTL * 1000 / tScale());
        await alignAhead();
        continue;
      }
      stats.obstacles++;
      log(`Obstacle at ${fmtCm(r.front)} – waiting to see if it moves`, 'warn');
      let clear = false;
      for (let k = 0; k < 3 && !clear; k++) {
        await appWait(1500);
        const s = await robot.status();
        tele.front = s.front; updateUI();
        clear = !(s.front > 0 && s.front < S.safeCm + 12);
      }
      if (clear && r.moved === 0 && ++stuck >= 2) clear = false;   // keeps failing: treat as blocked
      if (clear) { log('Way is clear again – continuing'); continue; }
      stuck = 0;
      const k = Math.max(1, Math.round((tele.front > 0 ? tele.front : S.cellCm * 0.5) / S.cellCm + 0.3));
      const ox = pose.x + DX[pose.h] * k, oy = pose.y + DY[pose.h] * k;
      if (ox === goal.x && oy === goal.y) { log('Obstacle is standing on the target – waiting', 'warn'); await appWait(3000); continue; }
      map.addDyn(ox, oy, S.dynTTL * 1000 / tScale());
      stats.replans++;
      log('Still blocked – planning a detour around it', 'warn');
    }
    plannedPath = null; updateUI();
  }

  /* ============================ MISSION ============================= */
  async function doMission() {
    const trips = Math.max(0, parseInt($('#trips').value, 10) || 0);
    S.trips = trips; S.dwell = Math.max(0, +$('#dwell').value || 0); saveSettings();
    setPhase('MISSION', `Porter shuttle: START ⇄ END${trips ? ` × ${trips}` : ' (continuous)'}.`);
    if (!same(pose, startCell)) { log('Driving to the START point…'); await navigateTo(startCell); }
    for (let t = 1; trips === 0 || t <= trips; t++) {
      const tag = `Trip ${t}${trips ? '/' + trips : ''}`;
      log(`${tag}: carrying load to END`);
      await navigateTo(endCell);
      stats.deliveries++; updateUI();
      log(`${tag}: arrived at END – unloading`, 'ok');
      await relocalize();
      await appWait(S.dwell * 1000);
      log(`${tag}: returning to START`);
      await navigateTo(startCell);
      log(`${tag}: back at START`, 'ok');
      await relocalize();
      if (trips === 0 || t < trips) await appWait(S.dwell * 1000);
    }
    setPhase('READY', 'Shuttle finished. Robot waiting at START.');
    log('Shuttle mission complete', 'ok');
  }

  function autoEnd() {
    const from = startCell || map.home;
    const c = map.farthest(from);
    if (!c) { log('Not enough map yet to pick an end point', 'warn'); return; }
    endCell = c;
    const m = cellMeters(c);
    log(`Auto end point: ${m} from home (farthest safe reachable spot)`, 'ok');
    updateUI();
  }
  function cellMeters(c) {
    const fx = (c.x - map.home.x) * S.cellCm / 100, fy = (map.home.y - c.y) * S.cellCm / 100;
    return `(${fx.toFixed(2)} m, ${fy.toFixed(2)} m)`;
  }

  /* ============================ RENDER ============================== */
  const cv = $('#map'), ctx = cv.getContext('2d');
  let T = { s: 10, ox: 0, oy: 0 };
  const css = n => getComputedStyle(document.documentElement).getPropertyValue(n).trim();
  let C = {};
  function readColors() { C = { free: css('--free'), occ: css('--occ'), dyn: css('--dyn'), unk: css('--unk'), accent: css('--accent'), accent2: css('--accent2'), ok: css('--ok'), warn: css('--warn'), err: css('--err'), line: css('--line') }; }

  function simToBelief(x, y) {
    const hc = map.center(map.home), st = robot.room.start;
    return { x: hc.x + (x - st.x), y: hc.y + (y - st.y) };
  }
  function computeView() {
    let x0 = map.home.x - 6, y0 = map.home.y - 6, x1 = map.home.x + 6, y1 = map.home.y + 6;
    const b = map.knownBounds();
    if (b) { x0 = Math.min(x0, b.x0 - 2); y0 = Math.min(y0, b.y0 - 2); x1 = Math.max(x1, b.x1 + 2); y1 = Math.max(y1, b.y1 + 2); }
    x0 = Math.min(x0, pose.x - 2); y0 = Math.min(y0, pose.y - 2); x1 = Math.max(x1, pose.x + 2); y1 = Math.max(y1, pose.y + 2);
    if (robot && robot.room && window.SP_SHOW_TRUTH) {
      const a = simToBelief(0, 0), z = simToBelief(robot.room.w, robot.room.h);
      x0 = Math.min(x0, Math.floor(a.x / S.cellCm) - 1); y0 = Math.min(y0, Math.floor(a.y / S.cellCm) - 1);
      x1 = Math.max(x1, Math.floor(z.x / S.cellCm) + 1); y1 = Math.max(y1, Math.floor(z.y / S.cellCm) + 1);
    }
    if (view) { x0 = Math.min(view.x0, x0); y0 = Math.min(view.y0, y0); x1 = Math.max(view.x1, x1); y1 = Math.max(view.y1, y1); }
    view = { x0: Math.max(0, x0), y0: Math.max(0, y0), x1: Math.min(map.size - 1, x1), y1: Math.min(map.size - 1, y1) };
  }
  const cellPx = (x, y) => [T.ox + (x - view.x0) * T.s, T.oy + (y - view.y0) * T.s];
  const cmPx = (x, y) => [T.ox + (x / S.cellCm - view.x0) * T.s, T.oy + (y / S.cellCm - view.y0) * T.s];

  function render() {
    const wrap = $('#canvasWrap'), W = wrap.clientWidth, H = wrap.clientHeight, dpr = window.devicePixelRatio || 1;
    if (cv.width !== Math.round(W * dpr) || cv.height !== Math.round(H * dpr)) { cv.width = Math.round(W * dpr); cv.height = Math.round(H * dpr); }
    ctx.setTransform(dpr, 0, 0, dpr, 0, 0);
    computeView();
    const cols = view.x1 - view.x0 + 1, rows = view.y1 - view.y0 + 1;
    const s = Math.min((W - 20) / cols, (H - 50) / rows);
    T = { s, ox: (W - cols * s) / 2, oy: (H - 40 - rows * s) / 2 + 4 };
    ctx.fillStyle = C.unk; ctx.fillRect(0, 0, W, H);

    const now = Date.now();
    for (let y = view.y0; y <= view.y1; y++) for (let x = view.x0; x <= view.x1; x++) {
      const st = map.state(x, y);
      const [px, py] = cellPx(x, y);
      if (st === FREE) {
        ctx.fillStyle = map.dynBlocked(x, y, now) ? C.dyn : C.free;
        ctx.fillRect(px, py, s + 0.5, s + 0.5);
      } else if (st === OCC) {
        ctx.fillStyle = C.occ; ctx.fillRect(px, py, s + 0.5, s + 0.5);
      } else if (map.L[map.i(x, y)] !== 0) {
        ctx.fillStyle = 'rgba(34,52,74,.35)'; ctx.fillRect(px, py, s + 0.5, s + 0.5);
      }
    }
    if (s > 9) {                                         // grid lines
      ctx.strokeStyle = 'rgba(255,255,255,.04)'; ctx.lineWidth = 1; ctx.beginPath();
      for (let x = view.x0; x <= view.x1 + 1; x++) { const [px] = cellPx(x, 0); ctx.moveTo(px, T.oy); ctx.lineTo(px, T.oy + rows * s); }
      for (let y = view.y0; y <= view.y1 + 1; y++) { const [, py] = cellPx(0, y); ctx.moveTo(T.ox, py); ctx.lineTo(T.ox + cols * s, py); }
      ctx.stroke();
    }

    // simulator ground truth (the "real" room)
    if (robot && robot.room && window.SP_SHOW_TRUTH) {      // test rig overlay only
      ctx.save(); ctx.setLineDash([4, 4]); ctx.lineWidth = 1.2;
      for (const r of robot.walls) drawRect(r, 'rgba(58,160,255,.55)');
      for (const r of robot.extra) drawRect(r, 'rgba(255,159,67,.9)');
      ctx.restore();
      const b = simToBelief(robot.pose.x, robot.pose.y), [gx, gy] = cmPx(b.x, b.y);
      ctx.strokeStyle = 'rgba(58,160,255,.8)'; ctx.lineWidth = 1.5;
      ctx.beginPath(); ctx.arc(gx, gy, Math.max(4, s * 0.36), 0, Math.PI * 2); ctx.stroke();
    }

    // trail
    if (trail.length > 1) {
      ctx.strokeStyle = 'rgba(53,208,186,.25)'; ctx.lineWidth = Math.max(2, s * 0.15); ctx.lineJoin = 'round';
      ctx.beginPath();
      trail.forEach((c, i) => { const [px, py] = cellPx(c.x + 0.5, c.y + 0.5); i ? ctx.lineTo(px, py) : ctx.moveTo(px, py); });
      ctx.stroke();
    }
    // last scan rays
    if (lastRays) {
      ctx.strokeStyle = 'rgba(58,160,255,.35)'; ctx.lineWidth = 1;
      const [rx, ry] = cmPx(lastRays.x, lastRays.y);
      for (const r of lastRays.rays) {
        const len = r.d > 0 ? Math.min(r.d, S.maxTrust) : 60, t = r.th * Math.PI / 180;
        const [ex, ey] = cmPx(lastRays.x + Math.cos(t) * len, lastRays.y + Math.sin(t) * len);
        ctx.beginPath(); ctx.moveTo(rx, ry); ctx.lineTo(ex, ey); ctx.stroke();
        if (r.d > 0 && r.d <= S.maxTrust) { ctx.fillStyle = C.accent2; ctx.beginPath(); ctx.arc(ex, ey, 2.5, 0, Math.PI * 2); ctx.fill(); }
      }
    }
    // planned path
    if (plannedPath && plannedPath.length > 1) {
      ctx.save(); ctx.strokeStyle = C.accent; ctx.lineWidth = Math.max(2, s * 0.12); ctx.setLineDash([s * 0.35, s * 0.25]);
      ctx.beginPath();
      plannedPath.forEach((c, i) => { const [px, py] = cellPx(c.x + 0.5, c.y + 0.5); i ? ctx.lineTo(px, py) : ctx.moveTo(px, py); });
      ctx.stroke(); ctx.restore();
    }
    if (targetCell) {
      const [px, py] = cellPx(targetCell.x + 0.5, targetCell.y + 0.5);
      ctx.strokeStyle = '#b892ff'; ctx.lineWidth = 2;
      ctx.beginPath(); ctx.arc(px, py, s * 0.42, 0, Math.PI * 2); ctx.stroke();
    }
    // markers
    if (!same(startCell, map.home)) marker(map.home, '⌂', '#64748b');
    if (startCell) marker(startCell, 'S', C.ok);
    if (endCell) marker(endCell, 'E', C.warn);
    // robot (belief)
    const [cx, cy] = cmPx(pose.cx, pose.cy), a = HEAD_DEG[pose.h] * Math.PI / 180, R = Math.max(7, s * 0.42);
    ctx.save(); ctx.translate(cx, cy); ctx.rotate(a);
    ctx.fillStyle = C.accent; ctx.strokeStyle = '#06131c'; ctx.lineWidth = 2;
    ctx.beginPath(); ctx.moveTo(R, 0); ctx.lineTo(-R * 0.7, R * 0.7); ctx.lineTo(-R * 0.35, 0); ctx.lineTo(-R * 0.7, -R * 0.7); ctx.closePath();
    ctx.fill(); ctx.stroke(); ctx.restore();
    // scale bar (1 m)
    const m = 100 / S.cellCm * s, bx = W - m - 16, by = H - 18;
    ctx.strokeStyle = '#8a99ad'; ctx.lineWidth = 2; ctx.beginPath();
    ctx.moveTo(bx, by - 4); ctx.lineTo(bx, by); ctx.lineTo(bx + m, by); ctx.lineTo(bx + m, by - 4); ctx.stroke();
    ctx.fillStyle = '#8a99ad'; ctx.font = '11px system-ui'; ctx.textAlign = 'center'; ctx.fillText('1 m', bx + m / 2, by - 6);
  }
  function drawRect(r, color) {
    const a = simToBelief(r[0], r[1]), [x, y] = cmPx(a.x, a.y);
    const w = r[2] / S.cellCm * T.s, h = r[3] / S.cellCm * T.s;
    ctx.strokeStyle = color; ctx.strokeRect(x, y, w, h);
  }
  function marker(c, label, color) {
    const [px, py] = cellPx(c.x + 0.5, c.y + 0.5), r = Math.max(8, T.s * 0.4);
    ctx.fillStyle = color; ctx.beginPath(); ctx.arc(px, py, r, 0, Math.PI * 2); ctx.fill();
    ctx.fillStyle = '#06131c'; ctx.font = `700 ${Math.round(r * 1.1)}px system-ui`; ctx.textAlign = 'center'; ctx.textBaseline = 'middle';
    ctx.fillText(label, px, py + 1);
    ctx.textBaseline = 'alphabetic';
  }

  /* ============================ UI update ========================== */
  function applyTele(s) {
    if (!s) return;
    if (typeof s.front === 'number') tele.front = s.front;
    if (typeof s.back === 'number') tele.back = s.back;
    if (typeof s.servo === 'number') tele.servo = s.servo;
    updateTele();
  }
  function gauge(id, bar, v) {
    const el = $(id), box = el.parentElement;
    el.textContent = v > 0 ? v.toFixed(0) + ' cm' : '--';
    $(bar).style.width = (v > 0 ? Math.min(100, v / 200 * 100) : 0) + '%';
    box.classList.toggle('warn', v > 0 && v < S.safeCm + 10);
  }
  function updateTele() {
    gauge('#gFront', '#bFront', tele.front);
    gauge('#gBack', '#bBack', tele.back);
    $('#tServo').textContent = tele.servo + '°';
    $('#tAct').textContent = tele.act;
    $('#tPose').textContent = cellMeters(pose);
    $('#tHead').textContent = ['Ahead ↑', 'Right →', 'Back ↓', 'Left ←'][pose.h];
  }
  let rafPending = false;
  function updateUI() {
    updateTele();
    const st = map.stats();
    $('#sArea').textContent = st.areaM2.toFixed(1);
    $('#sScans').textContent = stats.scans;
    $('#sDist').textContent = (stats.dist / 100).toFixed(1);
    $('#sDeliv').textContent = stats.deliveries;
    $('#sObs').textContent = stats.obstacles;
    $('#sReplan').textContent = stats.replans;
    $('#seText').textContent = (startCell || endCell)
      ? `Start ${startCell ? cellMeters(startCell) : '—'} · End ${endCell ? cellMeters(endCell) : '—'}`
      : 'Start = home. End is picked automatically – or click the map.';
    $('#st1').className = mapped ? 'done' : (phase === 'MAPPING' ? 'active' : '');
    $('#st2').className = startCell && endCell ? 'done' : (mapped ? 'active' : '');
    $('#st3').className = phase === 'MISSION' ? 'active' : (stats.deliveries > 0 ? 'done' : '');
    refreshButtons();
    if (!rafPending) { rafPending = true; requestAnimationFrame(() => { rafPending = false; render(); }); }
  }
  function refreshButtons() {
    const idle = connected && !running;
    $('#mapBtn').disabled = !idle;
    $('#runBtn').disabled = !(idle && mapped && startCell && endCell);
    $('#homeBtn').disabled = !(idle && map.stats().free > 1);
    $('#pauseBtn').disabled = !running;
    $('#pauseBtn').textContent = paused ? 'Resume' : 'Pause';
    $('#autoEndBtn').disabled = running;
    $('#connectBtn').disabled = running;
    ['#calDriveBtn', '#calTurnBtn'].forEach(id => { $(id).disabled = !idle; });
    $('#mapBtn').textContent = mapped ? 'Re-map room' : 'Initiate mapping';
  }

  /* =========================== connection ========================== */
  function newMap() {
    map = new GridMap(81, +S.cellCm);
    pose = homePose();
    startCell = endCell = null; mapped = false;
    trail = []; plannedPath = null; targetCell = null; lastRays = null; view = null;
    blacklist.clear();
    Object.keys(stats).forEach(k => { stats[k] = 0; });
  }
  async function connect() {
    if (running) return;
    const pill = $('#connPill');
    try {
      robot = SP.TestRobot ? new SP.TestRobot() : new SP.RealRobot($('#ip').value.trim() || '192.168.4.1');
      robot.onStatus = applyTele;
      const p = await robot.ping();
      connected = true; badPolls = 0;
      pill.className = 'pill on'; pill.lastElementChild.textContent = 'Robot online';
      log(`Connected to ${p.name} (firmware ${p.fw})`, 'ok');
      setPhase(mapped ? 'READY' : 'IDLE', mapped ? 'Map loaded – ready for missions.' : 'Put the robot on its home mark, facing "ahead", and click Initiate mapping.');
      view = null;
    } catch (e) {
      connected = false;
      pill.className = 'pill bad'; pill.lastElementChild.textContent = 'Not reachable';
      log('Could not reach the robot: ' + e.message, 'err');
      log('Check: laptop joined Wi-Fi "SafePath_Robot", robot runs safepath_auto.ino, IP is 192.168.4.1', 'warn');
    }
    updateUI();
  }
  // live sensor polling while idle
  setInterval(async () => {
    if (!connected || running || !robot || robot.inFlight > 0) return;
    try { applyTele(await robot.status()); badPolls = 0; $('#connPill').className = 'pill on'; }
    catch (e) {
      if (++badPolls === 3) { $('#connPill').className = 'pill bad'; log('Robot is not answering…', 'warn'); }
    }
  }, 700);

  /* ============================ controls =========================== */
  function setTool(t) {
    tool = tool === t ? null : t;
    document.querySelectorAll('.tool').forEach(b => b.classList.toggle('on', b.dataset.tool === tool));
    const hints = { start: 'Click a free (explored) cell to set START', end: 'Click a free (explored) cell to set END' };
    const h = $('#toolHint'); h.textContent = hints[tool] || ''; h.classList.toggle('show', !!tool);
  }
  document.querySelectorAll('.tool').forEach(b => b.addEventListener('click', () => setTool(b.dataset.tool)));

  cv.addEventListener('click', e => {
    if (!tool || !view) return;
    const r = cv.getBoundingClientRect(), mx = e.clientX - r.left, my = e.clientY - r.top;
    const fx = (mx - T.ox) / T.s + view.x0, fy = (my - T.oy) / T.s + view.y0;
    const c = { x: Math.floor(fx), y: Math.floor(fy) };
    if (!map.inB(c.x, c.y) || map.state(c.x, c.y) !== FREE) { log('Pick a free (explored) cell – shown in blue', 'warn'); return; }
    if (tool === 'start') { startCell = c; log('START set to ' + cellMeters(c)); }
    if (tool === 'end') { endCell = c; log('END set to ' + cellMeters(c)); }
    if (same(startCell, endCell)) log('Start and end are the same cell – pick different spots', 'warn');
    setTool(null); updateUI();
  });

  $('#connectBtn').addEventListener('click', connect);
  $('#estop').addEventListener('click', () => {
    abortFlag = true; paused = false;
    if (robot) robot.stop();
    log('EMERGENCY STOP pressed', 'err');
    if (!running) setPhase('STOPPED', 'Robot halted.');
  });
  $('#pauseBtn').addEventListener('click', () => {
    if (!running) return;
    paused = !paused;
    if (paused) { prevPhase = phase; setPhase('PAUSED', 'Paused – finishes the current small move, then waits.'); log('Paused', 'warn'); }
    else { setPhase(prevPhase, ''); log('Resumed'); }
    updateUI();
  });
  $('#mapBtn').addEventListener('click', () => {
    if (mapped && !confirm('Start a new mapping run? The current map will be cleared. Put the robot back on its home mark first.')) return;
    if (mapped || map.stats().free > 0) {
      newMap();
      if (robot && robot.resetToStart) robot.resetToStart();
    }
    runTask(doMapping);
  });
  $('#runBtn').addEventListener('click', () => {
    if (same(startCell, endCell)) { log('Start and end must be different', 'warn'); return; }
    runTask(doMission);
  });
  $('#homeBtn').addEventListener('click', () => runTask(async () => {
    setPhase('RETURNING', 'Driving back to home.');
    await navigateTo(map.home); await turnTo(0);
    setPhase(mapped ? 'READY' : 'IDLE', 'Robot is home.');
    log('Robot is home', 'ok');
  }));
  $('#autoEndBtn').addEventListener('click', autoEnd);

  // map file
  $('#saveMap').addEventListener('click', () => {
    const data = { app: 'SafePath', map: map.toJSON(), start: startCell, end: endCell, mapped, savedAt: new Date().toISOString() };
    const a = document.createElement('a');
    a.href = URL.createObjectURL(new Blob([JSON.stringify(data)], { type: 'application/json' }));
    a.download = 'safepath-map.json'; a.click();
    setTimeout(() => URL.revokeObjectURL(a.href), 2000);
    log('Map saved to safepath-map.json', 'ok');
  });
  $('#loadMapBtn').addEventListener('click', () => $('#loadMap').click());
  $('#loadMap').addEventListener('change', e => {
    const f = e.target.files[0]; if (!f) return;
    const rd = new FileReader();
    rd.onload = () => {
      try {
        const o = JSON.parse(rd.result);
        map = GridMap.fromJSON(o.map);
        S.cellCm = map.cellCm; saveSettings(); buildSettings();
        startCell = o.start; endCell = o.end; mapped = !!o.mapped;
        pose = homePose(); trail = []; view = null; lastRays = null;
        log('Map loaded. Place the robot on its home mark facing ahead.', 'ok');
        setPhase(connected ? 'READY' : 'IDLE', 'Map loaded.');
      } catch (err) { log('Could not read that map file', 'err'); }
      updateUI();
    };
    rd.readAsText(f); e.target.value = '';
  });
  $('#resetPose').addEventListener('click', () => {
    if (running) return;
    pose = homePose(); trail = [];
    if (robot && robot.resetToStart) robot.resetToStart();
    log('Position reset: robot is at home, facing ahead', 'ok'); updateUI();
  });
  $('#clearMap').addEventListener('click', () => {
    if (running || !confirm('Clear the whole map?')) return;
    newMap(); if (robot && robot.resetToStart) robot.resetToStart();
    setPhase('IDLE', 'Map cleared.'); log('Map cleared'); updateUI();
  });

  // calibration
  $('#calDriveBtn').addEventListener('click', () => runTask(async () => {
    log('Calibration: driving forward for 2 s – make sure there is 1.5 m of space…');
    const s = check(await robot.drive('F', 2000, S.driveSpd, S.safeCm));
    calElapsed = s.elapsed;
    log(`Drove ${s.elapsed} ms (${s.result}). Measure how far it went (cm) and click Apply.`, 'ok');
    log('Note: put the robot back on its home mark before mapping.', 'warn');
  }));
  $('#calCmApply').addEventListener('click', () => {
    const cm = +$('#calCm').value;
    if (!(cm > 0) || !(calElapsed > 0)) { log('Run the drive test first, then enter the measured cm', 'warn'); return; }
    S.cmPerSec = Math.round(cm / (calElapsed / 1000) * 10) / 10; saveSettings(); buildSettings();
    log(`Drive speed set to ${S.cmPerSec} cm/s`, 'ok');
  });
  $('#calTurnBtn').addEventListener('click', () => runTask(async () => {
    log(`Calibration: turning right for ${ms90()} ms (should be 90°)…`);
    check(await robot.turn('R', ms90(), S.turnSpd));
    log('Estimate how many degrees it actually turned and click Apply.', 'ok');
  }));
  $('#calDegApply').addEventListener('click', () => {
    const deg = +$('#calDeg').value;
    if (!(deg > 0)) { log('Enter the degrees it actually turned', 'warn'); return; }
    S.degPerSec = Math.round(S.degPerSec * deg / 90 * 10) / 10; saveSettings(); buildSettings();
    log(`Turn speed set to ${S.degPerSec} °/s – a 90° turn now takes ${ms90()} ms`, 'ok');
  });
  $('#resetSettings').addEventListener('click', () => {
    const cell = S.cellCm; S = Object.assign({}, DEFAULTS); S.cellCm = cell; saveSettings(); buildSettings();
    log('Settings reset to defaults');
  });

  function buildSettings() {
    const g = $('#settingsGrid'); g.innerHTML = '';
    for (const [key, label, hint, type] of FIELDS) {
      const l = document.createElement('label');
      l.innerHTML = `${label}${hint ? ` <small>${hint}</small>` : ''}`;
      const inp = document.createElement('input');
      inp.type = type === 'text' ? 'text' : 'number'; inp.value = S[key]; inp.step = 'any';
      inp.addEventListener('change', () => {
        const v = type === 'text' ? inp.value : +inp.value;
        if (type !== 'text' && !(v > 0)) { inp.value = S[key]; return; }
        if (key === 'cellCm' && v !== S.cellCm) {
          if (running || (map.stats().free > 0 && !confirm('Changing cell size clears the map. Continue?'))) { inp.value = S[key]; return; }
          S.cellCm = v; newMap();
        }
        S[key] = v; saveSettings();
        updateUI();
      });
      l.appendChild(inp); g.appendChild(l);
    }
    $('#trips').value = S.trips; $('#dwell').value = S.dwell;
  }

  /* ============================== init ============================= */
  readColors();
  buildSettings();
  window.addEventListener('resize', () => updateUI());
  setPhase('IDLE', 'Join the robot Wi-Fi, then click Connect.');
  log('SafePath dashboard ready. Connect your laptop to Wi-Fi "SafePath_Robot", then click Connect.');
  if (settingsReset) log('New slower speed settings loaded – please run both calibration tests again.', 'warn');
  updateUI();

  // exposed for debugging / testing in the browser console
  window.SafePath = {
    get state() { return { phase, mapped, pose, startCell, endCell, stats, running }; },
    get simError() {                                        // simulator only: belief vs truth (cm, deg)
      if (!robot || !robot.room) return null;
      const b = simToBelief(robot.pose.x, robot.pose.y), c = poseCm();
      const th = ((robot.pose.th - HEAD_DEG[pose.h]) % 360 + 540) % 360 - 180;
      return { posCm: Math.round(Math.hypot(b.x - c.x, b.y - c.y)), headDeg: Math.round(th * 10) / 10 };
    },
    get robot() { return robot; }, get pose() { return pose; }, get plan() { return plannedPath; }, get cellCm() { return S.cellCm; }, get home() { return map.home; },
    connect, S
  };
})();
