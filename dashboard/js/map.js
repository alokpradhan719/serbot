/* SafePath – occupancy grid map, exploration and path planning (runs on the laptop) */
(function () {
  'use strict';
  const SP = (window.SP = window.SP || {});

  const UNK = 0, FREE = 1, OCC = 2;
  // headings: 0 = North (up), 1 = East, 2 = South, 3 = West
  const DX = [0, 1, 0, -1];
  const DY = [-1, 0, 1, 0];
  const HEAD_DEG = [-90, 0, 90, 180];   // screen angle of each heading (y points down)

  class MinHeap {
    constructor() { this.a = []; }
    get size() { return this.a.length; }
    push(k, p) {
      const a = this.a; a.push([p, k]);
      let i = a.length - 1;
      while (i > 0) {
        const j = (i - 1) >> 1;
        if (a[j][0] <= a[i][0]) break;
        [a[i], a[j]] = [a[j], a[i]]; i = j;
      }
    }
    pop() {
      const a = this.a, top = a[0], last = a.pop();
      if (a.length) {
        a[0] = last;
        let i = 0;
        for (;;) {
          const l = 2 * i + 1, r = l + 1;
          let m = i;
          if (l < a.length && a[l][0] < a[m][0]) m = l;
          if (r < a.length && a[r][0] < a[m][0]) m = r;
          if (m === i) break;
          [a[i], a[m]] = [a[m], a[i]]; i = m;
        }
      }
      return top[1];
    }
  }

  class GridMap {
    constructor(size = 81, cellCm = 25) {
      this.size = size;
      this.cellCm = cellCm;
      this.L = new Float32Array(size * size);        // log-odds of "occupied"
      this.visited = new Uint8Array(size * size);    // robot has physically been here
      this.dyn = new Map();                          // temporary obstacles: idx -> expiry time
      this.hx = new Float32Array(size * size);       // sum of echo positions per cell
      this.hy = new Float32Array(size * size);
      this.hn = new Uint16Array(size * size);
      this.home = { x: size >> 1, y: size >> 1 };
    }
    i(x, y) { return y * this.size + x; }
    inB(x, y) { return x >= 0 && y >= 0 && x < this.size && y < this.size; }

    state(x, y) {
      if (!this.inB(x, y)) return OCC;
      const k = this.i(x, y);
      if (this.visited[k]) return FREE;
      const l = this.L[k];
      if (l > 0.8) return OCC;
      if (l < -0.6) return FREE;
      return UNK;
    }
    dynBlocked(x, y, now = Date.now()) {
      const k = this.i(x, y), e = this.dyn.get(k);
      if (e === undefined) return false;
      if (e < now) { this.dyn.delete(k); return false; }
      return true;
    }
    addDyn(x, y, ttlMs) { if (this.inB(x, y)) this.dyn.set(this.i(x, y), Date.now() + ttlMs); }
    passable(x, y, useDyn, explore) {
      return (this.state(x, y) === FREE || (explore && this.weak(x, y))) && !(useDyn && this.dynBlocked(x, y));
    }
    // seen as an obstacle only once or twice: may be a false echo (ultrasonic beams are wide)
    weak(x, y) {
      if (!this.inB(x, y)) return false;
      const k = this.i(x, y);
      return !this.visited[k] && this.L[k] > 0.8 && this.L[k] <= 2.0;
    }
    nearOcc(x, y) {
      for (let dy = -1; dy <= 1; dy++)
        for (let dx = -1; dx <= 1; dx++)
          if ((dx || dy) && this.state(x + dx, y + dy) === OCC) return true;
      return false;
    }
    touchesOcc(x, y) {
      for (let d = 0; d < 4; d++) if (this.state(x + DX[d], y + DY[d]) === OCC) return true;
      return false;
    }
    markVisited(x, y) {
      if (!this.inB(x, y)) return;
      const k = this.i(x, y);
      this.visited[k] = 1;
      this.L[k] = -4;
    }
    hitMean(x, y, minN) {
      if (!this.inB(x, y) || this.state(x, y) !== OCC) return null;
      const k = this.i(x, y), n = this.hn[k];
      return n >= minN ? { x: this.hx[k] / n, y: this.hy[k] / n } : null;
    }
    center(c) { return { x: (c.x + 0.5) * this.cellCm, y: (c.y + 0.5) * this.cellCm }; }

    /* Add one ultrasonic reading.  (px,py) robot position in cm, theta = beam angle (deg),
       d = distance in cm (-1 = no echo). An HC-SR04 reports the NEAREST thing anywhere in its
       ~15–25° wide beam, so: the narrow centre of the beam is marked free up to the echo, and
       the echo is marked as an obstacle across the beam's width (capped to ±15 cm sideways).
       This makes thin things like chair legs show up instead of being missed. */
    integrate(px, py, thetaDeg, d, opts = {}) {
      const maxTrust = opts.maxTrust || 150, noHit = opts.noHit || 60;
      const hit = d > 0 && d <= maxTrust;
      const range = hit ? d : (d > maxTrust ? maxTrust : noHit);
      const cs = this.cellCm, free = new Set(), occ = new Map();
      const step = cs / 4;
      for (const off of [-7, 0, 7]) {
        const th = (thetaDeg + off) * Math.PI / 180, c = Math.cos(th), s = Math.sin(th);
        const lim = range - cs * 0.6;
        for (let r = 0; r < lim; r += step) {
          const cx = Math.floor((px + c * r) / cs), cy = Math.floor((py + s * r) / cs);
          if (this.inB(cx, cy)) free.add(this.i(cx, cy));
        }
      }
      if (hit) {
        const half = Math.min(10, Math.atan(10 / d) * 180 / Math.PI);
        const offs = [0], stepA = Math.max(2, half / 3);
        for (let o = stepA; o <= half + 0.01; o += stepA) offs.push(o, -o);
        for (const off of offs) {
          const th = (thetaDeg + off) * Math.PI / 180;
          const hx = px + Math.cos(th) * d, hy = py + Math.sin(th) * d;
          const cx = Math.floor(hx / cs), cy = Math.floor(hy / cs);
          if (!this.inB(cx, cy)) continue;
          const k = this.i(cx, cy), w = Math.abs(off) < 1 ? 1.2 : 0.45;
          occ.set(k, Math.max(occ.get(k) || 0, w));
          if (Math.abs(off) < 1 && opts.record) { this.hx[k] += hx; this.hy[k] += hy; this.hn[k]++; }   // straight-on echo: wall position
        }
      }
      for (const k of free) if (!occ.has(k)) this.L[k] = Math.max(-4, this.L[k] - 0.7);
      for (const [k, w] of occ) this.L[k] = Math.min(4, this.L[k] + w);
    }

    isFrontier(x, y) {
      if (this.state(x, y) !== FREE) return false;
      for (let d = 0; d < 4; d++) if (this.state(x + DX[d], y + DY[d]) === UNK) return true;
      return false;
    }
    // exploring: a free cell next to unknown space, or next to a doubtful obstacle worth a closer look
    isFrontierX(x, y) {
      if (this.state(x, y) !== FREE) return false;
      for (let d = 0; d < 4; d++) {
        const nx = x + DX[d], ny = y + DY[d];
        if (this.state(nx, ny) === UNK) return true;
        if (this.weak(nx, ny)) {
          for (let e = 0; e < 4; e++) if (this.state(nx + DX[e], ny + DY[e]) === UNK) return true;
        }
      }
      return false;
    }
    unknownCount(x, y) {
      let n = 0;
      for (let dy = -1; dy <= 1; dy++) for (let dx = -1; dx <= 1; dx++)
        if (this.state(x + dx, y + dy) === UNK) n++;
      return n;
    }
    unknownDir(c) {
      for (let d = 0; d < 4; d++) if (this.state(c.x + DX[d], c.y + DY[d]) === UNK) return d;
      return -1;
    }

    bfs(start, useDyn, explore) {
      const N = this.size, dist = new Int32Array(N * N).fill(-1), q = [];
      dist[this.i(start.x, start.y)] = 0; q.push(start.x, start.y);
      for (let h = 0; h < q.length; h += 2) {
        const x = q[h], y = q[h + 1], dd = dist[this.i(x, y)];
        for (let d = 0; d < 4; d++) {
          const nx = x + DX[d], ny = y + DY[d];
          if (!this.inB(nx, ny)) continue;
          const k = this.i(nx, ny);
          if (dist[k] >= 0 || !this.passable(nx, ny, useDyn, explore)) continue;
          dist[k] = dd + 1; q.push(nx, ny);
        }
      }
      return dist;
    }

    // Nearest reachable cell on the border between known-free and unknown space
    findFrontier(robot, blacklist, origin, limitCells) {
      const dist = this.bfs(robot, false, true), N = this.size;
      let best = null, bestScore = Infinity;
      for (let y = 0; y < N; y++) for (let x = 0; x < N; x++) {
        const k = this.i(x, y);
        if (dist[k] < 0 || !this.isFrontierX(x, y)) continue;
        if (this.unknownCount(x, y) < 2) continue;          // ignore tiny unknown specks
        if ((blacklist.get(k) || 0) >= 3) continue;
        if (Math.max(Math.abs(x - origin.x), Math.abs(y - origin.y)) > limitCells) continue;
        const score = dist[k] - 0.35 * this.unknownCount(x, y);
        if (score < bestScore) { bestScore = score; best = { x, y }; }
      }
      return best;
    }

    // Farthest reachable, comfortable (not hugging walls) cell from a point – auto end point
    farthest(from) {
      const dist = this.bfs(from, false), N = this.size;
      let best = null, bd = -1, fb = null, fbd = -1;
      for (let y = 0; y < N; y++) for (let x = 0; x < N; x++) {
        const d = dist[this.i(x, y)];
        if (d <= 0) continue;
        if (d > fbd) { fbd = d; fb = { x, y }; }
        if (!this.nearOcc(x, y) && this.unknownCount(x, y) === 0 && d > bd) { bd = d; best = { x, y }; }
      }
      return best || fb;
    }

    /* A* on the grid. State includes heading so the planner prefers fewer turns
       (every turn adds drift on a robot without wheel encoders). */
    // Keep a full cell away from obstacles if possible (room for drift); squeeze closer only if needed
    plan(start, goal, h0, useDyn, explore) {
      return this.planA(start, goal, h0, useDyn, 2, explore) || this.planA(start, goal, h0, useDyn, 1, explore) ||
        this.planA(start, goal, h0, useDyn, 0, explore);
    }
    planA(start, goal, h0, useDyn, inflate, explore) {
      if (!this.inB(goal.x, goal.y) || !this.passable(goal.x, goal.y, useDyn)) return null;
      const N = this.size, total = N * N * 4;
      const g = new Float32Array(total).fill(Infinity);
      const came = new Int32Array(total).fill(-1);
      const closed = new Uint8Array(total);
      const heap = new MinHeap();
      const H = (x, y) => Math.abs(x - goal.x) + Math.abs(y - goal.y);
      const sk = (start.y * N + start.x) * 4 + h0;
      g[sk] = 0; heap.push(sk, H(start.x, start.y));
      while (heap.size) {
        const k = heap.pop();
        if (closed[k]) continue;
        closed[k] = 1;
        const h = k & 3, c = k >> 2, x = c % N, y = (c - x) / N;
        if (x === goal.x && y === goal.y) {
          const path = [];
          for (let q = k; q !== -1; q = came[q]) { const cc = q >> 2; path.push({ x: cc % N, y: (cc - cc % N) / N }); }
          return path.reverse();
        }
        for (let d = 0; d < 4; d++) {
          const nx = x + DX[d], ny = y + DY[d];
          if (!this.inB(nx, ny) || !this.passable(nx, ny, useDyn, explore)) continue;
          if (inflate && !(nx === goal.x && ny === goal.y) &&
              (inflate === 2 ? this.nearOcc(nx, ny) : this.touchesOcc(nx, ny))) continue;
          const turn = (d - h + 4) & 3;
          const cost = 1 + (turn === 0 ? 0 : turn === 2 ? 2.4 : 1.2) + (this.nearOcc(nx, ny) ? 1.5 : 0) +
            (explore && this.state(nx, ny) === OCC ? 4 : 0);
          const nk = (ny * N + nx) * 4 + d, ng = g[k] + cost;
          if (ng < g[nk]) { g[nk] = ng; came[nk] = k; heap.push(nk, ng + H(nx, ny)); }
        }
      }
      return null;
    }

    knownBounds() {
      const N = this.size;
      let x0 = N, y0 = N, x1 = -1, y1 = -1;
      for (let y = 0; y < N; y++) for (let x = 0; x < N; x++) {
        if (this.state(x, y) !== UNK || this.L[this.i(x, y)] !== 0) {
          if (x < x0) x0 = x; if (y < y0) y0 = y; if (x > x1) x1 = x; if (y > y1) y1 = y;
        }
      }
      return x1 < 0 ? null : { x0, y0, x1, y1 };
    }
    stats() {
      let free = 0, occ = 0;
      for (let y = 0; y < this.size; y++) for (let x = 0; x < this.size; x++) {
        const s = this.state(x, y);
        if (s === FREE) free++; else if (s === OCC) occ++;
      }
      return { free, occ, areaM2: free * this.cellCm * this.cellCm / 10000 };
    }

    toJSON() {
      return {
        v: 1, size: this.size, cellCm: this.cellCm, home: this.home,
        L: Array.from(this.L, v => Math.round(v * 10) / 10),
        visited: Array.from(this.visited),
        hits: Array.from(this.hn).map((n, k) => n ? [k, Math.round(this.hx[k] / n), Math.round(this.hy[k] / n), Math.min(n, 20)] : null).filter(Boolean)
      };
    }
    static fromJSON(o) {
      const m = new GridMap(o.size, o.cellCm);
      m.home = o.home;
      m.L.set(o.L); m.visited.set(o.visited);
      for (const [k, x, y, n] of o.hits || []) { m.hx[k] = x * n; m.hy[k] = y * n; m.hn[k] = n; }
      return m;
    }
  }

  // Split a path into straight runs: [{dir, n}] ; first run limited to maxCells
  function firstRun(path, maxCells) {
    if (!path || path.length < 2) return null;
    const dirOf = (a, b) => { for (let d = 0; d < 4; d++) if (a.x + DX[d] === b.x && a.y + DY[d] === b.y) return d; return -1; };
    const dir = dirOf(path[0], path[1]);
    let n = 1;
    while (n < maxCells && n + 1 < path.length && dirOf(path[n], path[n + 1]) === dir) n++;
    return { dir, n };
  }

  Object.assign(SP, { GridMap, firstRun, DX, DY, HEAD_DEG, UNK, FREE, OCC });
})();
