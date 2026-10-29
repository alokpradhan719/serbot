/* SafePath – connection to the ESP32 robot over Wi-Fi */
(function () {
  'use strict';
  const SP = (window.SP = window.SP || {});
  const sleep = ms => new Promise(r => setTimeout(r, ms));

  class StopError extends Error { constructor() { super('Stopped'); this.name = 'StopError'; } }

  /* ------------------------------------------------------------------ */
  /*  Real robot: talks to safepath_auto.ino                             */
  /* ------------------------------------------------------------------ */
  class RealRobot {
    constructor(ip) {
      this.kind = 'real';
      this.base = 'http://' + ip.replace(/^https?:\/\//, '').replace(/\/$/, '');
      this.id = Math.floor(Math.random() * 100000) + 1;
      this.onStatus = null;
      this.inFlight = 0;
    }
    async get(path, timeout = 2500) {
      const ctl = new AbortController();
      const t = setTimeout(() => ctl.abort(), timeout);
      this.inFlight++;
      try {
        const r = await fetch(this.base + path, { cache: 'no-store', signal: ctl.signal });
        if (!r.ok) throw new Error('HTTP ' + r.status);
        return await r.json();
      } finally { clearTimeout(t); this.inFlight--; }
    }
    async tryGet(path, tries = 6) {               // Wi-Fi drops for a moment now and then: keep retrying
      let err;
      for (let i = 0; i < tries; i++) {
        try { return await this.get(path); } catch (e) { err = e; await sleep(300 + i * 200); }
      }
      throw new Error('Robot not responding (' + (err && err.message) + ')');
    }
    ping() { return this.tryGet('/api/ping', 3); }
    async status() {
      const s = await this.tryGet('/api/status', 6);
      if (this.onStatus) this.onStatus(s);
      return s;
    }
    stop() { return this.tryGet('/api/stop', 3).catch(() => null); }

    async cmd(path, expectMs) {
      const id = ++this.id;
      const url = path + (path.includes('?') ? '&' : '?') + 'id=' + id;
      for (let k = 0; ; k++) {                      // wait if robot is still busy
        const r = await this.tryGet(url, 3);
        if (!r.err) break;
        if (k > 25) throw new Error('Robot stayed busy');
        await sleep(200);
      }
      const deadline = Date.now() + expectMs + 6000;
      for (;;) {
        await sleep(160);                            // also keeps the robot's watchdog happy
        const s = await this.status();
        if (s.id === id && s.state === 'idle') return s;
        if (Date.now() > deadline) { await this.stop(); throw new Error('Robot did not finish the command in time'); }
      }
    }
    drive(dir, ms, spd, safe) { return this.cmd(`/api/drive?dir=${dir}&ms=${ms}&spd=${spd}&safe=${safe}`, ms + 500); }
    turn(dir, ms, spd)        { return this.cmd(`/api/turn?dir=${dir}&ms=${ms}&spd=${spd}`, ms); }
    scan(angles)              { return this.cmd(`/api/scan?a=${angles.join(',')}`, angles.length * 900 + 2000); }
  }

  Object.assign(SP, { RealRobot, StopError, sleep });
})();
