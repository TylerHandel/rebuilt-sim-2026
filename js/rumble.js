// Controller rumble (the Gamepad API's dual-rumble, where the browser and pad support it): what
// your robot feels, only at events, never just for driving. Hitting something (another robot, a
// wall, a field element) jolts both motors, harder the bigger the sudden change of velocity; so
// does landing after being in the air (off a BUMP), harder the faster it came down; each shot a
// short buzz of the light motor; each FUEL taken in a faint tick. Off in the menus and when you
// aren't driving.
export class Rumble {
  constructor() {
    this.on = true;
    this.v = null; this.shots = 0; this.stored = 0; this.until = 0; this.level = 0;
  }

  _pad() {
    const pads = navigator.getGamepads ? navigator.getGamepads() : [];
    for (const p of pads) if (p && p.connected && p.vibrationActuator) return p;
    return null;
  }

  _play(pad, ms, strong, weak) {
    try { pad.vibrationActuator.playEffect('dual-rumble', { duration: ms, strongMagnitude: Math.min(1, strong), weakMagnitude: Math.min(1, weak) }); } catch { /* not supported */ }
  }

  // r: your robot (or null), active: a match on screen and you driving
  update(r, active) {
    if (!this.on || !r || !active) { this.v = null; return; }
    const now = performance.now();
    const n = r.stored ? r.stored.length : 0, shots = r.stats ? r.stats.shots : 0;
    if (!this.v) { this.v = { x: r.vel.x, z: r.vel.z, y: r.pos.y, vy: 0, t: now }; this.shots = shots; this.stored = n; return; }
    const v = this.v, dt = (now - v.t) / 1000;
    const dv = Math.hypot(r.vel.x - v.x, r.vel.z - v.z);
    v.x = r.vel.x; v.z = r.vel.z;
    // (falling: its height dropping; landing: the fall stopping at once. Measured when the height
    // has changed, or after 0.1 s: on the physics thread it changes only when a new state arrives)
    let land = 0;
    if (r.pos.y !== v.y || dt > 0.1) {
      const vy = (r.pos.y - v.y) / Math.max(1e-3, dt);
      if (v.vy < -0.6 && vy > v.vy * 0.3) land = -v.vy;
      v.y = r.pos.y; v.vy = vy; v.t = now;
    }
    const pad = this._pad();
    if (!pad) { this.shots = shots; this.stored = n; return; }
    // (one effect at a time: a stronger one replaces a weaker one still playing)
    let ms = 0, s = 0, w = 0;
    const hit = Math.max(dv > 1.2 ? (dv - 1.2) / 3 : -1, land > 0 ? (land - 0.6) / 2 : -1);
    if (hit >= 0) { ms = 90 + 70 * Math.min(1, hit); s = 0.25 + 0.75 * Math.min(1, hit); w = s; }
    else if (shots > this.shots) { ms = 45; s = 0.05; w = 0.45; }
    else if (n > this.stored) { ms = 30; s = 0; w = 0.22; }
    this.shots = shots; this.stored = n;
    if (ms && (now >= this.until || s >= this.level)) {
      this._play(pad, ms, s, w);
      this.until = now + ms; this.level = s;
    }
  }
}
