// Controller rumble (the Gamepad API's dual-rumble, where the browser and pad support it): what
// your robot feels. A bumper hit or a knock jolts both motors, harder the bigger the sudden change
// of velocity; each shot a short buzz of the light motor; each FUEL taken in a faint tick; driving
// a soft hum of the heavy motor with the drive's speed. Off in the menus and when you aren't driving.
export class Rumble {
  constructor() {
    this.on = true;
    this.v = null; this.shots = 0; this.stored = 0; this.until = 0; this.hum = 0;
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
    const pad = this._pad();
    const now = performance.now();
    const n = r.stored ? r.stored.length : 0, shots = r.stats ? r.stats.shots : 0;
    if (!this.v) { this.v = { x: r.vel.x, z: r.vel.z }; this.shots = shots; this.stored = n; return; }
    const dv = Math.hypot(r.vel.x - this.v.x, r.vel.z - this.v.z);
    this.v.x = r.vel.x; this.v.z = r.vel.z;
    if (!pad) { this.shots = shots; this.stored = n; return; }
    // (one effect at a time: a stronger one replaces a weaker that's still playing)
    let ms = 0, s = 0, w = 0;
    if (dv > 1.2) { ms = 90 + 60 * Math.min(1, dv / 4); s = 0.25 + 0.75 * Math.min(1, (dv - 1.2) / 3); w = s; }
    else if (shots > this.shots) { ms = 45; s = 0.05; w = 0.45; }
    else if (n > this.stored) { ms = 30; s = 0; w = 0.22; }
    this.shots = shots; this.stored = n;
    if (ms && (now >= this.until || s >= this.level)) {
      this._play(pad, ms, s, w);
      this.until = now + ms; this.level = s;
      return;
    }
    // driving: a soft hum with speed, renewed while nothing else plays
    const sp = Math.hypot(r.vel.x, r.vel.z);
    if (now >= this.until && sp > 0.5) {
      const h = 0.04 + 0.08 * Math.min(1, sp / 4.5);
      this._play(pad, 120, h, 0);
      this.until = now + 100; this.level = h;
    }
  }
}
