// Match sound: the robots (drive motors whining with speed, shooter flywheels spinning up, shots,
// bumper hits), FUEL bouncing, the field's cues (match start, TELEOP bells, the endgame whistle,
// the final buzzer) and the crowd, which cheers when a volley of FUEL goes in. Positional: what's
// near the camera is louder, and left/right follows the view. Recordings are CC0 from
// freesound.org (sounds/CREDITS.md). The browser only allows sound after a click or key press.
import * as THREE from 'three';
import { TIMING } from './constants.js';

const FILES = ['drive', 'flywheel', 'shot', 'bounce', 'bump', 'match-start', 'teleop-start', 'endgame', 'match-end', 'cheer', 'cheer-big', 'crowd'];
export const SOUND_LEVELS = { off: 0, low: 0.35, medium: 0.7, high: 1 };

export class Sound {
  constructor() {
    this.ctx = null;
    this.buf = {};
    this.level = SOUND_LEVELS.medium;
    this.units = new Map();   // robot -> its looping voices
    this.balls = new Map();   // FUEL -> last vertical speed
    this.game = null;
    this.recent = [];         // [time, alliance, points] for volley detection
    this.lastCheer = -99;
    this.bouncesThisFrame = 0;
    const start = () => this._start();
    for (const ev of ['pointerdown', 'keydown', 'touchstart']) window.addEventListener(ev, start, { once: false, passive: true });
    this._poke = start;
  }

  setLevel(name) {
    this.level = SOUND_LEVELS[name] ?? SOUND_LEVELS.medium;
    if (this.master) this.master.gain.setTargetAtTime(this.level, this.ctx.currentTime, 0.05);
  }

  // a gamepad press counts as a gesture too (main.js calls this when one is pressed)
  gesture() { this._start(); }

  _start() {
    if (this.ctx) { if (this.ctx.state === 'suspended') this.ctx.resume(); return; }
    const AC = window.AudioContext || window.webkitAudioContext;
    if (!AC) return;
    this.ctx = new AC();
    this.master = this.ctx.createGain();
    this.master.gain.value = this.level;
    this.master.connect(this.ctx.destination);
    for (const f of FILES) {
      fetch(`sounds/${f}.mp3`).then((r) => r.arrayBuffer()).then((a) => this.ctx.decodeAudioData(a)).then((b) => { this.buf[f] = b; }).catch(() => {});
    }
  }

  get ready() { return !!this.ctx && this.ctx.state === 'running' && this.level > 0; }

  // a one-shot, at a world position (or flat if pos is null)
  play(name, { pos = null, gain = 1, rate = 1 } = {}) {
    const b = this.buf[name];
    if (!this.ready || !b) return;
    const s = this.ctx.createBufferSource();
    s.buffer = b;
    s.playbackRate.value = rate;
    const g = this.ctx.createGain();
    g.gain.value = gain;
    s.connect(g);
    g.connect(pos ? this._panner(pos, g) : this.master);
    s.start();
  }

  _panner(pos, from) {
    const p = this.ctx.createPanner();
    p.panningModel = 'equalpower';
    p.distanceModel = 'inverse';
    p.refDistance = 2.5;
    p.rolloffFactor = 1;
    p.positionX.value = pos.x; p.positionY.value = pos.y ?? 0.4; p.positionZ.value = pos.z;
    p.connect(this.master);
    return p;
  }

  _loop(name, gain) {
    const s = this.ctx.createBufferSource();
    s.buffer = this.buf[name];
    s.loop = true;
    s.loopStart = 0.03; // skip the MP3 encoder's padding at each end
    s.loopEnd = s.buffer.duration - 0.03;
    const g = this.ctx.createGain();
    g.gain.value = 0;
    s.connect(g);
    s.start(0, 0.03 + Math.random() * (s.buffer.duration - 0.1));
    return { s, g, gain };
  }

  _voices(robot) {
    let v = this.units.get(robot);
    if (v || !this.buf.drive || !this.buf.flywheel) return v;
    const pan = this._panner(robot.pos, null);
    v = { pan, drive: this._loop('drive'), fly: this._loop('flywheel'), shots: robot.stats.shots, vel: robot.vel.clone(), seed: 0.9 + 0.2 * Math.random() };
    v.drive.g.connect(pan);
    v.fly.g.connect(pan);
    this.units.set(robot, v);
    return v;
  }

  _stopVoices() {
    for (const v of this.units.values()) {
      for (const k of ['drive', 'fly']) { try { v[k].s.stop(); } catch { /* */ } }
      v.pan.disconnect();
    }
    this.units.clear();
  }

  _crowd(on) {
    if (!this.buf.crowd) return;
    if (!this.crowdV) {
      this.crowdV = this._loop('crowd');
      this.crowdV.g.connect(this.master);
    }
    this.crowdV.g.gain.setTargetAtTime(on ? 0.22 : 0.08, this.ctx.currentTime, 0.8);
  }

  // every frame. active: a match is on screen and running (not paused)
  update(game, camera, active, dt) {
    if (!this.ctx) return;
    const now = this.ctx.currentTime;
    if (game !== this.game) { this._stopVoices(); this.balls.clear(); this.game = game; this.prev = null; this.recent = []; this.lastCheer = -99; }
    if (!this.ready) return;
    this._crowd(!!game && active);
    // the listener is the camera
    const L = this.ctx.listener, f = new THREE.Vector3(0, 0, -1).applyQuaternion(camera.quaternion), u = new THREE.Vector3(0, 1, 0).applyQuaternion(camera.quaternion);
    if (L.positionX) {
      L.positionX.value = camera.position.x; L.positionY.value = camera.position.y; L.positionZ.value = camera.position.z;
      L.forwardX.value = f.x; L.forwardY.value = f.y; L.forwardZ.value = f.z; L.upX.value = u.x; L.upY.value = u.y; L.upZ.value = u.z;
    } else {
      L.setPosition(camera.position.x, camera.position.y, camera.position.z);
      L.setOrientation(f.x, f.y, f.z, u.x, u.y, u.z);
    }
    if (!game) return;
    const m = game.match;
    // ---- robots
    for (const r of game.robots) {
      const v = this._voices(r);
      if (!v) continue;
      v.pan.positionX.value = r.pos.x; v.pan.positionY.value = 0.3; v.pan.positionZ.value = r.pos.z;
      const d = r.cfg.drive, sp = Math.hypot(r.vel.x, r.vel.z) / d.maxSpeed, w = Math.abs(r.omega || 0) / d.maxOmega;
      const load = Math.min(1, sp + 0.5 * w);
      const on = active && r.enabled;
      // swerve modules: a whine that rises with speed (and a little with turning)
      v.drive.s.playbackRate.setTargetAtTime((0.55 + 0.55 * load) * v.seed, now, 0.05);
      v.drive.g.gain.setTargetAtTime(on ? 0.04 + 0.4 * load : 0, now, 0.06);
      // shooter flywheel: pitch follows its speed
      const fly = Math.max(0, Math.min(1.2, (r.flywheel || 0) / (r.cfg.shooter.speedMax || 17)));
      v.fly.s.playbackRate.setTargetAtTime(0.4 + 0.9 * fly, now, 0.08);
      v.fly.g.gain.setTargetAtTime(on && fly > 0.05 ? 0.1 + 0.3 * fly : 0, now, 0.1);
      // shots
      if (r.stats.shots > v.shots) {
        const n = Math.min(3, r.stats.shots - v.shots);
        for (let i = 0; i < n; i++) this.play('shot', { pos: r.pos, gain: 0.55, rate: 0.9 + 0.2 * Math.random() });
      }
      v.shots = r.stats.shots;
      // bumper hits: a sudden change of velocity
      const dv = Math.hypot(r.vel.x - v.vel.x, r.vel.z - v.vel.z);
      if (active && dv > 1.4 && (!v.lastHit || now - v.lastHit > 0.25)) {
        v.lastHit = now;
        this.play('bump', { pos: r.pos, gain: Math.min(1, dv / 4), rate: 0.85 + 0.3 * Math.random() });
      }
      v.vel.copy(r.vel);
    }
    // ---- FUEL bouncing on the carpet, field elements and robots (the loudest few per frame)
    if (active) {
      let n = 0;
      for (const b of game.fuel.balls) {
        if (b.state !== 'field' || !b.body) continue;
        const vy = b.body.linvel().y, prev = this.balls.get(b) ?? 0;
        if (prev < -2.2 && vy > prev * 0.2 && n < 2) {
          n++;
          this.play('bounce', { pos: b.pos, gain: Math.min(0.6, -prev / 10), rate: 0.85 + 0.35 * Math.random() });
        }
        this.balls.set(b, vy);
      }
    }
    // ---- the field's cues
    const ph = m.phase, prev = this.prev;
    if (prev && ph !== prev.phase) {
      if (ph === 'auto') this.play('match-start', { gain: 0.8 });
      else if (ph === 'teleop') this.play('teleop-start', { gain: 0.7 });
      else if (ph === 'post') this.play('match-end', { gain: 0.8 });
    }
    if (prev && ph === 'teleop' && prev.phase === 'teleop' && prev.left > TIMING.endgame && TIMING.teleop - m.phaseTime <= TIMING.endgame) this.play('endgame', { gain: 0.7 });
    // ---- the crowd cheers a volley: lots of FUEL in a HUB in a short time
    const tot = { blue: m.total('blue'), red: m.total('red') }, gt = m.t; // match time: a pause doesn't count
    if (prev) {
      for (const a of ['blue', 'red']) {
        const d = tot[a] - prev.tot[a];
        if (d > 0) this.recent.push([gt, d]);
      }
    }
    this.recent = this.recent.filter(([t]) => gt - t < 2);
    const burst = this.recent.reduce((s, [, d]) => s + d, 0);
    if (burst >= 8 && gt - this.lastCheer > 9) {
      this.lastCheer = gt;
      this.recent = [];
      this.play(burst >= 16 ? 'cheer-big' : 'cheer', { gain: 0.55 + Math.min(0.35, burst / 50) });
    }
    this.prev = { phase: ph, left: m.phase === 'teleop' ? TIMING.teleop - m.phaseTime : 999, tot };
    void dt;
  }
}
