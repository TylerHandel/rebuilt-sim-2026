// Match sound: the robots (drive motors whining with speed, shooter flywheels spinning up, shots,
// bumper hits), FUEL bouncing, the field's cues (match start, TELEOP bells, the endgame whistle,
// the final buzzer) and the crowd, which cheers when a volley of FUEL goes in. Positional: what's
// near the camera is louder, and left/right follows the view. Recordings are CC0 from
// freesound.org (sounds/CREDITS.md). The browser only allows sound after a click or key press.
import * as THREE from 'three';
import { TIMING, HUB, HALF_L } from './constants.js';

const HUB_HITS = ['hub-hit-1', 'hub-hit-2', 'hub-hit-3', 'hub-hit-4'];
const SHOTS = ['shot-1', 'shot-2', 'shot-3'];
// loops that must be seamless are WAV (an MP3 pads its start and end)
const WAV = new Set(['drive']);
const FILES = [...HUB_HITS, ...SHOTS, 'drive', 'flywheel', 'firing', 'bounce', 'bump', 'match-start', 'teleop-start', 'endgame', 'match-end', 'cheer', 'cheer-big', 'crowd'];
// your own match-cue recordings (e.g. the real field sounds from your FRC Driver Station install),
// matched to a cue by file name; kept in this browser only (IndexedDB), never uploaded
export const CUES = ['match-start', 'teleop-start', 'endgame', 'match-end'];
const CUE_NAMES = { 'match-start': 'match start', 'teleop-start': 'TELEOP start', endgame: 'endgame warning', 'match-end': 'match end' };
export function cueFor(name) {
  const n = name.toLowerCase();
  if (/abort|fault|e-?stop/.test(n)) return null;
  if (/tele/.test(n)) return 'teleop-start';
  if (/end.?game|warning|whistle|30/.test(n)) return 'endgame';
  if (/end|buzzer|stop|finish/.test(n)) return 'match-end';
  if (/start|auto|charge|begin/.test(n)) return 'match-start';
  return null;
}
const DB = 'rebuiltSim.sounds';
function idb() {
  return new Promise((res, rej) => {
    const r = indexedDB.open(DB, 1);
    r.onupgradeneeded = () => r.result.createObjectStore('cues');
    r.onsuccess = () => res(r.result);
    r.onerror = () => rej(r.error);
  });
}
async function idbAll() {
  const db = await idb();
  return new Promise((res) => {
    const out = {}, tx = db.transaction('cues'), st = tx.objectStore('cues');
    st.openCursor().onsuccess = (e) => { const c = e.target.result; if (c) { out[c.key] = c.value; c.continue(); } else res(out); };
    tx.onerror = () => res(out);
  });
}
async function idbPut(entries, clear = false) {
  const db = await idb();
  return new Promise((res) => {
    const tx = db.transaction('cues', 'readwrite'), st = tx.objectStore('cues');
    if (clear) st.clear();
    for (const [k, v] of Object.entries(entries)) st.put(v, k);
    tx.oncomplete = res;
    tx.onerror = res;
  });
}

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
    this.custom = {};
    const own = idbAll().catch(() => ({}));
    for (const f of FILES) {
      fetch(`sounds/${f}.${WAV.has(f) ? 'wav' : 'mp3'}`).then((r) => r.arrayBuffer()).then((a) => this.ctx.decodeAudioData(a)).then(async (b) => {
        const mine = (await own)[f];
        if (mine) {
          try { this.buf[f] = await this.ctx.decodeAudioData(mine.slice(0)); this.custom[f] = true; return; } catch { /* fall back to ours */ }
        }
        this.buf[f] = b;
      }).catch(() => {});
    }
  }

  // files: File objects picked by the player. Returns [cue name, file name] for each one used.
  async loadCustom(files) {
    this._start();
    const got = {}, used = [];
    for (const f of files) {
      const cue = cueFor(f.name);
      if (!cue || got[cue]) continue;
      const data = await f.arrayBuffer();
      try { await this.ctx.decodeAudioData(data.slice(0)); } catch { continue; } // not audio this browser can play
      got[cue] = data;
      used.push([CUE_NAMES[cue], f.name]);
    }
    if (!used.length) return used;
    await idbPut(got);
    for (const [cue, data] of Object.entries(got)) { this.buf[cue] = await this.ctx.decodeAudioData(data.slice(0)); this.custom[cue] = true; }
    return used;
  }

  async clearCustom() {
    await idbPut({}, true);
    for (const cue of Object.keys(this.custom || {})) {
      try { this.buf[cue] = await this.ctx.decodeAudioData(await (await fetch(`sounds/${cue}.mp3`)).arrayBuffer()); } catch { /* */ }
    }
    this.custom = {};
  }

  get ready() { return !!this.ctx && this.ctx.state === 'running' && this.level > 0; }

  // a one-shot, at a world position (or flat if pos is null)
  play(name, { pos = null, gain = 1, rate = 1, near = 2.5 } = {}) {
    const b = this.buf[name];
    if (!this.ready || !b) return;
    const s = this.ctx.createBufferSource();
    s.buffer = b;
    s.playbackRate.value = rate;
    const g = this.ctx.createGain();
    g.gain.value = gain;
    s.connect(g);
    g.connect(pos ? this._panner(pos, near) : this.master);
    s.start();
  }

  _panner(pos, near = 2.5) {
    const p = this.ctx.createPanner();
    p.panningModel = 'equalpower';
    p.distanceModel = 'inverse';
    p.refDistance = near;
    p.rolloffFactor = 1;
    p.positionX.value = pos.x; p.positionY.value = pos.y ?? 0.4; p.positionZ.value = pos.z;
    p.connect(this.master);
    return p;
  }

  _loop(name, gain) {
    const s = this.ctx.createBufferSource();
    s.buffer = this.buf[name];
    s.loop = true;
    const pad = WAV.has(name) ? 0 : 0.03; // skip an MP3's encoder padding at each end
    s.loopStart = pad;
    s.loopEnd = s.buffer.duration - pad;
    const g = this.ctx.createGain();
    g.gain.value = 0;
    s.connect(g);
    s.start(0, pad + Math.random() * (s.buffer.duration - 2 * pad - 0.01));
    return { s, g, gain };
  }

  _voices(robot) {
    let v = this.units.get(robot);
    if (v || !this.buf.drive || !this.buf.flywheel || !this.buf.firing) return v;
    const pan = this._panner(robot.pos);
    v = { pan, drive: this._loop('drive'), fly: this._loop('flywheel'), fire: this._loop('firing'), shots: robot.stats.shots, lastShot: -9, vel: robot.vel.clone(), seed: 0.92 + 0.16 * Math.random() };
    v.drive.g.connect(pan);
    v.fly.g.connect(pan);
    v.fire.g.connect(pan);
    this.units.set(robot, v);
    return v;
  }

  _stopVoices() {
    for (const v of this.units.values()) {
      for (const k of ['drive', 'fly', 'fire']) { try { v[k].s.stop(); } catch { /* */ } }
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
      // swerve modules: a whine that rises with speed (and a little with turning). Recorded
      // from a real robot near top speed; slower is lower in pitch
      v.drive.s.playbackRate.setTargetAtTime((0.4 + 0.65 * load) * v.seed, now, 0.05);
      v.drive.g.gain.setTargetAtTime(on ? 0.03 + 0.5 * load : 0, now, 0.06);
      // shooter flywheel: pitch follows its speed
      const fly = Math.max(0, Math.min(1.2, (r.flywheel || 0) / (r.cfg.shooter.speedMax || 17)));
      v.fly.s.playbackRate.setTargetAtTime(0.4 + 0.9 * fly, now, 0.08);
      v.fly.g.gain.setTargetAtTime(on && fly > 0.05 ? 0.1 + 0.3 * fly : 0, now, 0.1);
      // shots: a real robot feeding its shooter (recorded) while it fires, and a pop per FUEL
      if (r.stats.shots > v.shots) {
        v.lastShot = now;
        const n = Math.min(2, r.stats.shots - v.shots);
        for (let i = 0; i < n; i++) this.play(SHOTS[Math.floor(Math.random() * SHOTS.length)], { pos: r.pos, gain: 0.6, rate: 0.92 + 0.16 * Math.random() });
      }
      v.shots = r.stats.shots;
      v.fire.g.gain.setTargetAtTime(on && now - v.lastShot < 0.35 ? 0.6 : 0, now, 0.08);
      // bumper hits: a sudden change of velocity
      const dv = Math.hypot(r.vel.x - v.vel.x, r.vel.z - v.vel.z);
      if (active && dv > 1.4 && (!v.lastHit || now - v.lastHit > 0.25)) {
        v.lastHit = now;
        this.play('bump', { pos: r.pos, gain: Math.min(1, dv / 4), rate: 0.85 + 0.3 * Math.random() });
      }
      v.vel.copy(r.vel);
    }
    // ---- FUEL hitting the HUB's polycarbonate (the deep drum-like boom of a real HUB, recorded),
    // and bouncing on the carpet, field elements and robots (the loudest few per frame)
    if (active) {
      let n = 0, nh = 0;
      const hx = Math.abs(HUB.fx - HALF_L), reach = HUB.size / 2 + 0.35;
      for (const b of game.fuel.balls) {
        if (b.state !== 'field' || !b.body) continue;
        const v = b.body.linvel(), prev = this.balls.get(b);
        if (prev) {
          const dv = Math.hypot(v.x - prev.x, v.y - prev.y, v.z - prev.z);
          const atHub = !b.inHub && b.pos.y > 1.3 && Math.abs(Math.abs(b.pos.x) - hx) < reach && Math.abs(b.pos.z) < reach;
          if (atHub && dv > 2.2 && nh < 3 && !(now - (prev.hit || -9) < 0.4)) {
            nh++;
            prev.hit = now;
            this.play(HUB_HITS[Math.floor(Math.random() * HUB_HITS.length)], { pos: b.pos, gain: Math.min(1.6, 0.8 + dv / 6), rate: 0.92 + 0.16 * Math.random(), near: 6 });
          } else if (!atHub && prev.y < -2.2 && v.y > prev.y * 0.2 && n < 2) {
            n++;
            this.play('bounce', { pos: b.pos, gain: Math.min(0.6, -prev.y / 10), rate: 0.85 + 0.35 * Math.random() });
          }
          prev.x = v.x; prev.y = v.y; prev.z = v.z;
        } else this.balls.set(b, { x: v.x, y: v.y, z: v.z });
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
