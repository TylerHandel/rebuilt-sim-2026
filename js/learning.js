// Learning in the browser, saved in localStorage:
//  - DrivingRecorder watches the human driver and measures the same decisions the AI brain
//    makes (where you shoot from, how full you get before a cycle, how you pin and shove...).
//  - Learner holds "Your trained AI" brain. In Training mode each match plays a slightly
//    different version of it against you (mirrored pairs of random nudges) and keeps whatever
//    did better; when you do better than the AI it also copies part of your driving style.
//    Role "score": the AI wants to win the match. Role "defense": you defend and the AI wants to
//    score as many points as it can through your defense.
import { BRAIN_SPEC, DEFAULT_BRAIN } from './opponent.js';
import { TRAINED_BRAIN } from './trainedBrain.js';
import { BLUE, HALF_L } from './constants.js';
import { Field } from './field.js';
import { brainFileText, parseBrainText } from './brainFile.js';

const KEY = 'rebuiltSim.learner';
export const KEYS = Object.keys(BRAIN_SPEC);
const DEFENSE_KEYS = ['pinLimit', 'pushSpeed', 'blockLead', 'engage'];
// samples needed before a measurement of your driving is trusted
const MIN_SAMPLES = { spotFx: 3, spotZ: 3, fill: 2, cycleTime: 2, stageMargin: 1, collectSpeed: 20, intakeDist: 4, pinLimit: 2, pushSpeed: 2, blockLead: 12, engage: 2, shuttle: 2, shuttleKeep: 2 };
export const EXPLORE_LEVELS = [[0, 'Off'], [0.04, 'Small'], [0.08, 'Medium'], [0.14, 'Large']];
export const IMITATE_LEVELS = [[0, 'Off'], [0.15, 'A little'], [0.3, 'Some'], [0.5, 'A lot']];

const clamp01 = (v) => Math.min(1, Math.max(0, v));
export const toNorm = (k, v) => clamp01((v - BRAIN_SPEC[k].min) / (BRAIN_SPEC[k].max - BRAIN_SPEC[k].min));
export const fromNorm = (k, x) => +(BRAIN_SPEC[k].min + clamp01(x) * (BRAIN_SPEC[k].max - BRAIN_SPEC[k].min)).toFixed(3);
const median = (a) => { const s = [...a].sort((p, q) => p - q); const n = s.length; return n ? (n % 2 ? s[(n - 1) / 2] : (s[n / 2 - 1] + s[n / 2]) / 2) : null; };
const gauss = () => { let u = 0; while (!u) u = Math.random(); return Math.sqrt(-2 * Math.log(u)) * Math.cos(2 * Math.PI * Math.random()); };

function fresh() {
  return {
    brain: { ...DEFAULT_BRAIN, ...TRAINED_BRAIN },
    explore: 0.08,
    imitate: 0.3,
    matches: 0,
    record: { score: { w: 0, l: 0, t: 0 }, defense: { n: 0, sumAi: 0, best: null } },
    pairs: { score: null, defense: null },
    demo: {},
    history: [],
  };
}

export class Learner {
  constructor() { this.load(); }

  load() {
    let s = null;
    try { s = JSON.parse(localStorage.getItem(KEY) || 'null'); } catch { /* storage unavailable */ }
    this.s = { ...fresh(), ...(s || {}) };
    this.s.brain = { ...DEFAULT_BRAIN, ...this.s.brain };
  }

  save() {
    try { localStorage.setItem(KEY, JSON.stringify(this.s)); } catch { /* storage unavailable */ }
  }

  get brain() { return { ...this.s.brain }; }

  set(k, v) {
    const sp = BRAIN_SPEC[k];
    this.s.brain[k] = +Math.min(sp.max, Math.max(sp.min, v)).toFixed(3);
    this.s.pairs = { score: null, defense: null }; // a manual edit restarts the exploration pairs
    this.save();
  }

  setBrain(b) {
    this.s.brain = { ...DEFAULT_BRAIN, ...b };
    this.s.pairs = { score: null, defense: null };
    this.save();
  }

  // average AI points in your defense drills (the number to beat)
  defenseAverage() {
    const d = this.s.record.defense;
    return d.n ? Math.round(d.sumAi / d.n) : null;
  }

  // ------------------------------------------------------------------ exploration
  // The brain the AI plays in the next Training match: the current brain nudged by +noise,
  // then by -noise in the following match (a mirrored pair), for the given role.
  candidate(role) {
    const sigma = this.s.explore;
    const brain = this.brain;
    if (!sigma) return { brain, info: { role, sign: 0 } };
    let P = this.s.pairs[role];
    if (!P || P.phase === 0) {
      P = { eps: KEYS.map(() => gauss()), phase: 1, fPlus: null };
      this.s.pairs[role] = P;
      this.save();
    }
    const sign = P.phase === 1 ? 1 : -1;
    const out = {};
    KEYS.forEach((k, i) => { out[k] = fromNorm(k, toNorm(k, brain[k]) + sign * sigma * P.eps[i]); });
    return { brain: out, info: { role, sign } };
  }

  // ------------------------------------------------------------------ learning after a match
  // res: { role, info, aiTotal, humanTotal, obs (DrivingRecorder.finish() or null) }
  learn(res) {
    const S = this.s;
    const { role, info, aiTotal, humanTotal, obs } = res;
    const before = this.brain;
    const lines = [];
    const fitness = role === 'defense' ? aiTotal : aiTotal - humanTotal;
    // 1) exploration: compare the two halves of a mirrored pair and step toward the better one
    if (info.sign !== 0) {
      const P = S.pairs[role];
      if (P && info.sign > 0 && P.phase === 1) {
        P.fPlus = fitness;
        P.phase = 2;
        lines.push(`The AI tried variation A (${role === 'defense' ? `${aiTotal} points through your defense` : aiResult(fitness)}). Next match it tries the mirror image, B, then keeps whichever worked better.`);
      } else if (P && info.sign < 0 && P.phase === 2) {
        const scale = role === 'defense' ? 60 : 120;
        const g = Math.max(-1, Math.min(1, (P.fPlus - fitness) / scale));
        KEYS.forEach((k, i) => { S.brain[k] = fromNorm(k, toNorm(k, S.brain[k]) + S.explore * g * P.eps[i]); });
        P.phase = 0;
        const better = Math.abs(P.fPlus - fitness) < 5 ? 'about the same' : g > 0 ? 'A did better' : 'B did better';
        lines.push(`Variation B: ${role === 'defense' ? `${aiTotal} points` : aiResult(fitness)}; variation A: ${role === 'defense' ? `${P.fPlus} points` : aiResult(P.fPlus)} — ${better}, so the AI moved ${Math.round(Math.abs(g) * 100)}% of a step that way.`);
      }
    }
    // 2) imitation: copy part of your style when it worked better than the AI's
    let strength = 0;
    if (role === 'defense') {
      const avg = this.defenseAverage();
      if (avg !== null && aiTotal < avg) strength = Math.min(1, (avg - aiTotal) / 100);
    } else if (humanTotal > aiTotal) strength = Math.min(1, Math.max(0.2, (humanTotal - aiTotal) / 150));
    if (obs && strength > 0 && S.imitate > 0) {
      const copied = [];
      for (const k of role === 'defense' ? DEFENSE_KEYS : KEYS) {
        const a = obs[k];
        if (!a || a.length < (MIN_SAMPLES[k] ?? 99)) continue;
        const w = S.imitate * strength;
        S.brain[k] = fromNorm(k, toNorm(k, S.brain[k]) + w * (toNorm(k, median(a)) - toNorm(k, S.brain[k])));
        copied.push(BRAIN_SPEC[k].label.toLowerCase());
      }
      if (copied.length) lines.push(`You ${role === 'defense' ? `held it under its average of ${this.defenseAverage()}` : `won by ${humanTotal - aiTotal}`}, so it copied ${Math.round(S.imitate * strength * 100)}% of your ${copied.join(', ')}.`);
    }
    // records and history
    S.matches++;
    if (role === 'defense') {
      const d = S.record.defense;
      d.n++; d.sumAi += aiTotal;
      d.best = d.best === null ? aiTotal : Math.min(d.best, aiTotal);
    } else {
      const r = S.record.score;
      if (humanTotal > aiTotal) r.w++; else if (humanTotal < aiTotal) r.l++; else r.t++;
    }
    const changes = KEYS
      .map((k) => ({ k, from: before[k], to: S.brain[k], d: Math.abs(toNorm(k, S.brain[k]) - toNorm(k, before[k])) }))
      .filter((c) => c.d > 0.004)
      .sort((p, q) => q.d - p.d);
    S.history.push({ date: new Date().toISOString(), role, aiTotal, humanTotal, sign: info.sign, changed: changes.length });
    if (S.history.length > 100) S.history.shift();
    this.save();
    return { lines, changes };
  }

  // ------------------------------------------------------------------ your driving profile
  addDemo(obs) {
    if (!obs) return;
    for (const [k, a] of Object.entries(obs)) {
      if (!a.length) continue;
      const list = (this.s.demo[k] ||= []);
      list.push(...a.map((v) => +v.toFixed(3)));
      if (list.length > 300) list.splice(0, list.length - 300);
    }
    this.save();
  }

  demoEstimate(k) {
    const a = this.s.demo[k];
    if (!a || a.length < (MIN_SAMPLES[k] ?? 99)) return a && a.length ? { value: null, n: a.length } : null;
    const sp = BRAIN_SPEC[k];
    return { value: Math.min(sp.max, Math.max(sp.min, median(a))), n: a.length };
  }

  // blend the brain toward your measured driving (weight 0..1); returns the keys changed
  applyDemo(weight) {
    const changed = [];
    for (const k of KEYS) {
      const e = this.demoEstimate(k);
      if (!e || e.value === null) continue;
      this.s.brain[k] = fromNorm(k, toNorm(k, this.s.brain[k]) + weight * (toNorm(k, e.value) - toNorm(k, this.s.brain[k])));
      changed.push(k);
    }
    this.s.pairs = { score: null, defense: null };
    this.save();
    return changed;
  }

  clearDemo() { this.s.demo = {}; this.save(); }

  resetStats() {
    const f = fresh();
    Object.assign(this.s, { matches: 0, record: f.record, pairs: f.pairs, history: [] });
    this.save();
  }

  // ------------------------------------------------------------------ export / import
  info() {
    const S = this.s;
    return {
      source: 'in-game AI Tuning / Training mode',
      date: new Date().toISOString().slice(0, 10),
      trainingMatches: S.matches,
      recordVsYou: S.record.score,
      defenseDrills: { matches: S.record.defense.n, aiAverage: this.defenseAverage(), aiBest: S.record.defense.best },
    };
  }

  fileText() { return brainFileText(this.brain, this.info()); }
  jsonText() { return JSON.stringify({ brain: this.brain, info: this.info() }, null, 2); }

  importText(text) {
    const b = parseBrainText(text);
    if (!b) return false;
    this.setBrain({ ...this.brain, ...b });
    return true;
  }
}

function aiResult(margin) { return margin === 0 ? 'tied' : `AI ${margin > 0 ? 'won' : 'lost'} by ${Math.abs(margin)}`; }

// ------------------------------------------------------------------ recording your driving
// Measured during TELEOP in matches you drive. Each entry is a list of samples in the same
// units as the brain parameter it teaches.
export class DrivingRecorder {
  constructor(game) {
    this.g = game;
    this.r = game.robot;
    this.obs = Object.fromEntries(Object.keys(MIN_SAMPLES).map((k) => [k, []]));
    this.lastShots = 0;
    this.inBurst = false;
    this.lastShotT = -99;
    this.lastBurstEnd = null;
    this.wasInZone = true;
    this.sampleT = 0;
    this.prevIntake = false;
    this.pinMax = 0;
    this.wasContact = false;
    this.wasActive = null;
    this.offPasses = 0;
    this.lastPasses = 0;
    this.passBurstT = -99;
  }

  step(dt) {
    const g = this.g, m = g.match, r = this.r;
    if (!m.isTeleop) { this.lastShots = r.stats.shots; return; }
    const t = m.t, own = r.alliance, O = this.obs;
    const wallX = own === BLUE ? -HALF_L : HALF_L;
    const maxSp = r.cfg.drive.maxSpeed;
    const speed = Math.hypot(r.vel.x, r.vel.z);
    this.sampleT += dt;
    const tick = this.sampleT >= 0.25;
    if (tick) this.sampleT = 0;

    // ---- shooting: where from, how full, how long you collected
    const fired = r.stats.shots - this.lastShots;
    this.lastShots = r.stats.shots;
    if (fired > 0 && r.shot && r.shot.mode === 'hub') {
      if (!this.inBurst) {
        this.inBurst = true;
        O.spotFx.push(Math.abs(r.pos.x - wallX));
        O.spotZ.push(Math.abs(r.pos.z));
        const cap = r.capacity();
        O.fill.push((r.stored.length + fired) / (cap < 20 ? cap : Math.min(cap, 60)));
        if (this.lastBurstEnd !== null && m.hubActive(own)) O.cycleTime.push(t - this.lastBurstEnd);
      }
      this.lastShotT = t;
    }
    if (this.inBurst && t - this.lastShotT > 1.0) { this.inBurst = false; this.lastBurstEnd = t; }

    // ---- shuttling: do you pass FUEL into your zone during your off shifts, and how much do
    // you keep in the hopper when you do?
    const act = m.hubActive(own);
    const passed = r.stats.passes - this.lastPasses;
    this.lastPasses = r.stats.passes;
    if (!act && passed > 0) { this.offPasses += passed; this.passBurstT = t; }
    if (!act && this.passBurstT > 0 && t - this.passBurstT > 1.0) {
      O.shuttleKeep.push(r.stored.length / Math.max(1, r.capacity()));
      this.passBurstT = -99;
    }
    if (this.wasActive === false && act && m.shiftIndex() >= 1) { O.shuttle.push(this.offPasses >= 3 ? 1 : 0); this.offPasses = 0; }
    this.wasActive = act;

    // ---- staging: how early you got back while your HUB was about to turn active
    const inZone = r.lastInZone;
    if (inZone && !this.wasInZone && r.stored.length >= 5 && !m.hubActive(own)) {
      const nc = m.hubNextChange(own);
      if (nc !== null && nc < 12) O.stageMargin.push(nc);
    }
    this.wasInZone = inZone;

    // ---- collecting: speed with the intake running, and how far away you drop the intake
    if (tick && r.intakeSpeed > 0 && !inZone) O.collectSpeed.push(speed / maxSp);
    if (r.cmd.intake && !this.prevIntake) {
      const f = r.forward();
      let best = Infinity;
      for (const b of g.fuel.balls) {
        if (b.state !== 'field' || b.pos.y > 0.3) continue;
        const dx = b.pos.x - r.pos.x, dz = b.pos.z - r.pos.z, d = Math.hypot(dx, dz);
        if (d < best && d > 0.1 && (dx * f.x + dz * f.z) / d > 0.5) best = d;
      }
      if (best < 6) O.intakeDist.push(best);
    }
    this.prevIntake = r.cmd.intake;

    // ---- defense against the opponent robot
    const A = g.opp && g.opp.robot;
    if (A) {
      const pin = g.rules.pinState(r, A);
      if (pin.t > this.pinMax) this.pinMax = pin.t;
      else if (pin.t === 0 && this.pinMax > 0.5) { O.pinLimit.push(this.pinMax); this.pinMax = 0; }
      const contact = g.rules.inContact(r, A);
      if (contact && !this.wasContact) {
        let ux = A.pos.x - r.pos.x, uz = A.pos.z - r.pos.z;
        const d = Math.hypot(ux, uz) || 1;
        ux /= d; uz /= d;
        const mine = r.vel.x * ux + r.vel.z * uz, theirs = -(A.vel.x * ux + A.vel.z * uz);
        if (mine > 0.3 && mine >= theirs) {
          O.pushSpeed.push(mine / maxSp);
          const lineX = Field.allianceLineX(A.alliance);
          O.engage.push(A.alliance === BLUE ? A.pos.x - lineX : lineX - A.pos.x);
        }
      }
      this.wasContact = contact;
      // blocking: how far ahead of the other robot (toward its HUB) you stand
      if (tick && !contact && !A.lastInZone) {
        const h = Field.hubCenter(A.alliance);
        let hx = h.x - A.pos.x, hz = h.z - A.pos.z;
        const hd = Math.hypot(hx, hz) || 1;
        hx /= hd; hz /= hd;
        const px = r.pos.x - A.pos.x, pz = r.pos.z - A.pos.z;
        const lead = px * hx + pz * hz, lateral = Math.abs(px * hz - pz * hx);
        if (lead > 0.3 && lead < 3 && lateral < 1.0) O.blockLead.push(lead);
      }
    }
  }

  finish() {
    if (this.pinMax > 0.5) this.obs.pinLimit.push(this.pinMax);
    this.pinMax = 0;
    // keep samples inside each parameter's range
    for (const [k, a] of Object.entries(this.obs)) {
      const sp = BRAIN_SPEC[k];
      for (let i = 0; i < a.length; i++) a[i] = Math.min(sp.max, Math.max(sp.min, a[i]));
    }
    return this.obs;
  }
}
