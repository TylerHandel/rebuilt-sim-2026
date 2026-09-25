// Robot AI, used for the PvE opponent, for watch mode and for self-play training. In AUTO the
// robot runs a normal auto routine; in TELEOP it drives with one of these strategies:
//   scorer  - collects FUEL, stages in its ALLIANCE ZONE and shoots while its HUB is active
//   defense - guards the lane into the other robot's ALLIANCE ZONE and rams it back, shoves it
//             if it gets in, and backs off before a PIN turns into a foul
//   hybrid  - plays defense while only the other HUB is active, scores the rest of the time
// How well it drives comes from two places:
//   skill  - handicaps (speed, accuracy, reaction time...) that set the difficulty
//   brain  - strategy choices (when to cycle, where to shoot, how to defend...). These are the
//            numbers tools/train.mjs tunes by self-play; the result is js/trainedBrain.js.
import { BLUE, HALF_L, HALF_W, TIMING, other } from './constants.js';
import { Field } from './field.js';
import { NavGrid, obstacleAt } from './nav.js';
import { towerProtected, PIN_RESET } from './rules.js';
import { clamp, wrapAngle, rand } from './util.js';
import { TRAINED_BRAIN } from './trainedBrain.js';

export const OPP_STRATEGIES = {
  off: { name: 'Off (solo)', desc: 'No opponent: just you, the FUEL and the clock.' },
  scorer: { name: 'Scorer', desc: 'Runs its own cycles: collects FUEL, stages when its HUB is inactive and shoots on the move when it is active.' },
  defense: { name: 'Defense', desc: 'Blocks your path to your HUB and pushes you while you shoot. Backs off before a 3 s PIN (G418).' },
  hybrid: { name: 'Hybrid', desc: 'Shift-aware: plays defense while only your HUB is active and scores whenever its own HUB is active.' },
};
export const OPP_ORDER = ['off', 'scorer', 'defense', 'hybrid'];

// Trainable strategy parameters: range searched by the trainer and the hand-tuned default
export const BRAIN_SPEC = {
  fill: { min: 0.25, max: 1.0, def: 0.85, group: 'Scoring', label: 'Cycle fill', unit: '%', desc: 'fraction of the hopper (up to 60 FUEL) collected before a cycle' },
  cycleTime: { min: 5, max: 24, def: 14, group: 'Scoring', label: 'Max collect time', unit: 's', desc: 's of collecting before it scores what it has while its HUB is active' },
  stageMargin: { min: 0, max: 6, def: 2, group: 'Scoring', label: 'Stage early by', unit: 's', desc: 's of slack when heading in to stage before its HUB turns active' },
  topUp: { min: 2, max: 14, def: 6, group: 'Scoring', label: 'Top-up window', unit: 's', desc: 'tops up its hopper if its HUB stays inactive this much longer than the trip back (s)' },
  spotFx: { min: 1.6, max: 3.7, def: 2.7, group: 'Scoring', label: 'Shot distance from wall', unit: 'm', desc: 'shooting spot distance from its ALLIANCE WALL (m)' },
  spotZ: { min: 0.7, max: 2.6, def: 0.9, group: 'Scoring', label: 'Shot side offset', unit: 'm', desc: 'minimum sideways offset of the shooting spot from the HUB (m)' },
  pushThrough: { min: 0.3, max: 6, def: 1.5, group: 'Scoring', label: 'Push through a blocker after', unit: 's', desc: 's of being blocked on the way to its zone before it stops going around and drives through' },
  shuttle: { min: 0, max: 1, def: 1, group: 'Shuttling', label: 'Shuttle in off shifts', unit: 'switch', desc: 'during an inactive shift, keep collecting and pass FUEL into its ALLIANCE ZONE instead of waiting there (on at 50% or more)' },
  shuttleKeep: { min: 0.2, max: 1, def: 0.75, group: 'Shuttling', label: 'Keep in hopper while shuttling', unit: '%', desc: 'fraction of its hopper it keeps while shuttling; it passes the rest' },
  collectSpeed: { min: 0.3, max: 1.0, def: 0.7, group: 'Collecting', label: 'Speed through FUEL', unit: '%', desc: 'speed among FUEL (fraction of top speed)' },
  density: { min: 0, max: 0.8, def: 0.35, group: 'Collecting', label: 'Cluster preference', unit: '', desc: 'preference for dense FUEL clusters over the nearest FUEL' },
  ownZone: { min: -1, max: 2, def: 0.4, group: 'Collecting', label: 'Own-zone FUEL bonus', unit: 'm', desc: 'preference for FUEL in its own ALLIANCE ZONE (m)' },
  intakeDist: { min: 1.0, max: 4.0, def: 2.5, group: 'Collecting', label: 'Intake drop distance', unit: 'm', desc: 'distance from the target FUEL at which the intake drops (m)' },
  stockpile: { min: 0, max: 3, def: 1.5, group: 'Collecting', label: 'Stockpile pickup bonus', unit: 'm', desc: 'extra preference for FUEL in its own ALLIANCE ZONE while its HUB is active (m)' },
  zoneMin: { min: 1, max: 20, def: 6, group: 'Collecting', label: 'Min FUEL worth staying in zone', unit: '', desc: 'if fewer FUEL than this are lying in its own ALLIANCE ZONE it ignores them and goes to the NEUTRAL ZONE' },
  pinLimit: { min: 0.8, max: 2.9, def: 1.8, group: 'Defense', label: 'Pin hold time', unit: 's', desc: 's it holds a PIN before backing off' },
  pushSpeed: { min: 0.3, max: 1.0, def: 1.0, group: 'Defense', label: 'Shove / ram speed', unit: '%', desc: 'speed it hits the other robot at (fraction of top speed)' },
  ramDist: { min: 0.5, max: 5, def: 2.5, group: 'Defense', label: 'Ram distance', unit: 'm', desc: 'charges the other robot once it comes this close while guarding the lane to its zone (m)' },
  blockLead: { min: 0.4, max: 2.5, def: 1.2, group: 'Defense', label: 'Block lead', unit: 'm', desc: 'how far ahead of the other robot, toward the lane into its zone, it guards (m)' },
  engage: { min: -1.5, max: 3, def: 0, group: 'Defense', label: 'Engage distance', unit: 'm', desc: 'starts shoving once the other robot is this close to its ALLIANCE ZONE (m)' },
  hybridLoad: { min: 0, max: 1, def: 0, group: 'Hybrid', label: 'Load before defending', unit: '%', desc: 'Hybrid only goes to defend once its hopper is at least this full' },
};
export const DEFAULT_BRAIN = Object.fromEntries(Object.entries(BRAIN_SPEC).map(([k, v]) => [k, v.def]));

export const OPP_SKILLS = {
  // handicaps: speed = drive speed scale · load = most of the hopper it uses · noise = shot
  // scatter multiplier · hesitate = pause between cycles (s) · sotm = shoots on the move ·
  // think = decision interval (s) · lag = defender reaction (s). brain = strategy overrides.
  rookie: { name: 'Rookie', speed: 0.6, load: 0.35, noise: 3.0, hesitate: 2.5, sotm: false, think: 0.6, lag: 0.7, brain: { fill: 0.3, collectSpeed: 0.35, pinLimit: 3.6 }, desc: 'Slow, small loads, misses more, stops to shoot, and sometimes holds a PIN too long.' },
  regional: { name: 'Regional', speed: 0.8, load: 0.65, noise: 1.7, hesitate: 1.0, sotm: true, think: 0.35, lag: 0.45, brain: { fill: 0.5, collectSpeed: 0.5, pinLimit: 2.4 }, desc: 'A solid district/regional robot.' },
  champs: { name: 'Champs', speed: 1.0, load: 1.0, noise: 1.0, hesitate: 0, sotm: true, think: 0.2, lag: 0.25, brain: {}, desc: 'Full speed, full hoppers, tight cycles, clean defense (hand-tuned strategy).' },
  trained: { name: 'Trained (self-play)', speed: 1.0, load: 1.0, noise: 1.0, hesitate: 0, sotm: true, think: 0.2, lag: 0.25, brain: null, desc: 'Champs-level robot running the strategy learned by AI-vs-AI self-play (tools/train.mjs).' },
  mine: { name: 'Your trained AI', speed: 1.0, load: 1.0, noise: 1.0, hesitate: 0, sotm: true, think: 0.2, lag: 0.25, brain: {}, desc: 'Champs-level robot running the brain you trained in Training mode and AI Tuning (saved in this browser).' },
};
export const SKILL_ORDER = ['rookie', 'regional', 'champs', 'trained', 'mine'];

export function brainFor(skill, override = null) {
  const sk = OPP_SKILLS[skill] || OPP_SKILLS.regional;
  return { ...DEFAULT_BRAIN, ...(sk.brain === null ? TRAINED_BRAIN : sk.brain), ...(override || {}) };
}

// AUTO routine the AI runs for each strategy
export function opponentAuto(strategy) {
  return strategy === 'defense' ? { routine: 'preload', start: 'hub' } : { routine: 'sweep', start: 'rightTrench' };
}

export class OpponentAI {
  // foe: the other alliance's robot (or null when it has the field to itself)
  constructor({ robot, foe = null, match, fuel, rules, strategy, skill, brain = null }) {
    this.robot = robot;
    this.foe = foe;
    this.match = match;
    this.fuel = fuel;
    this.rules = rules;
    this.strategy = strategy;
    this.skillKey = skill;
    this.skill = OPP_SKILLS[skill] || OPP_SKILLS.regional;
    this.brain = brainFor(skill, brain);
    this.nav = new NavGrid(Math.min(robot.halfL, robot.halfW) + 0.07);
    this.state = 'collect';
    this.mode = 'score';
    this.label = 'Waiting';
    this.path = null;
    this.goal = null;
    this.replanT = 0;
    this.thinkT = 0;
    this.ball = null;
    this.ballBest = 1e9;
    this.ballProgressT = 0;
    this.blacklist = new Map();
    this.collectT = 0;
    this.stuckT = 0;
    this.unstick = 0;
    this.unstickDir = null;
    this.backoff = false;
    this.cooldown = 0;
    this.t = 0;
    this.hesitateT = 0;
    this.seen = []; // where the other robot was, for the defender's reaction lag
    this.rate = 2.5; // FUEL per second it has been collecting at (learned as it plays)
    this.cycleStart = null;
    const cap = robot.capacity();
    robot.noiseScale = this.skill.noise;
    this.maxLoad = cap < 20 ? cap : Math.max(10, Math.round(this.skill.load * cap));
    this.fillTarget = cap < 20 ? cap : Math.min(this.maxLoad, Math.max(8, Math.round(this.brain.fill * Math.min(cap, 60))));
  }

  get own() { return this.robot.alliance; }

  update(dt) {
    const r = this.robot, m = this.match, cmd = r.cmd;
    this.t += dt;
    this.dt = dt;
    this.rethink = false;
    const F = this.foe;
    if (F) {
      this.seen.push({ t: this.t, x: F.pos.x, z: F.pos.z });
      while (this.seen.length > 2 && this.seen[1].t < this.t - this.skill.lag) this.seen.shift();
    }
    Object.assign(cmd, { vx: 0, vz: 0, omega: 0, intake: false, shoot: false, pass: false, outtake: false });
    if (!m.isTeleop) { this.label = m.isAuto ? 'AUTO' : 'Waiting'; return; }

    // mode selection (with nobody to defend against, everyone scores)
    if (!F) this.mode = 'score';
    else if (this.strategy === 'defense') this.mode = 'defend';
    else if (this.strategy === 'hybrid') {
      const s = m.shiftIndex();
      const loaded = r.stored.length >= this.brain.hybridLoad * this.maxLoad;
      this.mode = s >= 1 && s <= 4 && !m.hubActive(this.own) && m.hubActive(F.alliance) && !this._inGrace() && loaded ? 'defend' : 'score';
    } else this.mode = 'score';

    // stuck on something: back out for a moment
    if (this.unstick > 0) {
      this.unstick -= dt;
      const sp = r.cfg.drive.maxSpeed * 0.5;
      cmd.vx = this.unstickDir.x * sp; cmd.vz = this.unstickDir.z * sp;
      this.label = 'Unsticking';
      return;
    }

    if (this.mode === 'defend') this._defend(dt);
    else this._score(dt);

    // stuck detection (pushing against the other robot isn't being stuck)
    const want = Math.hypot(cmd.vx, cmd.vz), have = Math.hypot(r.vel.x, r.vel.z);
    const pushing = F && this.rules.inContact(r, F);
    if (want > 1.0 && have < 0.15 && !pushing) this.stuckT += dt; else this.stuckT = Math.max(0, this.stuckT - dt);
    if (this.stuckT > 1.2) {
      this.stuckT = 0;
      this.unstick = 0.7;
      const a = Math.atan2(-cmd.vz, -cmd.vx) + rand(-1, 1);
      this.unstickDir = { x: Math.cos(a), z: Math.sin(a) };
      if (this.ball) this.blacklist.set(this.ball, this.t + 8);
      this.ball = null;
      this.path = null;
    }
  }

  _inGrace() {
    const m = this.match;
    return !m.hubActive(this.own) && m.t - m.lastDeactivate[this.own] < 1.5;
  }

  // ------------------------------------------------------------------ driving
  _drive(x, z, { speed = 1, face = 'travel', arrive = 0.08, avoid = false } = {}) {
    const r = this.robot, cmd = r.cmd, d = r.cfg.drive;
    this.replanT -= this.dt;
    const P = this.foe;
    if (!this.path || !this.goal || this.replanT <= 0 || Math.hypot(this.goal.x - x, this.goal.z - z) > 0.3) {
      const circles = P && avoid && Math.hypot(P.pos.x - r.pos.x, P.pos.z - r.pos.z) < 5 ? [{ x: P.pos.x, z: P.pos.z, r: Math.max(P.halfL, P.halfW) + 0.1 }] : null;
      this.path = this.nav.plan(r.pos.x, r.pos.z, x, z, circles);
      this.goal = { x, z };
      this.replanT = 0.3;
    }
    const path = this.path;
    while (path.length > 1 && Math.hypot(path[0].x - r.pos.x, path[0].z - r.pos.z) < 0.35) path.shift();
    const next = path[0];
    let remain = Math.hypot(next.x - r.pos.x, next.z - r.pos.z);
    for (let k = 0; k + 1 < path.length; k++) remain += Math.hypot(path[k + 1].x - path[k].x, path[k + 1].z - path[k].z);
    const vmax = d.maxSpeed * this.skill.speed * speed;
    const sp = Math.min(vmax, Math.sqrt(2 * 5 * Math.max(0, remain - arrive)));
    const dx = next.x - r.pos.x, dz = next.z - r.pos.z;
    const dn = Math.hypot(dx, dz) || 1;
    let vx = (dx / dn) * sp, vz = (dz / dn) * sp;
    if (avoid && P) {
      // steer around the other robot
      const ox = r.pos.x - P.pos.x, oz = r.pos.z - P.pos.z;
      const od = Math.hypot(ox, oz);
      const R = 1.4;
      if (od < R && od > 1e-3) {
        const k = ((R - od) / R) * vmax * 0.8;
        const side = (ox * dz - oz * dx) > 0 ? 1 : -1; // go around on the side we're already on
        vx += (ox / od) * k + side * (-oz / od) * k * 0.6;
        vz += (oz / od) * k + side * (ox / od) * k * 0.6;
      }
    }
    cmd.vx = vx; cmd.vz = vz;
    let heading = null;
    if (face === 'travel') { if (sp > 0.3) heading = Math.atan2(-vz, vx); }
    else if (typeof face === 'number') heading = face;
    if (heading !== null) cmd.omega = clamp(5 * wrapAngle(heading - r.yaw), -d.maxOmega, d.maxOmega);
    return remain;
  }

  // ------------------------------------------------------------------ scoring
  _spot() {
    // staging / shooting spot inside our ALLIANCE ZONE, on the side we're already on
    const s = this.own === BLUE ? 1 : -1;
    const zMin = this.brain.spotZ;
    const z = clamp(this.robot.pos.z, -2.6, 2.6);
    return { x: s * (-HALF_L + this.brain.spotFx), z: Math.abs(z) < zMin ? Math.sign(z || 1) * zMin : z };
  }

  _score(dt) {
    const r = this.robot, m = this.match, B = this.brain;
    const stored = r.stored.length, cap = this.maxLoad;
    const active = m.hubActive(this.own);
    const nc = m.hubNextChange(this.own);
    const untilActive = active ? 0 : nc ?? Infinity;
    const spot = this._spot();
    const travel = Math.hypot(spot.x - r.pos.x, spot.z - r.pos.z) / (r.cfg.drive.maxSpeed * this.skill.speed * 0.7) + 1;
    const left = TIMING.teleop - m.phaseTime;
    const canShoot = active || this._inGrace();
    // shuttling: with time to kill in an inactive shift, keep collecting and pass the surplus
    // into our ALLIANCE ZONE instead of waiting there with a full hopper
    const shuttle = B.shuttle >= 0.5 && !canShoot && untilActive > travel + B.stageMargin;

    this.thinkT -= dt;
    if (this.thinkT <= 0) {
      this.thinkT = this.skill.think;
      this.rethink = true;
      if (this.state === 'collect') {
        // time budget: can we still fill up, get back and shoot before our HUB turns off?
        const windowLeft = active ? (nc ?? left) + 1.5 : Infinity;
        const shootTime = (n) => 0.8 + n / r.cfg.shooter.bps;
        const toFill = Math.max(0, this.fillTarget - stored) / Math.max(0.8, this.rate);
        const noTimeToFill = active && stored >= 3 && toFill + travel + shootTime(this.fillTarget) > windowLeft;
        const go = (stored >= cap && !shuttle)
          || (stored >= this.fillTarget && active)
          || noTimeToFill
          || (stored >= 3 && !active && untilActive <= travel + B.stageMargin)
          || (stored >= 6 && active && this.collectT > B.cycleTime)
          || (stored > 0 && left < travel + 4);
        if (go) { this.state = 'score'; this.path = null; this.progress = null; }
      } else if (stored === 0) {
        this.state = 'collect'; this.collectT = 0; this.path = null;
        this.hesitateT = this.skill.hesitate;
      } else if (!canShoot && untilActive > travel + B.topUp && (stored < cap || B.shuttle >= 0.5)) {
        this.state = 'collect'; this.path = null; // plenty of time: top up / shuttle first
      }
    }

    if (this.state === 'collect') {
      if (this.hesitateT > 0) { this.hesitateT -= dt; this.label = 'Deciding…'; return; }
      if (!this.cycleStart) this.cycleStart = { t: this.t, n: r.stats.intaked };
      this.collectT += dt;
      this._collect(dt, shuttle);
      // shoot on the move while collecting in our zone: a turret aims on its own while the
      // intake faces the FUEL (a chassis-aimed shooter keeps its intake on the FUEL instead)
      const turret = r.cfg.shooter.type === 'turret';
      if (turret && canShoot && r.lastInZone && stored > 0 && this.skill.sotm) r.cmd.shoot = true;
      return;
    }
    if (this.cycleStart) {
      // update the collection-rate estimate from the phase that just ended
      const T = this.t - this.cycleStart.t, n = r.stats.intaked - this.cycleStart.n;
      if (T > 2) this.rate = 0.7 * this.rate + 0.3 * (n / T);
      this.cycleStart = null;
    }
    // heading in to score: go around a blocker at first, then drive straight through it
    // "blocked" = a robot is close and we haven't gained 0.3 m on the spot for a while
    // (a defender mirroring us keeps us moving, just never closer)
    const F = this.foe;
    const dF = F ? Math.hypot(F.pos.x - r.pos.x, F.pos.z - r.pos.z) : Infinity;
    const toSpot = Math.hypot(spot.x - r.pos.x, spot.z - r.pos.z);
    if (!this.progress || toSpot < this.progress.d - 0.3 || dF > 3 || toSpot < 0.8) this.progress = { d: toSpot, t: this.t };
    const bully = this.t - this.progress.t > B.pushThrough;
    const remain = this._drive(spot.x, spot.z, { avoid: !bully, speed: 1 });
    const settled = this.skill.sotm || (remain < 0.3 && Math.hypot(r.vel.x, r.vel.z) < 0.3);
    r.cmd.shoot = canShoot && r.lastInZone && settled;
    if (r.capacity() < 20 && r.lastInZone) r.cmd.intake = true;
    this.label = r.cmd.shoot ? (r.ready ? 'Shooting' : 'Aiming') : bully ? 'Pushing through' : remain > 0.3 ? 'Returning to score' : 'Staged — waiting for HUB';
  }

  _pickBall() {
    const r = this.robot, P = this.foe, B = this.brain, m = this.match;
    const oppZone = other(this.own);
    const active = m.hubActive(this.own) || this._inGrace();
    const bins = new Map();
    const cands = [];
    for (const b of this.fuel.balls) {
      if (b.state !== 'field' || b.inFlight || b.hubFresh || b.inCorral) continue;
      const p = b.pos;
      if (p.y > 0.32 || Math.abs(p.x) > HALF_L - 0.2 || Math.abs(p.z) > HALF_W - 0.2) continue;
      if (Field.inAllianceZone(oppZone, p.x)) continue;
      if (obstacleAt(p.x, p.z, 0.12)) continue;
      const bl = this.blacklist.get(b);
      if (bl && bl > this.t) continue;
      const k = Math.floor(p.x / 0.7) * 1000 + Math.floor(p.z / 0.7);
      bins.set(k, (bins.get(k) || 0) + 1);
      cands.push([b, k]);
    }
    // too little FUEL in our own zone to be worth picking through: go to the NEUTRAL ZONE
    const inOwn = (b) => Field.inAllianceZone(this.own, b.pos.x);
    const ownCount = cands.reduce((n, [b]) => n + (inOwn(b) ? 1 : 0), 0);
    const sparse = ownCount < B.zoneMin;
    let best = null, bc = Infinity;
    for (const [b, k] of cands) {
      const p = b.pos;
      const d = Math.hypot(p.x - r.pos.x, p.z - r.pos.z);
      let c = d - B.density * Math.min(bins.get(k), 8);
      if (P && Math.hypot(p.x - P.pos.x, p.z - P.pos.z) < 1.3) c += 2.5;
      // our own zone: FUEL there is close to the HUB (and it's where our shuttled FUEL lands)
      if (inOwn(b)) {
        if (sparse && d > 1.0) continue; // scraps: only grab them if they're right here
        c -= B.ownZone + (active ? B.stockpile : -B.stockpile);
      }
      if (c < bc) { bc = c; best = b; }
    }
    return best;
  }

  _collect(dt, shuttle = false) {
    const r = this.robot, B = this.brain;
    if (!this.ball || this.ball.state !== 'field' || this.rethink) {
      const b = this._pickBall();
      if (b !== this.ball) { this.ball = b; this.ballBest = 1e9; this.ballProgressT = 0; }
    }
    // shuttle: pass the surplus over our ALLIANCE ZONE line, keeping shuttleKeep of the hopper
    const keep = Math.round(B.shuttleKeep * this.maxLoad);
    // only from our half of the NEUTRAL ZONE: shorter lobs land where they're aimed
    const lineX = Field.allianceLineX(this.own);
    const fromLine = this.own === BLUE ? r.pos.x - lineX : lineX - r.pos.x;
    if (shuttle && !r.lastInZone && fromLine < 5) {
      if (r.stored.length >= Math.min(this.maxLoad, keep + 6)) this.passing = true;
    } else this.passing = false;
    if (this.passing && (r.stored.length <= keep || fromLine >= 5)) this.passing = false;
    r.cmd.pass = !!this.passing;
    const b = this.ball;
    if (!b) {
      this.label = 'No FUEL in reach';
      const s = this._spot();
      this._drive(s.x, s.z, { avoid: true });
      r.cmd.pass = !!this.passing;
      return;
    }
    const d = Math.hypot(b.pos.x - r.pos.x, b.pos.z - r.pos.z);
    // give up on FUEL we're not getting closer to
    if (d < this.ballBest - 0.1) { this.ballBest = d; this.ballProgressT = 0; }
    else if ((this.ballProgressT += dt) > 2.5) { this.blacklist.set(b, this.t + 8); this.ball = null; return; }
    const near = d < 1.6;
    this._drive(b.pos.x, b.pos.z, { speed: near ? B.collectSpeed : 1, avoid: !near, arrive: -0.3 });
    // intake down when close, unless that would put it into the other robot (G415)
    const P = this.foe;
    let foeAhead = false;
    if (P) {
      const f = r.forward();
      const toP = { x: P.pos.x - r.pos.x, z: P.pos.z - r.pos.z };
      const dP = Math.hypot(toP.x, toP.z);
      foeAhead = dP < 2.2 && (toP.x * f.x + toP.z * f.z) / dP > -0.1;
    }
    r.cmd.intake = d < B.intakeDist && !foeAhead && r.stored.length < this.maxLoad && !this._hubFuelNear();
    r.cmd.pass = !!this.passing;
    this.label = this.passing ? `Shuttling FUEL to its zone (${r.stored.length})` : `${shuttle ? 'Shuttling' : 'Collecting'} (${r.stored.length}/${r.capacity()})`;
  }

  // FUEL just released by a HUB can't be caught before it touches the carpet (G408)
  _hubFuelNear() {
    const r = this.robot;
    for (const b of this.fuel.balls) {
      if (b.state === 'field' && b.hubFresh && Math.abs(b.pos.x - r.pos.x) < 1.4 && Math.abs(b.pos.z - r.pos.z) < 1.4) return true;
    }
    return false;
  }

  // ------------------------------------------------------------------ defense
  _defend(dt) {
    const r = this.robot, m = this.match, F = this.foe, B = this.brain;
    // react to where the other robot was a moment ago (a juke can beat the defender)
    const seen = this.seen[0];
    const P = { alliance: F.alliance, pos: { x: seen.x, z: seen.z } };
    const pin = this.rules.pinState(r, F);
    const gap = this.rules.gap(r, F);
    const away = { x: r.pos.x - F.pos.x, z: r.pos.z - F.pos.z };
    const ad = Math.hypot(away.x, away.z) || 1;
    away.x /= ad; away.z /= ad;
    this.cooldown = Math.max(0, this.cooldown - dt);
    r.cmd.intake = false;

    if (!this.backoff && pin.t >= B.pinLimit + 3 * pin.fouls) this.backoff = true;
    if (this.backoff) {
      // G418: move 72in away so the PIN count resets
      if (gap >= PIN_RESET + 0.1) { this.backoff = false; this.cooldown = 0.8; }
      const tx = clamp(r.pos.x + away.x * 2.5, -HALF_L + 0.6, HALF_L - 0.6);
      const tz = clamp(r.pos.z + away.z * 2.5, -HALF_W + 0.6, HALF_W - 0.6);
      this._drive(tx, tz, { face: null });
      this.label = 'Backing off (pin)';
      return;
    }
    // latched intakes stay down: lead with the back so it never reaches inside the other frame
    const faceP = Math.atan2(-(P.pos.z - r.pos.z), P.pos.x - r.pos.x);
    const face = r.cfg.intake.latched ? faceP + Math.PI : faceP;
    const endGame = m.phaseTime >= TIMING.teleop - TIMING.endgame;
    if (endGame && towerProtected(F, 1.0)) {
      // G420: stay clear of a robot at (or being pushed toward) its TOWER in END GAME
      this._drive(P.pos.x + away.x * 2.2, P.pos.z + away.z * 2.2, { face });
      this.label = 'Holding off (TOWER)';
      return;
    }
    if (this.cooldown > 0) {
      this._drive(r.pos.x + away.x * 0.5, r.pos.z + away.z * 0.5, { face });
      this.label = 'Resetting';
      return;
    }
    // how far the other robot is from getting into its ALLIANCE ZONE (<= 0: inside)
    const lineX = Field.allianceLineX(P.alliance);
    const outside = P.alliance === BLUE ? P.pos.x - lineX : lineX - P.pos.x;
    const d = Math.hypot(P.pos.x - r.pos.x, P.pos.z - r.pos.z);
    if (outside <= B.engage || F.lastInZone) {
      // it got in: shove the shooter off its shot
      this._drive(P.pos.x, P.pos.z, { face, arrive: -1, speed: d < 1.8 ? B.pushSpeed : 1 });
      this.label = pin.active ? `Pinning ${pin.t.toFixed(1)} s` : 'Shoving';
      return;
    }
    // keep it out: guard the lane it would use to get into its zone (the BUMP or TRENCH
    // nearest it, on the NEUTRAL ZONE side of its HUB line) and ram it back when it comes close
    const hubX = Field.hubCenter(P.alliance).x;
    const toZone = P.alliance === BLUE ? -1 : 1; // direction from the NEUTRAL ZONE into its zone
    const lanes = [-3.39, -1.52, 1.52, 3.39];
    const laneZ = lanes.reduce((a, z) => (Math.abs(z - P.pos.z) < Math.abs(a - P.pos.z) ? z : a));
    const gate = { x: hubX - toZone * 1.5, z: laneZ };
    let gx = gate.x - P.pos.x, gz = gate.z - P.pos.z;
    const gd = Math.hypot(gx, gz) || 1;
    gx /= gd; gz /= gd;
    if (d < B.ramDist) {
      // charge where it's going to be and drive through it, back toward the NEUTRAL ZONE
      const lead = Math.min(0.5, d / 6);
      const tx = F.pos.x + F.vel.x * lead - gx * 0.6, tz = F.pos.z + F.vel.z * lead - gz * 0.6;
      this._drive(tx, tz, { face, arrive: -2, speed: B.pushSpeed });
      this.label = pin.active ? `Pinning ${pin.t.toFixed(1)} s` : this.rules.inContact(r, F) ? 'Driving it back' : 'Ramming';
    } else {
      const lead = Math.min(B.blockLead, gd * 0.7);
      this._drive(P.pos.x + gx * lead, P.pos.z + gz * lead, { face });
      this.label = 'Guarding its lane';
    }
  }
}
