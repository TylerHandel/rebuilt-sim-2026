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
import { BLUE, HALF_L, HALF_W, HUB, TIMING, TRENCH, other } from './constants.js';
import { Field } from './field.js';
import { NavGrid, obstacleAt, underTrench } from './nav.js';
import { towerProtected, PIN_RESET, foeAhead, yieldToFoes } from './rules.js';
import { clamp, wrapAngle, rand } from './util.js';
import { TRAINED_BRAIN } from './trainedBrain.js';
import { fitsTrench } from './robotConfigs.js';

export const OPP_STRATEGIES = {
  off: { name: 'Off (solo)', desc: 'No opponent: just you, the FUEL and the clock.' },
  scorer: { name: 'Scorer', desc: 'Runs its own cycles: collects FUEL, stages when its HUB is inactive and shoots on the move when it is active.' },
  defense: { name: 'Defense', desc: 'Blocks your path to your HUB and pushes you while you shoot. Backs off before a 3 s PIN (G418).' },
  hybrid: { name: 'Hybrid', desc: 'Shift-aware: plays defense while only your HUB is active and scores whenever its own HUB is active.' },
  nn: { name: 'Neural net', desc: 'The neural-network driver trained with tools/nn/train.py (js/nn/driver.json). It drives the whole match, AUTO included.' },
};
export const OPP_ORDER = ['off', 'scorer', 'defense', 'hybrid', 'nn'];

// Trainable strategy parameters: range searched by the trainer and the hand-tuned default
export const BRAIN_SPEC = {
  fill: { min: 0.25, max: 1.0, def: 0.85, group: 'Scoring', label: 'Cycle fill', unit: '%', desc: 'fraction of the hopper collected before a cycle, at the handicapped skills (Rookie, Regional); full-skill robots fill all the way unless time runs short' },
  stageMargin: { min: 0, max: 6, def: 2, group: 'Scoring', label: 'Stage early by', unit: 's', desc: 's of slack when heading in to stage before its HUB turns active' },
  topUp: { min: 2, max: 14, def: 6, group: 'Scoring', label: 'Top-up window', unit: 's', desc: 'tops up its hopper if its HUB stays inactive this much longer than the trip back (s)' },
  spotFx: { min: 1.6, max: 3.7, def: 2.7, group: 'Scoring', label: 'Shot spot from wall (2910, 1678)', unit: 'm', desc: 'shooting spot distance from its ALLIANCE WALL for a chassis-aimed shooter (turrets shoot from anywhere in the zone) (m)' },
  spotZ: { min: 0.7, max: 2.6, def: 0.9, group: 'Scoring', label: 'Shot spot side offset (2910, 1678)', unit: 'm', desc: 'minimum sideways offset of the chassis-aimed shooting spot from the HUB (m)' },
  shootSpeed: { min: 0.2, max: 1, def: 0.45, group: 'Scoring', label: 'Speed while shooting', unit: '%', desc: 'top speed while shooting or passing on the move, so the turret/flywheel can settle (fraction of top speed)' },
  shootAccel: { min: 1, max: 10, def: 3, group: 'Scoring', label: 'Acceleration while shooting', unit: 'm/s²', desc: 'how quickly it changes velocity while shooting or passing (m/s²)' },
  pushThrough: { min: 0.3, max: 6, def: 1.5, group: 'Scoring', label: 'Push through a blocker after', unit: 's', desc: 's of being blocked on the way to its zone before it stops going around and drives through' },
  shuttle: { min: 0, max: 1, def: 1, group: 'Shuttling', label: 'Feed its zone in off shifts', unit: 'switch', desc: 'during an inactive shift, collect in the NEUTRAL ZONE and pass everything into its ALLIANCE ZONE, then load up just before its HUB turns on (on at 50% or more)' },
  passBatch: { min: 2, max: 120, def: 45, group: 'Shuttling', label: 'Pass batch', unit: '', desc: 'passes once it holds this many FUEL (or its hopper is full, or there is nothing left to pick up), then dumps the whole load at once: one turn and one spin-up per batch (a hopperless robot, 341 or 8793, passes each as it comes)' },
  dropFx: { min: 1.4, max: 3.6, def: 2.6, group: 'Shuttling', label: 'Where passes land', unit: 'm', desc: 'distance from its ALLIANCE WALL where its passes land (m)' },
  collectSpeed: { min: 0.3, max: 1.0, def: 0.7, group: 'Collecting', label: 'Speed through FUEL', unit: '%', desc: 'speed among FUEL (fraction of top speed)' },
  density: { min: 0, max: 1, def: 0.6, group: 'Collecting', label: 'Cluster preference', unit: '', desc: 'how much each extra FUEL around a target counts: it goes where it gets the most FUEL per second of driving (0 = nearest FUEL)' },
  ownZone: { min: -1, max: 2, def: 0.4, group: 'Collecting', label: 'Own-zone FUEL bonus', unit: 'm', desc: 'preference for FUEL in its own ALLIANCE ZONE (m)' },
  intakeDist: { min: 1.0, max: 12.0, def: 8, group: 'Collecting', label: 'Intake drop distance', unit: 'm', desc: 'drops the intake once the FUEL it is going for is this close, then keeps it down while it collects (m)' },
  stockpile: { min: 0, max: 3, def: 1.5, group: 'Collecting', label: 'Stockpile pickup bonus', unit: 'm', desc: 'extra preference for FUEL in its own ALLIANCE ZONE while its HUB is active (m)' },
  steal: { min: 0, max: 3, def: 2.0, group: 'Collecting', label: 'Steal preference', unit: 'm', desc: "extra preference for FUEL in the opponent's ALLIANCE ZONE: taking it both denies them and feeds us (m)" },
  zoneMin: { min: 1, max: 20, def: 6, group: 'Collecting', label: 'Min FUEL worth staying in zone', unit: '', desc: 'if fewer FUEL than this are lying in its own ALLIANCE ZONE it ignores them (unless they are right there) and goes where the FUEL is' },
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

// AUTO routine the AI runs for each strategy (scorers run their robot's best AUTO)
export function opponentAuto(strategy) {
  // the defender starts beside its HUB (not behind it, where the driver station can't see it)
  // and ends AUTO waiting in the middle of the field
  return strategy === 'defense' ? { routine: 'defendStage', start: 'rightBump' } : { routine: 'best', start: 'rightTrench' };
}

export class OpponentAI {
  // foes: the other alliance's robots (foe: just one); mates: the rest of its own alliance.
  // this.foe is the opponent it's paying attention to right now: the one it defends, or else
  // the nearest.
  // shadow: it only says what it would do for a robot someone else drives (a teacher for the
  // neural network, tools/nn/worker.mjs): the caller swaps robot.cmd for its own while it runs
  // update(), and it leaves the robot alone otherwise (no climbing, no shot scatter, no
  // unsticking a robot it isn't driving)
  constructor({ robot, foe = null, foes = null, mates = [], match, fuel, rules, strategy, skill, brain = null, shadow = false }) {
    this.robot = robot;
    this.shadow = shadow;
    this.foes = foes || (foe ? [foe] : []);
    this.mates = mates;
    this.others = [...this.foes, ...this.mates];
    this.foe = this.foes[0] || null;
    this.focusT = 0;
    this.match = match;
    this.fuel = fuel;
    this.rules = rules;
    this.strategy = strategy;
    this.skillKey = skill;
    this.skill = OPP_SKILLS[skill] || OPP_SKILLS.regional;
    this.brain = brainFor(skill, brain);
    // routes with the TRENCH open, and without it for when a load bulging the net (or a raised
    // lid) makes the robot too tall to get under the arm
    const navR = Math.min(robot.halfL, robot.halfW) + 0.07;
    this.navFit = new NavGrid(navR, { trench: fitsTrench(robot.cfg) });
    this.navTall = fitsTrench(robot.cfg) ? new NavGrid(navR, { trench: false }) : this.navFit;
    this.nav = this.navFit;
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
    const cap = robot.maxCapacity();
    if (!shadow) robot.noiseScale = this.skill.noise;
    this.maxLoad = cap < 20 ? cap : Math.max(10, Math.round(this.skill.load * cap));
    // a full-skill scorer fills its hopper all the way before a trip (it leaves with less only when
    // time runs short); the handicapped skills (Rookie, Regional) make smaller trips
    this.fillTarget = cap < 20 || this.skill.load >= 1 ? this.maxLoad : Math.min(this.maxLoad, Math.max(8, Math.round(this.brain.fill * cap)));
    // a robot whose hopper is full when its intake can't put more in (robot.full; 2910, 1678:
    // js/mech/physical.js): at full skill it collects until it's full, however many that is (its
    // capacity is only its rating, for the plans), not up to the count
    this.natural = !!robot.phys?.naturalFull;
    if (this.natural && this.skill.load >= 1) this.stopAt = this.fillTarget = Infinity;
    else this.stopAt = this.maxLoad;
    // a turret robot that holds only a few FUEL (hopperless: 341, 8793) doesn't make trips with a
    // load: its FUEL go straight from the intake to the shooter. Out of its zone it passes what it
    // picks up across into its zone as it goes; in its zone it intakes and shoots on the move.
    this.flow = cap < 20 && robot.cfg.shooter.type === 'turret';
    this.lastGainT = 0; this.intakedSeen = 0;
  }

  // room for more: under the load it stops at (and, for one that fills until its intake can't
  // put more in, not full)
  _hasRoom() {
    const r = this.robot;
    return r.stored.length < this.stopAt && !(this.natural && r.full);
  }

  get own() { return this.robot.alliance; }

  update(dt) {
    const r = this.robot, m = this.match, cmd = r.cmd;
    this.t += dt;
    this.dt = dt;
    this.rethink = false;
    this._pickFocus(dt);
    const F = this.foe;
    if (F) {
      this.seen.push({ t: this.t, x: F.pos.x, z: F.pos.z });
      while (this.seen.length > 2 && this.seen[1].t < this.t - this.skill.lag) this.seen.shift();
    }
    Object.assign(cmd, { vx: 0, vz: 0, omega: 0, intake: false, shoot: false, pass: false, outtake: false, prespin: false, lower: false, lineup: false });
    // an intake that stows folded up (8793's) comes down on the way under a TRENCH
    if (r.cfg.intake.fold && fitsTrench(r.cfg) && underTrench(r.pos.x, r.pos.z, r.halfL + 1.2)) cmd.lower = true;
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
    // a physical robot's driver backing off FUEL caught in its intake while it lifts it clear, so
    // it comes back down in front of them (6328's ramp: js/mech/phys6328.js backOff)
    if (!this.shadow && r.phys?.backOff?.()) {
      const f = r.forward(), sp = r.cfg.drive.maxSpeed * 0.3;
      cmd.vx = -f.x * sp; cmd.vz = -f.z * sp; cmd.intake = true;
      this.label = 'Clearing its intake';
      return;
    }

    r.passTarget = null;
    this.shootingMode = false;
    if (this._climb()) return;
    if (this.mode === 'defend') this._defend(dt);
    else { this._score(dt); this._yieldToFoes(); }
    this._smooth(dt);
    // a physical robot whose ball path can lock up where its code can't see it (341's zipper):
    // nothing coming out, its driver lets go of the trigger for a moment and presses it again
    if (r.phys?.letGo?.()) { cmd.shoot = false; cmd.pass = false; }

    // stuck detection (pushing against another robot isn't being stuck)
    const want = Math.hypot(cmd.vx, cmd.vz), have = Math.hypot(r.vel.x, r.vel.z);
    const pushing = this.others.some((o) => (this.foes.includes(o) ? this.rules.inContact(r, o) : Math.hypot(o.pos.x - r.pos.x, o.pos.z - r.pos.z) < r.halfL + o.halfL + 0.15));
    if (want > 1.0 && have < 0.15 && !pushing) this.stuckT += dt; else this.stuckT = Math.max(0, this.stuckT - dt);
    if (this.stuckT > 1.2 && !this.shadow) {
      this.stuckT = 0;
      this.unstick = 0.7;
      const a = Math.atan2(-cmd.vz, -cmd.vx) + rand(-1, 1);
      this.unstickDir = { x: Math.cos(a), z: Math.sin(a) };
      if (this.ball) this.blacklist.set(this.ball, this.t + 8);
      this.ball = null;
      this.path = null;
    }
  }

  // END GAME: a robot with a climber (971, 1678) heads for its TOWER just in time to get up to
  // LEVEL 1 before the buzzer, one per ALLIANCE (the RUNGS have room for one here). Returns true
  // while it's busy with that.
  _climb() {
    const r = this.robot, m = this.match;
    if (this.shadow || !r.climberCfg || !r.cfg.climber) return false;
    if (r.climbState !== 'none') { this.label = r.climbState === 'hanging' ? `Hanging (L${r.climbLevel})` : 'Climbing'; return true; }
    const left = TIMING.teleop - m.phaseTime;
    const p = r.climbPose();
    const d = Math.hypot(p.x - r.pos.x, p.z - r.pos.z);
    if (!r.climbClaim) {
      const need = d / (0.55 * r.cfg.drive.maxSpeed * this.skill.speed) + (r.climberCfg.times[1] ?? 2) + 2.5;
      if (left > need || left < 1.5) return false;
      if (this.mates.some((o) => o.climbClaim || o.climbState !== 'none')) return false;
      r.climbClaim = true;
    }
    // shoot what it can on the way
    r.cmd.shoot = r.lastInZone && r.stored.length > 0;
    if (d > 1.0) this._drive(p.x, p.z, { face: p.yaw, avoid: true });
    else r.requestClimb();
    this.label = 'Going to climb';
    return true;
  }

  // Which opponent to watch: a defender sticks with the biggest threat (loaded, near or in its
  // zone) for a while; otherwise the nearest one
  _pickFocus(dt) {
    const r = this.robot;
    if (this.foes.length < 2) { this.foe = this.foes[0] || null; return; }
    const defending = this.strategy === 'defense' || this.strategy === 'hybrid';
    const dist = (o) => Math.hypot(o.pos.x - r.pos.x, o.pos.z - r.pos.z);
    let pick;
    if (defending) {
      this.focusT -= dt;
      if (this.focusT > 0 && this.foe) return;
      this.focusT = 2.5;
      const threat = (o) => o.stored.length + (o.lastInZone ? 25 : 0) - 3 * dist(o);
      pick = this.foes.reduce((a, b) => (threat(b) > threat(a) ? b : a));
    } else pick = this.foes.reduce((a, b) => (dist(b) < dist(a) ? b : a));
    if (pick !== this.foe) { this.foe = pick; this.seen = []; }
  }

  // Smooth driving while shooting or passing: cap the speed and limit acceleration and turning
  // so the turret / flywheel / hood (or the chassis aim) can settle and the shot is released.
  _smooth(dt) {
    const r = this.robot, cmd = r.cmd, B = this.brain;
    // shooting mode lasts the whole time it shoots/passes on the move, not just while it holds
    // FUEL, so it doesn't sprint off every time it runs empty for a moment
    const firing = ((cmd.shoot || cmd.pass) && r.stored.length > 0) || this.shootingMode;
    if (firing) {
      const last = this.wasFiring && this.lastCmd ? this.lastCmd : { vx: r.vel.x, vz: r.vel.z };
      let vx = cmd.vx, vz = cmd.vz;
      const vmax = r.cfg.drive.maxSpeed * B.shootSpeed;
      const v = Math.hypot(vx, vz);
      if (v > vmax) { vx *= vmax / v; vz *= vmax / v; }
      let dvx = vx - last.vx, dvz = vz - last.vz;
      const dv = Math.hypot(dvx, dvz), maxDv = B.shootAccel * dt;
      if (dv > maxDv) { dvx *= maxDv / dv; dvz *= maxDv / dv; }
      cmd.vx = last.vx + dvx; cmd.vz = last.vz + dvz;
      cmd.omega = clamp(cmd.omega, -2.5, 2.5);
    }
    this.wasFiring = firing;
    this.lastCmd = { vx: cmd.vx, vz: cmd.vz };
  }

  _inGrace() {
    const m = this.match;
    return !m.hubActive(this.own) && m.t - m.lastDeactivate[this.own] < 1.5;
  }

  // ------------------------------------------------------------------ driving
  _drive(x, z, { speed = 1, face = 'travel', arrive = 0.08, avoid = false } = {}) {
    const r = this.robot, cmd = r.cmd, d = r.cfg.drive;
    this.replanT -= this.dt;
    const nav = r.fitsTrenchLowered() ? this.navFit : this.navTall;
    if (nav !== this.nav) { this.nav = nav; this.path = null; }
    if (!this.path || !this.goal || this.replanT <= 0 || Math.hypot(this.goal.x - x, this.goal.z - z) > 0.3) {
      const near = avoid ? this.others.filter((P) => Math.hypot(P.pos.x - r.pos.x, P.pos.z - r.pos.z) < 5) : [];
      const circles = near.length ? near.map((P) => ({ x: P.pos.x, z: P.pos.z, r: Math.max(P.halfL, P.halfW) + 0.1 })) : null;
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
    if (avoid) {
      // steer around the other robots
      for (const P of this.others) {
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
    }
    cmd.vx = vx; cmd.vz = vz;
    let heading = null;
    if (face === 'travel') { if (sp > 0.3) heading = Math.atan2(-vz, vx); }
    else if (typeof face === 'number') heading = face;
    if (heading !== null) heading = this._laneHeading(heading);
    if (heading !== null) cmd.omega = clamp(5 * wrapAngle(heading - r.yaw), -d.maxOmega, d.maxOmega);
    return remain;
  }

  // Under a TRENCH's arm, between its skirt and the wall (1.28 m), a robot with its intake out
  // sweeps wider than that as it turns: turning round in there wedges it across the opening, its
  // corners on the wall and the skirt (6328 with its slam down: their code leaves it down). Once
  // a turn in there isn't happening (0.4 s of it asked for and not turning), it holds the nearest
  // square heading, which it gets to without sweeping wider, and drives on sideways or backward
  // until it's out from under.
  _laneHeading(heading) {
    const r = this.robot;
    if (!underTrench(r.pos.x, r.pos.z, r.halfL + (r.cfg.intake.reach || 0))) { this.laneLock = null; this.laneT = 0; return heading; }
    if (this.laneLock == null) {
      const blocked = Math.abs(wrapAngle(heading - r.yaw)) > 0.35 && Math.abs(r.omega) < 0.3;
      this.laneT = blocked ? (this.laneT || 0) + this.dt : 0;
      if (this.laneT > 0.4) this.laneLock = Math.round(r.yaw / (Math.PI / 2)) * (Math.PI / 2);
    }
    return this.laneLock ?? heading;
  }

  // ------------------------------------------------------------------ scoring
  get turret() { return this.robot.cfg.shooter.type === 'turret'; }

  // where a chassis-aimed shooter (2910) goes to shoot from
  _spot() {
    const s = this.own === BLUE ? 1 : -1;
    const zMin = this.brain.spotZ;
    const z = clamp(this.robot.pos.z, -2.6, 2.6);
    return { x: s * (-HALF_L + this.brain.spotFx), z: Math.abs(z) < zMin ? Math.sign(z || 1) * zMin : z };
  }

  // a turret can shoot from anywhere in the zone: the nearest point just inside the zone line,
  // beside the HUB rather than right in front of it
  _entry() {
    const r = this.robot;
    const lineX = Field.allianceLineX(this.own);
    const inside = this.own === BLUE ? -1 : 1;
    const z = clamp(Math.abs(r.pos.z), 1.3, 2.6) * (Math.sign(r.pos.z) || 1);
    const x = Field.inAllianceZone(this.own, r.pos.x) && Math.abs(r.pos.x - lineX) > 0.8 ? r.pos.x : lineX + inside * 0.9;
    return { x, z: Field.inAllianceZone(this.own, r.pos.x) ? r.pos.z : z };
  }

  _home() { return this.turret ? this._entry() : this._spot(); }

  // how much FUEL is lying in our own ALLIANCE ZONE (recounted each decision)
  _zoneFuel() {
    if (this._zoneFuelT === this.t) return this._zoneFuelN;
    let n = 0;
    for (const b of this.fuel.balls) {
      if (b.state === 'field' && !b.inFlight && !b.held && b.pos.y < 0.32 && Field.inAllianceZone(this.own, b.pos.x) && Math.abs(b.pos.x) < HALF_L - 0.2) n++;
    }
    this._zoneFuelT = this.t;
    this._zoneFuelN = n;
    return n;
  }

  _score(dt) {
    const r = this.robot, m = this.match, B = this.brain;
    const stored = r.stored.length, cap = this.maxLoad;
    // (when the last FUEL got in, or it started collecting)
    if (r.stats.intaked !== this.intakedSeen || this.state !== 'collect') { this.intakedSeen = r.stats.intaked; this.lastGainT = this.t; }
    const active = m.hubActive(this.own);
    const nc = m.hubNextChange(this.own);
    const untilActive = active ? 0 : nc ?? Infinity;
    const home = this._home();
    const travel = Math.hypot(home.x - r.pos.x, home.z - r.pos.z) / (r.cfg.drive.maxSpeed * this.skill.speed * 0.7) + 1;
    const left = TIMING.teleop - m.phaseTime;
    const canShoot = active || this._inGrace();
    // off shift: feed our zone until it's time to load the hopper and head in
    const loadTime = Math.max(0, cap - stored) / Math.max(0.8, this.rate);
    const feed = B.shuttle >= 0.5 && !canShoot && untilActive > travel + loadTime + B.stageMargin;

    this.thinkT -= dt;
    if (this.thinkT <= 0) {
      this.thinkT = this.skill.think;
      this.rethink = true;
      if (feed) {
        if (this.state !== 'feed') { this.state = 'feed'; this.path = null; this.ball = null; }
      } else if (this.state === 'feed') {
        this.state = 'collect'; this.collectT = 0; this.path = null; this.ball = null; // load up
      } else if (this.state === 'collect') {
        // Fill the hopper all the way before a trip (it's full when the intake stalls against the
        // load). It heads in with less only when time runs short: while our HUB is active, at the
        // last call, when it has to leave now to get back and empty the hopper before the HUB turns
        // off (every FUEL it picks up until then is one more to score); while it's inactive, in
        // time to be staged when it turns on; and at the end of the match.
        const windowLeft = active ? (nc ?? left) + 1.5 : Infinity;
        const shootTime = (n) => 0.8 + n / r.cfg.shooter.bps;
        const lastCall = active && stored >= 3 && windowLeft <= travel + shootTime(stored) + 1;
        // (one that fills until its intake can't put more in: past its rating with nothing more
        // getting in for 2 s, or three quarters of it and nothing for 3 s, it's as full as it'll
        // get here)
        const stale = this.t - this.lastGainT;
        const go = stored >= this.stopAt || (r.full && stored >= 0.5 * cap)
          || (this.natural && ((stored >= cap && stale > 2) || (stored >= 0.75 * cap && stale > 3)))
          || (stored >= this.fillTarget && active)
          || lastCall
          || (stored >= 3 && !active && untilActive <= travel + B.stageMargin)
          || (stored > 0 && left < travel + 4);
        // a turret already in its zone just keeps collecting there and shoots as it goes (one that
        // keeps its FUEL flowing, wherever the next FUEL is: it shoots on its way to it)
        const stay = this.turret && r.lastInZone && canShoot && (this.flow || (this._zoneWorthIt() && this._hasRoom()));
        if (go && !stay) {
          this.state = 'score'; this.path = null; this.progress = null;
        }
      } else if (stored === 0) {
        this.state = 'collect'; this.collectT = 0; this.path = null;
        this.hesitateT = this.skill.hesitate;
      } else if (!canShoot && untilActive > travel + B.topUp && this._hasRoom()) {
        this.state = 'collect'; this.path = null; // plenty of time: top up first
      } else if (this.turret && canShoot && r.lastInZone && (this.flow || (this._zoneWorthIt() && this._hasRoom()))) {
        this.state = 'collect'; this.path = null; // shoot while picking up the stockpile (or what's on the way)
      }
    }

    if (this.state !== this.intakeState) { this.intakeState = this.state; this.intakeDown = false; this.shotIntake = null; }
    if (this.state === 'feed') { this._feed(dt); return; }
    if (this.state === 'collect') {
      if (this.hesitateT > 0) { this.hesitateT -= dt; this.label = 'Deciding…'; return; }
      if (!this.cycleStart) this.cycleStart = { t: this.t, n: r.stats.intaked };
      this.collectT += dt;
      this._collect(dt);
      // shoot on the move while collecting in our zone: a turret aims on its own while the
      // intake faces the FUEL (a chassis-aimed shooter keeps its intake on the FUEL instead)
      if (this.turret && canShoot && r.lastInZone && this.skill.sotm) {
        this.shootingMode = stored > 0; // full speed when there's nothing to shoot
        r.cmd.shoot = stored > 0;
        this.label = `Shooting while collecting (${stored})`;
      } else if (this.flow && canShoot && !r.lastInZone && stored > 0 && this._passOn(left, travel)) {
        // out of its zone and staying out for more: pass what it picks up across as it goes
        r.passTarget = this._dropPoint();
        r.cmd.pass = this._passClear(r.pos, r.passTarget);
        r.cmd.prespin = true;
        this.label = `Passing while collecting (${r.stats.passes} passed)`;
      }
      return;
    }
    if (this.cycleStart) {
      // update the collection-rate estimate from the phase that just ended
      const T = this.t - this.cycleStart.t, n = r.stats.intaked - this.cycleStart.n;
      if (T > 2) this.rate = 0.7 * this.rate + 0.3 * (n / T);
      this.cycleStart = null;
    }
    // turret already in the zone: shoot from right here
    if (this.turret && r.lastInZone && canShoot && this.skill.sotm) {
      r.cmd.shoot = true;
      this.label = r.ready ? 'Shooting' : 'Aiming';
      return;
    }
    // (heading in to score with FUEL: lining up a shot, as a driver holding an aim trigger does;
    // only a physical robot that asks reads it: 1114's flywheel spins up to its shot)
    r.cmd.lineup = r.stored.length > 0;
    // heading in to score: go around a blocker at first, then drive straight through it.
    // "blocked" = a robot is close and we haven't gained 0.3 m on home for a while
    const dF = this.others.reduce((a, o) => Math.min(a, Math.hypot(o.pos.x - r.pos.x, o.pos.z - r.pos.z)), Infinity);
    const toHome = Math.hypot(home.x - r.pos.x, home.z - r.pos.z);
    if (!this.progress || toHome < this.progress.d - 0.3 || dF > 3 || toHome < 0.8) this.progress = { d: toHome, t: this.t };
    const bully = this.t - this.progress.t > B.pushThrough;
    // a chassis-aimed shooter turns its shooter to the HUB over the last few meters, so it arrives
    // aimed instead of turning round once it gets there
    let face = 'travel';
    if (!this.turret && (toHome < 3 || r.lastInZone)) {
      const c = Field.hubCenter(this.own);
      face = Math.atan2(-(c.z - r.pos.z), c.x - r.pos.x) - r.aimOffset;
    }
    const remain = this._drive(home.x, home.z, { avoid: !bully, speed: 1, face });
    const settled = this.skill.sotm || (remain < 0.3 && Math.hypot(r.vel.x, r.vel.z) < 0.3);
    r.cmd.shoot = canShoot && r.lastInZone && settled;
    // pick up what's on the way (an intake that raises the robot's top, 1678's lid, not near a
    // TRENCH: it wouldn't fit under)
    const lidUp = r.deployRaisesTop && underTrench(r.pos.x, r.pos.z, Math.max(r.halfL, r.halfW) + 2.0);
    const hold = this._shotHold();
    if (hold !== false && this._hasRoom() && !this._foeAhead() && !this._hubFuelNear() && !lidUp) r.cmd.intake = true;
    // a robot whose shot compacts its load only with its intake let go (1114's AgitateIntake
    // pulling its box in, 581's intake coming in, 1678's contraction, 2910's intake folding back
    // over the load, 971's and 4930's: robotConfigs.js compacts): it goes on intaking as it starts
    // shooting, for as long as its code waits before it compacts (_shotHold: 1114 0.65 s, 581 0.7,
    // 1678 1; 0.65 where it compacts at once), and while there's FUEL in front of its intake and
    // it's still taking some in (one in the last second), then lets go for the rest of the shot, so
    // the load is compacted behind it
    if (typeof hold === 'number') {
      // (shotIntake: until when it keeps intaking; false once it's let go, until the shot ends)
      if (!r.cmd.shoot) this.shotIntake = null;
      else if (r.cmd.intake) {
        if (this.shotIntake == null) { this.shotIntake = this.t + hold; this.shotGain = { n: r.stats.intaked, t: this.t }; }
        if (r.stats.intaked !== this.shotGain.n) this.shotGain = { n: r.stats.intaked, t: this.t };
        if (this.shotIntake !== false && this.t - this.shotGain.t < 1 && this._fuelAhead()) this.shotIntake = Math.max(this.shotIntake, this.t + 0.3);
        if (this.shotIntake !== false && this.t > this.shotIntake) this.shotIntake = false;
        if (this.shotIntake === false) r.cmd.intake = false;
      }
    }
    this.label = r.cmd.shoot ? (r.ready ? 'Shooting' : 'Aiming') : bully ? 'Pushing through' : remain > 0.3 ? 'Returning to score' : 'Staged — waiting for HUB';
  }

  // one that keeps its FUEL flowing, out of its zone while its HUB is active: is a pass worth more
  // than carrying it in? Only while it's staying out for more (the FUEL it's going for is out
  // here), with time left to pick the pass up again and shoot it
  _passOn(left, travel) {
    const b = this.ball;
    return (!b || !Field.inAllianceZone(this.own, b.pos.x)) && left > travel + 8;
  }

  // where our passes land: inside our zone, beside the HUB rather than behind it
  _dropPoint() {
    const r = this.robot, s = this.own === BLUE ? 1 : -1;
    const side = Math.sign(r.pos.z) || 1;
    return { x: s * (-HALF_L + this.brain.dropFx), z: side * clamp(Math.abs(r.pos.z), 1.4, 2.6) };
  }

  // a pass from p to q must not fly over our HUB (FUEL landing in it from outside the zone
  // would be a G407 MAJOR FOUL, and the HUB's net would stop it anyway)
  _passClear(p, q) {
    const hx = Field.hubCenter(this.own).x;
    if ((p.x - hx) * (q.x - hx) > 0) return true;
    const t = (hx - p.x) / (q.x - p.x);
    return Math.abs(p.z + (q.z - p.z) * t) > HUB.size / 2 + 0.55;
  }

  // Off shift: collect in the NEUTRAL ZONE and pass it all into our zone
  _feed(dt) {
    const r = this.robot, B = this.brain;
    const target = this._dropPoint();
    r.passTarget = target;
    if (!this.ball || this.ball.state !== 'field' || this.ball.held || this.rethink) {
      const b = this._pickBall({ feed: true });
      if (b !== this.ball) { this.ball = b; this.ballBest = 1e9; this.ballProgressT = 0; }
    }
    const lineX = Field.allianceLineX(this.own);
    // (from anywhere out of our zone, however far: from out of reach the robot throws as far as it
    // can and the FUEL fall short, closer to home than they were; js/robot.js _shooter)
    const canPass = !r.lastInZone && this._passClear(r.pos, target);
    const stored = r.stored.length;
    // collect a batch, then dump it all in one go: one spin-up (and for a chassis-aimed shooter
    // one turn) per batch instead of one per FUEL. It passes early if there's nothing left to get.
    // (one that keeps its FUEL flowing passes each one as it comes, its flywheel kept spinning)
    const batch = this.flow ? 1 : Math.min(this.maxLoad, Math.max(2, Math.round(B.passBatch)));
    if (canPass && stored > 0 && (stored >= batch || !this.ball)) this.passing = true;
    if (!canPass || stored === 0) this.passing = false;
    this.shootingMode = !!this.passing && !this.flow;
    // nearly there: spin the flywheel up (and aim a turret) ahead of the dump
    r.cmd.prespin = canPass && (this.flow || stored >= 0.7 * batch);
    const b = !this._hasRoom() || r.full ? null : this.ball; // full: stop chasing FUEL, go pass it
    if (this.passing && !this.turret) {
      // chassis-aimed: ease off while the chassis turns and dumps the batch
      if (b) this._drive(b.pos.x, b.pos.z, { speed: 0.3, avoid: true });
    } else if (b) {
      const d = Math.hypot(b.pos.x - r.pos.x, b.pos.z - r.pos.z);
      if (d < this.ballBest - 0.1) { this.ballBest = d; this.ballProgressT = 0; }
      else if ((this.ballProgressT += dt) > 2.5) { this.blacklist.set(b, this.t + 8); this.ball = null; }
      this._drive(b.pos.x, b.pos.z, { speed: d < 1.6 ? B.collectSpeed : 1, avoid: d >= 1.6, arrive: -0.3 });
      r.cmd.intake = this._intakeFor(d);
    } else if (!stored) {
      // nothing left out there: wait at home
      const h = this._home();
      this._drive(h.x, h.z, { avoid: true });
    }
    // full or nothing left, and it can't pass from here (in our zone, or our HUB in the way): out
    // past the line, beside the HUB
    if (!b && stored && !canPass) {
      const tx = lineX + (this.own === BLUE ? 2.5 : -2.5), tz = target.z;
      this._drive(tx, tz, { avoid: true });
    }
    r.cmd.pass = !!this.passing;
    this.label = `Feeding its zone (${r.stats.passes} passed)`;
  }

  // an opponent right in front of the intake: it would reach inside its FRAME PERIMETER (G415;
  // js/rules.js foeAhead)
  _foeAhead() { return foeAhead(this.robot, this.foes); }

  // with an opponent in front of an intake that's out (or can't come up, a latched one), don't
  // drive into it (G415 is on the ROBOT driving in)
  _yieldToFoes() { yieldToFoes(this.robot, this.foes, this.robot.cmd); }

  // Where to collect next: the FUEL that gets the most FUEL per second of driving. Each candidate
  // is valued by the cluster around it (up to what still fits in the hopper), so a big pile a few
  // meters away beats picking through scraps. A trip out of our zone also pays for the drive back
  // before it can score; FUEL we can shoot while picking it up (our zone, HUB active) counts extra,
  // and so does FUEL stolen from the other zone. FUEL behind us costs a turn.
  _pickBall({ feed = false } = {}) {
    const r = this.robot, B = this.brain, m = this.match;
    const oppZone = other(this.own);
    const active = m.hubActive(this.own) || this._inGrace();
    const lineX = Field.allianceLineX(this.own);
    const bins = new Map();
    const cands = [];
    const CELL = 0.5;
    const tall = !r.fitsTrenchLowered();
    for (const b of this.fuel.balls) {
      // (held: inside a robot built of its mechanisms, still a body on the FIELD, but taken)
      if (b.state !== 'field' || b.inFlight || b.hubFresh || b.inCorral || b.held) continue;
      const p = b.pos;
      if (p.y > 0.32 || Math.abs(p.x) > HALF_L - 0.2 || Math.abs(p.z) > HALF_W - 0.2) continue;
      if (obstacleAt(p.x, p.z, 0.12)) continue;
      if (tall && underTrench(p.x, p.z, 0.3)) continue; // can't get under the arm to it
      // under its own frame (341's, where one slips past the intake): its intake can't get at it
      // from here, and it would circle on the spot chasing it
      if (Math.hypot(p.x - r.pos.x, p.z - r.pos.z) < r.halfL + r.halfW) {
        const l = r.worldToLocal(p.x, p.z);
        if (Math.abs(l.x) < r.halfL - 0.05 && Math.abs(l.z) < r.halfW - 0.05) continue;
      }
      const bl = this.blacklist.get(b);
      if (bl && bl > this.t) continue;
      const i = Math.floor(p.x / CELL), j = Math.floor(p.z / CELL);
      bins.set(i * 1000 + j, (bins.get(i * 1000 + j) || 0) + 1);
      if (feed && Field.inAllianceZone(this.own, p.x)) continue; // already on our side
      cands.push([b, i, j]);
    }
    // FUEL within about 0.75 m (the 3x3 cells round it)
    const around = (i, j) => { let n = 0; for (let a = -1; a <= 1; a++) for (let c = -1; c <= 1; c++) n += bins.get((i + a) * 1000 + j + c) || 0; return n; };
    const inOwn = (b) => Field.inAllianceZone(this.own, b.pos.x);
    const sparse = this._zoneFuel() < B.zoneMin;
    const room = Math.max(1, this.maxLoad - r.stored.length);
    const f = r.forward();
    let best = null, bc = Infinity;
    for (const [b, i, j] of cands) {
      const p = b.pos;
      const d = Math.hypot(p.x - r.pos.x, p.z - r.pos.z);
      const own = inOwn(b);
      if (!feed && own && sparse && d > 1.0) continue; // scraps: only grab them if they're right here
      let gain = 1 + B.density * (Math.min(room, around(i, j)) - 1);
      for (const P of this.others) if (Math.hypot(p.x - P.pos.x, p.z - P.pos.z) < 1.3) gain *= 0.5; // leave it to them
      // stealing from the opponent's zone counts twice: one fewer for them, one more for us
      if (Field.inAllianceZone(oppZone, p.x)) gain *= 1 + 0.3 * B.steal;
      // turning round to go back for it
      let dist = d + (d > 0.3 ? 0.5 * (1 - ((p.x - r.pos.x) * f.x + (p.z - r.pos.z) * f.z) / d) : 0);
      if (feed) {
        // feeding: stay on our half, and out of the HUB's shadow so the passes are clear
        const fromLine = this.own === BLUE ? p.x - lineX : lineX - p.x;
        if (!Field.inAllianceZone(oppZone, p.x)) dist += 0.35 * Math.max(0, fromLine - 2);
        if (Math.abs(p.z) < 1.1) dist += 1.5;
      } else if (own) {
        gain *= Math.max(0.2, 1 + 0.4 * (B.ownZone + (active ? B.stockpile : 0)));
      } else {
        // it has to come back into our zone before any of this scores
        dist += Math.abs(p.x - lineX) * 0.5;
      }
      const c = Math.max(0.2, dist) / gain;
      if (c < bc) { bc = c; best = b; }
    }
    return best;
  }

  // a turret collecting in its zone while its HUB is active: is our zone still the best place to
  // collect (it shoots as it goes), or is there more to be had outside?
  _zoneWorthIt() {
    if (this._zoneFuel() < this.brain.zoneMin) return false;
    const b = this._pickBall();
    return !!b && Field.inAllianceZone(this.own, b.pos.x);
  }

  // the intake goes down once the FUEL it's going for is within intakeDist and stays down while
  // it collects (up only when full, or to keep it off another robot or out of FUEL a HUB is
  // releasing)
  _intakeFor(d) {
    const r = this.robot;
    if (d < this.brain.intakeDist) this.intakeDown = true;
    // near a TRENCH with the FUEL almost up to the arm: more would wedge it under there (held
    // until the load drops, so the intake doesn't bob as the load settles). An intake that raises
    // the top when it's out (1678's lid) stays in near the arm.
    const near = underTrench(r.pos.x, r.pos.z, 1.0);
    if (r.loadNearTrench()) this.tallHold = true;
    else if (r.loadTop < TRENCH.clearHeight - 0.08) this.tallHold = false;
    const wedge = near && (this.tallHold || r.deployRaisesTop);
    return this.intakeDown && this._hasRoom() && !wedge && !this._foeAhead() && !this._hubFuelNear();
  }

  _collect(dt) {
    const r = this.robot, B = this.brain;
    if (!this.ball || this.ball.state !== 'field' || this.ball.held || this.rethink) {
      const b = this._pickBall();
      if (b !== this.ball) { this.ball = b; this.ballBest = 1e9; this.ballProgressT = 0; }
    }
    const b = this.ball;
    if (!b) {
      this.label = 'No FUEL in reach';
      const h = this._home();
      this._drive(h.x, h.z, { avoid: true });
      return;
    }
    const d = Math.hypot(b.pos.x - r.pos.x, b.pos.z - r.pos.z);
    // one that keeps its FUEL flowing, shooting in its zone with the next FUEL out of it: it
    // goes as far as the zone line and shoots what it has before it leaves
    const lineX = Field.allianceLineX(this.own), inside = this.own === BLUE ? -1 : 1;
    const hold = this.flow && r.lastInZone && r.stored.length > 0 && (this.match.hubActive(this.own) || this._inGrace())
      && (b.pos.x - lineX) * inside < 0.6;
    // give up on FUEL we're not getting closer to
    if (hold || d < this.ballBest - 0.1) { this.ballBest = d; this.ballProgressT = 0; }
    else if ((this.ballProgressT += dt) > 2.5) { this.blacklist.set(b, this.t + 8); this.ball = null; return; }
    const near = d < 1.6;
    if (hold) this._drive(lineX + inside * 0.6, clamp(b.pos.z, -HALF_W + 0.6, HALF_W - 0.6), { speed: B.collectSpeed, avoid: true });
    else this._drive(b.pos.x, b.pos.z, { speed: near ? B.collectSpeed : 1, avoid: !near, arrive: -0.3 });
    r.cmd.intake = this._intakeFor(d);
    this.label = `Collecting (${r.stored.length}${r.full ? ', full' : '/~' + r.capacity()})`;
  }

  // how long a robot whose shot compacts its load (with its intake let go) keeps intaking as the
  // shot starts: its own code's wait before it compacts (a physical robot's shotHold), 0.65 s
  // where it compacts at once; null for one that doesn't compact; false for one that keeps its
  // intake up on the way in to score (1706: scoring alone, seeds 1-4, it scored 227 TELEOP FUEL
  // on average that way and 211 picking up what's on the way)
  _shotHold() {
    const r = this.robot, h = r.phys?.shotHold;
    if (typeof h === 'number' || h === false) return h;
    return r.cfg.intake.compacts ? 0.65 : null;
  }

  // loose FUEL on the carpet in front of the intake, about to be in its reach
  _fuelAhead() {
    const r = this.robot, ic = r.cfg.intake, f = r.forward();
    const v = Math.max(0, r.vel.x * f.x + r.vel.z * f.z), far = r.halfL + ic.reach + 0.3 + 0.5 * v;
    for (const b of this.fuel.balls) {
      if (b.state !== 'field' || b.held || b.inFlight || b.pos.y > 0.3) continue;
      if (Math.abs(b.pos.x - r.pos.x) > far + 0.5 || Math.abs(b.pos.z - r.pos.z) > far + 0.5) continue;
      const l = r.worldToLocal(b.pos.x, b.pos.z);
      if (l.x > r.halfL - 0.1 && l.x < far && Math.abs(l.z) < ic.width / 2 + 0.05) return true;
    }
    return false;
  }

  // FUEL just released by a HUB can't be caught before it touches the carpet (G408): is any of it
  // about to come down into our intake? (Only then does the intake pause: it's in the air for a
  // fraction of a second, and the piles in front of a HUB are where the FUEL is.)
  _hubFuelNear() {
    const r = this.robot, ic = r.cfg.intake, g = 9.81;
    for (const b of this.fuel.balls) {
      if (b.state !== 'field' || !b.hubFresh) continue;
      if (Math.abs(b.pos.x - r.pos.x) > 2.5 || Math.abs(b.pos.z - r.pos.z) > 2.5) continue;
      const v = b.prevVel || { x: 0, y: 0, z: 0 };
      // when it gets down to where the rollers could grab it, and where it is (relative to us) then
      const h = b.pos.y - r.pos.y - 0.33;
      const t = h <= 0 ? 0 : (v.y + Math.sqrt(v.y * v.y + 2 * g * h)) / g;
      const l = r.worldToLocal(b.pos.x + (v.x - r.vel.x) * t, b.pos.z + (v.z - r.vel.z) * t);
      if (l.x > r.halfL - 0.2 && l.x < r.halfL + ic.reach + 0.35 && Math.abs(l.z) < ic.width / 2 + 0.3) return true;
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
