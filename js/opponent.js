// AI opponent (PvE). In AUTO it runs a normal auto routine; in TELEOP it drives with one of
// these strategies:
//   scorer  - collects FUEL, stages in its ALLIANCE ZONE and shoots while its HUB is active
//   defense - blocks and pushes the player, backing off before a PIN turns into a foul
//   hybrid  - plays defense while only the player's HUB is active, scores the rest of the time
import { BLUE, HALF_L, HALF_W, TIMING } from './constants.js';
import { Field } from './field.js';
import { NavGrid, obstacleAt } from './nav.js';
import { towerProtected, PIN_RESET } from './rules.js';
import { clamp, wrapAngle, rand } from './util.js';

export const OPP_STRATEGIES = {
  off: { name: 'Off (solo)', desc: 'No opponent: just you, the FUEL and the clock.' },
  scorer: { name: 'Scorer', desc: 'Runs its own cycles: collects FUEL, stages when its HUB is inactive and shoots on the move when it is active.' },
  defense: { name: 'Defense', desc: 'Blocks your path to your HUB and pushes you while you shoot. Backs off before a 3 s PIN (G418).' },
  hybrid: { name: 'Hybrid', desc: 'Shift-aware: plays defense while only your HUB is active and scores whenever its own HUB is active.' },
};
export const OPP_ORDER = ['off', 'scorer', 'defense', 'hybrid'];

export const OPP_SKILLS = {
  // speed: drive speed scale · collect: speed scale among FUEL · fill: how full before a cycle
  // hesitate: pause between cycles (s) · sotm: shoots on the move · lag: defender reaction (s)
  rookie: { name: 'Rookie', speed: 0.6, collect: 0.35, fill: 0.4, hesitate: 2.0, sotm: false, pinLimit: 3.6, think: 0.6, lag: 0.7, desc: 'Slow, stops to shoot, and sometimes holds a PIN too long.' },
  regional: { name: 'Regional', speed: 0.8, collect: 0.5, fill: 0.6, hesitate: 0.8, sotm: true, pinLimit: 2.4, think: 0.35, lag: 0.45, desc: 'A solid district/regional robot.' },
  champs: { name: 'Champs', speed: 1.0, collect: 0.7, fill: 0.85, hesitate: 0, sotm: true, pinLimit: 1.8, think: 0.2, lag: 0.25, desc: 'Full speed, tight cycles, clean defense.' },
};
export const SKILL_ORDER = ['rookie', 'regional', 'champs'];

// AUTO routine the AI runs for each strategy
export function opponentAuto(strategy) {
  return strategy === 'defense' ? { routine: 'preload', start: 'hub' } : { routine: 'sweep', start: 'rightTrench' };
}

export class OpponentAI {
  constructor({ robot, player, match, fuel, rules, strategy, skill }) {
    this.robot = robot;
    this.player = player;
    this.match = match;
    this.fuel = fuel;
    this.rules = rules;
    this.strategy = strategy;
    this.skill = OPP_SKILLS[skill] || OPP_SKILLS.regional;
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
    this.seen = []; // where the player was, for the defender's reaction lag
    const cap = robot.capacity();
    this.fillTarget = cap < 20 ? cap : Math.max(8, Math.round(this.skill.fill * Math.min(cap, 60)));
  }

  get own() { return this.robot.alliance; }

  update(dt) {
    const r = this.robot, m = this.match, cmd = r.cmd;
    this.t += dt;
    this.dt = dt;
    this.rethink = false;
    this.seen.push({ t: this.t, x: this.player.pos.x, z: this.player.pos.z });
    while (this.seen.length > 2 && this.seen[1].t < this.t - this.skill.lag) this.seen.shift();
    Object.assign(cmd, { vx: 0, vz: 0, omega: 0, intake: false, shoot: false, pass: false, outtake: false });
    if (!m.isTeleop) { this.label = m.isAuto ? 'AUTO' : 'Waiting'; return; }

    // mode selection
    if (this.strategy === 'defense') this.mode = 'defend';
    else if (this.strategy === 'hybrid') {
      const s = m.shiftIndex();
      this.mode = s >= 1 && s <= 4 && !m.hubActive(this.own) && m.hubActive(this.player.alliance) && !this._inGrace() ? 'defend' : 'score';
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

    // stuck detection (not while deliberately pushing the player)
    const want = Math.hypot(cmd.vx, cmd.vz), have = Math.hypot(r.vel.x, r.vel.z);
    const pushing = this.mode === 'defend' && this.rules.inContact(r, this.player);
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
    if (!this.path || !this.goal || this.replanT <= 0 || Math.hypot(this.goal.x - x, this.goal.z - z) > 0.3) {
      const P = this.player;
      const circles = avoid && Math.hypot(P.pos.x - r.pos.x, P.pos.z - r.pos.z) < 5 ? [{ x: P.pos.x, z: P.pos.z, r: Math.max(P.halfL, P.halfW) + 0.1 }] : null;
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
      // steer around the player's robot
      const P = this.player;
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
    const z = clamp(this.robot.pos.z, -2.6, 2.6);
    return { x: s * (-HALF_L + 2.7), z: Math.abs(z) < 0.7 ? Math.sign(z || 1) * 0.9 : z };
  }

  _score(dt) {
    const r = this.robot, m = this.match;
    const stored = r.stored.length, cap = r.capacity();
    const active = m.hubActive(this.own);
    const nc = m.hubNextChange(this.own);
    const untilActive = active ? 0 : nc ?? Infinity;
    const spot = this._spot();
    const travel = Math.hypot(spot.x - r.pos.x, spot.z - r.pos.z) / (r.cfg.drive.maxSpeed * this.skill.speed * 0.7) + 1;
    const left = TIMING.teleop - m.phaseTime;
    const canShoot = active || this._inGrace();

    this.thinkT -= dt;
    if (this.thinkT <= 0) {
      this.thinkT = this.skill.think;
      this.rethink = true;
      if (this.state === 'collect') {
        const go = stored >= cap
          || (stored >= this.fillTarget && untilActive <= travel + 2)
          || (stored >= 6 && active && this.collectT > 14)
          || (stored > 0 && left < travel + 4);
        if (go) { this.state = 'score'; this.path = null; }
      } else if (stored === 0) {
        this.state = 'collect'; this.collectT = 0; this.path = null;
        this.hesitateT = this.skill.hesitate;
      } else if (!canShoot && untilActive > travel + 6 && stored < cap) {
        this.state = 'collect'; this.path = null; // plenty of time: top up first
      }
    }

    if (this.state === 'collect') {
      if (this.hesitateT > 0) { this.hesitateT -= dt; this.label = 'Deciding…'; return; }
      this.collectT += dt;
      this._collect(dt);
      // the hopperless robot keeps scoring as it collects in its own zone
      if (cap < 20 && canShoot && r.lastInZone) r.cmd.shoot = true;
      return;
    }
    const remain = this._drive(spot.x, spot.z, { avoid: true, speed: 1 });
    const settled = this.skill.sotm || (remain < 0.3 && Math.hypot(r.vel.x, r.vel.z) < 0.3);
    r.cmd.shoot = canShoot && r.lastInZone && settled;
    if (cap < 20 && r.lastInZone) r.cmd.intake = true;
    this.label = r.cmd.shoot ? (r.ready ? 'Shooting' : 'Aiming') : remain > 0.3 ? 'Returning to score' : 'Staged — waiting for HUB';
  }

  _pickBall() {
    const r = this.robot, P = this.player;
    const oppZone = P.alliance;
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
    let best = null, bc = Infinity;
    for (const [b, k] of cands) {
      const p = b.pos;
      const d = Math.hypot(p.x - r.pos.x, p.z - r.pos.z);
      let c = d - 0.35 * Math.min(bins.get(k), 8);
      if (Math.hypot(p.x - P.pos.x, p.z - P.pos.z) < 1.3) c += 2.5;
      if (Field.inAllianceZone(this.own, p.x)) c -= 0.4;
      if (c < bc) { bc = c; best = b; }
    }
    return best;
  }

  _collect(dt) {
    const r = this.robot;
    if (!this.ball || this.ball.state !== 'field' || this.rethink) {
      const b = this._pickBall();
      if (b !== this.ball) { this.ball = b; this.ballBest = 1e9; this.ballProgressT = 0; }
    }
    const b = this.ball;
    if (!b) {
      this.label = 'No FUEL in reach';
      const s = this._spot();
      this._drive(s.x, s.z, { avoid: true });
      return;
    }
    const d = Math.hypot(b.pos.x - r.pos.x, b.pos.z - r.pos.z);
    // give up on FUEL we're not getting closer to
    if (d < this.ballBest - 0.1) { this.ballBest = d; this.ballProgressT = 0; }
    else if ((this.ballProgressT += dt) > 2.5) { this.blacklist.set(b, this.t + 8); this.ball = null; return; }
    const near = d < 1.6;
    this._drive(b.pos.x, b.pos.z, { speed: near ? this.skill.collect : 1, avoid: !near, arrive: -0.3 });
    // intake down when close, unless that would put it into the player (G415)
    const P = this.player;
    const f = r.forward();
    const toP = { x: P.pos.x - r.pos.x, z: P.pos.z - r.pos.z };
    const dP = Math.hypot(toP.x, toP.z);
    const playerAhead = dP < 1.5 && (toP.x * f.x + toP.z * f.z) / dP > 0.3;
    r.cmd.intake = d < 2.5 && !playerAhead;
    this.label = `Collecting (${r.stored.length}/${r.capacity()})`;
  }

  // ------------------------------------------------------------------ defense
  _defend(dt) {
    const r = this.robot, m = this.match;
    // react to where the player was a moment ago (a juke can beat the defender)
    const seen = this.seen[0];
    const P = { alliance: this.player.alliance, pos: { x: seen.x, z: seen.z }, lastInZone: this.player.lastInZone };
    const pin = this.rules.pinState(r, this.player);
    const gap = this.rules.gap(r, this.player);
    const away = { x: r.pos.x - this.player.pos.x, z: r.pos.z - this.player.pos.z };
    const ad = Math.hypot(away.x, away.z) || 1;
    away.x /= ad; away.z /= ad;
    this.cooldown = Math.max(0, this.cooldown - dt);

    if (!this.backoff && pin.t >= this.skill.pinLimit + 3 * pin.fouls) this.backoff = true;
    if (this.backoff) {
      // G418: move 72in away so the PIN count resets
      if (gap >= PIN_RESET + 0.1) { this.backoff = false; this.cooldown = 0.8; }
      const tx = clamp(r.pos.x + away.x * 2.5, -HALF_L + 0.6, HALF_L - 0.6);
      const tz = clamp(r.pos.z + away.z * 2.5, -HALF_W + 0.6, HALF_W - 0.6);
      this._drive(tx, tz, { face: null });
      this.label = 'Backing off (pin)';
      return;
    }
    // latched intakes stay down: lead with the back so it never reaches inside the player's frame
    const faceP = Math.atan2(-(P.pos.z - r.pos.z), P.pos.x - r.pos.x);
    const face = r.cfg.intake.latched ? faceP + Math.PI : faceP;
    const endGame = m.phaseTime >= TIMING.teleop - TIMING.endgame;
    if (endGame && towerProtected(this.player)) {
      // G420: stay clear of a robot at its TOWER in END GAME
      this._drive(P.pos.x + away.x * 2.2, P.pos.z + away.z * 2.2, { face });
      this.label = 'Holding off (TOWER)';
      return;
    }
    if (this.cooldown > 0) {
      this._drive(r.pos.x + away.x * 0.5, r.pos.z + away.z * 0.5, { face });
      this.label = 'Resetting';
      return;
    }
    const inZone = Field.inAllianceZone(P.alliance, P.pos.x) || P.lastInZone;
    if (inZone) {
      // shove the shooter; slow down before impact so it isn't a ram (G416)
      const d = Math.hypot(P.pos.x - r.pos.x, P.pos.z - r.pos.z);
      this._drive(P.pos.x, P.pos.z, { face, arrive: -1, speed: d < 1.8 ? 0.45 : 1 });
      this.label = pin.active ? `Pinning ${pin.t.toFixed(1)} s` : 'Pushing';
    } else {
      // block the lane between the player and its HUB
      const h = Field.hubCenter(P.alliance);
      let bx = h.x - P.pos.x, bz = h.z - P.pos.z;
      const bd = Math.hypot(bx, bz) || 1;
      bx /= bd; bz /= bd;
      const lead = Math.min(1.4, bd * 0.5);
      const d = Math.hypot(P.pos.x - r.pos.x, P.pos.z - r.pos.z);
      this._drive(P.pos.x + bx * lead, P.pos.z + bz * lead, { face, speed: d < 1.8 ? 0.55 : 1 });
      this.label = 'Blocking';
    }
  }
}
