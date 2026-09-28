// Autonomous routines. G402: drive teams may not control the ROBOT during AUTO, so the robot
// runs one of these pre-programmed routines. Waypoints are in BLUE field coordinates
// (fx from the blue ALLIANCE WALL, fy from the blue drivers' right) and are mirrored for
// the red ALLIANCE and for left-side starts.
import { FIELD_W, ALLIANCE_ZONE_DEPTH, HALF_L, HUB, DEPOT, BUMP, TRENCH, fw } from './constants.js';
import { clamp, wrapAngle } from './util.js';
import { getCustom, customSteps } from './customAutos.js';
import { fitsTrench } from './robotConfigs.js';
import { underTrench } from './nav.js';

export const START_POSITIONS = {
  leftTrench: { name: 'Left Trench', fy: FIELD_W - 0.64 },
  leftBump: { name: 'Left Bump', fy: HUB.fy + HUB.size / 2 + 0.93 },
  hub: { name: 'Hub', fy: HUB.fy },
  rightBump: { name: 'Right Bump', fy: HUB.fy - HUB.size / 2 - 0.93 },
  rightTrench: { name: 'Right Trench', fy: 0.64 },
};
export const START_ORDER = ['leftTrench', 'leftBump', 'hub', 'rightBump', 'rightTrench'];

// Starting pose: BUMPERS overlapping the ROBOT STARTING LINE, not touching the BUMP (G303)
export function startPose(key, robot, alliance, custom = null) {
  let fy;
  if (custom) {
    fy = custom.mirror ? FIELD_W - custom.auto.startFy : custom.auto.startFy;
    fy = clamp(fy, robot.halfW + 0.02, FIELD_W - robot.halfW - 0.02);
  } else fy = START_POSITIONS[key].fy;
  const fx = ALLIANCE_ZONE_DEPTH - 0.02 - robot.halfL;
  const w = fw(fx, fy, alliance);
  return { x: w.x, z: w.z, yaw: alliance === 'blue' ? 0 : Math.PI };
}

// Resolve a custom auto selection: { auto, mirror } or null for built-in routines
export function customSelection(routine, side) {
  const auto = getCustom(routine);
  return auto ? { auto, mirror: side === 'mirror' } : null;
}

// The best AUTO found for each robot (tools/auto-search.mjs plays candidates headless against
// each opponent and keeps the one that scores the most AUTO FUEL). Blue field coordinates,
// A trip either sweeps one line
// of the NEUTRAL ZONE FUEL (out through a TRENCH or over a BUMP, sweep from fy a to b along
// fx, home the same way on that side, shooting once back in the ALLIANCE ZONE), or clears
// the DEPOT. Plans are in absolute blue coordinates (not mirrored). Robot BUMPERS may reach
// past the CENTER LINE, never fully across it.
export const BEST_AUTOS = {
  // 110 AUTO FUEL on average: three trips, working in toward the CENTER LINE (it turns its back
  // to the HUB to shoot)
  2910: { start: 'leftTrench', preload: 'stand', trips: [
    { out: 'trench', fx: 7.53, a: 6.44, b: 3.36, speed: 0.42, home: 'bump', shootAt: [2.31, 1.5] },
    { out: 'trench', fx: 7.88, a: 2.02, b: 6.7, speed: 0.52, home: 'trench', shootAt: [2.74, 6.2] },
    { out: 'trench', fx: 8.13, a: 6.09, b: 4.7, speed: 0.84, home: 'bump', shootAt: [3.21, 6.48] },
  ] },
  // 152: its 88-FUEL hopper sweeps the line nearest the CENTER LINE, then a second long pass
  4414: { start: 'rightTrench', preload: 'stand', trips: [
    { out: 'trench', fx: 8.4, a: 2.32, b: 6.7, speed: 0.53, home: 'bump', shootAt: [2.21, 5.46] },
    { out: 'bump', fx: 7.32, a: 6.65, b: 3.95, speed: 0.47, home: 'bump', shootAt: [2.95, 5.72] },
  ] },
  // 66: clears the DEPOT while shooting, then short slow passes (it only holds 12 and intakes
  // slowly) with a trip home to shoot after each
  // 145: a sweep in from the right, a second from the left, then the DEPOT
  971: { start: 'rightBump', preload: 'move', trips: [
    { out: 'trench', fx: 8.21, a: 1.76, b: 4.14, speed: 0.39, home: 'bump', shootAt: [2.96, 5.55] },
    { out: 'trench', fx: 8.41, a: 6.51, b: 4.77, speed: 0.58, home: 'bump', shootAt: [2.63, 5.65] },
    { depot: true, speed: 0.56 },
  ] },
  // 123: two long sweeps, one each way (2910's pattern)
  1678: { start: 'leftTrench', preload: 'stand', trips: [
    { out: 'trench', fx: 7.53, a: 6.44, b: 3.36, speed: 0.42, home: 'bump', shootAt: [2.31, 1.5] },
    { out: 'trench', fx: 7.88, a: 2.02, b: 6.7, speed: 0.52, home: 'trench', shootAt: [2.74, 6.2] },
  ] },
  // 108: a short slow sweep near the wall, then a fast pass across
  1690: { start: 'rightTrench', preload: 'move', trips: [
    { out: 'trench', fx: 8.42, a: 1.35, b: 2.57, speed: 0.32, home: 'bump', shootAt: [2.77, 2.44] },
    { out: 'bump', fx: 7.54, a: 1.51, b: 5.27, speed: 0.87, home: 'trench', shootAt: [2.96, 5.6] },
  ] },
  // 174: over the BUMP both ways (too tall for the TRENCH), two sweeps, then the DEPOT twice
  4946: { start: 'rightBump', preload: 'move', trips: [
    { out: 'bump', fx: 7.66, a: 2.17, b: 4.03, speed: 0.72, home: 'bump', shootAt: [3.2, 2.32] },
    { out: 'bump', fx: 7.44, a: 2.3, b: 5.7, speed: 0.67, home: 'bump', shootAt: [2.42, 6.47] },
    { depot: true, speed: 0.49 },
    { depot: true, speed: 0.35 },
  ] },
  // Set by reasoning, not searched: the NEUTRAL ZONE FUEL is 6 rows deep on each side of the
  // CENTER LINE (fx ~7.4-8.2) across fy 1.5-6.6, so a sweep along fy at fx 7.8 takes ~30 FUEL/m.
  // 139: 3928 holds 100+ but its spindexer feeds ~8.5/s, so one long sweep over the BUMPS, as much
  // as it can shoot before the buzzer, turret firing on the way home
  3928: { start: 'rightBump', preload: 'move', trips: [
    { out: 'bump', fx: 7.8, a: 1.6, b: 6.4, speed: 0.5, home: 'bump', shootAt: [2.6, 5.5] },
  ] },
  // 38: 341 holds 7 and shoots while it intakes, so the DEPOT first (in the ALLIANCE ZONE, shooting
  // all the way), then short trips under the TRENCH (intake lowered) to the nearest FUEL
  341: { start: 'leftTrench', preload: 'move', trips: [
    { depot: true, speed: 0.35 },
    { out: 'trench', fx: 7.9, a: 6.4, b: 5.6, speed: 0.5, home: 'trench', shootAt: [3.2, 6.9] },
    { out: 'trench', fx: 7.9, a: 6.4, b: 5.6, speed: 0.5, home: 'trench', shootAt: [3.2, 6.9] },
  ] },
  // 92: 4930 holds 41 and empties it in ~2 s, so short sweeps (a full load each) over the nearest
  // BUMP, turning to face the HUB just past it
  4930: { start: 'rightBump', preload: 'stand', trips: [
    { out: 'bump', fx: 7.8, a: 1.6, b: 2.9, speed: 0.55, home: 'bump', shootAt: [2.9, 2.4] },
    { out: 'bump', fx: 7.8, a: 2.9, b: 4.0, speed: 0.55, home: 'bump', shootAt: [2.9, 2.4] },
    { out: 'bump', fx: 7.8, a: 1.6, b: 2.9, speed: 0.6, home: 'bump', shootAt: [2.9, 2.4] },
  ] },
  8793: { start: 'leftTrench', preload: 'move', trips: [
    { depot: true, speed: 0.44 },
    { out: 'trench', fx: 8.15, a: 6.16, b: 5.18, speed: 0.42, home: 'trench', shootAt: [2.3, 6.2] },
    { out: 'bump', fx: 8.07, a: 6.04, b: 5.16, speed: 0.44, home: 'trench', shootAt: [3.38, 5.58] },
    { out: 'bump', fx: 8.23, a: 5.79, b: 4.67, speed: 0.72, home: 'bump', shootAt: [2.78, 5.92] },
  ] },
};

// The same plan run from the other side of the field. DEPOT runs are dropped (there's only
// one DEPOT, on the left).
const FLIP_START = { leftTrench: 'rightTrench', rightTrench: 'leftTrench', leftBump: 'rightBump', rightBump: 'leftBump', hub: 'hub' };
export function mirrorPlan(plan) {
  return {
    ...plan,
    start: FLIP_START[plan.start],
    trips: plan.trips.filter((t) => !t.depot).map((t) => ({ ...t, a: FIELD_W - t.a, b: FIELD_W - t.b, shootAt: [t.shootAt[0], FIELD_W - t.shootAt[1]] })),
  };
}

// Starting spots for one ALLIANCE's robots, so no two start on top of each other. entries (in
// driver station order): { robot (key), auto, start, plan? }. Each gets the spot it wants if
// it's free; a 'best' plan runs mirrored if only the other side is free; otherwise the nearest
// free spot. Custom autos start where they were drawn. Returns [{ start, plan }].
export function assignStarts(entries) {
  const taken = new Set();
  const nearest = (fy) => START_ORDER.reduce((a, k) => (Math.abs(START_POSITIONS[k].fy - fy) < Math.abs(START_POSITIONS[a].fy - fy) ? k : a));
  const out = entries.map((e) => {
    const custom = getCustom(e.auto);
    if (custom) { taken.add(nearest(custom.startFy)); return { start: null, plan: null, custom: true }; }
    return null;
  });
  entries.forEach((e, i) => {
    if (out[i]) return;
    const plan = e.auto === 'best' ? e.plan || BEST_AUTOS[e.robot] : null;
    const tries = plan ? [[plan.start, plan], [FLIP_START[plan.start], mirrorPlan(plan)]] : [[e.start, null]];
    let pick = tries.find(([k]) => !taken.has(k));
    if (!pick) {
      // nothing it planned for is free: the nearest free spot (a plan then drives over to its lane)
      const want = START_POSITIONS[tries[0][0]].fy;
      const free = START_ORDER.filter((k) => !taken.has(k)).sort((a, b) => Math.abs(START_POSITIONS[a].fy - want) - Math.abs(START_POSITIONS[b].fy - want));
      pick = [free[0] || tries[0][0], plan];
    }
    taken.add(pick[0]);
    out[i] = { start: pick[0], plan: pick[1] };
  });
  return out;
}

// Lanes to and from the NEUTRAL ZONE on the right side (low fy); mirrored for the left
const ROUTE_FY = { trench: 0.64, bump: HUB.fy - HUB.size / 2 - BUMP.width / 2 };

export class AutoRunner {
  constructor(robot, routine, startKey, alliance, custom = null, plan = null) {
    this.robot = robot;
    this.alliance = alliance;
    this.plan = plan || (routine === 'best' ? BEST_AUTOS[robot.cfg.key] : null);
    // built-in routines are authored for right-side starts and mirrored; plans are absolute
    this.left = !custom && !this.plan && START_POSITIONS[startKey].fy > FIELD_W / 2 + 0.01;
    this.steps = custom ? customSteps(custom.auto, custom.mirror, robot.cfg.drive.maxSpeed) : this._build(routine);
    this.i = 0;
    this.stepT = 0;
    this.done = false;
  }

  // mirror for start side (routines are authored for the right side / low fy)
  P(fx, fy, sideMirror = true) {
    const y = sideMirror && this.left ? FIELD_W - fy : fy;
    return fw(fx, y, this.alliance);
  }

  _build(routine) {
    const r = this.robot;
    const small = r.maxCapacity() < 20; // hopperless: pass while collecting
    const hl = r.halfL;
    const steps = [];
    const shootPreload = () => steps.push({ type: 'shoot', timeout: 3.0 });
    const sweep = (fxLine, back) => {
      // through the TRENCH into the NEUTRAL ZONE, sweep across the FUEL, back over the BUMP
      const out = fitsTrench(r.cfg) ? ROUTE_FY.trench : ROUTE_FY.bump; // over the BUMP if it's too tall
      steps.push({ type: 'drive', pts: [[3.2, out], [5.6, out], [fxLine - 0.2, 1.25]], intake: true, speed: 1.0 });
      steps.push({ type: 'drive', pts: [[fxLine, 1.45], [fxLine, 4.4]], intake: true, shoot: small, speed: small ? 0.75 : 0.55 });
      steps.push({ type: 'drive', pts: [[6.3, 3.0], [5.2, 2.55], [3.3, 2.4]], intake: small, shoot: small ? true : 'hub', speed: back ? 0.9 : 1.0 });
      steps.push({ type: 'shoot', timeout: 2.5 });
    };
    if (routine === 'best' && this.plan) return this._planSteps(this.plan);
    switch (routine) {
      case 'preload':
        shootPreload();
        break;
      case 'depot': {
        shootPreload();
        const dfy = DEPOT.fy;
        steps.push({ type: 'drive', pts: [[2.4, 5.2], [1.5, dfy]], intake: true, speed: 1.0, mirror: false, face: Math.PI });
        const dx = hl + r.cfg.intake.reach + 0.04; // intake bar against the ALLIANCE WALL
        steps.push({ type: 'drive', pts: [[dx, dfy - 0.3]], intake: true, speed: 0.5, mirror: false, face: Math.PI });
        steps.push({ type: 'drive', pts: [[dx, dfy + 0.3]], intake: true, speed: 0.3, mirror: false, face: Math.PI });
        steps.push({ type: 'drive', pts: [[2.6, 5.0]], intake: true, shoot: 'hub', speed: 0.8, mirror: false });
        steps.push({ type: 'shoot', timeout: 4 });
        break;
      }
      case 'sweep':
        shootPreload();
        sweep(7.55, false);
        break;
      case 'sweep2':
        shootPreload();
        sweep(7.55, false);
        sweep(7.85, true);
        break;
      case 'climb':
        shootPreload();
        steps.push({ type: 'climb' });
        break;
      case 'defendStage':
        // shoot the preload, then over the BUMP to wait in the middle of the field, still on our
        // side of the CENTER LINE (G403), ready to play defense when TELEOP starts
        shootPreload();
        steps.push({ type: 'drive', pts: [[3.3, 2.5], [5.9, 2.5], [6.9, 4.0]], intake: false, speed: 0.9 });
        break;
      default:
        break;
    }
    return steps;
  }

  // Steps for a BEST_AUTOS-style plan (see above)
  _planSteps(plan) {
    const r = this.robot;
    const small = r.maxCapacity() < 20;
    const bps = r.cfg.shooter.bps;
    const maxFx = HALF_L + r.halfW - 0.08; // BUMPERS past the CENTER LINE but not fully across
    const steps = [];
    // a robot too tall for the TRENCH goes over the BUMP instead
    const lane = (kind, fy) => { const k = fitsTrench(r.cfg) ? kind : 'bump'; return fy > FIELD_W / 2 ? FIELD_W - ROUTE_FY[k] : ROUTE_FY[k]; };
    const shootAll = () => steps.push({ type: 'shoot', timeout: r.maxCapacity() / bps + 1.2 });
    if (plan.preload === 'stand') steps.push({ type: 'shoot', timeout: 1.5 });
    let first = plan.preload !== 'stand';
    for (const trip of plan.trips) {
      if (trip.depot) {
        // along the ALLIANCE WALL through the DEPOT, intake and shooter both running
        const dfy = DEPOT.fy, dx = r.halfL + r.cfg.intake.reach + 0.04;
        steps.push({ type: 'drive', pts: [[2.4, dfy - 1.0], [dx + 0.3, dfy - 0.7]], intake: true, shoot: 'hub', speed: 1.0 });
        steps.push({ type: 'drive', pts: [[dx, dfy - 0.45], [dx, dfy + 0.5]], intake: true, shoot: 'hub', speed: trip.speed ?? 0.35, face: Math.PI / 2 });
        steps.push({ type: 'drive', pts: [[2.2, dfy]], intake: true, shoot: 'hub', speed: 0.6 });
        if (!small) shootAll();
        first = false;
        continue;
      }
      const fx = Math.min(trip.fx, maxFx);
      const outY = lane(trip.out, trip.a), homeY = lane(trip.home, trip.b);
      steps.push({ type: 'drive', pts: [[3.1, outY], [5.9, outY], [fx - 0.35, trip.a]], intake: true, shoot: first ? 'hub' : false, speed: 1.0 });
      first = false;
      steps.push({ type: 'drive', pts: [[fx, trip.a], [fx, trip.b]], intake: true, speed: trip.speed });
      steps.push({ type: 'drive', pts: [[6.0, homeY], [3.2, homeY], trip.shootAt], intake: small, shoot: 'hub', speed: 1.0 });
      shootAll();
    }
    return steps;
  }

  // whether what's left of this route passes under a TRENCH arm
  _trenchAhead() {
    if (this._taI === this.i) return this._ta;
    let under = false;
    for (let k = this.i; k < this.steps.length && !under; k++) {
      const s = this.steps[k];
      if (s.type !== 'drive') continue;
      const pts = s.pts.map(([fx, fy]) => this.P(fx, fy, s.mirror !== false));
      for (let j = 0; j + 1 < pts.length && !under; j++) {
        const a = pts[j], b = pts[j + 1], n = Math.ceil(Math.hypot(b.x - a.x, b.z - a.z) / 0.25);
        for (let q = 0; q <= n && !under; q++) under = underTrench(a.x + ((b.x - a.x) * q) / n, a.z + ((b.z - a.z) * q) / n);
      }
    }
    this._taI = this.i; this._ta = under;
    return under;
  }

  update(dt) {
    const r = this.robot;
    const cmd = r.cmd;
    cmd.vx = 0; cmd.vz = 0; cmd.omega = 0;
    cmd.intake = false; cmd.shoot = false; cmd.pass = false; cmd.outtake = false;
    // an intake that stows folded up (8793's) comes down on the way under a TRENCH
    cmd.lower = !!r.cfg.intake.fold && underTrench(r.pos.x, r.pos.z, r.halfL + 1.2);
    if (this.done || this.i >= this.steps.length) {
      this.done = true;
      return;
    }
    const s = this.steps[this.i];
    this.stepT += dt;
    const next = () => { this.i++; this.stepT = 0; this.pi = 0; };
    if (s.type === 'shoot') {
      cmd.shoot = true;
      // too close to the HUB for this shooter: back away toward our ALLIANCE WALL
      if (/range/.test(r.status)) cmd.vx = (this.alliance === 'blue' ? -1 : 1) * 1.2;
      if (r.stored.length === 0 || this.stepT > s.timeout) next();
      return;
    }
    if (s.type === 'wait') {
      if (this.stepT >= s.t) next();
      return;
    }
    if (s.type === 'climb') {
      if (r.climbState === 'none' && !s.requested) {
        // approach the climb pose, then request the climb
        const p = r.climbPose();
        const dx = p.x - r.pos.x, dz = p.z - r.pos.z;
        const d = Math.hypot(dx, dz);
        if (d > 1.0) {
          const sp = Math.min(r.cfg.drive.maxSpeed * 0.8, d * 2);
          cmd.vx = (dx / d) * sp; cmd.vz = (dz / d) * sp;
          cmd.omega = clamp(4 * wrapAngle(p.yaw - r.yaw), -5, 5);
        } else {
          r.requestClimb();
          s.requested = true;
        }
      }
      return;
    }
    if (s.type === 'drive') {
      if (this.pi === undefined) this.pi = 0;
      const pts = s.pts.map(([fx, fy]) => this.P(fx, fy, s.mirror !== false));
      const tgt = pts[this.pi];
      const last = this.pi === pts.length - 1;
      const dx = tgt.x - r.pos.x, dz = tgt.z - r.pos.z;
      const d = Math.hypot(dx, dz);
      let remain = d;
      for (let k = this.pi; k + 1 < pts.length; k++) remain += Math.hypot(pts[k + 1].x - pts[k].x, pts[k + 1].z - pts[k].z);
      const vmax = r.cfg.drive.maxSpeed * (s.speed ?? 1);
      const sp = Math.min(vmax, Math.sqrt(2 * 6 * Math.max(0, remain - 0.02)) + 0.05);
      if (d > 1e-3) { cmd.vx = (dx / d) * sp; cmd.vz = (dz / d) * sp; }
      // heading: face the direction of travel (intake first) unless told otherwise
      let face = s.face !== undefined ? (this.alliance === 'blue' ? s.face : s.face + Math.PI) : Math.atan2(-dz, dx);
      if (this.left && s.face === undefined) face = Math.atan2(-dz, dx);
      if (d > 0.15 || s.face !== undefined) cmd.omega = clamp(5 * wrapAngle(face - r.yaw), -r.cfg.drive.maxOmega, r.cfg.drive.maxOmega);
      cmd.intake = !!s.intake;
      // Under a TRENCH later on this route: stop taking FUEL in once the load is nearly up to the
      // arm (it would bulge the net or hold the lid up too high to get under), and if it's still
      // too tall right before the arm, stop and spit FUEL out until it fits.
      if (fitsTrench(r.cfg) && this._trenchAhead()) {
        if (r.growTop > TRENCH.clearHeight - 0.06) cmd.intake = false;
        const ahead = [0.4, 0.8, 1.2].some((k) => d > 1e-3 && underTrench(r.pos.x + (dx / d) * k, r.pos.z + (dz / d) * k, Math.max(r.halfL, r.halfW)));
        if (ahead && !r.fitsTrenchLowered()) { cmd.vx = 0; cmd.vz = 0; cmd.intake = false; cmd.outtake = true; cmd.shoot = false; return; }
      }
      // 'hub' = shoot on the move only once back in our ALLIANCE ZONE (don't pass the FUEL away)
      cmd.shoot = s.shoot === 'hub' ? r.lastInZone : !!s.shoot;
      if ((last && d < 0.1) || (!last && d < 0.35)) {
        if (last) next();
        else this.pi++;
      }
      if (this.stepT > (s.timeout ?? 8)) next();
    }
  }
}
