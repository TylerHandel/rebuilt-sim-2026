// Autonomous routines. G402: drive teams may not control the ROBOT during AUTO, so the robot
// runs one of these pre-programmed routines. Waypoints are in BLUE field coordinates
// (fx from the blue ALLIANCE WALL, fy from the blue drivers' right) and are mirrored for
// the red ALLIANCE and for left-side starts.
import { FIELD_W, ALLIANCE_ZONE_DEPTH, HALF_L, HUB, DEPOT, BUMP, TRENCH, fw } from './constants.js';
import { clamp, wrapAngle } from './util.js';
import { getCustom, customSteps } from './customAutos.js';
import { fitsTrench } from './robotConfigs.js';
import { underTrench } from './nav.js';
import { TEAM_AUTOS } from './teamAutos.js';
import { holdOffFoes } from './rules.js';
import { step6328 } from './auto6328.js';

export const START_POSITIONS = {
  leftTrench: { name: 'Left Trench', fy: FIELD_W - 0.64 },
  leftBump: { name: 'Left Bump', fy: HUB.fy + HUB.size / 2 + 0.93 },
  hub: { name: 'Hub', fy: HUB.fy },
  rightBump: { name: 'Right Bump', fy: HUB.fy - HUB.size / 2 - 0.93 },
  rightTrench: { name: 'Right Trench', fy: 0.64 },
};
export const START_ORDER = ['leftTrench', 'leftBump', 'hub', 'rightBump', 'rightTrench'];

// Starting pose: BUMPERS overlapping the ROBOT STARTING LINE, not touching the BUMP (G303). A
// team's own auto starts where theirs does (its startFx, as far past the line as the BUMPERS still
// overlap it: 8793's under the TRENCH's arm, its stowed intake behind it), facing as theirs does
// (its startYaw, deg in blue field coordinates: their trajectory's first heading), or else
// forward, toward the NEUTRAL ZONE.
export function startPose(key, robot, alliance, custom = null) {
  let fy, fx = ALLIANCE_ZONE_DEPTH - 0.02 - robot.halfL, yaw = 0;
  if (custom) {
    fy = custom.mirror ? FIELD_W - custom.auto.startFy : custom.auto.startFy;
    fy = clamp(fy, robot.halfW + 0.02, FIELD_W - robot.halfW - 0.02);
    if (custom.auto.startFx !== undefined) fx = clamp(custom.auto.startFx, ALLIANCE_ZONE_DEPTH - robot.halfL - 0.02, ALLIANCE_ZONE_DEPTH + robot.halfL - 0.05);
    if (custom.auto.startYaw !== undefined) yaw = ((custom.mirror ? -1 : 1) * custom.auto.startYaw * Math.PI) / 180;
  } else fy = START_POSITIONS[key].fy;
  const w = fw(fx, fy, alliance);
  return { x: w.x, z: w.z, yaw: wrapAngle(yaw + (alliance === 'blue' ? 0 : Math.PI)) };
}

// A timed trajectory's stretch k (chain c: AutoRunner._chain) at time tau: w, how far through its
// time, and f, how far along its length, at an even acceleration from its start speed to its end's
function along(c, k, tau) {
  const h = c.T[k + 1] - c.T[k];
  const w = h > 1e-6 ? clamp((tau - c.T[k]) / h, 0, 1) : 1;
  const v0 = c.V[k], v1 = c.V[k + 1], vm = (v0 + v1) / 2;
  return { w, f: vm > 1e-6 ? (v0 * w + 0.5 * (v1 - v0) * w * w) / vm : w };
}

// a heading in blue field coordinates (deg) as the robot's yaw on this ALLIANCE's side
const yawOf = (deg, alliance) => wrapAngle((deg * Math.PI) / 180 + (alliance === 'blue' ? 0 : Math.PI));

// The team's own auto for a robot started at spot key (js/teamAutos.js): the one from the side
// nearest it, or null if the team's code isn't in. avoid: spots already taken.
const nearestSpot = (fy) => START_ORDER.reduce((a, k) => (Math.abs(START_POSITIONS[k].fy - fy) < Math.abs(START_POSITIONS[a].fy - fy) ? k : a));
export function teamAutoFor(robotKey, key, avoid = new Set()) {
  const list = TEAM_AUTOS[robotKey];
  if (!list) return null;
  const want = START_POSITIONS[key]?.fy ?? FIELD_W / 2;
  const free = list.filter((a) => !avoid.has(nearestSpot(a.startFy)));
  return free.sort((a, b) => Math.abs(a.startFy - want) - Math.abs(b.startFy - want))[0] || null;
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
  // 112: 3928 holds 80-120 but its spindexer feeds ~8.5/s, so one long sweep over the BUMPS, as much
  // as it can shoot before the buzzer, turret firing on the way home
  3928: { start: 'rightBump', preload: 'move', trips: [
    { out: 'bump', fx: 7.8, a: 1.6, b: 6.4, speed: 0.5, home: 'bump', shootAt: [2.6, 5.5] },
  ] },
  // 34: 341 holds 7-11 and shoots while it intakes, so the DEPOT first (in the ALLIANCE ZONE, shooting
  // all the way), then short trips under the TRENCH (intake lowered) to the nearest FUEL
  341: { start: 'leftTrench', preload: 'move', trips: [
    { depot: true, speed: 0.35 },
    { out: 'trench', fx: 7.9, a: 6.4, b: 5.6, speed: 0.5, home: 'trench', shootAt: [3.2, 6.9] },
    { out: 'trench', fx: 7.9, a: 6.4, b: 5.6, speed: 0.5, home: 'trench', shootAt: [3.2, 6.9] },
  ] },
  // 90: 4930 holds 52 and empties it in ~2 s, so short sweeps (a full load each) over the nearest
  // BUMP, turning to face the HUB just past it
  4930: { start: 'rightBump', preload: 'stand', trips: [
    { out: 'bump', fx: 7.8, a: 1.6, b: 2.9, speed: 0.55, home: 'bump', shootAt: [2.9, 2.4] },
    { out: 'bump', fx: 7.8, a: 2.9, b: 4.0, speed: 0.55, home: 'bump', shootAt: [2.9, 2.4] },
    { out: 'bump', fx: 7.8, a: 1.6, b: 2.9, speed: 0.6, home: 'bump', shootAt: [2.9, 2.4] },
  ] },
  // 102: 1706 holds 64 and both turrets fire ~19/s on the move, so two full-load sweeps, one on
  // each half of the line: out under the TRENCH and home over the BUMP, then back out over the BUMP
  // and home under the other TRENCH (the DEPOT after that doesn't fit in the time)
  1706: { start: 'rightTrench', preload: 'move', trips: [
    { out: 'trench', fx: 7.8, a: 1.6, b: 3.9, speed: 0.5, home: 'bump', shootAt: [2.8, 2.4] },
    { out: 'bump', fx: 7.8, a: 4.1, b: 6.4, speed: 0.5, home: 'trench', shootAt: [2.8, 6.4] },
  ] },
  // Set by their code, not searched: an approximation of 6328's Kachow Salesman from LEFT_BUMP (a
  // sweep of the NEUTRAL ZONE's near half, home over the left BUMP to launch, then again). Their own
  // AUTOs (js/teamAutos.js, run live by js/auto6328.js) take its place; this is what a plan for it
  // gives, for comparison (tools/team-auto-eval.mjs)
  6328: { start: 'leftBump', preload: 'move', trips: [
    { out: 'bump', fx: 7.8, a: 6.4, b: 4.4, speed: 0.5, home: 'bump', shootAt: [3.0, 5.6] },
    { out: 'bump', fx: 7.5, a: 6.4, b: 4.6, speed: 0.5, home: 'bump', shootAt: [3.0, 5.6] },
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

// 2056: set by reasoning, not searched: 1690's plan (the robot 2056's follows, its turret likewise at
// the back): a short slow sweep near the wall out through the TRENCH, then a fast pass across out over
// the BUMP and home through the TRENCH. (Kept out of the BEST_AUTOS literal so it merges on its own.)
BEST_AUTOS[2056] = { start: 'rightTrench', preload: 'move', trips: [
  { out: 'trench', fx: 8.42, a: 1.35, b: 2.57, speed: 0.32, home: 'bump', shootAt: [2.77, 2.44] },
  { out: 'bump', fx: 7.54, a: 1.51, b: 5.27, speed: 0.87, home: 'trench', shootAt: [2.96, 5.6] },
] };

// Starting spots for one ALLIANCE's robots, so no two start on top of each other. entries (in
// driver station order): { robot (key), auto, start, plan? }. Each gets the spot it wants if
// it's free; 'best' on a robot whose team's code is in runs the team's own auto from the side it
// starts on (another side's if that spot's taken); a 'best' plan runs mirrored if only the other
// side is free; otherwise the nearest free spot. Custom autos start where they were drawn.
// Returns [{ start, plan, team }] (team: the team auto to run, as a custom auto).
export function assignStarts(entries) {
  const taken = new Set();
  const nearest = nearestSpot;
  const out = entries.map((e) => {
    const custom = getCustom(e.auto);
    if (custom) { taken.add(nearest(custom.startFy)); return { start: null, plan: null, custom: true }; }
    return null;
  });
  entries.forEach((e, i) => {
    if (out[i] || e.auto !== 'best' || e.plan) return;
    const team = teamAutoFor(e.robot, e.start, taken);
    if (team) { taken.add(nearest(team.startFy)); out[i] = { start: nearest(team.startFy), plan: null, team }; }
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
    this.foes = []; // the other alliance's robots (game.js): its intake out, it doesn't drive into one (G415)
  }

  // mirror for start side (routines are authored for the right side / low fy); never so close to
  // a wall that the robot can't turn there with its intake out (a team's waypoint by the wall is
  // for its own robot's size)
  P(fx, fy, sideMirror = true) {
    const r = this.robot, m = Math.hypot(r.halfL + (r.cfg.intake.reach || 0), r.halfW * 0.5) + 0.03;
    const y = sideMirror && this.left ? FIELD_W - fy : fy;
    return fw(clamp(fx, m, 2 * HALF_L - m), clamp(y, m, FIELD_W - m), this.alliance);
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
    // (best for a robot with neither its team's own AUTO nor a plan found for it: the preload and
    // a sweep of the NEUTRAL ZONE, rather than sitting still)
    if (routine === 'best') routine = 'sweep';
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

  // whether the route passes under a TRENCH arm before the robot next empties (a shoot step
  // unloads it, so a TRENCH after that doesn't limit what it can carry now)
  _trenchAhead() {
    // (what's left of the route: from where it is now, on from the waypoint it's heading for)
    const pi = this.steps[this.i]?.type === 'drive' ? this.pi ?? 0 : 0;
    if (this._taI === this.i && this._taP === pi) return this._ta;
    let under = false;
    for (let k = this.i; k < this.steps.length && !under; k++) {
      const s = this.steps[k];
      if (s.type === 'shoot') break;
      if (s.type !== 'drive') continue;
      let pts = s.pts.map(([fx, fy]) => this.P(fx, fy, s.mirror !== false));
      if (k === this.i) pts = [this.robot.pos.clone(), ...pts.slice(pi)];
      for (let j = 0; j + 1 < pts.length && !under; j++) {
        const a = pts[j], b = pts[j + 1], n = Math.ceil(Math.hypot(b.x - a.x, b.z - a.z) / 0.25);
        for (let q = 0; q <= n && !under; q++) under = underTrench(a.x + ((b.x - a.x) * q) / n, a.z + ((b.z - a.z) * q) / n);
      }
    }
    this._taI = this.i; this._taP = pi; this._ta = under;
    return under;
  }

  update(dt) {
    const r = this.robot;
    const cmd = r.cmd;
    cmd.vx = 0; cmd.vz = 0; cmd.omega = 0;
    cmd.intake = false; cmd.shoot = false; cmd.pass = false; cmd.outtake = false; cmd.prespin = false; cmd.spin = undefined;
    // an intake that stows folded up (8793's) comes down on the way under a TRENCH
    // (only a robot that fits under it: the others go over the BUMPS and leave it stowed)
    cmd.lower = !!r.cfg.intake.fold && fitsTrench(r.cfg) && underTrench(r.pos.x, r.pos.z, r.halfL + 1.2);
    this.elapsed = (this.elapsed || 0) + dt;
    if (this.done || this.i >= this.steps.length) {
      this.done = true;
      return;
    }
    const s = this.steps[this.i];
    this.stepT += dt;
    const next = () => { this.i++; this.stepT = 0; this.pi = 0; this.segT = 0; this.segBudget = undefined; this.lag = 0; this.aimT = undefined; };
    // (a timed auto's wait or held shot keeps the heading its trajectory left it at, turning to it
    // if that turned on the spot: 604's; a chassis-aimed shooter's aim takes over as it shoots)
    if ((s.type === 'shoot' || s.type === 'wait') && s.rot !== undefined) cmd.omega = clamp(s.gains[1] * wrapAngle(yawOf(s.rot, this.alliance) - r.yaw), -r.cfg.drive.maxOmega, r.cfg.drive.maxOmega);
    // (6328's own commands, run live: js/auto6328.js)
    if (s.type === 'team6328') { step6328(this, s, dt, next); return; }
    if (s.type === 'dtp') {
      // a team's drive to point (2910's driveToPoint / driveToPointWhileIntaking): straight at its
      // target at 3 x the distance left, no faster than its speed, turned to the target's heading;
      // done once within its tolerance, or at its time limit (their raceWith a wait), or when its
      // time's up (twice what it would take, and a second: so a robot it's pushing against can't
      // pin it there for the rest of AUTO)
      // (their targets as they are: a pose by the wall, their trenchEnter, is where they aim,
      // within half the robot's width of it, not the waypoints' margin for turning)
      const tgt = fw(s.fx, clamp(s.fy, r.halfW + 0.02, FIELD_W - r.halfW - 0.02), this.alliance);
      const dx = tgt.x - r.pos.x, dz = tgt.z - r.pos.z, d = Math.hypot(dx, dz);
      if (this.stepT <= dt) this.segBudget = 2 * (d / s.vmax + s.vmax / 6) + 1;
      const sp = Math.min(3 * d, s.vmax);
      // (how fast their robot's speed follows the command: 6 m/s^2, an estimate; it carries them
      // round wide at speed, as their sweeps do, the first out past the middle of the NEUTRAL
      // ZONE's FUEL, the second down its near side; the command, kept from step to step)
      const want = d > 1e-3 ? { x: (dx / d) * sp, z: (dz / d) * sp } : { x: 0, z: 0 };
      if (!this.dtpV || this.dtpI !== this.i - 1 && this.dtpI !== this.i) this.dtpV = { x: r.vel.x, z: r.vel.z };
      const ex = want.x - this.dtpV.x, ez = want.z - this.dtpV.z, e = Math.hypot(ex, ez), k = e > 6 * dt ? (6 * dt) / e : 1;
      this.dtpV.x += ex * k; this.dtpV.z += ez * k; this.dtpI = this.i;
      cmd.vx = this.dtpV.x; cmd.vz = this.dtpV.z;
      if (s.face !== undefined) cmd.omega = clamp(5 * wrapAngle((this.alliance === 'blue' ? s.face : s.face + Math.PI) - r.yaw), -r.cfg.drive.maxOmega, r.cfg.drive.maxOmega);
      cmd.intake = !!s.intake;
      // (on to the next command; this step's command stands for the step it ends on)
      if (d < s.tol || (s.limit && this.stepT >= s.limit) || this.stepT >= this.segBudget) next();
      return;
    }
    if (s.type === 'shoot') {
      // (a team auto whose flywheel runs only where its code runs it: up for its shot here)
      if (s.spinPts) cmd.spin = cmd.prespin = true;
      // aiming first (1114's timedShoot): turned to the HUB and stopped, not shooting, until the
      // heading's within 2.75 deg and it's stopped, or for at most s.aim; then shooting for its time
      if (s.aim && this.aimT === undefined) {
        const h = fw(HUB.fx, HUB.fy, this.alliance);
        const err = r.aimErr ?? wrapAngle(Math.atan2(-(h.z - r.pos.z), h.x - r.pos.x) - r.yaw - (r.aimOffset || 0));
        cmd.omega = clamp(5 * err, -r.cfg.drive.maxOmega, r.cfg.drive.maxOmega);
        if ((Math.abs(err) < (2.75 * Math.PI) / 180 && Math.hypot(r.vel.x, r.vel.z) < 0.05) || this.stepT >= s.aim) { this.aimT = this.stepT; this.stepT = 0; }
        return;
      }
      cmd.shoot = true;
      // too close to the HUB for this shooter: back away toward our ALLIANCE WALL
      if (/range/.test(r.status)) cmd.vx = (this.alliance === 'blue' ? -1 : 1) * 1.2;
      // (a team auto's held shot sits out its time, intaking if it says so: 604's)
      if (s.intake) cmd.intake = true;
      if ((r.stored.length === 0 && !s.hold) || this.stepT > s.timeout) next();
      return;
    }
    if (s.type === 'wait') {
      cmd.intake = !!s.intake;
      if (s.until !== undefined ? this.elapsed >= s.until : this.stepT >= s.t) next();
      return;
    }
    if (s.type === 'climb') {
      if (r.climbState === 'none' && !s.requested) {
        // approach the climb pose, then request the climb
        const p = r.climbPose();
        const dx = p.x - r.pos.x, dz = p.z - r.pos.z;
        const d = Math.hypot(dx, dz);
        // (held up on the way longer than the drive there takes, twice over plus 2 s: AUTO's
        // climb given up, the routine over where the robot is)
        if (s.budget === undefined) s.budget = (2 * d) / (r.cfg.drive.maxSpeed * 0.8) + 2;
        if (this.stepT > s.budget) { next(); return; }
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
    if (s.type === 'drive' && s.timed) { this._timed(s, next); return; }
    if (s.type === 'drive') {
      if (this.pi === undefined) this.pi = 0;
      this.segT = (this.segT || 0) + dt;
      // Every point has a time budget, set as the robot heads for it: what a trapezoid at this
      // step's speed and the drive's acceleration takes from where it is, half again, plus a
      // second's grace. Held up longer than that (by another robot, a pile, the DEPOT), it goes
      // on to the next point as if it had got there, and past the step's last to the next step,
      // so no AUTO sits pushing in the middle of the field. (A timed one's are its own times.)
      if (this.segBudget === undefined) {
        const t0 = this.P(s.pts[this.pi][0], s.pts[this.pi][1], s.mirror !== false);
        const L = Math.hypot(t0.x - r.pos.x, t0.z - r.pos.z), v = Math.max(0.3, r.cfg.drive.maxSpeed * (s.pts[this.pi][2] ?? s.speed ?? 1)), a = Math.min(6, r.cfg.drive.maxAccel || 6);
        this.segBudget = 1.5 * (L < (v * v) / a ? 2 * Math.sqrt(L / a) : L / v + v / a) + 1.0;
      }
      let pts = s.pts.map(([fx, fy]) => this.P(fx, fy, s.mirror !== false));
      // (a team's path that runs on into the next drive step without stopping (s.flow: PathPlanner's
      // event zones start and end beside it): that step's points ahead too, up to the next stop)
      const n = pts.length;
      let after = this.i + 1; // (the step after where it stops)
      for (; s.flow && this.steps[after]?.type === 'drive' && this.steps[after].flow; after++) {
        const st = this.steps[after];
        pts = pts.concat(st.pts.map(([fx, fy]) => this.P(fx, fy, st.mirror !== false)));
      }
      const flows = pts.length > n;
      const tgt = pts[this.pi];
      const last = this.pi === pts.length - 1;
      const dx = tgt.x - r.pos.x, dz = tgt.z - r.pos.z;
      const d = Math.hypot(dx, dz);
      let remain = d;
      for (let k = this.pi; k + 1 < pts.length; k++) remain += Math.hypot(pts[k + 1].x - pts[k].x, pts[k + 1].z - pts[k].z);
      const vmax = r.cfg.drive.maxSpeed * (s.pts[this.pi][2] ?? s.speed ?? 1);
      let sp = Math.min(vmax, Math.sqrt(2 * 6 * Math.max(0, remain - 0.02)) + 0.05);
      // slow for the turn at the waypoint ahead (it's left 0.35 m before it): a right angle at
      // half speed, a U-turn at a fifth
      if (!last) {
        const nx = pts[this.pi + 1].x - tgt.x, nz = pts[this.pi + 1].z - tgt.z, nl = Math.hypot(nx, nz);
        if (nl > 1e-3 && d > 1e-3) {
          const turn = Math.acos(clamp((dx * nx + dz * nz) / (d * nl), -1, 1));
          const vc = vmax * Math.max(0.2, 1 - (0.5 * turn) / (Math.PI / 2));
          sp = Math.min(sp, Math.sqrt(vc * vc + 2 * 6 * Math.max(0, d - 0.35)));
        }
      }
      // like a path follower's position feedback: falling behind where it should be by now (pushing
      // over a DEPOT barrier, through a pile) builds up a lag that adds to the command, 3 m/s per m
      const moving = Math.hypot(r.vel.x, r.vel.z);
      this.lag = clamp((this.lag || 0) + (sp - moving) * dt - (moving > 0.8 * sp ? 0.5 * dt : 0), 0, 0.6);
      sp = Math.min(r.cfg.drive.maxSpeed, sp + 3 * this.lag);
      if (d > 1e-3) { cmd.vx = (dx / d) * sp; cmd.vz = (dz / d) * sp; }
      // its intake out, about to reach inside an opponent's FRAME PERIMETER (one crossing right in
      // front of it): not into it (G415 is on the robot driving in); the route goes on round it
      holdOffFoes(r, this.foes, cmd);
      // heading: face the direction of travel (intake first) unless told otherwise
      let face = s.face !== undefined ? (this.alliance === 'blue' ? s.face : s.face + Math.PI) : Math.atan2(-dz, dx);
      if (this.left && s.face === undefined) face = Math.atan2(-dz, dx);
      // a chassis-aimed shooter about to stop and shoot turns to its shot over the last 2.5 m, so
      // it arrives aimed (as the teams' autos do) instead of swinging round where it stops
      const aimAhead = s.face === undefined && r.cfg.shooter.type === 'fixed' && this.steps[after]?.type === 'shoot' && remain < 2.5;
      if (aimAhead) { const h = fw(HUB.fx, HUB.fy, this.alliance); face = Math.atan2(-(h.z - r.pos.z), h.x - r.pos.x) - (r.aimOffset || 0); }
      if (d > 0.15 || s.face !== undefined || aimAhead) cmd.omega = clamp(5 * wrapAngle(face - r.yaw), -r.cfg.drive.maxOmega, r.cfg.drive.maxOmega);
      cmd.intake = !!s.intake;
      // Under a TRENCH later on this route: stop taking FUEL in once the load is nearly up to the
      // arm (it would bulge the net or hold the lid up too high to get under), and if it's still
      // too tall right before the arm, stop and spit FUEL out until it fits.
      if (fitsTrench(r.cfg) && this._trenchAhead()) {
        // once the FUEL nears the arm's height, no more for the rest of this leg (held, so the
        // intake doesn't bob up and down as the load settles)
        if (r.loadNearTrench()) this.fullHold = this.i;
        if (this.fullHold === this.i) cmd.intake = false;
        // an intake that raises the top when it's out (1678's lid) stays in near the arm
        if (r.deployRaisesTop && underTrench(r.pos.x, r.pos.z, Math.max(r.halfL, r.halfW) + 2.0)) cmd.intake = false;
        const ahead = [0.4, 0.8, 1.2].some((k) => d > 1e-3 && underTrench(r.pos.x + (dx / d) * k, r.pos.z + (dz / d) * k, Math.max(r.halfL, r.halfW)));
        if (ahead && !r.fitsTrenchLowered()) { this.fullHold = this.i; cmd.vx = 0; cmd.vz = 0; cmd.intake = false; cmd.outtake = true; cmd.shoot = false; return; }
      }
      // 'hub' = shoot on the move only once back in our ALLIANCE ZONE (don't pass the FUEL away)
      cmd.shoot = s.shoot === 'hub' ? r.lastInZone : !!s.shoot;
      // (a team auto whose flywheel runs only where its code runs it: up to the HUB shot on the way
      // to a point that says so, 1114's AimShooterHub; not aiming otherwise, their ShooterStop)
      if (s.spinPts) cmd.spin = cmd.prespin = !!s.pts[Math.min(this.pi, n - 1)][3];
      // (a team's or the editor's point not reached in its time, held up on the way by another
      // robot or a pile: on to the next as if it had)
      const late = this.segT > this.segBudget;
      if ((last && d < 0.1) || (!last && d < 0.35) || late) {
        if (last) next();
        // (flowing on: the next step from its first point once past this one's last)
        else if (flows && this.pi === n - 1) next();
        else { this.pi++; this.segT = 0; this.segBudget = undefined; }
      }
      if (this.stepT > (s.timeout ?? 8)) next();
    }
  }

  // A timed drive (a team's trajectory: PathPlanner's, Choreo's, Quikplan's), run by the clock as
  // their followers run it: the setpoint moves along the trajectory with the time since it started
  // (each point's dt after the one before, speeding up or slowing down evenly between them, its
  // heading turning evenly to the next point's), and the robot chases it, their feedforward plus
  // their controller's P gains (s.gains: translation per m, rotation per rad) on how far it is from
  // where it should be by now, as PPHolonomicDriveController and Choreo's controller do. Nothing is
  // projected onto the path: fallen behind (held up), the robot is pulled after the setpoint as it
  // moves on; ahead of it, held back. The sum is capped at the drive's top speed and turn rate; its
  // motors, grip and swerve modules limit how fast it gets there (drivetrain.js). Each step is over
  // when its time is up, wherever the robot got to (it moves on to what comes next), as their
  // FollowPathCommand / AutoTrajectory / FollowQuikplan end. A run of timed steps is one trajectory:
  // their paths chain on without a stop where theirs do.
  _timed(s, next) {
    const r = this.robot, cmd = r.cmd, d = r.cfg.drive;
    let c = this.chain;
    if (!c || this.i < c.i0 || this.i >= c.i0 + c.ends.length) c = this.chain = this._chain();
    const tau = this.elapsed - c.t0;
    // (its steps over by now: on to the next, in the same tick; past the last, the step after it)
    while (tau >= c.T[c.ends[this.i - c.i0]]) {
      next();
      if (this.i >= c.i0 + c.ends.length) { this.chain = this.setpoint = null; return; }
    }
    const st = this.steps[this.i], j = this.i - c.i0;
    while (c.k + 2 < c.T.length && c.T[c.k + 1] <= tau) c.k++;
    const k = c.k, h = c.T[k + 1] - c.T[k];
    const { w, f } = along(c, k, tau);
    const v0 = c.V[k], v1 = c.V[k + 1], vm = (v0 + v1) / 2;
    const A = c.C[k], B = c.C[k + 1], L = Math.hypot(B.x - A.x, B.z - A.z);
    const v = h > 1e-6 ? (vm > 1e-6 ? ((L / h) * (v0 + (v1 - v0) * w)) / vm : L / h) : 0;
    const ux = L > 1e-6 ? (B.x - A.x) / L : 0, uz = L > 1e-6 ? (B.z - A.z) / L : 0;
    const [kp, kr] = st.gains;
    const turn = wrapAngle(c.R[k + 1] - c.R[k]);
    // (where it should be by now, how fast, facing which way: the setpoint)
    const sp0 = this.setpoint = { x: A.x + (B.x - A.x) * f, z: A.z + (B.z - A.z) * f, vx: ux * v, vz: uz * v, yaw: wrapAngle(c.R[k] + turn * w) };
    // (PathPlanner's target position trails its trajectory by st.posLag: PathPlannerTrajectory's
    // sample() integrates the speed between two states 0.01 s at a time and leaves the first 0.01 s
    // out, in their Java as in its Python; its speeds and rotation are where they should be)
    if (st.posLag && tau > st.posLag) {
      const t2 = tau - st.posLag;
      let k2 = k;
      while (k2 > 0 && c.T[k2] > t2) k2--;
      const p = along(c, k2, t2), A2 = c.C[k2], B2 = c.C[k2 + 1];
      sp0.x = A2.x + (B2.x - A2.x) * p.f; sp0.z = A2.z + (B2.z - A2.z) * p.f;
    }
    let vx = sp0.vx + kp * (sp0.x - r.pos.x), vz = sp0.vz + kp * (sp0.z - r.pos.z);
    const sp = Math.hypot(vx, vz);
    if (sp > d.maxSpeed) { vx *= d.maxSpeed / sp; vz *= d.maxSpeed / sp; }
    cmd.vx = vx; cmd.vz = vz;
    cmd.omega = clamp((h > 1e-6 ? turn / h : 0) + kr * wrapAngle(sp0.yaw - r.yaw), -d.maxOmega, d.maxOmega);
    // (the point of this step it's on its way to, for what looks ahead along the route)
    const first = j === 0 ? 1 : c.ends[j - 1] + 1;
    this.pi = clamp(k + 1 - first, 0, st.pts.length - 1);
    cmd.intake = !!st.intake;
    // under a TRENCH later on this route: as a drive step's (below), the load kept under the arm
    if (fitsTrench(r.cfg) && this._trenchAhead()) {
      if (r.loadNearTrench()) this.fullHold = this.i;
      if (this.fullHold === this.i) cmd.intake = false;
      if (r.deployRaisesTop && underTrench(r.pos.x, r.pos.z, Math.max(r.halfL, r.halfW) + 2.0)) cmd.intake = false;
      const m = Math.hypot(vx, vz);
      const ahead = m > 0.05 && [0.4, 0.8, 1.2].some((q) => underTrench(r.pos.x + (vx / m) * q, r.pos.z + (vz / m) * q, Math.max(r.halfL, r.halfW)));
      if (ahead && !r.fitsTrenchLowered()) { this.fullHold = this.i; cmd.vx = 0; cmd.vz = 0; cmd.intake = false; cmd.outtake = true; return; }
    }
    cmd.shoot = st.shoot === 'hub' ? r.lastInZone : !!st.shoot;
    if (st.spinPts) cmd.spin = cmd.prespin = !!st.pts[this.pi][3];
  }

  // the trajectory from this step on through the timed steps that flow into it: the robot where it
  // is now, then each point, the time to it (T, s from the start), the speed there (V, m/s) and the
  // heading (R, the robot's yaw); ends: where each step's points end
  _chain() {
    const r = this.robot, C = [{ x: r.pos.x, z: r.pos.z }], T = [0], V = [Math.hypot(r.vel.x, r.vel.z)], R = [r.yaw], ends = [];
    for (let k = this.i; k < this.steps.length; k++) {
      const st = this.steps[k];
      if (st.type !== 'drive' || !st.timed) break;
      st.pts.forEach(([fx, fy], q) => {
        const tp = st.tp[q];
        C.push(this.P(fx, fy, st.mirror !== false));
        T.push(T[T.length - 1] + tp.dt);
        V.push(tp.v);
        R.push(tp.rot === undefined ? R[R.length - 1] : yawOf(tp.rot, this.alliance));
      });
      ends.push(C.length - 1);
      if (!st.flow) break;
    }
    return { i0: this.i, t0: this.elapsed, C, T, V, R, ends, k: 0 };
  }
}
