// Custom autos made in the Auto Editor, saved in localStorage. Points are in BLUE field
// coordinates (fx from the blue ALLIANCE WALL, fy from the blue drivers' right), like the
// built-in routines; red autos are the same path rotated 180 degrees.
import { FIELD_W, ALLIANCE_ZONE_DEPTH } from './constants.js';
import { modelCapacity } from './hopper.js';

const KEY = 'rebuiltSim.customAutos';
export const CUSTOM_PREFIX = 'custom:';

export const SHOOT_MODES = [
  ['off', "Don't shoot"],
  ['hub', 'Once in our zone'],
  ['always', 'Anywhere (pass)'],
];
export const ARRIVE_ACTIONS = [
  ['none', 'Keep driving'],
  ['stop', 'Stop'],
  ['shoot', 'Stop and shoot it all'],
  ['wait1', 'Wait 1 s'],
  ['wait2', 'Wait 2 s'],
  ['wait3', 'Wait 3 s'],
];
export const SPEEDS = [0.3, 0.4, 0.5, 0.6, 0.7, 0.8, 0.9, 1.0];

export function loadAutos() {
  try {
    const list = JSON.parse(localStorage.getItem(KEY) || '[]');
    return Array.isArray(list) ? list.filter((a) => a && a.id && Array.isArray(a.points)) : [];
  } catch { return []; }
}

export function saveAutos(list) {
  try { localStorage.setItem(KEY, JSON.stringify(list)); } catch { /* storage unavailable */ }
}

export function newPoint(fx, fy) {
  return { fx, fy, intake: true, shoot: 'off', speed: 1, action: 'none' };
}

export function newAuto(list, base = null) {
  let n = list.length + 1;
  while (list.some((a) => a.name === `Custom ${n}`)) n++;
  const a = base
    ? JSON.parse(JSON.stringify(base))
    : { startFy: 0.64, shootPreload: true, points: [newPoint(5.8, 0.64), newPoint(7.5, 1.6)] };
  a.id = 'a' + Date.now().toString(36) + Math.floor(Math.random() * 1e4).toString(36);
  a.name = base ? `${base.name} copy` : `Custom ${n}`;
  return a;
}

export function isCustom(key) { return typeof key === 'string' && key.startsWith(CUSTOM_PREFIX); }
export function getCustom(key) {
  if (!isCustom(key)) return null;
  const id = key.slice(CUSTOM_PREFIX.length);
  return loadAutos().find((a) => a.id === id) || null;
}

// Flip left/right (about the field's long axis)
export function mirrorAuto(a) {
  a.startFy = FIELD_W - a.startFy;
  for (const p of a.points) p.fy = FIELD_W - p.fy;
}

export function waitTime(action, t) {
  if (action === 'wait') return t ?? 1;
  const m = /^wait(\d+)$/.exec(action);
  return m ? +m[1] : action === 'stop' ? 0.05 : 0;
}

// Convert to AutoRunner steps. Consecutive points that share segment settings and don't stop
// are merged into one drive so the robot flows through them. (The teams' own autos, js/teamAutos.js,
// also give a point's speed in m/s (v), how long a shoot or wait lasts (t), a wait until a time
// into AUTO (action 'until'), and where the robot starts along the field (startFx). A timed one's,
// a.timed (PathPlanner's, Choreo's, Quikplan's trajectories), give each point's time from the point
// before (dt) and heading (rot, deg, blue), which its drive steps carry in tp: the AutoRunner
// follows those by the clock; the others, as every drive step, give each point a time limit.)
export function customSteps(a, mirror, maxSpeed) {
  const fy = (y) => (mirror ? FIELD_W - y : y);
  const rot = (r) => (r === undefined || r === null ? undefined : mirror ? -r : r);
  const steps = [];
  if (a.shootPreload) steps.push({ type: 'shoot', timeout: a.shootPreload === true ? 3 : a.shootPreload });
  let cur = null;
  let prev = { fx: a.startFx ?? ALLIANCE_ZONE_DEPTH - 0.4, fy: a.startFy };
  const flush = () => { if (cur) { steps.push(cur); cur = null; } };
  for (const p of a.points) {
    const shoot = p.shoot === 'hub' ? 'hub' : p.shoot === 'always';
    // (a team auto's speed is per point: its path speeds up and slows down without stopping; on a
    // timed one, a point's speed can be 0: where its path comes to a stop)
    const hasV = a.timed ? p.v !== undefined : !!p.v;
    const speed = hasV ? Math.min(1, p.v / maxSpeed) : p.speed;
    const same = cur && cur.intake === p.intake && cur.shoot === shoot && (hasV ? cur.perPoint : cur.speed === speed);
    if (!same) {
      flush();
      cur = { type: 'drive', pts: [], intake: p.intake, shoot, speed, mirror: false, timeout: 3, perPoint: hasV };
      // (run by its trajectory's clock, chasing where it should be by now with their controller's
      // gains: each point's time from the one before, heading, speed (m/s) and distance from it)
      if (a.timed) Object.assign(cur, { timed: true, tp: [], gains: a.gains || [5, 5] });
      // (a team's path that runs its events beside it, PathPlanner's: it doesn't stop where they
      // start or end; and one whose flywheel runs only where its code runs it: 1114's)
      if (a.flow) cur.flow = true;
      if (a.spinPts) cur.spinPts = true;
    }
    // (its flywheel up to the HUB shot on the way to this point: spin)
    cur.pts.push(hasV ? (a.spinPts ? [p.fx, fy(p.fy), speed, !!p.spin] : [p.fx, fy(p.fy), speed]) : [p.fx, fy(p.fy)]);
    const ds = Math.hypot(p.fx - prev.fx, p.fy - prev.fy);
    cur.timeout += (2 * ds) / Math.max(0.5, maxSpeed * speed);
    if (cur.timed) cur.tp.push({ dt: p.dt ?? 0, rot: rot(p.rot), v: p.v, ds });
    prev = p;
    // (a timed auto's wait or held shot keeps the heading its trajectory left it at)
    const hold = a.timed && rot(p.rot) !== undefined ? { rot: rot(p.rot), gains: a.gains || [5, 5] } : {};
    // (a team auto's shot that holds its spot for the time it's given, its intake running or not, as
    // 604's Quikplan waits do: hold)
    // (aim: it aims, not shooting, until it's aimed and stopped, for at most that long, then shoots
    // for its time: 1114's timedShoot)
    if (p.action === 'shoot') { flush(); steps.push(p.hold ? { type: 'shoot', timeout: p.t ?? 3, hold: true, intake: !!p.intake, ...hold, ...(p.aim ? { aim: p.aim } : {}), ...(a.spinPts ? { spinPts: true } : {}) } : { type: 'shoot', timeout: p.t ?? 3, ...hold }); }
    else if (p.action === 'until') { flush(); steps.push({ type: 'wait', until: p.t, intake: p.intake, ...hold }); }
    else if (p.action !== 'none') { flush(); steps.push({ type: 'wait', t: waitTime(p.action, p.t), intake: p.intake, ...hold }); }
  }
  flush();
  return steps;
}

// Rough duration estimate for the editor: trapezoid drive profiles + shooting time
export function estimateTime(a, cfg, preload = 8) {
  const d = cfg.drive;
  const acc = 6;
  let t = 0;
  let stored = preload;
  const shoot = () => { t += stored ? 0.5 + stored / cfg.shooter.bps : 0; stored = 0; };
  if (a.shootPreload) shoot();
  let prev = { fx: a.startFx ?? ALLIANCE_ZONE_DEPTH - 0.4, fy: a.startFy };
  for (const p of a.points) {
    const L = Math.hypot(p.fx - prev.fx, p.fy - prev.fy);
    const v = p.v !== undefined ? Math.max(0.3, Math.min(p.v, d.maxSpeed)) : d.maxSpeed * p.speed;
    t += L < (v * v) / acc ? 2 * Math.sqrt(L / acc) : L / v + v / acc;
    if (p.intake) stored = Math.min(modelCapacity(cfg), stored + Math.round(L * 3));
    if (p.shoot !== 'off' && p.fx < ALLIANCE_ZONE_DEPTH) stored = 0;
    if (p.action === 'shoot') shoot();
    t += waitTime(p.action, p.t);
    prev = p;
  }
  return t;
}
