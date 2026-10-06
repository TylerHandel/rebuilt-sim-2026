// What the neural-network driver sees and how its outputs become a robot command.
// Shared by the browser game, the Node training workers and the in-game recorder, so a trained
// network sees exactly the same numbers everywhere.
//
// Everything is in the robot's ALLIANCE frame: the field is point-symmetric, so the red robot's
// view is rotated 180° and one network plays both alliances. "Own" = this robot's alliance.
import { HALF_L, HALF_W, BLUE, other, TIMING } from '../constants.js';
import { Field } from '../field.js';
import { OBSTACLES } from '../nav.js';
import { ROBOT_ORDER } from '../robotConfigs.js';

// v3 appended the teammates and the 2nd/3rd opponents (3v3). v4 appended which robot each one is
// for robots added after v3 (581 on): the one-hots in the blocks above stay the 11 robots v3 knew
// (a newer robot shows as none of them there) and the new robots get slots at the end, room for
// EXTRA_ROBOTS more without changing the layout again. Blocks are only ever appended, so a network
// trained on an older version reads the start of the vector it knows (see policy.js).
export const OBS_VERSION = 4;
export const DECISION_DT = 0.1;            // the network acts 10 times a second
export const ACT_CONT = 3;                 // vx, vz (own frame, fraction of top speed), omega
export const ACT_BIN = ['intake', 'shoot', 'pass', 'outtake'];
export const ACT_DIM = ACT_CONT + ACT_BIN.length;

const RAYS = 16, RAY_MAX = 4;
const EGO_N = 12, EGO_CELL = 0.4;          // FUEL around the robot (robot frame), 4.8 m square
const GX = 16, GZ = 8;                     // FUEL over the whole field (alliance frame)
const NEAR = 8;                            // nearest FUEL positions

// which robot (one-hot): the 11 robots of v3, in their v3 order, then (v4) the ones added since
export const V3_ROBOTS = ['2910', '4414', '8793', '971', '1678', '1690', '4946', '3928', '341', '4930', '1706'];
export const EXTRA_ROBOTS = 8;
export const NEW_ROBOTS = ROBOT_ORDER.filter((k) => !V3_ROBOTS.includes(k));
if (NEW_ROBOTS.length > EXTRA_ROBOTS) throw new Error(`more than ${EXTRA_ROBOTS} robots added since obs v3: grow the observation`);
const NR = V3_ROBOTS.length;
export const OBS_LAYOUT = [
  ['self', 21 + NR], ['hubs', 7], ['match', 16], ['opponent', 13 + NR], ['rays', RAYS],
  ['fuelEgo', EGO_N * EGO_N], ['fuelField', GX * GZ], ['fuelNear', NEAR * 2], ['fuelZones', 3],
  ['teammates', 2 * (13 + NR)], ['opponents2', 2 * (13 + NR)],
  ['robotsNew', 6 * EXTRA_ROBOTS], // self, the nearest opponent, the teammates, the 2nd/3rd opponents
];
export const OBS_DIM_V2 = OBS_LAYOUT.slice(0, 9).reduce((s, [, n]) => s + n, 0);
export const OBS_DIM = OBS_LAYOUT.reduce((s, [, n]) => s + n, 0);

const TOTAL = TIMING.auto + TIMING.teleop;
const sgn = (a) => (a === BLUE ? 1 : -1);
const clip1 = (v) => (v > 1 ? 1 : v < -1 ? -1 : v);

// ray march against the static obstacles + field walls (axis-aligned rectangles)
function rayDist(x, z, dx, dz) {
  let best = RAY_MAX;
  // walls
  if (dx > 1e-6) best = Math.min(best, (HALF_L - x) / dx);
  else if (dx < -1e-6) best = Math.min(best, (-HALF_L - x) / dx);
  if (dz > 1e-6) best = Math.min(best, (HALF_W - z) / dz);
  else if (dz < -1e-6) best = Math.min(best, (-HALF_W - z) / dz);
  for (const o of OBSTACLES) {
    // slab test
    let t0 = 0, t1 = best;
    const ax = [[x, dx, o.x - o.hx, o.x + o.hx], [z, dz, o.z - o.hz, o.z + o.hz]];
    let hit = true;
    for (const [p, d, lo, hi] of ax) {
      if (Math.abs(d) < 1e-9) { if (p < lo || p > hi) { hit = false; break; } continue; }
      let ta = (lo - p) / d, tb = (hi - p) / d;
      if (ta > tb) [ta, tb] = [tb, ta];
      if (ta > t0) t0 = ta;
      if (tb < t1) t1 = tb;
      if (t0 > t1) { hit = false; break; }
    }
    if (hit && t0 < best) best = t0;
  }
  return Math.max(0, best);
}

const dist2 = (a, b) => (a.pos.x - b.pos.x) ** 2 + (a.pos.z - b.pos.z) ** 2;
// others: { foes: [robots], mates: [robots] }, or (1v1) just the other robot or null
function sides(robot, others) {
  if (!others || others.pos) return { foes: others ? [others] : [], mates: [] };
  const by = (l) => [...(l || [])].sort((p, q) => dist2(robot, p) - dist2(robot, q));
  return { foes: by(others.foes), mates: by(others.mates) };
}

// Build the observation vector for `robot` (out: Float32Array(OBS_DIM), reused if given).
// others: { foes, mates } (the other robots on each ALLIANCE), or one opposing robot / null.
export function buildObs(robot, others, match, fuel, out = new Float32Array(OBS_DIM)) {
  const { foes, mates } = sides(robot, others);
  const foe = foes[0] || null;
  const a = robot.alliance, s = sgn(a);
  const cfg = robot.cfg;
  let i = 0;
  const put = (v) => { out[i++] = Number.isFinite(v) ? v : 0; };
  const x = s * robot.pos.x, z = s * robot.pos.z;
  const yaw = robot.yaw + (s > 0 ? 0 : Math.PI);
  const c = Math.cos(yaw), sn = Math.sin(yaw);
  // world-frame vector (own frame) -> robot frame: forward = (cos yaw, -sin yaw)
  const toLocal = (dx, dz) => [dx * c - dz * sn, dx * sn + dz * c];

  // ---- self (21 + robots)
  const turret = cfg.shooter.type !== 'fixed';
  put(x / HALF_L); put(z / HALF_W); put(c); put(sn);
  put((s * robot.vel.x) / 5); put((s * robot.vel.z) / 5); put(robot.omega / 10);
  const n = robot.stored.length, cap = robot.maxCapacity();
  put(n / 60); put(n / cap); put(robot.capacity() / cap);
  put(robot.intakeDeploy); put(robot.hopperDeploy);
  put(robot.flywheel / cfg.shooter.speedMax); put(robot.ready ? 1 : 0); put(robot.shot ? 1 : 0);
  put(robot.inAllianceZone() ? 1 : 0); put(robot.feeding ? 1 : 0);
  put(turret ? robot.turretYaw / Math.PI : 0);
  for (const k of V3_ROBOTS) put(cfg.key === k ? 1 : 0);
  put(cfg.drive.maxSpeed / 5); put(cfg.shooter.bps / 40); put(turret ? 1 : 0);

  // ---- hubs + zone line (7)
  for (const al of [a, other(a)]) {
    const h = Field.hubCenter(al);
    const [lx, lz] = toLocal(s * h.x - x, s * h.z - z);
    put(lx / 8); put(lz / 8); put(Math.hypot(lx, lz) / 8);
  }
  put((s * Field.allianceLineX(a) - x) / HALF_L);

  // ---- match (16)
  const ph = match.phase;
  const auto = ph === 'auto', tele = ph === 'teleop';
  const left = auto ? TOTAL - match.phaseTime : tele ? TIMING.teleop - match.phaseTime : ph === 'autoGap' ? TIMING.teleop : ph === 'pre' ? TOTAL : 0;
  put(auto ? 1 : 0); put(tele ? 1 : 0); put(left / TOTAL);
  put(auto ? (TIMING.auto - match.phaseTime) / TIMING.auto : tele ? (TIMING.teleop - match.phaseTime) / TIMING.teleop : 0);
  const si = match.shiftIndex();
  for (let k = 0; k < 6; k++) put(si === k ? 1 : 0);
  const nc = (al) => { const v = match.hubNextChange(al); return v === null ? 1 : Math.min(1, v / TIMING.shift); };
  put(match.hubActive(a) ? 1 : 0); put(match.hubActive(other(a)) ? 1 : 0);
  put(nc(a)); put(nc(other(a)));
  const mine = match.total(a), theirs = match.total(other(a));
  put(Math.tanh((mine - theirs) / 100)); put(mine / 300);

  // ---- the nearest opponent (13 + robots)
  const robotBlock = (o) => {
    if (!o) { for (let k = 0; k < 13 + NR; k++) put(0); return; }
    const fx = s * o.pos.x, fz = s * o.pos.z;
    const [lx, lz] = toLocal(fx - x, fz - z);
    const fyaw = o.yaw + (s > 0 ? 0 : Math.PI);
    put(1); put(fx / HALF_L); put(fz / HALF_W); put(lx / 8); put(lz / 8); put(Math.hypot(lx, lz) / 8);
    put((s * o.vel.x) / 5); put((s * o.vel.z) / 5); put(Math.cos(fyaw)); put(Math.sin(fyaw));
    put(o.stored.length / 60);
    for (const k of V3_ROBOTS) put(o.cfg.key === k ? 1 : 0);
    put(o.inAllianceZone(a) ? 1 : 0); put(o.inAllianceZone(o.alliance) ? 1 : 0);
  };
  robotBlock(foe);

  // ---- rays (16), robot frame, measured from the robot center
  for (let k = 0; k < RAYS; k++) {
    const ang = yaw + (k / RAYS) * 2 * Math.PI;
    // own-frame direction of a robot-frame angle; back to world for the obstacle map
    const dx = Math.cos(ang), dz = -Math.sin(ang);
    put(rayDist(robot.pos.x, robot.pos.z, s * dx, s * dz) / RAY_MAX);
  }

  // ---- FUEL
  const ego = i; for (let k = 0; k < EGO_N * EGO_N; k++) out[i++] = 0;
  const grid = i; for (let k = 0; k < GX * GZ; k++) out[i++] = 0;
  const near = []; // [d2, lx, lz]
  let zOwn = 0, zMid = 0, zOpp = 0;
  const line = s * Field.allianceLineX(a);
  const half = (EGO_N * EGO_CELL) / 2;
  for (const b of fuel.balls) {
    if (b.state !== 'field' || b.pos.y > 0.5) continue;
    const bx = s * b.pos.x, bz = s * b.pos.z;
    if (bx < line) zOwn++; else if (bx > -line) zOpp++; else zMid++;
    const gx = Math.floor(((bx + HALF_L) / (2 * HALF_L)) * GX), gz = Math.floor(((bz + HALF_W) / (2 * HALF_W)) * GZ);
    if (gx >= 0 && gx < GX && gz >= 0 && gz < GZ) out[grid + gx * GZ + gz] += 1;
    const [lx, lz] = toLocal(bx - x, bz - z);
    if (lx > -half && lx < half && lz > -half && lz < half) {
      const ex = Math.floor((lx + half) / EGO_CELL), ez = Math.floor((lz + half) / EGO_CELL);
      out[ego + ex * EGO_N + ez] += 1;
    }
    const d2 = lx * lx + lz * lz;
    if (near.length < NEAR || d2 < near[near.length - 1][0]) {
      near.push([d2, lx, lz]);
      near.sort((p, q) => p[0] - q[0]);
      if (near.length > NEAR) near.pop();
    }
  }
  for (let k = 0; k < EGO_N * EGO_N; k++) out[ego + k] = Math.min(1, out[ego + k] / 4);
  const L = Math.log1p(20);
  for (let k = 0; k < GX * GZ; k++) out[grid + k] = Math.log1p(out[grid + k]) / L;
  for (let k = 0; k < NEAR; k++) {
    const p = near[k];
    out[i++] = p ? clip1(p[1] / 4) : 1;
    out[i++] = p ? clip1(p[2] / 4) : 0;
  }
  out[i++] = zOwn / 50; out[i++] = zMid / 400; out[i++] = zOpp / 50;
  // ---- teammates (nearest first), then the 2nd and 3rd nearest opponents
  robotBlock(mates[0]); robotBlock(mates[1]);
  robotBlock(foes[1]); robotBlock(foes[2]);
  // ---- (v4) which robot each is, for robots added since v3
  for (const o of [robot, foe, mates[0], mates[1], foes[1], foes[2]]) {
    const j = o ? NEW_ROBOTS.indexOf(o.cfg.key) : -1;
    for (let k = 0; k < EXTRA_ROBOTS; k++) put(k === j ? 1 : 0);
  }
  if (i !== OBS_DIM) throw new Error(`obs size ${i} != ${OBS_DIM}`);
  return out;
}

// Network action (own frame, [-1, 1] continuous + 0/1 buttons) -> robot.cmd (world frame)
export function actionToCmd(robot, act, cmd = {}) {
  const s = sgn(robot.alliance), d = robot.cfg.drive;
  let vx = clip1(act[0]), vz = clip1(act[1]);
  const m = Math.hypot(vx, vz);
  if (m > 1) { vx /= m; vz /= m; }
  cmd.vx = s * vx * d.maxSpeed;
  cmd.vz = s * vz * d.maxSpeed;
  cmd.omega = clip1(act[2]) * d.maxOmega;
  ACT_BIN.forEach((k, j) => { cmd[k] = act[ACT_CONT + j] > 0.5; });
  return cmd;
}

// A driver's robot.cmd (world frame) -> network action, for learning from human driving
export function cmdToAction(robot, cmd, out = new Float32Array(ACT_DIM)) {
  const s = sgn(robot.alliance), d = robot.cfg.drive;
  out[0] = clip1((s * cmd.vx) / d.maxSpeed);
  out[1] = clip1((s * cmd.vz) / d.maxSpeed);
  out[2] = clip1(cmd.omega / d.maxOmega);
  ACT_BIN.forEach((k, j) => { out[ACT_CONT + j] = cmd[k] ? 1 : 0; });
  return out;
}
