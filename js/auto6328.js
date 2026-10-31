// 6328's own AUTOs run live (tools/autos6328.py writes their programs into js/teamAutos.js): their
// commands from Mechanical-Advantage/RobotCode2026Public (AutoBuilder, AutoCommands, DriveToPose,
// DriveTrajectory, SalesmanCommands, SalesmanSolver, SalesAssociate, ObjectDetection), one step of
// js/auto.js's AutoRunner (type 'team6328') running a program of them in turn. Their poses are in
// blue field coordinates for the left side (the right's mirrored in the program); the robot's own
// pose is read in the same frame for either ALLIANCE (their AllianceFlipUtil: red turned round).
import { HALF_L, HALF_W, FIELD_L, FIELD_W } from './constants.js';
import { clamp, wrapAngle } from './util.js';

const IN = 0.0254;
// their DriveConstants and FieldConstants (Darwin; the welded official layout)
const D = {
  fullApothemX: (33 * IN) / 2, fullBaseRadius: Math.hypot((33 * IN) / 2, (34 * IN) / 4), driveBaseRadius: Math.hypot((21.25 * IN) / 2, (22.25 * IN) / 2),
  maxLinearSpeed: 4.2, intakeReferenceX: (33 * IN) / 2, intakeWidth: 27.5 * IN, intakeFarX: (26.5 * IN) / 2 + 12 * IN, frameWidthY: 27.5 * IN,
};
const F = { starting: 4.0219, neutralZoneNear: 5.2225, neutralZoneFar: 11.3185, center: FIELD_L / 2, hubX: 4.6188, midY: FIELD_W / 2, leftBumpMiddle: 5.5585, rightBumpStart: 3.4376, leftBumpEnd: 4.6314, fuelPoolLeft: [8.2705, 6.3446], fuelPoolRight: [8.2705, 1.7244] };
// their launch poses (AutoFieldConstants.Launch: stationary, the launcher aimed at the HUB)
const LAUNCH = { leftBump: [3.3219, 5.5585, 130.3973], rightBump: [3.3219, FIELD_W - 5.5585, -130.3973], leftTrench: [3.0504, 7.1297, 116.8714], rightTrench: [3.0504, FIELD_W - 7.1297, -116.8714] };
const AUTO_START = { intake: 1.0, spin: 1.5 }; // (RobotContainer: the intake runs 1 s in, the flywheel tracks 1.5 s in)
const LAUNCH_TIME = 3.0, DEPOT_LAUNCH_TIME = 1.5, INDEX_T = 0.35 + 1.2; // AutoBuilder; CompactingCommands' delay and intake timeout

// the robot's pose in their frame (blue field coordinates, its ALLIANCE's side turned to blue's)
function own(r) {
  const b = r.alliance === 'blue';
  return { x: b ? r.pos.x + HALF_L : HALF_L - r.pos.x, y: b ? HALF_W - r.pos.z : HALF_W + r.pos.z, h: wrapAngle(b ? r.yaw : r.yaw - Math.PI), vx: b ? r.vel.x : -r.vel.x, vy: b ? -r.vel.z : r.vel.z, w: r.omega ?? 0 };
}
// a field-relative command in their frame to the sim's
function drive(r, vx, vy, w) {
  const b = r.alliance === 'blue', d = r.cfg.drive, s = Math.hypot(vx, vy), k = s > d.maxSpeed ? d.maxSpeed / s : 1;
  r.cmd.vx = (b ? vx : -vx) * k; r.cmd.vz = (b ? -vy : vy) * k;
  r.cmd.omega = clamp(w, -d.maxOmega, d.maxOmega);
}

// ------------------------------------------------------------------ DriveToPose
// Their DriveToPose: a trapezoid profile (4 m/s, 4 m/s^2) on the distance to the target, kP 1.8 on
// where it should be along it plus the profile's speed; the heading a profiled PID (kP 5, kD 0.5,
// 500 deg/s, 8 rad/s^2) plus its setpoint's speed. st: its state from step to step
function driveToPose(r, st, tgt, dt, { omega = null } = {}) {
  const p = own(r);
  if (!st.init) { st.init = true; st.spX = p.x; st.spY = p.y; st.spVx = p.vx; st.spVy = p.vy; st.th = p.h; st.thV = p.w; st.lastErr = 0; }
  const ex = tgt[0] - p.x, ey = tgt[1] - p.y, err = Math.hypot(ex, ey);
  // (the profile from the last setpoint: its distance and speed toward the target)
  const dx = tgt[0] - st.spX, dy = tgt[1] - st.spY, dd = Math.hypot(dx, dy);
  let v0 = dd <= 0.01 ? Math.hypot(st.spVx, st.spVy) : (st.spVx * dx + st.spVy * dy) / dd;
  v0 = Math.max(v0, -0.5);
  // (a trapezoid step from dd at v0 toward 0, 4 m/s, 4 m/s^2)
  const A = 4, V = 4;
  let v = Math.min(V, v0 + A * dt);
  const vStop = Math.sqrt(2 * A * Math.max(0, dd - v0 * dt));
  v = Math.max(Math.min(v, vStop), v0 - A * dt, 0);
  const pos = Math.max(0, dd - ((v0 + v) / 2) * dt);
  let vs = 1.8 * (err - pos) + v * clamp((err - 0.01) / 0.04, 0, 1);
  if (err < 0.01) vs = 0;
  const ux = err > 1e-6 ? ex / err : 0, uy = err > 1e-6 ? ey / err : 0;
  st.spX = tgt[0] - ux * pos; st.spY = tgt[1] - uy * pos; st.spVx = ux * v; st.spVy = uy * v;
  // heading
  const goal = (tgt[2] * Math.PI) / 180, VW = (500 * Math.PI) / 180, AW = 8;
  const e = wrapAngle(goal - st.th);
  let wv = Math.min(VW, Math.abs(st.thV) + AW * dt, Math.sqrt(2 * AW * Math.abs(e)));
  wv = Math.sign(e) * Math.min(wv, Math.abs(e) / dt);
  st.thV = wv; st.th = wrapAngle(st.th + wv * dt);
  const he = wrapAngle(st.th - p.h);
  let w = 5 * he + 0.5 * (he - st.lastErr) / dt + st.thV;
  st.lastErr = he;
  if (Math.abs(wrapAngle(goal - p.h)) < Math.PI / 180) w = 0;
  drive(r, ux * vs, uy * vs, omega ?? w);
  return { err, herr: Math.abs(wrapAngle(goal - p.h)) };
}

// ------------------------------------------------------------------ the salesman
// SalesAssociate: the intake's next second as a parabola from where it is and how fast it's going,
// x(t) = x0 + vx t + ax t^2 (ax, ay within +-1.5), the one that maximizes the FUEL it would pass
// within 0.75 of half the intake's width of (each FUEL's reward a sigmoid of its summed nearness over
// the second's eleven samples), its end inside the zoning limits and 1 m clear of either HUB's
// avoidance point. Their Sleipnir interior point solver, warm-started from the last answer, 0.15 s
// at most: here gradient ascent from the last answer, the limits kept by projection, 12 steps.
const SA = { tMax: 1.0, tStep: 0.1, maxSq: 1.5, range: ((27.5 * IN) / 2) * 0.75, cDist: 6.0, cReward: 8.0, hubR: 1.0 };
const HUBS = [[5.2225, 4.0345], [11.3185, 4.0345]];
const sig = (x, c, k) => 1 / (1 + Math.exp(-k * (x - c)));
function reward(fuel, x0, y0, vx, vy, ax, ay, grad) {
  let R = 0, gx = 0, gy = 0;
  for (const [fx, fy] of fuel) {
    let s = 0, sx = 0, sy = 0;
    for (let i = 0; i <= 10; i++) {
      const t = i * SA.tStep, t2 = t * t, x = x0 + vx * t + ax * t2, y = y0 + vy * t + ay * t2;
      const dx = x - fx, dy = y - fy, d = Math.hypot(dx, dy) || 1e-9;
      const g = sig(d, SA.range, SA.cDist); // (1 - g: the nearness)
      s += 1 - g;
      if (grad) { const dg = -SA.cDist * g * (1 - g); sx += dg * (dx / d) * t2; sy += dg * (dy / d) * t2; }
    }
    const q = sig(s, 1, SA.cReward);
    R += q;
    if (grad) { const dq = SA.cReward * q * (1 - q); gx += dq * sx; gy += dq * sy; }
  }
  return grad ? [R, gx, gy] : R;
}
function project(ax, ay, x0, y0, vx, vy, B) {
  ax = clamp(ax, -SA.maxSq, SA.maxSq); ay = clamp(ay, -SA.maxSq, SA.maxSq);
  // (the end in bounds: linear in ax, ay)
  ax = clamp(ax, B.minX - x0 - vx, B.maxX - x0 - vx); ay = clamp(ay, B.minY - y0 - vy, B.maxY - y0 - vy);
  for (const [hx, hy] of HUBS) {
    const ex = x0 + vx + ax, ey = y0 + vy + ay, d = Math.hypot(ex - hx, ey - hy);
    if (d < SA.hubR) { const k = d > 1e-6 ? SA.hubR / d : 1; ax += (hx + (ex - hx) * k) - ex + (d > 1e-6 ? 0 : SA.hubR); ay += (hy + (ey - hy) * k) - ey; }
  }
  return [ax, ay];
}
function solve(st, fuel, x0, y0, vx, vy, B) {
  const near = fuel.filter(([fx, fy]) => B.minX <= fx && fx <= B.maxX && B.minY <= fy && fy <= B.maxY && Math.hypot(fx - x0, fy - y0) < Math.hypot(vx, vy) + SA.maxSq + 1.0);
  let [ax, ay] = project(st.ax ?? 0, st.ay ?? 0, x0, y0, vx, vy, B);
  let [R, gx, gy] = reward(near, x0, y0, vx, vy, ax, ay, true);
  let step = 0.5;
  for (let it = 0; it < 12 && step > 1e-3; it++) {
    const g = Math.hypot(gx, gy);
    if (g < 1e-6) break;
    const [nx, ny] = project(ax + (step * gx) / g, ay + (step * gy) / g, x0, y0, vx, vy, B);
    const r2 = reward(near, x0, y0, vx, vy, nx, ny, true);
    if (r2[0] > R) { ax = nx; ay = ny; [R, gx, gy] = r2; step *= 1.3; } else step *= 0.4;
  }
  st.ax = ax; st.ay = ay;
  return { x: [x0, vx, ax], y: [y0, vy, ay], reward: R };
}

// ObjectDetection: what its three color cameras see (front, 10 deg to the right, and both sides at
// 70 deg, 90 deg across each: from 115 deg right round the front to 115 deg left), not behind a
// HUB or another robot (the line to it passing within 0.6 m of a HUB's center or 0.45 of a robot's),
// within 7 m (an estimate: their 1280 x 960 cameras still resolve a FUEL there), on the carpet; each
// one seen kept 3 s (their fuelPersistanceTime), dropped when its place is in view and it isn't
// there, or under the robot; at AUTO's start the NEUTRAL ZONE's FUEL where they're placed
// (initializeFuelPoses), kept 3 s
const CAM = { half: (115 * Math.PI) / 180, range: 7.0, hubR: 0.6, botR: 0.45, persist: 3.0 };
function seeFuel(run, st, t) {
  const r = run.robot, p = own(r);
  if (!st.seen) {
    st.seen = new Map();
    let k = 0;
    for (let i = 0; i < 15; i++) for (let j = 0; j < 6; j++) for (const [sx, sy] of [[1, 1], [-1, 1], [1, -1], [-1, -1]]) st.seen.set('init' + k++, { x: F.center + sx * (0.076 + 0.152 * j), y: F.midY + sy * (0.0254 + 0.076 + 0.152 * i), t });
  }
  const others = run.foes.concat(run.mates || []).filter((o) => o !== r).map((o) => own(o));
  const hubs = [[F.hubX, F.midY], [FIELD_L - F.hubX, F.midY]];
  const blocked = (fx, fy) => {
    const sx = fx - p.x, sy = fy - p.y, L2 = sx * sx + sy * sy || 1e-9;
    const near = (cx, cy, rad) => { const u = clamp(((cx - p.x) * sx + (cy - p.y) * sy) / L2, 0, 1); return Math.hypot(p.x + sx * u - cx, p.y + sy * u - cy) < rad; };
    return hubs.some(([cx, cy]) => near(cx, cy, CAM.hubR)) || others.some((o) => near(o.x, o.y, CAM.botR));
  };
  const inView = (fx, fy) => { const d = Math.hypot(fx - p.x, fy - p.y); return d < CAM.range && d > 0.3 && Math.abs(wrapAngle(Math.atan2(fy - p.y, fx - p.x) - p.h)) <= CAM.half; };
  const now = new Set();
  for (const b of r.fuel.balls) {
    if (b.state !== 'field' || b.held || b.inFlight || b.pos.y > 0.3) continue;
    const fx = r.alliance === 'blue' ? b.pos.x + HALF_L : HALF_L - b.pos.x, fy = r.alliance === 'blue' ? HALF_W - b.pos.z : HALF_W + b.pos.z;
    if (!inView(fx, fy) || blocked(fx, fy)) continue;
    now.add(b.id);
    // (a new sighting replaces the ones within half a FUEL of it: their overlap check)
    for (const [k, q] of st.seen) if (typeof k === 'string' && Math.hypot(q.x - fx, q.y - fy) < 0.075) st.seen.delete(k);
    st.seen.set(b.id, { x: fx, y: fy, t });
  }
  const c = Math.cos(p.h), s = Math.sin(p.h);
  for (const [k, q] of st.seen) {
    const lx = (q.x - p.x) * c + (q.y - p.y) * s, ly = -(q.x - p.x) * s + (q.y - p.y) * c;
    const under = lx > -((26.5 * IN) / 2) && lx < D.intakeFarX && Math.abs(ly) < D.frameWidthY / 2;
    if (t - q.t >= CAM.persist || under || (!now.has(k) && inView(q.x, q.y) && !blocked(q.x, q.y))) st.seen.delete(k);
  }
  return [...st.seen.values()].map((q) => [q.x, q.y]);
}

// their zoning limits (SalesmanSolver's bounds; AutoCommands.getDynamicBounds, ethical: its own
// quarter of the near NEUTRAL ZONE, and the other near quarter if no robot's in it; past the CENTER
// LINE by a bumper's apothem). The robots it knows of: every other robot (their robot detection)
function bounds(run, kind, left) {
  const full = { minX: F.neutralZoneNear, maxX: F.center + D.driveBaseRadius, minY: 0, maxY: FIELD_W };
  if (kind === 'full') return full;
  if (kind === 'left') return { ...full, minY: FIELD_W / 2 - D.fullApothemX };
  if (kind === 'right') return { ...full, maxY: FIELD_W / 2 + D.fullApothemX };
  const mid = F.center, others = run.foes.concat(run.mates || []).filter((o) => o !== run.robot).map((o) => own(o));
  const home = left ? { minX: F.neutralZoneNear, maxX: mid, minY: F.midY, maxY: FIELD_W } : { minX: F.neutralZoneNear, maxX: mid, minY: 0, maxY: F.midY };
  const ally = left ? { ...home, minY: 0, maxY: F.midY } : { ...home, minY: F.midY, maxY: FIELD_W };
  const any = (q) => others.some((o) => o.x >= q.minX && o.x <= q.maxX && o.y >= q.minY && o.y <= q.maxY);
  const b = any(ally) ? home : { minX: home.minX, maxX: home.maxX, minY: 0, maxY: FIELD_W };
  return { ...b, maxX: Math.max(b.maxX, F.center + D.fullApothemX) };
}

// SalesmanCommands.autoIntake: the intake chases its plan 0.25 s ahead of where it is along it (P 6
// on position, at 3 m/s with little reward to 1.8 with lots; P 8 on heading, turned only past 30 deg
// off it, by the 30), the robot inside the limits less its radius
function salesman(run, st, a, dt) {
  const r = run.robot, p = own(r), t = run.elapsed;
  const B = bounds(run, a.bounds, !run.prog.mirror);
  const c = Math.cos(p.h), s = Math.sin(p.h);
  const ix = p.x + D.intakeReferenceX * c, iy = p.y + D.intakeReferenceX * s;
  const ivx = p.vx - p.w * D.intakeReferenceX * s, ivy = p.vy + p.w * D.intakeReferenceX * c;
  // (their solver runs on its own thread, each answer taken if it starts where the intake is: here
  // every 0.1 s)
  if (!st.traj || t - st.solvedT >= 0.1) {
    const fuel = seeFuel(run, run.prog, t);
    st.traj = solve(run.prog, fuel, ix, iy, ivx, ivy, B);
    st.solvedT = t; st.trajT = t;
  }
  const T = st.traj, at = (u) => [T.x[0] + T.x[1] * u + T.x[2] * u * u, T.y[0] + T.y[1] * u + T.y[2] * u * u];
  let u = 0, last = Infinity;
  for (; u < 3.0; u += 0.01) { const [x, y] = at(u); const d = Math.hypot(x - ix, y - iy); if (d >= last) break; last = d; }
  u += 0.25;
  const [tx, ty] = at(u), th = Math.atan2(T.y[1] + 2 * T.y[2] * u, T.x[1] + 2 * T.x[2] * u);
  const m = (30 * Math.PI) / 180, e = wrapAngle(th - p.h);
  const goalH = Math.abs(e) < m ? p.h : e >= m ? th - m : th + m;
  let rx = tx - D.intakeReferenceX * Math.cos(goalH), ry = ty - D.intakeReferenceX * Math.sin(goalH);
  rx = clamp(rx, B.minX + D.driveBaseRadius, B.maxX - D.driveBaseRadius); ry = clamp(ry, B.minY + D.driveBaseRadius, B.maxY - D.driveBaseRadius);
  let vx = 6 * (rx - p.x), vy = 6 * (ry - p.y);
  const cap = 3.0 + (1.8 - 3.0) * clamp((T.reward - 15) / (70 - 15), 0, 1), sp = Math.hypot(vx, vy);
  if (sp > cap) { vx *= cap / sp; vy *= cap / sp; }
  drive(r, vx, vy, 8 * wrapAngle(goalH - p.h));
}

// ------------------------------------------------------------------ the steps
// DriveTrajectory: the trajectory's sample at its time (between samples, linearly), its speed and
// turn rate fed forward, kP 8 on position and 4 on heading. (The step's points: x, y, heading deg,
// speed, time)
function follow(run, st, a, dt) {
  const r = run.robot, pts = a.pts, T = st.t, p = own(r);
  let k = 0;
  while (k + 2 < pts.length && pts[k + 1][4] <= T) k++;
  const A = pts[k], Bq = pts[Math.min(k + 1, pts.length - 1)], h = Bq[4] - A[4], w = h > 1e-6 ? clamp((T - A[4]) / h, 0, 1) : 1;
  let [x, y] = [A[0] + (Bq[0] - A[0]) * w, A[1] + (Bq[1] - A[1]) * w];
  const ha = (A[2] * Math.PI) / 180, dh = wrapAngle(((Bq[2] - A[2]) * Math.PI) / 180);
  let head = ha + dh * w;
  let fvx = h > 1e-6 ? (Bq[0] - A[0]) / h : 0, fvy = h > 1e-6 ? (Bq[1] - A[1]) / h : 0;
  const fw = h > 1e-6 ? dh / h : 0;
  if (a.relative) {
    // (from where it starts: their DriveForward1m, reset there)
    if (!st.o) st.o = { x: p.x, y: p.y, h: p.h };
    const c = Math.cos(st.o.h), s = Math.sin(st.o.h);
    [x, y] = [st.o.x + x * c - y * s, st.o.y + x * s + y * c];
    [fvx, fvy] = [fvx * c - fvy * s, fvx * s + fvy * c];
    head += st.o.h;
  }
  if (T > pts[pts.length - 1][4]) { fvx = 0; fvy = 0; }
  drive(r, fvx + 8 * (x - p.x), fvy + 8 * (y - p.y), fw + 4 * wrapAngle(head - p.h));
  return T >= pts[pts.length - 1][4];
}

// returnToDeterminedLaunchPose (left or right bump): toward the launch pose's y first, out at the
// NEUTRAL ZONE's edge + 1.5 m, then in; facing back over the BUMP, along the way in the NEUTRAL ZONE
function returnTarget(run) {
  const p = own(run.robot), T = LAUNCH[run.prog.mirror ? 'rightBump' : 'leftBump'];
  let t = clamp(Math.cbrt((Math.abs(p.y - T[1]) - 0.1) / (0.4 - 0.1)), 0, 1);
  if (p.x < F.neutralZoneNear + 1.0 && (p.y < F.rightBumpStart - D.fullBaseRadius || p.y > F.leftBumpEnd + D.fullBaseRadius)) t = 0;
  const x = T[0] + (F.neutralZoneNear + 1.5 - T[0]) * t;
  const deg = p.x < (F.starting + F.neutralZoneNear) / 2 ? T[2] : p.x < F.neutralZoneNear + 0.838 ? 180 : (Math.atan2(T[1] - p.y, T[0] - p.x) * 180) / Math.PI;
  return [x, T[1], deg];
}

// a stationary pose with the launcher (at the back) aimed at the HUB from (x, y)
const aimedAt = (x, y) => [x, y, (Math.atan2(y - F.midY, x - F.hubX) * 180) / Math.PI];

const STEPS = {
  wait(run, st, a) { return st.t >= a.t; },
  rush(run, st, a) {
    const p = own(run.robot);
    drive(run.robot, D.maxLinearSpeed * (a.slow ? 0.6 : 1), 0, 0);
    if (a.coastX && p.x > a.coastX) run.prog.coast = true;
    return st.t >= a.t;
  },
  traj(run, st, a, dt) {
    const done = follow(run, st, a, dt);
    const p = own(run.robot);
    if (a.coastX && p.x > a.coastX) run.prog.coast = true;
    // (launching on the move once back behind a.aim: their LOTM, the robot aiming itself)
    if (a.aim && p.x < a.aim) { run.robot.cmd.shoot = true; run.prog.noIntake = true; }
    return done;
  },
  turn(run, st, a, dt) {
    // salesmanTurn: into the corner of the FUEL pool, 1 s at most
    const p = own(run.robot), left = !run.prog.mirror, xT = F.fuelPoolLeft[0] - 0.8;
    const k = clamp(Math.abs(p.x - xT) / (xT - F.neutralZoneNear), -1, 1);
    const y0 = left ? F.fuelPoolLeft[1] : F.fuelPoolRight[1], y1 = left ? y0 + 3.5 : y0 - 3.5;
    driveToPose(run.robot, st, [xT, y0 + (y1 - y0) * k, left ? -50 : 50], dt);
    return st.t >= 1.0;
  },
  salesman(run, st, a, dt) { salesman(run, st, a, dt); return st.t >= a.t; },
  return(run, st, a, dt) {
    driveToPose(run.robot, st, returnTarget(run), dt);
    return own(run.robot).x < a.untilX;
  },
  // returnLaunchAndIndexMindfully: to the launch pose until past the starting line less the
  // bumpers' radius, then held aimed where it is, launching, for launchTime and until leaveAt
  launch(run, st, a, dt) {
    const r = run.robot, p = own(r);
    if (!st.phase2) {
      driveToPose(r, st, LAUNCH[a.target], dt);
      if (p.x < F.starting - D.fullBaseRadius) { st.phase2 = run.elapsed; st.hold = [p.x, p.y]; st.dtp = {}; }
      return false;
    }
    driveToPose(r, st.dtp, aimedAt(st.hold[0], st.hold[1]), dt);
    r.cmd.shoot = true; run.prog.noIntake = true;
    return run.elapsed - st.phase2 >= (a.hasty ? DEPOT_LAUNCH_TIME : LAUNCH_TIME) && (a.leaveAt == null || run.elapsed >= a.leaveAt);
  },
  // driveToPose with an index once within 0.1 m and 5 deg (Monopoly's at the TOWER, Timid's): the
  // index runs its compaction's length (0.35 s, then the intake 1.2 s) once the flywheel's at speed
  launchAt(run, st, a, dt) {
    const r = run.robot, { err, herr } = driveToPose(r, st, a.pose, dt);
    if (st.idx === undefined && err < 0.1 && herr < (5 * Math.PI) / 180) st.idx = run.elapsed;
    if (st.idx !== undefined) {
      if (r.phys?.launching && st.go === undefined) st.go = run.elapsed;
      const over = st.go !== undefined && run.elapsed - st.go >= INDEX_T;
      if (!over) { r.cmd.shoot = true; run.prog.noIntake = true; }
      return a.until === 'empty' ? over : false;
    }
    return false;
  },
  // passingCommand (BOTH): held where it is (on its side of the CENTER LINE), aimed at their pass
  // point, passing for its time once within 0.5 m and 15 deg
  pass(run, st, a, dt) {
    const r = run.robot, p = own(r);
    if (!st.at) st.at = [Math.min(p.x, F.center + D.fullApothemX - 0.2), p.y];
    r.cmd.pass = true; run.prog.noIntake = true;
    drive(r, 3 * (st.at[0] - p.x), 3 * (st.at[1] - p.y), 0);
    return st.t >= a.t + 1.0;
  },
  toPose(run, st, a, dt) {
    driveToPose(run.robot, st, a.pose, dt);
    if (a.coastX && own(run.robot).x > a.coastX) run.prog.coast = true;
    return false;
  },
};

// The step: its program's commands in turn. The intake runs from 1 s in (their autonomous trigger),
// but not while it launches (their index retracts the slam, its intake timing out); the flywheel
// tracks the HUB from 1.5 s (runTrackTargetCommand: js/mech/phys6328.js, prespin). Coasting (their
// ALWAYS_COAST): the drive let go, the robot rolling on and slowing
export function step6328(run, s, dt, next) {
  const r = run.robot, prog = (run.prog ??= { k: 0, st: { t: 0 }, mirror: !!s.mirror, coast: false });
  if (prog.coast) {
    const p = own(r), k = Math.exp(-dt / 1.2);
    drive(r, p.vx * k, p.vy * k, p.w * k);
    r.cmd.intake = true;
    return;
  }
  prog.noIntake = false;
  r.cmd.prespin = run.elapsed >= AUTO_START.spin;
  while (prog.k < s.program.length) {
    const [kind, a] = s.program[prog.k];
    const done = STEPS[kind](run, prog.st, a, dt);
    prog.st.t += dt;
    if (!done) break;
    prog.k++; prog.st = { t: 0 };
    // (done; one coasting rolls on to the end of AUTO)
    if (prog.k >= s.program.length) { if (!prog.coast) next(); break; }
  }
  r.cmd.intake = run.elapsed >= AUTO_START.intake && !prog.noIntake;
}
