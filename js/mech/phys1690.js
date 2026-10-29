// 1690's Kepler in play, built of its physical mechanisms (js/mech/r1690.js; the shared part:
// js/mech/physical.js). Their code isn't public, so its controls are what the team said on Chief
// Delphi about how it runs (their CAD release and reveal threads), the rest estimates:
//   - intake: the carriage slides out at the start of the match and stays out (its motor brings it
//     in only for "travel"), and the kicker bar under its rollers falls on surgical tubing and stays
//     down; its rollers at 12 V while intaking, back out to outtake.
//   - shooting (held: the AI's shot; out of its zone, a pass), on the move and while intaking: the
//     turret aims itself (the robot's turret model turns it, on its motor and its energy chain:
//     turretStep), the shooter wheel's velocity loop to the speed for the shot and the hood to its
//     angle (the sim's shot table for the distance from where the FUEL leave to the target, less the
//     turret's own velocity a latency ahead: "tangential and radial velocity ... derivative of the
//     desired flywheel speed"); once the turret, the hood and the wheel are there, the indexer runs:
//     the floor at 75 RPS and the top and bottom belts at 95 RPS (passing: 85 and 100), the vertical
//     kicker at 60 RPS, the feeder's wheels at 60% of the shooter wheel's surface speed and at least
//     5 m/s (all the team's numbers, their motors' speeds), each a velocity loop on its Talon.
//   - "if our shooter wheels stay below their target velocity for a sustained period, we
//     automatically slow down the conveyor": under 92% for 0.1 s, the floor and belts at 60% until
//     it's back (estimates).
//   - "if we detect a conveyor jam (based on high stator current for several code cycles), we briefly
//     reverse the kicker": the vertical kicker over 60 A under a fifth of its speed for 5 cycles (0.1
//     s), back for 0.15 s (estimates).
//   - not shooting: the indexer stops, the wheel coasts, the hood goes to its steep end (low: under
//     a TRENCH; "travel ... stows the hood").
//   - unjam (the AI's outtake): everything backward.
import { Mechanism } from './mechanism.js';
import { PhysicalRobot } from './physical.js';
import {
  build1690, wheels1690, shooterWheel, feederWheels, pathSurfaces1690, placeIntake, placeTurret, turretPoint, intakeFront, kickAt,
  HALF_L, HALF_W, INTAKE, FLOOR, TOP, BELT, VKICK, FEED, SHOOTER, TURRET, NET, WALL, PRELOAD,
} from './r1690.js';
import { speedLoop, profile } from './phys1706.js';
import { Field } from '../field.js';
import { FUEL } from '../constants.js';
import { clamp, wrapAngle } from '../util.js';
import { drive1690 } from './turretDrive.js';

const DEG = Math.PI / 180, TAU = 2 * Math.PI, R = FUEL.radius;
// the indexer's speeds (their motors' RPS: the team), scoring and passing
export const RPS = { floor: [75, 85], belts: [95, 100], vkick: 60 };
// the feeder's surface: a share of the shooter wheel's, and at least this (m/s)
export const FEED_SHARE = 0.6, FEED_MIN = 5;
const LATENCY = 0.05; // s: their loop and the turret's lag (estimate)
// Its shots, as the bench measures them (tools/mechlab1690.mjs shooter and calibrate): where the FUEL
// leave (the turret's frame: s along the shot from its axis, y up), the elevation each hood angle
// gives (deg: the hood's angle about the axle, 90 + its geometric elevation, -> the FUEL's), and the
// wheel's surface speed each exit speed takes (m/s). Tuned on FUEL fed up the bore in a stream by the
// indexer (calibrate FED=1), the way it fires: those leave about 4% faster than lone FUEL thrown up
// the bore, pressed into the wheel by the next one. Past 9.3 m/s, lone FUEL (the wheel saturates).
export const EXIT = { s: 0.03, y: 0.66 };
export const EL_MAP = [[125, 37.4], [144.2, 57.6], [144.9, 58], [145.9, 59.1], [147, 60.2], [148.3, 61], [150.4, 62.9], [151.6, 64], [153.8, 65.9], [156.9, 68.6], [160.5, 71.7], [163.4, 75], [172, 84.9]];
export const EXIT_MAP = [[0, 0], [4, 6], [6.05, 9.02], [6.4, 9.57], [6.79, 10.22], [7.15, 10.87], [7.56, 11.66], [7.89, 12.15], [8.26, 13.04], [8.61, 13.7], [8.95, 14.2], [9.3, 15.71], [9.8, 19.14]];
const interp = (t, x, a = 0, b = 1) => {
  if (x <= t[0][a]) return t[0][b];
  for (let k = 1; k < t.length; k++) if (x <= t[k][a]) return t[k - 1][b] + ((t[k][b] - t[k - 1][b]) * (x - t[k - 1][a])) / (t[k][a] - t[k - 1][a]);
  return t[t.length - 1][b];
};
const HOOD_RANGE = [SHOOTER.elMin + 90, SHOOTER.elMax + 90];
// the hood angle (deg about the axle) for an elevation (deg), and the elevation an angle gives
export const hoodFor = (el) => clamp(interp([...EL_MAP].sort((p, q) => p[1] - q[1]), el, 1, 0), ...HOOD_RANGE);
export const elevationAt = (h) => interp(EL_MAP, h);
// the wheel's surface speed (m/s) for an exit speed (m/s), and the exit speed a surface speed gives
export const surfFor = (v) => interp(EXIT_MAP, v);
export const exitAt = (surf) => interp(EXIT_MAP, surf, 1, 0);
// (the hood's stow: its steep end)
export const HOOD_STOW = HOOD_RANGE[1];

export class Physical1690 extends PhysicalRobot {
  constructor(robot) {
    const t = build1690(), w = wheels1690(t), s = pathSurfaces1690(t), sw = { wheel: shooterWheel(t), feeder: feederWheels(t) };
    const pathWheels = [...w.intake, ...w.floor, w.vkick, ...w.feed];
    super(robot, {
      path: new Mechanism({ body: robot.body, substeps: 6, shafts: [...new Set(pathWheels.map((x) => x.shaft)), ...t.belts], wheels: pathWheels, surfaces: [s.body, s.bodyFine, s.ptfe, s.bottom, s.top, s.carriage, s.kick, s.bumper, s.net] }),
      // (the turret and the hood with the wheel and the hood's feeder wheels: a FUEL leaves at up to 12
      // m/s, too far in a step to follow without sub-steps)
      shooter: new Mechanism({ body: robot.body, substeps: 6, sub: 8, shafts: [t.wheel, ...t.hoodWheels], wheels: [sw.wheel, ...sw.feeder], surfaces: [s.turret, s.hood] }),
      preload: PRELOAD,
    });
    Object.assign(this, { t, w, s, sw });
    this.ownsIntake = true;
    this.ownsHood = true;
    this.reset();
  }

  reset() {
    super.reset();
    // the intake folded in (inside the frame, as it starts its matches), the hood stowed
    this.deploy = 0;
    this.hood = HOOD_STOW; this.hoodSet = [HOOD_STOW, 0]; this.hoodGoal = HOOD_STOW;
    this.feedingNow = false; this.slowT = 0; this.jamT = 0; this.backT = 0; this.ready = false;
    this.want = null; this.aimPsi = null;
    const T = this.t;
    for (const tr of [T.intake, T.floor, T.feed, T.shooter, T.hoodTrain]) { tr.omega = 0; tr.cmd = null; tr.vel = null; }
    for (const sh of [T.tube, T.vkick, ...T.belts]) { sh._omega = 0; sh.cmd = null; sh.vel = null; }
  }

  // its FUEL: in the frame (the hopper over the floor, the belts, the corridor, up into the turret),
  // in the U-frame out over the intake, or on the intake out front climbing the bumper (not one below
  // the bumper's bottom edge)
  where(x, y, z) {
    const inFrame = x > -HALF_L - 0.02 && x < HALF_L && Math.abs(z) < HALF_W && y > 0.09 && y < 0.8;
    const out = this.deploy > 0.5;
    const inFront = out && x >= HALF_L && x < INTAKE.frame.front + 0.01 && Math.abs(z) < INTAKE.frame.sides && y < 0.62;
    return {
      own: inFrame || inFront,
      // (held: in it; not one on the carpet in the intake's mouth or still climbing the bumper's face)
      held: inFrame || (inFront && y > 0.20 && x < INTAKE.frame.front - 0.04),
      near: x > -HALF_L - 0.1 && x < INTAKE.frame.front + 0.15 && Math.abs(z) < HALF_W + 0.1 && y < 0.9,
    };
  }
  // what the shooter might touch: up in the turret
  shooterBalls(near) {
    return near.filter((b) => { const [x, y, z] = this._local(b.pos); return y > TURRET.bore.y[0] && Math.hypot(x - TURRET.at[0], z - TURRET.at[1]) < 0.25; });
  }
  // (the U-frame's front, x, the carriage this.deploy of the way out)
  _front() { return intakeFront(this.deploy); }

  preStep(dt, t) {
    const r = this.r;
    // (put somewhere by the game, not by its motors: a staged scene, its intake out or in)
    if (this.lastDeploy !== undefined && Math.abs(r.intakeDeploy - this.lastDeploy) > 1e-6) this.deploy = r.intakeDeploy;
    // out at the start of the match, and down from then on
    if (r.enabled) this.deploy = Math.min(1, this.deploy + dt / r.cfg.intake.deployTime);
    this.hoodSet = profile(...this.hoodSet, this.hoodGoal, SHOOTER.hoodV, SHOOTER.hoodA, dt);
    this.hood = this.hoodSet[0];
    this._place(r.turretYaw || 0);
    this.stepMechanisms(dt, t);
    r.intakeDeploy = this.lastDeploy = this.deploy;
  }
  _place(yaw) {
    const s = this.s;
    placeIntake(s, this.w.intake, this.deploy);
    s.net.place(Math.max(HALF_L, this._front()));
    placeTurret(s, this.sw, yaw, this.hood);
  }

  // the shot (as 1706's: their thread says what goes into it, not how): its exit LATENCY ahead on the
  // robot's motion; the table's shot to the target; less the exit's own velocity: its heading (aimPsi,
  // which the robot turns the turret to), elevation and exit speed
  _aim(tgt, table) {
    const r = this.r, b = r.body, v = b.linvel(), w = b.angvel().y, L = LATENCY;
    const c = b.translation(), yaw = r.yaw + w * L, cy = Math.cos(yaw), sy = Math.sin(yaw);
    const [tx, tz] = TURRET.at;
    const ox = tx * cy + tz * sy, oz = -tx * sy + tz * cy;
    const px = c.x + v.x * L + ox, pz = c.z + v.z * L + oz;
    const vx = v.x + w * oz, vz = v.z - w * ox;
    const dx = tgt.x - px, dz = tgt.z - pz, d = Math.hypot(dx, dz) - EXIT.s;
    const row = table.lookup(d);
    if (!row) return null;
    const ux = dx / (d + EXIT.s || 1), uz = dz / (d + EXIT.s || 1);
    const sx = row.v * Math.cos(row.theta) * ux - vx, sz = row.v * Math.cos(row.theta) * uz - vz, syv = row.v * Math.sin(row.theta);
    const vr = Math.hypot(sx, syv, sz);
    return { psi: Math.atan2(-sz, sx), el: Math.asin(syv / vr) / DEG, v: vr, d };
  }

  postStep(dt, t, { on, feeding, outtake, mode }) {
    const r = this.r, T = this.t, S = SHOOTER;
    this.account(t, mode);
    const intaking = on && !!r.cmd.intake, unjam = on && !!outtake;
    // (its rollers run on while it feeds, intaking or not: what its driver does, emulated (the AI's
    // scorer keeps intaking as it shoots anyway): the FUEL out over the intake roll back in)
    T.intake.cmd = !on ? 0 : unjam ? -0.6 : intaking || this.feedingNow ? 1 : null;
    // ---- the shot: aiming and spinning up while the robot has one (r.shot: its target)
    const tgt = r.shot?.target, pass = tgt?.mode === 'pass';
    const table = pass ? r.passTable : r.hubTable;
    const point = tgt ? (pass ? { x: tgt.x, z: tgt.z } : Field.hubCenter(r.alliance)) : null;
    const sol = on && point && !unjam ? this._aim(point, table) : null;
    this.want = sol;
    this.aimPsi = sol ? sol.psi : null;
    const surf = sol ? surfFor(sol.v) : 0;
    // (the hood's feeder wheels at 60% of the wheel's target, at least 5 m/s)
    if (sol) { this.hoodGoal = hoodFor(sol.el); speedLoop(T.shooter, surf / S.r, S.kP); speedLoop(T.hoodTrain, Math.max(FEED_MIN, FEED_SHARE * surf) / S.feeder.r); }
    else { this.hoodGoal = HOOD_STOW; speedLoop(T.shooter, null); speedLoop(T.hoodTrain, unjam ? -3 / S.feeder.r : null); }
    const wheelSurf = Math.abs(T.wheel.omega) * S.r;
    const atV = !!sol && Math.abs(wheelSurf - surf) < S.tolV * surf;
    const atHood = Math.abs(this.hood - this.hoodGoal) < S.tolHood;
    const tol = pass ? 4 * DEG : Math.max(0.8 * DEG, Math.atan2(0.14, sol?.d || 1));
    const atYaw = Math.abs(r.aimErr ?? Infinity) < tol;
    this.ready = !!sol && atV && atHood && atYaw;
    // (it starts feeding once ready, and keeps on through the wheel's dip as each shot leaves: their
    // code slows the conveyor for a wheel that stays low, it doesn't stop it; off the aim or the hood,
    // it stops)
    this.feedingNow = on && feeding && !!sol && !unjam && atYaw && atHood && (atV || this.feedingNow);
    // ---- the indexer
    this.slowT = sol && wheelSurf < 0.92 * surf ? this.slowT + dt : 0;
    const slow = this.slowT > 0.1 ? 0.6 : 1, k = pass ? 1 : 0;
    const rps = (n, sh) => (n * TAU) / sh.motor.ratio; // (a motor's RPS -> its shaft's or train's rad/s)
    // (the vertical kicker's jam: over 60 A at under a fifth of its speed for 0.1 s)
    const vk = T.vkick;
    this.jamT = this.feedingNow && Math.abs(vk.motor.current) > 60 && Math.abs(vk.omega) < 0.2 * rps(RPS.vkick, vk) ? this.jamT + dt : 0;
    if (this.jamT > 0.1) { this.backT = 0.15; this.jamT = 0; r.stats.unjams = (r.stats.unjams || 0) + 1; }
    this.backT = Math.max(0, this.backT - dt);
    if (unjam) {
      speedLoop(T.floor, -rps(30, T.floor)); for (const b of T.belts) speedLoop(b, -rps(40, b)); speedLoop(vk, -rps(30, vk)); speedLoop(T.feed, -rps(30, T.feed));
    } else if (this.feedingNow) {
      speedLoop(T.floor, rps(RPS.floor[k], T.floor) * slow);
      for (const b of T.belts) speedLoop(b, rps(RPS.belts[k], b) * slow);
      speedLoop(vk, (this.backT > 0 ? -1 : 1) * rps(RPS.vkick, vk));
      // (the bottom and top kickers: their speed isn't public; at the belts' motor speed, an estimate)
      speedLoop(T.feed, rps(RPS.belts[k], T.feed) * slow);
    } else {
      for (const a of [T.floor, ...T.belts, vk, T.feed]) speedLoop(a, null);
    }
    // what the robot's aiming, its HUD and drawing see
    r.hoodDeg = elevationAt(this.hood);
    r.flywheel = exitAt(wheelSurf);
    r.ready = this.ready;
    r.feeding = this.feedingNow ? 1 : 0;
    r.intakeSpeed = T.intake.cmd > 0 ? 1 : T.intake.cmd < 0 ? -1 : 0;
  }

  // how far the U-frame is out past the frame (m), for the hopper's collider and drawing it
  extNow() { return Math.max(0, this._front() - HALF_L); }
  // the other moving parts, for drawing them: how far the carriage is out (0 to 1) and the kicker
  // bar's fall (rad), the hood's angle (rad about the axle), the wheels' and belts' speeds
  mechPose() {
    const T = this.t;
    return { ext: this.deploy, kick: kickAt(this.deploy) * DEG, hood: this.hood * DEG, wheel: T.wheel.omega, feeder: T.hoodTrain.omega, floor: T.floor.omega, top: T.top[0].omega, belts: T.belts.map((b) => b.omega * 0.0175), vkick: T.vkick.omega, feed: T.feed.omega, intakeRollers: T.intake.omega, tube: T.tube.omega };
  }
  // the load's top (for the net and the TRENCH)
  loadTop() { let top = 0; for (const b of this.held) { const y = this._local(b.pos)[1]; if (y + R > top) top = y + R; } return top; }
  // how many it holds (tools/mechlab1690.mjs intakefull and hold)
  capacity() { return 47; }

  // The turret on its motor (js/mech/turretDrive.js): its X60 under a position loop, against its
  // bearings and its energy chain (js/echain.js). js/robot.js asks it each step where the turret gets
  // to on its way from cur to goal (rad, its yaw on the robot; t: the time, s).
  turretStep(cur, goal, dt, t) {
    this.turretDrive ||= drive1690();
    return this.turretDrive.step(cur, goal, dt, t);
  }
}
export { NET, WALL, FLOOR, VKICK, turretPoint };
