// 1706's Mirage in play, built of its physical mechanisms (js/mech/r1706.js; the shared part:
// js/mech/physical.js). Their 2026 code isn't public, so its controls are what their CAD and thread
// say the robot does, run the way their 2025 code runs its mechanisms (rr1706/konshu: Motion Magic
// on the arms, velocity loops on the rollers, the same current limits); the setpoints are estimates:
//   - intake (held): the arm out and down (113.4 deg, its Motion Magic), the rollers at 100% once
//     within 20 deg of it; let go, the arm folds back in (the team: pulling the intake in), the
//     rollers off, sweeping what's out over it back into the hopper and pulling the box in with it;
//     it stalls against a load that won't squeeze any further (its X60 at its 80 A), and holds
//     there. Shooting and not intaking, its rollers run while it folds, until it's at 30 deg (what a
//     driver does, emulated: the FUEL in its mouth go up the sloped plate instead of wedging there,
//     which their thread says the plate is for).
//   - shooting (held: the AI's shot; out of its zone, a pass): each turret aims itself: the
//     robot's turret model turns it to the heading this gives, and it fires when it's there. Its
//     shot is the sim's shot table for the distance from its exit to the HUB's center (or the pass
//     table to the pass point: their tables aren't public), less the turret's own velocity over the
//     carpet (shoot on the move): the hood swings to the elevation, the accelerators' velocity loop
//     to the speed (the bench's exit over the accelerators' surface: tools/mechlab1706.mjs
//     shooter). A turret is ready within 3 deg of yaw, 1.5 deg of hood and 0.3 m/s of exit speed;
//     then that side's spindexer turns at SPIN_RPS and its kicker at KICK_RPM (both velocity loops
//     on the Talons at their 1 kHz), and stops as soon as it isn't.
//   - let go: the shooters coast, the hoods back to their steep end, the spindexers and kickers
//     stop.
//   - unjam (the AI's outtake): the intake out, its rollers, the spindexers and kickers backward.
import { Mechanism } from './mechanism.js';
import { PhysicalRobot } from './physical.js';
import {
  build1706, wheels1706, shooterWheels, pathSurfaces1706, placeIntake, placeBox, placeSpin, placeTurret, placeShooter, armRot, boxExt, hoodElevation, turretPoint,
  HALF_L, HALF_W, PIVOT, INTAKE, BOX, SPIN, TURRET, SHOOTER, NET, PRELOAD, SPIN_DIR,
} from './r1706.js';
import PATH from './path1706.js';
import { Field } from '../field.js';
import { FUEL } from '../constants.js';
import { BUMPER_Y0 } from '../robotConfigs.js';
import { clamp, wrapAngle } from '../util.js';

const DEG = Math.PI / 180, TAU = 2 * Math.PI, R = FUEL.radius;
// the spindexers' and kickers' setpoints while a turret is ready (estimates: their code isn't public)
export const SPIN_RPS = Number(globalThis.process?.env?.SPIN_RPS ?? 12);
export const KICK_RPM = Number(globalThis.process?.env?.KICK_RPM ?? 3000);
// Its shots, as the bench measures them (tools/mechlab1706.mjs shooter with MAP=1, and calibrate:
// FUEL fed by the kicker into the turret's wheels at hood swings and accelerator speeds, and the
// sim's table's rows tuned the way a team tunes its table): where the FUEL leaves (its turret's
// frame: x along the shot, y up; out over the accelerators a little further as the hood swings up),
// the elevation each hood swing gives (deg; the CAD's geometry says 88 to 43, the FUEL leave flatter,
// rolling off the hood's 1in wheels), and the accelerators' surface speed each exit speed takes (m/s):
// about 1.25 times it up to ~7 m/s, then the FUEL slip in the pinch (16 mm of squeeze by the CAD) and
// it tops out near 8.6 m/s, the X60 near its free speed
export const EXIT = { x: 0.06, y: 0.62 };
export const EL_MAP = [[0, 84.5], [5.3, 76.6], [8.3, 72.3], [11.3, 69.7], [14.8, 65.0], [19.3, 61.1], [22.5, 55.2], [30, 48.4], [40, 40.5], [45, 37.5]];
export const EXIT_MAP = [[0, 0], [4.7, 5.9], [5.6, 7.0], [6.1, 7.75], [6.45, 8.27], [6.84, 8.83], [7.17, 9.87], [7.5, 12.44], [7.9, 14.52], [8.28, 16.9], [8.44, 18.19], [8.6, 19.5]];
const interp = (t, x, a = 0, b = 1) => {
  if (x <= t[0][a]) return t[0][b];
  for (let k = 1; k < t.length; k++) if (x <= t[k][a]) return t[k - 1][b] + ((t[k][b] - t[k - 1][b]) * (x - t[k - 1][a])) / (t[k][a] - t[k - 1][a]);
  return t[t.length - 1][b];
};
// the hood swing (deg) for an elevation (deg), and the elevation a swing gives
export const hoodFor = (el) => clamp(interp([...EL_MAP].reverse(), el, 1, 0), 0, SHOOTER.hoodMax);
export const elevationAt = (h) => interp(EL_MAP, h);
// a trapezoidal profile's next setpoint (position p, velocity v) toward goal
export function profile(p, v, goal, vmax, amax, dt) {
  const d = goal - p, dir = Math.sign(d);
  if (Math.abs(d) < 1e-6 && Math.abs(v) < amax * dt) return [goal, 0];
  const want = dir * Math.min(vmax, Math.sqrt(2 * amax * Math.abs(d)));
  const nv = v + clamp(want - v, -amax * dt, amax * dt);
  const np = p + nv * dt;
  if ((goal - np) * dir < 0) return [goal, 0];
  return [np, nv];
}
// the arm: its Motion Magic toward goal (deg of swing), its position loop on the X60 (kP volts per
// motor rotation, kV on the profile's speed), its weight on the pivot; returns the profile's state
export function servoArm(T, deg, set, goal, dt) {
  const I = INTAKE, m = T.pivot.motor, rot = (rad) => (rad * m.ratio) / TAU;
  set = profile(...set, goal * DEG, I.mmV, I.mmA, dt);
  T.pivot.cmd = clamp((I.kP * (rot(set[0]) - rot(deg * DEG)) + I.kV * rot(set[1])) / 12, -1, 1);
  // (its weight: its center of mass forward of the pivot swings it out)
  const a = armRot(deg), cx = I.com[0] * Math.cos(a) - I.com[1] * Math.sin(a);
  T.pivot.impulse(I.mass * 9.81 * cx * dt);
  return set;
}
// after the mechanisms' step: the arm turns as its shaft did (to its stops: folded in, out)
export function turnArm(T, deg, dt) {
  deg += (T.pivot.omega * dt) / DEG;
  if (deg < 0) { deg = 0; if (T.pivot._omega < 0) T.pivot._omega = 0; }
  if (deg > INTAKE.max) { deg = INTAKE.max; if (T.pivot._omega > 0) T.pivot._omega = 0; }
  return deg;
}
// velocity loops on the Talons at their 1 kHz (Shaft.vel / Train.vel; velLoop in mechanism.js):
// kV the motor's (12 V over its free speed: 0.124 V per rotation/s for an X60, 0.096 for an X44), kP
// as given (estimates). target in rad/s of the
// shaft or train; null: coast
export function speedLoop(a, target, kP = 0.2, kV = null) {
  // (kV: the motor's own volts per rotation/s, 12 V over its free speed, as a tuned kV is)
  kV ??= (12 * 2 * Math.PI) / a.motor.m.free;
  a.vel = target === null ? null : { target, kV, kP };
  a.cmd = target === null ? null : 0;
}
// the accelerators' speed (rad/s) for an exit speed (m/s), and the exit speed an accelerator speed gives
export const accelFor = (v) => interp(EXIT_MAP, v) / SHOOTER.accel.r;
export const exitAt = (accelW) => interp(EXIT_MAP, Math.abs(accelW) * SHOOTER.accel.r, 1, 0);

export class Physical1706 extends PhysicalRobot {
  constructor(robot) {
    const t = build1706(), w = wheels1706(t), s = pathSurfaces1706(PATH, t), sw = shooterWheels(t);
    const pathWheels = [...w.intake, ...w.kicker];
    super(robot, {
      path: new Mechanism({ body: robot.body, substeps: 6, shafts: [...new Set(pathWheels.map((x) => x.shaft)), t.pivot, ...t.spin], wheels: pathWheels, surfaces: [s.body, s.bowl, s.intake, s.hopper, ...s.spin, ...s.turret, ...s.hood, s.bumper, s.net] }),
      shooter: new Mechanism({ body: robot.body, substeps: 6, sub: 8, shafts: sw.flat().map((x) => x.shaft), wheels: sw.flat() }),
      // (full when its intake can't put more in: js/mech/physical.js. Over the front half of the
      // hopper and the box, under the net: a layer of ~12 FUEL, packed against it when two thirds
      // are)
      preload: PRELOAD, naturalFull: { topX: 0.12, topN: 10 },
    });
    Object.assign(this, { t, w, s, sw });
    this.ownsIntake = true;
    this.ownsHood = true;
    this.ownsFlywheel = true;
    this.reset();
  }

  reset() {
    super.reset();
    // folded in at the start (inside the frame, as it starts its matches)
    this.armDeg = 0; this.armSet = [0, 0]; this.armGoal = 0;
    this.hood = [0, 0]; this.hoodSet = [[0, 0], [0, 0]]; this.hoodGoal = [0, 0];
    this.phi = [0, 0];
    this.aimPsis = [null, null]; this.want = [null, null]; this.readyT = [false, false];
    const T = this.t;
    for (const tr of [T.intake, ...T.kicker.map((k) => k.train), ...T.shooter.map((s) => s.train)]) { tr.omega = 0; tr.cmd = null; tr.vel = null; }
    for (const sh of [T.pivot, ...T.spin]) { sh._omega = 0; sh.cmd = 0; sh.vel = null; }
    this.ext = 0;
    const r = this.r;
    r.turretYaws = [90 * DEG, -90 * DEG];
    r.turretYaw = r.turretYaws[0];
  }

  // its FUEL: in the frame (the hopper over the spindexers, the ramps, the kickers, up into the
  // turrets), in the hopper box out over the intake, or on the intake out front climbing the bumper
  // (not one below the bumper's bottom edge: the intake can't reach it)
  where(x, y, z) {
    const inFrame = Math.abs(x) < HALF_L && Math.abs(z) < HALF_W && y > 0.09 && y < 0.8;
    const inBox = x >= HALF_L && x < HALF_L + this.ext + 0.01 && Math.abs(z) < BOX.side && y > 0.16 && y < 0.78;
    const onArm = this.armDeg > 60 && x >= HALF_L && x < 0.66 && Math.abs(z) < 0.37 && y > BUMPER_Y0 && y < 0.45;
    return {
      own: inFrame || inBox || onArm,
      // (held: in it; not one on the carpet or still climbing the bumper's face)
      held: inFrame || inBox || (onArm && y > 0.24 && x < 0.56),
      near: x > -HALF_L - 0.1 && x < 0.75 && Math.abs(z) < HALF_W + 0.1 && y < 0.9,
    };
  }
  // what the shooters' wheels might touch: up in a turret
  shooterBalls(near) {
    return near.filter((b) => { const [x, y, z] = this._local(b.pos); return y > 0.36 && TURRET.some(([tx, tz]) => Math.hypot(x - tx, z - tz) < 0.17); });
  }

  preStep(dt, t) {
    const r = this.r, T = this.t, on = r.enabled;
    // (put somewhere by the game, not by its motors: a staged scene, its intake out or in)
    if (this.lastDeploy !== undefined && Math.abs(r.intakeDeploy - this.lastDeploy) > 1e-6) {
      this.armDeg = INTAKE.out * r.intakeDeploy; this.armSet = [this.armDeg * DEG, 0]; T.pivot._omega = 0;
    }
    if (on) this.armSet = servoArm(T, this.armDeg, this.armSet, this.armGoal, dt);
    else { T.pivot.cmd = 0; T.pivot._omega = 0; this.armSet = [this.armDeg * DEG, 0]; } // (disabled: braked)
    for (let i = 0; i < 2; i++) {
      this.hoodSet[i] = profile(...this.hoodSet[i], this.hoodGoal[i], SHOOTER.hoodV, SHOOTER.hoodA, dt);
      this.hood[i] = this.hoodSet[i][0];
    }
    this._place(r.turretYaws || [0, 0]);
    this.stepMechanisms(dt, t);
    this.armDeg = turnArm(T, this.armDeg, dt);
    for (let i = 0; i < 2; i++) this.phi[i] = (this.phi[i] + T.spin[i].omega * dt) % TAU;
    r.intakeDeploy = this.lastDeploy = clamp(this.armDeg / INTAKE.out, 0, 1);
  }
  _place(yaws) {
    const s = this.s;
    this.ext = boxExt(PATH.ext, this.armDeg);
    s.intake.setPose([...PIVOT, 0], 0, armRot(this.armDeg));
    placeIntake(this.w.intake, this.armDeg);
    placeBox(s.hopper, PATH.ext, this.armDeg);
    s.net.place(this.ext);
    for (let i = 0; i < 2; i++) {
      placeSpin(s.spin[i], i, this.phi[i]);
      placeTurret(s, i, yaws[i], this.hood[i]);
      placeShooter(i, this.sw[i], yaws[i], this.hood[i]);
    }
  }

  // each turret's shot (604's LaunchCalculator's way, as their thread doesn't say): its exit LATENCY
  // ahead on the robot's motion; the table's shot to the target; less the exit's own velocity: its
  // heading (aimPsis[i], which the robot turns the turret to), elevation and exit speed
  _aim(i, tgt, table) {
    const r = this.r, b = r.body, v = b.linvel(), w = b.angvel().y, L = 0.05;
    const c = b.translation(), yaw = r.yaw + w * L, cy = Math.cos(yaw), sy = Math.sin(yaw);
    const [tx, tz] = TURRET[i];
    const ox = tx * cy + tz * sy, oz = -tx * sy + tz * cy;
    const px = c.x + v.x * L + ox, pz = c.z + v.z * L + oz;
    const vx = v.x + w * oz, vz = v.z - w * ox;
    const dx = tgt.x - px, dz = tgt.z - pz, d = Math.hypot(dx, dz) - EXIT.x;
    const row = table.lookup(d);
    if (!row) return null;
    const ux = dx / (d + EXIT.x || 1), uz = dz / (d + EXIT.x || 1);
    const sx = row.v * Math.cos(row.theta) * ux - vx, sz = row.v * Math.cos(row.theta) * uz - vz, syv = row.v * Math.sin(row.theta);
    const vr = Math.hypot(sx, syv, sz);
    return { psi: Math.atan2(-sz, sx), el: Math.asin(syv / vr) / DEG, v: vr, d };
  }

  postStep(dt, t, { on, feeding, outtake, mode }) {
    const r = this.r, T = this.t, S = SHOOTER;
    this.account(t, mode);
    // ---- the intake
    const intaking = on && !!r.cmd.intake, unjam = on && !!outtake;
    this.armGoal = intaking || unjam ? INTAKE.out : 0;
    const armReady = Math.abs(this.armDeg - INTAKE.out) < INTAKE.tol;
    // (what its driver does, emulated: shooting and not intaking, the intake folds in with its rollers
    // running until it's most of the way in (30 deg), pushing what's in its mouth up the sloped plate
    // and sweeping the box back, so the load comes back over the bowls' rims)
    const sweep = on && feeding && !!r.shot && !intaking && this.armDeg > 30;
    T.intake.cmd = !on ? 0 : unjam ? -0.5 : (intaking && armReady) || sweep ? 1 : null;

    // ---- the turrets: aiming and spinning up while the robot has a shot (r.shot: its target)
    const tgt = r.shot?.target, pass = tgt?.mode === 'pass';
    const table = pass ? r.passTable : r.hubTable;
    const point = tgt ? (pass ? { x: tgt.x, z: tgt.z } : Field.hubCenter(r.alliance)) : null;
    let anyReady = false;
    for (let i = 0; i < 2; i++) {
      const sol = on && point && !unjam ? this._aim(i, point, table) : null;
      this.want[i] = sol;
      this.aimPsis[i] = sol ? sol.psi : null;
      const sh = T.shooter[i];
      if (sol) {
        this.hoodGoal[i] = hoodFor(sol.el);
        speedLoop(sh.train, accelFor(sol.v) / S.kAccel, S.kP);
      } else {
        this.hoodGoal[i] = 0;
        speedLoop(sh.train, null);
      }
      const yawErr = sol ? Math.abs(wrapAngle(sol.psi - (r.yaw + (r.turretYaws?.[i] ?? 0)))) : Infinity;
      const ready = !!sol && yawErr < S.tolYaw * DEG && Math.abs(this.hood[i] - this.hoodGoal[i]) < S.tolHood && Math.abs(exitAt(sh.accel.omega) - sol.v) < S.tolV;
      this.readyT[i] = ready;
      // ---- that side's spindexer and kicker: feeding while it's ready and the shot's held
      const feed = on && feeding && ready && !unjam;
      if (unjam) { speedLoop(T.spin[i], -SPIN_RPS * TAU); speedLoop(T.kicker[i].train, -(KICK_RPM / 60) * TAU); }
      else if (feed) { speedLoop(T.spin[i], SPIN_RPS * TAU); speedLoop(T.kicker[i].train, (KICK_RPM / 60) * TAU); }
      else { speedLoop(T.spin[i], null); speedLoop(T.kicker[i].train, null); }
      if (feed) anyReady = true;
      // (what the robot's aiming, its HUD and drawing see)
      if (r.tws?.[i]) { r.tws[i].fly = exitAt(sh.accel.omega); r.tws[i].hood = elevationAt(this.hood[i]); r.tws[i].ok = ready; }
    }
    r.ready = this.readyT.some(Boolean);
    r.feeding = anyReady ? 1 : 0;
    r.intakeSpeed = T.intake.cmd > 0 ? 1 : T.intake.cmd < 0 ? -1 : 0;
    const k = this.readyT[1] && !this.readyT[0] ? 1 : 0;
    r.flywheel = exitAt(T.shooter[k].accel.omega);
    r.hoodDeg = elevationAt(this.hood[k]);
  }

  // how far the hopper box is out (m), for drawing it
  extNow() { return this.ext; }
  // the arm's turn from its CAD pose (rad), for drawing it
  armTurn() { return armRot(this.armDeg); }
  // the other moving parts, for drawing them: the spindexers' turns (rad, counterclockwise seen
  // from above), the hoods' swings (rad, up and over), the rollers' and wheels' speeds
  mechPose() { return { spin: this.phi.map((p, i) => p * SPIN_DIR[i]), hood: this.hood.map((h) => h * DEG), accel: this.t.shooter.map((s) => s.accel.omega) }; }
  // the load's top (for the net and the TRENCH)
  loadTop() { let top = 0; for (const b of this.held) { const y = this._local(b.pos)[1]; if (y + R > top) top = y + R; } return top; }
  // its rating (the HUD's and the AI's plans: full is when its intake can't put more in)
  capacity() { return 78; }
}
export { NET };
