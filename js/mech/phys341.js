// 341's Miss Daisy XXIV in play, built of its physical mechanisms (js/mech/r341.js; the shared
// part: js/mech/physical.js). Its controls are its code's (Team341/FRC2026-Public: Intake,
// Shooter, Turret and RobotContainer's bindings):
//   - the intake down (their 149 deg; their encoder's zeroed there at boot) and held stiff;
//   - shooting (the driver's right trigger, the AI's shot): the flywheel's velocity loop at their
//     table's RPM for the distance from the turret to the target, the hood to their table's angle
//     (stowed under a TRENCH); once it's within 130 RPM (debounced 0.1 s) and the turret's on
//     target, SHOOTING: the intake's rollers and the top rollers / uptake run at 12 V
//     (DEPLOYED_INTAKING, their intake's default command while it shoots) and the feeder (the
//     powered omni) at 3 V while the hood's within 3 deg;
//   - their unjam: while it intakes and shoots, the top rollers or the feeder stalled (under 500
//     RPM at over 75 A) for 0.2 s pulses them out for 0.25 s (feeder -3 V, rollers -12 V, top
//     rollers -6 V);
//   - let go: the flywheel stays at speed (AT_SPEED), the hood stowed, everything else stopped.
// Their robot only intakes while it shoots (it's hopperless: the right trigger does both); the
// AI's intake command runs its rollers too, as if the driver held the trigger with the shot not
// yet on (the rollers and top rollers at 12 V, nothing fed).
import { Mechanism } from './mechanism.js';
import { PhysicalRobot } from './physical.js';
import {
  build341, wheels341, shooterWheels, placeShooter, placeIntake, intakeRot, hoodElevation, pathSurfaces341, placeTurretSurfaces,
  shotFor, HALF, INTAKE, SHOOTER, PRELOAD, PIVOT,
} from './r341.js';
import PATH from './path341.js';
import { Field } from '../field.js';
import { underTrench } from '../nav.js';
import { clamp } from '../util.js';

const DEG = Math.PI / 180;
const R = 0.075;
// the hood: an X44 through 133.7:1 (about 340 deg/s free), a profile toward its goal
const HOOD_MM = { v: 300, a: 3000 };
const profile = (p, v, goal, vmax, amax, dt) => {
  const d = goal - p, stop = (v * v) / (2 * amax);
  let want = Math.sign(d) * Math.min(vmax, Math.sqrt(2 * amax * Math.abs(d)));
  if (Math.abs(d) < 1e-3 && Math.abs(v) < amax * dt) return [goal, 0];
  if (Math.abs(d) <= stop && Math.sign(v) === Math.sign(d)) want = Math.sign(d) * Math.sqrt(2 * amax * Math.abs(d));
  const nv = v + clamp(want - v, -amax * dt, amax * dt);
  return [p + nv * dt, nv];
};
// FUEL leave at this fraction of the flywheel's surface speed (tools/mechlab341.mjs shooter: 0.68-0.73
// over their tables' rows, at the elevation the CAD's geometry gives; their HUB table's shots
// come down within 0.05 m of its distance at 1.5, 2.4, 4.2 and 5.5 m, 0.2 m short at 3-3.5 m)
export const EXIT_PER_FLY_SURFACE = 0.7;

// the arm's position loop toward goal (their angles, deg; positive swings it down), its X44 at 12 V
// at most: their gentle slot while it's moving, the stiff one within 30 deg (holding it against
// FUEL hitting it); and the arm's weight on the pivot
export function servoArm(T, deg, goal, dt) {
  const a = intakeRot(deg); // (the arm's turn from folded up; its center of mass straight up when folded)
  if (goal !== null) {
    const err = goal - deg, k = Math.abs(err) > 30 ? 0.25 : 3;
    T.pivot.cmd = clamp((k * err) / 12, -1, 1);
  } else T.pivot.cmd = 0;
  // gravity: positive swings it down, so its weight helps once it's past upright
  T.pivot.impulse(INTAKE.mass * 9.81 * INTAKE.com * -Math.sin(a) * dt);
}
export function turnArm(T, deg, dt) {
  deg += (T.pivot.omega * dt) / DEG;
  if (deg > INTAKE.down + 1) { deg = INTAKE.down + 1; if (T.pivot._omega > 0) T.pivot._omega = 0; } // (the carpet, the hard stop)
  if (deg < INTAKE.stowed) { deg = INTAKE.stowed; if (T.pivot._omega < 0) T.pivot._omega = 0; }
  return deg;
}

export class Physical341 extends PhysicalRobot {
  constructor(robot) {
    const t = build341(), w = wheels341(t), s = pathSurfaces341(PATH, t);
    const pathWheels = [...w.pivotRow, ...w.arm, ...w.table, ...w.uptake, ...w.omnis];
    const sw = shooterWheels(t);
    super(robot, {
      path: new Mechanism({ body: robot.body, substeps: 6, shafts: [...new Set(pathWheels.map((x) => x.shaft)), t.pivot], wheels: pathWheels, surfaces: Object.values(s) }),
      shooter: new Mechanism({ body: robot.body, substeps: 6, sub: 8, shafts: [t.fly, t.stealth, t.feed], wheels: sw }),
      preload: PRELOAD,
    });
    Object.assign(this, { t, w, s, sw });
    this.ownsIntake = true;
    this.ownsHood = true;
    this.reset();
  }

  reset() {
    super.reset();
    // down from the start (their encoder's zeroed there)
    this.armDeg = INTAKE.down; this.armGoal = INTAKE.down;
    this.hoodDeg = 0; this.hoodSet = [0, 0]; this.hoodGoal = 0;
    this.state = 'stopped'; // their shooter: stopped, spinning up, shooting, at speed, pulsing out
    this.targetRpm = 0; this.atSpeedT = 0; this.jamT = 0; this.pulseT = 0;
    for (const tr of [this.t.intake, this.t.serializer, this.t.feederT, this.t.shooter]) { tr.omega = 0; tr.cmd = 0; }
    this.t.pivot._omega = 0; this.t.pivot.cmd = 0;
  }

  where(x, y, z) {
    const inFrame = Math.abs(x) < HALF && Math.abs(z) < HALF && y > 0.13 && y < 0.8;
    // out on the arm, in front of the bumper (the intake's 24in, a bit more each side for the
    // mecanums' reach)
    const onArm = this.armDeg > 90 && x >= HALF && x < HALF + 0.33 && Math.abs(z) < 0.32 && y < 0.45;
    return { own: inFrame || onArm, held: inFrame || (onArm && y > 0.12), near: x > -HALF - 0.1 && x < HALF + 0.4 && Math.abs(z) < HALF + 0.08 && y < 0.85 };
  }
  shooterBalls(near) { return near.filter((b) => this._local(b.pos)[1] > 0.33); }

  preStep(dt, t) {
    const r = this.r, T = this.t;
    servoArm(T, this.armDeg, r.enabled ? this.armGoal : null, dt);
    this.hoodSet = profile(...this.hoodSet, this.hoodGoal, HOOD_MM.v, HOOD_MM.a, dt);
    this.hoodDeg = this.hoodSet[0];
    // pose the moving parts
    this.s.intake.setPose([PIVOT[0], PIVOT[1], 0], 0, intakeRot(this.armDeg));
    placeIntake(this.w.arm, this.armDeg);
    const yaw = r.turretYaw || 0;
    placeTurretSurfaces(this.s, yaw, this.hoodDeg);
    placeShooter(this.sw, yaw, this.hoodDeg);
    this.stepMechanisms(dt, t);
    this.armDeg = turnArm(T, this.armDeg, dt);
    r.intakeDeploy = clamp(this.armDeg / INTAKE.down, 0, 1);
  }

  postStep(dt, t, { on, feeding, outtake, setpoint, mode }) {
    const r = this.r, T = this.t;
    this.account(t, mode);
    const pass = mode === 'pass';
    const wantShot = on && feeding && setpoint > 0 && !!r.shot;
    // their RPM and hood for the distance from the turret to the target
    if (r.shot) {
      const tgt = pass ? r.shot.target : Field.hubCenter(r.alliance), tp = this._world([-0.102, 0, -0.076]);
      const want = shotFor(pass ? 'ferry' : 'hub', Math.hypot(tgt.x - tp.x, tgt.z - tp.z));
      if (wantShot || this.state !== 'stopped') this.targetRpm = want.rpm;
      this.wantHood = want.hood;
    }
    const rpm = (Math.abs(T.fly.omega) * 60) / (2 * Math.PI);
    // (their atSpeed: within 130 RPM, its falling edge debounced 0.1 s)
    this.atSpeedT = Math.abs(rpm - this.targetRpm) <= SHOOTER.tol ? 0.1 : Math.max(0, this.atSpeedT - dt);
    const atSpeed = this.atSpeedT > 0 && this.targetRpm > 0;
    const turretReady = Math.abs(r.aimErr ?? Infinity) < 3 * DEG;
    const inTrench = underTrench(r.pos.x, r.pos.z, 0);
    if (!on) this.state = 'stopped';
    else if (wantShot) this.state = this.pulseT > 0 ? 'pulsing' : atSpeed && turretReady ? 'shooting' : 'spinning up';
    else if (this.state !== 'stopped') this.state = 'at speed';
    const shooting = this.state === 'shooting' || this.state === 'pulsing';
    // the hood: their table's angle while it shoots (stowed under a TRENCH), stowed otherwise
    this.hoodGoal = (this.state === 'shooting' || this.state === 'spinning up' || this.state === 'pulsing') && !inTrench ? this.wantHood ?? 0 : 0;
    // the intake: their default command runs it while it shoots; the AI's intake command too
    const intaking = on && (shooting || !!r.cmd.intake);
    // their unjam: the top rollers or the feeder stalled for 0.2 s while it intakes and shoots
    const stalled = (tr) => (Math.abs(tr.omega) * 60) / (2 * Math.PI) < 500 && Math.abs(tr.motor.current) > 75;
    this.jamT = intaking && shooting && (stalled(T.serializer) || stalled(T.feederT)) ? this.jamT + dt : 0;
    if (this.jamT > 0.2 && this.pulseT <= 0) { this.pulseT = 0.25; this.jamT = 0; r.stats.unjams = (r.stats.unjams || 0) + 1; }
    this.pulseT = Math.max(0, this.pulseT - dt);
    const pulsing = this.pulseT > 0;
    T.intake.cmd = !on ? 0 : outtake || pulsing ? -1 : intaking ? 1 : 0;
    T.serializer.cmd = !on ? 0 : outtake || pulsing ? -0.5 : intaking ? 1 : 0;
    T.feederT.cmd = !on ? 0 : pulsing ? -3 / 12 : this.state === 'shooting' && Math.abs(this.hoodDeg - this.hoodGoal) <= 3 ? 3 / 12 : 0;
    // the flywheel's velocity loop (no reverse voltage: it coasts down)
    if (this.state !== 'stopped' && this.targetRpm > 0) {
      const rps = T.fly.omega / (2 * Math.PI) * -1, tgt = this.targetRpm / 60;
      T.shooter.cmd = clamp((SHOOTER.kV * tgt + SHOOTER.kP * (tgt - rps)) / 12, 0, 1);
    } else T.shooter.cmd = 0;
    this.armGoal = INTAKE.down;
    // what the robot's aiming and drawing see
    r.hoodDeg = hoodElevation(this.hoodDeg);
    r.flywheel = this.exitSpeed();
    r.ready = this.state === 'shooting';
    r.feeding = T.feederT.cmd > 0 ? 1 : 0;
    r.intakeSpeed = T.intake.cmd > 0 ? 1 : T.intake.cmd < 0 ? -1 : 0;
  }

  // the arm's turn from the CAD's pose (rad), for drawing it
  armTurn() { return intakeRot(this.armDeg); }
  // about how fast a FUEL leaves at the flywheel's speed now (m/s; tools/mechlab341.mjs shooter)
  exitSpeed() { return EXIT_PER_FLY_SURFACE * Math.abs(this.t.fly.omega) * SHOOTER.fly.d / 2; }
  // how many it holds in its path (tools/mechlab341.mjs hold)
  capacity() { return 8; }
}
export { R };
