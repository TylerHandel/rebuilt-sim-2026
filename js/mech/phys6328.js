// 6328's Darwin in play, built of its physical mechanisms (js/mech/r6328.js; the shared part:
// js/mech/physical.js). Its controls are its code's (Mechanical-Advantage/RobotCode2026Public:
// RobotContainer's bindings, Slamtake, Hopper, Kicker, Flywheel, Hood, LaunchCalculator,
// CompactingCommands, DriveCommands):
//   always: the hood at 10 deg (Hood.minAngle; their default command) and the flywheel idling at 80
//     rad/s open loop (LaunchCalculator.idleSpeed; Flywheel.runIdle, their default command).
//   intake (left trigger, held): the slam down (DEPLOY: their position loop to 17.4 deg, its hard
//     stop) and the roller at 13 V (the battery's all). Let go, the roller stops; the arm stays down.
//   launch (left bumper, held): the flywheel and hood track LaunchCalculator's maps for the
//     launcher's distance to the HUB (or to their pass point, past the HUB's line): the flywheel's
//     setpoint profiled up at what its 160 A supply budget allows (60 A in AUTO), then bang-bang
//     past 3.25 m (twice the feedforward below the setpoint, the feedforward above) or their velocity
//     loop (4 rad/s over); the robot turns its launcher at the target (the AI's aim). Once valid (in
//     range, not behind a HUB or under the TOWER), the hood within 1.5 deg, the flywheel's setpoint at
//     its goal (passing: within 40 rad/s), aimed within 10 deg, not tilted (5 deg), it launches (0.1 s
//     falling debounce): the floor at 12 V, the kickers' velocity loops at the map's surface speed
//     (6 m/s, 3 from 4 m out), and 0.35 s in the compaction: the slam retracts slowly (0.88 rad/s
//     slew) with the intake roller running for 1.2 s, the hopper's front coming in with it and
//     pushing the load back. Let go, the slam goes back down and the kickers run back for 0.5 s.
//   outtake (B): floor -12 V, kickers back, roller -6 V.
//   under a TRENCH the hood is held at 10 (Hood.autonomousCommand's TrenchBounds; the robot is
//     21.9in at the hood's top either way).
import { Mechanism } from './mechanism.js';
import { PhysicalRobot } from './physical.js';
import {
  build6328, wheels6328, pathSurfaces6328, placeIntake, placeHood, placeNet, intakeRot, hoodRot, hoodElevation, slideExt, slideRate, shotFor,
  INTAKE, SHOOTER, KICKER, KICK, BACK_R, PRELOAD, PIVOT, DRUM, LAUNCHER_X, RANGE, HALF_L, SLIDE_MASS,
} from './r6328.js';
import PATH from './path6328.js';
import { MOTORS } from '../drivetrain.js';
import { Field } from '../field.js';
import { FUEL, HUB, IN, fw } from '../constants.js';
import { underTrench } from '../nav.js';

const DEG = Math.PI / 180, R = FUEL.radius, TAU = 2 * Math.PI;
const clamp = (x, a, b) => Math.max(a, Math.min(b, x));
const HALF = 0.343; // the hopper's half width (its walls), about
// Slam.java and Slamtake.java: the position loop (PositionVoltage, kP V per rotation of the arm) and
// the retract slews (rad/s)
const SLAM = { kP: 80, retract: 3.0, slow: 0.88, fast: 1.3, tol: 4 };
// Hood.java: its loop (PositionTorqueCurrentFOC, kP 30000 A per rotation) saturates on its 12 A supply
// limit for any real move: an X44 through 148:1 at its 12 A is ~300 deg/s free; it moves at about
// that, 1.5 deg tolerance (an estimate of its speed)
const HOOD = { rate: 250, tol: 1.5 };
// DriveCommands: the yaw tolerance to launch (10 deg at the HUB and passing), CompactingCommands'
// delay and the intake's run, the kickers' run back after a launch (RobotContainer)
const TOL = { yaw: 10 * DEG, passFly: 40, tilt: 5 * DEG };
const COMPACT = { delay: 0.35, roll: 1.2 };
const KICK_BACK = 0.5;
// their pass point (LaunchCalculator.getPassingTarget): 37in out from their ALLIANCE WALL, 65in in
// from the nearer side wall
const PASS_PT = { x: 37 * IN, y: 65 * IN };

// the flywheel's motors (four X60s, FOC) as WPILib's DCMotor sees them, for their power budget
const X60 = MOTORS.krakenX60, NFLY = 4;
const FLY = { Kv: X60.free / 12, R: 12 / (X60.stallCurrent * NFLY), kt: X60.stallTorque / X60.stallCurrent, moi: 0.02 };

// Flywheel.runVelocity: the setpoint's profile (power budget from the supply limit), feedforward,
// bang-bang or velocity loop; returns the volts. st: { sp, acc, init } (their setpointVel,
// filteredAccel), omega: the drum's speed (rad/s, the motors' 1:1)
export function flywheelVolts(st, goal, omega, dt, { bangBang = false, idle = false, supply = SHOOTER.supplyTeleop, vBus = 12 } = {}) {
  const S = SHOOTER;
  const P = supply * vBus * S.eff, emf = st.sp / FLY.Kv;
  let Imax = (-emf + Math.sqrt(emf * emf + 4 * FLY.R * P)) / (2 * FLY.R);
  Imax = Math.min(Imax, Math.max(0, (vBus - emf) / FLY.R));
  const aMax = clamp((FLY.kt * Imax - FLY.kt * (S.kS / FLY.R)) / FLY.moi, 0, S.maxAccel);
  const step = aMax * dt, err = goal - st.sp;
  let raw;
  if (Math.abs(err) <= step) { st.sp = goal; raw = err / dt; } else { st.sp += Math.sign(err) * step; raw = Math.sign(err) * aMax; }
  st.acc = st.init ? st.acc + (raw - st.acc) * dt / 0.05 : raw;
  st.init = true;
  st.atGoal = Math.abs(st.sp - goal) <= 2;
  let v = Math.sign(st.sp) * S.kS + st.sp * S.kV + st.acc * S.kA;
  if (bangBang) { if (omega < st.sp) v *= S.bangBang; } else if (!idle) v += (S.kP * (st.sp + S.pidOffset - omega)) / TAU;
  return clamp(v, -12, 12);
}
// the kickers' velocity loops at a surface speed (m/s; negative: back), on the Talons at 1 kHz
// (js/mech/mechanism.js velLoop: their kV per rad/s is 2 pi kV per rotation/s); null: stopped (0 V,
// brake)
export function kickers(T, surf) {
  const set = (tr, r, g, k) => {
    if (surf === null) { tr.vel = null; tr.cmd = 0; return; }
    const per = Math.abs(k) / TAU; // (roller rotations per radian of the motor)
    tr.vel = { target: surf / r / Math.abs(k), kV: g.kV * TAU, kP: g.kP, kS: g.kS, per };
    tr.cmd = 0;
  };
  set(T.frontT, KICK.r, KICKER.front, T.kick.k);
  set(T.backT, BACK_R, KICKER.back, T.back[0].k);
}
// the slam's loop (PositionVoltage: kP 80 V per rotation of the arm; positive up), the arm's weight
// on it; goal null: brake
export function servoArm(T, deg, goal, dt) {
  const a = (deg + INTAKE.comDeg) * DEG;
  T.pivot.cmd = goal === null ? 0 : clamp((SLAM.kP * (goal - deg)) / 360 / 12, -1, 1);
  T.pivot.impulse(-INTAKE.mass * 9.81 * INTAKE.com * Math.cos(a) * dt);
}
// after the step: the arm turns as its shaft did, between its hard stops
export function turnArm(T, deg, dt) {
  deg += (T.pivot.omega * dt) / DEG;
  if (deg < INTAKE.down) { deg = INTAKE.down; if (T.pivot._omega < 0) T.pivot._omega = 0; }
  if (deg > INTAKE.up) { deg = INTAKE.up; if (T.pivot._omega > 0) T.pivot._omega = 0; }
  return deg;
}

export class Physical6328 extends PhysicalRobot {
  constructor(robot) {
    const t = build6328(), w = wheels6328(t), s = pathSurfaces6328(PATH, t);
    const pathWheels = [...w.floor, ...w.kick, ...w.intake];
    super(robot, {
      path: new Mechanism({ body: robot.body, substeps: 6, shafts: [...new Set(pathWheels.map((x) => x.shaft)), t.pivot], wheels: pathWheels, surfaces: Object.values(s) }),
      shooter: new Mechanism({ body: robot.body, substeps: 6, sub: 8, shafts: [t.drum, t.hood], wheels: [w.drum, ...w.hood] }),
      preload: PRELOAD,
    });
    Object.assign(this, { t, w, s });
    this.ownsIntake = true;
    this.ownsHood = true;
    // (firing, it compacts 0.35 s in, its intake coming up: the AI keeps intaking that long)
    this.shotHold = COMPACT.delay;
    this.reset();
  }

  reset() {
    super.reset();
    // its starting configuration: the slam up (where their homing zeroes it, slamMaxAngle)
    this.armDeg = INTAKE.up; this.armSet = INTAKE.up; this.slamGoal = 'RETRACT';
    this.hoodDeg = SHOOTER.hoodMin; this.hoodGoal = SHOOTER.hoodMin;
    this.fly = { sp: 0, acc: 0, init: false, atGoal: false };
    this.launching = false; this.launchT = 0; this.tolT = 0; this.kickBackT = 0; this.wasHeld = false;
    this.rollerOn = false;
    for (const tr of [this.t.intake, this.t.floor, this.t.backT, this.t.frontT, this.t.flywheel]) { tr.omega = 0; tr.cmd = 0; tr.vel = null; }
    this.t.pivot._omega = 0; this.t.pivot.cmd = 0;
    this.want = null;
  }

  where(x, y, z) {
    const inFrame = Math.abs(x) < HALF_L + 0.01 && Math.abs(z) < HALF && y > 0.12 && y < 0.8;
    // out front: the intake and the hopper's front out over it
    const front = HALF_L + 0.01 + slideExt(this.armDeg) + 0.06;
    const inFront = x >= HALF_L + 0.01 && x < Math.max(front, 0.7) && Math.abs(z) < HALF && y < 0.75;
    return {
      own: inFrame || inFront,
      held: inFrame || (inFront && y > 0.22 && x < HALF_L + slideExt(this.armDeg) + 0.01),
      near: x > -HALF_L - 0.1 && x < 0.78 && Math.abs(z) < HALF + 0.08 && y < 0.85,
    };
  }
  shooterBalls(near) { return near.filter((b) => { const [x, y] = this._local(b.pos); return y > 0.38 && x < 0.0; }); }
  // the top of the load (robot frame), for the TRENCH
  loadTop() {
    let top = 0;
    for (const b of this.held) { const [x, y] = this._local(b.pos); if (x > -0.05) top = Math.max(top, y + R); }
    return top;
  }

  preStep(dt, t) {
    const r = this.r, T = this.t, on = r.enabled;
    // the slam: their position loop toward its goal (DEPLOY straight to 17.4; RETRACT slewed)
    if (this.slamGoal === 'DEPLOY') this.armSet = INTAKE.down;
    else {
      const rate = (this.slamGoal === 'RETRACT_SLOW' ? SLAM.slow : SLAM.retract) / DEG;
      this.armSet = Math.min(INTAKE.up, this.armSet + rate * dt);
    }
    servoArm(T, this.armDeg, on ? this.armSet : null, dt);
    // the hood: toward its goal at its rate
    this.hoodDeg += clamp(this.hoodGoal - this.hoodDeg, -HOOD.rate * dt, HOOD.rate * dt);
    // pose the moving parts: the arm, the hopper's front on its pin (its mass felt at the arm), the hood
    const ext = slideExt(this.armDeg), k = slideRate(this.armDeg);
    this.s.slide.slideK = k;
    T.pivot.inertia = INTAKE.J + SLIDE_MASS * k * k;
    this.s.intake.setPose([PIVOT[0], PIVOT[1], 0], 0, intakeRot(this.armDeg));
    this.s.ramp.setPose([PIVOT[0], PIVOT[1], 0], 0, intakeRot(this.armDeg));
    placeIntake(this.w.intake, this.armDeg);
    this.s.slide.setPose([ext, 0, 0]);
    placeNet(this.s.net, ext);
    this.s.hood.setPose([DRUM[0], DRUM[1], 0], 0, hoodRot(this.hoodDeg));
    placeHood(this.w.hood, this.hoodDeg);
    this.stepMechanisms(dt, t);
    this.armDeg = turnArm(T, this.armDeg, dt);
    r.intakeDeploy = clamp((INTAKE.up - this.armDeg) / (INTAKE.up - INTAKE.down), 0, 1);
  }

  // their pass point (LaunchCalculator.getPassingTarget), on the robot's side of the field
  passTarget() {
    const r = this.r, a = fw(PASS_PT.x, PASS_PT.y, r.alliance), b = fw(PASS_PT.x, 8.069 - PASS_PT.y, r.alliance);
    return Math.abs(a.z - r.pos.z) < Math.abs(b.z - r.pos.z) ? a : b;
  }
  // where the launcher is (world): 0.151 m behind the robot's center
  _launcher() {
    const r = this.r;
    return { x: r.pos.x + Math.cos(r.yaw) * LAUNCHER_X, z: r.pos.z - Math.sin(r.yaw) * LAUNCHER_X };
  }

  postStep(dt, t, { on, feeding, outtake, setpoint, mode }) {
    const r = this.r, T = this.t;
    this.account(t, mode);
    const auto = !!r.match?.isAuto;
    const intaking = on && !!r.cmd.intake;
    const held = on && feeding && !!r.shot;
    const pass = mode === 'pass';
    // ---- the launch: LaunchCalculator's parameters for the launcher's distance (the AI's aim point,
    // which leads the target for the robot's motion as their lookahead does)
    let want = null, dist = 0, valid = false;
    if (held) {
      const tgt = pass ? this.passTarget() : Field.hubCenter(r.alliance);
      const aim = !pass && r.shot.aim ? r.shot.aim : tgt, L = this._launcher();
      dist = Math.hypot(aim.x - L.x, aim.z - L.z);
      want = shotFor(pass ? 'pass' : 'hub', dist);
      const [lo, hi] = pass ? RANGE.pass : RANGE.hub;
      valid = dist >= lo && dist <= hi;
    }
    const trench = underTrench(r.pos.x, r.pos.z, 0.4);
    this.hoodGoal = want && !trench ? clamp(want.hood, SHOOTER.hoodMin, SHOOTER.hoodMax) : SHOOTER.hoodMin;
    // the flywheel: tracking (bang-bang far, at the HUB), or idling at 80 rad/s; off when disabled
    const supply = auto ? SHOOTER.supplyAuto : SHOOTER.supplyTeleop, omega = T.drum.omega;
    let volts = 0;
    if (!on) { this.fly.sp = omega; this.fly.init = false; T.flywheel.cmd = null; } else {
      if (want) volts = flywheelVolts(this.fly, want.fly, omega, dt, { bangBang: !pass && dist > SHOOTER.bangMin, supply });
      else volts = flywheelVolts(this.fly, SHOOTER.idle, omega, dt, { idle: true, supply });
      T.flywheel.cmd = volts / 12;
    }
    // ---- in tolerance (RobotContainer's inLaunchingTolerance, 0.1 s falling debounce)
    const hoodOk = Math.abs(this.hoodDeg - this.hoodGoal) <= HOOD.tol;
    const flyOk = want && (pass ? Math.abs(omega - want.fly) < TOL.passFly : this.fly.atGoal);
    const aimed = Math.abs(r.aimErr ?? Infinity) <= TOL.yaw;
    const q = r.body.rotation(), tilt = Math.acos(clamp(1 - 2 * (q.x * q.x + q.z * q.z), -1, 1));
    const inTol = !!want && hoodOk && flyOk && aimed && tilt < TOL.tilt;
    this.tolT = inTol ? 0.1 : Math.max(0, this.tolT - dt);
    const go = held && valid && this.tolT > 0 && r.passClear !== false;
    if (go && !this.launching) { this.launching = true; this.launchT = 0; }
    if (!go && this.launching) { this.launching = false; this.kickBackT = KICK_BACK; }
    if (this.launching) this.launchT += dt;
    if (!held && this.wasHeld) { this.slamGoal = 'DEPLOY'; if (!this.launching) this.kickBackT = Math.max(this.kickBackT, 0); }
    this.wasHeld = held;
    // ---- the slam and the intake roller
    let roller = 0;
    if (intaking) { this.slamGoal = 'DEPLOY'; roller = 13; }
    // (CompactingCommands.compact: 0.35 s into a launch, retract slowly, the roller running 1.2 s)
    if (this.launching && this.launchT >= COMPACT.delay && !intaking) {
      this.slamGoal = 'RETRACT_SLOW';
      if (this.launchT < COMPACT.delay + COMPACT.roll) roller = 13;
    }
    // (the AI stows it to go under a TRENCH with nothing to do: the driver's right trigger, RETRACT)
    if (!intaking && !this.launching && r.cmd.stow) this.slamGoal = 'RETRACT';
    // ---- the floor and the kickers
    let floor = 0, kick = null;
    if (this.launching) { floor = 12; kick = want.kicker; } else if (this.kickBackT > 0) { this.kickBackT -= dt; kick = -(want?.kicker ?? 6); }
    if (on && outtake) { floor = -12; kick = -6; roller = -6; }
    if (!on) { floor = 0; kick = null; roller = 0; }
    T.intake.cmd = clamp(roller / 12, -1, 1);
    T.floor.cmd = floor / 12;
    kickers(T, kick);
    this.want = want;
    // what the robot's aiming and drawing see
    r.hoodDeg = hoodElevation(this.hoodDeg);
    r.flywheel = this.exitSpeed();
    r.ready = inTol && valid;
    r.feeding = this.launching ? 1 : 0;
    r.intakeSpeed = roller > 6 ? 1 : roller < 0 ? -1 : 0;
  }
  // what it's waiting on, for the HUD
  shotStatus() {
    if (!this.want) return null;
    if (this.launching) return 'LAUNCHING';
    if (Math.abs(this.hoodDeg - this.hoodGoal) > HOOD.tol) return 'Hood';
    if (!this.fly.atGoal) return 'Spinning up';
    return null;
  }
  // the arm's turn from the CAD's pose (rad) and the hood's, for drawing them
  armTurn() { return intakeRot(this.armDeg); }
  hoodTurn() { return hoodRot(this.hoodDeg); }
  // how far out the hopper's front is (m)
  extNow() { return slideExt(this.armDeg); }
  // about how fast a FUEL leaves at the drum's speed now (m/s; tools/mechlab6328.mjs shooter)
  exitSpeed() { return EXIT_PER_DRUM_SURFACE * Math.abs(this.t.drum.omega) * SHOOTER.drum.r; }
  // how many it holds, about: the binder's ~75 with the net up. Only a rating, for the HUD and the
  // AI's plans (tools/mechlab6328.mjs hold, intakefull)
  capacity() { return 75; }
}
// a FUEL's exit speed over the drum's surface speed (tools/mechlab6328.mjs shooter measures it)
export const EXIT_PER_DRUM_SURFACE = 0.75;
