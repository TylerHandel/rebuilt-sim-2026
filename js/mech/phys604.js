// 604's Toploader in play, built of its physical mechanisms (js/mech/r604.js; the shared part:
// js/mech/physical.js). Its controls are its code's (frc604/2026-public FRC-2026: RobotContainer's
// bindings, TrackAndLaunchCommand, LaunchCalculator, the intake, serializer and turret subsystems,
// DeployIntake / RetractIntake / UnjamCommand, FollowQuikplan's actions):
//   - the intake (their right trigger, the AI's intake): the arms down to -5 deg (Motion Magic at 4
//     rad/s), the rollers at 100% once within 30 deg of it; let go, up to their 10 deg standby (their
//     DeployIntake keeps it there), rollers off. It boots at 98 (their startingAngle) and stays
//     there until it's first deployed; in AUTO, between intakes, their Quikplan stows it (80 deg,
//     slowly). The hopper box rides out and in with the arms.
//   - launching (their left trigger, held: the AI's shot): TrackAndLaunch's aim (LaunchCalculator):
//     the turret's position 0.05 s ahead (their latency) on the robot's velocity and acceleration;
//     for the HUB, their table's speed and angle for the distance from the turret to its center;
//     out of their zone, a feed to their feed point on that side at 55 deg, its speed solved
//     ballistically; the shot vector less the turret's own velocity over the carpet (shoot on the
//     move) gives the turret's yaw, the launch angle (the hood at 90 less it, 15 to 35) and the
//     speed (the main roller at their lookup's rad/s, untrimmed: js/mech/r604.js). Their drive supply limit drops to
//     20 A while it's on (40 otherwise).
//   - the serializer runs (3.6 m/s, scoring and feeding alike) only while the turret's ready (yaw
//     within 5 deg, angle 3, speed 0.8 m/s; feeding 40, 15 and 2) and it's not under the TOWER, on
//     the far half of a BUMP or right behind the HUB (their zones); the rotor follows the rollers'
//     measured speed (a 10-sample average) / 1.2 at its 7.8in effective radius; their jam detection
//     (rotor over 40 A and under 2 rad/s, or the rollers over 80 A and under a tenth of their
//     target, for 0.3 s) backs the rollers out at 0.5 m/s with the rotor off for 0.1 s.
//   - let go: the turret back to 180 (facing back), the hood to 15, the launcher coasting.
//   - unjam (their D-pad down; the AI's outtake): the intake down, its rollers at -30%, the
//     serializer's rollers at -0.5 m/s.
//   - what its driver does (emulated): their right bumper (RetractIntake) when it's feeding with
//     FUEL still in it and none shot for 0.7 s: the last few rest on the shelves round the bucket
//     (its rim, the Ball Bar, the corner gussets) out of the rotor's reach, and stowing slides the
//     hopper in over them; it stays stowed until the next intake.
import { Mechanism } from './mechanism.js';
import { PhysicalRobot } from './physical.js';
import {
  build604, wheels604, shooterWheels, bumper604, pathSurfaces604, placeIntake, placeHopper, placeRotor, placeRotorSurface, placeShooter, placeTurretSurfaces,
  armRot, hopperExt, hoodRot, hoodElevation, launchFor, feedSpeed, radFor, velocityAt, rollerRad, rotorPoint,
  HALF, AXIS, PIVOT, INTAKE, ROTOR, SHOOTER, SERIALIZER, FEED, MAX_RAD, PRELOAD, HOPPER,
} from './r604.js';
import PATH from './path604.js';
import { Field } from '../field.js';
import { IN, HALF_L, HALF_W, BLUE, fw } from '../constants.js';
import { BUMPER_Y0 } from '../robotConfigs.js';
import { clamp } from '../util.js';

const DEG = Math.PI / 180, TAU = 2 * Math.PI;
// their field zones (Constants.FieldZones, blue-origin inches; the red alliance's turned round the
// field's center, as theirs): [x0, x1, y0, y1]
const ZONE = {
  TOWER: [-1 / IN, 45, 110, 180], BUMP: [180, 210, 50, 317 - 50],
  LEFT: [182.11, 1e4, 317 / 2, 1e4], RIGHT: [182.11, 1e4, -1e4, 317 / 2], BLOCKED: [182.11, 182.11 + 72, 317 / 2 - 40, 317 / 2 + 40],
};
// their feed points: 100in out from their own ALLIANCE WALL, 60in in from the side wall
const FEED_PT = { x: 100 * IN, y: 60 * IN };
const LATENCY = 0.05; // LaunchCalculator.kSystemLatencySec
// a trapezoidal profile's next setpoint (position p, velocity v) toward goal (their Motion Magic)
export function profile(p, v, goal, vmax, amax, dt) {
  const d = goal - p, dir = Math.sign(d);
  if (Math.abs(d) < 1e-6 && Math.abs(v) < amax * dt) return [goal, 0];
  const want = dir * Math.min(vmax, Math.sqrt(2 * amax * Math.abs(d)));
  const nv = v + clamp(want - v, -amax * dt, amax * dt);
  const np = p + nv * dt;
  if ((goal - np) * dir < 0) return [goal, 0];
  return [np, nv];
}
// their velocity loop (VelocityVoltage: kV x target + kP x error, volts, in motor rotations/s) on a
// shaft or train turning at w (rad/s at its output) toward target (rad/s at the output)
export function velocityDuty(motor, w, target, kV, kP) {
  const tgt = (target * motor.ratio) / TAU, rps = (w * motor.ratio) / TAU;
  return clamp((kV * tgt + kP * (tgt - rps)) / 12, -1, 1);
}

export class Physical604 extends PhysicalRobot {
  constructor(robot) {
    const t = build604(), w = wheels604(t), s = pathSurfaces604(PATH, t), sw = shooterWheels(t);
    const pathWheels = [...w.intake, ...w.rotor, ...bumper604()];
    super(robot, {
      path: new Mechanism({ body: robot.body, substeps: 6, shafts: [...new Set(pathWheels.map((x) => x.shaft)), t.pivot, t.rotor], wheels: pathWheels, surfaces: Object.values(s) }),
      shooter: new Mechanism({ body: robot.body, substeps: 6, sub: 8, shafts: [t.main, t.hoodRoller], wheels: sw }),
      preload: PRELOAD,
    });
    Object.assign(this, { t, w, s, sw });
    this.ownsIntake = true;
    this.ownsHood = true;
    this.reset();
  }

  reset() {
    super.reset();
    // boots stowed (their startingAngle 98), the hood on 15, the turret facing back, the rotor at
    // the CAD's pose
    this.armDeg = INTAKE.start; this.armSet = [INTAKE.start * DEG, 0]; this.armGoal = INTAKE.start; this.deployedOnce = false;
    this.hoodDeg = SHOOTER.hoodMin; this.hoodSet = [SHOOTER.hoodMin * DEG, 0]; this.hoodGoal = SHOOTER.hoodMin;
    this.phi = 0;
    this.targetRoller = 0; this.rollerBuf = []; this.targetRotor = 0;
    this.jam = 'NORMAL'; this.jamT = 0;
    this.compress = false; this.feedingNow = false; this.shots = this.r.stats?.shots ?? 0; this.lastShotT = 0;
    this.targetRad = 0;
    this.launching = false; this.ready = false;
    this.fv = null; this.va = null;
    for (const tr of [this.t.intake, this.t.roller, this.t.launcher]) { tr.omega = 0; tr.cmd = null; }
    for (const sh of [this.t.pivot, this.t.rotor]) { sh._omega = 0; sh.cmd = 0; }
    this.ext = hopperExt(this.armDeg);
    this.r.turretYaw = SHOOTER.yawStart * DEG;
    if (this.r.turretYaws) this.r.turretYaws[0] = this.r.turretYaw;
  }

  // its FUEL: in the frame (over the bucket's floor), in the hopper box out over the intake, or on
  // the intake's rollers in front with the arm down (not one below the bumper's bottom edge: the
  // intake can't reach it, and the robot's collider has to keep it out from under the frame, as
  // 2910's)
  where(x, y, z) {
    const inFrame = Math.abs(x) < HALF && Math.abs(z) < HALF && y > 0.14 && y < 0.8;
    const inBox = x >= HALF && x < HALF + this.ext + 0.01 && Math.abs(z) < 0.336 && y > 0.26 && y < 0.72;
    const onArm = this.armDeg < 45 && x >= HALF && x < 0.68 && Math.abs(z) < 0.3 && y > BUMPER_Y0 && y < 0.45;
    return {
      own: inFrame || inBox || onArm,
      // (held: in it; not one on the carpet or still climbing the bumper)
      held: inFrame || inBox || (onArm && y > 0.24),
      near: x > -HALF - 0.1 && x < 0.72 && Math.abs(z) < HALF + 0.1 && y < 0.85,
    };
  }
  // what the launcher's rollers might touch: up in the turret
  shooterBalls(near) { return near.filter((b) => { const [x, y, z] = this._local(b.pos); return y > 0.5 && Math.hypot(x - AXIS[0], z - AXIS[1]) < 0.16; }); }

  // the arm: their Motion Magic (one profile; their two sides independent) toward the goal (their
  // degrees), its position loop on the two X44s (kP 1 V per motor rotation, kV 0.12 on the
  // profile's speed), its weight on the pivot
  _arm(dt, on) {
    const T = this.t, I = INTAKE, m = T.pivot.motor;
    const slow = this.armGoal === I.stow;
    this.armSet = profile(...this.armSet, this.armGoal * DEG, slow ? I.mmSlow : I.mmV, I.mmA, dt);
    if (on) {
      const rot = (rad) => (rad * m.ratio) / TAU;
      const V = I.kP * (rot(this.armSet[0]) - rot(this.armDeg * DEG)) + I.kV * rot(this.armSet[1]);
      T.pivot.cmd = clamp(V / 12, -1, 1);
    } else {
      // (disabled: their X44s in brake mode, through 30:1 of gears and chain, hold it where it is)
      T.pivot.cmd = 0; T.pivot._omega = 0; this.armSet = [this.armDeg * DEG, 0];
      return;
    }
    // (its weight about the pivot: its center of mass out along the rollers' line, 11 deg below
    // the CAD's pose's horizontal)
    T.pivot.impulse(-I.mass * 9.81 * I.com * Math.cos(armRot(this.armDeg) - 0.19) * dt);
  }
  _turnArm(dt) {
    const T = this.t, I = INTAKE;
    this.armDeg += (T.pivot.omega * dt) / DEG;
    // (its stops: the soft limits their motors hold it inside, a degree past)
    if (this.armDeg < I.min - 1) { this.armDeg = I.min - 1; if (T.pivot._omega < 0) T.pivot._omega = 0; }
    if (this.armDeg > I.max + 1) { this.armDeg = I.max + 1; if (T.pivot._omega > 0) T.pivot._omega = 0; }
  }

  preStep(dt, t) {
    const r = this.r, T = this.t, on = r.enabled;
    // (put somewhere by the game, not by its motors: a staged scene, its intake down or up)
    if (this.lastDeploy !== undefined && Math.abs(r.intakeDeploy - this.lastDeploy) > 1e-6) {
      this.armDeg = INTAKE.start + (INTAKE.deploy - INTAKE.start) * r.intakeDeploy;
      this.armSet = [this.armDeg * DEG, 0]; T.pivot._omega = 0;
    }
    this._arm(dt, on);
    this.hoodSet = profile(...this.hoodSet, this.hoodGoal * DEG, SHOOTER.hoodV, SHOOTER.hoodA, dt);
    this.hoodDeg = this.hoodSet[0] / DEG;
    // pose the moving parts
    this.ext = hopperExt(this.armDeg);
    this.s.intake.setPose([...PIVOT, 0], 0, armRot(this.armDeg));
    placeIntake(this.w.intake, this.armDeg);
    placeHopper(this.s.hopper, this.armDeg);
    placeRotorSurface(this.s.rotor, this.phi);
    placeRotor(this.w.rotor, this.phi);
    const yaw = r.turretYaw || 0;
    placeTurretSurfaces(this.s, yaw, this.hoodDeg);
    placeShooter(this.sw, yaw, this.hoodDeg);
    this.stepMechanisms(dt, t);
    this._turnArm(dt);
    this.phi = (this.phi + T.rotor.omega * dt) % TAU;
    r.intakeDeploy = this.lastDeploy = clamp((INTAKE.start - this.armDeg) / (INTAKE.start - INTAKE.deploy), 0, 1);
  }

  // the robot's pose in their field frame (blue origin: x from their wall... the alliance's own,
  // turned round for red as their FieldZone does) in inches, for their zones
  _fieldIn() {
    const p = this.r.pos, s = this.r.alliance === BLUE ? 1 : -1;
    return { x: (s * p.x + HALF_L) / IN, y: (HALF_W - s * p.z) / IN };
  }
  _in(zone) { const { x, y } = this._fieldIn(), [x0, x1, y0, y1] = ZONE[zone]; return x >= x0 && x <= x1 && y >= y0 && y <= y1; }
  // their feed point on the side it's on (their LEFT_ZONE / RIGHT_ZONE split at the field's middle)
  passTarget() {
    const left = this._fieldIn().y > 317 / 2;
    return fw(FEED_PT.x, left ? FIELD_W_IN * IN - FEED_PT.y : FEED_PT.y, this.r.alliance);
  }

  // their LaunchCalculator: the turret LATENCY ahead on the robot's velocity and acceleration; the
  // shot to the target (their table's for the HUB, a 55 deg feed otherwise) less the turret's own
  // velocity; its heading (aimPsi, which the robot turns the turret to), launch angle and speed
  _aim(dt, feed) {
    const r = this.r, b = r.body, v = b.linvel(), w = b.angvel().y;
    // (the robot's acceleration, low-passed over their 20 ms loop)
    const k = 1 - Math.pow(0.5, dt / 0.02);
    if (!this.va) this.va = { vx: v.x, vz: v.z, ax: 0, az: 0 };
    this.va.ax += k * ((v.x - this.va.vx) / dt - this.va.ax); this.va.az += k * ((v.z - this.va.vz) / dt - this.va.az);
    this.va.vx = v.x; this.va.vz = v.z;
    const L = LATENCY, c = b.translation(), yaw = r.yaw + w * L;
    const px = c.x + v.x * L + 0.5 * this.va.ax * L * L, pz = c.z + v.z * L + 0.5 * this.va.az * L * L;
    // the turret's axis then, and its velocity (the robot's, plus its turn about the offset)
    const ox = AXIS[0] * Math.cos(yaw) + AXIS[1] * Math.sin(yaw), oz = -AXIS[0] * Math.sin(yaw) + AXIS[1] * Math.cos(yaw);
    const tx = px + ox, tz = pz + oz;
    const vtx = v.x + this.va.ax * L + w * oz, vtz = v.z + this.va.az * L - w * ox;
    const tgt = feed ? this.passTarget() : Field.hubCenter(r.alliance);
    const dx = tgt.x - tx, dz = tgt.z - tz, d = Math.hypot(dx, dz);
    let v0, phi0;
    if (feed) { phi0 = FEED.angle * DEG; v0 = feedSpeed(d); }
    else { const row = launchFor(d); v0 = row.v; phi0 = row.angle * DEG; }
    const ux = dx / (d || 1), uz = dz / (d || 1);
    const sx = v0 * Math.cos(phi0) * ux - vtx, sz = v0 * Math.cos(phi0) * uz - vtz, sy = v0 * Math.sin(phi0);
    const vr = Math.hypot(sx, sy, sz);
    this.aimDist = d;
    this.aimPsi = Math.atan2(-sz, sx);
    this.launchAngle = vr > 0 ? Math.asin(sy / vr) / DEG : 90;
    this.launchV = vr;
    return { d, v0, phi0 };
  }

  postStep(dt, t, { on, feeding, outtake, setpoint, mode }) {
    const r = this.r, T = this.t, S = SHOOTER;
    this.account(t, mode);
    const match = r.match;
    // ---- the intake (DeployIntake, RetractIntake, their Quikplan's flags in AUTO)
    const intaking = on && !!r.cmd.intake, unjam = on && !!outtake;
    // (its driver's RetractIntake: firing with FUEL still in it but none shot for 0.7 s, the last
    // ones are left on the shelves around the rotor's bucket (its rim, the Ball Bar, the corner
    // gussets), and stowing slides the hopper in over them; it stays in until the next intake)
    if (r.stats.shots !== this.shots) { this.shots = r.stats.shots; this.lastShotT = t; }
    if (!this.feedingNow) this.lastShotT = Math.max(this.lastShotT, t - 0.35);
    if (intaking || unjam) this.compress = false;
    else if (this.feedingNow && t - this.lastShotT > 0.7 && r.stored.length > 0) this.compress = true;
    if (intaking || unjam) { this.armGoal = INTAKE.deploy; this.deployedOnce = true; }
    else if (this.deployedOnce) this.armGoal = match?.isAuto || this.compress ? INTAKE.stow : INTAKE.standby;
    const armReady = Math.abs(this.armDeg - this.armGoal) < INTAKE.tol;
    T.intake.cmd = !on ? 0 : unjam ? -0.3 : intaking && armReady ? 1 : 0;

    // ---- TrackAndLaunch: launching while the shot's held
    const launch = on && feeding && setpoint > 0 && !!r.shot && !unjam;
    const feed = mode === 'pass';
    this._aim(dt, feed);
    this.launching = launch;
    // their drive supply limit (20 A launching, 40 otherwise)
    if (r.swerve?.d) r.swerve.d.supply = launch ? 20 : 40;
    let targetRad = 0;
    if (launch) {
      targetRad = radFor(this.launchV);
      this.hoodGoal = clamp(90 - this.launchAngle, S.hoodMin, S.hoodMax);
    } else {
      this.hoodGoal = S.hoodMin;
      // (the turret back to their 180, at their Motion Magic's 10 rad/s)
      const goal = S.yawStart * DEG, cur = r.turretYaw ?? goal;
      r.turretYaw = cur + clamp(goal - cur, -S.yawV * dt, S.yawV * dt);
      if (r.turretYaws) r.turretYaws[0] = r.turretYaw;
    }
    this.targetRad = targetRad;
    // the launcher's velocity loop (0: coast)
    T.launcher.cmd = targetRad > 0 ? velocityDuty(T.launcher.motor, Math.abs(T.main.omega), targetRad, S.kV, S.kP) : null;
    // turretReady: yaw, angle and speed within their tolerances (atVelocity never at their 600
    // rad/s ceiling)
    const yawTol = feed ? S.feedTolYaw : S.tolYaw, angleTol = feed ? S.feedTolAngle : S.tolAngle, vTol = feed ? S.feedTolV : S.tolV;
    const atYaw = Math.abs(r.aimErr ?? Infinity) < yawTol * DEG;
    const atAngle = Math.abs(this.hoodDeg - this.hoodGoal) < angleTol;
    const atV = targetRad > 0 && targetRad < MAX_RAD && Math.abs(velocityAt(targetRad) - velocityAt(Math.abs(T.main.omega))) < vTol;
    const inLaunchZone = !this._in('TOWER') && !this._in('BUMP') && !this._in('BLOCKED');
    this.ready = launch && atYaw && atAngle && atV;

    // ---- the serializer: the rollers' FUEL speed, and the rotor following them
    let ballV = this.ready && inLaunchZone ? SERIALIZER.ballV : 0;
    this.feedingNow = on && ballV > 0;
    if (unjam) ballV = SERIALIZER.unjamV;
    this._serializer(dt, on ? ballV : 0);

    // what the robot's aiming and drawing see
    r.hoodDeg = hoodElevation(this.hoodDeg);
    r.flywheel = this.exitSpeed();
    r.ready = this.ready && inLaunchZone;
    r.feeding = this.targetRoller > 0 ? 1 : 0;
    r.intakeSpeed = T.intake.cmd > 0 ? 1 : T.intake.cmd < 0 ? -1 : 0;
  }

  // SerializerSubsystem: setRollerVelocity(ballV) and its periodic
  _serializer(dt, ballV) {
    const T = this.t, Z = SERIALIZER, roller = T.roller, rotor = T.rotor;
    const target = rollerRad(ballV), active = target !== 0;
    this.targetRoller = target;
    const rollerW = T.infeed.omega; // (their sensor: the roller mechanism's speed)
    const loop = (v) => velocityDuty(roller.motor, roller.omega, v / T.infeed.k, Z.kV, Z.kP);
    // their jam state: the rotor over 40 A and under 2 rad/s, or the rollers over 80 A and under a
    // tenth of their target, for 0.3 s while active: back out at -0.5 m/s, the rotor off, 0.1 s
    const rotorJam = Math.abs(rotor.motor.current) > 40 && rotor.omega < 2;
    const rollerJam = Math.abs(roller.motor.current) / roller.motor.count > 80 && rollerW < 0.1 * target;
    if (this.jam === 'NORMAL') {
      if (active && (rotorJam || rollerJam)) {
        this.jamT += dt;
        if (this.jamT >= 0.3) { this.jam = 'JAMMED'; this.jamT = 0; this.r.stats.unjams = (this.r.stats.unjams || 0) + 1; }
      } else this.jamT = 0;
    } else if ((this.jamT += dt) >= 0.1) { this.jam = 'NORMAL'; this.jamT = 0; }
    if (this.jam === 'JAMMED') {
      roller.cmd = loop(rollerRad(Z.unjamV));
      rotor.cmd = 0; this.targetRotor = 0;
      return;
    }
    if (!active) { roller.cmd = null; this.rollerBuf = []; rotor.cmd = null; this.targetRotor = 0; return; }
    roller.cmd = loop(target);
    // the rotor: the rollers' measured FUEL speed (10-sample average) / 1.2 at its effective radius
    this.rollerBuf.push(rollerW * 0.5 * Z.rollerR);
    if (this.rollerBuf.length > 10) this.rollerBuf.shift();
    const avg = this.rollerBuf.reduce((a, b) => a + b, 0) / this.rollerBuf.length;
    this.targetRotor = Math.max(0, avg / Z.rotorPerRoller / ROTOR.effR);
    rotor.cmd = velocityDuty(rotor.motor, rotor.omega, this.targetRotor, Z.kV, Z.kP);
  }

  // how far the hopper box is out (m), for drawing it
  extNow() { return this.ext; }
  // the arm's, the rotor's and the hood's turns from their CAD poses (rad), for drawing them
  armTurn() { return armRot(this.armDeg); }
  rotorTurn() { return this.phi; }
  hoodTurn() { return hoodRot(this.hoodDeg); }
  // about how fast a FUEL leaves at the main roller's speed now (m/s; tools/mechlab604.mjs shooter)
  exitSpeed() { return EXIT_PER_MAIN_SURFACE * Math.abs(this.t.main.omega) * SHOOTER.main.r; }
  // how many it holds (tools/mechlab604.mjs hold)
  capacity() { return 63; }
}
const FIELD_W_IN = 317;
// a FUEL's exit speed over the main roller's surface speed (tools/mechlab604.mjs shooter measures it)
export const EXIT_PER_MAIN_SURFACE = 0.9;
export { rotorPoint, HOPPER };
