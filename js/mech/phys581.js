// 581's Rubble in play, built of its physical mechanisms (js/mech/r581.js; the shared part:
// js/mech/physical.js). Its controls are its code's (team581/frc-2026 comp-bot: RobotManager,
// HopperManager, the deploy, intake, conveyor, feeder, shooter and hood subsystems, PowerManager):
//   always: the drum's velocity loop (torque current) at 750 RPM idle, or their table's RPM for
//     the distance to the target while it's preparing or shooting; the hood at 13 deg, or their
//     table's angle (13 again near a TRENCH). The intake slides out at the start (their homing,
//     4 V out to its stop) and stays out (their INTAKE, 11.8in) unless it's compacting.
//   intake (held; their INTAKING): its rollers at 12 V; the floor at 2 V and the tower at -1 V
//     (holding FUEL off the drum), both stopped once the tower's beam break sees one. Let go
//     (IDLE_DEPLOYED), the rollers and floor coast and the tower brakes.
//   ball filling: once their hopper CANrange reads 8in or less (its 1 s moving average) and the
//     tower's beam is clear, the floor and tower at 1 V, intaking or not, until the beam sees one.
//   shoot (held; PREPARE_SCORE): it fires (SCORE) once aimed (5in at the HUB, at least 2 deg, or
//     about to be, turning toward it within their 0.25 s lookahead), going 1.5 m/s or less, the
//     drum within 50 RPM (300 once firing) and the hood within 3 deg, and not near a TRENCH; it
//     stops (back to PREPARE) as soon as any of that fails (their CANCEL_IN_PROGRESS_SHOT).
//     Firing: the floor and tower at 10 V, the intake's rollers at 5 V and the intake in to 8.3in;
//     after 0.7 s the intake comes in to 5in (Motion Magic), which lifts the floor's front 13.5
//     deg (compaction), its rollers coasting. Intaking while it fires (SCORE_AND_INTAKE), the
//     intake stays out with its rollers at 12 V. The last 5 s of the match (their beast mode), it
//     compacts at once. The shooter's torque current gets their feeder-current feedforward.
//   pass (out of its zone; PREPARE_FEED / FEED): their feed table to the nearer of their two feed
//     points (0.79 m out from the ALLIANCE WALL, 3 m in from a side wall; the backup ones 1.39 m
//     out, 1 m in, when the HUB's in the way); aimed within 20in (2 deg at least; 45 after a
//     second), 4 m/s or less, within 500 RPM, the hood within 5 deg; compacting to 6.915in after
//     0.7 s.
//   eject (EJECTING): intake rollers and floor at -12 V.
//   PowerManager's supply limits by state (scoring, far, feeding, beast, and their autos' intake
//     priority).
import { Mechanism } from './mechanism.js';
import { PhysicalRobot } from './physical.js';
import {
  build581, wheels581, pathSurfaces581, placeIntake, placeFloor, placeFloorSurface, placeHood, placeNet, slideOffset, floorRot, floorRate, hoodRot, hoodElevation, shotFor, feederFF,
  SLIDE, FLOOR, SHOOTER, DEPLOY, RANGES, PRELOAD, PIVOT, DRUM,
} from './r581.js';
import PATH from './path581.js';
import { Field } from '../field.js';
import { FUEL, FIELD_W, HUB, IN, fw } from '../constants.js';
import { TRENCH_OPENINGS, underTrench } from '../nav.js';

const DEG = Math.PI / 180, R = FUEL.radius, TAU = 2 * Math.PI;
const HALF_L = 0.343, HALF_W = 0.362; // its frame (27 x 28.5in)
// their swerve's limits while shooting (MAX_LINEAR_RATE_SCORING, _FEEDING), the aim tolerances
// (AimParameterUtil, Swerve: MIN_AIMED_TOLERANCE, LOOSE_TOLERANCE), the hood's (3, feeding 5)
const TOL = { scoreSpeed: 1.5, feedSpeed: 4.0, scoreGoal: 5 * IN, feedGoal: 20 * IN, minAim: 2 * DEG, loose: 45 * DEG, lookahead: 0.25, hood: 3, feedHood: 5 };
const COMPACTION_DELAY = 0.7; // HopperManagerConfig
// their hood's position loop (PositionVoltage, kP 300 V per rotation of the hood, +-10 V): an X44
// through 119.4:1 turns it at most 7530 RPM / 119.4 = 378 deg/s at 12 V
const HOOD = { kP: 300 / 360, maxV: 10, free: (7530 / 60 / ((62 / 8) * (154 / 10))) * 360 };
// PowerManagerState: supply limits (A) for the shooter, intake, deploy, hood, feeder, conveyor
const POWER = {
  IDLE: [50, 25, 25, 40, 10, 10], SCORING: [50, 20, 15, 40, 20, 20], SCORING_FAR: [50, 20, 15, 40, 10, 10],
  BEAST_MODE: [50, 10, 15, 40, 30, 30], FEEDING: [35, 20, 15, 40, 30, 30], FEEDING_FAR: [35, 20, 15, 40, 15, 15],
  PRIORITIZE_INTAKE: [50, 80, 15, 40, 10, 10],
};
const clamp = (x, a, b) => Math.max(a, Math.min(b, x));
// a trapezoidal profile's next setpoint (position p, velocity v) toward goal
function profile(p, v, goal, vmax, amax, dt) {
  const d = goal - p, dir = Math.sign(d);
  if (Math.abs(d) < 1e-6 && Math.abs(v) < 1e-6) return [goal, 0];
  const want = dir * Math.min(vmax, Math.sqrt(2 * amax * Math.abs(d)));
  const nv = v + Math.max(-amax * dt, Math.min(amax * dt, want - v));
  const np = p + nv * dt;
  if ((goal - np) * dir < 0) return [goal, 0];
  return [np, nv];
}
// a motor's duty cycle for torque current amps (per motor) at its output speed w: what their FOC
// torque-current control does (the battery's 12 V permitting)
export function dutyFor(motor, amps, w, V = 12) {
  const Eb = (w * motor.ratio) / motor.kV;
  return clamp((amps * motor.Rm + Eb) / V, -1, 1);
}

export class Physical581 extends PhysicalRobot {
  constructor(robot) {
    const t = build581(), w = wheels581(t), s = pathSurfaces581(PATH, t);
    const pathWheels = [...w.floor, ...w.tower, ...w.intake];
    super(robot, {
      path: new Mechanism({ body: robot.body, substeps: 6, shafts: [...new Set(pathWheels.map((x) => x.shaft)), t.deploy], wheels: pathWheels, surfaces: Object.values(s) }),
      shooter: new Mechanism({ body: robot.body, substeps: 6, sub: 8, shafts: [t.drum, t.hood], wheels: [w.drum, ...w.hood] }),
      preload: PRELOAD,
    });
    Object.assign(this, { t, w, s });
    this.ownsIntake = true;
    // (firing, it compacts COMPACTION_DELAY in, only with the intake let go: the AI keeps intaking
    // that long as it starts to shoot, js/opponent.js _shotHold)
    this.shotHold = COMPACTION_DELAY;
    this.ownsHood = true;
    this.reset();
  }

  reset() {
    super.reset();
    // its starting configuration: the intake all the way in (their UNHOMED, 0), the hood on its
    // stop (11, where their homing zeroes it), the floor lifted by the ramps
    this.ext = 0;
    this.deploy = { state: 'UNHOMED', goal: 0, mm: null };
    this.hoodDeg = SHOOTER.hoodCad; this.hoodGoal = SHOOTER.hoodCad;
    this.robotState = 'IDLE'; this.stateT = 0;
    this.hopperState = 'IDLE_DEPLOYED'; this.hopperT = 0;
    this.shooterState = 'IDLE'; this.shooterT = 0;
    this.atGoalT = 0; this.rangeAvg = 20 * IN; this.towerT = 0; this.tower = false;
    this.unjamT = 0; this.lastShotT = 0; this.shots = 0;
    this.power = 'IDLE';
    this.set = { intake: null, floor: null, feeder: 0 };
    for (const tr of [this.t.intake, this.t.floor, this.t.feederT, this.t.shooter]) { tr.omega = 0; tr.cmd = null; }
    this.t.deploy._omega = 0; this.t.deploy.cmd = 0;
    this.want = null;
  }

  where(x, y, z) {
    const inFrame = Math.abs(x) < HALF_L && Math.abs(z) < HALF_W && y > 0.09 && y < 0.75;
    // out front: the intake and the front of the hopper on it, as far out as it is
    const front = HALF_L + this.ext + 0.05;
    const inFront = x >= HALF_L && x < front && Math.abs(z) < 0.345 && y < 0.56;
    return {
      own: inFrame || inFront,
      held: inFrame || (inFront && y > 0.2),
      near: x > -HALF_L - 0.1 && x < front + 0.08 && Math.abs(z) < HALF_W + 0.08 && y < 0.8,
    };
  }
  shooterBalls(near) { return near.filter((b) => { const [x, y] = this._local(b.pos); return y > 0.38 && x < -0.03; }); }
  // A FUEL can come to rest on top of the intake, in the pocket between its front 2x2 tube and the
  // crossbar over its 2in roller (the CAD has nothing over it): out of the ball path, it rides in
  // and out with the intake and never feeds. A driver sees it won't and goes on, so it isn't
  // counted as the load (the AI waits for an empty hopper before it leaves to collect)
  parked(b) {
    const [x, y] = this._local(b.pos), [dx, dy] = slideOffset(this.ext);
    return x - dx > 0.5 && x - dx < 0.62 && y - dy > 0.31 && y - dy < 0.4;
  }
  account(t, mode) {
    super.account(t, mode);
    this.r.stored = this.r.stored.filter((b) => !this.parked(b));
  }

  // the deploy: their position loop (torque current, kP per inch) toward its goal, or homing out
  // at 4 V; the intake's weight down its racks (they rise 5.5 deg toward the robot)
  _deployStep(dt, on) {
    const T = this.t, D = this.deploy, m = T.deploy.motor, w = T.deploy._omega;
    const inches = this.ext / IN;
    if (!on || D.state === 'UNHOMED') T.deploy.cmd = 0; // (brake)
    else if (D.state === 'HOME_OUTWARD') T.deploy.cmd = DEPLOY.homeV / 12;
    else {
      let goal = D.goal;
      // (Motion Magic for the compaction: a profile at 200 in/s, 50 in/s^2)
      if (D.mm) { [D.mm[0], D.mm[1]] = profile(D.mm[0], D.mm[1], goal, DEPLOY.mmV, DEPLOY.mmA, dt); goal = D.mm[0]; }
      // (their peak torque currents: 100 A out, 35 A in)
      const amps = clamp(DEPLOY.kP * (goal - inches), -35, 100);
      T.deploy.cmd = dutyFor(m, amps, w);
    }
    // the floor's weight on the ramps comes back on the pinions through the cam; the intake's
    // own weight runs it out
    const floorTorque = FLOOR.mass * 9.81 * FLOOR.com * Math.cos(floorRot(this.ext) + 0.21);
    T.deploy.impulse((SLIDE.mass * 9.81 * SLIDE.dir[1] * SLIDE.pinion + floorTorque * floorRate(this.ext) * SLIDE.pinion) * dt);
  }

  preStep(dt, t) {
    const r = this.r, T = this.t, on = r.enabled;
    this._deployStep(dt, on);
    // the hood: their position loop (an X44 through 119.4:1; nothing pushes it hard)
    const v = clamp(HOOD.kP * (this.hoodGoal - this.hoodDeg), -HOOD.maxV, HOOD.maxV);
    if (on) this.hoodDeg += (v / 12) * HOOD.free * dt;
    // pose the moving parts
    const [dx, dy] = slideOffset(this.ext);
    this.s.slide.setPose([dx, dy, 0]);
    placeIntake(this.w.intake, this.ext);
    placeFloorSurface(this.s.floor, this.ext);
    placeFloor(this.w.floor, this.ext);
    this.s.hood.setPose([DRUM[0], DRUM[1], 0], 0, hoodRot(this.hoodDeg));
    placeHood(this.w.hood, this.hoodDeg);
    placeNet(this.s.net, this.ext);
    this.stepMechanisms(dt, t);
    // the intake moves as its pinions turned (to its stops: all the way in, all the way out)
    this.ext += T.deploy.omega * SLIDE.pinion * dt;
    if (this.ext < 0) { this.ext = 0; if (T.deploy._omega < 0) T.deploy._omega = 0; }
    if (this.ext > SLIDE.out) { this.ext = SLIDE.out; if (T.deploy._omega > 0) T.deploy._omega = 0; }
    r.intakeDeploy = clamp(this.ext / SLIDE.out, 0, 1);
  }

  // their hopper CANrange: the nearest FUEL in its narrow cone, forward along the hopper
  _hopperRange() {
    const { at: [x0, y0], z, reach, half } = RANGES.hopper, tn = Math.tan(half * DEG);
    let d = reach;
    for (const b of this.held) {
      const [x, y, bz] = this._local(b.pos), dx = x - x0, cone = R + dx * tn;
      if (dx > 0 && Math.abs(y - y0) < cone && Math.abs(bz - z) < cone) d = Math.min(d, dx - R);
    }
    return Math.max(0, d);
  }
  // the tower's beam break: across the tower, a FUEL in it
  _towerBeam() {
    const [x0, y0] = RANGES.tower.at;
    return this.held.some((b) => { const [x, y] = this._local(b.pos); return Math.hypot(x - x0, y - y0) < R * 0.8; });
  }

  // their feed points (FieldUtil: FEED_LEFT/RIGHT, BACKUP_FEED_LEFT/RIGHT), the nearer side's;
  // the backup when the HUB's in the way (their isFeedPathObstructed)
  _feedPoint() {
    const r = this.r, a = r.alliance;
    const pts = [[fw(0.79, 3.0, a), fw(1.39, 1.0, a)], [fw(0.79, FIELD_W - 3.0, a), fw(1.39, FIELD_W - 1.0, a)]];
    const [main, backup] = pts.reduce((p, q) => (Math.abs(p[0].z - r.pos.z) <= Math.abs(q[0].z - r.pos.z) ? p : q));
    const c = Field.hubCenter(a), dx = main.x - r.pos.x, dz = main.z - r.pos.z, L2 = dx * dx + dz * dz || 1;
    const u = clamp(((c.x - r.pos.x) * dx + (c.z - r.pos.z) * dz) / L2, 0, 1);
    const blocked = Math.hypot(r.pos.x + u * dx - c.x, r.pos.z + u * dz - c.z) < HUB.size / 2 + 0.15;
    return blocked ? backup : main;
  }
  passTarget() { return this._feedPoint(); }

  // their nearTrench: in one, or about to be (driving toward it)
  _nearTrench() {
    const r = this.r, v = r.body.linvel();
    return TRENCH_OPENINGS.some((o) => {
      const dx = o.x - r.pos.x, dz = o.z - r.pos.z;
      if (Math.abs(dx) < o.hx + HALF_L && Math.abs(dz) < o.hz + HALF_L) return true;
      return Math.abs(dx) < o.hx + HALF_L + 1.0 && Math.abs(dz) < o.hz + HALF_L + 0.3 && dx * v.x > 0 && Math.abs(v.x) > 0.3;
    });
  }

  postStep(dt, t, { on, feeding, outtake, setpoint, mode }) {
    const r = this.r, T = this.t, S = this.set, D = this.deploy;
    this.account(t, mode);
    const match = r.match;
    // ---- the sensors: the CANrange's 1 s average (50 samples) and the beam break's 0.25 s falling
    // debounce
    this.rangeAvg += (this._hopperRange() - this.rangeAvg) * Math.min(1, dt / 0.5);
    const capacity = this.rangeAvg <= RANGES.hopper.high ? 'HIGH' : this.rangeAvg <= RANGES.hopper.medium ? 'MEDIUM' : 'LOW';
    this.towerT = this._towerBeam() ? 0.25 : Math.max(0, this.towerT - dt);
    this.tower = this.towerT > 0;
    // ---- the deploy homes out at the start, then holds where it's asked
    if (on && D.state === 'UNHOMED') D.state = 'HOME_OUTWARD';
    if (D.state === 'HOME_OUTWARD' && this.ext >= SLIDE.out - 0.002) { D.state = 'READY'; this.ext = SLIDE.out; }
    const homed = D.state === 'READY';
    const deployTo = (inches, mm = false) => {
      if (!homed) return;
      const g = clamp(inches, DEPLOY.min, DEPLOY.max);
      if (mm && !D.mm) D.mm = [this.ext / IN, (T.deploy.omega * SLIDE.pinion) / IN];
      if (!mm) D.mm = null;
      D.goal = g;
    };
    const atIntake = homed && Math.abs(this.ext / IN - DEPLOY.INTAKE) <= DEPLOY.tol;

    // ---- RobotManager: what the driver asks (the AI's triggers) and whether a shot's on
    const intaking = on && !!r.cmd.intake, eject = on && !!outtake;
    const pass = mode === 'pass';
    const held = on && feeding && setpoint > 0 && !!r.shot;
    const v = r.body.linvel(), speed = Math.hypot(v.x, v.z);
    const beast = !!match?.isTeleop && match.phaseTime > 135;
    let want = null, dist = 0;
    if (held) {
      const tgt = pass ? this._feedPoint() : Field.hubCenter(r.alliance);
      // (moving, their shoot-on-the-move aims at the target moved by the shot's flight: the
      // distance to that)
      const aim = speed > 0.2 && r.shot.aim && !pass ? r.shot.aim : tgt;
      dist = Math.hypot(aim.x - r.pos.x, aim.z - r.pos.z);
      want = shotFor(pass ? 'feed' : 'score', dist);
    }
    const prev = this.robotState;
    const rpm = (T.drum.omega / TAU) * 60;
    // (WARMUP_SCORE: in teleop, full, in its zone, not intaking, facing within 90 deg: the drum to
    // the shot's speed ahead of time)
    const hubD = (() => { const c = Field.hubCenter(r.alliance); return Math.hypot(c.x - r.pos.x, c.z - r.pos.z); })();
    const warm = on && !held && !!match?.isTeleop && capacity === 'HIGH' && r.inAllianceZone?.() && !intaking && Math.abs(r.aimErr ?? Math.PI) < Math.PI / 2;
    if (!held) this.robotState = warm ? 'WARMUP_SCORE' : 'IDLE';
    else if (!this.robotState.includes(pass ? 'FEED' : 'SCORE') || this.robotState === 'WARMUP_SCORE') this.robotState = pass ? 'PREPARE_FEED' : 'PREPARE_SCORE';
    if (this.robotState !== prev) this.stateT = 0;
    this.stateT += dt;
    // the shooter's at its goal: within 50 RPM preparing, 300 firing, 500 feeding (1/15 s falling
    // debounce)
    const tol = this.robotState === 'SCORE' ? SHOOTER.rpmTolShooting : pass ? SHOOTER.rpmTolFeeding : SHOOTER.rpmTol;
    this.atGoalT = want && Math.abs(rpm - want.rpm) <= tol ? 1 / 15 : Math.max(0, this.atGoalT - dt);
    const shooterOk = this.atGoalT > 0;
    // the hood's goal: their table's angle (the idle 13 preparing near a TRENCH, or warming up)
    const nearTrench = this._nearTrench();
    this.hoodGoal = !homed ? SHOOTER.hoodCad : want && !(this.robotState === 'PREPARE_SCORE' && nearTrench) ? want.hood : SHOOTER.hoodIdle;
    const hoodOk = Math.abs(this.hoodGoal - this.hoodDeg) <= (pass ? TOL.feedHood : TOL.hood);
    // aimed (Swerve.atGoal): within the tolerance, or turning toward it and getting there within
    // their lookahead
    const err = Math.abs(r.aimErr ?? Infinity), turn = Math.abs(r.body.angvel().y);
    const aimTol = Math.max(TOL.minAim, Math.atan2(pass ? TOL.feedGoal : TOL.scoreGoal, dist || 1)), loose = pass && this.stateT > 1 && this.robotState === 'FEED' ? TOL.loose : 0;
    const aimed = err <= Math.max(aimTol, loose) || (err < Math.PI && turn * TOL.lookahead >= err && Math.sign(r.aimErr) === Math.sign(r.body.angvel().y));
    if (held) {
      const ok = pass
        ? aimed && speed <= TOL.feedSpeed && shooterOk && hoodOk && r.passClear !== false
        : aimed && speed <= TOL.scoreSpeed && shooterOk && hoodOk && !nearTrench;
      const fire = pass ? 'FEED' : 'SCORE';
      if (ok && this.robotState !== fire) { this.robotState = fire; this.stateT = 0; }
      if (!ok && this.robotState === fire) { this.robotState = pass ? 'PREPARE_FEED' : 'PREPARE_SCORE'; this.stateT = 0; }
    }
    const firing = this.robotState === 'SCORE' || this.robotState === 'FEED';
    // what its operator does (their UNJAM button): firing, with FUEL still in it but none shot for a
    // second, it holds unjam for 0.6 s (the intake back out, its rollers reversed, the floor and
    // tower at 10 V), which frees FUEL the compaction has wedged between the intake's ramps and
    // the lifted floor
    if (r.stats.shots !== this.shots) { this.shots = r.stats.shots; this.lastShotT = t; }
    if (!firing) this.lastShotT = Math.max(this.lastShotT, t - 0.5);
    if (firing && this.unjamT <= 0 && this.stateT > 1.5 && t - this.lastShotT > 1 && r.stored.length > 0) this.unjamT = 0.6;
    if (this.unjamT > 0) { this.unjamT -= dt; if (this.unjamT <= 0) this.lastShotT = t; }

    // ---- HopperManager
    const hPrev = this.hopperState;
    this.hopperState = eject ? 'EJECTING' : this.unjamT > 0 ? 'UNJAMMING'
      : firing ? (pass ? 'FEED' : 'SCORE') + (intaking ? '_AND_INTAKE' : '')
        : intaking ? 'INTAKING' : 'IDLE_DEPLOYED';
    if (this.hopperState !== hPrev) this.hopperT = 0;
    this.hopperT += dt;
    // shouldFillBalls: the tower's empty, the intake's out, and the CANrange says it's filling
    const fill = !this.tower && atIntake && (capacity === 'MEDIUM' || capacity === 'HIGH');
    switch (this.hopperState) {
      case 'IDLE_DEPLOYED': deployTo(DEPLOY.INTAKE); S.intake = null; S.floor = fill ? 1 : null; S.feeder = fill ? 1 : 0; break;
      case 'INTAKING': deployTo(DEPLOY.INTAKE); S.intake = 12; S.floor = fill ? 1 : this.tower ? null : 2; S.feeder = fill ? 1 : this.tower ? 0 : -1; break;
      case 'EJECTING': deployTo(DEPLOY.INTAKE); S.intake = -12; S.floor = -12; S.feeder = 0; break;
      case 'UNJAMMING': deployTo(DEPLOY.INTAKE); S.intake = -12; S.floor = 10; S.feeder = 10; break;
      case 'SCORE':
        S.feeder = 10;
        if (beast) { deployTo(DEPLOY.BEAST_MODE_COMPACTION); S.intake = 5; S.floor = 10; }
        else if (this.hopperT >= COMPACTION_DELAY) { deployTo(DEPLOY.SCORE_COMPACTION, true); S.intake = null; S.floor = 10; }
        else { deployTo(DEPLOY.SCORE_COMPACTION_WAITING, true); S.intake = 5; S.floor = 10; }
        break;
      case 'FEED':
        S.feeder = 10; S.floor = 10;
        if (this.hopperT >= COMPACTION_DELAY) { deployTo(DEPLOY.FEED_COMPACTION); S.intake = null; } else { if (D.goal !== DEPLOY.FEED_COMPACTION) deployTo(DEPLOY.INTAKE); S.intake = 5; }
        break;
      default: // SCORE_AND_INTAKE, FEED_AND_INTAKE
        deployTo(DEPLOY.INTAKE); S.intake = 12; S.floor = 10; S.feeder = 10;
    }
    if (!on) { S.intake = null; S.floor = null; S.feeder = 0; }

    // ---- PowerManager's supply limits
    const far = pass ? Math.hypot(this._feedPoint().x - r.pos.x, this._feedPoint().z - r.pos.z) > 7 : hubD > 4;
    this.power = firing || this.robotState.startsWith('PREPARE')
      ? (pass ? (far ? 'FEEDING_FAR' : 'FEEDING') : beast ? 'BEAST_MODE' : far ? 'SCORING_FAR' : 'SCORING')
      : match?.isAuto ? 'PRIORITIZE_INTAKE' : 'IDLE';
    const [pS, pI, pD, , pF, pC] = POWER[this.power];
    T.shooter.motor.supplyLimit = pS; T.intake.motor.supplyLimit = pI; T.deploy.motor.supplyLimit = pD;
    T.feederT.motor.supplyLimit = pF; T.floor.motor.supplyLimit = pC;

    // ---- the motors: voltages (null: neutral, coasting; the tower's neutral brakes)
    T.intake.cmd = S.intake === null ? null : S.intake / 12;
    T.floor.cmd = S.floor === null ? null : S.floor / 12;
    T.feederT.cmd = S.feeder / 12;
    // the shooter: their velocity loop in torque current, on the drum's speed (the motors' 1:1)
    const sPrev = this.shooterState;
    this.shooterState = !on ? 'OFF' : firing ? 'SHOOT' : want || warm ? 'PREPARE' : 'IDLE';
    if (this.shooterState !== sPrev) this.shooterT = 0;
    this.shooterT += dt;
    const target = this.shooterState === 'OFF' ? 0 : want ? want.rpm : warm ? shotFor('score', hubD).rpm : SHOOTER.idle;
    if (target > 0) {
      const rps = T.drum.omega / TAU, tgt = target / 60;
      const feederAmps = Math.abs(T.feederT.motor.current) / T.feederT.motor.count;
      let ff = feederFF(feederAmps);
      if (this.shooterState === 'SHOOT') {
        if (this.shooterT < 0.7 && capacity === 'HIGH') ff = Math.max(ff, 20);
        else if (this.shooterT < 0.1) ff = Math.max(ff, 15);
      }
      const amps = clamp(SHOOTER.kS + SHOOTER.kV * tgt + SHOOTER.kP * (tgt - rps) + ff, -200, 200);
      T.shooter.cmd = dutyFor(T.shooter.motor, amps, T.shooter.omega);
    } else T.shooter.cmd = null;

    this.want = want;
    // what the robot's aiming and drawing see
    r.hoodDeg = hoodElevation(this.hoodDeg);
    r.flywheel = this.exitSpeed();
    r.ready = !!want && shooterOk && hoodOk && aimed;
    r.feeding = firing ? 1 : 0;
    r.intakeSpeed = (S.intake ?? 0) > 6 ? 1 : (S.intake ?? 0) < 0 ? -1 : 0;
    // under a TRENCH its driver doesn't turn it to aim (their nearTrench holds the shot anyway, and
    // with its intake out it's longer than the TRENCH is wide)
    if (underTrench(r.pos.x, r.pos.z, 0.2)) r.aimOverride = null;
  }

  // how far out the intake is (m along its racks), for the hopper drawn on it
  extNow() { return this.ext; }
  // how high the load holds its net up (for the TRENCH): the top of the FUEL under it, as far as
  // the net's slack lets it rise (one pressing past that stretches the bungee, which gives under a
  // TRENCH arm); 0 with none under it. The rest of the load is under the polycarbonate top, which
  // fits under the arm (js/mech/r581.js NET)
  loadTop() {
    const n = this.s.net;
    let top = 0;
    for (const b of this.held) {
      const [x, y, z] = this._local(b.pos);
      if (n.under(x, z)) top = Math.max(top, Math.min(y + R, n.at(x, z)[0]));
    }
    return top;
  }
  // the floor's lift and the hood's turn from their CAD poses (rad), for drawing them
  floorTurn() { return floorRot(this.ext); }
  hoodTurn() { return hoodRot(this.hoodDeg); }

  // about how fast a FUEL leaves at the drum's speed now (m/s; tools/mechlab581.mjs shooter)
  exitSpeed() { return EXIT_PER_DRUM_SURFACE * Math.abs(this.t.drum.omega) * SHOOTER.drum.r; }

  // how many it holds (tools/mechlab581.mjs hold, and intaking in matches)
  capacity() { return 50; }
}
// a FUEL's exit speed over the drum's surface speed (tools/mechlab581.mjs shooter measures it)
export const EXIT_PER_DRUM_SURFACE = 0.87;
