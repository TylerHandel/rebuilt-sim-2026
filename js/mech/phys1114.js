// 1114's Simbot Tim in play, built of its physical mechanisms (js/mech/r1114.js; the shared part:
// js/mech/physical.js). Its controls are its code's (Simbotics/2026-Simbot-Tim: RobotContainer's
// bindings, CommandFactory, the intake, indexer and shooter subsystems and their commands):
//   aim (their left trigger, AimShooter / AimShooterPass): the shooter's four X60s on their
//     velocity loop at their ShootMoveLUT's RPM (less 50) for the distance from the robot's center
//     to the HUB's (or to their pass point: 2 m out from the ALLIANCE WALL, 1.5 m in from the side
//     wall on the robot's side of the field), the hood to the table's angle (42-74). It runs until
//     the intake's button (their right bumper, also the shooter's off button) is pressed: then the
//     hood brakes where it is and the flywheel drops to their ShooterIdle's 1000 RPM (their
//     shooter's default command, commented out of their bindings; with it out their code coasts
//     it). In TELEOP it's held, as a driver holds the trigger, while it lines up a shot: shooting
//     (intaking or not), heading in to score (the AI's lineup, intaking or not), in its zone
//     holding FUEL and not intaking; otherwise (only intaking, out of its zone, empty) it idles.
//   shoot (their right trigger, Shoot with AgitateIntake): while the flywheel is within 500 RPM
//     under its setpoint and the chassis within 7 deg of its heading: the elevator at 4500 RPM
//     (their velocity loop, at the Talon's 1 kHz), the floor at 12 V and the side rollers at 10
//     V; otherwise all off (coasting). AgitateIntake: once the flywheel's within 400 RPM and the
//     heading within 5 deg, 0.65 s after the trigger, the box comes in while it's over their
//     6.3in (at -3.5 V, their SLIDE_FAST_PULL_VOLTAGE, not their AgitateIntake's -2: js/mech/r1114.js
//     SLIDE.pull), its roller at 6 V (their code cuts the roller's positive voltage inside 10.8in),
//     then stops; it stays where it is when the trigger's let go (their default command coasts it).
//   intake (their right bumper, LowerIntake then RunIntakeBallevator): the box out on their
//     position loop (13.55, kP 4 V per turn) until it's out (over 9in and within 1in), then the
//     roller at 12 V (not inside 10.8in), and, pushed in under 13in for 0.5 s, 20 A of torque
//     current out until it's past 13.4in again. After 3 s of intaking the floor runs at 3 V while
//     their hood CANrange sees nothing.
//   eject (their D-pad down, RunOuttake): the roller at -12 V.
//   by default the box coasts (0 V) and so does the roller.
//   intake and shoot together (TELEOP; not their bindings, where the intake's button also turns the
//   shooter off): both run, the intake as RunIntakeBallevator (the roller at 12 V, the box held
//   out) and the shot as Shoot (the flywheel at the shot for where it is, the elevator, floor and
//   side rollers once its gate is met), with no AgitateIntake: the box comes in only once the
//   intake's let go. The AI keeps intaking as it starts to shoot and while FUEL lie in front of
//   the intake, then lets go so the box compacts the rest (js/opponent.js, shotHold). In
//   AUTO the same, where their AUTO intakes beside a shot (the Depot auto's last FullShoot).
import { Mechanism } from './mechanism.js';
import { PhysicalRobot } from './physical.js';
import {
  build1114, wheels1114, pathSurfaces1114, placeSlide, placeIntake, placeHood, hoodElevation, hoodRot, kickerRot, shotFor, slideTurns,
  SLIDE, SLIDE_POS, SHOOTER, BALLEVATOR, RANGE, PRELOAD, FLY,
} from './r1114.js';
import PATH from './path1114.js';
import { Field } from '../field.js';
import { FUEL, FIELD_W, fw } from '../constants.js';

const DEG = Math.PI / 180, R = FUEL.radius, TAU = 2 * Math.PI;
const HALF_L = 0.311, HALF_W = 0.381; // its frame (24.5 x 30in)
// their gates (Shoot, AgitateIntake): the flywheel within 500 (agitating 400) RPM under its
// setpoint, the heading within 7 (5) deg; agitation waits 0.65 s
const GATE = { rpm: 500, aim: 7 * DEG, agitateRpm: 400, agitateAim: 5 * DEG, agitateWait: 0.65 };
// their hood's position loop (PositionVoltage, kP 8 V per motor turn; an X44 at 2.43 deg a turn
// moves it at most 7530 RPM x 2.43 = 305 deg/s at 12 V)
const HOOD = { kP: SHOOTER.hoodKP / SHOOTER.hoodDegPerRot, free: (7530 / 60) * SHOOTER.hoodDegPerRot };
const clamp = (x, a, b) => Math.max(a, Math.min(b, x));
// heading in to score (the AI's lineup), the HUB shot held within these distances (m)
const LINEUP = { near: 1.6, far: 4.5 };
// a motor's duty cycle for torque current amps (per motor) at its output speed w (their FOC
// torque current, the battery's 12 V permitting)
function dutyFor(motor, amps, w, V = 12) {
  const Eb = (w * motor.ratio) / motor.kV;
  return clamp((amps * motor.Rm + Eb) / V, -1, 1);
}

export class Physical1114 extends PhysicalRobot {
  constructor(robot) {
    const t = build1114(), w = wheels1114(t), s = pathSurfaces1114(PATH, t);
    const pathWheels = [...w.floor, ...w.squish, ...w.side, ...w.passive, ...w.elev, ...w.wall, ...w.intake];
    super(robot, {
      path: new Mechanism({ body: robot.body, substeps: 6, shafts: [...new Set(pathWheels.map((x) => x.shaft)), t.slide], wheels: pathWheels, surfaces: Object.values(s) }),
      shooter: new Mechanism({ body: robot.body, substeps: 6, sub: 8, shafts: [t.fly, t.hood], wheels: [w.fly, ...w.hood] }),
      // (full when its intake can't put more in: js/mech/physical.js. Under the blanket over the
      // box's front half, 0.3 m out to its front at 0.6 with the box out, 0.74 m across: a layer
      // of about 9, packed against it when two thirds of them are)
      preload: PRELOAD, naturalFull: { topX: 0.3, topN: 6 },
    });
    Object.assign(this, { t, w, s });
    this.ownsIntake = true;
    this.ownsHood = true;
    // (its shot compacts the load, their AgitateIntake pulling the box in, only with the intake let
    // go, after their 0.65 s wait: the AI keeps intaking that long, and while there's FUEL in front
    // of it to pick up, js/opponent.js _shotHold)
    this.shotHold = GATE.agitateWait;
    this.reset();
  }

  reset() {
    super.reset();
    // its starting configuration: the box all the way in (their IntakeSubsystem zeroes it there),
    // the hood on its stop (74.484, where their ShooterSubsystem zeroes it)
    this.ext = 0;
    this.hoodDeg = SHOOTER.hoodHome; this.hoodGoal = null;
    this.aim = null; this.aimMode = null;
    this.intakeT = -1; this.lowered = false; this.pushedT = -1; this.deploying = false;
    this.shootT = -1;
    for (const tr of [this.t.intake, this.t.floor, this.t.sideT, this.t.elev, this.t.shooter]) { tr.omega = 0; tr.cmd = null; tr.vel = null; }
    this.t.slide._omega = 0; this.t.slide.cmd = null;
    this.slideMode = { kind: 'volts', v: 0 };
    this.want = null;
    this.fireT = 0; this.lastShotT = 0; this.shots = 0; this.parked = new Set();
  }

  where(x, y, z) {
    const inFrame = Math.abs(x) < HALF_L && Math.abs(z) < HALF_W && y > 0.05 && y < 0.8;
    // out front: the box, as far out as it is (the intake roller and kicker at its front)
    const front = HALF_L + this.ext + 0.02;
    const inFront = x >= HALF_L && x < front && Math.abs(z) < 0.375 && y < 0.75;
    return {
      own: inFrame || inFront,
      // (held: over the bumper's top; not one on the kicker or the carpet the roller's reaching for)
      held: inFrame || (inFront && y > 0.2),
      near: x > -HALF_L - 0.1 && x < front + 0.1 && Math.abs(z) < HALF_W + 0.1 && y < 0.9,
    };
  }
  // FUEL up at the top of the elevator and round the flywheel
  shooterBalls(near) { return near.filter((b) => { const [x, y] = this._local(b.pos); return y > 0.47 && x < 0.08; }); }

  // the slide: their requests (VoltageOut, PositionVoltage kP 4 V per turn, TorqueCurrentFOC), and
  // the box's weight down its racks (they rise 8.7 deg toward the robot)
  _slideStep(dt, on) {
    const T = this.t, m = T.slide.motor, M = this.slideMode;
    if (!on) T.slide.cmd = null;
    else if (M.kind === 'volts') T.slide.cmd = M.v === 0 ? null : M.v / 12;
    else if (M.kind === 'pos') T.slide.cmd = clamp((4 * (M.turns - slideTurns(this.ext))) / 12, -1, 1);
    else T.slide.cmd = dutyFor(m, M.amps, T.slide._omega);
    T.slide.impulse(SLIDE.mass * 9.81 * SLIDE.dir[1] * SLIDE.pinion * dt);
  }

  preStep(dt, t) {
    const r = this.r, T = this.t, on = r.enabled;
    this._slideStep(dt, on);
    // the hood: their position loop (an X44 through 2.43 deg a turn; nothing pushes it hard); out
    // of an aim it brakes where it is
    if (on && this.hoodGoal !== null) {
      const v = clamp(HOOD.kP * (this.hoodGoal - this.hoodDeg), -12, 12);
      this.hoodDeg = clamp(this.hoodDeg + (v / 12) * HOOD.free * dt, SHOOTER.hoodMin - 2, SHOOTER.hoodHome);
    }
    // pose the moving parts
    placeSlide(this.s, this.ext);
    placeIntake(this.w.intake, this.ext);
    this.s.hood.setPose([...FLY, 0], 0, hoodRot(this.hoodDeg));
    placeHood(this.w.hood, this.hoodDeg);
    this.stepMechanisms(dt, t);
    // the box moves as its pinions turned (to its stops: all the way in, all the way out)
    this.ext += T.slide.omega * SLIDE.pinion * dt;
    if (this.ext < 0) { this.ext = 0; if (T.slide._omega < 0) T.slide._omega = 0; }
    if (this.ext > SLIDE.out) { this.ext = SLIDE.out; if (T.slide._omega > 0) T.slide._omega = 0; }
    r.intakeDeploy = clamp(this.ext / SLIDE.out, 0, 1);
  }

  // their hood CANrange: a FUEL near the top of the elevator, under the hood (an estimate of where)
  _range() {
    const [x0, y0] = RANGE.at;
    return this.held.some((b) => { const [x, y] = this._local(b.pos); return Math.abs(x - x0) < RANGE.half && y > y0; });
  }

  // their pass point (VisionSubsystem.getPassPose2d: 2 m out from the ALLIANCE WALL, 1.5 m in from
  // the side wall on the robot's side of the field)
  passTarget() {
    const r = this.r, pts = [fw(2.0, 1.5, r.alliance), fw(2.0, FIELD_W - 1.5, r.alliance)];
    return pts.reduce((a, b) => (Math.abs(a.z - r.pos.z) <= Math.abs(b.z - r.pos.z) ? a : b));
  }

  postStep(dt, t, { on, feeding, outtake, setpoint, mode }) {
    const r = this.r, T = this.t;
    this.account(t, mode);
    const turns = slideTurns(this.ext), pass = mode === 'pass';
    const auto = !!r.match?.isAuto;
    // (TELEOP: intake and shoot held together (the human's triggers, or the AI picking up what's on
    // its way in and shooting as it gets there) both run: the intake as their RunIntakeBallevator
    // (the roller at 12 V, the box held out), the shot as their Shoot (the flywheel at the table's
    // shot, FUEL fed up the elevator once its gate is met), and no AgitateIntake: the box doesn't
    // come in while the intake's held. In their bindings the intake's button is also the shooter's
    // off button, so their driver does one or the other. In AUTO the same, where their AUTO runs
    // its intake beside a shot: the Depot auto's last FullShoot, IntakeDeploy still running)
    const intaking = on && !!r.cmd.intake, eject = on && !!outtake;
    // ---- aim (their AimShooter / AimShooterPass): started by a shot to line up, ended by the
    // intake's button
    // (their AUTO: AimShooterHub, an AimShooterAuto on the shooter alone, spins it up to the HUB
    // shot beside IntakeDeploy (on the intake and indexer) from its marker to the path's end and
    // through FullShoot; ShooterStop after each FullShoot ends the aim until the next marker (their
    // code coasts the flywheel there; here it idles, below). The AUTO runner says which:
    // spin)
    const spin = auto ? r.cmd.spin : undefined, has = r.stored.length > 0;
    // (TELEOP: held, as their driver holds the aim trigger, while it lines up a shot: shooting or
    // passing, intaking or not; heading in to score (the AI's lineup: from when it sets off for its
    // spot with FUEL, the intake picking up what's on the way beside it, as their AUTO's
    // AimShooterHub runs beside IntakeDeploy); in its ALLIANCE ZONE holding FUEL and not intaking
    // (the human's: the sim's prespin, which gives the shared shot it reads), or asked to spin up
    // for a pass. Otherwise (only intaking, out of its zone, empty) it idles)
    const lineup = !auto && on && has && !!r.cmd.lineup && !feeding;
    if (!auto) {
      this.aim = on && (lineup || (setpoint > 0 && !!r.shot && (feeding || !intaking))) ? true : null;
      this.aimMode = pass && !lineup ? 'pass' : 'hub';
    } else {
      if (on && setpoint > 0 && r.shot) { this.aim = true; this.aimMode = pass && !spin ? 'pass' : 'hub'; }
      if ((intaking && !spin && !feeding) || !on || (spin === false && !feeding)) this.aim = null;
    }
    let want = null;
    if (this.aim) {
      const tgt = this.aimMode === 'pass' ? this.passTarget() : Field.hubCenter(r.alliance), d = Math.hypot(tgt.x - r.pos.x, tgt.z - r.pos.z);
      // (heading in, the HUB shot for where it is, held within its table's rows from 1.6 to 4.5 m,
      // 2390-3500 RPM: farther out it would ask up to 5800, and their Shoot's gate (within 500 under)
      // lets an over-speed shot go long)
      want = shotFor(this.aimMode, lineup ? clamp(d, LINEUP.near, LINEUP.far) : d);
      this.hoodGoal = want.hood;
    } else this.hoodGoal = null;
    // the flywheel: their velocity loop (motor RPM) at the shot. Not aiming, their ShooterIdle:
    // the same loop at 1000 RPM (SHOOTER_IDLE_VELOCITY; their shooter's default command, commented
    // out of their bindings, where their intake button and ShooterStop leave it coasting)
    const loop = want ? want.rpm : on ? SHOOTER.idle : 0;
    if (loop) T.shooter.vel = { target: (loop / 60) * TAU, kV: SHOOTER.kV, kP: SHOOTER.kP, lo: -12, hi: 12, per: 1 / TAU };
    else { T.shooter.vel = null; T.shooter.cmd = null; }
    const rpm = (T.shooter.omega * 60) / TAU, sp = want ? want.rpm : 0;
    const err = Math.abs(r.aimErr ?? Infinity);
    // ---- shoot (their Shoot and AgitateIntake, while the trigger's held)
    const shooting = on && feeding && !!want;
    if (shooting && this.shootT < 0) this.shootT = t;
    if (!shooting) this.shootT = -1;
    const fire = shooting && sp >= 1 && rpm >= sp - GATE.rpm && err <= GATE.aim;
    const agitate = shooting && sp >= 1 && rpm >= sp - GATE.agitateRpm && err <= GATE.agitateAim;
    // ---- intake (their LowerIntake, then RunIntakeBallevator)
    if (intaking && this.intakeT < 0) { this.intakeT = t; this.lowered = false; this.pushedT = -1; this.deploying = false; this.ballevT = -1; }
    if (!intaking) this.intakeT = -1;
    let roller = 0, floorV = 0;
    this.slideMode = { kind: 'volts', v: 0 };
    if (intaking) {
      if (!this.lowered) {
        this.slideMode = { kind: 'pos', turns: SLIDE_POS.drop };
        this.lowered = turns > SLIDE_POS.outCheck && Math.abs(turns - SLIDE_POS.drop) <= SLIDE_POS.tol;
        if (this.lowered) this.ballevT = t;
      }
      if (this.lowered) {
        roller = 12;
        if (turns < SLIDE_POS.redeploy || this.deploying) {
          if (this.pushedT < 0) this.pushedT = t;
          else if (t - this.pushedT > 0.5) {
            this.slideMode = { kind: 'amps', amps: 20 };
            this.deploying = true;
            if (turns > SLIDE_POS.out) this.deploying = false;
          }
        } else this.pushedT = -1;
        // (after 3 s, the floor at 3 V while their hood CANrange sees nothing)
        if (t - this.ballevT > 3 && !this._range()) floorV = 3;
      }
    } else if (shooting) {
      // their AgitateIntake (the intake's own command while the trigger's held, the intake's not)
      if (agitate && turns >= 1 && t - this.shootT > GATE.agitateWait && turns > SLIDE_POS.agitateMin) { this.slideMode = { kind: 'volts', v: SLIDE.pull }; roller = 6; }
    }
    if (eject) roller = -12;
    // (what its driver does: firing, with nothing shot for a second, they see the last few won't
    // come (FUEL left in the side pockets beside the floor's front, on the box's side ledges) and go
    // on: those aren't counted as the load until it next intakes, so the AI doesn't wait on them)
    if (r.stats.shots !== this.shots) { this.shots = r.stats.shots; this.lastShotT = t; }
    this.fireT = fire ? this.fireT + dt : shooting ? this.fireT : 0;
    if (fire && this.fireT > 1.5 && t - this.lastShotT > 1.0) for (const b of this.held) this.parked.add(b);
    if (intaking && !shooting) this.parked.clear();
    if (this.parked.size) r.stored = r.stored.filter((b) => !this.parked.has(b));
    // (their IntakeSubsystem cuts the roller's positive voltage with the box inside 10.8in)
    if (roller > 0.1 && turns < SLIDE_POS.rollersOff) roller = 0;
    if (!on) roller = 0;
    T.intake.cmd = roller === 0 ? null : roller / 12;
    // ---- the indexer and elevator: their Shoot (fire), RunIntakeBallevator's 3 V, or off
    if (fire) {
      T.floor.cmd = 1; T.sideT.cmd = 10 / 12;
      T.elev.vel = { target: (BALLEVATOR.shoot / 60) * TAU, kV: BALLEVATOR.kV, kP: BALLEVATOR.kP, lo: -12, hi: 12, per: 1 / TAU };
    } else {
      T.floor.cmd = floorV ? floorV / 12 : null; T.sideT.cmd = null;
      T.elev.vel = null; T.elev.cmd = null;
    }
    if (!on) { T.floor.cmd = null; T.sideT.cmd = null; T.elev.vel = null; T.elev.cmd = null; T.shooter.vel = null; T.shooter.cmd = null; }
    this.want = want;
    // what the robot's aiming and drawing see
    r.hoodDeg = hoodElevation(this.hoodDeg);
    r.flywheel = this.exitSpeed();
    r.ready = !!want && rpm >= sp - GATE.rpm && err <= GATE.aim;
    // (and its HUD, by its own gates: the shared one compares the exit speed with the generic
    // shot's to 2.5%, which this flywheel's table never is, so it said 'Spinning up' throughout)
    if (want && /^(Spinning up|Aiming|Hood|READY)$/.test(r.status)) r.status = rpm < sp - GATE.rpm ? 'Spinning up' : err > GATE.aim ? 'Aiming' : 'READY';
    r.feeding = fire ? 1 : 0;
    r.intakeSpeed = roller > 6 ? 1 : roller < 0 ? -1 : 0;
  }

  // how far out the box is (m along its racks), for the box drawn on it
  extNow() { return this.ext; }
  hoodTurn() { return hoodRot(this.hoodDeg); }
  kickerTurn() { return kickerRot(this.ext); }
  floorSpin() { return this.t.floor.omega * 0.5; }

  // about how fast a FUEL leaves at the flywheel's speed now (m/s; tools/mechlab1114.mjs shooter)
  exitSpeed() { return EXIT_PER_FLY_SURFACE * Math.abs(this.t.fly.omega) * SHOOTER.fly.r; }

  // how many it holds, about: 82 with the box out. Only a rating, for the HUD and the AI's plans:
  // it's full when its intake can't put more in (js/mech/physical.js): packed against the blanket
  // over the box's front half at 80-85 through the NEUTRAL ZONE's FUEL; its rollers can cram ~10
  // more (tools/mechlab1114.mjs intakefull)
  capacity() { return 82; }
}
// a FUEL's exit speed over the flywheel's surface speed (tools/mechlab1114.mjs shooter measures it)
export const EXIT_PER_FLY_SURFACE = 0.86;
