// 8793 in play, built of its physical mechanisms (js/mech/r8793.js): the FUEL it takes in stay
// bodies on the FIELD the whole way, moved only by its rollers, wheels and the surfaces of its CAD
// (a Mechanism on the robot's own body, so it drives off with them, and every push on them pushes
// back on it), until its shooter flings them. Its controls are its code's (ShooterMode): the
// rollers and the vertical rollers always running at their duty cycles, with its automatic jam
// recovery; the turret indexer while the shot is on; and the flywheel on its velocity loop toward
// the motor speed that gives the exit speed the aim asks for (SHOT_TABLE).
import { Mechanism } from './mechanism.js';
import { PhysicalRobot } from './physical.js';
import {
  build8793, intake8793, conveyorWheels, armWheels, placeArm, indexerWheels, shooterWheels, placeShooter,
  flywheelDuty, exitSpeed, rpsFor, pathSurfaces, ARM, DUTY, DUTY_INDEXER, PRELOAD, SHOOTER,
} from './r8793.js';
import PATH from './path8793.js';

const DEG = Math.PI / 180;
const HALF = 0.349;   // the frame's half length and width (27.5in)
const TURRET_X = -0.127;
// their jam recovery (GroundIntakeConstants): the roller motor over 55 A stator and under 10
// rotations/s for 0.125 s reverses at full for 0.125 s
export const JAM = { amps: 55, rps: 10, debounce: 0.125, reverse: 0.125 };

export class Physical8793 extends PhysicalRobot {
  constructor(robot) {
    const t = build8793(), s = pathSurfaces(PATH), arm = armWheels(t);
    const wheels = [...conveyorWheels(t), ...indexerWheels(t), ...arm];
    const sw = shooterWheels(t, 60);
    super(robot, {
      path: new Mechanism({ body: robot.body, substeps: 6, shafts: [...new Set(wheels.map((w) => w.shaft))], wheels, surfaces: Object.values(s) }),
      shooter: new Mechanism({ body: robot.body, substeps: 6, sub: 8, shafts: [t.large, t.small], wheels: sw }),
      preload: PRELOAD,
    });
    Object.assign(this, { t, s, arm, shooterWheels: sw });
    this.autoShuttle = true; // (its ShooterMode shuttles whatever it picks up, out of its zone)
    this.reset();
  }

  reset() {
    super.reset();
    this.jamT = 0; this.unjamT = 0;
    this.targetRps = 0;
    this.feed = false;
    for (const tr of [this.t.horizontal, this.t.indexer, this.t.shooter]) { tr.omega = 0; tr.cmd = 0; }
  }

  where(x, y, z) {
    const deployed = this.r.intakeDeploy > 0.5;
    // (in its ball path: up off the carpet, where its floor and rails carry them, 0.13 m up and
    // more; one that's got under the frame on the carpet is the chassis's to push out)
    const inside = Math.abs(x) < HALF && Math.abs(z) < HALF && y > 0.105 && y < 0.7;
    const onArm = deployed && x >= HALF && x < HALF + 0.33 && Math.abs(z) < ARM.halfWidth && y < 0.45;
    return {
      own: inside || onArm,
      // (held: in the robot, or up on its arm off the carpet; not one the arm's still reaching for)
      held: inside || (onArm && y > 0.17),
      // (anything of it a FUEL could touch: the frame, the arm out to its tip, the shooter)
      near: x > -HALF - 0.05 && x < HALF + 0.42 && Math.abs(z) < HALF + 0.08 && y < 0.75,
    };
  }
  shooterBalls(near) { return near.filter((b) => this._local(b.pos)[1] > 0.3); }

  // before the world's step: put the arm, turret and hood where they are, and run the mechanisms
  preStep(dt, t) {
    const r = this.r;
    const roll = (1 - r.intakeDeploy) * r.cfg.intake.fold.stowDeg * DEG;
    this.s.arm.setPose([ARM.pivot[0], ARM.pivot[1], 0], 0, roll);
    placeArm(this.arm, roll);
    this.s.turret.setPose([TURRET_X, 0, 0], r.turretYaw, 0);
    placeShooter(this.shooterWheels, r.hoodDeg, r.turretYaw, TURRET_X);
    this.stepMechanisms(dt, t);
  }

  // after the world's step: what got in, what got shot, and the controls for the next
  postStep(dt, t, { on, feeding, outtake, setpoint, mode }) {
    const r = this.r;
    this.account(t, mode);
    // the intake path, as in their ShooterMode: the rollers and vertical rollers always running
    // inward (bringing what it picks up to the throat under the turret), with their automatic jam
    // recovery (runIntakeWithAutoUnjam): the roller motor stalled (over 55 A, under 10 rotations/s)
    // for 0.125 s reverses the whole path at full for 0.125 s, the turret indexer too, and then the
    // detector needs a fresh 0.125 s of stall to fire again. Reversed too to spit FUEL out.
    const h = this.t.horizontal, motorRps = Math.abs(h.omega) / (2 * Math.PI);
    const stalled = Math.abs(h.motor.current) > JAM.amps && motorRps < JAM.rps;
    if (this.unjamT > 0) {
      this.unjamT -= dt;
      if (this.unjamT <= 0) { this.unjamT = 0; this.jamT = 0; }
    } else {
      this.jamT = on && !outtake && stalled ? this.jamT + dt : 0;
      if (this.jamT > JAM.debounce) { this.unjamT = JAM.reverse; this.jamT = 0; r.stats.unjams = (r.stats.unjams || 0) + 1; }
    }
    const unjam = on && this.unjamT > 0;
    intake8793(this.t, on, { dir: outtake || unjam ? -1 : 1 });
    // the turret indexer feeds while there's a shot (or pass) to make, reversed with the unjam:
    // their ShooterMode doesn't hold it for the flywheel or hood (HOLD_FEED_UNTIL_AT_SPEED is off),
    // so what's still coming up to speed goes as it is; only while the turret is swinging round
    // to its target (more than TURRET_RESET_TOLERANCE_DEG, 10 deg, off) is the feeder off
    // A pass goes whether it'll get there or not (their ShuttleMode doesn't wait either), as long
    // as it won't drop into a HUB (G407)
    this.feed = on && feeding && setpoint > 0 && Math.abs(r.aimErr ?? 0) < 10 * DEG && r.passClear !== false;
    this.t.indexer.cmd = this.feed ? (unjam ? -DUTY_INDEXER : DUTY_INDEXER) : on && outtake ? -DUTY_INDEXER : 0;
    // the flywheel: toward the speed the shot wants while there's one to make (it holds that
    // speed a moment after); coasting otherwise (their code)
    if (setpoint > 0) { this.targetRps = Math.min(SHOOTER.maxRps, rpsFor(setpoint)); this.holdT = t; }
    else if (!(on && this.targetRps && t - (this.holdT ?? -9) < 1.5)) this.targetRps = 0;
    this.t.shooter.cmd = on && this.targetRps > 0 ? flywheelDuty(this.t, this.targetRps) : 0;
    // what the robot's aiming sees: the exit speed its flywheel gives right now
    r.flywheel = exitSpeed(Math.abs(this.t.shooter.omega) / (2 * Math.PI));
    r.intakeSpeed = h.cmd < 0 ? 1 : h.cmd > 0 ? -1 : 0;
    r.feeding = this.feed ? 1 : 0;
  }

  // how many it holds, about, with the arm down: 7 in the frame (tools/mechlab.mjs slots) and a
  // couple more up on its arm (what it carries in matches)
  capacity() { return 9; }
}
