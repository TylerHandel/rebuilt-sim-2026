// 8793 in play, built of its physical mechanisms (js/mech/r8793.js): the FUEL it takes in stay
// bodies on the FIELD the whole way, moved only by its rollers, wheels and the surfaces of its CAD
// (a Mechanism on the robot's own body, so it drives off with them, and every push on them pushes
// back on it), until its shooter flings them. Its controls are its code's (ShooterMode): the
// rollers and the vertical rollers always running at their duty cycles, with its automatic jam
// recovery; the turret indexer while the shot is on; and the flywheel on its velocity loop toward
// the motor speed that gives the exit speed the aim asks for (SHOT_TABLE).
import { Mechanism } from './mechanism.js';
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
const JAM = { amps: 55, rps: 10, debounce: 0.125, reverse: 0.125 };

export class Physical8793 {
  constructor(robot) {
    this.r = robot;
    this.t = build8793();
    this.s = pathSurfaces(PATH);
    this.arm = armWheels(this.t);
    const wheels = [...conveyorWheels(this.t), ...indexerWheels(this.t), ...this.arm];
    this.path = new Mechanism({ body: robot.body, substeps: 6, shafts: [...new Set(wheels.map((w) => w.shaft))], wheels, surfaces: Object.values(this.s) });
    this.shooterWheels = shooterWheels(this.t, 60);
    this.shooter = new Mechanism({ body: robot.body, substeps: 6, sub: 8, shafts: [this.t.large, this.t.small], wheels: this.shooterWheels });
    this.reset();
  }

  reset() {
    this.held = [];          // FUEL in its ball path
    this.near = [];          // FUEL its mechanisms might touch this step
    this.inShooter = new Set();
    this.owned = new Set();   // FUEL its ball path (or arm) has hold of
    this.shot = new Map();    // FUEL it's just shot -> when (on their way out: not its to hold)
    this.counted = new Set(); // FUEL counted as intaked, until shot or gone from around it
    this.jamT = 0; this.unjamT = 0;
    this.targetRps = 0;
    this.feed = false;
    for (const tr of [this.t.horizontal, ...this.t.vertical, this.t.indexer, this.t.shooter]) { tr.omega = 0; tr.cmd = 0; }
  }

  // the robot's frame <-> the world (x forward, y up, z right, origin on the carpet at its center)
  _pose() { const b = this.r.body; this.pose = { t: b.translation(), q: b.rotation() }; return this.pose; }
  _local(p) {
    const { t, q } = this.pose || this._pose();
    const dx = p.x - t.x, dy = p.y - t.y, dz = p.z - t.z;
    // rotate by the inverse of q
    const x = -q.x, y = -q.y, z = -q.z, w = q.w;
    const ix = w * dx + y * dz - z * dy, iy = w * dy + z * dx - x * dz, iz = w * dz + x * dy - y * dx, iw = -x * dx - y * dy - z * dz;
    return [ix * w + iw * -x + iy * -z - iz * -y, iy * w + iw * -y + iz * -x - ix * -z, iz * w + iw * -z + ix * -y - iy * -x];
  }
  _world(l) {
    const { t, q } = this._pose();
    const [x, y, z] = l, qx = q.x, qy = q.y, qz = q.z, qw = q.w;
    const ix = qw * x + qy * z - qz * y, iy = qw * y + qz * x - qx * z, iz = qw * z + qx * y - qy * x, iw = -qx * x - qy * y - qz * z;
    return { x: t.x + ix * qw + iw * -qx + iy * -qz - iz * -qy, y: t.y + iy * qw + iw * -qy + iz * -qx - ix * -qz, z: t.z + iz * qw + iw * -qz + ix * -qy - iy * -qx };
  }

  // the preload: into its slots (what doesn't fit goes to its alliance's human player)
  load(balls) {
    const fuel = this.r.fuel;
    balls.forEach((b, i) => {
      if (i >= PRELOAD.length) { fuel.addToChute(this.r.alliance, b); return; }
      const p = this._world(PRELOAD[i]);
      fuel.launch(b, p, { x: 0, y: 0, z: 0 }, {});
      b.inFlight = false;
      fuel.own(b, 0);
      this.held.push(b);
      this.owned.add(b);
      this.counted.add(b); // (a preload isn't intaked)
    });
    this.r.stored = this.held.slice();
  }

  // before the world's step: put the arm, turret and hood where they are, and run the mechanisms
  preStep(dt, t) {
    const r = this.r;
    const roll = (1 - r.intakeDeploy) * r.cfg.intake.fold.stowDeg * DEG;
    this.s.arm.setPose([ARM.pivot[0], ARM.pivot[1], 0], 0, roll);
    placeArm(this.arm, roll);
    this.s.turret.setPose([TURRET_X, 0, 0], r.turretYaw, 0);
    placeShooter(this.shooterWheels, r.hoodDeg, r.turretYaw, TURRET_X);
    this._pose();
    // the FUEL near enough to touch anything of it; the ones in its ball path (or on its arm)
    // don't hit its own colliders: its CAD's surfaces are what they touch there
    const pos = r.body.translation(), deployed = r.intakeDeploy > 0.5;
    const near = [], held = [], owned = new Set();
    for (const b of r.fuel.balls) {
      if (b.state !== 'field') continue;
      const p = b.pos;
      if (Math.abs(p.x - pos.x) > 0.95 || Math.abs(p.z - pos.z) > 0.95 || p.y > 0.9) continue;
      if (this.shot.has(b)) {
        if (t - this.shot.get(b) < 0.35) continue;
        this.shot.delete(b);
      }
      const [x, y, z] = this._local(p);
      const inside = Math.abs(x) < HALF && Math.abs(z) < HALF && y > 0.04 && y < 0.7;
      const onArm = deployed && x >= HALF && x < HALF + 0.33 && Math.abs(z) < ARM.halfWidth && y < 0.45;
      if (inside || onArm) {
        r.fuel.own(b, t);
        owned.add(b);
        // (held: in the robot, or up on its arm off the carpet; not one the arm's still reaching for)
        if (inside || y > 0.17) held.push(b);
      }
      // (anything of it a FUEL could touch: the frame, the arm out to its tip, the shooter)
      if (inside || onArm || (x > -HALF - 0.05 && x < HALF + 0.42 && Math.abs(z) < HALF + 0.08 && y < 0.75)) near.push(b);
    }
    // (the ones it's let go of: solid to the other FUEL again)
    for (const b of this.owned) if (!owned.has(b) && b.state === 'field') r.fuel.unown(b);
    this.owned = owned;
    this.near = near;
    this.held = held;
    // (FUEL it holds pressing on each other: foam on foam, worked out with its wheels)
    this.path.step(dt, near.map((b) => b.body), [...owned].map((b) => b.body));
    this.shooter.step(dt, near.filter((b) => this._local(b.pos)[1] > 0.3).map((b) => b.body));
  }

  // after the world's step: what got in, what got shot, and the controls for the next
  postStep(dt, t, { on, feeding, outtake, setpoint, mode }) {
    const r = this.r;
    this.pose = null;
    r.stored = this.held.slice();
    // intaked: once a FUEL's in, however it jostles in and out of the frame's line on its way
    const around = new Set(this.near);
    for (const b of this.counted) if (!around.has(b)) this.counted.delete(b);
    for (const b of r.stored) if (!this.counted.has(b)) { this.counted.add(b); r.stats.intaked++; }
    // shot: through the shooter's wheels and out
    for (const [b, ts] of this.shot) if (t - ts > 0.35) this.shot.delete(b);
    const byBody = new Map(this.near.map((b) => [b.body, b]));
    const touching = new Set(this.shooter.contacts.map((c) => byBody.get(c.ball)).filter(Boolean));
    for (const b of touching) this.inShooter.add(b);
    for (const b of [...this.inShooter]) {
      if (touching.has(b)) continue;
      this.inShooter.delete(b);
      const v = b.body.linvel();
      if (Math.hypot(v.x, v.y, v.z) < 3 || v.y < 0) continue; // dropped back down, not shot
      this.counted.delete(b);
      this.owned.delete(b);
      this.shot.set(b, t);
      r.fuel.release(b, { by: 'robot', alliance: r.alliance, legal: r.lastInZone, t, ignoreRobot: 0.35 });
      r.stats.shots++;
      if (mode === 'pass') r.stats.passes++;
    }
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
    if (on && (outtake || unjam)) {
      h.cmd = 1; // (reversed: the rollers' bottoms run forward)
      for (const tr of this.t.vertical) tr.cmd = -1;
    } else intake8793(this.t, on);
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
