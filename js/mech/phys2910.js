// 2910's Re•Blitz in play, built of its physical mechanisms (js/mech/r2910.js; the shared part:
// js/mech/physical.js). Its controls are its code's (SuperStructure and the intake, hopper and
// shooter subsystems):
//   intake: down (DEPLOY, rollers off) by default; INTAKE runs its rollers at full; while a shot
//     is being fed it RETRACTs (to 10 deg, its rollers at 15%) unless it's intaking, pushing the
//     load back toward the shooter. The pivot is a Kraken X60 under their Motion Magic profile
//     and gains, so the FUEL in its way push back on it.
//   hopper floor: full (SHOOT) while a shot is being fed; otherwise off, or 7% (PRELOADING) when
//     its hopper CANrange says it's full and its feeder CANrange sees nothing waiting.
//   shooter: the drum's motors on their velocity loop at their table's RPM for the distance to
//     the HUB (or their passing table's, the hood flat out), the hood to its table's angle.
//   a shot is fed when the hood's within 3 deg of its angle and the chassis is aimed (4 deg for
//     the HUB, 15 for a pass); for the HUB also stopped (0.15 m/s) and 1.5 m or more away. Their
//     flywheel tolerance (3000 RPM) doesn't hold anything.
import { Mechanism } from './mechanism.js';
import { PhysicalRobot } from './physical.js';
import {
  build2910, wheels2910, pathSurfaces2910, placeIntake, placeHood, intakeRot, hoodElevation, shotFor, passFor,
  INTAKE, SHOOTER, RANGES, PRELOAD, PIVOT, DRUM,
} from './r2910.js';
import PATH from './path2910.js';
import { Field } from '../field.js';
import { FUEL, FIELD_W, fw } from '../constants.js';

const DEG = Math.PI / 180, R = FUEL.radius;
const HALF_L = 0.343, HALF_W = 0.349; // its frame (27in x 27.5in)
// their intake pivot's Motion Magic (degrees of arm): cruise, acceleration; Slot0 kP (V per
// degree) and kV (V per degree/s), in their mechanism units
export const PIVOT_MM = { v: (5500 / 60 / 40) * 360, a: 4000, kP: (5 * 40) / 360, kV: 12 / ((5800 / 60) / (40 / 360)) };
const HOOD_MM = { v: (7000 / 60 / ((52 / 12) * (18 / 12) * (146 / 10))) * 360, a: 3000 };
const TOL = { hood: 3, aim: 4 * DEG, passAim: 15 * DEG, speed: 0.15, minDist: 1.5 };

// a trapezoidal profile's next setpoint (position p, velocity v) toward goal
export function profile(p, v, goal, vmax, amax, dt) {
  const d = goal - p, dir = Math.sign(d);
  if (Math.abs(d) < 1e-6 && Math.abs(v) < 1e-6) return [goal, 0];
  // the speed it can still stop from
  const vStop = Math.sqrt(2 * amax * Math.abs(d));
  const want = dir * Math.min(vmax, vStop);
  const nv = v + Math.max(-amax * dt, Math.min(amax * dt, want - v));
  let np = p + nv * dt;
  if ((goal - np) * dir < 0) return [goal, 0];
  return [np, nv];
}

export class Physical2910 extends PhysicalRobot {
  constructor(robot) {
    const t = build2910(), w = wheels2910(t), s = pathSurfaces2910(PATH, t);
    const pathWheels = [...w.floor, ...w.indexer, ...w.intake];
    super(robot, {
      path: new Mechanism({ body: robot.body, substeps: 6, shafts: [...new Set(pathWheels.map((x) => x.shaft)), t.pivot], wheels: pathWheels, surfaces: Object.values(s) }),
      shooter: new Mechanism({ body: robot.body, substeps: 6, sub: 8, shafts: [t.drum, t.hood], wheels: [w.drum, ...w.hood] }),
      preload: PRELOAD,
    });
    Object.assign(this, { t, w, s });
    this.ownsIntake = true;
    this.ownsHood = true;
    this.reset();
  }

  reset() {
    super.reset();
    // stowed at the start (their starting configuration), the hood flat in
    this.armDeg = 0; this.armSet = [0, 0];
    this.hoodDeg = 0; this.hoodSet = [0, 0];
    this.targetRpm = 0;
    this.fullT = 0;
    this.feed = false;
    this.armGoal = INTAKE.deploy;
    for (const tr of [this.t.intake, this.t.floor, this.t.shooter]) { tr.omega = 0; tr.cmd = 0; }
    this.t.pivot._omega = 0; this.t.pivot.cmd = 0;
  }

  // the hopper runs out over the intake (its panels slide out with the first deploy and stay)
  where(x, y, z) {
    const inFrame = Math.abs(x) < HALF_L && Math.abs(z) < HALF_W && y > 0.105 && y < 0.62;
    const inFront = x >= HALF_L && x < 0.72 && Math.abs(z) < HALF_W && y < 0.6;
    return {
      own: inFrame || inFront,
      // (held: in the hopper; not one on the carpet the intake's still reaching for)
      held: inFrame || (inFront && y > 0.17),
      near: x > -HALF_L - 0.08 && x < 0.76 && Math.abs(z) < HALF_W + 0.08 && y < 0.66,
    };
  }
  shooterBalls(near) { return near.filter((b) => { const [x, y] = this._local(b.pos); return y > 0.33 && x < -0.03; }); }

  // the arm's angle up from forward (rad)
  armAngle() { return INTAKE.cadDeg * DEG + intakeRot(this.armDeg); }

  preStep(dt, t) {
    const r = this.r, T = this.t, on = r.enabled;
    // the intake's pivot: their Motion Magic toward the goal, the X60 doing what it can against
    // the arm's weight and the FUEL in its way (positive turns the arm up, toward stowed)
    this.armSet = profile(...this.armSet, this.armGoal, PIVOT_MM.v, PIVOT_MM.a, dt);
    const volts = on ? PIVOT_MM.kV * this.armSet[1] + PIVOT_MM.kP * (this.armSet[0] - this.armDeg) : 0;
    T.pivot.cmd = Math.max(-1, Math.min(1, -volts / 12));
    T.pivot.impulse(-INTAKE.mass * 9.81 * INTAKE.com * Math.cos(this.armAngle()) * dt);
    // the hood: their Motion Magic (an X44 through 95:1; nothing pushes it hard)
    this.hoodSet = profile(...this.hoodSet, this.hoodGoal ?? 0, HOOD_MM.v, HOOD_MM.a, dt);
    this.hoodDeg = this.hoodSet[0];
    // pose the moving parts
    this.s.intake.setPose([PIVOT[0], PIVOT[1], 0], 0, intakeRot(this.armDeg));
    placeIntake(this.w.intake, this.armDeg);
    this.s.hood.setPose([DRUM[0], DRUM[1], 0], 0, this.hoodDeg * DEG);
    placeHood(this.w.hood, this.hoodDeg);
    this.stepMechanisms(dt, t);
    // the arm turns as its shaft did (down to the bumper, its hardstop; up to stowed)
    this.armDeg -= (T.pivot.omega * dt) / DEG;
    if (this.armDeg > INTAKE.deploy) { this.armDeg = INTAKE.deploy; if (T.pivot._omega < 0) T.pivot._omega = 0; }
    if (this.armDeg < -2) { this.armDeg = -2; if (T.pivot._omega > 0) T.pivot._omega = 0; }
    r.intakeDeploy = Math.max(0, Math.min(1, this.armDeg / INTAKE.deploy));
  }

  // a CANrange's beam (across the robot at its height), tripped by a FUEL in it within reach
  _range({ at: [x0, y0], z }) {
    for (const b of this.held) {
      const [x, y, bz] = this._local(b.pos);
      if (Math.hypot(x - x0, y - y0) < R * 0.8 && Math.abs(bz - z) < RANGES.reach + R) return true;
    }
    return false;
  }

  postStep(dt, t, { on, feeding, outtake, setpoint, mode }) {
    const r = this.r, T = this.t;
    const ranges = { hopper: this._range(RANGES.hopper), feeder: this._range(RANGES.feeder) };
    this.account(t, mode);
    // the shot: their tables, by the distance from the robot's center to the HUB's (or to where
    // the pass is going)
    const pass = mode === 'pass';
    let want = null, dist = 0;
    if (setpoint > 0 && r.shot) {
      const tgt = r.shot.target;
      dist = Math.hypot(tgt.x - r.pos.x, tgt.z - r.pos.z);
      if (!pass) { const c = Field.hubCenter(r.alliance); dist = Math.hypot(c.x - r.pos.x, c.z - r.pos.z); }
      want = pass ? passFor(dist) : shotFor(dist);
    }
    // (their shooter is at RUN all match, DEFAULT_STATE included, at their table's speed for the
    // distance to the HUB: the drum's always up to speed, and only coasts when the robot's
    // disabled)
    if (want) { this.targetRpm = want.rpm; this.hoodGoal = want.hood; }
    else { const c = Field.hubCenter(r.alliance); this.targetRpm = shotFor(Math.hypot(c.x - r.pos.x, c.z - r.pos.z)).rpm; }
    if (!on) this.targetRpm = 0;
    // their velocity loop (no reverse voltage: it coasts down)
    const rps = T.shooter.omega / (2 * Math.PI), target = this.targetRpm / 60;
    const volts = target > 0 ? SHOOTER.kV * target + SHOOTER.kP * (target - rps) : 0;
    T.shooter.cmd = on ? Math.max(0, Math.min(1, volts / 12)) : 0;
    // ready to feed it: their isReadyToScore / isReadyToPass
    const v = r.body.linvel(), speed = Math.hypot(v.x, v.z);
    const hoodOk = Math.abs(this.hoodDeg - (this.hoodGoal ?? 0)) <= TOL.hood;
    const aim = Math.abs(r.aimErr ?? Infinity);
    const ready = !!want && hoodOk && (pass ? aim <= TOL.passAim : aim <= TOL.aim && speed <= TOL.speed && dist >= TOL.minDist);
    this.feed = on && feeding && ready && r.passClear !== false;
    // the hopper floor: SHOOT, PRELOADING (its hopper CANrange full for 0.25 s, nothing at the
    // feeder's) or off
    this.fullT = ranges.hopper ? this.fullT + dt : 0;
    T.floor.cmd = !on ? 0 : this.feed ? 1 : this.fullT > 0.25 && !ranges.feeder ? 0.07 : 0;
    // the intake: INTAKE, RETRACT while a shot's fed (unless intaking), otherwise DEPLOY
    const intaking = on && r.cmd.intake;
    if (intaking) { this.armGoal = INTAKE.deploy; T.intake.cmd = 1; }
    else if (this.feed) { this.armGoal = INTAKE.retract; T.intake.cmd = 0.15; }
    else { this.armGoal = on ? INTAKE.deploy : this.armGoal; T.intake.cmd = 0; }
    if (on && outtake) T.intake.cmd = -1;
    // what the robot's aiming and drawing see
    r.hoodDeg = hoodElevation(this.hoodDeg);
    r.flywheel = this.exitSpeed();
    r.ready = ready;
    r.feeding = this.feed ? 1 : 0;
    r.intakeSpeed = T.intake.cmd > 0.5 ? 1 : T.intake.cmd < 0 ? -1 : 0;
  }

  // where its passes go (their FieldConstants: 4 m out from the ALLIANCE WALL, 2.5 m in from a
  // side wall; the nearer of the two)
  passTarget() {
    const r = this.r, pts = [fw(4.0, 2.5, r.alliance), fw(4.0, FIELD_W - 2.5, r.alliance)];
    return pts.reduce((a, b) => (Math.hypot(a.x - r.pos.x, a.z - r.pos.z) <= Math.hypot(b.x - r.pos.x, b.z - r.pos.z) ? a : b));
  }

  // about how fast a FUEL leaves at the drum's speed now (m/s; tools/mechlab.mjs shooter2910)
  exitSpeed() { return EXIT_PER_DRUM_SURFACE * Math.abs(this.t.drum.omega) * SHOOTER.drum.d / 2; }

  // how many it holds: about 60 or more, the FUEL squashed together (the team; its hopper takes
  // 45 set in a layer at a time, tools/mechlab2910.mjs hold, and the intake packs it to 58-60 in
  // matches)
  capacity() { return 60; }
}
// a FUEL's exit speed over the drum's surface speed (tools/mechlab.mjs shooter2910 measures it)
export const EXIT_PER_DRUM_SURFACE = 0.8;
