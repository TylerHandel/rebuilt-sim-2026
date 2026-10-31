// 7769's CHUNK in play, built of its physical mechanisms (js/mech/r7769.js; the shared part:
// js/mech/physical.js). Their code isn't public, so its controls are what the team said on Chief
// Delphi (t/521008) and estimates where they said nothing (each marked so):
//   intake: the slide out on its position loop (an estimate: 40 V per m of error, at most 12 V) at
//     the team's 45 A deploy limit, down to 5 A once it's out ("to make the intake more collapsible on
//     hits"), its roller at 12 V while intaking. It stays out once it's out (an estimate: nothing in
//     the frame perimeter's way; the hopper's front rides on it), until it's told to stow.
//   shoot: the four X60s on their velocity loop at the shot table's RPM for the distance from the
//     robot's center to the HUB's (or to the pass point), the hood to the table's angle; when the
//     wheels are within 4% of it, the hood within 1.5 deg and the chassis within 4 deg (10 passing),
//     stopped (the team: "We never implemented shoot on the move"; under 0.25 m/s, an estimate), the
//     floor and the feeder run at 12 V. Between shots, holding FUEL in its zone, the wheels idle at
//     70% of the shot for where it is (an estimate), otherwise at 1500 RPM. Under a TRENCH the hood
//     is down.
//   the "Truffle Shuffle" (the team: "jostling the intake in and out would help keep fuel from
//     becoming jammed in the hopper"): while it shoots and isn't intaking, the slide runs in and
//     out between all the way out and SHUFFLE.depth in, every SHUFFLE.period (estimates), the
//     hopper's front pushing the load back toward the floor as it comes in.
//   eject: the roller, floor and feeder reversed.
import { Mechanism } from './mechanism.js';
import { PhysicalRobot } from './physical.js';
import {
  build7769, wheels7769, pathSurfaces7769, placeIntake, placeSlide, slideStops, hoodRot, hoodElevation, kickRot, slideOffset,
  SHOOTER_SURFACES, SLIDE, SLIDE_CURRENT, SHOOTER, HOOD, AXLE, PRELOAD, HALF_L, FRONT_X,
} from './r7769.js';
import PATH from './path7769.js';
import { Field } from '../field.js';
import { FUEL, FIELD_W, fw } from '../constants.js';
import { underTrench } from '../nav.js';

const DEG = Math.PI / 180, R = FUEL.radius, TAU = 2 * Math.PI;
const clamp = (x, a, b) => Math.max(a, Math.min(b, x));
const HALF = 0.35; // the hopper's half width (its walls)
// its gates (estimates): the wheels within 4% of the shot, the hood within HOOD.tol, the heading within
// 4 deg (10 passing), stopped (under 0.25 m/s) for a HUB shot
const TOL = { rpm: 0.04, aim: 4 * DEG, passAim: 10 * DEG, speed: 0.25 };
const IDLE = { share: 0.7, rpm: 1500 };
// the shot table (estimates: no code is public): distance from the robot's center to the HUB's center
// (m) -> shooter RPM, hood (deg up and back from down). The hood's schedule is set (the flattest
// shots that still come down into the HUB's opening from that far, by the CAD's geometry); the RPMs
// are the ones at which the sim's FUEL, fed as in play, come down through the HUB's rim height over
// its center (tools/mechlab7769.mjs tune)
export const SHOTS = [[1.3, 2485, 0], [2.0, 2790, 3], [2.5, 2860, 6], [3.0, 2855, 10], [3.5, 3000, 13], [4.0, 3200, 16], [4.5, 3400, 19], [5.0, 3600, 22], [5.5, 3800, 24], [6.0, 4000, 26]];
// passes (to the carpet at the row's distance): the hood all the way up (40 deg); past ~8 m its FUEL
// slip on the wheels and go no farther (they leave at no more than ~9.5 m/s)
export const PASSES = [[4.0, 2210, 35], [5.0, 2605, 35], [6.0, 3010, 35], [7.0, 3515, 35], [8.0, 4035, 35]];
const interp = (t, x, j) => {
  if (x <= t[0][0]) return t[0][j];
  for (let k = 1; k < t.length; k++) if (x <= t[k][0]) return t[k - 1][j] + ((t[k][j] - t[k - 1][j]) * (x - t[k - 1][0])) / (t[k][0] - t[k - 1][0]);
  return t[t.length - 1][j];
};
export const shotFor = (kind, d) => { const t = kind === 'pass' ? PASSES : SHOTS; return { rpm: interp(t, d, 1), hood: interp(t, d, 2) }; };
// the slide's position loop (an estimate): volts toward goal (m out) from ext
export const slideVolts = (goal, ext) => clamp(40 * 12 * (goal - ext) / 1.0, -12, 12);
// the Truffle Shuffle (estimates): in to depth and back out, every period, from when it starts
export const SHUFFLE = { depth: 0.12, period: 0.8, delay: 0.4 };
export const shuffleGoal = (t) => (t < SHUFFLE.delay ? SLIDE.out : ((t - SHUFFLE.delay) % SHUFFLE.period) < SHUFFLE.period / 2 ? SLIDE.out - SHUFFLE.depth : SLIDE.out);

export class Physical7769 extends PhysicalRobot {
  constructor(robot) {
    const t = build7769(), w = wheels7769(t), s = pathSurfaces7769(PATH, t);
    const pathWheels = [...w.intake, ...w.feed];
    const pathS = Object.entries(s).filter(([k]) => !SHOOTER_SURFACES.includes(k)).map(([, v]) => v);
    super(robot, {
      path: new Mechanism({ body: robot.body, substeps: 6, shafts: [...new Set(pathWheels.map((x) => x.shaft)), t.belts, t.slide], wheels: pathWheels, surfaces: pathS }),
      shooter: new Mechanism({ body: robot.body, substeps: 6, sub: 8, shafts: [t.wheels], wheels: w.shooter, surfaces: SHOOTER_SURFACES.map((k) => s[k]) }),
      // (full when its intake can't put more in: js/mech/physical.js. Under the net over the
      // hopper's front, 0.25 m out to its front at 0.62 with the intake out, 0.7 m across: a layer
      // of ~9, packed against it when two thirds of them are)
      preload: PRELOAD, naturalFull: { topX: 0.25, topN: 6 },
    });
    Object.assign(this, { t, w, s });
    this.ownsIntake = true;
    this.ownsHood = true;
    this.reset();
  }

  reset() {
    super.reset();
    // its starting configuration: the intake in, the hood down
    this.ext = 0; this.slideGoal = null;
    this.hoodDeg = 0; this.hoodGoal = 0;
    this.feed = false; this.feedT = 0; this.want = null;
    for (const tr of [this.t.intake, this.t.floor, this.t.feedT, this.t.shooter]) { tr.omega = 0; tr.cmd = null; tr.vel = null; }
    this.t.slide._omega = 0; this.t.slide.cmd = null;
  }

  frontX() { return FRONT_X + slideOffset(this.ext)[0]; }
  where(x, y, z) {
    const inFrame = Math.abs(x) < HALF_L && Math.abs(z) < HALF && y > 0.05 && y < 0.62;
    const front = this.frontX() + 0.03;
    const inFront = x >= HALF_L && x < Math.max(front, HALF_L + 0.2) && Math.abs(z) < HALF && y < 0.6;
    return {
      own: inFrame || inFront,
      // (held: over the bumper's top; not one on the kick bar or the carpet the roller's reaching for)
      held: inFrame || (inFront && y > 0.2 && x < front),
      near: x > -HALF_L - 0.1 && x < Math.max(front, HALF_L + 0.2) + 0.1 && Math.abs(z) < HALF + 0.1 && y < 0.75,
    };
  }
  // FUEL up the ramp and round the shooter's wheels, under the hoods
  shooterBalls(near) { return near.filter((b) => { const [x, y] = this._local(b.pos); return y > 0.33 && x < 0.03; }); }
  // the top of the load under the net (for the TRENCH)
  loadTop() {
    let top = 0;
    for (const b of this.held) { const [x, y] = this._local(b.pos); if (x > -0.05) top = Math.max(top, y + R); }
    return top;
  }

  preStep(dt, t) {
    const r = this.r, T = this.t, on = r.enabled;
    // the slide: its position loop toward its goal (null: coasting), at the deploy limit, or the
    // held one once it's out; its weight down the racks
    slideStops(T.slide, this.ext);
    const out = this.ext >= SLIDE.out - 0.005;
    T.slide.motor.statorLimit = this.slideGoal !== null && this.slideGoal >= SLIDE.out - 0.005 && out ? SLIDE_CURRENT.held : SLIDE_CURRENT.deploy;
    T.slide.cmd = on && this.slideGoal !== null ? slideVolts(this.slideGoal, this.ext) / 12 : null;
    T.slide.impulse(SLIDE.mass * 9.81 * SLIDE.dir[1] * SLIDE.pinion * dt);
    // the hood: its position loop (an X44 through 8:1; nothing pushes it hard)
    if (on) this.hoodDeg += clamp(this.hoodGoal - this.hoodDeg, -HOOD.rate * dt, HOOD.rate * dt);
    // pose the moving parts
    placeSlide(this.s, this.ext);
    placeIntake(this.w.intake, this.ext);
    this.s.hood.setPose([...AXLE, 0], 0, hoodRot(this.hoodDeg));
    this.stepMechanisms(dt, t);
    // the intake moves as its pinions turned (to its stops: all the way in, all the way out)
    this.ext += T.slide.omega * SLIDE.pinion * dt;
    if (this.ext < 0) { this.ext = 0; if (T.slide._omega < 0) T.slide._omega = 0; }
    if (this.ext > SLIDE.out) { this.ext = SLIDE.out; if (T.slide._omega > 0) T.slide._omega = 0; }
    r.intakeDeploy = clamp(this.ext / SLIDE.out, 0, 1);
  }

  // where its passes go (an estimate: 3 m out from its ALLIANCE WALL, 2 m in from the nearer side wall)
  passTarget() {
    const r = this.r, pts = [fw(3.0, 2.0, r.alliance), fw(3.0, FIELD_W - 2.0, r.alliance)];
    return pts.reduce((a, b) => (Math.hypot(a.x - r.pos.x, a.z - r.pos.z) <= Math.hypot(b.x - r.pos.x, b.z - r.pos.z) ? a : b));
  }

  postStep(dt, t, { on, feeding, outtake, setpoint, mode }) {
    const r = this.r, T = this.t;
    this.account(t, mode);
    const pass = mode === 'pass', intaking = on && !!r.cmd.intake, has = r.stored.length > 0;
    // the shot: the table's, by the distance from the robot's center to the HUB's or the pass point
    let want = null, dist = 0;
    if (setpoint > 0 && r.shot) {
      const tgt = pass ? this.passTarget() : Field.hubCenter(r.alliance);
      dist = Math.hypot(tgt.x - r.pos.x, tgt.z - r.pos.z);
      want = shotFor(pass ? 'pass' : 'hub', dist);
    }
    const trench = underTrench(r.pos.x, r.pos.z, 0.45);
    this.hoodGoal = want && !trench ? clamp(want.hood, SHOOTER.hoodMin, SHOOTER.hoodMax) : 0;
    // the wheels: the shot's RPM, or the idle
    let rpm = 0;
    if (on) {
      if (want) rpm = want.rpm;
      else if (has && r.inAllianceZone?.()) { const c = Field.hubCenter(r.alliance); rpm = IDLE.share * shotFor('hub', Math.hypot(c.x - r.pos.x, c.z - r.pos.z)).rpm; }
      else rpm = IDLE.rpm;
    }
    if (rpm > 0) T.shooter.vel = { target: (rpm / 60) * TAU, kV: SHOOTER.kV, kP: SHOOTER.kP, lo: 0, hi: 12, per: 1 / TAU };
    else { T.shooter.vel = null; T.shooter.cmd = null; }
    const now = (T.wheels.omega * 60) / TAU;
    const v = r.body.linvel(), speed = Math.hypot(v.x, v.z), aim = Math.abs(r.aimErr ?? Infinity);
    const spun = !!want && Math.abs(now - want.rpm) <= TOL.rpm * want.rpm;
    const hoodOk = Math.abs(this.hoodDeg - this.hoodGoal) <= HOOD.tol;
    const ready = !!want && spun && hoodOk && (pass ? aim <= TOL.passAim : aim <= TOL.aim && speed <= TOL.speed);
    this.feed = on && feeding && ready && r.passClear !== false;
    this.feedT = this.feed ? this.feedT + dt : 0;
    // the floor and feeder: 12 V while it shoots
    let floor = this.feed ? 1 : 0, feed = this.feed ? 1 : 0, roller = 0;
    // the intake: out and running while intaking; shuffling while it shoots (not intaking); out
    // stays out; stowed when told to
    if (intaking) { this.slideGoal = SLIDE.out; roller = 12; }
    else if (this.feed) this.slideGoal = this.ext > 0.05 ? shuffleGoal(this.feedT) : this.slideGoal;
    else if (r.cmd.stow) this.slideGoal = 0;
    else if (this.slideGoal !== null && this.slideGoal < SLIDE.out && this.slideGoal > 0) this.slideGoal = SLIDE.out;
    if (on && outtake) { roller = -12; floor = -0.5; feed = -0.5; }
    if (!on) { roller = 0; floor = 0; feed = 0; }
    T.intake.cmd = roller ? roller / 12 : null;
    T.floor.cmd = floor || null;
    T.feedT.cmd = feed || null;
    this.want = want;
    // what the robot's aiming and drawing see
    r.hoodDeg = hoodElevation(this.hoodDeg);
    r.flywheel = this.exitSpeed();
    r.ready = ready;
    r.feeding = this.feed ? 1 : 0;
    r.intakeSpeed = roller > 6 ? 1 : roller < 0 ? -1 : 0;
  }

  shotStatus() {
    if (!this.want) return null;
    if (this.feed) return 'SHOOTING';
    if (Math.abs(this.hoodDeg - this.hoodGoal) > HOOD.tol) return 'Hood';
    if (Math.abs((this.t.wheels.omega * 60) / TAU - this.want.rpm) > TOL.rpm * this.want.rpm) return 'Spinning up';
    return null;
  }
  // for drawing it: how far out the intake is, the hood's and kick bar's turns, the rollers' spins
  extNow() { return this.ext; }
  hoodTurn() { return hoodRot(this.hoodDeg); }
  kickTurn() { return kickRot(this.ext); }
  floorSpin() { return this.t.belts.omega; }
  mechPose() { return { ext: this.ext, hood: hoodRot(this.hoodDeg), kick: kickRot(this.ext), roller: this.t.roller.omega, belts: this.t.belts.omega, feed: this.t.feed.omega, wheels: this.t.wheels.omega }; }

  // about how fast a FUEL leaves at the wheels' speed now (m/s; tools/mechlab7769.mjs shooter)
  exitSpeed() { return EXIT_PER_WHEEL_SURFACE * Math.abs(this.t.wheels.omega) * SHOOTER.wheel.r; }
  // how many it holds, about (the team: "almost 70 (mostly 50-60 during game play)"). Only a rating,
  // for the HUD and the AI's plans: it's full when its intake can't put more in (js/mech/physical.js)
  capacity() { return 60; }
}
// a FUEL's exit speed over the wheels' surface speed (tools/mechlab7769.mjs shooter measures it)
export const EXIT_PER_WHEEL_SURFACE = 0.5;
