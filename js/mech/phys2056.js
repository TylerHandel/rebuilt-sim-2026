// 2056's "Desperate Measures" in play, built of its physical mechanisms (js/mech/r2056.js; the shared
// part: js/mech/physical.js). Their 2026 code isn't public (the PublicCodeBank repository holds their
// pre-season base: the swerve, its 100 A stator and 80/60 A supply limits), so the controls are what
// their binder and their Chief Delphi thread say, the rest estimates:
//   - "C++ ... finite state machines control all subsystems. CTRE's Motion Magic used to drive all
//     mechanisms to their desired setpoints ... with standard PID for continuous targeting."
//   - intake: racks out at the start ("Dynamic Motion Magic for distinct extension and retraction
//     speed"; "a low stator current limit allowing the intake to retract like a spring during
//     impacts"); the kicker bar falls once it's out, held down by surgical tubing. Rollers at 12 V
//     while intaking ("fairly high current limits"), the floor running with them.
//   - "Intake Retract (while held): for temporarily retracting the intake to fully empty hopper while
//     shooting": emulated, as their driver does, once the load's down to RETRACT_AT while it shoots
//     and isn't intaking, the intake comes in (its low current limit: it presses the last of the load
//     back gently, est.) and goes out again when it stops.
//   - shooting: "Kinematically computed shot vector with time-of-flight compensation for shooting on
//     the move", the turret aiming itself on its X44 (+-185 deg), the hood to the shot's release
//     angle (its geometry: "73.5 to 39.5 release angle from horizontal") and the wheel to the exit
//     speed through "a manually tuned coefficient to map desired exit velocity to wheel speed"
//     (EXIT_K, fitted on the bench as they tuned theirs). "When passing we would pass with the hood at
//     its lowest release angle and vary flywheel RPM and when shooting we would vary both shooter RPM
//     and hood angle, the kicker speed a constant ratio of the flywheel RPM throughout" (KICK_RATIO,
//     est.). Once the turret, hood and wheel are there the indexer runs: the floor, the belts, the side
//     kicker, the Stealth wheels (speeds: estimates), and keeps running through the wheel's dip.
//   - "A CANrange pointed into hopper measures hopper fill level. During 'Passing' robot automatically
//     fills hopper to a minimum level before starting to pass. While passing, if hopper level drops
//     below minimum, robot stops passing and starts accumulating" (PASS_MIN, est.; only while it's
//     intaking as it passes: a pass the AI sends it home to make empties it).
//   - "Smart stator current monitoring used to detect jams and automatically unjam hopper": the side
//     kicker over 60 A at under a fifth of its speed for 0.1 s backs off 0.15 s (est., as 1690's).
//   - "Polygon zone localization ... reduces trench-hood collisions": under a TRENCH, or about to be,
//     it doesn't feed and the hood stays down at its steep end (its arm lowest), and the arm presses
//     the net back down onto the load.
//   - unjam (the AI's outtake; their "Purge ... reverses all rollers while it's held"): everything
//     backward.
import { Mechanism } from './mechanism.js';
import { PhysicalRobot } from './physical.js';
import {
  build2056, wheels2056, shooterWheel, kickerWheel, pathSurfaces2056, placeIntake, placeTurret, turretPoint, intakeFront, kickAt, kickLoad, floorRoll,
  HALF_L, HALF_W, INTAKE, FLOOR, TOP, BELT, VKICK, FEED, SHOOTER, TURRET, NET, WALL, PRELOAD, SLIDE, PLATE, GUARD, bowShape,
} from './r2056.js';
import { speedLoop, profile } from './phys1706.js';
import { Field } from '../field.js';
import { underTrench } from '../nav.js';
import { ShotTable, solveMovingShot } from '../ballistics.js';
import { FUEL, TRENCH } from '../constants.js';
import { clamp } from '../util.js';
import { drive2056 } from './turretDrive.js';

const DEG = Math.PI / 180, TAU = 2 * Math.PI, R = FUEL.radius;
// the indexer's motors' RPS (est.: their speeds aren't published; near the top of what each motor
// does through its gearing, a "high throughput" singulator: the floor's X60 at 90 (3.3 m/s at the
// rollers), the belts' X44s at 115 (4.6 m/s); at 1690's 75 and 100 a full hopper dumps ~6 FUEL/s
// on the bench, at these ~7), scoring and passing
export const RPS = { floor: [90, 90], belts: [115, 115], vkick: 60 };
// the Stealth wheels' surface: a share of the shooter wheel's, and at least this (m/s; est., as 1690's)
export const FEED_SHARE = 0.6, FEED_MIN = 5;
// the backspin kicker's RPM per the flywheel's (est.: its surface at 0.8 of the wheel's)
export const KICK_RATIO = 1.6;
const LATENCY = 0.05; // s: their loop and the turret's lag (est.); about a FUEL's time on the wheel
// the last shot's setpoints held this long once it's lost (s, est.)
const HOLD = 0.3;
// Its shots, as the bench measures them (tools/mechlab2056.mjs calibrate: FUEL fed up the bore in a
// stream by the indexer, the way it fires): where the FUEL leave (the turret's frame: s along the shot
// from its axis, y up); the elevation each arm angle gives (EL_MAP: within 1-4 deg of the geometry's
// release angle; ELEV_GEOM=1 commands the geometric angle, as their binder states the range); the
// wheel's surface speed for an exit speed. Their code maps exit speed to wheel speed with "a manually
// tuned coefficient" (EXIT_K: the bench's best single one, 0.59); in the sim the exit stops growing
// near 8.8 m/s (the foam's grip on the wheel over the hood's wrap: 1690's does the same near 9.8), so
// one coefficient would put long shots well short: EXIT_MAP is that coefficient tuned at each speed
// over the HUB table's pairs (its arm 153-162 deg), PASS_MAP at the lowest release angle, where
// passes go (ONE_K=1: their single coefficient). EXIT_MAP as their drive team would finish it, on the
// field: AI matches' HUB shots (against 2910, ~3400 of them, the wheel within 2% of its target at the
// release) leave at these speeds in a match's stream; the bench's map expected 3-5% more from 14
// m/s up (its 4.5 m row out at 7.8-8.1 of the 8.25 asked, every way the turret faces), 2-3% less
// at 11-12: the long shots 0.15-0.3 m short, the close ones 0.1-0.2 long.
export const EXIT = { s: 0.048, y: 0.631 };
export const EL_MAP = [[129.5, 35.5], [135, 41.1], [141, 45.6], [147, 50.5], [153, 57.2], [156, 60.8], [159, 64.8], [161, 67.1], [163, 70.1], [163.5, 70.9]];
export const EXIT_K = 0.578;
export const EXIT_MAP = [[0, 0], [5.75, 8], [6.1, 9], [6.45, 10], [6.75, 11], [7.0, 12], [7.25, 13], [7.45, 14], [7.65, 15], [7.8, 16], [7.95, 17], [8.1, 18], [8.2, 19], [8.25, 20]];
export const PASS_MAP = [[0, 0], [5.9, 8], [7.2, 10], [7.95, 12], [8.32, 14], [8.74, 16], [8.86, 18], [8.91, 20]];
const GEOM = typeof process !== 'undefined' && process.env?.ELEV_GEOM === '1';
const ONE_K = typeof process !== 'undefined' && process.env?.ONE_K === '1';
const interp = (t, x, a = 0, b = 1) => {
  if (x <= t[0][a]) return t[0][b];
  for (let k = 1; k < t.length; k++) if (x <= t[k][a]) return t[k - 1][b] + ((t[k][b] - t[k - 1][b]) * (x - t[k - 1][a])) / (t[k][a] - t[k - 1][a]);
  return t[t.length - 1][b];
};
const HOOD_RANGE = [SHOOTER.elMin + 90, SHOOTER.elMax + 90];
// the hood's arm angle (deg about the axle) for an elevation (deg), and the elevation an angle gives
export const hoodFor = (el) => clamp(GEOM ? el + 90 : interp([...EL_MAP].sort((p, q) => p[1] - q[1]), el, 1, 0), ...HOOD_RANGE);
export const elevationAt = (h) => (GEOM ? h - 90 : interp(EL_MAP, h));
// the wheel's surface speed (m/s) for an exit speed, and back
export const surfFor = (v, pass = false) => (ONE_K ? v / EXIT_K : interp(pass ? PASS_MAP : EXIT_MAP, v));
export const exitAt = (surf, pass = false) => (ONE_K ? surf * EXIT_K : interp(pass ? PASS_MAP : EXIT_MAP, surf, 1, 0));
// (the hood down: its steep end, its arm lowest)
export const HOOD_STOW = HOOD_RANGE[1];
// the elevation passes go at: the hood at its lowest release angle
export const PASS_EL = () => elevationAt(HOOD_RANGE[0]);

// ---- the intake on its racks
const TURNS_PER_M = SLIDE.ratio / (2 * Math.PI * SLIDE.pinion);
// its position loop (volts per motor turn: est.); out and in at different speeds (Dynamic Motion
// Magic: est. as the volts each way)
const SLIDE_KP = 4;
export const RACK = { outV: 12, inV: 8, hold: 0.5 };
// "temporarily retracting the intake to fully empty hopper while shooting": once the load's down to
// RETRACT_AT while it shoots and isn't intaking (est.)
export const RETRACT_AT = Number(globalThis.process?.env?.RETRACT_AT ?? 20);
export const FLOOR_INTAKE = 75; // the floor's RPS while it intakes (est.)
// passing while intaking: the least it keeps in the hopper (est.: their CANrange's level)
export const PASS_MIN = Number(globalThis.process?.env?.PASS_MIN ?? 8);
// its X44 for the intake's state st ({ pos (m out) }): out, held there; in (retract)
export function driveRacks(st, slide, { on, retract }) {
  const T = SLIDE.travel;
  if (st.pos >= T - 1e-6) slide.stop(-Infinity, 0); else if (st.pos <= 1e-6) slide.stop(0, Infinity); else slide.stop();
  if (!on) { slide.cmd = null; return; }
  const target = retract ? 0 : T;
  let v = clamp(SLIDE_KP * (target - st.pos) * TURNS_PER_M, -RACK.inV, RACK.outV);
  if (target >= T) v = Math.max(v, RACK.hold);
  if (target <= 0) v = Math.min(v, -RACK.hold);
  slide.cmd = v / 12;
}
export function moveRacks(st, slide, dt) {
  st.pos += slide.omega * SLIDE.pinion * dt;
  if (st.pos < 0) { st.pos = 0; if (slide._omega < 0) slide._omega = 0; }
  if (st.pos > SLIDE.travel) { st.pos = SLIDE.travel; if (slide._omega > 0) slide._omega = 0; }
}
// ---- the kicker bar's arms on their pin (st.kick: deg from folded, toward fallen the sign of
// INTAKE.kick.fall): no further out than the intake lets them (kickAt: it folds them against the bumper
// coming in) and their stop; their weight and the tubing pull them down, what FUEL push on the bar
// lifts them
const kickRange = (lim) => [Math.min(0, lim), Math.max(0, lim)];
export function driveKick(st, arm, ext) {
  const lim = kickAt(ext), [lo, hi] = kickRange(lim), s = Math.sign(INTAKE.kick.fall);
  st.kick = clamp(st.kick, lo, hi);
  const atFall = Math.abs(st.kick - lim) < 1e-6, atFold = Math.abs(st.kick) < 1e-6;
  // (at a limit it can only move back inside the range)
  const towardFold = s > 0 ? [-Infinity, 0] : [0, Infinity], towardFall = s > 0 ? [0, Infinity] : [-Infinity, 0];
  if (atFall && atFold) arm.stop(0, 0); else if (atFall) arm.stop(...towardFold); else if (atFold) arm.stop(...towardFall); else arm.stop();
  arm.load = kickLoad(st.kick);
  arm.cmd = null;
}
export function moveKick(st, arm, dt, ext) {
  st.kick += (arm.omega * dt) / DEG;
  const [lo, hi] = kickRange(kickAt(ext));
  if (st.kick < lo) { st.kick = lo; if (arm._omega < 0) arm._omega = 0; }
  if (st.kick > hi) { st.kick = hi; if (arm._omega > 0) arm._omega = 0; }
  st.kickMax = Math.max(st.kickMax ?? -Infinity, (INTAKE.kick.fall - st.kick) * Math.sign(INTAKE.kick.fall));
}

export class Physical2056 extends PhysicalRobot {
  constructor(robot) {
    const t = build2056(), w = wheels2056(t), s = pathSurfaces2056(t), sw = { wheel: shooterWheel(t), kicker: kickerWheel(t) };
    const pathWheels = [...w.intake, ...w.floor, w.vkick, ...w.feed];
    super(robot, {
      path: new Mechanism({ body: robot.body, substeps: 6, shafts: [...new Set(pathWheels.map((x) => x.shaft)), ...t.belts, t.slide, t.kickArm], wheels: pathWheels, surfaces: [s.body, s.bore, s.bottom, s.top, s.carriage, s.kickTube, s.bumper, s.net] }),
      // (the turret's side plates with the wheel and the backspin kicker: a FUEL leaves at up to ~15
      // m/s, too far in a step to follow without sub-steps; carried through them with the robot's
      // motion, or a FUEL in it creeps against them driving: 4% faster at 1.5 m/s along the shot)
      shooter: new Mechanism({ body: robot.body, substeps: 6, sub: 8, carried: true, shafts: [t.wheel, t.kicker], wheels: [sw.wheel, sw.kicker], surfaces: [s.turret, s.hood] }),
      preload: PRELOAD,
    });
    Object.assign(this, { t, w, s, sw });
    this.ownsIntake = true;
    this.ownsHood = true;
    // passes at the hood's lowest release angle, the wheel's speed varied (their thread): a pass table
    // of that one angle (ballistics.js), in place of the robot's
    const sh = robot.cfg.shooter, el = PASS_EL();
    robot.passTable = new ShotTable({ h0: sh.exitY, Ht: FUEL.radius + 0.02, hoodMin: el, hoodMax: el, speedMax: sh.speedMax, mode: 'pass', passTheta: el, dMax: 22 });
    this.reset();
  }

  reset() {
    super.reset();
    this.deploy = 0; this.pos = 0; this.kick = 0; this.deployed = false; this.retracting = false;
    this.hood = HOOD_STOW; this.hoodSet = [HOOD_STOW, 0]; this.hoodGoal = HOOD_STOW;
    this.feedingNow = false; this.slowT = 0; this.jamT = 0; this.backT = 0; this.ready = false; this.accumulating = false;
    this.want = null; this.aimPsi = null; this.last = null;
    const T = this.t;
    for (const tr of [T.intake, T.floor, T.feed, T.shooter]) { tr.omega = 0; tr.cmd = null; tr.vel = null; }
    for (const sh of [T.kickArm, T.vkick, T.slide, T.kicker, ...T.belts]) { sh._omega = 0; sh.cmd = null; sh.vel = null; }
  }

  // its FUEL: in the frame (the hopper over the floor, the belts' channel, the quarter pipe, up into
  // the turret), in the intake out front, or on the intake climbing the bumper
  where(x, y, z) {
    const inFrame = x > -HALF_L - 0.02 && x < HALF_L && Math.abs(z) < HALF_W && y > 0.09 && y < 0.8;
    const out = this.deploy > 0.5;
    const inFront = out && x >= HALF_L && x < this._front() + 0.01 && Math.abs(z) < INTAKE.frame.sides && y < 0.62;
    return {
      own: inFrame || inFront,
      held: inFrame || (inFront && y > 0.20 && x < this._front() - 0.04),
      near: x > -HALF_L - 0.1 && x < this._front() + 0.15 && Math.abs(z) < HALF_W + 0.1 && y < 0.9,
    };
  }
  // what the shooter might touch: up in the turret
  shooterBalls(near) {
    return near.filter((b) => { const [x, y, z] = this._local(b.pos); return y > 0.33 && Math.hypot(x - TURRET.at[0], z - TURRET.at[1]) < 0.3; });
  }
  _front() { return intakeFront(this.deploy); }

  preStep(dt, t) {
    const r = this.r;
    if (this.lastDeploy !== undefined && Math.abs(r.intakeDeploy - this.lastDeploy) > 1e-6) { this.pos = SLIDE.travel * r.intakeDeploy; this.kick = kickAt(r.intakeDeploy); this.deployed = r.intakeDeploy > 0.5; this.t.slide._omega = 0; this.t.kickArm._omega = 0; }
    if (r.enabled) this.deployed = true;
    driveRacks(this, this.t.slide, { on: r.enabled && this.deployed, retract: this.retracting });
    this.deploy = this.pos / SLIDE.travel;
    driveKick(this, this.t.kickArm, this.deploy);
    this.hoodSet = profile(...this.hoodSet, this.hoodGoal, SHOOTER.hoodV, SHOOTER.hoodA, dt);
    this.hood = this.hoodSet[0];
    this._place(r.turretYaw || 0);
    this.stepMechanisms(dt, t);
    this.s.net.flex(dt, this.path.contacts);
    floorRoll(this.path.contacts, dt);
    moveRacks(this, this.t.slide, dt);
    this.deploy = this.pos / SLIDE.travel;
    moveKick(this, this.t.kickArm, dt, this.deploy);
    r.intakeDeploy = this.lastDeploy = this.deploy;
  }
  _place(yaw) {
    const s = this.s;
    placeIntake(s, this.w.intake, this.deploy, this.kick);
    s.net.place(Math.max(HALF_L, this._front()));
    placeTurret(s, this.sw, yaw, this.hood);
  }

  // the shot ("kinematically computed shot vector with time-of-flight compensation"): from where the
  // FUEL will leave LATENCY ahead on the robot's motion, with that point's velocity (the turret's
  // axis, 5.25in back and 7.25in right of the center, swinging with the chassis), the table's shot
  // to the target moved back along it by the flight's time and refined against the drag model's
  // flight (ballistics.js solveMovingShot). Its exit speed and elevation are a pair of the table's,
  // relative to the turret: the pairs EXIT_MAP and EL_MAP were measured on, the elevation in the
  // hood's range. (The table's shot less the robot's velocity, as it was, asked for elevations past
  // the hood's steep end driving in close, 1 shot in 10 toward the HUB, which went long, and pairs
  // off the maps'.)
  _aim(tgt, table) {
    const r = this.r, b = r.body, v = b.linvel(), w = b.angvel().y, L = LATENCY;
    const c = b.translation(), yaw = r.yaw + w * L, cy = Math.cos(yaw), sy = Math.sin(yaw);
    const [tx, tz] = TURRET.at;
    const ox = tx * cy + tz * sy, oz = -tx * sy + tz * cy;
    const px = c.x + v.x * L + ox, pz = c.z + v.z * L + oz;
    const dx = tgt.x - px, dz = tgt.z - pz, dd = Math.hypot(dx, dz) || 1;
    const exit = { x: px + (dx / dd) * EXIT.s, y: EXIT.y, z: pz + (dz / dd) * EXIT.s };
    const sol = solveMovingShot(table, exit, { x: v.x + w * oz, z: v.z - w * ox }, tgt);
    return sol ? { psi: sol.psi, el: sol.theta / DEG, v: sol.v, d: sol.dist } : null;
  }

  postStep(dt, t, { on, feeding, outtake, mode }) {
    const r = this.r, T = this.t, S = SHOOTER;
    this.account(t, mode);
    const intaking = on && !!r.cmd.intake, unjam = on && !!outtake;
    T.intake.cmd = !on ? 0 : unjam ? -0.6 : intaking || this.feedingNow ? 1 : null;
    // ---- the shot
    const tgt = r.shot?.target, pass = tgt?.mode === 'pass';
    const table = pass ? r.passTable : r.hubTable;
    const point = tgt ? (pass ? { x: tgt.x, z: tgt.z } : Field.hubCenter(r.alliance)) : null;
    const sol = on && point && !unjam ? this._aim(point, table) : null;
    this.want = sol;
    this.aimPsi = sol ? sol.psi : null;
    // (the shot lost with FUEL on their way up, a moment out of range or the AI letting go: the wheel
    // and hood hold the last shot's setpoints HOLD s, as a state machine leaving its shooting state
    // would, the indexer stopping. Dropped at once, the hood swung to its stow and the wheel coasted
    // under the FUEL already in the turret: 6% of the HUB shots, 1 in 8 of them scoring)
    if (sol) this.last = { sol, pass, t };
    const hold = !sol && on && !unjam && this.last && t - this.last.t < HOLD ? this.last : null;
    const aim = sol || hold?.sol, aimPass = sol ? pass : !!hold?.pass;
    const surf = aim ? surfFor(aim.v, aimPass) : 0;
    const inTrench = underTrench(r.pos.x, r.pos.z, 0.45);
    this.s.net.cap = inTrench ? Math.max(0, TRENCH.clearHeight - 0.003 - NET.y) : PLATE.max;
    if (aim) {
      this.hoodGoal = inTrench ? HOOD_STOW : aimPass ? HOOD_RANGE[0] : hoodFor(aim.el);
      speedLoop(T.shooter, surf / S.r, S.kP);
      speedLoop(T.kicker, (KICK_RATIO * surf) / S.r, S.kP);
    } else { this.hoodGoal = HOOD_STOW; speedLoop(T.shooter, null); speedLoop(T.kicker, null); }
    const wheelSurf = Math.abs(T.wheel.omega) * S.r;
    const atV = !!sol && Math.abs(wheelSurf - surf) < S.tolV * surf;
    const atHood = Math.abs(this.hood - this.hoodGoal) < S.tolHood;
    const tol = pass ? 4 * DEG : Math.max(0.8 * DEG, Math.atan2(0.14, sol?.d || 1));
    const atYaw = Math.abs(r.aimErr ?? Infinity) < tol;
    this.ready = !!sol && atV && atHood && atYaw && !inTrench;
    // (passing while intaking: down to PASS_MIN it stops passing and accumulates, back over it + 4)
    const n = this.held.length;
    if (!(pass && intaking)) this.accumulating = false;
    else if (n <= PASS_MIN) this.accumulating = true;
    else if (n >= PASS_MIN + 4) this.accumulating = false;
    this.feedingNow = on && feeding && !!sol && !unjam && !inTrench && !this.accumulating && atYaw && atHood && (atV || this.feedingNow);
    // (the intake in to push the last of the load to the floor, while it shoots and isn't intaking)
    this.retracting = this.feedingNow && !intaking && n <= RETRACT_AT;
    // ---- the indexer
    this.slowT = sol && wheelSurf < 0.92 * surf ? this.slowT + dt : 0;
    const k = pass ? 1 : 0;
    const rps = (n0, sh) => (n0 * TAU) / sh.motor.ratio;
    const vk = T.vkick;
    this.jamT = this.feedingNow && Math.abs(vk.motor.current) > 60 && Math.abs(vk.omega) < 0.2 * rps(RPS.vkick, vk) ? this.jamT + dt : 0;
    if (this.jamT > 0.1) { this.backT = 0.15; this.jamT = 0; r.stats.unjams = (r.stats.unjams || 0) + 1; }
    this.backT = Math.max(0, this.backT - dt);
    if (unjam) {
      speedLoop(T.floor, -rps(30, T.floor)); for (const b of T.belts) speedLoop(b, -rps(40, b)); speedLoop(vk, -rps(30, vk)); speedLoop(T.feed, -rps(30, T.feed));
    } else if (this.feedingNow) {
      speedLoop(T.floor, rps(RPS.floor[k], T.floor));
      for (const b of T.belts) speedLoop(b, rps(RPS.belts[k], b));
      speedLoop(vk, (this.backT > 0 ? -1 : 1) * rps(RPS.vkick, vk));
      speedLoop(T.feed, Math.max(FEED_MIN, FEED_SHARE * surf) / FEED.r);
    } else {
      for (const a of [T.floor, ...T.belts, vk, T.feed]) speedLoop(a, null);
      if (intaking) speedLoop(T.floor, rps(FLOOR_INTAKE, T.floor));
    }
    r.hoodDeg = elevationAt(this.hood);
    r.flywheel = exitAt(wheelSurf, pass);
    r.ready = this.ready;
    r.feeding = this.feedingNow ? 1 : 0;
    r.intakeSpeed = T.intake.cmd > 0 ? 1 : T.intake.cmd < 0 ? -1 : 0;
  }

  // how far the intake's box is out past the frame (m), for the hopper's collider and drawing it
  extNow() { return Math.max(0, this._front() - HALF_L); }
  // the other moving parts, for drawing them
  mechPose() {
    const T = this.t;
    return { ext: this.deploy, kick: this.kick * DEG, hood: this.hood * DEG, wheel: T.wheel.omega, kicker: T.kicker.omega, floor: T.floor.omega, belts: T.belts.map((b) => b.omega * BELT.pitch), vkick: T.vkick.omega, feed: T.feed.omega, intakeRollers: T.intake.omega, bow: this.s.net.bow };
  }
  // the load's top (for the TRENCH): where the net's lifted over a FUEL, as far down as a TRENCH's arm
  // could press it (PLATE.press)
  loadTop() {
    const bow = this.s.net.bow;
    let top = 0;
    for (const b of this.held) {
      const [x, y, z] = this._local(b.pos);
      // (not one up in the turret, under its guard: on its way to the wheel, not under the net)
      if (Math.hypot(x - TURRET.at[0], z - TURRET.at[1]) < GUARD.r) continue;
      const tp = y + R - (bow > 0 && x > NET.x0 && Math.abs(z) < NET.hw ? Math.min(PLATE.press, bow * bowShape(Math.min(x, HALF_L), z)) : 0);
      if (tp > top) top = tp;
    }
    return top;
  }
  // how many it holds, rated (tools/mechlab2056.mjs intakefull: filled through its intake it holds
  // up to 63 with the net lifted, the intake's box full; the NEUTRAL ZONE's line at 1.5 m/s puts ~45
  // in, tools/intake-test.mjs)
  capacity() { return 55; }

  // the turret on its X44 (js/mech/turretDrive.js drive2056)
  turretStep(cur, goal, dt, t) {
    this.turretDrive ||= drive2056();
    return this.turretDrive.step(cur, goal, dt, t);
  }
}
export { NET, WALL, FLOOR, VKICK, turretPoint };
