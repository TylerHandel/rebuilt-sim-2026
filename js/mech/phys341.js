// 341's Miss Daisy XXIV in play, built of its physical mechanisms (js/mech/r341.js; the shared
// part: js/mech/physical.js). Its controls are its code's (Team341/FRC2026-Public: Intake,
// Shooter, Turret and RobotContainer's bindings):
//   - the intake down (their 149 deg, a degree off the bumper it rests on, which their encoder's
//     zeroed to at boot as 150) and held stiff;
//   - shooting (the driver's right trigger, the AI's shot): the flywheel's velocity loop at their
//     table's RPM for the distance from the turret to the target, the hood to their table's angle
//     (stowed under a TRENCH); once it's within 130 RPM (debounced 0.1 s) and the turret's within
//     6 deg (Turret.readyToShoot, debounced 0.2 s), SHOOTING: the intake's rollers and the top
//     rollers / uptake run at 12 V (DEPLOYED_INTAKING, their intake's default command while it
//     shoots) and the feeder (the powered omni) at 3 V while the hood's within 3 deg;
//   - their unjam: while it intakes and shoots, the top rollers or the feeder stalled (under 500
//     RPM at over 75 A on the leader motor) for 0.2 s pulses them out for 0.25 s (feeder -3 V,
//     rollers -12 V, top rollers -6 V). (Neither stalls when the zipper locks up, below: the top
//     rollers slip on the FUEL they're over, and a Falcon at 3 V can't draw 75 A.);
//   - let go: the flywheel stays at speed (AT_SPEED), the hood stowed, the feeder and the intake's
//     rollers stopped, and the top rollers and uptake run backward at 3 V (their DEPLOYED case
//     has no break: it falls through into ZEROING's -3 V);
//   - the AI's driver lets go and presses again when nothing comes out (letGo).
//   - not their code: the feeder backed off a moment when nothing's come out for 0.3 s with FUEL
//     in it (FEED_BACKOFF; FEED_BACKOFF=0: off), which clears the zipper's lock their unjam can't
//     see; FEEDER_V (their 3 V) sets the feeder's voltage. JAMLOG=1: each let-go and unjam logged.
// Their robot only intakes while it shoots (it's hopperless: the right trigger does both, and the
// intake's default command runs its rollers only while the shooter's SHOOTING). The AI's intake
// command without a shot runs just the intake's rollers, so what it picks up waits on the tray:
// the top rollers would run the uptake too (they share its motors), feeding FUEL up into the
// spinning flywheel unaimed.
import { Mechanism } from './mechanism.js';
import { PhysicalRobot } from './physical.js';
import {
  build341, wheels341, shooterWheels, placeShooter, placeIntake, intakeRot, hoodElevation, pathSurfaces341, placeTurretSurfaces,
  shotFor, HALF, INTAKE, SHOOTER, PRELOAD, PIVOT, TURRET,
} from './r341.js';
import PATH from './path341.js';
import { Field } from '../field.js';
import { underTrench } from '../nav.js';
import { clamp } from '../util.js';
import { HALF_L, ALLIANCE_ZONE_DEPTH } from '../constants.js';

const DEG = Math.PI / 180;
const R = 0.075;
const ENV = (typeof process !== 'undefined' && process.env) || {};
const JAMLOG = !!ENV.JAMLOG;
export const FEEDER_V = Number(ENV.FEEDER_V ?? 3);
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
// FUEL leave at this fraction of the flywheel's surface speed (tools/mechlab341.mjs shooter: 0.67-0.72
// over the HUB's rows, a few degrees steeper than the CAD's geometry gives), up to ~2700 RPM;
// past that they slip on it, and leave at no more than 8 m/s plus 0.15 of its surface speed
// (10.9 m/s at 3670 RPM, 12.4 at 5500): exitSpeed. (The shot tables are re-found by shooting:
// js/mech/r341.js SIM_SHOTS)
export const EXIT_PER_FLY_SURFACE = 0.7, EXIT_SLIP = [8.0, 0.15];
// the AI's driver letting go of the trigger when nothing comes out (letGo): after (s) without a
// shot, for (s)
export const LET_GO = { after: 0.8, for: 0.4 };
// Not their code: the feeder backed off on its own (at FEEDER_V, the other way) for FEED_BACKOFF.for
// s after FEED_BACKOFF.after s shooting with FUEL in it and nothing out, again each time. (The
// zipper's last pair: one in the lane just short of the uptake's flex wheels, the other pressed
// into it from the right by the powered omni and the front top roller's mecanums, the lane's
// FUEL held against the left wall; nothing stalls, so their unjam can't see it. Backing the omni
// off a moment lets go of the one from the right, and the lane's goes on into the uptake.
// tools/mechlab341.mjs path; FEED_BACKOFF=0: off.)
export const FEED_BACKOFF = { after: 0.3, for: Number(ENV.FEED_BACKOFF ?? 0.1) };

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
  if (deg > INTAKE.down + 1) { deg = INTAKE.down + 1; if (T.pivot._omega > 0) T.pivot._omega = 0; } // (the bumper: the arm's plate 5 mm into its foam, where it rests at their 150)
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
    // down from the start (their encoder's zeroed resting on the bumper)
    this.armDeg = INTAKE.down; this.armGoal = INTAKE.down;
    this.hoodDeg = 0; this.hoodSet = [0, 0]; this.hoodGoal = 0;
    this.state = 'stopped'; // their shooter: stopped, spinning up, shooting, at speed, pulsing out
    this.targetRpm = 0; this.atSpeedT = 0; this.turretT = 0; this.jamT = 0; this.pulseT = 0;
    this.lastShotT = 0; this.shotsSeen = 0; this.letGoUntil = -1; this.now = 0; this.backT = 0; this.backFrom = 0;
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
    // their RPM and hood for the distance from the turret to where it aims (their shooting on the
    // move, RobotState.calculateAimingPoint: the target less the turret's own field velocity,
    // filtered, times the shot's time of flight)
    this._aim(dt, pass);
    if (r.shot) {
      const want = shotFor(pass ? 'ferry' : 'hub', this.aimDist);
      if (wantShot || this.state !== 'stopped') this.targetRpm = want.rpm;
      this.wantHood = want.hood;
    }
    const rpm = (Math.abs(T.fly.omega) * 60) / (2 * Math.PI);
    // (their atSpeed: within 130 RPM, its falling edge debounced 0.1 s)
    this.atSpeedT = Math.abs(rpm - this.targetRpm) <= SHOOTER.tol ? 0.1 : Math.max(0, this.atSpeedT - dt);
    const atSpeed = this.atSpeedT > 0 && this.targetRpm > 0;
    // (their Turret.readyToShoot: within 6 deg of where it should point, its falling edge
    // debounced 0.2 s)
    this.turretT = Math.abs(r.aimErr ?? Infinity) < 6 * DEG ? 0.2 : Math.max(0, this.turretT - dt);
    const turretReady = this.turretT > 0;
    const inTrench = underTrench(r.pos.x, r.pos.z, 0);
    if (!on) this.state = 'stopped';
    else if (wantShot) this.state = this.pulseT > 0 ? 'pulsing' : atSpeed && turretReady ? 'shooting' : 'spinning up';
    else if (this.state !== 'stopped') this.state = 'at speed';
    const shooting = this.state === 'shooting' || this.state === 'pulsing';
    // the hood: their table's angle while it shoots (stowed under a TRENCH), stowed otherwise
    this.hoodGoal = (this.state === 'shooting' || this.state === 'spinning up' || this.state === 'pulsing') && !inTrench ? this.wantHood ?? 0 : 0;
    // the intake: their default command runs it while it shoots; the AI's intake command too
    // (not while its driver has let go of the trigger: letGo)
    this.now = t;
    const lettingGo = t < this.letGoUntil;
    const intaking = on && (shooting || (!!r.cmd.intake && !lettingGo));
    if (r.stats.shots !== this.shotsSeen) { this.shotsSeen = r.stats.shots; this.lastShotT = t; }
    if (this.state !== 'shooting') this.lastShotT = t;
    // their unjam: the top rollers or the feeder stalled for 0.2 s while it intakes and shoots
    // (Intake.isStopped, Shooter.feederStopped: the leader motor's current, under 500 RPM)
    const stalled = (tr) => (Math.abs(tr.omega) * 60) / (2 * Math.PI) < 500 && Math.abs(tr.motor.current) / tr.motor.count > 75;
    this.jamT = intaking && shooting && (stalled(T.serializer) || stalled(T.feederT)) ? this.jamT + dt : 0;
    if (this.jamT > 0.2 && this.pulseT <= 0) { this.pulseT = 0.25; this.jamT = 0; r.stats.unjams = (r.stats.unjams || 0) + 1; if (JAMLOG) this.logJam(t, 'UNJAM'); }
    this.pulseT = Math.max(0, this.pulseT - dt);
    const pulsing = this.pulseT > 0;
    // (the feeder's back-off: FEED_BACKOFF)
    this.backFrom = Math.max(this.backFrom, this.lastShotT);
    if (FEED_BACKOFF.for > 0 && this.state === 'shooting' && r.stored.length > 0 && t - this.backFrom > FEED_BACKOFF.after) { this.backT = FEED_BACKOFF.for; this.backFrom = t + FEED_BACKOFF.for; r.stats.feedBackoffs = (r.stats.feedBackoffs || 0) + 1; }
    this.backT = Math.max(0, this.backT - dt);
    T.intake.cmd = !on ? 0 : outtake || pulsing ? -1 : intaking ? 1 : 0;
    // (not shooting, their intake's default command puts it in DEPLOYED, whose case falls through
    // into ZEROING's: the intake's rollers stopped and the top rollers and uptake run backward at
    // 3 V. The AI's intake command without a shot, which their code doesn't have, leaves them off.)
    T.serializer.cmd = !on ? 0 : outtake || pulsing ? -0.5 : shooting ? 1 : intaking ? 0 : -3 / 12;
    T.feederT.cmd = !on ? 0 : pulsing ? -3 / 12 : this.backT > 0 && this.state === 'shooting' ? -FEEDER_V / 12 : this.state === 'shooting' && Math.abs(this.hoodDeg - this.hoodGoal) <= 3 ? FEEDER_V / 12 : 0;
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

  // where the turret aims and how far that is (their RobotState): the turret's field velocity
  // (the robot's, plus its turn about the turret's offset), low-passed as theirs is (alpha 0.25 a
  // 20 ms loop), times the time of flight they use (the HUB's 1.16 s; a feed's 1.3 s from their
  // own half, 1.6 s from the far half, 1.8 s from the far ALLIANCE ZONE) off the target.
  // aimPsi: the turret's field heading to it (Robot._shooter turns the turret there)
  _aim(dt, pass) {
    const r = this.r, tp = this._world([TURRET[0], 0, TURRET[1]]), c = r.body.translation(), v = r.body.linvel(), w = r.body.angvel().y;
    const vx = v.x + w * (tp.z - c.z), vz = v.z - w * (tp.x - c.x);
    const k = 1 - Math.pow(0.75, dt / 0.02);
    this.fv = this.fv ?? { x: vx, z: vz };
    this.fv.x += k * (vx - this.fv.x); this.fv.z += k * (vz - this.fv.z);
    const tgt = pass && r.shot ? r.shot.target : Field.hubCenter(r.alliance);
    const own = r.alliance === 'blue' ? r.pos.x + HALF_L : HALF_L - r.pos.x; // (from its own wall)
    const tof = !pass ? 1.16 : own < HALF_L ? 1.3 : own < 2 * HALF_L - ALLIANCE_ZONE_DEPTH ? 1.6 : 1.8;
    const ax = tgt.x - this.fv.x * tof, az = tgt.z - this.fv.z * tof;
    this.aimDist = Math.hypot(ax - tp.x, az - tp.z);
    this.aimPsi = Math.atan2(-(az - tp.z), ax - tp.x);
  }
  // The AI's driver (js/opponent.js asks each step; true: let go of the trigger now). Shooting
  // with FUEL in it and nothing out for LET_GO.after s, it lets go for LET_GO.for s and presses
  // again, as a driver does when nothing comes out: their code then puts the intake in DEPLOYED
  // (the top rollers and uptake backward at 3 V, the intake's rollers and the feeder stopped)
  // and the shooter AT_SPEED (the hood stowed). That backs off the FUEL pressed into the zipper's
  // merge, which their unjam can't see (README: 341). (In matches it lets go a few times a
  // match; tools/mechlab341.mjs path LETGO=1.)
  letGo() {
    const t = this.now;
    if (t < this.letGoUntil) return true;
    if (this.state === 'shooting' && this.r.stored.length > 0 && t - this.lastShotT > LET_GO.after) {
      this.letGoUntil = t + LET_GO.for;
      this.r.stats.letGo = (this.r.stats.letGo || 0) + 1;
      if (JAMLOG) this.logJam(t, 'LETGO');
      return true;
    }
    return false;
  }
  // (JAMLOG=1: each let-go and unjam: when, how many it holds, the top rollers' and the feeder's
  // speed and current, and the FUEL pressed hardest (over 30 N), robot frame, and what by)
  logJam(t, tag) {
    const T = this.t, pts = new Map();
    for (const c of this.path.contacts) if (c.force > 30) { const e = pts.get(c.ball) ?? []; e.push(`${c.wheel.name.replace(/ \(.*\)/, '')} ${c.force.toFixed(0)}`); pts.set(c.ball, e); }
    const rps = (tr) => (Math.abs(tr.omega) / (2 * Math.PI)).toFixed(0);
    const out = [...pts].map(([b, e]) => { const p = this._local(b.translation()); return `${p.map((v) => v.toFixed(2)).join(',')} [${e.join('; ')}]`; });
    console.log(`${tag} t ${t.toFixed(1)} held ${this.r.stored.length}, top rollers ${rps(T.serializer)} rps ${Math.abs(T.serializer.motor.current).toFixed(0)} A, feeder ${rps(T.feederT)} rps ${Math.abs(T.feederT.motor.current).toFixed(0)} A: ${out.join(' | ')}`);
  }
  // the arm's turn from the CAD's pose (rad), for drawing it
  armTurn() { return intakeRot(this.armDeg); }
  // about how fast a FUEL leaves at the flywheel's speed now (m/s; tools/mechlab341.mjs shooter)
  exitSpeed() { const s = (Math.abs(this.t.fly.omega) * SHOOTER.fly.d) / 2; return Math.min(EXIT_PER_FLY_SURFACE * s, EXIT_SLIP[0] + EXIT_SLIP[1] * s); }
  // how many it holds in its path (tools/mechlab341.mjs hold)
  capacity() { return 8; }
}
export { R };
