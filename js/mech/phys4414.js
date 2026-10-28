// 4414's RIPCURRENT in play, built of its physical mechanisms (js/mech/r4414.js; the shared part:
// js/mech/physical.js). Their code isn't public; its controls follow their binder and what the team
// said on Chief Delphi:
//   - the intake box: out on its racks at the start of the match (an X44 a side, 3.33:1 at a 10t
//     10DP pinion), and it stays out ("extending hopper holds 85+ balls under the trench"); the
//     intake's rollers run while intaking (the AI's intake, their driver's), back out to outtake.
//   - the Dye Rotor: at their ~2.25 rps while it feeds; otherwise turning back slowly ("rotor
//     constantly spins in the opposite direction slowly to agitate balls and reduce packing
//     pressure"). Its rollers (the TTB wheels) feed only while the shot's ready; the sushi feeder in
//     the turret is geared off the flywheel ("the main bps regulator").
//   - aiming: their precomputed trajectories (js/mech/shot4414.js, made their way by
//     tools/shot4414.mjs: for each distance the shot most robust to errors) give the shot over the
//     carpet; the robot's velocity is taken out of it in 3D ("tangential velocity is compensated for
//     at runtime with the turret using 3D vector math"), leaving the shot the shooter has to make,
//     which their fits turn back into a flywheel speed and a hood. A pass: a fixed hood, its speed
//     solved to land at the pass point.
//   - jams (estimate: their code isn't public): the rotor stalled (over 70 A and under 2 rad/s for
//     0.3 s) turns back for 0.25 s; the TTB wheels stalled back out for 0.15 s.
import { Mechanism } from './mechanism.js';
import { PhysicalRobot } from './physical.js';
import {
  build4414, wheels4414, shooterWheels, pathSurfaces4414, placeIntake, placeRotor, placeRotorSurfaces, placeBox, placeShooter, placeTurretSurfaces,
  hoodElevation, AXIS, HALF_L, BAY, BOX, SHOOTER, PRELOAD,
} from './r4414.js';
import { FIT, LAUNCH } from './shot4414.js';
import { profile, velocityDuty } from './phys604.js';
import { sim2D } from '../ballistics.js';
import { Field } from '../field.js';
import { IN, FUEL } from '../constants.js';
import { clamp } from '../util.js';

const DEG = Math.PI / 180, TAU = 2 * Math.PI, R = FUEL.radius;
const ROTOR_RPS = 2.25, AGITATE_RPS = -0.15;
const RACK_SPEED = (7530 / 60 / 3.33) * Math.PI * 1.0 * IN; // m/s: an X44 through 3.33:1 at a 1in pitch circle
const LATENCY = 0.03; // s: their loop and the turret's lag (estimate)
const PASS_HOOD = 135;  // a pass's hood (about 45 deg)
// their fits: exit speed, elevation (deg), exit point against the flywheel (rad/s) and hood (deg)
const feats = (rad, hood) => { const r = rad / 100, h = (hood - 140) / 10; return [1, r, r * r, r * r * r, h, h * r, h * h]; };
export const fitOf = (k, rad, hood) => feats(rad, hood).reduce((a, f, i) => a + f * FIT[k][i], 0);
const RADS = [200, 700];
const bisect = (f, lo, hi, want) => { for (let k = 0; k < 30; k++) { const m = (lo + hi) / 2; if (f(m) < want) lo = m; else hi = m; } return (lo + hi) / 2; };
// the flywheel speed and hood that make a shot at v (m/s), el (deg): alternately the hood for the
// elevation and the speed for the exit speed
export function shooterFor(v, el) {
  let rad = 450, hood = 145;
  for (let k = 0; k < 4; k++) {
    hood = bisect((h) => fitOf('el', rad, h), SHOOTER.hoodMin, SHOOTER.hoodMax, el);
    rad = bisect((r) => fitOf('v', r, hood), RADS[0], RADS[1], v);
  }
  return { rad, hood };
}
// the table's shot for a distance (m, turret axis to the HUB's center): exit speed, elevation
export function launchFor(d) {
  const t = LAUNCH;
  if (d <= t[0][0]) return { v: t[0][3], el: t[0][4] };
  for (let k = 1; k < t.length; k++) if (d <= t[k][0]) { const f = (d - t[k - 1][0]) / (t[k][0] - t[k - 1][0]); return { v: t[k - 1][3] + f * (t[k][3] - t[k - 1][3]), el: t[k - 1][4] + f * (t[k][4] - t[k - 1][4]) }; }
  const e = t[t.length - 1];
  return { v: e[3], el: e[4] };
}

// the rotor's velocity loop, on its Talon (kV, kP as 604's; closed at the controller's 1 kHz, in
// Shaft.integrate: once a 120 Hz step it chattered at its current limit), at rps; null: coast
export function rotorLoop(shaft, rps) {
  shaft.vel = rps === null ? null : { target: rps * TAU, kV: 0.12, kP: 0.2 };
  shaft.cmd = rps === null ? null : 0;
}

export class Physical4414 extends PhysicalRobot {
  constructor(robot) {
    const t = build4414(), w = wheels4414(t), s = pathSurfaces4414(t), sw = shooterWheels(t);
    const pathWheels = [...w.feed, ...w.intake];
    super(robot, {
      path: new Mechanism({ body: robot.body, substeps: 6, shafts: [...new Set(pathWheels.map((x) => x.shaft)), t.rotor], wheels: pathWheels, surfaces: Object.values(s).filter((x) => x !== s.turret && x !== s.hood) }),
      // (the turret's and the hood's surfaces with its wheels: a FUEL goes round under the hood at up
      // to 10 m/s, too far in a step to follow without sub-steps)
      shooter: new Mechanism({ body: robot.body, substeps: 6, sub: 8, shafts: [t.fly, t.hoodRoller, t.feeder], wheels: sw, surfaces: [s.turret, s.hood] }),
      preload: PRELOAD,
    });
    Object.assign(this, { t, w, s, sw });
    this.ownsIntake = true;
    this.ownsHood = true;
    this.reset();
  }

  reset() {
    super.reset();
    // the box in (the starting configuration), the hood down, the rotor at its pose
    this.ext = 0; this.deployed = false;
    this.hoodDeg = SHOOTER.hoodStow; this.hoodSet = [SHOOTER.hoodStow * DEG, 0]; this.hoodGoal = SHOOTER.hoodStow;
    this.phi = 0;
    this.rotorJam = 0; this.rotorBack = 0; this.rollerJam = 0; this.rollerBack = 0;
    this.targetRad = 0; this.launching = false; this.ready = false; this.feedingNow = false;
    this.va = null;
    for (const tr of [this.t.rollers, this.t.launcher]) { tr.omega = 0; tr.cmd = null; }
    for (const sh of [this.t.rotor, this.t.primary, this.t.bar]) { sh._omega = 0; sh.cmd = 0; }
    this.t.rotor.vel = null;
  }

  // the box's front now
  _front() { return BOX.front - BOX.ext + this.ext; }
  // its FUEL: in the hopper (over the floor, inside the bumper backing), in the box out front (on the
  // intake's ramp, or held up in it), or at the intake's lip with the box out
  where(x, y, z) {
    const xf = this._front();
    const inFrame = x > BAY.x0 - 0.02 && x < HALF_L + 0.01 && Math.abs(z) < BAY.hw + 0.01 && y > 0.1 && y < 0.8;
    const inBox = this.ext > 0.05 && x >= HALF_L && x < xf + 0.01 && Math.abs(z) < BOX.side + 0.01 && y < 0.8;
    const atLip = this.ext > BOX.ext - 0.02 && x >= xf && x < xf + 0.09 && Math.abs(z) < BOX.side && y < 0.2;
    return {
      own: inFrame || inBox || atLip,
      // (held: in it; not one on the carpet at the lip or still on the ramp's foot)
      held: inFrame || (inBox && y > 0.12 && x < xf - 0.03),
      near: x > BAY.x0 - 0.1 && x < xf + 0.15 && Math.abs(z) < BOX.side + 0.1 && y < 0.9,
    };
  }
  // what the turret's wheels might touch: up the hub's bore and in the turret
  shooterBalls(near) { return near.filter((b) => { const [x, y, z] = this._local(b.pos); return y > 0.2 && Math.hypot(x - AXIS[0], z - AXIS[1]) < 0.3; }); }

  preStep(dt, t) {
    const r = this.r, T = this.t;
    // (put somewhere by the game, not by its motors: a staged scene with the intake out)
    if (this.lastDeploy !== undefined && Math.abs(r.intakeDeploy - this.lastDeploy) > 1e-6) { this.ext = BOX.ext * r.intakeDeploy; this.deployed = r.intakeDeploy > 0.5; }
    // the box: out on its racks once the match starts, and it stays out
    if (r.enabled) this.deployed = true;
    if (this.deployed) this.ext = Math.min(BOX.ext, this.ext + RACK_SPEED * dt);
    this.hoodSet = profile(...this.hoodSet, this.hoodGoal * DEG, SHOOTER.hoodV, SHOOTER.hoodA, dt);
    this.hoodDeg = this.hoodSet[0] / DEG;
    // pose the moving parts
    placeIntake(this.w.intake, this.ext);
    placeBox(this.s.box, this.ext);
    placeRotorSurfaces(this.s, this.phi);
    placeRotor(this.w.feed, this.phi);
    const yaw = r.turretYaw || 0;
    placeTurretSurfaces(this.s, yaw, this.hoodDeg);
    placeShooter(this.sw, yaw, this.hoodDeg);
    this.stepMechanisms(dt, t);
    this.phi = (this.phi + T.rotor.omega * dt) % TAU;
    r.intakeDeploy = this.lastDeploy = this.ext / BOX.ext;
  }

  // the shot: the table's (or a pass's) over the carpet, less the turret's own velocity LATENCY
  // ahead; its heading (aimPsi, which the robot turns the turret to), speed and elevation, and the
  // flywheel speed and hood that make it
  _aim(dt, feed) {
    const r = this.r, b = r.body, v = b.linvel(), w = b.angvel().y;
    const k = 1 - Math.pow(0.5, dt / 0.02);
    if (!this.va) this.va = { vx: v.x, vz: v.z, ax: 0, az: 0 };
    this.va.ax += k * ((v.x - this.va.vx) / dt - this.va.ax); this.va.az += k * ((v.z - this.va.vz) / dt - this.va.az);
    this.va.vx = v.x; this.va.vz = v.z;
    const L = LATENCY, c = b.translation(), yaw = r.yaw + w * L;
    const px = c.x + v.x * L + 0.5 * this.va.ax * L * L, pz = c.z + v.z * L + 0.5 * this.va.az * L * L;
    const ox = AXIS[0] * Math.cos(yaw) + AXIS[1] * Math.sin(yaw), oz = -AXIS[0] * Math.sin(yaw) + AXIS[1] * Math.cos(yaw);
    const tx = px + ox, tz = pz + oz;
    const vtx = v.x + this.va.ax * L + w * oz, vtz = v.z + this.va.az * L - w * ox;
    const tgt = feed ? (r.shot?.target ?? Field.hubCenter(r.alliance)) : Field.hubCenter(r.alliance);
    const dx = tgt.x - tx, dz = tgt.z - tz, d = Math.hypot(dx, dz);
    let v0, el;
    if (feed) {
      // a pass: the hood at PASS_HOOD, the speed that lands it at the pass point
      el = fitOf('el', 450, PASS_HOOD);
      const y0 = fitOf('y', 450, PASS_HOOD), x0 = fitOf('x', 450, PASS_HOOD);
      const reach = (s) => (sim2D(y0, s, el * DEG, R)?.d ?? 0) + x0;
      v0 = reach(fitOf('v', RADS[1], PASS_HOOD)) < d ? fitOf('v', RADS[1], PASS_HOOD) : bisect(reach, 3, 12, d);
    } else ({ v: v0, el } = launchFor(d));
    const ux = dx / (d || 1), uz = dz / (d || 1);
    const sx = v0 * Math.cos(el * DEG) * ux - vtx, sz = v0 * Math.cos(el * DEG) * uz - vtz, sy = v0 * Math.sin(el * DEG);
    const vr = Math.hypot(sx, sy, sz);
    this.aimDist = d;
    this.aimPsi = Math.atan2(-sz, sx);
    this.launchAngle = vr > 0 ? Math.asin(sy / vr) / DEG : 90;
    this.launchV = vr;
    return shooterFor(vr, this.launchAngle);
  }

  postStep(dt, t, { on, feeding, outtake, setpoint, mode }) {
    const r = this.r, T = this.t, S = SHOOTER;
    this.account(t, mode);
    // ---- the intake
    const out = this.ext > BOX.ext - 0.02;
    const intaking = on && !!r.cmd.intake && out, unjam = on && !!outtake;
    T.primary.cmd = T.bar.cmd = !on ? 0 : unjam ? -0.6 : intaking ? 1 : 0;
    // ---- the shot
    const feed = mode === 'pass';
    const launch = on && feeding && setpoint > 0 && !!r.shot && !unjam;
    const want = this._aim(dt, feed);
    this.launching = launch;
    if (launch) { this.targetRad = want.rad; this.hoodGoal = clamp(want.hood, S.hoodMin, S.hoodMax); }
    else { this.targetRad = 0; this.hoodGoal = S.hoodStow; }
    // (their velocity loop: the X44s' own feedforward, 12 V over their free speed, and a P term)
    const m = T.launcher.motor, kV = 12 / (m.m.free / TAU);
    T.launcher.cmd = this.targetRad > 0 ? velocityDuty(m, T.fly.omega, this.targetRad, kV, 0.3) : null;
    // ready: the flywheel within 2% (a pass: 6%), the hood within 1.5 deg, the turret on (their
    // tolerances aren't public: estimates)
    const atV = this.targetRad > 0 && Math.abs(T.fly.omega - this.targetRad) < (feed ? 0.06 : 0.02) * this.targetRad;
    const atHood = Math.abs(this.hoodDeg - this.hoodGoal) < 1.5;
    const tol = feed ? 6 * DEG : Math.max(0.8 * DEG, Math.atan2(0.15, this.aimDist || 1));
    const atYaw = Math.abs(r.aimErr ?? Infinity) < tol;
    this.ready = launch && atV && atHood && atYaw;
    // feeding: once ready, it keeps feeding through the flywheel's dip as each shot leaves (2%
    // is less than a shot takes out of it) and the turret's small swings, up to 0.3 s not ready
    // (their code isn't public: an estimate). Dropping out at each dip turned the rotor back to its
    // slow agitation and braked it hard at 80 A between shots
    this.notReady = this.ready ? 0 : (this.notReady ?? 1) + dt;
    this.feedingNow = on && launch && (this.ready || (this.feedingNow && this.notReady < 0.3));
    this._rotor(dt, on, unjam);
    // what the robot's aiming and drawing see
    r.hoodDeg = hoodElevation(this.hoodDeg);
    r.flywheel = this.exitSpeed();
    r.ready = this.ready;
    r.feeding = this.feedingNow ? 1 : 0;
    r.intakeSpeed = T.primary.cmd > 0 ? 1 : T.primary.cmd < 0 ? -1 : 0;
  }

  // the Dye Rotor and its rollers: feeding at their 2.25 rps with the TTB wheels on, otherwise
  // turning back slowly; a stall backs either out
  _rotor(dt, on, unjam) {
    const T = this.t, rotor = T.rotor, rollers = T.rollers;
    if (!on) { rotorLoop(rotor, null); rotor.cmd = 0; rollers.cmd = 0; return; }
    // (stalls: the rotor over 70 A under 2 rad/s, the rollers over 50 A a motor under a fifth of
    // their speed, for 0.3 s)
    if (this.feedingNow && Math.abs(rotor.motor.current) > 70 && Math.abs(rotor.omega) < 2) this.rotorJam += dt; else this.rotorJam = 0;
    if (this.rotorJam > 0.3) { this.rotorBack = 0.25; this.rotorJam = 0; this.r.stats.unjams = (this.r.stats.unjams || 0) + 1; }
    const free = rollers.motor.freeSpeed;
    if (this.feedingNow && Math.abs(rollers.motor.current) / rollers.motor.count > 50 && rollers.omega < 0.2 * free) this.rollerJam += dt; else this.rollerJam = 0;
    if (this.rollerJam > 0.3) { this.rollerBack = 0.15; this.rollerJam = 0; this.r.stats.unjams = (this.r.stats.unjams || 0) + 1; }
    this.rotorBack = Math.max(0, this.rotorBack - dt); this.rollerBack = Math.max(0, this.rollerBack - dt);
    if (unjam) { rotorLoop(rotor, -0.5); rollers.cmd = -0.5; return; }
    rotorLoop(rotor, this.rotorBack > 0 ? -1 : this.feedingNow ? ROTOR_RPS : AGITATE_RPS);
    rollers.cmd = this.rollerBack > 0 ? -0.6 : this.feedingNow ? 1 : null;
  }

  // how far the box is out (m), for the hopper's collider and drawing it
  extNow() { return this.ext; }
  // the rotor's turn (rad, clockwise from above) and the hood's (deg round the flywheel's axle)
  rotorTurn() { return this.phi; }
  hoodTurn() { return (this.hoodDeg - SHOOTER.hoodStow) * DEG; }
  // about how fast a FUEL leaves at the flywheel's speed now (m/s; the fits)
  exitSpeed() { return this.t.fly.omega > 100 ? fitOf('v', clamp(this.t.fly.omega, RADS[0], RADS[1]), this.hoodDeg) : 0; }
  // the top of its load (for the net and the TRENCH)
  loadTop() { let top = 0; for (const b of this.held) { const y = this._local(b.pos)[1]; if (y + R > top) top = y + R; } return top; }
  // how many it holds (tools/mechlab4414.mjs hold)
  capacity() { return 62; }
}
