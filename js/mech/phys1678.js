// 1678's Limestone in play, built of its physical mechanisms (js/mech/r1678.js; the shared part:
// js/mech/physical.js). Its controls are its code's (Superstructure, Targeting, the shot
// verifier and the subsystems' setpoints); every roller coasts when it's set to idle, as theirs do.
//   intake (held): the arm down (their position loop on the pivot's X60; the FUEL in its way push
//     back on it), its roller at 12 V, the feeder idle; once the hopper's CANrange has seen FUEL
//     for 0.75 s and the tunnel's sees none, it prefills the tunnel once (tunnel 250 RPM, floor and
//     feeder 3 V) until the tunnel's CANrange sees one. Let go, the roller idles.
//   shoot (held): the drum's velocity loop at their regression's RPM for the distance to the
//     target (the HUB, or their ferry points), the hood to its angle (stowed under a TRENCH).
//     Once aimed (1.3 deg at the HUB, 4 for a ferry), within 80 RPM (150 ferrying) with the hood
//     there, and (at the HUB) going 1 m/s or less, it fires: floor and feeder at 12 V, the tunnel
//     at the regression's RPM, while it stays aimed (40 ms grace); from the far end of the field,
//     floor and feeder at 3 V and the tunnel held to 5 V.
//   let go: the floor and tunnel idle and the drum winds down to 2100 RPM and holds there; not
//     intaking, the feeder runs back at 4 V (holding FUEL off the drum) and the arm goes limp.
//   the hood stows (17.5 deg) whenever it isn't shooting.
// And what a driver of it does: contractHopper when FUEL are left out over the intake while it
// fires (not while it intakes), and the lid (their climber) up while it shoots, down for a TRENCH.
// The extension rides out and in with the intake arm (a pin in a slot); the lid can't
// come down onto FUEL.
import { Mechanism } from './mechanism.js';
import { PhysicalRobot } from './physical.js';
import {
  build1678, wheels1678, pathSurfaces1678, placeIntake, placeHood, placeNet, intakeRot, slotExt, slotRate, SLOT, hoodRot, hoodElevation, shotFor,
  INTAKE, SHOOTER, TUNNEL, RANGES, PRELOAD, PIVOT, DRUM,
} from './r1678.js';
import PATH from './path1678.js';
import { Field } from '../field.js';
import { FUEL, FIELD_L, HALF_L, fw } from '../constants.js';
import { underTrench, TRENCH_OPENINGS } from '../nav.js';

const DEG = Math.PI / 180, R = FUEL.radius, FT = 0.3048;
const HALF = 0.343; // its frame (27in square)
// their intake pivot's position loop (Slot2: V per degree of arm, kS, kG) and hood's profile
const PIVOT_PID = { kP: 125 / 360, kS: 0.28, kG: 0.3 };
const HOOD_MM = { v: 300, a: 2500 };
// ShotVerifier and Superstructure's tolerances
const TOL = { hubAim: 1.3 * DEG, ferryAim: 4 * DEG, rpm: 80, ferryRpm: 150, hood: 0.5, speed: 1.0 };
// m/s, the climber moving the lid: an X60 through 33:1 on a 1.757in pulley is ~0.42 m/s free; up
// it's held to 30 A (their LOW_CURRENT, lifting the lid), down it gets 90 A
const LID_SPEED = { up: 0.3, down: 0.4 };

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
const clamp = (x, a, b) => Math.max(a, Math.min(b, x));

// the intake's pivot: their position loop toward goal (all 12 V for any real move), the X60 doing
// what it can against the arm's weight and the FUEL in its way; goal null: limp (brake)
export function servoArm(T, deg, goal, dt) {
  const a = (deg + INTAKE.comDeg) * DEG;
  if (goal !== null) {
    const v = PIVOT_PID.kP * (goal - deg) + PIVOT_PID.kG * Math.cos(a) + PIVOT_PID.kS * Math.sign(goal - deg);
    T.pivot.cmd = clamp(v / 12, -1, 1);
  } else T.pivot.cmd = 0;
  T.pivot.impulse(-INTAKE.mass * 9.81 * INTAKE.com * Math.cos(a) * dt);
}
// after the mechanisms' step: the arm turns as its shaft did (to its hardstops: down on the
// bumper, up folded)
export function turnArm(T, deg, dt) {
  deg += (T.pivot.omega * dt) / DEG;
  if (deg < INTAKE.down) { deg = INTAKE.down; if (T.pivot._omega < 0) T.pivot._omega = 0; }
  if (deg > INTAKE.stowed) { deg = INTAKE.stowed; if (T.pivot._omega > 0) T.pivot._omega = 0; }
  return deg;
}
// their contractHopper(): the intake partway in and out again, sweeping what's out over it back
// into the hopper (its roller at 2 V): to 80 deg, 25, 80, half a second, 50, then down; each step
// once it's within 12 deg (their setpointCommandWithWait)
export const CONTRACT = [80, 25, 80, 'wait', 50, null];

export class Physical1678 extends PhysicalRobot {
  constructor(robot) {
    const t = build1678(), w = wheels1678(t), s = pathSurfaces1678(PATH, t);
    const pathWheels = [...w.floor, ...w.tunnel, ...w.intake];
    super(robot, {
      path: new Mechanism({ body: robot.body, substeps: 6, shafts: [...new Set(pathWheels.map((x) => x.shaft)), t.pivot], wheels: pathWheels, surfaces: Object.values(s) }),
      shooter: new Mechanism({ body: robot.body, substeps: 6, sub: 8, shafts: [t.drum, t.hood], wheels: [w.drum, ...w.hood] }),
      preload: PRELOAD,
    });
    Object.assign(this, { t, w, s });
    // the extension, driven by the arm through its pin and slot (FUEL in it push back on the arm)
    s.slide.onSlide(t.pivot, [1, 0, 0]);
    this.ownsIntake = true;
    this.ownsHood = true;
    this.reset();
  }

  reset() {
    super.reset();
    // folded up at the start (their starting configuration; the code homes it there)
    this.armDeg = INTAKE.stowed;
    this.armGoal = INTAKE.stowed;
    this.armHold = true;
    this.hoodDeg = SHOOTER.hoodMin; this.hoodSet = [SHOOTER.hoodMin, 0]; this.hoodGoal = SHOOTER.hoodMin;
    this.lift = 0;
    // their subsystems' setpoints (null: neutral, coasting)
    this.set = { intake: null, floor: null, tunnel: null, feeder: null, shooter: null };
    this.wasIntaking = false; this.wasShooting = false;
    this.prefill = 'none'; // armed (waiting for the hopper's CANrange), running, done
    this.firing = false; this.verifiedT = 0; this.firingT = 0;
    this.seq = null; this.contractCool = 0; this.tucked = true; this.nearTrench = false;
    this.hopperT = 0; this.tunnelT = 0;
    for (const tr of [this.t.intake, this.t.floor, this.t.tunnel, this.t.feederT, this.t.shooter]) { tr.omega = 0; tr.cmd = null; }
    for (const s of this.t.idlers) s.omega = 0;
    this.t.pivot._omega = 0; this.t.pivot.cmd = 0;
  }

  where(x, y, z) {
    const inFrame = Math.abs(x) < HALF && Math.abs(z) < HALF && y > 0.105 && y < 0.8;
    // out front: the intake and the extension over it
    const inFront = x >= HALF && x < 0.72 && Math.abs(z) < HALF && y < 0.62;
    return {
      own: inFrame || inFront,
      held: inFrame || (inFront && y > 0.2),
      near: x > -HALF - 0.1 && x < 0.76 && Math.abs(z) < HALF + 0.08 && y < 0.85,
    };
  }
  shooterBalls(near) { return near.filter((b) => { const [x, y] = this._local(b.pos); return y > 0.38 && x < -0.03; }); }
  // the top of the FUEL it holds (robot frame), for the lid
  loadTop() {
    let top = 0;
    for (const b of this.held) { const [x, y] = this._local(b.pos); if (x > -0.06) top = Math.max(top, y + R); }
    return top;
  }

  preStep(dt, t) {
    const r = this.r, T = this.t, on = r.enabled;
    servoArm(T, this.armDeg, on && this.armHold ? this.armGoal : null, dt);
    // the hood: a profile to its goal (an X44 through 126:1; nothing pushes it hard)
    this.hoodSet = profile(...this.hoodSet, this.hoodGoal, HOOD_MM.v, HOOD_MM.a, dt);
    this.hoodDeg = this.hoodSet[0];
    // the lid on the climber: up (once the intake's been down), down for a TRENCH, but never onto
    // the FUEL under it
    this.lift += clamp(this.liftScale() * (r.cfg.bay.lift?.h ?? 0) - this.lift, -LID_SPEED.down * dt, LID_SPEED.up * dt);
    // pose the moving parts
    // the extension where the arm's pin puts it, and how it moves as the arm turns (its mass felt
    // through the slot at the arm)
    const ext = slotExt(this.armDeg), k = slotRate(this.armDeg);
    this.s.slide.slideK = k;
    T.pivot.inertia = INTAKE.J + SLOT.mass * k * k;
    this.s.intake.setPose([PIVOT[0], PIVOT[1], 0], 0, intakeRot(this.armDeg));
    placeIntake(this.w.intake, this.armDeg);
    this.s.hood.setPose([DRUM[0], DRUM[1], 0], 0, hoodRot(this.hoodDeg));
    placeHood(this.w.hood, this.hoodDeg);
    this.s.slide.setPose([ext, 0, 0]);
    this.s.lid.setPose([0, this.lift, 0]);
    placeNet(this.s.net, ext, this.lift);
    this.stepMechanisms(dt, t);
    this.armDeg = turnArm(T, this.armDeg, dt);
    r.intakeDeploy = clamp((INTAKE.stowed - this.armDeg) / (INTAKE.stowed - INTAKE.down), 0, 1);
  }

  // their CANranges: the tunnel's beam across it at its height; the hopper's, forward along the
  // hopper from its back, 27 deg wide
  _tunnelRange() {
    const { at: [x0, y0], z, reach } = RANGES.tunnel;
    return this.held.some((b) => { const [x, y, bz] = this._local(b.pos); return Math.hypot(x - x0, y - y0) < R * 0.8 && Math.abs(bz - z) < reach + R; });
  }
  _hopperRange() {
    const { at: [x0, y0], z, reach, half } = RANGES.hopper, tn = Math.tan(half * DEG);
    return this.held.some((b) => {
      const [x, y, bz] = this._local(b.pos), dx = x - x0;
      return dx > 0 && dx < reach + R && Math.abs(y - y0) < R * 0.8 && Math.abs(bz - z) < R + dx * tn;
    });
  }

  // their Targeting: the HUB in the ALLIANCE ZONE; otherwise a ferry to their points, by how far it
  // is from its own ALLIANCE WALL (alliance ferry, far alliance ferry, or back to the NEUTRAL ZONE
  // from deep in the other alliance's half)
  _ferry() {
    const r = this.r, fromWall = r.alliance === 'blue' ? r.pos.x + HALF_L : HALF_L - r.pos.x;
    const [kind, fx, left, right] = fromWall <= FIELD_L - 13 * FT ? ['ferry', 0.5, 15.5 * FT, 8 * FT]
      : fromWall <= FIELD_L - 6.5 * FT ? ['far', 0.5, 20 * FT, 5 * FT] : ['ferry', 5.5, 18 * FT, 9 * FT];
    const L = fw(fx, left, r.alliance), Rt = fw(fx, right, r.alliance);
    return { kind, target: Math.sign(r.pos.z) === Math.sign(L.z) ? L : Rt };
  }
  passTarget() { return this._ferry().target; }

  postStep(dt, t, { on, feeding, outtake, setpoint, mode }) {
    const r = this.r, T = this.t, S = this.set;
    this.account(t, mode);
    // their CANranges, debounced (the hopper's 0.75 s, the tunnel's 60 ms)
    this.hopperT = this._hopperRange() ? this.hopperT + dt : 0;
    this.tunnelT = this._tunnelRange() ? this.tunnelT + dt : 0;
    const hopperFull = this.hopperT >= 0.75, tunnelFull = this.tunnelT >= 0.06;
    const intaking = on && !!r.cmd.intake, shooting = on && feeding && setpoint > 0 && !!r.shot;

    // ---- intake (left trigger): down, roller in, feeder idle; then the prefill, once. (Folded at
    // the start; down from the first intake on: their code leaves it there)
    // ---- the lid on the climber (expandHopper): up, except driving toward or under a TRENCH,
    // where it comes down (their trenchContract's climber) as far as the FUEL under it let it
    this.nearTrench = on && this._nearingTrench();
    // (shooting with the intake still folded, as at the start: down, so FUEL left up on it fall in)
    if (this.tucked && !this.seq && shooting && !intaking) { this.tucked = false; this.armGoal = INTAKE.down; this.armHold = true; }
    if (intaking && this.seq?.kind === 'contract') { this.seq = null; this.contractCool = 1; this.armGoal = INTAKE.down; S.intake = 1; }
    if (intaking && !this.wasIntaking) {
      this.tucked = false; this.seq = null;
      this.armGoal = INTAKE.down; this.armHold = true;
      S.intake = 1; S.feeder = null;
      this.prefill = 'armed';
    }
    if (!intaking && this.wasIntaking) { S.intake = null; if (this.prefill === 'running') this._prefillStop(); this.prefill = 'none'; }
    if (this.prefill === 'armed' && hopperFull && !tunnelFull) {
      this.prefill = 'running';
      S.tunnel = { rpm: TUNNEL.preload }; S.floor = 3 / 12; S.feeder = 3 / 12;
    }
    if (this.prefill === 'running' && tunnelFull) { this._prefillStop(); this.prefill = 'done'; }

    // ---- shoot (right trigger)
    const pass = mode === 'pass';
    const ferry = pass ? this._ferry() : null;
    let want = null;
    if (shooting) {
      const tgt = pass ? ferry.target : Field.hubCenter(r.alliance);
      const dist = Math.hypot(tgt.x - r.pos.x, tgt.z - r.pos.z);
      want = shotFor(pass ? ferry.kind : 'hub', dist);
      S.shooter = want.rpm;
      if (!this.wasShooting) { S.feeder = null; this.firing = false; }
      this.hoodGoal = underTrench(r.pos.x, r.pos.z, 0) ? SHOOTER.hoodMin : want.hood;
    }
    if (!shooting && this.wasShooting) {
      // stopShooting (still intaking) or idle
      S.floor = null; S.tunnel = null; S.shooter = SHOOTER.slow;
      if (!intaking) { S.feeder = -4 / 12; S.intake = null; this.armHold = false; } else S.feeder = null;
      this.firing = false;
    }
    if (!shooting) this.hoodGoal = SHOOTER.hoodMin;
    if (!on) { S.intake = S.floor = S.tunnel = S.feeder = S.shooter = null; }
    // the drum's speed (RPM) and how its loop's doing
    const rpm = (T.drum.omega / (2 * Math.PI)) * 60;
    const v = r.body.linvel(), speed = Math.hypot(v.x, v.z);
    const aim = Math.abs(r.aimErr ?? Infinity);
    const aimed = aim <= (pass ? TOL.ferryAim : TOL.hubAim);
    // (their 40 ms falling debounce on the shot verifier)
    this.verifiedT = aimed ? 0.04 : Math.max(0, this.verifiedT - dt);
    const verified = this.verifiedT > 0;
    const spunUp = !!want && Math.abs(rpm - want.rpm) <= (pass ? TOL.ferryRpm : TOL.rpm) && Math.abs(this.hoodDeg - this.hoodGoal) <= TOL.hood;
    const ready = spunUp && aimed && (pass || speed <= TOL.speed);
    if (shooting && !this.firing && ready && r.passClear !== false) this.firing = true;
    const far = pass && ferry.kind === 'far';
    if (shooting && this.firing) {
      S.floor = verified ? (far ? 3 / 12 : 1) : null;
      S.feeder = verified ? (far ? 3 / 12 : 1) : null;
      S.tunnel = verified ? { rpm: want.tunnel, max: far ? 5 : 12 } : null;
    }
    // contractHopper (the driver's, or their autos' while they fire): once it's been firing a
    // second and FUEL are left out over the intake, where nothing carries them back, and it isn't
    // intaking
    this.firingT = shooting && this.firing ? (this.firingT || 0) + dt : 0;
    this.contractCool = Math.max(0, (this.contractCool || 0) - dt);
    // (never while it's intaking: folding the intake then stops it taking FUEL in)
    if (!this.seq && !this.tucked && on && !intaking && this.firingT > 1 && !this.contractCool && this.held.some((b) => this._local(b.pos)[0] > HALF)) this.seq = { kind: 'contract', steps: CONTRACT, i: 0, t: 0 };
    if (this.seq) {
      const q = this.seq, step = q.steps[q.i];
      q.t += dt;
      S.intake = 2 / 12; this.armHold = true;
      this.armGoal = step === 'wait' ? this.armGoal : step ?? INTAKE.down;
      const done = step === 'wait' ? q.t >= 0.5 : Math.abs(this.armDeg - this.armGoal) < 12;
      if (!on) this.seq = null;
      else if (done && ++q.i >= q.steps.length) {
        this.seq = null;
        if (q.kind === 'contract') this.contractCool = 1;
        S.intake = intaking && q.kind === 'contract' ? 1 : null;
      } else if (done) q.t = 0;
    }
    if (on && outtake) { S.intake = -1; S.floor = -1; S.feeder = -1; }

    // ---- the motors
    T.intake.cmd = S.intake;
    T.floor.cmd = S.floor;
    T.feederT.cmd = S.feeder;
    // the tunnel's velocity loop on its top roller (no reverse voltage)
    if (S.tunnel) {
      const rps = T.back[T.back.length - 1].omega / (2 * Math.PI), tgt = S.tunnel.rpm / 60;
      const volts = TUNNEL.kV * tgt + TUNNEL.kP * (tgt - rps) + TUNNEL.kS;
      T.tunnel.cmd = clamp(volts, 0, S.tunnel.max ?? 12) / 12;
    } else T.tunnel.cmd = null;
    // the drum's velocity loop (no reverse voltage: it coasts down)
    if (S.shooter) {
      const rps = T.drum.omega / (2 * Math.PI), tgt = S.shooter / 60;
      const volts = SHOOTER.kV * tgt + SHOOTER.kP * (tgt - rps) + SHOOTER.kS;
      T.shooter.cmd = clamp(volts / 12, 0, 1);
    } else T.shooter.cmd = null;

    this.wasIntaking = intaking; this.wasShooting = shooting;
    // what the robot's aiming and drawing see
    r.hoodDeg = hoodElevation(this.hoodDeg);
    r.flywheel = this.exitSpeed();
    r.ready = ready || (shooting && this.firing && verified);
    r.feeding = shooting && this.firing && verified ? 1 : 0;
    r.intakeSpeed = (S.intake ?? 0) > 0.5 ? 1 : (S.intake ?? 0) < 0 ? -1 : 0;
  }
  // the arm's turn from the CAD's pose (rad), for drawing it
  armTurn() { return intakeRot(this.armDeg); }
  // how far out the extension is (m): where the arm's pin puts it
  extNow() { return slotExt(this.armDeg); }
  // how far up the lid is now (0-1 of its travel), as the climber has it
  liftNow() { const h = this.r.cfg.bay.lift?.h; return h ? clamp(this.lift / h, 0, 1) : 0; }
  // how far up the lid wants to be (0-1 of its travel): up while it intakes and while it shoots
  // (the team) and in AUTO (their autos expand it early in the first path), down for a TRENCH;
  // never down onto FUEL piled under it, but the climber holds it where it is: FUEL can't push
  // it up
  liftScale() {
    const r = this.r, bay = r.cfg.bay, h = bay.lift?.h;
    if (!h) return 0;
    const up = !(this.tucked || this.nearTrench) && (this.wasIntaking || this.wasShooting || r.match?.isAuto);
    const now = this.liftNow(), onLoad = clamp((this.loadTop() - 0.01 - bay.top) / h, 0, 1);
    return up ? 1 : Math.min(now, onLoad);
  }
  // their isNearingTrench: under one, or within 1.5 m of one and driving toward it
  _nearingTrench() {
    const r = this.r, v = r.body.linvel();
    return TRENCH_OPENINGS.some((o) => {
      const dx = o.x - r.pos.x, dz = o.z - r.pos.z;
      if (Math.abs(dx) < o.hx + r.halfL && Math.abs(dz) < o.hz + r.halfL) return true;
      return Math.abs(dx) < o.hx + r.halfL + 1.5 && Math.abs(dz) < o.hz + r.halfL + 0.3 && dx * v.x > 0 && Math.abs(v.x) > 0.3;
    });
  }
  _prefillStop() { const S = this.set; S.tunnel = null; S.floor = null; S.feeder = null; }

  // about how fast a FUEL leaves at the drum's speed now (m/s; tools/mechlab1678.mjs shooter)
  exitSpeed() { return EXIT_PER_DRUM_SURFACE * Math.abs(this.t.drum.omega) * SHOOTER.drum.d / 2; }

  // how many it holds: what it gets to in matches, intaking with the extension out and the lid up
  // (a layer at a time it holds 62: tools/mechlab1678.mjs hold; the last few in from the front
  // take far longer than the rest)
  capacity() { return 52; }
}
// a FUEL's exit speed over the drum's surface speed (tools/mechlab1678.mjs shooter measures it)
export const EXIT_PER_DRUM_SURFACE = 0.71;
