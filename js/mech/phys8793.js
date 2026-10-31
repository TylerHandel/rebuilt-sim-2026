// 8793 in play, built of its physical mechanisms (js/mech/r8793.js): the FUEL it takes in stay
// bodies on the FIELD the whole way, moved only by its rollers, wheels and the surfaces of its CAD
// (a Mechanism on the robot's own body, so it drives off with them, and every push on them pushes
// back on it), until its shooter flings them. Its controls are its code's (ShooterMode): the
// rollers and the vertical rollers always running at their duty cycles, with its automatic jam
// recovery; the turret indexer while the shot is on; the flywheel on its velocity loop toward
// the motor speed that gives the exit speed the aim asks for (SHOT_TABLE); and the intake arm on
// its pivot's motors under their position loop, down when the game wants it down (r.intakeWant),
// where the FUEL squeezed under its rollers can lift it (js/mech/r8793.js PIVOT).
import { Mechanism } from './mechanism.js';
import { PhysicalRobot } from './physical.js';
import {
  build8793, intake8793, conveyorWheels, armWheels, placeArm, indexerWheels, shooterWheels, placeShooter,
  flywheelDuty, exitSpeed, rpsFor, pathSurfaces, armState, servoArm, turnArm, ARM, DUTY, DUTY_INDEXER, PRELOAD, PIVOT, SHOOTER,
} from './r8793.js';
import PATH from './path8793.js';
import * as THREE from 'three';
import { TRENCH } from '../constants.js';
import { TRENCH_ARMS } from '../nav.js';
import { drive8793 } from './turretDrive.js';

const DEG = Math.PI / 180;
const _q = new THREE.Quaternion(), _v = new THREE.Vector3();
const HALF = 0.349;   // the frame's half length and width (27.5in)
const TURRET_X = -0.127;
// their jam recovery (GroundIntakeConstants): the roller motor over 55 A stator and under 10
// rotations/s for 0.125 s reverses at full for 0.125 s
export const JAM = { amps: 55, rps: 10, debounce: 0.125, reverse: 0.125 };
// Stalled while nothing is being fed is the path full, not a jam: the rollers stop and hold what
// they have for HOLD_FULL s instead of reversing and spitting FUEL back out, then try again (not in
// their code, which reverses either way; HOLDFULL=0 for theirs)
const HOLD_FULL = Number(globalThis.process?.env?.HOLDFULL ?? 0.5);
// As a shot stops, their unjam (the whole path backward 0.125 s) runs once, so the FUEL left in the
// rails aren't wedged under the indexer for the next one (not their code; the user's idea:
// STOPUNJAM=0 for none, ix for the indexer alone)
// In play FUEL leave ~5% faster than SHOT_TABLE gives for the flywheel's speed as the aim reads it
// (a match's every shot, its exit against what was expected: median 1.05); scaled so the aim expects
// what they do (EXITK=1 for the table as it is)
const EXIT_K = Number(globalThis.process?.env?.EXITK ?? 1.05);
// The feed's push, as their AimSolver takes it off the shot (TurretConstants FEEDER_FORWARD_PUSH_MPS,
// FEEDER_BACKWARD_PUSH_AT_90_MPS; feederPushForwardMps): the indexer throws each FUEL rearward up
// the turret's bore, and it leaves carrying some of that, along the chassis: fwd cos^2 - back sin^2
// of the turret's angle on it (m/s forward). Theirs, 0.45 and 0.35, are untuned ("TODO: tune
// empirically from test shots"); these are the sim's FUEL in its matches (every HUB shot's heading
// off the aim against the turret's angle: 2-3 deg toward the rear with it out to the side, the
// elevation a degree flatter with it facing back, steeper facing forward). FEEDPUSH=fwd,back
const FEED_PUSH = (globalThis.process?.env?.FEEDPUSH ?? '-0.15,0.2').split(',').map(Number);
const HOLD_SPEED = Number(globalThis.process?.env?.HOLDSPEED ?? 0);
const _su = globalThis.process?.env?.STOPUNJAM ?? 'all', STOP_UNJAM = _su === '0' ? '' : _su;

// Points to test the intake arm with against a TRENCH arm (trenchArm): its side outline (hull,
// deployed, round the pivot) every 2 cm, and rings of it two thirds and a third of the way in to
// its middle; at five places across its width
function armSamples(hull, width) {
  const ring = [];
  hull.forEach((a, i) => {
    const b = hull[(i + 1) % hull.length], n = Math.max(1, Math.ceil(Math.hypot(b[0] - a[0], b[1] - a[1]) / 0.02));
    for (let k = 0; k < n; k++) ring.push([a[0] + ((b[0] - a[0]) * k) / n, a[1] + ((b[1] - a[1]) * k) / n]);
  });
  const cx = hull.reduce((s, p) => s + p[0], 0) / hull.length, cy = hull.reduce((s, p) => s + p[1], 0) / hull.length;
  const pts = [...ring];
  for (const f of [2 / 3, 1 / 3]) for (const [x, y] of ring) pts.push([cx + (x - cx) * f, cy + (y - cy) * f]);
  return { pts, zs: [-0.5, -0.25, 0, 0.25, 0.5].map((f) => f * width) };
}

export class Physical8793 extends PhysicalRobot {
  constructor(robot) {
    const t = build8793(), s = pathSurfaces(PATH), arm = armWheels(t);
    // (the arm's side plates swing on the pivot with its rollers)
    s.arm.onShaft(t.pivot, [ARM.pivot[0], ARM.pivot[1], 0]);
    const wheels = [...conveyorWheels(t), ...indexerWheels(t), ...arm];
    const sw = shooterWheels(t, 60);
    super(robot, {
      path: new Mechanism({ body: robot.body, substeps: 6, shafts: [...new Set(wheels.map((w) => w.shaft)), t.pivot], wheels, surfaces: Object.values(s) }),
      shooter: new Mechanism({ body: robot.body, substeps: 6, sub: 8, shafts: [t.large, t.small], wheels: sw }),
      preload: PRELOAD,
    });
    Object.assign(this, { t, s, arm, shooterWheels: sw });
    this.autoShuttle = true; // (its ShooterMode shuttles whatever it picks up, out of its zone)
    this.ownsArm = true;     // (the game says whether it wants it down: r.intakeWant)
    this.reset();
  }

  reset() {
    super.reset();
    this.jamT = 0; this.unjamT = 0; this.fullT = 0; this.ixBackT = 0;
    this.targetRps = 0;
    this.feed = false;
    for (const tr of [this.t.horizontal, this.t.indexer, this.t.shooter]) { tr.omega = 0; tr.cmd = 0; }
    // stowed, on its upper stop (the robot starts so)
    this.armSt = armState(this.r.cfg.intake.fold.stowDeg * DEG);
    this.t.pivot._omega = 0; this.t.pivot.cmd = 0;
    this.deploy = 0;
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
    const r = this.r, st = this.armSt;
    // (put somewhere by the game, not by its motors: a staged scene. It's there, at rest)
    if (Math.abs(r.intakeDeploy - this.deploy) > 1e-6) { st.roll = (1 - r.intakeDeploy) * st.stow; st.set = null; this.t.pivot._omega = 0; }
    // the arm: their loop toward down or up (whichever the game wants: deploying and stowing
    // take their profile's 0.9 s), its weight, its stops; posed where it is
    servoArm(this.t, st, !!r.intakeWant, dt, r.enabled);
    // (a TRENCH arm in its way: pushing it down, or up onto its stop)
    const contact = this.trenchArm(dt);
    const w0 = this.t.pivot.omega;
    this.s.arm.setPose([ARM.pivot[0], ARM.pivot[1], 0], 0, st.roll);
    placeArm(this.arm, st.roll);
    this.s.turret.setPose([TURRET_X, 0, 0], r.turretYaw, 0);
    placeShooter(this.shooterWheels, r.hoodDeg, r.turretYaw, TURRET_X);
    this.stepMechanisms(dt, t);
    if (contact) this.trenchPush(contact, w0, dt);
    // the arm turns as its shaft did (lifted by the FUEL under it, or swinging to stow, or pushed
    // by a TRENCH arm)
    turnArm(this.t, st, dt);
    // what the game sees: down (1) within their isAtPosition() tolerance of DOWN, stowed (0)
    // within it of the upper stop, in between as it swings
    const tol = PIVOT.tol * 2 * Math.PI;
    r.intakeDeploy = this.deploy = st.roll <= tol ? 1 : st.roll >= st.stow - tol ? 0 : 1 - st.roll / st.stow;
  }
  // The TRENCH arm against the intake arm. Up, the arm stands 0.69 m tall (from 59 deg up from
  // down, over the TRENCH's 0.565 m), but on 18:1 its pivot is backdrivable: driven under a TRENCH
  // arm backward, the TRENCH arm meets the back of the folded arm above the pivot and pushes it down
  // (forward, round its pivot) as the robot goes on under, against their loop and the arm's
  // inertia; and the robot feels that push (trenchPush). The arm's collider passes through the
  // TRENCH arms (robotConfigs fold.yields): here, each step, its side outline (and inside it, across
  // its width) as it is now is tested against the TRENCH arms near it; where it's in one, the face
  // it's least far into is what pushes it (a side face as the robot drives under, the underside as
  // the arm swings up into it), and the push's torque about the pivot says which way the arm goes.
  // The pivot is held this step to turn at least as fast as takes it out of the TRENCH arm (a
  // moving stop: Shaft.stop). Pushed up onto its upper stop (driving in forward with it stowed: the
  // TRENCH arm meets its front face), it can't give, and its collider meets the TRENCH arm: the
  // robot's stopped as by anything solid. Returns the contact, or null.
  trenchArm(dt) {
    const r = this.r, st = this.armSt, pivot = this.t.pivot, fold = r.cfg.intake.fold;
    const beams = TRENCH_ARMS.filter((a) => Math.abs(a.x - r.pos.x) < a.hx + 1 && Math.abs(a.z - r.pos.z) < a.hz + 1);
    const was = this.trench?.hard;
    this.trench = { hard: false, F: 0 };
    if (!beams.length) { if (was) r.armCollider?.setCollisionGroups(r.armGroups); this.prevPose = null; return null; }
    if (!this.armPts) this.armPts = armSamples(fold.hull, r.cfg.intake.width);
    const [px, py] = fold.pivot;
    // (well under them: nothing to test)
    const top = py + Math.max(...fold.hull.map(([x, y]) => x * Math.sin(st.roll) + y * Math.cos(st.roll)));
    if (!was && r.pos.y + top < TRENCH.clearHeight - 0.08) { this.prevPose = null; return null; }
    // Whether the arm, turned up by roll, is in a TRENCH arm grown by grow on each side; with
    // deepest, where the TRENCH arm pushes it: the face each point in it came in through (where it
    // was the step before: under the TRENCH arm, its underside; else the side it was on), and the
    // push's torque about the pivot from all of them (+ up). { torque, point (world), n (world),
    // lever: torque per newton about the pivot at the deepest point } or null
    const prev = this.prevPose;
    const test = (roll, grow = 0, deepest = false) => {
      const c = Math.cos(roll), s = Math.sin(roll);
      let best = null, bestSide = null, torque = 0, side = 0;
      for (const [x, y] of this.armPts.pts) {
        const lx = px + x * c - y * s, ly = py + x * s + y * c;
        for (const z of this.armPts.zs) {
          _v.set(lx, ly, z).applyQuaternion(r.quat).add(r.pos);
          for (const a of beams) {
            const bottom = a.y - a.hy - grow;
            const dx = a.hx + grow - Math.abs(_v.x - a.x), dy = _v.y - bottom;
            if (dx <= 0 || dy <= 0 || _v.y > TRENCH.height || Math.abs(_v.z - a.z) > a.hz) continue;
            if (!deepest) return true;
            let n;
            if (prev) {
              const pc = Math.cos(prev.roll), ps = Math.sin(prev.roll);
              const w = new THREE.Vector3(px + x * pc - y * ps, py + x * ps + y * pc, z).applyQuaternion(prev.quat).add(prev.pos);
              if (w.y <= bottom + 1e-4) n = new THREE.Vector3(0, -1, 0);
              else if (Math.abs(w.x - a.x) >= a.hx) n = new THREE.Vector3(Math.sign(w.x - a.x), 0, 0);
            }
            if (!n) n = dx < dy ? new THREE.Vector3(Math.sign(_v.x - a.x) || 1, 0, 0) : new THREE.Vector3(0, -1, 0);
            const d = n.y ? dy : dx;
            // (the push's direction in the robot's frame, and its torque about the pivot)
            const nl = n.clone().applyQuaternion(_q.copy(r.quat).invert());
            const lever = (lx - px) * nl.y - (ly - py) * nl.x;
            if (n.y) torque += d * lever; else side += d * lever;
            if (!best || d > best.depth) best = { depth: d, point: _v.clone(), n, lever };
            if (!n.y && (!bestSide || d > bestSide.depth)) bestSide = { depth: d, point: _v.clone(), n, lever };
          }
        }
      }
      // (a side face pushing it, as the robot drives under, says which way it goes; else the underside)
      if (bestSide) return Object.assign(bestSide, { torque: side });
      return deepest ? best && Object.assign(best, { torque }) : !!best;
    };
    const hit = test(st.roll, 0, true);
    this.prevPose = { pos: r.pos.clone(), quat: r.quat.clone(), roll: st.roll };
    // (held by its collider at the TRENCH arm's face: still there while it's within 3 cm of it)
    if (!hit && !(was && test(st.roll, 0.03))) { if (was) r.armCollider.setCollisionGroups(r.armGroups); return null; }
    // the turn that takes it out of the TRENCH arm one way (dir: +1 up): a degree at a time to the
    // first that's clear, then halved down to under a fiftieth of a degree; null if it isn't clear
    // short of the stop that way
    const seek = (dir) => {
      const end = dir < 0 ? 0 : st.stow;
      let lo = st.roll, clear = null;
      for (let k = 1; k <= 150 && clear === null; k++) {
        const a = st.roll + dir * k * DEG;
        if ((a - end) * dir >= 0) { if (!test(end)) clear = end; break; }
        if (!test(a)) clear = a; else lo = a;
      }
      if (clear === null) return null;
      for (let k = 0; k < 6; k++) { const m = (lo + clear) / 2; if (test(m)) lo = m; else clear = m; }
      return clear;
    };
    // which way: a side face's push says (as the robot drives under); the underside alone (the arm
    // swung up into it) pushes it whichever way gets it out sooner, as it went the step before if
    // that's about as soon
    let dir = hit ? Math.sign(hit.torque) || -1 : 1, clear = hit ? seek(dir) : null;
    if (hit?.n.y) {
      const other = seek(-dir), last = this.lastDir ?? -1;
      const d = (c) => (c === null ? Infinity : Math.abs(c - st.roll));
      const pick = Math.abs(d(clear) - d(other)) < 2 * DEG ? last : d(clear) <= d(other) ? dir : -dir;
      if (pick !== dir) { dir = pick; clear = other; }
    }
    if (clear === null) {
      // up against its upper stop (or held there at the face): it can't give
      if (!was) r.armCollider.setCollisionGroups(r.armGroupsRigid);
      this.trench.hard = true;
      return null;
    }
    if (was) r.armCollider.setCollisionGroups(r.armGroups);
    this.lastDir = dir;
    const rate = (clear - st.roll) / dt;
    if (dir < 0) pivot.stop(Math.min(pivot.lo, rate), Math.min(pivot.hi, rate));
    else pivot.stop(Math.max(pivot.lo, rate), Math.max(pivot.hi, rate));
    return hit;
  }
  // What the TRENCH arm's push on the intake arm took, back on the robot: the torque that turned
  // the pivot as it did beyond what its motors, the arm's weight and its drag would have, over the
  // push's lever, along the push, where it pushed
  trenchPush(hit, w0, dt) {
    const pivot = this.t.pivot, m = pivot.motor, w1 = pivot.omega;
    const cur = m.current, lim = m.limited;
    const motor = pivot.cmd === null ? 0 : m.torque(w1, pivot.cmd).tau;
    m.current = cur; m.limited = lim;
    const tau = (pivot.J * (w1 - w0)) / dt - motor - pivot.load + pivot.drag * w1;
    const F = Math.max(0, (Math.sign(hit.torque) * tau) / Math.max(0.05, Math.sign(hit.torque) * hit.lever));
    Object.assign(this.trench, { F, lever: hit.lever, n: `${hit.n.x},${hit.n.y}` });
    if (F <= 0) return;
    const J = hit.n.clone().multiplyScalar(F * dt);
    this.r.body.applyImpulseAtPoint({ x: J.x, y: J.y, z: J.z }, { x: hit.point.x, y: hit.point.y, z: hit.point.z }, true);
  }

  // the arm's turn up from down on its stop (rad), for drawing it
  armTurn() { return this.armSt.roll; }
  // ...in degrees (how far FUEL under it have lifted it, down)
  armLift() { return this.armSt.roll / DEG; }

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
    } else if (HOLD_FULL && this.fullT > 0 && !this.feed) {
      this.fullT -= dt;
    } else {
      this.jamT = on && !outtake && stalled ? this.jamT + dt : 0;
      // (HOLD_FULL: stalled with nothing being fed is a full path, not a jam: the rollers stop
      // and hold what they have instead of reversing, and try again after a moment)
      if (HOLD_FULL && this.jamT > JAM.debounce && !this.feed) { this.fullT = HOLD_FULL; this.jamT = 0; r.stats.holds = (r.stats.holds || 0) + 1; if (globalThis.process?.env?.JAMLOG && r.stats.holds < 40) console.log(`HOLD t ${t.toFixed(1)} feeding ${feeding} sp ${setpoint?.toFixed?.(1)} aimErr ${((r.aimErr ?? 0) / DEG).toFixed(0)} clear ${r.passClear} held ${r.stored?.length}`); }
      if (this.jamT > JAM.debounce) { this.unjamT = JAM.reverse; this.jamT = 0; r.stats.unjams = (r.stats.unjams || 0) + 1; if (globalThis.process?.env?.JAMLOG) this.logJam(t); }
    }
    const unjam = on && this.unjamT > 0, holding = HOLD_FULL && this.fullT > 0 && !this.feed && !outtake;
    intake8793(this.t, on && !holding, { dir: outtake || unjam ? -1 : 1 });
    // the turret indexer feeds while there's a shot (or pass) to make, reversed with the unjam:
    // their ShooterMode doesn't hold it for the flywheel or hood (HOLD_FEED_UNTIL_AT_SPEED is off),
    // so what's still coming up to speed goes as it is; only while the turret is swinging round
    // to its target (more than TURRET_RESET_TOLERANCE_DEG, 10 deg, off) is the feeder off
    // A pass goes whether it'll get there or not (their ShuttleMode doesn't wait either), as long
    // as it won't drop into a HUB (G407)
    const wasFeeding = this.feed;
    this.feed = on && feeding && setpoint > 0 && Math.abs(r.aimErr ?? 0) < 10 * DEG && r.passClear !== false;
    // (STOP_UNJAM: as the shot stops, back the indexer off ('ix', 0.15 s) or the whole path ('all',
    // their unjam), so the FUEL left in the rails aren't wedged there for the next shot)
    if (STOP_UNJAM && wasFeeding && !this.feed && on && !outtake) {
      if (STOP_UNJAM === 'all') { if (!(this.unjamT > 0)) { this.unjamT = JAM.reverse; this.jamT = 0; r.stats.stopUnjams = (r.stats.stopUnjams || 0) + 1; } }
      else { this.ixBackT = 0.15; r.stats.stopUnjams = (r.stats.stopUnjams || 0) + 1; }
    }
    if (this.ixBackT > 0) this.ixBackT -= dt;
    // (HOLD_SPEED: their HOLD_FEED_UNTIL_AT_SPEED, off in their code: the indexer waits while the
    // flywheel is under that share of its target, without the shot stopping)
    const fwRps = Math.abs(this.t.shooter.omega) / (2 * Math.PI);
    const atSpeed = !HOLD_SPEED || !this.targetRps || fwRps >= HOLD_SPEED * this.targetRps;
    this.t.indexer.cmd = this.feed ? (unjam ? -DUTY_INDEXER : atSpeed ? DUTY_INDEXER : 0) : on && (outtake || this.ixBackT > 0 || (STOP_UNJAM === 'all' && this.unjamT > 0)) ? -DUTY_INDEXER : 0;
    if (globalThis.process?.env?.JAMLOG) {
      // (the turret indexer while it's fed forward: how long it runs under a fifth of its free
      // speed, and how many FUEL go up to the shooter meanwhile)
      const ix = this.t.indexer, irps = Math.abs(ix.omega) / (2 * Math.PI), st = (this.ixStat ||= { fed: 0, slow: 0, slowRun: 0, runs: 0, amps: 0 });
      if (this.feed && !unjam) {
        st.fed += dt; st.amps += Math.abs(ix.motor.current) * dt;
        if (irps < 0.2 * 0.65 * 125.5) { st.slow += dt; st.slowRun += dt; if (st.slowRun > 0.125 && st.slowRun - dt <= 0.125) { st.runs++; if (st.runs <= 12) this.logJam(t, 'IXSTALL'); } } else st.slowRun = 0;
      } else st.slowRun = 0;
      if (t - (this.ixLast ?? 0) > 30) { this.ixLast = t; console.log(`IDX t ${t.toFixed(0)}: fed ${st.fed.toFixed(1)} s, under a fifth of its speed ${st.slow.toFixed(1)} s (${st.runs} stalls over 0.125 s), ${(st.amps / (st.fed || 1)).toFixed(0)} A on average; ${r.stats.stopUnjams || 0} backed off as a shot stopped`); }
    }
    // the flywheel: toward the speed the shot wants while there's one to make (it holds that
    // speed a moment after); coasting otherwise (their code)
    if (setpoint > 0) { this.targetRps = Math.min(SHOOTER.maxRps, rpsFor(setpoint)); this.holdT = t; }
    else if (!(on && this.targetRps && t - (this.holdT ?? -9) < 1.5)) this.targetRps = 0;
    this.t.shooter.cmd = on && this.targetRps > 0 ? flywheelDuty(this.t, this.targetRps) : 0;
    // what the robot's aiming sees: the exit speed its flywheel gives right now
    r.flywheel = EXIT_K * exitSpeed(Math.abs(this.t.shooter.omega) / (2 * Math.PI));
    r.intakeSpeed = h.cmd < 0 ? 1 : h.cmd > 0 ? -1 : 0;
    r.feeding = this.feed ? 1 : 0;
  }

  // (JAMLOG=1: each unjam, when, whether it was feeding, the FUEL held, and the FUEL pressed
  // hardest, robot frame, and what by)
  logJam(t, tag = 'JAM') {
    const body = this.r.body, p0 = body.translation(), q = body.rotation();
    _q.set(q.x, q.y, q.z, q.w).invert();
    const pts = new Map();
    for (const c of this.path.contacts) if (c.force > (tag === 'JAM' ? 30 : 10)) { const e = pts.get(c.ball) ?? []; e.push(c.wheel.name.replace(/ \(.*\)/, '') + ' ' + c.force.toFixed(0)); pts.set(c.ball, e); }
    const out = [...pts].map(([b, e]) => { const p = b.translation(); _v.set(p.x - p0.x, p.y - p0.y, p.z - p0.z).applyQuaternion(_q); return `${_v.x.toFixed(2)},${_v.y.toFixed(2)},${_v.z.toFixed(2)} [${e.join('; ')}]`; });
    const ix = this.t.indexer;
    console.log(`${tag} t ${t.toFixed(1)} feed ${this.feed ? 1 : 0} indexer ${(Math.abs(ix.omega) / (2 * Math.PI)).toFixed(0)} rps ${Math.abs(ix.motor.current).toFixed(0)} A, rollers ${(Math.abs(this.t.horizontal.omega) / (2 * Math.PI)).toFixed(1)} rps ${Math.abs(this.t.horizontal.motor.current).toFixed(0)} A, held ${this.r.stored?.length ?? '?'}: ${out.join(' | ')}`);
  }

  // the feed's push on a shot with the turret at a (rad) on the chassis: m/s along it, forward (FEED_PUSH)
  feedPush(a) { return FEED_PUSH[0] * Math.cos(a) ** 2 - FEED_PUSH[1] * Math.sin(a) ** 2; }

  // how many it holds, about, with the arm down: 7 in the frame (tools/mechlab.mjs slots) and a
  // couple more up on its arm (what it carries in matches)
  capacity() { return 9; }

  // The turret on its motor (js/mech/turretDrive.js): its X60 under their position loop, against its
  // bearings and its energy chain (js/echain.js). js/robot.js asks it each step where the turret
  // gets to on its way from cur to goal (rad, its yaw on the robot; t: the time, s).
  turretStep(cur, goal, dt, t) {
    this.turretDrive ||= drive8793();
    return this.turretDrive.step(cur, goal, dt, t);
  }
}
