// Autonomous routines. G402: drive teams may not control the ROBOT during AUTO, so the robot
// runs one of these pre-programmed routines. Waypoints are in BLUE field coordinates
// (fx from the blue ALLIANCE WALL, fy from the blue drivers' right) and are mirrored for
// the red ALLIANCE and for left-side starts.
import { FIELD_W, ALLIANCE_ZONE_DEPTH, HUB, DEPOT, fw } from './constants.js';
import { clamp, wrapAngle } from './util.js';
import { getCustom, customSteps } from './customAutos.js';

export const START_POSITIONS = {
  leftTrench: { name: 'Left Trench', fy: FIELD_W - 0.64 },
  leftBump: { name: 'Left Bump', fy: HUB.fy + HUB.size / 2 + 0.93 },
  hub: { name: 'Hub', fy: HUB.fy },
  rightBump: { name: 'Right Bump', fy: HUB.fy - HUB.size / 2 - 0.93 },
  rightTrench: { name: 'Right Trench', fy: 0.64 },
};
export const START_ORDER = ['leftTrench', 'leftBump', 'hub', 'rightBump', 'rightTrench'];

// Starting pose: BUMPERS overlapping the ROBOT STARTING LINE, not touching the BUMP (G303)
export function startPose(key, robot, alliance, custom = null) {
  let fy;
  if (custom) {
    fy = custom.mirror ? FIELD_W - custom.auto.startFy : custom.auto.startFy;
    fy = clamp(fy, robot.halfW + 0.02, FIELD_W - robot.halfW - 0.02);
  } else fy = START_POSITIONS[key].fy;
  const fx = ALLIANCE_ZONE_DEPTH - 0.02 - robot.halfL;
  const w = fw(fx, fy, alliance);
  return { x: w.x, z: w.z, yaw: alliance === 'blue' ? 0 : Math.PI };
}

// Resolve a custom auto selection: { auto, mirror } or null for built-in routines
export function customSelection(routine, side) {
  const auto = getCustom(routine);
  return auto ? { auto, mirror: side === 'mirror' } : null;
}

export class AutoRunner {
  constructor(robot, routine, startKey, alliance, custom = null) {
    this.robot = robot;
    this.alliance = alliance;
    this.left = !custom && START_POSITIONS[startKey].fy > FIELD_W / 2 + 0.01;
    this.steps = custom ? customSteps(custom.auto, custom.mirror, robot.cfg.drive.maxSpeed) : this._build(routine);
    this.i = 0;
    this.stepT = 0;
    this.done = false;
  }

  // mirror for start side (routines are authored for the right side / low fy)
  P(fx, fy, sideMirror = true) {
    const y = sideMirror && this.left ? FIELD_W - fy : fy;
    return fw(fx, y, this.alliance);
  }

  _build(routine) {
    const r = this.robot;
    const small = r.cfg.storage.capacity < 20; // hopperless: pass while collecting
    const hl = r.halfL;
    const steps = [];
    const shootPreload = () => steps.push({ type: 'shoot', timeout: 3.0 });
    const sweep = (fxLine, back) => {
      // through the TRENCH into the NEUTRAL ZONE, sweep across the FUEL, back over the BUMP
      steps.push({ type: 'drive', pts: [[3.2, 0.64], [5.6, 0.64], [fxLine - 0.2, 1.25]], intake: true, speed: 1.0 });
      steps.push({ type: 'drive', pts: [[fxLine, 1.45], [fxLine, 4.4]], intake: true, shoot: small, speed: small ? 0.75 : 0.55 });
      steps.push({ type: 'drive', pts: [[6.3, 3.0], [5.2, 2.55], [3.3, 2.4]], intake: small, shoot: small ? true : 'hub', speed: back ? 0.9 : 1.0 });
      steps.push({ type: 'shoot', timeout: 2.5 });
    };
    switch (routine) {
      case 'preload':
        shootPreload();
        break;
      case 'depot': {
        shootPreload();
        const dfy = DEPOT.fy;
        steps.push({ type: 'drive', pts: [[2.4, 5.2], [1.5, dfy]], intake: true, speed: 1.0, mirror: false, face: Math.PI });
        const dx = hl + r.cfg.intake.reach + 0.04; // intake bar against the ALLIANCE WALL
        steps.push({ type: 'drive', pts: [[dx, dfy - 0.3]], intake: true, speed: 0.5, mirror: false, face: Math.PI });
        steps.push({ type: 'drive', pts: [[dx, dfy + 0.3]], intake: true, speed: 0.3, mirror: false, face: Math.PI });
        steps.push({ type: 'drive', pts: [[2.6, 5.0]], intake: true, shoot: 'hub', speed: 0.8, mirror: false });
        steps.push({ type: 'shoot', timeout: 4 });
        break;
      }
      case 'sweep':
        shootPreload();
        sweep(7.55, false);
        break;
      case 'sweep2':
        shootPreload();
        sweep(7.55, false);
        sweep(7.85, true);
        break;
      case 'climb':
        shootPreload();
        steps.push({ type: 'climb' });
        break;
      default:
        break;
    }
    return steps;
  }

  update(dt) {
    const r = this.robot;
    const cmd = r.cmd;
    cmd.vx = 0; cmd.vz = 0; cmd.omega = 0;
    cmd.intake = false; cmd.shoot = false; cmd.pass = false; cmd.outtake = false;
    if (this.done || this.i >= this.steps.length) {
      this.done = true;
      return;
    }
    const s = this.steps[this.i];
    this.stepT += dt;
    const next = () => { this.i++; this.stepT = 0; this.pi = 0; };
    if (s.type === 'shoot') {
      cmd.shoot = true;
      // too close to the HUB for this shooter: back away toward our ALLIANCE WALL
      if (/range/.test(r.status)) cmd.vx = (this.alliance === 'blue' ? -1 : 1) * 1.2;
      if (r.stored.length === 0 || this.stepT > s.timeout) next();
      return;
    }
    if (s.type === 'wait') {
      if (this.stepT >= s.t) next();
      return;
    }
    if (s.type === 'climb') {
      if (r.climbState === 'none' && !s.requested) {
        // approach the climb pose, then request the climb
        const p = r.climbPose();
        const dx = p.x - r.pos.x, dz = p.z - r.pos.z;
        const d = Math.hypot(dx, dz);
        if (d > 1.0) {
          const sp = Math.min(r.cfg.drive.maxSpeed * 0.8, d * 2);
          cmd.vx = (dx / d) * sp; cmd.vz = (dz / d) * sp;
          cmd.omega = clamp(4 * wrapAngle(p.yaw - r.yaw), -5, 5);
        } else {
          r.requestClimb();
          s.requested = true;
        }
      }
      return;
    }
    if (s.type === 'drive') {
      if (this.pi === undefined) this.pi = 0;
      const pts = s.pts.map(([fx, fy]) => this.P(fx, fy, s.mirror !== false));
      const tgt = pts[this.pi];
      const last = this.pi === pts.length - 1;
      const dx = tgt.x - r.pos.x, dz = tgt.z - r.pos.z;
      const d = Math.hypot(dx, dz);
      let remain = d;
      for (let k = this.pi; k + 1 < pts.length; k++) remain += Math.hypot(pts[k + 1].x - pts[k].x, pts[k + 1].z - pts[k].z);
      const vmax = r.cfg.drive.maxSpeed * (s.speed ?? 1);
      const sp = Math.min(vmax, Math.sqrt(2 * 6 * Math.max(0, remain - 0.02)) + 0.05);
      if (d > 1e-3) { cmd.vx = (dx / d) * sp; cmd.vz = (dz / d) * sp; }
      // heading: face the direction of travel (intake first) unless told otherwise
      let face = s.face !== undefined ? (this.alliance === 'blue' ? s.face : s.face + Math.PI) : Math.atan2(-dz, dx);
      if (this.left && s.face === undefined) face = Math.atan2(-dz, dx);
      if (d > 0.15 || s.face !== undefined) cmd.omega = clamp(5 * wrapAngle(face - r.yaw), -r.cfg.drive.maxOmega, r.cfg.drive.maxOmega);
      cmd.intake = !!s.intake;
      // 'hub' = shoot on the move only once back in our ALLIANCE ZONE (don't pass the FUEL away)
      cmd.shoot = s.shoot === 'hub' ? r.lastInZone : !!s.shoot;
      if ((last && d < 0.1) || (!last && d < 0.35)) {
        if (last) next();
        else this.pi++;
      }
      if (this.stepT > (s.timeout ?? 8)) next();
    }
  }
}
