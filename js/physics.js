import RAPIER from 'rapier';
import { GRAVITY, GROUP, groups, PHYSICS_DT } from './constants.js';

export { RAPIER };

export class Physics {
  static async create() {
    await RAPIER.init();
    return new Physics();
  }

  constructor() {
    this.world = new RAPIER.World({ x: 0, y: -GRAVITY, z: 0 });
    this.world.timestep = PHYSICS_DT;
    this.world.integrationParameters.numSolverIterations = 6;
    this.fixedBody = this.world.createRigidBody(RAPIER.RigidBodyDesc.fixed());
    this.staticGroups = groups(GROUP.STATIC, GROUP.BALL | GROUP.HELD | GROUP.ROBOT | GROUP.WHEEL | GROUP.INTAKE);
    this.terrainGroups = groups(GROUP.TERRAIN, GROUP.BALL | GROUP.HELD | GROUP.ROBOT | GROUP.WHEEL);
    // (a low floor feature whose wheels' side is its own ramped twin, field.js rampC: as terrain, but
    // not to the wheels; and the twin, only to the wheels)
    this.terrainNoWheelGroups = groups(GROUP.TERRAIN, GROUP.BALL | GROUP.HELD | GROUP.ROBOT);
    this.wheelRampGroups = groups(GROUP.TERRAIN, GROUP.WHEEL);
    // (the TRENCH arms: as static, but a member of their own group, which an intake arm the TRENCH
    // pushes down on its pivot leaves out: GROUP.TRENCH_ARM)
    this.trenchArmGroups = groups(GROUP.TRENCH_ARM, GROUP.BALL | GROUP.HELD | GROUP.ROBOT | GROUP.WHEEL | GROUP.INTAKE);
    // What passes under a TRENCH arm without reaching it isn't stopped by it. Rapier makes contacts
    // within its prediction distance (2 cm) and lets the solver close each one by no more than its
    // gap per step along its normal (a speculative contact). Coming in under the arm, a robot's top
    // edge meets the arm's lower edge edge-on, so the normal points along the gap between the two
    // edges, slanted: a robot driving level under the arm with h to spare is read as closing on it
    // at its speed times the slant, and from 2h/dt (2.2 m/s at 9 mm, 3 at 13 mm, 4.6 at 19 mm) the
    // solver pushed back on one that was never going to touch it: it stopped dead at the arm's edge,
    // or slowed and was turned. A collider registered with passesUnder (a robot's frame) is left
    // out of the solver against the TRENCH arms (its solver groups, the arms' contacts still made)
    // while near one, its highest point by the end of the step, top(), stays under their underside:
    // reaching it, its contacts with them are solved as before.
    this.arms = [];            // the TRENCH arm colliders' boxes (world: center, half sizes)
    this.armsBottom = Infinity; // the lowest arm's underside
    this.underArm = [];        // { collider, top, under }
    this.armSolverGroups = ((GROUP.TRENCH_ARM & 0xffff) << 16) | 0xffff;
    this.underSolverGroups = (0xffff0000 | (0xffff & ~GROUP.TRENCH_ARM)) >>> 0;
  }

  step() {
    for (const u of this.underArm) {
      const p = u.collider.translation();
      // (only near an arm, where it matters: 1 m round it covers any frame and its step)
      if (!this.arms.some((a) => Math.abs(p.x - a.x) < a.hx + 1 && Math.abs(p.z - a.z) < a.hz + 1)) continue;
      const under = u.top() < this.armsBottom;
      if (under !== u.under) {
        u.under = under;
        u.collider.setSolverGroups(under ? this.underSolverGroups : 0xffffffff);
      }
    }
    this.world.step();
  }

  // collider (a robot's frame) passes under a TRENCH arm untouched while top() (the highest it
  // reaches by the end of the coming step) is under the arm's underside
  passesUnder(collider, top) {
    // (under to start with, as it's made: a robot that reaches the arm is found to when it nears it)
    collider.setSolverGroups(this.underSolverGroups);
    this.underArm.push({ collider, top, under: true });
  }

  forget(collider) {
    this.underArm = this.underArm.filter((u) => u.collider !== collider);
  }

  _finish(desc, opts = {}) {
    desc.setFriction(opts.friction ?? 0.5);
    desc.setRestitution(opts.restitution ?? 0.3);
    desc.setCollisionGroups(opts.groups ?? this.staticGroups);
    if (opts.translation) desc.setTranslation(opts.translation.x, opts.translation.y, opts.translation.z);
    if (opts.rotation) desc.setRotation(opts.rotation);
    // (a TRENCH arm, an unrotated box: a member of its own solver group, see underArm)
    const arm = opts.groups === this.trenchArmGroups;
    if (arm) desc.setSolverGroups(this.armSolverGroups);
    const c = this.world.createCollider(desc, this.fixedBody);
    if (arm) {
      const t = c.translation(), h = c.halfExtents();
      this.arms.push({ x: t.x, z: t.z, hx: h.x, hz: h.z });
      this.armsBottom = Math.min(this.armsBottom, t.y - h.y);
    }
    return c;
  }

  // Axis-aligned (optionally yaw-rotated) static box given center and half extents
  box(cx, cy, cz, hx, hy, hz, opts = {}) {
    const desc = RAPIER.ColliderDesc.cuboid(hx, hy, hz);
    return this._finish(desc, { ...opts, translation: { x: cx, y: cy, z: cz } });
  }

  // Box with an arbitrary quaternion rotation ({x,y,z,w})
  orientedBox(cx, cy, cz, hx, hy, hz, quat, opts = {}) {
    const desc = RAPIER.ColliderDesc.cuboid(hx, hy, hz);
    return this._finish(desc, { ...opts, translation: { x: cx, y: cy, z: cz }, rotation: quat });
  }

  hull(points, opts = {}) {
    const desc = RAPIER.ColliderDesc.convexHull(new Float32Array(points));
    if (!desc) throw new Error('convex hull failed');
    return this._finish(desc, opts);
  }

  trimesh(vertices, indices, opts = {}) {
    const desc = RAPIER.ColliderDesc.trimesh(new Float32Array(vertices), new Uint32Array(indices));
    return this._finish(desc, opts);
  }

  // Horizontal cylinder (pipe) between two points on the same height
  pipe(x1, y, z1, x2, z2, r, opts = {}) {
    const len = Math.hypot(x2 - x1, z2 - z1);
    const desc = RAPIER.ColliderDesc.capsule(len / 2, r);
    // capsule axis is Y; rotate to lie along the pipe direction
    const ang = Math.atan2(z2 - z1, x2 - x1);
    // first rotate Y->X (about Z by -90deg), then yaw by -ang about Y
    const qz = { x: 0, y: 0, z: Math.sin(-Math.PI / 4), w: Math.cos(-Math.PI / 4) };
    const qy = { x: 0, y: Math.sin(-ang / 2), z: 0, w: Math.cos(-ang / 2) };
    const q = mulQuat(qy, qz);
    return this._finish(desc, { ...opts, translation: { x: (x1 + x2) / 2, y, z: (z1 + z2) / 2 }, rotation: q });
  }
}

export function mulQuat(a, b) {
  return {
    w: a.w * b.w - a.x * b.x - a.y * b.y - a.z * b.z,
    x: a.w * b.x + a.x * b.w + a.y * b.z - a.z * b.y,
    y: a.w * b.y - a.x * b.z + a.y * b.w + a.z * b.x,
    z: a.w * b.z + a.x * b.y - a.y * b.x + a.z * b.w,
  };
}

export function yawQuat(yaw) {
  return { x: 0, y: Math.sin(yaw / 2), z: 0, w: Math.cos(yaw / 2) };
}
