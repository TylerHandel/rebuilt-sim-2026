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
    this.staticGroups = groups(GROUP.STATIC, GROUP.BALL | GROUP.ROBOT | GROUP.WHEEL | GROUP.INTAKE);
    this.terrainGroups = groups(GROUP.TERRAIN, GROUP.BALL | GROUP.ROBOT | GROUP.WHEEL);
  }

  step() {
    this.world.step();
  }

  _finish(desc, opts = {}) {
    desc.setFriction(opts.friction ?? 0.5);
    desc.setRestitution(opts.restitution ?? 0.3);
    desc.setCollisionGroups(opts.groups ?? this.staticGroups);
    if (opts.translation) desc.setTranslation(opts.translation.x, opts.translation.y, opts.translation.z);
    if (opts.rotation) desc.setRotation(opts.rotation);
    return this.world.createCollider(desc, this.fixedBody);
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
