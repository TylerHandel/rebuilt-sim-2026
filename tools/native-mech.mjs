// The native version's mechanism check (github.com/TylerHandel/rebuilt-sim-native, PLAN.md): this
// game's mechanisms (js/mech/mechanism.js) and its port step the same scene to the same bits. The
// scene: the exported field (tools/export-native.mjs), a box frame driving and turning on the
// carpet, and on it a ball path (a motor-driven roller, an omni idler, a mesh plate, a bumper Block,
// FUEL touching each other) and a sub-stepped shooter wheel (8 sub-steps: carried frames, twist),
// with 12 FUEL dropped onto them. A digest after every step: every FUEL's state and the frame's as
// little-endian f32 (as native-parity.mjs), then each shaft's speed as f64.
//   node --import ./tools/node-env.mjs tools/native-mech.mjs <world.json> <out.json>
import RAPIER from '@dimforge/rapier3d-simd-compat';
import { readFileSync, writeFileSync } from 'node:fs';
import { Mechanism, Motor, Shaft, Surface, Block, Wheel } from '../js/mech/mechanism.js';

await RAPIER.init();
const d = JSON.parse(readFileSync(process.argv[2], 'utf8'));
const W = new RAPIER.World({ x: 0, y: -d.physics.gravity, z: 0 });
W.timestep = d.physics.dt;
W.integrationParameters.numSolverIterations = d.physics.solverIterations;
const fixed = W.createRigidBody(RAPIER.RigidBodyDesc.fixed());
for (const c of d.field) {
  const s = c.shape;
  const cd = s.kind === 'cuboid' ? RAPIER.ColliderDesc.cuboid(...s.half)
    : s.kind === 'ball' ? RAPIER.ColliderDesc.ball(s.radius)
    : s.kind === 'capsule' ? RAPIER.ColliderDesc.capsule(s.halfHeight, s.radius)
    : s.kind === 'cylinder' ? RAPIER.ColliderDesc.cylinder(s.halfHeight, s.radius)
    : s.kind === 'convex' ? RAPIER.ColliderDesc.convexHull(new Float32Array(s.points))
    : RAPIER.ColliderDesc.trimesh(new Float32Array(s.vertices), new Uint32Array(s.indices));
  cd.setTranslation(...c.translation).setRotation({ x: c.rotation[0], y: c.rotation[1], z: c.rotation[2], w: c.rotation[3] })
    .setFriction(c.friction).setRestitution(c.restitution).setCollisionGroups(c.collisionGroups).setSolverGroups(c.solverGroups).setSensor(c.sensor);
  if (c.frictionCombine !== null) cd.setFrictionCombineRule(c.frictionCombine);
  if (c.restitutionCombine !== null) cd.setRestitutionCombineRule(c.restitutionCombine);
  W.createCollider(cd, fixed);
}
// (the scene: every number here is the native test's too, crates/sim/tests/mech.rs)
export const SCENE = {
  frame: { at: [-6, 0.12, 0], half: [0.35, 0.1, 0.35], v: [0.4, 0, 0.1], w: [0, 0.3, 0] },
  fuel: { n: 12, at: [-6.25, 0.45, -0.2], step: [0.15625, 0.15625, 0.125] },
  steps: 240,
};
const f = d.fuel, fuel = [];
const fr = SCENE.frame;
const frame = W.createRigidBody(RAPIER.RigidBodyDesc.dynamic().setTranslation(...fr.at));
W.createCollider(RAPIER.ColliderDesc.cuboid(...fr.half), frame);
frame.setLinvel({ x: fr.v[0], y: fr.v[1], z: fr.v[2] }, true);
frame.setAngvel({ x: fr.w[0], y: fr.w[1], z: fr.w[2] }, true);
for (let i = 0; i < SCENE.fuel.n; i++) {
  const [x0, y0, z0] = SCENE.fuel.at, [sx, sy, sz] = SCENE.fuel.step;
  const b = W.createRigidBody(RAPIER.RigidBodyDesc.dynamic().setTranslation(x0 + (i % 4) * sx, y0 + Math.floor(i / 4) * sy, z0 + (i % 3) * sz)
    .setLinearDamping(f.body.linearDamping).setAngularDamping(f.body.angularDamping).setCcdEnabled(f.body.ccd).setCanSleep(true));
  W.createCollider(RAPIER.ColliderDesc.ball(f.body.radius).setMass(f.body.mass).setFriction(f.collider.friction).setRestitution(f.collider.restitution).setCollisionGroups(f.collider.collisionGroups), b);
  fuel.push(b);
}
const roller = new Shaft({ name: 'roller', motor: new Motor('krakenX60', { ratio: 3 }) });
roller.cmd = 1;
const idler = new Shaft({ name: 'idler' });
const drum = new Shaft({ name: 'drum', motor: new Motor('krakenX60', { count: 2, ratio: 1, statorLimit: 80 }), inertia: 2e-3 });
drum.cmd = 0.8;
const plate = new Surface({ name: 'plate', vertices: [0, 0.1, -0.3, 0.6, 0.1, -0.3, 0.6, 0.1, 0.3, 0, 0.1, 0.3, 0, 0.3, 0.3], indices: [0, 2, 1, 0, 3, 2, 3, 4, 0], mu: 0.3, pos: [-0.3, 0.05, 0], yaw: 0.2, roll: 0.1 });
const bumper = new Block({ name: 'bumper', x0: 0.35, x1: 0.43, y0: 0.03, y1: 0.16, hw: 0.4, r: 0.02, mu: 0.35 });
const path = new Mechanism({
  body: frame, substeps: 4, shafts: [roller, idler],
  wheels: [
    new Wheel({ name: 'roller', shaft: roller, center: [0.25, 0.2, 0], axis: [0, 0, 1], radius: 0.05, width: 0.5, mu: 1.0 }),
    new Wheel({ name: 'omni', shaft: idler, center: [0.05, 0.38, 0], axis: [0, 0, 2], radius: 0.03, width: 0.4, kind: 'omni', mu: 0.8, tread: 4000 }),
  ],
  surfaces: [plate, bumper],
});
const shooter = new Mechanism({ body: frame, substeps: 4, sub: 8, shafts: [drum], wheels: [new Wheel({ name: 'drum', shaft: drum, center: [-0.2, 0.3, 0], axis: [0, 0, 1], radius: 0.0508, width: 0.6, mu: 1.1 })] });

const MASK = (1n << 64n) - 1n, f32 = new Float32Array(13), b32 = new Uint8Array(f32.buffer), f64 = new Float64Array(1), b64 = new Uint8Array(f64.buffer);
function digest() {
  let h = 0xcbf29ce484222325n;
  const eat = (bytes) => { for (const x of bytes) h = ((h ^ BigInt(x)) * 0x100000001b3n) & MASK; };
  for (const b of [...fuel, frame]) {
    const t = b.translation(), q = b.rotation(), v = b.linvel(), w = b.angvel();
    f32.set([t.x, t.y, t.z, q.x, q.y, q.z, q.w, v.x, v.y, v.z, w.x, w.y, w.z]);
    eat(b32);
  }
  for (const s of [roller, idler, drum]) { f64[0] = s.omega; eat(b64); }
  return h.toString(16).padStart(16, '0');
}
const dt = W.timestep, out = [];
for (let i = 0; i < SCENE.steps; i++) {
  path.step(dt, fuel, fuel);
  shooter.step(dt, fuel);
  W.step();
  out.push(digest());
}
writeFileSync(process.argv[3], JSON.stringify({ source: 'rebuilt-sim-2026 tools/native-mech.mjs', scene: SCENE, digests: out }, null, 1));
console.log(`mechanism parity: ${out.length} digests to ${process.argv[3]}`);
process.exit(0);
