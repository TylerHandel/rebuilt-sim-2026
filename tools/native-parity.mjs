// The native version's engine check (github.com/TylerHandel/rebuilt-sim-native, PLAN.md): its Rapier
// (rapier3d, native) and this game's (the same engine, as WebAssembly) step the same world to the
// same bits. This runs the exported world (tools/export-native.mjs) here, with nothing of the game
// but its field's colliders and its FUEL, and writes a digest of every FUEL's state at set steps;
// the native sim's tests run the same and compare.
//   node tools/native-parity.mjs <world.json> <out.json>
// The scenario: the FUEL as a match starts them, 360 steps; then a heap of 120 more dropped by the
// HUB, every seventh one thrown, 360 more. Digests at steps 1, 10, 60, 120, 360 of each part.
// A digest: FNV-1a (64 bit) over each FUEL's position, rotation (x, y, z, w), velocity and spin as
// little-endian f32, in the order they were made.
import RAPIER from '@dimforge/rapier3d-simd-compat';
import { readFileSync, writeFileSync } from 'node:fs';

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
const f = d.fuel, fuel = [];
function add(x, y, z, v = null) {
  const b = W.createRigidBody(RAPIER.RigidBodyDesc.dynamic().setTranslation(x, y, z).setLinearDamping(f.body.linearDamping).setAngularDamping(f.body.angularDamping).setCcdEnabled(f.body.ccd).setCanSleep(true));
  W.createCollider(RAPIER.ColliderDesc.ball(f.body.radius).setMass(f.body.mass).setFriction(f.collider.friction).setRestitution(f.collider.restitution).setCollisionGroups(f.collider.collisionGroups), b);
  if (v) { b.setLinvel({ x: v[0], y: v[1], z: v[2] }, true); b.setAngvel({ x: v[3], y: v[4], z: v[5] }, true); }
  fuel.push(b);
}
for (const p of f.start) add(...p);

const MASK = (1n << 64n) - 1n, f32 = new Float32Array(13), bytes = new Uint8Array(f32.buffer);
function digest() {
  let h = 0xcbf29ce484222325n;
  for (const b of fuel) {
    const t = b.translation(), q = b.rotation(), v = b.linvel(), w = b.angvel();
    f32.set([t.x, t.y, t.z, q.x, q.y, q.z, q.w, v.x, v.y, v.z, w.x, w.y, w.z]);
    for (const x of bytes) h = ((h ^ BigInt(x)) * 0x100000001b3n) & MASK;
  }
  return h.toString(16).padStart(16, '0');
}
// (every number here is exact in f32, so both sides place the heap on the same bits)
export const HEAP = { n: 120, step: 0.15625, at: [-1, 1, 1.125], thrown: [3, 2, -1.5, 8, 0, -4] };
const STEPS = [1, 10, 60, 120, 360], out = [];
for (const part of ['start', 'heap']) {
  let at = 0;
  for (const n of STEPS) { for (; at < n; at++) W.step(); out.push({ part, step: n, fuel: fuel.length, digest: digest() }); }
  if (part === 'start') for (let i = 0; i < HEAP.n; i++) add(HEAP.at[0] + (i % 10) * HEAP.step, HEAP.at[1] + Math.floor(i / 40) * HEAP.step, HEAP.at[2] + (Math.floor(i / 10) % 4) * HEAP.step, i % 7 === 0 ? HEAP.thrown : null);
}
writeFileSync(process.argv[3], JSON.stringify({ source: 'rebuilt-sim-2026 tools/native-parity.mjs', rapier: '@dimforge/rapier3d-simd-compat 0.21.0', heap: HEAP, digests: out }, null, 1));
console.log(`engine parity: ${out.length} digests to ${process.argv[3]}`);
process.exit(0);
