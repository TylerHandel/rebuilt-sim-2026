// The game's world as data, for the native version (github.com/TylerHandel/rebuilt-sim-native): every
// static collider of the field exactly as this game builds them (shape, pose, friction, restitution,
// collision and solver groups), the physics settings, and the FUEL (their body and collider settings,
// and where a match starts them). The native game loads this instead of rebuilding the field by hand,
// so its field is this one's to the vertex.
//   node --import ./tools/node-env.mjs tools/export-native.mjs <out.json> [--3v3 a,b,c:d,e,f]
import { writeFileSync } from 'node:fs';
import { createWorld, mulberry32 } from './headless.mjs';
import { createGame } from '../js/game.js';
import { FUEL, GROUP, GRAVITY, PHYSICS_DT } from '../js/constants.js';

const args = process.argv.slice(2);
const out = args.find((a) => !a.startsWith('--')) ?? 'world.json';
const arg = (k, d) => { const i = args.indexOf('--' + k); return i >= 0 ? args[i + 1] : d; };

const world = await createWorld();
Math.random = mulberry32(1);
const [blue, red] = arg('3v3', '2910,1678,4414:6328,7769,1690').split(':').map((s) => s.split(','));
const starts = ['leftTrench', 'hub', 'rightBump'];
const game = createGame(world, { climber: 'none', preload: 8, hp: 'auto', matchMode: '3v3', slots: [...blue, ...red].map((robot, i) => ({ robot, auto: 'best', start: starts[i % 3], driver: 'scorer', skill: 'champs' })) });

const P = world.physics, W = P.world, R = P.constructor.name && (await import('rapier')).default;
const ST = R.ShapeType;
const arr = (a) => Array.from(a, (v) => +v);
// one collider as data (shapes the field uses: cuboid, convex hull, trimesh, capsule, ball)
function shapeOf(c) {
  const s = c.shape, t = s.type ?? c.shapeType();
  switch (t) {
    case ST.Cuboid: { const h = c.halfExtents(); return { kind: 'cuboid', half: [h.x, h.y, h.z] }; }
    case ST.Ball: return { kind: 'ball', radius: c.radius() };
    case ST.Capsule: return { kind: 'capsule', halfHeight: c.halfHeight(), radius: c.radius() };
    case ST.ConvexPolyhedron: return { kind: 'convex', points: arr(c.vertices()) };
    case ST.TriMesh: return { kind: 'trimesh', vertices: arr(c.vertices()), indices: arr(c.indices()) };
    case ST.Cylinder: return { kind: 'cylinder', halfHeight: c.halfHeight(), radius: c.radius() };
    default: throw new Error('export-native: a shape of type ' + t);
  }
}
function colliderOf(c) {
  const t = c.translation(), q = c.rotation();
  return {
    shape: shapeOf(c), translation: [t.x, t.y, t.z], rotation: [q.x, q.y, q.z, q.w],
    friction: c.friction(), restitution: c.restitution(), frictionCombine: c.frictionCombineRule?.() ?? null, restitutionCombine: c.restitutionCombineRule?.() ?? null,
    collisionGroups: c.collisionGroups() >>> 0, solverGroups: c.solverGroups() >>> 0, sensor: c.isSensor(),
  };
}
const field = [];
W.forEachCollider((c) => { if (c.parent() && c.parent().handle === P.fixedBody.handle) field.push(colliderOf(c)); });

// the FUEL: what each body and collider has, and where this match starts them
const fuel = world.fuel ?? game.fuel;
const b0 = fuel.balls[0];
const fuelBody = { linearDamping: b0.body.linearDamping(), angularDamping: b0.body.angularDamping(), ccd: b0.body.isCcdEnabled(), mass: FUEL.mass, radius: FUEL.radius };
const fc = b0.body.collider(0);
const fuelCollider = { friction: fc.friction(), restitution: fc.restitution(), collisionGroups: fc.collisionGroups() >>> 0 };
// (on the field at the start: the ones a robot holds as preload are left out; the native game places those with its robots)
const onField = fuel.balls.filter((b) => b.body.isEnabled() && b.body.translation().y > -1);
const start = onField.map((b) => { const t = b.body.translation(); return [t.x, t.y, t.z]; });
const preloaded = fuel.balls.length - onField.length;

const ip = W.integrationParameters;
const data = {
  source: 'rebuilt-sim-2026 tools/export-native.mjs',
  physics: { gravity: GRAVITY, dt: PHYSICS_DT, solverIterations: ip.numSolverIterations, predictionDistance: ip.normalizedPredictionDistance ?? null },
  groups: GROUP,
  field,
  fuel: { total: fuel.balls.length, body: fuelBody, collider: fuelCollider, start, preloaded },
};
writeFileSync(out, JSON.stringify(data));
console.log(`exported ${field.length} field colliders (${[...new Set(field.map((c) => c.shape.kind))].join(', ')}), ${start.length} FUEL on the field (+${preloaded} preloaded) to ${out}`);
process.exit(0);
