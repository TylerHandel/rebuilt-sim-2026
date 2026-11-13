// The game's world as data, for the native version (github.com/TylerHandel/rebuilt-sim-native): every
// static collider of the field exactly as this game builds them (shape, pose, friction, restitution,
// collision and solver groups), the physics settings, and the FUEL (their body and collider settings,
// and where a match starts them). The native game loads this instead of rebuilding the field by hand,
// so its field is this one's to the vertex.
// Also, beside it: constants.json (js/constants.js's values; its functions are ported by hand) and
// robots.json (js/robotConfigs.js's ROBOTS, CLIMBER_OPTIONS, AUTO_ROUTINES, ROBOT_ORDER, which
// robots are built of physical mechanisms, what each one's modeled hopper holds as the menu shows it
// (js/hopper.js modelCapacity), and js/auto.js's START_POSITIONS and START_ORDER).
//   node --import ./tools/node-env.mjs tools/export-native.mjs <out.json> [--3v3 a,b,c:d,e,f]
import { writeFileSync } from 'node:fs';
import { dirname, join } from 'node:path';
import * as CONSTANTS from '../js/constants.js';
import { ROBOTS, CLIMBER_OPTIONS, AUTO_ROUTINES, ROBOT_ORDER, BUMPER_T, BUMPER_Y0, BUMPER_Y1 } from '../js/robotConfigs.js';
import { createWorld, mulberry32 } from './headless.mjs';
import { createGame } from '../js/game.js';
import { FUEL, GROUP, GRAVITY, PHYSICS_DT } from '../js/constants.js';
import { modelCapacity, measureCapacity, intakePush } from '../js/hopper.js';
import { START_POSITIONS, START_ORDER, BEST_AUTOS } from '../js/auto.js';
import { TEAM_AUTOS } from '../js/teamAutos.js';

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
// (every integration parameter the JS API has, as the engine holds it: the native sim checks its own
// defaults are these)
const integration = {};
for (const k of Object.getOwnPropertyNames(Object.getPrototypeOf(ip))) {
  if (k === 'constructor' || k === 'free') continue;
  const v = ip[k];
  if (typeof v === 'number' || typeof v === 'boolean') integration[k] = v;
}
// the OUTPOSTS' world-space info (field.js outposts[a]: its corral, the CHUTE's upper opening, where
// the HUMAN PLAYER throws from, which way is into the FIELD), and the TRENCH arms' boxes
// (physics.js: arms, armsBottom; the rule that lets a frame under them)
const outposts = {};
for (const a of ['blue', 'red']) {
  const o = world.field.outposts[a];
  outposts[a] = { corral: o.corral, upperOpening: o.upperOpening, hpThrowPos: o.hpThrowPos, intoField: o.intoField };
}
const data = {
  source: 'rebuilt-sim-2026 tools/export-native.mjs',
  outposts,
  trenchArms: { arms: P.arms, armsBottom: P.armsBottom, armSolverGroups: P.armSolverGroups >>> 0, underSolverGroups: P.underSolverGroups >>> 0 },
  physics: { gravity: GRAVITY, dt: PHYSICS_DT, solverIterations: ip.numSolverIterations, predictionDistance: ip.normalizedPredictionDistance ?? null, integration },
  groups: GROUP,
  field,
  fuel: { total: fuel.balls.length, body: fuelBody, collider: fuelCollider, start, preloaded },
};
writeFileSync(out, JSON.stringify(data));

// the constants (numbers, strings, and the objects and arrays of them; not the functions)
const constants = {};
for (const [k, v] of Object.entries(CONSTANTS)) if (typeof v !== 'function') constants[k] = v;
Object.assign(constants, { BUMPER_T, BUMPER_Y0, BUMPER_Y1 });
writeFileSync(join(dirname(out), 'constants.json'), JSON.stringify({ source: 'rebuilt-sim-2026 js/constants.js (and robotConfigs.js bumper sizes)', constants }, null, 1));
const physical = Object.entries(ROBOTS).filter(([, c]) => c.physical).map(([k]) => k);
const capacity = Object.fromEntries(Object.entries(ROBOTS).map(([k, c]) => [k, modelCapacity(c)]));
// a generic robot's hopper as js/robot.js's constructor measures it: what it holds retracted,
// extended and (a compacting intake) compacted, capped at the team's own number; and how hard its
// intake shoves FUEL in (the native game holds FUEL the same way, and needn't measure it again)
const geoCap = Object.fromEntries(Object.entries(ROBOTS).filter(([, c]) => !c.physical).map(([k, cfg]) => {
  const push = intakePush(cfg), ext = cfg.storage.extLen || 0;
  const g = { retracted: measureCapacity(cfg.bay, cfg.bay.x1, 0, push), extended: measureCapacity(cfg.bay, cfg.bay.x1 + ext, 1, push) };
  if (cfg.intake.compacts) g.compacted = Math.max(g.retracted, measureCapacity(cfg.bay, cfg.bay.x1, 0, cfg.intake.compactPush ?? push));
  const real = cfg.storage.capacity, capLimited = !!real && real < g.extended;
  if (capLimited) {
    const k = real / g.extended;
    g.retracted = Math.min(g.retracted, cfg.storage.retracted ?? Math.round(g.retracted * k));
    if (g.compacted !== undefined) g.compacted = Math.min(g.compacted, Math.max(g.retracted, Math.round(g.compacted * k)));
    g.extended = real;
  }
  return [k, { ...g, capLimited, intakePush: push }];
}));
writeFileSync(join(dirname(out), 'robots.json'), JSON.stringify({ source: 'rebuilt-sim-2026 js/robotConfigs.js', order: ROBOT_ORDER, physical, robots: ROBOTS, climbers: CLIMBER_OPTIONS, autoRoutines: AUTO_ROUTINES, capacity, geoCap, startPositions: START_POSITIONS, startOrder: START_ORDER, bestAutos: BEST_AUTOS, teamAutos: TEAM_AUTOS }, null, 1));
console.log(`constants.json, robots.json (${Object.keys(ROBOTS).length} robots, ${physical.length} physical); exported ${field.length} field colliders (${[...new Set(field.map((c) => c.shape.kind))].join(', ')}), ${start.length} FUEL on the field (+${preloaded} preloaded) to ${out}`);
process.exit(0);
