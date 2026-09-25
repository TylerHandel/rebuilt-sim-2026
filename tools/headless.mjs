// Headless AI-vs-AI matches: the same field, FUEL, robots, rules and AIs as the browser game,
// stepped as fast as the CPU allows with no rendering.
import * as THREE from 'three';
import { Physics } from '../js/physics.js';
import { Field } from '../js/field.js';
import { FuelManager } from '../js/fuel.js';
import { createGame, stepGame, frameGame } from '../js/game.js';
import { opponentAuto } from '../js/opponent.js';
import { PHYSICS_DT, BLUE, RED, other } from '../js/constants.js';

// Seeded PRNG so every candidate in a generation sees the same shot scatter, tiebreaks and
// FUEL bounces (common random numbers make comparisons far less noisy)
export function mulberry32(seed) {
  let a = seed >>> 0;
  return () => {
    a = (a + 0x6d2b79f5) >>> 0;
    let t = a;
    t = Math.imul(t ^ (t >>> 15), t | 1);
    t ^= t + Math.imul(t ^ (t >>> 7), t | 61);
    return ((t ^ (t >>> 14)) >>> 0) / 4294967296;
  };
}

export async function createWorld() {
  const physics = await Physics.create();
  const scene = new THREE.Scene();
  const field = new Field(scene, physics);
  field.build();
  const fuel = new FuelManager(physics, scene, field);
  return { physics, scene, field, fuel };
}

// side: { strategy, skill, robot, brain }. Side A drives the "player" slot, side B the opponent.
export function matchSettings(a, b, allianceA = BLUE) {
  const plan = opponentAuto(a.strategy);
  return {
    alliance: allianceA, robot: a.robot, climber: 'none', preload: 8, hp: 'auto',
    auto: plan.routine, start: plan.start, customSide: 'drawn',
    driver: a.strategy, driverSkill: a.skill, driverBrain: a.brain || null,
    opponent: b.strategy, oppRobot: b.robot, oppSkill: b.skill, oppBrain: b.brain || null,
  };
}

// Every match gets a fresh physics world: with 504 FUEL the simulation is chaotic, and state
// left over from an earlier match (contact caches, body order) would make identical inputs
// play out differently. Fresh world + seeded randomness = the same match every time.
export async function runMatch(a, b, { allianceA = BLUE, seed = 1 } = {}) {
  const realRandom = Math.random;
  const world = await createWorld();
  // seed after building the world: THREE draws Math.random for object UUIDs, and the field's
  // shared materials are only created on the first build in a process
  Math.random = mulberry32(seed);
  try {
    const game = createGame(world, matchSettings(a, b, allianceA));
    const m = game.match;
    let step = 0;
    while (!m.over && step < 30000) {
      stepGame(game, world, PHYSICS_DT);
      if (++step % 2 === 0) frameGame(game, 2 * PHYSICS_DT);
    }
    const A = allianceA, B = other(allianceA);
    const side = (al, r) => ({
      total: m.total(al), fuel: m.fuelPoints(al), autoFuel: m.score[al].autoFuel,
      fouls: m.score[al].fouls.map((f) => f.rule), foulPtsGiven: m.score[al].fouls.reduce((s, f) => s + f.pts, 0),
      shots: r.stats.shots, intaked: r.stats.intaked,
    });
    return { a: side(A, game.robot), b: side(B, game.opp.robot), margin: m.total(A) - m.total(B) };
  } finally {
    Math.random = realRandom;
    world.physics.world.free();
  }
}

export { BLUE, RED };
