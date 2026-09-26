// AUTO-only headless runs: how much AUTO FUEL a robot's plan scores against an opponent.
import { createWorld, mulberry32 } from './headless.mjs';
import { createGame, stepGame, frameGame } from '../js/game.js';
import { PHYSICS_DT, BLUE, RED } from '../js/constants.js';

// a/b: { robot, plan (optional, defaults to BEST_AUTOS) }. Returns AUTO FUEL for each side.
export async function runAuto(a, b, { seed = 1, allianceA = BLUE } = {}) {
  const realRandom = Math.random;
  const world = await createWorld();
  Math.random = mulberry32(seed);
  try {
    const settings = {
      alliance: allianceA, robot: a.robot, climber: 'none', preload: 8, hp: 'auto',
      auto: 'best', autoPlan: a.plan || null, start: 'rightTrench', customSide: 'drawn',
      driver: 'scorer', driverSkill: 'champs', driverBrain: null,
      opponent: b ? 'scorer' : 'off', oppRobot: b?.robot, oppSkill: 'champs', oppBrain: null, oppAutoPlan: b?.plan || null,
    };
    const game = createGame(world, settings);
    const m = game.match;
    let step = 0;
    while (!m.isTeleop && m.phase !== 'done' && step < 6000) {
      stepGame(game, world, PHYSICS_DT);
      if (++step % 2 === 0) frameGame(game, 2 * PHYSICS_DT);
    }
    const B = allianceA === BLUE ? RED : BLUE;
    const r = game.robot;
    return { a: m.score[allianceA].autoFuel, b: m.score[B].autoFuel, fouls: [...m.score[allianceA].fouls.map((f) => f.rule)], left: r.stored.length, shots: r.stats.shots, t: m.t };
  } finally {
    Math.random = realRandom;
    world.physics.world.free();
  }
}
