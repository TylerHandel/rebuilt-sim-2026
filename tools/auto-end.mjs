// When each robot's AUTO routine finishes (it then sits still until TELEOP), what it holds then and
// at the end of AUTO, and its AUTO FUEL: one AUTO against a champs 2910 per robot and seed.
//   node --import ./tools/node-env.mjs tools/auto-end.mjs [robots] [seeds 1]
import { createWorld, matchSettings, mulberry32 } from './headless.mjs';
import { createGame, stepGame, frameGame } from '../js/game.js';
import { PHYSICS_DT } from '../js/constants.js';

const keys = (process.argv[2] ?? '2910,4414,8793,1678,1690,341,1706,581,1114,604,6328,7769,2056').split(',');
const seeds = (process.argv[3] ?? '1').split(',').map(Number);
for (const key of keys) {
  const out = [];
  for (const seed of seeds) {
    const world = await createWorld();
    Math.random = mulberry32(seed);
    globalThis.SIM_SEED = seed;
    const game = createGame(world, matchSettings({ strategy: 'scorer', skill: 'champs', robot: key }, { strategy: 'scorer', skill: 'champs', robot: '2910' }));
    const u = game.units[0], m = game.match;
    let step = 0, doneAt = null, heldDone = 0;
    while (m.phase !== 'autoGap' && step < 5000) {
      stepGame(game, world, PHYSICS_DT);
      if (++step % 2 === 0) frameGame(game, 2 * PHYSICS_DT);
      if (m.isAuto && doneAt == null && u.auto.done) { doneAt = m.phaseTime; heldDone = u.robot.stored.length; }
    }
    out.push(`seed ${seed}: done ${doneAt == null ? '-' : doneAt.toFixed(1) + ' s'} holding ${heldDone}, at 20 s ${u.robot.stored.length}, AUTO FUEL ${m.score[u.robot.alliance].autoFuel}`);
    world.physics.world.free();
  }
  console.log(`${key.padEnd(5)} ${out.join(' | ')}`);
}
