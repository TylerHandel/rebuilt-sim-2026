// Every one of a team's own AUTOs (js/teamAutos.js) played headless, AUTO only: the AUTO FUEL each
// scores (and what it shot and still holds at the end), from its own side, alone on the field or
// against an opponent's AUTO.
//   node --import ./tools/node-env.mjs tools/team-auto-eval.mjs 604 [--opp 1678] [--seeds 1,2] [--only double-cycle]
import { createWorld, mulberry32 } from './headless.mjs';
import { createGame, stepGame, frameGame } from '../js/game.js';
import { TEAM_AUTOS } from '../js/teamAutos.js';
import { PHYSICS_DT, BLUE, RED } from '../js/constants.js';

const args = process.argv.slice(2);
const arg = (k, d) => { const i = args.indexOf('--' + k); return i >= 0 ? args[i + 1] : d; };
const robot = args[0] || '604', opp = arg('opp', null), seeds = arg('seeds', '1').split(',').map(Number), only = arg('only', '');
const all = TEAM_AUTOS[robot] || [];
if (!all.length) { console.log(`no team autos for ${robot}`); process.exit(1); }

async function play(auto, seed) {
  const realRandom = Math.random;
  const world = await createWorld();
  Math.random = mulberry32(seed);
  // (only this auto on offer: the game picks it from the side it starts on)
  const saved = TEAM_AUTOS[robot];
  TEAM_AUTOS[robot] = [auto];
  try {
    const game = createGame(world, {
      alliance: BLUE, robot, climber: 'none', preload: 8, hp: 'auto',
      auto: 'best', start: auto.startFy > 4.03 ? 'leftBump' : 'rightBump', customSide: 'drawn',
      driver: 'scorer', driverSkill: 'champs', driverBrain: null,
      opponent: opp ? 'scorer' : 'off', oppRobot: opp || '1678', oppSkill: 'champs', oppBrain: null,
    });
    const m = game.match, r = game.robot;
    let step = 0;
    while (!m.isTeleop && m.phase !== 'done' && step < 6000) {
      stepGame(game, world, PHYSICS_DT);
      if (++step % 2 === 0) frameGame(game, 2 * PHYSICS_DT);
    }
    return { fuel: m.score[BLUE].autoFuel, shots: r.stats.shots, passes: r.stats.passes, intaked: r.stats.intaked, left: r.stored.length, fouls: m.score[BLUE].fouls.map((f) => f.rule), opp: m.score[RED].autoFuel };
  } finally {
    TEAM_AUTOS[robot] = saved;
    Math.random = realRandom;
    world.physics.world.free();
  }
}

console.log(`${robot}'s own AUTOs, ${opp ? `against ${opp}'s AUTO` : 'alone on the field'}, seeds ${seeds.join(', ')}:`);
for (const auto of all) {
  if (only && !auto.id.includes(only)) continue;
  const runs = [];
  for (const seed of seeds) runs.push(await play(auto, seed));
  const avg = (k) => (runs.reduce((s, x) => s + x[k], 0) / runs.length).toFixed(1);
  console.log(`  ${auto.name}: ${avg('fuel')} AUTO FUEL (${runs.map((x) => x.fuel).join(', ')}); launched ${avg('shots')} (${avg('passes')} passes), took in ${avg('intaked')}, holding ${avg('left')} at the end${runs.some((x) => x.fouls.length) ? `; fouls ${runs.flatMap((x) => x.fouls).join(' ')}` : ''}${opp ? `; ${opp} ${avg('opp')}` : ''}`);
}
