// Every one of a team's own AUTOs (js/teamAutos.js) played headless, AUTO only: the AUTO FUEL each
// scores (and what it shot and still holds at the end), from its own side, alone on the field or
// against an opponent's AUTO.
//   node --import ./tools/node-env.mjs tools/team-auto-eval.mjs 604 [--opp 1678] [--seeds 1,2] [--only double-cycle] [--track]
// (--only: a piece of the AUTOs' ids; --track: the most the robot fell off its trajectory's
// setpoint, in position and heading, while it followed one; --contacts: loose FUEL against the
// robot's own simplified colliders, where and what became of them, tools/fuel-contacts.mjs)
import { createWorld, mulberry32 } from './headless.mjs';
import { createGame, stepGame, frameGame } from '../js/game.js';
import { TEAM_AUTOS } from '../js/teamAutos.js';
import { PHYSICS_DT, BLUE, RED } from '../js/constants.js';
import { wrapAngle } from '../js/util.js';
import { contactLog } from './fuel-contacts.mjs';

const args = process.argv.slice(2);
const arg = (k, d) => { const i = args.indexOf('--' + k); return i >= 0 ? args[i + 1] : d; };
const track = args.includes('--track'), contacts = args.includes('--contacts');
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
    const m = game.match, r = game.robot, runner = game.units.find((u) => u.robot === r)?.auto;
    const clog = contacts ? contactLog(r, world.fuel, world.physics) : null;
    let step = 0, ePos = 0, eYaw = 0, eAt = 0, eYawAt = 0;
    while (!m.isTeleop && m.phase !== 'done' && step < 6000) {
      stepGame(game, world, PHYSICS_DT);
      clog?.step(step * PHYSICS_DT);
      if (++step % 2 === 0) frameGame(game, 2 * PHYSICS_DT);
      // (--track: how far the robot is from its trajectory's setpoint, while it follows one; with
      // TRACKLOG=1 in the environment, every 0.05 s of it)
      const sp = runner?.chain && runner.setpoint;
      if (track && sp) {
        const e = Math.hypot(sp.x - r.pos.x, sp.z - r.pos.z), y = Math.abs(wrapAngle(sp.yaw - r.yaw)) * 180 / Math.PI;
        if (e > ePos) { ePos = e; eAt = runner.elapsed; }
        if (y > eYaw) { eYaw = y; eYawAt = runner.elapsed; }
        if (process.env.TRACKLOG && step % 6 === 0) console.log(`    ${runner.elapsed.toFixed(2)} pos ${e.toFixed(2)} yaw ${y.toFixed(1)} sp ${(sp.yaw * 180 / Math.PI).toFixed(0)} v ${Math.hypot(r.vel.x, r.vel.z).toFixed(2)} spv ${Math.hypot(sp.vx, sp.vz).toFixed(2)} w ${(r.omega * 180 / Math.PI).toFixed(0)} cmd ${r.cmd.vx.toFixed(2)},${r.cmd.vz.toFixed(2)},${r.cmd.omega.toFixed(2)} in ${r.cmd.intake ? 1 : 0}/${r.intakeDeploy?.toFixed(1)} held ${r.stored.length} fx ${(r.pos.x + 8.27).toFixed(2)} fy ${(4.035 - r.pos.z).toFixed(2)}`);
      }
    }
    if (clog) { clog.finish(step * PHYSICS_DT); console.log(`    ${auto.name}, seed ${seed}:\n${clog.report()}`); }
    return { ePos, eYaw, eAt, eYawAt, fuel: m.score[BLUE].autoFuel, shots: r.stats.shots, passes: r.stats.passes, intaked: r.stats.intaked, left: r.stored.length, fouls: m.score[BLUE].fouls.map((f) => f.rule), opp: m.score[RED].autoFuel };
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
  console.log(`  ${auto.name}: ${avg('fuel')} AUTO FUEL (${runs.map((x) => x.fuel).join(', ')}); launched ${avg('shots')} (${avg('passes')} passes), took in ${avg('intaked')}, holding ${avg('left')} at the end${runs.some((x) => x.fouls.length) ? `; fouls ${runs.flatMap((x) => x.fouls).join(' ')}` : ''}${opp ? `; ${opp} ${avg('opp')}` : ''}${track ? `; off its setpoint by up to ${runs.map((x) => `${x.ePos.toFixed(2)} m (${x.eAt.toFixed(1)} s) / ${x.eYaw.toFixed(0)} deg (${x.eYawAt.toFixed(1)} s)`).join(', ')}` : ''}`);
}
