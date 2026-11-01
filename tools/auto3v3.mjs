// A 3 v 3's AUTO, robot by robot: where each starts and which AUTO it runs, then every second its
// position, step, what it holds, and at the end its AUTO FUEL (by its launches).
//   node --import ./tools/node-env.mjs tools/auto3v3.mjs 2910+4414+1690 1678+7769+2056 [seed]
import { createWorld, mulberry32 } from './headless.mjs';
import { createGame, stepGame, frameGame } from '../js/game.js';
import { PHYSICS_DT } from '../js/constants.js';

const [A = '2910+4414+1690', B = '1678+7769+2056', seed = '1'] = process.argv.slice(2);
const sl = (k) => k.split('+').map((robot) => ({ robot, auto: 'best', start: 'rightTrench', driver: 'scorer', skill: 'champs' }));
const world = await createWorld();
Math.random = mulberry32(+seed);
globalThis.SIM_SEED = +seed;
const game = createGame(world, { climber: 'none', preload: 8, hp: 'auto', matchMode: '3v3', slots: [...sl(A), ...sl(B)] });
const m = game.match;
const scored = new Map();
const orig = m.fuelEnteredHub.bind(m);
m.fuelEnteredHub = (a, ball, t) => { const rid = ball.launch?.rid; if (rid != null && m.isAuto) scored.set(rid, (scored.get(rid) || 0) + 1); orig(a, ball, t); };
for (const u of game.units) console.log(`${u.robot.cfg.key} ${u.robot.alliance} start ${u.entry.startKey} ${u.entry.team ? 'team AUTO ' + u.entry.team.name : u.entry.plan ? 'plan' : u.entry.auto} at ${u.robot.pos.x.toFixed(2)},${u.robot.pos.z.toFixed(2)}`);
let step = 0, next = 0;
while (m.phase !== 'autoGap' && step < 6000) {
  stepGame(game, world, PHYSICS_DT);
  if (++step % 2 === 0) frameGame(game, 2 * PHYSICS_DT);
  if (m.isAuto && m.phaseTime >= next) {
    next += 2;
    console.log(`${m.phaseTime.toFixed(0).padStart(3)}s ` + game.units.map((u) => `${u.robot.cfg.key} ${u.robot.pos.x.toFixed(1)},${u.robot.pos.z.toFixed(1)} s${u.auto.i}${u.auto.done ? 'D' : ''} h${u.robot.stored.length}`).join(' | '));
  }
}
console.log('AUTO FUEL: ' + game.units.map((u) => `${u.robot.cfg.key} ${scored.get(u.robot.uid) || 0}`).join(', ') + ` | fouls ${m.foulLog.map((f) => f.rule + ' ' + f.committedBy).join(' ') || 'none'}`);
