// 2056 in an AI match (scorer, champs, against a champs 2910 scorer), its state each second: where it
// is, its AI's state and goal, how many FUEL it holds, the intake's extension, the shot (has one,
// ready, feeding), the wheel and hood, and what its drive is asked for. Run:
//   node --import ./tools/node-env.mjs tools/trace2056.mjs [seed]   (ROBOT=1690 for another)
import { runMatch } from './headless.mjs';
import { PHYSICS_DT } from '../js/constants.js';

const seed = Number(process.argv[2] ?? 1), key = process.env.ROBOT ?? '2056';
let next = 0;
const res = await runMatch({ strategy: 'scorer', skill: 'champs', robot: key }, { strategy: 'scorer', skill: 'champs', robot: '2910' }, {
  seed,
  trace(game, step) {
    const el = step * PHYSICS_DT;
    if (el < next) return;
    next = Math.floor(el) + 1;
    const r = game.robot, ph = r.phys, u = game.units?.find((x) => x.robot === r), ai = u?.ai ?? u?.brain ?? u;
    const c = r.cmd ?? {};
    console.log(`${el.toFixed(0).padStart(3)}s pos ${r.pos.x.toFixed(2)},${r.pos.z.toFixed(2)} v ${Math.hypot(r.vel?.x ?? 0, r.vel?.z ?? 0).toFixed(2)} | ai ${ai?.state ?? '-'}/${ai?.mode ?? '-'} goal ${ai?.goal ? `${ai.goal.x.toFixed(1)},${ai.goal.z.toFixed(1)}` : '-'} | cmd v ${(c.vx ?? 0).toFixed(1)},${(c.vz ?? 0).toFixed(1)} in ${c.intake ? 1 : 0} sh ${c.shoot ? 1 : 0} ps ${c.pass ? 1 : 0} out ${c.outtake ? 1 : 0}`
      + ` | held ${ph?.held?.length ?? r.stored?.length} ext ${(ph?.deploy ?? 0).toFixed(2)} shot ${r.shot?.target?.mode ?? '-'} want ${ph?.want ? 1 : 0} ready ${ph?.ready ? 1 : 0} feed ${ph?.feedingNow ? 1 : 0} acc ${ph?.accumulating ? 1 : 0} retr ${ph?.retracting ? 1 : 0} wheel ${(Math.abs(ph?.t?.wheel?.omega ?? 0) * 0.0508).toFixed(1)} hood ${(ph?.hood ?? 0).toFixed(0)} aimErr ${((r.aimErr ?? 0) * 57.3).toFixed(1)} unjams ${r.stats?.unjams ?? 0}`);
    // (WHERE=1: each held FUEL's place in the robot's frame)
    if (process.env.WHERE && ph?.held) console.log('      held at ' + ph.held.map((b) => ph._local(b.pos ?? b.translation()).map((v) => v.toFixed(2)).join(',')).join(' | '));
  },
});
console.log(JSON.stringify(res).slice(0, 300));
