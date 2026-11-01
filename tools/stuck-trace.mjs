// A robot's stuck moments in an AI match (scorer, champs): every 0.25 s while it's asked to move
// (over 1 m/s) and isn't (under 0.2 m/s), and 1 s either side: where it is and faces, its AI's label,
// the FUEL it's going for, the next path point, what it's asked for and what's around it.
//   node --import ./tools/node-env.mjs tools/stuck-trace.mjs <robot> [seed] [opponent 2910] [--all]
// --all prints every 0.5 s of TELEOP instead.
import { runMatch } from './headless.mjs';
import { PHYSICS_DT } from '../js/constants.js';

const pos = process.argv.slice(2).filter((a) => !a.startsWith('--'));
const key = pos[0] ?? '1690', seed = +(pos[1] ?? 1), opp = pos[2] ?? '2910', all = process.argv.includes('--all');
// what the robot's colliders are in contact with: FUEL (count), robots, and the field (where)
function touching(game, r) {
  const world = r.physics.world, ballOf = new Map(game.fuel.balls.map((b) => [b.col.handle, b]));
  const robotOf = new Map();
  for (const o of game.robots) for (let i = 0; i < o.body.numColliders(); i++) robotOf.set(o.body.collider(i).handle, o);
  let fuel = 0;
  const out = new Set();
  for (let i = 0; i < r.body.numColliders(); i++) {
    const c = r.body.collider(i);
    world.contactPairsWith(c, (o) => {
      let n = 0, pt = null;
      world.contactPair(c, o, (man) => {
        for (let k = 0; k < man.numContacts(); k++) if (man.contactDist(k) < 0.005) n++;
        if (man.numSolverContacts() > 0) pt = man.solverContactPoint(0);
      });
      if (!n) return;
      if (ballOf.has(o.handle)) { fuel++; return; }
      const ro = robotOf.get(o.handle);
      if (ro) { if (ro !== r) out.add(`robot ${ro.cfg.key}`); return; }
      if (pt && pt.y < 0.005) return; // the carpet
      const t = pt || o.translation();
      out.add(`${c === r.body.collider(0) ? '' : 'c' + i + ':'}field(${t.x.toFixed(2)},${t.y.toFixed(2)},${t.z.toFixed(2)})`);
    });
  }
  return `${fuel} FUEL ${[...out].join(' ')}`;
}
const hist = [];
let after = 0, nextAll = 0;
const f2 = (v) => v.toFixed(2);
await runMatch({ strategy: 'scorer', skill: 'champs', robot: key }, { strategy: 'scorer', skill: 'champs', robot: opp }, {
  seed,
  trace(game, step) {
    if (step % 30 || !game.match.isTeleop) return;
    const r = game.robot, u = game.units.find((x) => x.robot === r), ai = u.ai, c = r.cmd, m = game.match;
    const want = Math.hypot(c.vx, c.vz), have = Math.hypot(r.vel.x, r.vel.z);
    const o = game.opp.robot;
    const b = ai.ball;
    const nx = ai.path?.[0];
    const line = `${f2(m.phaseTime).padStart(6)} pos ${f2(r.pos.x)},${f2(r.pos.z)} yaw ${(r.yaw * 57.3).toFixed(0)} v ${f2(have)} cmd ${f2(c.vx)},${f2(c.vz)} w ${f2(c.omega)} in ${c.intake ? 1 : 0}/${f2(r.intakeDeploy)} sh ${c.shoot ? 1 : 0} ps ${c.pass ? 1 : 0} held ${r.stored.length} | ${ai.state} "${ai.label}" ball ${b ? `${f2(b.pos.x)},${f2(b.pos.z)}` : '-'} next ${nx ? `${f2(nx.x)},${f2(nx.z)}` : '-'} goal ${ai.goal ? `${f2(ai.goal.x)},${f2(ai.goal.z)}` : '-'} | opp ${f2(o.pos.x)},${f2(o.pos.z)} tilt ${f2(r.traction ?? 1)}`;
    const stuck = want > 1 && have < 0.2;
    if (all) { if (m.phaseTime >= nextAll) { console.log(line); nextAll += 0.5; } return; }
    if (stuck) {
      if (after <= 0) { console.log('--'); for (const h of hist) console.log('  ' + h); }
      console.log('* ' + line + ' | touching ' + touching(game, r));
      after = 4;
    } else if (after > 0) { console.log('  ' + line); after--; }
    hist.push(line); if (hist.length > 4) hist.shift();
  },
});
