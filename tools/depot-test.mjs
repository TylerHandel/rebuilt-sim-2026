// DEPOT entry: each robot drives straight into the BLUE DEPOT (head on, across its front bar) with
// its intake running; prints how far in it got (its front past the front bar's outside edge), the
// FUEL it took and the DEPOT FUEL pushed out of the DEPOT (outside its bars, not taken)
//   node --import ./tools/node-env.mjs tools/depot-test.mjs [speed m/s] [--robots 2910,1678] [--time s] [--settle s] [--offset dz]
import { createWorld, mulberry32 } from './headless.mjs';
import { Robot } from '../js/robot.js';
import { Match } from '../js/match.js';
import { ROBOTS } from '../js/robotConfigs.js';
import { PHYSICS_DT, BLUE, HALF_L, HALF_W, DEPOT } from '../js/constants.js';

const speed = +(process.argv[2] || 1.5);
const arg = (name, def) => { const i = process.argv.indexOf(name); return i > 0 ? process.argv[i + 1] : def; };
const T = +arg('--time', 3), SETTLE = +arg('--settle', 1), OFF = +arg('--offset', 0);
const x0 = -HALF_L, x1 = -HALF_L + DEPOT.depth, zc = HALF_W - DEPOT.fy;
const inDepot = (p) => p.x > x0 && p.x < x1 && Math.abs(p.z - zc) < DEPOT.width / 2;
for (const key of arg('--robots', '2910,1678,4414,6328,8793').split(',')) {
  const world = await createWorld();
  Math.random = mulberry32(7);
  const { physics, scene, field, fuel } = world;
  const match = new Match({ playerAlliance: BLUE, onEvent: () => {} });
  fuel.match = match;
  const r = new Robot({ cfg: ROBOTS[key], alliance: BLUE, physics, scene, field, fuel, match, climber: null });
  r.spawn(x1 + r.halfL + 0.6, zc + OFF, Math.PI); // facing the DEPOT (-x)
  fuel.stage(0);
  const pre = r.stored.length;
  const depotFuel = fuel.balls.filter((b) => b.state === 'field' && inDepot(b.pos));
  if (process.env.NOFUEL) for (const b of depotFuel) fuel._disable(b, 'off');
  r.enabled = true;
  let t = 0, minFront = Infinity;
  const step = (cmd) => {
    Object.assign(r.cmd, { ...cmd, intake: true, shoot: false, pass: false, outtake: false });
    r.preStep(PHYSICS_DT); fuel.preStep(PHYSICS_DT); physics.step(); t += PHYSICS_DT; fuel.postStep(PHYSICS_DT, t); r.postStep(PHYSICS_DT, t);
    if (Math.round(t / PHYSICS_DT) % 2) fuel.update(2 * PHYSICS_DT, t);
  };
  for (let i = 0; i < SETTLE / PHYSICS_DT; i++) step({ vx: 0, vz: 0, omega: 0 });
  const curve = [];
  for (let i = 0; i < T / PHYSICS_DT; i++) {
    step({ vx: -speed, vz: 0, omega: 0 });
    minFront = Math.min(minFront, r.pos.x - r.halfL);
    if (process.env.CONTACTS && i === Math.round(T / PHYSICS_DT) - 1) {
      // what of the robot touches what, at the end
      const w = physics.world, seen = new Map();
      for (let k = 0; k < r.body.numColliders(); k++) {
        const c = r.body.collider(k);
        w.contactPairsWith(c, (o) => w.contactPair(c, o, (m) => {
          if (!m.numContacts()) return;
          const n = m.normal(), ob = o.parent();
          const what = ob === physics.fixedBody ? `field(${(o.collisionGroups() >>> 16).toString(16)}) at y ${o.translation().y.toFixed(3)}` : fuel.balls.some((b) => b.col === o) ? 'FUEL' : 'other';
          const key = `collider ${k} (groups ${(c.collisionGroups() >>> 16).toString(16)}) - ${what} n ${n.x.toFixed(2)},${n.y.toFixed(2)},${n.z.toFixed(2)}`;
          seen.set(key, (seen.get(key) ?? 0) + 1);
        }));
      }
      for (const [k, n] of seen) console.log('   ', k, n > 1 ? `x${n}` : '');
      console.log('    pitch', r.body.rotation().z.toFixed(3), 'y', r.body.translation().y.toFixed(3), 'wheels down', r._wheelsDown().join(','), 'deploy', r.intakeDeploy);
    }
    if ((i + 1) % Math.round(0.5 / PHYSICS_DT) === 0) curve.push(`${r.stored.length}@${(x1 - (r.pos.x - r.halfL)).toFixed(2)}`);
  }
  const taken = depotFuel.filter((b) => r.stored.includes(b)).length;
  const out = depotFuel.filter((b) => !r.stored.includes(b) && b.state === 'field' && !inDepot(b.pos)).length;
  console.log(`${key}: frame front ${(x1 - minFront).toFixed(2)} m past the DEPOT's front edge (of ${DEPOT.depth.toFixed(2)}); took ${r.stored.length - pre} (${taken} of the DEPOT's ${depotFuel.length}); ${out} pushed out; every 0.5 s count@depth ${curve.join(' ')}`);
}
