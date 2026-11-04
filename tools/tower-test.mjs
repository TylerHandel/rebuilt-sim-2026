// Behind the TOWER: each robot drives along the BLUE ALLIANCE WALL across the TOWER's base plate
// (behind its uprights, 0.12 m off the wall); prints its slowest speed over the plate, how high
// its body rose and how far it tipped, and what of the field it touched (TRACE=1: the contacts
// near the plate's edge; TRACE=2: its pose every 0.1 s)
//   node --import ./tools/node-env.mjs tools/tower-test.mjs [speed m/s] [--robots 2910,1678]
import { createWorld, mulberry32 } from './headless.mjs';
import { Robot } from '../js/robot.js';
import { Match } from '../js/match.js';
import { ROBOTS } from '../js/robotConfigs.js';
import { PHYSICS_DT, BLUE, HALF_L, HALF_W, TOWER } from '../js/constants.js';

const speed = +(process.argv[2] || 2.5);
const arg = (name, def) => { const i = process.argv.indexOf(name); return i > 0 ? process.argv[i + 1] : def; };
const zc = HALF_W - TOWER.fy, plateX = -HALF_L + TOWER.baseDepth;
for (const key of arg('--robots', '2910,1678,4414,6328').split(',')) {
  for (const run of ['along']) {
    const world = await createWorld();
    Math.random = mulberry32(7);
    const { physics, scene, field, fuel } = world;
    const match = new Match({ playerAlliance: BLUE, onEvent: () => {} });
    fuel.match = match;
    const r = new Robot({ cfg: ROBOTS[key], alliance: BLUE, physics, scene, field, fuel, match, climber: null });
    fuel.stage(0);
    for (const b of fuel.balls) b.body.setEnabled(false);
    const along = run === 'along';
    // from 1.6 m to one side of the TOWER to 1.6 m past it (facing +z)
    r.spawn(-HALF_L + r.halfL + 0.12, zc - 1.6, -Math.PI / 2);
    r.enabled = true;
    let t = 0, vmin = Infinity, ymax = 0, tip = 0, on = false;
    const hits = new Set();
    for (let i = 0; i < 3 / PHYSICS_DT; i++) {
      const cmd = along ? { vx: 0, vz: speed, omega: 0 } : { vx: -speed, vz: 0, omega: 0 };
      Object.assign(r.cmd, { ...cmd, intake: false, shoot: false, pass: false, outtake: false });
      r.preStep(PHYSICS_DT); fuel.preStep(PHYSICS_DT); physics.step(); t += PHYSICS_DT; fuel.postStep(PHYSICS_DT, t); r.postStep(PHYSICS_DT, t);
      const p = r.body.translation(), v = r.body.linvel(), q = r.body.rotation();
      const over = along ? Math.abs(p.z - zc) < TOWER.baseWidth / 2 + r.halfL : p.x - r.halfL < plateX + 0.1 && p.x - r.halfL > plateX - 0.6;
      if (t > 0.8 && over) {
        on = true;
        vmin = Math.min(vmin, Math.hypot(v.x, v.z));
        ymax = Math.max(ymax, p.y);
        tip = Math.max(tip, 2 * Math.asin(Math.min(1, Math.hypot(q.x, q.z))) * 57.3);
      }
      if (process.env.TRACE === "2" && i % 12 === 0) console.log(`  t ${t.toFixed(2)} x ${p.x.toFixed(3)} y ${p.y.toFixed(3)} z ${(p.z - zc).toFixed(3)} v ${Math.hypot(v.x, v.z).toFixed(2)} q ${q.x.toFixed(3)},${q.y.toFixed(3)},${q.z.toFixed(3)} down ${r._wheelsDown().map(Number).join('')}`);
      if (along && p.z > zc + 1.6) break;
      if (!along && p.x - r.halfL < plateX - 0.7) break;
      const w = physics.world;
      for (let k = 0; k < r.body.numColliders(); k++) {
        const c = r.body.collider(k);
        w.contactPairsWith(c, (o) => {
          if (o.parent() !== physics.fixedBody || o.translation().y < -0.4) return;
          w.contactPair(c, o, (m) => { if (process.env.TRACE && m.numSolverContacts() && t > 0.4 && t < 0.56) { const n = m.normal(); console.log(`    t ${t.toFixed(3)} collider ${k} - field ${(o.collisionGroups() >>> 16).toString(16)} shape ${o.shapeType()} n ${n.x.toFixed(2)},${n.y.toFixed(2)},${n.z.toFixed(2)} pt ${JSON.stringify(m.solverContactPoint(0))}`); } if (m.numContacts() && m.contactDist(0) < 0.001) hits.add(`collider ${k} (${(c.collisionGroups() >>> 16).toString(16)}) - field ${(o.collisionGroups() >>> 16).toString(16)} at ${o.translation().x.toFixed(2)},${o.translation().y.toFixed(3)},${o.translation().z.toFixed(2)}`); });
        });
      }
    }
    console.log(`${key} ${run}: ${on ? `slowest ${vmin.toFixed(2)} m/s of ${speed}, body up ${(ymax * 1000).toFixed(0)} mm, tipped ${tip.toFixed(1)} deg` : 'never over the plate'}; touched ${[...hits].join('; ') || 'nothing'}`);
  }
}
