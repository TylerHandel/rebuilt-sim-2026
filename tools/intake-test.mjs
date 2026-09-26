// Controlled intake check: each robot drives straight through the NEUTRAL ZONE FUEL with its
// intake running; prints FUEL collected and how long it took.
//   node --import ./tools/node-env.mjs tools/intake-test.mjs [speed m/s]
import { createWorld, mulberry32 } from './headless.mjs';
import { Robot } from '../js/robot.js';
import { Match } from '../js/match.js';
import { ROBOTS } from '../js/robotConfigs.js';
import { PHYSICS_DT, BLUE } from '../js/constants.js';

const speed = +(process.argv[2] || 1.5);
for (const key of ['2910', '4414', '8793']) {
  for (const z of [-1.5, 0.4]) {
    const world = await createWorld();
    Math.random = mulberry32(7);
    const { physics, scene, field, fuel } = world;
    const match = new Match({ playerAlliance: BLUE, onEvent: () => {} });
    fuel.match = match;
    const r = new Robot({ cfg: ROBOTS[key], alliance: BLUE, physics, scene, field, fuel, match, climber: null });
    r.spawn(-2.4, z, 0);
    fuel.stage(0);
    r.enabled = true;
    let t = 0, first = null;
    for (let i = 0; i < 4.0 / PHYSICS_DT; i++) {
      Object.assign(r.cmd, { vx: speed, vz: 0, omega: 0, intake: true, shoot: false, pass: false, outtake: false });
      r.preStep(PHYSICS_DT);
      fuel.preStep(PHYSICS_DT);
      physics.step();
      t += PHYSICS_DT;
      fuel.postStep(PHYSICS_DT, t);
      r.postStep(PHYSICS_DT, t);
      if (i % 2) fuel.update(2 * PHYSICS_DT, t);
      if (first === null && r.stored.length) first = t;
    }
    console.log(`${key} z=${z}: collected ${r.stored.length}/${r.capacity()} (in rollers ${r.captured?.length}, dropped ${r.stats.dropped}), first after ${first?.toFixed(2)}s, ended at x=${r.pos.x.toFixed(2)}`);
    physics.world.free();
  }
}
