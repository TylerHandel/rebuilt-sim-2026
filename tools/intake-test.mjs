// Controlled intake check: each robot drives straight through the NEUTRAL ZONE FUEL with its
// intake running; prints FUEL collected, the count every half second, and when the intake stalled
// against the load (full).
//   node --import ./tools/node-env.mjs tools/intake-test.mjs [speed m/s] [--along] [--time s] [--robots 2910,4414]
// --along drives down the length of the FUEL line (the most FUEL per meter) instead of across it
import { createWorld, mulberry32 } from './headless.mjs';
import { Robot } from '../js/robot.js';
import { Match } from '../js/match.js';
import { ROBOTS, ROBOT_ORDER } from '../js/robotConfigs.js';
import { PHYSICS_DT, BLUE } from '../js/constants.js';

const speed = +(process.argv[2] || 1.5);
const along = process.argv.includes('--along');
const arg = (name, def) => { const i = process.argv.indexOf(name); return i > 0 ? process.argv[i + 1] : def; };
const T = +arg('--time', 4);
for (const key of arg('--robots', ROBOT_ORDER.join(',')).split(',')) {
  for (const z of along ? [0] : [-1.5, 0.4]) {
    const world = await createWorld();
    Math.random = mulberry32(7);
    const { physics, scene, field, fuel } = world;
    const match = new Match({ playerAlliance: BLUE, onEvent: () => {} });
    fuel.match = match;
    const r = new Robot({ cfg: ROBOTS[key], alliance: BLUE, physics, scene, field, fuel, match, climber: null });
    if (along) r.spawn(-0.45, 3.4, Math.PI / 2); // facing down the line (-z)
    else r.spawn(-2.4, z, 0);
    fuel.stage(0);
    r.enabled = true;
    let t = 0, first = null, full = null; const curve = [];
    for (let i = 0; i < T / PHYSICS_DT; i++) {
      Object.assign(r.cmd, { vx: along ? 0 : speed, vz: along ? -speed : 0, omega: 0, intake: true, shoot: false, pass: false, outtake: false });
      r.preStep(PHYSICS_DT);
      fuel.preStep(PHYSICS_DT);
      physics.step();
      t += PHYSICS_DT;
      fuel.postStep(PHYSICS_DT, t);
      r.postStep(PHYSICS_DT, t);
      if (i % 2) fuel.update(2 * PHYSICS_DT, t);
      if (first === null && r.stored.length) first = t;
      if (full === null && r.full) full = t;
      if ((i + 1) % Math.round(0.5 / PHYSICS_DT) === 0) curve.push(r.stored.length);
    }
    console.log(`${key} z=${z}: collected ${r.stored.length}/${r.capacity()} (in rollers ${r.captured?.length}, dropped ${r.stats.dropped}), first after ${first?.toFixed(2)}s, ${full ? `stalled after ${full.toFixed(2)}s` : 'not stalled'}, every 0.5 s ${curve.join(' ')}, ended at x=${r.pos.x.toFixed(2)} z=${r.pos.z.toFixed(2)}`);
    physics.world.free();
  }
}
