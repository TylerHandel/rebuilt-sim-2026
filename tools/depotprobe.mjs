// Robots driven empty over the blue DEPOT's barriers (1.125in), the intake out, at a steady speed:
// out of the DEPOT from against the wall over its front barrier at an angle off square, and along the
// wall across a side barrier turned toward the field: across in time or held up on it (and how slow).
//   node --import ./tools/node-env.mjs tools/depotprobe.mjs [robots 1690,4414] [speeds 0.8,1.6,3]
import { createWorld, matchSettings, mulberry32 } from './headless.mjs';
import { createGame, stepGame } from '../js/game.js';
import { PHYSICS_DT, HALF_L, HALF_W, DEPOT } from '../js/constants.js';
import { wrapAngle } from '../js/util.js';

const keys = (process.argv[2] ?? '2910,4414,1678,1690,341,1706,581,1114,604,6328,7769,2056,8793').split(',');
const speeds = (process.argv[3] ?? '0.8,1.6,3').split(',').map(Number);
const IDLE = { vx: 0, vz: 0, omega: 0, intake: false, shoot: false, pass: false, outtake: false, lower: false };
const barX = -HALF_L + DEPOT.depth - DEPOT.barrierW / 2, zc = HALF_W - DEPOT.fy;
for (const key of keys) {
  const out = [];
  for (const sp of speeds) {
    for (const ang of [0, 30, 60]) {
      for (const mode of ['out', 'side']) { // out of the DEPOT from against the wall; across a side barrier
        const world = await createWorld();
        Math.random = mulberry32(1);
        const game = createGame(world, matchSettings({ strategy: 'scorer', skill: 'champs', robot: key }, { strategy: 'scorer', skill: 'champs', robot: '2910' }));
        for (const u of game.units) { u.external = true; u.robot.cmd = { ...IDLE }; }
        let step = 0;
        const run = (n, each) => { for (let i = 0; i < n; i++) { if (each && each()) return; stepGame(game, world, PHYSICS_DT); step++; } };
        while (!game.match.isTeleop && step < 30000) run(1);
        const [A, B] = game.units.map((u) => u.robot);
        B.spawn(6, 0, 0);
        for (const b of world.fuel.balls) b.body.setEnabled(false);
        // out: from its back to the wall in the DEPOT, out at ang off square; side: along the wall
        // across the DEPOT's side barrier, turned ang toward the field
        const a = (ang * Math.PI) / 180;
        const inX = -HALF_L + Math.max(A.halfL, A.halfW) + 0.05;
        const [x0, z0, vx, vz] = mode === 'out' ? [inX, zc, Math.cos(a) * sp, Math.sin(a) * sp]
          : [inX + 0.1, zc - DEPOT.width / 2 - 1.0, Math.sin(a) * sp, Math.cos(a) * sp];
        const yaw = Math.atan2(-vz, vx);
        A.spawn(x0, z0, yaw);
        A.forceDeploy = true;
        run(Math.round(1.0 / PHYSICS_DT), () => { A.cmd = { ...IDLE }; });
        let t = 0, minV = Infinity, crossed = null;
        run(Math.round(4 / PHYSICS_DT), () => {
          t += PHYSICS_DT;
          A.cmd = { ...IDLE, vx, vz, omega: 5 * wrapAngle(yaw - A.yaw) };
          const v = Math.hypot(A.vel.x, A.vel.z);
          if (t > 0.5 && crossed == null) minV = Math.min(minV, v);
          if (crossed == null && (mode === 'out' ? A.pos.x - barX > 1.0 : A.pos.z - zc > DEPOT.width / 2 + 0.6)) crossed = t;
          return crossed != null;
        });
        out.push(`${sp} ${ang}° ${mode}: ${crossed != null ? crossed.toFixed(1) + 's' : 'HELD'} (${minV.toFixed(2)})`);
        world.physics.world.free();
      }
    }
  }
  console.log(`${key}: ${out.join(' | ')}`);
}
