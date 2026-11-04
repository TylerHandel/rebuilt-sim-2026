// Wall drive: a robot pushed diagonally into the side guardrail at full stick (30/45/60 degrees
// off the wall), its speed along the wall once steady, against the command's component along it
//   node --import ./tools/node-env.mjs tools/walldrive.mjs [robots: 2910,1678]
import { createWorld, matchSettings, mulberry32 } from './headless.mjs';
import { createGame, stepGame } from '../js/game.js';
import { PHYSICS_DT, HALF_W } from '../js/constants.js';

const IDLE = { vx: 0, vz: 0, omega: 0, intake: false, shoot: false, pass: false, outtake: false };
for (const robot of (process.argv[2] || '2910,1678').split(',')) {
  const world = await createWorld();
  Math.random = mulberry32(1);
  const game = createGame(world, matchSettings({ strategy: 'scorer', skill: 'champs', robot }, { strategy: 'scorer', skill: 'champs', robot }));
  for (const u of game.units) { u.external = true; u.robot.cmd = { ...IDLE }; }
  const run = (n, each) => { for (let i = 0; i < n; i++) { stepGame(game, world, PHYSICS_DT); if (each) each(i); } };
  let step = 0; while (!game.match.isTeleop && step++ < 30000) run(1);
  const [A, ...rest] = game.units.map((u) => u.robot);
  rest.forEach((r, i) => r.spawn(-7 + i, 2 + i * 0.01, 0));
  for (const b of world.fuel.balls) b.body.setEnabled(false);
  const top = A.cfg.drive.maxSpeed, out = [];
  for (const deg of [0, 30, 45, 60]) {
    const a = (deg * Math.PI) / 180;
    // against the -z guardrail, starting just off it, facing +x
    A.spawn(-2.5, -HALF_W + A.halfL + 0.2, 0); A.cmd = { ...IDLE }; run(30);
    A.cmd = { ...IDLE, vx: top * Math.cos(a), vz: -top * Math.sin(a) };
    let sum = 0, n = 0;
    run(240, (i) => { if (i >= 120) { sum += A.body.linvel().x; n++; } });
    out.push(`${deg} deg: ${(sum / n).toFixed(2)} m/s along the wall (command's component ${(top * Math.cos(a)).toFixed(2)}, ${((sum / n) / (top * Math.cos(a)) * 100).toFixed(0)}%)`);
  }
  console.log(`${robot} (top ${top.toFixed(2)} m/s):\n  ` + out.join('\n  '));
}
