// Drive lab: how robots move, on the real field with the game's own physics step. Run:
//   node --import ./tools/node-env.mjs tools/drivelab.mjs [robot]
// Off the line (time to 90% of top speed), a 90-degree change of direction and a reversal at full
// speed (how long until it's going the new way at half speed), spinning up, and pushing: two of
// the robot nose to nose both driving at each other, and one driving into another that just holds
// still (its drive commanded to stop).
import { createWorld, matchSettings, mulberry32 } from './headless.mjs';
import { createGame, stepGame } from '../js/game.js';
import { PHYSICS_DT } from '../js/constants.js';

const robot = process.argv[2] || '2910';
const IDLE = { vx: 0, vz: 0, omega: 0, intake: false, shoot: false, pass: false, outtake: false };
const world = await createWorld();
Math.random = mulberry32(1);
const game = createGame(world, matchSettings({ strategy: 'scorer', skill: 'champs', robot }, { strategy: 'scorer', skill: 'champs', robot }));
const units = game.units;
for (const u of units) { u.external = true; u.robot.cmd = { ...IDLE }; }
const m = game.match;
let step = 0;
const run = (n, each) => { for (let i = 0; i < n; i++) { stepGame(game, world, PHYSICS_DT); step++; if (each) each(i * PHYSICS_DT); } };
while (!m.isTeleop && step < 30000) run(1);
// park everyone else out of the way
const [A, B, ...rest] = units.map((u) => u.robot);
rest.forEach((r, i) => r.spawn(-7 + i, 3.6, 0));
const speed = (r) => Math.hypot(r.body.linvel().x, r.body.linvel().z);
const top = A.cfg.drive.maxSpeed;
const out = [];
// clear ground: the neutral zone with its FUEL taken away
for (const b of world.fuel.balls) b.body.setEnabled(false);
const X0 = -1.5, Z0 = 0;
function fresh(r, x = X0, z = Z0, yaw = 0) { r.spawn(x, z, yaw); r.cmd = { ...IDLE }; }
B.spawn(7, -3.6, 0);
fresh(A); run(60);
// off the line (across the field's width, the longest clear run)
{
  let t90 = null, vmax = 0;
  fresh(A, 0, -3.3); run(30);
  A.cmd = { ...IDLE, vz: top };
  run(240, (t) => { const v = speed(A); vmax = Math.max(vmax, v); if (t90 === null && v > 0.9 * top) t90 = t; });
  out.push(`off the line: 90% of ${top.toFixed(2)} m/s in ${t90?.toFixed(2) ?? '-'} s, up to ${vmax.toFixed(2)} m/s`);
}
// a change of direction at full speed (going +z): then (top, 0) [90 deg] or (0, -top) [reversal]
for (const [name, nx, nz] of [['90-degree turn', 1, 0], ['reversal', 0, -1]]) {
  fresh(A, nx ? -1.5 : 0, -3.3); run(30); A.cmd = { ...IDLE, vz: top }; run(150);
  A.cmd = { ...IDLE, vx: nx * top, vz: nz * top };
  let tHalf = null, tFull = null;
  run(180, (t) => {
    const v = A.body.linvel(), along = v.x * nx + v.z * nz;
    if (tHalf === null && along > 0.5 * top) tHalf = t;
    if (tFull === null && along > 0.9 * top) tFull = t;
  });
  out.push(`${name} at full speed: going the new way at half speed in ${tHalf?.toFixed(2) ?? '-'} s, 90% in ${tFull?.toFixed(2) ?? '-'} s`);
}
// spinning up
{
  fresh(A); run(30);
  const wmax = A.cfg.drive.maxOmega;
  A.cmd = { ...IDLE, omega: wmax };
  let t90 = null, wTop = 0;
  run(180, (t) => { const w = Math.abs(A.body.angvel().y); wTop = Math.max(wTop, w); if (t90 === null && w > 0.9 * wmax) t90 = t; });
  out.push(`spinning: 90% of ${wmax.toFixed(1)} rad/s in ${t90?.toFixed(2) ?? '-'} s, up to ${wTop.toFixed(1)} rad/s`);
}
// pushing: nose to nose, both at full; then one into a robot holding still
{
  const gap = A.halfL + B.halfL + 0.02 + 2 * 0.0826; // bumpers
  fresh(A, X0, Z0, 0); fresh(B, X0 + gap + 0.3, Z0, Math.PI); run(30);
  A.cmd = { ...IDLE, vx: top }; B.cmd = { ...IDLE, vx: -top };
  run(240);
  const ax = A.body.translation().x;
  run(120);
  out.push(`nose to nose, both at full: the pair moves ${(Math.abs(A.body.translation().x - ax) / 1).toFixed(2)} m/s, A drawing ${A.swerve ? A.swerve.current.toFixed(0) + ' A' : '?'}`);
  fresh(A, X0, Z0, 0); fresh(B, X0 + gap + 0.3, Z0, Math.PI); run(30);
  A.cmd = { ...IDLE, vx: top }; B.cmd = { ...IDLE };
  run(240);
  const bx = B.body.translation().x;
  run(120);
  out.push(`into a robot holding still: pushes it at ${(B.body.translation().x - bx).toFixed(2)} m/s`);
}
console.log(`${robot} (${A.cfg.mass} kg):\n  ` + out.join('\n  '));
