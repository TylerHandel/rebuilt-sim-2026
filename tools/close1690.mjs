// 1690 shooting at its HUB from close in, in the game: parked d m from the HUB's center (on the line to
// its ALLIANCE WALL), N FUEL in its hopper, shooting until they're gone. Where each comes down through
// the rim's height (along the line from the robot to the HUB's center: + past the center, - short of
// it), and how many score (dropped out of the bottom of the funnel). The shot maps behind it:
// js/mech/phys1690.js (tools/mechlab1690.mjs calibrate).
//   node --import ./tools/node-env.mjs tools/close1690.mjs [d ...]   (N=12, SEC=5)
import { createWorld, matchSettings, mulberry32 } from './headless.mjs';
import { createGame, stepGame } from '../js/game.js';
import { PHYSICS_DT, HUB, FUEL } from '../js/constants.js';
import { Field } from '../js/field.js';
import { NET, netY, SHOOTER } from '../js/mech/r1690.js';
import { elevationAt, exitAt } from '../js/mech/phys1690.js';

const IDLE = { vx: 0, vz: 0, omega: 0, intake: false, shoot: false, pass: false, outtake: false, lower: false };
const R = FUEL.radius, N = Number(process.env.N ?? 12), SEC = Number(process.env.SEC ?? 5);
const DS = process.argv.slice(2).map(Number);
const world = await createWorld();
Math.random = mulberry32(1);
const game = createGame(world, matchSettings({ strategy: 'scorer', skill: 'champs', robot: '1690' }, { strategy: 'scorer', skill: 'champs', robot: '2910' }));
for (const u of game.units) { u.external = true; u.robot.cmd = { ...IDLE }; }
const run = (n, each) => { for (let i = 0; i < n; i++) { stepGame(game, world, PHYSICS_DT); if (each && each(i * PHYSICS_DT)) return; } };
while (!game.match.isTeleop) run(1);
const [A, ...rest] = game.units.map((u) => u.robot);
rest.forEach((r, i) => r.spawn(6 + i, 3, 0));
const balls = world.fuel.balls, c = Field.hubCenter(A.alliance), toward = Math.sign(c.x) * -1 > 0 ? 1 : -1;
for (const b of balls) b.body.setEnabled(false);

function trial(d) {
  for (const b of balls) b.body.setEnabled(false);
  // (the robot on the far side of the HUB from... its own ALLIANCE WALL's side: x toward the wall)
  const x0 = c.x - toward * d;
  A.spawn(x0, c.z, toward > 0 ? 0 : Math.PI);
  A.cmd = { ...IDLE };
  run(Math.round(0.6 / PHYSICS_DT));
  const used = [];
  for (let i = 0; i < N; i++) {
    const lx = 0.2 - 0.15 * Math.floor(i / 4), lz = -0.22 + 0.15 * (i % 4), ly = Math.min(netY(lx, lz), NET.y) - R - 0.01 - 0.15 * Math.floor(i / 8);
    const b = balls[i], cy = Math.cos(A.yaw), sy = Math.sin(A.yaw);
    b.body.setEnabled(true);
    b.body.setTranslation({ x: A.pos.x + lx * cy + lz * sy, y: ly, z: A.pos.z - lx * sy + lz * cy }, true);
    b.body.setLinvel({ x: 0, y: 0, z: 0 }, true);
    used.push(b);
  }
  run(Math.round(1 / PHYSICS_DT), () => { A.cmd = { ...IDLE }; });
  const prev = new Map(), down = [], scored = new Set();
  run(Math.round(SEC / PHYSICS_DT), () => {
    // (MOVE: driving toward the HUB at MOVE m/s while it shoots, STRAFE across, until 1 m from it)
    const dd = Math.abs(A.pos.x - c.x), mv = dd > 1.0 ? Number(process.env.MOVE ?? 0) : 0;
    A.cmd = { ...IDLE, shoot: true, vx: mv * (toward > 0 ? 1 : -1) * (toward > 0 ? 1 : 1), vz: Number(process.env.STRAFE ?? 0) };
    for (const b of used) {
      const p = b.body.translation(), v = b.body.linvel(), was = prev.get(b);
      prev.set(b, { y: p.y, out: was?.out });
      if (process.env.TRACE && !was?.out && p.y > 0.75 && v.y > 2) {
        prev.get(b).out = true;
        const ph = A.phys, w = ph.want, rv = A.body.linvel();
        console.log(`   left: d ${Math.abs(A.pos.x - c.x).toFixed(2)} robot v ${(rv.x * toward).toFixed(2)}; wanted ${w ? w.v.toFixed(2) + ' m/s at ' + w.el.toFixed(1) + ' (d ' + w.d.toFixed(2) + ')' : '-'}; hood at ${elevationAt(ph.hood).toFixed(1)} (goal ${elevationAt(ph.hoodGoal).toFixed(1)}), wheel for ${exitAt(Math.abs(ph.t.wheel.omega) * SHOOTER.r).toFixed(2)} m/s; out (world) ${Math.hypot(v.x, v.y, v.z).toFixed(2)} m/s at ${(Math.atan2(v.y, Math.hypot(v.x, v.z)) * 180 / Math.PI).toFixed(1)}, rel ${Math.hypot(v.x - rv.x, v.y, v.z - rv.z).toFixed(2)} at ${(Math.atan2(v.y, Math.hypot(v.x - rv.x, v.z - rv.z)) * 180 / Math.PI).toFixed(1)}`);
      }
      if (was && was.y >= HUB.rimFront && p.y < HUB.rimFront && v.y < 0 && !down.some((q) => q.b === b)) down.push({ b, along: (p.x - c.x) * toward, side: p.z - c.z });
      if (Math.abs(p.x - c.x) < HUB.size / 2 && Math.abs(p.z - c.z) < HUB.size / 2 && p.y < HUB.funnelBottomY && p.y > 0.5) scored.add(b);
    }
  });
  const al = down.map((q) => q.along).sort((a, b) => a - b), med = al.length ? al[al.length >> 1] : NaN;
  console.log(`  ${d} m: ${down.length} of ${used.length} came down through the rim's height, ${scored.size} scored; along the line (+ past the center) ${al.map((v) => (v >= 0 ? '+' : '') + v.toFixed(2)).join(' ')} (median ${med >= 0 ? '+' : ''}${med.toFixed(2)})`);
}
console.log(`1690 parked d m from its HUB's center, ${N} FUEL in it, shooting ${SEC} s:`);
for (const d of DS.length ? DS : [1.2, 1.5, 1.8, 2.1, 2.5, 3]) trial(d);
