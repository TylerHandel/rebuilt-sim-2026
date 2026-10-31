// 7769 driven under the TRENCH empty, half full and full (as tools/trench1690.mjs does 1690): does it
// fit (its config's height, the load's top under its net, fitsTrenchLowered) and does it get through
// without the arm touching it or its load? Each run sets n FUEL in its hopper (dropped in waves under
// the net, let up 0.2 m while they settle and brought back down over a second, then 3 s to settle;
// the intake out), then drives it forward across the blue left TRENCH on its lane at a steady speed.
//   node --import ./tools/node-env.mjs tools/trench7769.mjs [speed m/s]   (N=n: one load)
import { createWorld, matchSettings, mulberry32 } from './headless.mjs';
import { createGame, stepGame } from '../js/game.js';
import { PHYSICS_DT, FIELD_W, fw, TRENCH, FUEL } from '../js/constants.js';
import { TRENCH_ARMS } from '../js/nav.js';
import { wrapAngle } from '../js/util.js';
import { NET, SLIDE } from '../js/mech/r7769.js';

const IDLE = { vx: 0, vz: 0, omega: 0, intake: false, shoot: false, pass: false, outtake: false, lower: false };
const SPEED = Number(process.argv[2] ?? 1.5), R = FUEL.radius;
const world = await createWorld();
Math.random = mulberry32(1);
const game = createGame(world, matchSettings({ strategy: 'scorer', skill: 'champs', robot: '7769' }, { strategy: 'scorer', skill: 'champs', robot: '2910' }));
for (const u of game.units) { u.external = true; u.robot.cmd = { ...IDLE }; }
let step = 0;
const run = (n, each) => { for (let i = 0; i < n; i++) { stepGame(game, world, PHYSICS_DT); step++; if (each && each(i * PHYSICS_DT)) return; } };
while (!game.match.isTeleop && step < 30000) run(1);
const [A, ...rest] = game.units.map((u) => u.robot);
rest.forEach((r, i) => r.spawn(-7 + i, 0, 0));
const balls = world.fuel.balls;
for (const b of balls) b.body.setEnabled(false);
const lane = fw(3.2, FIELD_W - 0.64);
const beam = TRENCH_ARMS.reduce((a, b) => (Math.hypot(b.x - lane.x, b.z - lane.z) < Math.hypot(a.x - lane.x, a.z - lane.z) ? b : a));
const ph = A.phys;

function trial(n) {
  A.spawn(beam.x - 1.8, beam.z, 0);
  A.cmd = { ...IDLE };
  // (the intake out)
  ph.slideGoal = SLIDE.out;
  run(Math.round(1.0 / PHYSICS_DT), () => { A.cmd = { ...IDLE }; });
  const B = NET.bulge;
  NET.bulge = 0.2;
  const used = [];
  for (const b of balls) b.body.setEnabled(false);
  const local = (b) => { const p = b.body.translation(), dx = p.x - A.pos.x, dz = p.z - A.pos.z, c = Math.cos(A.yaw), s = Math.sin(A.yaw); return [dx * c - dz * s, p.y, dx * s + dz * c]; };
  for (let wave = 0; used.length < n && wave < 14; wave++) {
    const have = used.map(local), pts = [];
    for (let i = 0; i < 6 && used.length + pts.length < n; i++) for (let k = 0; k < 5 && used.length + pts.length < n; k++) {
      const x = Math.min(0.53, -0.03 + 0.152 * i + (wave % 2) * 0.076), z = -0.27 + 0.135 * k + (wave % 2) * 0.067;
      if (z > 0.28) continue;
      const y = 0.52 - R;
      if ([...have, ...pts].some((p) => Math.hypot(p[0] - x, p[1] - y, p[2] - z) < 2 * R + 0.003)) continue;
      pts.push([x, y, z]);
    }
    const c = Math.cos(A.yaw), s = Math.sin(A.yaw);
    for (const [x, y, z] of pts) {
      const b = balls[used.length];
      b.body.setEnabled(true);
      b.body.setTranslation({ x: A.pos.x + x * c + z * s, y, z: A.pos.z - x * s + z * c }, true);
      b.body.setLinvel({ x: 0, y: 0, z: 0 }, true);
      used.push(b);
    }
    run(Math.round(0.6 / PHYSICS_DT), () => { A.cmd = { ...IDLE }; });
  }
  run(Math.round(1 / PHYSICS_DT), (dt) => { A.cmd = { ...IDLE }; NET.bulge = 0.2 + (B - 0.2) * Math.min(1, dt); });
  NET.bulge = B;
  run(Math.round(3 / PHYSICS_DT), () => { A.cmd = { ...IDLE }; });
  const held0 = ph.held.length;
  const fits = A.fitsTrenchLowered(), top = A.topStowed, load = A.loadTop;
  let passed = null, t = 0, minV = Infinity, maxY = 0, armHit = 0;
  run(Math.round(6 / PHYSICS_DT), (dt) => {
    t = dt;
    A.cmd = { ...IDLE, vx: SPEED, omega: 5 * wrapAngle(0 - A.yaw) };
    const dx = A.pos.x - beam.x, v = Math.hypot(A.vel.x, A.vel.z);
    if (Math.abs(dx) < 0.6 && t > 0.3) {
      minV = Math.min(minV, v);
      for (const b of used) { const p = b.body.translation(); if (Math.abs(p.x - beam.x) < 0.1 && Math.abs(p.z - beam.z) < TRENCH.clearWidth / 2) { maxY = Math.max(maxY, p.y + R); if (p.y + R > TRENCH.clearHeight) armHit++; } }
    }
    if (passed === null && dx > 0.9) passed = t;
    return passed !== null && t > passed + 0.3;
  });
  const lost = used.filter((b) => { const p = b.body.translation(); return Math.hypot(p.x - A.pos.x, p.z - A.pos.z) > 0.9; }).length;
  console.log(`  ${n} FUEL wanted, ${used.length} put in: ${held0} held; its top ${top.toFixed(3)} m stowed (the load's ${load.toFixed(3)}), ${fits ? 'fits' : 'does NOT fit'} under the arm (${TRENCH.clearHeight.toFixed(3)} m); ` +
    `${passed !== null ? `through in ${passed.toFixed(2)} s, ${minV.toFixed(2)} m/s at its slowest under it` : `STUCK at ${(A.pos.x - beam.x).toFixed(2)} m from the arm`}; ` +
    `its FUEL under the arm up to ${maxY ? maxY.toFixed(3) : '-'} m (${armHit} steps a FUEL touched the arm's height); ${ph.held.length} held after, ${lost} lost`);
}

console.log(`7769 (height ${A.height.toFixed(3)} m) driven forward at ${SPEED} m/s under the blue left TRENCH:`);
for (const n of (process.env.N ? [Number(process.env.N)] : [0, 30, 55])) trial(n);
