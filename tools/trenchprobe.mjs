// A robot driven empty under the blue left TRENCH's arm at a steady speed: through or stopped, how
// slow it got under the arm, and the contacts its colliders had with the arm (those the solver pushed
// on). KEY: the robot (6328); HEIGHT: its collider's height in m (its config's); SPEED (1.5 m/s);
// DIR=-1 the other way; FIXED=1 under the FIXED TRENCH (6in arm) instead of the HINGED one (4in);
// V=1 each contact; T=1 its height and tilt every 10 steps.
//   KEY=6328 HEIGHT=0.5715 SPEED=3 node --import ./tools/node-env.mjs tools/trenchprobe.mjs
import { createWorld, matchSettings, mulberry32 } from './headless.mjs';
import { ROBOTS } from '../js/robotConfigs.js';
import { createGame, stepGame } from '../js/game.js';
import { PHYSICS_DT, FIELD_W, fw, TRENCH, GROUP } from '../js/constants.js';
import { TRENCH_ARMS } from '../js/nav.js';
import { wrapAngle } from '../js/util.js';
const KEY = process.env.KEY ?? '6328', SPEED = Number(process.env.SPEED ?? 1.5);
if (process.env.HEIGHT) ROBOTS[KEY].height = Number(process.env.HEIGHT);
const IDLE = { vx: 0, vz: 0, omega: 0, intake: false, shoot: false, pass: false, outtake: false, lower: false };
const world = await createWorld();
Math.random = mulberry32(1);
const game = createGame(world, matchSettings({ strategy: 'scorer', skill: 'champs', robot: KEY }, { strategy: 'scorer', skill: 'champs', robot: '2910' }));
for (const u of game.units) { u.external = true; u.robot.cmd = { ...IDLE }; }
let step = 0;
const run = (n, each) => { for (let i = 0; i < n; i++) { if (each && each(i * PHYSICS_DT)) return; stepGame(game, world, PHYSICS_DT); step++; } };
while (!game.match.isTeleop && step < 30000) run(1);
const [A, ...rest] = game.units.map((u) => u.robot);
rest.forEach((r, i) => r.spawn(-7 + i, 0, 0));
for (const b of world.fuel.balls) b.body.setEnabled(false);
// (the blue left TRENCH, the HINGED one with its 4in arm; FIXED=1: the blue right one, its 6in arm)
const lane = fw(3.2, process.env.FIXED ? 0.64 : FIELD_W - 0.64);
const beam = TRENCH_ARMS.reduce((a, b) => (Math.hypot(b.x - lane.x, b.z - lane.z) < Math.hypot(a.x - lane.x, a.z - lane.z) ? b : a));
const W = world.physics.world;
const armCols = [];
W.forEachCollider((c) => { const g = c.collisionGroups() >>> 16; if (g & GROUP.TRENCH_ARM) armCols.push(c); });
const dir = Number(process.env.DIR ?? 1);
A.spawn(beam.x - dir * 1.8, beam.z, dir > 0 ? 0 : Math.PI);
run(Math.round(1.0 / PHYSICS_DT), () => { A.cmd = { ...IDLE }; });
console.log(`${KEY} height ${A.height.toFixed(4)} (clear ${TRENCH.clearHeight.toFixed(4)}), arm cols ${armCols.length}, body y ${A.body.translation().y.toFixed(4)}`);
let t = 0, passed = null, minV = Infinity, contacts = 0, pushed = 0, maxPitch = 0;
const nCol = A.body.numColliders();
run(Math.round(6 / PHYSICS_DT), (dt) => {
  t = dt;
  A.cmd = { ...IDLE, vx: dir * SPEED, omega: 5 * wrapAngle((dir > 0 ? 0 : Math.PI) - A.yaw) };
  const dx = (A.pos.x - beam.x) * dir, v = Math.hypot(A.vel.x, A.vel.z);
  const q = A.body.rotation(); const p = A.body.translation();
  const pitch = Math.asin(Math.max(-1, Math.min(1, 2 * (q.w * q.z - q.x * q.y))));
  const roll = Math.atan2(2 * (q.w * q.x + q.y * q.z), 1 - 2 * (q.x * q.x + q.z * q.z));
  if (Math.abs(dx) < 0.8) {
    if (Math.abs(dx) < 0.6) minV = Math.min(minV, v);
    maxPitch = Math.max(maxPitch, Math.abs(pitch), Math.abs(roll));
    for (let i = 0; i < nCol; i++) {
      const c = A.body.collider(i);
      for (const a of armCols) {
        W.contactPair(c, a, (m, flipped) => {
          for (let k = 0; k < m.numContacts(); k++) {
            const d = m.contactDist(k);
            contacts++;
            if (m.contactImpulse(k) > 0) pushed++;
            if (process.env.V && contacts < 40) console.log(`  t=${t.toFixed(3)} dx=${dx.toFixed(3)} col${i} shape=${c.shape.type} dist=${d.toFixed(4)} n=${JSON.stringify(m.normal())} imp=${m.contactImpulse(k).toFixed(3)} bodyY=${p.y.toFixed(4)} pitch=${(pitch * 180 / Math.PI).toFixed(2)} v=${v.toFixed(2)}`);
          }
        });
      }
    }
  }
  if (process.env.T && step % 10 === 0 && Math.abs(dx) < 1) console.log(`  t=${t.toFixed(2)} dx=${dx.toFixed(3)} y=${p.y.toFixed(4)} pitch=${(pitch * 180 / Math.PI).toFixed(2)} roll=${(roll * 180 / Math.PI).toFixed(2)} v=${v.toFixed(2)}`);
  if (passed === null && dx > 0.9) passed = t;
  return passed !== null && t > passed + 0.3;
});
console.log(`  ${passed !== null ? `through in ${passed.toFixed(2)} s, ${minV.toFixed(2)} m/s at its slowest under the arm` : `STUCK at ${((A.pos.x - beam.x) * dir).toFixed(2)} m`}, ${contacts} contact points with the arm, ${pushed} pushed on, max tilt ${(maxPitch * 180 / Math.PI).toFixed(2)} deg`);
