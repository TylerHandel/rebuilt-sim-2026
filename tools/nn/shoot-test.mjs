// Shooting accuracy in the full game, for calibrating the GPU simulator (tools/nn/gpusim.py).
// A loaded robot drives around its ALLIANCE ZONE at a fixed fraction of its top speed with the
// shoot button held, during AUTO (both HUBS active, no human player throws). Prints, per robot and
// speed: FUEL fired per second and the share that scored, and writes them as JSON.
//   node --import ./tools/node-env.mjs tools/nn/shoot-test.mjs [--robots 2910,4414] [--trials 6]
//        [--speeds 0,0.33,0.66,1] [--time 6] [--out tools/nn/shoot-calib.json]
import fs from 'node:fs';
import { createWorld, mulberry32 } from '../headless.mjs';
import { createGame, stepGame, frameGame } from '../../js/game.js';
import { ROBOTS, ROBOT_ORDER } from '../../js/robotConfigs.js';
import { PHYSICS_DT, BLUE, RED } from '../../js/constants.js';

const args = process.argv.slice(2);
const arg = (k, d) => { const i = args.indexOf('--' + k); return i >= 0 ? args[i + 1] : d; };
const robots = arg('robots', ROBOT_ORDER.join(',')).split(',');
const trials = +arg('trials', 6);
const speeds = arg('speeds', '0,0.33,0.66,1').split(',').map(Number);
const T = +arg('time', 6);
const OUT = arg('out', null);

const LINE = -4.24206, X0 = -7.3, X1 = LINE - 0.55, ZL = 3.2; // where it drives (blue ALLIANCE ZONE)

async function trial(key, frac, seed) {
  const rnd = mulberry32(seed);
  Math.random = mulberry32(seed + 99);
  const world = await createWorld();
  try {
    const slots = [{ driver: 'external', robot: key, auto: 'none', start: 'hub', skill: 'champs' },
      ...Array.from({ length: 5 }, () => ({ driver: 'empty', robot: key }))];
    const game = createGame(world, { matchMode: '3v3', slots, alliance: BLUE, preload: 8, hp: 'auto', climber: 'none' });
    for (const a of [BLUE, RED]) game.hps[a].auto = false;
    const r = game.robot, m = game.match, d = r.cfg.drive;
    // start somewhere in the zone, loaded up
    const x = X0 + (X1 - X0) * rnd(), z = (rnd() * 2 - 1) * ZL;
    r.spawn(x, z, rnd() * 2 * Math.PI);
    const extra = Math.max(0, Math.min(40, Math.floor(r.capacity())) - r.stored.length);
    const pool = world.fuel.balls.filter((b) => b.state === 'field').slice(0, extra);
    if (pool.length) r.loadFuel(pool);
    const loaded = r.stored.length;
    let th = rnd() * 2 * Math.PI, step = 0, shooting = 0;
    const sp = frac * d.maxSpeed;
    let pts0 = null, shots0 = 0, firstShot = null;
    while (m.phase !== 'done') {
      const auto = m.phase === 'auto';
      if (auto && pts0 === null) { pts0 = m.score.blue.autoFuel; shots0 = r.stats.shots; }
      const on = auto && m.phaseTime < T;
      // bounce around inside the zone
      if (r.pos.x < X0 && Math.cos(th) < 0) th = Math.PI - th;
      if (r.pos.x > X1 && Math.cos(th) > 0) th = Math.PI - th;
      if (Math.abs(r.pos.z) > ZL && Math.sign(-Math.sin(th)) === Math.sign(r.pos.z)) th = -th;
      Object.assign(r.cmd, { vx: on ? sp * Math.cos(th) : 0, vz: on ? -sp * Math.sin(th) : 0, omega: 0, intake: false, shoot: on, pass: false, outtake: false });
      stepGame(game, world, PHYSICS_DT);
      if (++step % 2 === 0) frameGame(game, 2 * PHYSICS_DT);
      if (on) shooting += PHYSICS_DT;
      if (firstShot === null && pts0 !== null && r.stats.shots > shots0) firstShot = m.phaseTime;
      if (m.phase === 'auto' && m.phaseTime > T + 3) break; // the last shots have landed
    }
    return { shots: r.stats.shots - shots0, pts: m.score.blue.autoFuel - pts0, loaded, time: shooting, firstShot };
  } finally {
    world.physics.world.free();
  }
}

const res = {};
for (const key of robots) {
  res[key] = {};
  for (const f of speeds) {
    let shots = 0, pts = 0, time = 0, loaded = 0, first = [];
    for (let i = 0; i < trials; i++) {
      const t = await trial(key, f, 1000 * i + Math.round(f * 100) + key.length);
      shots += t.shots; pts += t.pts; time += t.time; loaded += t.loaded;
      if (t.firstShot !== null) first.push(t.firstShot);
    }
    const acc = shots ? pts / shots : null;
    res[key][f] = { shots, pts, acc, rate: shots / time, loaded: loaded / trials, firstShot: first.length ? first.reduce((a, b) => a + b, 0) / first.length : null };
    console.log(`${key} speed ${(f * 100).toFixed(0).padStart(3)}%: ${shots} shots (${(shots / time).toFixed(1)}/s, ${(loaded / trials).toFixed(0)} loaded), scored ${pts} = ${acc === null ? '-' : (acc * 100).toFixed(0) + '%'}, first shot after ${res[key][f].firstShot?.toFixed(2) ?? '-'} s`);
  }
}
if (OUT) fs.writeFileSync(OUT, JSON.stringify(res, null, 1));
