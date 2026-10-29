// Long passes: each robot, BLUE, alone on an empty FIELD (no NEUTRAL ZONE FUEL), placed `d` m from
// its ALLIANCE WALL with 8 FUEL, told to pass (cmd.pass, as the driver's pass button does) for a few
// seconds. What it does: whether it fires, its exit speed and hood when it does, and where its FUEL
// come down (first touch of the carpet) against where it aimed (the pass target). It intakes (on the
// empty carpet) for --intake s first, as it would have been doing in a match.
//   node --import ./tools/node-env.mjs tools/pass-range.mjs [2910,1678,...] [--d 10,13,15] [--z 2.4] [--t 6] [--mode hub] [--intake 1.5] [--vmax 9]
// (--vmax: every robot's shooter.speedMax set to that, to try the shared out-of-reach path on a
// generic robot, 971's, 4946's or 4930's, that otherwise reaches anywhere on the FIELD)
// (PASSLOG=1 in the environment: every shot, its speed and climb, and where each FUEL came down)
// (--mode hub: shoot at the HUB instead, from inside the ALLIANCE ZONE: --d 2,3; the target's then the
// HUB's center, and 'lands at' is where the FUEL first touch something, the HUB's funnel mostly)
import { createWorld, mulberry32 } from './headless.mjs';
import { createGame, stepGame, frameGame } from '../js/game.js';
import { PHYSICS_DT, BLUE, HALF_L, FUEL } from '../js/constants.js';
import { DEG } from '../js/util.js';
import { ROBOTS } from '../js/robotConfigs.js';

const args = process.argv.slice(2);
const arg = (k, d) => { const i = args.indexOf('--' + k); return i >= 0 ? args[i + 1] : d; };
const robots = (args[0] && !args[0].startsWith('--') ? args[0] : '2910,1678,4414,604,1114,1690,8793,971,1706,581').split(',');
const dists = arg('d', '10,13,15').split(',').map(Number);
const zPos = Number(arg('z', '2.4')), T = Number(arg('t', '6')), hub = arg('mode', 'pass') === 'hub';
const R = FUEL.radius;
const med = (a) => { if (!a.length) return NaN; const s = [...a].sort((x, y) => x - y); return s[s.length >> 1]; };

async function run(id, d) {
  const realRandom = Math.random, vmax = arg('vmax', null), was = ROBOTS[id].shooter.speedMax;
  if (vmax) ROBOTS[id].shooter.speedMax = Number(vmax);
  const world = await createWorld();
  Math.random = mulberry32(1);
  try {
    const game = createGame(world, {
      alliance: BLUE, robot: id, climber: 'none', preload: 0, hp: 'auto', auto: 'none', start: 'leftBump', customSide: 'drawn',
      driver: 'external', driverSkill: 'champs', driverBrain: null, opponent: 'off', oppRobot: '1678', oppSkill: 'champs', oppBrain: null,
    });
    const m = game.match, r = game.robot, fuel = world.fuel;
    // an empty FIELD: every FUEL off it
    for (const b of fuel.balls) if (b.state === 'field') fuel._disable(b, 'off');
    m._decideShifts();
    m._go('teleop');
    const x = -HALF_L + d;
    // facing home: its shooter (or its turret, at the middle of its travel) pointing about where the
    // pass goes, so what's measured is the range, not the turn (2910's fires out its back)
    const mid = r.turrets.length === 1 ? (r.turrets[0].center ?? 0) * DEG : 0;
    r.spawn(x, zPos, hub ? 0 : Math.PI + (r.aimOffset || 0) - mid);
    const load = fuel.balls.filter((b) => b.state === 'off').slice(0, 8);
    r.loadFuel(load);
    // (one whose code shuttles whatever it picks up, 8793's: it does that as it intakes, the FUEL
    // coming up its path pushing on the ones ahead; its preload alone sticks in its throat. So it
    // creeps toward home for 3 s, intake down, over a row of 10 more laid out ahead of it, and it's
    // measured from the start: it shuttles its preload as soon as it's out of its zone)
    const shuttler = !!r.phys?.autoShuttle, creep = shuttler ? -0.3 : 0;
    if (shuttler) {
      const row = fuel.balls.filter((b) => b.state === 'off').slice(0, 10);
      row.forEach((b, k) => fuel._place(b, x - r.halfL - 0.35 - 0.22 * k, R + 0.01, zPos + (k % 2 ? 0.1 : -0.1)));
    }
    // as in a match, it's been intaking (its intake out, whatever that does to its ball path) up to
    // the moment it's told to pass
    const pre = shuttler ? 0 : Math.round(Number(arg('intake', '1.5')) / PHYSICS_DT);
    for (let i = 0; i < pre; i++) {
      Object.assign(r.cmd, { vx: creep, vz: 0, omega: 0, intake: true, shoot: false, pass: false, outtake: false, prespin: false });
      stepGame(game, world, PHYSICS_DT);
      if (i % 2) frameGame(game, 2 * PHYSICS_DT);
    }
    const start = r.stored.length, shots0 = r.stats.shots;
    const seen = new Map(), lands = [], fires = [];
    let tgt = null, status = new Map(), tFirst = null, ex = null;
    const steps = Math.round((T + 2.5) / PHYSICS_DT); // (and 2.5 s for the last to land)
    for (let i = 0; i < steps; i++) {
      Object.assign(r.cmd, { vx: i * PHYSICS_DT < 3 ? creep : 0, vz: 0, omega: 0, intake: shuttler, shoot: hub && i * PHYSICS_DT < T, pass: !hub && i * PHYSICS_DT < T, outtake: false, prespin: false });
      stepGame(game, world, PHYSICS_DT);
      if (i % 2) frameGame(game, 2 * PHYSICS_DT);
      if (r.shot?.target) tgt = r.shot.target;
      status.set(r.status, (status.get(r.status) || 0) + 1);
      for (const b of fuel.balls) {
        if (b.state !== 'field' || !b.launch || b.launch.by !== 'robot') continue;
        let s = seen.get(b);
        if (!s && b.body.linvel().y < 1) continue; // (spilled or dropped, not shot)
        if (!s) {
          s = { top: b.pos.y, done: false };
          seen.set(b, s);
          // (the FUEL's own speed and climb as it leaves, and what the robot thinks its exit speed is)
          const lv = b.body.linvel(), h = Math.hypot(lv.x, lv.z);
          fires.push({ v: r.flywheel, hood: r.hoodDeg, t: m.t, bv: Math.hypot(h, lv.y), bel: Math.atan2(lv.y, h) / DEG });
          tFirst ??= m.t;
          ex ??= { x: b.pos.x, z: b.pos.z };
        }
        s.top = Math.max(s.top, b.pos.y);
        // (its first touch of anything once it's well up: the carpet, mostly)
        if (!s.done && s.top > 0.4 && (!b.inFlight || b.pos.y < R + 0.03)) { s.done = true; lands.push({ x: b.pos.x, z: b.pos.z, y: b.pos.y }); }
      }
    }
    const fired = r.stats.shots - shots0;
    if (process.env.PASSLOG) for (const [k, f] of fires.entries()) console.log(`  ${id} ${d} m: shot ${k} at ${f.t.toFixed(2)} s, ${f.bv.toFixed(1)} m/s @${f.bel.toFixed(0)} (robot: ${f.v.toFixed(1)} @${f.hood.toFixed(0)}), lands ${lands[k] ? (lands[k].x + HALF_L).toFixed(1) + ", " + lands[k].z.toFixed(1) + " (y " + lands[k].y.toFixed(2) + ")" : "-"}`);
    ex ??= r.pos;
    let short = NaN, lat = NaN, landD = NaN, tD = NaN;
    if (tgt && lands.length) {
      const ux = tgt.x - ex.x, uz = tgt.z - ex.z, L = Math.hypot(ux, uz);
      tD = L;
      const along = lands.map((p) => ((p.x - ex.x) * ux + (p.z - ex.z) * uz) / L);
      const side = lands.map((p) => ((p.x - ex.x) * -uz + (p.z - ex.z) * ux) / L);
      landD = med(along); short = L - landD; lat = med(side);
    } else if (tgt) tD = Math.hypot(tgt.x - ex.x, tgt.z - ex.z);
    const topStatus = [...status].sort((a, b) => b[1] - a[1]).slice(0, 2).map(([s, n]) => `${s} ${(n * PHYSICS_DT).toFixed(1)}s`).join(', ');
    return {
      id, d, fired, of: start, left: r.stored.length, v: med(fires.map((f) => f.v)), hood: med(fires.map((f) => f.hood)), bv: med(fires.map((f) => f.bv)), bel: med(fires.map((f) => f.bel)), first: tFirst,
      tD, landD, short, lat, landX: med(lands.map((p) => p.x + HALF_L)), tgtX: tgt ? tgt.x + HALF_L : NaN, topStatus,
    };
  } finally {
    Math.random = realRandom;
    ROBOTS[id].shooter.speedMax = was;
  }
}

const f = (v, n = 1) => (Number.isFinite(v) ? v.toFixed(n) : '-');
console.log('robot  from wall  shots (held, left)  exit m/s  hood  FUEL m/s @deg  target dist  lands at  short by  aside   target (m from wall)  status');
for (const id of robots) {
  for (const d of dists) {
    const o = await run(id, d);
    console.log(`${id.padEnd(6)} ${String(d).padStart(5)} m   ${String(o.fired).padStart(2)} (${o.of}, ${o.left})   ${f(o.v).padStart(6)}   ${f(o.hood, 0).padStart(4)}   ${f(o.bv).padStart(5)} @${f(o.bel, 0).padStart(2)}   ${f(o.tD).padStart(8)} m  ${f(o.landD).padStart(6)} m  ${f(o.short).padStart(6)} m  ${f(o.lat).padStart(5)} m   x ${f(o.tgtX)} / lands x ${f(o.landX)}   ${o.topStatus}`);
  }
}
