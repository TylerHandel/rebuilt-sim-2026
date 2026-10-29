// 4414 shooting tilted: parked up on its ALLIANCE's BUMP (its back wheels on the carpet in the
// ALLIANCE ZONE, its front up the BUMP's slope), firing volleys at the HUB with its tilt
// compensation (js/mech/phys4414.js: their getTiltCompensatedAim) on and off, and flat on the
// carpet beside the BUMP for comparison. How many go in, how far the misses land from the HUB's
// middle, and the chassis' pitch and roll while it fires.
//   node --import ./tools/node-env.mjs tools/tilt4414.mjs [--volleys n] [--cases pitch,roll,flat,...] [--z m]
// Cases: pitch (facing the HUB's way along the FIELD, driven forward up the slope: pitched nose up,
// the HUB off to its side), roll (facing across the FIELD, driven sideways up the slope: rolled),
// back (backed up the slope: nose down), flat (on the carpet short of the BUMP); each from --z (2.0
// m: the BUMP's middle, the HUB 1.9-2.0 m off; at 1.2, its near half, 1.2-1.6 m off). Each volley is 8 FUEL set in its preload
// slots; --volleys (4) of them, a fresh world each case and setting.
import { createWorld, mulberry32 } from './headless.mjs';
import { Robot } from '../js/robot.js';
import { Match } from '../js/match.js';
import { ROBOTS } from '../js/robotConfigs.js';
import { Field } from '../js/field.js';
import { PHYSICS_DT, BLUE } from '../js/constants.js';

const arg = (name, def) => { const i = process.argv.indexOf(name); return i > 0 ? process.argv[i + 1] : def; };
const VOLLEYS = +arg('--volleys', 4);
const CASES = arg('--cases', 'pitch,roll,back,flat').split(',');
const DEG = 180 / Math.PI;
// where each case parks: start (x, z, yaw), drive (field-relative vx, vz), stop once its center
// reaches x
const Z0 = +arg('--z', 2.0);
const CASE = {
  pitch: { x: -5.0, yaw: 0, v: [0.8, 0], stopX: -3.95 },
  back: { x: -5.0, yaw: Math.PI, v: [0.8, 0], stopX: -3.95 },
  roll: { x: -5.0, yaw: Math.PI / 2, v: [0.8, 0], stopX: -4.02 },
  flat: { x: -5.0, yaw: 0, v: [0.8, 0], stopX: -4.75 },
};

async function run(name, comp) {
  const c = CASE[name];
  const world = await createWorld();
  Math.random = mulberry32(5);
  const { physics, scene, field, fuel } = world;
  const match = new Match({ playerAlliance: BLUE, onEvent: () => {} });
  match.phase = 'teleop';
  fuel.match = match;
  const r = new Robot({ cfg: ROBOTS['4414'], alliance: BLUE, physics, scene, field, fuel, match, climber: null });
  r.spawn(c.x, Z0, c.yaw);
  fuel.stage(0);
  r.enabled = true;
  r.phys.tiltComp = comp;
  let t = 0;
  const step = (cmd) => {
    Object.assign(r.cmd, { vx: 0, vz: 0, omega: 0, intake: false, shoot: false, pass: false, outtake: false, ...cmd });
    r.preStep(PHYSICS_DT); fuel.preStep(PHYSICS_DT); physics.step(); t += PHYSICS_DT; fuel.postStep(PHYSICS_DT, t); r.postStep(PHYSICS_DT, t);
    fuel.update(PHYSICS_DT, t);
  };
  // drive up onto the BUMP (or to the spot), stop, settle
  for (let i = 0; i < 6 / PHYSICS_DT && r.pos.x < c.stopX; i++) step({ vx: c.v[0], vz: c.v[1] });
  for (let i = 0; i < 1.5 / PHYSICS_DT; i++) step({});
  const q = r.body.rotation();
  // (pitch: nose up about its own side axis; roll: its right side up)
  const fwd = [1 - 2 * (q.y * q.y + q.z * q.z), 2 * (q.x * q.y + q.w * q.z), 2 * (q.x * q.z - q.w * q.y)];
  const right = [2 * (q.x * q.z + q.w * q.y), 2 * (q.y * q.z - q.w * q.x), 1 - 2 * (q.x * q.x + q.y * q.y)];
  const pitch = Math.asin(fwd[1]) * DEG, roll = -Math.asin(right[1]) * DEG;
  const hub = Field.hubCenter(BLUE), dist = Math.hypot(hub.x - r.pos.x, hub.z - r.pos.z);
  // volleys: 8 FUEL into its preload slots from the far side of the NEUTRAL ZONE, fire them all
  const scored = new Set(), out = { shots: 0, in: 0, miss: [], tilt: [], comped: 0, steps: 0 };
  const watched = new Set();
  for (let v = 0; v < VOLLEYS; v++) {
    const balls = fuel.balls.filter((b) => b.state === 'field' && b.pos.x > 0.5 && b.pos.y < 0.2 && !watched.has(b)).slice(0, 8);
    balls.forEach((b) => watched.add(b));
    r.phys.load(balls);
    const s0 = r.stats.shots;
    let last = t;
    for (let i = 0; i < 8 / PHYSICS_DT; i++) {
      step({ shoot: true });
      if (r.stats.shots > s0 && r.phys.feedingNow) { out.steps++; if (r.phys.tiltComped) out.comped++; out.tilt.push(r.phys.tilt * DEG); }
      if (r.stats.shots !== (out._n ?? s0)) { out._n = r.stats.shots; last = t; }
      for (const b of watched) if (b.inHub) scored.add(b);
      if (r.stored.length === 0 && t - last > 2) break;
    }
    out.shots += r.stats.shots - s0;
    // (let the last ones land)
    for (let i = 0; i < 2 / PHYSICS_DT; i++) { step({}); for (const b of watched) if (b.inHub) scored.add(b); }
  }
  out.in = scored.size;
  // where the misses ended up (their distance from the HUB's middle, on the carpet)
  for (const b of watched) if (!scored.has(b) && b.state === 'field' && Math.hypot(b.pos.x - r.pos.x, b.pos.z - r.pos.z) > 0.8) out.miss.push(Math.hypot(b.pos.x - hub.x, b.pos.z - hub.z));
  physics.world.free();
  return { ...out, pitch, roll, dist, x: r.pos.x, z: r.pos.z };
}

for (const name of CASES) {
  for (const comp of [false, true]) {
    const o = await run(name, comp);
    const med = (a) => { const b = [...a].sort((p, q) => p - q); return b.length ? b[b.length >> 1] : NaN; };
    console.log(`4414 ${name.padEnd(5)} tilt comp ${comp ? 'on ' : 'off'}: at ${o.x.toFixed(2)},${o.z.toFixed(2)} (${o.dist.toFixed(2)} m from the HUB), pitch ${o.pitch.toFixed(1)} deg, roll ${o.roll.toFixed(1)} deg; ${o.in}/${o.shots} in the HUB${o.miss.length ? `, misses resting ${med(o.miss).toFixed(1)} m from its middle (median)` : ''}; compensating ${o.steps ? ((100 * o.comped) / o.steps).toFixed(0) : 0}% of the time it fed`);
  }
}
