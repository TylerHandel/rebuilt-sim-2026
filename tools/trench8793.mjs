// 8793 driven under the TRENCH with its intake arm up or partly up: the TRENCH arm pushing the
// backdrivable arm down (js/mech/phys8793.js trenchArm), or, driven in forward with it stowed,
// stopping the robot. Each run starts 1.4 m short of the blue left TRENCH's arm on its lane, facing
// the way its AUTO does (180: driving out backward, the arm trailing) or forward, and drives at a
// steady speed command across to the NEUTRAL ZONE side, with the arm commanded up (their loop
// holding it on its upper stop) or down (their TrenchMode: swinging down from where it is).
//   node --import ./tools/node-env.mjs tools/trench8793.mjs
import { createWorld, matchSettings, mulberry32 } from './headless.mjs';
import { createGame, stepGame } from '../js/game.js';
import { PHYSICS_DT, FIELD_W, fw } from '../js/constants.js';
import { TRENCH_ARMS } from '../js/nav.js';
import { wrapAngle } from '../js/util.js';

const IDLE = { vx: 0, vz: 0, omega: 0, intake: false, shoot: false, pass: false, outtake: false, lower: false };
const world = await createWorld();
Math.random = mulberry32(1);
const game = createGame(world, matchSettings({ strategy: 'scorer', skill: 'champs', robot: '8793' }, { strategy: 'scorer', skill: 'champs', robot: '1678' }));
for (const u of game.units) { u.external = true; u.robot.cmd = { ...IDLE }; }
let step = 0;
const run = (n, each) => { for (let i = 0; i < n; i++) { stepGame(game, world, PHYSICS_DT); step++; if (each && each(i * PHYSICS_DT)) return; } };
while (!game.match.isTeleop && step < 30000) run(1);
const [A, ...rest] = game.units.map((u) => u.robot);
rest.forEach((r, i) => r.spawn(-7 + i, 0, 0));
for (const b of world.fuel.balls) b.body.setEnabled(false);
const lane = fw(3.2, FIELD_W - 0.64);
const beam = TRENCH_ARMS.reduce((a, b) => (Math.hypot(b.x - lane.x, b.z - lane.z) < Math.hypot(a.x - lane.x, a.z - lane.z) ? b : a));
const ph = A.phys, DEG = Math.PI / 180;

function trial(name, { speed, back = true, rollDeg, down, from = 1.4 }) {
  const yaw = back ? Math.PI : 0;
  A.spawn(beam.x - from, beam.z, yaw);
  A.cmd = { ...IDLE };
  run(2);
  const st = ph.armSt;
  st.roll = rollDeg * DEG; st.set = null; ph.t.pivot._omega = 0;
  A.intakeDeploy = ph.deploy = 1 - st.roll / st.stow;
  let minV = Infinity, maxF = 0, rollAt = null, minRoll = Infinity, hard = false, t = 0, passed = null, under = [];
  run(Math.round(4 / PHYSICS_DT), (dt) => {
    t = dt;
    A.cmd = { ...IDLE, vx: speed, omega: 5 * wrapAngle(yaw - A.yaw), lower: !!down };
    const x = A.pos.x, v = Math.hypot(A.vel.x, A.vel.z), roll = st.roll / DEG;
    const nearBeam = Math.abs(x - beam.x) < 0.6;
    if (nearBeam && t > 0.3) { minV = Math.min(minV, v); under.push(v); }
    if (ph.trench?.F) { maxF = Math.max(maxF, ph.trench.F); if (rollAt === null) rollAt = roll; }
    if (ph.trench?.hard) hard = true;
    if (nearBeam) minRoll = Math.min(minRoll, roll);
    if (passed === null && x > beam.x + 0.8) passed = t;
    if (process.env.TRACE && (ph.trench?.F || ph.trench?.hard)) console.log(`    t ${t.toFixed(3)} dx ${(x - beam.x).toFixed(3)} v ${v.toFixed(2)} roll ${roll.toFixed(1)} F ${ph.trench.F.toFixed(0)} hard ${ph.trench.hard} lever ${ph.trench.lever?.toFixed(3)} n ${ph.trench.n} cmd ${ph.t.pivot.cmd?.toFixed(2)}`);
    return passed !== null && t > passed + 0.2;
  });
  const avg = under.length ? under.reduce((s, v) => s + v, 0) / under.length : 0;
  console.log(`  ${name}: ${passed !== null ? `through in ${passed.toFixed(2)} s` : `STUCK (at ${(A.pos.x - beam.x).toFixed(2)} m from the TRENCH arm after 4 s)`}; ` +
    `${minV < Infinity ? `${avg.toFixed(2)} m/s under the TRENCH arm (slowest ${minV.toFixed(2)})` : ''}; ` +
    `arm ${rollDeg} deg up at the start, ${rollAt === null ? 'not touched' : `met at ${rollAt.toFixed(0)} deg, pushed down to ${minRoll.toFixed(0)} deg (up to ${maxF.toFixed(0)} N on the robot)`}` +
    `${hard ? '; held at its upper stop: blocked' : ''}; ${(st.roll / DEG).toFixed(0)} deg at the end`);
}

console.log('8793 under the blue left TRENCH (its arm 0.565 m up; the intake arm clears it below 59 deg up):');
for (const speed of [1, 2, 3]) trial(`backward at ${speed} m/s, arm up and held up (stowed, 145 deg)`, { speed, rollDeg: 145, down: false });
for (const speed of [1, 3]) trial(`backward at ${speed} m/s, arm partly up (90 deg), held up`, { speed, rollDeg: 90, down: false, from: 0.25 });
for (const speed of [1, 3]) trial(`backward at ${speed} m/s, arm partly up (110 deg), commanded down`, { speed, rollDeg: 110, down: true, from: 0.1 });
trial('backward at 3 m/s from stowed, commanded down as it sets off (its AUTO)', { speed: 3, rollDeg: 145, down: true });
trial('forward at 1 m/s, arm stowed and held up (its front face into the TRENCH arm)', { speed: 1, back: false, rollDeg: 145, down: false });
trial('forward at 1 m/s, arm partly up (70 deg), commanded down', { speed: 1, back: false, rollDeg: 70, down: true });
