// Record the pre-programmed AIs driving, for the neural network to learn from (imitation): full
// matches in the real game between the Champs-level AIs (Scorer, Defense, Hybrid), 1v1 and 3v3,
// writing what each robot saw (the network's observation) and what it did (as network actions)
// every 0.1 s. Runs one match per CPU thread at a time until it has --samples decisions.
//   node --import ./tools/node-env.mjs tools/nn/record-ai.mjs [--out recordings-ai] [--samples 1000000]
//        [--workers N] [--skill champs] [--teams 1,3]
// Then: nn-train-gpu.bat --bc recordings-ai   (or train.py --bc recordings-ai)
// Each file: a uint32 header length, a JSON header, then obs as float16 [n*obsDim] and actions as
// float32 [n*actDim].
import fs from 'node:fs';
import os from 'node:os';
import path from 'node:path';
import { fork } from 'node:child_process';
import { fileURLToPath } from 'node:url';

const args = process.argv.slice(2);
const arg = (k, d) => { const i = args.indexOf('--' + k); return i >= 0 ? args[i + 1] : d; };
const OUT = arg('out', 'recordings-ai');
const SAMPLES = +arg('samples', 1000000);
const WORKERS = +arg('workers', Math.max(1, os.cpus().length - 1));
const SKILL = arg('skill', 'champs');
const TEAMS = arg('teams', '1,3').split(',').map(Number);
const PARENT = +arg('parent', 0); // the trainer that started it: stop when it's gone
const alive = (pid) => { try { process.kill(pid, 0); return true; } catch (e) { return e.code === 'EPERM'; } };

// float32 -> float16 bits (round to nearest; enough for observations, which are all small numbers)
const f32 = new Float32Array(1), u32 = new Uint32Array(f32.buffer);
function toHalf(v) {
  f32[0] = v;
  const x = u32[0], sign = (x >>> 16) & 0x8000, e = ((x >>> 23) & 0xff) - 127 + 15, m = x & 0x7fffff;
  if (e >= 31) return sign | 0x7c00;
  if (e <= 0) return e < -10 ? sign : sign | ((m | 0x800000) >> (1 - e + 13));
  return sign | (e << 10) | ((m + 0x1000) >> 13);
}

if (!args.includes('--worker')) {
  // ---------------------------------------------------------------- manager
  fs.mkdirSync(OUT, { recursive: true });
  let have = 0;
  for (const f of fs.readdirSync(OUT)) {
    if (!f.endsWith('.aidemo')) continue;
    const buf = fs.readFileSync(path.join(OUT, f));
    have += JSON.parse(buf.subarray(4, 4 + buf.readUInt32LE(0)).toString()).n;
  }
  console.log(`${have.toLocaleString()} decisions recorded already in ${OUT}; recording up to ${SAMPLES.toLocaleString()} with ${WORKERS} workers (Ctrl+C stops; what's saved stays).`);
  if (have >= SAMPLES) process.exit(0);
  const me = fileURLToPath(import.meta.url);
  const kids = [];
  if (PARENT) setInterval(() => { if (!alive(PARENT)) { for (const c of kids) c.kill(); process.exit(0); } }, 5000);
  const t0 = Date.now();
  let matches = 0;
  for (let i = 0; i < WORKERS; i++) {
    const k = fork(me, [...args, '--worker', '--id', String(i)], { execArgv: process.execArgv });
    k.on('message', (m) => {
      have += m.n;
      matches++;
      const rate = have / ((Date.now() - t0) / 1000);
      process.stdout.write(`\r${have.toLocaleString()} / ${SAMPLES.toLocaleString()} decisions, ${matches} matches (${m.teams}v${m.teams} ${m.who}: ${m.score})          `);
      if (have >= SAMPLES) { for (const c of kids) c.kill(); console.log('\nDone.'); process.exit(0); }
      void rate;
    });
    kids.push(k);
  }
} else {
  // ---------------------------------------------------------------- worker: one match after another
  const { createWorld } = await import('../headless.mjs');
  const { createGame, stepGame, frameGame } = await import('../../js/game.js');
  const { buildObs, cmdToAction, OBS_DIM, ACT_DIM, OBS_VERSION, DECISION_DT } = await import('../../js/nn/obs.js');
  const { PHYSICS_DT, BLUE, RED } = await import('../../js/constants.js');
  const { ROBOT_ORDER } = await import('../../js/robotConfigs.js');
  const { START_ORDER } = await import('../../js/auto.js');
  const id = arg('id', '0');
  const pick = (a) => a[Math.floor(Math.random() * a.length)];
  const strategy = () => { const u = Math.random(); return u < 0.6 ? 'scorer' : u < 0.85 ? 'hybrid' : 'defense'; };
  const every = Math.round(DECISION_DT / PHYSICS_DT);
  for (let n = 0; ; n++) {
    if (!alive(process.ppid) || (PARENT && !alive(PARENT))) process.exit(0);
    const k = pick(TEAMS);
    const slots = [];
    for (const a of [BLUE, RED]) {
      const st = [...START_ORDER].sort(() => Math.random() - 0.5);
      for (let i = 0; i < 3; i++) slots.push(i < k ? { driver: strategy(), robot: pick(ROBOT_ORDER), auto: 'best', start: st[i], skill: SKILL } : { driver: 'empty', robot: '2910' });
    }
    const world = await createWorld();
    const obs = [], act = [];
    let score = '';
    try {
      const game = createGame(world, { matchMode: '3v3', slots, alliance: BLUE, preload: 8, hp: 'auto', climber: 'none' });
      const m = game.match, robots = game.robots;
      const others = robots.map((r) => ({ foes: robots.filter((q) => q.alliance !== r.alliance), mates: robots.filter((q) => q.alliance === r.alliance && q !== r) }));
      const o = new Float32Array(OBS_DIM), a = new Float32Array(ACT_DIM);
      let step = 0;
      while (!m.over && step < 30000) {
        stepGame(game, world, PHYSICS_DT); // the AIs set their commands in here
        if (m.robotEnabled && step % every === 0) {
          robots.forEach((r, i) => {
            buildObs(r, others[i], m, world.fuel, o);
            cmdToAction(r, r.cmd, a);
            obs.push(Uint16Array.from(o, toHalf));
            act.push(Float32Array.from(a));
          });
        }
        if (++step % 2 === 0) frameGame(game, 2 * PHYSICS_DT);
      }
      score = `${m.total(BLUE)}-${m.total(RED)}`;
    } catch (e) {
      process.stderr.write(String((e && e.stack) || e) + '\n');
      continue;
    } finally {
      world.physics.world.free();
    }
    const N = obs.length;
    const head = Buffer.from(JSON.stringify({ format: 'rebuilt-nn-aidemo', obsVersion: OBS_VERSION, obsDim: OBS_DIM, actDim: ACT_DIM, n: N, skill: SKILL, teams: k }));
    const len = Buffer.alloc(4);
    len.writeUInt32LE(head.length);
    const ob = Buffer.alloc(N * OBS_DIM * 2), ab = Buffer.alloc(N * ACT_DIM * 4);
    obs.forEach((x, i) => Buffer.from(x.buffer).copy(ob, i * OBS_DIM * 2));
    act.forEach((x, i) => Buffer.from(x.buffer).copy(ab, i * ACT_DIM * 4));
    const file = path.join(OUT, `ai_${Date.now()}_${id}_${n}.aidemo`);
    fs.writeFileSync(file + '.tmp', Buffer.concat([len, head, ob, ab]));
    fs.renameSync(file + '.tmp', file);
    process.send({ n: N, teams: k, who: slots.filter((s) => s.driver !== 'empty').map((s) => s.driver[0].toUpperCase()).join(''), score });
  }
}
