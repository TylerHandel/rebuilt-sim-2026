// Neural-network training worker (one per CPU thread, started by tools/nn/train.py).
// Plays matches nonstop with the current policy, sampling its actions, and hands the
// experience to the trainer in chunks. The trainer updates the network on the GPU while the
// workers keep playing; each worker picks up the new weights between chunks.
//
// stdin  (JSON lines): {cmd:'config', policy, version, opponents:[...], robots:[...], chunk}
//                      {cmd:'stop'}
// stdout (JSON lines): {type:'ready'} {type:'chunk', file, n, version} {type:'episode', ...}
//                      {type:'error', msg}
// A chunk file holds float32: obs[n*D] act[n*A] logp[n] rew[n*R] done[n] nextObs[D].
import fs from 'node:fs';
import path from 'node:path';
import readline from 'node:readline';
import { createWorld } from '../headless.mjs';
import { createGame, stepGame, frameGame } from '../../js/game.js';
import { Policy } from '../../js/nn/policy.js';
import { buildObs, actionToCmd, OBS_DIM, ACT_DIM, DECISION_DT } from '../../js/nn/obs.js';
import { PHYSICS_DT, BLUE, RED, other } from '../../js/constants.js';
import { ROBOT_ORDER } from '../../js/robotConfigs.js';
import { START_ORDER } from '../../js/auto.js';

const args = process.argv.slice(2);
const arg = (k, d) => { const i = args.indexOf('--' + k); return i >= 0 ? args[i + 1] : d; };
const ID = +arg('id', 0);
const DIR = arg('dir', '.');
const REW = 5; // dOwnPts, dOppPts, dIntaked, dInactiveFuel, dFoulPtsGiven
const STEPS_PER_DECISION = Math.round(DECISION_DT / PHYSICS_DT);

const send = (o) => process.stdout.write(JSON.stringify(o) + '\n');

let config = null;
let pending = null; // newest config not yet applied (applied between chunks)
let stop = false;
readline.createInterface({ input: process.stdin }).on('line', (line) => {
  let m;
  try { m = JSON.parse(line); } catch { return; }
  if (m.cmd === 'config') pending = m;
  else if (m.cmd === 'stop') stop = true;
});
process.stdin.on('end', () => { stop = true; });

const policies = new Map(); // path -> { mtime, policy }
function loadPolicy(file) {
  const st = fs.statSync(file);
  const c = policies.get(file);
  if (c && c.mtime === st.mtimeMs) return c.policy;
  const policy = new Policy(JSON.parse(fs.readFileSync(file, 'utf8')));
  policies.set(file, { mtime: st.mtimeMs, policy });
  return policy;
}

let policy = null, version = -1;
function applyConfig() {
  if (!pending) return;
  config = pending;
  pending = null;
  if (config.version !== version) {
    policy = loadPolicy(config.policy);
    version = config.version;
  }
}

const yieldToIO = () => new Promise((r) => setImmediate(r));
const pick = (arr) => arr[Math.floor(Math.random() * arr.length)];
function pickWeighted(list) {
  const tot = list.reduce((s, o) => s + o.weight, 0);
  let u = Math.random() * tot;
  for (const o of list) { u -= o.weight; if (u <= 0) return o; }
  return list[list.length - 1];
}

// ------------------------------------------------------------------ experience buffer
const CHUNK_MAX = 4096;
const buf = {
  obs: new Float32Array(CHUNK_MAX * OBS_DIM), act: new Float32Array(CHUNK_MAX * ACT_DIM),
  logp: new Float32Array(CHUNK_MAX), rew: new Float32Array(CHUNK_MAX * REW), done: new Float32Array(CHUNK_MAX),
  n: 0, version: -1,
};
let chunkSeq = 0;
function flush(nextObs) {
  if (!buf.n) return;
  const n = buf.n;
  const parts = [buf.obs.subarray(0, n * OBS_DIM), buf.act.subarray(0, n * ACT_DIM), buf.logp.subarray(0, n), buf.rew.subarray(0, n * REW), buf.done.subarray(0, n), nextObs];
  const file = path.join(DIR, `w${ID}_${chunkSeq++}.bin`);
  const fd = fs.openSync(file + '.tmp', 'w');
  for (const p of parts) fs.writeSync(fd, Buffer.from(p.buffer, p.byteOffset, p.byteLength));
  fs.closeSync(fd);
  fs.renameSync(file + '.tmp', file);
  send({ type: 'chunk', file, n, version: buf.version, D: OBS_DIM, A: ACT_DIM, R: REW });
  buf.n = 0;
}

// ------------------------------------------------------------------ one match
async function playEpisode() {
  const opp = pickWeighted(config.opponents);
  const robotKey = pick(config.robots || ROBOT_ORDER);
  const alliance = Math.random() < 0.5 ? BLUE : RED;
  const settings = {
    alliance, robot: robotKey, climber: 'none', preload: 8, hp: 'auto',
    auto: 'none', start: pick(START_ORDER), customSide: 'drawn',
    driver: 'external',
    opponent: opp.kind === 'off' ? 'off' : opp.kind === 'nn' ? 'nn' : opp.strategy,
    oppRobot: opp.robot || pick(ROBOT_ORDER), oppSkill: opp.skill || 'regional',
    oppPolicy: opp.kind === 'nn' ? loadPolicy(opp.path) : null,
    oppStart: opp.kind === 'nn' ? pick(START_ORDER) : null,
  };
  const world = await createWorld();
  try {
    const game = createGame(world, settings);
    const m = game.match, me = game.robot, foe = game.opp ? game.opp.robot : null;
    const A = alliance, B = other(alliance);
    const obs = new Float32Array(OBS_DIM);
    let last = null; // { comps at decision }
    const snap = () => [m.total(A), m.total(B), me.stats.intaked, m.score[A].inactiveFuel, m.score[A].fouls.reduce((s, f) => s + f.pts, 0)];
    let since = STEPS_PER_DECISION, step = 0, decisions = 0;
    const record = (done) => {
      // finish the previous transition: its reward is everything that happened since
      const now = snap();
      const j = buf.n - 1;
      for (let k = 0; k < REW; k++) buf.rew[j * REW + k] = now[k] - last[k];
      buf.done[j] = done ? 1 : 0;
      last = now;
    };
    while (!m.over && step < 30000) {
      if (m.robotEnabled && since >= STEPS_PER_DECISION) {
        since = 0;
        buildObs(me, foe, m, world.fuel, obs);
        if (last) {
          record(false);
          if (buf.n >= config.chunk) {
            flush(obs);
            await yieldToIO();
            applyConfig();
            if (stop) return null;
          }
        } else last = snap();
        if (buf.n === 0) buf.version = version;
        const { action, logp } = policy.act(obs, { stochastic: true });
        const j = buf.n++;
        buf.obs.set(obs, j * OBS_DIM);
        buf.act.set(action, j * ACT_DIM);
        buf.logp[j] = logp;
        actionToCmd(me, action, me.cmd);
        decisions++;
      }
      stepGame(game, world, PHYSICS_DT);
      since++;
      if (++step % 2 === 0) frameGame(game, 2 * PHYSICS_DT);
    }
    if (last && buf.n) record(true);
    return {
      type: 'episode', id: ID, version, opp: opp.key, robot: robotKey, oppRobot: settings.oppRobot,
      own: m.total(A), them: m.total(B), margin: m.total(A) - m.total(B), fuel: m.fuelPoints(A),
      inactive: m.score[A].inactiveFuel, fouls: m.score[A].fouls.map((f) => f.rule),
      shots: me.stats.shots, passes: me.stats.passes, intaked: me.stats.intaked, decisions,
    };
  } finally {
    world.physics.world.free();
  }
}

// ------------------------------------------------------------------ main loop
send({ type: 'ready', id: ID, obsDim: OBS_DIM, actDim: ACT_DIM });
while (!config && !stop) { await new Promise((r) => setTimeout(r, 20)); applyConfig(); }
while (!stop) {
  if (!buf.n) applyConfig(); // a half-full chunk keeps the policy version it was started with
  try {
    const ep = await playEpisode();
    if (ep) send(ep);
  } catch (e) {
    buf.n = 0;
    send({ type: 'error', id: ID, msg: String((e && e.stack) || e) });
  }
  await yieldToIO();
}
process.exit(0);
