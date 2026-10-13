// Neural-network training worker (one per CPU thread, started by tools/nn/train.py).
// Plays full-game matches nonstop with the current policy, sampling its actions, and hands the
// experience to the trainer in chunks. The trainer updates the network on the GPU while the
// workers keep playing; each worker picks up the new weights between chunks.
// Every robot of the network's ALLIANCE is the network, and each one's experience is kept (its
// own stream: 3 in a 3v3). A network with memory (attn-gru) keeps one per robot, and each chunk
// carries the memory it started with, so the trainer can replay it.
//
// stdin  (JSON lines): {cmd:'config', policy, version, opponents:[...], robots:[...], chunk}
//                      {cmd:'stop'}
// stdout (JSON lines): {type:'ready'} {type:'chunk', file, n, version, G} {type:'episode', ...}
//                      {type:'error', msg}
// A chunk file holds float32: obs[n*D] act[n*A] logp[n] rew[n*R] done[n] nextObs[D] h0[G]
// (G = 0 for a network without memory). A chunk is one robot's decisions in one match.
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
const REW = 6; // dOwnPts, dOppPts, dIntaked, dInactiveFuel, dFoulPtsGiven, margin after (not a delta)
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

// ------------------------------------------------------------------ experience buffers
const CHUNK_MAX = 4096;
// one per robot the network drives in this match
function newStream(robot) {
  return {
    robot, obs: new Float32Array(CHUNK_MAX * OBS_DIM), act: new Float32Array(CHUNK_MAX * ACT_DIM),
    logp: new Float32Array(CHUNK_MAX), rew: new Float32Array(CHUNK_MAX * REW), done: new Float32Array(CHUNK_MAX),
    n: 0, version: -1, state: policy.newState(), h0: null, last: null, cur: new Float32Array(OBS_DIM),
  };
}
let chunkSeq = 0;
function flush(st, nextObs) {
  if (!st.n) return;
  const n = st.n;
  const h0 = st.h0 || new Float32Array(0);
  const parts = [st.obs.subarray(0, n * OBS_DIM), st.act.subarray(0, n * ACT_DIM), st.logp.subarray(0, n), st.rew.subarray(0, n * REW), st.done.subarray(0, n), nextObs, h0];
  const file = path.join(DIR, `w${ID}_${chunkSeq++}.bin`);
  const fd = fs.openSync(file + '.tmp', 'w');
  for (const p of parts) fs.writeSync(fd, Buffer.from(p.buffer, p.byteOffset, p.byteLength));
  fs.closeSync(fd);
  fs.renameSync(file + '.tmp', file);
  send({ type: 'chunk', file, n, version: st.version, D: OBS_DIM, A: ACT_DIM, R: REW, G: h0.length });
  st.n = 0;
}

// ------------------------------------------------------------------ one match
// 2v2 / 3v3: the agent drives one robot; its teammates are the current network too (they learn to
// play together), the other ALLIANCE is the opponent of this match times k
function teamSettings(k, opp, robotKey, alliance) {
  const slots = [];
  const starts = (n) => [...START_ORDER].sort(() => Math.random() - 0.5).slice(0, n);
  const me = Math.floor(Math.random() * k);
  for (const a of [BLUE, RED]) {
    const st = starts(3);
    for (let i = 0; i < 3; i++) {
      let sl;
      if (i >= k) sl = { driver: 'empty' };
      else if (a === alliance) sl = { driver: 'external', robot: i === me ? robotKey : pick(config.robots || ROBOT_ORDER) };
      else if (opp.kind === 'off') sl = { driver: 'empty' };
      else if (opp.kind === 'nn') sl = { driver: 'nn', policy: loadPolicy(opp.path), robot: pick(ROBOT_ORDER) };
      else sl = { driver: opp.strategy, skill: opp.skill || 'regional', robot: pick(ROBOT_ORDER) };
      slots.push({ robot: sl.robot || pick(ROBOT_ORDER), auto: sl.driver === opp.strategy ? 'best' : 'none', start: st[i], skill: 'champs', ...sl });
    }
  }
  return { matchMode: '3v3', slots, alliance, preload: 8, hp: 'auto', climber: 'none' };
}

async function playEpisode() {
  const opp = pickWeighted(config.opponents);
  const robotKey = pick(config.robots || ROBOT_ORDER);
  const alliance = Math.random() < 0.5 ? BLUE : RED;
  const k = pick(config.teams || [1]);
  const settings = k > 1 ? teamSettings(k, opp, robotKey, alliance) : {
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
    const m = game.match, me = game.robot;
    const A = alliance, B = other(alliance);
    // every robot the network drives on its ALLIANCE (it decides for all of them at the same steps)
    const streams = game.units.filter((u) => u.external).map((u) => newStream(u.robot));
    for (const st of streams) st.others = { foes: game.robots.filter((r) => r.alliance !== A), mates: game.robots.filter((r) => r.alliance === A && r !== st.robot) };
    const snap = (r) => [m.total(A), m.total(B), r.stats.intaked, m.score[A].inactiveFuel, m.score[A].fouls.reduce((s, f) => s + f.pts, 0), m.total(A) - m.total(B)];
    let since = STEPS_PER_DECISION, step = 0, decisions = 0;
    const record = (st, done) => {
      // finish the previous transition: its reward is everything that happened since
      const now = snap(st.robot);
      const j = st.n - 1;
      for (let q = 0; q < REW - 1; q++) st.rew[j * REW + q] = now[q] - st.last[q];
      st.rew[j * REW + REW - 1] = now[REW - 1];
      st.done[j] = done ? 1 : 0;
      st.last = now;
    };
    while (!m.over && step < 30000) {
      if (m.robotEnabled && since >= STEPS_PER_DECISION) {
        since = 0;
        for (const st of streams) buildObs(st.robot, st.others, m, world.fuel, st.cur);
        if (streams[0].last) {
          for (const st of streams) record(st, false);
          // all streams fill up together: hand them over, then pick up the newest network
          if (streams[0].n >= config.chunk) {
            for (const st of streams) flush(st, st.cur);
            await yieldToIO();
            applyConfig();
            if (stop) return null;
          }
        } else for (const st of streams) st.last = snap(st.robot);
        for (const st of streams) {
          if (st.n === 0) { st.version = version; st.h0 = st.state ? Float32Array.from(st.state) : null; }
          const { action, logp } = policy.act(st.cur, { stochastic: true, state: st.state });
          const j = st.n++;
          st.obs.set(st.cur, j * OBS_DIM);
          st.act.set(action, j * ACT_DIM);
          st.logp[j] = logp;
          actionToCmd(st.robot, action, st.robot.cmd);
        }
        decisions++;
      }
      stepGame(game, world, PHYSICS_DT);
      since++;
      if (++step % 2 === 0) frameGame(game, 2 * PHYSICS_DT);
    }
    for (const st of streams) {
      if (st.last && st.n) record(st, true);
      flush(st, st.cur);
    }
    return {
      type: 'episode', id: ID, version, opp: opp.key, teams: k, robot: robotKey, oppRobot: settings.oppRobot,
      own: m.total(A), them: m.total(B), margin: m.total(A) - m.total(B), fuel: m.fuelPoints(A),
      inactive: m.score[A].inactiveFuel, fouls: m.score[A].fouls.map((f) => f.rule),
      shots: me.stats.shots, passes: me.stats.passes, intaked: me.stats.intaked, decisions, streams: streams.length,
    };
  } finally {
    world.physics.world.free();
  }
}

// ------------------------------------------------------------------ main loop
send({ type: 'ready', id: ID, obsDim: OBS_DIM, actDim: ACT_DIM });
while (!config && !stop) { await new Promise((r) => setTimeout(r, 20)); applyConfig(); }
while (!stop) {
  applyConfig(); // (every match ends with its chunks handed over)
  try {
    const ep = await playEpisode();
    if (ep) send(ep);
  } catch (e) {
    send({ type: 'error', id: ID, msg: String((e && e.stack) || e) });
  }
  await yieldToIO();
}
process.exit(0);
