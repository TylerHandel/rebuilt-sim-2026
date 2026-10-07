// Real-game scoreboard: plays the latest neural network (js/nn/driver.json, reloaded whenever the
// trainer publishes a new one) in full-game matches against the Champs-level AIs, nonstop, and
// appends each result to a JSON-lines file the dashboard (nn.html) reads. Started by
// tools/nn/train_gpu.py on the otherwise idle CPU, at low priority.
//
//   node --import ./tools/node-env.mjs tools/nn/eval.mjs --out runs/driver/eval.jsonl [--teams 3]
//        [--policy js/nn/driver.json] [--parent PID] [--matches N]
// Every robot on the network's ALLIANCE is the network (it learned to play together); the other
// ALLIANCE is the Scorer, Defense or Hybrid AI at Champs skill, taking turns.
import fs from 'node:fs';
import { createWorld } from '../headless.mjs';
import { createGame, stepGame, frameGame } from '../../js/game.js';
import { Policy } from '../../js/nn/policy.js';
import { PHYSICS_DT, BLUE, RED, other } from '../../js/constants.js';
import { ROBOT_ORDER } from '../../js/robotConfigs.js';
import { START_ORDER } from '../../js/auto.js';

const args = process.argv.slice(2);
const arg = (k, d) => { const i = args.indexOf('--' + k); return i >= 0 ? args[i + 1] : d; };
const OUT = arg('out', 'runs/driver/eval.jsonl');
const POLICY = arg('policy', 'js/nn/driver.json');
const TEAMS = (arg('teams', '3')).split(',').map(Number);
const PARENT = +arg('parent', 0);
const MATCHES = +arg('matches', Infinity);
const STRATEGIES = ['scorer', 'defense', 'hybrid'];
const MINE = arg('robots', '') ? arg('robots').split(',') : null; // robots the network drives (default: any)
let turn = +arg('seed', 0);

const pick = (arr) => arr[Math.floor(Math.random() * arr.length)];
const parentAlive = () => {
  if (!PARENT) return true;
  try { process.kill(PARENT, 0); return true; } catch (e) { return e.code === 'EPERM'; }
};

let policy = null, mtime = 0;
function latestPolicy() {
  try {
    const st = fs.statSync(POLICY);
    if (!policy || st.mtimeMs !== mtime) {
      policy = new Policy(JSON.parse(fs.readFileSync(POLICY, 'utf8')));
      mtime = st.mtimeMs;
    }
  } catch { /* being replaced right now, or not there yet: keep the one we have */ }
  return policy;
}

async function playMatch(k, strategy) {
  const pol = latestPolicy();
  if (!pol) return null;
  const nnSide = Math.random() < 0.5 ? BLUE : RED;
  const slots = [];
  for (const a of [BLUE, RED]) {
    const st = [...START_ORDER].sort(() => Math.random() - 0.5);
    for (let i = 0; i < 3; i++) {
      const robot = a === nnSide && MINE ? pick(MINE) : pick(ROBOT_ORDER);
      if (i >= k) slots.push({ driver: 'empty', robot });
      else if (a === nnSide) slots.push({ driver: 'nn', policy: pol, robot, auto: 'none', start: st[i], skill: 'champs' });
      else slots.push({ driver: strategy, robot, auto: 'best', start: st[i], skill: 'champs' });
    }
  }
  const world = await createWorld();
  try {
    const game = createGame(world, { matchMode: '3v3', slots, alliance: nnSide, preload: 8, hp: 'auto', climber: 'none' });
    const m = game.match;
    const mine = game.robots.filter((r) => r.alliance === nnSide);
    // how it drives: its turn command every decision (0.1 s) while enabled
    const tr = mine.map(() => ({ prev: 0, turn: 0, flips: 0, n: 0 }));
    let step = 0;
    const every = Math.round(0.1 / PHYSICS_DT);
    while (!m.over && step < 30000) {
      if (m.robotEnabled && step % every === 0) {
        mine.forEach((r, i) => {
          const w = r.cmd.omega / r.cfg.drive.maxOmega, s = tr[i];
          if (s.n && Math.abs(w) > 0.3 && Math.abs(s.prev) > 0.3 && Math.sign(w) !== Math.sign(s.prev)) s.flips++;
          s.turn += Math.min(1, Math.abs(w));
          s.prev = w;
          s.n++;
        });
      }
      stepGame(game, world, PHYSICS_DT);
      if (++step % 2 === 0) frameGame(game, 2 * PHYSICS_DT);
      if (step % 2000 === 0 && !parentAlive()) process.exit(0);
    }
    const A = nnSide, B = other(nnSide);
    const nn = m.total(A), them = m.total(B);
    return {
      t: Math.round(Date.now() / 1000), version: pol.version, updates: pol.info.updates ?? null, steps: pol.info.steps ?? null, hours: pol.info.hours ?? null,
      teams: k, opp: strategy, nn, them, margin: nn - them, result: nn > them ? 'win' : nn < them ? 'loss' : 'tie',
      intaked: mine.reduce((s, r) => s + r.stats.intaked, 0) / mine.length,
      shots: mine.reduce((s, r) => s + r.stats.shots, 0) / mine.length,
      passes: mine.reduce((s, r) => s + r.stats.passes, 0) / mine.length,
      auto: m.score[A].autoFuel, oppAuto: m.score[B].autoFuel, wall: Math.round(Date.now() / 1000),
      turn: tr.reduce((s, x) => s + x.turn / Math.max(1, x.n), 0) / tr.length,
      flipsPerMin: tr.reduce((s, x) => s + (600 * x.flips) / Math.max(1, x.n), 0) / tr.length,
      robots: slots.filter((s, i) => s.driver !== 'empty' && (i < 3 ? BLUE : RED) === A).map((s) => s.robot),
      oppRobots: slots.filter((s, i) => s.driver !== 'empty' && (i < 3 ? BLUE : RED) === B).map((s) => s.robot),
    };
  } finally {
    world.physics.world.free();
  }
}

for (let n = 0; n < MATCHES && parentAlive(); n++) {
  const strategy = STRATEGIES[turn % STRATEGIES.length];
  const k = TEAMS[Math.floor(turn / STRATEGIES.length) % TEAMS.length];
  turn++;
  try {
    const r = await playMatch(k, strategy);
    if (r) fs.appendFileSync(OUT, JSON.stringify(r) + '\n');
    else await new Promise((res) => setTimeout(res, 5000));
  } catch (e) {
    process.stderr.write(String((e && e.stack) || e) + '\n');
    await new Promise((res) => setTimeout(res, 5000));
  }
}
process.exit(0);
