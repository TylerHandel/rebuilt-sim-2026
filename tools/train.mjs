// Self-play training for the robot AI's strategy ("brain", see BRAIN_SPEC in js/opponent.js).
//
//   npm run train                       # 10 generations, ~40 min on 4 cores
//   npm run train -- --gens 30 --pop 12 --scenarios 6 --resume
//   npm run train -- --quick            # smoke test
//   npm run train -- --from rebuilt-ai-brain.json   # continue from a brain exported in-game
//
// Each generation samples candidate brains around the current mean (mirrored Gaussian noise
// in normalized parameter space) and plays every candidate through the same seeded scenarios:
// full 2:40 headless matches on the real field with all 504 FUEL, against opponents from a
// league (the hand-tuned brain plus earlier snapshots of the trained one). Scores are ranked
// per scenario so a lopsided matchup counts as much as a close one, and the mean moves toward
// the best half. At the end the trained brain and the hand-tuned brain each play the same
// held-out scenarios and the paired score difference is reported. The result is written to
// js/trainedBrain.js (the "Trained" skill level) and the history to tools/training-log.json.
import { Worker } from 'node:worker_threads';
import { writeFileSync, readFileSync, existsSync } from 'node:fs';
import { cpus } from 'node:os';
import { BRAIN_SPEC, DEFAULT_BRAIN } from '../js/opponent.js';
import { TRAINED_BRAIN } from '../js/trainedBrain.js';
import { mulberry32 } from './headless.mjs';
import { brainFileText, parseBrainText } from '../js/brainFile.js';

// ------------------------------------------------------------------ options
const args = process.argv.slice(2);
const opt = (k, d) => { const i = args.indexOf('--' + k); return i >= 0 ? +args[i + 1] : d; };
const flag = (k) => args.includes('--' + k);
const QUICK = flag('quick');
const GENS = opt('gens', QUICK ? 1 : 10);
const POP = Math.max(2, 2 * Math.round(opt('pop', QUICK ? 2 : 8) / 2)); // even (mirrored pairs)
const SCEN = opt('scenarios', QUICK ? 2 : 4);
const VALID = opt('validate', QUICK ? 2 : 12);
const WORKERS = Math.max(1, Math.min(opt('workers', cpus().length), POP + 1));
const SEED = opt('seed', Date.now() % 1e9);
const RESUME = flag('resume');
const FROM = (() => { const i = args.indexOf('--from'); return i >= 0 ? args[i + 1] : null; })();
const NO_SAVE = flag('no-save');
const rng = mulberry32(SEED);

const KEYS = Object.keys(BRAIN_SPEC);
const toX = (brain) => KEYS.map((k) => (brain[k] - BRAIN_SPEC[k].min) / (BRAIN_SPEC[k].max - BRAIN_SPEC[k].min));
const toBrain = (x) => Object.fromEntries(KEYS.map((k, i) => {
  const s = BRAIN_SPEC[k];
  return [k, +(s.min + Math.min(1, Math.max(0, x[i])) * (s.max - s.min)).toFixed(3)];
}));
const gauss = () => { let u = 0; while (!u) u = rng(); return Math.sqrt(-2 * Math.log(u)) * Math.cos(2 * Math.PI * rng()); };
const pick = (a) => a[Math.floor(rng() * a.length)];

// ------------------------------------------------------------------ worker pool
class Pool {
  constructor(n) {
    this.idle = [];
    this.queue = [];
    this.jobs = new Map();
    this.nextId = 1;
    this.workers = [];
    for (let i = 0; i < n; i++) {
      const w = new Worker(new URL('./worker.mjs', import.meta.url));
      w.on('message', (msg) => {
        if (msg.ready) { this.idle.push(w); this._pump(); return; }
        const job = this.jobs.get(msg.id);
        this.jobs.delete(msg.id);
        this.idle.push(w);
        if (msg.error) job.reject(new Error(msg.error)); else job.resolve(msg.result);
        this._pump();
      });
      w.on('error', (e) => { console.error('worker error', e); process.exit(1); });
      this.workers.push(w);
    }
  }
  run(task) {
    return new Promise((resolve, reject) => {
      const id = this.nextId++;
      this.jobs.set(id, { resolve, reject });
      this.queue.push({ id, ...task });
      this._pump();
    });
  }
  _pump() {
    while (this.idle.length && this.queue.length) this.idle.pop().postMessage(this.queue.shift());
  }
  close() { for (const w of this.workers) w.terminate(); }
}

// ------------------------------------------------------------------ scenarios
const ROBOTS = ['2910', '4414', '8793'];
const A_STRATS = ['scorer', 'hybrid', 'scorer', 'hybrid', 'defense']; // what the learner plays
function scenarios(n, league, offset) {
  const out = [];
  for (let i = 0; i < n; i++) {
    const sa = A_STRATS[(offset + i) % A_STRATS.length];
    const sb = sa === 'defense' ? pick(['scorer', 'hybrid']) : pick(['scorer', 'hybrid', 'defense']);
    out.push({
      a: { strategy: sa, skill: 'champs', robot: pick(ROBOTS) },
      b: { strategy: sb, skill: 'champs', robot: pick(ROBOTS), brain: pick(league) },
      allianceA: rng() < 0.5 ? 'blue' : 'red',
      seed: Math.floor(rng() * 1e9),
    });
  }
  return out;
}

// every candidate x scenario; returns margins[candidate][scenario]
async function evaluate(pool, brains, scen) {
  const tasks = [];
  brains.forEach((brain, c) => scen.forEach((s, j) => {
    tasks.push(pool.run({ ...s, a: { ...s.a, brain } }).then((r) => ({ c, j, r })));
  }));
  const margins = brains.map(() => new Array(scen.length).fill(0));
  const detail = brains.map(() => []);
  for (const { c, j, r } of await Promise.all(tasks)) {
    margins[c][j] = r.margin;
    detail[c][j] = r;
  }
  return { margins, detail };
}

// average per-scenario rank in [0, 1] (1 = best in every scenario)
function rankFitness(margins) {
  const n = margins.length, k = margins[0].length;
  const fit = new Array(n).fill(0);
  for (let j = 0; j < k; j++) {
    const order = [...margins.keys()].sort((p, q) => margins[p][j] - margins[q][j]);
    // ties share the average rank
    let i = 0;
    while (i < n) {
      let e = i;
      while (e + 1 < n && margins[order[e + 1]][j] === margins[order[i]][j]) e++;
      const r = (i + e) / 2 / Math.max(1, n - 1);
      for (let t = i; t <= e; t++) fit[order[t]] += r / k;
      i = e + 1;
    }
  }
  return fit;
}

const avg = (a) => a.reduce((s, v) => s + v, 0) / a.length;
const fmtBrain = (b) => KEYS.map((k) => `${k}=${b[k]}`).join(' ');

// ------------------------------------------------------------------ training loop
const t0 = performance.now();
const pool = new Pool(WORKERS);
let fromBrain = null;
if (FROM) {
  fromBrain = parseBrainText(readFileSync(FROM, 'utf8'));
  if (!fromBrain) { console.error(`${FROM} has no brain values`); process.exit(1); }
}
const start = fromBrain ? { ...DEFAULT_BRAIN, ...fromBrain } : RESUME ? TRAINED_BRAIN : DEFAULT_BRAIN;
let mean = toX({ ...DEFAULT_BRAIN, ...start });
let sigma = opt('sigma', 0.12);
const league = [DEFAULT_BRAIN];
if (RESUME || fromBrain) league.push(start);
const history = [];
let matches = 0;
console.log(`Self-play training: ${GENS} generations x (${POP} candidates + mean) x ${SCEN} scenarios, ${WORKERS} workers, seed ${SEED}`);
console.log(`start: ${fmtBrain(toBrain(mean))}`);

for (let g = 1; g <= GENS; g++) {
  const tg = performance.now();
  const eps = [];
  for (let i = 0; i < POP / 2; i++) { const e = KEYS.map(() => gauss()); eps.push(e, e.map((v) => -v)); }
  const xs = eps.map((e) => mean.map((m, i) => Math.min(1, Math.max(0, m + sigma * e[i]))));
  xs.push(mean.slice()); // the current mean, for progress reporting
  const brains = xs.map(toBrain);
  const scen = scenarios(SCEN, league, g * SCEN);
  const { margins } = await evaluate(pool, brains, scen);
  matches += brains.length * scen.length;
  const fit = rankFitness(margins);
  // recombine the best half with log-rank weights
  const order = [...fit.keys()].sort((p, q) => fit[q] - fit[p]);
  const mu = Math.max(1, Math.floor(xs.length / 2));
  const w = order.slice(0, mu).map((_, i) => Math.log(mu + 0.5) - Math.log(i + 1));
  const ws = w.reduce((s, v) => s + v, 0);
  mean = mean.map((_, d) => order.slice(0, mu).reduce((s, c, i) => s + (w[i] / ws) * xs[c][d], 0));
  sigma = Math.max(0.03, sigma * 0.92);
  if (g % 3 === 0) { league.push(toBrain(mean)); if (league.length > 5) league.splice(1, 1); }
  const meanIdx = xs.length - 1;
  const rec = {
    gen: g, sigma: +sigma.toFixed(3), seconds: Math.round((performance.now() - tg) / 1000),
    scenarios: scen.map((s) => `${s.a.strategy}/${s.a.robot} vs ${s.b.strategy}/${s.b.robot}`),
    meanMargin: Math.round(avg(margins[meanIdx])), bestMargin: Math.round(avg(margins[order[0]])),
    meanRank: +fit[meanIdx].toFixed(2), brain: toBrain(mean),
  };
  history.push(rec);
  const eta = ((performance.now() - t0) / g) * (GENS - g) / 60000;
  console.log(`gen ${g}/${GENS}  ${rec.seconds}s  old-mean margin ${rec.meanMargin}  best ${rec.bestMargin}  old-mean rank ${rec.meanRank}  sigma ${rec.sigma}  ETA ${eta.toFixed(0)} min`);
  console.log(`   ${rec.scenarios.join(' | ')}`);
}

// ------------------------------------------------------------------ validation
const trained = toBrain(mean);
console.log(`\ntrained: ${fmtBrain(trained)}`);
console.log(`validating on ${VALID} held-out scenarios (trained vs hand-tuned, same opponent, same seed)…`);
const vscen = scenarios(VALID, [DEFAULT_BRAIN], 1000);
const { margins: vm, detail } = await evaluate(pool, [trained, DEFAULT_BRAIN], vscen);
matches += 2 * VALID;
const gains = vm[0].map((m, j) => m - vm[1][j]);
const fouls = (d) => d.reduce((s, r) => s + r.a.foulPtsGiven, 0);
const byStrat = {};
vscen.forEach((s, j) => { (byStrat[s.a.strategy] ??= []).push(gains[j]); });
const validation = {
  scenarios: VALID,
  avgGain: Math.round(avg(gains)),
  better: gains.filter((x) => x > 0).length,
  worse: gains.filter((x) => x < 0).length,
  byStrategy: Object.fromEntries(Object.entries(byStrat).map(([k, v]) => [k, Math.round(avg(v))])),
  trainedFoulPts: fouls(detail[0]),
  handTunedFoulPts: fouls(detail[1]),
};
console.log(`average gain over the hand-tuned brain: ${validation.avgGain > 0 ? '+' : ''}${validation.avgGain} points per match (better in ${validation.better}/${VALID}, worse in ${validation.worse})`);
console.log(`by strategy: ${JSON.stringify(validation.byStrategy)} · foul points given: trained ${validation.trainedFoulPts}, hand-tuned ${validation.handTunedFoulPts}`);
pool.close();

// ------------------------------------------------------------------ save
const minutes = Math.round((performance.now() - t0) / 60000);
const info = { source: 'tools/train.mjs self-play', from: FROM || (RESUME ? 'shipped trained brain' : 'hand-tuned brain'), date: new Date().toISOString().slice(0, 10), generations: GENS, population: POP, scenariosPerGen: SCEN, matches, minutes, seed: SEED, resumed: RESUME, validation };
if (NO_SAVE || QUICK) {
  console.log(`\n(not saved: ${QUICK ? '--quick' : '--no-save'}) ${matches} matches in ${minutes} min`);
} else if (validation.avgGain <= 0 && !flag('force')) {
  console.log(`\nNot saved: the trained brain did not beat the hand-tuned one on validation (use --force to save anyway).`);
} else {
  writeFileSync(new URL('../js/trainedBrain.js', import.meta.url), brainFileText(trained, info));
  const logUrl = new URL('./training-log.json', import.meta.url);
  const log = existsSync(logUrl) ? JSON.parse(readFileSync(logUrl, 'utf8')) : [];
  log.push({ ...info, history, trained });
  writeFileSync(logUrl, JSON.stringify(log, null, 1) + '\n');
  console.log(`\nSaved js/trainedBrain.js and tools/training-log.json (${matches} matches in ${minutes} min)`);
}
