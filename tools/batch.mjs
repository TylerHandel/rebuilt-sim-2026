// Run many headless AI-vs-AI matches at once on a machine with more cores (one match per core),
// summarize them, and optionally push the results to the `batch-results` branch for analysis
// elsewhere. Each match is its own `tools/match.mjs` process; env vars pass through (MU_FUEL=...).
//   node tools/batch.mjs --pairs 6328:2910,2056:2910 --seeds 1-6 [--jobs 12] [--push]
//   node tools/batch.mjs --roundrobin [--robots 2910,4414,...] --seeds 1-2 --push
//   node tools/batch.mjs --pairs 2910:4414 --seeds 1-3 --skill champs --strategy scorer --name try1
// --pairs a:b,... plays robot a (BLUE) against robot b; --roundrobin plays every pair of --robots
// (default: every physical robot) once each way. Results: results/batch/<name>.json (every match)
// and a table of averages. --push commits only that file to `batch-results` (without touching
// the checkout or its branch) and pushes it.
import { spawn, execFileSync } from 'child_process';
import { cpus } from 'os';
import { mkdirSync, readdirSync, readFileSync, writeFileSync } from 'fs';
import { pushFiles } from './gitbranch.mjs';
import { dirname, join, relative } from 'path';
import { fileURLToPath } from 'url';

const ROOT = join(dirname(fileURLToPath(import.meta.url)), '..');
const args = process.argv.slice(2);
const arg = (k, d) => { const i = args.indexOf('--' + k); return i >= 0 ? args[i + 1] : d; };
const flag = (k) => args.includes('--' + k);

// physical robots, from their configs (`physical: '2910'`)
const PHYSICAL = [...new Set([...readFileSync(join(ROOT, 'js/robotConfigs.js'), 'utf8').matchAll(/physical: '(\d+)'/g)].map((m) => m[1]))];
const range = (s) => s.split(',').flatMap((p) => { const [a, b] = p.split('-').map(Number); return b ? Array.from({ length: b - a + 1 }, (_, i) => a + i) : [a]; });
const seeds = range(arg('seeds', '1'));
const strategy = arg('strategy', 'scorer'), skill = arg('skill', 'champs');
const jobs = Math.max(1, +arg('jobs', Math.max(1, cpus().length - 2)));
let pairs;
if (flag('roundrobin')) {
  const rs = arg('robots') ? arg('robots').split(',') : PHYSICAL;
  pairs = rs.flatMap((a) => rs.filter((b) => b !== a).map((b) => [a, b]));
} else pairs = (arg('pairs') || '').split(',').filter(Boolean).map((p) => p.split(':'));
if (!pairs.length) { console.error('nothing to run: give --pairs a:b,... or --roundrobin'); process.exit(1); }

const stamp = new Date().toISOString().replace(/[:.]/g, '-').slice(0, 19);
const name = arg('name', `batch-${stamp}`).replace(/[^\w.-]/g, '_');
const queue = pairs.flatMap(([a, b]) => seeds.map((seed) => ({ a, b, seed })));
console.log(`${queue.length} matches (${pairs.length} pairings x ${seeds.length} seeds), ${jobs} at a time on ${cpus().length} threads`);

// A scorer/champs/6328: 283 pts (FUEL 283, launched 291 incl. 0 passes, fouls none)
const LINE = /^([AB]) (\w+)\/(\w+)\/(\w+): (-?\d+) pts \(FUEL (\d+), launched (\d+) incl\. (\d+) passes, fouls (.*)\)$/;
function runOne(m) {
  return new Promise((res) => {
    const t0 = Date.now();
    const p = spawn(process.execPath, ['--import', './tools/node-env.mjs', 'tools/match.mjs', '--a', `${strategy}:${skill}:${m.a}`, '--b', `${strategy}:${skill}:${m.b}`, '--seed', String(m.seed)], { cwd: ROOT, env: process.env });
    let out = '', err = '';
    p.stdout.on('data', (d) => { out += d; });
    p.stderr.on('data', (d) => { err += d; });
    p.on('close', (code) => {
      const side = {};
      for (const l of out.split('\n')) {
        const x = LINE.exec(l.trim());
        if (x) side[x[1]] = { robot: x[4], pts: +x[5], fuel: +x[6], launched: +x[7], passes: +x[8], fouls: x[9] === 'none' ? [] : x[9].split(' ') };
      }
      res({ ...m, ok: code === 0 && side.A && side.B, A: side.A, B: side.B, s: (Date.now() - t0) / 1000, err: code === 0 ? undefined : err.slice(-2000) });
    });
  });
}

const results = [];
let next = 0, done = 0;
const t0 = Date.now();
async function worker() {
  while (next < queue.length) {
    const m = queue[next++];
    const r = await runOne(m);
    results.push(r);
    done++;
    const eta = ((Date.now() - t0) / done) * (queue.length - done) / 1000;
    console.log(`[${done}/${queue.length}] ${r.a} vs ${r.b} seed ${r.seed}: ${r.ok ? `${r.A.pts}-${r.B.pts}` : 'FAILED'} (${r.s.toFixed(0)} s; ~${(eta / 60).toFixed(1)} min left)`);
  }
}
await Promise.all(Array.from({ length: Math.min(jobs, queue.length) }, worker));

// averages per robot (points, FUEL, launched, wins) and per pairing
const per = {};
const add = (k, me, them) => {
  const p = (per[k] ??= { n: 0, pts: 0, fuel: 0, launched: 0, passes: 0, fouls: 0, w: 0, l: 0, t: 0 });
  p.n++; p.pts += me.pts; p.fuel += me.fuel; p.launched += me.launched; p.passes += me.passes; p.fouls += me.fouls.length;
  if (me.pts > them.pts) p.w++; else if (me.pts < them.pts) p.l++; else p.t++;
};
for (const r of results.filter((x) => x.ok)) { add(r.a, r.A, r.B); add(r.b, r.B, r.A); }
const rows = Object.entries(per).sort((x, y) => y[1].pts / y[1].n - x[1].pts / x[1].n);
const table = ['robot  matches  avg pts  avg FUEL  avg launched  avg passes  fouls  W-L-T',
  ...rows.map(([k, p]) => `${k.padEnd(6)} ${String(p.n).padStart(7)} ${(p.pts / p.n).toFixed(1).padStart(8)} ${(p.fuel / p.n).toFixed(1).padStart(9)} ${(p.launched / p.n).toFixed(1).padStart(13)} ${(p.passes / p.n).toFixed(1).padStart(11)} ${String(p.fouls).padStart(6)}  ${p.w}-${p.l}-${p.t}`)].join('\n');
const failed = results.filter((x) => !x.ok);
console.log(`\n${table}\n${failed.length ? `${failed.length} failed: ${failed.map((f) => `${f.a}v${f.b}#${f.seed}`).join(' ')}\n` : ''}${((Date.now() - t0) / 60000).toFixed(1)} min`);

let commit = '';
try { commit = execFileSync('git', ['rev-parse', '--short', 'HEAD'], { cwd: ROOT }).toString().trim(); } catch {}
// the sim's own knobs only (the names its code reads from process.env), never the machine's
const KNOBS = new Set();
for (const dir of ['js', 'js/mech', 'tools']) {
  for (const f of readdirSync(join(ROOT, dir))) {
    if (/\.m?js$/.test(f)) for (const m of readFileSync(join(ROOT, dir, f), 'utf8').matchAll(/env\??\.([A-Z][A-Z0-9_]*)/g)) KNOBS.add(m[1]);
  }
}
const env = Object.fromEntries(Object.entries(process.env).filter(([k]) => KNOBS.has(k)));
const file = join(ROOT, 'results/batch', `${name}.json`);
mkdirSync(dirname(file), { recursive: true });
writeFileSync(file, JSON.stringify({ name, threads: cpus().length, jobs, commit, strategy, skill, seeds, env, minutes: (Date.now() - t0) / 60000, summary: table, results }, null, 1));
console.log(`wrote ${relative(ROOT, file)}`);

// commit just that file onto `batch-results` (the checkout, its index and its branch stay as they
// are) and push it
if (flag('push')) {
  try {
    const c = pushFiles(ROOT, 'batch-results', { [`results/batch/${name}.json`]: file }, `batch ${name}: ${queue.length} matches at ${commit}`);
    console.log(`pushed to batch-results (${c.slice(0, 7)})`);
  } catch (e) {
    console.log(`push failed (${String(e.message).split('\n')[0]}); the results are still in ${relative(ROOT, file)}`);
  }
}
