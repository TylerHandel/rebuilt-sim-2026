// A test runner for a machine with spare cores: it watches the `batch-queue` branch on origin for
// match batches (queued with tools/queue-job.mjs), runs each one with tools/batch.mjs at the commit
// the job names, in a worktree of its own next to this checkout, and pushes the results to the
// `batch-results` branch. Leave it running:
//   node tools/runner.mjs [--jobs 12] [--poll 60]
// A job only picks batch.mjs's own options and the sim's own env knobs (names its code reads from
// process.env), checked against a fixed pattern; nothing else from a job reaches a shell. It does
// run this repository's code at the commit the job names, as running the tests by hand would.
import { spawn } from 'child_process';
import { cpus, hostname } from 'os';
import { createWriteStream, existsSync, mkdirSync, readdirSync, readFileSync, symlinkSync } from 'fs';
import { dirname, join, resolve } from 'path';
import { fileURLToPath } from 'url';
import { git, pushFiles, tip } from './gitbranch.mjs';

const ROOT = resolve(dirname(fileURLToPath(import.meta.url)), '..');
const args = process.argv.slice(2);
const arg = (k, d) => { const i = args.indexOf('--' + k); return i >= 0 ? args[i + 1] : d; };
const JOBS = +arg('jobs', Math.max(1, cpus().length - 4));
const POLL = +arg('poll', 60) * 1000;
const WT = resolve(ROOT, '..', 'rebuilt-sim-runner');
const LOGS = join(ROOT, 'results', 'runner-logs');
mkdirSync(LOGS, { recursive: true });

const SAFE = /^[\w.,:+-]{1,200}$/;
const FLAGS = { pairs: 1, roundrobin: 0, robots: 1, seeds: 1, strategy: 1, skill: 1, jobs: 1 };
const log = (s) => console.log(`${new Date().toLocaleTimeString()} ${s}`);
const sleep = (ms) => new Promise((r) => setTimeout(r, ms));

function knobs(dir) {
  const k = new Set();
  for (const d of ['js', 'js/mech', 'tools']) {
    if (!existsSync(join(dir, d))) continue;
    for (const f of readdirSync(join(dir, d))) if (/\.m?js$/.test(f)) for (const m of readFileSync(join(dir, d, f), 'utf8').matchAll(/env\??\.([A-Z][A-Z0-9_]*)/g)) k.add(m[1]);
  }
  return k;
}

// the job's batch.mjs arguments, only its own options with plain values
function batchArgs(job) {
  const a = Array.isArray(job.args) ? job.args.map(String) : [];
  const out = [];
  for (let i = 0; i < a.length; i++) {
    const f = a[i].replace(/^--/, '');
    if (!a[i].startsWith('--') || !(f in FLAGS)) throw new Error(`option not allowed: ${a[i]}`);
    out.push('--' + f);
    if (FLAGS[f]) { const v = a[++i]; if (!SAFE.test(v ?? '')) throw new Error(`bad value for --${f}`); out.push(v); }
  }
  if (!out.includes('--jobs')) out.push('--jobs', String(JOBS));
  return out;
}

function worktreeAt(ref) {
  git(ROOT, ['fetch', '-q', 'origin', ref]);
  const sha = git(ROOT, ['rev-parse', 'FETCH_HEAD']);
  if (!existsSync(WT)) git(ROOT, ['worktree', 'add', '-q', '--detach', WT, sha]);
  else git(WT, ['checkout', '-q', '--detach', '--force', sha]);
  if (!existsSync(join(WT, 'node_modules'))) symlinkSync(join(ROOT, 'node_modules'), join(WT, 'node_modules'));
  return sha;
}

function pending() {
  const q = tip(ROOT, 'batch-queue');
  if (!q) return [];
  const r = tip(ROOT, 'batch-results');
  const done = new Set(r ? git(ROOT, ['ls-tree', '-r', '--name-only', r]).split('\n') : []);
  return git(ROOT, ['ls-tree', '--name-only', q, 'queue/']).split('\n').filter((p) => p.endsWith('.json'))
    .map((p) => p.slice(6, -5)).filter((id) => /^[\w.-]{1,100}$/.test(id))
    .filter((id) => !done.has(`results/batch/${id}.json`) && !done.has(`status/${id}.failed`) && !done.has(`status/${id}.started`))
    .sort().map((id) => ({ id, job: JSON.parse(git(ROOT, ['show', `${q}:queue/${id}.json`])) }));
}

function status(id, kind, body) {
  try { pushFiles(ROOT, 'batch-results', { [`status/${id}.${kind}`]: { text: JSON.stringify(body, null, 1) } }, `${kind} ${id}`); } catch (e) { log(`status push failed: ${String(e.message).split('\n')[0]}`); }
}

async function run({ id, job }) {
  log(`job ${id}: ${JSON.stringify(job)}`);
  let sha;
  try {
    if (!SAFE.test(job.ref ?? '')) throw new Error('bad ref');
    sha = worktreeAt(job.ref);
    const a = batchArgs(job);
    const K = knobs(WT);
    const env = { ...process.env }; // (git's own credentials and proxy settings; knobs added below)
    for (const [k, v] of Object.entries(job.env ?? {})) {
      if (!K.has(k) || !SAFE.test(String(v))) throw new Error(`env not allowed: ${k}`);
      env[k] = String(v);
    }
    status(id, 'started', { at: new Date().toISOString(), commit: sha, threads: cpus().length, args: a, env: job.env ?? {} });
    const out = createWriteStream(join(LOGS, `${id}.log`));
    const code = await new Promise((res) => {
      const p = spawn(process.execPath, ['tools/batch.mjs', ...a, '--name', id, '--push'], { cwd: WT, env });
      p.stdout.on('data', (d) => { process.stdout.write(d); out.write(d); });
      p.stderr.on('data', (d) => { process.stderr.write(d); out.write(d); });
      p.on('close', res);
    });
    out.end();
    if (code !== 0) throw new Error(`batch.mjs exited ${code}`);
    log(`job ${id} done`);
  } catch (e) {
    log(`job ${id} failed: ${e.message}`);
    status(id, 'failed', { at: new Date().toISOString(), commit: sha, error: String(e.message).slice(0, 2000) });
  }
}

log(`runner on ${hostname()}: ${cpus().length} threads, ${JOBS} matches at a time, checking every ${POLL / 1000} s (Ctrl+C stops)`);
status(`runner-${hostname()}`, 'alive', { at: new Date().toISOString(), threads: cpus().length, jobs: JOBS });
for (;;) {
  let list = [];
  try { list = pending(); } catch (e) { log(`couldn't read the queue: ${String(e.message).split('\n')[0]}`); }
  if (list.length) await run(list[0]); else await sleep(POLL);
}
