// Queue a match batch for tools/runner.mjs (on a machine with spare cores) and fetch its results.
//   node tools/queue-job.mjs [--ref main-aogjl4] [--env MU_FUEL=0.3,...] [--label x] -- <batch.mjs options>
//       e.g. -- --pairs 6328:2910,2056:2910 --seeds 1-6     prints the job id
//   node tools/queue-job.mjs --wait <id> [--timeout 180]     waits (minutes) and prints the summary
//   node tools/queue-job.mjs --status                         runners alive, jobs started/failed/done
//   node tools/queue-job.mjs --cancel <id>[,<id>] [--reason x]  skip queued jobs
// The ref must be on origin (push first). Results land on `batch-results` as results/batch/<id>.json.
import { dirname, join, resolve } from 'path';
import { fileURLToPath } from 'url';
import { git, pushFiles, tip } from './gitbranch.mjs';

const ROOT = resolve(dirname(fileURLToPath(import.meta.url)), '..');
const argv = process.argv.slice(2);
const dd = argv.indexOf('--');
const own = dd >= 0 ? argv.slice(0, dd) : argv, rest = dd >= 0 ? argv.slice(dd + 1) : [];
const arg = (k, d) => { const i = own.indexOf('--' + k); return i >= 0 ? own[i + 1] : d; };
const show = (r, p) => { try { return git(ROOT, ['show', `${r}:${p}`]); } catch { return null; } };

if (own.includes('--status')) {
  const r = tip(ROOT, 'batch-results');
  const files = r ? git(ROOT, ['ls-tree', '-r', '--name-only', r]).split('\n') : [];
  for (const f of files.filter((x) => x.startsWith('status/'))) console.log(f, (show(r, f) || '').replace(/\s+/g, ' ').slice(0, 200));
  const q = tip(ROOT, 'batch-queue');
  const queued = q ? git(ROOT, ['ls-tree', '--name-only', q, 'queue/']).split('\n').filter(Boolean) : [];
  console.log(`${queued.length} queued; ${files.filter((x) => x.startsWith('results/batch/')).length} finished`);
} else if (arg('cancel')) {
  // a queued job the runner hasn't finished: marked failed, so it's skipped (one already running
  // still finishes)
  const ids = arg('cancel').split(','), why = arg('reason', 'cancelled');
  pushFiles(ROOT, 'batch-results', Object.fromEntries(ids.map((id) => [`status/${id}.failed`, { text: JSON.stringify({ at: new Date().toISOString(), error: `cancelled: ${why}` }) }])), `cancel ${ids.join(' ')}`);
  console.log(`cancelled ${ids.join(', ')}`);
} else if (arg('wait')) {
  const id = arg('wait'), until = Date.now() + +arg('timeout', 180) * 60000;
  for (;;) {
    const r = tip(ROOT, 'batch-results');
    const res = r && show(r, `results/batch/${id}.json`);
    if (res) { const d = JSON.parse(res); console.log(`${id}: ${d.results.length} matches in ${d.minutes.toFixed(1)} min at ${d.commit}\n${d.summary}`); break; }
    const fail = r && show(r, `status/${id}.failed`);
    if (fail) { console.log(`${id} failed: ${fail}`); process.exit(1); }
    if (Date.now() > until) { console.log(`${id}: not done yet (${r && show(r, `status/${id}.started`) ? 'started' : 'not started'})`); process.exit(2); }
    await new Promise((s) => setTimeout(s, 60000));
  }
} else {
  if (!rest.length) { console.error('give the batch.mjs options after --'); process.exit(1); }
  const env = Object.fromEntries((arg('env', '') || '').split(',').filter(Boolean).map((kv) => kv.split('=')));
  const stamp = new Date().toISOString().replace(/[:.]/g, '-').slice(0, 19);
  const id = `${stamp}-${(arg('label', 'job')).replace(/[^\w-]/g, '_').slice(0, 40)}`;
  const job = { ref: arg('ref', 'main-aogjl4'), args: rest, env };
  pushFiles(ROOT, 'batch-queue', { [`queue/${id}.json`]: { text: JSON.stringify(job, null, 1) } }, `queue ${id}`);
  console.log(id);
}
