// Each robot's own FUEL a match (its AUDIT rows: what it launched into its HUB) side by side across
// sets of 1 v 1 matches against a reference robot, for before/after comparisons of the AI:
//   node tools/own-pts.mjs <job id | batch.json> [...] [--vs 2910] [--seeds 4]
// The reference's own number is from its self-match (BLUE). Also each set's mean over the robots,
// and how far each robot moved against the first set, in standard errors of the difference.
import { spawnSync } from 'child_process';
import { existsSync, readFileSync } from 'fs';
import { dirname, join } from 'path';
import { fileURLToPath } from 'url';

const ROOT = join(dirname(fileURLToPath(import.meta.url)), '..');
const argv = process.argv.slice(2);
const arg = (k, d) => { const i = argv.indexOf('--' + k); return i >= 0 ? argv[i + 1] : d; };
const ids = argv.filter((a, i) => !a.startsWith('--') && !['--vs', '--seeds'].includes(argv[i - 1]));
const ref = arg('vs', '2910'), seedMax = +arg('seeds', 99);
const load = (src) => JSON.parse(existsSync(src) ? readFileSync(src, 'utf8')
  : spawnSync('git', ['show', `origin/batch-results:results/batch/${src.replace(/\.json$/, '')}.json`], { cwd: ROOT, maxBuffer: 1 << 28 }).stdout.toString());
const sets = ids.map((id) => {
  const per = {};
  for (const r of load(id).results) {
    if (!r.ok || !r.audit || r.seed > seedMax || r.b !== ref) continue;
    const x = r.audit.find((y) => y.alliance === 'blue');
    (per[r.a] ??= []).push(x.pts.auto + x.pts.trans + x.pts.a1 + x.pts.a2 + x.pts.end);
  }
  return per;
});
const robots = Object.keys(sets[0]).sort((a, b) => +a - +b);
const st = (a) => { const n = a.length, m = a.reduce((s, v) => s + v, 0) / n, sd = Math.sqrt(a.reduce((s, v) => s + (v - m) ** 2, 0) / Math.max(1, n - 1)); return { n, m, se: sd / Math.sqrt(n) }; };
console.log('robot ' + ids.map((i) => String(i).replace(/^.*T\d\d-\d\d-\d\d-/, '').slice(0, 18).padStart(20)).join(''));
const tot = ids.map(() => 0);
for (const k of robots) {
  const s0 = sets[0][k] && st(sets[0][k]);
  console.log(k.padEnd(6) + sets.map((p, j) => {
    if (!p[k]) return '-'.padStart(20);
    const s = st(p[k]);
    tot[j] += s.m;
    const z = j && s0 ? ` ${((s.m - s0.m) / Math.hypot(s.se, s0.se)).toFixed(1)}se` : '';
    return `${s.m.toFixed(0)} (${s.n})${z}`.padStart(20);
  }).join(''));
}
console.log('mean  ' + tot.map((t) => (t / robots.length).toFixed(1).padStart(20)).join(''));
