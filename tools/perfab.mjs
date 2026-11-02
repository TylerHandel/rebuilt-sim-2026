// A/B speed of two copies of the game on the same headless match: runs tools/perf3v3.mjs --noprof
// in each, alternating, and reports CPU ms per simulated second (median of the runs) and the
// real-time factor. CPU time, not wall time: on a shared machine the wall clock is noisy.
//   node tools/perfab.mjs --a <repo copy> [--b <repo copy, default this one>] [--runs 3]
//        [--secs 30] [--match "--3v3 2910,1678,4414:6328,7769,1690"]
// (The copies need their own tools/perf3v3.mjs: copy this one's in.)
import { spawnSync } from 'node:child_process';
import { dirname, join, resolve } from 'node:path';
import { fileURLToPath } from 'node:url';

const args = process.argv.slice(2);
const arg = (k, d) => { const i = args.indexOf('--' + k); return i >= 0 ? args[i + 1] : d; };
const here = resolve(dirname(fileURLToPath(import.meta.url)), '..');
const roots = { A: resolve(arg('a', here)), B: resolve(arg('b', here)) };
const runs = +arg('runs', 3), secs = arg('secs', '30');
const match = arg('match', '--3v3 2910,1678,4414:6328,7769,1690').split(' ');
const out = { A: [], B: [] };
for (let i = 0; i < runs; i++) {
  for (const k of ['A', 'B']) {
    const r = spawnSync(process.execPath, ['--import', './tools/node-env.mjs', 'tools/perf3v3.mjs', '--noprof', '--secs', secs, ...match], { cwd: roots[k], encoding: 'utf8' });
    const cpu = +((r.stdout.match(/CPU time: (\d+)/) || [])[1]);
    const rt = +((r.stdout.match(/real-time factor ([\d.]+)/) || [])[1]);
    out[k].push({ cpu, rt });
    if (!cpu) console.log(k, r.stdout, r.stderr);
  }
}
const med = (xs) => { const s = [...xs].sort((a, b) => a - b); return s[Math.floor(s.length / 2)]; };
for (const k of ['A', 'B']) console.log(`${k} ${roots[k]}: CPU ${med(out[k].map((x) => x.cpu))} ms per sim s (runs ${out[k].map((x) => x.cpu).join(' ')}), real-time ${med(out[k].map((x) => x.rt))}x`);
console.log(`B/A CPU: ${(med(out.B.map((x) => x.cpu)) / med(out.A.map((x) => x.cpu))).toFixed(3)}`);
