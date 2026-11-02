// Summarize a V8 .cpuprofile (node --cpu-prof): self time by function, and inclusive time for the
// functions named with --incl, as a share of the whole.
//   node --cpu-prof --cpu-prof-dir=prof --import ./tools/node-env.mjs tools/perf3v3.mjs --noprof --secs 30
//   node tools/cpuprof-top.mjs prof/*.cpuprofile [--top 40] [--incl step,_solve]
import { readFileSync } from 'node:fs';

const args = process.argv.slice(2);
const arg = (k, d) => { const i = args.indexOf('--' + k); return i >= 0 ? args[i + 1] : d; };
const file = args.find((a) => a.endsWith('.cpuprofile'));
const top = +arg('top', 40);
const P = JSON.parse(readFileSync(file, 'utf8'));
const byId = new Map(P.nodes.map((n) => [n.id, n]));
const self = new Map();
const dts = P.timeDeltas;
const hits = new Map();
for (let i = 0; i < P.samples.length; i++) hits.set(P.samples[i], (hits.get(P.samples[i]) || 0) + (dts[i] || 0));
const name = (n) => { const c = n.callFrame; return `${c.functionName || '(anon)'} ${c.url.split('/').slice(-2).join('/')}:${c.lineNumber + 1}`; };
let total = 0;
for (const [id, us] of hits) { const n = byId.get(id); const k = name(n); self.set(k, (self.get(k) || 0) + us); total += us; }
console.log(`total ${(total / 1e6).toFixed(1)} s sampled`);
for (const [k, us] of [...self.entries()].sort((a, b) => b[1] - a[1]).slice(0, top)) console.log(`${(100 * us / total).toFixed(1).padStart(5)}%  ${(us / 1e3).toFixed(0).padStart(7)} ms  ${k}`);
// inclusive: a node's own hits plus its descendants', counted once per sample stack
if (arg('incl')) {
  const want = arg('incl').split(',');
  const parent = new Map();
  for (const n of P.nodes) for (const c of n.children || []) parent.set(c, n.id);
  const incl = new Map();
  for (const [id, us] of hits) {
    const seen = new Set();
    for (let x = id; x !== undefined; x = parent.get(x)) {
      const fn = byId.get(x).callFrame.functionName;
      if (want.includes(fn) && !seen.has(fn)) { seen.add(fn); incl.set(fn, (incl.get(fn) || 0) + us); }
    }
  }
  for (const [k, us] of incl) console.log(`incl ${k}: ${(100 * us / total).toFixed(1)}%`);
}
