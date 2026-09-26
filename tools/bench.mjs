// Quick regression bench: a few seeded AI-vs-AI matches, with scores, shots and wall time.
//   node --import ./tools/node-env.mjs tools/bench.mjs
import { runMatch } from './headless.mjs';
const combos = [['2910', '4414'], ['4414', '8793'], ['8793', '2910']];
const rows = [];
const t0 = performance.now();
for (const [ra, rb] of combos) {
  for (const seed of [1, 2]) {
    const s = performance.now();
    const r = await runMatch({ strategy: 'scorer', skill: 'champs', robot: ra }, { strategy: 'scorer', skill: 'champs', robot: rb }, { seed });
    rows.push(`${ra} vs ${rb} seed ${seed}: ${r.a.total}-${r.b.total}  shots ${r.a.shots}/${r.b.shots}  intaked ${r.a.intaked}/${r.b.intaked}  fouls ${r.a.fouls.join(',') || '-'} / ${r.b.fouls.join(',') || '-'}  ${((performance.now() - s) / 1000).toFixed(1)}s`);
  }
}
console.log(rows.join('\n'));
console.log(`total ${((performance.now() - t0) / 1000).toFixed(1)}s`);
