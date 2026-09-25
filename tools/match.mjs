// Run one headless AI-vs-AI match and print the result.
//   npm run match -- --a scorer:trained:2910 --b defense:champs:4414 [--red] [--seed 7]
// Each side is strategy:skill:robot (strategy scorer|defense|hybrid, skill rookie|regional|
// champs|trained, robot 2910|4414|8793).
import { runMatch, BLUE, RED } from './headless.mjs';

const args = process.argv.slice(2);
const arg = (k, d) => { const i = args.indexOf('--' + k); return i >= 0 ? args[i + 1] : d; };
const side = (s) => { const [strategy, skill, robot] = s.split(':'); return { strategy, skill, robot }; };
const a = side(arg('a', 'scorer:trained:2910'));
const b = side(arg('b', 'scorer:champs:2910'));
const seed = +arg('seed', 1);
const t0 = performance.now();
const r = await runMatch(a, b, { allianceA: args.includes('--red') ? RED : BLUE, seed });
const fmt = (s, x) => `${s.strategy}/${s.skill}/${s.robot}: ${x.total} pts (FUEL ${x.fuel}, launched ${x.shots}, fouls ${x.fouls.join(' ') || 'none'})`;
console.log(`A ${fmt(a, r.a)}\nB ${fmt(b, r.b)}\nmargin ${r.margin > 0 ? '+' : ''}${r.margin} · ${((performance.now() - t0) / 1000).toFixed(1)} s`);
