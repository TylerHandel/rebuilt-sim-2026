// Run one headless AI-vs-AI match and print the result.
//   npm run match -- --a scorer:trained:2910 --b defense:champs:4414 [--red] [--seed 7]
// Each side is strategy:skill:robot (strategy scorer|defense|hybrid, skill rookie|regional|
// champs|trained, robot 2910|4414|8793...). A side of up to three robots joined by + plays a 3 v 3:
//   --a scorer:champs:2910+4414+1690 --b scorer:champs:1678+7769+2056
// Then a line per robot, AUDIT {...}: what it did with its match (tools/audit.mjs).
import { runMatch, runGame, BLUE, RED } from './headless.mjs';
import { makeAudit } from './audit.mjs';

const args = process.argv.slice(2);
const arg = (k, d) => { const i = args.indexOf('--' + k); return i >= 0 ? args[i + 1] : d; };
const side = (s) => { const [strategy, skill, robot] = s.split(':'); return { strategy, skill, robot }; };
const a = side(arg('a', 'scorer:trained:2910'));
const b = side(arg('b', 'scorer:champs:2910'));
const seed = +arg('seed', 1);
const red = args.includes('--red');
const audit = makeAudit();
const t0 = performance.now();
const fmt = (s, x) => `${s.strategy}/${s.skill}/${s.robot}: ${x.total} pts (FUEL ${x.fuel}, launched ${x.shots} incl. ${x.passes} passes, fouls ${x.fouls.join(' ') || 'none'})`;
let r;
if (a.robot.includes('+') || b.robot.includes('+')) {
  // 3 v 3: A's robots in stations 1-3 of its ALLIANCE, B's in the other's
  const slots = (s) => s.robot.split('+').concat(['', '', '']).slice(0, 3).map((robot) => (robot
    ? { robot, auto: 'best', start: 'rightTrench', driver: s.strategy, skill: s.skill }
    : { robot: '2910', auto: 'best', start: 'rightTrench', driver: 'empty', skill: s.skill }));
  const blue = red ? b : a, redS = red ? a : b;
  const g = await runGame({ matchMode: '3v3', slots: [...slots(blue), ...slots(redS)] }, { seed, trace: audit.trace });
  const sum = (al, k) => g.robots.filter((x) => x.alliance === al).reduce((n, x) => n + x[k], 0);
  const side3 = (al) => ({ ...g[al], shots: sum(al, 'shots'), passes: sum(al, 'passes') });
  const A = red ? RED : BLUE, B = red ? BLUE : RED;
  r = { a: side3(A), b: side3(B), margin: g[A].total - g[B].total };
} else r = await runMatch(a, b, { allianceA: red ? RED : BLUE, seed, trace: audit.trace });
console.log(`A ${fmt(a, r.a)}\nB ${fmt(b, r.b)}\nmargin ${r.margin > 0 ? '+' : ''}${r.margin} · ${((performance.now() - t0) / 1000).toFixed(1)} s`);
for (const x of audit.report()) console.log(`AUDIT ${JSON.stringify(x)}`);
