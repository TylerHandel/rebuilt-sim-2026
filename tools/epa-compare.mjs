// The sim's ranking against the real season's: each robot's own FUEL a match in a set of match
// batches (its AUDIT rows: FUEL it launched into its HUB, by period; not its HUMAN PLAYER's)
// against its team's 2026 Statbotics EPA (tools/epa-2026.json).
//   node tools/epa-compare.mjs <batch.json | runner job id> [...] [--exclude 2910] [--only-vs 2910]
// --only-vs: only matches against that robot (a 1 v 1 reference set); --exclude: leave robots out.
// Prints, by EPA: EPA and its rank; the sim's points, rank and points per EPA point, scaled so the
// median robot is 1.00 (over 1: the sim overrates it); the same for AUTO and TELEOP; and the
// rank correlation (Spearman) over the robots.
import { spawnSync } from 'child_process';
import { existsSync, readFileSync } from 'fs';
import { dirname, join } from 'path';
import { fileURLToPath } from 'url';

const ROOT = join(dirname(fileURLToPath(import.meta.url)), '..');
const argv = process.argv.slice(2);
const arg = (k, d) => { const i = argv.indexOf('--' + k); return i >= 0 ? argv[i + 1] : d; };
const VALUED = new Set(['--exclude', '--only-vs']);
const files = argv.filter((a, i) => !a.startsWith('--') && !VALUED.has(argv[i - 1]));
const exclude = new Set((arg('exclude', '') || '').split(',').filter(Boolean));
const onlyVs = arg('only-vs');
const EPA = JSON.parse(readFileSync(join(ROOT, 'tools/epa-2026.json'), 'utf8')).teams;

const load = (src) => JSON.parse(existsSync(src) ? readFileSync(src, 'utf8')
  : spawnSync('git', ['show', `origin/batch-results:results/batch/${src.replace(/\.json$/, '')}.json`], { cwd: ROOT, maxBuffer: 1 << 28 }).stdout.toString());

const per = {};
for (const f of files) {
  for (const r of load(f).results) {
    if (!r.ok || !r.audit) continue;
    for (const x of r.audit) {
      // (a 1 v 1 reference set: only the robot facing the reference, plus the reference's self-match)
      if (onlyVs) { const vs = r.a === x.robot ? r.b : r.a; if (vs !== onlyVs || (r.a === r.b && x.alliance !== 'blue')) continue; }
      const p = (per[x.robot] ??= { n: 0, pts: 0, auto: 0, tele: 0 });
      const auto = x.pts.auto, tele = x.pts.trans + x.pts.a1 + x.pts.a2 + x.pts.end;
      p.n++; p.pts += auto + tele; p.auto += auto; p.tele += tele;
    }
  }
}
const keys = Object.keys(per).filter((k) => EPA[k] && !exclude.has(k));
const rows = keys.map((k) => {
  const p = per[k], e = EPA[k];
  return { k, n: p.n, pts: p.pts / p.n, auto: p.auto / p.n, tele: p.tele / p.n, epa: e.total, eAuto: e.auto, eTele: e.trans + e.a1 + e.a2 + e.end, eRank: e.rank };
});
const median = (a) => { const s = [...a].sort((x, y) => x - y); return s.length % 2 ? s[(s.length - 1) / 2] : (s[s.length / 2 - 1] + s[s.length / 2]) / 2; };
const mPts = median(rows.map((r) => r.pts / r.epa)), mAuto = median(rows.map((r) => r.auto / r.eAuto)), mTele = median(rows.map((r) => r.tele / r.eTele));
const rankOf = (f) => { const s = [...rows].sort((a, b) => f(b) - f(a)); return new Map(s.map((r, i) => [r.k, i + 1])); };
const simRank = rankOf((r) => r.pts), epaRank = rankOf((r) => r.epa);
const spearman = (fa, fb) => {
  const ra = rankOf(fa), rb = rankOf(fb), n = rows.length;
  return 1 - (6 * rows.reduce((s, r) => s + (ra.get(r.k) - rb.get(r.k)) ** 2, 0)) / (n * (n * n - 1));
};
console.log('robot  EPA (rank)   sim pts  n | rank EPA sim | x EPA  AUTO sim/EPA x   TELEOP sim/EPA x');
for (const r of [...rows].sort((a, b) => b.epa - a.epa)) {
  console.log(`${r.k.padEnd(5)} ${r.epa.toFixed(0).padStart(4)} (${String(r.eRank).padStart(4)}) ${r.pts.toFixed(0).padStart(8)} ${String(r.n).padStart(3)} | ${String(epaRank.get(r.k)).padStart(8)} ${String(simRank.get(r.k)).padStart(3)} | ${(r.pts / r.epa / mPts).toFixed(2)}  ${r.auto.toFixed(0).padStart(4)}/${r.eAuto.toFixed(0).padStart(3)} ${(r.auto / r.eAuto / mAuto).toFixed(2)}  ${r.tele.toFixed(0).padStart(6)}/${r.eTele.toFixed(0).padStart(3)} ${(r.tele / r.eTele / mTele).toFixed(2)}`);
}
console.log(`median sim/EPA: points ${mPts.toFixed(2)}, AUTO ${mAuto.toFixed(2)}, TELEOP ${mTele.toFixed(2)}; Spearman rank correlation: points ${spearman((r) => r.pts, (r) => r.epa).toFixed(2)}, AUTO ${spearman((r) => r.auto, (r) => r.eAuto).toFixed(2)}, TELEOP ${spearman((r) => r.tele, (r) => r.eTele).toFixed(2)}`);
