// Where each robot's match goes: sums the AUDIT rows (tools/audit.mjs) of match batches per robot.
//   node tools/ai-audit.mjs <batch.json | runner job id> [...]  [--robot 1690] [--episodes] [--labels] [--vs <base>]
//   node tools/ai-audit.mjs --run -- <tools/batch.mjs options>   (runs the batch here first)
// A job id (2026-11-01T18-55-37-x) is read from the `batch-results` branch (git fetch it first).
// Per robot, means per match: FUEL it scored itself by period (AUTO, TRANSITION, its first and
// second active SHIFT, END GAME; wasted into an inactive HUB), launched / passes / intaked, its AI's
// unsticks, the TELEOP seconds idle / stuck (and against what) / staged / holding FUEL it can't get
// out, and the labels it was idle under. --vs prints each robot's change against another set.
import { spawnSync } from 'child_process';
import { existsSync, readFileSync } from 'fs';
import { dirname, join } from 'path';
import { fileURLToPath } from 'url';

const ROOT = join(dirname(fileURLToPath(import.meta.url)), '..');
const argv = process.argv.slice(2);
const dd = argv.indexOf('--');
const own = dd >= 0 ? argv.slice(0, dd) : argv, rest = dd >= 0 ? argv.slice(dd + 1) : [];
const arg = (k, d) => { const i = own.indexOf('--' + k); return i >= 0 ? own[i + 1] : d; };
const VALUED = new Set(['--robot', '--vs', '--name', '--top']);
const files = own.filter((a, i) => !a.startsWith('--') && !VALUED.has(own[i - 1]));

function load(src) {
  let text;
  if (existsSync(src)) text = readFileSync(src, 'utf8');
  else {
    const r = spawnSync('git', ['show', `origin/batch-results:results/batch/${src.replace(/\.json$/, '')}.json`], { cwd: ROOT, maxBuffer: 1 << 28 });
    if (r.status !== 0) throw new Error(`no such file or job: ${src}`);
    text = r.stdout.toString();
  }
  return JSON.parse(text);
}

if (own.includes('--run')) {
  const name = arg('name', `audit-${Date.now()}`);
  const r = spawnSync(process.execPath, ['tools/batch.mjs', ...rest, '--name', name], { cwd: ROOT, stdio: 'inherit' });
  if (r.status !== 0) process.exit(r.status ?? 1);
  files.push(join(ROOT, 'results/batch', `${name}.json`));
}

// one entry per robot, every match it played (in 3 v 3 each robot of an alliance is its own row)
function collect(list) {
  const per = {};
  for (const f of list) {
    for (const res of load(f).results) {
      if (!res.ok || !res.audit) continue;
      for (const x of res.audit) (per[x.robot] ??= []).push({ ...x, seed: res.seed, pair: `${res.a}v${res.b}` });
    }
  }
  return per;
}
const mean = (rows, f) => rows.reduce((s, x) => s + (f(x) || 0), 0) / Math.max(1, rows.length);
const total = (x) => x.pts.auto + x.pts.trans + x.pts.a1 + x.pts.a2 + x.pts.end;
function stats(rows) {
  const o = {
    n: rows.length, pts: mean(rows, total), auto: mean(rows, (x) => x.pts.auto), trans: mean(rows, (x) => x.pts.trans),
    a1: mean(rows, (x) => x.pts.a1), a2: mean(rows, (x) => x.pts.a2), end: mean(rows, (x) => x.pts.end), wasted: mean(rows, (x) => x.pts.inactive),
    launched: mean(rows, (x) => x.launched), passes: mean(rows, (x) => x.passes), intaked: mean(rows, (x) => x.intaked),
    unsticks: mean(rows, (x) => x.unsticks), unjams: mean(rows, (x) => x.unjams), cycles: mean(rows, (x) => x.cycles),
    first: mean(rows.filter((x) => x.firstScore != null), (x) => x.firstScore),
  };
  for (const k of ['idle', 'stuck', 'staged', 'holdIdle', 'carryOut', 'inZoneOff', 'emptyActive']) o[k] = mean(rows, (x) => x.sec[k]);
  // TELEOP FUEL a second its HUB is active (90 s: TRANSITION, two SHIFTS, END GAME)
  o.fps = (o.trans + o.a1 + o.a2 + o.end) / 90;
  const sumBy = (k) => { const s = {}; for (const x of rows) for (const [a, v] of Object.entries(x[k] || {})) s[a] = (s[a] || 0) + v / rows.length; return s; };
  o.stuckBy = sumBy('stuckBy'); o.idleBy = sumBy('idleBy'); o.labels = sumBy('labels');
  return o;
}
const fmtTop = (o, n = 4) => Object.entries(o).sort((a, b) => b[1] - a[1]).slice(0, n).filter(([, v]) => v >= 0.3).map(([k, v]) => `${k} ${v.toFixed(1)}`).join(', ');

const per = collect(files);
const base = arg('vs') ? collect([arg('vs')]) : null;
const robots = Object.keys(per).filter((k) => !arg('robot') || k === arg('robot')).sort((a, b) => stats(per[b]).pts - stats(per[a]).pts);
const f1 = (v, w = 6) => v.toFixed(1).padStart(w);
console.log('robot    n    pts  AUTO trans   a1    a2   end waste FUEL/s launch  pass unstk | idle stuck staged hold carryOut zoneOff');
for (const k of robots) {
  const s = stats(per[k]);
  console.log(`${k.padEnd(6)} ${String(s.n).padStart(3)} ${f1(s.pts)} ${f1(s.auto, 5)} ${f1(s.trans, 5)} ${f1(s.a1, 5)} ${f1(s.a2, 5)} ${f1(s.end, 5)} ${f1(s.wasted, 5)} ${s.fps.toFixed(2).padStart(6)} ${f1(s.launched)} ${f1(s.passes, 5)} ${f1(s.unsticks, 5)} | ${f1(s.idle, 4)} ${f1(s.stuck, 5)} ${f1(s.staged, 6)} ${f1(s.holdIdle, 4)} ${f1(s.carryOut, 8)} ${f1(s.inZoneOff, 7)}`);
  if (base?.[k]) {
    const b = stats(base[k]);
    console.log(`  vs base (${b.n}): pts ${f1(s.pts - b.pts, 0)}, AUTO ${f1(s.auto - b.auto, 0)}, idle ${f1(s.idle - b.idle, 0)}, stuck ${f1(s.stuck - b.stuck, 0)}, launched ${f1(s.launched - b.launched, 0)}`);
  }
}
console.log('\ncollecting: seconds a match and FUEL taken in a second there (own zone / NEUTRAL ZONE / their zone)');
for (const k of robots) {
  const rows = per[k].filter((x) => x.coll);
  if (!rows.length) continue;
  const f = (w) => { const s = mean(rows, (x) => x.coll[w]), n = mean(rows, (x) => x.took[w]); return `${s.toFixed(0).padStart(3)} s ${(n / Math.max(1, s)).toFixed(2)}/s`; };
  console.log(`${k.padEnd(6)} own ${f('own')} | mid ${f('mid')} | opp ${f('opp')}`);
}
console.log('\nshooting at its HUB: seconds a match and FUEL launched a second, moving | standing');
for (const k of robots) {
  const rows = per[k].filter((x) => x.fire);
  if (!rows.length) continue;
  const f = (T, N) => { const t = mean(rows, (x) => x.fire[T]), n = mean(rows, (x) => x.fire[N]); return `${t.toFixed(0).padStart(3)} s ${(n / Math.max(0.1, t)).toFixed(1).padStart(4)}/s`; };
  console.log(`${k.padEnd(6)} moving ${f('moveT', 'moveN')} | standing ${f('standT', 'standN')}`);
}
console.log('\nstuck against (s a match) / idle while (s a match) / first FUEL in TELEOP (s) / unjams');
for (const k of robots) {
  const s = stats(per[k]);
  console.log(`${k.padEnd(6)} stuck: ${fmtTop(s.stuckBy) || '-'} | idle: ${fmtTop(s.idleBy) || '-'} | first ${s.first.toFixed(1)} | unjams ${s.unjams.toFixed(1)}`);
}
if (own.includes('--labels')) {
  console.log('\nseconds a match by what its AI said it was doing: all TELEOP | its first 35 s');
  for (const k of robots) {
    const s = stats(per[k]), rows = per[k].filter((x) => x.early);
    const early = {};
    for (const x of rows) for (const [a, v] of Object.entries(x.early)) early[a] = (early[a] || 0) + v / rows.length;
    console.log(`${k.padEnd(6)} ${fmtTop(s.labels, 6)} | ${fmtTop(early, 5)}`);
  }
}
if (own.includes('--episodes')) {
  console.log('\nlongest stuck stretches');
  for (const k of robots) {
    const eps = per[k].flatMap((x) => (x.stuckEp || []).map((e) => ({ ...e, seed: x.seed, pair: x.pair }))).sort((a, b) => b.s - a.s).slice(0, +arg('top', 6));
    for (const e of eps) console.log(`${k.padEnd(6)} ${e.pair} seed ${e.seed}: ${e.s} s at t ${e.t} (${e.x}, ${e.z}) ${e.by}, ${e.label}`);
  }
}
