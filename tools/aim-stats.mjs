// What the aims know (js/localize.js): per robot, how far its pose estimate is off (all match long and
// while it shoots), and every HUB shot's distance, speed, how far off the HUB its estimate put, and
// whether it went in. Runs matches against champs 2910 (as the runner's batches) and writes one JSON
// line per match; --sum prints the tables from such files (before / after: AIMTRUTH=1 for before).
//   node --import ./tools/node-env.mjs tools/aim-stats.mjs --robot 6328 --seeds 1-2 --out a.jsonl
//   node tools/aim-stats.mjs --sum before.jsonl after.jsonl
import { appendFileSync, readFileSync } from 'fs';

const args = process.argv.slice(2);
const arg = (k, d) => { const i = args.indexOf('--' + k); return i >= 0 ? args[i + 1] : d; };
const range = (s) => s.split(',').flatMap((p) => { const [a, b] = p.split('-').map(Number); return b ? Array.from({ length: b - a + 1 }, (_, i) => a + i) : [a]; });
const pct = (xs, q) => { if (!xs.length) return NaN; const s = [...xs].sort((a, b) => a - b); return s[Math.min(s.length - 1, Math.floor(q * s.length))]; };

if (args.includes('--sum')) {
  const files = args.filter((a) => !a.startsWith('--'));
  for (const f of files) {
    const per = {};
    for (const l of readFileSync(f, 'utf8').split('\n').filter(Boolean)) {
      const m = JSON.parse(l), side = m.robots.A; // the robot under test (side A, blue)
      const p = (per[m.robot] ??= { n: 0, pts: 0, err: [], errShoot: [], hub: [], hubShoot: [], hubLive: [], th: [], shots: [] });
      p.n++; p.pts += side.pts;
      p.err.push(...side.err); p.errShoot.push(...side.errShoot); p.hub.push(...side.hub); p.hubShoot.push(...side.hubShoot); p.th.push(...side.thShoot); p.hubLive.push(...(side.hubLiveShoot ?? []));
      p.shots.push(...side.shots);
    }
    console.log(`\n${f}`);
    console.log('robot  m   pts | pose err cm (med/90) all  shooting | HUB err cm shooting aim / live | head deg 90 | HUB %  n  | <3m  3-5m  >5m | stand  moving(>0.5 m/s)');
    const acc = (s) => (s.length ? ((100 * s.filter((x) => x.in).length) / s.length).toFixed(0).padStart(4) : '   -');
    for (const [k, p] of Object.entries(per)) {
      const S = p.shots, f1 = (x) => (Number.isFinite(x) ? (100 * x).toFixed(1) : '-').padStart(5);
      console.log(`${k.padEnd(6)} ${p.n} ${(p.pts / p.n).toFixed(0).padStart(4)} | ${f1(pct(p.err, 0.5))} ${f1(pct(p.err, 0.9))}  ${f1(pct(p.errShoot, 0.5))} ${f1(pct(p.errShoot, 0.9))} | ${f1(pct(p.hubShoot, 0.5))} ${f1(pct(p.hubShoot, 0.9))} ${f1(pct(p.hubLive, 0.5))} ${f1(pct(p.hubLive, 0.9))} | ${(pct(p.th.map(Math.abs), 0.9) / (Math.PI / 180) || 0).toFixed(2).padStart(5)} | ${acc(S)} ${String(S.length).padStart(4)} | ${acc(S.filter((s) => s.d < 3))} ${acc(S.filter((s) => s.d >= 3 && s.d < 5))} ${acc(S.filter((s) => s.d >= 5))} | ${acc(S.filter((s) => s.v < 0.5))} ${acc(S.filter((s) => s.v >= 0.5))}`);
    }
  }
  process.exit(0);
}

const { runMatch } = await import('./headless.mjs');
const { Field } = await import('../js/field.js');
const robot = arg('robot', '6328'), opp = arg('opp', '2910'), out = arg('out', 'aim-stats.jsonl');
for (const seed of range(arg('seeds', '1'))) {
  const sides = {};
  let patched = false;
  const r0 = await runMatch({ strategy: 'scorer', skill: 'champs', robot }, { strategy: 'scorer', skill: 'champs', robot: opp }, {
    seed,
    trace(game, step) {
      const robots = { A: game.robot, B: game.opp.robot };
      if (!patched) {
        patched = true;
        for (const k of ['A', 'B']) sides[k] = { err: [], errShoot: [], hub: [], hubShoot: [], hubLiveShoot: [], thShoot: [], shots: [] };
        const fuel = game.fuel, live = new Map();
        const note = (b, info) => {
          const k = Object.keys(robots).find((x) => robots[x].alliance === info?.alliance);
          if (!k || info.by !== 'robot') return;
          const r = robots[k], mode = r.shot?.mode ?? r.lastMode;
          if (mode !== 'hub') return;
          const c = Field.hubCenter(r.alliance), e = r.loc.error();
          const s = { d: +Math.hypot(c.x - r.pos.x, c.z - r.pos.z).toFixed(2), v: +Math.hypot(r.vel.x, r.vel.z).toFixed(2), hub: e ? +e.hub.toFixed(3) : 0, hubLive: e ? +e.hubLive.toFixed(3) : 0, w: +Math.abs(r.omega).toFixed(2), in: false, t: +(info.t ?? 0).toFixed(2) };
          sides[k].shots.push(s);
          live.set(b, s);
        };
        const L = fuel.launch.bind(fuel), R = fuel.release.bind(fuel), H = fuel._enterHub.bind(fuel);
        fuel.launch = (b, p, v, info = {}) => { const x = L(b, p, v, info); note(b, info); return x; };
        fuel.release = (b, info = {}) => { const x = R(b, info); note(b, info); return x; };
        fuel._enterHub = (b, a, t) => { const s = live.get(b); if (s) { s.in = true; live.delete(b); } return H(b, a, t); };
      }
      if (step % 6) return;
      for (const k of ['A', 'B']) {
        const r = robots[k], e = r.loc.error();
        if (!e || !r.enabled) continue;
        const sd = sides[k];
        sd.err.push(+e.live.toFixed(3)); sd.hub.push(+e.hub.toFixed(3));
        if (r.wantShoot && r.shot?.mode === 'hub') { sd.errShoot.push(+e.live.toFixed(3)); sd.hubShoot.push(+e.hub.toFixed(3)); sd.thShoot.push(+e.th.toFixed(4)); sd.hubLiveShoot.push(+e.hubLive.toFixed(3)); }
      }
    },
  });
  // a FUEL in a HUB is the robot's whose ALLIANCE launched it (1v1: its own HUB)
  const line = { robot, opp, seed, aimTruth: process.env.AIMTRUTH === '1', robots: { A: { pts: r0.a.total, ...sides.A }, B: { pts: r0.b.total, ...sides.B } } };
  appendFileSync(out, JSON.stringify(line) + '\n');
  const S = sides.A.shots;
  console.log(`${robot} v ${opp} seed ${seed}: ${r0.a.total}-${r0.b.total}, HUB shots ${S.length}, in ${S.filter((s) => s.in).length}, pose err med ${(100 * pct(sides.A.err, 0.5)).toFixed(1)} cm`);
}
