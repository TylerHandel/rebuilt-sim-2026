// 2056 in an AI match (scorer, champs, against a champs 2910 scorer): each HUB shot it takes, where it
// comes down through the rim's height against the HUB's center (along the line from where it left:
// + long, - short; and aside), by the distance it was shot from and how fast the robot moved, and how
// many score. Run:
//   node --import ./tools/node-env.mjs tools/shots2056.mjs [seed]   (ROBOT=1690 for another)
import { runMatch } from './headless.mjs';
import { Field } from '../js/field.js';
import { HUB } from '../js/constants.js';

const seed = Number(process.argv[2] ?? 1), key = process.env.ROBOT ?? '2056';
const live = new Map(), done = [];
const res = await runMatch({ strategy: 'scorer', skill: 'champs', robot: key }, { strategy: 'scorer', skill: 'champs', robot: '2910' }, {
  seed,
  trace(game) {
    const r = game.robot, ph = r.phys;
    if (!ph) return;
    for (const [b] of ph.shot) {
      if (live.has(b) || done.some((s) => s.b === b)) continue;
      const tgt = r.shot?.target;
      if (tgt?.mode === 'pass') { done.push({ b, pass: true }); continue; }
      const c = Field.hubCenter(r.alliance), v = r.body.linvel();
      live.set(b, { b, c, from: { x: b.pos.x, z: b.pos.z }, d: Math.hypot(c.x - b.pos.x, c.z - b.pos.z), speed: Math.hypot(v.x, v.z), y: b.pos.y });
    }
    for (const [b, s] of live) {
      const p = b.pos;
      if (b.inHub) s.scored = true;
      if (s.y >= HUB.rimFront && p.y < HUB.rimFront && !s.at) {
        const ux = (s.c.x - s.from.x) / s.d, uz = (s.c.z - s.from.z) / s.d, dx = p.x - s.c.x, dz = p.z - s.c.z;
        s.at = { along: dx * ux + dz * uz, side: -dx * uz + dz * ux };
      }
      s.y = p.y;
      if (s.at && (p.y < 0.3 || s.scored)) { live.delete(b); done.push(s); }
    }
  },
});
for (const s of live.values()) done.push(s);
const hub = done.filter((s) => !s.pass);
const med = (a) => { const b = [...a].sort((p, q) => p - q); return b.length ? b[b.length >> 1] : NaN; };
const rows = [[0, 2.5], [2.5, 3.5], [3.5, 4.5], [4.5, 9]];
console.log(`${key} vs 2910 seed ${seed}: ${JSON.stringify(res.a)} (2910 ${res.b.total})`);
for (const [lo, hi] of rows) for (const mv of [false, true]) {
  const g = hub.filter((s) => s.d >= lo && s.d < hi && (s.speed > 0.5) === mv);
  if (!g.length) continue;
  const at = g.filter((s) => s.at);
  console.log(`  ${lo}-${hi} m ${mv ? 'moving' : 'still '}: ${g.length} shots, ${g.filter((s) => s.scored).length} scored; down at the rim along ${med(at.map((s) => s.at.along)).toFixed(2)} (median), aside ${med(at.map((s) => Math.abs(s.at.side))).toFixed(2)} (|median|); robot ${med(g.map((s) => s.speed)).toFixed(1)} m/s`);
}
console.log(`  passes: ${done.length - hub.length}`);
