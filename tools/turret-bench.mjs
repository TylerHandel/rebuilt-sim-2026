// The physical turrets on their motors (js/mech/turretDrive.js), with and without their energy
// chains (js/echain.js): how long a swing across their travel takes, how long they take to settle on
// a new aim, and where they come to rest.
//   node tools/turret-bench.mjs
import { drive8793, drive4414 } from '../js/mech/turretDrive.js';

const DEG = Math.PI / 180, DT = 1 / 120;

// from a (deg) told to go to b (deg): when it's first within `near`, when it's within `tol` for good
// (and stays 0.25 s), where it is after 2 s, the most current
function run(make, a, b, { tol = 0.6, near = 2 } = {}) {
  const d = make();
  let yaw = a * DEG, tNear = null, tIn = null, peak = 0, over = 0;
  // (rest at a first)
  for (let i = 0; i < 120; i++) yaw = d.step(yaw, a * DEG, DT);
  for (let i = 0; i < 2.5 / DT; i++) {
    yaw = d.step(yaw, b * DEG, DT);
    const t = (i + 1) * DT, e = Math.abs(yaw / DEG - b);
    peak = Math.max(peak, Math.abs(d.current));
    if (Math.sign(b - a) * (yaw / DEG - b) > over) over = Math.sign(b - a) * (yaw / DEG - b);
    if (tNear === null && e < near) tNear = t;
    if (e < tol) { if (tIn === null) tIn = t; } else tIn = null;
  }
  return { tNear, tIn, end: yaw / DEG - b, peak, over };
}

const fmt = (r) => `${r.tNear === null ? '  -  ' : r.tNear.toFixed(3)} s to 2 deg, ${r.tIn === null ? 'never' : r.tIn.toFixed(3) + ' s'} to stay in 0.6, overshoot ${r.over.toFixed(2)} deg, rest ${r.end >= 0 ? '+' : ''}${r.end.toFixed(3)} deg, ${r.peak.toFixed(0)} A`;
for (const [name, make, lo, hi, mid] of [['8793', drive8793, -200, 270, 35], ['4414', drive4414, -200, 200, 0]]) {
  console.log(`${name}:`);
  for (const chain of [false, true]) {
    const mk = () => make({ chain });
    console.log(`  ${chain ? 'with its chain' : 'no chain'}:`);
    console.log(`    across its travel ${lo + 5} -> ${hi - 5}: ${fmt(run(mk, lo + 5, hi - 5))}`);
    console.log(`    back ${hi - 5} -> ${lo + 5}: ${fmt(run(mk, hi - 5, lo + 5))}`);
    for (const step of [10, 30, 90, 180]) {
      console.log(`    ${step} deg (${mid} -> ${mid + step}): ${fmt(run(mk, mid, mid + step))}`);
      console.log(`    ${step} deg (${mid} -> ${mid - step}): ${fmt(run(mk, mid, mid - step))}`);
    }
  }
}
