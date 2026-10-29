// FUEL ballistics with quadratic air drag. The same model is applied to every airborne
// FUEL in the physics world (see fuel.js), so shot tables computed here match what the
// simulated balls actually do.
import { FUEL, GRAVITY } from './constants.js';
import { DEG } from './util.js';

const K = FUEL.dragK;
const SIM_DT = 1 / 240;

const XS = new Float32Array(1802), YS = new Float32Array(1802);

// Planar shot from height h0 at speed v and elevation th. Returns the point where the
// ball DESCENDS through height Ht, or null if it never gets there. `clearY` is the ball's
// height `clearDist` before that point (used to make sure shots clear the HUB's front edge).
export function sim2D(h0, v, th, Ht, clearDist = 0) {
  let x = 0, y = h0, vx = v * Math.cos(th), vy = v * Math.sin(th), t = 0, apex = h0;
  for (let i = 0; i < 1800; i++) {
    XS[i] = x;
    YS[i] = y;
    const sp = Math.hypot(vx, vy);
    vx += -K * sp * vx * SIM_DT;
    vy += (-GRAVITY - K * sp * vy) * SIM_DT;
    const nx = x + vx * SIM_DT, ny = y + vy * SIM_DT;
    t += SIM_DT;
    if (ny > apex) apex = ny;
    if (vy < 0 && ny <= Ht) {
      if (y >= Ht) {
        const f = (y - Ht) / (y - ny);
        const d = x + (nx - x) * f;
        let clearY = Infinity;
        const xc = d - clearDist;
        if (clearDist > 0 && xc > 0) {
          for (let j = i; j > 0; j--) {
            if (XS[j - 1] <= xc && XS[j] >= xc) {
              const g = (xc - XS[j - 1]) / (XS[j] - XS[j - 1] || 1);
              clearY = YS[j - 1] + (YS[j] - YS[j - 1]) * g;
              break;
            }
          }
        }
        return { d, tof: t - SIM_DT + SIM_DT * f, alpha: Math.atan2(-vy, vx), apex, clearY };
      }
      return null;
    }
    x = nx;
    y = ny;
  }
  return null;
}

// Full 3D flight including the launcher's own velocity. Returns where the ball descends
// through Ht (x, z, tof) or null.
export function sim3D(p, v, Ht) {
  let x = p.x, y = p.y, z = p.z, vx = v.x, vy = v.y, vz = v.z, t = 0;
  for (let i = 0; i < 1800; i++) {
    const sp = Math.hypot(vx, vy, vz);
    vx += -K * sp * vx * SIM_DT;
    vy += (-GRAVITY - K * sp * vy) * SIM_DT;
    vz += -K * sp * vz * SIM_DT;
    const nx = x + vx * SIM_DT, ny = y + vy * SIM_DT, nz = z + vz * SIM_DT;
    t += SIM_DT;
    if (vy < 0 && ny <= Ht) {
      if (y >= Ht) {
        const f = (y - Ht) / (y - ny);
        return { x: x + (nx - x) * f, z: z + (nz - z) * f, tof: t - SIM_DT + SIM_DT * f };
      }
      return null;
    }
    x = nx; y = ny; z = nz;
  }
  return null;
}

// Sample points of a trajectory (for the aim preview line)
export function trajectoryPoints(p, v, maxT = 3, floorY = 0.05) {
  const pts = [];
  let x = p.x, y = p.y, z = p.z, vx = v.x, vy = v.y, vz = v.z;
  const dt = 1 / 60;
  for (let t = 0; t < maxT; t += dt) {
    pts.push(x, y, z);
    for (let k = 0; k < 4; k++) {
      const sp = Math.hypot(vx, vy, vz);
      const h = dt / 4;
      vx += -K * sp * vx * h;
      vy += (-GRAVITY - K * sp * vy) * h;
      vz += -K * sp * vz * h;
      x += vx * h; y += vy * h; z += vz * h;
    }
    if (y < floorY) break;
  }
  return pts;
}

// Lookup table: horizontal distance -> {theta, v, tof, alpha}
// A pass table (mode 'pass') lobs at passTheta wherever that reaches; past it, at the hood angle
// nearest passTheta that still gets there (flatter, toward the angle that throws farthest). Past
// everything it can reach, lookup() gives its longest throw (`far`: the hood angle that carries
// farthest in the drag model, at speedMax, flagged reach: false): a pass from out of reach still
// goes, as far as the shooter throws, and comes down short.
export class ShotTable {
  constructor({ h0, Ht, hoodMin, hoodMax, speedMin = 2.5, speedMax, dMin = 0.4, dMax = 16, step = 0.05, mode = 'hub', passTheta = 55, clearDist = 0, clearHeight = 0 }) {
    this.h0 = h0;
    this.Ht = Ht;
    this.dMin = dMin;
    this.step = step;
    const thetas = [];
    const lob = Math.min(Math.max(passTheta, hoodMin), hoodMax);
    // (a pass's flatter angles, every 2 deg down to hoodMin, only matter past the lob's reach: fast
    // shots. In the drag model the longest throw is at about 41-44 deg, flatter than the lob)
    if (mode === 'pass') { thetas.push(lob); for (let t = lob - 2; t > hoodMin - 1e-6; t -= 2) thetas.push(t); if (lob > hoodMin + 1e-6 && (lob - hoodMin) % 2) thetas.push(hoodMin); }
    else for (let t = hoodMin; t <= hoodMax + 1e-6; t += 1) thetas.push(t);

    // For each hood angle, sweep exit speed (h0: the exit height, or a function of the hood
    // angle when where FUEL leaves moves with the hood)
    const curves = thetas.map((tdeg, k) => {
      const th = tdeg * DEG;
      const h = typeof h0 === 'function' ? h0(tdeg) : h0;
      const pts = [];
      for (let v = mode === 'pass' && k > 0 ? Math.max(speedMin, 0.5 * speedMax) : speedMin; v <= speedMax + 1e-6; v += 0.1) {
        const r = sim2D(h, v, th, Ht, clearDist);
        if (r) pts.push({ v, ...r });
      }
      // (a pass's longest throw at exactly speedMax, where the 0.1 m/s steps stop short of it)
      const last = pts[pts.length - 1];
      if (mode === 'pass' && last && last.v < speedMax - 1e-6) { const r = sim2D(h, speedMax, th, Ht, clearDist); if (r) pts.push({ v: speedMax, ...r }); }
      return { th, pts };
    });

    const n = Math.floor((dMax - dMin) / step) + 1;
    this.entries = new Array(n).fill(null);
    for (let i = 0; i < n; i++) {
      const D = dMin + i * step;
      let best = null, bestCost = Infinity;
      for (const c of curves) {
        const p = c.pts;
        for (let j = 0; j + 1 < p.length; j++) {
          const a = p[j], b = p[j + 1];
          if ((a.d - D) * (b.d - D) > 0) continue;
          const f = b.d === a.d ? 0 : (D - a.d) / (b.d - a.d);
          const cand = {
            theta: c.th,
            v: a.v + (b.v - a.v) * f,
            tof: a.tof + (b.tof - a.tof) * f,
            alpha: a.alpha + (b.alpha - a.alpha) * f,
            apex: a.apex + (b.apex - a.apex) * f,
            clearY: Math.min(a.clearY, b.clearY),
          };
          if (clearHeight && cand.clearY < clearHeight) break; // would clip the HUB's front edge
          let cost;
          if (mode === 'hub') {
            const ad = cand.alpha / DEG;
            cost = cand.tof + (ad < 52 ? (52 - ad) * 0.06 : 0) + (ad < 38 ? 100 : 0);
          } else cost = Math.abs(c.th / DEG - lob) * 100 + cand.tof; // (passTheta's own lob wherever it reaches)
          if (cost < bestCost) { bestCost = cost; best = cand; }
          break;
        }
      }
      this.entries[i] = best;
    }
    // valid range
    this.minD = Infinity;
    this.maxD = -Infinity;
    this.entries.forEach((e, i) => {
      if (!e) return;
      const d = dMin + i * step;
      this.minD = Math.min(this.minD, d);
      this.maxD = Math.max(this.maxD, d);
    });
    // a pass table's longest throw: flat out, at the hood angle that carries farthest
    this.far = null;
    if (mode === 'pass') {
      for (const c of curves) {
        const p = c.pts[c.pts.length - 1];
        if (p && (!this.far || p.d > this.far.d)) this.far = { theta: c.th, v: p.v, tof: p.tof, alpha: p.alpha, d: p.d, reach: false };
      }
    }
  }

  lookup(d) {
    // (past what a pass table reaches: its longest throw, which comes down short)
    if (this.far && d > this.maxD) return this.far;
    const fi = (d - this.dMin) / this.step;
    let i = Math.floor(fi);
    if (i < 0 || i + 1 >= this.entries.length) return null;
    const a = this.entries[i], b = this.entries[i + 1];
    if (!a || !b) return a || b || null;
    const f = fi - i;
    return {
      theta: a.theta + (b.theta - a.theta) * f,
      v: a.v + (b.v - a.v) * f,
      tof: a.tof + (b.tof - a.tof) * f,
      alpha: a.alpha + (b.alpha - a.alpha) * f,
    };
  }

  inRange(d) {
    return d >= this.minD && d <= this.maxD && !!this.lookup(d);
  }
}

// Shoot-on-the-move solution. `exit` = launch point (world), `vel` = launcher velocity
// (world, horizontal x/z), `target` = aim point (x,z) at height table.Ht.
// Returns heading psi (robot yaw convention: forward = (cos psi, -sin psi)), hood theta,
// exit speed v, tof and the "virtual target" the shooter points at.
export function solveMovingShot(table, exit, vel, target) {
  let ax = target.x, az = target.z;
  let sol = null;
  // a pass from out of reach (from where it is, or once led for the launcher's motion): its longest
  // throw, straight along the line to the target (no lead: it can't get there anyway, and a long
  // throw's lead swings the aim round with every change in the robot's velocity)
  const d0 = Math.hypot(ax - exit.x, az - exit.z);
  const straight = () => {
    const f = table.far;
    return { psi: Math.atan2(-(target.z - exit.z), target.x - exit.x), theta: f.theta, v: f.v, tof: f.tof, dist: d0, aim: { x: target.x, z: target.z }, reach: false };
  };
  if (table.far && d0 > table.maxD) return straight();
  for (let i = 0; i < 5; i++) {
    const d = Math.hypot(ax - exit.x, az - exit.z);
    sol = table.lookup(d);
    if (!sol) return null;
    if (sol.reach === false) return straight();
    ax = target.x - vel.x * sol.tof;
    az = target.z - vel.z * sol.tof;
  }
  // refine against the full 3D flight (drag couples launcher velocity and shot)
  for (let k = 0; k < 3; k++) {
    const dx = ax - exit.x, dz = az - exit.z;
    const d = Math.hypot(dx, dz);
    sol = table.lookup(d);
    if (!sol) return null;
    if (sol.reach === false) return straight();
    const ch = Math.cos(sol.theta) * sol.v / d;
    const v3 = { x: dx * ch + vel.x, y: Math.sin(sol.theta) * sol.v, z: dz * ch + vel.z };
    const hit = sim3D(exit, v3, table.Ht);
    if (!hit) break;
    const ex = hit.x - target.x, ez = hit.z - target.z;
    ax -= ex;
    az -= ez;
    if (Math.hypot(ex, ez) < 0.01) break;
  }
  const dx = ax - exit.x, dz = az - exit.z;
  const d = Math.hypot(dx, dz);
  sol = table.lookup(d);
  if (!sol) return null;
  if (sol.reach === false) return straight();
  return { psi: Math.atan2(-dz, dx), theta: sol.theta, v: sol.v, tof: sol.tof, dist: d, aim: { x: ax, z: az }, reach: true };
}
