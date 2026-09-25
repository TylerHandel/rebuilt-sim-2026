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
export class ShotTable {
  constructor({ h0, Ht, hoodMin, hoodMax, speedMin = 2.5, speedMax, dMin = 0.4, dMax = 16, step = 0.05, mode = 'hub', passTheta = 55, clearDist = 0, clearHeight = 0 }) {
    this.h0 = h0;
    this.Ht = Ht;
    this.dMin = dMin;
    this.step = step;
    const thetas = [];
    if (mode === 'pass') thetas.push(Math.min(Math.max(passTheta, hoodMin), hoodMax));
    else for (let t = hoodMin; t <= hoodMax + 1e-6; t += 1) thetas.push(t);

    // For each hood angle, sweep exit speed
    const curves = thetas.map((tdeg) => {
      const th = tdeg * DEG;
      const pts = [];
      for (let v = speedMin; v <= speedMax + 1e-6; v += 0.1) {
        const r = sim2D(h0, v, th, Ht, clearDist);
        if (r) pts.push({ v, ...r });
      }
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
          } else cost = cand.tof;
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
  }

  lookup(d) {
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
  for (let i = 0; i < 5; i++) {
    const d = Math.hypot(ax - exit.x, az - exit.z);
    sol = table.lookup(d);
    if (!sol) return null;
    ax = target.x - vel.x * sol.tof;
    az = target.z - vel.z * sol.tof;
  }
  // refine against the full 3D flight (drag couples launcher velocity and shot)
  for (let k = 0; k < 3; k++) {
    const dx = ax - exit.x, dz = az - exit.z;
    const d = Math.hypot(dx, dz);
    sol = table.lookup(d);
    if (!sol) return null;
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
  return { psi: Math.atan2(-dz, dx), theta: sol.theta, v: sol.v, tof: sol.tof, dist: d, aim: { x: ax, z: az } };
}
