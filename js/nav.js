// Grid A* navigation for the AI opponent. The only things a ROBOT can't drive over or under are
// the HUBS, the bump-side TRENCH posts, the TOWERS and the field perimeter; BUMPS and the
// TRENCH openings are free space.
import { HALF_L, HALF_W, HUB, TRENCH, TOWER, BLUE, RED } from './constants.js';
import { Field } from './field.js';

const CELL = 0.2;
const NX = Math.ceil((2 * HALF_L) / CELL);
const NZ = Math.ceil((2 * HALF_W) / CELL);
const SQ2 = Math.SQRT2;

// Axis-aligned obstacle rectangles in world coordinates: { x, z, hx, hz } (center, half extents)
export const OBSTACLES = (() => {
  const out = [];
  for (const a of [BLUE, RED]) {
    const s = a === BLUE ? 1 : -1;
    const hx = Field.hubCenter(a).x;
    const hs = HUB.size / 2;
    out.push({ x: hx, z: 0, hx: hs, hz: hs, kind: 'hub', alliance: a });
    const zOpen = HALF_W - TRENCH.clearWidth, zPost = HALF_W - TRENCH.width;
    for (const sz of [1, -1]) out.push({ x: hx, z: (sz * (zOpen + zPost)) / 2, hx: TRENCH.depth / 2, hz: (zOpen - zPost) / 2, kind: 'trench', alliance: a });
    out.push({ x: s * (-HALF_L + TOWER.baseDepth / 2), z: s * (HALF_W - TOWER.fy), hx: TOWER.baseDepth / 2, hz: TOWER.width / 2, kind: 'tower', alliance: a });
  }
  return out;
})();

export function towerRect(alliance) {
  return OBSTACLES.find((o) => o.kind === 'tower' && o.alliance === alliance);
}

// true if (x, z) is inside an obstacle (grown by pad) or outside the field
export function obstacleAt(x, z, pad = 0) {
  if (Math.abs(x) > HALF_L - pad || Math.abs(z) > HALF_W - pad) return true;
  for (const o of OBSTACLES) if (Math.abs(x - o.x) < o.hx + pad && Math.abs(z - o.z) < o.hz + pad) return true;
  return false;
}

// Minimal binary heap keyed by f-score
class Heap {
  constructor() { this.a = []; }
  get size() { return this.a.length; }
  push(n, f) {
    const a = this.a;
    a.push([f, n]);
    let i = a.length - 1;
    while (i > 0) {
      const p = (i - 1) >> 1;
      if (a[p][0] <= a[i][0]) break;
      [a[p], a[i]] = [a[i], a[p]];
      i = p;
    }
  }
  pop() {
    const a = this.a;
    const top = a[0];
    const last = a.pop();
    if (a.length) {
      a[0] = last;
      let i = 0;
      for (;;) {
        const l = 2 * i + 1, r = l + 1;
        let m = i;
        if (l < a.length && a[l][0] < a[m][0]) m = l;
        if (r < a.length && a[r][0] < a[m][0]) m = r;
        if (m === i) break;
        [a[m], a[i]] = [a[i], a[m]];
        i = m;
      }
    }
    return top[1];
  }
}

export class NavGrid {
  constructor(radius) {
    this.r = radius;
    this.blocked = new Uint8Array(NX * NZ);
    for (let i = 0; i < NX; i++) {
      for (let j = 0; j < NZ; j++) {
        const x = -HALF_L + (i + 0.5) * CELL, z = -HALF_W + (j + 0.5) * CELL;
        this.blocked[i * NZ + j] = obstacleAt(x, z, radius) ? 1 : 0;
      }
    }
    this.g = new Float32Array(NX * NZ);
    this.from = new Int32Array(NX * NZ);
    this.stamp = new Uint32Array(NX * NZ);
    this.closed = new Uint32Array(NX * NZ);
    this.dyn = new Uint32Array(NX * NZ);
    this.run = 0;
    this.dynCount = 0;
    this.dynRun = -1; // cells stamped with this run are temporarily blocked (moving robots)
  }

  _blocked(n) { return this.blocked[n] || this.dyn[n] === this.dynRun; }

  // mark circles { x, z, r } as blocked for the current plan() call
  _markDyn(circles) {
    this.dynRun = ++this.dynCount;
    for (const c of circles) {
      const R = c.r + this.r;
      const i0 = Math.max(0, Math.floor((c.x - R + HALF_L) / CELL)), i1 = Math.min(NX - 1, Math.floor((c.x + R + HALF_L) / CELL));
      const j0 = Math.max(0, Math.floor((c.z - R + HALF_W) / CELL)), j1 = Math.min(NZ - 1, Math.floor((c.z + R + HALF_W) / CELL));
      for (let i = i0; i <= i1; i++) {
        for (let j = j0; j <= j1; j++) {
          const x = -HALF_L + (i + 0.5) * CELL, z = -HALF_W + (j + 0.5) * CELL;
          if (Math.hypot(x - c.x, z - c.z) < R) this.dyn[i * NZ + j] = this.dynRun;
        }
      }
    }
  }

  cell(x, z) {
    const i = Math.min(NX - 1, Math.max(0, Math.floor((x + HALF_L) / CELL)));
    const j = Math.min(NZ - 1, Math.max(0, Math.floor((z + HALF_W) / CELL)));
    return i * NZ + j;
  }

  center(c) {
    const i = Math.floor(c / NZ), j = c % NZ;
    return { x: -HALF_L + (i + 0.5) * CELL, z: -HALF_W + (j + 0.5) * CELL };
  }

  free(x, z) { return !this._blocked(this.cell(x, z)); }

  // nearest free cell (breadth-first rings)
  nearestFree(c) {
    if (!this._blocked(c)) return c;
    const i0 = Math.floor(c / NZ), j0 = c % NZ;
    for (let rad = 1; rad < 20; rad++) {
      let best = -1, bd = 1e9;
      for (let di = -rad; di <= rad; di++) {
        for (let dj = -rad; dj <= rad; dj++) {
          if (Math.max(Math.abs(di), Math.abs(dj)) !== rad) continue;
          const i = i0 + di, j = j0 + dj;
          if (i < 0 || j < 0 || i >= NX || j >= NZ) continue;
          const n = i * NZ + j;
          const d = di * di + dj * dj;
          if (!this._blocked(n) && d < bd) { bd = d; best = n; }
        }
      }
      if (best >= 0) return best;
    }
    return c;
  }

  // straight line between two points stays in free cells
  los(x0, z0, x1, z1) {
    const d = Math.hypot(x1 - x0, z1 - z0);
    const n = Math.ceil(d / (CELL * 0.5));
    for (let k = 0; k <= n; k++) {
      const t = n ? k / n : 0;
      if (this._blocked(this.cell(x0 + (x1 - x0) * t, z0 + (z1 - z0) * t))) return false;
    }
    return true;
  }

  // Path from (x0,z0) to (x1,z1) as a list of waypoints (excluding the start). If the goal is
  // inside an obstacle the path ends at the nearest reachable point. `avoid` lists moving
  // robots to route around ({ x, z, r }).
  plan(x0, z0, x1, z1, avoid = null) {
    if (avoid && avoid.length) this._markDyn(avoid);
    try {
      return this._plan(x0, z0, x1, z1);
    } finally {
      this.dynRun = -1;
    }
  }

  _plan(x0, z0, x1, z1) {
    if (this.los(x0, z0, x1, z1)) return [{ x: x1, z: z1 }];
    const s = this.nearestFree(this.cell(x0, z0));
    const gCell = this.cell(x1, z1);
    const goalBlocked = this._blocked(gCell);
    const t = this.nearestFree(gCell);
    const run = ++this.run;
    const { g, from, stamp, closed } = this;
    const tc = this.center(t);
    const h = (c) => {
      const p = this.center(c);
      const dx = Math.abs(p.x - tc.x) / CELL, dz = Math.abs(p.z - tc.z) / CELL;
      return Math.max(dx, dz) + (SQ2 - 1) * Math.min(dx, dz);
    };
    const open = new Heap();
    stamp[s] = run; g[s] = 0; from[s] = -1;
    open.push(s, h(s));
    let found = false;
    let iter = 0;
    while (open.size && iter++ < 20000) {
      const c = open.pop();
      if (closed[c] === run) continue;
      closed[c] = run;
      if (c === t) { found = true; break; }
      const ci = Math.floor(c / NZ), cj = c % NZ;
      for (let di = -1; di <= 1; di++) {
        for (let dj = -1; dj <= 1; dj++) {
          if (!di && !dj) continue;
          const i = ci + di, j = cj + dj;
          if (i < 0 || j < 0 || i >= NX || j >= NZ) continue;
          const n = i * NZ + j;
          if (this._blocked(n) || closed[n] === run) continue;
          // no corner cutting
          if (di && dj && (this._blocked(ci * NZ + j) || this._blocked(i * NZ + cj))) continue;
          const ng = g[c] + (di && dj ? SQ2 : 1);
          if (stamp[n] !== run || ng < g[n]) {
            stamp[n] = run; g[n] = ng; from[n] = c;
            open.push(n, ng + h(n));
          }
        }
      }
    }
    if (!found) return [{ x: x1, z: z1 }];
    const cells = [];
    for (let c = t; c !== -1; c = from[c]) cells.push(c);
    cells.reverse();
    const pts = cells.map((c) => this.center(c));
    if (!goalBlocked) pts[pts.length - 1] = { x: x1, z: z1 };
    // string pulling
    const out = [];
    let ax = x0, az = z0, k = 0;
    while (k < pts.length) {
      let far = k;
      for (let m = pts.length - 1; m > k; m--) {
        if (this.los(ax, az, pts[m].x, pts[m].z)) { far = m; break; }
      }
      out.push(pts[far]);
      ax = pts[far].x; az = pts[far].z;
      k = far + 1;
    }
    return out;
  }
}
