// FUEL inside a robot. Held FUEL is simulated in the robot's own frame with a light
// position-based solver instead of full rigid bodies: gravity, the robot's acceleration and
// rotation (so the load sloshes when it brakes or spins), soft ball-to-ball contact (FUEL is
// foam), the hopper's walls, floor and internal parts, and what the mechanisms do to it (a
// powered floor, a rotor, a conveyor). Every ball moves on its own; nothing snaps to a slot.
// FUEL on its way to the shooter (or back out through the intake) follows that path.
//
// Frame: x forward, y up, z to the side, origin at the robot center on the carpet (the same
// frame as Robot.localToWorld and the robot model).
import * as THREE from 'three';
import { FUEL } from './constants.js';
import { clamp } from './util.js';

const R = FUEL.radius;
const D_BALL = 2 * R * 0.9; // foam squashes, so centers can get a little closer than a diameter
const R_WALL = R * 0.96;
const G = 9.81;
const ITER = 3;
const MAX_SPEED = 5;

// How much FUEL a hopper holds: pour n in, let it settle, and see whether the load fits without
// squashing past what foam gives (SQUEEZE_MAX). Largest n that fits, by bisection. Seeded, and
// cached per hopper, so it's the same every time.
const SQUEEZE_MAX = 0.01;
const capCache = new Map();
export function measureCapacity(spec, front) {
  const key = spec;
  let per = capCache.get(key);
  if (!per) { per = new Map(); capCache.set(key, per); }
  const k = front.toFixed(3);
  if (per.has(k)) return per.get(k);
  const env = { acc: { x: 0, z: 0 }, w: 0, alpha: 0 };
  const fits = (n) => {
    let s = 12345;
    const rng = () => ((s = (s * 16807) % 2147483647) / 2147483647);
    const h = new Hopper(spec);
    h.front = front;
    h.fill(Array.from({ length: n }, () => ({})), rng);
    let p = 0;
    for (let i = 0; i < 240; i++) { h.step(1 / 120, env); if (i >= 210) p = Math.max(p, h.pressure); }
    return p < SQUEEZE_MAX;
  };
  let lo = 1, hi = 8;
  while (fits(hi) && hi < 400) { lo = hi; hi *= 2; }
  while (hi - lo > 1) { const m = (lo + hi) >> 1; if (fits(m)) lo = m; else hi = m; }
  per.set(k, lo);
  return lo;
}

// the most a robot holds (hopper out)
export const modelCapacity = (cfg) => measureCapacity(cfg.bay, cfg.bay.x1 + (cfg.storage.extLen || 0));

// A Dye Rotor's hook: a fixed curved guide over the rotor, from its rim in to the feeder at the
// center column (FUEL carried round runs into it and slides in along it). Points [x, z] in the
// robot frame; angles about the rotor center in the fin's sense (direction (cos, 0, -sin)).
export function hookPath(bay, n = 20) {
  const rs = bay.rotor, h = bay.hook, f = bay.feed;
  const thF = Math.atan2(-(f.z - rs.z), f.x - rs.x);
  const pts = [];
  for (let i = 0; i <= n; i++) {
    const t = i / n, r = h.r0 + (h.r1 - h.r0) * t, th = thF + h.th0 + (h.th1 - h.th0) * t;
    pts.push([rs.x + r * Math.cos(th), rs.z - r * Math.sin(th)]);
  }
  return pts;
}

export class Hopper {
  constructor(spec) {
    this.spec = spec;
    this.list = [];
    this.front = spec.x1;
    this.wall = Infinity; // a mechanism sweeping in from the front (2910's intake compacting)
    this.pressure = 0;    // how hard the load is squeezed (deepest ball overlap, m)
    this.quiet = 0;
    this.finAngle = 0; // Dye Rotor: how far it has turned (where the Dolphin Fin is, robot frame, about +y)
    this.domeScale = 1;
    this.obstacles = [...(spec.obstacles || [])];
    if (spec.hook) {
      // the hook as a row of thin posts at its height over the rotor
      const h = spec.hook, y = spec.rotor.y;
      for (const [x, z] of hookPath(spec, 12)) this.obstacles.push({ x, z, r: h.r, y0: y + h.y0, y1: y + h.y1 });
    }
  }

  clear() {
    for (const e of this.list) e.b.hop = null;
    this.list = [];
  }

  get count() { return this.list.length; }

  floorAt(x, z = 0) {
    const s = this.spec, f = s.floor;
    if (s.ramp && x > s.x1) {
      // an intake box's ramp: up to a ridge over the bumper, then down to the front roller
      const r = s.ramp, base = this.floorAt(s.x1, z);
      if (x <= r.x) return base + ((r.y - base) * (x - s.x1)) / (r.x - s.x1);
      return Math.max(r.lo, r.y - r.fwd * (x - r.x)) + r.lift;
    }
    let y = clamp(f.a + f.b * x, f.lo, f.hi);
    if (s.funnel) {
      // terraces around the rotor slope down into it
      const r = s.rotor, d = Math.hypot(x - r.x, z - r.z) - r.r;
      if (d > 0) y += Math.min(s.funnel.cap, d * s.funnel.slope);
    }
    return y;
  }

  // Where a FUEL let in at (x, z) comes to rest: on the hopper floor, or on the FUEL already there
  dropHeight(x, z) {
    let y = this.floorAt(x, z) + R_WALL;
    for (const e of this.list) {
      if (e.tr) continue;
      const dx = e.p.x - x, dz = e.p.z - z, h2 = dx * dx + dz * dz;
      if (h2 < D_BALL * D_BALL) y = Math.max(y, e.p.y + Math.sqrt(D_BALL * D_BALL - h2));
    }
    return y;
  }

  // the ceiling over the load; a stretchy net (spec.dome) bulges up over its open part, as far
  // as there's room over the robot (domeScale, 0..1)
  topAt(x, z = 0) {
    const s = this.spec;
    if (s.above) return this.floorAt(x) + s.above;
    if (s.dome) return s.top + s.dome.h * this.domeScale * this.domeShape(x, z);
    if (s.extTop !== undefined && x > s.x1) return s.extTop;
    return s.top;
  }

  // 0..1: how far the net over the top can bulge here (pinned at its edges, the top plate and
  // round the turret)
  domeShape(x, z) {
    const d = this.spec.dome, hw = this.spec.hw;
    const u = (x - d.x0) / Math.max(0.05, this.front - d.x0), v = (z + hw) / (2 * hw);
    if (u <= 0 || u >= 1 || v <= 0 || v >= 1) return 0;
    const r = Math.hypot(x - d.cx, z - d.cz);
    const hole = clamp((r - d.rHole) / 0.14, 0, 1);
    return Math.sin(Math.PI * u) * Math.sin(Math.PI * v) * hole * hole * (3 - 2 * hole);
  }

  add(b, p, v) {
    const e = { b, p: p.clone(), v: v.clone(), prev: new THREE.Vector3(), tr: null };
    b.hop = e;
    this.list.push(e);
    this.quiet = 0;
    return e;
  }

  remove(e) {
    const i = this.list.indexOf(e);
    if (i >= 0) this.list.splice(i, 1);
    e.b.hop = null;
    this.quiet = 0;
  }

  // Drop FUEL in loosely (preload), back to front and bottom up, and let it settle
  fill(balls, rng = Math.random) {
    const s = this.spec;
    const step = 2 * R * 0.97;
    const slots = [];
    for (let layer = 0; layer < 8; layer++) {
      for (let x = s.x0 + R; x <= this.front - R + 1e-6; x += step) {
        for (let z = -s.hw + R; z <= s.hw - R + 1e-6; z += step) {
          const y = this.floorAt(x, z) + R + layer * step;
          if (y <= this.topAt(x, z) - R + 0.02) slots.push(new THREE.Vector3(x, y, z));
        }
      }
    }
    balls.forEach((b, i) => {
      const p = (slots[i] || slots[slots.length - 1] || new THREE.Vector3(0, 0.3, 0)).clone();
      p.x += (rng() - 0.5) * 0.02;
      p.z += (rng() - 0.5) * 0.02;
      this.add(b, p, new THREE.Vector3());
    });
    for (let i = 0; i < 30; i++) this.step(1 / 120, { acc: { x: 0, z: 0 }, w: 0, alpha: 0 });
  }

  // Send a ball along a path (robot-frame points; end() gives the moving last point, e.g. a
  // turret exit). onDone(e) fires when it gets there.
  startTransit(e, via, end, speed, onDone) {
    const pts = [e.p.clone(), ...via];
    let len = 0;
    let prev = pts[0];
    for (const q of [...pts.slice(1), end()]) { len += prev.distanceTo(q); prev = q; }
    e.tr = { pts, end, t: 0, T: clamp(len / speed, 0.04, 0.2), onDone };
    this.quiet = 0;
  }

  _transitPos(tr, out) {
    const pts = [...tr.pts, tr.end()];
    const segs = [];
    let len = 0;
    for (let i = 1; i < pts.length; i++) { const l = pts[i - 1].distanceTo(pts[i]); segs.push(l); len += l; }
    // ease in, then full speed into the wheels
    const u = Math.min(1, tr.t / tr.T);
    let s = len * (u < 0.3 ? (u * u) / 0.6 : u - 0.15) / 0.85;
    for (let i = 0; i < segs.length; i++) {
      if (s <= segs[i] || i === segs.length - 1) return out.lerpVectors(pts[i], pts[i + 1], segs[i] > 0 ? Math.min(1, s / segs[i]) : 1);
      s -= segs[i];
    }
    return out.copy(pts[pts.length - 1]);
  }

  // env: acc {x, y, z} robot acceleration and g gravity, both in the (tilted) robot frame, w yaw
  // rate, alpha yaw acceleration,
  // feeding / intaking (mechanisms running), feedPoint (Vector3) where the shooter takes FUEL
  step(dt, env) {
    const s = this.spec;
    const list = this.list;
    // transits first (they push the others out of the way)
    for (let i = list.length - 1; i >= 0; i--) {
      const e = list[i];
      if (!e.tr) continue;
      e.tr.t = Math.min(e.tr.T, e.tr.t + dt);
      e.prev.copy(e.p);
      this._transitPos(e.tr, e.p);
      e.v.copy(e.p).sub(e.prev).divideScalar(dt);
      if (e.tr.t >= e.tr.T) e.tr.onDone(e);
    }
    let spin = 0;
    if (s.drive === 'rotor') {
      spin = env.feeding ? s.rotor.spin : s.rotor.idle;
      this.finAngle = (this.finAngle + spin * dt) % (2 * Math.PI);
    }
    const w = env.w || 0, al = env.alpha || 0;
    const ax = env.acc.x, az = env.acc.z, ay = env.acc.y || 0;
    const gx = env.g ? env.g.x : 0, gy = env.g ? env.g.y : -G, gz = env.g ? env.g.z : 0;
    const running = env.feeding || env.intaking;
    const still = Math.abs(w) < 0.05 && Math.hypot(ax, az) < 0.3 && !running && !spin && !list.some((e) => e.tr);
    if (still && this.quiet > 0.4) return;

    let maxV = 0;
    for (const e of list) {
      if (e.tr) continue;
      const p = e.p, v = e.v;
      // gravity and the robot frame's pseudo-forces (it accelerates and turns under the FUEL)
      let fx = gx - ax + w * w * p.x - 2 * w * v.z - al * p.z;
      let fy = gy - ay;
      let fz = gz - az + w * w * p.z + 2 * w * v.x + al * p.x;
      const onFloor = p.y - R_WALL - this.floorAt(p.x, p.z) < 0.02;
      if (s.drive === 'floor' && onFloor) {
        // powered floor rollers carry FUEL back to the indexer
        const target = env.feeding ? -s.driveSpeed : env.intaking ? -0.5 : null;
        if (target !== null) fx += 12 * (target - v.x);
      } else if (s.drive === 'rotor' && onFloor) {
        // the rotor spins under the FUEL on it and carries it round (friction), and the Dolphin
        // Fin on its rim pushes the FUEL touching its leading face, which pushes the rest along
        const r = s.rotor, dx = p.x - r.x, dz = p.z - r.z, d = Math.hypot(dx, dz);
        if (d < r.r && d > 0.05 && spin) {
          const th = Math.atan2(-dz, dx); // same sense as the fin: direction (cos, 0, -sin)
          let gap = (spin > 0 ? th - this.finAngle : this.finAngle - th) % (2 * Math.PI);
          if (gap < 0) gap += 2 * Math.PI;
          const fin = d > (r.finR0 ?? 0) - R * 0.5 && gap < R / d + 0.08;
          const k = fin ? r.grip ?? 40 : r.drag ?? 0;
          const ux = -Math.sin(th) * spin * d, uz = -Math.cos(th) * spin * d;
          fx += k * (ux - v.x);
          fz += k * (uz - v.z);
        }
      } else if (s.drive === 'belt') {
        // compliant conveyor wheels grip the FUEL: carry it up to the turret, or hold it
        const b = s.floor.b, n = Math.hypot(1, b);
        const tx = -1 / n, ty = -b / n; // up the slope, toward the turret
        const gAlong = fy * ty;
        fx -= gAlong * tx; fy -= gAlong * ty;
        const sp = running ? s.driveSpeed : 0;
        fx += 15 * (sp * tx - v.x);
        fy += 15 * (sp * ty - v.y);
        fz += -4 * v.z;
      }
      if (env.feeding && env.feedPoint) {
        const dx = env.feedPoint.x - p.x, dz = env.feedPoint.z - p.z, d = Math.hypot(dx, dz);
        if (d > 0.05) { fx += (2.5 * dx) / d; fz += (2.5 * dz) / d; }
      }
      v.x += fx * dt; v.y += fy * dt; v.z += fz * dt;
      v.multiplyScalar(1 - 0.6 * dt);
      e.prev.copy(p);
      p.addScaledVector(v, dt);
    }

    // constraints: ball-ball contact, then the hopper around them
    list.sort((a, b) => a.p.x - b.p.x);
    const n = list.length;
    let squeeze = 0;
    for (let it = 0; it < ITER; it++) {
      for (let i = 0; i < n; i++) {
        const a = list[i];
        for (let j = i + 1; j < n; j++) {
          const b = list[j];
          const dx = b.p.x - a.p.x;
          if (dx > D_BALL) break;
          const dy = b.p.y - a.p.y, dz = b.p.z - a.p.z;
          const d2 = dx * dx + dy * dy + dz * dz;
          if (d2 >= D_BALL * D_BALL) continue;
          if (a.tr && b.tr) continue;
          const d = Math.sqrt(d2) || 1e-4;
          if (it === ITER - 1) squeeze = Math.max(squeeze, D_BALL - d);
          const corr = (D_BALL - d) / d;
          const ka = a.tr ? 0 : b.tr ? 1 : 0.5, kb = b.tr ? 0 : a.tr ? 1 : 0.5;
          const cx = dx * corr, cy = (d2 > 1e-8 ? dy : 1e-4) * corr, cz = dz * corr;
          a.p.x -= cx * ka; a.p.y -= cy * ka; a.p.z -= cz * ka;
          b.p.x += cx * kb; b.p.y += cy * kb; b.p.z += cz * kb;
        }
      }
      for (const e of list) if (!e.tr) this._bounds(e.p);
    }
    this.pressure = squeeze;

    for (const e of list) {
      if (e.tr) continue;
      const v = e.v.copy(e.p).sub(e.prev).divideScalar(dt);
      if (e.p.y - R_WALL - this.floorAt(e.p.x, e.p.z) < 0.004) { v.x *= 1 - Math.min(1, 2 * dt); v.z *= 1 - Math.min(1, 2 * dt); }
      const sp = v.length();
      if (sp > MAX_SPEED) v.multiplyScalar(MAX_SPEED / sp);
      maxV = Math.max(maxV, sp);
    }
    this.quiet = still && maxV < 0.03 ? this.quiet + dt : 0;
  }

  _bounds(p) {
    const s = this.spec;
    p.x = clamp(p.x, s.x0 + R_WALL, Math.min(this.front, this.wall) - R_WALL);
    p.z = clamp(p.z, -s.hw + R_WALL, s.hw - R_WALL);
    const fl = this.floorAt(p.x, p.z) + R_WALL;
    const top = this.topAt(p.x, p.z) - R_WALL;
    if (p.y < fl) {
      // push out along the floor's normal, so FUEL rolls down slopes and funnels
      const e = 0.01;
      const gx = (this.floorAt(p.x + e, p.z) - this.floorAt(p.x - e, p.z)) / (2 * e);
      const gz = (this.floorAt(p.x, p.z + e) - this.floorAt(p.x, p.z - e)) / (2 * e);
      const k = (fl - p.y) / (1 + gx * gx + gz * gz);
      p.x -= gx * k; p.y += k; p.z -= gz * k;
    } else if (p.y > top && top > fl) {
      // under a sloped ceiling (the net's bulge) push out along its normal, so FUEL slides out
      // under it instead of being pressed into the load
      const e = 0.01;
      const gx = (this.topAt(p.x + e, p.z) - this.topAt(p.x - e, p.z)) / (2 * e);
      const gz = (this.topAt(p.x, p.z + e) - this.topAt(p.x, p.z - e)) / (2 * e);
      const k = (p.y - top) / (1 + gx * gx + gz * gz);
      p.x += gx * k; p.y -= k; p.z += gz * k;
    }
    if (s.chamfer) {
      // cut back corners: (x - x0) - |z| + hw - chamfer >= R * sqrt2
      const sz = Math.sign(p.z) || 1;
      const g = (p.x - s.x0) - Math.abs(p.z) + s.hw - s.chamfer - R_WALL * Math.SQRT2;
      if (g < 0) { p.x -= g / 2; p.z += (sz * g) / 2; }
    }
    if (this.obstacles.length) {
      for (const o of this.obstacles) {
        if (o.box) {
          const [x0, x1, y0, y1, z0, z1] = o.box;
          const qx = clamp(p.x, x0, x1), qy = clamp(p.y, y0, y1), qz = clamp(p.z, z0, z1);
          const dx = p.x - qx, dy = p.y - qy, dz = p.z - qz;
          const d = Math.hypot(dx, dy, dz);
          if (d >= R_WALL) continue;
          if (d > 1e-6) { const k = (R_WALL - d) / d; p.x += dx * k; p.y += dy * k; p.z += dz * k; }
          else p.x = x1 + R_WALL; // center inside: out the front face
        } else {
          if (p.y + R_WALL < o.y0 || p.y - R_WALL > o.y1) continue;
          const dx = p.x - o.x, dz = p.z - o.z;
          const d = Math.hypot(dx, dz), m = o.r + R_WALL;
          if (d >= m) continue;
          if (d > 1e-6) { p.x = o.x + (dx / d) * m; p.z = o.z + (dz / d) * m; }
          else p.x = o.x + m;
        }
      }
    }
  }
}
