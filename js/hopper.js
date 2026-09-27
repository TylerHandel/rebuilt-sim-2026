// FUEL inside a robot. Held FUEL is simulated in the robot's own frame with a light
// position-based solver instead of full rigid bodies: gravity, the robot's acceleration and
// rotation (so the load sloshes when it brakes or spins), soft ball-to-ball contact (FUEL is
// foam: a spring at its measured rate, FUEL.springRate), the hopper's walls, floor and internal parts, and what the mechanisms do to it (a
// powered floor, a rotor, a conveyor). Every ball moves on its own; nothing snaps to a slot.
// FUEL on its way to the shooter (or back out through the intake) follows that path.
//
// Frame: x forward, y up, z to the side, origin at the robot center on the carpet (the same
// frame as Robot.localToWorld and the robot model).
import * as THREE from 'three';
import { FUEL } from './constants.js';
import { clamp } from './util.js';
import { CAPACITY_TABLE } from './capacities.js';

const R = FUEL.radius;
const D_BALL = 2 * R; // FUEL touch at a full diameter and squash (softly) past that
const K_BALL = FUEL.springRate;
const R_WALL = R * 0.96;
const G = 9.81;
const ITER = 3;
const MAX_SPEED = 5;

// How much FUEL a hopper holds, packed the way its intake packs it (push: how hard the intake
// shoves FUEL in, N; see intakePush). Seeded, and cached per hopper, so it's the same every time.
// lift: how far a lid that rises with the hopper (spec.lift) is up, 0..1.
const capCache = new Map();

// How hard a robot's intake shoves FUEL into its hopper: the roller grips a FUEL it squeezes
// (intake.squeeze, in: how much smaller the gap is than a FUEL) with the foam's spring force,
// and rubber on foam grips about as hard as it's pressed (INTAKE_GRIP). Teams squeeze FUEL
// 3/4-1in with compliant wheels, 1/2-5/8in with rigid rollers (Chief Delphi, 2026).
const INTAKE_GRIP = 1.0;
export const intakePush = (cfg) => INTAKE_GRIP * FUEL.springRate * (cfg.intake.squeeze ?? 0.75) * 0.0254;

// a held FUEL's contact patches, deepest kept: e.touch holds [nx, ny, nz, w] per contact (the
// patch is the plane w from its center, facing n)
const MAX_TOUCH = 4;
function touch(e, nx, ny, nz, w) {
  const t = e.touch;
  let i = e.nTouch;
  if (i >= MAX_TOUCH) {
    // full: replace the shallowest (largest w) if this one is deeper
    let k = 0;
    for (let j = 1; j < MAX_TOUCH; j++) if (t[4 * j + 3] > t[4 * k + 3]) k = j;
    if (t[4 * k + 3] <= w) return;
    i = k;
  } else e.nTouch++;
  t[4 * i] = nx; t[4 * i + 1] = ny; t[4 * i + 2] = nz; t[4 * i + 3] = w;
}
// a short fingerprint of a hopper and how it's filled, for the precomputed table
export function capacityKey(spec, front, lift, push) {
  const str = JSON.stringify(spec) + '|' + front.toFixed(3) + '|' + lift + '|' + push.toFixed(1);
  let h = 5381;
  for (let i = 0; i < str.length; i++) h = ((h * 33) ^ str.charCodeAt(i)) >>> 0;
  return h.toString(16);
}

export function measureCapacity(spec, front, lift = 1, push = FUEL.intakePush) {
  const key = spec;
  let per = capCache.get(key);
  if (!per) { per = new Map(); capCache.set(key, per); }
  const k = front.toFixed(3) + '|' + lift + '|' + push.toFixed(1);
  if (per.has(k)) return per.get(k);
  // measured ahead of time (tools/capacity.mjs) unless the hopper changed since
  const pre = CAPACITY_TABLE[capacityKey(spec, front, lift, push)];
  if (pre !== undefined) { per.set(k, pre); return pre; }
  let s = 12345;
  const rng = () => ((s = (s * 16807) % 2147483647) / 2147483647);
  const env = { acc: { x: 0, z: 0 }, w: 0, alpha: 0 };
  let n;
  if (spec.pack === false) {
    // no hopper, just a ball path (8793): pour n in, and it holds n if no two are pressed together
    // harder than the intake pushes. Largest n that fits, by bisection.
    const fits = (n) => {
      const h = new Hopper(spec);
      h.front = front;
      h.liftScale = lift;
      h.fill(Array.from({ length: n }, () => ({})), rng);
      let p = 0;
      for (let i = 0; i < 240; i++) { h.step(1 / 120, env); if (i >= 210) p = Math.max(p, h.pressure); }
      return p < push;
    };
    let lo = 1, hi = 8;
    while (fits(hi) && hi < 400) { lo = hi; hi *= 2; }
    while (hi - lo > 1) { const m = (lo + hi) >> 1; if (fits(m)) lo = m; else hi = m; }
    n = lo;
  } else {
    const h = new Hopper(spec);
    h.front = front;
    h.liftScale = lift;
    n = h.pack(Array.from({ length: 400 }, () => ({})), push, rng);
  }
  per.set(k, n);
  return n;
}

// the most a robot holds (hopper out)
export const modelCapacity = (cfg) => measureCapacity(cfg.bay, cfg.bay.x1 + (cfg.storage.extLen || 0), 1, intakePush(cfg));

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

// piecewise linear through [x, y] points (sorted by x), flat past the ends
function pwl(pts, x) {
  if (x <= pts[0][0]) return pts[0][1];
  for (let i = 1; i < pts.length; i++) {
    const [x1, y1] = pts[i];
    if (x <= x1) { const [x0, y0] = pts[i - 1]; return y0 + ((y1 - y0) * (x - x0)) / (x1 - x0); }
  }
  return pts[pts.length - 1][1];
}

export class Hopper {
  constructor(spec) {
    this.spec = spec;
    this.list = [];
    this.front = spec.x1;
    this.wall = Infinity; // a mechanism sweeping in from the front (2910's intake compacting)
    this.pressure = 0;    // how hard the load is squeezed (hardest ball-to-ball contact, N)
    this.lam = new Map(); // contact impulses, per step (soft contacts)
    this.quiet = 0;
    this.finAngle = 0; // Dye Rotor: how far it has turned (where the Dolphin Fin is, robot frame, about +y)
    this.domeScale = 1;
    this.liftScale = 1; // a lid on the climber (spec.lift): how far it's up, 0..1
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
    // a straight slope, or (floor.pts) a path's profile through points
    let y = f.pts ? pwl(f.pts, x) : clamp(f.a + f.b * x, f.lo, f.hi);
    if (s.funnel) {
      // terraces around the rotor slope down into it
      const r = s.rotor, d = Math.hypot(x - r.x, z - r.z) - r.r;
      if (d > 0) y += Math.min(s.funnel.cap, d * s.funnel.slope);
    }
    return y;
  }

  // half-width at x: the side walls, or a funnel narrowing toward the back (spec.taper.pts:
  // [x, half-width] points, as on 8793's conveyor that takes 4-wide FUEL down to 1-wide)
  hwAt(x) {
    const s = this.spec, t = s.taper;
    return t ? pwl(t.pts, x) : s.hw;
  }

  // a FUEL in a funnel's throat (too narrow for two) with another beside it
  _abreast(e) {
    const s = this.spec, p = e.p;
    if (this.hwAt(p.x) >= 2 * R + 0.01) return false;
    return this.list.some((o) => o !== e && !o.tr && Math.abs(o.p.x - p.x) < R && o.p.z * p.z < 0 && Math.abs(o.p.z - p.z) < 2.2 * R);
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
    // a lid that rises (1678's, on the climber) lifts the whole ceiling
    let top = s.lift ? s.top + s.lift.h * this.liftScale : s.top;
    // a net stretched diagonally from the hopper's top edge (slope.x) down to the front of the
    // intake or extension (slope.y at the front), as on 1678 and 4946
    if (s.slope && x > s.slope.x) top += (s.slope.y - top) * clamp((x - s.slope.x) / Math.max(0.05, this.front - s.slope.x), 0, 1);
    if (s.dome) return top + s.dome.h * this.domeScale * this.domeShape(x, z);
    if (s.extTop !== undefined && x > s.x1) return s.extTop;
    return top;
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
    // touch: where other FUEL presses on it (up to MAX_TOUCH: normal xyz, distance from its center)
    const e = { b, p: p.clone(), v: v.clone(), prev: new THREE.Vector3(), tr: null, touch: new Float32Array(4 * MAX_TOUCH), nTouch: 0 };
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
        const hw = this.hwAt(x);
        for (let z = -hw + R; z <= hw - R + 1e-6; z += step) {
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

  // Fill it the way the intake does: FUEL comes in one at a time at the front, on top of what's
  // there, and the roller shoves each one in (push, N). One that stays pressed into the load
  // harder than the roller pushes didn't fit; after a few of those across the intake, it's full.
  // Returns how many of balls went in (the rest are left out).
  pack(balls, push, rng = Math.random) {
    const s = this.spec, env = { acc: { x: 0, z: 0 }, w: 0, alpha: 0, intaking: true };
    let n = 0;
    for (let fails = 0; fails < 3 && n < balls.length;) {
      const z = (rng() - 0.5) * 2 * Math.max(0, s.hw - R - 0.02);
      const x = this.front - R - 0.01;
      const y = Math.min(this.dropHeight(x, z), this.topAt(x, z) - R);
      const e = this.add(balls[n], new THREE.Vector3(x, y, z), new THREE.Vector3(-1.5, 0, 0));
      e.push = push; e.pushT = 0.4;
      for (let i = 0; i < 60; i++) this.step(1 / 120, env);
      let w = R;
      for (let j = 0; j < e.nTouch; j++) w = Math.min(w, e.touch[4 * j + 3]);
      if ((D_BALL - 2 * w) * K_BALL > push * 1.25) { this.remove(e); fails++; } else { n++; fails = 0; }
    }
    return n;
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
    if (env.intaking && list.some((e) => e.push)) this.quiet = 0;
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
        const b = (this.floorAt(p.x + 0.01, p.z) - this.floorAt(p.x - 0.01, p.z)) / 0.02, n = Math.hypot(1, b);
        const tx = -1 / n, ty = -b / n; // up the slope, toward the turret
        const gAlong = fy * ty;
        fx -= gAlong * tx; fy -= gAlong * ty;
        let sp = running ? s.driveSpeed : 0;
        // at a funnel's throat, two wheels on opposite sides spin FUEL against each other
        // (taper.spin: the side whose wheel drives FUEL in; the other's pushes it back out), so
        // two arriving abreast roll round each other and go in single file instead of wedging.
        // (Held FUEL here has no ball-on-ball friction, so a pair can't lock up the way real
        // foam does; the wheels only act on a pair that's actually abreast in the throat.)
        const t = s.taper;
        let k = s.grip ?? 15; // how hard the wheels grab FUEL (1/s)
        if (t && t.spin && sp && p.z * t.spin < -0.02 && this._abreast(e)) { sp = -0.5 * sp; k *= 0.5; }
        fx += k * (sp * tx - v.x);
        fy += 15 * (sp * ty - v.y);
        fz += -4 * v.z;
        // in a funnel, omni wheels on the sides push the FUEL in toward the middle
        if (s.taper && sp && this.hwAt(p.x) < s.hw) fz -= s.taper.center * p.z;
      }
      // the intake roller shoves the FUEL it just brought in back into the load (e.push, N) until
      // it's a ball's width in or the push runs out; that packs the load against the walls, the
      // floor and the nets as hard as the roller can squeeze
      if (e.push) {
        if (env.intaking && e.pushT > 0 && this.front - p.x < D_BALL * 1.2) fx -= e.push / FUEL.mass;
        e.pushT -= dt;
        if (e.pushT <= 0) e.push = 0;
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
    // soft contacts (XPBD): each pair pushes apart with the foam's spring rate, so a load only
    // squashes as hard as something presses it (gravity, braking, an intake cramming FUEL in)
    let squeeze = 0;
    const lam = this.lam;
    lam.clear();
    const soft = 1 / (K_BALL * dt * dt), W = 2 / FUEL.mass;
    for (const e of list) e.nTouch = 0;
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
          if (it === ITER - 1) {
            squeeze = Math.max(squeeze, (D_BALL - d) * K_BALL);
            // each is flattened halfway between the centers, for drawing it
            if (!a.tr && !b.tr) {
              touch(a, dx / d, dy / d, dz / d, d / 2);
              touch(b, -dx / d, -dy / d, -dz / d, d / 2);
            }
          }
          let corr = (D_BALL - d) / d; // FUEL on its way to the shooter shoves the rest aside
          if (!a.tr && !b.tr) {
            const key = i * 4096 + j, l0 = lam.get(key) || 0;
            const dl = Math.max(-l0, (D_BALL - d - soft * l0) / (W + soft)); // pushes, never pulls
            lam.set(key, l0 + dl);
            corr = (dl * W) / d;
          }
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

  // Where a held FUEL is pressed flat, for drawing it: the other FUEL (from the last step) and
  // the walls, floor and ceiling it's up against. Fills out (4 floats per patch: normal, distance
  // from its center) with up to MAX_TOUCH patches and returns how many.
  touchPatches(e, out) {
    const s = this.spec, p = e.p;
    const tmp = { touch: out, nTouch: 0 };
    for (let i = 0; i < e.nTouch; i++) touch(tmp, e.touch[4 * i], e.touch[4 * i + 1], e.touch[4 * i + 2], e.touch[4 * i + 3]);
    if (e.tr) return tmp.nTouch;
    const wall = (nx, ny, nz, w) => { if (w < R) touch(tmp, nx, ny, nz, Math.max(w, R * 0.7)); };
    const hw = this.hwAt(p.x);
    wall(0, 0, 1, hw - p.z);
    wall(0, 0, -1, hw + p.z);
    wall(-1, 0, 0, p.x - s.x0);
    wall(1, 0, 0, Math.min(this.front, this.wall) - p.x);
    wall(0, -1, 0, p.y - this.floorAt(p.x, p.z));
    wall(0, 1, 0, this.topAt(p.x, p.z) - p.y);
    return tmp.nTouch;
  }

  _bounds(p) {
    const s = this.spec;
    p.x = clamp(p.x, s.x0 + R_WALL, Math.min(this.front, this.wall) - R_WALL);
    if (s.taper) {
      // funnel walls: push out along their normal (in and forward), so FUEL slides along them
      const lim = this.hwAt(p.x) - R_WALL;
      if (Math.abs(p.z) > lim) {
        const sz = Math.sign(p.z), k = (this.hwAt(p.x + 0.005) - this.hwAt(p.x - 0.005)) / 0.01;
        const n = Math.hypot(1, k), d = (Math.abs(p.z) - lim) / n;
        p.z -= (sz * d) / n; p.x += (k * d) / n;
      }
    }
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
