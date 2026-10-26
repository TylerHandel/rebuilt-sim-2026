// Physical mechanisms: wheels and rollers that move FUEL only by touching it.
//
// A mechanism is shafts (each turned by a motor through a gear ratio, or an idler) carrying
// wheels (plain rollers and flex wheels, omni wheels, mecanum wheels), mounted on a frame (a robot's
// rigid body, or the world). FUEL are ordinary rigid bodies in the physics world. Each step, every
// wheel touching a FUEL pushes on it through the foam's squeeze and through friction where the two
// surfaces slide, and the FUEL pushes back just as hard on the wheel (slowing its shaft and its
// motor) and on the frame. Nothing else moves FUEL: no paths, no target speeds, no forces from
// nowhere. Grip, speed, stalls and jams come out of that.
//
//   - Foam (FUEL): the measured spring curve (AndyMark's 2026 Scoring Element Testing Report,
//     constants.js): F = k0 xc (e^(x/xc) - 1) for a FUEL squeezed x between two plates, so a
//     contact that dents one side of it by d pushes F(2d); in series with the wheel's own tread
//     (a 30A flex wheel gives too), with damping set for FUEL's bounce.
//   - Contact solve: soft constraints (the spring and damper integrated implicitly, so stiff
//     foam stays stable at the game's 120 Hz) with Coulomb friction, iterated over every contact
//     at once (projected Gauss-Seidel), each impulse applied to the FUEL (moving and spinning it),
//     to the shaft (about its axis) and to the frame (equal and opposite).
//   - Motors: torque from voltage, back-EMF and the stator current limit (drivetrain.js MOTORS),
//     through the gear ratio, against the shaft's inertia (its wheels plus the motor's rotor seen
//     through the ratio), back-EMF integrated implicitly.
//   - Omni and mecanum wheels (a heuristic for their little rollers): they grip only along one
//     direction in the contact plane (an omni wheel along its rim, a mecanum wheel at its rollers'
//     45 degrees) and roll freely across it (a little rolling friction).
//   - Surfaces: the fixed parts FUEL touch (floors, rails, plates, from the team's CAD as triangle
//     meshes) push through the same foam and friction, so a FUEL squeezes past a tight spot the
//     way a foam ball does instead of jamming like a rigid one.
import { FUEL } from '../constants.js';
import { MOTORS } from '../drivetrain.js';

const R = FUEL.radius, MB = FUEL.mass;
const IB = 0.4 * MB * R * R;         // solid foam sphere
const ZETA = 0.25;                    // foam damping ratio (a free FUEL bounces back ~45%)
const MU_FREE = 0.04;                 // an omni/mecanum wheel's rollers across their free direction

// ------------------------------------------------------------------ foam
// one contact's dent d (m) -> force (N) and its slope (N/m): the plate test's curve at 2d
export function foam(d) {
  const k0 = FUEL.springRate0, xc = FUEL.stiffen;
  const e = Math.exp(Math.min(Math.max(0, 2 * d), 3.5 * xc) / xc);
  return { F: k0 * xc * (e - 1), k: 2 * k0 * e };
}
// the foam in series with a tread of stiffness kt (N/m): the total squeeze d splits between them
function squeeze(d, kt) {
  if (!(kt < Infinity)) return foam(d);
  let xf = d * 0.5;
  for (let i = 0; i < 6; i++) {
    const f = foam(xf), g = f.F - kt * (d - xf), dg = f.k + kt;
    xf = Math.min(d, Math.max(0, xf - g / dg));
  }
  const f = foam(xf);
  return { F: f.F, k: 1 / (1 / f.k + 1 / kt) };
}

// ------------------------------------------------------------------ motors
export class Motor {
  // type: a MOTORS key; count: motors on the shaft; ratio: motor turns per output turn;
  // eff: the drive's efficiency on torque; statorLimit, supplyLimit (A, per motor; the supply
  // current is the stator current times the fraction of the battery's voltage applied, the
  // back-EMF plus the winding's drop: it binds near full speed, not at a stall)
  constructor(type, { count = 1, ratio = 1, eff = 0.9, statorLimit = 60, supplyLimit = Infinity } = {}) {
    const m = MOTORS[type];
    if (!m) throw new Error('no motor ' + type);
    Object.assign(this, { type, count, ratio, eff, statorLimit, supplyLimit, m });
    this.Rm = m.V / m.stallCurrent;
    this.kT = m.stallTorque / m.stallCurrent;
    this.kV = m.free / (m.V - m.freeCurrent * this.Rm);
    this.J = count * (m.rotorInertia || 3e-5) * ratio * ratio; // the rotors, seen at the output
    this.current = 0;
  }
  // output torque at output speed w (rad/s), duty cmd (-1..1), battery V; and whether it's at
  // the current limit (then the torque doesn't fall with speed)
  torque(w, cmd, V = 12) {
    const Eb = (w * this.ratio) / this.kV, I = (cmd * V - Eb) / this.Rm;
    // the supply limit: I (Eb + I Rm) / V <= supplyLimit (in the direction it's driving)
    const e = Math.sign(I) * Eb, Isup = this.supplyLimit * V;
    const cap = Math.min(this.statorLimit, Isup < Infinity ? (-e + Math.sqrt(e * e + 4 * this.Rm * Isup)) / (2 * this.Rm) : Infinity);
    const lim = Math.abs(I) > cap;
    const Ic = lim ? Math.sign(I) * cap : I;
    this.current = Ic * this.count;
    return { tau: this.count * this.kT * Ic * this.ratio * this.eff, limited: lim };
  }
  // how fast the torque falls with output speed when it's not current limited (N·m per rad/s)
  get slope() { return (this.count * this.kT * this.ratio * this.ratio * this.eff) / (this.kV * this.Rm); }
  // free speed at the output (rad/s)
  get freeSpeed() { return this.m.free / this.ratio; }
}

export class Shaft {
  // motor: a Motor, or null for an idler (or a shaft of a Train); inertia: what turns with the
  // shaft (kg·m²: its wheels, pulleys, hex shaft), the motor's rotors are added; drag: bearing
  // and belt friction (N·m·s)
  constructor({ name = '', motor = null, inertia = 5e-5, drag = 2e-4 } = {}) {
    Object.assign(this, { name, motor, inertia, drag });
    this._omega = 0;
    this.cmd = 0;
    this.train = null; // on a Train: it turns at k x the train's speed
    this.k = 1;
  }
  get omega() { return this.train ? this.train.omega * this.k : this._omega; }
  set omega(v) { if (this.train) this.train.omega = v / this.k; else this._omega = v; }
  // the inertia felt turning this shaft (on a train: the whole train's, seen through its ratio)
  get J() { return this.train ? this.train.J / (this.k * this.k) : this.inertia + (this.motor ? this.motor.J : 0); }
  // an angular impulse about its axis (N·m·s)
  impulse(L) {
    if (this.train) this.train.omega += (L * this.k) / this.train.J;
    else this._omega += L / this.J;
  }
  // the motor's step, with back-EMF taken implicitly
  integrate(dt, V = 12) {
    if (this.train) return;
    const J = this.J;
    if (!this.motor) { this._omega /= 1 + (dt * this.drag) / J; return; }
    const { tau, limited } = this.motor.torque(this._omega, this.cmd, V);
    const kd = limited ? 0 : this.motor.slope;
    this._omega = (J * this._omega + dt * (tau + kd * this._omega)) / (J + dt * (kd + this.drag));
  }
}

// A gear train: one motor turning several shafts through belts, chain and gears at fixed ratios
// (8793's conveyor and intake rollers all run off one Kraken X60). It has one speed, the motor's
// output: each shaft turns at its k times that, the rotor's and every shaft's inertia move
// together, and a FUEL dragging on any one roller slows them all.
export class Train {
  // motor: a Motor (its own ratio: a gearbox on it); shafts: [[Shaft, k], ...], k: the shaft's
  // turns per turn of the motor's output (the belt/gear ratios along the way multiplied)
  constructor({ name = '', motor, shafts = [] } = {}) {
    Object.assign(this, { name, motor });
    this.omega = 0;
    this.cmd = 0;
    this.shafts = [];
    for (const [s, k] of shafts) this.add(s, k);
  }
  add(s, k) { s.train = this; s.k = k; this.shafts.push(s); return s; }
  get J() { return this.motor.J + this.shafts.reduce((a, s) => a + s.inertia * s.k * s.k, 0); }
  get drag() { return this.shafts.reduce((a, s) => a + s.drag * s.k * s.k, 0); }
  integrate(dt, V = 12) {
    const J = this.J;
    const { tau, limited } = this.motor.torque(this.omega, this.cmd, V);
    const kd = limited ? 0 : this.motor.slope;
    this.omega = (J * this.omega + dt * (tau + kd * this.omega)) / (J + dt * (kd + this.drag));
  }
}

export class Wheel {
  // shaft: the Shaft it's on; center, axis: in the frame's coordinates (m; axis a unit vector);
  // radius, width (m); kind: 'plain' | 'omni' | 'mecanum'; mecanum: its rollers' angle (deg,
  // the sign is the hand); mu: friction against FUEL; tread: the tread's stiffness (N/m;
  // Infinity for a hard wheel)
  constructor({ name = '', shaft, center, axis, radius, width, kind = 'plain', mecanum = 45, mu = 1.0, tread = Infinity }) {
    const n = Math.hypot(...axis);
    Object.assign(this, { name, shaft, center, axis: axis.map((v) => v / n), radius, width, kind, mecanum, mu, tread });
  }
}

// ------------------------------------------------------------------ small vector helpers
const sub = (a, b) => [a[0] - b[0], a[1] - b[1], a[2] - b[2]];
const add = (a, b) => [a[0] + b[0], a[1] + b[1], a[2] + b[2]];
const mul = (a, s) => [a[0] * s, a[1] * s, a[2] * s];
const dot = (a, b) => a[0] * b[0] + a[1] * b[1] + a[2] * b[2];
const cross = (a, b) => [a[1] * b[2] - a[2] * b[1], a[2] * b[0] - a[0] * b[2], a[0] * b[1] - a[1] * b[0]];
const len = (a) => Math.hypot(a[0], a[1], a[2]);
const norm = (a) => { const l = len(a) || 1; return [a[0] / l, a[1] / l, a[2] / l]; };
// rotate v by the inverse of unit quaternion q
const qrotInv = (q, v) => qrot({ x: -q.x, y: -q.y, z: -q.z, w: q.w }, v);
// rotate v by unit quaternion q
function qrot(q, v) {
  const u = [q.x, q.y, q.z], s = q.w;
  const t = mul(cross(u, v), 2);
  return add(add(v, mul(t, s)), cross(u, t));
}

// a fixed surface's stand-in for a shaft: it doesn't turn and takes any impulse
const FIXED = { omega: 0, J: Infinity, impulse() {} };

// A fixed surface FUEL can touch: a triangle mesh (vertices: flat [x, y, z, ...] in m, indices:
// flat triangles), placed on the frame at pos turned yaw (rad) about the vertical (both can
// change: a turret's), with friction mu against FUEL. Hard: the foam does all the giving.
export class Surface {
  constructor({ name = '', vertices, indices, mu = 0.3, pos = [0, 0, 0], yaw = 0, cell = 0.04 }) {
    Object.assign(this, { name, mu, pos, yaw, cell, kind: 'plain', tread: Infinity, shaft: FIXED });
    const n = indices.length / 3;
    this.tri = new Float64Array(n * 9);
    for (let t = 0; t < n; t++) for (let k = 0; k < 3; k++) for (let j = 0; j < 3; j++) this.tri[t * 9 + k * 3 + j] = vertices[indices[t * 3 + k] * 3 + j];
    // a grid of cells, each listing the triangles whose bounds reach into it
    this.grid = new Map();
    const T = this.tri, key = (i, j, k) => ((i + 512) * 1024 + (j + 512)) * 1024 + (k + 512);
    for (let t = 0; t < n; t++) {
      const lo = [0, 1, 2].map((j) => Math.floor(Math.min(T[t * 9 + j], T[t * 9 + 3 + j], T[t * 9 + 6 + j]) / cell));
      const hi = [0, 1, 2].map((j) => Math.floor(Math.max(T[t * 9 + j], T[t * 9 + 3 + j], T[t * 9 + 6 + j]) / cell));
      for (let i = lo[0]; i <= hi[0]; i++) for (let j = lo[1]; j <= hi[1]; j++) for (let k = lo[2]; k <= hi[2]; k++) {
        const c = key(i, j, k);
        let l = this.grid.get(c);
        if (!l) this.grid.set(c, (l = []));
        l.push(t);
      }
    }
    this.key = key;
    this.stamp = new Uint32Array(n);
    this.mark = 0;
  }
  // a point in the frame -> the surface's own coordinates, and back for a direction
  toLocal(p) {
    const d = sub(p, this.pos), c = Math.cos(this.yaw), s = Math.sin(this.yaw);
    return [d[0] * c - d[2] * s, d[1], d[0] * s + d[2] * c];
  }
  dirToFrame(v) {
    const c = Math.cos(this.yaw), s = Math.sin(this.yaw);
    return [v[0] * c + v[2] * s, v[1], -v[0] * s + v[2] * c];
  }
  // where a FUEL at p (the surface's coordinates) touches it: [{ depth, n (out of the surface,
  // toward the FUEL) }], one per face it's against (neighboring triangles of one face, and a thin
  // plate's far side, merged into the deepest)
  touches(p) {
    const r = R, cell = this.cell, T = this.tri;
    const lo = p.map((v) => Math.floor((v - r) / cell)), hi = p.map((v) => Math.floor((v + r) / cell));
    if (++this.mark === 0xffffffff) { this.stamp.fill(0); this.mark = 1; }
    const found = [];
    for (let i = lo[0]; i <= hi[0]; i++) for (let j = lo[1]; j <= hi[1]; j++) for (let k = lo[2]; k <= hi[2]; k++) {
      const l = this.grid.get(this.key(i, j, k));
      if (!l) continue;
      for (const t of l) {
        if (this.stamp[t] === this.mark) continue;
        this.stamp[t] = this.mark;
        const q = closestOnTri(p, T, t * 9);
        const d = sub(p, q), dl = len(d);
        if (dl >= r || dl < 1e-9) continue;
        found.push({ depth: r - dl, n: mul(d, 1 / dl) });
      }
    }
    found.sort((a, b) => b.depth - a.depth);
    const out = [];
    for (const f of found) if (!out.some((o) => dot(o.n, f.n) > 0.9)) out.push(f);
    return out;
  }
}
// the point of triangle t (at T[o..o+8]) nearest p (Ericson, Real-Time Collision Detection 5.1.5)
function closestOnTri(p, T, o) {
  const a = [T[o], T[o + 1], T[o + 2]], b = [T[o + 3], T[o + 4], T[o + 5]], c = [T[o + 6], T[o + 7], T[o + 8]];
  const ab = sub(b, a), ac = sub(c, a), ap = sub(p, a);
  const d1 = dot(ab, ap), d2 = dot(ac, ap);
  if (d1 <= 0 && d2 <= 0) return a;
  const bp = sub(p, b), d3 = dot(ab, bp), d4 = dot(ac, bp);
  if (d3 >= 0 && d4 <= d3) return b;
  const vc = d1 * d4 - d3 * d2;
  if (vc <= 0 && d1 >= 0 && d3 <= 0) return add(a, mul(ab, d1 / (d1 - d3)));
  const cp = sub(p, c), d5 = dot(ab, cp), d6 = dot(ac, cp);
  if (d6 >= 0 && d5 <= d6) return c;
  const vb = d5 * d2 - d1 * d6;
  if (vb <= 0 && d2 >= 0 && d6 <= 0) return add(a, mul(ac, d2 / (d2 - d6)));
  const va = d3 * d6 - d5 * d4;
  if (va <= 0 && d4 - d3 >= 0 && d5 - d6 >= 0) return add(b, mul(sub(c, b), (d4 - d3) / ((d4 - d3) + (d5 - d6))));
  const den = 1 / (va + vb + vc);
  return add(a, add(mul(ab, vb * den), mul(ac, vc * den)));
}

// ------------------------------------------------------------------ the mechanism
export class Mechanism {
  // body: the Rapier rigid body it's mounted on (null: fixed in the world); iterations: contact
  // solver passes per step; gravity: the world's, and substeps: its solver's substeps per step
  // (world.integrationParameters.numSolverIterations). The step's gravity is counted in the solve
  // as the world will apply it (spread over its substeps a body moves by (n+1)/2n of a full
  // step's worth), so a FUEL resting on a wheel settles where the foam holds its weight. sub:
  // contact steps per step, for contacts over in a step or two at the game's rate (a shooter).
  constructor({ body = null, shafts = [], wheels = [], surfaces = [], iterations = 6, gravity = [0, -9.81, 0], substeps = 4, sub = 1 } = {}) {
    Object.assign(this, { body, shafts, wheels, surfaces, iterations, gravity, sub });
    this.gShare = (substeps + 1) / (2 * substeps);
    this.contacts = [];  // last step's, for drawing and tests
    this.V = 12;
  }

  // the frame's pose and motion in the world
  _frame() {
    const b = this.body;
    if (!b) return { p: [0, 0, 0], q: { x: 0, y: 0, z: 0, w: 1 }, v: [0, 0, 0], w: [0, 0, 0], com: [0, 0, 0] };
    const t = b.translation(), q = b.rotation(), v = b.linvel(), w = b.angvel(), c = b.worldCom ? b.worldCom() : t;
    return { p: [t.x, t.y, t.z], q, v: [v.x, v.y, v.z], w: [w.x, w.y, w.z], com: [c.x, c.y, c.z] };
  }

  // balls: the FUEL rigid bodies that might touch it this step
  step(dt, balls) {
    for (const s of this.shafts) s.integrate(dt, this.V);
    for (const t of new Set(this.shafts.map((s) => s.train).filter(Boolean))) t.integrate(dt, this.V);
    const fr = this._frame();
    const frameVel = (x) => add(fr.v, cross(fr.w, sub(x, fr.com)));
    // wheels in the world
    const W = this.wheels.map((wh) => ({ wh, c: add(fr.p, qrot(fr.q, wh.center)), a: qrot(fr.q, wh.axis) }));
    const g = this.gravity, n = this.sub, h = dt / n;
    // ball states we work on. In one step (n = 1) a FUEL touching a wheel gets its new velocity
    // and the world moves it. Sub-stepped (n > 1, for contacts too quick for one step: a shooter's
    // nip), every FUEL that could reach a wheel this step is carried through it here in n steps,
    // touching the wheels as it goes, then handed back to the world placed so that its step lands
    // it where it got to.
    const S = new Map();
    const state = (b) => {
      let st = S.get(b);
      if (!st) {
        const t = b.translation(), v = b.linvel(), w = b.angvel(), f = n === 1 ? this.gShare * dt : 0;
        st = { b, c: [t.x, t.y, t.z], v: [v.x + g[0] * f, v.y + g[1] * f, v.z + g[2] * f], w: [w.x, w.y, w.z] };
        S.set(b, st);
      }
      return st;
    };
    if (n > 1) {
      for (const b of balls) {
        const t = b.translation(), v = b.linvel(), bc = [t.x, t.y, t.z], reach = Math.hypot(v.x, v.y, v.z) * dt + 0.01;
        if (W.some(({ wh, c, a }) => {
          const d = sub(bc, c), s = dot(d, a);
          return Math.abs(s) < wh.width / 2 + R + reach && len(sub(d, mul(a, s))) < wh.radius + R + reach;
        }) || (this.surfaces.length && this.surfaces.some((sf) => sf.touches(sf.toLocal(qrotInv(fr.q, sub(bc, fr.p)))).length))) state(b);
      }
    }
    const reaction = [];
    const seen = new Map(); // the step's contacts (the firmest of each wheel and FUEL), for drawing and tests
    for (let k = 0; k < n; k++) {
      if (n > 1) for (const st of S.values()) st.v = add(st.v, mul(g, h));
      const C = this._contacts(W, fr, n > 1 ? [...S.values()].map((st) => [st.b, st.c]) : balls.map((b) => { const t = b.translation(); return [b, [t.x, t.y, t.z]]; }), state);
      this._solve(C, h, frameVel, reaction);
      if (n > 1) for (const st of S.values()) st.c = add(st.c, mul(st.v, h));
      for (const ct of C) {
        const key = ct.W1.wh, m = seen.get(key) || new Map();
        seen.set(key, m);
        const was = m.get(ct.st.b);
        if (!was || ct.lamN / h > was.force) m.set(ct.st.b, { wheel: ct.W1.wh, ball: ct.st.b, depth: ct.depth, force: ct.lamN / h, x: ct.x, n: ct.n });
      }
    }
    // results
    if (n === 1) {
      const f = this.gShare * dt;
      for (const st of S.values()) {
        st.b.setLinvel({ x: st.v[0] - g[0] * f, y: st.v[1] - g[1] * f, z: st.v[2] - g[2] * f }, true);
        st.b.setAngvel({ x: st.w[0], y: st.w[1], z: st.w[2] }, true);
      }
    } else {
      // the world's step adds g dt to the velocity and moves it by (v + gShare g dt) dt
      for (const st of S.values()) {
        const v = sub(st.v, mul(g, dt)), p = sub(st.c, mul(add(v, mul(g, this.gShare * dt)), dt));
        st.b.setTranslation({ x: p[0], y: p[1], z: p[2] }, true);
        st.b.setLinvel({ x: v[0], y: v[1], z: v[2] }, true);
        st.b.setAngvel({ x: st.w[0], y: st.w[1], z: st.w[2] }, true);
      }
    }
    if (this.body && this.body.isDynamic()) {
      for (const [x, J] of reaction) this.body.applyImpulseAtPoint({ x: J[0], y: J[1], z: J[2] }, { x: x[0], y: x[1], z: x[2] }, true);
    }
    this.contacts = [...seen.values()].flatMap((m) => [...m.values()]);
  }

  // the wheels' and surfaces' contacts with FUEL at the given centers ([body, center] pairs)
  _contacts(W, fr, at, state) {
    const C = [];
    for (const [b, bc] of at) {
      for (const sf of this.surfaces) {
        const hits = sf.touches(sf.toLocal(qrotInv(fr.q, sub(bc, fr.p))));
        for (const { depth, n: nl } of hits) {
          const st = state(b), n = qrot(fr.q, sf.dirToFrame(nl));
          const x = sub(bc, mul(n, R - depth / 2)), rb = sub(x, bc), rw = [0, 0, 0];
          let t1 = cross(n, Math.abs(n[1]) < 0.9 ? [0, 1, 0] : [1, 0, 0]);
          t1 = norm(t1);
          const t2 = cross(n, t1);
          const { F, k } = foam(depth);
          const fric = [t1, t2].map((u) => { const ru = cross(rb, u); return { u, mu: sf.mu, m: 1 / (1 / MB + dot(ru, ru) / IB), lam: 0, wu: 0 }; });
          C.push({ st, W1: { wh: sf, a: [0, 0, 1] }, n, depth, x, rb, rw, F, k, c: 2 * ZETA * Math.sqrt(k * MB), lamN: 0, fric, plain: true });
        }
      }
      for (const W1 of W) {
        const { wh, c, a } = W1, hw = wh.width / 2;
        const d = sub(bc, c), s = dot(d, a);
        if (Math.abs(s) > hw + R) continue;
        const rad = sub(d, mul(a, s)), rl = len(rad);
        let n, depth, rim;
        if (Math.abs(s) <= hw) {
          // on the tread
          if (rl < 1e-6) continue;
          n = mul(rad, 1 / rl);
          depth = R + wh.radius - rl;
          rim = add(c, add(mul(a, s), mul(n, wh.radius)));
        } else {
          // on the edge of the tread
          const e = add(c, add(mul(a, Math.sign(s) * hw), mul(rl > 1e-6 ? mul(rad, 1 / rl) : [0, 0, 0], wh.radius)));
          const q = sub(bc, e), ql = len(q);
          if (ql < 1e-6) continue;
          n = mul(q, 1 / ql);
          depth = R - ql;
          rim = e;
        }
        if (depth <= 0) continue;
        const st = state(b);
        // the contact point: on the squashed surfaces, between the ball and the wheel
        const x = sub(bc, mul(n, R - depth / 2));
        const rb = sub(x, bc);                    // ball center -> contact
        const rw = sub(rim, add(c, mul(a, dot(sub(rim, c), a)))); // wheel axis -> rim point (radial)
        // tangent directions: t1 along the rim's motion, t2 across it
        let t1 = cross(a, n);
        if (len(t1) < 1e-6) t1 = norm(cross(n, [1, 0, 0])); else t1 = norm(t1);
        const t2 = cross(n, t1);
        let dirs;
        if (wh.kind === 'omni') dirs = [[t1, wh.mu], [t2, MU_FREE]];
        else if (wh.kind === 'mecanum') {
          const g = (wh.mecanum * Math.PI) / 180, cg = Math.cos(g), sg = Math.sin(g);
          dirs = [[add(mul(t1, cg), mul(t2, sg)), wh.mu], [add(mul(t1, -sg), mul(t2, cg)), MU_FREE]];
        } else dirs = [[t1, wh.mu], [t2, wh.mu]];
        const { F, k } = squeeze(depth, wh.tread);
        const cdamp = 2 * ZETA * Math.sqrt(k * MB);
        // effective masses for each friction direction (ball translation + spin, shaft spin)
        const J = wh.shaft.J;
        const fric = dirs.map(([u, mu]) => {
          const ru = cross(rb, u), wu = dot(cross(rw, u), a);
          const inv = 1 / MB + dot(ru, ru) / IB + (wu * wu) / J;
          return { u, mu, m: 1 / inv, lam: 0, wu };
        });
        C.push({ st, W1, n, depth, x, rb, rw, F, k, c: cdamp, lamN: 0, fric, plain: wh.kind === 'plain' });
      }
    }
    return C;
  }

  // the contacts' impulses over a step of h (projected Gauss-Seidel)
  _solve(C, h, frameVel, reaction) {
    // relative velocity of the ball's surface over the wheel's at the contact
    const relVel = (ct) => {
      const { st, W1, x, rb, rw } = ct;
      const vb = add(st.v, cross(st.w, rb));
      const vw = add(frameVel(x), mul(cross(W1.a, rw), W1.wh.shaft.omega));
      return sub(vb, vw);
    };
    const apply = (ct, u, lam) => {
      const { st, W1, rb, rw, x } = ct;
      st.v = add(st.v, mul(u, lam / MB));
      st.w = add(st.w, mul(cross(rb, u), lam / IB));
      W1.wh.shaft.impulse(-lam * dot(cross(rw, u), W1.a));
      reaction.push([x, mul(u, -lam)]);
    };
    for (let it = 0; it < this.iterations; it++) {
      for (const ct of C) {
        // normal: the foam spring and damper, implicit over the step, never pulling
        const vn = dot(relVel(ct), ct.n);
        const g = h * (ct.k * h + ct.c) / MB;
        const target = (h * ct.F - h * (ct.k * h + ct.c) * vn + g * ct.lamN) / (1 + g);
        const lamN = Math.max(0, target);
        const dN = lamN - ct.lamN;
        if (dN !== 0) { apply(ct, ct.n, dN); ct.lamN = lamN; }
        // friction: stop the sliding along each grip direction, up to mu x the squeeze
        const vr = relVel(ct);
        if (ct.plain) {
          // a plain wheel: one friction circle over both directions
          const [f1, f2] = ct.fric;
          let l1 = f1.lam - dot(vr, f1.u) * f1.m, l2 = f2.lam - dot(vr, f2.u) * f2.m;
          const lim = f1.mu * ct.lamN, m = Math.hypot(l1, l2);
          if (m > lim) { l1 *= lim / m; l2 *= lim / m; }
          apply(ct, f1.u, l1 - f1.lam); f1.lam = l1;
          apply(ct, f2.u, l2 - f2.lam); f2.lam = l2;
        } else {
          for (const f of ct.fric) {
            const lim = f.mu * ct.lamN;
            const l = Math.max(-lim, Math.min(lim, f.lam - dot(relVel(ct), f.u) * f.m));
            apply(ct, f.u, l - f.lam);
            f.lam = l;
          }
        }
      }
    }
  }
}
