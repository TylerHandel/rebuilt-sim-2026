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
//     constants.js): F = k0 xc (e^(x/xc) - 1), in series with the wheel's own tread (a 30A flex
//     wheel gives too), with damping set for FUEL's bounce.
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
import { FUEL } from '../constants.js';
import { MOTORS } from '../drivetrain.js';

const R = FUEL.radius, MB = FUEL.mass;
const IB = 0.4 * MB * R * R;         // solid foam sphere
const ZETA = 0.25;                    // foam damping ratio (a free FUEL bounces back ~45%)
const MU_FREE = 0.04;                 // an omni/mecanum wheel's rollers across their free direction

// ------------------------------------------------------------------ foam
// squeeze x (m) -> force (N) and its slope (N/m)
export function foam(x) {
  const k0 = FUEL.springRate0, xc = FUEL.stiffen;
  const e = Math.exp(Math.min(Math.max(0, x), 3.5 * xc) / xc);
  return { F: k0 * xc * (e - 1), k: k0 * e };
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
  // eff: the drive's efficiency on torque; statorLimit (A, per motor)
  constructor(type, { count = 1, ratio = 1, eff = 0.9, statorLimit = 60 } = {}) {
    const m = MOTORS[type];
    if (!m) throw new Error('no motor ' + type);
    Object.assign(this, { type, count, ratio, eff, statorLimit, m });
    this.Rm = m.V / m.stallCurrent;
    this.kT = m.stallTorque / m.stallCurrent;
    this.kV = m.free / (m.V - m.freeCurrent * this.Rm);
    this.J = count * (m.rotorInertia || 3e-5) * ratio * ratio; // the rotors, seen at the output
    this.current = 0;
  }
  // output torque at output speed w (rad/s), duty cmd (-1..1), battery V; and whether it's at
  // the current limit (then the torque doesn't fall with speed)
  torque(w, cmd, V = 12) {
    const I = (cmd * V - (w * this.ratio) / this.kV) / this.Rm;
    const lim = Math.abs(I) > this.statorLimit;
    const Ic = lim ? Math.sign(I) * this.statorLimit : I;
    this.current = Ic * this.count;
    return { tau: this.count * this.kT * Ic * this.ratio * this.eff, limited: lim };
  }
  // how fast the torque falls with output speed when it's not current limited (N·m per rad/s)
  get slope() { return (this.count * this.kT * this.ratio * this.ratio * this.eff) / (this.kV * this.Rm); }
  // free speed at the output (rad/s)
  get freeSpeed() { return this.m.free / this.ratio; }
}

export class Shaft {
  // motor: a Motor, or null for an idler; inertia: what turns with the shaft (kg·m²: its wheels,
  // pulleys, hex shaft), the motor's rotors are added; drag: bearing and belt friction (N·m·s)
  constructor({ name = '', motor = null, inertia = 5e-5, drag = 2e-4 } = {}) {
    Object.assign(this, { name, motor, inertia, drag });
    this.omega = 0;
    this.cmd = 0;
  }
  get J() { return this.inertia + (this.motor ? this.motor.J : 0); }
  // the motor's step, with back-EMF taken implicitly
  integrate(dt, V = 12) {
    const J = this.J;
    if (!this.motor) { this.omega /= 1 + (dt * this.drag) / J; return; }
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
// rotate v by unit quaternion q
function qrot(q, v) {
  const u = [q.x, q.y, q.z], s = q.w;
  const t = mul(cross(u, v), 2);
  return add(add(v, mul(t, s)), cross(u, t));
}

// ------------------------------------------------------------------ the mechanism
export class Mechanism {
  // body: the Rapier rigid body it's mounted on (null: fixed in the world); iterations: contact
  // solver passes per step; gravity: the world's, and substeps: its solver's substeps per step
  // (world.integrationParameters.numSolverIterations). The step's gravity is counted in the solve
  // as the world will apply it (spread over its substeps a body moves by (n+1)/2n of a full
  // step's worth), so a FUEL resting on a wheel settles where the foam holds its weight.
  constructor({ body = null, shafts = [], wheels = [], iterations = 6, gravity = [0, -9.81, 0], substeps = 4 } = {}) {
    Object.assign(this, { body, shafts, wheels, iterations, gravity });
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
    const fr = this._frame();
    const frameVel = (x) => add(fr.v, cross(fr.w, sub(x, fr.com)));
    // wheels in the world
    const W = this.wheels.map((wh) => ({ wh, c: add(fr.p, qrot(fr.q, wh.center)), a: qrot(fr.q, wh.axis) }));
    // ball states we work on
    const S = new Map();
    const state = (b) => {
      let s = S.get(b);
      if (!s) {
        const t = b.translation(), v = b.linvel(), w = b.angvel(), g = this.gravity, f = this.gShare * dt;
        s = { b, c: [t.x, t.y, t.z], v: [v.x + g[0] * f, v.y + g[1] * f, v.z + g[2] * f], w: [w.x, w.y, w.z] };
        S.set(b, s);
      }
      return s;
    };
    // contacts
    const C = [];
    for (const b of balls) {
      const t = b.translation(), bc = [t.x, t.y, t.z];
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
    // relative velocity of the ball's surface over the wheel's at the contact
    const relVel = (ct) => {
      const { st, W1, x, rb, rw } = ct;
      const vb = add(st.v, cross(st.w, rb));
      const vw = add(frameVel(x), mul(cross(W1.a, rw), W1.wh.shaft.omega));
      return sub(vb, vw);
    };
    const reaction = [];
    const apply = (ct, u, lam) => {
      const { st, W1, rb, rw, x } = ct;
      st.v = add(st.v, mul(u, lam / MB));
      st.w = add(st.w, mul(cross(rb, u), lam / IB));
      W1.wh.shaft.omega -= (lam * dot(cross(rw, u), W1.a)) / W1.wh.shaft.J;
      reaction.push([x, mul(u, -lam)]);
    };
    const h = dt;
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
    // results
    const g = this.gravity, f = this.gShare * dt;
    for (const st of S.values()) {
      st.b.setLinvel({ x: st.v[0] - g[0] * f, y: st.v[1] - g[1] * f, z: st.v[2] - g[2] * f }, true);
      st.b.setAngvel({ x: st.w[0], y: st.w[1], z: st.w[2] }, true);
    }
    if (this.body && this.body.isDynamic()) {
      for (const [x, J] of reaction) this.body.applyImpulseAtPoint({ x: J[0], y: J[1], z: J[2] }, { x: x[0], y: x[1], z: x[2] }, true);
    }
    this.contacts = C.map((ct) => ({ wheel: ct.W1.wh, ball: ct.st.b, depth: ct.depth, force: ct.lamN / dt, x: ct.x, n: ct.n }));
  }
}
