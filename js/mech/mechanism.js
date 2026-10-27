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
//   - Arms: a surface or a wheel carried on a swinging arm (an intake on its pivot) moves with the
//     pivot's shaft, and what it touches pushes back on that shaft (its motor, the arm's inertia,
//     a hard stop it's resting on): a FUEL squeezed under an arm's roller lifts the arm as far as
//     the pivot's motor and the arm's weight let it.
import { FUEL } from '../constants.js';
import { MOTORS } from '../drivetrain.js';

const R = FUEL.radius, MB = FUEL.mass;
const IB = 0.4 * MB * R * R;         // solid foam sphere
const ZETA = 0.25;                    // foam damping ratio (a free FUEL bounces back ~45%)
const MU_FREE = 0.04;                 // an omni/mecanum wheel's rollers across their free direction
// FUEL on FUEL: foam gripping foam (an estimate), so FUEL pressed together roll and spin each
// other along rather than slide
export const MU_FUEL = 0.6;
const MU_SIDE = 0.3;                  // FUEL on a wheel's side (its plates, hub)

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
  // friction: its bearings' running torque (N·m, Coulomb: the same at any speed, and holding it
  // still against anything less; 0, the default, for none)
  constructor({ name = '', motor = null, inertia = 5e-5, drag = 2e-4, friction = 0 } = {}) {
    Object.assign(this, { name, motor, inertia, drag, friction });
    this._p0 = 0; this._acc = 0; this._B = 0; // (the bearings' friction over the step: integrate)
    this._omega = 0;
    this.cmd = 0;
    this.train = null; // on a Train: it turns at k x the train's speed
    this.k = 1;
    this.lo = -Infinity; this.hi = Infinity; // (a hard stop it's against: stop())
    this._free = 0;
    this.load = 0; // a steady torque on it besides its motor's (N·m: an arm's weight), in integrate
  }
  // Against a hard stop this step (an arm down on its stop): its speed can't go below lo or above
  // hi (rad/s; 0 on the stop's side), the stop taking whatever would push it on through. The speed
  // it would have had without the stop is kept, so what pushes it back off the stop has to undo
  // what pushed it into it first (a FUEL lifting an arm off its stop lifts its weight and its
  // motor's push too). Set before each step; stop() with no bounds frees it.
  stop(lo = -Infinity, hi = Infinity) { this.lo = lo; this.hi = hi; this._free = this._omega; this._omega = Math.min(hi, Math.max(lo, this._omega)); }
  get omega() { return this.train ? this.train.omega * this.k : this._omega; }
  set omega(v) { if (this.train) this.train.omega = v / this.k; else this._omega = v; }
  // the inertia felt turning this shaft (on a train: the whole train's, seen through its ratio)
  get J() { return this.train ? this.train.J / (this.k * this.k) : this.inertia + (this.motor ? this.motor.J : 0); }
  // an angular impulse about its axis (N·m·s)
  impulse(L) {
    if (this.train) this.train.omega += (L * this.k) / this.train.J;
    else if (this._B > 0) this._fric(this._acc += L);
    else if (this.lo === -Infinity && this.hi === Infinity) this._omega += L / this.J;
    else { this._free += L / this.J; this._omega = Math.min(this.hi, Math.max(this.lo, this._free)); }
  }
  // the motor's step, with back-EMF taken implicitly
  integrate(dt, V = 12) {
    if (this.train) return;
    const J = this.J;
    // (no motor, or one in coast: its windings open, it only drags)
    if (!this.motor || this.cmd === null) {
      this._omega = (this._omega + (dt * this.load) / J) / (1 + (dt * this.drag) / J);
      if (this.motor) this.motor.current = 0;
    } else {
      const { tau, limited } = this.motor.torque(this._omega, this.cmd, V);
      const kd = limited ? 0 : this.motor.slope;
      this._omega = (J * this._omega + dt * (tau + this.load + kd * this._omega)) / (J + dt * (kd + this.drag));
    }
    // (against a stop: it takes what would drive the shaft on into it)
    if (this.lo !== -Infinity || this.hi !== Infinity) { this._free = this._omega; this._omega = Math.min(this.hi, Math.max(this.lo, this._omega)); }
    // (its bearings' friction: up to friction x dt of angular impulse against whichever way the
    // step's momentum, its own and what the contacts give it, would turn it; it stays still if
    // that's all there is, so a FUEL pressed on it meets a fixed face until it's pushed past it)
    if (this.friction > 0 && this.lo === -Infinity && this.hi === Infinity) {
      this._p0 = this._omega * J; this._acc = 0; this._B = this.friction * dt;
      this._fric(0);
    } else this._B = 0;
  }
  _fric(acc) { const p = this._p0 + acc; this._omega = (Math.sign(p) * Math.max(0, Math.abs(p) - this._B)) / this.J; }
}

// A gear train: one motor turning several shafts through belts, chain and gears at fixed ratios
// (8793's conveyor and intake rollers all run off one Kraken X60). It has one speed, the motor's
// output: each shaft turns at its k times that, the rotor's and every shaft's inertia move
// together, and a FUEL dragging on any one roller slows them all. More motors can be geared into
// it (couple: 8793's two X44s, locked to its X60 through their 90-degree gearboxes), each with its
// own command and current limits; their torques add.
export class Train {
  // motor: a Motor (its own ratio: a gearbox on it); shafts: [[Shaft, k], ...], k: the shaft's
  // turns per turn of the motor's output (the belt/gear ratios along the way multiplied)
  constructor({ name = '', motor, shafts = [] } = {}) {
    Object.assign(this, { name, motor });
    this.omega = 0;
    this.cmd = 0;
    this.shafts = [];
    this.coupled = [];
    for (const [s, k] of shafts) this.add(s, k);
  }
  add(s, k) { s.train = this; s.k = k; this.shafts.push(s); return s; }
  // another motor geared in, turning k times the main motor's output; set its .cmd (null: coast)
  couple(motor, k) { const m = { motor, k, cmd: 0 }; this.coupled.push(m); return m; }
  get J() { return this.motor.J + this.coupled.reduce((a, m) => a + m.motor.J * m.k * m.k, 0) + this.shafts.reduce((a, s) => a + s.inertia * s.k * s.k, 0); }
  get drag() { return this.shafts.reduce((a, s) => a + s.drag * s.k * s.k, 0); }
  integrate(dt, V = 12) {
    const J = this.J;
    // each motor's torque at its own speed, seen at the main motor's output; the ones that
    // aren't current limited fall off with speed (taken implicitly); a coasting one does nothing
    let tau = 0, kd = 0;
    for (const { motor, k, cmd } of [{ motor: this.motor, k: 1, cmd: this.cmd }, ...this.coupled]) {
      if (cmd === null) { motor.current = 0; continue; }
      const r = motor.torque(this.omega * k, cmd, V);
      tau += k * r.tau;
      if (!r.limited) kd += k * k * motor.slope;
    }
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
  // a wheel on a swinging arm (an intake's roller, the arm on its pivot): it still turns on its
  // own shaft, and it's carried round the pivot's shaft too. A FUEL it touches feels the wheel's
  // axle moving with the arm, and pushes back on the pivot (its motor, the arm's inertia, a stop):
  // squeezed under the roller, it lifts the arm. pivot, axis: a point on the pivot's axis and its
  // direction, in the frame's coordinates (center stays where the arm is now: pose it each step).
  onPivot(shaft, pivot, axis = [0, 0, 1]) { Object.assign(this, { pivotShaft: shaft, pivot, pivotAxis: axis }); return this; }
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
// what a FUEL touching another FUEL touches
const OTHER_FUEL = { wh: { name: 'FUEL', kind: 'plain', shaft: FIXED }, a: [0, 0, 1] };

// A fixed surface FUEL can touch: a triangle mesh (vertices: flat [x, y, z, ...] in m, indices:
// flat triangles), placed on the frame at pos, turned yaw (rad) about the vertical and then
// pitched roll (rad) about its own z (setPose: a turret turns, an intake arm swings), with
// friction mu against FUEL. Hard: the foam does all the giving.
export class Surface {
  // tread: how stiff the surface itself is where a FUEL presses it (N/m; Infinity for a hard one,
  // less for a foam bumper, which gives with the FUEL)
  constructor({ name = '', vertices, indices, mu = 0.3, pos = [0, 0, 0], yaw = 0, roll = 0, cell = 0.08, tread = Infinity }) {
    Object.assign(this, { name, mu, cell, kind: 'plain', tread, shaft: FIXED });
    this.setPose(pos, yaw, roll);
    // (no zero-area triangles: a mesh welded to a grid can collapse one to a line or a point, and
    // its contact normal would be NaN)
    indices = Array.from({ length: indices.length / 3 }, (_, t) => t).filter((t) => {
      const a = indices[t * 3] * 3, b = indices[t * 3 + 1] * 3, c = indices[t * 3 + 2] * 3, V = vertices;
      const ux = V[b] - V[a], uy = V[b + 1] - V[a + 1], uz = V[b + 2] - V[a + 2], vx = V[c] - V[a], vy = V[c + 1] - V[a + 1], vz = V[c + 2] - V[a + 2];
      return Math.hypot(uy * vz - uz * vy, uz * vx - ux * vz, ux * vy - uy * vx) > 1e-12;
    }).flatMap((t) => [indices[t * 3], indices[t * 3 + 1], indices[t * 3 + 2]]);
    const n = indices.length / 3;
    const T = (this.tri = new Float64Array(n * 9));
    for (let t = 0; t < n; t++) for (let k = 0; k < 3; k++) for (let j = 0; j < 3; j++) T[t * 9 + k * 3 + j] = vertices[indices[t * 3 + k] * 3 + j];
    // each triangle's plane (unit normal, offset), for a quick no
    const P = (this.plane = new Float64Array(n * 4));
    const lo = [Infinity, Infinity, Infinity], hi = [-Infinity, -Infinity, -Infinity];
    for (let t = 0; t < n; t++) {
      const o = t * 9;
      const ux = T[o + 3] - T[o], uy = T[o + 4] - T[o + 1], uz = T[o + 5] - T[o + 2];
      const vx = T[o + 6] - T[o], vy = T[o + 7] - T[o + 1], vz = T[o + 8] - T[o + 2];
      let nx = uy * vz - uz * vy, ny = uz * vx - ux * vz, nz = ux * vy - uy * vx;
      const l = Math.hypot(nx, ny, nz) || 1;
      nx /= l; ny /= l; nz /= l;
      P[t * 4] = nx; P[t * 4 + 1] = ny; P[t * 4 + 2] = nz; P[t * 4 + 3] = nx * T[o] + ny * T[o + 1] + nz * T[o + 2];
      for (let k = 0; k < 3; k++) for (let j = 0; j < 3; j++) { lo[j] = Math.min(lo[j], T[o + k * 3 + j]); hi[j] = Math.max(hi[j], T[o + k * 3 + j]); }
    }
    // and its bounds
    const B = (this.box = new Float64Array(n * 6));
    for (let t = 0; t < n; t++) for (let j = 0; j < 3; j++) {
      B[t * 6 + j] = Math.min(T[t * 9 + j], T[t * 9 + 3 + j], T[t * 9 + 6 + j]);
      B[t * 6 + 3 + j] = Math.max(T[t * 9 + j], T[t * 9 + 3 + j], T[t * 9 + 6 + j]);
    }
    this.lo = lo.map((v) => v - R); this.hi = hi.map((v) => v + R);
    // a grid of cells over its bounds, each listing the triangles whose bounds reach into it
    // (flat: cell c's triangles are ids[start[c] .. start[c + 1]])
    const g0 = (this.g0 = lo.map((v) => Math.floor((v - R) / cell)));
    const gn = (this.gn = hi.map((v, j) => Math.floor((v + R) / cell) - g0[j] + 1));
    const cellOf = (i, j, k) => (i * gn[1] + j) * gn[2] + k;
    const span = (t, j) => [Math.floor(Math.min(T[t * 9 + j], T[t * 9 + 3 + j], T[t * 9 + 6 + j]) / cell) - g0[j], Math.floor(Math.max(T[t * 9 + j], T[t * 9 + 3 + j], T[t * 9 + 6 + j]) / cell) - g0[j]];
    const count = new Uint32Array(gn[0] * gn[1] * gn[2] + 1);
    const each = (t, f) => {
      const [a0, a1] = span(t, 0), [b0, b1] = span(t, 1), [c0, c1] = span(t, 2);
      for (let i = a0; i <= a1; i++) for (let j = b0; j <= b1; j++) for (let k = c0; k <= c1; k++) f(cellOf(i, j, k));
    };
    for (let t = 0; t < n; t++) each(t, (c) => count[c + 1]++);
    for (let c = 1; c < count.length; c++) count[c] += count[c - 1];
    this.start = count;
    this.ids = new Uint32Array(count[count.length - 1]);
    const fill = count.slice();
    for (let t = 0; t < n; t++) each(t, (c) => { this.ids[fill[c]++] = t; });
    this.stamp = new Uint32Array(n);
    this.mark = 0;
    this.q = new Float64Array(3);
  }
  // a surface that swings on a shaft (an intake arm on its pivot): FUEL it touches feel it moving,
  // and push back on that shaft's motor. pivot, axis: a point on the shaft's axis and its
  // direction, in the frame's coordinates.
  onShaft(shaft, pivot, axis = [0, 0, 1]) { Object.assign(this, { shaft, pivot, pivotAxis: axis }); return this; }
  // a surface that slides along dir (frame coordinates) as a shaft turns, slideK m per radian of
  // the shaft (a pin in a slot): FUEL it touches feel it moving, and push back on that shaft
  onSlide(shaft, dir = [1, 0, 0]) { Object.assign(this, { shaft, slideDir: dir, slideK: 0 }); return this; }
  setPose(pos, yaw = 0, roll = 0) {
    this.pos = pos;
    // the surface's axes in the frame: Ry(yaw) Rz(roll), columns
    const cy = Math.cos(yaw), sy = Math.sin(yaw), cr = Math.cos(roll), sr = Math.sin(roll);
    const ex = [cy * cr, sr, -sy * cr], ey = [-cy * sr, cr, sy * sr], ez = [sy, 0, cy];
    this.axes = [ex, ey, ez];
  }
  // a point in the frame -> the surface's own coordinates, and back for a direction
  toLocal(p) {
    const d = sub(p, this.pos), [ex, ey, ez] = this.axes;
    return [dot(d, ex), dot(d, ey), dot(d, ez)];
  }
  dirToFrame(v) {
    const [ex, ey, ez] = this.axes;
    return add(add(mul(ex, v[0]), mul(ey, v[1])), mul(ez, v[2]));
  }
  // where a FUEL at p (the surface's coordinates) touches it: [{ depth, n (out of the surface,
  // toward the FUEL) }], one per face it's against (neighboring triangles of one face, and a thin
  // plate's far side, merged into the deepest)
  touches(p) {
    const [px, py, pz] = p, r = R;
    if (px < this.lo[0] || px > this.hi[0] || py < this.lo[1] || py > this.hi[1] || pz < this.lo[2] || pz > this.hi[2]) return NONE;
    const cell = this.cell, T = this.tri, P = this.plane, B = this.box, q = this.q, g0 = this.g0, gn = this.gn, start = this.start, ids = this.ids, stamp = this.stamp;
    const i0 = Math.max(0, Math.floor((px - r) / cell) - g0[0]), i1 = Math.min(gn[0] - 1, Math.floor((px + r) / cell) - g0[0]);
    const j0 = Math.max(0, Math.floor((py - r) / cell) - g0[1]), j1 = Math.min(gn[1] - 1, Math.floor((py + r) / cell) - g0[1]);
    const k0 = Math.max(0, Math.floor((pz - r) / cell) - g0[2]), k1 = Math.min(gn[2] - 1, Math.floor((pz + r) / cell) - g0[2]);
    if (++this.mark === 0xffffffff) { stamp.fill(0); this.mark = 1; }
    const mark = this.mark;
    let out = NONE;
    for (let i = i0; i <= i1; i++) for (let j = j0; j <= j1; j++) for (let k = k0; k <= k1; k++) {
      const c = (i * gn[1] + j) * gn[2] + k;
      for (let m = start[c], e = start[c + 1]; m < e; m++) {
        const t = ids[m];
        if (stamp[t] === mark) continue;
        stamp[t] = mark;
        const o = t * 6;
        if (px + r < B[o] || px - r > B[o + 3] || py + r < B[o + 1] || py - r > B[o + 4] || pz + r < B[o + 2] || pz - r > B[o + 5]) continue;
        const sd = P[t * 4] * px + P[t * 4 + 1] * py + P[t * 4 + 2] * pz - P[t * 4 + 3];
        if (sd >= r || sd <= -r) continue;
        // (the circle where the FUEL's sphere cuts the triangle's plane must reach the triangle's bounds)
        const cr = Math.sqrt(r * r - sd * sd), cx = px - P[t * 4] * sd, cy = py - P[t * 4 + 1] * sd, cz = pz - P[t * 4 + 2] * sd;
        if (cx + cr < B[o] || cx - cr > B[o + 3] || cy + cr < B[o + 1] || cy - cr > B[o + 4] || cz + cr < B[o + 2] || cz - cr > B[o + 5]) continue;
        closestOnTri(px, py, pz, T, t * 9, q);
        const dx = px - q[0], dy = py - q[1], dz = pz - q[2], dl = Math.sqrt(dx * dx + dy * dy + dz * dz);
        if (dl >= r || dl < 1e-9) continue;
        const depth = r - dl, nx = dx / dl, ny = dy / dl, nz = dz / dl;
        // one per face: into the one it's alongside if any (the deeper of the two kept)
        let merged = false;
        for (const f of out) {
          if (f.n[0] * nx + f.n[1] * ny + f.n[2] * nz > 0.9) {
            if (depth > f.depth) { f.depth = depth; f.n[0] = nx; f.n[1] = ny; f.n[2] = nz; }
            merged = true;
            break;
          }
        }
        if (!merged) (out === NONE ? (out = []) : out).push({ depth, n: [nx, ny, nz] });
      }
    }
    return out;
  }
}
const NONE = [];
// the point of triangle t (at T[o..o+8]) nearest p, into q (Ericson, Real-Time Collision
// Detection 5.1.5)
function closestOnTri(px, py, pz, T, o, q) {
  const ax = T[o], ay = T[o + 1], az = T[o + 2];
  const abx = T[o + 3] - ax, aby = T[o + 4] - ay, abz = T[o + 5] - az;
  const acx = T[o + 6] - ax, acy = T[o + 7] - ay, acz = T[o + 8] - az;
  const apx = px - ax, apy = py - ay, apz = pz - az;
  const d1 = abx * apx + aby * apy + abz * apz, d2 = acx * apx + acy * apy + acz * apz;
  const set = (x, y, z) => { q[0] = x; q[1] = y; q[2] = z; };
  if (d1 <= 0 && d2 <= 0) return set(ax, ay, az);
  const bpx = px - T[o + 3], bpy = py - T[o + 4], bpz = pz - T[o + 5];
  const d3 = abx * bpx + aby * bpy + abz * bpz, d4 = acx * bpx + acy * bpy + acz * bpz;
  if (d3 >= 0 && d4 <= d3) return set(T[o + 3], T[o + 4], T[o + 5]);
  const vc = d1 * d4 - d3 * d2;
  if (vc <= 0 && d1 >= 0 && d3 <= 0) { const v = d1 / (d1 - d3); return set(ax + abx * v, ay + aby * v, az + abz * v); }
  const cpx = px - T[o + 6], cpy = py - T[o + 7], cpz = pz - T[o + 8];
  const d5 = abx * cpx + aby * cpy + abz * cpz, d6 = acx * cpx + acy * cpy + acz * cpz;
  if (d6 >= 0 && d5 <= d6) return set(T[o + 6], T[o + 7], T[o + 8]);
  const vb = d5 * d2 - d1 * d6;
  if (vb <= 0 && d2 >= 0 && d6 <= 0) { const w = d2 / (d2 - d6); return set(ax + acx * w, ay + acy * w, az + acz * w); }
  const va = d3 * d6 - d5 * d4;
  if (va <= 0 && d4 - d3 >= 0 && d5 - d6 >= 0) {
    const w = (d4 - d3) / ((d4 - d3) + (d5 - d6));
    return set(T[o + 3] + (T[o + 6] - T[o + 3]) * w, T[o + 4] + (T[o + 7] - T[o + 4]) * w, T[o + 5] + (T[o + 8] - T[o + 5]) * w);
  }
  const den = 1 / (va + vb + vc), v = vb * den, w = vc * den;
  return set(ax + abx * v + acx * w, ay + aby * v + acy * w, az + abz * v + acz * w);
}

// A solid block of foam FUEL can touch, fixed to the frame: a bumper (solid, uniform foam under
// fabric, as teams build them now). Its section is a rectangle in x and y (x0 to x1, y0 to y1),
// edges rounded r, across z from -hw to hw (the ends rounded too). Solid: a FUEL pressed into it,
// however far, is pushed back out the nearest way, where a mesh's face alone would let one
// through. (Its triangles are only for drawing it.)
export class Block extends Surface {
  constructor({ name = '', x0, x1, y0, y1, hw, r = 0.02, mu = 0.6, tread = Infinity }) {
    const V = [], I = [];
    for (const z of [-hw, hw]) for (const [x, y] of [[x0, y0], [x1, y0], [x1, y1], [x0, y1]]) V.push(x, y, z);
    for (const [a, b, c, d] of [[1, 2, 6, 5], [2, 3, 7, 6], [0, 1, 5, 4], [0, 4, 7, 3], [0, 3, 2, 1], [4, 5, 6, 7]]) I.push(a, b, c, a, c, d);
    super({ name, vertices: V, indices: I, mu, tread });
    this.c = [(x0 + x1) / 2, (y0 + y1) / 2, 0];
    this.h = [(x1 - x0) / 2 - r, (y1 - y0) / 2 - r, hw - r];
    this.r = r;
  }
  touches(p) {
    const d = [0, 1, 2].map((k) => p[k] - this.c[k]), q = d.map((v, k) => Math.abs(v) - this.h[k]);
    const out = q.map((v) => Math.max(v, 0)), ol = Math.hypot(...out);
    let dist, n;
    if (ol > 0) {
      dist = ol - this.r;
      n = out.map((v, k) => (v / ol) * Math.sign(d[k]));
    } else {
      // (inside its core: out the nearest face)
      const k = q.indexOf(Math.max(...q));
      dist = q[k] - this.r;
      n = [0, 0, 0]; n[k] = Math.sign(d[k]) || 1;
    }
    const depth = R - dist;
    return depth > 0 ? [{ depth, n }] : NONE;
  }
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
    this.dents = [];     // its last substep's, every one (step)
    this.V = 12;
  }

  // the frame's pose and motion in the world
  _frame() {
    const b = this.body;
    if (!b) return { p: [0, 0, 0], q: { x: 0, y: 0, z: 0, w: 1 }, v: [0, 0, 0], w: [0, 0, 0], com: [0, 0, 0] };
    const t = b.translation(), q = b.rotation(), v = b.linvel(), w = b.angvel(), c = b.worldCom ? b.worldCom() : t;
    return { p: [t.x, t.y, t.z], q, v: [v.x, v.y, v.z], w: [w.x, w.y, w.z], com: [c.x, c.y, c.z] };
  }

  // balls: the FUEL rigid bodies that might touch it this step; together: those of them whose
  // touching each other it works out too (foam on foam, with friction, solved with the wheels';
  // the world mustn't also collide them with each other)
  step(dt, balls, together = null) {
    this.together = together ? new Set(together) : null;
    for (const s of this.shafts) s.integrate(dt, this.V);
    for (const t of new Set(this.shafts.map((s) => s.train).filter(Boolean))) t.integrate(dt, this.V);
    const fr = this._frame();
    const frameVel = (x) => add(fr.v, cross(fr.w, sub(x, fr.com)));
    // wheels in the world (and the pivot's axis, for one on an arm)
    const W = this.wheels.map((wh) => {
      const w = { wh, c: add(fr.p, qrot(fr.q, wh.center)), a: qrot(fr.q, wh.axis) };
      if (wh.pivotShaft) { w.p = add(fr.p, qrot(fr.q, wh.pivot)); w.pa = qrot(fr.q, wh.pivotAxis); }
      return w;
    });
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
    // (and every contact of its last substep, each FUEL's dent against a wheel or a surface: where
    // a FUEL is drawn flattened, js/mech/physical.js)
    this.dents = [];
    for (let k = 0; k < n; k++) {
      if (n > 1) for (const st of S.values()) st.v = add(st.v, mul(g, h));
      const C = this._contacts(W, fr, n > 1 ? [...S.values()].map((st) => [st.b, st.c]) : balls.map((b) => { const t = b.translation(); return [b, [t.x, t.y, t.z]]; }), state);
      this._solve(C, h, frameVel, reaction);
      if (n > 1) for (const st of S.values()) st.c = add(st.c, mul(st.v, h));
      for (const ct of C) {
        if (ct.st2) continue;
        const key = ct.W1.wh, m = seen.get(key) || new Map();
        seen.set(key, m);
        const was = m.get(ct.st.b);
        if (!was || ct.lamN / h > was.force) m.set(ct.st.b, { wheel: ct.W1.wh, ball: ct.st.b, depth: ct.depth, force: ct.lamN / h, x: ct.x, n: ct.n });
        if (k === n - 1) this.dents.push({ wheel: ct.W1.wh, ball: ct.st.b, depth: ct.depth, n: ct.n });
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
          const x = sub(bc, mul(n, R - depth / 2)), rb = sub(x, bc);
          // (on a shaft: the contact's arm about its axis, which moves the surface there)
          let a = [0, 0, 1], rw = [0, 0, 0];
          if (sf.slideDir) {
            // (sliding: its velocity slideK omega along slideDir, as (a x rw) omega with a normal to it)
            const L = mul(qrot(fr.q, sf.slideDir), sf.slideK), az = Math.abs(sf.slideDir[2]) < 0.9 ? [0, 0, 1] : [1, 0, 0];
            a = qrot(fr.q, az);
            rw = cross(L, a);
          } else if (sf.shaft !== FIXED) {
            a = qrot(fr.q, sf.pivotAxis);
            const d = sub(x, add(fr.p, qrot(fr.q, sf.pivot)));
            rw = sub(d, mul(a, dot(d, a)));
          }
          let t1 = cross(n, Math.abs(n[1]) < 0.9 ? [0, 1, 0] : [1, 0, 0]);
          t1 = norm(t1);
          const t2 = cross(n, t1);
          // (a soft surface, a foam bumper, gives with the FUEL: in series, like a tread)
          const { F, k } = squeeze(depth, sf.tread);
          const fric = [t1, t2].map((u) => { const ru = cross(rb, u); return { u, mu: sf.mu, m: 1 / (1 / MB + dot(ru, ru) / IB), lam: 0, wu: 0 }; });
          C.push({ st, W1: { wh: sf, a }, n, depth, x, rb, rw, F, k, c: 2 * ZETA * Math.sqrt(k * MB), lamN: 0, fric, plain: true });
        }
      }
      for (const W1 of W) {
        const { wh, c, a } = W1, hw = wh.width / 2;
        const dx = bc[0] - c[0], dy = bc[1] - c[1], dz = bc[2] - c[2];
        const s = dx * a[0] + dy * a[1] + dz * a[2];
        if (Math.abs(s) > hw + R) continue;
        // (too far out from its axis to touch)
        const reach = wh.radius + R;
        if (dx * dx + dy * dy + dz * dz - s * s >= reach * reach) continue;
        const d = [dx, dy, dz], rad = sub(d, mul(a, s)), rl = len(rad);
        let n, depth, rim, side = false;
        if (Math.abs(s) <= hw) {
          // on the tread
          if (rl < 1e-6) continue;
          n = mul(rad, 1 / rl);
          depth = R + wh.radius - rl;
          rim = add(c, add(mul(a, s), mul(n, wh.radius)));
        } else if (rl <= wh.radius) {
          // against its side (over the wheel, for a wheel lying flat)
          n = mul(a, Math.sign(s));
          depth = R - (Math.abs(s) - hw);
          rim = add(c, add(mul(a, Math.sign(s) * hw), rad));
          side = true;
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
        if (side) dirs = [[t1, MU_SIDE], [t2, MU_SIDE]];
        else if (wh.kind === 'omni') dirs = [[t1, wh.mu], [t2, MU_FREE]];
        else if (wh.kind === 'mecanum') {
          const g = (wh.mecanum * Math.PI) / 180, cg = Math.cos(g), sg = Math.sin(g);
          dirs = [[add(mul(t1, cg), mul(t2, sg)), wh.mu], [add(mul(t1, -sg), mul(t2, cg)), MU_FREE]];
        } else dirs = [[t1, wh.mu], [t2, wh.mu]];
        const { F, k } = squeeze(depth, side ? Infinity : wh.tread);
        const cdamp = 2 * ZETA * Math.sqrt(k * MB);
        // (on an arm: the contact's arm about the pivot's axis, which moves the wheel there; the
        // arm's inertia felt there, in every direction, normal too)
        let piv = null;
        if (W1.p) { const dp = sub(x, W1.p); piv = { shaft: wh.pivotShaft, a: W1.pa, r: sub(dp, mul(W1.pa, dot(dp, W1.pa))) }; }
        const pw = (u) => (piv ? dot(cross(piv.r, u), piv.a) ** 2 / piv.shaft.J : 0);
        // effective masses for each friction direction (ball translation + spin, shaft spin, arm)
        const J = wh.shaft.J;
        const fric = dirs.map(([u, mu]) => {
          const ru = cross(rb, u), wu = dot(cross(rw, u), a);
          const inv = 1 / MB + dot(ru, ru) / IB + (wu * wu) / J + pw(u);
          return { u, mu, m: 1 / inv, lam: 0, wu };
        });
        C.push({ st, W1, n, depth, x, rb, rw, F, k, c: cdamp, lamN: 0, fric, plain: side || wh.kind === 'plain', piv, im: piv ? 1 / MB + pw(n) : undefined });
      }
    }
    // FUEL against FUEL: squeezed together by d, each dents d/2, as against a plate (foam(d/2));
    // in series, half as stiff. Friction where they touch, both spinning.
    if (this.together) {
      const T = at.filter(([b]) => this.together.has(b));
      for (let i = 0; i < T.length; i++) for (let j = i + 1; j < T.length; j++) {
        const [b1, c1] = T[i], [b2, c2] = T[j];
        const ex = c1[0] - c2[0], ey = c1[1] - c2[1], ez = c1[2] - c2[2];
        if (ex * ex + ey * ey + ez * ez >= 4 * R * R) continue;
        const d = [ex, ey, ez], dl = len(d), depth = 2 * R - dl;
        if (depth <= 0 || dl < 1e-6) continue;
        const st = state(b1), st2 = state(b2), n = mul(d, 1 / dl);
        const x = sub(c1, mul(n, dl / 2)), rb = sub(x, c1), rb2 = sub(x, c2);
        const t1 = norm(cross(n, Math.abs(n[1]) < 0.9 ? [0, 1, 0] : [1, 0, 0])), t2 = cross(n, t1);
        const f = foam(depth / 2), F = f.F, k = f.k / 2;
        const fric = [t1, t2].map((u) => { const r1 = cross(rb, u), r2 = cross(rb2, u); return { u, mu: MU_FUEL, m: 1 / (2 / MB + (dot(r1, r1) + dot(r2, r2)) / IB), lam: 0, wu: 0 }; });
        C.push({ st, st2, W1: OTHER_FUEL, n, depth, x, rb, rb2, rw: [0, 0, 0], F, k, c: 2 * ZETA * Math.sqrt(k * MB / 2), im: 2 / MB, lamN: 0, fric, plain: true });
      }
    }
    return C;
  }

  // the contacts' impulses over a step of h (projected Gauss-Seidel)
  _solve(C, h, frameVel, reaction) {
    // relative velocity of the ball's surface over the wheel's at the contact
    const relVel = (ct) => {
      const { st, st2, W1, x, rb, rw } = ct;
      const vb = add(st.v, cross(st.w, rb));
      if (st2) return sub(vb, add(st2.v, cross(st2.w, ct.rb2)));
      let vw = add(frameVel(x), mul(cross(W1.a, rw), W1.wh.shaft.omega));
      if (ct.piv) vw = add(vw, mul(cross(ct.piv.a, ct.piv.r), ct.piv.shaft.omega));
      return sub(vb, vw);
    };
    const apply = (ct, u, lam) => {
      const { st, st2, W1, rb, rw, x } = ct;
      st.v = add(st.v, mul(u, lam / MB));
      st.w = add(st.w, mul(cross(rb, u), lam / IB));
      if (st2) {
        st2.v = add(st2.v, mul(u, -lam / MB));
        st2.w = add(st2.w, mul(cross(ct.rb2, u), -lam / IB));
        return;
      }
      W1.wh.shaft.impulse(-lam * dot(cross(rw, u), W1.a));
      if (ct.piv) ct.piv.shaft.impulse(-lam * dot(cross(ct.piv.r, u), ct.piv.a));
      reaction.push([x, mul(u, -lam)]);
    };
    for (let it = 0; it < this.iterations; it++) {
      for (const ct of C) {
        // normal: the foam spring and damper, implicit over the step, never pulling
        const vn = dot(relVel(ct), ct.n);
        const g = h * (ct.k * h + ct.c) * (ct.im ?? 1 / MB);
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
