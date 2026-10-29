// Energy chains (cable carriers) on the turrets of 8793 and 4414: where each link of the chain is
// for a turret yaw, and what the chain does to the turret (js/mech/turretDrive.js).
//
// The links are WCP's semi-enclosed energy chain (WCP-0157, sold as 15mm x 30mm; the user's STEP):
// a link measures 39.5 mm long, 24.2 mm along its pins and 19.5 mm across, its pins 20.0 mm apart
// on its middle line, and two links collide when the joint between them turns past 45-46 degrees
// either way (the STEP's link against a copy of itself one pitch on, turned about the pin: OCCT
// booleans, no overlap to 45, 0.1 mm^3 at 46), so the tightest bend, on the pin line, is
// 20 / (2 sin 22.5) = 26.1 mm. Both robots' chains lie on their sides (the pins vertical), so they
// bend in the horizontal plane.
//
// Everything here works in the horizontal plane, in the robot's frame about the turret's axis, as
// [x, w]: x forward, w = -z (the robot's left), so angles run counterclockwise seen from above, the
// way the turret's yaw does. No three.js: the physics (js/mech/turretDrive.js) runs headless.
const DEG = Math.PI / 180;

export const LINK = {
  pitch: 0.02,      // pin to pin
  length: 0.0395,   // end to end (neighbors overlap, nested)
  height: 0.0195,   // across the bend (radial, lying on its side)
  width: 0.02416,   // along the pins (vertical, lying on its side)
  stop: 45 * DEG,   // the most a joint turns, either way
  mass: 0.0047,     // kg: its volume in the STEP (4076 mm^3) in nylon, 1.15 g/cm^3
};
export const R_MIN = LINK.pitch / (2 * Math.sin(LINK.stop / 2));
// a taller one for 4414's bundle: WCP's 15mm x 45mm semi-enclosed chain is 15 mm wider inside
// (no STEP of it at hand: the 15x30 link's side plates moved 15 mm apart)
export const LINK_TALL = { ...LINK, width: LINK.width + 0.015, mass: 0.0062 };
// second neighbors can come no closer than this (the joint between them at its stop)

// ---------------------------------------------------------------- a path, and links along it
// A polyline sampled densely along the chain's middle line (its pins' line), with arc length.
class Path {
  constructor(cap = 4096) { this.x = new Float64Array(cap); this.w = new Float64Array(cap); this.s = new Float64Array(cap); this.n = 0; }
  reset() { this.n = 0; return this; }
  push(x, w) {
    const n = this.n;
    if (n === this.x.length) { for (const k of ['x', 'w', 's']) { const a = new Float64Array(n * 2); a.set(this[k]); this[k] = a; } }
    this.x[n] = x; this.w[n] = w;
    this.s[n] = n ? this.s[n - 1] + Math.hypot(x - this.x[n - 1], w - this.w[n - 1]) : 0;
    this.n = n + 1;
  }
  get length() { return this.n ? this.s[this.n - 1] : 0; }
  // a circular arc about (cx, cw) from angle a0 to a1 (the first point skipped if the path's there)
  arc(cx, cw, r, a0, a1, step = 0.001) {
    const k = Math.max(1, Math.ceil((Math.abs(a1 - a0) * r) / step));
    for (let i = this.n ? 1 : 0; i <= k; i++) { const a = a0 + ((a1 - a0) * i) / k; this.push(cx + r * Math.cos(a), cw + r * Math.sin(a)); }
  }
  line(x, w, step = 0.004) {
    if (!this.n) { this.push(x, w); return; }
    const x0 = this.x[this.n - 1], w0 = this.w[this.n - 1], k = Math.max(1, Math.ceil(Math.hypot(x - x0, w - w0) / step));
    for (let i = 1; i <= k; i++) this.push(x0 + ((x - x0) * i) / k, w0 + ((w - w0) * i) / k);
  }
}

// Links of pitch p along a path from its start: each pin where the path is next p (straight line)
// from the last. Writes n + 1 pins to pins ([x, w] pairs) and returns how far along the path (arc
// length) the last one is (past its end: extrapolated along its last segment).
function chordStep(P, n, p, pins) {
  let i = 0, cx = P.x[0], cw = P.w[0], s = 0;
  pins[0] = cx; pins[1] = cw;
  for (let k = 1; k <= n; k++) {
    let found = false;
    for (let j = i; s < P.length && j < P.n - 1; j++) {
      const bx = P.x[j + 1], bw = P.w[j + 1];
      if (Math.hypot(bx - cx, bw - cw) < p) continue;
      // on segment j: from where we are (if it's this one) to its end, where |a + u d - c| = p
      const t0 = j === i ? (s - P.s[j]) / Math.max(1e-12, P.s[j + 1] - P.s[j]) : 0;
      const ax = P.x[j] + (bx - P.x[j]) * t0, aw = P.w[j] + (bw - P.w[j]) * t0;
      const dx = bx - ax, dw = bw - aw, fx = ax - cx, fw = aw - cw;
      const A = dx * dx + dw * dw, B = 2 * (fx * dx + fw * dw), C = fx * fx + fw * fw - p * p;
      const u = A > 0 ? (-B + Math.sqrt(Math.max(0, B * B - 4 * A * C))) / (2 * A) : 0;
      cx = ax + u * dx; cw = aw + u * dw;
      s = P.s[j] + (t0 + u * (1 - t0)) * (P.s[j + 1] - P.s[j]);
      i = j; found = true;
      break;
    }
    if (!found) {
      // (off the end: on along the last segment's line)
      const j = P.n - 2, dx = P.x[j + 1] - P.x[j], dw = P.w[j + 1] - P.w[j], l = Math.hypot(dx, dw) || 1;
      const ux = dx / l, uw = dw / l;
      if (s >= P.length) { cx += ux * p; cw += uw * p; s += p; } else {
        const fx = P.x[j + 1] - cx, fw = P.w[j + 1] - cw;
        const b = fx * ux + fw * uw, c = fx * fx + fw * fw - p * p, u = -b + Math.sqrt(Math.max(0, b * b - c));
        cx = P.x[j + 1] + u * ux; cw = P.w[j + 1] + u * uw;
        s = P.length + u;
      }
      i = P.n - 2;
    }
    pins[2 * k] = cx; pins[2 * k + 1] = cw;
  }
  return s;
}

// the turn at each joint (rad, + counterclockwise) of a chain's pins
export function jointAngles(pins, n) {
  const out = new Float64Array(Math.max(0, n - 1));
  for (let k = 1; k < n; k++) {
    const a = Math.atan2(pins[2 * k + 1] - pins[2 * k - 1], pins[2 * k] - pins[2 * k - 2]);
    const b = Math.atan2(pins[2 * k + 3] - pins[2 * k + 1], pins[2 * k + 2] - pins[2 * k]);
    let d = b - a; while (d > Math.PI) d -= 2 * Math.PI; while (d < -Math.PI) d += 2 * Math.PI;
    out[k - 1] = d;
  }
  return out;
}

// ---------------------------------------------------------------- 8793: a track round the turret
// From their CAD ("Cable Carrier Quadrant 1-4": four printed quarters of a ring channel on the
// tower, 1 mm over the 200T turntable, fixed; cad/robots/8793-track.glb). Inside, the channel runs
// from 5.25in (its inner wall's face) to 8.0in (the outer wall's) from the turret's axis, its floor
// 0.3177 m up and its walls 25 mm above that: the chain on its side (24.2 mm) just fits under their
// tops. On the right side a wedge on the outer wall's inside ramps in from -120 deg to a step at
// -93.6 deg (0 straight ahead, + to the left): where the chain's fixed end stops.
// The chain (the user): it starts on the right of the track, its fixed end screwed through the
// outer ring there (holes drilled after the fact), runs counterclockwise round the outside, turns
// back on itself and snakes back round the inside, clockwise, to its moving end on a bracket under
// the back-right corner of the shooter's base plate. The turret's yaw changes how much is outside and
// how much inside; the U where it turns back rolls round the track at r_i / (r_o + r_i) of the
// turret's speed, the outside run lying still against the outer wall and the inside run turning
// with the turret, sliding along the inner wall and the floor.
//
// The U is tight. Lying against the walls, the runs' pin lines are 50.4 mm apart, so a U turning
// the chain round between them is a half circle of 25.2 mm (on the pin line), and its links turn
// 46.8 degrees each: 1.8 past where the STEP's links first touch (the overlap there is a few tenths
// of a mm^3, the stops pressed together; a half circle at 45 degrees needs 52.3 mm). No smooth U does
// better, since turning half round with nowhere bent tighter than r takes 2r across; the links could
// only make it at 45 as a polygon, and only at some phases of the links against each other (a fit
// of every split, SLSQP: about two fifths of each link's passage), lifting off the walls with
// sharp backward kinks at the rest. So the chain here is the half circle, its links pressed onto
// their stops round the U: on the robot, the stops' faces and the pins' play take it up.
export const TRACK8793 = {
  rIn: 5.25 * 0.0254, rOut: 8.0 * 0.0254, floor: 0.3177, wallTop: 0.343,
  stop: -93.6 * DEG,            // the wedge's step
  ringBracket: 0.032,           // the fixed end's bracket: from the step to its pin, along the wall
  plate: 0.348,                 // the shooter's base plate's underside
  corner: [-0.150, -0.114],     // its back-right corner ([x, w] about the axis, the turret at 0)
  mountA: -128 * DEG,           // the moving end's pin (on the inside run, under the corner), at yaw 0
  yaw: [-200 * DEG, 270 * DEG], // their soft limits
  innerMin: 13 * DEG,           // the least of the inside run, at the far limit (sets the length)
};

export class TrackChain {
  constructor(o = TRACK8793, link = LINK) {
    Object.assign(this, { o, link });
    const h = link.height, p = link.pitch;
    this.ro = o.rOut - h / 2; this.ri = o.rIn + h / 2;    // the runs' pin lines
    this.rho = (this.ro - this.ri) / 2;                   // the U's (on the pin line)
    this.aF = o.stop + o.ringBracket / this.ro;            // the fixed end's pin
    // the length: the inside run innerMin long at the far limit
    const aM = o.mountA + o.yaw[1], aU = aM + o.innerMin;
    this.n = Math.round((this.ro * (aU - this.aF) + Math.PI * this.rho + this.ri * o.innerMin) / p);
    this.path = new Path();
    this.pins = new Float64Array(2 * (this.n + 1));
    this.aU = null;
  }
  // where the moving end's pin is at yaw psi
  mountAngle(psi) { return this.o.mountA + psi; }
  // the U's middle (rad round the track) for a total arc length L: outside from aF to aU, the U, inside back to aM
  _aU(L, aM) { return (L - Math.PI * this.rho + this.ro * this.aF + this.ri * aM) / (this.ro + this.ri); }
  _build(aU, aM) {
    const P = this.path.reset(), rc = (this.ro + this.ri) / 2, cx = rc * Math.cos(aU), cw = rc * Math.sin(aU);
    P.arc(0, 0, this.ro, this.aF, aU);
    // the U: a half circle about the channel's middle line, out of the outside run into the inside one
    P.arc(cx, cw, this.rho, aU, aU + Math.PI);
    P.arc(0, 0, this.ri, aU, aM);
    return P;
  }
  // the pins for yaw psi: links of the chain's pitch from the fixed end along the outside run's pin
  // line, round the U, back along the inside run's, the U placed (secant on its arc length, the
  // links' chords a little shorter than the arcs) so that the last pin lands on the moving end
  solve(psi) {
    const p = this.link.pitch, n = this.n, aM = this.mountAngle(psi);
    let L = n * p, aU = this._aU(L, aM);
    for (let it = 0; it < 5; it++) {
      const P = this._build(aU, aM), s = chordStep(P, n, p, this.pins), res = P.length - s;
      if (Math.abs(res) < 1e-7) break;
      L -= res; aU = this._aU(L, aM);
    }
    this.aU = aU;
    return this.pins;
  }
  // how many links lie in the inside run at yaw psi (by arc length, for the load)
  insideLinks(psi) {
    const aM = this.mountAngle(psi), aU = this._aU(this.n * this.link.pitch, aM);
    return Math.max(0, (this.ri * (aU - aM)) / this.link.pitch);
  }
}

// ---------------------------------------------------------------- 4414: a spool with a tensioner
// Their binder: "A 55.7:1 reduction on a single Kraken X44, with a flex-wheel cable-chain
// tensioner", "2 back-to-back 8lb constant force springs retract a 1.625in flex wheel to tension the
// cable chain", "2 stacks of 4 thrust bushings constrain the tensioner to only linear motion", "44t
// idler gear machined down to 0.275in thick for cable chain clearance". The user: the chain comes in
// from the robot's left, goes round the constant-force-spring wheel, back the other way round a
// compliant wheel just behind the turret to the left, and on to the shooter, round whose corners
// (printed guides just below the turret's bearing) it spools. Their CAD isn't public: where things
// are is read off the binder's renders (Turret, Turret PT, Shooter isolated: the tensioner at the
// strip's right end, the compliant wheel through a hole by the turret's left neck), the rest is
// estimated. Here, under the top plate's back strip:
//   - the fixed end on the left, the chain running right along the strip (run A),
//   - round the tensioner's 1.625in flex wheel (it slides along the strip on its bushings; the
//     springs pull it right), back left (run B, in front of A),
//   - round the 3in compliant wheel behind the turret on the left, the other way,
//   - across to the shooter, and counterclockwise round the rounded box of guides at its corners to
//     where it's fixed on it, by its left-front corner.
// As the turret turns left (counterclockwise) it winds chain on and pulls the tensioner left; turned
// right, it pays chain out and the springs take it up. Past one turn on the guides it winds over
// itself (the turret's 400 degrees of travel need more than the one layer's ~340).
export const SPOOL4414 = {
  axis: [-0.02, 0],               // the turret's (robot frame, [x, w])
  y: 0.5005,                      // the chain's middle (on its side, 39.2 mm: just under the 0.525 m top plate)
  entry: [-0.305, 0.37],          // the fixed end's pin, the chain heading right from it
  tensionR: 1.625 / 2 * 0.0254,   // the flex wheel
  tension: [-0.385, 0.215],       // the tensioner's travel along w (its wheel's center)
  wheel: { at: 0.20, r: 1.5 * 0.0254 }, // the compliant wheel (w; its x: just in front of run B)
  // the guides round the shooter's corners: a box (turret frame, x along the shot) round its
  // flywheel and plates, corner radius r
  box: { x: [-0.105, 0.135], z: [-0.105, 0.105], r: 0.03 },
  spring: 2 * 8 * 4.448,          // N: the two 8 lb constant-force springs, together
  yaw: [-200 * DEG, 200 * DEG],
  wrapMin: 0.05,                  // m: the least wound on the guides, at the right-hand limit (sets where its end is fixed)
};

export class SpoolChain {
  constructor(o = SPOOL4414, link = LINK_TALL) {
    Object.assign(this, { o, link });
    const h = link.height;
    this.rhoT = o.tensionR + h / 2;            // round the tensioner (pin line)
    this.xA = o.entry[0]; this.xB = this.xA + 2 * this.rhoT;
    this.rhoC = o.wheel.r + h / 2;
    this.C = [this.xB + this.rhoC, o.wheel.at];
    // the guides' corners, and the chain's pin line round them (its first layer), in the turret's
    // frame ([x, w]), counterclockwise from the front-right corner
    const b = o.box, rc = b.r + h / 2;
    const cx = [b.x[0] + b.r, b.x[1] - b.r], cw = [-(b.z[1] - b.r), -(b.z[0] + b.r)];
    this.corners = [[cx[1], cw[0]], [cx[1], cw[1]], [cx[0], cw[1]], [cx[0], cw[0]]]; // front-right, front-left, back-left, back-right
    this.rc = rc;
    // sampled by arc length u: [x, w, outward normal angle (rad, growing round the loop), u]
    const loop = [];
    let u = 0;
    const push = (x, w, a) => { const q = loop[loop.length - 1]; if (q) u += Math.hypot(x - q[0], w - q[1]); loop.push([x, w, a, u]); };
    for (let k = 0; k <= 4; k++) {
      const [qx, qw] = this.corners[k % 4], a0 = -Math.PI / 2 + (k * Math.PI) / 2;
      if (k === 4) { push(qx + rc * Math.cos(a0), qw + rc * Math.sin(a0), a0); break; }
      const f0 = loop.length ? loop[loop.length - 1] : null;
      // (the flat from the last corner: in 2 mm steps)
      if (f0) { const x1 = qx + rc * Math.cos(a0), w1 = qw + rc * Math.sin(a0), l = Math.hypot(x1 - f0[0], w1 - f0[1]), m = Math.ceil(l / 0.002); for (let i = 1; i < m; i++) push(f0[0] + ((x1 - f0[0]) * i) / m, f0[1] + ((w1 - f0[1]) * i) / m, a0); }
      for (let i = 0; i <= 30; i++) { const a = a0 + ((Math.PI / 2) * i) / 30; push(qx + rc * Math.cos(a), qw + rc * Math.sin(a), a); }
    }
    // (closing flat, back to the start)
    { const f0 = loop[loop.length - 1], [x1, w1] = [loop[0][0], loop[0][1]], l = Math.hypot(x1 - f0[0], w1 - f0[1]), m = Math.ceil(l / 0.002); for (let i = 1; i <= m; i++) push(f0[0] + ((x1 - f0[0]) * i) / m, f0[1] + ((w1 - f0[1]) * i) / m, i === m ? 1.5 * Math.PI : f0[2]); }
    this.loop = loop; this.perim = u;
    this.path = new Path();
    // Where its end is fixed on the guides: wrapMin (of chain, m) on round from where it meets them
    // at the right-hand limit. The length: the tensioner at its right stop there.
    this.uAtt = 0;
    const u0 = this._meet(o.yaw[0], (q) => -q); // (uAtt 0: the loop itself, u forward)
    this.uAtt = u0 + o.wrapMin;
    this.n = Math.floor(this._build(o.yaw[0], o.tension[0], o.wrapMin).length / link.pitch);
    this.pins = new Float64Array(2 * (this.n + 1));
  }
  // the loop at arc length u (round and round): [x, w, outward normal angle]
  _at(u) {
    const P = this.perim, turns = Math.floor(u / P), uu = u - turns * P, L = this.loop;
    let lo = 0, hi = L.length - 1;
    while (hi - lo > 1) { const m = (lo + hi) >> 1; if (L[m][3] <= uu) lo = m; else hi = m; }
    const a = L[lo], b = L[hi], t = (uu - a[3]) / Math.max(1e-12, b[3] - a[3]);
    return [a[0] + (b[0] - a[0]) * t, a[1] + (b[1] - a[1]) * t, a[2] + (b[2] - a[2]) * t + turns * 2 * Math.PI];
  }
  // the chain wound on the guides, v (m of chain) back from its end, the turret at psi: [x, w] in the
  // robot's frame and its heading there (counterclockwise round the box). Past a turn it's the next
  // layer out: it climbs onto the first over the 55 mm before it comes round to the end's bracket
  // (and the end link behind it), so it passes over them. The heading is the climbing line's own.
  _wound(v, psi) {
    const pt = (vv) => {
      const [x, w, nrm] = this._at(this.uAtt - vv);
      const e = Math.min(1, Math.max(0, (vv - this.perim + 0.095) / 0.055)), d = this.link.height * e * e * (3 - 2 * e);
      return [x + d * Math.cos(nrm), w + d * Math.sin(nrm), nrm];
    };
    const [px, pw, nrm] = pt(v), a = pt(v + 0.001), b = pt(v - 0.001);
    const c = Math.cos(psi), s = Math.sin(psi);
    return [this.o.axis[0] + c * px - s * pw, this.o.axis[1] + s * px + c * pw, Math.atan2(b[1] - a[1], b[0] - a[0]) + psi, this._at(this.uAtt)[2] - nrm];
  }
  // How much is wound on (m back from the end) where the chain from the compliant wheel meets the
  // guides, turret at psi: a line along the chain's heading there just touches the wheel, the wheel
  // on its right and behind. (wound: v -> the point; for the first layer by itself, the loop's own
  // u.) The root nearest `near`.
  _meet(psi, map, near = null) {
    const [Cx, Cw] = this.C, R = this.rhoC;
    const pt = (v) => this._wound(map(v), psi);
    const f = (v) => { const [x, w, hd] = pt(v), tx = Math.cos(hd), tw = Math.sin(hd); return [tx * (Cw - w) - tw * (Cx - x) + R, tx * (Cx - x) + tw * (Cw - w)]; };
    const span = near === null ? [0, this.perim] : [Math.max(0, near - 0.35), near + 0.35];
    const N = near === null ? 240 : 90;
    // (a root is the place only if nothing of the guides (or what's wound on them) behind it, toward
    // the wheel, pokes through the line: where the chain climbs over its own end there are two)
    const clear = (v) => {
      const [x, w, hd] = pt(v), tx = Math.cos(hd), tw = Math.sin(hd);
      for (let k = 1; k <= 30; k++) { const [px, pw] = pt(v + k * 0.005); if (tx * (pw - w) - tw * (px - x) < -2e-4) return false; }
      return true;
    };
    const roots = [];
    let pv = span[0], [pf] = f(pv);
    for (let i = 1; i <= N; i++) {
      const v = span[0] + ((span[1] - span[0]) * i) / N, [fv, ahead] = f(v);
      if ((pf <= 0) !== (fv <= 0) && ahead < 0) {
        let lo = pv, hi = v, flo = pf;
        for (let j = 0; j < 40; j++) { const m = (lo + hi) / 2, [fm] = f(m); if ((fm <= 0) === (flo <= 0)) { lo = m; flo = fm; } else hi = m; }
        roots.push((lo + hi) / 2);
      }
      pv = v; pf = fv;
    }
    const ok = roots.filter(clear), pool = ok.length ? ok : roots;
    if (!pool.length) return near ?? 0;
    return pool.reduce((a, b) => (near === null ? (b < a ? b : a) : Math.abs(b - near) < Math.abs(a - near) ? b : a));
  }
  // the whole middle line, turret at psi, the tensioner's wheel at wT, v wound on
  _build(psi, wT, v) {
    const P = this.path.reset(), [Cx, Cw] = this.C, rT = this.rhoT, rC = this.rhoC;
    P.push(this.xA, this.o.entry[1]);
    P.line(this.xA, wT);
    P.arc(this.xA + rT, wT, rT, Math.PI, 2 * Math.PI);
    P.line(this.xB, Cw);
    // round the compliant wheel, clockwise, to where the line to the guides leaves it (the wheel on
    // the chain's right: the line's left normal from its center)
    const [qx, qw, hd] = this._wound(v, psi), tx = Math.cos(hd), tw = Math.sin(hd);
    let aOut = Math.atan2(tx, -tw);
    while (aOut > Math.PI) aOut -= 2 * Math.PI;
    while (aOut <= -Math.PI) aOut += 2 * Math.PI;
    P.arc(Cx, Cw, rC, Math.PI, aOut);
    P.line(qx, qw);
    // round the guides to the end
    const k = Math.max(2, Math.ceil(v / 0.002));
    for (let i = 1; i <= k; i++) { const [x, w] = this._wound(v * (1 - i / k), psi); P.push(x, w); }
    return P;
  }
  // how much is wound on at yaw psi, roughly (for where to look)
  _nearV(psi) { return this.o.wrapMin + ((psi - this.o.yaw[0]) * this.perim) / (2 * Math.PI); }
  // the pins for yaw psi; also this.wT (the tensioner's wheel), this.wound (m on the guides), this.arm
  // (the chain's pull's lever arm about the turret's axis, m), this.ok (the tensioner within its travel)
  solve(psi) {
    const p = this.link.pitch, n = this.n;
    const v = this._meet(psi, (x) => x, this._nearV(psi));
    this.wound = v;
    let wT = this.wT ?? this.o.tension[0];
    for (let it = 0; it < 4; it++) {
      const P = this._build(psi, wT, v);
      const s = chordStep(P, n, p, this.pins);
      const res = P.length - s; // (the last pin short of the end: the path's that much too long)
      if (Math.abs(res) < 1e-6) break;

      wT += res / 2;
    }
    this.wT = wT;
    this.ok = wT >= this.o.tension[0] - 2e-3 && wT <= this.o.tension[1] + 2e-3;
    const [qx, qw, hd, turn] = this._wound(v, psi);
    this.turn = turn;
    this.arm = (qx - this.o.axis[0]) * Math.sin(hd) - (qw - this.o.axis[1]) * Math.cos(hd);
    return this.pins;
  }
}

// ---------------------------------------------------------------- what the chains do to the turret
// The chain's resistance, as the turret's model wants it: a steady torque (N·m, + counterclockwise:
// a spring's), a Coulomb friction (N·m, against the way it turns, or holding it still) and a viscous
// part (N·m·s/rad). Estimates (their chains' insides aren't known); the reasoning:
//   - the wires inside: a bundle bends like a beam, EI. Stranded silicone wire is soft: 12 AWG
//     ~2e-4 N·m² (a 10 cm length sags ~2 cm under its own weight held out flat), the CAN pair and
//     the smaller ones less. 8793's (the flywheel's X60, the rack's motor, CAN, the Limelight's):
//     1.5e-3; 4414's, more to carry (four X44s on the flywheel, the hood's X44, CAN, a Limelight):
//     3e-3. At a 25 mm bend that's 0.06 N·m (a few N on the walls of 8793's U, the scale a loaded
//     15x30 chain takes to bend round a small radius) and 0.12 N·m.
//   - stranded wire and the plastic links lose about a quarter of the work that bends them
//     (hysteresis, the strands sliding on each other): eta 0.25 of EI/r through each joint's turn,
//     every time a link goes into a bend and comes out of it.
//   - the pins: plastic on plastic, mu 0.3 at the pin's 3.5 mm radius under the chain's tension.
//   - sliding: a link (4.7 g) and its share of the wires (~1.2-1.5 g) on a printed floor, mu 0.3; a
//     run pressed on a wall by the U's stiffness, mu 0.3.
export const CHAIN_LOSS = { eta: 0.25, muPin: 0.3, rPin: 0.0035, muSlide: 0.3, g: 9.81 };

// 8793: no spring. The inside run turns with the turret over the floor and the U's last links
// slide on the inner wall; links go round the U (r_o r_i / ((r_o + r_i) p) of them per radian).
export function chainLoad8793(chain, psi, { EI = 1.5e-3, wireMass = 0.0012 } = {}) {
  const c = chain, p = c.link.pitch, L = CHAIN_LOSS;
  const perRad = (c.ro * c.ri) / ((c.ro + c.ri) * p);
  const jU = Math.min(c.link.stop, 2 * Math.asin(p / (2 * c.rho)));      // a joint round the U
  const jO = 2 * Math.asin(p / (2 * c.ro)), jI = 2 * Math.asin(p / (2 * c.ri)); // round the runs (the inside one backward)
  // each link through the U: from the outside run's bend up to the U's and down through straight to
  // the inside run's backward bend
  const swing = (jU - jO) + (jU + jI);
  const Mu = EI / c.rho, hyst = L.eta * Mu * swing * perRad;
  const Fwall = Mu / c.rho;                     // the U's push on each wall
  const pin = L.muPin * L.rPin * Fwall * swing * perRad;
  const slideWall = L.muSlide * Fwall * c.ri;
  const slideFloor = L.muSlide * c.insideLinks(psi) * (c.link.mass + wireMass) * L.g * c.ri;
  // the spring part: links moving from the outside run (bent jO) to the inside (jI the other way)
  const k = EI / p, spring = -perRad * 0.5 * k * (jI * jI - jO * jO);
  return { torque: spring, friction: hyst + pin + slideWall + slideFloor, viscous: 0, parts: { hyst, pin, slideWall, slideFloor, spring } };
}

// 4414: the springs pull the tensioner with `spring` N, so the chain (two runs round it) pulls with
// half that: on the turret at the arm of the line it leaves the box along, unwinding it (right).
// The tensioner's bushings (~4 N, estimate) add to that winding on and take away unwinding; the
// pins under that tension, and the wires' hysteresis, at the three bends a length of chain goes
// through as it's wound (round the tensioner at half the speed, the compliant wheel the other way,
// onto the guides' corners).
export function chainLoad4414(chain, psi, { EI = 3e-3, bushing = 4 } = {}) {
  const c = chain, p = c.link.pitch, L = CHAIN_LOSS;
  const arm = Math.max(0.02, armAt(c, psi)), T = c.o.spring / 2;
  const jT = 2 * Math.asin(p / (2 * c.rhoT)), jC = 2 * Math.asin(p / (2 * c.rhoC));
  // per m wound: links round the tensioner (it moves half as far: 0.5/p of them) and round the wheel
  // (1/p), each bent in and out again, and the box's turn (2 pi per loop) bent once
  const perM = { T: 0.5 / p, C: 1 / p };
  const turnPerM = 2 * Math.PI / c.perim;
  const hyst = L.eta * EI * (perM.T * 2 * jT / c.rhoT + perM.C * 2 * jC / c.rhoC + turnPerM / c.rc);
  const pin = L.muPin * L.rPin * T * (perM.T * 2 * jT + perM.C * 2 * jC + turnPerM);
  // (J per m wound: a force along the chain; at the arm, the torque)
  return { torque: -T * arm, friction: (hyst + pin + bushing / 2) * arm, viscous: 0, parts: { hyst: hyst * arm, pin: pin * arm, bushing: (bushing / 2) * arm, spring: -T * arm, arm } };
}

// the arm (m) the chain's pull has about the turret's axis, by yaw: tabulated once a degree apart
// from a chain of its own (solving it each step is a millisecond)
const armTables = new Map();
function armAt(chain, psi) {
  let t = armTables.get(chain.o);
  if (!t) {
    const c = new SpoolChain(chain.o, chain.link), [y0, y1] = chain.o.yaw, n = Math.round((y1 - y0) / DEG);
    t = { y0, n, a: Float64Array.from({ length: n + 1 }, (_, i) => { c.solve(y0 + i * DEG); return c.arm; }) };
    armTables.set(chain.o, t);
  }
  const f = Math.min(t.n, Math.max(0, (psi - t.y0) / DEG)), i = Math.min(t.n - 1, Math.floor(f));
  return t.a[i] + (t.a[i + 1] - t.a[i]) * (f - i);
}

// ---------------------------------------------------------------- 1690: a spool with a tensioner
// The user: 1690's chain "works very similarly to how 4414's does, with a constant force spring".
// Their CAD (view-only: its renders, js/mech/r1690.js) has the spring (the BOM's "Constant Force
// Spring (6 lbs, 40in Travel, 3/4in Width)") and printed guides on the curved guard round the
// turret's front, but not the chain or where its runs go. So the routing is 4414's, laid out in the
// room 1690 has (estimates throughout): the fixed end at the back left (by the frame's corner behind
// the turret), the chain running right along the back of the frame round the spring's 1.5in wheel
// (it slides along the back; the spring pulls it right), back left round a 1.5in wheel behind the
// turret's right side, and on round the guides of the turret's rotating base (a rounded box round
// its plates) to its end there. The turret's +-200 deg of travel winds on or pays out ~0.9 m of
// chain; the spring's wheel takes up half of that (0.45 m of its travel; the spring's is 40in). Its
// chain: WCP's 15x30 (small: "the tiny form factor of the echain", their thread).
export const SPOOL1690 = {
  axis: [-0.14, 0.20],            // the turret's (robot frame, [x, w])
  y: 0.395,                       // the chain's middle (on its side, just over the turret's deck)
  entry: [-0.30, 0.355],          // the fixed end's pin, the chain heading right from it
  tensionR: 0.75 * 0.0254,        // the spring's wheel (1.5in)
  tension: [-0.32, 0.175],        // its travel along w
  wheel: { at: 0.185, r: 0.75 * 0.0254 }, // the wheel the chain turns back round (w; its x: just in front of run B)
  box: { x: [-0.12, 0.13], z: [-0.095, 0.095], r: 0.03 },
  spring: 6 * 4.448,              // N: the one 6 lb constant-force spring
  yaw: [-200 * DEG, 200 * DEG],
  wrapMin: 0.05,
};
// 1690: as 4414's (chainLoad4414: the spring's pull at the arm the chain leaves the turret's guides
// along, the pins and the wires' hysteresis at its bends, the spring wheel's slide), with its own
// spring, its wires (two X60s on the shooter on 14 AWG, the hood's X44, CAN, the turret's camera: EI
// 3e-3, as 4414's) and the slide's friction (2 N, estimate)
export function chainLoad1690(chain, psi, { EI = 3e-3, bushing = 2 } = {}) { return chainLoad4414(chain, psi, { EI, bushing }); }
