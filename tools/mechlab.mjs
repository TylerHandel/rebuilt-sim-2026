// Mechanism lab: physical mechanisms (js/mech/mechanism.js) on their own, no field, no robot,
// measured against what they should do. Run:
//   node --import ./tools/node-env.mjs tools/mechlab.mjs [test ...]
// Tests: foam (a FUEL resting on a roller squashes as the measured foam curve says), pinch (two
// motor-driven flex wheels grab a FUEL and fling it: exit speed vs their surface speed, how far
// the motors slow), funnel (FUEL rolling two abreast into a one-wide neck, with and without
// vertical-axis omni wheels at the neck: do they jam?).
import { RAPIER } from '../js/physics.js';
import { FUEL, IN } from '../js/constants.js';
import { Mechanism, Shaft, Wheel, Motor, foam } from '../js/mech/mechanism.js';

await RAPIER.init();
const R = FUEL.radius, DT = 1 / 120;

function world(g = -9.81) {
  const w = new RAPIER.World({ x: 0, y: g, z: 0 });
  w.timestep = DT;
  w.integrationParameters.numSolverIterations = 6;
  return w;
}
function ball(w, x, y, z, v = [0, 0, 0]) {
  const b = w.createRigidBody(RAPIER.RigidBodyDesc.dynamic().setTranslation(x, y, z).setLinvel(...v).setCcdEnabled(true).setAngularDamping(FUEL.angularDamping));
  w.createCollider(RAPIER.ColliderDesc.ball(R).setMass(FUEL.mass).setFriction(FUEL.friction).setRestitution(FUEL.restitution), b);
  return b;
}
const fixed = (w) => w.createRigidBody(RAPIER.RigidBodyDesc.fixed());
const run = (w, mech, balls, sec, each) => {
  const n = Math.round(sec / DT);
  for (let i = 0; i < n; i++) { if (mech) mech.step(DT, balls); w.step(); if (each) each(i * DT); }
};

const tests = {
  // a FUEL lying on a fixed 2in roller: its weight squashes it by x where F(x) = m g
  foam() {
    const w = world();
    const shaft = new Shaft({ inertia: 1e3 }); // held still
    const mech = new Mechanism({ substeps: 6, shafts: [shaft], wheels: [new Wheel({ shaft, center: [0, 0, 0], axis: [0, 0, 1], radius: 1 * IN, width: 0.3 })] });
    const b = ball(w, 0, 1 * IN + R + 0.02, 0);
    let rocked = 0;
    run(w, mech, [b], 3, () => { rocked = Math.max(rocked, Math.abs(b.translation().x)); });
    const sq = 1 * IN + R - b.translation().y;
    const mg = FUEL.mass * 9.81, k0 = FUEL.springRate0, xc = FUEL.stiffen;
    const want = xc * Math.log(1 + mg / (k0 * xc));
    console.log(`foam: resting squeeze ${(sq * 1000).toFixed(2)} mm (the foam curve: ${(want * 1000).toFixed(2)} mm); force check ${foam(sq).F.toFixed(2)} N vs weight ${mg.toFixed(2)} N; slid off sideways ${(rocked * 1000).toFixed(1)} mm`);
  },

  // two 2in 30A flex wheels a FUEL's width less 1/2in apart, each on a Kraken X44 through 3:1,
  // turning so their faces at the gap run +x; a FUEL rolls in at 1 m/s
  pinch() {
    const w = world(0);
    const sq = 0.5 * IN, rw = 1 * IN, off = rw + R - sq / 2;
    const mk = () => new Shaft({ motor: new Motor('krakenX44', { ratio: 3, statorLimit: 40 }), inertia: 4e-5 });
    const top = mk(), bot = mk();
    const flex = { radius: rw, width: 1 * IN, mu: 1.1, tread: 6000 };
    const mech = new Mechanism({ substeps: 6, gravity: [0, 0, 0], shafts: [top, bot], wheels: [
      new Wheel({ shaft: top, center: [0, off, 0], axis: [0, 0, 1], ...flex }),
      new Wheel({ shaft: bot, center: [0, -off, 0], axis: [0, 0, 1], ...flex }),
    ] });
    top.cmd = 1; bot.cmd = -1;
    run(w, mech, [], 1.0); // spin up
    const rim = top.omega * rw;
    const b = ball(w, -0.2, 0, 0, [1, 0, 0]);
    let minW = top.omega, maxI = 0, t0 = null, t1 = null, peakF = 0;
    run(w, mech, [b], 0.6, (t) => {
      minW = Math.min(minW, top.omega);
      maxI = Math.max(maxI, Math.abs(top.motor.current));
      const touching = mech.contacts.length > 0;
      for (const c of mech.contacts) peakF = Math.max(peakF, c.force);
      if (touching && t0 === null) t0 = t;
      if (!touching && t0 !== null && t1 === null) t1 = t;
    });
    const v = b.linvel(), sp = Math.hypot(v.x, v.y, v.z), spin = b.angvel();
    console.log(`pinch: wheel surface ${rim.toFixed(2)} m/s (free ${(top.motor.freeSpeed * rw).toFixed(2)}); FUEL out at ${sp.toFixed(2)} m/s (${((100 * sp) / rim).toFixed(0)}% of the surface), spin ${Math.hypot(spin.x, spin.y, spin.z).toFixed(1)} rad/s; in the wheels ${t0 !== null && t1 !== null ? ((t1 - t0) * 1000).toFixed(0) + ' ms' : '?'}, squeeze force up to ${peakF.toFixed(0)} N; wheels slowed to ${((100 * minW) / (rim / rw)).toFixed(0)}%, motor current up to ${maxI.toFixed(0)} A`);
  },

  // a sloped channel (12 deg down toward +x) 0.36 m wide that narrows to a one-FUEL neck at x=0;
  // FUEL two abreast roll down into it. With and without a vertical-axis 4in omni wheel at each
  // corner of the neck, turning so its face runs downstream.
  funnel() {
    let seed = 1;
    const rnd = () => ((seed = (seed * 16807) % 2147483647) / 2147483647 - 0.5);
    for (const variant of ['plain neck', 'omni 4in + 4in', 'omni 3in + 5in (8793)']) {
      let ok = 0, times = [];
      for (let trial = 0; trial < 8; trial++) {
        seed = trial + 1;
        const r = funnelRun(variant, rnd);
        if (r.through === 8) { ok++; times.push(r.last); }
      }
      times.sort((a, b) => a - b);
      console.log(`funnel, ${variant.padEnd(22)}: all 8 FUEL through in ${ok}/8 trials${times.length ? `, median ${times[times.length >> 1].toFixed(2)} s` : ''}`);
    }
  },

};
// one run of the funnel: FUEL two abreast (jittered by rnd) roll down a sloped channel 0.36 m
// wide into a one-FUEL neck. variant: no wheels, or a vertical-axis omni wheel at each funnel wall
// where two FUEL abreast wedge (the width is two FUEL there), faces running downstream
function funnelRun(variant, rnd) {
  const w = world();
  const fb = fixed(w);
  const tilt = (12 * Math.PI) / 180, q = { x: 0, y: 0, z: Math.sin(-tilt / 2), w: Math.cos(-tilt / 2) };
  const c = Math.cos(tilt), s = Math.sin(tilt);
  const box = (cx, cy, cz, hx, hy, hz, rotY = 0) => {
    const p = { x: cx * c + cy * s, y: -cx * s + cy * c, z: cz };
    let rq = q;
    if (rotY) { const a = rotY / 2; rq = mulq(q, { x: 0, y: Math.sin(a), z: 0, w: Math.cos(a) }); }
    w.createCollider(RAPIER.ColliderDesc.cuboid(hx, hy, hz).setTranslation(p.x, p.y, p.z).setRotation(rq).setFriction(0.3).setRestitution(0.2), fb);
  };
  box(0, -0.02, 0, 1.2, 0.02, 0.4); // floor
  const neck = R + 0.006, wide = 0.18;
  const fl = Math.hypot(0.34, wide - neck), ang = Math.atan2(wide - neck, 0.34);
  for (const sgn of [1, -1]) {
    box(-0.23, 0.1, sgn * ((wide + neck) / 2 + 0.01), fl / 2, 0.1, 0.01, sgn * ang);
    box(0.3, 0.1, sgn * (neck + 0.01), 0.36, 0.1, 0.01);
    box(-0.75, 0.1, sgn * (wide + 0.01), 0.35, 0.1, 0.01);
  }
  let mech = null;
  if (variant !== 'plain neck') {
    const [rl, rr] = variant.startsWith('omni 4in') ? [2 * IN, 2 * IN] : [1.5 * IN, 2.5 * IN];
    const mk = () => new Shaft({ motor: new Motor('krakenX44', { ratio: 9, statorLimit: 30 }), inertia: 8e-5 });
    const l = mk(), r = mk();
    const n = [s, c, 0];
    const at = (x, z) => [x * c + R * s, -x * s + R * c, z];
    const xw = -0.27, half = neck + ((wide - neck) * (-0.06 - xw)) / 0.34, reach = 0.025;
    const wl = new Wheel({ shaft: l, center: at(xw, half + rl - reach), axis: n, radius: rl, width: 1 * IN, kind: 'omni', mu: 1.0 });
    const wr = new Wheel({ shaft: r, center: at(xw, -(half + rr - reach)), axis: n, radius: rr, width: 1 * IN, kind: 'omni', mu: 1.0 });
    mech = new Mechanism({ substeps: 6, shafts: [l, r], wheels: [wl, wr] });
    l.cmd = -1; r.cmd = 1; // faces at the funnel run downstream (+x)
  }
  const balls = [];
  const place = (x, z) => { const y = R + 0.002; balls.push(ball(w, x * c + y * s, -x * s + y * c, z)); };
  for (let i = 0; i < 4; i++) { place(-0.62 + i * 0.16 + 0.02 * rnd(), 0.08 + 0.01 * rnd()); place(-0.62 + i * 0.16 + 0.02 * rnd(), -0.08 + 0.01 * rnd()); }
  const through = new Set();
  let last = 0;
  run(w, mech, balls, 6, (t) => { for (const b of balls) if (!through.has(b) && b.translation().x > 0.15) { through.add(b); last = t; } });
  w.free();
  return { through: through.size, last };
}
function mulq(a, b) {
  return { x: a.w * b.x + a.x * b.w + a.y * b.z - a.z * b.y, y: a.w * b.y - a.x * b.z + a.y * b.w + a.z * b.x, z: a.w * b.z + a.x * b.y - a.y * b.x + a.z * b.w, w: a.w * b.w - a.x * b.x - a.y * b.y - a.z * b.z };
}

const pick = process.argv.slice(2);
for (const [k, f] of Object.entries(tests)) if (!pick.length || pick.includes(k)) f();
