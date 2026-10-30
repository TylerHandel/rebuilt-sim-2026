// Mechanism lab: physical mechanisms (js/mech/mechanism.js) on their own, no field, no robot,
// measured against what they should do. Run:
//   node --import ./tools/node-env.mjs tools/mechlab.mjs [test ...]
// Tests: foam (a FUEL resting on a roller squashes as the measured foam curve says), pinch (two
// motor-driven flex wheels grab a FUEL and fling it: exit speed vs their surface speed, how far
// the motors slow), funnel (FUEL rolling two abreast into a one-wide neck, with and without
// vertical-axis omni wheels at the neck: do they jam?), and 8793's (js/mech/r8793.js): r8793
// (the rollers' speeds), conveyor (intake and conveyor, the X44s at the vertical rollers powered and
// unpowered: the rollers locked to the X60 either way), shooter, calibrate (the shooter's grip) and
// robot (carpet to shot).
// Env: HZ (steps/s, 120), SUB (contact sub-steps for the quick contacts, 8), SEEDS (12), TRACE
// (a FUEL's index to follow), STUCK (where the ones left over ended up), IDX_RATIO
// (the turret indexer's gearing: direct drive), TOGETHER=0 (FUEL on FUEL rigid, the world's,
// instead of foam in the mechanism).
import { RAPIER } from '../js/physics.js';
import { FUEL, IN, GROUP, groups } from '../js/constants.js';
import { Mechanism, Shaft, Wheel, Motor, foam } from '../js/mech/mechanism.js';
import { MU, COMPLIANT_TREAD, NECK, CONVEYOR, HORIZONTAL, build8793, intake8793, conveyorWheels, armWheels, shooterWheels, shooterSmallAt, flywheelDuty, codeExitSpeed, SHOOTER, INDEXER, DUTY_INDEXER, indexerWheels, pathSurfaces, placeArm, armState, servoArm, turnArm, ARM } from '../js/mech/r8793.js';
if (process.env.IDX_RATIO) INDEXER.ratio = Number(process.env.IDX_RATIO); // (to try other indexer gearing)
if (process.env.C1PUSH) CONVEYOR.c1Push = Number(process.env.C1PUSH);
import PATH_MESH from '../js/mech/path8793.js';
import { JAM } from '../js/mech/phys8793.js';
import { BUMPER_T, ROBOTS } from '../js/robotConfigs.js';

await RAPIER.init();
const R = FUEL.radius, DT = 1 / Number(process.env.HZ ?? 120), SUB = Number(process.env.SUB ?? 8), SEEDS = Number(process.env.SEEDS ?? 12);

function world(g = -9.81) {
  const w = new RAPIER.World({ x: 0, y: g, z: 0 });
  w.timestep = DT;
  w.integrationParameters.numSolverIterations = 6;
  return w;
}
// FUEL on FUEL: the mechanism's to work out (foam, with friction), as in the game; TOGETHER=0: the
// world's (rigid)
const TOGETHER = process.env.TOGETHER !== '0';
function ball(w, x, y, z, v = [0, 0, 0]) {
  const b = w.createRigidBody(RAPIER.RigidBodyDesc.dynamic().setTranslation(x, y, z).setLinvel(...v).setCcdEnabled(true).setAngularDamping(FUEL.angularDamping));
  const c = RAPIER.ColliderDesc.ball(R).setMass(FUEL.mass).setFriction(FUEL.friction).setRestitution(FUEL.restitution);
  w.createCollider(c, b);
  return b;
}
const fixed = (w) => w.createRigidBody(RAPIER.RigidBodyDesc.fixed());
const run = (w, mech, balls, sec, each) => {
  const n = Math.round(sec / DT);
  for (let i = 0; i < n; i++) { if (mech) mech.step(DT, balls); w.step(); if (each) each(i * DT); }
};

const tests = {
  // 8793's intake and conveyor (js/mech/r8793.js): FUEL on the carpet taken in by the arm's
  // rollers, over the bumper, under the pivot's and the two conveyor shafts' mecanums, past the
  // vertical rollers at the taper, into one file toward the turret
  conveyor() {
    for (const x44 of [false, true]) {
      const runs = [];
      for (let seed = 1; seed <= SEEDS; seed++) runs.push(pathRun(seed, { x44 }));
      const med = (a) => { const b = [...a].sort((p, q) => p - q); return b[b.length >> 1]; };
      console.log(`8793 conveyor, X44s ${x44 ? 'powered  ' : 'unpowered'} (every roller locked to the X60): a median ${med(runs.map((r) => r.through))} of ${runs[0].n} through (${Math.min(...runs.map((r) => r.through))}..${Math.max(...runs.map((r) => r.through))}, ${SEEDS} runs), ` +
        `${med(runs.map((r) => r.rate)).toFixed(1)} FUEL/s; X60 up to ${Math.max(...runs.map((r) => r.peakI)).toFixed(0)} A, rollers slowed to ${med(runs.map((r) => r.minSpeed)).toFixed(0)}% (median), ${med(runs.map((r) => r.unjams))} unjams; the arm lifted up to ${med(runs.map((r) => r.maxLift)).toFixed(1)} deg (median)`);
    }
  },
  // the shooter's grip: with their old gearing (both shafts at the motor's speed) their code says
  // they measured ~0.205 m/s of exit speed per motor rotation/s; this is what sets SHOOTER.mu
  calibrate() {
    const keep = { ...SHOOTER };
    Object.assign(SHOOTER, { ratio: 1, smallPerLarge: 1 });
    for (const mu of [1.2, 1.4, 1.7, 2, 2.5]) {
      const out = [];
      for (const [e, target] of [[65, 35], [60, 40], [55, 47]]) {
        const w = world();
        const t = build8793();
        const mech = new Mechanism({ substeps: 6, sub: SUB, shafts: [t.large, t.small], wheels: shooterWheels(t, e, { mu: { roller: mu, stealth: mu } }) });
        for (let i = 0; i < 3 / DT; i++) { t.shooter.cmd = flywheelDuty(t, target); mech.step(DT, []); }
        const [sx, sy] = shooterSmallAt(e);
        const b = ball(w, -0.127, sy - Math.sqrt((SHOOTER.small.r + R) ** 2 - sx * sx) - 0.01, 0, [0, 1, 0]);
        let touched = false, v = null;
        for (let i = 0; i < 1 / DT && !v; i++) {
          t.shooter.cmd = flywheelDuty(t, target); mech.step(DT, [b]); w.step();
          if (mech.contacts.some((x) => x.ball === b)) touched = true; else if (touched) v = b.linvel();
        }
        const sp = v ? Math.hypot(v.x, v.y, v.z) : 0;
        out.push(`${e} deg, ${target} rps: ${(sp / target).toFixed(3)}`);
        w.free();
      }
      console.log(`shooter grip ${mu} (their old gearing), m/s per motor rotation/s: ${out.join(', ')}`);
    }
    Object.assign(SHOOTER, keep);
  },
  // the whole of 8793's ball path: carpet to shot (their code's duty cycles; the shooter held at
  // 60 motor rotations/s with the hood at 60 deg)
  robot() {
    const runs = [];
    for (let seed = Number(process.env.SEED0 ?? 1); seed < Number(process.env.SEED0 ?? 1) + SEEDS; seed++) runs.push(pathRun(seed, { turret: true, e: 60, target: 60, sec: Number(process.env.SEC ?? 6), nBalls: Number(process.env.NB ?? 12) }));
    const med = (a) => { const b = [...a].sort((p, q) => p - q); return b[b.length >> 1]; };
    const all = runs.flatMap((r) => r.shots);
    const avg = (a) => a.reduce((p, q) => p + q, 0) / (a.length || 1);
    console.log(`8793 carpet to shot (indexer ${INDEXER.ratio}:1): a median ${med(runs.map((r) => r.shots.length))} of ${runs[0].n} shot (${Math.min(...runs.map((r) => r.shots.length))}..${Math.max(...runs.map((r) => r.shots.length))}, ${SEEDS} runs), ` +
      `${med(runs.map((r) => r.shotRate)).toFixed(1)} FUEL/s; out at ${avg(all.map((x) => x.v)).toFixed(2)} m/s, ${avg(all.map((x) => x.el)).toFixed(1)} deg (${all.filter((x) => x.el < 30).length} knocked off instead); the flywheel sagged to ${med(runs.map((r) => r.minRps)).toFixed(0)} rps; ${med(runs.map((r) => r.unjams))} unjams (${runs.reduce((q, r) => q + r.unjams, 0)} in all), stalled ${runs.reduce((q, r) => q + r.stallT, 0).toFixed(1)} s in all, the arm lifted up to ${med(runs.map((r) => r.maxLift)).toFixed(1)} deg (median)`);
  },
  // where 8793 holds FUEL: intake 12 with the indexer stopped, then stop the rollers too and let
  // them settle (its preload goes where they end up)
  slots() {
    const r = pathRun(1, { turret: true, hold: true, sec: 6 });
    const pts = r.held.sort((a, b) => a[0] - b[0]);
    console.log(`8793 holds ${pts.length} with the indexer stopped: [${pts.map((p) => `[${p.map((v) => v.toFixed(3)).join(', ')}]`).join(', ')}]`);
  },
  // 8793's drive chains (js/mech/r8793.js): each roller's speed with the motors at full, free
  r8793() {
    const t = build8793();
    intake8793(t);
    for (let i = 0; i < 240; i++) t.horizontal.integrate(DT);
    const rpm = (w) => Math.abs(w * 60 / (2 * Math.PI)).toFixed(0).padStart(5);
    const row = (s) => `  ${s.name.padEnd(22)} ${rpm(s.omega)} rpm  ${Math.abs(s.omega * s.diameter / 2).toFixed(2).padStart(5)} m/s at the surface (${(s.diameter / IN).toFixed(2)}in)  ${s.what || ''}`;
    console.log("8793 intaking (their code's duty cycles), nothing in the rollers:");
    console.log(`  Kraken X60 ${rpm(t.horizontal.omega)} rpm`);
    for (const s of t.rollers) console.log(row(s));
    console.log(`  Kraken X44s ${rpm(t.horizontal.omega * t.x44[0].k)} rpm (locked to the X60 through the 90-degree gearboxes)`);
    for (const s of t.vshafts) console.log(row(s));
  },
  // 8793's shooter (js/mech/r8793.js) under their velocity loop: spin up, then FUEL come up the
  // turret's axis into it one after another (each arriving at 1 m/s where the 2in wheels first
  // touch it, as the turret indexer's column delivers them); each one's exit speed, elevation and
  // spin, against their aim solver's exit speed for the motor speed at the catch
  shooter() {
    for (const [e, target, bps] of [[68, 60, 4], [60, 75, 6], [52, 90, 8]]) {
      const w = world();
      const t = build8793();
      const wheels = shooterWheels(t, e, process.env.MU ? { mu: JSON.parse(process.env.MU) } : {});
      const mech = new Mechanism({ substeps: 6, sub: SUB, shafts: [t.large, t.small], wheels });
      const rps = () => t.shooter.omega / (2 * Math.PI);
      let T = 0, upAt = null;
      const control = () => { t.shooter.cmd = flywheelDuty(t, target); };
      for (; T < 3; T += DT) { control(); mech.step(DT, []); if (upAt === null && Math.abs(rps() - target) < 1) upAt = T; }
      const [sx, sy] = shooterSmallAt(e), x0 = -0.127;
      const yIn = sy - Math.sqrt((SHOOTER.small.r + R) ** 2 - (sx) ** 2) - 0.01;
      const shots = [];
      let live = [], minRps = rps(), I = 0;
      const n = 6;
      for (let k = 0; k < (n / bps + 1) / DT; k++, T += DT) {
        if (k % Math.round(1 / bps / DT) === 0 && shots.length + live.length < n) {
          const b = ball(w, x0, yIn, 0, [0, 1, 0]);
          live.push({ b, at: rps(), touched: false });
        }
        control();
        mech.step(DT, live.map((l) => l.b));
        w.step();
        minRps = Math.min(minRps, rps());
        I = Math.max(I, Math.abs(t.shooter.motor.current));
        for (const l of live) {
          const c = mech.contacts.some((ct) => ct.ball === l.b);
          if (c && !l.touched) { l.touched = true; l.at = rps(); }
          if (l.touched && !c) {
            const v = l.b.linvel(), wv = l.b.angvel();
            shots.push({ v: Math.hypot(v.x, v.y, v.z), el: Math.atan2(v.y, Math.hypot(v.x, v.z)) * 180 / Math.PI, spin: Math.hypot(wv.x, wv.y, wv.z), at: l.at, want: codeExitSpeed(l.at) });
            l.done = true;
            l.b.setTranslation({ x: 50 + shots.length, y: 5, z: 0 }, true);
          }
        }
        live = live.filter((l) => !l.done);
      }
      const avg = (f) => shots.reduce((a, x) => a + f(x), 0) / shots.length;
      console.log(`shooter, elevation ${e} deg, ${target} rps (their exit speed ${codeExitSpeed(target).toFixed(2)} m/s), fed ${bps}/s: up to speed in ${upAt?.toFixed(2)} s, motor up to ${I.toFixed(0)} A; ${shots.length} out:`);
      for (const x of shots) console.log(`  caught at ${x.at.toFixed(1)} rps: out at ${x.v.toFixed(2)} m/s (${((100 * x.v) / x.want).toFixed(0)}% of their ${x.want.toFixed(2)}), ${x.el.toFixed(1)} deg up, spin ${x.spin.toFixed(0)} rad/s`);
      console.log(`  mean ${avg((x) => x.v).toFixed(2)} m/s at ${avg((x) => x.el).toFixed(1)} deg; the wheel sagged to ${minRps.toFixed(1)} rps (${((100 * minRps) / target).toFixed(0)}%)`);
      w.free();
    }
  },
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
    const want = (xc * Math.log(1 + mg / (k0 * xc))) / 2; // (a dent on one side: half the plate test's squeeze)
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
    const mech = new Mechanism({ substeps: 6, sub: SUB, gravity: [0, 0, 0], shafts: [top, bot], wheels: [
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
    for (const variant of ['plain neck', 'omni 4in + 4in', 'omni + compliant (8793)']) {
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
    // (8793's: a 4in omni on the left and a 4in 45A compliant on the right, the right's face
    // slower by its 18t:28t belt)
    const is8793 = variant.includes('8793');
    const [rl, rr] = [2 * IN, 2 * IN];
    const mk = (ratio) => new Shaft({ motor: new Motor('krakenX44', { ratio, statorLimit: 30 }), inertia: 8e-5 });
    const l = mk(9), r = mk(is8793 ? (9 * NECK.right.belt[1]) / NECK.right.belt[0] : 9);
    const n = [s, c, 0];
    const at = (x, z) => [x * c + R * s, -x * s + R * c, z];
    const xw = -0.27, half = neck + ((wide - neck) * (-0.06 - xw)) / 0.34, reach = 0.025;
    const wl = new Wheel({ shaft: l, center: at(xw, half + rl - reach), axis: n, radius: rl, width: 1 * IN, kind: 'omni', mu: MU.omni });
    const wr = is8793
      ? new Wheel({ shaft: r, center: at(xw, -(half + rr - reach)), axis: n, radius: rr, width: 1 * IN, mu: MU.compliant, tread: COMPLIANT_TREAD })
      : new Wheel({ shaft: r, center: at(xw, -(half + rr - reach)), axis: n, radius: rr, width: 1 * IN, kind: 'omni', mu: MU.omni });
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
// one run of 8793's ball path (js/mech/r8793.js), with its turret indexer and shooter if turret:
// FUEL on the carpet in front, three abreast, come at the robot at 1 m/s (it drives into them),
// the intake takes them over the bumper, the conveyor down to single file; with the turret, the
// indexer's rails and wheels carry them up the turret's axis to the shooter (at elevation e, the
// flywheel held at target motor rotations/s, the indexer on throughout)
function pathRun(seed, { x44 = true, turret = false, e = 60, target = 60, nBalls = 12, sec = 5, yaw = 0, hold = false } = {}) {
  let sd = seed;
  const rnd = () => ((sd = (sd * 16807) % 2147483647) / 2147483647 - 0.5);
  const w = world();
  const fb = fixed(w);
  // the carpet in front of the robot (8793's frame: 27.5in); its bumper (square foam) and the rest
  // of the robot FUEL touch are pathSurfaces, as in the game
  const halfL = 0.349, front = halfL + BUMPER_T;
  // the carpet, running under the robot at 1 m/s (the robot driving into the FUEL on it)
  const carpet = w.createRigidBody(RAPIER.RigidBodyDesc.kinematicVelocityBased().setLinvel(-1, 0, 0));
  w.createCollider(RAPIER.ColliderDesc.cuboid(15, 0.01, 1).setTranslation(front + 15, -0.01, 0).setFriction(0.3).setRestitution(0.2), carpet);
  const trains = build8793();
  const arm = armWheels(trains), wheels = [...conveyorWheels(trains), ...arm];
  const sf = pathSurfaces(PATH_MESH, { yaw });
  sf.arm.onShaft(trains.pivot, [ARM.pivot[0], ARM.pivot[1], 0]);
  const surfaces = Object.values(sf);
  const mech = new Mechanism({ substeps: 6, sub: Number(process.env.PATHSUB ?? 1), shafts: [...new Set(wheels.map((x) => x.shaft)), trains.pivot], wheels, surfaces });
  // the arm down on its pivot, under their position loop (as js/mech/phys8793.js runs it): the
  // FUEL under its rollers lift it. ARM=fixed holds it rigid on its stop instead.
  const armSt = armState((ROBOTS['8793'].intake.fold.stowDeg * Math.PI) / 180, 0), rigid = process.env.ARM === 'fixed';
  let maxLift = 0;
  const mechs = [mech];
  let smech = null;
  if (turret) {
    mechs.push(new Mechanism({ substeps: 6, shafts: trains.ishafts, wheels: indexerWheels(trains) }));
    smech = new Mechanism({ substeps: 6, sub: 8, shafts: [trains.large, trains.small], wheels: shooterWheels(trains, e, { yaw }) });
    mechs.push(smech);
  }
  // their auto-unjam (runIntakeWithAutoUnjam, as js/mech/phys8793.js runs it): the X60 stalled
  // (over 55 A, under 10 rotations/s) for 0.125 s reverses the whole path for 0.125 s, the turret
  // indexer too. NOUNJAM=1 leaves it out
  let jamT = 0, unjamT = 0, unjams = 0, jamBalls = null; const jamLog = [];
  const control = () => {
    const h = trains.horizontal, stalled = Math.abs(h.motor.current) > JAM.amps && Math.abs(h.omega) / (2 * Math.PI) < JAM.rps;
    if (unjamT > 0) { unjamT -= DT; if (unjamT <= 0) jamT = 0; } else if (!process.env.NOUNJAM) {
      jamT = stalled ? jamT + DT : 0;
      if (jamT > JAM.debounce) { unjamT = JAM.reverse; jamT = 0; unjams++; if (process.env.JAMLOG && jamBalls) jamLog.push(jamBalls().map((q) => { const p = q.translation(); const cs = mechs.flatMap((m) => m.contacts).filter((c) => c.ball === q && c.force > 20); return `${p.x.toFixed(2)},${p.y.toFixed(2)},${p.z.toFixed(2)}[${cs.map((c) => c.wheel.name.replace(/ \(.*\)/, '') + ' ' + c.force.toFixed(0)).join('; ')}]`; }).filter((x) => !x.endsWith('[]')).join(' | ')); }
    }
    intake8793(trains, true, { x44, dir: unjamT > 0 ? -1 : 1 });
    if (turret) { trains.indexer.cmd = hold ? 0 : unjamT > 0 ? -DUTY_INDEXER : DUTY_INDEXER; trains.shooter.cmd = hold ? 0 : flywheelDuty(trains, target); }
  };
  const step = (balls) => {
    control();
    servoArm(trains, armSt, true, DT);
    if (rigid) trains.pivot.stop(0, 0);
    sf.arm.setPose([ARM.pivot[0], ARM.pivot[1], 0], 0, armSt.roll);
    placeArm(arm, armSt.roll);
    for (const m of mechs) m.step(DT, balls, m === mech && TOGETHER ? balls : null);
    turnArm(trains, armSt, DT);
    maxLift = Math.max(maxLift, (armSt.roll * 180) / Math.PI);
    w.step();
  };
  for (let i = 0; i < 1 / DT; i++) step([]); // spin up
  const free = Math.abs(trains.horizontal.omega);
  const balls = [];
  for (let i = 0; i < nBalls; i++) {
    const AB = Number(process.env.ABREAST ?? 3), x = 0.68 + 0.17 * Math.floor(i / AB) + 0.02 * rnd(), z = AB === 3 ? (i % 3 - 1) * 0.17 + 0.02 * rnd() : (i % AB - (AB - 1) / 2) * 0.16 + 0.02 * rnd();
    balls.push(ball(w, x, R + 0.003, z, [-1, 0, 0]));
    if (TOGETHER) balls.at(-1).collider(0).setCollisionGroups(groups(GROUP.HELD, 0xffff & ~GROUP.HELD));
  }
  jamBalls = () => balls.filter((q) => !gone.has(q));
  const past = new Map(), shots = [], touched = new Set(), gone = new Set();
  let peakI = 0, minW = free, minRps = target, stallT = 0;
  const trace = process.env.TRACE ? balls[Number(process.env.TRACE)] : null;
  for (let k = 0, t = 0; k < sec / DT; k++, t += DT) {
    const live = balls.filter((b) => !gone.has(b));
    step(live);
    if (trace && k % 12 === 0 && !gone.has(trace)) { const p = trace.translation(), v = trace.linvel(); console.log(`t ${t.toFixed(2)} ${p.x.toFixed(3)},${p.y.toFixed(3)},${p.z.toFixed(3)} v ${v.x.toFixed(2)},${v.y.toFixed(2)},${v.z.toFixed(2)}: ${mechs.flatMap((m) => m.contacts).filter((c) => c.ball === trace).map((c) => c.wheel.name + ' ' + c.force.toFixed(0) + 'N').join(', ')}`); }
    peakI = Math.max(peakI, Math.abs(trains.horizontal.motor.current));
    if (Math.abs(trains.horizontal.omega) < 0.3 * free) stallT += DT;
    minW = Math.min(minW, Math.abs(trains.horizontal.omega));
    if (turret) minRps = Math.min(minRps, trains.shooter.omega / (2 * Math.PI));
    for (const b of live) {
      if (!past.has(b) && b.translation().x < 0.0) {
        past.set(b, t);
        // (without the turret, the indexer's stand-in: FUEL onto its rails are taken away)
        if (!turret) { gone.add(b); b.setTranslation({ x: -50 - past.size, y: 5, z: 0 }, true); }
      }
      if (!smech) continue;
      if (smech.contacts.some((c) => c.ball === b)) touched.add(b);
      else if (touched.has(b)) {
        const v = b.linvel();
        shots.push({ t, v: Math.hypot(v.x, v.y, v.z), el: (Math.atan2(v.y, Math.hypot(v.x, v.z)) * 180) / Math.PI });
        gone.add(b);
        b.setTranslation({ x: 50 + shots.length, y: 5, z: 0 }, true);
      }
    }
  }
  if (process.env.JAMLOG) for (const j of jamLog) console.log(`  seed ${seed} unjam: ${j}`);
  const times = [...past.values()].sort((a, b) => a - b);
  const left = balls.filter((b) => !(turret ? gone : past).has(b));
  if (process.env.STUCK) for (const b of left) { const p = b.translation(); console.log(`  seed ${seed} left at ${p.x.toFixed(3)} ${p.y.toFixed(3)} ${p.z.toFixed(3)}: ${mechs.flatMap((m) => m.contacts).filter((c) => c.ball === b).map((c) => c.wheel.name + ' ' + c.force.toFixed(0) + 'N').join(', ')}`); }
  const xs = left.map((b) => b.translation().x);
  const res = { n: balls.length, through: past.size, first: times[0] ?? NaN, last: times[times.length - 1] ?? NaN,
    rate: times.length > 1 ? (times.length - 1) / (times[times.length - 1] - times[0]) : 0, peakI, minSpeed: (100 * minW) / free,
    stuck: xs.length ? `${xs.length} at ${Math.min(...xs).toFixed(2)}..${Math.max(...xs).toFixed(2)}` : '',
    unjams, stallT, maxLift, shots, shotRate: shots.length > 1 ? (shots.length - 1) / (shots[shots.length - 1].t - shots[0].t) : 0, minRps,
    held: balls.map((b) => b.translation()).filter((p) => p.x < 0.35 && p.y > 0.05 && p.y < 0.7).map((p) => [p.x, p.y, p.z]) };
  w.free();
  return res;
}
function mulq(a, b) {
  return { x: a.w * b.x + a.x * b.w + a.y * b.z - a.z * b.y, y: a.w * b.y - a.x * b.z + a.y * b.w + a.z * b.x, z: a.w * b.z + a.x * b.y - a.y * b.x + a.z * b.w, w: a.w * b.w - a.x * b.x - a.y * b.y - a.z * b.z };
}

const pick = process.argv.slice(2);
for (const [k, f] of Object.entries(tests)) if (!pick.length || pick.includes(k)) f();
