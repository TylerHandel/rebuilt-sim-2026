// 7769's mechanisms (js/mech/r7769.js) on their own, no field, no robot: the CAD's ball path with its
// rollers and wheels. Run:
//   node --import ./tools/node-env.mjs tools/mechlab7769.mjs [test ...]
// Tests: rollers (each roller's free speed, the shooter's spin-up, the intake sliding out and in),
// vector (one FUEL at each end of the feeder: the vectored wheels' hand), shooter (FUEL fed from the
// floor up the ramp at the shot table's rows: how fast and how steep they leave, and where they come
// down; SHOT=rpm,hood tries one, MAP=1 a grid), tune (each row's shooter RPM re-found so the sim's FUEL
// come down at the row's distance: js/mech/phys7769.js SHOTS, PASSES; ONLY=hub|pass, FROM=m), path (FUEL on a treadmill carpet, the robot
// driving into them at 1 m/s, intaking and shooting), intake (the intake alone, 24 FUEL at SPEED),
// intakefull (filled through the intake from the NEUTRAL ZONE's FUEL until it can't put more in:
// tools/fillbench.mjs), hold (how many it holds, set in a layer at a time, everything off), dump (a
// full one fired at DIST, with the intake shuffling in and out as phys7769 runs it; NOSHUFFLE=1 holds
// it out).
// Env: SEEDS, SPEED (1 m/s), SEC, STUCK, TRACE, DIST (3 m), VECTOR (the vectored wheels' roller angle,
// 45), FILLSEED
import { RAPIER } from '../js/physics.js';
import { FUEL, GROUP, groups, HUB } from '../js/constants.js';
import { Mechanism } from '../js/mech/mechanism.js';
import { sim2D } from '../js/ballistics.js';
import {
  build7769, wheels7769, pathSurfaces7769, placeIntake, placeSlide, slideStops, hoodRot, hoodElevation, kickRot, slideOffset,
  SHOOTER_SURFACES, INTAKE, SLIDE, SHOOTER, FEED, FLOOR, AXLE, HALF_L, FRONT_X, NET,
} from '../js/mech/r7769.js';
import PATH from '../js/mech/path7769.js';
import { shotFor, SHOTS, PASSES, EXIT_PER_WHEEL_SURFACE, shuffleGoal, slideVolts, SHUFFLE } from '../js/mech/phys7769.js';
import { fillRun, fillReport } from './fillbench.mjs';

await RAPIER.init();
const R = FUEL.radius, DT = 1 / 120, SEEDS = Number(process.env.SEEDS ?? 6), SPEED = Number(process.env.SPEED ?? 1);
const HELD = groups(GROUP.HELD, 0xffff & ~GROUP.HELD);
const TAU = 2 * Math.PI, RPM = (w) => (w * 60) / TAU;

function world({ carpet = true } = {}) {
  const w = new RAPIER.World({ x: 0, y: -9.81, z: 0 });
  w.timestep = DT;
  w.integrationParameters.numSolverIterations = 6;
  if (carpet) w.createCollider(RAPIER.ColliderDesc.cuboid(5, 0.01, 5).setTranslation(0, -0.01, 0).setFriction(0.3).setRestitution(0.2), w.createRigidBody(RAPIER.RigidBodyDesc.fixed()));
  return w;
}
function ball(w, x, y, z, v = [0, 0, 0]) {
  const b = w.createRigidBody(RAPIER.RigidBodyDesc.dynamic().setTranslation(x, y, z).setLinvel(...v).setCcdEnabled(true).setAngularDamping(FUEL.angularDamping));
  w.createCollider(RAPIER.ColliderDesc.ball(R).setMass(FUEL.mass).setFriction(FUEL.friction).setRestitution(FUEL.restitution).setCollisionGroups(HELD), b);
  return b;
}
const med = (a) => { const b = [...a].sort((p, q) => p - q); return b[b.length >> 1]; };
const mean = (a) => a.reduce((p, q) => p + q, 0) / Math.max(1, a.length);
const pos = (b) => { const p = b.translation(); return [p.x, p.y, p.z]; };

// the robot's mechanisms fixed in the world: the intake ext m out, the hood at deg
function rig({ ext = SLIDE.out, hood = 0 } = {}) {
  const t = build7769(), wh = wheels7769(t, { vector: Number(process.env.VECTOR ?? 45) }), s = pathSurfaces7769(PATH, t);
  const pathS = Object.entries(s).filter(([k]) => !SHOOTER_SURFACES.includes(k)).map(([, v]) => v);
  const pathW = [...wh.intake, ...wh.feed];
  const r = { t, wh, s, ext, hood };
  r.path = new Mechanism({ substeps: 6, shafts: [...new Set(pathW.map((x) => x.shaft)), t.belts, t.slide], wheels: pathW, surfaces: pathS });
  r.shooter = new Mechanism({ substeps: 6, sub: 8, shafts: [t.wheels], wheels: wh.shooter, surfaces: SHOOTER_SURFACES.map((k) => s[k]) });
  for (const tr of [t.intake, t.floor, t.feedT, t.shooter]) tr.cmd = null;
  t.slide.cmd = null;
  r.pose = () => {
    placeSlide(s, r.ext);
    placeIntake(wh.intake, r.ext);
    s.hood.setPose([...AXLE, 0], 0, hoodRot(r.hood));
  };
  r.pose();
  // the shooter's velocity loop toward rpm (0 coasts)
  r.spin = (rpm) => { t.shooter.vel = rpm > 0 ? { target: (rpm / 60) * TAU, kV: SHOOTER.kV, kP: SHOOTER.kP, per: 1 / TAU } : null; if (!rpm) t.shooter.cmd = null; };
  // the slide toward goal (m out) as phys7769 runs it; the intake moving as its pinions turn
  r.slideTo = (goal) => {
    slideStops(t.slide, r.ext);
    t.slide.motor.statorLimit = goal >= SLIDE.out - 0.01 && r.ext >= SLIDE.out - 0.01 ? 5 : 45;
    t.slide.cmd = goal === null ? null : slideVolts(goal, r.ext) / 12;
    t.slide.impulse(SLIDE.mass * 9.81 * SLIDE.dir[1] * SLIDE.pinion * DT);
  };
  r.move = () => {
    r.ext += t.slide.omega * SLIDE.pinion * DT;
    if (r.ext < 0) { r.ext = 0; if (t.slide._omega < 0) t.slide._omega = 0; }
    if (r.ext > SLIDE.out) { r.ext = SLIDE.out; if (t.slide._omega > 0) t.slide._omega = 0; }
    r.pose();
  };
  return r;
}
// a FUEL that touched the shooter's wheels and has left them going up and back: shot
function shotWatch(shooter) {
  const touched = new Set(), shots = [];
  return {
    shots,
    check(balls, t) {
      for (const b of balls) {
        if (shooter.contacts.some((c) => c.ball === b && c.wheel.shaft?.name?.startsWith('shooter'))) touched.add(b);
        else if (touched.has(b) && !shooter.contacts.some((c) => c.ball === b)) {
          touched.delete(b);
          const v = b.linvel(), p = b.translation();
          if (v.y > 2) { shots.push({ t, v: Math.hypot(v.x, v.y, v.z), el: (Math.atan2(v.y, -v.x) * 180) / Math.PI, x: p.x, y: p.y, z: p.z, b }); b.setTranslation({ x: 50 + shots.length, y: 5, z: 0 }, true); }
        }
      }
    },
  };
}
// where a shot comes down through height h, back from the robot's center
const landAt = (o, h) => { const k = sim2D(o.y, o.v, (o.el * Math.PI) / 180, h); return k ? k.d - o.x : NaN; };
// where a FUEL is (robot frame), as js/mech/phys7769.js where() has it
const frontAt = (ext) => FRONT_X + slideOffset(ext)[0];
const inFrame = ([x, y, z]) => Math.abs(x) < HALF_L && Math.abs(z) < 0.35 && y > 0.05 && y < 0.62;
const inFront = ([x, y, z], ext) => x >= HALF_L && x < frontAt(ext) + 0.03 && Math.abs(z) < 0.35 && y < 0.6;
const held7769 = (p, ext = SLIDE.out) => inFrame(p) || (inFront(p, ext) && p[1] > 0.2);
const own7769 = (p, ext = SLIDE.out) => inFrame(p) || inFront(p, ext);

// fill it a layer at a time (FUEL set in over the ones already in, let settle)
function fill(w, balls, step, layers = 8, y = 0.47, ext = SLIDE.out) {
  let sd = Number(process.env.FILLSEED ?? 0);
  const rnd = () => (sd ? ((sd = (sd * 16807) % 2147483647) / 2147483647 - 0.5) * 0.008 : 0);
  for (let layer = 0; layer < layers; layer++) {
    for (let i = 0; i < 7; i++) for (let k = 0; k < 4; k++) {
      const x = -0.01 + 0.155 * i + rnd(), z = -0.233 + 0.155 * k + rnd();
      if (x > frontAt(ext) - R - 0.01) continue;
      if (balls.some((b) => { const p = b.translation(); return Math.hypot(p.x - x, p.y - y, p.z - z) < 2 * R; })) continue;
      balls.push(ball(w, x, y, z));
    }
    step(1.2 / DT);
  }
}

// FUEL set on the floor's back, fed up the ramp at a row's settings: what leaves
function shoot({ rpm, hood, n = 6, settle = 2.5 }) {
  const rg = rig({ hood }), { t, path, shooter } = rg;
  const w = world();
  for (let i = 0; i < settle / DT; i++) { rg.spin(rpm); shooter.step(DT, []); }
  t.floor.cmd = 1; t.feedT.cmd = 1;
  const balls = [];
  // (on the belts, two rows of two and two on top of them)
  const at = [[0.02, 0.25], [0.17, 0.28], [0.09, 0.42]];
  for (let i = 0; i < n; i++) { const [x, y] = at[Math.floor(i / 2) % 3]; balls.push(ball(w, x, y, i % 2 ? 0.09 : -0.09)); }
  const watch = shotWatch(shooter);
  let minRpm = Infinity;
  for (let i = 0; i < 5 / DT && watch.shots.length < balls.length; i++) {
    rg.spin(rpm);
    path.step(DT, balls, balls); shooter.step(DT, balls); w.step();
    watch.check(balls, i * DT);
    if (watch.shots.length) minRpm = Math.min(minRpm, RPM(t.wheels.omega));
    if (process.env.TRACE && i % 6 === 0) {
      const b = balls[Number(process.env.TRACE)], p = b.translation(), v = b.linvel();
      console.log(`   ${(i * DT).toFixed(2)} s at ${p.x.toFixed(3)},${p.y.toFixed(3)},${p.z.toFixed(3)} v ${v.x.toFixed(2)},${v.y.toFixed(2)},${v.z.toFixed(2)}: ${[...path.contacts, ...shooter.contacts].filter((c) => c.ball === b).map((c) => c.wheel.name + ' ' + c.force.toFixed(0)).join(', ')}`);
    }
  }
  const left = balls.filter((b) => b.translation().y < 3).map(pos);
  w.free();
  return { shots: watch.shots, minRpm, left };
}

const tests = {
  rollers() {
    const rg = rig(), { t, path, shooter } = rg;
    t.intake.cmd = 1; t.floor.cmd = 1; t.feedT.cmd = 1;
    for (let i = 0; i < 2 / DT; i++) { rg.spin(4000); path.step(DT, []); shooter.step(DT, []); }
    const sf = (sh, r) => (Math.abs(sh.omega) * r).toFixed(2);
    console.log(`7769's rollers, free (12 V; the shooter's loop at 4000 RPM):`);
    console.log(`  intake roller ${sf(t.roller, INTAKE.r)} m/s at its surface; floor belts ${sf(t.belts, FLOOR.pitch)} m/s; feeder: compliant ${sf(t.feed, FEED.compliant.r)}, vectored ${sf(t.feed, FEED.vector.r)} m/s`);
    console.log(`  shooter ${RPM(t.wheels.omega).toFixed(0)} RPM, ${sf(t.wheels, SHOOTER.wheel.r)} m/s at the Stealth wheels' surface`);
    for (const goal of [2500, 3500, 4500, 5500]) {
      const r2 = rig();
      let T = 0;
      for (; T < 8; T += DT) { r2.spin(goal); r2.shooter.step(DT, []); if (RPM(r2.t.wheels.omega) > goal * 0.97) break; }
      console.log(`  the shooter from rest to within 3% of ${goal} RPM: ${T.toFixed(2)} s`);
    }
    const r3 = rig({ ext: 0 });
    let T = 0;
    for (; T < 3; T += DT) { r3.slideTo(SLIDE.out); r3.path.step(DT, []); r3.move(); if (r3.ext >= SLIDE.out - 1e-6) break; }
    const out = T;
    for (T = 0; T < 3; T += DT) { r3.slideTo(0); r3.path.step(DT, []); r3.move(); if (r3.ext <= 1e-6) break; }
    console.log(`  the intake out ${out.toFixed(2)} s, back in ${T.toFixed(2)} s (empty; its kick bar up ${(kickRot(0) * 180 / Math.PI).toFixed(0)} deg in)`);
  },
  vector() {
    // (each hand at each end, from a few places under the wheels: where the FUEL is after 1.2 s)
    for (const z0 of [-0.27, 0.27]) for (const hand of [45, -45]) {
      const res = [];
      for (const [x0, y0] of [[-0.06, 0.23], [-0.1, 0.24], [-0.14, 0.26]]) {
        const rg = rig(), { t, path, wh } = rg;
        for (const w of wh.feed) if (w.kind === 'mecanum' && Math.sign(w.center[2]) === Math.sign(z0)) w.mecanum = hand;
        const w = world();
        t.feedT.cmd = 1; t.floor.cmd = 1;
        const b = ball(w, x0, y0, z0);
        for (let i = 0; i < 1.2 / DT; i++) { path.step(DT, [b], [b]); w.step(); }
        res.push(pos(b).map((v) => v.toFixed(2)).join(','));
        w.free();
      }
      console.log(`  z ${z0}, rollers at ${hand}: ${res.join(' | ')}`);
    }
  },
  shooter() {
    console.log(`7769's shooter, FUEL fed from the floor up the ramp (floor and feeder 12 V, the wheels on their loop):`);
    let rows = [...SHOTS.map(([d, rpm, hood]) => ['hub', d, rpm, hood]), ...PASSES.map(([d, rpm, hood]) => ['pass', d, rpm, hood])];
    if (process.env.SHOT) { const [rpm, hood] = process.env.SHOT.split(',').map(Number); rows = [['custom', 0, rpm, hood]]; }
    if (process.env.MAP) rows = [0, 10, 20, 30].flatMap((h) => [2000, 3000, 4000, 5000].map((rpm) => ['map', 0, rpm, h]));
    for (const [kind, d, rpm, hood] of rows) {
      const { shots, minRpm, left } = shoot({ rpm, hood });
      if (!shots.length) { console.log(`  ${kind} ${d} m (${rpm} RPM, hood ${hood}): nothing came out; left at ${left.map((p) => p.map((v) => v.toFixed(2)).join(',')).join(' | ')}`); continue; }
      const v = med(shots.map((q) => q.v)), el = med(shots.map((q) => q.el)), surf = (rpm / 60) * TAU * SHOOTER.wheel.r;
      const land = med(shots.map((o) => landAt(o, kind === "pass" ? R : HUB.rimFront)));
      if (process.env.STUCK && left.length) console.log("   left at", left.map((p) => p.map((v) => v.toFixed(2)).join(",")).join(" | "));
      console.log(`  ${kind} ${d} m (${rpm} RPM, hood ${hood}): ${shots.length}/6 out at ${v.toFixed(2)} m/s (${(v / surf).toFixed(2)} of the wheels' ${surf.toFixed(1)}), ${el.toFixed(1)} deg (${hoodElevation(hood).toFixed(1)} by the CAD), from ${med(shots.map((o) => o.x)).toFixed(3)},${med(shots.map((o) => o.y)).toFixed(3)}; the wheels down to ${minRpm.toFixed(0)}; ` +
        `comes down ${kind === 'pass' ? 'to the carpet' : 'through the rim'} ${land.toFixed(2)} m back (${kind === 'map' ? '' : (land - d > 0 ? '+' : '') + (land - d).toFixed(2)})`);
    }
  },
  // each row's RPM re-found: FUEL fed as in play, the RPM whose shots come down (through the rim's
  // height at the HUB, to the carpet passing) at the row's distance from the robot's center
  tune() {
    for (const kind of (process.env.ONLY ?? 'hub,pass').split(',')) {
      const rows = (kind === 'hub' ? SHOTS : PASSES).filter(([d]) => d >= Number(process.env.FROM ?? 0)), out = [];
      const H = kind === 'hub' ? HUB.rimFront : R;
      for (const [d, rpm0, hood] of rows) {
        const at = (rpm) => { const s = shoot({ rpm, hood }).shots; const L = s.map((o) => landAt(o, H)).filter(Number.isFinite); return L.length ? mean(L) : NaN; };
        // (a bracket round the row's guess, widened until it holds the distance; then secant steps,
        // halving where a guess comes down short of the rim; the closest try wins)
        const tries = [];
        const tryAt = (rpm) => { const L = at(rpm); tries.push([rpm, L]); return L; };
        let lo = Math.max(1200, rpm0 * 0.8), hi = Math.min(6000, rpm0 * 1.25), flo = tryAt(lo), fhi = tryAt(hi);
        while (!(flo <= d) && lo > 1200) { hi = lo; fhi = flo; lo = Math.max(1200, lo * 0.85); flo = tryAt(lo); }
        while (!(fhi >= d) && hi < 6000) { lo = hi; flo = fhi; hi = Math.min(6000, hi * 1.2); fhi = tryAt(hi); }
        for (let j = 0; j < 10; j++) {
          const [, lb0] = tries.reduce((a, q) => (Math.abs(q[1] - d) < Math.abs(a[1] - d) ? q : a), [0, Infinity]);
          if (Math.abs(lb0 - d) <= 0.04) break;
          let b = Number.isFinite(flo) && Number.isFinite(fhi) && fhi > flo ? lo + ((d - flo) * (hi - lo)) / (fhi - flo) : (lo + hi) / 2;
          b = Math.max(lo + 0.1 * (hi - lo), Math.min(hi - 0.1 * (hi - lo), b));
          const lb = tryAt(b);
          if (!(lb >= d)) { lo = b; flo = lb; } else { hi = b; fhi = lb; }
        }
        const [b, lb] = tries.reduce((a, q) => (Math.abs(q[1] - d) < Math.abs(a[1] - d) ? q : a), [0, Infinity]);
        const ok = Math.abs(lb - d) < 0.1;
        out.push([d, ok ? Math.round(b / 5) * 5 : null, hood]);
        console.log(`  ${kind} ${d} m, hood ${hood}: ${ok ? Math.round(b) : '-'} RPM (${lb.toFixed(2)} m)`);
      }
      console.log(`${kind === 'hub' ? 'SHOTS' : 'PASSES'}`, JSON.stringify(out));
    }
  },
  path() {
    const runs = [];
    for (let seed = 1; seed <= SEEDS; seed++) runs.push(pathRun(seed));
    console.log(`7769 carpet to shot (18 FUEL, three abreast, the robot at ${SPEED} m/s, everything on, shooting at the 3 m row): a median ${med(runs.map((r) => r.shots))} shot (${Math.min(...runs.map((r) => r.shots))}..${Math.max(...runs.map((r) => r.shots))}), ` +
      `${med(runs.map((r) => r.rate)).toFixed(1)} FUEL/s once they're coming; ${med(runs.map((r) => r.inside))} in it at the end`);
    if (process.env.STUCK) for (const r of runs) console.log('   left', r.left.map((p) => p.map((v) => v.toFixed(2)).join(',')).join(' | '));
  },
  intake() {
    for (let seed = 1; seed <= Math.min(SEEDS, 3); seed++) {
      const r = pathRun(seed, { shoot: false, n: 24, sec: 6 });
      console.log(`7769 intaking only (roller 12 V, the rest stopped), 24 FUEL at ${SPEED} m/s: ${r.inside} in it after 6 s (in past the bumper each second: ${r.timeline.join(', ')}); the roller down to ${r.minIntake.toFixed(0)}%`);
      if (process.env.STUCK) console.log('   left', r.left.map((p) => p.map((v) => v.toFixed(2)).join(',')).join(' | '));
    }
  },
  intakefull() {
    const runs = [];
    for (let seed = 1; seed <= Number(process.env.SEEDS ?? 3); seed++) {
      const rg = rig(), { t, path } = rg;
      t.intake.cmd = 1;
      const r = fillRun(RAPIER, { world: () => world({ carpet: false }), ball, path, DT, held: (p) => held7769(p), own: (p) => own7769(p), top: { topX: 0.25, topN: 6 }, speed: SPEED, sec: Number(process.env.SEC ?? 12), seed, front: 0.409, keep: true, control: () => { rg.slideTo(SLIDE.out); } });
      console.log(fillReport(`7769 intaking at ${SPEED} m/s, seed ${seed}`, r));
      const ps = r.balls.map(pos);
      const bins = {}; for (const p of ps.filter((q) => held7769(q))) { const k = Math.floor(p[0] * 10) / 10; bins[k] = (bins[k] || 0) + 1; }
      console.log('   load along x:', Object.keys(bins).sort((a, b) => a - b).map((k) => `${(+k).toFixed(1)}:${bins[k]}`).join(' '), '| in the mouth', ps.filter((p) => own7769(p) && !held7769(p)).length, '| top', Math.max(...ps.filter((p) => held7769(p)).map((p) => p[1] + R)).toFixed(3));
      if (process.env.STUCK) for (const b of r.balls) { const p = pos(b); if (own7769(p) && !held7769(p)) console.log(`   mouth ${p.map((v) => v.toFixed(2)).join(',')}: ${path.contacts.filter((c) => c.ball === b).map((c) => c.wheel.name + ' ' + c.force.toFixed(0)).join(', ')}`); }
      r.world.free();
      runs.push(r);
    }
    console.log(`7769 through its intake (${runs.length} fills): the most it held ${Math.min(...runs.map((r) => r.most))}-${Math.max(...runs.map((r) => r.most))} (mean ${mean(runs.map((r) => r.most)).toFixed(1)})`);
  },
  hold() {
    const ext = Number(process.env.EXT ?? SLIDE.out);
    const { path } = rig({ ext });
    const w = world();
    const balls = [];
    const step = (n) => { for (let i = 0; i < n; i++) { path.step(DT, balls, balls); w.step(); } };
    fill(w, balls, step, 9, 0.47, ext);
    step(2 / DT);
    const inn = balls.filter((b) => held7769(pos(b), ext));
    const top = Math.max(...inn.map((b) => b.translation().y + R));
    console.log(`7769 holds ${inn.length} with everything off, the intake ${ext.toFixed(2)} m out (a layer at a time; the load's top ${top.toFixed(2)} m, the net ${NET.bulge} m of slack); ${balls.length - inn.length} fell out`);
  },
  dump() {
    const d = Number(process.env.DIST ?? 3.0), want = shotFor('hub', d);
    const rg = rig({ hood: want.hood }), { t, path, shooter } = rg;
    const w = world();
    const balls = [];
    const step = (n, run) => { for (let i = 0; i < n; i++) { if (run) run(); path.step(DT, balls, balls); shooter.step(DT, balls); w.step(); } };
    fill(w, balls, (n) => step(n, () => { rg.spin(want.rpm); rg.slideTo(SLIDE.out); rg.move(); }), 9);
    const n0 = balls.filter((b) => held7769(pos(b))).length;
    const watch = shotWatch(shooter);
    let minRpm = Infinity;
    for (let k = 0; k < 8 / DT; k++) {
      const T = k * DT;
      step(1, () => {
        rg.spin(want.rpm); t.floor.cmd = 1; t.feedT.cmd = 1;
        rg.slideTo(process.env.NOSHUFFLE ? SLIDE.out : shuffleGoal(T, rg.ext));
        rg.move();
      });
      watch.check(balls, T);
      if (watch.shots.length) minRpm = Math.min(minRpm, RPM(t.wheels.omega));
    }
    const s = watch.shots.map((x) => x.t), first = s[0] ?? 0, in1 = s.filter((x) => x - first < 1).length;
    { const bins = []; for (const x of s) { const i = Math.floor((x - first) / 0.5); bins[i] = (bins[i] || 0) + 1; } console.log(`   shots each 0.5 s: ${Array.from(bins, (v) => v || 0).join(' ')}`); }
    const left = balls.filter((b) => b.translation().y < 3 && b.translation().x < 40 && held7769(pos(b)));
    if (process.env.STUCK) for (const b of left) { const p = b.translation(); console.log(`   left at ${p.x.toFixed(2)},${p.y.toFixed(2)},${p.z.toFixed(2)}: ${[...path.contacts, ...shooter.contacts].filter((c) => c.ball === b).map((c) => c.wheel.name + ' ' + c.force.toFixed(0)).join(', ')}`); }
    const land = watch.shots.map((o) => landAt(o, HUB.rimFront)).filter(Number.isFinite);
    const last = s[s.length - 1] ?? 0;
    console.log(`  (${left.length} still in it; the wheels down to ${minRpm.toFixed(0)} of ${want.rpm} RPM; shots come down at the rim ${med(land).toFixed(2)} m back (median; ${Math.min(...land).toFixed(2)}..${Math.max(...land).toFixed(2)}))`);
    console.log(`7769 dumps ${s.length} of ${n0} at its ${d} m row: ${in1} in the first second, ${((s.length - 1) / Math.max(1e-6, last - first)).toFixed(1)} FUEL/s over the whole dump (${(last - first).toFixed(2)} s)`);
  },
};

function pathRun(seed, { shoot = true, n = 18, sec = 7 } = {}) {
  let sd = seed;
  const rnd = () => ((sd = (sd * 16807) % 2147483647) / 2147483647 - 0.5);
  const want = shotFor('hub', 3.0);
  const rg = rig({ hood: want.hood }), { t, path, shooter } = rg;
  const w = world({ carpet: false });
  const carpet = w.createRigidBody(RAPIER.RigidBodyDesc.kinematicVelocityBased().setLinvel(-SPEED, 0, 0));
  w.createCollider(RAPIER.ColliderDesc.cuboid(15, 0.01, 1).setTranslation(0.409 + 15, -0.01, 0).setFriction(0.3).setRestitution(0.2), carpet);
  w.createCollider(RAPIER.ColliderDesc.cuboid(0.409, 0.01, 1).setTranslation(0, -0.01, 0).setFriction(0.3), w.createRigidBody(RAPIER.RigidBodyDesc.fixed()));
  t.intake.cmd = 1;
  const run = () => { rg.slideTo(SLIDE.out); if (shoot) { rg.spin(want.rpm); t.floor.cmd = 1; t.feedT.cmd = 1; } };
  for (let i = 0; i < 3 / DT; i++) { run(); shooter.step(DT, []); path.step(DT, []); rg.move(); }
  const free = Math.abs(t.intake.omega);
  const balls = [];
  for (let i = 0; i < n; i++) balls.push(ball(w, 0.9 + 0.17 * Math.floor(i / 3) + 0.02 * rnd(), R + 0.003, (i % 3 - 1) * 0.18 + 0.02 * rnd(), [-SPEED, 0, 0]));
  const watch = shotWatch(shooter);
  let minIntake = 1;
  const timeline = [];
  for (let k = 0; k < sec / DT; k++) {
    const live = balls.filter((b) => b.translation().y < 3);
    run();
    path.step(DT, live, live); shooter.step(DT, live); w.step(); rg.move();
    minIntake = Math.min(minIntake, Math.abs(t.intake.omega) / free);
    watch.check(live, k * DT);
    if (k % Math.round(1 / DT) === Math.round(1 / DT) - 1) timeline.push(balls.filter((b) => { const p = b.translation(); return p.y < 3 && p.x < 0.409 && p.y > 0.12; }).length);
    if (process.env.TRACE && k % 6 === 0) {
      const b = balls[Number(process.env.TRACE)], p = b.translation(), v = b.linvel();
      console.log(`   ${(k * DT).toFixed(2)} s at ${p.x.toFixed(3)},${p.y.toFixed(3)},${p.z.toFixed(3)} v ${v.x.toFixed(2)},${v.y.toFixed(2)},${v.z.toFixed(2)}: ${[...path.contacts, ...shooter.contacts].filter((c) => c.ball === b).map((c) => c.wheel.name + ' ' + c.force.toFixed(0)).join(', ')}`);
    }
  }
  const s = watch.shots.map((x) => x.t);
  const inside = balls.filter((b) => { const p = b.translation(); return p.y < 3 && held7769([p.x, p.y, p.z]); });
  const left = balls.filter((b) => b.translation().y < 3 && !inside.includes(b)).map(pos);
  w.free();
  return { timeline, shots: s.length, rate: s.length > 2 ? (s.length - 1) / (s[s.length - 1] - s[0]) : 0, inside: inside.length, minIntake: 100 * minIntake, left };
}

const pick = process.argv.slice(2);
for (const [name, fn] of Object.entries(tests)) if (!pick.length || pick.includes(name)) fn();
