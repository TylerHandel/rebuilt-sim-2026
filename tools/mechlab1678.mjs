// 1678's mechanisms (js/mech/r1678.js) on their own, no field, no robot: the CAD's ball path with
// its rollers, measured against what the team's code expects. Run:
//   node --import ./tools/node-env.mjs tools/mechlab1678.mjs [test ...]
// Tests: rollers (each roller's free speed, the drum's spin-up), shooter (FUEL fed up the tunnel at
// their regressions' drum and tunnel speeds and hood angles: how fast and how steep they leave,
// and whether that reaches the HUB from the regression's distance), path (FUEL on a treadmill
// carpet, the robot driving into them at 1 m/s: intake, floor, tunnel, drum; how many and how
// fast), intake (their intake() alone, 24 FUEL), intakefull (filled through the intake from the
// NEUTRAL ZONE's FUEL until it can't take more: tools/fillbench.mjs), hold (how many it holds,
// everything off, set in a layer at a time), dump (a full one fired at their 3.2 m shot).
// Env: SEEDS (6), STUCK (where the ones left over are), SPEED (1 m/s), SEC (intakefull's 12 s),
// FILLSEED (hold and dump: each FUEL set a few mm off its spot, another fill), BINS (dump's shots
// each quarter second)
import { RAPIER } from '../js/physics.js';
import { FUEL, GROUP, groups, HUB } from '../js/constants.js';
import { Mechanism } from '../js/mech/mechanism.js';
import { sim2D } from '../js/ballistics.js';
import {
  build1678, wheels1678, pathSurfaces1678, placeIntake, placeHood, placeNet, intakeRot, slotExt, slotRate, SLOT, hoodRot, hoodElevation, shotFor, REGRESSIONS,
  INTAKE, SHOOTER, TUNNEL, FEEDER, FLEX, FLOOR, TUNNEL_D, DRUM, PIVOT, RANGES,
} from '../js/mech/r1678.js';
import PATH from '../js/mech/path1678.js';
import { EXIT_PER_DRUM_SURFACE, servoArm, turnArm, tunnelLoop, CONTRACT } from '../js/mech/phys1678.js';
import { fillRun, fillReport } from './fillbench.mjs';

await RAPIER.init();
const R = FUEL.radius, DT = 1 / 120, SEEDS = Number(process.env.SEEDS ?? 6);
const HELD = groups(GROUP.HELD, 0xffff & ~GROUP.HELD);
const EXT = slotExt(INTAKE.down), LIFT = 8.5 * 0.0254 /* their climber's kMaxPosition */, SPEED = Number(process.env.SPEED ?? 1);
if (process.env.INTAKE_D) INTAKE.main.d = Number(process.env.INTAKE_D); // (to try other roller sizes)

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
const RPM = (w) => (w * 60) / (2 * Math.PI);

// the robot's mechanisms, fixed in the world: the intake at their angle deg, the hood at hood, the
// extension out and the lid up (as in a match once it's intaked)
function rig({ deg = Number(process.env.ARM ?? INTAKE.down), hood = SHOOTER.hoodMin, ext = slotExt(deg), lift = LIFT } = {}) {
  const t = build1678(), wh = wheels1678(t), s = pathSurfaces1678(PATH, t);
  placeIntake(wh.intake, deg);
  s.intake.setPose([PIVOT[0], PIVOT[1], 0], 0, intakeRot(deg));
  placeHood(wh.hood, hood);
  s.hood.setPose([DRUM[0], DRUM[1], 0], 0, hoodRot(hood));
  s.slide.setPose([ext, 0, 0]); s.slide.onSlide(t.pivot, [1, 0, 0]);
  s.lid.setPose([0, lift, 0]);
  placeNet(s.net, ext, lift);
  const pathW = [...wh.floor, ...wh.tunnel, ...wh.intake];
  const path = new Mechanism({ substeps: 6, shafts: [...new Set(pathW.map((x) => x.shaft)), t.pivot], wheels: pathW, surfaces: Object.values(s) });
  const shooter = new Mechanism({ substeps: 6, sub: 8, shafts: [t.drum, t.hood], wheels: [wh.drum, ...wh.hood] });
  for (const tr of [t.intake, t.floor, t.tunnel, t.feederT, t.shooter]) tr.cmd = null;
  return { t, wh, s, path, shooter };
}
// their velocity loops: the drum toward rpm, the tunnel's top roller toward rpm (max volts; phys1678's
// tunnelLoop, at the Talons' 1 kHz)
function drum(t, rpm) {
  const rps = t.drum.omega / (2 * Math.PI), tgt = rpm / 60;
  t.shooter.cmd = Math.max(0, Math.min(1, (SHOOTER.kV * tgt + SHOOTER.kP * (tgt - rps) + SHOOTER.kS) / 12));
}
const tunnel = (t, rpm, max = 12) => tunnelLoop(t, { rpm, max });
// a FUEL leaving the drum: where it comes down through the HUB's rim height, back from the exit
function landing(o) {
  const hit = sim2D(o.y, o.v, (o.el * Math.PI) / 180, HUB.rimFront);
  return hit ? hit.d : NaN;
}
// a FUEL that touched the drum and has left it going up: shot
function shotWatch(shooter) {
  const touched = new Set(), shots = [];
  return {
    shots,
    check(balls, t) {
      for (const b of balls) {
        if (shooter.contacts.some((c) => c.ball === b)) touched.add(b);
        else if (touched.has(b)) {
          touched.delete(b);
          const v = b.linvel(), p = b.translation();
          if (v.y > 2) { shots.push({ t, v: Math.hypot(v.x, v.y, v.z), el: (Math.atan2(v.y, -v.x) * 180) / Math.PI, x: p.x, y: p.y, b }); b.setTranslation({ x: 50 + shots.length, y: 5, z: 0 }, true); }
        }
      }
    },
  };
}
const inHopper = (b) => { const p = b.translation(); return p.x > -0.36 && p.x < 0.66 && Math.abs(p.z) < 0.36 && p.y > 0.1 && p.y < 0.8; };
// fill it a layer at a time (FUEL set into the space over the ones already in, let settle)
// (FILLSEED: each FUEL set a few mm off its spot, seeded: another fill of the same hopper)
function fill(w, balls, step, layers = 8, y = 0.62, x1 = 0.76) {
  let sd = Number(process.env.FILLSEED ?? 0);
  const rnd = () => (sd ? ((sd = (sd * 16807) % 2147483647) / 2147483647 - 0.5) * 0.008 : 0);
  for (let layer = 0; layer < layers; layer++) {
    for (let i = 0; i < 6; i++) for (let k = 0; k < 4; k++) {
      const x = -0.02 + 0.155 * i + rnd(), z = -0.233 + 0.155 * k + rnd();
      if (x > x1) continue;
      if (balls.some((b) => { const p = b.translation(); return Math.hypot(p.x - x, p.y - y, p.z - z) < 2 * R; })) continue;
      balls.push(ball(w, x, y, z));
    }
    step(1.5 / DT);
  }
}

const tests = {
  rollers() {
    const { t, path, shooter } = rig();
    t.intake.cmd = 1; t.floor.cmd = 1; t.feederT.cmd = 1;
    for (let i = 0; i < 2 / DT; i++) { drum(t, 3550); tunnel(t, TUNNEL.up); path.step(DT, []); shooter.step(DT, []); }
    const surf = (sh, d) => Math.abs(sh.omega) * d / 2;
    console.log(`1678's rollers, free (12 V; the tunnel at their UP 5000 RPM, the drum at 3550 RPM, their longest ferry):`);
    console.log(`  intake X60s ${RPM(t.intake.omega).toFixed(0)} rpm: 2in roller ${surf(t.main, INTAKE.main.d).toFixed(2)} m/s at its surface`);
    console.log(`  floor X60 ${RPM(t.floor.omega).toFixed(0)} rpm: flex wheels ${surf(t.flex[0], 2 * 0.0254).toFixed(2)} m/s, polycarbonate rollers ${surf(t.rollers[0], 0.025).toFixed(2)} m/s`);
    console.log(`  tunnel X60s ${RPM(t.tunnel.omega).toFixed(0)} rpm: back rollers ${t.back.map((s) => surf(s, TUNNEL_D).toFixed(2)).join(', ')} m/s bottom to top, front ${t.front.map((s) => surf(s, TUNNEL_D).toFixed(2)).join(', ')}`);
    console.log(`  feeder X60 ${RPM(t.feederT.omega).toFixed(0)} rpm: ${surf(t.feeder, FEEDER.d).toFixed(2)} m/s`);
    console.log(`  shooter X60s ${RPM(t.shooter.omega).toFixed(0)} rpm: drum ${RPM(t.drum.omega).toFixed(0)} rpm, ${surf(t.drum, SHOOTER.drum.d).toFixed(2)} m/s; hood rollers ${surf(t.hood, SHOOTER.hoodD).toFixed(2)} m/s`);
    for (const target of [2100, 3200]) {
      const r2 = rig();
      let T = 0;
      for (; T < 8; T += DT) { drum(r2.t, target); r2.shooter.step(DT, []); if (RPM(r2.t.drum.omega) > target - 80) break; }
      console.log(`  the drum from rest to within 80 of ${target} RPM: ${T.toFixed(2)} s`);
    }
  },
  // FUEL up the tunnel into the drum at their regressions' rows
  shooter() {
    console.log(`1678's shooter, FUEL fed up the tunnel three at a time (feeder 12 V, the tunnel at the regression's RPM):`);
    const rows = [...REGRESSIONS.hub.rpm.map(([d]) => ['hub', d]).filter(([, d]) => d > 1), ...REGRESSIONS.ferry.rpm.map(([d]) => ['ferry', d]).filter(([, d]) => d > 4), ['far', 13]];
    // (SHOT=rpm,hood,tunnel tries one shot of your own)
    if (process.env.SHOT) { const [rpm, hood, tun] = process.env.SHOT.split(',').map(Number); rows.splice(0, rows.length, ['custom', 0, { rpm, hood, tunnel: tun }]); }
    for (const [kind, d, custom] of rows) {
      const want = custom ?? shotFor(kind, d);
      const { t, path, shooter } = rig({ hood: want.hood });
      const w = world();
      for (let i = 0; i < 3 / DT; i++) { drum(t, want.rpm); shooter.step(DT, []); }
      t.feederT.cmd = 1; t.floor.cmd = 1;
      const balls = [-0.15, 0, 0.15].map((z) => ball(w, -0.17, 0.22, z));
      const watch = shotWatch(shooter);
      for (let i = 0; i < 2.5 / DT && watch.shots.length < balls.length; i++) {
        drum(t, want.rpm); tunnel(t, want.tunnel);
        path.step(DT, balls, balls); shooter.step(DT, balls); w.step();
        watch.check(balls, i * DT);
      }
      w.free();
      const outs = watch.shots;
      if (!outs.length) { console.log(`  ${kind} ${d} m (${want.rpm.toFixed(0)} RPM, hood ${want.hood}): nothing came out`); continue; }
      const o = { v: med(outs.map((q) => q.v)), el: med(outs.map((q) => q.el)), x: med(outs.map((q) => q.x)), y: med(outs.map((q) => q.y)) };
      const surf = t.drum.omega * SHOOTER.drum.d / 2;
      const land = landing(o) - o.x;
      console.log(`  ${kind} ${d} m (${want.rpm.toFixed(0)} RPM, hood ${want.hood}): ${outs.length}/3 out at ${o.v.toFixed(2)} m/s (${(o.v / surf).toFixed(2)} of the drum's ${surf.toFixed(1)}; the estimate ${EXIT_PER_DRUM_SURFACE}), ${o.el.toFixed(1)} deg (${hoodElevation(want.hood).toFixed(1)} by the CAD's geometry), ` +
        (kind === 'hub' ? `comes down at the rim ${land.toFixed(2)} m back (HUB's center ${d} m: ${land - d > 0 ? '+' : ''}${(land - d).toFixed(2)})` : `comes down to the carpet ~${(sim2D(o.y, o.v, (o.el * Math.PI) / 180, 0.1)?.d - o.x).toFixed(1)} m back`));
    }
  },
  // carpet to shot: FUEL in front, the robot driving into them; intaking, then shooting at 3.2 m
  path() {
    const runs = [];
    for (let seed = 1; seed <= SEEDS; seed++) runs.push(pathRun(seed));
    console.log(`1678 carpet to shot (18 FUEL, three abreast, the robot at 1 m/s, the intake and floor and tunnel and feeder all on): a median ${med(runs.map((r) => r.shots))} shot (${Math.min(...runs.map((r) => r.shots))}..${Math.max(...runs.map((r) => r.shots))}), ` +
      `${med(runs.map((r) => r.rate)).toFixed(1)} FUEL/s once they're coming; ${med(runs.map((r) => r.inside))} in it at the end; intake X60s down to ${med(runs.map((r) => r.minIntake)).toFixed(0)}%, floor X60 to ${med(runs.map((r) => r.minFloor)).toFixed(0)}%`);
    if (process.env.STUCK) for (const r of runs) console.log('   left', r.left.map((p) => p.map((v) => v.toFixed(2)).join(',')).join(' | '));
  },
  // intake only (their intake(): the floor idle), FUEL on the treadmill: how many it takes in
  intake() {
    for (let seed = 1; seed <= Math.min(SEEDS, 3); seed++) {
      const r = pathRun(seed, { shoot: false, n: 24, sec: 6 });
      console.log(`1678 intaking only (their intake(): roller 12 V, everything else idle), 24 FUEL at ${SPEED} m/s: ${r.inside} in it after 6 s (in past the bumper each second: ${r.timeline.join(', ')}); intake X60s down to ${r.minIntake.toFixed(0)}%`);
      if (process.env.STUCK) console.log('   left', r.left.map((p) => p.map((v) => v.toFixed(2)).join(',')).join(' | '));
    }
  },
  // filled through the intake until it can't take more: driving at SPEED (1 m/s) into the NEUTRAL
  // ZONE's FUEL, their intake() (roller 12 V, the rest idle) and its prefill, the extension out
  // and the lid up (tools/fillbench.mjs)
  intakefull() {
    const runs = [];
    for (let seed = 1; seed <= Number(process.env.SEEDS ?? 3); seed++) {
      const { t, path } = rig();
      t.intake.cmd = 1;
      const fill = intakeFill(t);
      const r = fillRun(RAPIER, { world: () => world({ carpet: false }), ball, path, DT, held: held1678, own: own1678, top: TOP1678, control: fill, speed: SPEED, sec: Number(process.env.SEC ?? 12), seed, front: 0.4 });
      console.log(fillReport(`1678 intaking at ${SPEED} m/s, seed ${seed}`, r));
      runs.push(r);
    }
    const f = runs.map((r) => r.fulls[0]?.n).filter((n) => n !== undefined);
    console.log(`1678 through its intake (${runs.length} fills): the most it held ${Math.min(...runs.map((r) => r.most))}-${Math.max(...runs.map((r) => r.most))} (mean ${(runs.reduce((a, r) => a + r.most, 0) / runs.length).toFixed(1)}), full at ${f.length ? `${Math.min(...f)}-${Math.max(...f)}` : '-'} (${f.length} of ${runs.length})`);
  },
  hold() {
    const { path } = rig();
    const w = world();
    const balls = [];
    const step = (n) => { for (let i = 0; i < n; i++) { path.step(DT, balls, balls); w.step(); } };
    fill(w, balls, step, 10);
    step(2 / DT);
    const t0 = performance.now();
    step(1 / DT);
    const ms = (performance.now() - t0) / (1 / DT);
    console.log(`  (its mechanisms with them all in: ${ms.toFixed(2)} ms a step)`);
    console.log(`1678 holds ${balls.filter(inHopper).length} with everything off, the intake down, the extension out and the lid up (a layer at a time)`);
    const lo = rig({ lift: 0, ext: 0, deg: INTAKE.stowed });
    const w2 = world(), b2 = [];
    const step2 = (n) => { for (let i = 0; i < n; i++) { lo.path.step(DT, b2, b2); w2.step(); } };
    fill(w2, b2, step2, 6, 0.45, 0.3);
    step2(2 / DT);
    console.log(`1678 holds ${b2.filter((b) => { const p = b.translation(); return inHopper(b) && p.x < 0.34; }).length} folded up (intake stowed, extension in, lid down)`);
  },
  // a full one fired: filled as hold does, then their shoot() at 3.2 m (aimed: floor and feeder
  // 12 V, the tunnel at the regression's RPM)
  dump() {
    const want = shotFor('hub', 3.222);
    const { t, wh, s: surf, path, shooter } = rig({ hood: want.hood });
    const w = world();
    const balls = [];
    const step = (n, run) => { for (let i = 0; i < n; i++) { if (run) run(); path.step(DT, balls, balls); shooter.step(DT, balls); w.step(); } };
    fill(w, balls, (n) => step(n, () => drum(t, want.rpm)), 10);
    const n0 = balls.filter(inHopper).length;
    const watch = shotWatch(shooter);
    t.floor.cmd = 1; t.feederT.cmd = 1;
    let minRpm = Infinity;
    // (their contractHopper after a second, again each time it's done while FUEL are out front,
    // as js/mech/phys1678.js runs it; NOCONTRACT=1 leaves the intake down)
    let deg = INTAKE.down, c = null, cool = 0, goal = INTAKE.down;
    const arm = () => {
      if (process.env.NOCONTRACT) return;
      cool = Math.max(0, cool - DT);
      if (!c && k1 * DT > 1 && !cool && balls.some((b) => { const p = b.translation(); return p.y < 3 && p.x > 0.343 && p.x < 0.72; })) c = { i: 0, t: 0 };
      if (c) {
        const st = CONTRACT[c.i];
        c.t += DT; t.intake.cmd = 2 / 12;
        if (st !== 'wait') goal = st ?? INTAKE.down;
        const done = st === 'wait' ? c.t >= 0.5 : Math.abs(deg - goal) < 12;
        if (done && ++c.i >= CONTRACT.length) { c = null; cool = 1; t.intake.cmd = null; } else if (done) c.t = 0;
      }
      servoArm(t, deg, goal, DT);
      placeIntake(wh.intake, deg);
      surf.intake.setPose([PIVOT[0], PIVOT[1], 0], 0, intakeRot(deg));
      // (the extension on the arm's pin and slot)
      const k = slotRate(deg);
      surf.slide.setPose([slotExt(deg), 0, 0]); surf.slide.slideK = k; t.pivot.inertia = INTAKE.J + SLOT.mass * k * k;
      placeNet(surf.net, slotExt(deg), LIFT);
    };
    let k1 = 0;
    for (let k = 0; k < 8 / DT; k++) {
      k1 = k;
      step(1, () => { drum(t, want.rpm); tunnel(t, want.tunnel); arm(); });
      if (!process.env.NOCONTRACT) deg = turnArm(t, deg, DT);
      watch.check(balls, k * DT);
      if (watch.shots.length) minRpm = Math.min(minRpm, RPM(t.drum.omega));
    }
    const s = watch.shots.map((x) => x.t), first = s[0] ?? 0, in1 = s.filter((x) => x - first < 1).length;
    // (BINS=1: shots each quarter second from the first; and when the 50th went)
    if (process.env.BINS) { const bins = []; for (const x of s) { const i = Math.floor((x - first) / 0.25); bins[i] = (bins[i] || 0) + 1; } console.log(`   shots each 0.25 s: ${Array.from(bins, (v) => v || 0).join(' ')}; the 50th ${(s[49] - first).toFixed(2)} s after the first`); }
    const left = balls.filter((b) => b.translation().y < 3 && b.translation().x < 40 && inHopper(b));
    if (process.env.STUCK) for (const b of left) { const p = b.translation(); console.log(`   left at ${p.x.toFixed(2)},${p.y.toFixed(2)},${p.z.toFixed(2)}: ${[...path.contacts, ...shooter.contacts].filter((c) => c.ball === b).map((c) => c.wheel.name + ' ' + c.force.toFixed(0)).join(', ')}`); }
    const out = watch.shots;
    const land = out.map((o) => landing(o) - o.x).filter(Number.isFinite);
    console.log(`  (${left.length} still in it; the drum down to ${minRpm.toFixed(0)} RPM of ${want.rpm.toFixed(0)}; shots come down at the rim ${med(land).toFixed(2)} m back (median; ${Math.min(...land).toFixed(2)}..${Math.max(...land).toFixed(2)}), the HUB's center 3.22)`);
    console.log(`1678 dumps ${s.length} of ${n0}: ${in1} in the first second, ${((s.length - 1) / (s[s.length - 1] - first)).toFixed(1)} FUEL/s over the whole dump`);
  },
};

// where a FUEL is (robot frame), as js/mech/phys1678.js where() has it: in it (not one on the carpet
// the intake's reaching for), and the intake's
const HALF = 0.343;
const inFrame1678 = ([x, y, z]) => Math.abs(x) < HALF && Math.abs(z) < HALF && y > 0.105 && y < 0.8;
const inFront1678 = ([x, y, z]) => x >= HALF && x < 0.72 && Math.abs(z) < HALF && y < 0.8;
const held1678 = (p) => inFrame1678(p) || (inFront1678(p) && p[1] > 0.2 && p[0] < 0.6);
const own1678 = (p) => inFrame1678(p) || inFront1678(p);
const TOP1678 = { topX: 0.1, topN: 10 }; // (js/mech/phys1678.js naturalFull)
// their intake()'s prefill, once: once the hopper's CANrange has seen FUEL 0.75 s and the tunnel's
// sees none, the tunnel at 250 RPM and the floor and feeder at 3 V until the tunnel's sees one
// (js/mech/phys1678.js)
function intakeFill(t) {
  let hopperT = 0, tunnelT = 0, state = 'armed';
  const tunnelSees = (ps) => ps.some(([x, y, z]) => Math.hypot(x - RANGES.tunnel.at[0], y - RANGES.tunnel.at[1]) < R * 0.8 && Math.abs(z - RANGES.tunnel.z) < RANGES.tunnel.reach + R);
  const hopperSees = (ps) => ps.some(([x, y, z]) => { const dx = x - RANGES.hopper.at[0]; return dx > 0 && dx < RANGES.hopper.reach + R && Math.abs(y - RANGES.hopper.at[1]) < R * 0.8 && Math.abs(z - RANGES.hopper.z) < R + dx * Math.tan((RANGES.hopper.half * Math.PI) / 180); });
  return (_, balls) => {
    const ps = balls.map((b) => { const p = b.translation(); return [p.x, p.y, p.z]; }).filter(held1678);
    hopperT = hopperSees(ps) ? hopperT + DT : 0;
    tunnelT = tunnelSees(ps) ? tunnelT + DT : 0;
    if (state === 'armed' && hopperT >= 0.75 && tunnelT < 0.06) { state = 'running'; t.floor.cmd = 3 / 12; t.feederT.cmd = 3 / 12; }
    if (state === 'running' && tunnelT >= 0.06) { state = 'done'; t.floor.cmd = null; t.feederT.cmd = null; }
    tunnelLoop(t, state === 'running' ? { rpm: TUNNEL.preload } : null);
  };
}

function pathRun(seed, { shoot = true, n = 18, sec = 7 } = {}) {
  let sd = seed;
  const rnd = () => ((sd = (sd * 16807) % 2147483647) / 2147483647 - 0.5);
  const want = shotFor('hub', 3.222);
  const { t, path, shooter } = rig({ hood: want.hood });
  const w = world({ carpet: false });
  // the carpet, running under the robot at 1 m/s
  const carpet = w.createRigidBody(RAPIER.RigidBodyDesc.kinematicVelocityBased().setLinvel(-SPEED, 0, 0));
  w.createCollider(RAPIER.ColliderDesc.cuboid(15, 0.01, 1).setTranslation(0.4 + 15, -0.01, 0).setFriction(0.3).setRestitution(0.2), carpet);
  w.createCollider(RAPIER.ColliderDesc.cuboid(0.4, 0.01, 1).setTranslation(0, -0.01, 0).setFriction(0.3), w.createRigidBody(RAPIER.RigidBodyDesc.fixed()));
  t.intake.cmd = 1;
  if (shoot) { t.floor.cmd = 1; t.feederT.cmd = 1; }
  const run = () => { if (shoot) { drum(t, want.rpm); tunnel(t, want.tunnel); } };
  for (let i = 0; i < 3 / DT; i++) { run(); shooter.step(DT, []); path.step(DT, []); }
  const free = { intake: Math.abs(t.intake.omega), floor: Math.abs(t.floor.omega) || 1 };
  const balls = [];
  for (let i = 0; i < n; i++) balls.push(ball(w, 0.8 + 0.17 * Math.floor(i / 3) + 0.02 * rnd(), R + 0.003, (i % 3 - 1) * 0.18 + 0.02 * rnd(), [-SPEED, 0, 0]));
  const watch = shotWatch(shooter);
  let minIntake = 1, minFloor = 1;
  const timeline = [];
  for (let k = 0; k < sec / DT; k++) {
    const live = balls.filter((b) => b.translation().y < 3);
    run();
    path.step(DT, live, live); shooter.step(DT, live); w.step();
    minIntake = Math.min(minIntake, Math.abs(t.intake.omega) / free.intake);
    if (shoot) minFloor = Math.min(minFloor, Math.abs(t.floor.omega) / free.floor);
    watch.check(live, k * DT);
    // (how many are in, each second)
    if (k % Math.round(1 / DT) === Math.round(1 / DT) - 1) timeline.push(balls.filter((b) => { const p = b.translation(); return p.y < 3 && p.x < 0.45 && p.y > 0.15; }).length);
    // (TRACE=i follows FUEL i: where it is and what it's touching)
    if (process.env.TRACE && k % 6 === 0) {
      const b = balls[Number(process.env.TRACE)], p = b.translation(), v = b.linvel();
      console.log(`   ${(k * DT).toFixed(2)} s at ${p.x.toFixed(3)},${p.y.toFixed(3)},${p.z.toFixed(3)} v ${v.x.toFixed(2)},${v.y.toFixed(2)},${v.z.toFixed(2)}: ${[...path.contacts, ...shooter.contacts].filter((c) => c.ball === b).map((c) => c.wheel.name + ' ' + c.force.toFixed(0)).join(', ')}`);
    }
  }
  const s = watch.shots.map((x) => x.t);
  const inside = balls.filter((b) => { const p = b.translation(); return p.y < 3 && p.x < 0.66 && p.x > -0.36 && p.y > 0.1; });
  const left = balls.filter((b) => b.translation().y < 3 && !inside.includes(b)).map((b) => { const p = b.translation(); return [p.x, p.y, p.z]; });
  w.free();
  return { timeline, shots: s.length, rate: s.length > 2 ? (s.length - 1) / (s[s.length - 1] - s[0]) : 0, inside: inside.length, minIntake: 100 * minIntake, minFloor: 100 * minFloor, left };
}

const pick = process.argv.slice(2);
for (const [name, fn] of Object.entries(tests)) if (!pick.length || pick.includes(name)) fn();
