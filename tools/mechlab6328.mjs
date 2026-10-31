// 6328's mechanisms (js/mech/r6328.js) on their own, no field, no robot: the CAD's ball path with its
// rollers, measured against what the team's code expects. Run:
//   node --import ./tools/node-env.mjs tools/mechlab6328.mjs [test ...]
// Tests: rollers (each roller's free speed, the flywheel's spin-up under their profile), shooter
// (FUEL fed up the kickers at their maps' rows: how fast and how steep they leave, and where they come
// down; SHOT=fly,hood,kicker tries one), tune (each HUB and pass row's flywheel speed re-found so the
// sim's FUEL come down at the row's distance: js/mech/r6328.js SIM_FLY), path (FUEL on a treadmill
// carpet, the robot driving into them: intake, floor, kickers, drum), intake (the intake alone, 24
// FUEL), intakefull (filled through the intake from the NEUTRAL ZONE's FUEL until it can't take
// more: tools/fillbench.mjs), hold (how many it holds, set in a layer at a time), dump (filled through
// its intake as intakefull, then launched at 3 m with their compaction; FILL=drop: dropped in).
// Env: SEEDS, SPEED (1 m/s), SEC, FILLSEED, STUCK, TRACE, NOCOMPACT, PROFILE, DIST, FILL, FLOORR,
// NET_BULGE
import { RAPIER } from '../js/physics.js';
import { FUEL, GROUP, groups, HUB } from '../js/constants.js';
import { Mechanism } from '../js/mech/mechanism.js';
import { sim2D } from '../js/ballistics.js';
import {
  build6328, wheels6328, pathSurfaces6328, placeIntake, placeHood, placeNet, intakeRot, hoodRot, hoodElevation, slideExt, slideRate, shotFor,
  INTAKE, SHOOTER, KICK, BACK_R, FLOOR_R, PIVOT, DRUM, HUB_MAP, PASS_MAP, SLIDE_MASS, NET, LAUNCHER_X,
} from '../js/mech/r6328.js';
import PATH from '../js/mech/path6328.js';
import { EXIT_PER_DRUM_SURFACE, flywheelVolts, kickers, servoArm, turnArm } from '../js/mech/phys6328.js';
import { fillRun, fillReport } from './fillbench.mjs';

await RAPIER.init();
const R = FUEL.radius, DT = 1 / 120, SEEDS = Number(process.env.SEEDS ?? 6);
const HELD = groups(GROUP.HELD, 0xffff & ~GROUP.HELD);
const SPEED = Number(process.env.SPEED ?? 1);

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
const mean = (a) => a.reduce((p, q) => p + q, 0) / a.length;

// the robot's mechanisms fixed in the world: the intake at their angle deg (down), the hopper's
// front where its pin puts it, the hood at hood
function rig({ deg = Number(process.env.ARM ?? INTAKE.down), hood = SHOOTER.hoodMin } = {}) {
  const t = build6328(), wh = wheels6328(t), s = pathSurfaces6328(PATH, t);
  const r = { t, wh, s, deg };
  r.pose = (d) => {
    r.deg = d;
    placeIntake(wh.intake, d);
    s.intake.setPose([PIVOT[0], PIVOT[1], 0], 0, intakeRot(d));
    s.ramp.setPose([PIVOT[0], PIVOT[1], 0], 0, intakeRot(d));
    const k = slideRate(d);
    s.slide.setPose([slideExt(d), 0, 0]); s.slide.slideK = k; t.pivot.inertia = INTAKE.J + SLIDE_MASS * k * k;
    placeNet(s.net, slideExt(d));
  };
  r.pose(deg);
  placeHood(wh.hood, hood);
  s.hood.setPose([DRUM[0], DRUM[1], 0], 0, hoodRot(hood));
  const pathW = [...wh.floor, ...wh.kick, ...wh.intake];
  r.path = new Mechanism({ substeps: 6, shafts: [...new Set(pathW.map((x) => x.shaft)), t.pivot], wheels: pathW, surfaces: Object.values(s) });
  // (the net stretched as far as the load pushed it last step, as js/mech/phys6328.js has it)
  const pathStep = r.path.step.bind(r.path);
  r.path.step = (dt, ...a) => { s.net.flex(r.path.contacts, dt); pathStep(dt, ...a); r.netMax = Math.max(r.netMax ?? 0, s.net.B); };
  r.shooter = new Mechanism({ substeps: 6, sub: 8, shafts: [t.drum, t.hood], wheels: [wh.drum, ...wh.hood] });
  for (const tr of [t.intake, t.floor, t.backT, t.frontT, t.flywheel]) tr.cmd = 0;
  r.fly = { sp: 0, acc: 0, init: false };
  // their flywheel loop toward goal (rad/s; bang-bang past 3.25 m at the HUB)
  r.drum = (goal, bang = false) => { t.flywheel.cmd = flywheelVolts(r.fly, goal, t.drum.omega, DT, { bangBang: bang }) / 12; };
  return r;
}
// a FUEL that touched the drum and has left it going up and back: shot
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
          if (v.y > 2 && v.x < 0) { shots.push({ t, v: Math.hypot(v.x, v.y, v.z), el: (Math.atan2(v.y, -v.x) * 180) / Math.PI, x: p.x, y: p.y, z: p.z, b }); b.setTranslation({ x: 50 + shots.length, y: 5, z: 0 }, true); }
        }
      }
    },
  };
}
// where a shot comes down through height h, back from the robot's center
const landAt = (o, h) => { const k = sim2D(o.y, o.v, (o.el * Math.PI) / 180, h); return k ? k.d - o.x : NaN; };
const inHopper = (b) => { const p = b.translation(); return p.x > -0.36 && p.x < 0.72 && Math.abs(p.z) < 0.36 && p.y > 0.12 && p.y < 0.8; };
function fill(w, balls, step, layers = 8, y = 0.6, x1 = 0.62) {
  let sd = Number(process.env.FILLSEED ?? 0);
  const rnd = () => (sd ? ((sd = (sd * 16807) % 2147483647) / 2147483647 - 0.5) * 0.008 : 0);
  for (let layer = 0; layer < layers; layer++) {
    for (let i = 0; i < 6; i++) for (let k = 0; k < 4; k++) {
      const x = 0.03 + 0.155 * i + rnd(), z = -0.233 + 0.155 * k + rnd();
      if (x > x1) continue;
      if (balls.some((b) => { const p = b.translation(); return Math.hypot(p.x - x, p.y - y, p.z - z) < 2 * R; })) continue;
      balls.push(ball(w, x, y, z));
    }
    step(1.5 / DT);
  }
}

// FUEL fed from the floor's back into the kickers at a row's settings: what leaves
function shoot({ fly, hood, kicker, bang = false, n = 6, settle = 2.5 }) {
  const rg = rig({ hood }), { t, path, shooter } = rg;
  const w = world();
  for (let i = 0; i < settle / DT; i++) { rg.drum(fly, bang); shooter.step(DT, []); }
  t.floor.cmd = 1; kickers(t, kicker);
  const balls = [];
  const zs = [-0.15, 0, 0.15];
  for (let i = 0; i < n; i++) balls.push(ball(w, -0.06 + 0.16 * Math.floor(i / 3), 0.29, zs[i % 3]));
  const watch = shotWatch(shooter);
  let minFly = Infinity;
  for (let i = 0; i < 4 / DT && watch.shots.length < balls.length; i++) {
    rg.drum(fly, bang); kickers(t, kicker);
    path.step(DT, balls, balls); shooter.step(DT, balls); w.step();
    watch.check(balls, i * DT);
    if (watch.shots.length) minFly = Math.min(minFly, t.drum.omega);
  }
  w.free();
  return { shots: watch.shots, minFly, rg };
}

const tests = {
  rollers() {
    const rg = rig(), { t, path, shooter } = rg;
    t.intake.cmd = 1; t.floor.cmd = 1; kickers(t, 6);
    for (let i = 0; i < 2 / DT; i++) { rg.drum(195); path.step(DT, []); shooter.step(DT, []); }
    const sf = (sh, r) => (Math.abs(sh.omega) * r).toFixed(2);
    console.log(`6328's rollers, free (12 V; the kickers' loops at 6 m/s; the flywheel at 195 rad/s, their farthest HUB row):`);
    console.log(`  intake X60s ${(t.intake.omega * 60 / 2 / Math.PI).toFixed(0)} rpm: 2in roller ${sf(t.roller, INTAKE.roller.r)} m/s at its surface`);
    console.log(`  floor X60s ${(t.floor.omega * 60 / 2 / Math.PI).toFixed(0)} rpm: rollers ${sf(t.rollers[0], FLOOR_R)} m/s`);
    console.log(`  front kicker ${sf(t.kick, KICK.r)} m/s, back kicker ${sf(t.back[0], BACK_R)} m/s (their loops' 6)`);
    console.log(`  drum ${t.drum.omega.toFixed(1)} rad/s, ${sf(t.drum, SHOOTER.drum.r)} m/s; hood roller ${sf(t.hood, SHOOTER.roller.r)} m/s`);
    for (const goal of [145, 195, 400]) {
      const r2 = rig();
      let T = 0;
      for (; T < 8; T += DT) { r2.drum(goal); r2.shooter.step(DT, []); if (r2.t.drum.omega > goal - 2) break; }
      console.log(`  the flywheel from rest to within 2 rad/s of ${goal} rad/s (their profile, 160 A): ${T.toFixed(2)} s`);
    }
  },
  shooter() {
    console.log(`6328's shooter, FUEL fed from the floor's back up the kickers (floor 12 V, kickers at the map's surface speed):`);
    let rows = [...HUB_MAP.fly.map(([d]) => ['hub', d]).filter(([, d]) => d < 5), ...PASS_MAP.fly.map(([d]) => ['pass', d])];
    if (process.env.SHOT) { const [fly, hood, kicker] = process.env.SHOT.split(',').map(Number); rows = [['custom', 0, { fly, hood, kicker }]]; }
    for (const [kind, d, custom] of rows) {
      const want = custom ?? shotFor(kind === 'custom' ? 'hub' : kind, d);
      const { shots, minFly } = shoot({ ...want, bang: kind === 'hub' && d > SHOOTER.bangMin });
      if (!shots.length) { console.log(`  ${kind} ${d} m: nothing came out`); continue; }
      const v = med(shots.map((q) => q.v)), el = med(shots.map((q) => q.el)), surf = want.fly * SHOOTER.drum.r;
      const land = med(shots.map((o) => landAt(o, kind === 'pass' ? R : HUB.rimFront)));
      const from = -LAUNCHER_X; // (their distances are the launcher's, 0.151 m behind the center)
      console.log(`  ${kind} ${d} m (${want.fly.toFixed(0)} rad/s, hood ${want.hood}, kicker ${want.kicker} m/s): ${shots.length}/6 out at ${v.toFixed(2)} m/s (${(v / surf).toFixed(2)} of the drum's ${surf.toFixed(1)}), ${el.toFixed(1)} deg (${hoodElevation(want.hood).toFixed(1)} by the CAD), the flywheel down to ${minFly.toFixed(0)}; ` +
        `comes down ${kind === 'pass' ? 'to the carpet' : 'through the rim'} ${(land - from).toFixed(2)} m from the launcher (${(land - from - d) > 0 ? '+' : ''}${(land - from - d).toFixed(2)})`);
    }
  },
  // each row's flywheel speed re-found: FUEL fed as in play, the speed whose shots come down (through
  // the rim's height at the HUB, to the carpet passing) at the row's distance from the launcher
  tune() {
    const out = {};
    for (const kind of ['hub', 'pass']) {
      const m = kind === 'hub' ? HUB_MAP : PASS_MAP;
      out[kind] = [];
      for (const [d, fly0] of m.fly) {
        const hood = m.hood.length > 2 ? m.hood.find(([x]) => x === d)?.[1] ?? 38 : 38;
        const kicker = shotFor(kind, d).kicker, bang = kind === 'hub' && d > SHOOTER.bangMin;
        const H = kind === 'hub' ? HUB.rimFront : R;
        const at = (fly) => { const s = shoot({ fly, hood, kicker, bang }).shots; const L = s.map((o) => landAt(o, H) + LAUNCHER_X).filter(Number.isFinite); return L.length ? mean(L) : NaN; };
        let lo = 60, hi = kind === 'hub' ? 400 : 600, flo = at(lo), fhi = at(hi), b = fly0, lb = at(b);
        for (let j = 0; j < 12 && Math.abs(lb - d) > 0.05; j++) {
          if (lb < d) { lo = b; flo = lb; } else { hi = b; fhi = lb; }
          b = Number.isFinite(flo) && Number.isFinite(fhi) && fhi > flo ? lo + ((d - flo) * (hi - lo)) / (fhi - flo) : (lo + hi) / 2;
          b = Math.max(lo + 0.05 * (hi - lo), Math.min(hi - 0.05 * (hi - lo), b));
          lb = at(b);
        }
        const ok = Math.abs(lb - d) < 0.1;
        out[kind].push(ok ? Math.round(b) : null);
        console.log(`  ${kind} ${d} m, hood ${hood}: their ${fly0} rad/s; the sim's FUEL come down there at ${ok ? Math.round(b) : '-'} rad/s (${lb.toFixed(2)} m)`);
      }
      console.log(`SIM_FLY.${kind}`, JSON.stringify(out[kind]));
    }
  },
  path() {
    const runs = [];
    for (let seed = 1; seed <= SEEDS; seed++) runs.push(pathRun(seed));
    console.log(`6328 carpet to shot (18 FUEL, three abreast, the robot at ${SPEED} m/s, everything on, launching at their 3 m row): a median ${med(runs.map((r) => r.shots))} shot (${Math.min(...runs.map((r) => r.shots))}..${Math.max(...runs.map((r) => r.shots))}), ` +
      `${med(runs.map((r) => r.rate)).toFixed(1)} FUEL/s once they're coming; ${med(runs.map((r) => r.inside))} in it at the end`);
    if (process.env.STUCK) for (const r of runs) console.log('   left', r.left.map((p) => p.map((v) => v.toFixed(2)).join(',')).join(' | '));
  },
  intake() {
    for (let seed = 1; seed <= Math.min(SEEDS, 3); seed++) {
      const r = pathRun(seed, { shoot: false, n: 24, sec: 6 });
      console.log(`6328 intaking only (roller 13 V, the rest stopped), 24 FUEL at ${SPEED} m/s: ${r.inside} in it after 6 s (in past the bumper each second: ${r.timeline.join(', ')}); the roller down to ${r.minIntake.toFixed(0)}%`);
      if (process.env.STUCK) console.log('   left', r.left.map((p) => p.map((v) => v.toFixed(2)).join(',')).join(' | '));
    }
  },
  intakefull() {
    const runs = [];
    for (let seed = 1; seed <= Number(process.env.SEEDS ?? 3); seed++) {
      const rg = rig(), { t, path, s } = rg;
      t.intake.cmd = 1;
      if (process.env.FLOORIN) t.floor.cmd = Number(process.env.FLOORIN) / 12; // (not their code: the floor run while it intakes)
      const r = fillRun(RAPIER, { world: () => world({ carpet: false }), ball, path, DT, held: held6328, own: own6328, top: { topX: 0.1, topN: 10 }, speed: SPEED, sec: Number(process.env.SEC ?? 12), seed, front: 0.42, keep: true });
      console.log(fillReport(`6328 intaking at ${SPEED} m/s, seed ${seed}`, r));
      // (where the load is: how many in each 0.1 m along the robot, and the mouth's)
      const ps = r.balls.map((b) => { const p = b.translation(); return [p.x, p.y, p.z]; });
      const bins = {}; for (const p of ps.filter(held6328)) { const k = Math.floor(p[0] * 10) / 10; bins[k] = (bins[k] || 0) + 1; }
      if (process.env.PROFILE) { const L = ps.filter(held6328), xs = [-0.2, -0.1, 0, 0.1, 0.2, 0.3, 0.4, 0.5, 0.6]; console.log('   top along x:', xs.map((x) => { const q = L.filter((p) => p[0] >= x && p[0] < x + 0.1); return `${x.toFixed(1)}:${q.length ? Math.max(...q.map((p) => p[1] + R)).toFixed(2) : '-'}/${q.filter((p) => p[1] > 0.42).length}`; }).join(' '), '| the net risen', s.net.B.toFixed(3), 'm at its middle, at most', rg.netMax.toFixed(3)); }
      console.log('   load along x:', Object.keys(bins).sort((a, b) => a - b).map((k) => `${(+k).toFixed(1)}:${bins[k]}`).join(' '), '| in the mouth', ps.filter((p) => own6328(p) && !held6328(p)).length);
      if (process.env.STUCK) for (const b of r.balls) { const q = b.translation(), p = [q.x, q.y, q.z]; if (own6328(p) && !held6328(p) || (held6328(p) && p[0] > 0.3)) console.log(`   ${held6328(p) ? 'held ' : 'mouth'} ${p.map((v) => v.toFixed(2)).join(',')}: ${path.contacts.filter((c) => c.ball === b).map((c) => c.wheel.name + ' ' + c.force.toFixed(0)).join(', ')}`); }
      r.world.free();
      runs.push(r);
    }
    console.log(`6328 through its intake (${runs.length} fills; the net's stretch to ${NET.bulge} m, k ${NET.k} N/m): the most it held ${Math.min(...runs.map((r) => r.most))}-${Math.max(...runs.map((r) => r.most))} (mean ${mean(runs.map((r) => r.most)).toFixed(1)})`);
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
    const top = Math.max(...balls.filter(inHopper).map((b) => b.translation().y + R));
    if (process.env.STUCK) for (const b of balls.filter(inHopper)) { const p = b.translation(); if (p.y > 0.5) console.log(`   high: ${p.x.toFixed(2)},${p.y.toFixed(2)},${p.z.toFixed(2)}`); }
    console.log(`6328 holds ${balls.filter(inHopper).length} with everything off, the intake down and the hopper out (a layer at a time; the load's top ${top.toFixed(2)} m, the net ${NET.bulge} m of slack); ${ms.toFixed(2)} ms a step`);
  },
  // a full one launched at a row (DIST, their 3 m), with their compaction. Filled through its intake
  // (fillRun, as intakefull: SEEDS fills, FILLSEED the first) or, FILL=drop, dropped in a layer at a
  // time. Reported: the first second (from the first shot), the best second (any 1 s window), the
  // whole dump's average, and the middle's (the 10th to the 10th from last shot: past the start, before
  // the stragglers)
  dump() {
    const runs = [], n = Number(process.env.SEEDS ?? 3), s0 = Number(process.env.FILLSEED ?? 1);
    for (let seed = s0; seed < s0 + n; seed++) runs.push(dumpRun(seed));
    const f = (k, d = 1) => `${Math.min(...runs.map((r) => r[k])).toFixed(d)}-${Math.max(...runs.map((r) => r[k])).toFixed(d)} (mean ${mean(runs.map((r) => r[k])).toFixed(d)})`;
    console.log(`6328 dumps (${runs.length} fills, ${process.env.FILL === 'drop' ? 'dropped in' : 'through its intake'}): full ${f('n0', 0)}; first second ${f('in1', 0)}, best second ${f('best', 0)}, ${f('avg')} FUEL/s over the dump, ${f('mid')} through the middle`);
  },
};
function dumpRun(seed) {
  const d = Number(process.env.DIST ?? 3.0), want = shotFor('hub', d);
  const rg = rig({ hood: want.hood }), { t, path, shooter } = rg;
  let w, balls;
  if (process.env.FILL === 'drop') {
    w = world(); balls = [];
    fill(w, balls, (k) => { for (let i = 0; i < k; i++) { rg.drum(want.fly); path.step(DT, balls, balls); shooter.step(DT, balls); w.step(); } }, 10);
  } else {
    // (the flywheel spun up while it fills: the dump starts at speed)
    t.intake.cmd = 1;
    const r = fillRun(RAPIER, { world: () => world({ carpet: false }), ball, path, DT, held: held6328, own: own6328, top: { topX: 0.1, topN: 10 }, speed: SPEED, sec: Number(process.env.SEC ?? 12), seed, front: 0.42, keep: true, control: () => { rg.drum(want.fly); shooter.step(DT, []); } });
    w = r.world; r.carpet.setLinvel({ x: 0, y: 0, z: 0 }, true);
    t.intake.cmd = 0;
    // (what it didn't take in: out of the way; it's backed off them)
    for (const b of r.balls) { const q = b.translation(); if (!held6328([q.x, q.y, q.z])) b.setTranslation({ x: 60, y: -5, z: 0 }, true); }
    balls = r.balls.filter((b) => b.translation().x < 40);
  }
  const step = (k, run) => { for (let i = 0; i < k; i++) { if (run) run(); path.step(DT, balls, balls); shooter.step(DT, balls); w.step(); } };
  const n0 = balls.filter(inHopper).length;
  const watch = shotWatch(shooter);
  t.floor.cmd = 1;
  let deg = INTAKE.down, set = INTAKE.down, minFly = Infinity;
  const who = {};
  for (let k = 0; k < 8 / DT; k++) {
    const T = k * DT;
    step(1, () => {
      rg.drum(want.fly, d > SHOOTER.bangMin); kickers(t, want.kicker);
      // (their compaction: 0.35 s in, the slam retracts at 0.88 rad/s, the roller on for 1.2 s)
      if (!process.env.NOCOMPACT && T >= 0.35) { set = Math.min(INTAKE.up, set + (0.88 * 180 / Math.PI) * DT); t.intake.cmd = T < 1.55 ? 1 : 0; }
      servoArm(t, deg, set, DT);
      rg.pose(deg);
    });
    deg = turnArm(t, deg, DT);
    watch.check(balls, T);
    if (watch.shots.length) minFly = Math.min(minFly, t.drum.omega);
    if (process.env.WHO) for (const c of [...path.contacts, ...shooter.contacts]) { const k = c.wheel.name; who[k] = (who[k] || 0) + c.force * DT; }
    if (process.env.WHO && k % 30 === 0) console.log(`   ${T.toFixed(2)} s: ${watch.shots.length} out; floor ${(t.rollers[0].omega * FLOOR_R).toFixed(2)} m/s ${t.floor.motor.current.toFixed(0)} A, front kicker ${(Math.abs(t.kick.omega) * KICK.r).toFixed(2)} m/s ${t.frontT.motor.current.toFixed(0)} A, back ${(t.back[0].omega * BACK_R).toFixed(2)} m/s, drum ${t.drum.omega.toFixed(0)}`);
  }
  if (process.env.WHO) console.log('   impulse by part (N s):', Object.entries(who).sort((a, b) => b[1] - a[1]).map(([k, v]) => `${k} ${v.toFixed(1)}`).join(', '));
  const s = watch.shots.map((x) => x.t), first = s[0] ?? 0, in1 = s.filter((x) => x - first < 1).length;
  const best = Math.max(0, ...s.map((x) => s.filter((y) => y >= x && y < x + 1).length));
  const avg = s.length > 1 ? (s.length - 1) / (s[s.length - 1] - first) : 0;
  const m0 = Math.min(9, s.length - 1), m1 = Math.max(m0, s.length - 10), mid = m1 > m0 ? (m1 - m0) / (s[m1] - s[m0]) : 0;
  const bins = []; for (const x of s) { const i = Math.floor((x - first) / 0.5); bins[i] = (bins[i] || 0) + 1; }
  const left = balls.filter((b) => b.translation().y < 3 && b.translation().x < 40 && inHopper(b));
  if (process.env.STUCK) for (const b of left) { const p = b.translation(); console.log(`   left at ${p.x.toFixed(2)},${p.y.toFixed(2)},${p.z.toFixed(2)}: ${[...path.contacts, ...shooter.contacts].filter((c) => c.ball === b).map((c) => c.wheel.name + ' ' + c.force.toFixed(0)).join(', ')}`); }
  const land = watch.shots.map((o) => landAt(o, HUB.rimFront) + LAUNCHER_X).filter(Number.isFinite);
  console.log(`  fill ${seed}: ${s.length} of ${n0} at their ${d} m row: ${in1} in the first second, best second ${best}, ${avg.toFixed(1)} FUEL/s over the dump, ${mid.toFixed(1)} through the middle; each 0.5 s ${Array.from(bins, (v) => v || 0).join(' ')}` +
    ` (${left.length} left; the flywheel down to ${minFly.toFixed(0)} of ${want.fly.toFixed(0)} rad/s; shots down at the rim ${med(land).toFixed(2)} m from the launcher, ${Math.min(...land).toFixed(2)}..${Math.max(...land).toFixed(2)}; the arm at ${deg.toFixed(0)} deg)`);
  w.free();
  return { n0, shots: s.length, in1, best, avg, mid };
}
// where a FUEL is (robot frame), as js/mech/phys6328.js where() has it with the intake down
const HALF_L = 0.3366, EXT = slideExt(INTAKE.down);
const inFrame6328 = ([x, y, z]) => Math.abs(x) < HALF_L + 0.01 && Math.abs(z) < 0.343 && y > 0.12 && y < 0.8;
const inFront6328 = ([x, y, z]) => x >= HALF_L + 0.01 && x < 0.72 && Math.abs(z) < 0.343 && y < 0.75;
const held6328 = (p) => inFrame6328(p) || (inFront6328(p) && (p[1] > 0.22 || p[0] < 0.53) && p[0] < HALF_L + EXT + 0.01);
const own6328 = (p) => inFrame6328(p) || inFront6328(p);

function pathRun(seed, { shoot = true, n = 18, sec = 7 } = {}) {
  let sd = seed;
  const rnd = () => ((sd = (sd * 16807) % 2147483647) / 2147483647 - 0.5);
  const want = shotFor('hub', 3.0);
  const rg = rig({ hood: want.hood }), { t, path, shooter } = rg;
  const w = world({ carpet: false });
  const carpet = w.createRigidBody(RAPIER.RigidBodyDesc.kinematicVelocityBased().setLinvel(-SPEED, 0, 0));
  w.createCollider(RAPIER.ColliderDesc.cuboid(15, 0.01, 1).setTranslation(0.42 + 15, -0.01, 0).setFriction(0.3).setRestitution(0.2), carpet);
  w.createCollider(RAPIER.ColliderDesc.cuboid(0.42, 0.01, 1).setTranslation(0, -0.01, 0).setFriction(0.3), w.createRigidBody(RAPIER.RigidBodyDesc.fixed()));
  t.intake.cmd = 1;
  const run = () => { if (shoot) { rg.drum(want.fly); kickers(t, want.kicker); t.floor.cmd = 1; } };
  for (let i = 0; i < 3 / DT; i++) { run(); shooter.step(DT, []); path.step(DT, []); }
  const free = Math.abs(t.intake.omega);
  const balls = [];
  for (let i = 0; i < n; i++) balls.push(ball(w, 0.85 + 0.17 * Math.floor(i / 3) + 0.02 * rnd(), R + 0.003, (i % 3 - 1) * 0.18 + 0.02 * rnd(), [-SPEED, 0, 0]));
  const watch = shotWatch(shooter);
  let minIntake = 1;
  const timeline = [];
  for (let k = 0; k < sec / DT; k++) {
    const live = balls.filter((b) => b.translation().y < 3);
    run();
    path.step(DT, live, live); shooter.step(DT, live); w.step();
    minIntake = Math.min(minIntake, Math.abs(t.intake.omega) / free);
    watch.check(live, k * DT);
    if (k % Math.round(1 / DT) === Math.round(1 / DT) - 1) timeline.push(balls.filter((b) => { const p = b.translation(); return p.y < 3 && p.x < 0.42 && p.y > 0.15; }).length);
    if (process.env.TRACE && k % 6 === 0) {
      const b = balls[Number(process.env.TRACE)], p = b.translation(), v = b.linvel();
      console.log(`   ${(k * DT).toFixed(2)} s at ${p.x.toFixed(3)},${p.y.toFixed(3)},${p.z.toFixed(3)} v ${v.x.toFixed(2)},${v.y.toFixed(2)},${v.z.toFixed(2)}: ${[...path.contacts, ...shooter.contacts].filter((c) => c.ball === b).map((c) => c.wheel.name + ' ' + c.force.toFixed(0)).join(', ')}`);
    }
  }
  const s = watch.shots.map((x) => x.t);
  const inside = balls.filter((b) => { const p = b.translation(); return p.y < 3 && p.x < 0.66 && p.x > -0.36 && p.y > 0.12; });
  const left = balls.filter((b) => b.translation().y < 3 && !inside.includes(b)).map((b) => { const p = b.translation(); return [p.x, p.y, p.z]; });
  w.free();
  return { timeline, shots: s.length, rate: s.length > 2 ? (s.length - 1) / (s[s.length - 1] - s[0]) : 0, inside: inside.length, minIntake: 100 * minIntake, left };
}

const pick = process.argv.slice(2);
for (const [name, fn] of Object.entries(tests)) if (!pick.length || pick.includes(name)) fn();
