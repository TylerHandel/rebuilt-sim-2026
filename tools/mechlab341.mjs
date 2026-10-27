// 341's mechanisms (js/mech/r341.js) on their own, no field, no robot: the CAD's ball path with
// its rollers, measured against what the team's code expects. Run:
//   node --import ./tools/node-env.mjs tools/mechlab341.mjs [test ...]
// Tests: geometry (where the arm's rollers are, down), rollers (each roller's free speed, the
// flywheel's spin-up), shooter (FUEL fed up the uptake at their tables' RPM and hood angles: how
// fast and how steep they leave, and whether that reaches the HUB from the table's distance),
// path (FUEL on a treadmill carpet, the robot driving into them at 1 m/s, shooting at 3 m: intake,
// serializer, uptake, shooter; how many and how fast), hold (how many it holds, everything off).
// Env: SEEDS (6), SPEED (1 m/s), ARM (their angle down, 149), TRACE (a FUEL's index to follow),
// STUCK (where the ones left over are), LEAK (how ones get out other than shot), EXIT (the exit/flywheel-surface ratio to report against)
import { RAPIER } from '../js/physics.js';
import { FUEL, GROUP, groups, HUB } from '../js/constants.js';
import { Mechanism } from '../js/mech/mechanism.js';
import { sim2D } from '../js/ballistics.js';
import {
  build341, wheels341, shooterWheels, pathSurfaces341, placeIntake, intakeRot, placeShooter, placeTurretSurfaces, hoodElevation,
  shotFor, REGRESSIONS, INTAKE, SHOOTER, OMNI, PIVOT, TURRET, HALF,
} from '../js/mech/r341.js';
import PATH from '../js/mech/path341.js';
import { EXIT_PER_FLY_SURFACE } from '../js/mech/phys341.js';

await RAPIER.init();
const R = FUEL.radius, DT = 1 / 120, SEEDS = Number(process.env.SEEDS ?? 6), SPEED = Number(process.env.SPEED ?? 1);
const HELD = groups(GROUP.HELD, 0xffff & ~GROUP.HELD);
const DOWN = Number(process.env.ARM ?? INTAKE.down);

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

// the robot's mechanisms, fixed in the world: the intake at their angle deg, the turret facing
// forward, the hood at their angle hood
function rig({ deg = DOWN, hood = 0, yaw = 0 } = {}) {
  const t = build341(), w = wheels341(t), s = pathSurfaces341(PATH, t), sw = shooterWheels(t);
  placeIntake(w.arm, deg);
  s.intake.setPose([PIVOT[0], PIVOT[1], 0], 0, intakeRot(deg));
  placeTurretSurfaces(s, yaw, hood);
  placeShooter(sw, yaw, hood);
  const pathW = [...w.pivotRow, ...w.arm, ...w.table, ...w.uptake, ...w.omnis];
  const path = new Mechanism({ substeps: 6, shafts: [...new Set(pathW.map((x) => x.shaft)), t.pivot], wheels: pathW, surfaces: Object.values(s) });
  const shooter = new Mechanism({ substeps: 6, sub: 8, shafts: [t.fly, t.stealth, t.feed], wheels: sw });
  for (const tr of [t.intake, t.serializer, t.feederT, t.shooter]) tr.cmd = null;
  // (the arm held where it is)
  t.pivot.inertia = 1e6;
  return { t, w, s, sw, path, shooter };
}
// their flywheel velocity loop toward rpm (motor RPM, direct to the flywheel), no reverse volts
function fly(t, rpm) {
  const rps = t.shooter.omega / (2 * Math.PI), tgt = rpm / 60;
  t.shooter.cmd = Math.max(0, Math.min(1, (SHOOTER.kV * tgt + SHOOTER.kP * (tgt - rps)) / 12));
}
// a FUEL leaving the flywheel: where it comes down through the HUB's rim height, ahead of the exit
function landing(o) {
  const hit = sim2D(o.y, o.v, (o.el * Math.PI) / 180, HUB.rimFront);
  return hit ? hit.d : NaN;
}
// a FUEL that touched the shooter's wheels and has left them going up: shot
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
          if (v.y > 1) { shots.push({ t, v: Math.hypot(v.x, v.y, v.z), el: (Math.atan2(v.y, Math.hypot(v.x, v.z)) * 180) / Math.PI, x: p.x, y: p.y, b }); b.setTranslation({ x: 50 + shots.length, y: 5, z: 0 }, true); }
        }
      }
    },
  };
}
const inRobot = (b) => { const p = b.translation(); return p.x > -HALF - 0.02 && p.x < HALF + 0.33 && Math.abs(p.z) < HALF + 0.02 && p.y > 0.1 && p.y < 0.8; };

const tests = {
  geometry() {
    const { w } = rig();
    console.log(`341's intake at their ${DOWN} deg (robot frame, m): the frame's front ${HALF.toFixed(3)}`);
    for (const x of w.arm) console.log(`  ${x.name.padEnd(34)} at ${x.center[0].toFixed(3)}, ${x.center[1].toFixed(3)} (r ${x.radius.toFixed(4)}; its bottom ${(x.center[1] - x.radius).toFixed(3)} up)`);
    const [b, tp] = w.arm, d = Math.hypot(b.center[0] - tp.center[0], b.center[1] - tp.center[1]) - b.radius - tp.radius;
    console.log(`  the gap between the two powered rollers: ${d.toFixed(3)} (a FUEL ${(2 * R).toFixed(3)})`);
  },
  rollers() {
    const { t, path, shooter } = rig();
    t.intake.cmd = 1; t.serializer.cmd = 1; t.feederT.cmd = 3 / 12;
    for (let i = 0; i < 2 / DT; i++) { fly(t, 2200); path.step(DT, []); shooter.step(DT, []); }
    const surf = (sh, d) => (Math.abs(sh.omega) * d) / 2;
    console.log(`341's rollers, free (12 V; the feeder at their 3 V; the flywheel at 2200 RPM):`);
    console.log(`  intake X60s ${RPM(t.intake.omega).toFixed(0)} rpm: rollers ${surf(t.bottom, INTAKE.bottom.d).toFixed(2)} m/s at their surface, the pivot's wheels ${surf(t.pivotRoller, 2.25 * 0.0254).toFixed(2)}`);
    console.log(`  top-roller X60s ${RPM(t.serializer.omega).toFixed(0)} rpm: top rollers ${surf(t.tableA, 2.25 * 0.0254).toFixed(2)} m/s, uptake ${surf(t.upBack, 1.625 * 0.0254).toFixed(2)}`);
    console.log(`  feeder Falcon ${RPM(t.feederT.omega).toFixed(0)} rpm: the 6in omni ${surf(t.feeder, OMNI.d).toFixed(2)} m/s`);
    console.log(`  shooter X60s ${RPM(t.shooter.omega).toFixed(0)} rpm: flywheel ${surf(t.fly, SHOOTER.fly.d).toFixed(2)} m/s, 1in wheel ${surf(t.feed, SHOOTER.feed.d).toFixed(2)}, Stealth wheels ${surf(t.stealth, SHOOTER.hoodD).toFixed(2)}`);
    for (const target of [2000, 3245]) {
      const r2 = rig();
      let T = 0;
      for (; T < 8; T += DT) { fly(r2.t, target); r2.shooter.step(DT, []); if (RPM(r2.t.shooter.omega) > target - SHOOTER.tol) break; }
      console.log(`  the flywheel from rest to within ${SHOOTER.tol} of ${target} RPM: ${T.toFixed(2)} s`);
    }
  },
  // FUEL up the uptake into the shooter at their tables' rows
  shooter() {
    console.log(`341's shooter, FUEL pushed up the uptake one at a time (top rollers and uptake 12 V, the flywheel at the table's RPM):`);
    const rows = [...REGRESSIONS.rpm.map(([d]) => ['hub', d]).filter(([, d]) => d < 10), ...REGRESSIONS.feedRpm.map(([d]) => ['ferry', d])];
    if (process.env.SHOT) { const [rpm, hood] = process.env.SHOT.split(',').map(Number); rows.splice(0, rows.length, ['custom', 0, { rpm, hood }]); }
    const ratios = [];
    for (const [kind, d, custom] of rows) {
      const want = custom ?? shotFor(kind, d);
      const { t, path, shooter } = rig({ hood: want.hood });
      const w = world();
      for (let i = 0; i < 3 / DT; i++) { fly(t, want.rpm); shooter.step(DT, []); }
      t.serializer.cmd = 1;
      const balls = [];
      const watch = shotWatch(shooter);
      for (let i = 0; i < 3 / DT && watch.shots.length < 3; i++) {
        if (i % Math.round(0.6 / DT) === 0 && balls.length < 3) balls.push(ball(w, -0.10, 0.36, TURRET[1], [0, 1.5, 0]));
        const live = balls.filter((b) => b.translation().y < 3);
        fly(t, want.rpm);
        path.step(DT, live, live); shooter.step(DT, live); w.step();
        watch.check(live, i * DT);
        if (process.env.TRACE && i % 6 === 0 && balls[0]) {
          const b = balls[0], p = b.translation(), v = b.linvel();
          console.log(`   ${(i * DT).toFixed(2)} s at ${p.x.toFixed(3)},${p.y.toFixed(3)},${p.z.toFixed(3)} v ${v.x.toFixed(2)},${v.y.toFixed(2)},${v.z.toFixed(2)}: ${[...path.contacts, ...shooter.contacts].filter((c) => c.ball === b).map((c) => c.wheel.name + ' ' + c.force.toFixed(0)).join(', ')}`);
        }
      }
      w.free();
      const outs = watch.shots;
      if (!outs.length) { console.log(`  ${kind} ${d} m (${want.rpm.toFixed(0)} RPM, hood ${want.hood.toFixed(1)}): nothing came out`); continue; }
      const o = { v: med(outs.map((q) => q.v)), el: med(outs.map((q) => q.el)), x: med(outs.map((q) => q.x)), y: med(outs.map((q) => q.y)) };
      const surf = (Math.abs(t.fly.omega) * SHOOTER.fly.d) / 2;
      ratios.push(o.v / surf);
      const land = landing(o) + (o.x - TURRET[0]);
      console.log(`  ${kind} ${d} m (${want.rpm.toFixed(0)} RPM, hood ${want.hood.toFixed(1)}): ${outs.length}/3 out at ${o.v.toFixed(2)} m/s (${(o.v / surf).toFixed(2)} of the flywheel's ${surf.toFixed(1)}; the estimate ${EXIT_PER_FLY_SURFACE}), ${o.el.toFixed(1)} deg (${hoodElevation(want.hood).toFixed(1)} by the CAD's geometry), ` +
        (kind === 'hub' ? `comes down at the rim ${land.toFixed(2)} m out (the HUB's center ${d} m: ${land - d > 0 ? '+' : ''}${(land - d).toFixed(2)})` : `comes down to the carpet ~${(sim2D(o.y, o.v, (o.el * Math.PI) / 180, 0.1)?.d + o.x - TURRET[0]).toFixed(1)} m out`));
    }
    if (ratios.length) console.log(`  exit speed / flywheel surface speed: median ${med(ratios).toFixed(3)}`);
  },
  path() {
    const runs = [];
    for (let seed = 1; seed <= SEEDS; seed++) runs.push(pathRun(seed));
    console.log(`341 carpet to shot (18 FUEL, three abreast, the robot at ${SPEED} m/s, everything on, shooting at 3 m): a median ${med(runs.map((r) => r.shots))} shot (${Math.min(...runs.map((r) => r.shots))}..${Math.max(...runs.map((r) => r.shots))}), ` +
      `${med(runs.map((r) => r.rate)).toFixed(1)} FUEL/s once they're coming; ${med(runs.map((r) => r.inside))} in it at the end; in each second ${runs[0].timeline.join(', ')} (seed 1); intake X60s down to ${med(runs.map((r) => r.minIntake)).toFixed(0)}%, top rollers to ${med(runs.map((r) => r.minSer)).toFixed(0)}%`);
    if (process.env.STUCK) for (const r of runs) console.log('   left', r.left.map((p) => p.map((v) => v.toFixed(2)).join(',')).join(' | '));
  },
  // intake only (the rollers and top rollers on, nothing fed): how many it takes in
  intake() {
    for (let seed = 1; seed <= Math.min(SEEDS, 3); seed++) {
      const r = pathRun(seed, { shoot: false, n: 18, sec: 5 });
      console.log(`341 intaking only (rollers and top rollers 12 V, no feeder), 18 FUEL at ${SPEED} m/s: ${r.inside} in it after 5 s (in it each second: ${r.timeline.join(', ')}); intake X60s down to ${r.minIntake.toFixed(0)}%`);
      if (process.env.STUCK) console.log('   left', r.left.map((p) => p.map((v) => v.toFixed(2)).join(',')).join(' | '));
    }
  },
  hold() {
    const { path } = rig();
    const w = world();
    const balls = [];
    const step = (n) => { for (let i = 0; i < n; i++) { path.step(DT, balls, balls); w.step(); } };
    for (let layer = 0; layer < 4; layer++) {
      for (let i = 0; i < 5; i++) for (let k = 0; k < 4; k++) {
        const x = -0.05 + 0.155 * i, z = -0.233 + 0.155 * k, y = 0.45;
        if (balls.some((b) => { const p = b.translation(); return Math.hypot(p.x - x, p.y - y, p.z - z) < 2 * R; })) continue;
        balls.push(ball(w, x, y, z));
      }
      step(1.5 / DT);
    }
    step(2 / DT);
    const ins = balls.filter(inRobot);
    console.log(`341 holds ${ins.length} with everything off and the intake down (${ins.filter((b) => b.translation().x > HALF).length} of them out on the arm)`);
  },
};

function pathRun(seed, { shoot = true, n = 18, sec = 7 } = {}) {
  let sd = seed;
  const rnd = () => ((sd = (sd * 16807) % 2147483647) / 2147483647 - 0.5);
  const want = shotFor('hub', 3.0);
  const { t, path, shooter } = rig({ hood: want.hood });
  const w = world({ carpet: false });
  // the carpet, running under the robot
  const carpet = w.createRigidBody(RAPIER.RigidBodyDesc.kinematicVelocityBased().setLinvel(-SPEED, 0, 0));
  w.createCollider(RAPIER.ColliderDesc.cuboid(15, 0.01, 1).setTranslation(0.45 + 15, -0.01, 0).setFriction(0.3).setRestitution(0.2), carpet);
  w.createCollider(RAPIER.ColliderDesc.cuboid(0.45, 0.01, 1).setTranslation(0, -0.01, 0).setFriction(0.3), w.createRigidBody(RAPIER.RigidBodyDesc.fixed()));
  t.intake.cmd = 1; t.serializer.cmd = 1;
  if (shoot) t.feederT.cmd = 3 / 12;
  const run = () => { if (shoot) fly(t, want.rpm); };
  for (let i = 0; i < 3 / DT; i++) { run(); shooter.step(DT, []); path.step(DT, []); }
  const free = { intake: Math.abs(t.intake.omega), ser: Math.abs(t.serializer.omega) || 1 };
  const balls = [];
  for (let i = 0; i < n; i++) balls.push(ball(w, 0.85 + 0.17 * Math.floor(i / 3) + 0.02 * rnd(), R + 0.003, (i % 3 - 1) * 0.18 + 0.02 * rnd(), [-SPEED, 0, 0]));
  const watch = shotWatch(shooter);
  let minIntake = 1, minSer = 1;
  const timeline = [];
  for (let k = 0; k < sec / DT; k++) {
    const live = balls.filter((b) => b.translation().y < 3);
    run();
    path.step(DT, live, live); shooter.step(DT, live); w.step();
    minIntake = Math.min(minIntake, Math.abs(t.intake.omega) / free.intake);
    minSer = Math.min(minSer, Math.abs(t.serializer.omega) / free.ser);
    watch.check(live, k * DT);
    // (LEAK=1: where and how FUEL get out of it other than by the shooter)
    if (process.env.LEAK) for (const [i, b] of balls.entries()) {
      const p = b.translation(), v = b.linvel(), last = (b.last ??= { out: false, c: '' });
      const c = [...path.contacts, ...shooter.contacts].filter((q) => q.ball === b).map((q) => q.wheel.name).join(', ');
      if (c) last.c = `${(k * DT).toFixed(2)} s at ${p.x.toFixed(2)},${p.y.toFixed(2)},${p.z.toFixed(2)}: ${c}`;
      if (!last.out && p.y < 3 && (Math.abs(p.z) > HALF + 0.12 || p.x < -HALF - 0.12 || p.y < -0.05) && last.c) { last.out = true; console.log(`   FUEL ${i} out at ${p.x.toFixed(2)},${p.y.toFixed(2)},${p.z.toFixed(2)} v ${v.x.toFixed(1)},${v.y.toFixed(1)},${v.z.toFixed(1)}; last touched ${last.c}`); }
    }
    if (k % Math.round(1 / DT) === Math.round(1 / DT) - 1) timeline.push(balls.filter((b) => { const p = b.translation(); return p.y < 3 && p.x < HALF && p.y > 0.15; }).length);
    if (process.env.TRACE && k % 6 === 0) {
      const b = balls[Number(process.env.TRACE)], p = b.translation(), v = b.linvel();
      console.log(`   ${(k * DT).toFixed(2)} s at ${p.x.toFixed(3)},${p.y.toFixed(3)},${p.z.toFixed(3)} v ${v.x.toFixed(2)},${v.y.toFixed(2)},${v.z.toFixed(2)}: ${[...path.contacts, ...shooter.contacts].filter((c) => c.ball === b).map((c) => c.wheel.name + ' ' + c.force.toFixed(0)).join(', ')}`);
    }
  }
  const s = watch.shots.map((x) => x.t);
  const inside = balls.filter((b) => b.translation().y < 3 && inRobot(b));
  const left = balls.filter((b) => b.translation().y < 3 && b.translation().x < 40 && !inside.includes(b)).map((b) => { const p = b.translation(); return [p.x, p.y, p.z]; });
  if (process.env.STUCK) console.log('   inside', inside.map((b) => { const p = b.translation(); return `${p.x.toFixed(2)},${p.y.toFixed(2)},${p.z.toFixed(2)} ${[...path.contacts].filter((c) => c.ball === b).map((c) => c.wheel.name.replace(/ \(.*\)/, '') + ' ' + c.force.toFixed(0)).join('+')}`; }).join(' | '));
  w.free();
  return { timeline, shots: s.length, rate: s.length > 2 ? (s.length - 1) / (s[s.length - 1] - s[0]) : 0, inside: inside.length, minIntake: 100 * minIntake, minSer: 100 * minSer, left };
}

const pick = process.argv.slice(2);
for (const [name, fn] of Object.entries(tests)) if (!pick.length || pick.includes(name)) fn();
