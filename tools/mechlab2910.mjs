// 2910's mechanisms (js/mech/r2910.js) on their own, no field, no robot: the CAD's ball path with
// its rollers, measured against what the team says it does. Run:
//   node --import ./tools/node-env.mjs tools/mechlab2910.mjs [test ...]
// Tests: rollers (each roller's free speed), shooter (FUEL fed up the lift at their table's motor
// speeds and hood angles: how fast and how steep they leave, and whether that reaches the HUB from
// the table's distance), path (FUEL on a treadmill carpet, the robot driving into them at 1 m/s:
// intake, floor, lift, drum; how many and how fast), hold (how many it holds: filled a layer at a
// time, the lift staged as their PRELOADING does), dump (a full one fired, staged first).
// Env: SEEDS (6), DRUM (the drum's turns per motor turn, to try others than the CAD's ~24:42)
import { RAPIER } from '../js/physics.js';
import { FUEL, GROUP, groups, HUB } from '../js/constants.js';
import { Mechanism } from '../js/mech/mechanism.js';
import { sim2D } from '../js/ballistics.js';
import {
  build2910, wheels2910, pathSurfaces2910, placeIntake, placeHood, hoodElevation, shotFor, SHOT_TABLE, PASS_TABLE, passFor,
  INTAKE, SHOOTER, FLOOR, DRUM, PIVOT, RANGES,
} from '../js/mech/r2910.js';
import PATH from '../js/mech/path2910.js';
import { profile, PIVOT_MM } from '../js/mech/phys2910.js';

await RAPIER.init();
const R = FUEL.radius, DT = 1 / 120, SEEDS = Number(process.env.SEEDS ?? 6);
if (process.env.DRUM) SHOOTER.drumPerMotor = Number(process.env.DRUM);
const HELD = groups(GROUP.HELD, 0xffff & ~GROUP.HELD);

function world({ carpet = true } = {}) {
  const w = new RAPIER.World({ x: 0, y: -9.81, z: 0 });
  w.timestep = DT;
  w.integrationParameters.numSolverIterations = 6;
  // the carpet, under and around the robot
  if (carpet) w.createCollider(RAPIER.ColliderDesc.cuboid(5, 0.01, 5).setTranslation(0, -0.01, 0).setFriction(0.3).setRestitution(0.2), w.createRigidBody(RAPIER.RigidBodyDesc.fixed()));
  return w;
}
function ball(w, x, y, z, v = [0, 0, 0]) {
  const b = w.createRigidBody(RAPIER.RigidBodyDesc.dynamic().setTranslation(x, y, z).setLinvel(...v).setCcdEnabled(true).setAngularDamping(FUEL.angularDamping));
  w.createCollider(RAPIER.ColliderDesc.ball(R).setMass(FUEL.mass).setFriction(FUEL.friction).setRestitution(FUEL.restitution).setCollisionGroups(HELD), b);
  return b;
}
const med = (a) => { const b = [...a].sort((p, q) => p - q); return b[b.length >> 1]; };

// the robot's mechanisms, fixed in the world (the intake at angle deg, the hood at hood)
function rig({ deg = INTAKE.deploy, hood = 0 } = {}) {
  const t = build2910(), wh = wheels2910(t), s = pathSurfaces2910(PATH, t);
  placeIntake(wh.intake, deg);
  s.intake.setPose([PIVOT[0], PIVOT[1], 0], 0, ((INTAKE.deploy - deg) * Math.PI) / 180);
  placeHood(wh.hood, hood);
  s.hood.setPose([DRUM[0], DRUM[1], 0], 0, (hood * Math.PI) / 180);
  const pathW = [...wh.floor, ...wh.indexer, ...wh.intake];
  const path = new Mechanism({ substeps: 6, shafts: [...new Set(pathW.map((x) => x.shaft)), t.pivot], wheels: pathW, surfaces: Object.values(s) });
  const shooter = new Mechanism({ substeps: 6, sub: 8, shafts: [t.drum, t.hood], wheels: [wh.drum, ...wh.hood] });
  return { t, wh, s, path, shooter };
}
// their flywheel velocity loop toward rpm (motor)
function flywheel(t, rpm) {
  const rps = t.shooter.omega / (2 * Math.PI), target = rpm / 60;
  t.shooter.cmd = Math.max(0, Math.min(1, (SHOOTER.kV * target + SHOOTER.kP * (target - rps)) / 12));
}

// a FUEL leaving the drum: where it comes down through the HUB's rim height, back from the exit
function landing(o) {
  const hit = sim2D(o.y, o.v, (o.el * Math.PI) / 180, HUB.rimFront);
  return hit ? hit.d : NaN;
}

// their PRELOADING (js/mech/phys2910.js): the floor at 7% while the hopper's CANrange has seen FUEL
// for 0.25 s and the feeder's sees none, staging FUEL up the lift; for up to sec, until it stops.
// Returns how many FUEL are up the lift (at the feeder's beam or above)
const beam = ({ at: [x0, y0], z }, balls) => balls.some((b) => { const p = b.translation(); return Math.hypot(p.x - x0, p.y - y0) < R * 0.8 && Math.abs(p.z - z) < RANGES.reach + R; });
function stage(t, balls, step, sec = 4) {
  let fullT = 0;
  for (let i = 0; i < sec / DT; i++) {
    fullT = beam(RANGES.hopper, balls) ? fullT + DT : 0;
    t.floor.cmd = fullT > 0.25 && !beam(RANGES.feeder, balls) ? 0.07 : 0;
    step(1);
  }
  t.floor.cmd = 0;
  return balls.filter((b) => { const p = b.translation(); return p.x < -0.15 && p.y > RANGES.feeder.at[1] - R && p.y < 0.6; }).length;
}

const tests = {
  rollers() {
    const { t, path, shooter } = rig();
    t.intake.cmd = 1; t.floor.cmd = 1;
    for (let i = 0; i < 2 / DT; i++) { flywheel(t, 6000); path.step(DT, []); shooter.step(DT, []); }
    const surf = (sh, d) => Math.abs(sh.omega) * d / 2;
    console.log(`2910's rollers, free (their duty cycles; the shooter at 6000 RPM, their longest pass):`);
    console.log(`  intake X60s ${(t.intake.omega * 60 / 2 / Math.PI).toFixed(0)} rpm: 2in roller ${surf(t.main, INTAKE.main.d).toFixed(2)} m/s at its surface, 1in rollers ${surf(t.lower[0], INTAKE.lower[0].d).toFixed(2)} m/s`);
    console.log(`  hopper X60s ${(t.floor.omega * 60 / 2 / Math.PI).toFixed(0)} rpm: floor rollers ${surf(t.rollers[0], FLOOR[0][2]).toFixed(2)} m/s, indexer wheels ${surf(t.indexer, 2.25 * 0.0254).toFixed(2)} m/s`);
    console.log(`  shooter X60s ${(t.shooter.omega * 60 / 2 / Math.PI).toFixed(0)} rpm: drum ${surf(t.drum, SHOOTER.drum.d).toFixed(2)} m/s, hood rollers ${surf(t.hood, SHOOTER.hood[0].d).toFixed(2)} m/s`);
    // spin-up: from rest to their 3 m shot
    const r2 = rig();
    let T = 0;
    for (; T < 8; T += DT) { flywheel(r2.t, 2400); r2.shooter.step(DT, []); if (r2.t.shooter.omega * 60 / 2 / Math.PI > 2400 - 60) break; }
    console.log(`  the shooter from rest to 2400 RPM (their 3 m shot): ${T.toFixed(2)} s (the brass flywheel at 6.7x the drum)`);
  },
  // FUEL up the lift into the drum at each of their table's shots
  shooter() {
    console.log(`2910's shooter (drum ${SHOOTER.drumPerMotor.toFixed(3)} turns per motor turn), FUEL fed up the lift one at a time:`);
    const rows = [...SHOT_TABLE.map(([d]) => ['hub', d, shotFor(d)]), ...PASS_TABLE.map(([d]) => ['pass', d, passFor(d)])];
    for (const [kind, d, { rpm, hood }] of rows) {
      const { t, path, shooter } = rig({ hood });
      const w = world();
      t.floor.cmd = 1;
      for (let i = 0; i < 4 / DT; i++) { flywheel(t, rpm); shooter.step(DT, []); path.step(DT, []); }
      const outs = [];
      for (let k = 0; k < 4; k++) {
        const b = ball(w, -0.17, 0.205, -0.19 + 0.127 * k);
        let touched = false, out = null;
        for (let i = 0; i < 2 / DT && !out; i++) {
          flywheel(t, rpm); path.step(DT, [b], [b]); shooter.step(DT, [b]); w.step();
          if (shooter.contacts.some((c) => c.ball === b)) touched = true;
          else if (touched) { const v = b.linvel(), p = b.translation(); out = { v: Math.hypot(v.x, v.y, v.z), el: (Math.atan2(v.y, -v.x) * 180) / Math.PI, x: p.x, y: p.y }; }
        }
        if (out) outs.push(out);
        b.setTranslation({ x: 50 + k, y: 5, z: 0 }, true);
      }
      w.free();
      if (!outs.length) { console.log(`  ${kind} ${d} m (${rpm} RPM, hood ${hood}): nothing came out`); continue; }
      const o = { v: med(outs.map((q) => q.v)), el: med(outs.map((q) => q.el)), x: med(outs.map((q) => q.x)), y: med(outs.map((q) => q.y)) };
      const drum = t.drum.omega * SHOOTER.drum.d / 2;
      // (from the robot's center, d to the HUB's center; the FUEL leaves behind the center, toward it)
      const land = landing(o) - o.x, need = d;
      console.log(`  ${kind} ${d} m (${rpm} RPM, hood ${hood}): out at ${o.v.toFixed(2)} m/s (${(o.v / drum).toFixed(2)} of the drum's ${drum.toFixed(1)}), ${o.el.toFixed(1)} deg (CAD's geometry: ${hoodElevation(hood).toFixed(1)}), ` +
        (kind === 'hub' ? `comes down at the rim ${land.toFixed(2)} m back (HUB's center ${need} m: ${land - need > 0 ? '+' : ''}${(land - need).toFixed(2)})` : `lands ~${landing({ ...o, y: o.y }).toFixed(1)} m+ out`));
    }
  },
  // carpet to shot: FUEL in front, the robot driving into them; the intake down, the floor and
  // shooter running as when it shoots
  path() {
    const runs = [];
    for (let seed = 1; seed <= SEEDS; seed++) runs.push(pathRun(seed));
    console.log(`2910 carpet to shot (16 FUEL, four abreast, the robot at 1 m/s): a median ${med(runs.map((r) => r.shots))} shot (${Math.min(...runs.map((r) => r.shots))}..${Math.max(...runs.map((r) => r.shots))}), ` +
      `${med(runs.map((r) => r.rate)).toFixed(1)} FUEL/s once they're coming; ${med(runs.map((r) => r.inside))} in it at the end; intake X60s down to ${med(runs.map((r) => r.minIntake)).toFixed(0)}%, hopper X60s to ${med(runs.map((r) => r.minFloor)).toFixed(0)}%`);
    if (process.env.STUCK) for (const r of runs) console.log('   left', r.left.map((p) => p.map((v) => v.toFixed(2)).join(',')).join(' | '));
  },
  // how many it holds: the hopper filled a layer at a time (FUEL set into the space over the
  // ones already in, let settle), the floor off, until no more fit
  hold() {
    const held = rig(), { path } = held;
    const w = world();
    const balls = [];
    const step = (n) => { for (let i = 0; i < n; i++) { path.step(DT, balls, balls); w.step(); } };
    const inside = () => balls.filter((b) => { const p = b.translation(); return p.x > -0.36 && p.x < 0.66 && Math.abs(p.z) < 0.35 && p.y > 0.1 && p.y < 0.6; });
    const fill = (layers) => {
      for (let layer = 0; layer < layers; layer++) {
        for (let i = 0; i < 5; i++) for (let k = 0; k < 4; k++) {
          const x = -0.1 + 0.155 * i, z = -0.233 + 0.155 * k, y = 0.47;
          // (only where there's room: nothing within a FUEL's width already)
          if (balls.some((b) => { const p = b.translation(); return Math.hypot(p.x - x, p.y - y, p.z - z) < 2 * R; })) continue;
          balls.push(ball(w, x, y, z));
        }
        step(1.5 / DT);
      }
      step(2 / DT);
    };
    fill(8);
    const before = inside().length;
    // their PRELOADING stages FUEL up the lift, and intaking fills the room that leaves
    const lifted = stage(held.t, balls, step);
    fill(3);
    console.log(`  (${before} in the hopper alone; ${lifted} staged up the lift, the floor stopped once the feeder's CANrange saw one, and the hopper topped up again)`);
    const t0 = performance.now();
    step(1 / DT);
    const ms = (performance.now() - t0) / (1 / DT);
    console.log(`  (its mechanisms with them all in: ${ms.toFixed(2)} ms a step)`);
    console.log(`2910 holds ${inside().length} with the intake down, filled a layer at a time and the lift staged as their PRELOADING does (the team: 50-58, the ones staged in the shooter included)`);
  },
  // a full hopper dumped: filled as hold does, then the floor and the shooter at their 3 m shot
  dump() {
    const { t, wh, s: surf, path, shooter } = rig({ hood: 17 });
    const w = world();
    const balls = [];
    const step = (n, run) => { for (let i = 0; i < n; i++) { if (run) run(); path.step(DT, balls, balls); shooter.step(DT, balls); w.step(); } };
    // (their code retracts the intake while it shoots, its rollers at 15%: the pivot's X60 under
    // their Motion Magic, as js/mech/phys2910.js runs it, pushing on the load)
    let deg = INTAKE.deploy, set = [deg, 0];
    const retract = () => {
      flywheel(t, 2400);
      set = profile(...set, INTAKE.retract, PIVOT_MM.v, PIVOT_MM.a, DT);
      const volts = PIVOT_MM.kV * set[1] + PIVOT_MM.kP * (set[0] - deg);
      t.pivot.cmd = Math.max(-1, Math.min(1, -volts / 12));
      const ang = INTAKE.cadDeg * Math.PI / 180 + ((INTAKE.deploy - deg) * Math.PI) / 180;
      t.pivot.impulse(-INTAKE.mass * 9.81 * INTAKE.com * Math.cos(ang) * DT);
      deg -= (t.pivot.omega * DT * 180) / Math.PI;
      deg = Math.min(INTAKE.deploy, deg);
      placeIntake(wh.intake, deg);
      surf.intake.setPose([PIVOT[0], PIVOT[1], 0], 0, ((INTAKE.deploy - deg) * Math.PI) / 180);
    };
    for (let layer = 0; layer < 8; layer++) {
      for (let i = 0; i < 5; i++) for (let k = 0; k < 4; k++) {
        const x = -0.1 + 0.155 * i, z = -0.233 + 0.155 * k, y = 0.47;
        if (balls.some((b) => { const p = b.translation(); return Math.hypot(p.x - x, p.y - y, p.z - z) < 2 * R; })) continue;
        balls.push(ball(w, x, y, z));
      }
      step(1.5 / DT, () => flywheel(t, 2400));
    }
    // (staged up the lift first, as it would be between shots: NOSTAGE=1 skips it)
    if (!process.env.NOSTAGE) console.log(`  (${stage(t, balls, (n) => step(n, () => flywheel(t, 2400)))} staged up the lift before it fires)`);
    const n0 = balls.length, touched = new Set(), shots = [];
    t.floor.cmd = 1; t.intake.cmd = 0.15;
    for (let k = 0; k < 6 / DT; k++) {
      step(1, process.env.NORETRACT ? () => flywheel(t, 2400) : retract);
      for (const b of balls) {
        if (shooter.contacts.some((c) => c.ball === b)) touched.add(b);
        else if (touched.has(b)) { touched.delete(b); if (b.linvel().y > 2) { shots.push(k * DT); b.setTranslation({ x: 50 + shots.length, y: 5, z: 0 }, true); } }
      }
    }
    const first = shots[0] ?? 0, in1 = shots.filter((x) => x - first < 1).length;
    const left = balls.filter((b) => b.translation().y < 3 && b.translation().x < 40);
    const inHopper = left.filter((b) => { const p = b.translation(); return p.x > -0.36 && p.x < 0.66 && Math.abs(p.z) < 0.35 && p.y > 0.1 && p.y < 0.6; });
    if (process.env.STUCK) for (const b of inHopper) { const p = b.translation(); console.log(`   left at ${p.x.toFixed(2)},${p.y.toFixed(2)},${p.z.toFixed(2)}: ${[...path.contacts, ...shooter.contacts].filter((c) => c.ball === b).map((c) => c.wheel.name + ' ' + c.force.toFixed(0)).join(', ')}`); }
    console.log(`  (${inHopper.length} still in it, ${left.length - inHopper.length} out of it; the arm got to ${deg.toFixed(0)} deg of its 10)`);
    console.log(`2910 dumps ${shots.length} of ${n0}: ${in1} in the first second, ${((shots.length - 1) / (shots[shots.length - 1] - first)).toFixed(1)} FUEL/s over the whole dump (the team: 23-25 typical, 30+ in short volleys, "34bps"); the flywheel down to ${(t.shooter.omega * 9.549).toFixed(0)} RPM at the end`);
  },
};

function pathRun(seed, { rpm = 2400, hood = 17, sec = 6 } = {}) {
  let sd = seed;
  const rnd = () => ((sd = (sd * 16807) % 2147483647) / 2147483647 - 0.5);
  const { t, path, shooter } = rig({ hood });
  const w = world({ carpet: false });
  // the carpet, running under the robot at 1 m/s
  const carpet = w.createRigidBody(RAPIER.RigidBodyDesc.kinematicVelocityBased().setLinvel(-1, 0, 0));
  w.createCollider(RAPIER.ColliderDesc.cuboid(15, 0.01, 1).setTranslation(0.45 + 15, -0.01, 0).setFriction(0.3).setRestitution(0.2), carpet);
  w.createCollider(RAPIER.ColliderDesc.cuboid(0.4, 0.01, 1).setTranslation(0, -0.01, 0).setFriction(0.3), w.createRigidBody(RAPIER.RigidBodyDesc.fixed()));
  t.intake.cmd = 1; t.floor.cmd = 1;
  for (let i = 0; i < 4 / DT; i++) { flywheel(t, rpm); shooter.step(DT, []); path.step(DT, []); }
  const free = { intake: Math.abs(t.intake.omega), floor: Math.abs(t.floor.omega) };
  const balls = [];
  for (let i = 0; i < 16; i++) balls.push(ball(w, 0.8 + 0.17 * Math.floor(i / 4) + 0.02 * rnd(), R + 0.003, (i % 4 - 1.5) * 0.16 + 0.02 * rnd(), [-1, 0, 0]));
  const touched = new Set(), shots = [];
  let minIntake = 1, minFloor = 1;
  for (let k = 0; k < sec / DT; k++) {
    const live = balls.filter((b) => b.translation().y < 3);
    flywheel(t, rpm);
    path.step(DT, live, live); shooter.step(DT, live); w.step();
    minIntake = Math.min(minIntake, Math.abs(t.intake.omega) / free.intake);
    minFloor = Math.min(minFloor, Math.abs(t.floor.omega) / free.floor);
    for (const b of live) {
      if (shooter.contacts.some((c) => c.ball === b)) touched.add(b);
      else if (touched.has(b)) { touched.delete(b); const v = b.linvel(); if (v.y > 2) { shots.push(k * DT); b.setTranslation({ x: 50 + shots.length, y: 5, z: 0 }, true); } }
    }
  }
  const inside = balls.filter((b) => { const p = b.translation(); return p.y < 3 && p.x < 0.66 && p.x > -0.35 && p.y > 0.1; });
  const left = inside.map((b) => { const p = b.translation(); return [p.x, p.y, p.z]; });
  w.free();
  return { shots: shots.length, rate: shots.length > 2 ? (shots.length - 1) / (shots[shots.length - 1] - shots[0]) : 0, inside: inside.length, minIntake: 100 * minIntake, minFloor: 100 * minFloor, left };
}

const pick = process.argv.slice(2);
for (const [name, fn] of Object.entries(tests)) if (!pick.length || pick.includes(name)) fn();
