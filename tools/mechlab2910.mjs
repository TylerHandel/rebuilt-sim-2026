// 2910's mechanisms (js/mech/r2910.js) on their own, no field, no robot: the CAD's ball path with
// its rollers, measured against what the team says it does. Run:
//   node --import ./tools/node-env.mjs tools/mechlab2910.mjs [test ...]
// Tests: rollers (each roller's free speed), shooter (FUEL fed up the lift at their table's motor
// speeds and hood angles: how fast and how steep they leave, and whether that reaches the HUB from
// the table's distance), tune (their tables' rows, sixteen FUEL fed as in play: the HUB's where they
// come down, and the passing table's speeds re-found: js/mech/r2910.js SIM_PASS_TABLE; MAXRPM,
// ROWS, IDLE_RPM, SETTLED, TUNELOG), path (FUEL on a treadmill carpet, the robot driving into them at 1 m/s:
// intake, floor, lift, drum; how many and how fast), hold (how many it holds: filled a layer at a
// time, the lift staged as their PRELOADING does), intakefull (filled through the intake from the
// NEUTRAL ZONE's FUEL until it can't take more: tools/fillbench.mjs), dump (a full one fired,
// staged first).
// Env: SEEDS (6; dump's 3), SPEED (intakefull's 1 m/s), SEC (its 10 s), AUTO (intakefull and dump
// with the intake rollers' AUTO current limits, not teleop's), dump's FILL=layers, NOSTAGE,
// NORETRACT, NOGATE (the floor feeding whatever the drum's speed), BINS (shots each quarter
// second), TRACE (the drum, floor and arm), LIFTLOG (how many FUEL are up the lift), DRUM (the drum's turns per motor turn, to try others than the CAD's ~24:42)
import { RAPIER } from '../js/physics.js';
import { FUEL, GROUP, groups, HUB } from '../js/constants.js';
import { Mechanism } from '../js/mech/mechanism.js';
import { sim2D } from '../js/ballistics.js';
import {
  build2910, wheels2910, pathSurfaces2910, placeIntake, placeHood, hoodElevation, shotFor, SHOT_TABLE, PASS_TABLE, THEIR_PASS_TABLE, passFor, intakeLimits,
  INTAKE, SHOOTER, FLOOR, DRUM, PIVOT, RANGES,
} from '../js/mech/r2910.js';
import PATH from '../js/mech/path2910.js';
import { profile, PIVOT_MM } from '../js/mech/phys2910.js';
import { BUMPER_Y0 } from '../js/robotConfigs.js';
import { fillRun, fillReport } from './fillbench.mjs';

await RAPIER.init();
const R = FUEL.radius, DT = 1 / 120, SEEDS = Number(process.env.SEEDS ?? 6), SPEED = Number(process.env.SPEED ?? 1);
if (process.env.DRUM) SHOOTER.drumPerMotor = Number(process.env.DRUM);
const HELD = groups(GROUP.HELD, 0xffff & ~GROUP.HELD);
// (a FUEL comes down through the HUB's hexagonal opening when its center's within the opening's
// inradius less its own radius of the HUB's center, along the line of the shot)
const HUB_R = HUB.hexR * Math.cos(Math.PI / 6) - R;

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
  // their tables' rows shot as play feeds them: the drum from its idle to the row's speed, and
  // once it's within 5% (as phys2910 gates a shot) sixteen FUEL (ROWS rows of four, from where the
  // preload sits forward) fed by the floor at full (their SHOOT) while it stays within it; where they come down, from the
  // robot's center: the HUB's rows through the rim's height (at their RPMs, and the pass rows'
  // re-found: at their PASSING_ANGLE, the motor RPM at which they come down on the carpet at the
  // row's distance, a search between 600 RPM and MAXRPM, 5600, about what the drum gets to
  // on its 35 A); prints what js/mech/r2910.js keeps as SIM_PASS_TABLE
  tune() {
    const MAX = Number(process.env.MAXRPM ?? 5600), SETTLED = !!process.env.SETTLED, ROWS = Number(process.env.ROWS ?? 4), IDLE_RPM = Number(process.env.IDLE_RPM ?? 2378);
    const shoot = (rpm, hood, H, idle = IDLE_RPM) => {
      const { t, path, shooter } = rig({ hood });
      const w = world();
      // (from its idle, 82% of the HUB shot's speed for the distance to the HUB: IDLE_RPM, 2378
      // from 5 m on; SETTLED=1: the drum held at the row's speed until it's there, instead)
      const now = () => (t.shooter.omega * 60) / 2 / Math.PI, within = () => Math.abs(now() - rpm) <= 0.05 * rpm;
      if (SETTLED) {
        for (let i = 0, last = 0; i < 15 / DT; i++) {
          flywheel(t, rpm); shooter.step(DT, []); path.step(DT, []);
          if (i % 12 === 0) { if (Math.abs(now() - rpm) < 0.01 * rpm || (i > 0 && now() - last < 0.002 * rpm)) break; last = now(); }
        }
      } else {
        for (let i = 0; i < 6 / DT; i++) { flywheel(t, idle); shooter.step(DT, []); }
        for (let i = 0; i < 15 / DT && !within(); i++) { flywheel(t, rpm); shooter.step(DT, []); path.step(DT, []); }
      }
      const balls = [[-0.17, 0.205], [-0.02, 0.225], [0.13, 0.26], [0.28, 0.3]].slice(0, ROWS).flatMap(([x, y]) => [-0.19, -0.063, 0.063, 0.19].map((z) => ball(w, x, y, z)));
      const touched = new Set(), outs = [];
      for (let i = 0; i < 6 / DT && outs.length < balls.length; i++) {
        // (their SHOOT while the drum's within 5% of the shot's speed, as play gates it)
        t.floor.cmd = SETTLED || within() ? 1 : 0;
        flywheel(t, rpm); path.step(DT, balls, balls); shooter.step(DT, balls); w.step();
        for (const b of balls) {
          if (shooter.contacts.some((c) => c.ball === b)) touched.add(b);
          else if (touched.has(b)) {
            touched.delete(b);
            const v = b.linvel(), p = b.translation();
            // (up and back over the drum: not one kicked forward off it by the one ahead)
            if (v.y > 2 && v.x < 0) { outs.push({ drum: now() / rpm, v: Math.hypot(v.x, v.y, v.z), el: (Math.atan2(v.y, -v.x) * 180) / Math.PI, x: p.x, y: p.y }); b.setTranslation({ x: 50 + outs.length, y: 5, z: 0 }, true); }
          }
        }
      }
      w.free();
      const L = outs.map((o) => { const h = sim2D(o.y, o.v, (o.el * Math.PI) / 180, H); return h ? h.d - o.x : NaN; }).filter(Number.isFinite);
      return { land: L.length ? L.reduce((a, b) => a + b, 0) / L.length : NaN, n: L.length, spread: L.length ? Math.max(...L) - Math.min(...L) : NaN, v: med(outs.map((o) => o.v)), drum: med(outs.map((o) => o.drum)) };
    };
    for (const [d, rpm, hood] of SHOT_TABLE) {
      const s = shoot(rpm, hood, HUB.rimFront, 0.82 * rpm);
      console.log(`  hub ${d} m, hood ${hood}, ${rpm} RPM: ${s.n}/${4 * ROWS} out at ${s.v.toFixed(2)} m/s (the drum at ${s.drum.toFixed(3)} of it), come down through the rim's height ${s.land.toFixed(2)} m back (${s.land - d > 0 ? '+' : ''}${(s.land - d).toFixed(2)}; spread ${s.spread.toFixed(2)})`);
    }
    const out = [], hood = SHOOTER.hoodMax;
    for (const [d, rpm0] of THEIR_PASS_TABLE) {
      const theirs = shoot(Math.min(rpm0, MAX), hood, R);
      // (bracketed, then regula falsi: past ~4000 RPM the FUEL slip on the drum and the
      // landing barely moves with the speed, where a secant runs off)
      let lo = 600, flo = shoot(lo, hood, R).land, hi = MAX, fhi = shoot(hi, hood, R).land, b = hi, lb = fhi;
      if (!(flo < d)) flo = 0;
      for (let j = 0; j < 14 && fhi > d && Math.abs(lb - d) > 0.03 && hi - lo > 5; j++) {
        b = lo + ((d - flo) * (hi - lo)) / (fhi - flo);
        b = Math.max(lo + 0.1 * (hi - lo), Math.min(hi - 0.1 * (hi - lo), b));
        lb = shoot(b, hood, R).land;
        if (process.env.TUNELOG) console.log(`    ${b.toFixed(0)} RPM: ${lb.toFixed(2)} m`);
        if (!(lb >= d)) { lo = b; flo = Number.isFinite(lb) ? lb : 0; } else { hi = b; fhi = lb; }
      }
      const ok = Number.isFinite(lb) && Math.abs(lb - d) < 0.1;
      const best = ok ? Math.round(b / 5) * 5 : MAX, at = ok ? lb : fhi;
      out.push([d, best]);
      console.log(`  pass ${d} m, hood ${hood}: their ${rpm0} RPM${rpm0 > MAX ? ` (flat out, ${MAX})` : ''} lands ${theirs.land.toFixed(2)} m (${theirs.land - d > 0 ? '+' : ''}${(theirs.land - d).toFixed(2)}); ${ok ? `(the drum at ${shoot(b, hood, R).drum.toFixed(3)} of it) ` : ''}${ok ? `the sim's FUEL come down there at ${best} RPM (x${(best / rpm0).toFixed(3)})` : `out of reach: flat out (${MAX}) lands ${at.toFixed(2)} m (${(at - d).toFixed(2)})`}`);
    }
    // (past the last row it asks that row's speed: flat out carries only a little farther, and takes
    // seconds more to get to)
    console.log(`  flat out (${MAX}): ${shoot(MAX, hood, R).land.toFixed(2)} m`);
    console.log('SIM_PASS_TABLE', JSON.stringify(out));
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
  // filled through the intake until it can't take more: driving at SPEED (1 m/s) into the NEUTRAL
  // ZONE's FUEL, their INTAKE (its rollers at full) with the floor staging FUEL up the lift as
  // their PRELOADING does (tools/fillbench.mjs)
  intakefull() {
    for (let seed = 1; seed <= Math.min(SEEDS, 3); seed++) {
      const r = intakeFull(seed, { keep: !!process.env.STUCK });
      console.log(fillReport(`2910 intaking at ${SPEED} m/s, seed ${seed}`, r));
      // (STUCK=1: where they are)
      if (process.env.STUCK) {
        const ps = r.balls.map((b) => { const p = b.translation(); return [p.x, p.y, p.z]; }).filter(held2910).sort((a, b) => a[0] - b[0]);
        console.log('   ' + ps.map((p) => p.map((v) => v.toFixed(2)).join(',')).join(' | '));
        r.world.free();
      }
    }
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
  // a full hopper dumped at their 3 m shot (2400 RPM, hood 17): filled through the intake until
  // it's full (intakefull), its rollers let go (coasting) and the lift staged as their PRELOADING
  // does while it drives to the HUB (1.5 s), then fired as their code feeds a shot, the intake
  // retracting on the load (FILL=layers: filled as hold does instead)
  dump() {
    const runs = [];
    for (let seed = 1; seed <= (process.env.FILL === 'layers' ? 1 : Number(process.env.SEEDS ?? 3)); seed++) runs.push(dumpRun(seed));
    const mean = (f) => runs.reduce((a, r) => a + f(r), 0) / runs.length;
    if (runs.length > 1) console.log(`2910 dumps (mean of ${runs.length}): ${mean((r) => r.n).toFixed(1)} of ${mean((r) => r.n0).toFixed(1)}, ${mean((r) => r.rate).toFixed(1)} FUEL/s over the whole dump (${runs.map((r) => r.rate.toFixed(1)).join(', ')}), best second ${mean((r) => r.best).toFixed(1)}`);
  },
};

// a full one dumped (dump)
function dumpRun(seed) {
  const RPM = 2400, HOOD = 17, D = 3.0; // their table's 3 m shot
  let rg, w, balls;
  if (process.env.FILL === 'layers') {
    rg = rig({ hood: HOOD }); w = world(); balls = [];
  } else {
    const f = intakeFull(seed, { keep: true, hood: HOOD, until: (full) => full, sec: 12 });
    rg = f.rig; w = f.world;
    f.carpet.setLinvel({ x: 0, y: 0, z: 0 }, true);
    // (what it didn't take in stays behind on the carpet: gone)
    balls = [];
    for (const b of f.balls) { const p = b.translation(); if (own2910([p.x, p.y, p.z])) balls.push(b); else b.setEnabled(false); }
  }
  const { t, wh, s: surf, path, shooter } = rg;
  const step = (n, run) => { for (let i = 0; i < n; i++) { if (run) run(); path.step(DT, balls, balls); shooter.step(DT, balls); w.step(); } };
  // (their code retracts the intake while it shoots, its rollers at 15%: the pivot's X60 under
  // their Motion Magic, as js/mech/phys2910.js runs it, pushing on the load)
  let deg = INTAKE.deploy, set = [deg, 0], armT = 0;
  const armAt = {};
  const retract = () => {
    flywheel(t, RPM);
    set = profile(...set, INTAKE.retract, PIVOT_MM.v, PIVOT_MM.a, DT);
    const volts = PIVOT_MM.kV * set[1] + PIVOT_MM.kP * (set[0] - deg);
    t.pivot.cmd = Math.max(-1, Math.min(1, -volts / 12));
    const ang = INTAKE.cadDeg * Math.PI / 180 + ((INTAKE.deploy - deg) * Math.PI) / 180;
    t.pivot.impulse(-INTAKE.mass * 9.81 * INTAKE.com * Math.cos(ang) * DT);
    deg -= (t.pivot.omega * DT * 180) / Math.PI;
    deg = Math.min(INTAKE.deploy, deg);
    armT += DT; for (const [a, k] of [[80, 'to80'], [40, 'to40'], [12, 'to12']]) if (deg <= a && armAt[k] === undefined) armAt[k] = armT;
    if (process.env.ARMLOG && Math.round(armT / DT) % 6 === 0) console.log(`   arm ${armT.toFixed(2)} s: ${deg.toFixed(1)} deg, pivot X60 ${t.pivot.motor.current.toFixed(0)} A`);
    placeIntake(wh.intake, deg);
    surf.intake.setPose([PIVOT[0], PIVOT[1], 0], 0, ((INTAKE.deploy - deg) * Math.PI) / 180);
  };
  if (process.env.FILL === 'layers') {
    for (let layer = 0; layer < 8; layer++) {
      for (let i = 0; i < 5; i++) for (let k = 0; k < 4; k++) {
        const x = -0.1 + 0.155 * i, z = -0.233 + 0.155 * k, y = 0.47;
        if (balls.some((b) => { const p = b.translation(); return Math.hypot(p.x - x, p.y - y, p.z - z) < 2 * R; })) continue;
        balls.push(ball(w, x, y, z));
      }
      step(1.5 / DT, () => flywheel(t, RPM));
    }
  }
  // (driving to the HUB, 3 s: the drum idling at 82% of the shot's speed as it does between shots,
  // the intake's rollers coasting, the lift staged as their PRELOADING does: NOSTAGE=1 skips that;
  // then the drum up to the shot's speed, the floor waiting on it as below)
  t.intake.cmd = null;
  const idle = () => flywheel(t, 0.82 * RPM);
  const staged = process.env.NOSTAGE ? (step(3 / DT, idle), 0) : stage(t, balls, (n) => step(n, idle), 3);
  const inIt = () => balls.filter((b) => { const p = b.translation(); return held2910([p.x, p.y, p.z]); });
  const n0 = balls.filter((b) => { const p = b.translation(); return own2910([p.x, p.y, p.z]); }).length, touched = new Set(), shots = [];
  t.floor.cmd = 1; t.intake.cmd = 0.15;
  let minRpm = Infinity, gated = 0, feedT = null;
  const trace = [];
  for (let k = 0; k < 6 / DT; k++) {
    // (as in play the floor feeds only while the drum's within 5% of the shot's speed: its
    // ready, js/mech/phys2910.js; NOGATE=1 feeds regardless)
    const spunUp = Math.abs(t.shooter.omega * 9.549 - RPM) <= 0.05 * RPM;
    t.floor.cmd = spunUp || process.env.NOGATE ? 1 : 0;
    if (t.floor.cmd && feedT === null) feedT = k * DT;
    if (!spunUp && shots.length) gated += DT;
    if (process.env.TRACE && k % 6 === 0) trace.push(`${(k * DT).toFixed(2)}:${(t.shooter.omega * 9.549).toFixed(0)}/${shots.length} floor ${(t.floor.omega * 9.549).toFixed(0)} ${t.floor.motor.current.toFixed(0)}A arm ${deg.toFixed(0)} ${t.intake.motor.current.toFixed(0)}A`);
    step(1, process.env.NORETRACT ? () => flywheel(t, RPM) : retract);
    if (shots.length) minRpm = Math.min(minRpm, t.shooter.omega * 9.549);
    if (process.env.LIFTLOG && k % 6 === 0 && shots.length > 5 && shots.length < n0 - 10) {
      const lift = balls.filter((b) => { const p = b.translation(); return p.x < -0.17 && p.y > 0.2 && p.y < 0.47; });
      const base = balls.filter((b) => { const p = b.translation(); return p.x < -0.1 && p.x > -0.3 && p.y <= 0.2; });
      console.log(`   ${(k * DT).toFixed(2)} lift ${lift.length} (up at ${(lift.reduce((a, b) => a + b.linvel().y, 0) / Math.max(1, lift.length)).toFixed(2)} m/s; by lane ${[-0.19, -0.063, 0.063, 0.19].map((z) => lift.filter((b) => Math.abs(b.translation().z - z) < 0.065).length).join('')}), at its foot ${base.length}, floor ${(t.floor.omega * 9.549).toFixed(0)} RPM`);
    }
    for (const b of balls) {
      if (shooter.contacts.some((c) => c.ball === b)) touched.add(b);
      else if (touched.has(b)) {
        touched.delete(b);
        const v = b.linvel(), p = b.translation();
        if (v.y > 2) { shots.push({ t: k * DT, v: Math.hypot(v.x, v.y, v.z), el: (Math.atan2(v.y, -v.x) * 180) / Math.PI, x: p.x, y: p.y }); b.setTranslation({ x: 50 + shots.length, y: 5, z: 0 }, true); }
      }
    }
  }
  if (process.env.TRACE) console.log('   ' + trace.join(' '));
  // (BINS=1: shots each quarter second from the first)
  if (process.env.BINS) { const f0 = shots[0]?.t ?? 0, bins = []; for (const o of shots) { const i = Math.floor((o.t - f0) / 0.25); bins[i] = (bins[i] || 0) + 1; } console.log('   shots each 0.25 s: ' + Array.from(bins, (v) => v || 0).join(' ')); }
  // (a FUEL the staging pushed up into the idling drum goes before the feed starts, short: not
  // the dump's)
  const strays = shots.filter((o) => feedT === null || o.t < feedT).length;
  const ts = shots.filter((o) => o.t >= feedT).map((o) => o.t), first = ts[0] ?? 0, in1 = ts.filter((x) => x - first < 1).length;
  const best = Math.max(0, ...ts.map((a) => ts.filter((b) => b >= a && b < a + 1).length));
  // (FUEL fired over the time from the first to the last)
  const rate = ts.length / (ts[ts.length - 1] - first);
  const left = inIt();
  if (process.env.STUCK) for (const b of left) { const p = b.translation(); console.log(`   left at ${p.x.toFixed(2)},${p.y.toFixed(2)},${p.z.toFixed(2)}: ${[...path.contacts, ...shooter.contacts].filter((c) => c.ball === b).map((c) => c.wheel.name + ' ' + c.force.toFixed(0)).join(', ')}`); }
  // (where they come down at the HUB's rim height, back from the robot's center: their table's
  // shot is for the HUB's center D m away)
  const land = shots.map((o) => landing(o) - o.x).filter(Number.isFinite).sort((a, b) => a - b);
  const inHub = land.filter((d) => Math.abs(d - D) < HUB_R).length;
  console.log(`  (${staged} staged up the lift before it fires; ${left.length} still in it; the arm got to ${deg.toFixed(0)} deg of its 10: to 80 deg in ${armAt.to80?.toFixed(2)} s, 40 in ${armAt.to40?.toFixed(2)} s, 12 in ${armAt.to12?.toFixed(2)} s; the drum's motors down to ${minRpm.toFixed(0)} RPM of ${RPM}, the floor waiting on it ${gated.toFixed(2)} s; shots come down at the rim ${land[land.length >> 1]?.toFixed(2)} m back (median; ${land[0]?.toFixed(2)}..${land[land.length - 1]?.toFixed(2)}), ${inHub} of ${land.length} within the HUB's opening of its center ${D} m)`);
  console.log(`2910 dumps ${ts.length} of ${n0}${strays ? ` (and ${strays} launched by the idling drum before)` : ''}${process.env.FILL === 'layers' ? ' (filled a layer at a time)' : ` (filled through its intake, seed ${seed})`}: ${in1} in the first second, best second ${best}, ${rate.toFixed(1)} FUEL/s over the whole dump (the team: 23-25 typical, 30+ in short volleys, "34bps")`);
  w.free();
  return { n: ts.length, n0, rate, best, in1 };
}

// where a FUEL is (robot frame), as js/mech/phys2910.js where() has it: in it (not one on the carpet
// the intake's reaching for), and the intake's
const inFrame2910 = ([x, y, z]) => Math.abs(x) < 0.343 && Math.abs(z) < 0.349 && y > 0.105 && y < 0.62;
const inFront2910 = ([x, y, z]) => x >= 0.343 && x < 0.72 && Math.abs(z) < 0.349 && y > BUMPER_Y0 && y < 0.6;
const held2910 = (p) => inFrame2910(p) || (inFront2910(p) && p[1] > 0.17 && p[0] < 0.6);
const own2910 = (p) => inFrame2910(p) || inFront2910(p);
const TOP2910 = { topX: 0.1, topN: 10 }; // (js/mech/phys2910.js naturalFull)
// a fill through the intake (intakefull), returning the rig with the FUEL in it as well (dump)
function intakeFull(seed, { keep = false, sec = Number(process.env.SEC ?? 10), until = null, hood = 0 } = {}) {
  const rg = rig({ hood }), { t, path } = rg;
  // (the intake rollers' teleop current limits, as most intaking is in teleop; AUTO=1: AUTO's)
  intakeLimits(t, !process.env.AUTO);
  t.intake.cmd = 1;
  let fullT = 0, balls = null;
  const control = (_, live) => {
    balls = live;
    fullT = beam(RANGES.hopper, live) ? fullT + DT : 0;
    t.floor.cmd = fullT > 0.25 && !beam(RANGES.feeder, live) ? 0.07 : 0;
  };
  const r = fillRun(RAPIER, { world: () => world({ carpet: false }), ball, path, DT, held: held2910, own: own2910, top: TOP2910, control, speed: SPEED, sec, seed, front: 0.45, keep, until });
  return { ...r, rig: rg, balls };
}

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
