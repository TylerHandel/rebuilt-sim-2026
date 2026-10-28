// 1114's mechanisms (js/mech/r1114.js) on their own, no field, no robot: the CAD's ball path with its
// rollers, measured against what the team's code expects. Run:
//   node --import ./tools/node-env.mjs tools/mechlab1114.mjs [test ...]
// Tests: rollers (each roller's free speed at their voltages, the flywheel's spin-up, the box
// sliding out and in), shooter (FUEL fed up the elevator at their LUT's rows: how fast and how
// steep they leave, and whether their table's shots come down in the HUB), path (FUEL on a
// treadmill carpet, the robot driving into them at 1 m/s, intaking and shooting: intake, floor,
// elevator, flywheel; how many and how fast), intake (their RunIntakeBallevator alone, 24 FUEL at
// 1 and 3 m/s), intakefull (filled through the intake from the NEUTRAL ZONE's FUEL until it can't
// put more in: tools/fillbench.mjs), hold (how many it holds, dropped in, everything off), dump (a
// full one fired at their 3 m shot, with their AgitateIntake pulling the box in).
// tune (their LUT's rows re-found by shooting: js/mech/r1114.js SIM_HUB, SIM_PASS).
// Env: SEEDS (6), SPEED (path's 1 m/s; intake's 1,3), SEC (intakefull's 10 s), STUCK (where the
// ones left over are), SHOT=rpm,hood (one shot of your own), MAP=1 (shooter: a grid of RPMs and
// hoods), THEIR_RPM=1 (their LUT's own RPMs), NOAGITATE=1 (the dump without the box coming in),
// FILL=layers (dump: filled as hold does), BINS (dump: shots each quarter second), DUMPLOG (dump:
// where the load is each quarter second), DUMPSNAP=t (dump: every FUEL and what it touches at t),
// HOLDFILE / FILLFILE=path (hold / intakefull: where the FUEL are, as JSON), TRACE=i (path)
import { RAPIER } from '../js/physics.js';
import { FUEL, GROUP, groups, HUB } from '../js/constants.js';
import { Mechanism } from '../js/mech/mechanism.js';
import { sim2D } from '../js/ballistics.js';
import {
  build1114, wheels1114, pathSurfaces1114, placeSlide, placeIntake, placeHood, hoodRot, hoodElevation, shotFor, slideTurns, kickerRot,
  HUB_LUT, PASS_LUT, SLIDE, SLIDE_POS, SHOOTER, BALLEVATOR, FLOOR, ELEV, SQUISH, SIDE, INTAKE, FLY, RANGE,
} from '../js/mech/r1114.js';
import PATH from '../js/mech/path1114.js';
import { EXIT_PER_FLY_SURFACE } from '../js/mech/phys1114.js';
import { fillRun, fillReport } from './fillbench.mjs';
import { writeFileSync } from 'fs';

await RAPIER.init();
const R = FUEL.radius, DT = 1 / 120, SEEDS = Number(process.env.SEEDS ?? 6), SPEED = Number(process.env.SPEED ?? 1);
const HELD = groups(GROUP.HELD, 0xffff & ~GROUP.HELD);
const TAU = 2 * Math.PI, RPM = (w) => (w * 60) / TAU;
const HUB_R = HUB.hexR * Math.cos(Math.PI / 6) - R;

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
const mean = (a) => a.reduce((s, v) => s + v, 0) / Math.max(1, a.length);

// the robot's mechanisms, fixed in the world: the box ext m out, the hood at their angle hood
function rig({ ext = SLIDE.out, hood = SHOOTER.hoodHome } = {}) {
  const t = build1114(), wh = wheels1114(t), s = pathSurfaces1114(PATH, t);
  const pathW = [...wh.floor, ...wh.squish, ...wh.side, ...wh.passive, ...wh.elev, ...wh.wall, ...wh.intake];
  const path = new Mechanism({ substeps: 6, shafts: [...new Set(pathW.map((x) => x.shaft)), t.slide], wheels: pathW, surfaces: Object.values(s) });
  const shooter = new Mechanism({ substeps: 6, sub: 8, shafts: [t.fly, t.hood], wheels: [wh.fly, ...wh.hood] });
  for (const tr of [t.intake, t.floor, t.sideT, t.elev, t.shooter]) tr.cmd = null;
  t.slide.cmd = null;
  const r = { t, wh, s, path, shooter, ext, hood };
  r.pose = () => {
    placeSlide(s, r.ext);
    placeIntake(wh.intake, r.ext);
    s.hood.setPose([...FLY, 0], 0, hoodRot(r.hood));
    placeHood(wh.hood, r.hood);
  };
  r.pose();
  return r;
}
// their shooter velocity loop toward rpm (motor RPM), or coasting
function flywheel(t, rpm) { t.shooter.vel = rpm > 0 ? { target: (rpm / 60) * TAU, kV: SHOOTER.kV, kP: SHOOTER.kP, per: 1 / TAU } : null; if (!rpm) t.shooter.cmd = null; }
// their Shoot: elevator 4500 RPM, floor 12 V, side rollers 10 V (or all coasting)
function feed(t, on) {
  t.elev.vel = on ? { target: (BALLEVATOR.shoot / 60) * TAU, kV: BALLEVATOR.kV, kP: BALLEVATOR.kP, per: 1 / TAU } : null;
  if (!on) t.elev.cmd = null;
  t.floor.cmd = on ? 1 : null; t.sideT.cmd = on ? 10 / 12 : null;
}
// the slide: volts (0 coasts), or their position loop (kP 4 V per motor turn) toward turns, or
// torque current amps; its weight down the racks; the box moving as its pinions turn
function slide(r, { volts = null, turns = null, amps = null } = {}) {
  const T = r.t;
  if (turns !== null) T.slide.cmd = Math.max(-1, Math.min(1, (4 * (turns - slideTurns(r.ext))) / 12));
  else if (amps !== null) { const m = T.slide.motor; T.slide.cmd = Math.max(-1, Math.min(1, (amps * m.Rm + (T.slide._omega * m.ratio) / m.kV) / 12)); }
  else T.slide.cmd = volts ? volts / 12 : null;
  T.slide.impulse(SLIDE.mass * 9.81 * SLIDE.dir[1] * SLIDE.pinion * DT);
}
function moveSlide(r) {
  r.ext += r.t.slide.omega * SLIDE.pinion * DT;
  if (r.ext < 0) { r.ext = 0; if (r.t.slide._omega < 0) r.t.slide._omega = 0; }
  if (r.ext > SLIDE.out) { r.ext = SLIDE.out; if (r.t.slide._omega > 0) r.t.slide._omega = 0; }
  r.pose();
}
// a FUEL leaving the flywheel: how far forward from the robot's center it comes down through the
// HUB's rim height
function landing(o) {
  const hit = sim2D(o.y, o.v, (o.el * Math.PI) / 180, HUB.rimFront);
  return hit ? o.x + hit.d : NaN;
}
// a FUEL that touched the flywheel and has left it going up: shot
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
          if (v.y > 2) { shots.push({ t, v: Math.hypot(v.x, v.y, v.z), el: (Math.atan2(v.y, v.x) * 180) / Math.PI, x: p.x, y: p.y, z: p.z, b }); b.setTranslation({ x: 50 + shots.length, y: 5, z: 0 }, true); }
        }
      }
    },
  };
}
// in it (as js/mech/phys1114.js where() has it: in the frame, or in the box over the bumper)
const heldP = ([x, y, z], ext = SLIDE.out) => (Math.abs(x) < 0.311 && Math.abs(z) < 0.381 && y > 0.05 && y < 0.8) || (x >= 0.311 && x < 0.311 + ext + 0.02 && Math.abs(z) < 0.375 && y > 0.2 && y < 0.75);
const ownP = ([x, y, z], ext = SLIDE.out) => (Math.abs(x) < 0.311 && Math.abs(z) < 0.381 && y > 0.05 && y < 0.8) || (x >= 0.311 && x < 0.311 + ext + 0.02 && Math.abs(z) < 0.375 && y < 0.75);
const pos = (b) => { const p = b.translation(); return [p.x, p.y, p.z]; };
const inIt = (b, ext) => heldP(pos(b), ext);
// fill it a layer at a time (FUEL set into the space over the ones already in, let settle)
function fill(w, balls, step, { layers = 12, ext = SLIDE.out } = {}) {
  const [dx, dy] = [SLIDE.dir[0] * (SLIDE.out - ext), SLIDE.dir[1] * (SLIDE.out - ext)];
  const x1 = 0.598 + dx - 0.08, y = 0.645 + dy;
  for (let layer = 0; layer < layers; layer++) {
    for (let i = 0; i < 6; i++) for (let k = 0; k < 5; k++) {
      const x = 0.09 + 0.152 * i, z = -0.30 + 0.15 * k;
      if (x > x1) continue;
      if (balls.some((b) => { const p = b.translation(); return Math.hypot(p.x - x, p.y - y, p.z - z) < 2 * R; })) continue;
      balls.push(ball(w, x, y, z));
    }
    step(1.2 / DT);
  }
}
const TOP = { topX: 0.3, topN: 6 }; // (js/mech/phys1114.js naturalFull)

const tests = {
  rollers() {
    const { t, path, shooter } = rig();
    t.intake.cmd = 1; feed(t, true); flywheel(t, 2500);
    for (let i = 0; i < 4 / DT; i++) { path.step(DT, []); shooter.step(DT, []); }
    const surf = (sh, r) => Math.abs(sh.omega) * r;
    console.log(`1114's rollers, free (intake 12 V; their Shoot: floor 12 V, side rollers 10 V, the elevator's loop at 4500 RPM; the flywheel's at 2500 RPM):`);
    console.log(`  intake X60s ${RPM(t.intake.omega).toFixed(0)} rpm: 2in roller ${surf(t.roller, INTAKE.r).toFixed(2)} m/s at its surface`);
    console.log(`  floor X60s ${RPM(t.floor.omega).toFixed(0)} rpm: 1.25in rollers ${surf(t.rollers[0], FLOOR.r).toFixed(2)} m/s, squish wheels ${surf(t.squish, SQUISH.r).toFixed(2)} m/s`);
    console.log(`  side X44 ${RPM(t.sideT.omega).toFixed(0)} rpm: 2in side rollers ${surf(t.side[0], SIDE.r).toFixed(2)} m/s`);
    console.log(`  elevator X60 ${RPM(t.elev.omega).toFixed(0)} rpm (their 4500): front column ${surf(t.front[0], ELEV.r).toFixed(2)} m/s, back ${surf(t.back[0], ELEV.r).toFixed(2)} m/s`);
    console.log(`  shooter X60s ${RPM(t.shooter.omega).toFixed(0)} rpm: flywheel ${RPM(t.fly.omega).toFixed(0)} rpm, ${surf(t.fly, SHOOTER.fly.r).toFixed(2)} m/s; hood rollers ${surf(t.hood, SHOOTER.hoodR).toFixed(2)} m/s`);
    const r2 = rig(); r2.t.floor.cmd = 3 / 12;
    for (let i = 0; i < 1 / DT; i++) r2.path.step(DT, []);
    console.log(`  the floor at their intaking 3 V: ${surf(r2.t.rollers[0], FLOOR.r).toFixed(2)} m/s`);
    for (const target of [1000, 2505, 3150]) {
      const r3 = rig();
      let T = 0;
      flywheel(r3.t, target);
      for (; T < 10; T += DT) { r3.shooter.step(DT, []); if (RPM(r3.t.shooter.omega) > target - 100) break; }
      console.log(`  the flywheel from rest to within 100 of ${target} motor RPM: ${T.toFixed(2)} s (its motors at ${(r3.t.shooter.motor.current / 4).toFixed(0)} A each at the end)`);
    }
    // the box: out on their position loop (13.55 turns), in at their agitation's -2 V
    const r4 = rig({ ext: 0 });
    let T = 0;
    for (; T < 3 && slideTurns(r4.ext) < SLIDE_POS.drop - 0.98; T += DT) { slide(r4, { turns: SLIDE_POS.drop }); r4.path.step(DT, []); moveSlide(r4); }
    console.log(`  the box out on their position loop: out (their 12.6 turns) in ${T.toFixed(2)} s, the kicker ${(kickerRot(r4.ext) * 180 / Math.PI).toFixed(0)} deg up`);
    for (T = 0; T < 4 && slideTurns(r4.ext) > SLIDE_POS.agitateMin; T += DT) { slide(r4, { volts: -2 }); r4.path.step(DT, []); moveSlide(r4); }
    console.log(`  in at their agitation's -2 V to 6.3 turns: ${T.toFixed(2)} s (empty)`);
    for (T = 0; T < 2; T += DT) { slide(r4, {}); r4.path.step(DT, []); moveSlide(r4); }
    console.log(`  coasting 2 s: ${slideTurns(r4.ext).toFixed(1)} turns`);
  },
  // FUEL up the elevator into the flywheel at their LUT's rows
  shooter() {
    console.log(`1114's shooter, FUEL fed up the elevator two at a time (their Shoot; their LUT's RPM less 50 and hood):`);
    let rows = [...HUB_LUT.filter((_, i) => i % 3 === 0 || i === HUB_LUT.length - 1).map(([d]) => ['hub', d]), ...PASS_LUT.map(([d]) => ['pass', d])];
    if (process.env.SHOT) { const [rpm, hood] = process.env.SHOT.split(',').map(Number); rows = [['custom', 0, { rpm, hood }]]; }
    if (process.env.MAP) { rows = []; for (const hood of [44, 50, 56, 62, 68, 74]) for (const rpm of [1800, 2200, 2600, 3000, 3600, 4400, 5200]) rows.push(['map', 0, { rpm, hood }]); }
    for (const [kind, d, custom] of rows) {
      const want = custom ?? shotFor(kind, d);
      const { t, path, shooter } = rig({ hood: want.hood });
      const w = world();
      flywheel(t, want.rpm);
      for (let i = 0; i < 6 / DT; i++) shooter.step(DT, []);
      feed(t, true);
      const balls = [-0.08, 0.08].map((z) => ball(w, -0.122, 0.3, z));
      const watch = shotWatch(shooter);
      let minRpm = Infinity;
      for (let i = 0; i < 3 / DT && watch.shots.length < balls.length; i++) {
        path.step(DT, balls, balls); shooter.step(DT, balls); w.step();
        watch.check(balls, i * DT);
        minRpm = Math.min(minRpm, RPM(t.shooter.omega));
      }
      w.free();
      const outs = watch.shots;
      if (!outs.length) { console.log(`  ${kind} ${d} m (${want.rpm.toFixed(0)} RPM, hood ${want.hood}): nothing came out`); continue; }
      const o = { v: med(outs.map((q) => q.v)), el: med(outs.map((q) => q.el)), x: med(outs.map((q) => q.x)), y: med(outs.map((q) => q.y)) };
      const surf = Math.abs(t.fly.omega) * SHOOTER.fly.r;
      const land = landing(o);
      if (process.env.MAP) { console.log(`  map ${want.rpm} ${want.hood} ${o.v.toFixed(3)} ${o.el.toFixed(2)} ${o.x.toFixed(3)} ${o.y.toFixed(3)}`); continue; }
      console.log(`  ${kind} ${d} m (${want.rpm.toFixed(0)} RPM, hood ${want.hood.toFixed(1)}): ${outs.length}/2 out at ${o.v.toFixed(2)} m/s (${(o.v / surf).toFixed(2)} of the flywheel's ${surf.toFixed(1)}; the estimate ${EXIT_PER_FLY_SURFACE}), ${o.el.toFixed(1)} deg (${hoodElevation(want.hood).toFixed(1)} by the CAD's geometry) from ${o.x.toFixed(2)},${o.y.toFixed(2)}, the motors down to ${minRpm.toFixed(0)}; ` +
        (kind === 'hub' ? `comes down at the rim ${land.toFixed(2)} m out (the HUB's center ${d} m: ${land - d > 0 ? '+' : ''}${(land - d).toFixed(2)})` : kind === 'pass' ? `comes down to the carpet ~${(o.x + sim2D(o.y, o.v, (o.el * Math.PI) / 180, 0.1)?.d).toFixed(1)} m out (their ${d})` : `rim ${land.toFixed(2)} m`));
    }
  },
  // their LUT's rows re-tuned to the sim's FUEL: at each row's hood angle, the motor RPM whose
  // shots come down at the HUB's rim height over its center (or, passing, on the carpet at the
  // row's distance), found by shooting (a secant search from their RPM); prints the table
  // js/mech/r1114.js keeps as SIM_HUB / SIM_PASS
  tune() {
    const shoot = (rpm, hood, H) => {
      const { t, path, shooter } = rig({ hood });
      const w = world();
      flywheel(t, rpm);
      for (let i = 0; i < 6 / DT; i++) shooter.step(DT, []);
      feed(t, true);
      const balls = [-0.08, 0.08].map((z) => ball(w, -0.122, 0.3, z));
      const watch = shotWatch(shooter);
      for (let i = 0; i < 3 / DT && watch.shots.length < balls.length; i++) { path.step(DT, balls, balls); shooter.step(DT, balls); w.step(); watch.check(balls, i * DT); }
      w.free();
      const L = watch.shots.map((o) => { const h = sim2D(o.y, o.v, (o.el * Math.PI) / 180, H); return h ? o.x + h.d : NaN; }).filter(Number.isFinite);
      return L.length ? mean(L) : NaN;
    };
    const out = { hub: [], pass: [] };
    for (const [kind, lut, H] of [['hub', HUB_LUT, HUB.rimFront], ['pass', PASS_LUT, 0.1]]) {
      for (const [d, rpm0, hood0] of lut) {
        const hood = Math.max(SHOOTER.hoodMin, Math.min(SHOOTER.hoodMax, hood0));
        let a = (rpm0 - 50) * 1.1, la = shoot(a, hood, H), b = a * 1.06, lb = shoot(b, hood, H);
        for (let k = 0; k < 6 && Number.isFinite(la) && Number.isFinite(lb) && Math.abs(lb - d) > 0.03 && Math.abs(lb - la) > 1e-3; k++) {
          const c = Math.max(1500, Math.min(5800, b + ((d - lb) * (b - a)) / (lb - la)));
          a = b; la = lb; b = c; lb = shoot(b, hood, H);
        }
        const ok = Number.isFinite(lb) && Math.abs(lb - d) < 0.1;
        out[kind].push([d, ok ? Math.round(b + 50) : null, hood0]);
        console.log(`  ${kind} ${d} m, hood ${hood0}: their ${rpm0} RPM; the sim's FUEL come down there at ${ok ? Math.round(b + 50) : 'none (out of reach: ' + lb.toFixed(2) + ' m at ' + Math.round(b + 50) + ')'} (x${ok ? ((b + 50) / rpm0).toFixed(3) : '-'})`);
      }
    }
    console.log('SIM_HUB', JSON.stringify(out.hub));
    console.log('SIM_PASS', JSON.stringify(out.pass));
  },
  // carpet to shot: FUEL in front, the robot driving into them; intaking and shooting at 3 m
  path() {
    const runs = [];
    for (let seed = 1; seed <= SEEDS; seed++) runs.push(pathRun(seed));
    console.log(`1114 carpet to shot (20 FUEL, four abreast, the robot at ${SPEED} m/s, the roller at 12 V, their Shoot at their 3 m shot): a median ${med(runs.map((r) => r.shots))} shot (${Math.min(...runs.map((r) => r.shots))}..${Math.max(...runs.map((r) => r.shots))}), ` +
      `${med(runs.map((r) => r.rate)).toFixed(1)} FUEL/s once they're coming; ${med(runs.map((r) => r.inside))} in it at the end; the roller's X60s down to ${med(runs.map((r) => r.minIntake)).toFixed(0)}%, the floor to ${med(runs.map((r) => r.minFloor)).toFixed(0)}%`);
    if (process.env.STUCK) for (const r of runs) console.log('   left', r.left.map((p) => p.map((v) => v.toFixed(2)).join(',')).join(' | '));
  },
  // their RunIntakeBallevator alone (roller 12 V, the floor off its first 3 s), FUEL on the treadmill
  intake() {
    const speeds = process.env.SPEED ? [SPEED] : [1, 3];
    for (const sp of speeds) {
      const runs = [];
      for (let seed = 1; seed <= Math.min(SEEDS, 4); seed++) runs.push(pathRun(seed, { shoot: false, n: 24, sec: 6, speed: sp }));
      console.log(`1114 intaking only (their RunIntakeBallevator: roller 12 V), 24 FUEL four abreast at ${sp} m/s: ${runs.map((r) => r.inside).join(', ')} in it after 6 s; in past the bumper each second ${runs.map((r) => r.timeline.join('/')).join(', ')}; the roller's X60s down to ${med(runs.map((r) => r.minIntake)).toFixed(0)}%`);
      if (process.env.STUCK) for (const r of runs) console.log('   left', r.left.map((p) => p.map((v) => v.toFixed(2)).join(',')).join(' | '));
    }
  },
  intakefull() {
    const res = [];
    for (let seed = 1; seed <= Math.min(SEEDS, 4); seed++) {
      const r = intakeFull(seed, { keep: !!process.env.FILLFILE });
      res.push(r);
      if (process.env.FILLFILE) { writeFileSync(`${process.env.FILLFILE}-${seed}.json`, JSON.stringify(r.balls.map(pos).filter((p) => heldP(p, r.rig.ext)))); r.world.free(); }
      console.log(fillReport(`1114 intaking at ${SPEED} m/s, seed ${seed}`, r));
    }
    const fullAt = res.map((r) => r.fulls[0]?.n).filter((n) => n !== undefined);
    console.log(`1114 through its intake: the most it held ${res.map((r) => r.most).join(', ')} (mean ${mean(res.map((r) => r.most)).toFixed(1)}); full at ${fullAt.join(', ')}`);
  },
  hold() {
    for (const ext of [SLIDE.out, 0]) {
      const { path } = rig({ ext });
      const w = world();
      const balls = [];
      const step = (n) => { for (let i = 0; i < n; i++) { path.step(DT, balls, balls); w.step(); } };
      fill(w, balls, step, { ext });
      step(2 / DT);
      const n = balls.filter((b) => inIt(b, ext)).length, top = Math.max(...balls.filter((b) => inIt(b, ext)).map((b) => b.translation().y));
      if (process.env.HOLDFILE) writeFileSync(`${process.env.HOLDFILE}-${ext ? 'out' : 'in'}.json`, JSON.stringify(balls.map(pos)));
      console.log(`1114 holds ${n} with the box ${ext ? 'out' : 'in'}, dropped in a layer at a time, everything off (the load's top ${(top + R).toFixed(2)} m)`);
      w.free();
    }
  },
  // a full one fired: filled through its intake until it's full (intakefull), then their Shoot at
  // their 3 m shot with AgitateIntake (after 0.65 s the box in at -2 V to 6.3 turns, the roller at
  // 6 V while it's out past 10.8)
  dump() {
    const runs = [];
    for (let seed = 1; seed <= (process.env.FILL === 'layers' ? 1 : Number(process.env.SEEDS ?? 3)); seed++) runs.push(dumpRun(seed));
    console.log(`1114 dumps (mean of ${runs.length}): ${mean(runs.map((r) => r.n)).toFixed(1)} of ${mean(runs.map((r) => r.n0)).toFixed(1)}, ${mean(runs.map((r) => r.rate)).toFixed(1)} FUEL/s over the whole dump (${runs.map((r) => r.rate.toFixed(1)).join(', ')}), best second ${mean(runs.map((r) => r.best)).toFixed(1)} (${runs.map((r) => r.best).join(', ')}), ${mean(runs.map((r) => r.inHub)).toFixed(1)} in the HUB's opening`);
  },
};

function intakeFull(seed, { keep = false, sec = Number(process.env.SEC ?? 10), until = null } = {}) {
  const rg = rig(), { t, path } = rg;
  t.intake.cmd = 1;
  // their RunIntakeBallevator: after 3 s the floor at 3 V while the hood CANrange sees nothing (a
  // FUEL up the elevator: js/mech/r1114.js RANGE)
  const control = (time, live) => {
    const staged = live.some((b) => { const p = b.translation(); return Math.abs(p.x - RANGE.at[0]) < RANGE.half && p.y > RANGE.at[1] && p.y < 0.8; });
    t.floor.cmd = time > 3 && !staged ? 3 / 12 : null;
    slide(rg, {}); moveSlide(rg);
  };
  const r = fillRun(RAPIER, { world: () => world({ carpet: false }), ball, path, DT, held: (p) => heldP(p, rg.ext), own: (p) => ownP(p, rg.ext), top: TOP, control, speed: SPEED, sec, seed, front: 0.31 + SLIDE.out + 0.03, keep, until });
  return { ...r, rig: rg };
}

function dumpRun(seed) {
  const D = 3.02, want = shotFor('hub', D);
  let rg, w, balls;
  if (process.env.FILL === 'layers') {
    rg = rig({ hood: want.hood }); w = world(); balls = [];
    fill(w, balls, (n) => { for (let i = 0; i < n; i++) { rg.path.step(DT, balls, balls); w.step(); } });
  } else {
    const f = intakeFull(seed, { keep: true, until: (full) => full, sec: 14 });
    rg = f.rig; w = f.world;
    f.carpet.setLinvel({ x: 0, y: 0, z: 0 }, true);
    balls = [];
    for (const b of f.balls) { if (ownP(pos(b), rg.ext)) balls.push(b); else b.setEnabled(false); }
    rg.hood = want.hood; rg.pose();
  }
  const { t, path, shooter } = rg;
  const step = (run) => { run(); path.step(DT, balls, balls); shooter.step(DT, balls); w.step(); moveSlide(rg); };
  // (the roller let go, the flywheel up to the shot's speed on the way to the HUB: 3 s)
  t.intake.cmd = null; t.floor.cmd = null;
  flywheel(t, want.rpm);
  for (let i = 0; i < 3 / DT; i++) step(() => slide(rg, {}));
  const n0 = balls.filter((b) => ownP(pos(b), rg.ext)).length;
  const watch = shotWatch(shooter);
  let minRpm = Infinity;
  for (let k = 0; k < 8 / DT; k++) {
    const T = k * DT, rpm = RPM(t.shooter.omega);
    const fire = rpm >= want.rpm - 500, agit = !process.env.NOAGITATE && rpm >= want.rpm - 400 && T > 0.65 && slideTurns(rg.ext) > SLIDE_POS.agitateMin;
    feed(t, fire);
    t.intake.cmd = agit && slideTurns(rg.ext) >= SLIDE_POS.rollersOff ? 6 / 12 : null;
    step(() => slide(rg, { volts: agit ? -2 : 0 }));
    watch.check(balls, T);
    if (watch.shots.length) minRpm = Math.min(minRpm, RPM(t.shooter.omega));
    // (DUMPLOG=1: where the load is each 0.25 s: up the elevator, at its foot, over the floor's back
    // slope and valley, on the floor, in the box out front, and the elevator's and floor's speeds)
    if (process.env.DUMPSNAP && k === Math.round(Number(process.env.DUMPSNAP) / DT)) {
      for (const b of balls) { const p = b.translation(); if (p.y > 3 || p.x > 40) continue; console.log(`   at ${p.x.toFixed(2)},${p.y.toFixed(2)},${p.z.toFixed(2)} v ${b.linvel().y.toFixed(2)}: ${[...path.contacts, ...shooter.contacts].filter((c) => c.ball === b).map((c) => c.wheel.name + ' ' + c.force.toFixed(0)).join(', ')}`); }
    }
    if (process.env.DUMPLOG && k % 30 === 0) {
      const P = balls.map(pos).filter((p) => p[1] < 3 && p[0] < 40);
      const n = (f) => P.filter(f).length;
      console.log(`   ${T.toFixed(2)} s: shot ${watch.shots.length}; elevator ${n((p) => p[0] < -0.04 && p[0] > -0.2 && p[1] > 0.3)}, foot ${n((p) => p[0] < -0.04 && p[1] <= 0.3)}, valley ${n((p) => p[0] >= -0.04 && p[0] < 0.1 && p[1] < 0.3)}, floor ${n((p) => p[0] >= 0.1 && p[0] < 0.31 && p[1] < 0.3)}, upper ${n((p) => p[0] >= -0.04 && p[0] < 0.31 && p[1] >= 0.3)}, box ${n((p) => p[0] >= 0.31)}; elevator ${RPM(t.elev.omega).toFixed(0)} rpm ${t.elev.motor.current.toFixed(0)} A, floor ${RPM(t.floor.omega).toFixed(0)} rpm ${t.floor.motor.current.toFixed(0)} A, side ${t.sideT.motor.current.toFixed(0)} A, the box at ${slideTurns(rg.ext).toFixed(1)} turns, FUEL on it ${(path.contacts.filter((c) => c.wheel === rg.s.slide).reduce((a, c) => a + c.torque, 0) / SLIDE.pinion).toFixed(0)} N, its X44 ${t.slide.motor.current.toFixed(0)} A`);
    }
  }
  const s = watch.shots.map((x) => x.t), first = s[0] ?? 0, in1 = s.filter((x) => x - first < 1).length;
  const best = Math.max(0, ...s.map((a) => s.filter((b) => b >= a && b < a + 1).length));
  const rate = s.length / Math.max(1e-6, s[s.length - 1] - first);
  const left = balls.filter((b) => b.translation().x < 40 && inIt(b, rg.ext));
  if (process.env.STUCK) for (const b of left) { const p = b.translation(); console.log(`   left at ${p.x.toFixed(2)},${p.y.toFixed(2)},${p.z.toFixed(2)}: ${[...path.contacts, ...shooter.contacts].filter((c) => c.ball === b).map((c) => c.wheel.name + ' ' + c.force.toFixed(0)).join(', ')}`); }
  if (process.env.BINS) { const bins = []; for (const o of s) { const i = Math.floor((o - first) / 0.25); bins[i] = (bins[i] || 0) + 1; } console.log('   shots each 0.25 s: ' + Array.from(bins, (v) => v || 0).join(' ')); }
  const land = watch.shots.map((o) => landing(o)).filter(Number.isFinite).sort((a, b) => a - b);
  const inHub = land.filter((d) => Math.abs(d - D) < HUB_R).length;
  const sh = watch.shots;
  console.log(`  (shots leave at ${med(sh.map((o) => o.v)).toFixed(2)} m/s, ${med(sh.map((o) => o.el)).toFixed(1)} deg; come down at the rim ${land[land.length >> 1]?.toFixed(2)} m out (median; ${land[0]?.toFixed(2)}..${land[land.length - 1]?.toFixed(2)}), ${inHub} of ${land.length} in the HUB's opening ${D} m out; the motors down to ${minRpm.toFixed(0)} of ${want.rpm.toFixed(0)} RPM; the box in to ${slideTurns(rg.ext).toFixed(1)} turns; ${left.length} still in it)`);
  console.log(`1114 dumps ${s.length} of ${n0}${process.env.FILL === 'layers' ? ' (filled a layer at a time)' : ` (filled through its intake, seed ${seed})`}: ${in1} in the first second, best second ${best}, ${rate.toFixed(1)} FUEL/s over the whole dump`);
  w.free();
  return { n: s.length, n0, rate, best, in1, inHub };
}

function pathRun(seed, { shoot = true, n = 20, sec = 7, speed = SPEED } = {}) {
  let sd = seed;
  const rnd = () => ((sd = (sd * 16807) % 2147483647) / 2147483647 - 0.5);
  const want = shotFor('hub', 3.02);
  const rg = rig({ hood: want.hood }), { t, path, shooter } = rg;
  const w = world({ carpet: false });
  const front = 0.31 + SLIDE.out + 0.03;
  // the carpet, running under the robot
  const carpet = w.createRigidBody(RAPIER.RigidBodyDesc.kinematicVelocityBased().setLinvel(-speed, 0, 0));
  w.createCollider(RAPIER.ColliderDesc.cuboid(15, 0.01, 1).setTranslation(front + 15, -0.01, 0).setFriction(0.3).setRestitution(0.2), carpet);
  w.createCollider(RAPIER.ColliderDesc.cuboid(0.6, 0.01, 1).setTranslation(front - 0.6, -0.01, 0).setFriction(0.3), w.createRigidBody(RAPIER.RigidBodyDesc.fixed()));
  t.intake.cmd = 1;
  if (shoot) { flywheel(t, want.rpm); feed(t, true); }
  for (let i = 0; i < 5 / DT; i++) { shooter.step(DT, []); path.step(DT, []); }
  const free = { intake: Math.abs(t.intake.omega), floor: Math.abs(t.floor.omega) || 1 };
  const balls = [];
  for (let i = 0; i < n; i++) balls.push(ball(w, front + 0.3 + 0.17 * Math.floor(i / 4) + 0.02 * rnd(), R + 0.003, ((i % 4) - 1.5) * 0.16 + 0.02 * rnd(), [-speed, 0, 0]));
  const watch = shotWatch(shooter);
  let minIntake = 1, minFloor = 1;
  const timeline = [];
  for (let k = 0; k < sec / DT; k++) {
    const live = balls.filter((b) => b.translation().y < 3 && b.translation().x > -1);
    if (!shoot) t.floor.cmd = k * DT > 3 && !live.some((b) => { const p = b.translation(); return Math.abs(p.x - RANGE.at[0]) < RANGE.half && p.y > RANGE.at[1] && p.y < 0.8; }) ? 3 / 12 : null;
    slide(rg, {}); moveSlide(rg);
    path.step(DT, live, live); shooter.step(DT, live); w.step();
    minIntake = Math.min(minIntake, Math.abs(t.intake.omega) / free.intake);
    if (shoot) minFloor = Math.min(minFloor, Math.abs(t.floor.omega) / free.floor);
    watch.check(live, k * DT);
    if (k % Math.round(1 / DT) === Math.round(1 / DT) - 1) timeline.push(balls.filter((b) => inIt(b, rg.ext)).length);
    if (process.env.TRACE && k % 6 === 0) {
      const b = balls[Number(process.env.TRACE)], p = b.translation(), v = b.linvel();
      console.log(`   ${(k * DT).toFixed(2)} s at ${p.x.toFixed(3)},${p.y.toFixed(3)},${p.z.toFixed(3)} v ${v.x.toFixed(2)},${v.y.toFixed(2)},${v.z.toFixed(2)}: ${[...path.contacts, ...shooter.contacts].filter((c) => c.ball === b).map((c) => c.wheel.name + ' ' + c.force.toFixed(0)).join(', ')}`);
    }
  }
  const s = watch.shots.map((x) => x.t);
  const inside = balls.filter((b) => b.translation().y < 3 && inIt(b, rg.ext));
  const left = balls.filter((b) => b.translation().y < 3 && b.translation().x < 40 && !inside.includes(b)).map(pos);
  w.free();
  return { timeline, shots: s.length, rate: s.length > 2 ? (s.length - 1) / (s[s.length - 1] - s[0]) : 0, inside: inside.length, minIntake: 100 * minIntake, minFloor: 100 * minFloor, left };
}

const pick = process.argv.slice(2);
for (const [name, fn] of Object.entries(tests)) if (!pick.length || pick.includes(name)) fn();
