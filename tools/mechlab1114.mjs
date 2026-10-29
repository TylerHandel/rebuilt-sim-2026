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
// full one fired at their 3 m shot, with their AgitateIntake pulling the box in), feed (what the
// floor, squish wheels and elevator take with the floor's front kept full: FEEDLOG, FEEDSNAP=t).
// tune (their LUT's rows re-found by shooting: js/mech/r1114.js SIM_HUB, SIM_PASS). spinup (the
// flywheel from their ShooterIdle's 1000 RPM up to their 2.34, 3.02 and 4 m shots at their 50 A
// stator limit and higher ones: LIMITS=a,b,..; BATT=1 with the battery sagging under it).
// fullshake (full through the intake, then driven about: SHAKE_A, SHAKE_W, SHAKE_INTAKE=1 with the
// roller at 12 V), knock (full, then stopped dead against a wall: KNOCK times, KNOCK_A m/s^2 for
// KNOCK_T s), panel (one FUEL thrown at the box's front panel from inside, or pressed against it).
// Env: SEEDS (6), SPEED (path's 1 m/s; intake's 1,3), SEC (intakefull's 10 s), STUCK (where the
// ones left over are), SHOT=rpm,hood (one shot of your own), MAP=1 (shooter: a grid of RPMs and
// hoods), THEIR_RPM=1 (their LUT's own RPMs), NOAGITATE=1 (the dump without the box coming in),
// BOTH=1 (dump: intake and shoot held together, as play has them: the roller at 12 V and the box
// held out by their RunIntakeBallevator, no AgitateIntake), BOTHFOR=s (dump: that for the first s
// seconds, then the intake let go, as the AI does),
// AGITATE_V=v (dump: the box pulled in at v volts, not the 3.5 play uses, their SLIDE_FAST_PULL_VOLTAGE
// (js/mech/r1114.js SLIDE.pull); their AgitateIntake runs 2), AGITATE=cycle (dump: the box in and out,
// their commented-out cycle),
// FILL=layers (dump: filled as hold does; EXT=m with the box m out), SEED=a,b (dump: those fills),
// NOROLL=1 (dump: the intake roller off while the box comes in), FORCEIN=v (dump: the box driven in
// at v m/s whatever the load), DUMPTALLY=t0,t1 (dump: what holds the load up, TALLYAT=1 where), SPINUP=s (dump: the flywheel's head start, 3 s; 0 from rest), IDLE=1 (dump: the head start at their 1000 RPM idle, so it spins up from there as the trigger's pulled), STATOR=a / SUPPLY=a (dump, spinup: the shooter's X60s' limits, per motor, not their 50 A stator and Phoenix's 70 A supply), DIST=m (dump: from m, not 3.02), BINS (dump: shots each quarter
// second), DUMPLOG (dump: where the load is each quarter second), DUMPSNAP=t (dump: every FUEL
// and what it touches at t), HOLDFILE / FILLFILE=path (hold / intakefull: where the FUEL are, as
// JSON), TRACE=i (path)
import { RAPIER } from '../js/physics.js';
import { FUEL, GROUP, groups, HUB } from '../js/constants.js';
import { Mechanism } from '../js/mech/mechanism.js';
import { sim2D } from '../js/ballistics.js';
import {
  build1114, wheels1114, pathSurfaces1114, placeSlide, placeIntake, placeHood, hoodRot, hoodElevation, shotFor, slideTurns, kickerRot, slideOffset, slideStops,
  HUB_LUT, PASS_LUT, SLIDE, SLIDE_POS, SHOOTER, BALLEVATOR, FLOOR, ELEV, SQUISH, SIDE, INTAKE, FLY, RANGE, BLANKET, PANEL,
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
  if (process.env.STATOR) t.shooter.motor.statorLimit = Number(process.env.STATOR);
  if (process.env.SUPPLY) t.shooter.motor.supplyLimit = Number(process.env.SUPPLY);
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
  slideStops(T.slide, r.ext);
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
// FUEL that got out, not as shots (each once it's been in it): where it left the robot's volume
// (the frame, 0.80 m up, and the box as far out as it is): over the top (behind the blanket, x
// under its back edge, or through it), over a side, out the back, or out the front over the
// intake, through the front panel or under it (along the slot over the bottom front tube: which
// side of the panel's bottom edge its center first got past the panel's inner face); with ESC=1
// where it crossed and how fast, and out the front its center's last steps in the box's frame as
// exported (u: how far past the panel's inner face, mm; y: how high, m) and its dent on the panel
// then (mm)
function escapeWatch(rg) {
  const seen = new Set(), out = new Map(), last = new Map(), hist = new Map();
  const inVol = ([x, y, z], ext) => y > -0.02 && y < 0.8 && Math.abs(z) < 0.39 && x > -0.331 && x < 0.311 + ext + 0.05;
  return {
    out,
    check(balls) {
      const ext = rg.ext, bx0 = BLANKET.x0 + slideOffsetX(ext);
      for (const b of balls) {
        if (out.has(b)) continue;
        const p = pos(b);
        if (!seen.has(b)) { if (heldP(p, ext)) seen.add(b); else continue; }
        // (shot: on its way up off the flywheel; set aside)
        if (p[0] > 40 || p[1] > 3 || (b.linvel().y > 2 && p[1] > 0.55 && p[0] < 0.1)) { seen.delete(b); continue; }
        {
          const [dx, dy] = slideOffset(ext), h = hist.get(b) ?? [], c = rg.path.contacts.find((k) => k.ball === b && k.wheel === rg.s.panel);
          h.push([(p[0] - dx - PANEL.x0) * 1000, p[1] - dy, c ? c.depth * 1000 : 0]);
          if (h.length > 10) h.shift();
          hist.set(b, h);
        }
        if (inVol(p, ext)) { last.set(b, p); continue; }
        const q = last.get(b) ?? p;
        let why = p[1] >= 0.8 ? (q[0] < bx0 ? 'top, behind the blanket' : 'top, through the blanket') : Math.abs(p[2]) >= 0.39 ? 'side' : p[0] <= -0.331 ? 'back' : p[0] >= 0.311 + ext + 0.05 ? 'front' : 'under';
        // (out the front: through the panel if its center first got past the panel's inner face
        // above the panel's bottom edge)
        const h = hist.get(b) ?? [], past = h.find((e) => e[0] > 0);
        if (why === 'front' && past) why = past[1] >= PANEL.y0 ? 'front, through the panel' : 'front, under the panel';
        out.set(b, why);
        if (process.env.ESC) { const v = b.linvel(); console.log(`   escaped (${why}): last in at ${q.map((u) => u.toFixed(3)).join(',')}, now ${p.map((u) => u.toFixed(3)).join(',')} v ${v.x.toFixed(1)},${v.y.toFixed(1)},${v.z.toFixed(1)}; the box ${slideTurns(ext).toFixed(1)} turns out${why.startsWith('front') ? `; its last steps (u mm, y m, dent on the panel mm): ${h.map((e) => `${e[0].toFixed(0)} ${e[1].toFixed(3)} ${e[2].toFixed(0)}`).join(' | ')}` : ''}`); }
      }
    },
    report() {
      const c = {};
      for (const w of out.values()) c[w] = (c[w] || 0) + 1;
      return `${out.size} escaped${out.size ? ` (${Object.entries(c).map(([k, n]) => `${k}: ${n}`).join(', ')})` : ''}`;
    },
  };
}
const slideOffsetX = (ext) => SLIDE.dir[0] * (SLIDE.out - ext);

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
    console.log(`  elevator X60 ${RPM(t.elev.omega).toFixed(0)} rpm (their 4500): front column ${surf(t.front[0], ELEV.r).toFixed(2)} m/s, back ${surf(t.back[0], ELEV.r).toFixed(2)} m/s; the roller wall's face (down) ${surf(t.wall, ELEV.r).toFixed(2)} m/s`);
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
    // the box: out on their position loop (13.55 turns), in at their agitation's -2 V and at the
    // -3.5 V play pulls it in with (their SLIDE_FAST_PULL_VOLTAGE)
    const r4 = rig({ ext: 0 });
    let T = 0;
    for (; T < 3 && slideTurns(r4.ext) < SLIDE_POS.drop - 0.98; T += DT) { slide(r4, { turns: SLIDE_POS.drop }); r4.path.step(DT, []); moveSlide(r4); }
    console.log(`  the box out on their position loop: out (their 12.6 turns) in ${T.toFixed(2)} s, the kicker ${(kickerRot(r4.ext) * 180 / Math.PI).toFixed(0)} deg up`);
    const out = r4.ext;
    for (const v of [SLIDE.theirPull, SLIDE.pull]) {
      r4.ext = out; r4.t.slide._omega = 0;
      for (T = 0; T < 4 && slideTurns(r4.ext) > SLIDE_POS.agitateMin; T += DT) { slide(r4, { volts: v }); r4.path.step(DT, []); moveSlide(r4); }
      console.log(`  in at ${v} V to 6.3 turns: ${T.toFixed(2)} s (empty)`);
    }
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
      console.log(fillReport(`1114 intaking at ${SPEED} m/s, seed ${seed}`, r) + `; ${r.esc.report()}`);
    }
    const fullAt = res.map((r) => r.fulls[0]?.n).filter((n) => n !== undefined);
    console.log(`1114 through its intake: the most it held ${res.map((r) => r.most).join(', ')} (mean ${mean(res.map((r) => r.most)).toFixed(1)}); full at ${fullAt.join(', ')}`);
  },
  // what the floor, squish wheels and elevator can take, fed as fast as they'll take them: FUEL
  // set on the floor's front (between the tower and the bumper, the box out) wherever there's room
  // for 4 s, their Shoot firing at their 3 m shot
  feed() {
    const keep = 30, want = shotFor('hub', 3.02);
    const rg = rig({ hood: want.hood }), { t, path, shooter } = rg, w = world(), balls = [];
    flywheel(t, want.rpm);
    for (let i = 0; i < 3 / DT; i++) shooter.step(DT, []);
    const watch = shotWatch(shooter);
    let sd = 7;
    const stage = new Map();
    const rnd = () => ((sd = (sd * 16807) % 2147483647) / 2147483647 - 0.5);
    const over = (p) => p.x > 0.04 && p.x < 0.31 && p.y > 0.3 && p.y < 0.75 && Math.abs(p.z) < 0.3;
    for (let k = 0; k < 6 / DT; k++) {
      const T = k * DT;
      feed(t, true);
      for (let tries = 0; T < 4 && tries < 10 && balls.filter((b) => over(b.translation())).length < keep; tries++) {
        const x = 0.2 + 0.16 * rnd(), z = 0.36 * rnd(), y = 0.3 + 0.1 * rnd();
        if (balls.some((b) => { const p = b.translation(); return p.y < 3 && Math.hypot(p.x - x, p.y - y, p.z - z) < 2 * R; })) continue;
        balls.push(ball(w, x, y, z, [0, -1, 0]));
      }
      const live = balls.filter((b) => b.translation().y < 3);
      slide(rg, {}); path.step(DT, live, live); shooter.step(DT, live); w.step(); moveSlide(rg);
      watch.check(live, T);
      // (each one's first time on the floor, in the valley, at the elevator's foot, up it)
      for (const b of live) {
        const [x, y] = pos(b), st = stage.get(b) ?? {};
        stage.set(b, st);
        const at = x >= 0.1 && x < 0.31 && y < 0.3 ? 'floor' : x >= -0.04 && x < 0.1 && y < 0.3 ? 'valley' : x < -0.04 && y <= 0.3 ? 'foot' : x < -0.04 && y > 0.3 && y < 0.8 ? 'elev' : null;
        if (at && st[at] === undefined) st[at] = T;
      }
      for (const o of watch.shots) if (stage.get(o.b) && stage.get(o.b).shot === undefined) stage.get(o.b).shot = o.t;
      if (process.env.FEEDSNAP && k === Math.round(Number(process.env.FEEDSNAP) / DT)) {
        for (const b of live) { const p = b.translation(); if (p.x > 40) continue; console.log(`   at ${p.x.toFixed(3)},${p.y.toFixed(3)},${p.z.toFixed(3)} v ${b.linvel().y.toFixed(2)}: ${[...path.contacts, ...shooter.contacts].filter((c) => c.ball === b).map((c) => c.wheel.name + ' ' + c.force.toFixed(0)).join(', ')}`); }
      }
      if (process.env.FEEDLOG && k % 30 === 0) {
        const P = live.map(pos).filter((p) => p[0] < 40), n = (f) => P.filter(f).length;
        console.log(`   ${T.toFixed(2)} s: shot ${watch.shots.length}; elevator ${n((p) => p[0] < -0.04 && p[0] > -0.2 && p[1] > 0.3)}, foot ${n((p) => p[0] < -0.04 && p[1] <= 0.3)}, valley ${n((p) => p[0] >= -0.04 && p[0] < 0.1 && p[1] < 0.3)}, floor ${n((p) => p[0] >= 0.1 && p[0] < 0.31 && p[1] < 0.3)}, upper ${n((p) => p[0] >= -0.04 && p[0] < 0.31 && p[1] >= 0.3)}, box ${n((p) => p[0] >= 0.31)}; floor ${RPM(t.floor.omega).toFixed(0)} rpm ${t.floor.motor.current.toFixed(0)} A, elevator ${RPM(t.elev.omega).toFixed(0)} rpm ${t.elev.motor.current.toFixed(0)} A, squish ${(Math.abs(t.squish.omega) * SQUISH.r).toFixed(2)} m/s`);
      }
    }
    const s = watch.shots.map((x) => x.t), bins = [];
    for (const o of s) bins[Math.floor(o)] = (bins[Math.floor(o)] || 0) + 1;
    console.log(`1114 fed onto the floor's front for 4 s, wherever there's room: ${s.length} shot, each second ${Array.from(bins, (v) => v || 0).join(' ')}; ${balls.filter((b) => b.translation().y < 3 && inIt(b)).length} still in it`);
    const seq = ['floor', 'valley', 'foot', 'elev', 'shot'], sh = [...stage.values()].filter((st) => st.shot !== undefined);
    console.log('   median time from stage to stage (s), of those shot: ' + seq.slice(0, -1).map((a, i) => { const d = sh.filter((st) => st[a] !== undefined && st[seq[i + 1]] !== undefined).map((st) => st[seq[i + 1]] - st[a]); return `${a}->${seq[i + 1]} ${d.length ? med(d).toFixed(3) : '-'} (${d.length})`; }).join(', '));
    w.free();
  },
  hold() {
    for (const ext of [SLIDE.out, 0]) {
      const rg = rig({ ext }), { path } = rg, esc = escapeWatch(rg);
      const w = world();
      const balls = [];
      // (the box held where it is: its pinions locked, so FUEL pressing on it meet a fixed box)
      const step = (n) => { for (let i = 0; i < n; i++) { rg.t.slide.lo = rg.t.slide.hi = 0; path.step(DT, balls, balls); w.step(); esc.check(balls); } };
      fill(w, balls, step, { ext });
      step(2 / DT);
      const n = balls.filter((b) => inIt(b, ext)).length, top = Math.max(...balls.filter((b) => inIt(b, ext)).map((b) => b.translation().y));
      if (process.env.HOLDFILE) writeFileSync(`${process.env.HOLDFILE}-${ext ? 'out' : 'in'}.json`, JSON.stringify(balls.map(pos)));
      console.log(`1114 holds ${n} with the box ${ext ? 'out' : 'in'}, dropped in a layer at a time, everything off (the load's top ${(top + R).toFixed(2)} m); ${esc.report()}`);
      if (process.env.SHAKE) shake(rg, w, balls, esc, `dropped in, the box ${ext ? 'out' : 'in'}`);
      w.free();
    }
  },
  // the front panel alone (js/mech/r1114.js FrontPanel), the box out, rollers off: one FUEL thrown
  // at it from inside the box at 3-20 m/s, and one pressed against it from inside with 200-800 N
  // (0.6 s each): does it stay in, and how far past the panel's inner face did its center get?
  panel() {
    const one = ({ at, v = 0, F = 0 }) => {
      const rg = rig(), { path } = rg, w = world(), b = ball(w, ...at, [v, 0, 0]);
      let far = -Infinity;
      for (let k = 0; k < 0.6 / DT; k++) {
        if (F) b.applyImpulse({ x: F * DT, y: 0, z: 0 }, true);
        slide(rg, {}); path.step(DT, [b], [b]); w.step(); moveSlide(rg);
        far = Math.max(far, b.translation().x - PANEL.x0);
      }
      const x = b.translation().x;
      w.free();
      return `${x > PANEL.x1 ? 'OUT' : 'in'} (its center at most ${(far * 1000).toFixed(0)} mm past the panel's inner face)`;
    };
    console.log(`1114's front panel, one FUEL from inside the box (the box out):`);
    for (const v of [3, 6, 10, 15, 20]) console.log(`  thrown at it at ${v} m/s: ${one({ at: [0.45, 0.45, 0], v })}`);
    for (const F of [200, 400, 800]) console.log(`  pressed against it with ${F} N: ${one({ at: [0.535, 0.45, 0], F })}`);
  },
  // hard knocks: full through its intake (the box out), then stopped dead against a wall KNOCK
  // times (3), 0.6 s apart: its load thrown forward at KNOCK_A m/s^2 (250) for KNOCK_T s (0.03:
  // 7.5 m/s each, harder than its 4.5 m/s stopped on its bumper, for a margin); SHAKE_INTAKE=1
  // with the roller at 12 V
  knock() {
    const A = Number(process.env.KNOCK_A ?? 250), T = Number(process.env.KNOCK_T ?? 0.03), N = Number(process.env.KNOCK ?? 3);
    for (let seed = 1; seed <= Math.min(SEEDS, 4); seed++) {
      const f = intakeFull(seed, { keep: true, until: (full) => full, sec: 14 });
      f.carpet.setLinvel({ x: 0, y: 0, z: 0 }, true);
      const balls = [];
      for (const b of f.balls) { if (ownP(pos(b), f.rig.ext)) balls.push(b); else b.setEnabled(false); }
      f.rig.t.intake.cmd = process.env.SHAKE_INTAKE ? 1 : null; f.rig.t.floor.cmd = null;
      const rg = f.rig, n0 = balls.filter((b) => inIt(b, rg.ext)).length, before = f.esc.out.size;
      const stepAll = () => { rg.path.step(DT, balls, balls); slide(rg, {}); moveSlide(rg); f.world.step(); f.esc.check(balls); };
      for (let i = 0; i < N; i++) {
        for (let k = 0; k < Math.round(T / DT); k++) {
          for (const b of balls) { const p = b.translation(); if (p.x < 40 && p.y < 3) b.applyImpulse({ x: FUEL.mass * A * DT, y: 0, z: 0 }, true); }
          stepAll();
        }
        for (let k = 0; k < 0.6 / DT; k++) stepAll();
      }
      const n1 = balls.filter((b) => inIt(b, rg.ext)).length;
      console.log(`1114 full through its intake (seed ${seed}), knocked ${N} times (${A} m/s^2 for ${T} s${process.env.SHAKE_INTAKE ? ', the roller at 12 V' : ''}): ${n1} of ${n0} still in it, ${f.esc.out.size - before} escaped; in all ${f.esc.report()}`);
      f.world.free();
    }
  },
  // full through its intake, then driven about with the load in it (the box out, everything off):
  // how many get out, and where (SEEDS fills; SHAKE_A the accelerations, m/s^2)
  fullshake() {
    for (let seed = 1; seed <= Math.min(SEEDS, 4); seed++) {
      const f = intakeFull(seed, { keep: true, until: (full) => full, sec: 14 });
      f.carpet.setLinvel({ x: 0, y: 0, z: 0 }, true);
      const balls = [];
      for (const b of f.balls) { if (ownP(pos(b), f.rig.ext)) balls.push(b); else b.setEnabled(false); }
      // (SHAKE_INTAKE=1: the roller kept at 12 V, a driver holding the intake with it full)
      f.rig.t.intake.cmd = process.env.SHAKE_INTAKE ? 1 : null; f.rig.t.floor.cmd = null;
      console.log(`1114 full through its intake (seed ${seed}): ${balls.filter((b) => inIt(b, f.rig.ext)).length} in it; filling, ${f.esc.report()}`);
      shake(f.rig, f.world, balls, f.esc, `full, seed ${seed}${process.env.SHAKE_INTAKE ? ', the roller at 12 V' : ''}`);
      f.world.free();
    }
  },
  // the flywheel from their ShooterIdle (1000 RPM, settled) up to their shots: how long to Shoot's
  // gate (within 500 RPM), AgitateIntake's (400), their tolerance (300) and within 50, at their 50
  // A stator limit and higher ones (LIMITS), each X60 under the supply limit (SUPPLY, Phoenix's 70
  // A); BATT=1: the battery's 12.6 V sagging 0.02 ohm under the shooter's draw and 25 A for the rest
  // of the robot (js/drivetrain.js), else a steady 12 V
  spinup() {
    const limits = (process.env.LIMITS ?? '50,80,100,120').split(',').map(Number), supply = Number(process.env.SUPPLY ?? SHOOTER.supply);
    const dists = (process.env.DISTS ?? '2.34,3.02,4').split(',').map(Number);
    console.log(`1114's flywheel from their ShooterIdle (${SHOOTER.idle} RPM) to their shots, ${supply} A supply limit per motor${process.env.BATT ? ', the battery sagging' : ', 12 V'}:`);
    for (const d of dists) {
      const want = shotFor('hub', d);
      for (const lim of limits) {
        const { t, shooter } = rig({ hood: want.hood });
        t.shooter.motor.statorLimit = lim; t.shooter.motor.supplyLimit = supply;
        flywheel(t, SHOOTER.idle);
        for (let i = 0; i < 3 / DT; i++) shooter.step(DT, []);
        flywheel(t, want.rpm);
        const hit = {}, gates = [500, 400, 300, 50];
        let T = 0, peakStator = 0, peakSupply = 0, minV = 12;
        for (; T < 4; T += DT) {
          // (each X60's supply current: its stator current at the volts it applies, back-EMF and the
          // windings' drop, over the battery's)
          const m = t.shooter.motor, I = Math.abs(m.current) / 4, sup = (I * ((t.shooter.omega * m.ratio) / m.kV + I * m.Rm)) / shooter.V;
          if (process.env.BATT) { shooter.V = 12.6 - 0.02 * (4 * sup + 25); minV = Math.min(minV, shooter.V); }
          shooter.step(DT, []);
          const rpm = RPM(t.shooter.omega);
          peakStator = Math.max(peakStator, I);
          peakSupply = Math.max(peakSupply, sup);
          for (const g of gates) if (hit[g] === undefined && rpm >= want.rpm - g) hit[g] = T + DT;
          if (hit[50] !== undefined) break;
        }
        console.log(`  ${d} m (${want.rpm.toFixed(0)} RPM) at ${lim} A stator: ${gates.map((g) => `within ${g} ${hit[g]?.toFixed(2) ?? '-'} s`).join(', ')}; each X60 at most ${peakStator.toFixed(0)} A stator, ${peakSupply.toFixed(0)} A supply${process.env.BATT ? `, the battery down to ${minV.toFixed(1)} V` : ''}`);
      }
    }
  },
  // a full one fired: filled through its intake until it's full (intakefull), then their Shoot at
  // their 3 m shot with AgitateIntake (after 0.65 s the box in at -2 V to 6.3 turns, the roller at
  // 6 V while it's out past 10.8)
  dump() {
    const runs = [];
    const seeds = process.env.SEED ? process.env.SEED.split(',').map(Number) : Array.from({ length: process.env.FILL === 'layers' ? 1 : Number(process.env.SEEDS ?? 3) }, (_, i) => i + 1);
    for (const seed of seeds) runs.push(dumpRun(seed));
    console.log(`1114 dumps (mean of ${runs.length}): ${mean(runs.map((r) => r.n)).toFixed(1)} of ${mean(runs.map((r) => r.n0)).toFixed(1)}, ${mean(runs.map((r) => r.rate)).toFixed(1)} FUEL/s over the whole dump (${runs.map((r) => r.rate.toFixed(1)).join(', ')}), best second ${mean(runs.map((r) => r.best)).toFixed(1)} (${runs.map((r) => r.best).join(', ')}), ${mean(runs.map((r) => r.inHub)).toFixed(1)} in the HUB's opening (${runs.map((r) => r.n - r.inHub).join(', ')} missed); the first shot ${mean(runs.map((r) => r.first)).toFixed(2)} s in, the motors down to ${mean(runs.map((r) => r.minRpm)).toFixed(0)} RPM, held off ${mean(runs.map((r) => r.heldOff)).toFixed(2)} s; the box in (to 6.3 turns) in ${mean(runs.map((r) => r.inTime)).toFixed(2)} s (${runs.map((r) => r.inTime.toFixed(2)).join(', ')})`);
  },
};

// driving about with the load in it, the robot's mechanisms fixed in the world: what its FUEL feel
// in its frame (the bench's robot doesn't move): full drive forward and back (SHAKE_A, 8 m/s^2,
// about what its tires give), hard to each side, and turning on the spot (SHAKE_W, 6 rad/s, the
// load flung out from the robot's center), 1 s each, then 1 s still
function shake(rg, w, balls, esc, name) {
  const A = Number(process.env.SHAKE_A ?? 8), W = Number(process.env.SHAKE_W ?? 6), m = FUEL.mass;
  const n0 = balls.filter((b) => inIt(b, rg.ext)).length, before = esc.out.size;
  const phases = [[A, 0], [-A, 0], [0, A], [0, -A], ['spin', 1], ['spin', -1], [0, 0]];
  for (const [ax, az] of phases) {
    for (let k = 0; k < 1 / DT; k++) {
      for (const b of balls) {
        const p = b.translation();
        if (p.x > 40 || p.y > 3) continue;
        // (in the robot's frame: the push back against its acceleration; spinning, out from its center
        // and the Coriolis push on what moves in it)
        let fx = -ax * m, fz = -az * m;
        if (ax === 'spin') { const v = b.linvel(); fx = m * (W * W * p.x - 2 * az * W * v.z); fz = m * (W * W * p.z + 2 * az * W * v.x); }
        b.applyImpulse({ x: fx * DT, y: 0, z: fz * DT }, true);
      }
      rg.path.step(DT, balls, balls); slide(rg, {}); moveSlide(rg); w.step(); esc.check(balls);
    }
  }
  const n1 = balls.filter((b) => inIt(b, rg.ext)).length;
  console.log(`  driven about (${name}, ${A} m/s^2, spun at ${W} rad/s): ${n1} of ${n0} still in it, ${esc.out.size - before} escaped; in all ${esc.report()}`);
}
function intakeFull(seed, { keep = false, sec = Number(process.env.SEC ?? 10), until = null } = {}) {
  const rg = rig(), { t, path } = rg, esc = escapeWatch(rg);
  t.intake.cmd = 1;
  // their RunIntakeBallevator: after 3 s the floor at 3 V while the hood CANrange sees nothing (a
  // FUEL up the elevator: js/mech/r1114.js RANGE)
  const control = (time, live) => {
    esc.check(live);
    const staged = live.some((b) => { const p = b.translation(); return Math.abs(p.x - RANGE.at[0]) < RANGE.half && p.y > RANGE.at[1] && p.y < 0.8; });
    t.floor.cmd = time > 3 && !staged ? 3 / 12 : null;
    slide(rg, {}); moveSlide(rg);
  };
  const r = fillRun(RAPIER, { world: () => world({ carpet: false }), ball, path, DT, held: (p) => heldP(p, rg.ext), own: (p) => ownP(p, rg.ext), top: TOP, control, speed: SPEED, sec, seed, front: 0.31 + SLIDE.out + 0.03, keep, until });
  return { ...r, rig: rg, esc };
}

const CYCLE = process.env.AGITATE === 'cycle', BOTH_FOR = process.env.BOTHFOR ? Number(process.env.BOTHFOR) : process.env.BOTH ? Infinity : 0;
function dumpRun(seed) {
  const D = Number(process.env.DIST ?? 3.02), want = shotFor('hub', D);
  let rg, w, balls;
  if (process.env.FILL === 'layers') {
    const ext = process.env.EXT ? Number(process.env.EXT) : SLIDE.out;
    rg = rig({ hood: want.hood, ext }); w = world(); balls = [];
    fill(w, balls, (n) => { for (let i = 0; i < n; i++) { rg.path.step(DT, balls, balls); w.step(); } }, { ext });
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
  // (the roller let go, the flywheel up to the shot's speed on the way to the HUB: 3 s; SPINUP=s
  // gives it s instead, 0 from rest: their Shoot feeds once it's within 500 RPM)
  const spin = Number(process.env.SPINUP ?? 3);
  t.intake.cmd = null; t.floor.cmd = null;
  for (let i = 0; i < 3 / DT; i++) { if (process.env.IDLE) flywheel(t, SHOOTER.idle); else if (i >= (3 - spin) / DT) flywheel(t, want.rpm); step(() => slide(rg, {})); }
  flywheel(t, want.rpm);
  const n0 = balls.filter((b) => ownP(pos(b), rg.ext)).length;
  const watch = shotWatch(shooter);
  let minRpm = Infinity, tallyN = 0, gateT = -1, heldOff = 0, inT0 = -1, inMid = -1, inT1 = -1;
  let pushedT = -1, pushOut = false;
  const tally = new Map();
  for (let k = 0; k < 8 / DT; k++) {
    const T = k * DT, rpm = RPM(t.shooter.omega), BOTH = T < BOTH_FOR;
    const fire = rpm >= want.rpm - 500, aimed = !process.env.NOAGITATE && !BOTH && rpm >= want.rpm - 400 && T > 0.65;
    const agit = aimed && (CYCLE || slideTurns(rg.ext) > SLIDE_POS.agitateMin);
    feed(t, fire);
    t.intake.cmd = BOTH ? 1 : agit && !process.env.NOROLL && slideTurns(rg.ext) >= SLIDE_POS.rollersOff ? 6 / 12 : null;
    // (BOTH=1: intake and shoot both held, as play has them: their RunIntakeBallevator on the box,
    // the roller at 12 V, the box coasting out unless pushed in under 13 turns for 0.5 s, then
    // 20 A out until it's past 13.4; no AgitateIntake. BOTHFOR=s: for the dump's first s seconds,
    // then the intake let go: AgitateIntake, as the AI does)
    if (BOTH) {
      const turns = slideTurns(rg.ext);
      if (turns < SLIDE_POS.redeploy || pushOut) { pushedT = pushedT < 0 ? T : pushedT; if (T - pushedT > 0.5) pushOut = turns <= SLIDE_POS.out; } else pushedT = -1;
      step(() => slide(rg, pushOut ? { amps: 20 } : {}));
    }
    // (AGITATE=cycle: their commented-out last mode, the box's setpoint 6.4 in and 9 in by turns, 240 ms each)
    else if (CYCLE && agit) step(() => slide(rg, { turns: Math.floor((T - 0.65) / 0.24) % 2 ? 9 : 6.4 }));
    else if (process.env.FORCEIN && agit) step(() => { t.slide.cmd = null; t.slide._omega = -Number(process.env.FORCEIN) / SLIDE.pinion; t.slide.lo = t.slide.hi = t.slide._omega; });
    else { t.slide.lo = -Infinity; t.slide.hi = Infinity; step(() => slide(rg, { volts: agit ? (process.env.AGITATE_V ? -Number(process.env.AGITATE_V) : SLIDE.pull) : 0 })); }
    watch.check(balls, T);
    if (fire && gateT < 0) gateT = T;
    if (watch.shots.length && !fire) heldOff += DT;
    // (how long the box takes to come in: from their AgitateIntake's first pull to 10.8 turns and
    // to their 6.3)
    if (agit && inT0 < 0) inT0 = T;
    if (inT0 >= 0 && inMid < 0 && slideTurns(rg.ext) <= SLIDE_POS.rollersOff) inMid = T;
    if (inT0 >= 0 && inT1 < 0 && slideTurns(rg.ext) <= SLIDE_POS.agitateMin + 0.05) inT1 = T;
    if (watch.shots.length) minRpm = Math.min(minRpm, RPM(t.shooter.omega));
    // (DUMPLOG=1: where the load is each 0.25 s: up the elevator, at its foot, over the floor's back
    // slope and valley, on the floor, in the box out front, and the elevator's and floor's speeds)
    if (process.env.DUMPSNAP && k === Math.round(Number(process.env.DUMPSNAP) / DT)) {
      for (const b of balls) { const p = b.translation(); if (p.y > 3 || p.x > 40) continue; console.log(`   at ${p.x.toFixed(3)},${p.y.toFixed(3)},${p.z.toFixed(3)} v ${b.linvel().y.toFixed(2)}: ${[...path.contacts, ...shooter.contacts].filter((c) => c.ball === b).map((c) => c.wheel.name + ' ' + c.force.toFixed(0)).join(', ')}`); }
      for (const c of path.contacts.filter((q) => q.wheel === rg.s.slide)) { const p = c.ball.translation(); console.log(`   on the box: FUEL at ${p.x.toFixed(2)},${p.y.toFixed(2)},${p.z.toFixed(2)} contact at ${c.x.map((v) => v.toFixed(2)).join(',')} n ${c.n.map((v) => v.toFixed(2)).join(',')} ${c.force.toFixed(0)} N, ${(c.torque / SLIDE.pinion).toFixed(0)} N along its racks`); }
    }
    // (DUMPTALLY=t0,t1: what holds the load up from t0 to t1: each wheel's and surface's lift on
    // the FUEL in it, on average)
    if (process.env.DUMPTALLY) {
      const [t0, t1] = process.env.DUMPTALLY.split(',').map(Number);
      if (T >= t0 && T < t1) {
        for (const c of [...path.contacts, ...shooter.contacts]) { const k2 = c.wheel.name.replace(/ \d+ /, ' ') + (process.env.TALLYAT && /CAD/.test(c.wheel.name) ? ` @${(Math.round(c.x[0] * 20) / 20).toFixed(2)},${(Math.round(c.x[1] * 20) / 20).toFixed(2)},|z|${(Math.round(Math.abs(c.x[2]) * 10) / 10).toFixed(1)}` : ''); tally.set(k2, (tally.get(k2) ?? 0) + (c.force * c.n[1] * DT) / (t1 - t0)); }
        tallyN += balls.filter((b) => inIt(b, rg.ext) && b.translation().y < 0.8).length * DT / (t1 - t0);
      }
      if (Math.abs(T - t1) < DT / 2) console.log(`   lifting the load (${tallyN.toFixed(1)} in it on average, ${(tallyN * FUEL.mass * 9.81).toFixed(1)} N): ` + [...tally].sort((a, b) => b[1] - a[1]).map(([n, f]) => `${n} ${f.toFixed(1)}`).join(', '));
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
  if (process.env.BINS) { const bins = []; for (const o of s) { const i = Math.floor((o - first) / 0.25); bins[i] = (bins[i] || 0) + 1; } console.log('   shots each 0.25 s: ' + Array.from(bins, (v) => v || 0).join(' ')); const miss = []; for (const o of watch.shots) { const d = landing(o); if (!(Math.abs(d - D) < HUB_R)) { const i = Math.floor((o.t - first) / 0.25); miss[i] = (miss[i] || 0) + 1; } } console.log('   missed each 0.25 s: ' + Array.from(miss, (v) => v || 0).join(' ') + '; first shot ' + first.toFixed(2) + ' s in'); }
  const land = watch.shots.map((o) => landing(o)).filter(Number.isFinite).sort((a, b) => a - b);
  const inHub = land.filter((d) => Math.abs(d - D) < HUB_R).length;
  const sh = watch.shots;
  console.log(`  (shots leave at ${med(sh.map((o) => o.v)).toFixed(2)} m/s, ${med(sh.map((o) => o.el)).toFixed(1)} deg; come down at the rim ${land[land.length >> 1]?.toFixed(2)} m out (median; ${land[0]?.toFixed(2)}..${land[land.length - 1]?.toFixed(2)}), ${inHub} of ${land.length} in the HUB's opening ${D} m out; the motors down to ${minRpm.toFixed(0)} of ${want.rpm.toFixed(0)} RPM; the box in to ${slideTurns(rg.ext).toFixed(1)} turns${inT0 >= 0 ? `, pulled in from ${inT0.toFixed(2)} s: to 10.8 turns in ${inMid >= 0 ? (inMid - inT0).toFixed(2) + ' s' : '-'}, to 6.3 in ${inT1 >= 0 ? (inT1 - inT0).toFixed(2) + ' s' : '-'}` : ''}; ${left.length} still in it)`);
  console.log(`  (the flywheel at Shoot's gate ${gateT.toFixed(2)} s in, the first shot ${first.toFixed(2)} s in; held off under its gate ${heldOff.toFixed(2)} s after that)`);
  console.log(`1114 dumps ${s.length} of ${n0}${process.env.FILL === 'layers' ? ' (filled a layer at a time)' : ` (filled through its intake, seed ${seed})`}: ${in1} in the first second, best second ${best}, ${rate.toFixed(1)} FUEL/s over the whole dump`);
  w.free();
  return { n: s.length, n0, rate, best, in1, inHub, first, minRpm, heldOff, inTime: inT0 >= 0 && inT1 >= 0 ? inT1 - inT0 : NaN };
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
