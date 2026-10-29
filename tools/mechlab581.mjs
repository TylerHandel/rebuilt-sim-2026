// 581's mechanisms (js/mech/r581.js) on their own, no field, no robot: the CAD's ball path with its
// rollers, measured against what the team's code expects. Run:
//   node --import ./tools/node-env.mjs tools/mechlab581.mjs [test ...]
// Tests: rollers (each roller's free speed at their voltages, the drum's spin-up, the intake
// sliding out and in), shooter (FUEL fed up the tower at their tables' drum speeds and hood angles:
// how fast and how steep they leave, and whether their table's shots come down in the HUB), path
// (FUEL on a treadmill carpet, the robot driving into them at 1 m/s: intake, floor, tower, drum;
// how many and how fast), intake (their INTAKING alone), hold (how many it holds, everything off),
// dump (a full one fired at their 2.8 m shot, with their compaction).
// Env: SEEDS (6), SPEED (1 m/s), STUCK (where the ones left over are), SHOT=rpm,hood (one shot of
// your own), NOCOMPACT=1 (the dump without the intake coming in)
import { RAPIER } from '../js/physics.js';
import { FUEL, GROUP, groups, HUB, IN, TRENCH } from '../js/constants.js';
import { Mechanism } from '../js/mech/mechanism.js';
import { sim2D } from '../js/ballistics.js';
import {
  build581, wheels581, pathSurfaces581, placeIntake, placeFloor, placeFloorSurface, placeHood, placeNet, slideOffset, floorRot, hoodRot, hoodElevation, shotFor,
  SCORE_RPM, FEED_RPM, SLIDE, FLOOR, TOWER, INTAKE, SHOOTER, DEPLOY, DRUM, NET,
} from '../js/mech/r581.js';
import PATH from '../js/mech/path581.js';
import { EXIT_PER_DRUM_SURFACE, dutyFor } from '../js/mech/phys581.js';
import { fillRun, fillReport } from './fillbench.mjs';

await RAPIER.init();
const R = FUEL.radius, DT = 1 / 120, SEEDS = Number(process.env.SEEDS ?? 6), SPEED = Number(process.env.SPEED ?? 1);
const HELD = groups(GROUP.HELD, 0xffff & ~GROUP.HELD);
const OUT = DEPLOY.INTAKE * IN; // their INTAKE: 11.8in out
// (BULGE=m, TREAD=N/m: the net's slack and give, js/mech/r581.js NET, to try others)
if (process.env.BULGE) NET.bulge = Number(process.env.BULGE);
if (process.env.TREAD) NET.tread = Number(process.env.TREAD);

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

// the robot's mechanisms, fixed in the world: the intake ext m out (the floor where its ramps put
// it), the hood at their angle hood
function rig({ ext = OUT, hood = SHOOTER.hoodMin } = {}) {
  const t = build581(), wh = wheels581(t), s = pathSurfaces581(PATH, t);
  const pathW = [...wh.floor, ...wh.tower, ...wh.intake];
  const path = new Mechanism({ substeps: 6, shafts: [...new Set(pathW.map((x) => x.shaft)), t.deploy], wheels: pathW, surfaces: Object.values(s) });
  const shooter = new Mechanism({ substeps: 6, sub: 8, shafts: [t.drum, t.hood], wheels: [wh.drum, ...wh.hood] });
  for (const tr of [t.intake, t.floor, t.feederT, t.shooter]) tr.cmd = null;
  const r = { t, wh, s, path, shooter, ext, hood };
  r.pose = () => {
    const [dx, dy] = slideOffset(r.ext);
    s.slide.setPose([dx, dy, 0]);
    placeIntake(wh.intake, r.ext);
    placeFloorSurface(s.floor, r.ext);
    placeFloor(wh.floor, r.ext);
    s.hood.setPose([DRUM[0], DRUM[1], 0], 0, hoodRot(r.hood));
    placeHood(wh.hood, r.hood);
    placeNet(s.net, r.ext);
  };
  r.pose();
  t.deploy.cmd = 0;
  return r;
}
// their velocity loop (torque current) on the drum toward rpm, plus ff amps
function drum(t, rpm, ff = 0) {
  const rps = t.drum.omega / (2 * Math.PI), tgt = rpm / 60;
  t.shooter.cmd = dutyFor(t.shooter.motor, Math.min(200, SHOOTER.kS + SHOOTER.kV * tgt + SHOOTER.kP * (tgt - rps) + ff), t.shooter.omega);
}
// the deploy's position loop toward inches (torque current; Motion Magic when mm), as phys581 runs
// it; and the intake moving as its pinions turn
function deploy(r, inches, mm) {
  const t = r.t;
  let goal = inches;
  if (mm) {
    const p = r.mm ?? [r.ext / IN, 0], d = goal - p[0], dir = Math.sign(d);
    const want = dir * Math.min(DEPLOY.mmV, Math.sqrt(2 * DEPLOY.mmA * Math.abs(d)));
    p[1] += Math.max(-DEPLOY.mmA * DT, Math.min(DEPLOY.mmA * DT, want - p[1])); p[0] += p[1] * DT;
    if ((goal - p[0]) * dir < 0) { p[0] = goal; p[1] = 0; }
    r.mm = p; goal = p[0];
  } else r.mm = null;
  t.deploy.cmd = dutyFor(t.deploy.motor, Math.max(-35, Math.min(100, DEPLOY.kP * (goal - r.ext / IN))), t.deploy._omega);
}
function moveDeploy(r) {
  r.ext = Math.max(0, Math.min(SLIDE.out, r.ext + r.t.deploy.omega * SLIDE.pinion * DT));
  if (r.ext <= 0 || r.ext >= SLIDE.out) r.t.deploy._omega = 0;
  r.pose();
}
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
const inHopper = (b, ext = OUT) => { const p = b.translation(); return p.x > -0.36 && p.x < 0.36 + ext && Math.abs(p.z) < 0.37 && p.y > 0.08 && p.y < 0.75; };
// as js/mech/phys581.js where() has it: in it (held), and the intake's (own: in its mouth too)
const HALF_L = 0.343, HALF_W = 0.362;
const inFrame = ([x, y, z]) => Math.abs(x) < HALF_L && Math.abs(z) < HALF_W && y > 0.09 && y < 0.75;
const inFront = (ext) => ([x, y, z]) => x >= HALF_L && x < HALF_L + ext + 0.05 && Math.abs(z) < 0.345 && y < 0.56;
const held581 = (ext) => (p) => inFrame(p) || (inFront(ext)(p) && p[1] > 0.2);
const own581 = (ext) => (p) => inFrame(p) || inFront(ext)(p);
const P = (b) => { const p = b.translation(); return [p.x, p.y, p.z]; };
const LIMIT = 30 * IN; // the height limit (30in)
const TOWER_X = -0.06; // the hopper's back: the shooter tower's front (the top's back edge, their CAD)
// the net's height over (x, z) (as far as its slack lets the load lift it), or null where it isn't
const netTop = (r, x, z) => (r.s.net.under(x, z) ? r.s.net.at(x, z)[0] : null);
// FUEL that got out of it, not as shots (each once it's been in it): up through the net (its
// center 3 cm over the net there: past where the net holds it), over the hopper's top elsewhere
// (its center over the walls' and the front plate's tops), out a side, out the front over the
// front plate, out through the drum and hood (up the tower and out over them, too slow to count as
// a shot), or out the back below them; and how high the load stands (the top of any FUEL in the
// hopper, ahead of the tower, the most it got; and under the net), and how many it held when the load first
// stood taller than a TRENCH arm's underside (22.25in) for a quarter second, or the 30in limit
function escapeWatch(r) {
  const seen = new Set(), out = new Map();
  let top = 0, topNet = 0, tallT = 0, tallAt = null, overAt = null;
  return {
    out,
    get top() { return top; }, get topNet() { return topNet; }, get tallAt() { return tallAt; }, get overAt() { return overAt; },
    check(balls) {
      const isHeld = held581(r.ext);
      let now = 0, held = 0;
      const [dx] = slideOffset(r.ext), front = NET.frontX + dx;
      for (const b of balls) {
        const p = P(b), [x, y, z] = p;
        // (the load's top: in the hopper, not one in the tower)
        if (isHeld(p)) { held++; if (x > TOWER_X) now = Math.max(now, y + R); }
        if (out.has(b) || x > 40) continue;
        if (!seen.has(b)) { if (isHeld(p)) seen.add(b); else continue; }
        const h = netTop(r, x, z);
        if (isHeld(p) && x > TOWER_X) { top = Math.max(top, y + R); if (h !== null) topNet = Math.max(topNet, y + R); }
        if (b.linvel().y > 2 && x < -0.03) continue; // (on its way up into the drum)
        // (out the back over the tower: through the drum and hood, idling or not, or over them)
        const why = h !== null && y > h + 0.03 ? 'through the net' : h === null && y > 0.56 && x > TOWER_X ? 'over the top' : x < TOWER_X && y > 0.6 ? 'out through the drum'
          : Math.abs(z) > HALF_W + 0.03 && y > 0.2 ? 'out a side' : x > front + 0.03 && y > 0.3 ? 'out the front' : x < -HALF_L - 0.05 && y > 0.3 ? 'over the back' : null;
        if (why) out.set(b, why);
        if (why && process.env.ESC) { const v = b.linvel(); console.log(`   escaped (${why}) at ${x.toFixed(3)},${y.toFixed(3)},${z.toFixed(3)} v ${v.x.toFixed(1)},${v.y.toFixed(1)},${v.z.toFixed(1)}; the net there ${h === null ? '-' : h.toFixed(3)}; the intake ${(r.ext / IN).toFixed(1)}in out`); }
      }
      tallT = now > TRENCH.clearHeight ? tallT + DT : 0;
      if (tallAt === null && tallT >= 0.25) tallAt = held;
      if (overAt === null && now > LIMIT) overAt = held;
    },
    report() {
      const c = {};
      for (const w of out.values()) c[w] = (c[w] ?? 0) + 1;
      return `${out.size} escaped${out.size ? ` (${Object.entries(c).map(([k, n]) => `${k}: ${n}`).join(', ')})` : ''}; the load's top ${top.toFixed(3)} m (under the net ${topNet.toFixed(3)})` +
        `${tallAt !== null ? `, taller than the TRENCH from ${tallAt} in it` : ', never taller than the TRENCH'}${overAt !== null ? `, over 30in from ${overAt}` : ''}`;
    },
  };
}
// fill it a layer at a time (FUEL set into the space over the ones already in, let settle)
function fill(w, balls, step, layers = 8, y = 0.47, x1 = 0.6) {
  for (let layer = 0; layer < layers; layer++) {
    for (let i = 0; i < 7; i++) for (let k = 0; k < 4; k++) {
      const x = -0.03 + 0.155 * i, z = -0.233 + 0.155 * k;
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
    t.intake.cmd = 1; t.floor.cmd = 10 / 12; t.feederT.cmd = 10 / 12;
    for (let i = 0; i < 2 / DT; i++) { drum(t, 2500); path.step(DT, []); shooter.step(DT, []); }
    const surf = (sh, r) => Math.abs(sh.omega) * r;
    console.log(`581's rollers, free (intake 12 V, floor and tower at their SHOOT 10 V, the drum at 2500 RPM, their longest feed):`);
    console.log(`  intake X60s ${RPM(t.intake.omega).toFixed(0)} rpm: 3in roller ${surf(t.big, INTAKE.big.r).toFixed(2)} m/s at its surface, 2in ${surf(t.small, INTAKE.small.r).toFixed(2)} m/s`);
    console.log(`  floor X60s ${RPM(t.floor.omega).toFixed(0)} rpm: 1in rollers ${surf(t.rollers[0], FLOOR.r).toFixed(2)} m/s`);
    console.log(`  tower X60s ${RPM(t.feederT.omega).toFixed(0)} rpm: 1.5in rollers ${surf(t.tower[0], TOWER.r).toFixed(2)} m/s`);
    console.log(`  shooter X60s ${RPM(t.shooter.omega).toFixed(0)} rpm: drum ${surf(t.drum, SHOOTER.drum.r).toFixed(2)} m/s; hood rollers ${surf(t.hood, SHOOTER.hoodR).toFixed(2)} m/s`);
    for (const [v, name] of [[1, 'BALL_FILLING'], [2, 'INTAKE (floor)'], [-1, 'INTAKING (tower)']]) {
      const r2 = rig(); r2.t.floor.cmd = v / 12; r2.t.feederT.cmd = v / 12;
      for (let i = 0; i < 1 / DT; i++) r2.path.step(DT, []);
      console.log(`  at ${v} V (${name}): floor ${surf(r2.t.rollers[0], FLOOR.r).toFixed(2)} m/s, tower ${surf(r2.t.tower[0], TOWER.r).toFixed(2)} m/s`);
    }
    for (const target of [750, 1560, 1900]) {
      const r2 = rig();
      let T = 0;
      for (; T < 8; T += DT) { drum(r2.t, target); r2.shooter.step(DT, []); if (RPM(r2.t.drum.omega) > target - 50) break; }
      console.log(`  the drum from rest to within 50 of ${target} RPM: ${T.toFixed(2)} s`);
    }
    // the deploy: homing out (4 V), then in to their compaction and back out
    const r3 = rig({ ext: 0 });
    let T = 0;
    for (; T < 3 && r3.ext < SLIDE.out - 0.002; T += DT) { r3.t.deploy.cmd = DEPLOY.homeV / 12; r3.path.step(DT, []); moveDeploy(r3); }
    console.log(`  the intake homing out at 4 V: all the way out in ${T.toFixed(2)} s`);
    const run = (inches, mm) => { let k = 0; for (; k < 3 / DT; k++) { deploy(r3, inches, mm); r3.path.step(DT, []); moveDeploy(r3); if (Math.abs(r3.ext / IN - inches) < DEPLOY.tol && Math.abs(r3.t.deploy.omega) < 1) break; } return k * DT; };
    run(DEPLOY.INTAKE);
    const a = run(DEPLOY.SCORE_COMPACTION_WAITING, true), b = run(5, true);
    console.log(`  in to their 8.3in (Motion Magic) in ${a.toFixed(2)} s, then to 5in in ${b.toFixed(2)} s: the floor up ${(floorRot(r3.ext) * 180 / Math.PI).toFixed(1)} deg; back out to 11.8in in ${run(DEPLOY.INTAKE).toFixed(2)} s`);
  },
  // FUEL up the tower into the drum at their tables' rows
  shooter() {
    console.log(`581's shooter, FUEL fed up the tower four at a time (floor and tower 10 V, their tables' RPM and hood):`);
    const rows = [...SCORE_RPM.map(([d]) => ['score', d]), ['score', 2.38], ...FEED_RPM.map(([d]) => ['feed', d])].sort((a, b) => (a[0] === b[0] ? a[1] - b[1] : a[0] < b[0] ? 1 : -1));
    if (process.env.SHOT) { const [rpm, hood] = process.env.SHOT.split(',').map(Number); rows.splice(0, rows.length, ['custom', 0, { rpm, hood }]); }
    for (const [kind, d, custom] of rows) {
      const want = custom ?? shotFor(kind, d);
      const { t, path, shooter } = rig({ hood: want.hood });
      const w = world();
      for (let i = 0; i < 3 / DT; i++) { drum(t, want.rpm); shooter.step(DT, []); }
      t.feederT.cmd = 10 / 12; t.floor.cmd = 10 / 12;
      const balls = [-0.225, -0.075, 0.075, 0.225].map((z) => ball(w, -0.17, 0.21, z));
      const watch = shotWatch(shooter);
      for (let i = 0; i < 3 / DT && watch.shots.length < balls.length; i++) {
        drum(t, want.rpm, 15);
        path.step(DT, balls, balls); shooter.step(DT, balls); w.step();
        watch.check(balls, i * DT);
      }
      w.free();
      const outs = watch.shots;
      if (!outs.length) { console.log(`  ${kind} ${d} m (${want.rpm.toFixed(0)} RPM, hood ${want.hood}): nothing came out`); continue; }
      const o = { v: med(outs.map((q) => q.v)), el: med(outs.map((q) => q.el)), x: med(outs.map((q) => q.x)), y: med(outs.map((q) => q.y)) };
      const surf = t.drum.omega * SHOOTER.drum.r;
      const land = landing(o) - o.x;
      console.log(`  ${kind} ${d} m (${want.rpm.toFixed(0)} RPM, hood ${want.hood.toFixed(1)}): ${outs.length}/4 out at ${o.v.toFixed(2)} m/s (${(o.v / surf).toFixed(2)} of the drum's ${surf.toFixed(1)}; the estimate ${EXIT_PER_DRUM_SURFACE}), ${o.el.toFixed(1)} deg (${hoodElevation(want.hood).toFixed(1)} by the CAD's geometry) from ${o.x.toFixed(2)},${o.y.toFixed(2)}, ` +
        (kind !== 'feed' ? `comes down at the rim ${land.toFixed(2)} m back (HUB's center ${d} m: ${land - d > 0 ? '+' : ''}${(land - d).toFixed(2)})` : `comes down to the carpet ~${(sim2D(o.y, o.v, (o.el * Math.PI) / 180, 0.1)?.d - o.x).toFixed(1)} m back`));
    }
  },
  // carpet to shot: FUEL in front, the robot driving into them; intaking and shooting at 2.8 m
  path() {
    const runs = [];
    for (let seed = 1; seed <= SEEDS; seed++) runs.push(pathRun(seed));
    console.log(`581 carpet to shot (20 FUEL, four abreast, the robot at ${SPEED} m/s, the intake at 12 V and the floor and tower at 10 V, their SCORE_AND_INTAKE): a median ${med(runs.map((r) => r.shots))} shot (${Math.min(...runs.map((r) => r.shots))}..${Math.max(...runs.map((r) => r.shots))}), ` +
      `${med(runs.map((r) => r.rate)).toFixed(1)} FUEL/s once they're coming; ${med(runs.map((r) => r.inside))} in it at the end; intake X60s down to ${med(runs.map((r) => r.minIntake)).toFixed(0)}%, floor to ${med(runs.map((r) => r.minFloor)).toFixed(0)}%`);
    if (process.env.STUCK) for (const r of runs) console.log('   left', r.left.map((p) => p.map((v) => v.toFixed(2)).join(',')).join(' | '));
  },
  // their INTAKING alone (rollers 12 V, floor 2 V, tower -1 V), FUEL on the treadmill
  intake() {
    for (let seed = 1; seed <= Math.min(SEEDS, 3); seed++) {
      const r = pathRun(seed, { shoot: false, n: 24, sec: 6 });
      console.log(`581 intaking only (their INTAKING: rollers 12 V, floor 2 V, tower -1 V), 24 FUEL at ${SPEED} m/s: ${r.inside} in it after 6 s (in past the bumper each second: ${r.timeline.join(', ')}); intake X60s down to ${r.minIntake.toFixed(0)}%`);
      if (process.env.STUCK) console.log('   left', r.left.map((p) => p.map((v) => v.toFixed(2)).join(',')).join(' | '));
    }
  },
  // filled through the intake until it can't put more in (tools/fillbench.mjs): the NEUTRAL ZONE's
  // FUEL, five abreast, at SPEED; the intake out, their INTAKING (rollers 12 V, floor 2 V, tower
  // -1 V), the drum idling; full when FUEL wait in its mouth pressed against the load and fewer than
  // 2 more get in for 2 s (js/mech/physical.js FullWatch; it's rated, not naturalFull, in play)
  async intakefull() {
    const runs = [];
    for (let seed = 1; seed <= SEEDS; seed++) {
      const r = rig(), esc = escapeWatch(r), { t } = r;
      t.intake.cmd = 1; t.floor.cmd = 2 / 12; t.feederT.cmd = -1 / 12;
      const path = { step: (dt, l) => { drum(t, SHOOTER.idle); r.path.step(dt, l, l); r.shooter.step(dt, l); esc.check(l); }, get contacts() { return r.path.contacts; } };
      const res = fillRun(RAPIER, { world: () => world({ carpet: false }), ball, path, DT, held: held581(r.ext), own: own581(r.ext), top: { topX: 0, topN: Infinity }, speed: SPEED, sec: Number(process.env.SEC ?? 12), seed, front: 0.62, keep: !!process.env.SNAP });
      // (SNAP=file: where the FUEL in it are at the end (the first fill's), for a picture)
      if (process.env.SNAP && seed === 1) (await import('node:fs')).writeFileSync(process.env.SNAP, JSON.stringify({ ext: r.ext, load: res.balls.map(P).filter(held581(r.ext)) }));
      if (res.world) res.world.free();
      console.log(fillReport(`581 intaking at ${SPEED} m/s, seed ${seed}`, res) + `; ${esc.report()}`);
      runs.push({ ...res, esc: esc.out.size, loadTop: esc.top, tallAt: esc.tallAt, overAt: esc.overAt });
    }
    const f = runs.map((x) => x.fulls[0]?.n).filter((n) => n !== undefined);
    console.log(`581 through its intake (${runs.length} fills): the most it held ${Math.min(...runs.map((x) => x.most))}-${Math.max(...runs.map((x) => x.most))} (mean ${(runs.reduce((a, x) => a + x.most, 0) / runs.length).toFixed(1)}), full at ${f.length ? `${Math.min(...f)}-${Math.max(...f)}` : '-'} (${f.length} of ${runs.length}); ${runs.reduce((a, x) => a + x.esc, 0)} escaped in all; the load's top ${Math.min(...runs.map((x) => x.loadTop)).toFixed(3)}-${Math.max(...runs.map((x) => x.loadTop)).toFixed(3)} m, taller than the TRENCH from ${runs.map((x) => x.tallAt ?? '-').join(', ')} in it; over 30in from ${runs.map((x) => x.overAt ?? '-').join(', ')}`);
  },
  async hold() {
    const r = rig(), { path } = r, esc = escapeWatch(r);
    const w = world();
    const balls = [];
    const step = (n) => { for (let i = 0; i < n; i++) { path.step(DT, balls, balls); w.step(); esc.check(balls); } };
    fill(w, balls, step, 8);
    step(2 / DT);
    // (SNAP=file: where the FUEL in it are, for a picture)
    if (process.env.SNAP) (await import('node:fs')).writeFileSync(process.env.SNAP, JSON.stringify({ ext: r.ext, load: balls.map(P).filter(held581(r.ext)) }));
    const t0 = performance.now();
    step(1 / DT);
    const ms = (performance.now() - t0) / (1 / DT);
    console.log(`  (its mechanisms with them all in: ${ms.toFixed(2)} ms a step)`);
    console.log(`581 holds ${balls.filter((b) => inHopper(b)).length} with everything off and the intake out (a layer at a time); ${esc.report()}`);
    const lo = rig({ ext: DEPLOY.STOW * IN });
    const w2 = world(), b2 = [];
    const step2 = (n) => { for (let i = 0; i < n; i++) { lo.path.step(DT, b2, b2); w2.step(); } };
    fill(w2, b2, step2, 6, 0.47, 0.25);
    step2(2 / DT);
    console.log(`581 holds ${b2.filter((b) => inHopper(b, DEPLOY.STOW * IN)).length} with the intake in to their 5in (the floor up ${(floorRot(DEPLOY.STOW * IN) * 180 / Math.PI).toFixed(1)} deg)`);
  },
  // a full one fired: filled as hold does, then their SCORE at 2.8 m: floor and tower 10 V, the
  // intake 5 V and in to 8.3in, after 0.7 s in to 5in (compaction)
  dump() {
    const want = shotFor('score', 2.79);
    const r = rig({ hood: want.hood });
    const { t, path, shooter } = r, esc = escapeWatch(r);
    const w = world();
    const balls = [];
    const step = (n, run) => { for (let i = 0; i < n; i++) { if (run) run(); path.step(DT, balls, balls); shooter.step(DT, balls); w.step(); esc.check(balls); } };
    fill(w, balls, (n) => step(n, () => { drum(t, want.rpm); deploy(r, DEPLOY.INTAKE); }), 8);
    const n0 = balls.filter((b) => inHopper(b)).length;
    const watch = shotWatch(shooter);
    t.floor.cmd = 10 / 12; t.feederT.cmd = 10 / 12; t.intake.cmd = 5 / 12;
    let minRpm = Infinity;
    for (let k = 0; k < 8 / DT; k++) {
      const T = k * DT, compact = !process.env.NOCOMPACT && T >= 0.7;
      if (compact) t.intake.cmd = null;
      step(1, () => { drum(t, want.rpm, T < 0.7 ? 20 : 0); deploy(r, process.env.NOCOMPACT ? DEPLOY.INTAKE : compact ? 5 : DEPLOY.SCORE_COMPACTION_WAITING, !process.env.NOCOMPACT); });
      moveDeploy(r);
      watch.check(balls, T);
      if (watch.shots.length) minRpm = Math.min(minRpm, RPM(t.drum.omega));
    }
    const s = watch.shots.map((x) => x.t), first = s[0] ?? 0, in1 = s.filter((x) => x - first < 1).length;
    const best = s.reduce((m, x, i) => Math.max(m, s.slice(i).filter((y) => y < x + 1).length), 0);
    const left = balls.filter((b) => b.translation().y < 3 && b.translation().x < 40 && inHopper(b));
    if (process.env.STUCK) for (const b of left) { const p = b.translation(); console.log(`   left at ${p.x.toFixed(2)},${p.y.toFixed(2)},${p.z.toFixed(2)}: ${[...path.contacts, ...shooter.contacts].filter((c) => c.ball === b).map((c) => c.wheel.name + ' ' + c.force.toFixed(0)).join(', ')}`); }
    const land = watch.shots.map((o) => landing(o) - o.x).filter(Number.isFinite);
    const sh = watch.shots;
    console.log(`  (shots leave at ${med(sh.map((o) => o.v)).toFixed(2)} m/s (${Math.min(...sh.map((o) => o.v)).toFixed(2)}..${Math.max(...sh.map((o) => o.v)).toFixed(2)}), ${med(sh.map((o) => o.el)).toFixed(1)} deg (${Math.min(...sh.map((o) => o.el)).toFixed(1)}..${Math.max(...sh.map((o) => o.el)).toFixed(1)}))`);
    console.log(`  (${left.length} still in it, the intake ${(r.ext / IN).toFixed(1)}in out, the floor up ${(floorRot(r.ext) * 180 / Math.PI).toFixed(1)} deg; the drum down to ${minRpm.toFixed(0)} RPM of ${want.rpm.toFixed(0)}; shots come down at the rim ${med(land).toFixed(2)} m back (median; ${Math.min(...land).toFixed(2)}..${Math.max(...land).toFixed(2)}), the HUB's center 2.79)`);
    console.log(`581 dumps ${s.length} of ${n0}: ${in1} in the first second, ${best} in its best, ${((s.length - 1) / (s[s.length - 1] - first)).toFixed(1)} FUEL/s over the whole dump; ${esc.report()}`);
  },
};

function pathRun(seed, { shoot = true, n = 20, sec = 7 } = {}) {
  let sd = seed;
  const rnd = () => ((sd = (sd * 16807) % 2147483647) / 2147483647 - 0.5);
  const want = shotFor('score', 2.79);
  const { t, path, shooter } = rig({ hood: want.hood });
  const w = world({ carpet: false });
  // the carpet, running under the robot
  const carpet = w.createRigidBody(RAPIER.RigidBodyDesc.kinematicVelocityBased().setLinvel(-SPEED, 0, 0));
  w.createCollider(RAPIER.ColliderDesc.cuboid(15, 0.01, 1).setTranslation(0.4 + 15, -0.01, 0).setFriction(0.3).setRestitution(0.2), carpet);
  w.createCollider(RAPIER.ColliderDesc.cuboid(0.4, 0.01, 1).setTranslation(0, -0.01, 0).setFriction(0.3), w.createRigidBody(RAPIER.RigidBodyDesc.fixed()));
  t.intake.cmd = 1;
  if (shoot) { t.floor.cmd = 10 / 12; t.feederT.cmd = 10 / 12; } else { t.floor.cmd = 2 / 12; t.feederT.cmd = -1 / 12; }
  const run = () => { drum(t, shoot ? want.rpm : SHOOTER.idle); };
  for (let i = 0; i < 3 / DT; i++) { run(); shooter.step(DT, []); path.step(DT, []); }
  const free = { intake: Math.abs(t.intake.omega), floor: Math.abs(t.floor.omega) || 1 };
  const balls = [];
  for (let i = 0; i < n; i++) balls.push(ball(w, 0.85 + 0.17 * Math.floor(i / 4) + 0.02 * rnd(), R + 0.003, ((i % 4) - 1.5) * 0.16 + 0.02 * rnd(), [-SPEED, 0, 0]));
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
    if (k % Math.round(1 / DT) === Math.round(1 / DT) - 1) timeline.push(balls.filter((b) => { const p = b.translation(); return p.y < 3 && p.x < 0.34 && p.y > 0.1; }).length);
    // (TRACE=i follows FUEL i: where it is and what it's touching)
    if (process.env.TRACE && k % 6 === 0) {
      const b = balls[Number(process.env.TRACE)], p = b.translation(), v = b.linvel();
      console.log(`   ${(k * DT).toFixed(2)} s at ${p.x.toFixed(3)},${p.y.toFixed(3)},${p.z.toFixed(3)} v ${v.x.toFixed(2)},${v.y.toFixed(2)},${v.z.toFixed(2)}: ${[...path.contacts, ...shooter.contacts].filter((c) => c.ball === b).map((c) => c.wheel.name + ' ' + c.force.toFixed(0)).join(', ')}`);
    }
  }
  const s = watch.shots.map((x) => x.t);
  const inside = balls.filter((b) => inHopper(b) && b.translation().y < 3);
  const left = balls.filter((b) => b.translation().y < 3 && b.translation().x < 40 && !inside.includes(b)).map((b) => { const p = b.translation(); return [p.x, p.y, p.z]; });
  w.free();
  return { timeline, shots: s.length, rate: s.length > 2 ? (s.length - 1) / (s[s.length - 1] - s[0]) : 0, inside: inside.length, minIntake: 100 * minIntake, minFloor: 100 * minFloor, left };
}

const pick = process.argv.slice(2);
for (const [name, fn] of Object.entries(tests)) if (!pick.length || pick.includes(name)) await fn();
