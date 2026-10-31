// 2056's mechanisms (js/mech/r2056.js) on their own, no field, no robot: the ball path built from their
// binder and drawings, with its rollers, belts and wheels. Run:
//   node --import ./tools/node-env.mjs tools/mechlab2056.mjs [test ...]
// Tests: rollers (each drive's speed under its loop, the shooter wheel's spin-up), shooter (FUEL pushed
// up the bore into the wheel at hood angles and wheel speeds: exit speed and elevation; the sim's HUB
// table fired as js/mech/phys2056.js fires it, FED=1 fed in a stream by the indexer), calibrate (the
// elevation each hood angle gives and their one exit-speed coefficient, stream-fed: EL_MAP and EXIT_K),
// feed (FUEL laid on the floor, the indexer running and the shooter firing), intake (24 FUEL four
// abreast on a treadmill at SPEED m/s, intaking alone), intakefull (filled through the intake from the
// NEUTRAL ZONE's FUEL until it can't put more in: tools/fillbench.mjs), dump (a full one fired at 3 m:
// FUEL/s, with the intake coming in at RETRACT_AT as phys2056 runs it; RETRACT=0: held out).
// Env: SEEDS (4), SPEED (1 m/s), DS (the distances, m), HS, VS (shooter: hood angles, wheel speeds),
// FED=1, YAW (deg), SEC, SEC2, PASS=1 (dump at the pass speeds), RR (the foam's rolling resistance on
// the floor's rollers), TRACE (shooter: each FUEL's path), STUCK=1 (dump: where the FUEL left sit).
import { RAPIER } from '../js/physics.js';
import { FUEL, GROUP, groups, HUB } from '../js/constants.js';
import { Mechanism } from '../js/mech/mechanism.js';
import { sim2D, ShotTable } from '../js/ballistics.js';
import {
  build2056, wheels2056, shooterWheel, kickerWheel, pathSurfaces2056, placeIntake, placeTurret, hoodElevation, kickAt, floorRoll, ROLL,
  HALF_L, HALF_W, INTAKE, FLOOR, TOP, BELT, VKICK, FEED, SHOOTER, TURRET, NET, WALL, SLIDE,
} from '../js/mech/r2056.js';
import { speedLoop } from '../js/mech/phys1706.js';
import { RPS, FEED_SHARE, FEED_MIN, KICK_RATIO, EXIT, hoodFor, surfFor, HOOD_STOW, FLOOR_INTAKE, RETRACT_AT, driveRacks, moveRacks, driveKick, moveKick } from '../js/mech/phys2056.js';
import { ROBOTS } from '../js/robotConfigs.js';
import { fillRun, fillReport } from './fillbench.mjs';

await RAPIER.init();
if (process.env.RR) ROLL.rr = Number(process.env.RR);
const R = FUEL.radius, DT = 1 / 120, SEEDS = Number(process.env.SEEDS ?? 4), SPEED = Number(process.env.SPEED ?? 1);
const HELD = groups(GROUP.HELD, 0xffff & ~GROUP.HELD);
const DEG = Math.PI / 180, TAU = 2 * Math.PI;
const env = (k, d) => (process.env[k] !== undefined ? Number(process.env[k]) : d);
const med = (a) => { const b = [...a].sort((p, q) => p - q); return b[b.length >> 1]; };
const P = (b) => { const p = b.translation(); return [p.x, p.y, p.z]; };
const rps = (n, sh) => (n * TAU) / sh.motor.ratio;

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

// the robot's mechanisms, fixed in the world: the intake ext of the way out, the turret at yaw (rad: 0
// shoots forward), the hood's arm at h (deg about the axle); the racks' X44 holding the intake (out,
// or in: r.retract)
function rig({ ext = 1, yaw = 0, hood = HOOD_STOW } = {}) {
  const t = build2056(), wh = wheels2056(t), s = pathSurfaces2056(t), sw = shooterWheel(t), kw = kickerWheel(t);
  const pathW = [...wh.intake, ...wh.floor, wh.vkick, ...wh.feed];
  const path = new Mechanism({ substeps: 6, shafts: [...new Set(pathW.map((x) => x.shaft)), ...t.belts, t.slide, t.kickArm], wheels: pathW, surfaces: [s.body, s.bore, s.bottom, s.top, s.carriage, s.kickTube, s.bumper, s.net] });
  const shooter = new Mechanism({ substeps: 6, sub: 8, shafts: [t.wheel, t.kicker], wheels: [sw, kw], surfaces: [s.turret, s.hood] });
  for (const a of [t.intake, t.floor, t.feed, t.shooter, t.vkick, t.kicker, ...t.belts]) a.cmd = null;
  const r = { t, wh, s, sw, kw, path, shooter, ext, yaw, hood, retract: false, cs: { pos: ext * SLIDE.travel }, ks: { kick: kickAt(ext) }, slideAmps: 0 };
  r.pose = () => { placeIntake(s, wh.intake, r.ext, r.ks.kick); s.net.place(Math.max(HALF_L, INTAKE.frame.front - INTAKE.slide[0] * (1 - r.ext))); placeTurret(s, { wheel: sw, kicker: kw }, r.yaw, r.hood); };
  r.step = (balls, together = balls) => {
    driveRacks(r.cs, t.slide, { on: ext > 0.5, retract: r.retract });
    driveKick(r.ks, t.kickArm, r.ext);
    r.pose();
    path.step(DT, balls, together);
    s.net.flex(DT, path.contacts);
    floorRoll(path.contacts, DT);
    r.bowMax = Math.max(r.bowMax || 0, s.net.bow);
    moveRacks(r.cs, t.slide, DT);
    r.ext = r.cs.pos / SLIDE.travel;
    moveKick(r.ks, t.kickArm, DT, r.ext);
    r.slideAmps = Math.max(r.slideAmps, Math.abs(t.slide.motor.current || 0));
    shooter.step(DT, balls.filter((b) => { const [x, y, z] = P(b); return y > 0.33 && Math.hypot(x - TURRET.at[0], z - TURRET.at[1]) < 0.3; }));
  };
  // the shooter at a wheel surface speed (m/s), the kicker at its ratio; the indexer as phys2056 runs it
  r.surf = 0;
  r.spin = (surf) => {
    r.surf = surf || 0;
    speedLoop(t.shooter, surf ? surf / SHOOTER.r : null, SHOOTER.kP);
    speedLoop(t.kicker, surf ? (KICK_RATIO * surf) / SHOOTER.r : null, SHOOTER.kP);
  };
  r.index = (on, pass = false) => {
    const k = pass ? 1 : 0;
    if (!on) { for (const a of [t.floor, ...t.belts, t.vkick, t.feed]) speedLoop(a, null); return; }
    speedLoop(t.floor, rps(RPS.floor[k], t.floor));
    for (const b of t.belts) speedLoop(b, rps(RPS.belts[k], b));
    speedLoop(t.vkick, rps(RPS.vkick, t.vkick));
    speedLoop(t.feed, Math.max(FEED_MIN, FEED_SHARE * r.surf) / FEED.r);
  };
  r.pose();
  return r;
}
// a FUEL leaving the shooter going up: shot (speed, elevation, heading against the turret's yaw, where
// it left from in the turret's frame)
function shotWatch(r) {
  const touched = new Set(), shots = [];
  return {
    shots,
    check(balls, t) {
      for (const b of balls) {
        if (r.shooter.contacts.some((q) => q.ball === b)) { touched.add(b); continue; }
        if (!touched.has(b)) continue;
        touched.delete(b);
        const v = b.linvel(), p = b.translation();
        if (v.y > 1 && Math.hypot(v.x, v.y, v.z) > 3) {
          const fx = Math.cos(r.yaw), fz = -Math.sin(r.yaw), [tx, tz] = TURRET.at;
          shots.push({ t, v: Math.hypot(v.x, v.y, v.z), el: (Math.atan2(v.y, Math.hypot(v.x, v.z)) * 180) / Math.PI, side: (Math.atan2(v.x * -fz + v.z * fx, v.x * fx + v.z * fz) * 180) / Math.PI, x: (p.x - tx) * fx + (p.z - tz) * fz, y: p.y, w: b.angvel() });
          b.setTranslation({ x: 50 + shots.length, y: 5, z: 0 }, true); b.setGravityScale(0, true); b.setLinvel({ x: 0, y: 0, z: 0 }, true);
        }
      }
    },
  };
}
function landing(o) { const hit = sim2D(o.y, o.v, (o.el * Math.PI) / 180, HUB.rimFront); return hit ? hit.d + o.x : NaN; }
function hubTable() {
  const sh = ROBOTS['2056'].shooter;
  return new ShotTable({ h0: sh.exitY, Ht: HUB.targetHeight, hoodMin: sh.hoodMin, hoodMax: sh.hoodMax, speedMax: sh.speedMax, mode: 'hub', clearDist: HUB.size / 2 + FUEL.radius + 0.02, clearHeight: HUB.rimFront + FUEL.radius + 0.04 });
}
const live = (balls) => balls.filter((b) => b.translation().y < 3 && b.translation().y > -1);
const inRobot = (b) => { const [x, y, z] = P(b); return x > -HALF_L - 0.02 && x < INTAKE.frame.front + 0.01 && Math.abs(z) < HALF_W && y > 0.09 && y < 0.8; };
// as js/mech/phys2056.js where() has it (the intake out)
const held2056 = ([x, y, z]) => (x > -HALF_L - 0.02 && x < HALF_L && Math.abs(z) < HALF_W && y > 0.09 && y < 0.8) || (x >= HALF_L && x < INTAKE.frame.front - 0.04 && Math.abs(z) < INTAKE.frame.sides && y > 0.20 && y < 0.62);
const own2056 = (p) => held2056(p) || (p[0] >= HALF_L && p[0] < INTAKE.frame.front + 0.01 && Math.abs(p[2]) < INTAKE.frame.sides && p[1] < 0.62);
const TOP2056 = { topX: 0.1, topN: 10 };

// fire a few FUEL up the bore at hood h (deg) and wheel surface surf (m/s): their median exit, how far
// the wheel dipped and how long it took to come back
function fireSome(h, surf) {
  const r = rig({ hood: h });
  const w = world({ carpet: false });
  for (let i = 0; i < 2 / DT; i++) { r.spin(surf); r.shooter.step(DT, []); }
  const balls = [], watch = shotWatch(r), [tx, tz] = TURRET.at;
  let dip = Infinity, low = null, rec = 0;
  for (let i = 0; i < 3 / DT && watch.shots.length < 4; i++) {
    if (i % Math.round(0.4 / DT) === 0 && balls.length < 4) balls.push(ball(w, tx + 0.005, 0.36, tz, [0, env('UP', 3), 0]));
    r.spin(surf);
    const l = balls.filter((b) => b.translation().y < 3);
    r.step(l); w.step(); watch.check(l, i * DT);
    if (process.env.TRACE && i % env("TRACEK", 3) === 0) for (const b of l) { const [x, y, z] = P(b), v = b.linvel(), [tx0, tz0] = TURRET.at, ss = x - tx0, dd = Math.hypot(ss - SHOOTER.axle[0], y - SHOOTER.axle[1]), aa = Math.atan2(y - SHOOTER.axle[1], ss - SHOOTER.axle[0]) / DEG; console.log(`   ${(i * DT).toFixed(3)} ${balls.indexOf(b)} d ${dd.toFixed(3)} a ${aa.toFixed(0)} ${x.toFixed(3)},${y.toFixed(3)},${z.toFixed(3)} v ${v.x.toFixed(2)},${v.y.toFixed(2)},${v.z.toFixed(2)}: ${[...r.path.contacts, ...r.shooter.contacts].filter((c) => c.ball === b).map((c) => c.wheel.name + ' ' + c.force.toFixed(0)).join(', ')}`); }
    const now = Math.abs(r.t.wheel.omega) * SHOOTER.r;
    if (now < dip) { dip = now; low = i; }
    if (low !== null && rec === 0 && now > 0.97 * surf && i > low) rec = (i - low) * DT;
  }
  w.free();
  const o = watch.shots;
  if (!o.length) return null;
  return { n: o.length, v: med(o.map((q) => q.v)), el: med(o.map((q) => q.el)), side: med(o.map((q) => q.side)), x: med(o.map((q) => q.x)), y: med(o.map((q) => q.y)), surf, dip, rec };
}
// FUEL fed in a stream: n laid on the floor, the indexer on for sec s (seeds pooled)
function fedSome(h, surf, { n = 9, sec = 3, yaw = env('YAW', 0) * DEG, seeds = SEEDS } = {}) {
  const all = [];
  for (let seed = 1; seed <= seeds; seed++) {
    const r = rig({ hood: h, yaw }), w = world({ carpet: false }), balls = [];
    let sd = seed; const rnd = () => ((sd = (sd * 16807) % 2147483647) / 2147483647 - 0.5);
    for (let i = 0; i < n; i++) balls.push(ball(w, 0.2 - 0.155 * (Math.floor(i / 3) % 2) + 0.01 * rnd(), 0.26 + 0.16 * Math.floor(i / 6), -0.2 + 0.155 * (i % 3) + 0.01 * rnd()));
    for (let i = 0; i < 1 / DT; i++) { r.spin(surf); r.step(live(balls)); w.step(); }
    const watch = shotWatch(r);
    for (let k = 0; k < sec / DT; k++) { r.spin(surf); r.index(true); r.t.intake.cmd = 1; const l = live(balls); r.step(l); w.step(); watch.check(l, k * DT); }
    w.free();
    all.push(...watch.shots);
  }
  if (!all.length) return null;
  return { n: all.length, v: med(all.map((q) => q.v)), el: med(all.map((q) => q.el)), side: med(all.map((q) => q.side)), x: med(all.map((q) => q.x)), y: med(all.map((q) => q.y)), surf, shots: all };
}

const tests = {
  rollers() {
    const r = rig(), t = r.t;
    t.intake.cmd = 1; r.spin(16); r.index(true);
    for (let i = 0; i < 2 / DT; i++) { r.index(true); r.step([]); }
    const surf = (sh, rr) => Math.abs(sh.omega) * rr, mrps = (sh) => (Math.abs(sh.omega) * sh.motor.ratio) / TAU;
    console.log(`2056's drives, free (intake 100%, the indexer at its speeds, the shooter wheel at 16 m/s):`);
    console.log(`  intake X60 ${mrps(t.intake).toFixed(1)} rps: rollers ${surf(t.rollers[0], INTAKE.rollers[0][2]).toFixed(2)} m/s at their surface`);
    console.log(`  floor X60 ${mrps(t.floor).toFixed(1)} rps (${RPS.floor[0]}): rollers ${surf(t.floor.shafts[0], FLOOR.r).toFixed(2)} m/s; the two over its back ${surf(t.top[0], TOP.r).toFixed(2)} m/s`);
    console.log(`  belts X44s ${t.belts.map((b) => mrps(b).toFixed(1)).join('/')} rps (${RPS.belts[0]}): ${t.belts.map((b) => surf(b, BELT.pitch).toFixed(2)).join('/')} m/s`);
    console.log(`  side kicker X44 ${mrps(t.vkick).toFixed(1)} rps (${RPS.vkick}): ${surf(t.vkick, VKICK.r).toFixed(2)} m/s`);
    console.log(`  Stealth feed wheels (X60 ${mrps(t.feed).toFixed(1)} rps): ${surf(t.feedShafts[0], FEED.r).toFixed(2)} m/s (60% of the shooter's ${surf(t.wheel, SHOOTER.r).toFixed(2)}, at least ${FEED_MIN})`);
    console.log(`  backspin kicker (X44 ${mrps(t.kicker).toFixed(1)} rps): ${surf(t.kicker, SHOOTER.kicker.r).toFixed(2)} m/s`);
    for (const v of [12, 16, 20, 24]) {
      const r2 = rig();
      let T = 0;
      for (; T < 4; T += DT) { r2.spin(v); r2.shooter.step(DT, []); if (Math.abs(Math.abs(r2.t.wheel.omega) * SHOOTER.r - v) < SHOOTER.tolV * v) break; }
      console.log(`  the shooter wheel from rest to within 3% of ${v} m/s: ${T < 4 ? T.toFixed(2) + ' s' : 'never'}`);
    }
  },
  shooter() {
    const list = (k) => process.env[k]?.split(',').map(Number);
    const hs = list('HS') ?? [129.5, 146.5, 163.5];
    const vs = list('VS') ?? [10, 15, 20];
    console.log(`2056's shooter: FUEL thrown up the bore into the wheel (the turret facing forward):`);
    const ratios = [];
    for (const h of hs) for (const v of vs) {
      const o = fireSome(h, v);
      if (!o) { console.log(`  arm ${h} (${hoodElevation(h).toFixed(1)} deg by the geometry), ${v} m/s: nothing came out`); continue; }
      ratios.push(o.v / o.surf);
      console.log(`  arm ${h} (${hoodElevation(h).toFixed(1)} by the geometry), wheel ${v} m/s: ${o.n}/4 out at ${o.v.toFixed(2)} m/s (${(o.v / o.surf).toFixed(2)} of the wheel's), ${o.el.toFixed(1)} deg, ${o.side.toFixed(1)} deg aside, from ${o.x.toFixed(3)},${o.y.toFixed(3)}; the wheel dipped to ${o.dip.toFixed(1)} m/s, back in ${o.rec.toFixed(3)} s`);
    }
    if (ratios.length) console.log(`  exit over the wheel's surface: median ${med(ratios).toFixed(3)} (${Math.min(...ratios).toFixed(2)}..${Math.max(...ratios).toFixed(2)})`);
    if (process.env.NOTABLE) return;
    const hub = hubTable();
    console.log(`  the sim's HUB table (exit at ${ROBOTS['2056'].shooter.exitY} m), fired as phys2056 fires it${process.env.FED ? ' (stream-fed)' : ''}:`);
    for (const d of list('DS') ?? [1.5, 2, 2.5, 3, 3.5, 4, 4.5, 5, 5.5, 6]) {
      const row = hub.lookup(d - EXIT.s);
      if (!row) { console.log(`   ${d} m: out of the table`); continue; }
      const o = (process.env.FED ? fedSome : fireSome)(hoodFor(row.theta / DEG), surfFor(row.v));
      if (!o) { console.log(`   ${d} m: nothing came out`); continue; }
      const land = landing(o);
      console.log(`   ${d} m: the table's ${row.v.toFixed(2)} m/s at ${(row.theta / DEG).toFixed(1)} deg (arm ${hoodFor(row.theta / DEG).toFixed(1)}, wheel ${surfFor(row.v).toFixed(1)}): out at ${o.v.toFixed(2)} m/s, ${o.el.toFixed(1)} deg, from ${o.x.toFixed(2)},${o.y.toFixed(2)}; down at the rim ${land.toFixed(2)} m (${land - d >= 0 ? '+' : ''}${(land - d).toFixed(2)})`);
    }
  },
  // their method: one coefficient from the exit speed asked to the wheel's surface speed, and the
  // elevation each arm angle gives (stream-fed: the way it fires)
  calibrate() {
    const hs = (process.env.HS?.split(',').map(Number)) ?? [129.5, 135, 141, 147, 153, 158, 163.5];
    const vs = (process.env.VS?.split(',').map(Number)) ?? [9, 13, 17];
    const el = [], k = [], xs = [], ys = [];
    for (const h of hs) {
      const row = [];
      for (const v of vs) {
        const o = fedSome(h, v, { seeds: env('SEEDS', 2) });
        if (!o) continue;
        row.push(o.el); k.push(o.v / v); xs.push(o.x); ys.push(o.y);
      }
      if (row.length) el.push([h, med(row)]);
      console.log(`  arm ${h}: elevation ${row.map((e) => e.toFixed(1)).join('/')} at ${vs.join('/')} m/s`);
    }
    console.log(`EL_MAP = ${JSON.stringify(el.map(([h, e]) => [h, Number(e.toFixed(1))]))}`);
    console.log(`EXIT_K = ${med(k).toFixed(3)} (${Math.min(...k).toFixed(3)}..${Math.max(...k).toFixed(3)}); EXIT s ${med(xs).toFixed(3)}, y ${med(ys).toFixed(3)}`);
  },
  // FUEL laid on the floor, the indexer running and the shooter firing at 3 m
  feed() {
    const n = env('N', 12), row = hubTable().lookup(3 - EXIT.s), h = hoodFor(row.theta / DEG), surf = surfFor(row.v), res = [];
    for (let seed = 1; seed <= SEEDS; seed++) {
      const r = rig({ hood: h }), w = world(), balls = [];
      let sd = seed; const rnd = () => ((sd = (sd * 16807) % 2147483647) / 2147483647 - 0.5);
      for (let i = 0; i < n; i++) balls.push(ball(w, 0.24 - 0.155 * (Math.floor(i / 4) % 3) + 0.01 * rnd(), 0.26 + 0.16 * Math.floor(i / 12), -0.25 + 0.155 * (i % 4) + 0.01 * rnd()));
      for (let i = 0; i < 1 / DT; i++) { r.spin(surf); r.step(live(balls)); w.step(); }
      const watch = shotWatch(r);
      for (let k = 0; k < env('SEC', 5) / DT; k++) { r.spin(surf); r.index(true); r.t.intake.cmd = 1; const l = live(balls); r.step(l); w.step(); watch.check(l, k * DT); }
      const s = watch.shots, rate = s.length > 1 ? (s.length - 1) / (s[s.length - 1].t - s[0].t) : 0;
      const land = s.map((o) => landing(o) - 3);
      res.push(s.length);
      console.log(`  seed ${seed}: ${s.length} of ${n} shot in ${env('SEC', 5)} s at ${rate.toFixed(1)} FUEL/s, the first at ${s[0]?.t.toFixed(2) ?? '-'} s; down at the rim ${med(land)?.toFixed(2)} m from the HUB's center (median); ${balls.filter(inRobot).length} left${process.env.STUCK ? ': ' + balls.filter(inRobot).map((b) => P(b).map((v) => v.toFixed(2)).join(',')).join(' | ') : ''}`);
      w.free();
    }
  },
  // intaking alone: 24 FUEL four abreast on a treadmill at SPEED
  intake() {
    for (let seed = 1; seed <= SEEDS; seed++) {
      const o = pathRun(seed, { n: 24, sec: 6 });
      console.log(`  seed ${seed} at ${SPEED} m/s: ${o.inside} of 24 inside the frame after 6 s (each second: ${o.timeline.join(' ')}); the intake's X60 down to ${(o.minIntake * 100).toFixed(0)}% of its free speed; the kicker bar lifted ${o.kickMax.toFixed(1)} deg at most${o.left.length ? `; ${o.left.length} left out front` : ''}`);
    }
  },
  intakefull() {
    const res = [];
    for (let seed = 1; seed <= SEEDS; seed++) {
      const r = rig();
      const out = fillRun(RAPIER, {
        world: () => world({ carpet: false }), ball, path: { step: (dt, l, tg) => r.step(l, tg), contacts: [] }, DT, held: held2056, own: own2056, top: TOP2056, speed: env('SPEED', 1), sec: env('SEC', 12), front: HALF_L + 3.25 * 0.0254 + 0.18, seed,
        control: () => { r.t.intake.cmd = 1; speedLoop(r.t.floor, rps(FLOOR_INTAKE, r.t.floor)); },
      });
      res.push(out);
      console.log(fillReport(`  fill ${seed}`, out) + `; the net lifted ${(r.bowMax * 100).toFixed(1)} cm at most`);
    }
    console.log(`2056 filled through its intake, ${res.length} fills: the most it held ${res.map((x) => x.most).join(', ')} (median ${med(res.map((x) => x.most))})`);
  },
  // a full one (filled through the intake) fired at 3 m, the intake coming in at RETRACT_AT
  dump() {
    const pass = !!process.env.PASS, row = hubTable().lookup(3 - EXIT.s), h = hoodFor(row.theta / DEG), surf = surfFor(row.v), res = [];
    for (let seed = 1; seed <= SEEDS; seed++) {
      const r = rig({ hood: h });
      const f = fillRun(RAPIER, {
        world: () => world({ carpet: false }), ball, path: { step: (dt, l, tg) => r.step(l, tg), contacts: [] }, DT, held: held2056, own: own2056, top: TOP2056, speed: 1, sec: env('SEC', 10), front: HALF_L + 3.25 * 0.0254 + 0.18, seed, keep: true,
        control: () => { r.t.intake.cmd = 1; speedLoop(r.t.floor, rps(FLOOR_INTAKE, r.t.floor)); },
      });
      const w = f.world, balls = f.balls;
      f.carpet.setLinvel({ x: 0, y: 0, z: 0 }, true);
      r.t.intake.cmd = null; speedLoop(r.t.floor, null);
      for (const b of balls) { const [x, y] = P(b); if (x > INTAKE.frame.front + 0.02 || y < 0.03) b.setTranslation({ x: 50, y: -5, z: 0 }, true); }
      for (let i = 0; i < 1 / DT; i++) { r.spin(surf); const l = live(balls); r.step(l); w.step(); }
      const n0 = balls.filter((b) => held2056(P(b))).length;
      const watch = shotWatch(r);
      let minSurf = Infinity, retractAt = null;
      for (let k = 0; k < env('SEC2', 10) / DT; k++) {
        const l = live(balls), n = l.filter((b) => held2056(P(b))).length;
        r.spin(surf); r.index(true, pass); r.t.intake.cmd = 1;
        r.retract = process.env.RETRACT !== '0' && n <= RETRACT_AT;
        if (r.retract && retractAt === null) retractAt = k * DT;
        r.step(l); w.step(); watch.check(l, k * DT);
        if (k > 60) minSurf = Math.min(minSurf, Math.abs(r.t.wheel.omega) * SHOOTER.r);
      }
      const s = watch.shots;
      const rate = (q) => (q.length > 1 ? (q.length - 1) / (q[q.length - 1].t - q[0].t) : 0);
      const mid = (q) => { const a = Math.floor(q.length * 0.1), b = Math.ceil(q.length * 0.9) - 1; return b > a ? (b - a) / (q[b].t - q[a].t) : 0; };
      const best = s.reduce((m, x, i) => Math.max(m, s.slice(i).filter((y) => y.t < x.t + 1).length), 0);
      const land = s.map((o) => landing(o) - 3), hits = land.filter((d) => Math.abs(d) < 0.45).length;
      const left = balls.filter(inRobot);
      res.push({ n0, n: s.length, rate: rate(s), mid: mid(s), best, left: left.length });
      console.log(`  fill ${seed}: ${s.length} of ${n0} out in ${env('SEC2', 10)} s, ${rate(s).toFixed(1)} FUEL/s over the dump, ${mid(s).toFixed(1)} through the middle, ${best} in its best second; down at the rim ${med(land) >= 0 ? '+' : ''}${med(land)?.toFixed(2)} m (median), ${hits} of ${s.length} within 0.45 m; the wheel ${surf.toFixed(1)} m/s dipped to ${minSurf.toFixed(1)}; the intake in from ${retractAt?.toFixed(1) ?? '-'} s; ${left.length} left`);
      if (process.env.STUCK) for (const b of left) { const [x, y, z] = P(b); console.log(`   left at ${x.toFixed(2)},${y.toFixed(2)},${z.toFixed(2)}: ${r.path.contacts.filter((c) => c.ball === b).map((c) => c.wheel.name + ' ' + c.force.toFixed(0)).join(', ')}`); }
      w.free();
    }
    const m = (fn) => med(res.map(fn));
    console.log(`2056 dumps a full hopper at 3 m (${pass ? 'passing' : 'scoring'} speeds), ${res.length} fills: ${m((x) => x.n)} of ${m((x) => x.n0)} out (median), ${m((x) => x.rate).toFixed(1)} FUEL/s over the dump (${Math.min(...res.map((x) => x.rate)).toFixed(1)}..${Math.max(...res.map((x) => x.rate)).toFixed(1)}), ${m((x) => x.mid).toFixed(1)} through the middle, ${m((x) => x.best)} in its best second, ${m((x) => x.left)} left`);
  },
};

// intaking alone: FUEL four abreast on a treadmill carpet at speed
function pathRun(seed, { n = 20, sec = 7, speed = SPEED } = {}) {
  let sd = seed;
  const rnd = () => ((sd = (sd * 16807) % 2147483647) / 2147483647 - 0.5);
  const r = rig();
  const w = world({ carpet: false });
  const carpet = w.createRigidBody(RAPIER.RigidBodyDesc.kinematicVelocityBased().setLinvel(-speed, 0, 0));
  w.createCollider(RAPIER.ColliderDesc.cuboid(15, 0.01, 1).setTranslation(0.4 + 15, -0.01, 0).setFriction(0.3).setRestitution(0.2), carpet);
  w.createCollider(RAPIER.ColliderDesc.cuboid(0.4, 0.01, 1).setTranslation(0, -0.01, 0).setFriction(0.3), w.createRigidBody(RAPIER.RigidBodyDesc.fixed()));
  r.t.intake.cmd = 1;
  speedLoop(r.t.floor, rps(FLOOR_INTAKE, r.t.floor));
  for (let i = 0; i < 1 / DT; i++) r.step([]);
  const free = Math.abs(r.t.intake.omega) || 1;
  const balls = [];
  for (let i = 0; i < n; i++) balls.push(ball(w, 0.95 + 0.17 * Math.floor(i / 4) + 0.02 * rnd(), R + 0.003, ((i % 4) - 1.5) * 0.16 + 0.02 * rnd(), [-speed, 0, 0]));
  let minIntake = 1;
  const timeline = [];
  for (let k = 0; k < sec / DT; k++) {
    const l = balls.filter((b) => b.translation().y < 3 && b.translation().y > -1);
    r.step(l); w.step();
    minIntake = Math.min(minIntake, Math.abs(r.t.intake.omega) / free);
    if (k % Math.round(1 / DT) === Math.round(1 / DT) - 1) timeline.push(balls.filter((b) => { const [x, y] = P(b); return y < 3 && x < HALF_L && y > 0.12; }).length);
    for (const b of l) { const p = b.translation(); if (p.x < -0.8) b.setTranslation({ x: 50, y: -5, z: 0 }, true); }
    if (process.env.TRACE && k % 6 === 0) {
      const b = balls[Number(process.env.TRACE)], [x, y, z] = P(b), v = b.linvel();
      console.log(`   ${(k * DT).toFixed(2)} ${x.toFixed(3)},${y.toFixed(3)},${z.toFixed(3)} v ${v.x.toFixed(2)},${v.y.toFixed(2)}: ${r.path.contacts.filter((c) => c.ball === b).map((c) => c.wheel.name + ' ' + c.force.toFixed(0)).join(', ')}`);
    }
  }
  const inside = balls.filter((b) => { const [x, y] = P(b); return y < 3 && x < HALF_L && y > 0.12; }).length;
  const left = balls.filter((b) => { const [x, y] = P(b); return y < 3 && x >= HALF_L && x < 1; });
  w.free();
  return { inside, timeline, minIntake, left, kickMax: r.ks.kickMax ?? 0 };
}

const which = process.argv.slice(2);
for (const name of which.length ? which : ['rollers', 'shooter', 'feed', 'intake']) {
  if (!tests[name]) { console.log(`no test ${name}`); continue; }
  await tests[name]();
}
