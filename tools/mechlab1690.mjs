// 1690's mechanisms (js/mech/r1690.js) on their own, no field, no robot: the ball path built from its
// CAD's renders, with its rollers, belts and wheels, measured against what the team says (their reveal
// thread: 13 FUEL/s counted over a cycle, the shooter's recovery "enough for 20 BPS", 50-55 open-
// topped) and the estimates where nothing's published. Run:
//   node --import ./tools/node-env.mjs tools/mechlab1690.mjs [test ...]
// Tests: rollers (each roller's and belt's speed under its loop, the shooter's spin-up and recovery),
// shooter (FUEL pushed up the turret's bore into the wheel at hood angles and wheel speeds: exit speed
// and elevation; and the sim's shot table's rows: where they come down), calibrate (the shot maps
// behind js/mech/phys1690.js's EXIT_MAP and EL_MAP), feed (FUEL laid on the floor, the indexer
// running and the shooter firing: how fast they come out), intake (24 FUEL at SPEED m/s, intaking
// alone), intakefull (filled through the intake from the NEUTRAL ZONE's FUEL until it can't put more
// in: tools/fillbench.mjs), hold (filled a layer at a time, everything off), dump (a full one fired
// at 3 m: FUEL/s).
// Env: SEEDS, SPEED (1 m/s), BULGE (the net's, m), GRIP_<material>, TREAD_<material>, STUCK (where the
// ones left over are), MAP=1 (shooter: the full sweep), LAYERS, SEC, SEC2, PASS=1 (dump: the passing
// speeds), FILL=layers (dump: filled a layer at a time)
import { RAPIER } from '../js/physics.js';
import { FUEL, GROUP, groups, HUB, TRENCH } from '../js/constants.js';
import { Mechanism } from '../js/mech/mechanism.js';
import { sim2D, ShotTable } from '../js/ballistics.js';
import {
  build1690, wheels1690, shooterWheel, pathSurfaces1690, placeIntake, placeTurret, turretPoint, netY, hoodElevation,
  HALF_L, HALF_W, INTAKE, FLOOR, BELT, BACK, VKICK, FEED, SHOOTER, TURRET, NET, WALL, MU as GRIP, TREAD,
} from '../js/mech/r1690.js';
import { speedLoop } from '../js/mech/phys1706.js';
import { RPS, FEED_SHARE, FEED_MIN, EXIT, hoodFor, elevationAt, surfFor, exitAt, HOOD_STOW } from '../js/mech/phys1690.js';
import { ROBOTS } from '../js/robotConfigs.js';
import { fillRun, fillReport } from './fillbench.mjs';

await RAPIER.init();
for (const k of Object.keys(GRIP)) if (process.env['GRIP_' + k]) GRIP[k] = Number(process.env['GRIP_' + k]);
for (const k of Object.keys(TREAD)) if (process.env['TREAD_' + k]) TREAD[k] = Number(process.env['TREAD_' + k]);
if (process.env.BULGE) NET.bulge = Number(process.env.BULGE);
// (TOPY, BOTY: the belts' runs, m; BACKX: the backing plate: other estimates)
if (process.env.TOPY) BELT.top.y = Number(process.env.TOPY);
if (process.env.BOTY) BELT.bottom.y = Number(process.env.BOTY);
if (process.env.BACKX) BACK.x = Number(process.env.BACKX);
if (process.env.FLOORR) FLOOR.ratio = Number(process.env.FLOORR);
const R = FUEL.radius, DT = 1 / 120, SEEDS = Number(process.env.SEEDS ?? 4), SPEED = Number(process.env.SPEED ?? 1);
const HELD = groups(GROUP.HELD, 0xffff & ~GROUP.HELD);
const DEG = Math.PI / 180, TAU = 2 * Math.PI;
const env = (k, d) => (process.env[k] !== undefined ? Number(process.env[k]) : d);

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
const RPM = (w) => (w * 60) / TAU;
const P = (b) => { const p = b.translation(); return [p.x, p.y, p.z]; };
const rps = (n, sh) => (n * TAU) / sh.motor.ratio;

// the robot's mechanisms, fixed in the world: the intake out (deg up from deployed: 0), the turret at
// yaw (rad: 0 shoots forward), the hood at angle h (deg about the axle)
function rig({ deg = 0, yaw = 0, hood = HOOD_STOW } = {}) {
  const t = build1690(), wh = wheels1690(t), s = pathSurfaces1690(t), sw = shooterWheel(t);
  const pathW = [...wh.intake, ...wh.floor, wh.vkick, ...wh.feed];
  const path = new Mechanism({ substeps: 6, shafts: [...new Set(pathW.map((x) => x.shaft)), ...t.belts], wheels: pathW, surfaces: [s.body, s.ptfe, s.bottom, s.top, s.intake, s.bumper, s.net] });
  const shooter = new Mechanism({ substeps: 6, sub: Number(process.env.SUB ?? 8), shafts: [t.wheel], wheels: [sw], surfaces: [s.turret, s.hood] });
  for (const a of [t.intake, t.floor, t.feed, t.shooter, t.vkick, ...t.belts]) a.cmd = null;
  const r = { t, wh, s, sw, path, shooter, deg, yaw, hood };
  r.pose = () => { placeIntake(s.intake, wh.intake, r.deg); placeTurret(s, sw, r.yaw, r.hood); };
  r.step = (balls, together = balls) => {
    r.pose();
    path.step(DT, balls, together);
    shooter.step(DT, balls.filter((b) => { const [x, y, z] = P(b); return y > TURRET.bore.y[0] && Math.hypot(x - TURRET.at[0], z - TURRET.at[1]) < 0.25; }));
  };
  // the shooter at a wheel surface speed (m/s; null: coast); the indexer feeding (as phys1690 runs it)
  r.spin = (surf) => speedLoop(t.shooter, surf ? surf / SHOOTER.r : null, SHOOTER.kP);
  r.index = (on, pass = false) => {
    const k = pass ? 1 : 0, ws = Math.abs(t.wheel.omega) * SHOOTER.r;
    if (!on) { for (const a of [t.floor, ...t.belts, t.vkick, t.feed]) speedLoop(a, null); return; }
    speedLoop(t.floor, rps(RPS.floor[k], t.floor));
    for (const b of t.belts) speedLoop(b, rps(RPS.belts[k], b));
    speedLoop(t.vkick, rps(RPS.vkick, t.vkick));
    speedLoop(t.feed, Math.max(FEED_MIN, FEED_SHARE * ws) / FEED.r);
  };
  r.pose();
  return r;
}
// a FUEL leaving the shooter going up: shot (its speed, elevation, heading against the turret's yaw,
// where it left from, turret frame)
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
          shots.push({ t, v: Math.hypot(v.x, v.y, v.z), el: (Math.atan2(v.y, Math.hypot(v.x, v.z)) * 180) / Math.PI, side: (Math.atan2(v.x * -fz + v.z * fx, v.x * fx + v.z * fz) * 180) / Math.PI, x: (p.x - tx) * fx + (p.z - tz) * fz, y: p.y, b });
          b.setTranslation({ x: 50 + shots.length, y: 5, z: 0 }, true); b.setGravityScale(0, true); b.setLinvel({ x: 0, y: 0, z: 0 }, true);
        }
      }
    },
  };
}
function landing(o) { const hit = sim2D(o.y, o.v, (o.el * Math.PI) / 180, HUB.rimFront); return hit ? hit.d + o.x : NaN; }
function tables() {
  const sh = ROBOTS['1690'].shooter;
  const hub = new ShotTable({ h0: sh.exitY, Ht: HUB.targetHeight, hoodMin: sh.hoodMin, hoodMax: sh.hoodMax, speedMax: sh.speedMax, mode: 'hub', clearDist: HUB.size / 2 + FUEL.radius + 0.02, clearHeight: HUB.rimFront + FUEL.radius + 0.04 });
  return { hub };
}
const live = (balls) => balls.filter((b) => b.translation().y < 3 && b.translation().y > -1);
// in it: the frame, the U-frame out front
const inRobot = (b) => { const [x, y, z] = P(b); return x > -HALF_L - 0.02 && x < INTAKE.frame.front + 0.01 && Math.abs(z) < HALF_W && y > 0.09 && y < 0.8; };
// as js/mech/phys1690.js where() has it (the intake out)
const held1690 = ([x, y, z]) => (x > -HALF_L - 0.02 && x < HALF_L && Math.abs(z) < HALF_W && y > 0.09 && y < 0.8) || (x >= HALF_L && x < INTAKE.frame.front - 0.04 && Math.abs(z) < INTAKE.frame.sides && y > 0.20 && y < 0.62);
const own1690 = (p) => held1690(p) || (p[0] >= HALF_L && p[0] < INTAKE.frame.front + 0.01 && Math.abs(p[2]) < INTAKE.frame.sides && p[1] < 0.62);
const TOP1690 = { topX: 0.1, topN: 10 };
// FUEL that got out other than as shots (each once it's been in it): over a wall or the U-frame's
// front (its center over the walls' tops outside the net's ties), up through the net (3 cm over it),
// out the back over the guard; how high the load pushes the net; how many it held when the load first
// stood taller than a TRENCH arm's underside for a quarter second
function escapeWatch(r) {
  const seen = new Set(), out = new Map();
  let top = 0, tallT = 0, tallAt = null;
  return {
    out,
    get top() { return top; },
    get tallAt() { return tallAt; },
    check(balls) {
      const n = r.s.net;
      let now = 0, held = 0;
      for (const b of balls) {
        const p = P(b), [x, y, z] = p, under = x > NET.x0 && x < n.x1 && Math.abs(z) < NET.hw;
        if (held1690(p)) { held++; if (under) now = Math.max(now, y + R); }
        if (out.has(b)) continue;
        if (!seen.has(b)) { if (held1690(p)) seen.add(b); else continue; }
        if (under && y + R > top) top = y + R;
        if (Math.hypot(x - TURRET.at[0], z - TURRET.at[1]) < 0.2 || x > 40 || b.linvel().y > 2) continue;
        const why = y > NET.y && !under ? 'over' : under && y > n.at(x, z)[0] + 0.03 ? 'through' : x < NET.x0 - 0.05 && y > TURRET.guardY[0] ? 'back' : null;
        if (why) out.set(b, why);
        if (why && process.env.ESC) { const v = b.linvel(); console.log(`   escaped (${why}) at ${x.toFixed(3)},${y.toFixed(3)},${z.toFixed(3)} v ${v.x.toFixed(1)},${v.y.toFixed(1)},${v.z.toFixed(1)}; the net there ${under ? n.at(x, z)[0].toFixed(3) : "-"}, its front ${n.x1.toFixed(3)}`); }
      }
      tallT = now > TRENCH.clearHeight ? tallT + DT : 0;
      if (tallAt === null && tallT >= 0.25) tallAt = held;
    },
    report() {
      const c = { over: 0, through: 0, back: 0 };
      for (const w of out.values()) c[w]++;
      return `${out.size} escaped (over the walls or the front: ${c.over}, through the net: ${c.through}, out over the back: ${c.back}); the load's top under the net ${top.toFixed(3)} m${tallAt !== null ? `, taller than the TRENCH from ${tallAt} in it` : ''}`;
    },
  };
}
// fill it a layer at a time (FUEL set into the space over the ones already in, under the net, let
// settle), the net held up out of the way (bulge 0.2 m), then let down onto the load over a second
function fill(w, balls, step, layers = 6, y = 0.62) {
  const B = NET.bulge;
  NET.bulge = 0.2;
  for (let layer = 0; layer < layers; layer++) {
    for (let i = 0; i < 6; i++) for (let k = 0; k < 5; k++) {
      const x = -0.02 + 0.125 * i, z = -0.29 + 0.145 * k;
      if (x > INTAKE.frame.front - 0.08) continue;
      const yy = Math.min(y, netY(x, z) - R - 0.005);
      if (balls.some((b) => { const p = b.translation(); return Math.hypot(p.x - x, p.y - yy, p.z - z) < 2 * R; })) continue;
      balls.push(ball(w, x, yy, z));
    }
    step(1.0 / DT);
  }
  for (let k = 1; k <= 1 / DT; k++) { NET.bulge = 0.2 + (B - 0.2) * k * DT; step(1); }
  NET.bulge = B;
}

const tests = {
  rollers() {
    const r = rig(), t = r.t;
    t.intake.cmd = 1; r.spin(16); r.index(true);
    for (let i = 0; i < 2 / DT; i++) { r.index(true); r.step([]); }
    const surf = (sh, rr) => Math.abs(sh.omega) * rr;
    console.log(`1690's rollers, free (intake 100%, the indexer at the team's speeds, the shooter wheel at 16 m/s):`);
    console.log(`  intake X60 ${RPM(t.intake.omega * t.intake.motor.ratio).toFixed(0)} rpm: rollers ${surf(t.rollers[0], INTAKE.rollers[0][2]).toFixed(2)} m/s at their surface`);
    console.log(`  floor X60 ${(t.floor.omega * t.floor.motor.ratio / TAU).toFixed(1)} rps (theirs ${RPS.floor[0]}): ${surf(t.floor.shafts[0], FLOOR.r).toFixed(2)} m/s at the rollers' surface`);
    console.log(`  belts X44s ${t.belts.map((b) => (b.omega * b.motor.ratio / TAU).toFixed(1)).join('/')} rps (theirs ${RPS.belts[0]}): ${t.belts.map((b) => surf(b, BELT.r).toFixed(2)).join('/')} m/s (the walkthrough: 8)`);
    console.log(`  vertical kicker X44 ${(t.vkick.omega * t.vkick.motor.ratio / TAU).toFixed(1)} rps (theirs ${RPS.vkick}): ${surf(t.vkick, VKICK.r).toFixed(2)} m/s`);
    console.log(`  feeder ${surf(t.feedShafts[0], FEED.r).toFixed(2)} m/s at its wheels (60% of the shooter's ${surf(t.wheel, SHOOTER.r).toFixed(2)}, at least ${FEED_MIN})`);
    for (const v of [12, 16, 20, 24]) {
      const r2 = rig();
      let T = 0;
      for (; T < 4; T += DT) { r2.spin(v); r2.shooter.step(DT, []); if (Math.abs(Math.abs(r2.t.wheel.omega) * SHOOTER.r - v) < SHOOTER.tolV * v) break; }
      console.log(`  the shooter wheel from rest to within ${(SHOOTER.tolV * 100).toFixed(0)}% of ${v} m/s: ${T.toFixed(2)} s`);
    }
  },
  // FUEL pushed up the turret's bore into the wheel (the feeder off): at hood angles and wheel speeds
  shooter() {
    const list = (k) => process.env[k]?.split(',').map(Number);
    const hs = list('HS') ?? (process.env.MAP ? [125, 130, 140, 150, 160, 172] : [125, 145, 172]);
    const vs = list('VS') ?? (process.env.MAP ? [8, 12, 16, 20, 24, 28] : [12, 18, 24]);
    console.log(`1690's shooter: FUEL thrown up the turret's bore into the wheel (the turret facing forward), the wheel at a surface speed, the hood at an angle about its axle:`);
    const ratios = [];
    for (const h of hs) for (const v of vs) {
      const o = fireSome(h, v);
      if (!o) { console.log(`  hood ${h} (${hoodElevation(h).toFixed(1)} deg by the geometry), ${v} m/s: nothing came out`); continue; }
      ratios.push(o.v / o.surf);
      console.log(`  hood ${h} (${hoodElevation(h).toFixed(1)} by the geometry), wheel ${v} m/s: ${o.n}/4 out at ${o.v.toFixed(2)} m/s (${(o.v / o.surf).toFixed(2)} of the wheel's ${o.surf.toFixed(1)}), ${o.el.toFixed(1)} deg, ${o.side.toFixed(1)} deg aside, from ${o.x.toFixed(3)},${o.y.toFixed(3)}; down at the rim ${landing(o).toFixed(2)} m out; the wheel dipped to ${o.dip.toFixed(1)} m/s, back in ${o.rec.toFixed(3)} s`);
    }
    if (ratios.length) console.log(`  exit over the wheel's surface: median ${med(ratios).toFixed(3)} (${Math.min(...ratios).toFixed(2)}..${Math.max(...ratios).toFixed(2)})`);
    if (process.env.NOTABLE) return;
    const { hub } = tables();
    console.log(`  the sim's HUB table (its config's exit at ${ROBOTS['1690'].shooter.exitY} m), fired as js/mech/phys1690.js fires it:`);
    for (const d of [1.5, 2, 2.5, 3, 3.5, 4, 4.5, 5, 5.5, 6]) {
      const row = hub.lookup(d - EXIT.s);
      if (!row) { console.log(`   ${d} m: out of the table`); continue; }
      const o = fireSome(hoodFor(row.theta / DEG), surfFor(row.v));
      if (!o) { console.log(`   ${d} m: nothing came out`); continue; }
      const land = landing(o);
      console.log(`   ${d} m: the table's ${row.v.toFixed(2)} m/s at ${(row.theta / DEG).toFixed(1)} deg (hood ${hoodFor(row.theta / DEG).toFixed(1)}, wheel ${surfFor(row.v).toFixed(1)}): out at ${o.v.toFixed(2)} m/s, ${o.el.toFixed(1)} deg, from ${o.x.toFixed(2)},${o.y.toFixed(2)}; down at the rim ${land.toFixed(2)} m (${land - d >= 0 ? '+' : ''}${(land - d).toFixed(2)})`);
    }
  },
  // the shot maps behind phys1690's EXIT_MAP and EL_MAP, tuned the way a team tunes its table: for
  // each of the sim's HUB table's rows, the wheel's speed and the hood moved until the FUEL leave at
  // the row's speed and elevation
  calibrate() {
    const { hub } = tables(), pairs = [], els = [];
    for (const d of [1.25, 1.5, 2, 2.5, 3, 3.5, 4, 4.5, 5, 5.5, 6]) {
      const row = hub.lookup(d - EXIT.s);
      if (!row) continue;
      const el = row.theta / DEG;
      let surf = surfFor(row.v), h = hoodFor(el), o = null;
      for (let k = 0; k < 5; k++) {
        o = fireSome(h, surf);
        if (!o) break;
        surf *= Math.pow(row.v / o.v, 1.2); h = Math.max(SHOOTER.elMin + 90, Math.min(SHOOTER.elMax + 90, h + (el - o.el)));
      }
      if (!o) continue;
      pairs.push([o.v, o.surf]); els.push([h, o.el]);
      console.log(`  ${d} m: the table's ${row.v.toFixed(2)} m/s at ${el.toFixed(1)} deg: the wheel at ${surf.toFixed(2)} m/s, the hood at ${h.toFixed(1)} (last: out at ${o.v.toFixed(2)} m/s, ${o.el.toFixed(1)} deg from ${o.surf.toFixed(2)} m/s, ${o.x.toFixed(3)},${o.y.toFixed(3)}; down at the rim ${landing(o).toFixed(2)} m)`);
    }
    console.log(`  EXIT_MAP pairs (exit, wheel's surface): ${JSON.stringify(pairs.map((p) => p.map((v) => +v.toFixed(2))).sort((a, b) => a[0] - b[0]))}`);
    console.log(`  EL_MAP pairs (hood, elevation): ${JSON.stringify(els.map((p) => p.map((v) => +v.toFixed(1))).sort((a, b) => a[0] - b[0]))}`);
  },
  // FUEL laid on the floor in front of the belts (LAYERS rows), the indexer at the team's speeds and
  // the shooter at a 3 m shot: how fast they come out
  feed() {
    const { hub } = tables(), row = hub.lookup(3 - EXIT.s), h = hoodFor(row.theta / DEG), surf = surfFor(row.v);
    const res = [];
    for (let seed = 1; seed <= SEEDS; seed++) {
      const r = rig({ hood: h }), w = world({ carpet: false }), balls = [];
      let sd = seed; const rnd = () => ((sd = (sd * 16807) % 2147483647) / 2147483647 - 0.5);
      for (let i = 0; i < env('N', 12); i++) balls.push(ball(w, 0.2 - 0.155 * (Math.floor(i / 3) % 2) + 0.01 * rnd(), 0.26 + 0.16 * Math.floor(i / 6), -0.09 + 0.155 * (i % 3) + 0.01 * rnd()));
      for (let i = 0; i < 1 / DT; i++) { r.spin(surf); r.step(live(balls)); w.step(); if (process.env.TRACE0 && i % 6 === 0) { const b = balls[Number(process.env.TRACE0)], [x, y, z] = P(b), v = b.linvel(); console.log(`   pre ${(i * DT).toFixed(2)} ${x.toFixed(3)},${y.toFixed(3)},${z.toFixed(3)} v ${v.x.toFixed(2)},${v.y.toFixed(2)},${v.z.toFixed(2)}: ${r.path.contacts.filter((c) => c.ball === b).map((c) => c.wheel.name + " " + c.force.toFixed(0)).join(", ")}`); } }
      const watch = shotWatch(r);
      for (let k = 0; k < env('SEC', 5) / DT; k++) {
        r.spin(surf); r.index(true); r.t.intake.cmd = process.env.NOINTAKE ? null : 1;
        const l = live(balls); r.step(l); w.step(); watch.check(l, k * DT);
        if (process.env.TRACE && k % 15 === 0) for (const b of l) { const [x, y, z] = P(b); if (inRobot(b)) console.log(`   ${(k * DT).toFixed(2)} ${balls.indexOf(b)} ${x.toFixed(3)},${y.toFixed(3)},${z.toFixed(3)}: ${[...r.path.contacts, ...r.shooter.contacts].filter((c) => c.ball === b).map((c) => c.wheel.name + ' ' + c.force.toFixed(0)).join(', ')}`); }
      }
      const s = watch.shots.map((x) => x.t);
      res.push({ n: s.length, rate: s.length > 1 ? (s.length - 1) / (s[s.length - 1] - s[0]) : 0, first: s[0] ?? NaN, land: med(watch.shots.map((o) => landing(o) - 3)) });
      if (process.env.STUCK) for (const b of live(balls).filter(inRobot)) { const [x, y, z] = P(b); console.log(`   left at ${x.toFixed(3)},${y.toFixed(3)},${z.toFixed(3)}: ${r.path.contacts.filter((c) => c.ball === b).map((c) => c.wheel.name + ' ' + c.force.toFixed(0)).join(', ')}`); }
      w.free();
    }
    console.log(`1690 feeding ${env('N', 12)} FUEL laid on its floor (the 3 m shot): ${res.map((x) => x.n).join(', ')} shot in ${env('SEC', 5)} s, ${res.map((x) => x.rate.toFixed(1)).join(', ')} FUEL/s, the first after ${res.map((x) => x.first.toFixed(2)).join(', ')} s; down at the rim ${res.map((x) => (x.land >= 0 ? '+' : '') + x.land.toFixed(2)).join(', ')} m from the HUB's center`);
  },
  intake() {
    for (const speed of process.env.SPEED ? [SPEED] : [1, 3]) {
      const res = [];
      for (let seed = 1; seed <= SEEDS; seed++) res.push(pathRun(seed, { n: 24, sec: 6, speed }));
      console.log(`1690 intaking only (rollers 100%, the intake out), 24 FUEL at ${speed} m/s: ${res.map((x) => x.inside).join(', ')} in it after 6 s; past the bumper each second ${res.map((x) => x.timeline.join('/')).join(', ')}; intake X60 down to ${med(res.map((x) => x.minIntake * 100)).toFixed(0)}% (median)`);
      if (process.env.STUCK) for (const x of res) console.log('   left', x.left.map((p) => p.map((v) => v.toFixed(2)).join(',')).join(' | '));
    }
  },
  async intakefull() {
    const runs = [];
    for (let seed = 1; seed <= SEEDS; seed++) {
      const r = rig();
      r.t.intake.cmd = 1;
      const esc = escapeWatch(r);
      const res = fillRun(RAPIER, { world: () => world({ carpet: false }), ball, path: { step: (dt, l) => { r.step(l); esc.check(l); }, get contacts() { return r.path.contacts; } }, DT, held: held1690, own: own1690, top: TOP1690, speed: SPEED, sec: env('SEC', 12), seed, front: 0.40, keep: !!process.env.SNAP });
      if (process.env.SNAP && seed === 1) (await import('node:fs')).writeFileSync(process.env.SNAP, JSON.stringify({ load: res.balls.map(P).filter(held1690) }));
      if (res.world) res.world.free();
      console.log(fillReport(`1690 intaking at ${SPEED} m/s, seed ${seed}`, res) + `; ${esc.report()}`);
      runs.push({ ...res, esc: esc.out.size, loadTop: esc.top, tallAt: esc.tallAt });
    }
    const f = runs.map((x) => x.fulls[0]?.n).filter((n) => n !== undefined);
    console.log(`1690 through its intake (${runs.length} fills): the most it held ${Math.min(...runs.map((x) => x.most))}-${Math.max(...runs.map((x) => x.most))} (mean ${(runs.reduce((a, x) => a + x.most, 0) / runs.length).toFixed(1)}), full at ${f.length ? `${Math.min(...f)}-${Math.max(...f)}` : '-'} (${f.length} of ${runs.length}); ${runs.reduce((a, x) => a + x.esc, 0)} escaped in all; the load's top under the net ${Math.min(...runs.map((x) => x.loadTop)).toFixed(3)}-${Math.max(...runs.map((x) => x.loadTop)).toFixed(3)} m, taller than the TRENCH from ${runs.map((x) => x.tallAt ?? '-').join(', ')} in it`);
  },
  hold() {
    const r = rig();
    const w = world();
    const balls = [], esc = escapeWatch(r);
    const step = (n) => { for (let i = 0; i < n; i++) { const l = live(balls); r.step(l); w.step(); esc.check(l); } };
    fill(w, balls, step, env('LAYERS', 6));
    step(2 / DT);
    const t0 = performance.now();
    step(1 / DT);
    const ms = (performance.now() - t0) / (1 / DT);
    console.log(`  (its mechanisms with them all in: ${ms.toFixed(2)} ms a step)`);
    const n = balls.filter(inRobot).length, top = Math.max(...balls.filter(inRobot).map((b) => b.translation().y + R));
    console.log(`1690 holds ${n} of the ${balls.length} dropped in with everything off, the intake out (a layer at a time, under the net), the load's top ${top.toFixed(2)} m; ${esc.report()}`);
    if (process.env.STUCK) for (const b of balls.filter((q) => !inRobot(q))) { const [x, y, z] = P(b); console.log(`   out at ${x.toFixed(2)},${y.toFixed(2)},${z.toFixed(2)}`); }
  },
  // a full one fired at 3 m, the turret facing the shot (TURN=deg: the turret turned that far)
  async dump() {
    const { hub } = tables(), row = hub.lookup(3 - EXIT.s), h = hoodFor(row.theta / DEG), surf = surfFor(row.v), pass = !!process.env.PASS;
    const res = [];
    for (let seed = 1; seed <= SEEDS; seed++) {
      let r, w, balls, esc;
      if (process.env.FILL === 'layers') {
        r = rig({ hood: h, yaw: env('TURN', 0) * DEG }); w = world(); balls = []; esc = escapeWatch(r);
        fill(w, balls, (n) => { for (let i = 0; i < n; i++) { const l = live(balls); r.step(l); w.step(); esc.check(l); } }, env('LAYERS', 6));
      } else {
        r = rig({ hood: h, yaw: env('TURN', 0) * DEG }); r.t.intake.cmd = 1; esc = escapeWatch(r);
        const f = fillRun(RAPIER, { world: () => world({ carpet: false }), ball, path: { step: (dt, l) => { r.step(l); esc.check(l); }, get contacts() { return r.path.contacts; } }, DT, held: held1690, own: own1690, top: TOP1690, speed: SPEED, sec: env('SEC', 12), seed, front: 0.40, keep: true, until: (full, t) => full || t > env('SEC', 12) - DT });
        w = f.world; balls = f.balls; f.carpet.setLinvel({ x: 0, y: 0, z: 0 }, true);
        r.t.intake.cmd = null;
        for (const b of balls) { const [x, y] = P(b); if (x > INTAKE.frame.front + 0.02 || y < 0.03) b.setTranslation({ x: 50, y: -5, z: 0 }, true); }
      }
      for (let i = 0; i < 1 / DT; i++) { r.spin(surf); const l = live(balls); r.step(l); w.step(); esc.check(l); }
      const n0 = balls.filter((b) => held1690(P(b))).length;
      const watch = shotWatch(r);
      let minSurf = Infinity, gates = null;
      for (let k = 0; k < env('SEC2', 8) / DT; k++) {
        r.spin(surf); r.index(true, pass); r.t.intake.cmd = process.env.NOINTAKE ? null : 1;
        const l = live(balls);
        r.step(l); w.step(); watch.check(l, k * DT); esc.check(l);
        // (GATES=1: when each FUEL first gets into the belts' channel, the corridor, the turret's bore)
        // (SNAPAT=s with SNAP=file: where the FUEL in it are s into the dump, the first fill's, for a picture)
        if (process.env.SNAPAT && seed === 1 && k === Math.round(Number(process.env.SNAPAT) / DT)) (await import('node:fs')).writeFileSync(process.env.SNAP, JSON.stringify(balls.filter(inRobot).map(P)));
        if (process.env.GATES) {
          gates ??= { channel: new Map(), corridor: new Map(), bore: new Map() };
          for (const b of l) {
            const [x, y, z] = P(b);
            if (x < BELT.x[1] && z > BELT.bottom.z[0] && !gates.channel.has(b)) gates.channel.set(b, k * DT);
            if (z < BELT.bottom.z[0] - 0.02 && x < FEED.front && !gates.corridor.has(b)) gates.corridor.set(b, k * DT);
            if (y > TURRET.deck && Math.hypot(x - TURRET.at[0], z - TURRET.at[1]) < 0.1 && !gates.bore.has(b)) gates.bore.set(b, k * DT);
          }
        }
        if (k > 60) minSurf = Math.min(minSurf, Math.abs(r.t.wheel.omega) * SHOOTER.r);
        // (JUNCTION=s: until s into the dump, every 0.1 s, the FUEL by the belts' left end, the vertical
        // kicker and the corridor, and what they touch)
        if (process.env.JUNCTION && k * DT < Number(process.env.JUNCTION) && k % 12 === 0) {
          console.log(`   ${(k * DT).toFixed(2)} s: ${watch.shots.length} shot`);
          for (const b of l) { const [x, y, z] = P(b); if (x < 0.06 && z < 0.0 && y < 0.5 && inRobot(b)) { const v = b.linvel(); console.log(`     ${x.toFixed(3)},${y.toFixed(3)},${z.toFixed(3)} v ${v.x.toFixed(1)},${v.y.toFixed(1)},${v.z.toFixed(1)}: ${[...r.path.contacts, ...r.shooter.contacts].filter((c) => c.ball === b).map((c) => c.wheel.name.replace(/ \(.*\)/, '') + ' ' + c.force.toFixed(0)).join(', ')}`); } }
        }
        if (process.env.TALLY && k % 60 === 0) {
          const c = { floor: 0, front: 0, belts: 0, corridor: 0, turret: 0, over: 0 };
          for (const b of l) {
            if (!inRobot(b)) continue;
            const [x, y, z] = P(b);
            if (Math.hypot(x - TURRET.at[0], z - TURRET.at[1]) < 0.12 && y > 0.33) c.turret++; else if (x < -0.05 && z < BELT.bottom.z[0]) c.corridor++; else if (x < -0.05) c.belts++; else if (x > HALF_L) c.front++; else if (y < 0.33) c.floor++; else c.over++;
          }
          console.log(`   ${(k * DT).toFixed(1)} s: ${watch.shots.length} shot; wheel ${(Math.abs(r.t.wheel.omega) * SHOOTER.r).toFixed(1)} m/s; ${Object.entries(c).map(([q, n]) => q + ' ' + n).join(', ')}`);
        }
      }
      if (process.env.SNAP && !process.env.SNAPAT) (await import("node:fs")).writeFileSync(process.env.SNAP, JSON.stringify(balls.filter(inRobot).map(P)));
      if (gates) for (const [g, m] of Object.entries(gates)) { const ts = [...m.values()].sort((a, b) => a - b); console.log(`   ${g}: ${ts.length} in, ${ts.length > 1 ? ((ts.length - 1) / (ts[ts.length - 1] - ts[0])).toFixed(1) : '-'} a second; by the half second ${Array.from({ length: 16 }, (_, i) => ts.filter((x) => x < (i + 1) * 0.5 && x >= i * 0.5).length).join(' ')}`); }
      const s = watch.shots;
      const rate = (q) => (q.length > 1 ? (q.length - 1) / (q[q.length - 1].t - q[0].t) : 0);
      const mid = (q) => { const a = Math.floor(q.length * 0.1), b = Math.ceil(q.length * 0.9) - 1; return b > a ? (b - a) / (q[b].t - q[a].t) : 0; };
      const best = s.reduce((m, x, i) => Math.max(m, s.slice(i).filter((y) => y.t < x.t + 1).length), 0);
      const land = s.map((o) => landing(o) - 3), hits = land.filter((d) => Math.abs(d) < 0.45).length;
      const first = s.filter((x) => x.t - (s[0]?.t ?? 0) < 1).length;
      res.push({ esc: esc.out.size, n0, n: s.length, rate: rate(s), mid: mid(s), best, first, land: med(land), left: balls.filter(inRobot).length, minSurf });
      console.log(`  fill ${seed}: ${s.length} of ${n0} out in ${env("SEC2", 8)} s, ${first} in the second from the first shot, ${best} in its best second, ${rate(s).toFixed(1)} FUEL/s over the dump, ${mid(s).toFixed(1)} through the middle; down at the rim ${med(land) >= 0 ? "+" : ""}${med(land).toFixed(2)} m from the HUB's center (median), ${hits} of ${s.length} within 0.45 m of it; the wheel ${surf.toFixed(1)} m/s dipped to ${minSurf.toFixed(1)}; ${res.at(-1).left} left in it; ${esc.report()}`);
      if (process.env.STUCK) for (const b of balls.filter(inRobot)) { const [x, y, z] = P(b); console.log(`   left at ${x.toFixed(2)},${y.toFixed(2)},${z.toFixed(2)}: ${r.path.contacts.filter((c) => c.ball === b).map((c) => c.wheel.name + ' ' + c.force.toFixed(0)).join(', ')}`); }
      w.free();
    }
    const m = (f) => med(res.map(f));
    console.log(`1690 dumps a full hopper at 3 m (${pass ? 'the passing' : 'the scoring'} speeds), ${res.length} fills: ${m((x) => x.n)} of ${m((x) => x.n0)} (median), ${m((x) => x.rate).toFixed(1)} FUEL/s over the dump (${Math.min(...res.map((x) => x.rate)).toFixed(1)}..${Math.max(...res.map((x) => x.rate)).toFixed(1)}), ${m((x) => x.first)} in the second from the first shot (${res.map((x) => x.first).join(", ")}), ${m((x) => x.best)} in its best second, ${m((x) => x.mid).toFixed(1)} through the middle (${Math.min(...res.map((x) => x.mid)).toFixed(1)}..${Math.max(...res.map((x) => x.mid)).toFixed(1)}); ${res.reduce((a, x) => a + x.esc, 0)} escaped in all`);
  },
};

// fire a few FUEL up the turret's bore at hood angle h (deg about the axle) and wheel surface speed
// surf (m/s): their median exit, and how far the wheel dipped and how long it took to come back
function fireSome(h, surf) {
  const r = rig({ hood: h });
  const w = world({ carpet: false });
  for (let i = 0; i < 2 / DT; i++) { r.spin(surf); r.shooter.step(DT, []); }
  const balls = [], watch = shotWatch(r), [tx, tz] = TURRET.at;
  let dip = Infinity, low = null, rec = 0;
  for (let i = 0; i < 3 / DT && watch.shots.length < 4; i++) {
    if (i % Math.round(0.4 / DT) === 0 && balls.length < 4) balls.push(ball(w, tx + 0.01, 0.36, tz, [0, 2.5, 0]));
    r.spin(surf);
    const l = balls.filter((b) => b.translation().y < 3);
    r.step(l); w.step(); watch.check(l, i * DT);
    if (process.env.TRACE && i % 3 === 0) for (const b of l) { const [x, y, z] = P(b), v = b.linvel(); console.log(`   ${(i * DT).toFixed(3)} ${balls.indexOf(b)} ${x.toFixed(3)},${y.toFixed(3)},${z.toFixed(3)} v ${v.x.toFixed(2)},${v.y.toFixed(2)},${v.z.toFixed(2)}: ${[...r.path.contacts, ...r.shooter.contacts].filter((c) => c.ball === b).map((c) => c.wheel.name + ' ' + c.force.toFixed(0)).join(', ')}`); }
    const now = Math.abs(r.t.wheel.omega) * SHOOTER.r;
    if (now < dip) { dip = now; low = i; }
    if (low !== null && rec === 0 && now > 0.97 * surf && i > low) rec = (i - low) * DT;
  }
  w.free();
  const o = watch.shots;
  if (!o.length) return null;
  return { n: o.length, v: med(o.map((q) => q.v)), el: med(o.map((q) => q.el)), side: med(o.map((q) => q.side)), x: med(o.map((q) => q.x)), y: med(o.map((q) => q.y)), surf, dip, rec };
}

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
  for (let i = 0; i < 1 / DT; i++) r.step([]);
  const free = Math.abs(r.t.intake.omega) || 1;
  const balls = [];
  for (let i = 0; i < n; i++) balls.push(ball(w, 0.9 + 0.17 * Math.floor(i / 4) + 0.02 * rnd(), R + 0.003, ((i % 4) - 1.5) * 0.16 + 0.02 * rnd(), [-speed, 0, 0]));
  let minIntake = 1;
  const timeline = [];
  for (let k = 0; k < sec / DT; k++) {
    const l = balls.filter((b) => b.translation().y < 3 && b.translation().y > -1);
    r.step(l); w.step();
    minIntake = Math.min(minIntake, Math.abs(r.t.intake.omega) / free);
    if (k % Math.round(1 / DT) === Math.round(1 / DT) - 1) timeline.push(balls.filter((b) => { const [x, y] = P(b); return y < 3 && x < 0.40 && y > 0.16; }).length);
    for (const b of l) { const p = b.translation(); if (p.x < -0.8) b.setTranslation({ x: 50, y: -5, z: 0 }, true); }
    if (process.env.TRACE && k % 6 === 0) {
      const b = balls[Number(process.env.TRACE)], [x, y, z] = P(b), v = b.linvel();
      console.log(`   ${(k * DT).toFixed(2)} ${x.toFixed(3)},${y.toFixed(3)},${z.toFixed(3)} v ${v.x.toFixed(2)},${v.y.toFixed(2)}: ${r.path.contacts.filter((c) => c.ball === b).map((c) => c.wheel.name + ' ' + c.force.toFixed(0)).join(', ')}`);
    }
  }
  const inside = balls.filter((b) => { const [x, y] = P(b); return y < 3 && x < HALF_L && y > 0.12; }).length;
  const left = balls.filter((b) => { const [x, y] = P(b); return y < 3 && x >= HALF_L && x < 1; }).map(P);
  w.free();
  return { inside, timeline, minIntake, left };
}

const which = process.argv.slice(2);
for (const name of which.length ? which : ['rollers', 'shooter', 'feed', 'intake', 'hold']) {
  if (!tests[name]) { console.log(`no test ${name}`); continue; }
  await tests[name]();
}
