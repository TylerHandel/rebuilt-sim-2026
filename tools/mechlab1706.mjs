// 1706's mechanisms (js/mech/r1706.js) on their own, no field, no robot: the CAD's ball path with its
// rollers and wheels, measured against what the team says (their thread: about 12-13 BPS) and the
// estimates where nothing's published. Run:
//   node --import ./tools/node-env.mjs tools/mechlab1706.mjs [test ...]
// Tests: rollers (each roller's free speed under its loop, the shooters' spin-up, the arm out and in),
// shooter (FUEL pushed up a turret's throat into its wheels at hood swings and speeds: exit speed and
// elevation against the accelerators' surface and the CAD's geometry; and the sim's shot table's
// rows: where they come down), spin (FUEL laid in one spindexer's bowl, that side feeding: how fast
// it delivers, either way it turns), intake (24 FUEL at SPEED m/s, intaking alone), intakefull
// (filled through the intake from the NEUTRAL ZONE's FUEL until it can't put more in:
// tools/fillbench.mjs), hold (filled a layer at a time, everything off), path (carpet to shot:
// intaking and both turrets firing), dump (a full one fired at 3 m: FUEL/s per turret and in all).
// Env: SEEDS, SPEED (1 m/s), STUCK (where the ones left over are), TRACE=i (follow FUEL i), SPIN_RPS
// and KICK_RPM (phys1706's setpoints), FILL=layers (dump: filled a layer at a time instead of
// through the intake), LAYERS, SEC, MAP=1 (shooter: the full sweep)
import { RAPIER } from '../js/physics.js';
import { FUEL, GROUP, groups, HUB } from '../js/constants.js';
import { Mechanism } from '../js/mech/mechanism.js';
import { sim2D, ShotTable } from '../js/ballistics.js';
import {
  build1706, wheels1706, shooterWheels, pathSurfaces1706, placeIntake, placeBox, placeSpin, placeTurret, placeShooter, armRot, boxExt, hoodElevation, turretPoint,
  INTAKE, SPIN, TURRET, SHOOTER, PIVOT, HALF_L, HALF_W, BOX, SPIN_DIR, MU as GRIP, TREAD,
} from '../js/mech/r1706.js';
import PATH from '../js/mech/path1706.js';
import { servoArm, turnArm, speedLoop, profile, accelFor, exitAt, hoodFor, elevationAt, SPIN_RPS, KICK_RPM, EXIT } from '../js/mech/phys1706.js';
import { ROBOTS } from '../js/robotConfigs.js';
import { fillRun, fillReport } from './fillbench.mjs';

await RAPIER.init();
// (GRIP_<material>=mu, TREAD_<material>=N/m: try another estimate)
for (const k of Object.keys(GRIP)) if (process.env['GRIP_' + k]) GRIP[k] = Number(process.env['GRIP_' + k]);
for (const k of Object.keys(TREAD)) if (process.env['TREAD_' + k]) TREAD[k] = Number(process.env['TREAD_' + k]);
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

// the robot's mechanisms, fixed in the world: the arm at deg (out: 113.4), the turrets at yaws (rad;
// facing out to their sides), the hoods at swing h (deg)
function rig({ deg = INTAKE.out, yaws = [90 * DEG, -90 * DEG], hood = [0, 0] } = {}) {
  const t = build1706(), wh = wheels1706(t), s = pathSurfaces1706(PATH, t, process.env.MU ? { mu: Number(process.env.MU) } : undefined), sw = shooterWheels(t);
  const pathW = [...wh.intake, ...wh.kicker];
  const path = new Mechanism({ substeps: 6, shafts: [...new Set(pathW.map((x) => x.shaft)), t.pivot, ...t.spin], wheels: pathW, surfaces: [s.body, s.bowl, s.intake, s.hopper, ...s.spin, ...s.turret, ...s.hood, s.bumper, s.net] });
  const shooter = new Mechanism({ substeps: 6, sub: Number(process.env.SUB ?? 8), shafts: sw.flat().map((x) => x.shaft), wheels: sw.flat() });
  t.intake.cmd = null;
  for (const k of t.kicker) k.train.cmd = null;
  for (const s2 of t.shooter) s2.train.cmd = null;
  for (const sp of t.spin) sp.cmd = null;
  const r = { t, wh, s, sw, path, shooter, deg, set: [deg * DEG, 0], goal: deg, yaws, hood: hood.slice(), phi: [0, 0], servo: false };
  r.pose = () => {
    s.intake.setPose([...PIVOT, 0], 0, armRot(r.deg));
    placeIntake(wh.intake, r.deg);
    placeBox(s.hopper, PATH.ext, r.deg);
    s.net.place(boxExt(PATH.ext, r.deg));
    for (let i = 0; i < 2; i++) { placeSpin(s.spin[i], i, r.phi[i]); placeTurret(s, i, r.yaws[i], r.hood[i]); placeShooter(i, sw[i], r.yaws[i], r.hood[i]); }
  };
  // the arm: held where it is (r.servo false: its position loop holding it, what the FUEL do to it
  // aside), or its Motion Magic toward r.goal
  r.step = (balls, together = balls) => {
    r.pose();
    if (r.servo) r.set = servoArm(t, r.deg, r.set, r.goal, DT); else { t.pivot.cmd = 0; t.pivot._omega = 0; }
    path.step(DT, balls, together);
    shooter.step(DT, balls.filter((b) => b.translation().y > 0.36));
    if (r.servo) r.deg = turnArm(t, r.deg, DT);
    for (let i = 0; i < 2; i++) r.phi[i] = (r.phi[i] + t.spin[i].omega * DT) % TAU;
  };
  // a turret firing: its accelerators at the exit speed v (m/s), its spindexer and kicker feeding
  r.fire = (i, v, feed = true) => {
    speedLoop(t.shooter[i].train, v ? accelFor(v) / SHOOTER.kAccel : null, SHOOTER.kP);
    speedLoop(t.spin[i], feed ? SPIN_RPS * TAU : null);
    speedLoop(t.kicker[i].train, feed ? (KICK_RPM / 60) * TAU : null);
  };
  r.pose();
  return r;
}
// a FUEL leaving the shooters' wheels going up: shot (which turret, its speed, elevation, heading
// against that turret's yaw, and where it left from, turret frame)
function shotWatch(r) {
  const touched = new Map(), shots = [];
  return {
    shots,
    check(balls, t) {
      for (const b of balls) {
        const c = r.shooter.contacts.find((q) => q.ball === b);
        if (c) touched.set(b, r.sw.findIndex((ws) => ws.includes(c.wheel)));
        else if (touched.has(b)) {
          const i = touched.get(b); touched.delete(b);
          const v = b.linvel(), p = b.translation();
          if (v.y > 1 && Math.hypot(v.x, v.y, v.z) > 3) {
            const yaw = r.yaws[i], fx = Math.cos(yaw), fz = -Math.sin(yaw), [tx, tz] = TURRET[i];
            shots.push({ t, i, v: Math.hypot(v.x, v.y, v.z), el: (Math.atan2(v.y, Math.hypot(v.x, v.z)) * 180) / Math.PI, side: (Math.atan2(v.x * -fz + v.z * fx, v.x * fx + v.z * fz) * 180) / Math.PI, x: (p.x - tx) * fx + (p.z - tz) * fz, y: p.y, b });
            b.setTranslation({ x: 50 + shots.length, y: 5, z: 0 }, true); b.setGravityScale(0, true); b.setLinvel({ x: 0, y: 0, z: 0 }, true);
          }
        }
      }
    },
  };
}
// where a shot comes down through the HUB's rim height, out from the turret's axis along the shot
function landing(o) { const hit = sim2D(o.y, o.v, (o.el * Math.PI) / 180, HUB.rimFront); return hit ? hit.d + o.x : NaN; }
// the sim's shot tables for 1706 (js/robot.js builds the same from its config's shooter)
function tables() {
  const sh = ROBOTS['1706'].shooter;
  const hub = new ShotTable({ h0: sh.exitY, Ht: HUB.targetHeight, hoodMin: sh.hoodMin, hoodMax: sh.hoodMax, speedMax: sh.speedMax, mode: 'hub', clearDist: HUB.size / 2 + FUEL.radius + 0.02, clearHeight: HUB.rimFront + FUEL.radius + 0.04 });
  return { hub };
}
const live = (balls) => balls.filter((b) => b.translation().y < 3 && b.translation().y > -1);
// in it: the hopper (the frame), the box, the ramps and turrets
const inRobot = (b) => { const [x, y, z] = P(b); return x > -HALF_L - 0.02 && x < HALF_L + 0.31 && Math.abs(z) < HALF_W && y > 0.09 && y < 0.8; };
// as js/mech/phys1706.js where() has it: in it, and the intake's (climbing the bumper)
const held1706 = (ext) => ([x, y, z]) => (Math.abs(x) < HALF_L && Math.abs(z) < HALF_W && y > 0.09 && y < 0.8) || (x >= HALF_L && x < HALF_L + ext + 0.01 && Math.abs(z) < BOX.side && y > 0.16 && y < 0.78) || (x >= HALF_L && x < 0.56 && Math.abs(z) < 0.37 && y > 0.24 && y < 0.45);
const own1706 = (ext) => (p) => held1706(ext)(p) || (p[0] >= HALF_L && p[0] < 0.66 && Math.abs(p[2]) < 0.37 && p[1] > 0.03 && p[1] < 0.45);
const TOP1706 = { topX: 0.12, topN: 10 }; // (js/mech/phys1706.js naturalFull)
// fill it a layer at a time (FUEL set into the space over the ones already in, let settle)
function fill(w, balls, step, layers = 8, y = 0.62, x1 = HALF_L + 0.3) {
  for (let layer = 0; layer < layers; layer++) {
    for (let i = 0; i < 6; i++) for (let k = 0; k < 5; k++) {
      const x = -0.0 + 0.155 * i, z = -0.31 + 0.155 * k;
      if (x > x1 - 0.06) continue;
      if (balls.some((b) => { const p = b.translation(); return Math.hypot(p.x - x, p.y - y, p.z - z) < 2 * R; })) continue;
      balls.push(ball(w, x, y, z));
    }
    step(1.0 / DT);
  }
}

const tests = {
  rollers() {
    const r = rig(), t = r.t;
    t.intake.cmd = 1;
    for (let i = 0; i < 2; i++) r.fire(i, 10);
    for (let i = 0; i < 2 / DT; i++) r.step([]);
    const surf = (sh, rr) => Math.abs(sh.omega) * rr;
    console.log(`1706's rollers, free (intake 100%, the spindexers at ${SPIN_RPS} rps, the kickers at ${KICK_RPM} rpm, the shooters at a 10 m/s shot):`);
    console.log(`  intake X60s ${RPM(t.intake.omega).toFixed(0)} rpm: top roller ${surf(t.top, INTAKE.top.r).toFixed(2)} m/s at its surface, bottom ${surf(t.bottom, INTAKE.bottom.r).toFixed(2)}, 1in wheels ${surf(t.wheels, INTAKE.wheels.r).toFixed(2)}`);
    console.log(`  spindexers ${(t.spin[0].omega / TAU).toFixed(2)} / ${(t.spin[1].omega / TAU).toFixed(2)} rps (free ${(t.spin[0].motor.freeSpeed / TAU).toFixed(1)}): ${(t.spin[0].omega * 0.115).toFixed(2)} m/s at a FUEL's center by the wall (0.115 m out)`);
    console.log(`  kickers ${RPM(t.kicker[0].train.omega).toFixed(0)} rpm (free ${RPM(t.kicker[0].train.motor.freeSpeed).toFixed(0)}): ${surf(t.kicker[0].front, 0.0254).toFixed(2)} m/s at the 2in wheels' surface`);
    console.log(`  shooters: accelerators ${surf(t.shooter[0].accel, SHOOTER.accel.r).toFixed(2)} m/s (${RPM(t.shooter[0].accel.omega).toFixed(0)} rpm), 1in hood wheels ${surf(t.shooter[0].hood, SHOOTER.hood[0].r).toFixed(2)} m/s; free: accelerators ${(t.shooter[0].train.motor.freeSpeed * SHOOTER.kAccel * SHOOTER.accel.r).toFixed(1)} m/s`);
    for (const v of [6, 8, 10, 12]) {
      const r2 = rig();
      let T = 0;
      for (; T < 4; T += DT) { r2.fire(0, v, false); r2.shooter.step(DT, []); if (Math.abs(exitAt(r2.t.shooter[0].accel.omega) - v) < SHOOTER.tolV) break; }
      console.log(`  a shooter from rest to within ${SHOOTER.tolV} m/s of a ${v} m/s shot: ${T.toFixed(2)} s`);
    }
    // the arm: out from folded in, and back in
    const r3 = rig({ deg: 0 });
    r3.servo = true;
    const move = (goal) => { r3.goal = goal; let k = 0; for (; k < 3 / DT; k++) { r3.step([]); if (Math.abs(r3.deg - goal) < 2 && Math.abs(r3.t.pivot.omega) < 0.05) break; } return k * DT; };
    const out = move(INTAKE.out), outAt = r3.deg, back = move(0);
    console.log(`  the arm out (0 to ${INTAKE.out}) in ${out.toFixed(2)} s (at ${outAt.toFixed(1)}; the box ${boxExt(PATH.ext, outAt).toFixed(3)} m out), back in in ${back.toFixed(2)} s (at ${r3.deg.toFixed(1)})`);
  },
  // FUEL pushed up a turret's throat into its wheels (the kickers off): at hood swings and exit speeds
  shooter() {
    const list = (k) => process.env[k]?.split(',').map(Number);
    const hs = list('HS') ?? (process.env.MAP ? [0, 5, 10, 15, 20, 25, 30, 35, 40, 45] : [0, 15, 30, 45]);
    const vs = list('VS') ?? (process.env.MAP ? [5, 6, 7, 8, 9, 10, 11, 12] : [6, 8, 10]);
    console.log(`1706's shooter: FUEL set into the right turret's kicker (at ${KICK_RPM} rpm) (the turret facing out to its side), the accelerators at the speed js/mech/phys1706.js's EXIT_MAP gives a shot ("asked"), the hood at a swing:`);
    const ratios = [], els = [];
    for (const h of hs) for (const v of vs) {
      const o = fireSome(h, v);
      if (!o) { console.log(`  hood ${h} (${hoodElevation(h).toFixed(1)} by the CAD), ${v} m/s: nothing came out`); continue; }
      ratios.push(o.v / o.surf); els.push([h, o.el]);
      console.log(`  hood ${String(h).padStart(2)} (${hoodElevation(h).toFixed(1)} deg by the CAD), ${v} m/s asked: ${o.n}/4 out at ${o.v.toFixed(2)} m/s (${(o.v / o.surf).toFixed(2)} of the accelerators' ${o.surf.toFixed(1)}), ${o.el.toFixed(1)} deg, ${o.side.toFixed(1)} deg aside, from ${o.x.toFixed(3)},${o.y.toFixed(3)}; down at the rim ${landing(o).toFixed(2)} m out`);
    }
    if (ratios.length) console.log(`  exit over the accelerators' surface: median ${med(ratios).toFixed(3)} (${Math.min(...ratios).toFixed(2)}..${Math.max(...ratios).toFixed(2)})`);
    // the sim's table's rows (js/robot.js's hubTable from the config), fired as phys1706 fires them
    if (process.env.NOTABLE) return;
    const { hub } = tables();
    console.log(`  the sim's HUB table (its config's exit at ${ROBOTS['1706'].shooter.exitY} m), fired as js/mech/phys1706.js fires it:`);
    for (const d of [1.5, 2, 2.5, 3, 3.5, 4, 4.5, 5, 5.5, 6]) {
      const row = hub.lookup(d - EXIT.x);
      if (!row) { console.log(`   ${d} m: out of the table`); continue; }
      const o = fireSome(hoodFor(row.theta / DEG), row.v);
      if (!o) { console.log(`   ${d} m: nothing came out`); continue; }
      const land = landing(o);
      console.log(`   ${d} m: the table's ${row.v.toFixed(2)} m/s at ${(row.theta / DEG).toFixed(1)} deg (hood ${hoodFor(row.theta / DEG).toFixed(1)}): out at ${o.v.toFixed(2)} m/s, ${o.el.toFixed(1)} deg, from ${o.x.toFixed(2)},${o.y.toFixed(2)}; down at the rim ${land.toFixed(2)} m (${land - d >= 0 ? '+' : ''}${(land - d).toFixed(2)})`);
    }
  },
  // the shot maps behind js/mech/phys1706.js's EXIT_MAP and EL_MAP, tuned the way a team tunes its
  // table: for each of the sim's HUB table's rows, the accelerators' speed and the hood moved until the
  // FUEL leave at the row's speed and elevation (fed by the kicker, one after another)
  calibrate() {
    const { hub } = tables(), pairs = [], els = [];
    for (const d of [1.25, 1.5, 2, 2.5, 3, 3.5, 4, 4.5, 5, 5.5]) {
      const row = hub.lookup(d - EXIT.x);
      if (!row) continue;
      const el = row.theta / DEG;
      let surf = accelFor(row.v) * SHOOTER.accel.r, h = hoodFor(el), o = null;
      for (let k = 0; k < 4; k++) {
        o = fireSome(h, null, surf);
        if (!o) break;
        surf *= Math.pow(row.v / o.v, 1.5); h = Math.max(0, Math.min(SHOOTER.hoodMax, h + (o.el - el)));
      }
      if (!o) continue;
      pairs.push([o.v, o.surf]); els.push([h, o.el]);
      console.log(`  ${d} m: the table's ${row.v.toFixed(2)} m/s at ${el.toFixed(1)} deg: the accelerators at ${surf.toFixed(2)} m/s, the hood at ${h.toFixed(1)} (last: out at ${o.v.toFixed(2)} m/s, ${o.el.toFixed(1)} deg from ${o.surf.toFixed(2)} m/s; down at the rim ${landing(o).toFixed(2)} m)`);
    }
    console.log(`  EXIT_MAP pairs (exit, accelerators' surface): ${JSON.stringify(pairs.map((p) => p.map((v) => +v.toFixed(2))))}`);
    console.log(`  EL_MAP pairs (hood, elevation): ${JSON.stringify(els.map((p) => p.map((v) => +v.toFixed(1))))}`);
  },
  // FUEL laid in one spindexer's bowl, that side feeding and firing (DIR=-1 turns it the other way)
  spin() {
    for (const dir of process.env.DIR ? [Number(process.env.DIR)] : [1, -1]) {
      const res = [];
      for (let seed = 1; seed <= SEEDS; seed++) {
        SPIN_DIR[0] = dir;
        const r = rig();
        const w = world();
        const balls = [];
        let sd = seed; const rnd = () => ((sd = (sd * 16807) % 2147483647) / 2147483647 - 0.5);
        const [sx, sz] = SPIN.axes[0];
        for (let k = 0; k < 10; k++) { const a = (k / 5) * TAU + rnd(); balls.push(ball(w, sx + (k < 5 ? 0.11 : 0.08) * Math.cos(a), SPIN.top + R + 0.01 + (k < 5 ? 0 : 0.15), sz + (k < 5 ? 0.11 : 0.08) * Math.sin(a))); }
        for (let i = 0; i < 1 / DT; i++) { r.fire(0, 8, false); r.step(live(balls)); w.step(); }
        const watch = shotWatch(r);
        for (let k = 0; k < 6 / DT; k++) {
          r.fire(0, 8); const l = live(balls); r.step(l); w.step(); watch.check(l, k * DT);
          // (TRACE=1: every quarter second, each FUEL and what it touches)
          if (process.env.TRACE && k % 30 === 0) for (const b of l) { const [x, y, z] = P(b), v = b.linvel(); console.log(`   ${(k * DT).toFixed(2)} ${balls.indexOf(b)} at ${x.toFixed(3)},${y.toFixed(3)},${z.toFixed(3)} v ${v.x.toFixed(2)},${v.y.toFixed(2)},${v.z.toFixed(2)}: ${[...r.path.contacts, ...r.shooter.contacts].filter((c) => c.ball === b.handle || c.ball === b).map((c) => c.wheel.name + ' ' + c.force.toFixed(0)).join(', ')}`); }
        }
        const s = watch.shots.map((x) => x.t);
        res.push({ n: s.length, rate: s.length > 1 ? (s.length - 1) / (s[s.length - 1] - s[0]) : 0, first: s[0] ?? NaN });
        if (process.env.STUCK) for (const b of live(balls)) { const [x, y, z] = P(b); console.log(`   left at ${x.toFixed(2)},${y.toFixed(2)},${z.toFixed(2)}: ${r.path.contacts.filter((c) => c.ball === b).map((c) => c.wheel.name + ' ' + c.force.toFixed(0)).join(', ')}`); }
        w.free();
      }
      SPIN_DIR[0] = 1;
      console.log(`1706's right spindexer turning ${dir > 0 ? 'counterclockwise' : 'clockwise'} seen from above at ${SPIN_RPS} rps, 10 FUEL in its bowl, its kicker at ${KICK_RPM} rpm: ${res.map((x) => x.n).join(', ')} shot in 6 s, ${res.map((x) => x.rate.toFixed(1)).join(', ')} FUEL/s, the first after ${res.map((x) => x.first.toFixed(2)).join(', ')} s`);
    }
  },
  // intaking alone: 24 FUEL four abreast on a treadmill carpet at SPEED
  intake() {
    for (const speed of process.env.SPEED ? [SPEED] : [1, 3]) {
      const res = [];
      for (let seed = 1; seed <= SEEDS; seed++) res.push(pathRun(seed, { shoot: false, n: 24, sec: 6, speed }));
      console.log(`1706 intaking only (rollers 100%, the arm out), 24 FUEL at ${speed} m/s: ${res.map((x) => x.inside).join(', ')} in it after 6 s; past the bumper each second ${res.map((x) => x.timeline.join('/')).join(', ')}; intake X60s down to ${med(res.map((x) => x.minIntake)).toFixed(0)}% (median)`);
      if (process.env.STUCK) for (const x of res) console.log('   left', x.left.map((p) => p.map((v) => v.toFixed(2)).join(',')).join(' | '));
    }
  },
  // filled through the intake until it can't put more in (tools/fillbench.mjs): the NEUTRAL ZONE's
  // FUEL, five abreast, at SPEED; the arm out, the rollers at 100%, the spindexers still
  intakefull() {
    const runs = [];
    for (let seed = 1; seed <= SEEDS; seed++) {
      const r = rig();
      r.t.intake.cmd = 1;
      const ext = boxExt(PATH.ext, r.deg);
      const res = fillRun(RAPIER, { world: () => world({ carpet: false }), ball, path: { step: (dt, l) => r.step(l), get contacts() { return r.path.contacts; } }, DT, held: held1706(ext), own: own1706(ext), top: TOP1706, speed: SPEED, sec: env('SEC', 12), seed, front: 0.398 });
      console.log(fillReport(`1706 intaking at ${SPEED} m/s, seed ${seed}`, res));
      runs.push(res);
    }
    const f = runs.map((x) => x.fulls[0]?.n).filter((n) => n !== undefined);
    console.log(`1706 through its intake (${runs.length} fills): the most it held ${Math.min(...runs.map((x) => x.most))}-${Math.max(...runs.map((x) => x.most))} (mean ${(runs.reduce((a, x) => a + x.most, 0) / runs.length).toFixed(1)}), full at ${f.length ? `${Math.min(...f)}-${Math.max(...f)}` : '-'} (${f.length} of ${runs.length})`);
  },
  hold() {
    const r = rig();
    const w = world();
    const balls = [];
    const step = (n) => { for (let i = 0; i < n; i++) { r.step(live(balls)); w.step(); } };
    fill(w, balls, step, env('LAYERS', 9));
    step(2 / DT);
    const t0 = performance.now();
    step(1 / DT);
    const ms = (performance.now() - t0) / (1 / DT);
    console.log(`  (its mechanisms with them all in: ${ms.toFixed(2)} ms a step)`);
    const n = balls.filter(inRobot).length, top = Math.max(...balls.filter(inRobot).map((b) => b.translation().y + R));
    console.log(`1706 holds ${n} with everything off, the intake out and the box out (a layer at a time), the load's top ${top.toFixed(2)} m`);
    if (process.env.STUCK) for (const b of balls.filter((q) => !inRobot(q))) { const [x, y, z] = P(b); console.log(`   out at ${x.toFixed(2)},${y.toFixed(2)},${z.toFixed(2)}`); }
  },
  // carpet to shot: FUEL in front, the robot driving into them, intaking and both turrets firing
  path() {
    const runs = [];
    for (let seed = 1; seed <= SEEDS; seed++) runs.push(pathRun(seed, { n: 20, sec: 8, speed: SPEED }));
    console.log(`1706 carpet to shot (20 FUEL, four abreast, at ${SPEED} m/s, the intake at 100%, both turrets firing a 10 m/s shot): ${runs.map((x) => x.shots).join(', ')} shot (median ${med(runs.map((x) => x.shots))}), ${runs.map((x) => x.rate.toFixed(1)).join(', ')} FUEL/s once they're coming; ${runs.map((x) => x.inside).join(', ')} in it at the end`);
  },
  // a full one fired at 3 m: both turrets, each its own side's spindexer and kicker
  async dump() {
    const { hub } = tables(), row = hub.lookup(3 - EXIT.x), h = hoodFor(row.theta / DEG);
    const res = [];
    for (let seed = 1; seed <= SEEDS; seed++) {
      let r, w, balls;
      if (process.env.FILL === 'layers') {
        r = rig({ hood: [h, h] }); w = world(); balls = [];
        fill(w, balls, (n) => { for (let i = 0; i < n; i++) { r.step(live(balls)); w.step(); } }, env('LAYERS', 9));
      } else {
        // filled through the intake until it's full (intakefull), its rollers let go, the arm folding in
        r = rig({ hood: [h, h] }); r.t.intake.cmd = 1;
        const ext = boxExt(PATH.ext, r.deg);
        const f = fillRun(RAPIER, { world: () => world({ carpet: false }), ball, path: { step: (dt, l) => r.step(l), get contacts() { return r.path.contacts; } }, DT, held: held1706(ext), own: own1706(ext), top: TOP1706, speed: SPEED, sec: env('SEC', 12), seed, front: 0.398, keep: true, until: (full, t) => full || t > env('SEC', 12) - DT });
        w = f.world; balls = f.balls; f.carpet.setLinvel({ x: 0, y: 0, z: 0 }, true);
        r.t.intake.cmd = null;
        for (const b of balls) { const [x, y] = P(b); if (x > 0.66 || y < 0.03) b.setTranslation({ x: 50, y: -5, z: 0 }, true); }
      }
      // (spun up and settled for a second with the arm folding in, as it would driving to the HUB)
      r.servo = !process.env.NOFOLD; r.goal = 0;
      for (let i = 0; i < 1 / DT; i++) { for (let k = 0; k < 2; k++) r.fire(k, row.v, false); r.step(live(balls)); w.step(); }
      // (what it holds: in it, not ones in the intake's mouth on the carpet)
      const n0 = balls.filter((b) => held1706(boxExt(PATH.ext, r.deg))(P(b))).length;
      const watch = shotWatch(r);
      for (let k = 0; k < env('SEC2', 10) / DT; k++) {
        for (let i = 0; i < 2; i++) r.fire(i, row.v);
        const l = live(balls);
        // (ESC=1: each FUEL that leaves it other than as a shot: where, how fast, what it last touched)
        const was = process.env.ESC ? new Map(l.map((b) => [b, inRobot(b) ? [...r.path.contacts, ...r.shooter.contacts].filter((c) => c.ball === b).map((c) => c.wheel.name + ' ' + c.force.toFixed(0)).join(', ') || '-' : null])) : null;
        r.step(l); w.step(); watch.check(l, k * DT);
        if (was) for (const b of l) if (was.get(b) && !inRobot(b) && b.translation().x < 40) { const [x, y, z] = P(b), v = b.linvel(); console.log(`   ${(k * DT).toFixed(2)} s out at ${x.toFixed(2)},${y.toFixed(2)},${z.toFixed(2)} v ${v.x.toFixed(1)},${v.y.toFixed(1)},${v.z.toFixed(1)}; touching ${was.get(b)}`); }
        // (RAMP=1: every half second, the FUEL behind the spindexers, on the ramps and in the kickers
        // and turrets, and what they touch; the shots so far; the shooters' and kickers' speeds)
        if (process.env.RAMP && k % 60 === 30) {
          console.log(`   ${(k * DT).toFixed(1)} s: ${watch.shots.length} shot; accelerators ${r.t.shooter.map((q) => exitAt(q.accel.omega).toFixed(2)).join('/')} m/s; kickers ${r.t.kicker.map((q) => RPM(q.train.omega).toFixed(0)).join('/')} rpm; spindexers ${r.t.spin.map((q) => (q.omega / TAU).toFixed(2)).join('/')} rps`);
          for (const b of l) { const [x, y, z] = P(b); if ((process.env.RAMP === 'bowl' ? Math.hypot(x - SPIN.axes[0][0], z - SPIN.axes[0][1]) < 0.2 && y < 0.45 : x < 0.0) && inRobot(b)) console.log(`     ${x.toFixed(3)},${y.toFixed(3)},${z.toFixed(3)} v${b.linvel().y.toFixed(2)}: ${[...r.path.contacts, ...r.shooter.contacts].filter((c) => c.ball === b).map((c) => c.wheel.name + ' ' + c.force.toFixed(0)).join(', ')}`); }
        }
      }
      // (SNAP=file: where the FUEL in it are at the end, for a picture)
      if (process.env.SNAP) (await import('node:fs')).writeFileSync(process.env.SNAP, JSON.stringify(balls.filter(inRobot).map(P)));
      const s = watch.shots;
      const rate = (q) => (q.length > 1 ? (q.length - 1) / (q[q.length - 1].t - q[0].t) : 0);
      const best = s.reduce((m, x, i) => Math.max(m, s.slice(i).filter((y) => y.t < x.t + 1).length), 0);
      const per = [0, 1].map((i) => s.filter((x) => x.i === i));
      const land = s.map((o) => landing(o) - 3);
      res.push({ n0, n: s.length, rate: rate(s), best, per: per.map((q) => [q.length, rate(q)]), first: s.filter((x) => x.t - (s[0]?.t ?? 0) < 1).length, land: med(land), left: balls.filter(inRobot).length });
      console.log(`  fill ${seed}: ${s.length} of ${n0} out in ${env('SEC2', 10)} s, ${res.at(-1).first} in the first second, ${best} in its best, ${rate(s).toFixed(1)} FUEL/s over the dump (right ${per[0].length} at ${rate(per[0]).toFixed(1)}/s, left ${per[1].length} at ${rate(per[1]).toFixed(1)}/s); down at the rim ${med(land) >= 0 ? '+' : ''}${med(land).toFixed(2)} m from the HUB's center (median); ${res.at(-1).left} left in it`);
      if (process.env.STUCK) console.log(`   the arm at ${r.deg.toFixed(1)} deg (the box ${boxExt(PATH.ext, r.deg).toFixed(3)} m out); out of it: ${balls.filter((b) => !inRobot(b) && b.translation().y < 3 && b.translation().y > -1).map((b) => P(b).map((v) => v.toFixed(2)).join(',')).join(' | ')}`);
      if (process.env.STUCK) for (const b of balls.filter(inRobot)) { const [x, y, z] = P(b); console.log(`   left at ${x.toFixed(2)},${y.toFixed(2)},${z.toFixed(2)}: ${r.path.contacts.filter((c) => c.ball === b).map((c) => c.wheel.name + ' ' + c.force.toFixed(0)).join(', ')}`); }
      w.free();
    }
    const m = (f) => med(res.map(f));
    console.log(`1706 dumps a full hopper at 3 m (the spindexers at ${SPIN_RPS} rps, the kickers at ${KICK_RPM} rpm), ${res.length} fills: ${m((x) => x.n)} of ${m((x) => x.n0)} (median), ${m((x) => x.rate).toFixed(1)} FUEL/s over the dump (${Math.min(...res.map((x) => x.rate)).toFixed(1)}..${Math.max(...res.map((x) => x.rate)).toFixed(1)}), ${m((x) => x.best)} in its best second; per turret ${m((x) => x.per[0][1]).toFixed(1)} and ${m((x) => x.per[1][1]).toFixed(1)} FUEL/s`);
  },
};

// fire a few FUEL up turret 0's throat at hood swing h (deg) and exit speed v (m/s): their median exit
// (surf: the accelerators' surface speed instead, m/s)
function fireSome(h, v, surf = null) {
  const r = rig({ deg: 0, hood: [h, h] });
  const w = world({ carpet: false });
  if (surf) r.fire = (i, _, feed) => { speedLoop(r.t.shooter[i].train, surf / SHOOTER.accel.r / SHOOTER.kAccel, SHOOTER.kP); speedLoop(r.t.spin[i], null); };
  for (let i = 0; i < 2 / DT; i++) { r.fire(0, v, false); r.shooter.step(DT, []); }
  const balls = [], watch = shotWatch(r), [tx, tz] = TURRET[0];
  // (each set into the kicker's pinch from below, the kicker running as it feeds)
  for (let i = 0; i < 3 / DT && watch.shots.length < 4; i++) {
    if (i % Math.round(0.4 / DT) === 0 && balls.length < 4) balls.push(ball(w, tx, 0.22, tz, [0, 1, 0]));
    r.fire(0, v, false);
    speedLoop(r.t.kicker[0].train, (KICK_RPM / 60) * TAU);
    const l = balls.filter((b) => b.translation().y < 3);
    r.step(l); w.step(); watch.check(l, i * DT);
  }
  w.free();
  const o = watch.shots;
  if (!o.length) return null;
  return { n: o.length, v: med(o.map((q) => q.v)), el: med(o.map((q) => q.el)), side: med(o.map((q) => q.side)), x: med(o.map((q) => q.x)), y: med(o.map((q) => q.y)), surf: Math.abs(r.t.shooter[0].accel.omega) * SHOOTER.accel.r };
}

function pathRun(seed, { shoot = true, n = 20, sec = 7, speed = SPEED } = {}) {
  let sd = seed;
  const rnd = () => ((sd = (sd * 16807) % 2147483647) / 2147483647 - 0.5);
  const r = rig();
  const w = world({ carpet: false });
  // the carpet, running under the robot
  const carpet = w.createRigidBody(RAPIER.RigidBodyDesc.kinematicVelocityBased().setLinvel(-speed, 0, 0));
  w.createCollider(RAPIER.ColliderDesc.cuboid(15, 0.01, 1).setTranslation(0.4 + 15, -0.01, 0).setFriction(0.3).setRestitution(0.2), carpet);
  w.createCollider(RAPIER.ColliderDesc.cuboid(0.4, 0.01, 1).setTranslation(0, -0.01, 0).setFriction(0.3), w.createRigidBody(RAPIER.RigidBodyDesc.fixed()));
  r.t.intake.cmd = 1;
  for (let i = 0; i < 1 / DT; i++) { if (shoot) for (let k = 0; k < 2; k++) r.fire(k, 10, false); r.step([]); }
  const free = Math.abs(r.t.intake.omega) || 1;
  const balls = [];
  for (let i = 0; i < n; i++) balls.push(ball(w, 0.9 + 0.17 * Math.floor(i / 4) + 0.02 * rnd(), R + 0.003, ((i % 4) - 1.5) * 0.16 + 0.02 * rnd(), [-speed, 0, 0]));
  const watch = shotWatch(r);
  let minIntake = 1;
  const timeline = [];
  for (let k = 0; k < sec / DT; k++) {
    const l = balls.filter((b) => b.translation().y < 3 && b.translation().y > -1);
    if (shoot) for (let i = 0; i < 2; i++) r.fire(i, 10);
    r.step(l); w.step();
    watch.check(l, k * DT);
    minIntake = Math.min(minIntake, Math.abs(r.t.intake.omega) / free);
    if (k % Math.round(1 / DT) === Math.round(1 / DT) - 1) timeline.push(balls.filter((b) => { const [x, y] = P(b); return y < 3 && x < 0.40 && y > 0.16; }).length);
    if (process.env.TRACE && k % 6 === 0) {
      const b = balls[Number(process.env.TRACE)], [x, y, z] = P(b), v = b.linvel();
      console.log(`   ${(k * DT).toFixed(2)} s at ${x.toFixed(3)},${y.toFixed(3)},${z.toFixed(3)} v ${v.x.toFixed(2)},${v.y.toFixed(2)},${v.z.toFixed(2)}: ${[...r.path.contacts, ...r.shooter.contacts].filter((c) => c.ball === b).map((c) => c.wheel.name + ' ' + c.force.toFixed(0)).join(', ')}`);
    }
  }
  const s = watch.shots.map((x) => x.t);
  const inside = balls.filter((b) => inRobot(b) && b.translation().y < 3);
  const left = balls.filter((b) => b.translation().y < 3 && b.translation().x < 40 && !inside.includes(b)).map(P);
  w.free();
  return { timeline, shots: s.length, rate: s.length > 2 ? (s.length - 1) / (s[s.length - 1] - s[0]) : 0, inside: inside.length, minIntake: 100 * minIntake, left };
}

const pick = process.argv.slice(2);
for (const [name, fn] of Object.entries(tests)) if (!pick.length || pick.includes(name)) await fn();
