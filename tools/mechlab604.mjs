// 604's mechanisms (js/mech/r604.js) on their own, no field, no robot: the CAD's ball path with its
// rollers, measured against what the team's code and release thread say. Run:
//   node --import ./tools/node-env.mjs tools/mechlab604.mjs [test ...]
// Tests: rollers (each roller's free speed under their loops and duty cycles, the launcher's
// spin-up, the arms deploying and stowing), shooter (FUEL pushed up the tube into the turret at
// their launch table's rows: how fast and how steep they leave against their table, and whether
// they come down in the HUB), path (FUEL on a treadmill carpet, the robot driving into them at
// 1 m/s: intake, bucket, rotor, tube, turret; how many and how fast), intake (intaking alone, the
// serializer off, as their code does), hold (how many it holds, everything off), dump (a full one
// fired at 3 m: the serializer's rate, their "~15 BPS").
// Env: SEEDS (6), SPEED (1 m/s), STUCK (where the ones left over are), TRACE=i (follow FUEL i),
// SHOT=v,angle (one shot of your own: exit speed m/s and launch angle deg), SUB (the shooter's
// contact sub-steps, 8); dump and hold: LAYERS (how many layers it's filled with: dump 7, hold 10),
// STOW=s (dump: their RetractIntake after s seconds), INTAKE (dump: the intake's rollers running),
// TIMELINE, COLUMN (every half second: shots and speeds; the FUEL in the rotor's column)
import { RAPIER } from '../js/physics.js';
import { FUEL, GROUP, groups, HUB } from '../js/constants.js';
import { Mechanism } from '../js/mech/mechanism.js';
import { sim2D } from '../js/ballistics.js';
import {
  build604, wheels604, shooterWheels, pathSurfaces604, placeIntake, placeHopper, placeRotor, placeRotorSurface, placeShooter, placeTurretSurfaces,
  hoodElevation, launchFor, radFor, velocityAt, rollerRad, VELOCITY_MULTIPLIER, LAUNCH, INTAKE, ROTOR, SHOOTER, SERIALIZER, AXIS,
} from '../js/mech/r604.js';
import PATH from '../js/mech/path604.js';
import { profile, velocityDuty, EXIT_PER_MAIN_SURFACE } from '../js/mech/phys604.js';

await RAPIER.init();
const R = FUEL.radius, DT = 1 / 120, SEEDS = Number(process.env.SEEDS ?? 6), SPEED = Number(process.env.SPEED ?? 1);
const HELD = groups(GROUP.HELD, 0xffff & ~GROUP.HELD);
const DEG = Math.PI / 180, TAU = 2 * Math.PI;

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

// the robot's mechanisms, fixed in the world: the arms at their deg, the turret at yaw (rad; 0
// facing forward), the hood at their angle
function rig({ arm = INTAKE.deploy, yaw = 0, hood = SHOOTER.hoodMin } = {}) {
  const t = build604(), wh = wheels604(t), s = pathSurfaces604(PATH, t, process.env.MU ? { mu: Number(process.env.MU) } : undefined), sw = shooterWheels(t);
  const pathW = [...wh.intake, ...wh.rotor];
  const path = new Mechanism({ substeps: 6, shafts: [...new Set(pathW.map((x) => x.shaft)), t.pivot, t.rotor], wheels: pathW, surfaces: Object.values(s) });
  const shooter = new Mechanism({ substeps: 6, sub: Number(process.env.SUB ?? 8), shafts: [t.main, t.hoodRoller], wheels: sw });
  for (const tr of [t.intake, t.roller, t.launcher]) tr.cmd = null;
  t.rotor.cmd = null;
  const r = { t, wh, s, sw, path, shooter, arm, yaw, hood, phi: 0, buf: [] };
  r.pose = () => {
    s.intake.setPose([0.3048, 0.2064, 0], 0, (r.arm * Math.PI) / 180);
    placeIntake(wh.intake, r.arm);
    placeHopper(s.hopper, r.arm);
    placeRotorSurface(s.rotor, r.phi);
    placeRotor(wh.rotor, r.phi);
    placeTurretSurfaces(s, r.yaw, r.hood);
    placeShooter(sw, r.yaw, r.hood);
  };
  // the arm held where it is (their position loop holds it; what a FUEL does to it is in play)
  // (or swung toward armTo at armV rad/s: their RetractIntake's slow Motion Magic)
  r.holdArm = () => {
    t.pivot.cmd = 0; t.pivot._omega = 0;
    if (r.armTo === undefined || Math.abs(r.armTo - r.arm) < 0.01) return;
    const w = Math.sign(r.armTo - r.arm) * (r.armV ?? INTAKE.mmSlow);
    t.pivot._omega = w;
    r.arm += (w * DT * 180) / Math.PI;
    if ((w > 0 && r.arm > r.armTo) || (w < 0 && r.arm < r.armTo)) r.arm = r.armTo;
  };
  r.step = (balls, together = balls) => {
    r.pose();
    r.holdArm();
    path.step(DT, balls, together);
    shooter.step(DT, balls.filter((b) => b.translation().y > 0.5));
    r.phi = (r.phi + t.rotor.omega * DT) % TAU;
  };
  r.pose();
  return r;
}
// their serializer: the rollers' velocity loop at a FUEL speed (m/s), the rotor following the
// rollers' measured speed (10 samples) / 1.2 at its effective radius
function serializer(r, ballV) {
  const t = r.t, Z = SERIALIZER;
  if (!ballV) { t.roller.cmd = null; t.rotor.cmd = null; r.buf = []; return; }
  t.roller.cmd = velocityDuty(t.roller.motor, t.roller.omega, rollerRad(ballV) / t.infeed.k, Z.kV, Z.kP);
  r.buf.push(t.infeed.omega * 0.5 * Z.rollerR);
  if (r.buf.length > 10) r.buf.shift();
  const avg = r.buf.reduce((a, b) => a + b, 0) / r.buf.length;
  t.rotor.cmd = velocityDuty(t.rotor.motor, t.rotor.omega, Math.max(0, avg / Z.rotorPerRoller / ROTOR.effR), Z.kV, Z.kP);
}
// their launcher's velocity loop toward rad (main roller, rad/s)
function launcher(t, rad) { t.launcher.cmd = rad > 0 ? velocityDuty(t.launcher.motor, Math.abs(t.main.omega), rad, SHOOTER.kV, SHOOTER.kP) : null; }
// a FUEL leaving the turret: where it comes down through the HUB's rim height, out from the axis
// along the shot
function landing(o) {
  const hit = sim2D(o.y, o.v, (o.el * Math.PI) / 180, HUB.rimFront);
  return hit ? hit.d + o.x : NaN;
}
// a FUEL that touched the launcher's rollers and has left them going up and out: shot (turret
// facing forward: x out along the shot)
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
          if (v.y > 2 && v.x > 0.5) { shots.push({ t, v: Math.hypot(v.x, v.y, v.z), el: (Math.atan2(v.y, Math.hypot(v.x, v.z)) * 180) / Math.PI, side: (Math.atan2(v.z, v.x) * 180) / Math.PI, x: p.x - AXIS[0], y: p.y, b }); b.setTranslation({ x: 50 + shots.length, y: 5, z: 0 }, true); b.setGravityScale(0, true); b.setLinvel({ x: 0, y: 0, z: 0 }, true); }
        }
      }
    },
  };
}
const inRobot = (b) => { const p = b.translation(); return Math.abs(p.x) < 0.343 + 0.3 && Math.abs(p.z) < 0.343 && p.y > 0.14 && p.y < 0.8; };
// fill it a layer at a time (FUEL set into the space over the ones already in, let settle)
function fill(w, balls, step, layers = 6, y = 0.62, x1 = 0.6) {
  for (let layer = 0; layer < layers; layer++) {
    for (let i = 0; i < 7; i++) for (let k = 0; k < 4; k++) {
      const x = -0.27 + 0.155 * i, z = -0.233 + 0.155 * k;
      if (x > x1) continue;
      if (Math.hypot(x - AXIS[0], z) < 0.24 && y > 0.45) continue; // (not into the turret)
      if (balls.some((b) => { const p = b.translation(); return Math.hypot(p.x - x, p.y - y, p.z - z) < 2 * R; })) continue;
      balls.push(ball(w, x, y, z));
    }
    step(1.2 / DT);
  }
}

const tests = {
  rollers() {
    const r = rig(), t = r.t;
    t.intake.cmd = 1;
    for (let i = 0; i < 2 / DT; i++) { serializer(r, SERIALIZER.ballV); launcher(t, radFor(8)); r.step([]); }
    const surf = (sh, rr) => Math.abs(sh.omega) * rr;
    console.log(`604's rollers, free (intake 100%, the serializer at their 3.6 m/s, the launcher at their 8 m/s):`);
    console.log(`  intake X60s ${RPM(t.intake.omega).toFixed(0)} rpm: rollers ${surf(t.front, INTAKE.r).toFixed(2)} m/s at their surface`);
    console.log(`  roller X60 ${RPM(t.roller.omega).toFixed(0)} rpm (their target ${RPM(rollerRad(3.6) * 1.6).toFixed(0)}): infeed omnis ${surf(t.infeed, ROTOR.omni.r).toFixed(2)} m/s, up-feed flex wheel ${surf(t.upfeed, ROTOR.upfeed.flex.r).toFixed(2)} m/s`);
    console.log(`  rotor ${RPM(t.rotor.omega).toFixed(0)} rpm (${t.rotor.omega.toFixed(1)} rad/s; ${(t.rotor.omega * ROTOR.effR).toFixed(2)} m/s at their 7.8in; their seconds per ball ${(0.15 / (ROTOR.effR * t.rotor.omega)).toFixed(3)})`);
    console.log(`  launcher X60s ${RPM(t.launcher.omega).toFixed(0)} rpm: main roller ${Math.abs(t.main.omega).toFixed(0)} rad/s (their ${radFor(8).toFixed(0)}), ${surf(t.main, SHOOTER.main.r).toFixed(2)} m/s; hood roller ${surf(t.hoodRoller, SHOOTER.hoodRoller.r).toFixed(2)} m/s`);
    for (const v of [5.6, 7.0, 8.5, 9.9]) {
      const r2 = rig(), rad = radFor(v);
      let T = 0;
      for (; T < 6; T += DT) { launcher(r2.t, rad); r2.shooter.step(DT, []); if (Math.abs(velocityAt(Math.abs(r2.t.main.omega)) - velocityAt(rad)) < SHOOTER.tolV) break; }
      console.log(`  the launcher from rest to within their 0.8 m/s of ${v} m/s (${rad.toFixed(0)} rad/s): ${T.toFixed(2)} s`);
    }
    // the arms: their Motion Magic and position loop, from boot (98) down to -5, up to 10, to 80
    const r3 = rig({ arm: INTAKE.start });
    let set = [INTAKE.start * DEG, 0];
    const move = (goal, slow) => {
      let k = 0;
      for (; k < 4 / DT; k++) {
        set = profile(...set, goal * DEG, slow ? INTAKE.mmSlow : INTAKE.mmV, INTAKE.mmA, DT);
        const m = r3.t.pivot.motor, rot = (rad) => (rad * m.ratio) / TAU;
        r3.t.pivot.cmd = Math.max(-1, Math.min(1, (INTAKE.kP * (rot(set[0]) - rot(r3.arm * DEG)) + INTAKE.kV * rot(set[1])) / 12));
        r3.t.pivot.impulse(-INTAKE.mass * 9.81 * INTAKE.com * Math.cos(r3.arm * DEG - 0.19) * DT);
        r3.path.step(DT, []);
        r3.arm += (r3.t.pivot.omega * DT) / DEG;
        if (r3.arm < INTAKE.min - 1) { r3.arm = INTAKE.min - 1; if (r3.t.pivot._omega < 0) r3.t.pivot._omega = 0; }
        if (r3.arm > INTAKE.max + 1) { r3.arm = INTAKE.max + 1; if (r3.t.pivot._omega > 0) r3.t.pivot._omega = 0; }
        if (Math.abs(r3.arm - goal) < 2 && Math.abs(r3.t.pivot.omega) < 0.05) break;
      }
      return k * DT;
    };
    // (4 s: never within 2 degrees, held off by its weight against their kP alone)
    const down = move(INTAKE.deploy), downAt = r3.arm, up = move(INTAKE.standby), upAt = r3.arm, stow = move(INTAKE.stow, true);
    console.log(`  the arms down from boot (98) to -5 in ${down.toFixed(2)} s (at ${downAt.toFixed(1)}), up to standby (10) in ${up.toFixed(2)} s (at ${upAt.toFixed(1)}), stowed (80, slow) in ${stow.toFixed(2)} s (at ${r3.arm.toFixed(1)})`);
  },
  // FUEL pushed up the tube into the turret at their table's rows
  shooter() {
    const rows = process.env.SHOT ? [['custom', 0, ...process.env.SHOT.split(',').map(Number)]] : [1.5, 2.0, 2.5, 3.0, 4.0, 5.0, 6.0, 7.0, 8.0].map((d) => ['hub', d, launchFor(d).v, launchFor(d).angle]);
    console.log(`604's launcher, FUEL pushed up the tube at their 3.6 m/s, their launch table's rows (the main roller at their lookup's speed, x ${VELOCITY_MULTIPLIER}, the hood at 90 less the angle):`);
    const ratios = [];
    for (const [kind, d, v, ang] of rows) {
      const hood = 90 - ang, rad = radFor(v);
      const r = rig({ hood });
      const w = world({ carpet: false });
      for (let i = 0; i < 3 / DT; i++) { launcher(r.t, rad); r.shooter.step(DT, []); }
      const balls = [];
      const watch = shotWatch(r.shooter);
      for (let i = 0; i < 3 / DT && watch.shots.length < 4; i++) {
        if (i % Math.round(0.4 / DT) === 0 && balls.length < 4) balls.push(ball(w, AXIS[0], 0.46, 0, [0, 3.6, 0]));
        launcher(r.t, rad);
        const live = balls.filter((b) => b.translation().y < 3);
        r.step(live); w.step();
        watch.check(live, i * DT);
      }
      w.free();
      const outs = watch.shots;
      if (!outs.length) { console.log(`  ${kind} ${d} m (${v.toFixed(2)} m/s, ${ang.toFixed(1)} deg): nothing came out`); continue; }
      const o = { v: med(outs.map((q) => q.v)), el: med(outs.map((q) => q.el)), side: med(outs.map((q) => q.side)), x: med(outs.map((q) => q.x)), y: med(outs.map((q) => q.y)) };
      const surf = Math.abs(r.t.main.omega) * SHOOTER.main.r;
      ratios.push(o.v / surf);
      const land = landing(o);
      console.log(`  ${kind} ${d} m (their ${v.toFixed(2)} m/s at ${ang.toFixed(1)} deg; ${rad.toFixed(0)} rad/s, hood ${hood.toFixed(1)}): ${outs.length}/4 out at ${o.v.toFixed(2)} m/s (${(o.v / surf).toFixed(2)} of the main roller's ${surf.toFixed(1)}), ${o.el.toFixed(1)} deg (${hoodElevation(hood).toFixed(1)} by the CAD), ${o.side.toFixed(1)} deg aside, from ${o.x.toFixed(2)},${o.y.toFixed(2)}; ` +
        `comes down at the rim ${land.toFixed(2)} m out (${land - d > 0 ? '+' : ''}${(land - d).toFixed(2)})`);
    }
    if (ratios.length) console.log(`  exit over the main roller's surface: median ${med(ratios).toFixed(2)} (the estimate ${EXIT_PER_MAIN_SURFACE})`);
  },
  // carpet to shot: FUEL in front, the robot driving into them; intaking and shooting at 3 m
  path() {
    const runs = [];
    for (let seed = 1; seed <= SEEDS; seed++) runs.push(pathRun(seed));
    console.log(`604 carpet to shot (20 FUEL, four abreast, the robot at ${SPEED} m/s, the intake at 100%, the serializer at their 3.6 m/s, a 3 m shot): a median ${med(runs.map((r) => r.shots))} shot (${Math.min(...runs.map((r) => r.shots))}..${Math.max(...runs.map((r) => r.shots))}), ` +
      `${med(runs.map((r) => r.rate)).toFixed(1)} FUEL/s once they're coming; ${med(runs.map((r) => r.inside))} in it at the end; intake X60s down to ${med(runs.map((r) => r.minIntake)).toFixed(0)}%`);
    if (process.env.STUCK) for (const r of runs) console.log('   left', r.left.map((p) => p.map((v) => v.toFixed(2)).join(',')).join(' | '));
  },
  // intaking alone (rollers 100%, the serializer off, as their code does when it isn't launching)
  intake() {
    for (let seed = 1; seed <= Math.min(SEEDS, 3); seed++) {
      const r = pathRun(seed, { shoot: false, n: 24, sec: 6 });
      console.log(`604 intaking only (rollers 100%, the serializer off), 24 FUEL at ${SPEED} m/s: ${r.inside} in it after 6 s (in past the bumper each second: ${r.timeline.join(', ')}); intake X60s down to ${r.minIntake.toFixed(0)}%`);
      if (process.env.STUCK) console.log('   left', r.left.map((p) => p.map((v) => v.toFixed(2)).join(',')).join(' | '));
    }
  },
  hold() {
    const r = rig();
    const w = world();
    const balls = [];
    const step = (n) => { for (let i = 0; i < n; i++) { r.step(balls); w.step(); } };
    fill(w, balls, step, Number(process.env.LAYERS ?? 10));
    step(2 / DT);
    const t0 = performance.now();
    step(1 / DT);
    const ms = (performance.now() - t0) / (1 / DT);
    console.log(`  (its mechanisms with them all in: ${ms.toFixed(2)} ms a step)`);
    console.log(`604 holds ${balls.filter(inRobot).length} with everything off and the intake down (the box out; a layer at a time)`);
    if (process.env.STUCK) for (const b of balls.filter((q) => !inRobot(q))) { const p = b.translation(); console.log(`   out at ${p.x.toFixed(2)},${p.y.toFixed(2)},${p.z.toFixed(2)}`); }
  },
  // a full one fired at 3 m (the turret facing back, as it would at the HUB behind it): the
  // serializer at their 3.6 m/s
  dump() {
    const row = launchFor(3.0), hood = 90 - row.angle, rad = radFor(row.v);
    const r = rig({ hood, yaw: Math.PI });
    const w = world();
    const balls = [];
    const step = (n, run) => { for (let i = 0; i < n; i++) { if (run) run(); const live = balls.filter((b) => b.translation().y < 3); r.step(live); w.step(); } };
    fill(w, balls, (n) => step(n, () => launcher(r.t, rad)), Number(process.env.LAYERS ?? 7));
    const n0 = balls.filter(inRobot).length;
    const watch = {
      touched: new Set(), shots: [],
      check(t) {
        for (const b of balls) {
          if (r.shooter.contacts.some((c) => c.ball === b)) this.touched.add(b);
          else if (this.touched.has(b)) {
            this.touched.delete(b);
            const v = b.linvel(), p = b.translation();
            if (v.y > 2) { this.shots.push({ t, v: Math.hypot(v.x, v.y, v.z), el: (Math.atan2(v.y, Math.hypot(v.x, v.z)) * 180) / Math.PI }); b.setTranslation({ x: 50 + this.shots.length, y: 5, z: 0 }, true); b.setGravityScale(0, true); b.setLinvel({ x: 0, y: 0, z: 0 }, true); }
          }
        }
      },
    };
    let minRad = Infinity, jams = 0;
    for (let k = 0; k < 8 / DT; k++) {
      step(1, () => { launcher(r.t, rad); serializer(r, SERIALIZER.ballV); });
      watch.check(k * DT);
      if (watch.shots.length) minRad = Math.min(minRad, Math.abs(r.t.main.omega));
      if (Math.abs(r.t.rotor.motor.current) > 40 && r.t.rotor.omega < 2) jams++;
      if (process.env.INTAKE) r.t.intake.cmd = 1;
      // (STOW=s: their RetractIntake after s seconds, the hopper sliding in on the arm)
      if (process.env.STOW && k * DT >= Number(process.env.STOW)) r.armTo = INTAKE.stow;
      // (COLUMN=1: every half second, the FUEL in the rotor's column and what they touch)
      if (process.env.COLUMN && k % 60 === 30) for (const b of balls) { const p = b.translation(); if (p.y < 3 && Math.hypot(p.x - AXIS[0], p.z - AXIS[1]) < 0.16 && p.y > 0.2) console.log(`     ${p.x.toFixed(3)},${p.y.toFixed(3)},${p.z.toFixed(3)} v${b.linvel().y.toFixed(2)}: ${[...r.path.contacts, ...r.shooter.contacts].filter((c) => c.ball === b).map((c) => c.wheel.name + ' ' + c.force.toFixed(0)).join(', ')}`); }
      // (TIMELINE=1: every half second, the shots so far, the rotor's and rollers' speeds and currents)
      if (process.env.TIMELINE && k % 60 === 59) console.log(`   ${((k + 1) * DT).toFixed(1)} s: ${watch.shots.length} shot; rotor ${r.t.rotor.omega.toFixed(1)} rad/s ${r.t.rotor.motor.current.toFixed(0)} A, rollers ${r.t.infeed.omega.toFixed(0)} rad/s ${r.t.roller.motor.current.toFixed(0)} A, main ${Math.abs(r.t.main.omega).toFixed(0)}`);
    }
    const s = watch.shots.map((x) => x.t), first = s[0] ?? 0, in1 = s.filter((x) => x - first < 1).length;
    const left = balls.filter((b) => b.translation().y < 3 && inRobot(b));
    if (process.env.STUCK) for (const b of left) { const p = b.translation(); console.log(`   left at ${p.x.toFixed(2)},${p.y.toFixed(2)},${p.z.toFixed(2)}: ${[...r.path.contacts, ...r.shooter.contacts].filter((c) => c.ball === b).map((c) => c.wheel.name + ' ' + c.force.toFixed(0)).join(', ')}`); }
    const sh = watch.shots;
    if (sh.length) console.log(`  (shots leave at ${med(sh.map((o) => o.v)).toFixed(2)} m/s (${Math.min(...sh.map((o) => o.v)).toFixed(2)}..${Math.max(...sh.map((o) => o.v)).toFixed(2)}), ${med(sh.map((o) => o.el)).toFixed(1)} deg; the main roller down to ${minRad.toFixed(0)} rad/s of ${rad.toFixed(0)}; the rotor stalled ${(jams * DT).toFixed(2)} s; ${left.length} still in it)`);
    const span = s.length > 1 ? s[s.length - 1] - first : 0;
    // (and its best second: as many shot in any one second, the load still coming)
    const best = s.reduce((m, x, i) => Math.max(m, s.slice(i).filter((y) => y < x + 1).length), 0);
    console.log(`604 dumps ${s.length} of ${n0}: ${in1} in the first second, ${best} in its best, ${span ? ((s.length - 1) / span).toFixed(1) : 0} FUEL/s over the whole dump`);
  },
};

function pathRun(seed, { shoot = true, n = 20, sec = 7 } = {}) {
  let sd = seed;
  const rnd = () => ((sd = (sd * 16807) % 2147483647) / 2147483647 - 0.5);
  const row = launchFor(3.0), hood = 90 - row.angle, rad = radFor(row.v);
  const r = rig({ hood, yaw: Math.PI });
  const w = world({ carpet: false });
  // the carpet, running under the robot
  const carpet = w.createRigidBody(RAPIER.RigidBodyDesc.kinematicVelocityBased().setLinvel(-SPEED, 0, 0));
  w.createCollider(RAPIER.ColliderDesc.cuboid(15, 0.01, 1).setTranslation(0.45 + 15, -0.01, 0).setFriction(0.3).setRestitution(0.2), carpet);
  w.createCollider(RAPIER.ColliderDesc.cuboid(0.45, 0.01, 1).setTranslation(0, -0.01, 0).setFriction(0.3), w.createRigidBody(RAPIER.RigidBodyDesc.fixed()));
  r.t.intake.cmd = 1;
  for (let i = 0; i < 2 / DT; i++) { if (shoot) launcher(r.t, rad); r.step([]); }
  const free = Math.abs(r.t.intake.omega) || 1;
  const balls = [];
  for (let i = 0; i < n; i++) balls.push(ball(w, 0.9 + 0.17 * Math.floor(i / 4) + 0.02 * rnd(), R + 0.003, ((i % 4) - 1.5) * 0.16 + 0.02 * rnd(), [-SPEED, 0, 0]));
  const watch = shotWatch(r.shooter);
  const shots = [];
  let minIntake = 1;
  const timeline = [];
  for (let k = 0; k < sec / DT; k++) {
    const live = balls.filter((b) => b.translation().y < 3);
    if (shoot) { launcher(r.t, rad); serializer(r, SERIALIZER.ballV); }
    r.step(live); w.step();
    minIntake = Math.min(minIntake, Math.abs(r.t.intake.omega) / free);
    // (shot: off the launcher's rollers going up)
    for (const b of live) {
      if (r.shooter.contacts.some((c) => c.ball === b)) b.touched = true;
      else if (b.touched) { b.touched = false; const v = b.linvel(); if (v.y > 2) { shots.push(k * DT); b.setTranslation({ x: 50 + shots.length, y: 5, z: 0 }, true); b.setGravityScale(0, true); b.setLinvel({ x: 0, y: 0, z: 0 }, true); } }
    }
    if (k % Math.round(1 / DT) === Math.round(1 / DT) - 1) timeline.push(balls.filter((b) => { const p = b.translation(); return p.y < 3 && p.x < 0.343 && p.y > 0.14; }).length);
    if (process.env.TRACE && k % 6 === 0) {
      const b = balls[Number(process.env.TRACE)], p = b.translation(), v = b.linvel();
      console.log(`   ${(k * DT).toFixed(2)} s at ${p.x.toFixed(3)},${p.y.toFixed(3)},${p.z.toFixed(3)} v ${v.x.toFixed(2)},${v.y.toFixed(2)},${v.z.toFixed(2)}: ${[...r.path.contacts, ...r.shooter.contacts].filter((c) => c.ball === b).map((c) => c.wheel.name + ' ' + c.force.toFixed(0)).join(', ')}`);
    }
  }
  const inside = balls.filter((b) => inRobot(b) && b.translation().y < 3);
  const left = balls.filter((b) => b.translation().y < 3 && b.translation().x < 40 && !inside.includes(b)).map((b) => { const p = b.translation(); return [p.x, p.y, p.z]; });
  w.free();
  return { timeline, shots: shots.length, rate: shots.length > 2 ? (shots.length - 1) / (shots[shots.length - 1] - shots[0]) : 0, inside: inside.length, minIntake: 100 * minIntake, left };
}

const pick = process.argv.slice(2);
for (const [name, fn] of Object.entries(tests)) if (!pick.length || pick.includes(name)) fn();
