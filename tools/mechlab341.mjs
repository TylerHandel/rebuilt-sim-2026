// 341's mechanisms (js/mech/r341.js) on their own, no field, no robot: the CAD's ball path with
// its rollers, measured against what the team's code expects. Run:
//   node --import ./tools/node-env.mjs tools/mechlab341.mjs [test ...]
// Tests: geometry (where the arm's rollers are, down), rollers (each roller's free speed, the
// flywheel's spin-up), shooter (FUEL fed up the uptake at their tables' RPM and hood angles: how
// fast and how steep they leave, and whether that reaches the HUB from the table's distance),
// tune (the preload fed as in play at each row of their tables: the flywheel RPM at which the
// HUB's come down over its center and the ferry's at the row's distance: js/mech/r341.js
// SIM_SHOTS; MAXRPM, SETTLED, TUNELOG), path (FUEL on a treadmill carpet, the robot driving into them at 1 m/s, shooting at 3 m: intake,
// serializer, uptake, shooter; how many and how fast), rate (the intake on its own: FUEL taken off
// the serializer's floor as soon as they're past the pivot's shaft, 30 of them three or four
// abreast; how fast it takes them in, against their binder's 10 FUEL/s), intake (intaking, not
// shooting: the uptake pushes FUEL on up through the idle shooter, so it's no measure of what it
// holds), hold (how many it holds, everything off).
// Env: SEEDS (6), SPEED (1 m/s), ARM (their angle down, 149), ABREAST (rate: 3 or 4), TRACE (a FUEL's index to follow),
// STUCK (where the ones left over are), LEAK (how ones get out other than shot), EXIT (the exit/flywheel-surface ratio to report against),
// LETGO (path: the AI driver letting go of the trigger when nothing comes out, as in play),
// SEED0 (the first seed, 1), FEEDV (path: the feeder's volts; js/mech/phys341.js FEEDER_V, their 3),
// FEED_BACKOFF (path, as in play: the feeder backed off when nothing comes out; 0: off),
// JAMLOG (path: the first time nothing's come out for 1 s, each FUEL in it, what presses it and the
// FUEL it's pressed into). The path's summary: runs locked (12 or fewer of the 18 shot), runs that
// ended with FUEL still in it, and where the shots come down through the HUB's rim (3 m away).
// js/mech/r341.js's switches apply too: OLD_BUMPER=1, GUARDS=none
// THEIR_SHOTS=1: their tables' own RPMs instead of the re-found ones (js/mech/r341.js)
import { RAPIER } from '../js/physics.js';
import { FUEL, GROUP, groups, HUB } from '../js/constants.js';
import { Mechanism } from '../js/mech/mechanism.js';
import { sim2D } from '../js/ballistics.js';
import {
  build341, wheels341, shooterWheels, pathSurfaces341, placeIntake, intakeRot, placeShooter, placeTurretSurfaces, hoodElevation,
  shotFor, REGRESSIONS, INTAKE, SHOOTER, OMNI, PIVOT, TURRET, HALF, PRELOAD,
} from '../js/mech/r341.js';
import PATH from '../js/mech/path341.js';
import { EXIT_PER_FLY_SURFACE, LET_GO, FEEDER_V, FEED_BACKOFF } from '../js/mech/phys341.js';

await RAPIER.init();
const R = FUEL.radius, DT = 1 / 120, SEEDS = Number(process.env.SEEDS ?? 6), SPEED = Number(process.env.SPEED ?? 1);
const HELD = groups(GROUP.HELD, 0xffff & ~GROUP.HELD);
const FEEDV = Number(process.env.FEEDV ?? FEEDER_V); // (path: the feeder's volts, as in play: js/mech/phys341.js FEEDER_V)
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
  // their tables' rows re-found by shooting: at each row's hood angle (their hood table at the
  // row's distance; the ferry's 28), the flywheel RPM whose FUEL, the preload's eight fed as play
  // feeds them (the flywheel from rest to within their 130 RPM, then the intake's rollers, top
  // rollers and uptake at 12 V and the feeder at 3 V while it stays within it, debounced 0.1 s as
  // theirs), come down at the row's distance from the turret: the HUB's through the rim's height,
  // the ferry's on the carpet (a search between 1200 RPM and MAXRPM, 5500: the flywheel's 5700
  // free, less room for its loop to catch up between FUEL); prints what
  // js/mech/r341.js keeps as SIM_SHOTS. SETTLED=1: 3 s at the row's speed first; TUNELOG: each shot
  tune() {
    const MAX = Number(process.env.MAXRPM ?? 5500), SETTLED = !!process.env.SETTLED;
    const shoot = (kind, rpm, hood) => {
      const { t, path, shooter } = rig({ hood });
      const w = world();
      const now = () => RPM(t.shooter.omega), within = () => Math.abs(now() - rpm) <= SHOOTER.tol;
      if (SETTLED) for (let i = 0; i < 3 / DT; i++) { fly(t, rpm); shooter.step(DT, []); }
      else for (let i = 0; i < 6 / DT && !within(); i++) { fly(t, rpm); shooter.step(DT, []); }
      const balls = PRELOAD.map(([x, y, z]) => ball(w, x, y, z));
      const watch = shotWatch(shooter);
      let atSpeedT = 0.1;
      // (until all eight are out, or nothing's come out for a second: the zipper locked, as it can
      // in play, where the AI's driver lets go and presses again; the ones shot count)
      const quiet = (i) => i * DT - (watch.shots.length ? watch.shots[watch.shots.length - 1].t : 0) > 1;
      for (let i = 0; i < 6 / DT && watch.shots.length < balls.length && !quiet(i); i++) {
        const live = balls.filter((b) => b.translation().y < 3);
        fly(t, rpm);
        atSpeedT = within() ? 0.1 : Math.max(0, atSpeedT - DT);
        const on = atSpeedT > 0;
        t.intake.cmd = on ? 1 : 0; t.serializer.cmd = on ? 1 : 0; t.feederT.cmd = on ? 3 / 12 : 0;
        path.step(DT, live, live); shooter.step(DT, live); w.step();
        watch.check(live, i * DT);
      }
      w.free();
      const H = kind === 'hub' ? HUB.rimFront : R;
      const L = watch.shots.map((o) => { const h = sim2D(o.y, o.v, (o.el * Math.PI) / 180, H); return h ? h.d + o.x - TURRET[0] : NaN; }).filter(Number.isFinite);
      if (process.env.TUNELOG) console.log(`      ${rpm.toFixed(0)}: ${watch.shots.map((o, k) => `${o.t.toFixed(2)}s ${o.v.toFixed(2)}@${o.el.toFixed(0)} ${(L[k] ?? NaN).toFixed(2)}`).join(' | ')}`);
      return { land: L.length ? L.reduce((a, b) => a + b, 0) / L.length : NaN, n: L.length, spread: L.length ? Math.max(...L) - Math.min(...L) : NaN };
    };
    const lerp = (t, x) => { if (x <= t[0][0]) return t[0][1]; for (let k = 1; k < t.length; k++) if (x <= t[k][0]) return t[k - 1][1] + ((t[k][1] - t[k - 1][1]) * (x - t[k - 1][0])) / (t[k][0] - t[k - 1][0]); return t[t.length - 1][1]; };
    // (and two rows of the sim's own for ferrying: at 4 m, short of their first (5.5 m), where the
    // sim's pass targets can be; at 9 m, where the FUEL's slip on the flywheel bends the curve
    // between their 7.5 and 10.5 m rows)
    const extra = { rpm: [], feedRpm: [4.0, 9.0] }, out = { rpm: [], feedRpm: [] };
    for (const [key, kind] of [['rpm', 'hub'], ['feedRpm', 'ferry']]) {
      const rows = [...REGRESSIONS[key].map(([d, r]) => [d, r, true]), ...extra[key].map((d) => [d, Math.round(lerp(REGRESSIONS[key], d)), false])].sort((p, q) => p[0] - q[0]);
      for (const [d, rpm0, own] of rows) {
        const hood = shotFor(kind, d, { theirs: true }).hood;
        const theirs = shoot(kind, rpm0, hood);
        // (bracketed, then regula falsi, as tools/mechlab1678.mjs tune)
        let lo = 1200, flo = shoot(kind, lo, hood).land, hi = MAX, fhi = shoot(kind, hi, hood).land, b = hi, lb = fhi;
        if (!(flo < d)) flo = 0;
        for (let j = 0; j < 14 && fhi > d && Math.abs(lb - d) > 0.03 && hi - lo > 5; j++) {
          b = lo + ((d - flo) * (hi - lo)) / (fhi - flo);
          b = Math.max(lo + 0.1 * (hi - lo), Math.min(hi - 0.1 * (hi - lo), b));
          lb = shoot(kind, b, hood).land;
          if (process.env.TUNELOG) console.log(`    ${b.toFixed(0)} RPM: ${lb.toFixed(2)} m`);
          if (!(lb >= d)) { lo = b; flo = Number.isFinite(lb) ? lb : 0; } else { hi = b; fhi = lb; }
        }
        const ok = Number.isFinite(lb) && Math.abs(lb - d) < 0.1;
        // (out of reach: a ferry's flat out; the HUB's 14 m row, which no HUB shot comes from (the
        // ALLIANCE ZONE's far corner is ~6 m from it) and only sets the table's slope past 5.5 m,
        // their RPM times the last row's factor, so that slope stays theirs)
        const last = out[key][out[key].length - 1], prev = last && lerp(REGRESSIONS[key], last[0]);
        const best = ok ? Math.round(b / 5) * 5 : kind === 'hub' && prev ? Math.round((rpm0 * last[1]) / prev / 5) * 5 : MAX, at = ok ? lb : fhi;
        const mine = ok ? shoot(kind, best, hood) : null;
        out[key].push([d, best]);
        console.log(`  ${kind} ${d} m, hood ${hood.toFixed(1)}: ${own ? 'their' : '(the sim\'s own row; their table gives)'} ${rpm0} RPM: ${theirs.n}/8 land ${theirs.land.toFixed(2)} m (${theirs.land - d > 0 ? '+' : ''}${(theirs.land - d).toFixed(2)}); ` +
          (ok ? `the sim's FUEL come down there at ${best} RPM (x${(best / rpm0).toFixed(3)}): ${mine.n}/8 land ${mine.land.toFixed(2)} m (${mine.land - d > 0 ? '+' : ''}${(mine.land - d).toFixed(2)}, spread ${mine.spread.toFixed(2)})` : `out of reach: flat out (${MAX}) lands ${at.toFixed(2)} m (${(at - d).toFixed(2)}); ${best} kept`));
      }
    }
    console.log('SIM_SHOTS', JSON.stringify(out));
    // and between the rows, at the interpolated RPM: their table's, and the re-found one's
    for (const [key, kind, ds] of [['rpm', 'hub', [2.0, 2.7, 3.3, 3.9, 4.9, 6.0]], ['feedRpm', 'ferry', [4.5, 6.5, 8.3, 9.8, 12.0]]]) {
      for (const d of ds) {
        const hood = shotFor(kind, d, { theirs: true }).hood, r0 = lerp(REGRESSIONS[key], d), r1 = lerp(out[key], d);
        const a = shoot(kind, r0, hood).land, b2 = shoot(kind, r1, hood).land, e = (x) => `${x - d > 0 ? '+' : ''}${(x - d).toFixed(2)}`;
        console.log(`  between: ${kind} ${d} m, hood ${hood.toFixed(1)}: their ${r0.toFixed(0)} RPM lands ${a.toFixed(2)} m (${e(a)}), the re-found ${r1.toFixed(0)} ${b2.toFixed(2)} m (${e(b2)})`);
      }
    }
  },
  path() {
    const runs = [];
    for (let seed = Number(process.env.SEED0 ?? 1); seed <= SEEDS; seed++) runs.push(pathRun(seed));
    console.log(`341 carpet to shot (18 FUEL, three abreast, the robot at ${SPEED} m/s, ${process.env.LETGO ? 'shooting, the AI driver letting go when nothing comes out' : 'everything on'}, shooting at 3 m): a median ${med(runs.map((r) => r.shots))} shot (${Math.min(...runs.map((r) => r.shots))}..${Math.max(...runs.map((r) => r.shots))}; ${runs.filter((r) => r.shots <= 12).length} of ${runs.length} runs locked up with 12 or fewer; ${runs.filter((r) => r.inside > 0).length} ended with FUEL still in it), ` +
      `${med(runs.map((r) => r.rate)).toFixed(1)} FUEL/s once they're coming; ${med(runs.map((r) => r.inside))} in it at the end; their shots come down through the rim ${(() => { const L = runs.flatMap((r) => r.lands); return `${(med(L.map((d) => d - 3)) >= 0 ? "+" : "") + med(L.map((d) => d - 3)).toFixed(2)} m from the 3 m (median), ${Math.round((100 * L.filter((d) => Math.abs(d - 3) < 0.45).length) / (L.length || 1))}% within 0.45 m`; })()}; in each second ${runs[0].timeline.join(', ')} (seed 1); intake X60s down to ${med(runs.map((r) => r.minIntake)).toFixed(0)}%, top rollers to ${med(runs.map((r) => r.minSer)).toFixed(0)}%`);
    if (process.env.STUCK) for (const r of runs) console.log(`   ${r.shots} shot; left`, r.left.map((p) => p.map((v) => v.toFixed(2)).join(',')).join(' | '));
  },
  // intake only (the rollers and top rollers on, nothing fed): how many it takes in
  intake() {
    for (let seed = 1; seed <= Math.min(SEEDS, 3); seed++) {
      const r = pathRun(seed, { shoot: false, n: 18, sec: 5 });
      console.log(`341 intaking only (rollers and top rollers 12 V, no feeder), 18 FUEL at ${SPEED} m/s: ${r.inside} in it after 5 s (in it each second: ${r.timeline.join(', ')}); intake X60s down to ${r.minIntake.toFixed(0)}%`);
      if (process.env.STUCK) console.log('   left', r.left.map((p) => p.map((v) => v.toFixed(2)).join(',')).join(' | '));
    }
  },
  rate() {
    const A = Number(process.env.ABREAST ?? 3), runs = [];
    for (let seed = 1; seed <= SEEDS; seed++) runs.push(pathRun(seed, { shoot: false, n: 30, sec: 4, drain: true, abreast: A }));
    console.log(`341's intake on its own (30 FUEL ${A} abreast, rows 0.17 m apart, at ${SPEED} m/s: ${((A * SPEED) / 0.17).toFixed(1)} FUEL/s coming): a median ${med(runs.map((r) => r.taken))} taken in (${Math.min(...runs.map((r) => r.taken))}..${Math.max(...runs.map((r) => r.taken))}), ` +
      `${med(runs.map((r) => r.inRate)).toFixed(1)} FUEL/s first to last; intake X60s down to ${med(runs.map((r) => r.minIntake)).toFixed(0)}%`);
    if (process.env.STUCK) for (const r of runs) console.log(`   ${r.taken} taken; left`, r.left.filter((p) => p[0] < 1).map((p) => p.map((v) => v.toFixed(2)).join(',')).join(' | '));
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

// the carpet's rolling resistance (js/fuel.js, as in play): a FUEL rolling on it slows, roll and
// spin together, relative to the carpet (here running under the robot at speed, from the bumper
// out). (Without it a FUEL the rollers kick back out can roll along ahead of the arm forever.)
function rollResist(balls, speed) {
  for (const b of balls) {
    const p = b.translation(), v = b.linvel();
    if (p.y > R + 0.012 || Math.abs(v.y) > 0.35) continue;
    const cx = p.x > 0.45 ? -speed : 0, rx = v.x - cx, hs = Math.hypot(rx, v.z), dec = FUEL.rollDecel * DT;
    const f = hs <= dec + 0.01 ? 0 : (hs - dec) / hs, a = b.angvel();
    b.setLinvel({ x: cx + rx * f, y: v.y, z: v.z * f }, true);
    b.setAngvel({ x: a.x * f, y: a.y * f, z: a.z * f }, true);
  }
}
function pathRun(seed, { shoot = true, n = 18, sec = 7, drain = false, abreast = 3 } = {}) {
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
  if (shoot) t.feederT.cmd = FEEDV / 12;
  const run = () => { if (shoot) fly(t, want.rpm); };
  for (let i = 0; i < 3 / DT; i++) { run(); shooter.step(DT, []); path.step(DT, []); }
  const free = { intake: Math.abs(t.intake.omega), ser: Math.abs(t.serializer.omega) || 1 };
  const balls = [];
  // (three abreast 0.18 m apart; four, 0.15: the intake's 24in)
  const gap = abreast === 3 ? 0.18 : 0.15, jit = abreast === 3 ? 0.02 : 0.004;
  for (let i = 0; i < n; i++) balls.push(ball(w, 0.85 + 0.17 * Math.floor(i / abreast) + 0.02 * rnd(), R + 0.003, (i % abreast - (abreast - 1) / 2) * gap + jit * rnd(), [-SPEED, 0, 0]));
  const taken = [];
  const watch = shotWatch(shooter);
  let minIntake = 1, minSer = 1;
  const timeline = [];
  let letGoUntil = -1, sinceShot = 0, jamLogged = false, backFrom = 0, backUntil = -1, backoffs = 0;
  for (let k = 0; k < sec / DT; k++) {
    const live = balls.filter((b) => b.translation().y < 3);
    run();
    // (LETGO=1: as in play, the AI's driver lets go of the trigger when nothing's come out for a
    // while with FUEL in it, and their code runs the top rollers and uptake backward at 3 V, the
    // intake's rollers and the feeder stopped: js/mech/phys341.js letGo)
    if (shoot && process.env.LETGO) {
      const now = k * DT;
      sinceShot = watch.shots.length && watch.shots[watch.shots.length - 1].t > now - DT * 1.5 ? 0 : sinceShot + DT;
      if (now < letGoUntil) { t.intake.cmd = 0; t.serializer.cmd = -3 / 12; t.feederT.cmd = 0; }
      else {
        t.intake.cmd = 1; t.serializer.cmd = 1; t.feederT.cmd = FEEDV / 12;
        if (sinceShot > LET_GO.after && now > 1 && live.some(inRobot)) { letGoUntil = now + LET_GO.for; sinceShot = -LET_GO.for; }
      }
    }
    // (as in play, the feeder backed off a moment when nothing's come out for a while with FUEL in
    // it: js/mech/phys341.js FEED_BACKOFF; FEED_BACKOFF=0: off)
    if (shoot && FEED_BACKOFF.for > 0 && !(k * DT < letGoUntil)) {
      const now = k * DT, lastShot = watch.shots.length ? watch.shots[watch.shots.length - 1].t : 0;
      backFrom = Math.max(backFrom, lastShot);
      if (now > 1 && now - backFrom > FEED_BACKOFF.after && live.some(inRobot)) { backUntil = now + FEED_BACKOFF.for; backFrom = backUntil; backoffs++; }
      t.feederT.cmd = now < backUntil ? -FEEDV / 12 : FEEDV / 12;
    }
    rollResist(live, SPEED);
    path.step(DT, live, live); shooter.step(DT, live); w.step();
    minIntake = Math.min(minIntake, Math.abs(t.intake.omega) / free.intake);
    minSer = Math.min(minSer, Math.abs(t.serializer.omega) / free.ser);
    watch.check(live, k * DT);
    // (JAMLOG=1: the first time nothing's come out for a second with FUEL in it, each FUEL in it,
    // what presses it over 10 N (and which way: the normal on it), and the FUEL it's pressed into)
    if (process.env.JAMLOG && shoot && !jamLogged && k * DT > 1.5 && k * DT - (watch.shots.length ? watch.shots[watch.shots.length - 1].t : 0) > 1) {
      const ins = live.filter(inRobot);
      if (ins.length) {
        jamLogged = true;
        console.log(`  seed ${seed}: nothing out for 1 s at ${(k * DT).toFixed(2)} s, ${ins.length} in it (serializer ${RPM(t.serializer.omega).toFixed(0)} rpm, feeder ${RPM(t.feederT.omega).toFixed(0)} rpm)`);
        for (const b of ins) {
          const p = b.translation();
          const cs = path.contacts.filter((c) => c.ball === b && c.force > 10).map((c) => `${c.wheel.name.replace(/ \(.*\)/, '')} ${c.force.toFixed(0)}N n(${c.n.map((v) => (-v).toFixed(1)).join(',')})`);
          const fu = ins.filter((q) => q !== b && Math.hypot(q.translation().x - p.x, q.translation().y - p.y, q.translation().z - p.z) < 2 * R - 0.002).map((q) => { const o = q.translation(); return `${o.x.toFixed(2)},${o.z.toFixed(2)} ${((2 * R - Math.hypot(o.x - p.x, o.y - p.y, o.z - p.z)) * 1000).toFixed(0)}mm`; });
          console.log(`    ${p.x.toFixed(3)},${p.y.toFixed(3)},${p.z.toFixed(3)}: ${cs.join('; ')}${fu.length ? ' | FUEL ' + fu.join(', ') : ''}`);
        }
      }
    }
    // (rate: FUEL past the pivot's shaft, onto the serializer's floor, taken away)
    if (drain) for (const b of live) {
      const p = b.translation();
      if (p.x < 0.2 && p.y > 0.17) { taken.push(k * DT); b.setTranslation({ x: 50 + taken.length, y: 5, z: 0 }, true); b.setLinvel({ x: 0, y: 0, z: 0 }, true); b.setBodyType(RAPIER.RigidBodyType.Fixed, true); }
    }
    // (LEAK=1: where and how FUEL get out of it other than by the shooter)
    if (process.env.LEAK) for (const [i, b] of balls.entries()) {
      const p = b.translation(), v = b.linvel(), last = (b.last ??= { out: false, c: '' });
      const c = [...path.contacts, ...shooter.contacts].filter((q) => q.ball === b).map((q) => q.wheel.name).join(', ');
      if (c) last.c = `${(k * DT).toFixed(2)} s at ${p.x.toFixed(2)},${p.y.toFixed(2)},${p.z.toFixed(2)}: ${c}`;
      if (!last.out && p.y < 3 && (Math.abs(p.z) > HALF + 0.12 || p.x < -HALF - 0.12 || p.y < -0.05 || (p.y < 0.12 && p.x < HALF - 0.02 && Math.abs(p.z) < HALF)) && last.c) { last.out = true; console.log(`   FUEL ${i} (seed ${seed}) out at ${p.x.toFixed(2)},${p.y.toFixed(2)},${p.z.toFixed(2)} v ${v.x.toFixed(1)},${v.y.toFixed(1)},${v.z.toFixed(1)}; last touched ${last.c}`); }
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
  if (process.env.STUCK) console.log(`   seed ${seed} inside`, inside.map((b) => { const p = b.translation(); return `#${balls.indexOf(b)} ${p.x.toFixed(2)},${p.y.toFixed(2)},${p.z.toFixed(2)} ${[...path.contacts].filter((c) => c.ball === b).map((c) => c.wheel.name.replace(/ \(.*\)/, '') + ' ' + c.force.toFixed(0)).join('+')}`; }).join(' | '));
  w.free();
  const lands = watch.shots.map((o) => landing(o) + (o.x - TURRET[0])).filter(Number.isFinite);
  return { lands, taken: taken.length, inRate: taken.length > 2 ? (taken.length - 1) / (taken[taken.length - 1] - taken[0]) : 0, timeline, shots: s.length, rate: s.length > 2 ? (s.length - 1) / (s[s.length - 1] - s[0]) : 0, inside: inside.length, minIntake: 100 * minIntake, minSer: 100 * minSer, left };
}

const pick = process.argv.slice(2);
for (const [name, fn] of Object.entries(tests)) if (!pick.length || pick.includes(name)) fn();
