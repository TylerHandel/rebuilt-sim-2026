// 4414's mechanisms (js/mech/r4414.js) on their own, no field, no robot: the ball path built from
// their tech binder, measured against what the binder and the team say. Run:
//   node --import ./tools/node-env.mjs tools/mechlab4414.mjs [test ...]
// Tests: rollers (each drive's free speed, the flywheel's spin-up), shooter (FUEL pushed up the
// column into the turret: how fast and how steep they leave against the flywheel's speed and the
// hood; MAP=1 sweeps both and writes the map tools/shot4414.mjs fits the launch table to), path
// (FUEL on a treadmill carpet, the robot driving into them: intake, rotor, hook, feeder, column,
// turret; how many and how fast), intake (intaking alone), intakefull (filling it through the
// intake), hold (how many it holds), dump (a full
// one fired at 3 m: the feed rate against their 12-14 BPS). And the feed's stages one at a time:
// bend (one FUEL from the hook's root to the shot), column (FUEL fed at the hook's root as fast as
// there's room), pillar (one FUEL set down still round the bend and up the bore: does it get to the
// shot alone, or where does it stop), ring (a ring of FUEL on the floor: what the hook takes in a
// turn), lip (FUEL on the stadium round an empty bowl: how fast they come down into it), hit (one
// FUEL on the floor in the turning rotor's way, then the rotor turning back: its speed, and what
// holds it back), rest (one FUEL set down still against the hook's rollers, the rotor agitating:
// does it stay put along them).
// Env: SEEDS (6), SPEED (1 m/s), STUCK (where the ones left over are), TRACE=i (follow FUEL i;
// shooter: every TEVERY steps, 3, with its spin), SHOT=rad,hood (one shot of your own), LAYERS
// (fill), TIMELINE, ROTOR (rps, 2.25), ROLLERS (duty, 1);
// dump: DSEEDS (fills), NOBOX (none in the box), LOAD (where the load is, each 0.5 s), SNAP=t and
// SNAPFILE (where the FUEL low round the rotor are, in its frame, nine times from t s), PARKED (the
// ones that stay put while it feeds; =file writes them), ROTORLOG (the rotor's speed and current
// each tenth of a second, and the torque each part on it takes from the FUEL it touches: torqueTally;
// ROTORLOG=n lists n of them); hold: HSEEDS, HOLDFILE (where the first fill's FUEL are at the end, and
// where they're pressed, for drawing); hold, intakefull: WALLS (wallWatch); bend, column, pillar:
// PATHLOG=file (each step, each FUEL's place and everything touching it, and the surfaces and
// wheels round the path, for drawing; pillar: PILLAR=tag picks the run it logs)
import { writeFileSync } from 'fs';
import { RAPIER } from '../js/physics.js';
import { FUEL, GROUP, groups, HUB } from '../js/constants.js';
import { Mechanism, Surface, foam } from '../js/mech/mechanism.js';
import { sim2D } from '../js/ballistics.js';
import {
  build4414, wheels4414, shooterWheels, pathSurfaces4414, placeIntake, placeRotor, placeRotorSurfaces, placeBox, placeShooter, placeTurretSurfaces,
  hoodElevation, chan, rotorPoint, hookCurve, hookBack, stadiumY, netY, AXIS, CH, HOOK, YB, FLY, SHOOTER, FEED, BORE, BOX, HALF_L, HALF_W, BAY, INTAKE, BEND, RAMP, STADIUM, FLOOR,
} from '../js/mech/r4414.js';
import { velocityDuty } from '../js/mech/phys604.js';
import { launchFor, shooterFor, rotorLoop } from '../js/mech/phys4414.js';

await RAPIER.init();
const R = FUEL.radius, DT = 1 / 120, SEEDS = Number(process.env.SEEDS ?? 6), SPEED = Number(process.env.SPEED ?? 1);
const HELD = groups(GROUP.HELD, 0xffff & ~GROUP.HELD);
const TAU = 2 * Math.PI;
const ROTOR_RPS = Number(process.env.ROTOR ?? 2.25), AGITATE = -0.15, ROLLERS = Number(process.env.ROLLERS ?? 1), NSHOT = Number(process.env.NSHOT ?? 4);

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
const names = (r, b) => [...r.path.contacts, ...r.shooter.contacts].filter((c) => c.ball === b).map((c) => c.wheel.name + ' ' + c.force.toFixed(0)).join(', ');

// (PATHLOG=file, in bend and column: each step, each FUEL's place, velocity and everything touching
// it: the force, the dent, where and which way, and the surface's own velocity there (a wheel's
// rim, a surface turning with the rotor); FUEL on FUEL from their places (the foam at half the
// overlap each). With the surfaces' triangles and the wheels near the path, posed as they are, for
// drawing. Off unless set.)
function pathLog(r) {
  if (!process.env.PATHLOG) return null;
  const steps = [];
  const sub = (a, b) => a.map((v, i) => v - b[i]), dot = (a, b) => a[0] * b[0] + a[1] * b[1] + a[2] * b[2];
  const cross = (a, b) => [a[1] * b[2] - a[2] * b[1], a[2] * b[0] - a[0] * b[2], a[0] * b[1] - a[1] * b[0]];
  const vel = (c) => {
    const wh = c.wheel;
    if (wh instanceof Surface) {
      if (!wh.pivot) return [0, 0, 0];
      const d = sub(c.x, wh.pivot), a = wh.pivotAxis, rr = sub(d, a.map((v) => v * dot(d, a)));
      return cross(a, rr).map((v) => v * wh.shaft.omega);
    }
    const d = sub(c.x, wh.center), rr = sub(d, wh.axis.map((v) => v * dot(d, wh.axis))), l = Math.hypot(...rr) || 1;
    return cross(wh.axis, rr.map((v) => (v / l) * wh.radius)).map((v) => v * wh.shaft.omega);
  };
  const near = (p) => Math.abs(p[0] - AXIS[0]) < 0.34 && Math.abs(p[2] - AXIS[1]) < 0.36 && p[1] < 0.8;
  return {
    rec(balls, t) {
      const P = balls.map((b) => { const p = b.translation(); return [p.x, p.y, p.z]; });
      balls.forEach((b, i) => {
        const p = P[i], v = b.linvel();
        if (p[1] > 3) return;
        const cs = [...r.path.contacts, ...r.shooter.contacts].filter((c) => c.ball === b).map((c) => ({ name: c.wheel.name, wheel: !(c.wheel instanceof Surface), kind: c.wheel.kind, F: c.force, d: c.depth, x: c.x, n: c.n, vs: vel(c) }));
        P.forEach((q, j) => {
          const dl = Math.hypot(...sub(p, q)), d = 2 * R - dl;
          if (j !== i && q[1] < 3 && d > 0) cs.push({ name: 'FUEL', wheel: false, F: foam(d / 2).F, d, x: p.map((v, k) => (v + q[k]) / 2), n: sub(p, q).map((v) => v / dl), vs: [0, 0, 0], other: j });
        });
        steps.push({ t, i, p, v: [v.x, v.y, v.z], c: cs });
      });
    },
    save() {
      const sfs = [...r.path.surfaces, ...r.shooter.surfaces].map((sf) => {
        const T = sf.tri, out = [], [ex, ey, ez] = sf.axes;
        const w = (k) => [0, 1, 2].map((j) => sf.pos[j] + ex[j] * T[k] + ey[j] * T[k + 1] + ez[j] * T[k + 2]);
        for (let o = 0; o < T.length; o += 9) {
          const a = w(o), b = w(o + 3), c = w(o + 6);
          if (near([0, 1, 2].map((j) => (a[j] + b[j] + c[j]) / 3))) out.push(...a, ...b, ...c);
        }
        return { name: sf.name, tri: out.map((v) => +v.toFixed(5)) };
      }).filter((s) => s.tri.length);
      const whs = [...r.path.wheels, ...r.shooter.wheels].filter((wh) => near(wh.center)).map((wh) => ({ name: wh.name, kind: wh.kind, c: wh.center, a: wh.axis, r: wh.radius, w: wh.width, omega: wh.shaft.omega, shaft: wh.shaft.name }));
      writeFileSync(process.env.PATHLOG, JSON.stringify({ axis: AXIS, R, surfaces: sfs, wheels: whs, steps }));
    },
  };
}

// the robot's mechanisms, fixed in the world: the box out ext, the turret at yaw (rad; 0 facing
// forward), the hood at its angle (deg)
function rig({ yaw = 0, hood = 140, ext = BOX.ext } = {}) {
  const t = build4414(), wh = wheels4414(t), s = pathSurfaces4414(t), sw = shooterWheels(t);
  const pathW = [...wh.feed, ...wh.intake];
  const path = new Mechanism({ substeps: 6, shafts: [...new Set(pathW.map((x) => x.shaft)), t.rotor], wheels: pathW, surfaces: Object.values(s).filter((x) => x !== s.turret && x !== s.hood) });
  const shooter = new Mechanism({ substeps: 6, sub: Number(process.env.SUB ?? 8), shafts: [t.fly, t.hoodRoller, t.feeder], wheels: sw, surfaces: [s.turret, s.hood] });
  for (const tr of [t.rollers, t.launcher]) tr.cmd = null;
  for (const sh of [t.rotor, t.primary, t.bar]) sh.cmd = null;
  const r = { t, wh, s, sw, path, shooter, yaw, hood, ext, phi: 0 };
  r.pose = () => {
    placeIntake(wh.intake, r.ext);
    placeBox(s.box, r.ext);
    placeRotorSurfaces(s, r.phi);
    placeRotor(wh.feed, r.phi);
    placeTurretSurfaces(s, r.yaw, r.hood);
    placeShooter(sw, r.yaw, r.hood);
  };
  r.step = (balls, together = balls) => {
    r.pose();
    path.step(DT, balls, together);
    shooter.step(DT, balls.filter((b) => { const p = b.translation(); return p.y > 0.2 && Math.hypot(p.x - AXIS[0], p.z - AXIS[1]) < 0.3; }));
    r.phi = (r.phi + t.rotor.omega * DT) % TAU;
  };
  r.pose();
  return r;
}
// (TORQUE: what holds the rotor back. Each contact's angular impulse on the rotor's shaft about its
// axis (r.path.tally, js/mech/mechanism.js), from a wheel or surface on it itself or through the
// pivot a wheel rides (the stack, the hook's rollers, the omnis: their drive's reaction too), by
// that wheel or surface and what else the FUEL touches then (FUEL: another FUEL), summed over a
// window: report() gives each one's mean torque over it (N·m, + turning it forward, the way it feeds)
// and the mean upward share of its contact's normal on the FUEL (- pushing it down), the biggest
// first, and clears it)
function torqueTally(r) {
  r.path.tally = { shaft: r.t.rotor };
  const sums = new Map(), short = (n) => n.replace(/ \(.*\)$/, '').replace(/ \d+$/, '');
  let T = 0;
  return {
    add(balls) {
      T += DT;
      const step = r.path.tally.step;
      if (!step) return;
      for (const [wh, byB] of step) for (const [b, e] of byB) {
        const p = b.translation();
        const with_ = new Set([...r.path.contacts, ...r.shooter.contacts].filter((c) => c.ball === b && c.wheel !== wh).map((c) => short(c.wheel.name)));
        if (balls.some((q) => q !== b && q.translation().y < 3 && Math.hypot(q.translation().x - p.x, q.translation().y - p.y, q.translation().z - p.z) < 2 * R)) with_.add('FUEL');
        for (const [L, tag] of [[e.L, ''], [e.Lp, ' (its pivot)']]) {
          if (!L) continue;
          const k = `${short(wh.name)}${tag} | ${[...with_].sort().join(' + ') || 'nothing else'}`, o = sums.get(k) ?? { L: 0, ny: 0, w: 0 };
          o.L += L; o.ny += e.n[1] * Math.abs(L); o.w += Math.abs(L);
          sums.set(k, o);
        }
      }
    },
    report(n = 6, min = 0.05) {
      const out = [...sums.entries()].map(([k, o]) => [k, o.L / T, o.ny / (o.w || 1)]).filter((q) => Math.abs(q[1]) >= min).sort((a, b) => Math.abs(b[1]) - Math.abs(a[1])).slice(0, n);
      const tot = [...sums.values()].reduce((a, o) => a + o.L, 0) / T;
      sums.clear(); T = 0;
      return `all ${tot.toFixed(1)} N·m` + out.map(([k, t, ny]) => `\n       ${t.toFixed(1)} N·m (n up ${ny.toFixed(2)}) ${k}`).join('');
    },
  };
}
// (where each FUEL first meets the TTB stack: in the rotor's frame (u out along the channel, w
// ahead), and what else it's touching then: on the hook's rollers, its plate, or loose on the floor;
// into firsts. stackReport sums them up)
function stackWatch(r, firsts) {
  const seen = new Set();
  return {
    check(balls) {
      for (const b of balls) {
        if (seen.has(b)) continue;
        const cs = r.path.contacts.filter((c) => c.ball === b);
        if (!cs.some((c) => c.wheel.name.startsWith('TTB'))) continue;
        seen.add(b);
        const p = b.translation(), q = rotorPoint([p.x, p.y, p.z], -r.phi);
        const on = cs.some((c) => c.wheel.name === 'hook rollers') ? 'rollers' : cs.some((c) => c.wheel.name.startsWith('hook and')) ? 'plate' : cs.some((c) => c.wheel.name === 'FUEL') ? 'FUEL' : 'loose';
        firsts.push({ u: q[2] - AXIS[1], w: -(q[0] - AXIS[0]), y: p.y, on });
      }
    },
  };
}
function stackReport(firsts) {
  if (!firsts.length) return '  the stack touched none';
  const by = (k) => firsts.filter((f) => f.on === k), u = firsts.map((f) => f.u).sort((a, b) => a - b);
  const fmt = (k) => { const a = by(k); return a.length ? `${a.length} ${k === 'rollers' ? "on the hook's rollers" : k === 'plate' ? "on the hook's plate past them" : 'loose on the floor'} (u ${Math.min(...a.map((f) => f.u)).toFixed(3)}..${Math.max(...a.map((f) => f.u)).toFixed(3)}, w ${Math.min(...a.map((f) => f.w)).toFixed(3)}..${Math.max(...a.map((f) => f.w)).toFixed(3)})` : ''; };
  return `  where each FUEL first met the TTB stack (the rotor's frame: u out along the channel, w ahead), of ${firsts.length}: ` + ['rollers', 'plate', 'loose'].map(fmt).filter(Boolean).join('; ') + `; u median ${u[u.length >> 1].toFixed(3)}`;
}
// the rotor at rps (a velocity loop on its X60: kV, kP as 604's), the rollers at a duty
function feed(r, rps = ROTOR_RPS, rollers = ROLLERS) {
  const t = r.t;
  rotorLoop(t.rotor, rps || null);
  t.rollers.cmd = rollers || null;
}
const launcher = (t, rad) => { t.launcher.cmd = rad > 0 ? velocityDuty(t.launcher.motor, t.fly.omega, rad, 12 / (t.launcher.motor.m.free / TAU), 0.3) : null; };
function landing(o) {
  const hit = sim2D(o.y, o.v, (o.el * Math.PI) / 180, HUB.rimFront);
  return hit ? hit.d + o.x : NaN;
}
// a FUEL that touched the flywheel and left it going up and out: shot (turret at yaw: out along it)
function shotWatch(r) {
  const touched = new Set(), shots = [];
  const c = Math.cos(r.yaw), s = Math.sin(r.yaw);
  return {
    shots,
    check(balls, t) {
      for (const b of balls) {
        if (r.shooter.contacts.some((q) => q.ball === b)) touched.add(b);
        else if (touched.has(b)) {
          touched.delete(b);
          const v = b.linvel(), p = b.translation(), out = v.x * c - v.z * s;
          if (v.y > 2 && out > 0.5) {
            shots.push({ t, v: Math.hypot(v.x, v.y, v.z), el: (Math.atan2(v.y, Math.hypot(v.x, v.z)) * 180) / Math.PI, side: (Math.atan2(v.x * s + v.z * c, out) * 180) / Math.PI, x: (p.x - AXIS[0]) * c - (p.z - AXIS[1]) * s, y: p.y, b });
            b.setTranslation({ x: 50 + shots.length, y: 5, z: 0 }, true); b.setGravityScale(0, true); b.setLinvel({ x: 0, y: 0, z: 0 }, true);
          }
        }
      }
    },
  };
}
const inRobot = (b) => { const p = b.translation(); return p.x > BAY.x0 - 0.05 && p.x < BOX.front + 0.05 && Math.abs(p.z) < HALF_W + 0.05 && p.y > 0.1 && p.y < 0.75; };
// how far FUEL get out past the hopper's walls, each step: the most a FUEL's surface is out past the
// walls' inner faces (as drawn: the bumper backing, BAY, chamfered at the back; the box's sides,
// BOX.side; WALLS=x0,hw,chamfer,side for others), and the deepest a FUEL dents into the walls (the
// foam's squeeze against them: hard walls, so all of it in the FUEL). And the FUEL that get out:
// through the top (the net, the top plate, round the turret: its center over them) or past the
// walls (its center out past their faces), each counted once
function wallWatch(r) {
  const [x0, hw, ch, side] = process.env.WALLS ? process.env.WALLS.split(',').map(Number) : [BAY.x0, BAY.hw, BAY.chamfer, BOX.side];
  const cover = (x, z) => (x < -0.21 ? BAY.top : netY(x, z));
  const o = { escaped: new Map() };
  o.reset = () => Object.assign(o, { out: -Infinity, where: '', dent: 0, dentAt: '', netDent: 0 });
  o.check = (balls) => {
    const atWall = new Set(); // (the ones against the walls: the dents counted are theirs)
    for (const b of balls) {
      const p = b.translation(), z = Math.abs(p.z);
      if (p.y < 0.1 || p.y > 1.2 || p.x < x0 - 0.3 || p.x > BOX.front || z > HALF_W + 0.3) continue;
      const box = p.x > HALF_L, turret = Math.hypot(p.x - AXIS[0], p.z - AXIS[1]) < 0.12 && p.y > 0.3;
      const out = box ? [[z - side, 'box side']] : [[z - hw, 'side wall'], [x0 - p.x, 'back wall'], [(ch - (p.x - x0) - (hw - z)) * Math.SQRT1_2, 'chamfer']];
      const over = p.y - cover(p.x, p.z);
      if (!o.escaped.has(b) && !turret && (over > 0 || out.some(([d]) => d > 0))) o.escaped.set(b, `${over > 0 ? 'top' : out.find(([d]) => d > 0)[1]} at ${p.x.toFixed(2)},${p.y.toFixed(2)},${p.z.toFixed(2)}`);
      if (o.escaped.has(b) || over > 0) continue;
      for (const [d, k] of out) if (d + R > o.out) { o.out = d + R; o.where = `${k} at ${p.x.toFixed(3)},${p.y.toFixed(3)},${p.z.toFixed(3)}`; }
      if (out.some(([d]) => d + R > -0.005)) atWall.add(b);
    }
    for (const c of r.path.dents) if (c.wheel === r.s.net) o.netDent = Math.max(o.netDent, c.depth);
    for (const c of r.path.dents) {
      if ((c.wheel !== r.s.body && c.wheel !== r.s.box) || !atWall.has(c.ball) || Math.abs(c.n[1]) > 0.5 || c.depth <= o.dent) continue;
      const p = c.ball.translation();
      o.dent = c.depth; o.dentAt = `at ${p.x.toFixed(3)},${p.y.toFixed(3)},${p.z.toFixed(3)}`;
    }
  };
  o.report = () => `FUEL out past the walls' faces at most ${(1000 * o.out).toFixed(1)} mm (${o.where}); dented into them at most ${(1000 * o.dent).toFixed(1)} mm (${o.dentAt}); into the net ${(1000 * o.netDent).toFixed(0)} mm`;
  o.escapes = () => `${o.escaped.size} got out${o.escaped.size ? ` (${[...o.escaped.values()].slice(0, 4).join('; ')})` : ''}`;
  o.reset();
  return o;
}
// fill it a layer at a time (FUEL set over the ones already in, let settle)
function fill(w, balls, step, layers = 6, y = 0.47, seed = 1) {
  let sd = seed * 7919 + 1;
  const rnd = () => ((sd = (sd * 16807) % 2147483647) / 2147483647 - 0.5);
  for (let layer = 0; layer < layers; layer++) {
    for (let i = 0; i < 6; i++) for (let k = 0; k < 5; k++) {
      const x = -0.23 + 0.155 * i + 0.03 * rnd(), z = -0.31 + 0.155 * k + 0.03 * rnd();
      if (Math.hypot(x - AXIS[0], z) < 0.13) continue; // (not into the turret)
      if (process.env.NOBOX && x > HALF_L - R) continue; // (NOBOX: none in the box out front)
      if (balls.some((b) => { const p = b.translation(); return Math.hypot(p.x - x, p.y - y, p.z - z) < 2 * R; })) continue;
      balls.push(ball(w, x, y, z));
    }
    step(1.0 / DT);
  }
}

// fill it to the top: FUEL dropped in at random over the hopper and the box, a few at a time, until
// two rounds running find no room
function fillUp(w, balls, step, seed = 1) {
  let sd = seed * 104729 + 7;
  const rnd = () => ((sd = (sd * 16807) % 2147483647) / 2147483647);
  for (let round = 0, idle = 0; round < 80 && idle < 3; round++) {
    let added = 0;
    for (let k = 0; k < 40 && added < 8; k++) {
      const x = BAY.x0 + 0.08 + rnd() * (BOX.front - BAY.x0 - 0.16), z = (rnd() - 0.5) * 2 * (BAY.hw - 0.08), y = netY(x, z) - R - 0.01;
      if (Math.hypot(x - AXIS[0], z - AXIS[1]) < 0.25) continue; // (not onto the turret)
      if (balls.some((b) => { const p = b.translation(); return Math.hypot(p.x - x, p.y - y, p.z - z) < 2 * R + 0.005; })) continue;
      balls.push(ball(w, x, y, z)); added++;
    }
    idle = added ? 0 : idle + 1;
    step(0.4 / DT);
  }
}

const tests = {
  rollers() {
    const r = rig(), t = r.t;
    t.primary.cmd = 1; t.bar.cmd = 1;
    for (let i = 0; i < 3 / DT; i++) { feed(r); launcher(t, 500); r.step([]); }
    const surf = (sh, rr) => Math.abs(sh.omega) * rr;
    console.log(`4414's drives, free (the intake 100%, the rotor at ${ROTOR_RPS} rps, the rollers ${ROLLERS * 100}%, the flywheel at 500 rad/s):`);
    console.log(`  intake: primary 3in ${RPM(t.primary.omega).toFixed(0)} rpm (${surf(t.primary, INTAKE.primary.r).toFixed(2)} m/s), bottom 1.5in ${RPM(t.bar.omega).toFixed(0)} rpm (${surf(t.bar, INTAKE.bar.r).toFixed(2)} m/s)`);
    console.log(`  rotor ${(t.rotor.omega / TAU).toFixed(2)} rps (free ${(t.rotor.motor.freeSpeed / TAU).toFixed(2)}; their ~2.25), the hook's tip at ${(t.rotor.omega * HOOK.tipR).toFixed(2)} m/s`);
    console.log(`  rollers: TTB 3in ${RPM(t.ttb.omega).toFixed(0)} rpm (${surf(t.ttb, FEED.ttb.r).toFixed(2)} m/s), sushi feeder ${RPM(t.feeder.omega).toFixed(0)} rpm (${surf(t.feeder, SHOOTER.feeder.r).toFixed(2)} m/s)`);
    console.log(`  flywheel ${RPM(t.fly.omega).toFixed(0)} rpm (${surf(t.fly, FLY.r).toFixed(1)} m/s), hood roller ${surf(t.hoodRoller, SHOOTER.roller.r).toFixed(1)} m/s; free ${RPM(t.launcher.motor.freeSpeed).toFixed(0)} rpm`);
    for (const rad of [300, 400, 500, 600, 700]) {
      const r2 = rig();
      let T = 0;
      for (; T < 6; T += DT) { launcher(r2.t, rad); r2.shooter.step(DT, []); if (Math.abs(r2.t.fly.omega - rad) < 0.03 * rad) break; }
      console.log(`  the flywheel from rest to within 3% of ${rad} rad/s (${RPM(rad).toFixed(0)} rpm): ${T.toFixed(2)} s`);
    }
  },
  // FUEL pushed up the column into the turret at a flywheel speed and hood
  shooter() {
    const map = !!process.env.MAP;
    const runs = process.env.SHOT ? [process.env.SHOT.split(',').map(Number)]
      : map ? [200, 250, 300, 350, 400, 450, 500, 550, 600, 650, 700].flatMap((rad) => [125, 130, 135, 140, 145, 150, 155, 160].map((h) => [rad, h]))
        : [[300, 150], [400, 145], [500, 140], [600, 136], [700, 132]];
    const out = [];
    console.log(`4414's shooter: FUEL pushed up the column into the sushi feeder (the rollers at ${ROLLERS * 100}%), the flywheel held at its speed:`);
    for (const [rad, hood] of runs) {
      const r = rig({ hood });
      const w = world({ carpet: false });
      for (let i = 0; i < 3 / DT; i++) { launcher(r.t, rad); r.shooter.step(DT, []); }
      const balls = [];
      const watch = shotWatch(r);
      let minFly = Infinity;
      for (let i = 0; i < 5 / DT && watch.shots.length < NSHOT; i++) {
        if (i % Math.round(0.35 / DT) === 0 && balls.length < NSHOT) balls.push(ball(w, AXIS[0], 0.275, AXIS[1], [0, 1.5, 0]));
        launcher(r.t, rad); feed(r, 0, ROLLERS);
        const live = balls.filter((b) => b.translation().y < 3);
        r.step(live); w.step();
        watch.check(live, i * DT);
        if (watch.shots.length) minFly = Math.min(minFly, r.t.fly.omega);
        if (process.env.TRACE && i % Number(process.env.TEVERY ?? 3) === 0 && balls[0] && balls[0].translation().y < 3) { const p = balls[0].translation(), v = balls[0].linvel(); console.log(`   ${(i * DT).toFixed(3)} ${p.x.toFixed(3)},${p.y.toFixed(3)},${p.z.toFixed(3)} v ${v.x.toFixed(2)},${v.y.toFixed(2)} w ${balls[0].angvel().z.toFixed(0)}: ${names(r, balls[0])}`); }
      }
      w.free();
      const o = watch.shots;
      if (!o.length) { console.log(`  ${rad} rad/s, hood ${hood}: nothing came out`); out.push({ rad, hood, n: 0 }); continue; }
      const m = { v: med(o.map((q) => q.v)), el: med(o.map((q) => q.el)), side: med(o.map((q) => q.side)), x: med(o.map((q) => q.x)), y: med(o.map((q) => q.y)) };
      const surf = rad * FLY.r, land = landing(m);
      out.push({ rad, hood, n: o.length, ...m, sd: Math.sqrt(o.reduce((a, q) => a + (q.v - m.v) ** 2, 0) / o.length), drop: rad - minFly });
      console.log(`  ${rad} rad/s (${surf.toFixed(1)} m/s at its surface), hood ${hood} (${hoodElevation(hood)} deg by the geometry): ${o.length}/${NSHOT} out at ${m.v.toFixed(2)} m/s (${(m.v / surf).toFixed(2)} of the surface), ${m.el.toFixed(1)} deg (${Math.min(...o.map((q) => q.el)).toFixed(0)}..${Math.max(...o.map((q) => q.el)).toFixed(0)}), ${m.side.toFixed(1)} aside, from ${m.x.toFixed(2)},${m.y.toFixed(2)}; down through the rim height ${land.toFixed(2)} m out; the flywheel dips ${(rad - minFly).toFixed(0)} rad/s`);
    }
    if (map) writeFileSync(process.env.MAP === '1' ? 'tools/shot4414-map.json' : process.env.MAP, JSON.stringify(out));
  },
  path() {
    const runs = [];
    for (let seed = 1; seed <= SEEDS; seed++) runs.push(pathRun(seed));
    console.log(`4414 carpet to shot (20 FUEL, four abreast, the robot at ${SPEED} m/s, the intake at 100%, the rotor at ${ROTOR_RPS} rps, the rollers on, a 3 m shot): a median ${med(runs.map((r) => r.shots))} shot (${Math.min(...runs.map((r) => r.shots))}..${Math.max(...runs.map((r) => r.shots))}), ` +
      `${med(runs.map((r) => r.rate)).toFixed(1)} FUEL/s once they're coming; ${med(runs.map((r) => r.inside))} in it at the end`);
    if (process.env.STUCK) for (const r of runs) console.log('   left', r.left.map((p) => p.map((v) => v.toFixed(2)).join(',')).join(' | '));
  },
  intake() {
    for (let seed = 1; seed <= Math.min(SEEDS, 3); seed++) {
      const r = pathRun(seed, { shoot: false, n: 24, sec: 6 });
      console.log(`4414 intaking only (rollers 100%, the rotor turning back slowly), 24 FUEL at ${SPEED} m/s: ${r.inside} in it after 6 s (in past the bumper each second: ${r.timeline.join(', ')}); the primary down to ${(100 * r.minPrimary).toFixed(0)}% of its free speed, the kick wheel and black roller to ${(100 * r.minBar).toFixed(0)}%`);
      if (process.env.STUCK) console.log('   left', r.left.map((p) => p.map((v) => v.toFixed(2)).join(',')).join(' | '));
    }
  },
  // filling it through the intake alone: 120 FUEL four abreast at SPEED, the rotor turning back
  // slowly, as many as it takes in before they back up out of the box (the hold test drops them in
  // under the net instead)
  intakefull() {
    for (let seed = 1; seed <= Math.min(SEEDS, 2); seed++) {
      const r = pathRun(seed, { shoot: false, n: 120, sec: 16 });
      console.log(`4414 filled through its intake (120 FUEL at ${SPEED} m/s, 16 s): ${r.inside} in it (each second: ${r.timeline.join(', ')}); ${r.walls}`);
    }
  },
  hold() {
    // (HSEEDS fills, each its own random drop: how it packs varies a few FUEL from fill to fill)
    const n = [];
    for (let seed = 1; seed <= Number(process.env.HSEEDS ?? 1); seed++) {
      const r = rig();
      const w = world();
      const balls = [], walls = wallWatch(r);
      const step = (k) => { for (let i = 0; i < k; i++) { feed(r, -0.15, 0); r.step(balls); w.step(); walls.check(balls); } };
      fillUp(w, balls, step, seed);
      step(2 / DT);
      const filling = walls.report();
      walls.reset();
      const t0 = performance.now();
      step(0.5 / DT);
      const ms = (performance.now() - t0) / (0.5 / DT);
      const inn = balls.filter((b) => inRobot(b) && !walls.escaped.has(b)), top = Math.max(...inn.map((b) => b.translation().y + R));
      if (seed === 1) console.log(`  (its mechanisms with them all in: ${ms.toFixed(2)} ms a step; the load's top ${top.toFixed(3)} m)`);
      n.push(inn.length);
      console.log(`  fill ${seed}: ${inn.length}; ${walls.escapes()}; filling, ${filling}; settled, ${walls.report()}`);
      // (each FUEL's center and its squash patches, as js/mech/physical.js draws them)
      if (process.env.HOLDFILE && seed === 1) {
        const sq = new Map();
        for (const c of r.path.dents) {
          if (c.wheel.name === 'FUEL' || !(c.depth > 0.0005)) continue;
          const d = Math.min(R * 0.6, c.wheel.tread < Infinity ? c.depth / 2 : c.depth);
          if (!sq.has(c.ball)) sq.set(c.ball, []);
          sq.get(c.ball).push([-c.n[0], -c.n[1], -c.n[2], R - d - 0.001]);
        }
        writeFileSync(process.env.HOLDFILE, JSON.stringify(inn.map((b) => { const p = b.translation(); return { p: [p.x, p.y, p.z], sq: (sq.get(b) || []).sort((u, v) => u[3] - v[3]).slice(0, 4) }; })));
      }
      if (process.env.STUCK) for (const b of balls.filter((q) => !inRobot(q))) { const p = b.translation(); console.log(`   out at ${p.x.toFixed(2)},${p.y.toFixed(2)},${p.z.toFixed(2)}`); }
      w.free();
    }
    console.log(`4414 holds ${n.join(', ')} (the box out, the rotor turning back slowly; dropped in until there's no room; their 85+ under the TRENCH, 100+ with the net up)`);
  },
  // one FUEL set down still along the way up, the rotor held still, the rollers on: round the
  // ramp's arc (deg from under the bend's center, 0, to the bore's foot, 90) and up the bore (y m):
  // whether it gets to the shot alone, or where it comes to rest and what touches it there
  pillar() {
    const at = [0, 15, 30, 45, 60, 75, 90].map((a) => ['ramp ' + a, (a * Math.PI) / 180]).concat([0.29, 0.31, 0.33, 0.35, 0.37].map((y) => ['bore ' + y, y]));
    for (const [tag, q] of at) {
      const r = rig({ hood: 140, yaw: Math.PI });
      const w = world({ carpet: false });
      for (let i = 0; i < 1 / DT; i++) { launcher(r.t, 480); feed(r, 0, ROLLERS); r.t.rotor._omega = 0; r.step([]); }
      const rc = RAMP.RO - R, [u, y] = tag.startsWith('ramp') ? [BEND.u - rc * Math.sin(q), BEND.y - rc * Math.cos(q)] : [BEND.u - rc, q];
      const [x, , z] = chan(u, 0, 0), b = ball(w, x, y, z);
      const watch = shotWatch(r), log = process.env.PILLAR === tag ? pathLog(r) : null;
      let k = 0, slow = 0;
      for (; k < 2 / DT; k++) {
        launcher(r.t, 480); feed(r, 0, ROLLERS); r.t.rotor._omega = 0;
        const live = [b].filter((q2) => q2.translation().y < 3);
        r.step(live); log?.rec([b], k * DT); w.step(); watch.check(live, k * DT);
        if (!live.length) break;
        const v = b.linvel(); slow = Math.hypot(v.x, v.y, v.z) < 0.02 ? slow + 1 : 0;
        if (slow > 0.3 / DT) break;
      }
      log?.save();
      const p = b.translation(), dx = p.x - AXIS[0], dz = p.z - AXIS[1];
      console.log(`  set down at ${tag} (u ${u.toFixed(3)}, y ${y.toFixed(3)}): ${watch.shots.length ? `shot after ${watch.shots[0].t.toFixed(2)} s` : `stuck after ${(k * DT).toFixed(2)} s at u ${dz.toFixed(3)} w ${(-dx).toFixed(3)} y ${p.y.toFixed(3)}: ${names(r, b)}`}`);
      w.free();
    }
  },
  // one FUEL on the floor in the hook's way (HIT_R m out from the axis, 0.19; HIT_A rad round from
  // the channel, 1.5, ahead of it), the rotor turning from rest at ROT rps (2.25, feeding: the
  // rollers and the flywheel on; -0.15 or any other under 0: turning back as it does when it isn't
  // feeding, the rollers coasting, the FUEL put HIT_A behind the channel instead), for HIT_T s (2.5): each
  // 0.05 s the rotor's speed and current, where the FUEL is (the rotor's frame: u out along its
  // channel, w ahead; y) and what holds the rotor back (torqueTally). HIT_SW=t: at t s it stops
  // feeding and turns back to agitate (AGITATE rps, -0.15), as it does in play once it's shot
  hit() {
    const rps = Number(process.env.ROT ?? ROTOR_RPS), rr = Number(process.env.HIT_R ?? 0.19), ah = Number(process.env.HIT_A ?? 1.5), SW = Number(process.env.HIT_SW ?? Infinity);
    const r = rig({ hood: 150, yaw: Math.PI });
    const w = world({ carpet: false });
    const a = CH + Math.sign(rps) * ah;
    const b = ball(w, AXIS[0] + rr * Math.cos(a), YB + 0.001, AXIS[1] + rr * Math.sin(a));
    const watch = shotWatch(r), tq = torqueTally(r);
    let slow = Infinity, at = 0, win = { turn: 0, lo: Infinity, hi: -Infinity, A: 0 };
    console.log(`4414's rotor at ${rps} rps from rest, one FUEL on the floor ${rr} m out, ${ah} rad ${rps > 0 ? 'ahead of' : 'behind'} the channel:`);
    for (let k = 0; k < Number(process.env.HIT_T ?? 2.5) / DT; k++) {
      const now = k * DT >= SW ? AGITATE : rps;
      launcher(r.t, now > 0 ? 400 : 0);
      feed(r, now, now > 0 ? ROLLERS : 0);
      const live = [b].filter((q) => q.translation().y < 3);
      r.step(live); w.step(); watch.check(live, k * DT); tq.add(live);
      const om = r.t.rotor.omega / TAU;
      if (k * DT > 0.3 && Math.abs(om) < Math.abs(slow)) { slow = om; at = k * DT; }
      win.turn += om * DT; win.lo = Math.min(win.lo, om); win.hi = Math.max(win.hi, om); win.A = Math.max(win.A, Math.abs(r.t.rotor.motor.current));
      if (k % 6 === 5) {
        const p = b.translation(), q = rotorPoint([p.x, p.y, p.z], -r.phi), u = q[2] - AXIS[1], ww = -(q[0] - AXIS[0]);
        console.log(`   ${((k + 1) * DT).toFixed(2)} s: rotor ${(win.turn / (6 * DT)).toFixed(2)} rps (${win.lo.toFixed(2)}..${win.hi.toFixed(2)}; up to ${win.A.toFixed(0)} A); ${live.length ? `FUEL u ${u.toFixed(3)} w ${ww.toFixed(3)} y ${p.y.toFixed(3)}: ${names(r, b) || 'nothing'}` : 'shot'}; ${tq.report(4)}`);
        win = { turn: 0, lo: Infinity, hi: -Infinity, A: 0 };
      }
    }
    console.log(`  slowest after 0.3 s: ${slow.toFixed(2)} rps at ${at.toFixed(2)} s; ${watch.shots.length ? `shot at ${watch.shots[0].t.toFixed(2)} s` : 'not shot'}`);
    w.free();
  },
  // one FUEL set down still against the hook's rollers (s along it from the tip: REST=s,s,...), the
  // rotor turning slowly, back as it does when it isn't feeding and forward as slowly (AGITATE=rps,
  // ...: -0.15,0.15), the TTB stack coasting: how far it goes along the hook's face in 3 s (in the
  // rotor's frame), how much of the time it's on the rollers and how far they turn. Free rollers
  // let it drift along the face; held by their bearings it should go round with the hook
  rest() {
    const ss = (process.env.REST ?? '0.15,0.3,0.45,0.6,0.7').split(',').map(Number);
    for (const rps of (process.env.AGITATE ?? '-0.15,0.15').split(',').map(Number)) restRun(ss, rps);
  },
  // one FUEL at the hook's root, the rotor held still, the rollers on: in, round the bend, up
  bend() {
    const r = rig({ hood: 140, yaw: Math.PI });
    const w = world({ carpet: false });
    for (let i = 0; i < 1 / DT; i++) { launcher(r.t, 480); feed(r, 0, ROLLERS); r.t.rotor._omega = 0; r.step([]); }
    const [x, , z] = chan(Number(process.env.U ?? 0.175), 0, Number(process.env.W ?? -0.005));
    const b = ball(w, x, YB + 0.002, z);
    const watch = shotWatch(r), log = pathLog(r), firsts = [], sw = stackWatch(r, firsts);
    for (let i = 0; i < 1.5 / DT; i++) {
      launcher(r.t, 480); feed(r, 0, ROLLERS); r.t.rotor._omega = 0;
      const live = [b].filter((q) => q.translation().y < 3);
      r.step(live); log?.rec([b], i * DT); w.step(); watch.check(live, i * DT); sw.check(live);
      if (i % 3 === 0 && live.length) { const p = b.translation(), v = b.linvel(), wv = b.angvel(); console.log(`   ${(i * DT).toFixed(3)} ${(p.x - AXIS[0]).toFixed(3)},${p.y.toFixed(3)},${(p.z - AXIS[1]).toFixed(3)} v ${Math.hypot(v.x, v.y, v.z).toFixed(2)} vy ${v.y.toFixed(2)} spin ${wv.x.toFixed(0)},${wv.y.toFixed(0)},${wv.z.toFixed(0)} feeder ${r.t.feeder.omega.toFixed(0)}: ${process.env.N ? [...r.path.contacts].filter((c) => c.ball === b).map((c) => c.wheel.name + ' n' + c.n.map((q) => q.toFixed(2)).join(',')).join('; ') : names(r, b)}`); }
      if (!live.length) break;
    }
    log?.save();
    console.log((watch.shots.length ? `shot after ${watch.shots[0].t.toFixed(2)} s` : 'not shot') + '\n' + stackReport(firsts));
  },
  // the column alone: FUEL set down at the hook's root (U, W in the channel's frame) whenever
  // there's room, the rotor held still: how fast the TTB wheels, the bend, the bore and the shooter
  // take them. ROT=1: the rotor turning at its speed, the FUEL set down moving with it; ROT=2: set
  // down still, as on the floor when the rotor comes round to them
  column() {
    const rot = !!process.env.ROT, still = process.env.ROT === '2';
    const r = rig({ hood: 150, yaw: Math.PI });
    const w = world({ carpet: false });
    const run = () => { launcher(r.t, 400); if (rot) feed(r); else { feed(r, 0, ROLLERS); r.t.rotor._omega = 0; r.phi = 0; } };
    for (let i = 0; i < 1 / DT; i++) { run(); r.step([]); }
    const at = chan(Number(process.env.U ?? 0.2), 0, Number(process.env.W ?? 0));
    const balls = [], watch = shotWatch(r), log = pathLog(r);
    for (let k = 0; k < 3 / DT; k++) {
      run();
      const live = balls.filter((b) => b.translation().y < 3);
      const [x, , z] = rotorPoint(at, r.phi), om = r.t.rotor.omega;
      if (!live.some((b) => { const p = b.translation(); return Math.hypot(p.x - x, p.y - YB, p.z - z) < 2 * R + 0.005; })) balls.push(ball(w, x, YB + 0.002, z, still ? [0, 0, 0] : [-om * (z - AXIS[1]), 0, om * (x - AXIS[0])]));
      r.step(live); log?.rec(balls, k * DT); w.step(); watch.check(live, k * DT);
      if (process.env.TRACE && k > 1.5 / DT && k < 1.9 / DT && k % 3 === 0) console.log(`  ${(k * DT).toFixed(3)} ${watch.shots.length} shot, rollers ${RPM(r.t.ttb.omega).toFixed(0)} rpm ${r.t.rollers.motor.current.toFixed(0)} A, feeder ${r.t.feeder.omega.toFixed(0)}:` + live.map((b) => { const p = b.translation(), v = b.linvel(); return `\n     ${(p.x - AXIS[0]).toFixed(3)},${p.y.toFixed(3)},${(p.z - AXIS[1]).toFixed(3)} v ${Math.hypot(v.x, v.y, v.z).toFixed(2)}: ${names(r, b)}`; }).join(''));
    }
    log?.save();
    const s = watch.shots.map((o) => o.t).filter((t) => t > 1);
    console.log(`4414's column${rot ? ' (the rotor turning)' : ''}, FUEL fed at the hook's root as fast as there's room: ${s.length > 1 ? ((s.length - 1) / (s[s.length - 1] - s[0])).toFixed(1) : 0} FUEL/s shot (after the first second)`);
  },
  // the hook alone: a ring of FUEL laid on the floor round the hub (RING=n of them, 8 by default,
  // at RING_R m from the axis, 0.19), nothing over them, the rotor feeding from rest: how many it
  // takes in on its first turn, and on its second (the ones it pushed round)
  ring() {
    const n = Number(process.env.RING ?? 8), rr = Number(process.env.RING_R ?? 0.19), res = [], firsts = [];
    for (let seed = 1; seed <= SEEDS; seed++) {
      const r = rig({ hood: 150, yaw: Math.PI });
      const w = world({ carpet: false });
      const a0 = seed * 0.37;
      const balls = Array.from({ length: n }, (_, k) => { const a = a0 + (k / n) * TAU; return ball(w, AXIS[0] + rr * Math.cos(a), YB + 0.001, AXIS[1] + rr * Math.sin(a)); });
      const turns = [0, 0, 0], sw = stackWatch(r, firsts);
      for (let k = 0; k < 1.5 / DT; k++) {
        launcher(r.t, 400); feed(r);
        const live = balls.filter((b) => b.translation().y < 3);
        r.step(live); w.step();
        if (process.env.SNAP && seed === 1 && k % 5 === 0 && k * DT >= Number(process.env.SNAP)) snap(r, live, k * DT);
        sw.check(live);
        const turn = Math.floor(Math.abs(r.t.rotor.omega) > 0 ? (k * DT) / (1 / ROTOR_RPS) : 0);
        for (const b of live) { const p = b.translation(); if (Math.hypot(p.x - AXIS[0], p.z - AXIS[1]) < 0.06 && p.y > 0.26 && !b.up) { b.up = true; turns[Math.min(2, turn)]++; } }
      }
      res.push(turns);
      w.free();
    }
    const tot = res.map((t) => t[0] + t[1] + t[2]);
    console.log(`4414's hook, ${n} FUEL on the floor ${rr} m out: up the bore in its first 0.44 s ${res.map((t) => t[0]).join(', ')}; by 0.89 s ${res.map((t) => t[0] + t[1]).join(', ')}; by 1.5 s ${tot.join(', ')} (mean ${(tot.reduce((a, b) => a + b, 0) / tot.length).toFixed(1)})`);
    console.log(stackReport(firsts));
  },
  // the funnel alone: LIP=n FUEL (12) laid on the stadium round the bowl, LIP_R (0.33) m out, the
  // bowl empty, the rotor feeding: how fast they come down into it and are shot
  lip() {
    const n = Number(process.env.LIP ?? 12), rr = Number(process.env.LIP_R ?? 0.33), res = [];
    for (let seed = 1; seed <= SEEDS; seed++) {
      const r = rig({ hood: 150, yaw: Math.PI });
      const w = world({ carpet: false });
      const balls = [];
      for (let k = 0; k < n; k++) { const a = seed * 0.37 + (k / n) * TAU, x = AXIS[0] + rr * Math.cos(a), z = AXIS[1] + rr * Math.sin(a); if (x < HALF_L - 0.03 && x > BAY.x0 + R + 0.005 && Math.abs(z) < BAY.hw - R - 0.005 && (x - BAY.x0) + BAY.hw - Math.abs(z) > BAY.chamfer + R * 1.5) balls.push(ball(w, x, stadiumY(x, z) + R + 0.002, z)); }
      const watch = shotWatch(r);
      for (let k = 0; k < 4 / DT; k++) {
        launcher(r.t, 400); feed(r);
        const live = balls.filter((b) => b.translation().y < 3);
        r.step(live); w.step(); watch.check(live, k * DT);
        if (process.env.SNAP && seed === 1 && k % 6 === 0 && k * DT >= Number(process.env.SNAP)) snap(r, live, k * DT);
        if (process.env.TRACE && seed === 1 && k % 3 === 0) { const b = balls[Number(process.env.TRACE)], p = b.translation(), v = b.linvel(); if (p.y < 3) console.log(`   ${(k * DT).toFixed(3)} ${(p.x - AXIS[0]).toFixed(3)},${p.y.toFixed(3)},${(p.z - AXIS[1]).toFixed(3)} (${Math.hypot(p.x - AXIS[0], p.z - AXIS[1]).toFixed(3)} out) v ${v.x.toFixed(2)},${v.y.toFixed(2)},${v.z.toFixed(2)}: ${names(r, b)}`); }
      }
      res.push(`${watch.shots.length}/${balls.length} (${[1, 2, 3, 4].map((t) => watch.shots.filter((o) => o.t < t).length).join(', ')})`);
      if (process.env.STUCK) for (const b of balls) { const p = b.translation(), v = b.linvel(); if (p.y < 3) console.log(`   left at ${(p.x - AXIS[0]).toFixed(3)},${p.y.toFixed(3)},${(p.z - AXIS[1]).toFixed(3)} (${Math.hypot(p.x - AXIS[0], p.z - AXIS[1]).toFixed(3)} out) v ${Math.hypot(v.x, v.z).toFixed(2)}: ${names(r, b)}`); }
      w.free();
    }
    console.log(`4414's funnel, FUEL laid on the stadium ${rr} m out round an empty bowl: shot (by 1, 2, 3, 4 s) ${res.join('; ')}`);
  },
  dump() {
    const res = [];
    for (let seed = 1 + Number(process.env.SEED0 ?? 0); seed <= Number(process.env.SEED0 ?? 0) + Number(process.env.DSEEDS ?? 1); seed++) res.push(dumpRun(seed));
    if (res.length > 1) console.log(`  over ${res.length} fills: through the middle ${res.map((x) => x.mid.toFixed(1)).join(', ')} (median ${med(res.map((x) => x.mid)).toFixed(1)}); its best second ${res.map((x) => x.best).join(', ')} (median ${med(res.map((x) => x.best))}); dumped ${res.map((x) => x.n + '/' + x.n0).join(', ')}`);
  },
};

// (rest: its FUEL one at a time, the rotor at rps)
function restRun(ss, rps) {
  // (the face's length from the tip to s, and the s nearest a point in the rotor's frame)
  const pts = Array.from({ length: 401 }, (_, i) => { const { r, a } = hookCurve(i / 400); return [AXIS[0] + r * Math.cos(a), AXIS[1] + r * Math.sin(a)]; });
  const arc = [0]; for (let i = 1; i < pts.length; i++) arc.push(arc[i - 1] + Math.hypot(pts[i][0] - pts[i - 1][0], pts[i][1] - pts[i - 1][1]));
  const nearest = (x, z) => { let k = 0, d = Infinity; pts.forEach((p, i) => { const e = Math.hypot(p[0] - x, p[1] - z); if (e < d) { d = e; k = i; } }); return { s: k / 400, at: arc[k], d }; };
  const out = [];
  for (const s0 of ss) {
    const r = rig({ hood: 150, yaw: Math.PI });
    const w = world({ carpet: false });
    const { r: rr, a } = hookBack(s0, -(R + 0.001)), b = ball(w, AXIS[0] + rr * Math.cos(a), YB + 0.001, AXIS[1] + rr * Math.sin(a));
    const hr = r.wh.feed.filter((x) => x.name === 'hook rollers'), turned = hr.map(() => 0);
    const n0 = nearest(b.translation().x, b.translation().z);
    let onHook = 0, k = 0;
    for (; k < 3 / DT; k++) {
      feed(r, rps, 0);
      r.step([b]); w.step();
      hr.forEach((x, i) => { turned[i] += Math.abs(x.shaft.omega) * DT; });
      if (r.path.contacts.some((c) => c.ball === b && c.wheel.name === 'hook rollers')) onHook++;
    }
    const q = b.translation(), p = rotorPoint([q.x, q.y, q.z], -r.phi), n1 = nearest(p[0], p[2]);
    out.push(`  from s ${s0}: ${(100 * (n1.at - n0.at)).toFixed(1)} cm along the face (to s ${n1.s.toFixed(2)}), ${(100 * (n1.d - n0.d)).toFixed(1)} cm off it; on the rollers ${((100 * onHook) / k).toFixed(0)}% of the time, its rollers turned ${Math.max(...turned).toFixed(1)} rad at most; at the end ${names(r, b) || 'touching nothing on the rotor'}`);
    w.free();
  }
  console.log(`4414's hook: a lone FUEL set down against its rollers, the rotor at ${rps} rps for 3 s, the TTB stack coasting (the rollers' bearings ${(HOOK.rollers.friction * 1000).toFixed(0)} mN·m):\n${out.join('\n')}`);
}

// FUEL low round the rotor in its frame (u, w, height, speed), into SNAPFILE: nine frames from SNAP s
function snap(r, balls, t) {
  if ((r.snaps ??= []).length >= 9) return;
  const f = [];
  for (const b of balls) { const p = b.translation(), v = b.linvel(), dx = p.x - AXIS[0], dz = p.z - AXIS[1], rr = Math.hypot(dx, dz); if (p.y > 0.5 || rr > 0.45) continue; const a = Math.atan2(dz, dx) - Math.PI / 2 - r.phi; f.push([rr * Math.cos(a), rr * Math.sin(a), p.y, Math.hypot(v.x, v.z)]); }
  r.snaps.push({ t, f });
  writeFileSync(process.env.SNAPFILE ?? 'snap.json', JSON.stringify(r.snaps));
}

function dumpRun(seed = 1) {
    // (the table's shot for DIST m, 3 by default; or RAD and HOOD of your own)
    const D = Number(process.env.DIST ?? 3), row = launchFor(D), want = shooterFor(row.v, row.el);
    const rad = Number(process.env.RAD ?? want.rad), hood = Number(process.env.HOOD ?? want.hood);
    const r = rig({ hood, yaw: Math.PI });
    const w = world();
    const balls = [];
    const step = (n, run) => { for (let i = 0; i < n; i++) { if (run) run(); const live = balls.filter((b) => b.translation().y < 3); r.step(live); w.step(); } };
    fill(w, balls, (n) => step(n, () => { launcher(r.t, rad); feed(r, -0.15, 0); }), Number(process.env.LAYERS ?? 8), 0.47, seed);
    const n0 = balls.filter(inRobot).length;
    const watch = shotWatch(r), tq = process.env.ROTORLOG ? torqueTally(r) : null;
    let minFly = Infinity;
    for (let k = 0; k < 10 / DT; k++) {
      step(1, () => { launcher(r.t, rad); feed(r); r.t.primary.cmd = r.t.bar.cmd = process.env.INTAKE === '0' ? null : 1; });
      tq?.add(balls);
      watch.check(balls, k * DT);
      if (watch.shots.length) minFly = Math.min(minFly, r.t.fly.omega);
      if (process.env.COLUMN && k % 30 === 29) {
        const col = balls.filter((b) => { const p = b.translation(); return p.y < 3 && p.y > 0.2 && Math.hypot(p.x - AXIS[0], p.z - AXIS[1]) < 0.12; });
        console.log(`   -- ${((k + 1) * DT).toFixed(2)} s, ${watch.shots.length} shot; in the column:` + col.map((b) => { const p = b.translation(), v = b.linvel(); return `\n     ${(p.x - AXIS[0]).toFixed(3)},${p.y.toFixed(3)},${(p.z - AXIS[1]).toFixed(3)} vy ${v.y.toFixed(2)}: ${names(r, b)}`; }).join(''));
      }
      if (process.env.STAGES) {
        for (const b of balls) {
          const p = b.translation(); if (p.y > 3) continue;
          const dx = p.x - AXIS[0], dz = p.z - AXIS[1], rr = Math.hypot(dx, dz), a = Math.atan2(dz, dx) - Math.PI / 2 - r.phi, u = rr * Math.cos(a), ww = rr * Math.sin(a);
          if (b.tIn === undefined && u < 0.15 && u > 0 && Math.abs(ww) < 0.08 && p.y < 0.26) b.tIn = k * DT;
          if (b.tBore === undefined && rr < 0.06 && p.y > 0.29) b.tBore = k * DT;
        }
      }
      if (process.env.HEAT && k % 3 === 0 && k > 120) {
        for (const b of balls) { const p = b.translation(); if (p.y > 0.26 || p.y > 3) continue; const dx = p.x - AXIS[0], dz = p.z - AXIS[1], rr = Math.hypot(dx, dz), a = Math.atan2(dz, dx) - Math.PI / 2 - r.phi; (r.heat ??= []).push([rr * Math.cos(a), rr * Math.sin(a)]); }
        if (k === Math.round(10 / DT) - 3) writeFileSync(process.env.HEAT, JSON.stringify(r.heat));
      }
      if (process.env.DIAG && (process.env.DIAG === '1' ? k % 240 === 239 : Math.abs(k * DT - Number(process.env.DIAG)) < DT / 2)) {
        console.log(`   -- ${((k + 1) * DT).toFixed(1)} s, FUEL low round the rotor (in the rotor's frame: u out along its channel, w ahead; y; speed):`);
        for (const b of balls) { const p = b.translation(), v = b.linvel(), dx = p.x - AXIS[0], dz = p.z - AXIS[1], rr = Math.hypot(dx, dz); if (p.y > 0.3 || rr > 0.34 || p.y > 3) continue; const a = Math.atan2(dz, dx) - Math.PI / 2 - r.phi; console.log(`     u ${(rr * Math.cos(a)).toFixed(3)} w ${(rr * Math.sin(a)).toFixed(3)} y ${p.y.toFixed(3)} v ${Math.hypot(v.x, v.z).toFixed(2)}: ${names(r, b)}`); }
      }
      // (SNAP=t: snapshots every 0.05 s from t)
      if (process.env.SNAP && k % 6 === 0 && k * DT >= Number(process.env.SNAP)) snap(r, balls, k * DT);
      // (where the load is: on the floor the rotor sweeps, on the stadium round it, or up on the others)
      if (process.env.LOAD && k % 60 === 59) {
        let fl = 0, st = 0, bx = 0, up = 0, ride = 0;
        for (const b of balls) { if (!inRobot(b)) continue; const p = b.translation(), d = Math.hypot(p.x - AXIS[0], p.z - AXIS[1]); if (p.x > HALF_L) bx++; else if (p.y < 0.23 && d < 0.3) { fl++; const v = b.linvel(); if (Math.hypot(v.x, v.z) > 1.5) ride++; } else if (p.y < 0.3) st++; else up++; }
        console.log(`   ${((k + 1) * DT).toFixed(1)} s: ${watch.shots.length} shot; on the rotor's floor ${fl} (${ride} riding round with it), low round it ${st}, in the box ${bx}, up on the others ${up}`);
        if (process.env.LOAD === 'box' && k % 120 === 119) for (const b of balls) { const p = b.translation(); if (inRobot(b) && p.x > HALF_L - 0.05) { const v = b.linvel(); console.log(`     at ${p.x.toFixed(3)},${p.y.toFixed(3)},${p.z.toFixed(3)} v ${Math.hypot(v.x, v.y, v.z).toFixed(2)}: ${names(r, b)}`); } }
      }
      // (PARKED: each 0.5 s, the FUEL in it that moved under 1.5 cm over the last second while the
      // rotor fed: where (the robot's frame; the rotor's u, w), what touches them; PARKED=file also
      // writes them all, for drawing)
      // (ESCAPE: FUEL that leave the bowl while it feeds, from on its floor (inside the rim, low) out
      // over the wall onto the stadium: how many, a second and a rotor turn, and where round the rim
      // (deg from the front, toward the right))
      if (process.env.ESCAPE) {
        for (const b of balls) {
          const p = b.translation(); if (p.y > 3) continue;
          const d = Math.hypot(p.x - AXIS[0], p.z - AXIS[1]);
          if (d < STADIUM.r - 0.03 && p.y < FLOOR + R + 0.02) b.inBowl = true;
          else if (b.inBowl && d > STADIUM.r + 0.01) { b.inBowl = false; (r.esc ??= []).push({ t: k * DT, a: (Math.atan2(p.z - AXIS[1], p.x - AXIS[0]) * 180) / Math.PI }); }
        }
        r.turns = (r.turns ?? 0) + (Math.abs(r.t.rotor.omega) * DT) / TAU;
      }
      if (process.env.PARKED && k % 60 === 59) {
        const now = (k + 1) * DT, park = [];
        for (const b of balls) {
          if (!inRobot(b)) continue;
          const p = b.translation(), h = (b.hist ??= []);
          h.push([now, p.x, p.y, p.z]);
          const o = h.find((q) => q[0] >= now - 1.0 - 1e-6);
          if (now < 1.5 || !o || Math.hypot(p.x - o[1], p.y - o[2], p.z - o[3]) > 0.015) continue;
          const dx = p.x - AXIS[0], dz = p.z - AXIS[1], rr = Math.hypot(dx, dz), a = Math.atan2(dz, dx) - Math.PI / 2 - r.phi;
          park.push({ t: now, p: [p.x, p.y, p.z], u: rr * Math.cos(a), w: rr * Math.sin(a), rr, c: names(r, b) });
        }
        if (park.length) console.log(`   -- ${now.toFixed(1)} s, ${park.length} parked (rotor ${r.t.rotor.motor.current.toFixed(0)} A, rollers ${r.t.rollers.motor.current.toFixed(0)} A, intake ${r.t.primary.motor.current.toFixed(0)}/${r.t.bar.motor.current.toFixed(0)} A):` + park.map((q) => `\n     at ${q.p.map((v) => v.toFixed(3)).join(',')} (${q.rr.toFixed(3)} out; rotor's u ${q.u.toFixed(3)} w ${q.w.toFixed(3)}): ${q.c}`).join(''));
        (r.parked ??= []).push(...park);
        if (process.env.PARKED !== '1' && k === Math.round(10 / DT) - 1) writeFileSync(process.env.PARKED, JSON.stringify(r.parked));
      }
      // (ROTORLOG: the rotor's speed each tenth of a second, its slowest and its current there)
      if (process.env.ROTORLOG) {
        const w = r.t.rotor.omega / TAU, rl = (r.rotorLog ??= { min: Infinity, maxA: 0 });
        if (w < rl.min) { rl.min = w; rl.at = r.t.rotor.motor.current; }
        rl.maxA = Math.max(rl.maxA, Math.abs(r.t.rotor.motor.current));
        (r.rollLog ??= []).push([RPM(r.t.ttb.omega), Math.abs(r.t.rollers.motor.current) / r.t.rollers.motor.count, watch.shots.length]);
        if (k % 12 === 11) { const q = tq.report(Number(process.env.ROTORLOG) > 1 ? Number(process.env.ROTORLOG) : 3, 0.5); console.log(`   ${((k + 1) * DT).toFixed(1)} s: rotor ${w.toFixed(2)} rps (slowest ${rl.min.toFixed(2)} at ${rl.at.toFixed(0)} A; up to ${rl.maxA.toFixed(0)} A), ${watch.shots.length} shot; on it, ${q}`); r.rotorLog = null; }
      }
      if (process.env.TIMELINE && k % 60 === 59) console.log(`   ${((k + 1) * DT).toFixed(1)} s: ${watch.shots.length} shot; rotor ${(r.t.rotor.omega / TAU).toFixed(2)} rps ${r.t.rotor.motor.current.toFixed(0)} A, rollers ${RPM(r.t.ttb.omega).toFixed(0)} rpm ${r.t.rollers.motor.current.toFixed(0)} A, flywheel ${r.t.fly.omega.toFixed(0)}`);
    }
    if (process.env.STAGES) {
      const ins = balls.map((b) => b.tIn).filter((x) => x !== undefined).sort((p, q) => p - q), bores = balls.map((b) => b.tBore).filter((x) => x !== undefined).sort((p, q) => p - q);
      const gaps = (t) => t.slice(1).map((x, i) => x - t[i]);
      const shotB = watch.shots.map((o) => o.b);
      console.log(`  stages: into the channel ${ins.length}, into the bore ${bores.length}, shot ${watch.shots.length}; median gap in ${med(gaps(ins)).toFixed(3)} s, bore ${med(gaps(bores)).toFixed(3)} s, shots ${med(gaps(watch.shots.map((o) => o.t))).toFixed(3)} s; channel->bore median ${med(shotB.filter((b) => b.tIn !== undefined && b.tBore !== undefined).map((b) => b.tBore - b.tIn)).toFixed(3)} s, bore->shot ${med(watch.shots.filter((o) => o.b.tBore !== undefined).map((o) => o.t - o.b.tBore)).toFixed(3)} s`);
      console.log('  entries: ' + ins.slice(0, 40).map((x) => x.toFixed(2)).join(' '));
    }
    if (process.env.ESCAPE) {
      const e = r.esc ?? [], last = watch.shots.length ? watch.shots[watch.shots.length - 1].t : 10, ef = e.filter((q) => q.t <= last), turns = r.turns * (last / 10);
      const bin = (lo, hi) => ef.filter((q) => Math.abs(q.a) >= lo && Math.abs(q.a) < hi).length;
      console.log(`  out of the bowl over its wall while it fed (to the last shot, ${last.toFixed(1)} s, ~${turns.toFixed(0)} turns): ${ef.length} (${(ef.length / last).toFixed(2)}/s, ${(ef.length / turns).toFixed(2)} a turn); at the front (within 45 deg) ${bin(0, 45)}, the sides ${bin(45, 135)}, the back ${bin(135, 181)}`);
    }
    const s = watch.shots.map((x) => x.t), first = s[0] ?? 0, in1 = s.filter((x) => x - first < 1).length;
    const left = balls.filter((b) => b.translation().y < 3 && inRobot(b));
    if (process.env.STUCK) for (const b of left) { const p = b.translation(); console.log(`   left at ${p.x.toFixed(2)},${p.y.toFixed(2)},${p.z.toFixed(2)}: ${names(r, b)}`); }
    const sh = watch.shots;
    if (sh.length) {
      const lands = sh.map((o) => landing(o)).filter((x) => !Number.isNaN(x));
      const good = lands.filter((x) => Math.abs(x - D) < 0.4).length;
      console.log(`  (landing for ${D} m: median ${med(lands).toFixed(2)}, ${good}/${sh.length} within 0.4 m; ${rad.toFixed(0)} rad/s, hood ${hood.toFixed(1)})`);
    }
    if (sh.length) console.log(`  (shots leave at ${med(sh.map((o) => o.v)).toFixed(2)} m/s (${Math.min(...sh.map((o) => o.v)).toFixed(2)}..${Math.max(...sh.map((o) => o.v)).toFixed(2)}), ${med(sh.map((o) => o.el)).toFixed(1)} deg; the flywheel down to ${minFly.toFixed(0)} rad/s of ${rad}; ${left.length} still in it)`);
    const span = s.length > 1 ? s[s.length - 1] - first : 0;
    // (the steady rate: the middle half of the dump)
    const mid = s.slice(Math.floor(s.length / 4), Math.ceil((3 * s.length) / 4)), midRate = mid.length > 2 ? (mid.length - 1) / (mid[mid.length - 1] - mid[0]) : 0;
    // (and its best second: as many shot in any one second as it manages, the load still coming)
    const best = s.reduce((m, x, i) => Math.max(m, s.slice(i).filter((y) => y < x + 1).length), 0);
    console.log(`4414 dumps ${s.length} of ${n0}: ${in1} in the first second, ${best} in its best, ${span ? ((s.length - 1) / span).toFixed(1) : 0} FUEL/s over the whole dump, ${midRate.toFixed(1)} through its middle (their gap-free 12-14 BPS)`);
    // (ROTORLOG: the TTB stack's speed against its free speed while it feeds, and its X60s' current)
    if (process.env.ROTORLOG && r.rollLog?.length) {
      const L = r.rollLog.filter((q) => q[2] > 0 && q[2] < s.length), q = (a, f) => [...a].sort((x, y) => x - y)[Math.floor(f * (a.length - 1))];
      const rpm = L.map((x) => x[0]), A = L.map((x) => x[1]), free = RPM(r.t.rollers.motor.freeSpeed * r.t.ttb.k);
      console.log(`   the TTB stack while it feeds: free ${free.toFixed(0)} rpm; median ${q(rpm, 0.5).toFixed(0)} (${(100 * q(rpm, 0.5) / free).toFixed(0)}%), 10th percentile ${q(rpm, 0.1).toFixed(0)} rpm (${(100 * q(rpm, 0.1) / free).toFixed(0)}%); its X60s median ${q(A, 0.5).toFixed(0)} A, 90th ${q(A, 0.9).toFixed(0)} A, at 60 A ${(100 * A.filter((x) => x > 59).length / A.length).toFixed(0)}% of the time`);
    }
    return { n: s.length, n0, mid: midRate, best };
}

function pathRun(seed, { shoot = true, n = 20, sec = 8 } = {}) {
  let sd = seed;
  const rnd = () => ((sd = (sd * 16807) % 2147483647) / 2147483647 - 0.5);
  const rad = 480, hood = 140;
  const r = rig({ hood, yaw: Math.PI });
  const w = world({ carpet: false });
  const carpet = w.createRigidBody(RAPIER.RigidBodyDesc.kinematicVelocityBased().setLinvel(-SPEED, 0, 0));
  w.createCollider(RAPIER.ColliderDesc.cuboid(15, 0.01, 1).setTranslation(0.65 + 15, -0.01, 0).setFriction(0.3).setRestitution(0.2), carpet);
  w.createCollider(RAPIER.ColliderDesc.cuboid(0.65, 0.01, 1).setTranslation(0, -0.01, 0).setFriction(0.3), w.createRigidBody(RAPIER.RigidBodyDesc.fixed()));
  r.t.primary.cmd = 1; r.t.bar.cmd = 1;
  for (let i = 0; i < 2 / DT; i++) { if (shoot) launcher(r.t, rad); r.step([]); }
  const balls = [];
  for (let i = 0; i < n; i++) balls.push(ball(w, 1.0 + 0.17 * Math.floor(i / 4) + 0.02 * rnd(), R + 0.003, ((i % 4) - 1.5) * 0.17 + 0.02 * rnd(), [-SPEED, 0, 0]));
  const watch = shotWatch(r), walls = wallWatch(r);
  const timeline = [];
  let minPrimary = Infinity, minBar = Infinity;
  for (let k = 0; k < sec / DT; k++) {
    const live = balls.filter((b) => b.translation().y < 3);
    if (shoot) { launcher(r.t, rad); feed(r); } else feed(r, -0.15, 0);
    r.step(live); w.step();
    watch.check(live, k * DT); walls.check(live);
    if (k * DT > 0.3) { minPrimary = Math.min(minPrimary, r.t.primary.omega / r.t.primary.motor.freeSpeed); minBar = Math.min(minBar, r.t.bar.omega / r.t.bar.motor.freeSpeed); }
    if (k === Math.round((sec - 1) / DT)) { walls.filling = walls.report(); walls.reset(); }
    if (k % Math.round(1 / DT) === Math.round(1 / DT) - 1) timeline.push(balls.filter((b) => { const p = b.translation(); return p.y < 3 && p.x < HALF_L && p.y > 0.1; }).length);
    // (FRAMES=file: every 0.025 s for FRAMES_T s (2), the FUEL and the rotor's angle, for drawing)
    if (process.env.FRAMES && k % 3 === 0 && k * DT <= Number(process.env.FRAMES_T ?? 2)) (r.frames ??= []).push({ t: k * DT, phi: r.phi, balls: balls.map((b) => { const p = b.translation(); return [p.x, p.y, p.z]; }).filter((p) => p[1] < 3 && p[0] < 2) });
    if (process.env.TRACE && k % 6 === 0) {
      const b = balls[Number(process.env.TRACE)], p = b.translation(), v = b.linvel();
      if (p.y < 3) console.log(`   ${(k * DT).toFixed(2)} s at ${p.x.toFixed(3)},${p.y.toFixed(3)},${p.z.toFixed(3)} v ${v.x.toFixed(2)},${v.y.toFixed(2)},${v.z.toFixed(2)}: ${names(r, b)}`);
    }
  }
  if (process.env.FRAMES && r.frames) writeFileSync(process.env.FRAMES, JSON.stringify(r.frames));
  const shots = watch.shots.map((s) => s.t);
  const inside = balls.filter((b) => inRobot(b) && b.translation().y < 3 && !walls.escaped.has(b));
  if (process.env.BOXED) for (const b of inside) { const p = b.translation(); if (p.x > HALF_L) console.log(`   in the box at ${p.x.toFixed(3)},${p.y.toFixed(3)},${p.z.toFixed(3)}: ${names(r, b)}`); }
  const left = balls.filter((b) => b.translation().y < 3 && b.translation().x < 40 && !inside.includes(b)).map((b) => { const p = b.translation(); return [p.x, p.y, p.z]; });
  w.free();
  return { minPrimary, minBar, timeline, shots: shots.length, rate: shots.length > 2 ? (shots.length - 1) / (shots[shots.length - 1] - shots[0]) : 0, inside: inside.length, left, walls: `${walls.escapes()}; all the way, ${walls.filling}; its last second, ${walls.report()}` };
}

const pick = process.argv.slice(2);
for (const [name, fn] of Object.entries(tests)) if (!pick.length || pick.includes(name)) fn();
