// FUEL that end up on the carpet under a robot's own frame: how they got there. Plays headless AI
// matches (or intake runs) and, for every FUEL that comes to lie under a robot's frame perimeter
// (its center inside the frame in plan and low, under UNDER_Y in the robot's frame: on the carpet,
// not up in its ball path), logs its path in the robot's frame over the second before (each 0.05 s:
// where, whether the robot's mechanisms had hold of it (own), whether it was solid to the robot's
// colliders, what of the robot it touched: its mechanisms' wheels and surfaces by name, its Rapier
// colliders: bumper, frame, wheel, arm...), and sorts it into a route:
//   mouth     in over the frame's front edge, within the intake's width, while the intake had hold
//             of it (held FUEL don't meet the robot's colliders: only its mechanisms' surfaces stop
//             them there)
//   corner    in over the front edge outside the intake's width, or over a side or the back edge,
//             while owned or not solid to the robot
//   rideover  in over an edge while solid to the robot's colliders (the robot rode up over it, or
//             it was pushed through them)
//   tunnel    in over an edge in one step by more than a quarter of its diameter (relative to the
//             robot)
//   landed    (any edge) the robot pitched up (on a BUMP) came down over it while its intake had
//             hold of it: its center under the bumpers' bottom edge as it came in
//   riding    the robot up on FUEL (all its wheels off the carpet, level): over it, its bumpers'
//             bottom edge rides over the FUEL in front
//   dropped   came down from the ball path inside the frame (through its floor, or pushed down)
//   ghost     in over an edge while not solid to robots after being let go (released, shot)
// Then per robot: how many, and the FUEL-seconds under its frame (AUTO and TELEOP).
//   node --import ./tools/node-env.mjs tools/under-frame.mjs --pairs 341:2910,8793:2910 --seeds 1-2 [--quiet]
//   node --import ./tools/node-env.mjs tools/under-frame.mjs --intake 1.5 [--along] [--robots 341,8793] [--time 4]
//   --3v3 a,b,c:d,e,f   one 3 v 3 match (blue a,b,c against red d,e,f), all scorers at champs
// --quiet: only each episode's one-line summary (no paths); --json f: the episodes to a file
import { runMatch, runGame, createWorld, mulberry32 } from './headless.mjs';
import { Robot } from '../js/robot.js';
import { Match } from '../js/match.js';
import { ROBOTS } from '../js/robotConfigs.js';
import { PHYSICS_DT, BLUE, FUEL, GROUP } from '../js/constants.js';
import { BUMPER_Y0 } from '../js/robotConfigs.js';
import { writeFileSync } from 'fs';

// under: its center within the frame's perimeter (MARGIN past it: 2 cm into the bumper behind it)
// and lower than UNDER_Y (on the carpet, 0.075; 1678's tunnel under its hopper carries FUEL 0.10-0.11
// up), and its top over the bumpers' bottom edge (a FUEL lower than that, under a
// robot up on a BUMP, isn't under its frame: the robot overhangs it, and that's counted apart)
const R = FUEL.radius, UNDER_Y = 0.095, MARGIN = 0.02, KEEP = 1.0, SAMPLE = 0.05;
const args = process.argv.slice(2);
const arg = (k, d) => { const i = args.indexOf('--' + k); return i >= 0 ? args[i + 1] : d; };
const flag = (k) => args.includes('--' + k);
const QUIET = flag('quiet'), UNTIL = +arg('until', Infinity);
const range = (s) => s.split(',').flatMap((p) => { const [a, b] = p.split('-').map(Number); return b ? Array.from({ length: b - a + 1 }, (_, i) => a + i) : [a]; });

function qrotInv(q, v) {
  const x = -q.x, y = -q.y, z = -q.z, w = q.w;
  const ix = w * v[0] + y * v[2] - z * v[1], iy = w * v[1] + z * v[0] - x * v[2], iz = w * v[2] + x * v[1] - y * v[0], iw = -x * v[0] - y * v[1] - z * v[2];
  return [ix * w + iw * -x + iy * -z - iz * -y, iy * w + iw * -y + iz * -x - ix * -z, iz * w + iw * -z + ix * -y - iy * -x];
}

function qrot(q, v) { return qrotInv({ x: -q.x, y: -q.y, z: -q.z, w: q.w }, v); }

// One robot's watch over the FUEL near it. field: for the carpet's height under its wheels (how
// high it's riding: up on FUEL, its wheels off the carpet, or not)
function watcher(r, fuel, physics, field) {
  const world = physics.world, cfg = r.cfg, ic = cfg.intake;
  const hL = cfg.frame.length / 2 + MARGIN, hW = cfg.frame.width / 2 + MARGIN;
  // the robot's Rapier colliders, named (as tools/fuel-contacts.mjs)
  const named = [];
  for (let i = 0; i < r.body.numColliders(); i++) {
    const c = r.body.collider(i);
    let name = null;
    if (i === 0) name = 'bumper';
    else if (i === 1) name = 'frame';
    else if (c.handle === r.armCollider?.handle) name = ic.fold ? 'arm' : ic.slide ? 'slide' : 'intakeBlock';
    else if (c.handle === r.hopperCollider?.handle) name = 'hopperBox';
    else if (c.handle === r.lidCollider?.handle) name = 'lid';
    else if (c.handle === r.intakeCollider?.handle) name = 'roller';
    else if (r.sidePlates?.some((s) => s.handle === c.handle)) name = 'sidePlate';
    else if ((c.collisionGroups() >>> 16) & GROUP.WHEEL) name = 'wheel';
    if (name) named.push({ c, name });
  }
  const byCol = new Map(fuel.balls.map((b) => [b.col.handle, b]));
  const hist = new Map(); // ball -> [{t, l, own, solid, mech, rap}]
  const open = new Map(); // ball -> episode
  const episodes = [];
  const time = { auto: 0, teleop: 0, other: 0, overhang: 0 };
  let maxNow = 0;
  // the intake's half width (where its CAD's mouth is): the widest of its front wheels
  const ph = r.phys;
  const mouthHW = Math.max(ic.width / 2, ...((ph?.path?.wheels || []).filter((w) => w.center[0] > hL - 0.05 && w.center[1] < 0.3).map((w) => Math.abs(w.center[2]) + w.width / 2)));
  const wname = (w) => w.name || w.shaft?.name || w.kind || '?';

  function step(t, phase) {
    const tr = r.body.translation(), q = r.body.rotation();
    const local = (p) => qrotInv(q, [p.x - tr.x, p.y - tr.y, p.z - tr.z]);
    // what of the robot each FUEL touched this step
    const mech = new Map(), rap = new Map();
    if (ph) {
      for (const m of [ph.path, ph.shooter]) for (const c of m?.contacts || []) {
        const s = mech.get(c.ball) || new Set(); s.add(wname(c.wheel)); mech.set(c.ball, s);
      }
    }
    for (const { c, name } of named) {
      if (!c.collisionGroups()) continue;
      world.contactPairsWith(c, (o) => {
        const b = byCol.get(o.handle);
        if (!b) return;
        world.contactPair(c, o, (m) => {
          let d = Infinity;
          for (let i = 0; i < m.numContacts(); i++) d = Math.min(d, m.contactDist(i));
          if (d < 0.002) { const s = rap.get(b) || new Set(); s.add(name); rap.set(b, s); }
        });
      });
    }
    // (robot: speed, tilt, how high its bumper's front bottom edge is off the carpet)
    const v = r.body.linvel(), up = qrotInv(q, [0, 1, 0]);
    const tilt = Math.acos(Math.min(1, up[1])) * 57.3;
    // (the lowest wheel's height off the carpet or BUMP under it: over a couple of cm, the robot's
    // riding on something, FUEL or a robot)
    let ride = Infinity;
    for (const w of r.wheelPts || []) {
      const c = qrot(q, [w.x, 0, w.z]);
      ride = Math.min(ride, tr.y + c[1] - (field ? field.terrainHeight(tr.x + c[0], tr.z + c[2]) : 0));
    }
    let now = 0;
    for (const b of fuel.balls) {
      const H = hist.get(b);
      if (b.state !== 'field') { if (H) hist.delete(b); if (open.has(b)) close(b, t, 'gone'); continue; }
      const p = b.pos;
      if (Math.abs(p.x - tr.x) > 1.3 || Math.abs(p.z - tr.z) > 1.3 || p.y > 1.2) {
        if (H) hist.delete(b);
        if (open.has(b)) close(b, t, 'far');
        continue;
      }
      const l = local(p);
      const s = { t, l, w: [p.x, p.y, p.z], ry: tr.y, own: !!ph?.owned?.has(b), held: !!b.held, solid: !b.ignoring && !b.held, mech: mech.get(b.body), rap: rap.get(b), v: Math.hypot(v.x, v.z), tilt, ride, dep: r.intakeDeploy };
      const h = H || [];
      if (!H) hist.set(b, h);
      h.push(s);
      while (h.length && h[0].t < t - KEEP - 0.01) h.shift();
      const inPlan = Math.abs(l[0]) < hL && Math.abs(l[2]) < hW;
      // (once under, it stays under until it's 3 cm higher: one pressed up into the belly bobs about
      // UNDER_Y)
      const low = inPlan && l[1] < UNDER_Y + (open.has(b) ? 0.03 : 0);
      const under = low && l[1] + R > BUMPER_Y0 + 0.01;
      if (low && !under) time.overhang += PHYSICS_DT;
      if (under) {
        now++;
        time[phase in time ? phase : 'other'] += PHYSICS_DT;
        const e = open.get(b);
        if (e) {
          e.dur += PHYSICS_DT; e.minY = Math.min(e.minY, l[1]); e.maxY = Math.max(e.maxY, l[1]);
          e.tiltSum += tilt * PHYSICS_DT; e.ownT += s.own ? PHYSICS_DT : 0; e.solidT += s.solid ? PHYSICS_DT : 0; e.last = l;
          continue;
        }
        open.set(b, start(b, h, t, phase));
      } else if (open.has(b)) close(b, t, 'out');
    }
    maxNow = Math.max(maxNow, now);
  }

  function start(b, h, t, phase) {
    // where it came in: the last sample outside the frame in plan, or (inside) up in the ball path
    let k = h.length - 1;
    while (k > 0 && Math.abs(h[k - 1].l[0]) < hL && Math.abs(h[k - 1].l[2]) < hW && h[k - 1].l[1] < UNDER_Y) k--;
    const a = h[k - 1], c = h[k];
    let route, edge = '-';
    if (!a) route = 'unknown';
    else {
      const inPlanA = Math.abs(a.l[0]) < hL && Math.abs(a.l[2]) < hW;
      const jump = Math.hypot(c.l[0] - a.l[0], c.l[2] - a.l[2]);
      if (inPlanA) { route = 'dropped'; edge = `from y ${a.l[1].toFixed(3)}`; }
      else {
        const ex = Math.abs(a.l[0]) - hL, ez = Math.abs(a.l[2]) - hW;
        edge = ex > ez ? (a.l[0] > 0 ? 'front' : 'back') : (a.l[2] > 0 ? 'right' : 'left');
        if (jump > R / 2) route = 'tunnel';
        else if (a.ride > 0.025 && a.tilt < 4) route = 'riding';
        else if ((a.tilt > 4 && a.l[1] < 0.05) || a.l[1] < BUMPER_Y0) route = (a.solid ? 'rideover' : a.own || a.held ? 'landed' : 'ghost');
        else if (a.solid) route = 'rideover';
        else if (edge === 'front' && Math.abs(a.l[2]) < mouthHW && (a.own || a.held)) route = 'mouth';
        else if (a.own || a.held) route = 'corner';
        else route = 'ghost';
      }
    }
    // what it touched over the second before (and how many steps)
    const touched = {};
    for (const s of h) for (const n of [...(s.mech || []), ...[...(s.rap || [])].map((x) => 'R:' + x)]) touched[n] = (touched[n] || 0) + 1;
    const path = [];
    let next = -Infinity;
    for (const s of h) {
      if (s.t < next && s !== h[h.length - 1] && s !== a && s !== c) continue;
      next = s.t + SAMPLE;
      path.push(s);
    }
    return { robot: cfg.key, ball: b.id, t0: t, phase, route, edge, at: a?.l, into: c?.l, inState: a ? { own: a.own, held: a.held, solid: a.solid } : null, touched, path, dur: 0, minY: c.l[1], maxY: c.l[1], tiltSum: 0, ownT: 0, solidT: 0, last: c.l, robotV: c.v, tilt: c.tilt, ride: a ? a.ride : c.ride, dep: c.dep };
  }
  function close(b, t, why) {
    const e = open.get(b);
    open.delete(b);
    e.end = why;
    episodes.push(e);
  }
  function finish(t) { for (const b of [...open.keys()]) close(b, t, 'match end'); }
  return { step, finish, episodes, time, get maxNow() { return maxNow; }, key: cfg.key, mouthHW };
}

const f3 = (v) => v.map((u) => u.toFixed(3)).join(',');
function printEpisode(e, label) {
  const st = e.inState ? `${e.inState.own ? 'own' : ''}${e.inState.held ? ' held' : ''}${e.inState.solid ? ' solid' : ''}`.trim() || 'ghost' : '-';
  const top = Object.entries(e.touched).sort((a, b) => b[1] - a[1]).slice(0, 8).map(([n, c]) => `${n}x${c}`).join(' ');
  console.log(`  ${label} ${e.robot} FUEL ${e.ball} t ${e.t0.toFixed(2)} (${e.phase}) ${e.route} [${e.edge}] at ${e.at ? f3(e.at) : '-'} -> ${e.into ? f3(e.into) : '-'} (${st}) robot v ${e.robotV.toFixed(2)} tilt ${e.tilt.toFixed(1)} ride ${e.ride.toFixed(3)} deploy ${e.dep.toFixed(2)}; under ${e.dur.toFixed(2)} s (${e.end}; y ${e.minY.toFixed(3)}-${e.maxY.toFixed(3)}, tilt ${(e.tiltSum / (e.dur || 1)).toFixed(1)}, owned ${(e.ownT / (e.dur || 1) * 100).toFixed(0)}% solid ${(e.solidT / (e.dur || 1) * 100).toFixed(0)}%, last ${f3(e.last)}); touched ${top}`);
  if (QUIET) return;
  for (const s of e.path) {
    const fl = `${s.own ? 'O' : '-'}${s.held ? 'H' : '-'}${s.solid ? 'S' : '-'}`;
    console.log(`      ${(s.t - e.t0).toFixed(2).padStart(6)} ${f3(s.l)} ${fl} v ${s.v.toFixed(2)} tilt ${s.tilt.toFixed(1)} ride ${s.ride.toFixed(3)} world ${f3(s.w)} robot y ${s.ry.toFixed(3)} | ${[...(s.mech || [])].join(' ')}${s.rap ? ' | R:' + [...s.rap].join(',') : ''}`);
  }
}

function summarize(ws, label) {
  for (const w of ws) {
    const byRoute = {};
    for (const e of w.episodes) {
      const k = e.route + (e.route === 'dropped' || e.route === 'unknown' ? '' : '/' + e.edge);
      byRoute[k] ||= { n: 0, s: 0 };
      byRoute[k].n++; byRoute[k].s += e.dur;
    }
    const routes = Object.entries(byRoute).sort((a, b) => b[1].s - a[1].s).map(([k, v]) => `${k} ${v.n} (${v.s.toFixed(1)} s)`).join(', ');
    console.log(`${label} ${w.key}: ${w.episodes.length} in, ${w.time.auto.toFixed(1)} s AUTO + ${w.time.teleop.toFixed(1)} s TELEOP FUEL-under-frame (most at once ${w.maxNow}; overhung on a BUMP ${w.time.overhang.toFixed(1)} s); ${routes || '-'}`);
  }
}

const all = [];
async function matchRun(a, b, seed) {
  let ws = null;
  const trace = (game) => {
    if (!ws) ws = game.robots.map((r) => watcher(r, fuelOf(game), physOf(game), r.field));
    const m = game.match;
    for (const w of ws) w.step(m.t, m.phase);
    if (m.t > UNTIL) throw new Error('until');
  };
  // (the world, for the watchers: game has fuel; physics via the robots)
  const fuelOf = (g) => g.robots[0].fuel, physOf = (g) => g.robots[0].physics;
  const t0 = Date.now();
  let res;
  try {
    if (b === null) res = await runGame(a, { seed, trace });
    else res = await runMatch({ strategy: 'scorer', skill: 'champs', robot: a }, { strategy: 'scorer', skill: 'champs', robot: b }, { seed, trace });
  } catch (err) { if (err.message !== 'until') throw err; res = { a: { total: '-' }, b: { total: '-' } }; }
  for (const w of ws) w.finish();
  const label = b === null ? `3v3 seed ${seed}` : `${a} v ${b} seed ${seed}`;
  for (const w of ws) for (const e of w.episodes) { printEpisode(e, label); all.push({ ...e, match: label }); }
  summarize(ws, label);
  console.log(`  (${((Date.now() - t0) / 1000).toFixed(0)} s; ${JSON.stringify(res.a ? { a: res.a.total, b: res.b.total } : { blue: res.blue.total, red: res.red.total })})`);
  return ws;
}

async function intakeRun(key, speed, along, T, settle) {
  const world = await createWorld();
  Math.random = mulberry32(7);
  const { physics, scene, field, fuel } = world;
  const match = new Match({ playerAlliance: BLUE, onEvent: () => {} });
  fuel.match = match;
  match.phase = 'teleop';
  const r = new Robot({ cfg: ROBOTS[key], alliance: BLUE, physics, scene, field, fuel, match, climber: null });
  if (along) r.spawn(-0.45, 3.4, Math.PI / 2); else r.spawn(-2.4, -1.5, 0);
  fuel.stage(0);
  r.enabled = true;
  const w = watcher(r, fuel, physics, field);
  let t = 0;
  for (let i = 0; i < (settle + T) / PHYSICS_DT; i++) {
    const go = t >= settle;
    Object.assign(r.cmd, { vx: go && !along ? speed : 0, vz: go && along ? -speed : 0, omega: 0, intake: true, shoot: false, pass: false, outtake: false });
    r.preStep(PHYSICS_DT); fuel.preStep(PHYSICS_DT); physics.step(); t += PHYSICS_DT; fuel.postStep(PHYSICS_DT, t); r.postStep(PHYSICS_DT, t);
    if (i % 2) fuel.update(2 * PHYSICS_DT, t);
    w.step(t, 'teleop');
  }
  w.finish(t);
  const label = `intake ${speed} ${along ? 'along' : 'across'}`;
  for (const e of w.episodes) { printEpisode(e, label); all.push({ ...e, match: label }); }
  console.log(`${label} ${key}: held ${r.stored.length}`);
  summarize([w], label);
  world.physics.world.free();
}

if (arg('intake')) {
  const keys = arg('robots', '2910,4414,8793,1678,1690,341,1706,581,1114,604,6328,7769,2056').split(',');
  for (const k of keys) await intakeRun(k, +arg('intake'), flag('along'), +arg('time', 4), +arg('settle', 0));
} else if (arg('3v3')) {
  const [blue, red] = arg('3v3').split(':').map((s) => s.split(','));
  const slot = (robot) => ({ robot, auto: 'best', start: 'hub', driver: 'scorer', skill: 'champs' });
  for (const seed of range(arg('seeds', '1'))) {
    await matchRun({ matchMode: '3v3', slots: [...blue, ...red].map(slot) }, null, seed);
  }
} else {
  const pairs = arg('pairs', '341:2910').split(',').map((p) => p.split(':'));
  for (const seed of range(arg('seeds', '1'))) for (const [a, b] of pairs) await matchRun(a, b, seed);
}
if (arg('json')) writeFileSync(arg('json'), JSON.stringify(all.map(({ path, ...e }) => ({ ...e, path: path.map((s) => ({ ...s, mech: s.mech ? [...s.mech] : undefined, rap: s.rap ? [...s.rap] : undefined })) }))));
