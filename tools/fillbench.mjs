// A bench fill through the intake (tools/mechlab2910.mjs and mechlab1678.mjs intakefull): the
// robot's mechanisms fixed in the world, driving at speed into a field of FUEL laid out as the
// NEUTRAL ZONE's (a FUEL every 0.1535 m across, rows 2r + 4 mm apart: js/field.js neutralSlots)
// on a treadmill carpet, its intake running as its code runs it, until its intake can't put more
// in. Counts what it holds each step, and goes full as a robot in play does (js/mech/physical.js
// FullWatch: FUEL waiting in its mouth pressed against the load, none getting in, for FULL_T).
import { FUEL } from '../js/constants.js';
import { FullWatch, mouthPressed, topCount } from '../js/mech/physical.js';

const R = FUEL.radius;
// world(): a world without a carpet; ball(w, x, y, z, v): a FUEL; path: the ball path's Mechanism;
// held(p) / own(p): a FUEL at p (robot frame) is in it / the intake has hold of it; control(t,
// balls): its code, once a step (staging, prefill); front: the bumper's face (x), where the FUEL
// start 0.45 m ahead of; top: its naturalFull ({ topX, topN }: js/mech/physical.js)
// until(full, t): stop there (sec at most); keep: the world isn't freed, and comes back with its
// FUEL and the carpet (to go on with it: a dump)
export function fillRun(RAPIER, { world, ball, path, DT, held, own, top, control = () => {}, speed = 1, sec = 12, front = 0.45, seed = 1, rows = 70, until = null, keep = false }) {
  let sd = seed;
  const rnd = () => ((sd = (sd * 16807) % 2147483647) / 2147483647 - 0.5);
  const w = world();
  const carpet = w.createRigidBody(RAPIER.RigidBodyDesc.kinematicVelocityBased().setLinvel(-speed, 0, 0));
  w.createCollider(RAPIER.ColliderDesc.cuboid(20, 0.01, 1).setTranslation(front + 20, -0.01, 0).setFriction(0.3).setRestitution(0.2), carpet);
  w.createCollider(RAPIER.ColliderDesc.cuboid(0.6, 0.01, 1).setTranslation(front - 0.6, -0.01, 0).setFriction(0.3), w.createRigidBody(RAPIER.RigidBodyDesc.fixed()));
  // a FUEL every 0.1535 m across the intake's width (5 abreast, the outer two half on it), rows
  // 2r + 4 mm apart, a few mm of jitter (the seed)
  const balls = [];
  for (let i = 0; i < rows; i++) for (let k = -2; k <= 2; k++) balls.push(ball(w, front + 0.45 + (2 * R + 0.004) * i + 0.004 * rnd(), R + 0.002, 0.1535 * k + 0.02 * (i % 2 ? 0.5 : -0.5) + 0.004 * rnd(), [-speed, 0, 0]));
  const watch = new FullWatch(), counts = [], fulls = [], tops = [];
  let wasFull = false;
  for (let k = 0; k < sec / DT; k++) {
    const t = k * DT;
    const live = balls.filter((b) => { const p = b.translation(); return p.y > -1 && p.x < front + 0.6; });
    control(t, live);
    path.step(DT, live, live);
    w.step();
    // (the ones it's let go of, carried off behind it: out of the way)
    for (const b of live) { const p = b.translation(); if (p.x < -0.8 || p.y < -0.5) b.setTranslation({ x: 50, y: -5, z: 0 }, true); }
    const pts = live.map((b) => { const p = b.translation(); return [p.x, p.y, p.z]; });
    const load = pts.filter((p) => held(p)), mouth = pts.filter((p) => own(p) && !held(p));
    const inLoad = new Set(live.filter((_, i) => held(pts[i])));
    counts.push(load.length);
    const waiting = mouthPressed(mouth, load), nTop = topCount(path.contacts, [0, 1, 0], (b) => inLoad.has(b), (b) => b.translation().x, top.topX), packed = waiting && nTop >= top.topN;
    tops.push(nTop);
    const full = watch.step(DT, load.length, waiting, packed);
    if (full && !wasFull) fulls.push({ t, n: load.length, i: k, by: watch.by });
    wasFull = full;
    if (until && until(full, t)) break;
  }
  if (!keep) w.free();
  // (held a quarter second: not a FUEL bobbing over the line for a step)
  const q = Math.round(0.25 / DT), kept = counts.map((_, i) => Math.min(...counts.slice(i, i + q)));
  const most = Math.max(...kept), iMost = kept.indexOf(most), i10 = kept.findIndex((n) => n >= most - 10);
  const early = fulls.filter((f) => Math.max(...kept.slice(f.i)) >= f.n + 2);
  const last = fulls[fulls.length - 1];
  return { most, tMost: iMost * DT, last10: (iMost - i10) * DT, fulls, early: early.length, fullAfter: last ? last.t - iMost * DT : null, end: counts[counts.length - 1], timeline: kept.filter((_, i) => i % Math.round(1 / DT) === Math.round(1 / DT) - 1), tops: tops.filter((_, i) => i % Math.round(1 / DT) === Math.round(1 / DT) - 1), ...(keep ? { world: w, balls, carpet } : {}) };
}
export function fillReport(name, r) {
  return `${name}: the most it held ${r.most} (at ${r.tMost.toFixed(2)} s; its last 10 took ${r.last10.toFixed(2)} s), ${r.end} at the end; each second ${r.timeline.join(' ')} (pressed against the top: ${r.tops.join(' ')}); ` +
    (r.fulls.length ? `full at ${r.fulls.map((f) => `${f.t.toFixed(2)} s (${f.n}, ${f.by})`).join(', ')}, ${r.fullAfter.toFixed(2)} s after it first held ${r.most}${r.early ? `, ${r.early} early (it took 2+ more after)` : ''}` : 'never full');
}
