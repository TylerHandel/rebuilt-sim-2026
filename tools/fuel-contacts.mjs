// Loose FUEL against a robot's own simplified colliders (js/robot.js _createBody): the bumpers,
// the frame over them, the wheels, the sliding hopper box (storage extension), the intake's hull
// (8793's/604's/341's arm, 581's slide) or block, its lid. A FUEL its mechanisms have hold of
// (fuel.own, HELD) doesn't meet these: what it meets is its CAD's surfaces (js/mech/physical.js),
// so a contact here is one the intake hasn't taken yet, or one beside or under it.
// Each step (after the physics step) it reads Rapier's contact graph for each of those colliders;
// each FUEL's first touch starts a 0.5 s watch:
//   taken:   its mechanisms took hold of it (owned) within 0.5 s
//   wedged:  still touching the collider at the end, hardly moving against the robot
//   stalled: not taken, still at the mouth (in reach of the intake's rollers), hardly moving
//   pushed:  not taken, and not at the mouth (pushed off, or it went by)
// and whether, while touching it, an intake roller was on it too (pinched between the two), and
// whether it was in reach of the intake's rollers (within REACH of a front roller's tread) when it
// first touched. The robot's full (stalled against the load) during the watch is noted.
//   import { contactLog } from './fuel-contacts.mjs'; const log = contactLog(robot, fuel, physics);
//   ... each step after physics.step(): log.step(t); at the end: console.log(log.report())
// (the intake test: tools/intake-test.mjs --contacts; the AUTOs: tools/team-auto-eval.mjs --contacts)
import { Wheel } from '../js/mech/mechanism.js';
import { FUEL, GROUP } from '../js/constants.js';
import { BUMPER_T } from '../js/robotConfigs.js';

const R = FUEL.radius, WATCH = 0.5, REACH = 0.03, BIN = 0.025;

function qrotInv(q, v) {
  // rotate v by the inverse of the unit quaternion q
  const x = -q.x, y = -q.y, z = -q.z, w = q.w;
  const ix = w * v[0] + y * v[2] - z * v[1], iy = w * v[1] + z * v[0] - x * v[2], iz = w * v[2] + x * v[1] - y * v[0], iw = -x * v[0] - y * v[1] - z * v[2];
  return [ix * w + iw * -x + iy * -z - iz * -y, iy * w + iw * -y + iz * -x - ix * -z, iz * w + iw * -z + ix * -y - iy * -x];
}

export function contactLog(robot, fuel, physics) {
  const r = robot, world = physics.world, cfg = r.cfg, ic = cfg.intake;
  const halfF = cfg.frame.length / 2, halfW = cfg.frame.width / 2;
  // the robot's colliders, named
  const named = new Map();
  const n = r.body.numColliders();
  for (let i = 0; i < n; i++) {
    const c = r.body.collider(i);
    let name = null;
    if (i === 0) name = 'bumper';
    else if (i === 1) name = 'frame';
    else if (c === r.armCollider || c.handle === r.armCollider?.handle) name = ic.fold ? 'arm' : ic.slide ? 'slide' : 'intakeBlock';
    else if (c.handle === r.hopperCollider?.handle) name = 'hopperBox';
    else if (c.handle === r.lidCollider?.handle) name = 'lid';
    else if (c.handle === r.intakeCollider?.handle) name = 'roller';
    else if (r.sidePlates?.some((s) => s.handle === c.handle)) name = 'sidePlate';
    else if ((c.collisionGroups() >>> 16) & GROUP.WHEEL) name = 'wheel';
    if (name) named.set(c.handle, { c, name });
  }
  const byCol = new Map(fuel.balls.map((b) => [b.col.handle, b]));
  const watch = new Map(); // ball -> episode
  const done = [];
  const heat = new Map(); // collider -> Map(bin key -> count) side view (x, y) and top view (x, z)

  const pose = () => ({ t: r.body.translation(), q: r.body.rotation(), v: r.body.linvel(), w: r.body.angvel() });
  const local = (P, p) => qrotInv(P.q, [p.x - P.t.x, p.y - P.t.y, p.z - P.t.z]);
  // the FUEL's velocity relative to the robot's at its center, robot frame
  const relVel = (P, b) => {
    const p = b.body.translation(), v = b.body.linvel(), d = [p.x - P.t.x, p.y - P.t.y, p.z - P.t.z], w = P.w;
    const vr = [P.v.x + w.y * d[2] - w.z * d[1], P.v.y + w.z * d[0] - w.x * d[2], P.v.z + w.x * d[1] - w.y * d[0]];
    return qrotInv(P.q, [v.x - vr[0], v.y - vr[1], v.z - vr[2]]);
  };
  // the intake's front rollers: the ball path's wheels out front and low (robot frame, posed)
  const frontWheels = () => (r.phys?.path?.wheels || []).filter((w) => w.center[0] > halfF - 0.05 && w.center[1] < 0.3);
  const rollerGap = (pl) => {
    let g = Infinity;
    for (const w of frontWheels()) {
      const d = [pl[0] - w.center[0], pl[1] - w.center[1], pl[2] - w.center[2]];
      const t = Math.max(-w.width / 2, Math.min(w.width / 2, d[0] * w.axis[0] + d[1] * w.axis[1] + d[2] * w.axis[2]));
      const rad = Math.hypot(d[0] - w.axis[0] * t, d[1] - w.axis[1] * t, d[2] - w.axis[2] * t);
      g = Math.min(g, Math.max(0, rad - w.radius - R) + Math.max(0, Math.abs(d[0] * w.axis[0] + d[1] * w.axis[1] + d[2] * w.axis[2]) - w.width / 2 - R));
    }
    return g;
  };
  // the mouth: across the intake's width, from the frame's front out past the bumpers
  const front = ic.front ?? (r.halfL + ic.reach);
  const atMouth = (pl) => pl[0] > halfF - 0.02 && pl[0] < Math.max(front, r.halfL) + R + 0.05 && Math.abs(pl[2]) < ic.width / 2 + R && pl[1] < 0.45;
  const rollerOn = (b) => (r.phys?.path?.contacts || []).some((c) => c.ball === b.body && c.wheel instanceof Wheel);
  const owned = (b) => !!r.phys?.owned?.has(b) || b.held;

  function close(e, t, why) {
    const P = pose(), b = e.b, pl = local(P, b.body.translation()), v = relVel(P, b);
    const sp = Math.hypot(v[0], v[1], v[2]);
    if (!why) {
      if (e.taken) why = 'taken';
      else if (b.state !== 'field') why = 'gone';
      else if (e.lastTouch >= t - 2 * 1 / 120 && sp < 0.15) why = 'wedged';
      else if ((atMouth(pl) || rollerGap(pl) < REACH) && sp < 0.15) why = 'stalled';
      else why = 'pushed';
    }
    e.outcome = why;
    if (process.env.CONTACTLOG) console.log(`    ${e.t0.toFixed(2)} ${e.name} at ${e.pt.map((u) => u.toFixed(3)).join(',')} center ${e.c.map((u) => u.toFixed(3)).join(',')} v ${e.v.map((u) => u.toFixed(2)).join(',')} deploy ${e.dep.toFixed(2)} gap ${e.gap.toFixed(3)} ${e.mouth ? 'mouth ' : ''}${e.steps} steps${e.pinch ? ` pinched ${e.pinch}` : ''} -> ${why} after ${(t - e.t0).toFixed(2)} s, then at ${pl.map((u) => u.toFixed(3)).join(',')}`);
    done.push(e);
    watch.delete(b);
  }

  return {
    step(t) {
      const P = pose();
      // what's touching each collider now
      const touching = new Map(); // ball -> [name, point(local), depth]
      for (const { c, name } of named.values()) {
        if (!c.collisionGroups()) continue;
        world.contactPairsWith(c, (o) => {
          const b = byCol.get(o.handle);
          if (!b || b.state !== 'field' || owned(b)) return;
          world.contactPair(c, o, (m, flipped) => {
            let dmin = Infinity;
            for (let i = 0; i < m.numContacts(); i++) dmin = Math.min(dmin, m.contactDist(i));
            if (!(dmin < 0.002)) return;
            const nrm = m.normal(), s = flipped ? 1 : -1, p = b.body.translation();
            // (the normal runs from the first collider to the second: the point is on the FUEL's
            // side facing the robot's collider)
            const cp = { x: p.x + s * nrm.x * R, y: p.y + s * nrm.y * R, z: p.z + s * nrm.z * R };
            const prev = touching.get(b);
            if (!prev || dmin < prev.d) touching.set(b, { name, pt: local(P, cp), d: dmin });
          });
        });
      }
      for (const [b, h] of touching) {
        let e = watch.get(b);
        const pl = local(P, b.body.translation());
        if (!e) {
          const v = relVel(P, b);
          e = { b, t0: t, name: h.name, pt: h.pt, c: pl, dep: r.intakeDeploy, v, gap: rollerGap(pl), mouth: atMouth(pl), steps: 0, pinch: 0, lastTouch: t, taken: false, full: false, names: new Set() };
          watch.set(b, e);
          const hm = heat.get(h.name) || { side: new Map(), top: new Map() };
          heat.set(h.name, hm);
          const ks = `${Math.floor(h.pt[0] / BIN)},${Math.floor(h.pt[1] / BIN)}`, kt = `${Math.floor(h.pt[0] / BIN)},${Math.floor(h.pt[2] / BIN)}`;
          hm.side.set(ks, (hm.side.get(ks) || 0) + 1);
          hm.top.set(kt, (hm.top.get(kt) || 0) + 1);
        }
        e.steps++;
        e.lastTouch = t;
        e.names.add(h.name);
        if (rollerOn(b)) e.pinch++;
      }
      for (const e of [...watch.values()]) {
        if (owned(e.b)) e.taken = true;
        if (r.full) e.full = true;
        if (e.taken || t - e.t0 >= WATCH || e.b.state !== 'field') close(e, t);
      }
    },
    finish(t) { for (const e of [...watch.values()]) close(e, t); },
    events: done,
    // a table per collider: how many FUEL touched it (at the mouth / anywhere), what became of them
    report({ heatmap = true } = {}) {
      const lines = [];
      const names = [...new Set(done.map((e) => e.name))];
      const outs = ['taken', 'stalled', 'wedged', 'pushed', 'gone'];
      lines.push(`  contacts by first collider touched (mouth: in reach of a front roller, ${REACH * 100} cm, or in front of the frame across the intake):`);
      lines.push(`    ${'collider'.padEnd(12)} ${'FUEL'.padStart(5)} ${'mouth'.padStart(6)}  ${outs.map((o) => o.padStart(8)).join('')}  ${'pinched'.padStart(8)} ${'full'.padStart(5)}  mouth: ${outs.map((o) => o.padStart(8)).join('')}  median x,y,|z| (mouth)  rel v (mouth)`);
      for (const nm of names) {
        const es = done.filter((e) => e.name === nm), ms = es.filter((e) => e.mouth || e.gap < REACH);
        const cnt = (arr, o) => arr.filter((e) => e.outcome === o).length;
        const med = (arr, f) => { const a = arr.map(f).sort((p, q) => p - q); return a.length ? a[a.length >> 1] : NaN; };
        const pos = ms.length ? `${med(ms, (e) => e.pt[0]).toFixed(3)},${med(ms, (e) => e.pt[1]).toFixed(3)},${med(ms, (e) => Math.abs(e.pt[2])).toFixed(3)}` : '-';
        const vel = ms.length ? `${med(ms, (e) => e.v[0]).toFixed(2)},${med(ms, (e) => e.v[1]).toFixed(2)}` : '-';
        lines.push(`    ${nm.padEnd(12)} ${String(es.length).padStart(5)} ${String(ms.length).padStart(6)}  ${outs.map((o) => String(cnt(es, o)).padStart(8)).join('')}  ${String(es.filter((e) => e.pinch).length).padStart(8)} ${String(es.filter((e) => e.full).length).padStart(5)}  mouth: ${outs.map((o) => String(cnt(ms, o)).padStart(8)).join('')}  ${pos.padEnd(22)} ${vel}`);
      }
      if (heatmap) {
        for (const nm of names) {
          const ms = done.filter((e) => e.name === nm && (e.mouth || e.gap < REACH));
          if (!ms.length) continue;
          // side view (x forward across, y up) of where FUEL at the mouth touched it, and what
          // became of them: counts per 2.5 cm cell, a letter where most weren't taken
          lines.push(`    ${nm} at the mouth, side view (x ->, y ^; 2.5 cm cells; digit = FUEL, * = 10+, lowercase s/w/p = most stalled/wedged/pushed):`);
          const cell = new Map();
          for (const e of ms) {
            const k = `${Math.floor(e.pt[0] / BIN)},${Math.floor(e.pt[1] / BIN)}`;
            const c = cell.get(k) || { n: 0, o: {} };
            c.n++; c.o[e.outcome] = (c.o[e.outcome] || 0) + 1; cell.set(k, c);
          }
          const xs = ms.map((e) => Math.floor(e.pt[0] / BIN)), ys = ms.map((e) => Math.floor(e.pt[1] / BIN));
          const x0 = Math.min(...xs), x1 = Math.max(...xs), y0 = Math.min(...ys), y1 = Math.max(...ys);
          for (let y = y1; y >= y0; y--) {
            let row = '';
            for (let x = x0; x <= x1; x++) {
              const c = cell.get(`${x},${y}`);
              if (!c) { row += '.'; continue; }
              const bad = Object.entries(c.o).filter(([o]) => o !== 'taken' && o !== 'gone').sort((p, q) => q[1] - p[1]);
              const badN = bad.reduce((s, [, k]) => s + k, 0);
              row += badN * 2 > c.n ? bad[0][0][0] : c.n >= 10 ? '*' : String(c.n);
            }
            lines.push(`      y ${(y * BIN).toFixed(3).padStart(6)} ${row}`);
          }
          lines.push(`      x from ${(x0 * BIN).toFixed(3)} to ${((x1 + 1) * BIN).toFixed(3)} (frame front ${halfF.toFixed(3)}, bumper front ${(halfF + BUMPER_T).toFixed(3)}, intake front ${front.toFixed(3)})`);
        }
      }
      return lines.join('\n');
    },
  };
}
