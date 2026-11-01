// What each robot does with its match: an observer for headless matches (tools/headless.mjs's
// trace callback) that only reads the game, so a match plays the same with it on or off.
//   const au = makeAudit();  runMatch(a, b, { trace: au.trace });  au.report()  -> one row per robot
// Per robot (tools/ai-audit.mjs prints and sums them; tools/match.mjs prints them with AUDIT=1):
//   pts       FUEL it put in its own HUB (by the launch's robot: rid), by period: auto, trans
//             (TRANSITION SHIFT), a1 / a2 (its HUB's first and second active SHIFT), end (END GAME);
//             inactive = FUEL it put in while the HUB was off (wasted)
//   launched, passes, intaked, unjams, unsticks (its AI backing out of being stuck), fouls (its ALLIANCE's)
//   sec       TELEOP seconds: idle (nothing taken in or launched for 2 s+, while not staged or climbing,
//             counted from the 2nd s), staged (waiting at home for its HUB), stuck (asked to move over
//             1 m/s, moving under 0.2) and what it's against (stuckBy: trench, tower, hub, skirt, bump,
//             depot, wall, robot, open), holdIdle (HUB active, holding FUEL, trying to score, nothing out for
//             3 s+), carryOut (HUB active, holding 5+, out of its zone), inZoneOff (its HUB off, in its
//             zone, not staged), emptyActive (HUB active, empty, in its zone)
//   idleBy    idle seconds by what its AI said it was doing (its label)
//   coll/took TELEOP seconds collecting (its AI's collect or feed) and FUEL taken in, by where it was:
//             own (its ALLIANCE ZONE), mid (the NEUTRAL ZONE), opp (the other ALLIANCE's ZONE)
//   labels    TELEOP seconds by label (early: the first 35 s of it); cycles (trips to score), firstScore (TELEOP s to its first FUEL)
//   stuckEp   the longest stuck stretches: { t, s, by, x, z, label }
import { PHYSICS_DT, HALF_L, HALF_W, HUB, TIMING, BUMP, DEPOT } from '../js/constants.js';
import { Field } from '../js/field.js';
import { OBSTACLES, underTrench } from '../js/nav.js';

const EVERY = 6; // sample every 6 steps (20 Hz)
const DT = EVERY * PHYSICS_DT;
const key = (label) => String(label || '-').replace(/\s*\(.*?\)/g, '').replace(/\s[\d.]+ s$/, '').replace(/…/g, '').trim();

// what a robot at (x, z) is up against
function against(r, others) {
  const R = Math.max(r.halfL, r.halfW) + 0.35;
  for (const o of others) if (Math.hypot(o.pos.x - r.pos.x, o.pos.z - r.pos.z) < R + Math.max(o.halfL, o.halfW) + 0.1) return 'robot';
  const { x, z } = r.pos;
  if (underTrench(x, z, 0.4)) return 'trench';
  for (const o of OBSTACLES) {
    if (Math.abs(x - o.x) < o.hx + R && Math.abs(z - o.z) < o.hz + R) return o.kind === 'trench' ? 'skirt' : o.kind;
  }
  // the DEPOT, its barriers along its ALLIANCE WALL
  const dz = HALF_W - DEPOT.fy, dx = HALF_L - DEPOT.depth;
  if (Math.abs(x) > dx - R && Math.abs(z - (x < 0 ? dz : -dz)) < DEPOT.width / 2 + R) return 'depot';
  if (Math.abs(x) > HALF_L - R || Math.abs(z) > HALF_W - R) return 'wall';
  // a BUMP: beside each HUB, between it and the TRENCH skirts
  const hx = Field.hubCenter('blue').x;
  if (Math.abs(Math.abs(x) - Math.abs(hx)) < BUMP.depth / 2 + R && Math.abs(z) > HUB.size / 2 - 0.1 && Math.abs(z) < HUB.size / 2 + BUMP.width + 0.2) return 'bump';
  return 'open';
}

export function makeAudit() {
  let game = null, m = null;
  const rows = new Map(); // robot uid -> row
  const row = (r) => rows.get(r.uid);

  function install(g) {
    game = g; m = g.match;
    for (const u of g.units) {
      const r = u.robot;
      rows.set(r.uid, {
        u, robot: r.cfg.key, alliance: r.alliance, station: (u.entry.station ?? 0) + 1, driver: u.entry.driver,
        pts: { auto: 0, trans: 0, a1: 0, a2: 0, end: 0, inactive: 0 },
        sec: { teleop: 0, idle: 0, staged: 0, stuck: 0, holdIdle: 0, carryOut: 0, inZoneOff: 0, emptyActive: 0 },
        // TELEOP seconds collecting, and FUEL taken in, in its own ALLIANCE ZONE / the NEUTRAL ZONE / the other's
        coll: { own: 0, mid: 0, opp: 0 }, took: { own: 0, mid: 0, opp: 0 },
        stuckBy: {}, idleBy: {}, labels: {}, early: {}, stuckEp: [], cycles: 0, unsticks: 0, firstScore: null,
        last: { t: 0, intaked: r.stats.intaked, shots: r.stats.shots, prog: 0, state: null, label: null, stuck: null, outT: 0 },
      });
    }
    // which robot scored: wrap the HUB's scoring (the launch names its robot: rid)
    const orig = m.fuelEnteredHub.bind(m);
    m.fuelEnteredHub = (a, ball, t) => {
      const before = m.score[a].autoFuel + m.score[a].teleFuel, offBefore = m.score[a].inactiveFuel;
      const rid = ball.launch && ball.launch.by === 'robot' && ball.launch.alliance === a ? ball.launch.rid : null;
      const per = m.phase === 'teleop' ? m.shiftIndex() : -1, active = m.hubActive(a);
      orig(a, ball, t);
      const x = rid != null && rows.get(rid);
      if (!x) return;
      if (m.score[a].inactiveFuel > offBefore) { x.pts.inactive++; return; }
      if (m.score[a].autoFuel + m.score[a].teleFuel === before) return;
      let k;
      if (m.phase !== 'teleop') k = 'auto';
      else {
        // in the 3 s after it turned off: the SHIFT that just ended
        const s = active ? per : per - 1;
        if (s <= 0) k = 'trans'; else if (s === 5) k = 'end';
        else k = activeOrdinal(a, s) === 1 ? 'a1' : 'a2';
        if (x.firstScore == null) x.firstScore = +m.phaseTime.toFixed(1);
      }
      x.pts[k]++;
    };
  }
  // the n-th of its HUB's active SHIFTS (1-4) that SHIFT s is
  const activeOrdinal = (a, s) => ((s % 2 === 1) === (a !== m.firstInactive) ? (s <= 2 ? 1 : 2) : 0);

  function trace(g, step) {
    if (!game) install(g);
    if (step % EVERY) return;
    if (!m.isTeleop) {
      if (m.phase === 'autoGap') for (const x of rows.values()) Object.assign(x.last, { intaked: x.u.robot.stats.intaked, shots: x.u.robot.stats.shots });
      return;
    }
    const T = m.phaseTime;
    for (const x of rows.values()) {
      const { u } = x, r = u.robot, ai = u.ai, L = x.last, s = x.sec;
      const others = game.robots.filter((o) => o !== r);
      s.teleop += DT;
      const label = key(ai?.label), cmd = r.cmd || {};
      x.labels[label] = (x.labels[label] || 0) + DT;
      // (the first 35 s of TELEOP, TRANSITION and SHIFT 1, apart: one ALLIANCE's HUB is on throughout)
      if (T < 35) x.early[label] = (x.early[label] || 0) + DT;
      const where = Field.inAllianceZone(r.alliance, r.pos.x) ? 'own' : Field.inAllianceZone(r.alliance === 'blue' ? 'red' : 'blue', r.pos.x) ? 'opp' : 'mid';
      if (ai && (ai.state === 'collect' || ai.state === 'feed')) x.coll[where] += DT;
      x.took[where] += r.stats.intaked - L.intaked;
      if (r.stats.intaked !== L.intaked || r.stats.shots !== L.shots) { L.prog = T; L.intaked = r.stats.intaked; L.shots = r.stats.shots; }
      if (ai && ai.state !== L.state) { if (ai.state === 'score') x.cycles++; L.state = ai.state; }
      if (label === 'Unsticking' && L.label !== 'Unsticking') x.unsticks++;
      L.label = label;
      const active = m.hubActive(r.alliance), stored = r.stored.length, inZone = r.lastInZone;
      const staged = /^Staged|^Hanging|^Climbing|^Going to climb/.test(label);
      if (staged) s.staged += DT;
      if (T - L.prog > 2 && !staged) { s.idle += DT; x.idleBy[label] = (x.idleBy[label] || 0) + DT; }
      // stuck: asked to move, not moving
      const want = Math.hypot(cmd.vx || 0, cmd.vz || 0), have = Math.hypot(r.vel.x, r.vel.z);
      if (want > 1.0 && have < 0.2) {
        const by = against(r, others);
        s.stuck += DT; x.stuckBy[by] = (x.stuckBy[by] || 0) + DT;
        if (!L.stuck) L.stuck = { t: +T.toFixed(1), s: 0, by, x: +r.pos.x.toFixed(2), z: +r.pos.z.toFixed(2), label };
        L.stuck.s += DT;
      } else if (L.stuck) { if (L.stuck.s >= 1) x.stuckEp.push(L.stuck); L.stuck = null; }
      if (active && stored > 0 && ai?.state === 'score' && (cmd.shoot || cmd.pass)) {
        if (r.stats.shots !== L.outShots) { L.outShots = r.stats.shots; L.outT = T; }
        if (T - L.outT > 3) s.holdIdle += DT;
      } else { L.outT = T; L.outShots = r.stats.shots; }
      if (active && stored >= 5 && !inZone) s.carryOut += DT;
      if (!active && inZone && !staged) s.inZoneOff += DT;
      if (active && inZone && stored === 0) s.emptyActive += DT;
    }
  }

  function report() {
    const out = [];
    for (const x of rows.values()) {
      const r = x.u.robot, L = x.last;
      if (L.stuck && L.stuck.s >= 1) x.stuckEp.push(L.stuck);
      const r1 = (o) => Object.fromEntries(Object.entries(o).filter(([, v]) => v >= 0.05).map(([k, v]) => [k, +v.toFixed(1)]));
      out.push({
        robot: x.robot, alliance: x.alliance, station: x.station, driver: x.driver,
        pts: x.pts, launched: r.stats.shots, passes: r.stats.passes, intaked: r.stats.intaked, unjams: r.stats.unjams || 0,
        unsticks: x.unsticks, cycles: x.cycles, firstScore: x.firstScore, fouls: m.score[x.alliance].fouls.length,
        coll: r1(x.coll), took: x.took, sec: r1(x.sec), stuckBy: r1(x.stuckBy), idleBy: r1(x.idleBy), labels: r1(x.labels), early: r1(x.early),
        stuckEp: x.stuckEp.sort((a, b) => b.s - a.s).slice(0, 4).map((e) => ({ ...e, s: +e.s.toFixed(1) })),
      });
    }
    return out;
  }
  return { trace, report };
}

// TELEOP time each SHIFT is active for an ALLIANCE: TRANSITION 10, two SHIFTS 50, END GAME 30
export const ACTIVE_TELEOP = TIMING.transition + 2 * TIMING.shift + TIMING.endgame;
