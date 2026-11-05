// FUEL wear: how scuffed and torn each ball's skin is, for drawing only (b.dmg, 0 = intact; at
// DAMAGE.broken a chunk is torn out). Nothing here feeds back into the simulation: it reads what
// the physics already worked out (a free FUEL's change of velocity over a step, fuel.js postStep;
// each mechanism contact's squeeze, friction and slip, js/mech/mechanism.js) and never calls the
// physics engine or Math.random, so play stays bit-identical (tools/samecheck.mjs).
//
// Calibrated on headless matches (tools/perf3v3.mjs --damage) so that a full match leaves a few
// dozen FUEL scuffed (over 0.1), a handful badly torn and only a rare one broken: the ones a
// shooter has squeezed and spun many times, crushed in a mechanism, or slammed at speed.
export const DAMAGE = {
  // a free FUEL's change of velocity in one physics step beyond what gravity gives (m/s): bounces
  // off the carpet from a high shot are ~6-8 m/s and do nothing; a shot driven straight into a wall,
  // the HUB or a robot changes by 10-16 m/s. d += hitK (dv - hitV)^2
  hitV: 10, hitK: 0.0025,
  // a wheel or a mechanism surface first meeting a FUEL (m/s of the surfaces' relative speed):
  // intake rollers meet them at ~3-8 m/s and do nothing; a shooter's flywheel at 12-30 m/s scuffs a
  // little every shot. d += nipK (v - nipV)^2
  nipV: 12, nipK: 0.00012,
  // rubbing: friction work (J) in a contact slipping hard, above rubP W (a wheel spinning on a
  // FUEL it can't move: a flywheel's nip, an intake roller on a jammed or full load).
  // d += rubK (J over rubP x time)
  rubP: 60, rubK: 0.0012,
  // squeeze: a contact pressing harder than sqF N (a shooter's nip presses ~100-400 N, a FUEL
  // wedged in a mechanism or crushed against a robot's frame more). d += sqK (F / sqF - 1) per s
  sqF: 700, sqK: 0.6,
  // the most a single step can add (one violent event: no ball goes from new to shredded at once)
  stepMax: 0.25,
  // a chunk torn out from here (the drawn hole grows on to 2 x broken)
  broken: 1,
  max: 2,
};

// totals by cause over a run (for calibrating: tools/perf3v3.mjs --damage)
export const damageStats = { hit: 0, nip: 0, rub: 0, sq: 0, events: { hit: 0, nip: 0, rub: 0, sq: 0 } };

// add d of one cause to FUEL b
export function addDamage(b, d, cause) {
  if (!(d > 0)) return;
  d = Math.min(d, DAMAGE.stepMax);
  b.dmg = Math.min(DAMAGE.max, (b.dmg || 0) + d);
  b.dmgDirty = true;
  damageStats[cause] += d;
  damageStats.events[cause]++;
}

// a free FUEL's velocity change dv (m/s) over one step
export function impactDamage(b, dv) {
  if (globalThis.__hitLog && dv > 3) globalThis.__hitLog.push([b.id, dv]);
  if (dv > DAMAGE.hitV) addDamage(b, DAMAGE.hitK * (dv - DAMAGE.hitV) ** 2, 'hit');
}

// one step's mechanism wear on FUEL b: w = { rub: friction work beyond rubP (J), nip: fastest
// first-contact relative speed (m/s), sq: firmest squeeze (N) }, dt the step
export function mechDamage(b, w, dt) {
  if (globalThis.__dmgLog && (w.rub > 0 || w.nip > 0 || w.sq > 50)) globalThis.__dmgLog.push([b.id, w.rub, w.nip, w.sq]);
  if (w.nip > DAMAGE.nipV) addDamage(b, DAMAGE.nipK * (w.nip - DAMAGE.nipV) ** 2, 'nip');
  if (w.rub > 0) addDamage(b, DAMAGE.rubK * w.rub, 'rub');
  if (w.sq > DAMAGE.sqF) addDamage(b, DAMAGE.sqK * (w.sq / DAMAGE.sqF - 1) * dt, 'sq');
}

// each FUEL's fixed pattern of where its skin gives first (0..1, from its id: no Math.random)
export function damageSeed(id) {
  let h = Math.imul(id + 1, 0x9e3779b1) >>> 0;
  h = Math.imul(h ^ (h >>> 15), 0x85ebca6b) >>> 0;
  return ((h ^ (h >>> 13)) >>> 0) / 4294967296;
}
