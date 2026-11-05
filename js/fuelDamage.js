// FUEL wear: how scuffed and torn each ball's skin is, for drawing only (b.dmg, 0 = intact; at
// DAMAGE.broken a chunk is torn out). Nothing here feeds back into the simulation: it reads what
// the physics already worked out (a free FUEL's change of velocity over a step, fuel.js postStep;
// each mechanism contact's squeeze, friction and slip, js/mech/mechanism.js) and never calls the
// physics engine or Math.random, so play stays bit-identical (tools/samecheck.mjs).
//
// Calibrated on the two full 3 v 3s of tools/samecheck.mjs (tools/perf3v3.mjs --damage prints the
// counts): 45 and 67 FUEL scuffed (over 0.1), 19 and 8 clearly (over 0.2), 11 and 1 torn open
// (over 0.5), 1 and 0 broken. Mostly the shooters' work (rubbing, then the nip), so the FUEL shot
// most show it; what a step of each cause measured there is noted with its threshold (median /
// 99th / 99.99th percentile / most).
export const DAMAGE = {
  // a free FUEL's change of velocity in one physics step beyond what gravity gives (m/s; 4.5 / 10.5
  // / 12.5 / 12.9: bounces off the carpet are ~4-8; a fast shot into the HUB, a wall or a robot
  // 10-13). d += hitK (dv - hitV)^2
  hitV: 9, hitK: 0.004,
  // a wheel or a mechanism surface first meeting a FUEL, their surfaces' relative speed (m/s; 1.3 /
  // 11.3 / 19 / 21.5): intake rollers meet them at a few m/s and do nothing, a shooter's flywheel
  // at 12-21 scuffs a little each shot. d += nipK (v - nipV)^2
  nipV: 12, nipK: 0.0006,
  // rubbing: the friction's work in a contact still slipping after the solve, beyond rubPower W of
  // it (mechanism.js: a flywheel spinning over a FUEL it hasn't got up to speed, an intake roller
  // or a belt on a load that can't move), each step's over rubE J (0.35 / 4.4 / 10-18 / 76):
  // d += rubK (J - rubE)^1.5
  rubPower: 60, rubE: 0.5, rubK: 0.0006,
  // squeeze: a contact pressing harder than sqF N (40 / 210 / 550-800 / 8,100: a shooter's nip
  // presses ~100-300, a FUEL wedged in a mechanism or crushed against a frame up to thousands).
  // d += sqK (F / sqF - 1)^2 each step
  sqF: 300, sqK: 0.03,
  // the most one step can add: one violent event tears it open, a crush that lasts breaks it
  stepMax: 0.5,
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
  if (dv > DAMAGE.hitV) addDamage(b, DAMAGE.hitK * (dv - DAMAGE.hitV) ** 2, 'hit');
}

// one step's mechanism wear on FUEL b: w = { rub: friction work beyond rubPower (J), nip: fastest
// first-contact relative speed (m/s), sq: firmest squeeze (N) }
export function mechDamage(b, w) {
  if (w.nip > DAMAGE.nipV) addDamage(b, DAMAGE.nipK * (w.nip - DAMAGE.nipV) ** 2, 'nip');
  if (w.rub > DAMAGE.rubE) addDamage(b, DAMAGE.rubK * (w.rub - DAMAGE.rubE) ** 1.5, 'rub');
  if (w.sq > DAMAGE.sqF) addDamage(b, DAMAGE.sqK * (w.sq / DAMAGE.sqF - 1) ** 2, 'sq');
}

// each FUEL's fixed pattern of where its skin gives first (0..1, from its id: no Math.random)
export function damageSeed(id) {
  let h = Math.imul(id + 1, 0x9e3779b1) >>> 0;
  h = Math.imul(h ^ (h >>> 15), 0x85ebca6b) >>> 0;
  return ((h ^ (h >>> 13)) >>> 0) / 4294967296;
}
