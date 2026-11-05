// FUEL wear kept between matches (the browser's localStorage): real FUEL get scuffed and torn over
// an event's matches, not just one. Each FUEL's damage and where on it, by its id (js/fuelDamage.js); the menu's
// "FUEL wear" setting chooses kept or new FUEL each match (choosing new clears what's saved).
const KEY = 'rebuiltSim.fuelWear';

// the saved wear, [damage, x, y, z] per FUEL id (where on it: fuelDamage.js wearSlot), or null
export function loadWear() {
  try {
    const a = JSON.parse(localStorage.getItem(KEY) || 'null');
    if (!Array.isArray(a) || !a.length) return null;
    if (!a.some(Array.isArray)) return Float32Array.from(a.flatMap((d) => [+d || 0, 0, 0, 0])); // (older: damage only)
    return Float32Array.from(a.flatMap((w) => (Array.isArray(w) ? [0, 1, 2, 3].map((j) => +w[j] || 0) : [0, 0, 0, 0])));
  } catch { return null; }
}

// from the FUEL's drawn wear attribute ([damage + 4 x seed, x, y, z] per FUEL: fuelDamage.js wearSlot)
export function wearOf(dmgArray) {
  const out = Float32Array.from(dmgArray);
  for (let i = 0; i < out.length; i += 4) out[i] %= 4;
  return out;
}

export function saveWear(wear) {
  try { const r = (x) => Math.round(x * 1000) / 1000, out = [];
    for (let i = 0; i < wear.length; i += 4) out.push(wear[i] > 0 ? [r(wear[i]), r(wear[i + 1]), r(wear[i + 2]), r(wear[i + 3])] : 0);
    localStorage.setItem(KEY, JSON.stringify(out)); } catch { /* storage unavailable */ }
}

export function clearWear() {
  try { localStorage.removeItem(KEY); } catch { /* storage unavailable */ }
}
