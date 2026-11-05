// FUEL wear kept between matches (the browser's localStorage): real FUEL get scuffed and torn over
// an event's matches, not just one. Each FUEL's damage by its id (js/fuelDamage.js); the menu's
// "FUEL wear" setting chooses kept or new FUEL each match (choosing new clears what's saved).
const KEY = 'rebuiltSim.fuelWear';

// the saved damage per FUEL id, or null
export function loadWear() {
  try {
    const a = JSON.parse(localStorage.getItem(KEY) || 'null');
    return Array.isArray(a) ? Float32Array.from(a, (x) => +x || 0) : null;
  } catch { return null; }
}

// from the FUEL's drawn wear attribute ([damage, seed] per FUEL)
export function wearOf(dmgArray) {
  const n = dmgArray.length >> 1, out = new Float32Array(n);
  for (let i = 0; i < n; i++) out[i] = dmgArray[2 * i];
  return out;
}

export function saveWear(wear) {
  try { localStorage.setItem(KEY, JSON.stringify(Array.from(wear, (x) => Math.round(x * 1000) / 1000))); } catch { /* storage unavailable */ }
}

export function clearWear() {
  try { localStorage.removeItem(KEY); } catch { /* storage unavailable */ }
}
