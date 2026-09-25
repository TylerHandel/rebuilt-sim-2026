export const clamp = (v, lo, hi) => (v < lo ? lo : v > hi ? hi : v);
export const lerp = (a, b, t) => a + (b - a) * t;
export const DEG = Math.PI / 180;

export function wrapAngle(a) {
  while (a > Math.PI) a -= 2 * Math.PI;
  while (a < -Math.PI) a += 2 * Math.PI;
  return a;
}

// Standard normal via Box-Muller
export function gauss() {
  let u = 0, v = 0;
  while (u === 0) u = Math.random();
  while (v === 0) v = Math.random();
  return Math.sqrt(-2 * Math.log(u)) * Math.cos(2 * Math.PI * v);
}

export const rand = (lo, hi) => lo + Math.random() * (hi - lo);

// Move `cur` toward `target` by at most `maxStep`
export function approach(cur, target, maxStep) {
  const d = target - cur;
  if (Math.abs(d) <= maxStep) return target;
  return cur + Math.sign(d) * maxStep;
}

export function approachAngle(cur, target, maxStep) {
  const d = wrapAngle(target - cur);
  if (Math.abs(d) <= maxStep) return target;
  return wrapAngle(cur + Math.sign(d) * maxStep);
}

// Point-in-regular-hexagon test (flat side facing +/-x), circumradius r
export function inHexagon(dx, dz, r) {
  const ax = Math.abs(dx), az = Math.abs(dz);
  const inr = r * Math.cos(Math.PI / 6);
  if (ax > inr) return false;
  // edges with corners at +/- z
  return az <= r - ax * Math.tan(Math.PI / 6);
}

export function fmtClock(sec) {
  sec = Math.max(0, Math.ceil(sec - 1e-6));
  const m = Math.floor(sec / 60);
  const s = sec % 60;
  return `${m}:${s.toString().padStart(2, '0')}`;
}
