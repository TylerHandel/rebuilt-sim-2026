// ROBOT-to-ROBOT rules, checked every physics step for each pair of opposing ROBOTS.
//   G403 (MAJOR) AUTO: contacting an opponent while fully across the CENTER LINE.
//   G415 (MINOR) a COMPONENT outside the FRAME PERIMETER (a deployed over-the-bumper intake)
//                reaching inside an opponent's FRAME PERIMETER.
//   G416 (MAJOR) high-speed ramming, treated as an attempt to damage. (G417 tipping can't
//                happen here: robots are kept flat.)
//   G418 (MINOR) PINNING an opponent against a FIELD element for more than 3 s, plus another
//                MINOR for every further 3 s. The count resets once the ROBOTS are 72 in apart.
//   G420 (MAJOR) END GAME: contacting an opponent that is touching its own TOWER or climbing.
import { TIMING, IN } from './constants.js';
import { obstacleAt, towerRect } from './nav.js';

export const PIN_LIMIT = 3;
export const PIN_RESET = 72 * IN;
const CONTACT_GAP = 0.03;
const RAM_CLOSING = 3.3; // m/s closing speed
export const RAM_AGGRESSOR = 2.6; // m/s of that from the rammer

// ---------------------------------------------------------------- 2D polygon helpers
function rectPoly(r, hl, hw, ox = 0) {
  const c = Math.cos(r.yaw), s = Math.sin(r.yaw);
  const out = [];
  for (const [a, b] of [[1, 1], [1, -1], [-1, -1], [-1, 1]]) {
    const lx = ox + a * hl, lz = b * hw;
    out.push({ x: r.pos.x + lx * c + lz * s, z: r.pos.z - lx * s + lz * c });
  }
  return out;
}

function project(P, ax, az) {
  let lo = Infinity, hi = -Infinity;
  for (const p of P) { const d = p.x * ax + p.z * az; if (d < lo) lo = d; if (d > hi) hi = d; }
  return [lo, hi];
}

export function polysOverlap(P, Q) {
  for (const poly of [P, Q]) {
    for (let i = 0; i < poly.length; i++) {
      const a = poly[i], b = poly[(i + 1) % poly.length];
      const ax = -(b.z - a.z), az = b.x - a.x;
      const [p0, p1] = project(P, ax, az);
      const [q0, q1] = project(Q, ax, az);
      if (p1 < q0 || q1 < p0) return false;
    }
  }
  return true;
}

function segDist(p, a, b) {
  const dx = b.x - a.x, dz = b.z - a.z;
  const l2 = dx * dx + dz * dz || 1e-9;
  const t = Math.max(0, Math.min(1, ((p.x - a.x) * dx + (p.z - a.z) * dz) / l2));
  return Math.hypot(p.x - (a.x + dx * t), p.z - (a.z + dz * t));
}

export function polyGap(P, Q) {
  if (polysOverlap(P, Q)) return 0;
  let d = Infinity;
  for (const [A, B] of [[P, Q], [Q, P]]) {
    for (const p of A) for (let i = 0; i < B.length; i++) d = Math.min(d, segDist(p, B[i], B[(i + 1) % B.length]));
  }
  return d;
}

export const bumperPoly = (r) => rectPoly(r, r.halfL, r.halfW);
export const framePoly = (r) => rectPoly(r, r.cfg.frame.length / 2, r.cfg.frame.width / 2);
function intakePoly(r) {
  const ic = r.cfg.intake;
  return rectPoly(r, ic.reach / 2 + 0.01, ic.width / 2, r.halfL + ic.reach / 2 - 0.01);
}

// Is the robot touching its own TOWER (or hanging on it)?
export function towerProtected(r) {
  if (r.climbState && r.climbState !== 'none') return true;
  const t = towerRect(r.alliance);
  const T = [
    { x: t.x - t.hx - 0.05, z: t.z - t.hz - 0.05 }, { x: t.x + t.hx + 0.05, z: t.z - t.hz - 0.05 },
    { x: t.x + t.hx + 0.05, z: t.z + t.hz + 0.05 }, { x: t.x - t.hx - 0.05, z: t.z + t.hz + 0.05 },
  ];
  return polysOverlap(bumperPoly(r), T);
}

// Is Y backed against a FIELD element on the far side from X (so X's push traps it)?
function againstElement(Y, dx, dz) {
  const P = bumperPoly(Y);
  let ext = 0;
  for (const p of P) ext = Math.max(ext, (p.x - Y.pos.x) * dx + (p.z - Y.pos.z) * dz);
  const px = -dz, pz = dx; // lateral
  for (const lat of [-0.3, 0, 0.3]) {
    const x = Y.pos.x + dx * (ext + 0.12) + px * lat, z = Y.pos.z + dz * (ext + 0.12) + pz * lat;
    if (obstacleAt(x, z, 0)) return true;
  }
  return false;
}

export class RobotRules {
  constructor(match) {
    this.match = match;
    this.pairs = new Map();
  }

  _state(A, B) {
    const k = A.alliance < B.alliance ? `${A.alliance}|${B.alliance}` : `${B.alliance}|${A.alliance}`;
    let st = this.pairs.get(k);
    if (!st) {
      st = { contact: false, noContactT: 99, gap: 99, prevV: new Map(), pin: new Map(), g415: new Map(), episode: { g403: false, g416: false, g420: new Map() } };
      this.pairs.set(k, st);
    }
    return st;
  }

  pinState(X, Y) {
    const st = this._state(X, Y);
    let p = st.pin.get(X);
    if (!p) { p = { t: 0, fouls: 0, active: false }; st.pin.set(X, p); }
    return p;
  }

  gap(A, B) { return this._state(A, B).gap; }
  inContact(A, B) { return this._state(A, B).contact; }

  update(dt, robots) {
    for (let i = 0; i < robots.length; i++) {
      for (let j = i + 1; j < robots.length; j++) {
        if (robots[i].alliance !== robots[j].alliance) this._pair(dt, robots[i], robots[j]);
      }
    }
  }

  _pair(dt, A, B) {
    const m = this.match;
    const st = this._state(A, B);
    const live = m.robotEnabled;
    const gap = polyGap(bumperPoly(A), bumperPoly(B));
    st.gap = gap;
    const contact = gap < CONTACT_GAP;
    const onset = contact && !st.contact;
    if (contact) st.noContactT = 0; else st.noContactT += dt;
    // a contact "episode" ends after half a second apart
    if (st.noContactT > 0.5) {
      st.episode.g403 = false;
      st.episode.g416 = false;
      st.episode.g420.clear();
    }
    let dx = B.pos.x - A.pos.x, dz = B.pos.z - A.pos.z;
    const dd = Math.hypot(dx, dz) || 1;
    dx /= dd; dz /= dd;

    if (live) {
      // ---- G416 high-speed ram (use pre-impact velocities)
      if (onset && !st.episode.g416) {
        const va = st.prevV.get(A) || A.vel, vb = st.prevV.get(B) || B.vel;
        const ca = va.x * dx + va.z * dz; // A toward B
        const cb = -(vb.x * dx + vb.z * dz); // B toward A
        if (ca + cb > RAM_CLOSING) {
          const X = ca >= cb ? A : B;
          if (Math.max(ca, cb) > RAM_AGGRESSOR) {
            st.episode.g416 = true;
            m.addFoul(X.alliance, 'major', 'G416', 'High-speed ram into an opponent ROBOT (damage risk)');
          }
        }
      }
      // ---- G403 AUTO: contact while fully across the CENTER LINE
      if (m.isAuto && contact && !st.episode.g403) {
        for (const X of [A, B]) {
          const s = X.alliance === 'blue' ? 1 : -1; // blue's side is -X
          if (bumperPoly(X).every((p) => s * p.x > 0)) {
            st.episode.g403 = true;
            m.addFoul(X.alliance, 'major', 'G403', 'Contacted an opponent in AUTO while fully across the CENTER LINE');
          }
        }
      }
      // ---- G415 deployed intake inside the opponent's FRAME PERIMETER
      for (const [X, Y, ux, uz] of [[A, B, dx, dz], [B, A, -dx, -dz]]) {
        let g = st.g415.get(X);
        if (!g) { g = { on: false, off: 0 }; st.g415.set(X, g); }
        const inside = X.intakeDeploy > 0.85 && gap < 0.4 && polysOverlap(intakePoly(X), framePoly(Y));
        if (inside) {
          // only the ROBOT that initiated the contact (was driving into the other one) is at fault
          const vx = st.prevV.get(X) || X.vel, vy = st.prevV.get(Y) || Y.vel;
          const xIn = vx.x * ux + vx.z * uz, yIn = -(vy.x * ux + vy.z * uz);
          if (!g.on && xIn > 0.3 && xIn >= yIn) m.addFoul(X.alliance, 'minor', 'G415', 'Deployed intake reached inside the opponent FRAME PERIMETER');
          g.on = true; g.off = 0;
        } else if (g.on && (g.off += dt) > 1) g.on = false;
      }
      // ---- G420 END GAME tower protection
      const endGame = m.isTeleop && m.phaseTime >= TIMING.teleop - TIMING.endgame;
      if (endGame && contact) {
        for (const [X, Y] of [[A, B], [B, A]]) {
          if (towerProtected(Y) && !towerProtected(X) && !st.episode.g420.get(X)) {
            st.episode.g420.set(X, true);
            m.addFoul(X.alliance, 'major', 'G420', 'Contacted an opponent touching its TOWER during END GAME');
          }
        }
      }
    }

    // ---- G418 pinning (X pins Y)
    for (const [X, Y, ux, uz] of [[A, B, dx, dz], [B, A, -dx, -dz]]) {
      const p = this.pinState(X, Y);
      if (gap >= PIN_RESET) { p.t = 0; p.fouls = 0; }
      const push = X.cmd.vx * ux + X.cmd.vz * uz;
      const ySpeed = Math.hypot(Y.vel.x, Y.vel.z);
      p.active = live && contact && X.climbState === 'none' && push > 0.5 && ySpeed < 0.35 && againstElement(Y, ux, uz);
      if (p.active) {
        p.t += dt;
        if (p.t >= PIN_LIMIT * (p.fouls + 1)) {
          p.fouls++;
          m.addFoul(X.alliance, 'minor', 'G418', `PINNED an opponent for more than ${PIN_LIMIT * p.fouls} s`);
        }
      }
    }
    st.contact = contact;
    st.prevV.set(A, { x: A.vel.x, z: A.vel.z });
    st.prevV.set(B, { x: B.vel.x, z: B.vel.z });
  }
}
