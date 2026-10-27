// A robot in play built of its physical mechanisms (js/mech/mechanism.js): the FUEL it takes in
// stay bodies on the FIELD the whole way, moved only by its rollers, wheels and the surfaces of its
// CAD (Mechanisms on the robot's own body, so it drives off with them, and every push on them
// pushes back on it), until its shooter flings them. What every such robot shares: its frame, which
// FUEL it has hold of, what got in, and what got shot. Each robot (js/mech/phys8793.js,
// js/mech/phys2910.js) adds its mechanisms, where its ball path is, and its code's controls.
import { FUEL } from '../constants.js';

export class PhysicalRobot {
  // path: the Mechanism of its ball path (FUEL in it touch each other there, foam on foam);
  // shooter: the Mechanism whose wheels shoot (a FUEL leaving them fast and upward is a shot);
  // preload: where its preload sits (robot frame)
  constructor(robot, { path, shooter, preload }) {
    Object.assign(this, { r: robot, path, shooter, preload });
  }

  reset() {
    this.held = [];           // FUEL in its ball path
    this.near = [];           // FUEL its mechanisms might touch this step
    this.inShooter = new Set();
    this.owned = new Set();   // FUEL its ball path (or intake) has hold of
    this.shot = new Map();    // FUEL it's just shot -> when (on their way out: not its to hold)
    this.counted = new Set(); // FUEL counted as intaked, until shot or gone from around it
    for (const b of this.squashed || []) b.squash = null;
    this.squashed = new Set(); // FUEL drawn flattened where it presses them (_squash)
  }

  // the robot's frame <-> the world (x forward, y up, z right, origin on the carpet at its center)
  _pose() { const b = this.r.body; this.pose = { t: b.translation(), q: b.rotation() }; return this.pose; }
  _local(p) {
    const { t, q } = this.pose || this._pose();
    const dx = p.x - t.x, dy = p.y - t.y, dz = p.z - t.z;
    // rotate by the inverse of q
    const x = -q.x, y = -q.y, z = -q.z, w = q.w;
    const ix = w * dx + y * dz - z * dy, iy = w * dy + z * dx - x * dz, iz = w * dz + x * dy - y * dx, iw = -x * dx - y * dy - z * dz;
    return [ix * w + iw * -x + iy * -z - iz * -y, iy * w + iw * -y + iz * -x - ix * -z, iz * w + iw * -z + ix * -y - iy * -x];
  }
  // the FUEL it holds, in its frame ({ p }), for what's drawn over them: a net drapes over them
  netLoad() { return this.held.map((b) => { const [x, y, z] = this._local(b.pos); return { p: { x, y, z } }; }); }
  _world(l) {
    const { t, q } = this._pose();
    const [x, y, z] = l, qx = q.x, qy = q.y, qz = q.z, qw = q.w;
    const ix = qw * x + qy * z - qz * y, iy = qw * y + qz * x - qx * z, iz = qw * z + qx * y - qy * x, iw = -qx * x - qy * y - qz * z;
    return { x: t.x + ix * qw + iw * -qx + iy * -qz - iz * -qy, y: t.y + iy * qw + iw * -qy + iz * -qx - ix * -qz, z: t.z + iz * qw + iw * -qz + ix * -qy - iy * -qx };
  }

  // the preload: into its slots (what doesn't fit goes to its alliance's human player)
  load(balls) {
    const fuel = this.r.fuel;
    balls.forEach((b, i) => {
      if (i >= this.preload.length) { fuel.addToChute(this.r.alliance, b); return; }
      const p = this._world(this.preload[i]);
      fuel.launch(b, p, { x: 0, y: 0, z: 0 }, {});
      b.inFlight = false;
      fuel.own(b, 0);
      this.held.push(b);
      this.owned.add(b);
      this.counted.add(b); // (a preload isn't intaked)
    });
    this.r.stored = this.held.slice();
  }

  // where a FUEL at (x, y, z) in the robot's frame is: { own: its ball path or intake has hold of
  // it, held: it's in (not one the intake's still reaching for), near: something of the robot
  // could touch it } (each robot's own)
  where() { return { own: false, held: false, near: false }; }
  // the FUEL near it that the shooter's wheels might touch (each robot's own)
  shooterBalls(near) { return near; }

  // before the world's step (after the robot has posed its moving parts): which FUEL are its,
  // and run the mechanisms
  stepMechanisms(dt, t) {
    const r = this.r;
    this._pose();
    // the FUEL near enough to touch anything of it; the ones in its ball path (or on its intake)
    // don't hit its own colliders: its CAD's surfaces are what they touch there
    const pos = r.body.translation();
    const near = [], held = [], owned = new Set();
    for (const b of r.fuel.balls) {
      if (b.state !== 'field') continue;
      const p = b.pos;
      if (Math.abs(p.x - pos.x) > 1.0 || Math.abs(p.z - pos.z) > 1.0 || p.y > 1.0) continue;
      if (this.shot.has(b)) {
        if (t - this.shot.get(b) < 0.35) continue;
        this.shot.delete(b);
      }
      const w = this.where(...this._local(p));
      if (w.own) {
        r.fuel.own(b, t);
        owned.add(b);
        if (w.held) held.push(b);
      }
      if (w.own || w.near) near.push(b);
    }
    // (the ones it's let go of: solid to the other FUEL again)
    for (const b of this.owned) if (!owned.has(b) && b.state === 'field') r.fuel.unown(b);
    this.owned = owned;
    this.near = near;
    this.held = held;
    // (FUEL it holds pressing on each other: foam on foam, worked out with its wheels)
    this.path.step(dt, near.map((b) => b.body), [...owned].map((b) => b.body));
    if (this.shooter) this.shooter.step(dt, this.shooterBalls(near).map((b) => b.body));
    this._squash(near);
  }

  // the FUEL its mechanisms press, drawn flattened where they're pressed (fuel.js: foam, not pushed
  // through a wall or a floor), the four deepest; the rest round again. b.squash: [nx, ny, nz, w]
  // per patch, the normal (world) from the FUEL's center toward the contact and the patch's
  // distance from it: the FUEL's own dent is all of the squeeze against something hard, part of it
  // against a tread that gives (a roller's, the net's: half, about)
  _squash(near) {
    const byBody = new Map(near.map((b) => [b.body, b])), R = FUEL.radius, now = new Set();
    for (const c of [...this.path.dents, ...(this.shooter ? this.shooter.dents : [])]) {
      const b = byBody.get(c.ball);
      if (!b || c.wheel.name === 'FUEL' || !(c.depth > 0.0005)) continue;
      const d = Math.min(R * 0.6, c.wheel.tread < Infinity ? c.depth / 2 : c.depth);
      if (!now.has(b)) { now.add(b); b.squash = []; }
      b.squash.push([-c.n[0], -c.n[1], -c.n[2], R - d - 0.001]); // (a hair short of the face, so the flat doesn't flicker through it)
    }
    for (const b of now) if (b.squash.length > 4) b.squash = b.squash.sort((p, q) => p[3] - q[3]).slice(0, 4);
    for (const b of this.squashed || []) if (!now.has(b)) b.squash = null;
    this.squashed = now;
  }

  // after the world's step: what got in, and what got shot (mode: what the shot's for)
  account(t, mode) {
    const r = this.r;
    this.pose = null;
    r.stored = this.held.slice();
    // intaked: once a FUEL's in, however it jostles in and out of the frame's line on its way
    const around = new Set(this.near);
    for (const b of this.counted) if (!around.has(b)) this.counted.delete(b);
    for (const b of r.stored) if (!this.counted.has(b)) { this.counted.add(b); r.stats.intaked++; }
    // shot: through the shooter's wheels and out
    for (const [b, ts] of this.shot) if (t - ts > 0.35) this.shot.delete(b);
    if (!this.shooter) return;
    const byBody = new Map(this.near.map((b) => [b.body, b]));
    const touching = new Set(this.shooter.contacts.map((c) => byBody.get(c.ball)).filter(Boolean));
    for (const b of touching) this.inShooter.add(b);
    for (const b of [...this.inShooter]) {
      if (touching.has(b)) continue;
      this.inShooter.delete(b);
      const v = b.body.linvel();
      if (Math.hypot(v.x, v.y, v.z) < 3 || v.y < 0) continue; // dropped back down, not shot
      this.counted.delete(b);
      this.owned.delete(b);
      this.shot.set(b, t);
      r.fuel.release(b, { by: 'robot', alliance: r.alliance, legal: r.lastInZone, t, ignoreRobot: 0.35 });
      r.stats.shots++;
      if (mode === 'pass') r.stats.passes++;
    }
  }
}
