// All 504 FUEL. Each ball is a Rapier rigid body while it is on the FIELD (including while an
// intake is pulling it in); FUEL held by a ROBOT (simulated by the robot's Hopper), inside a
// HUB, in an OUTPOST CHUTE or waiting to be returned by field staff has its body disabled and
// is drawn (or hidden) separately.
import * as THREE from 'three';
import { RAPIER } from './physics.js';
import { FUEL, HUB, HALF_L, HALF_W, BLUE, RED, GROUP, groups, PHYSICS_DT } from './constants.js';
import { Field } from './field.js';
import { rand, clamp } from './util.js';

const R = FUEL.radius;
const NORMAL_GROUPS = groups(GROUP.BALL, GROUP.STATIC | GROUP.TERRAIN | GROUP.BALL | GROUP.ROBOT | GROUP.INTAKE);
const NO_ROBOT_GROUPS = groups(GROUP.BALL, GROUP.STATIC | GROUP.TERRAIN | GROUP.BALL);
const HIDDEN = new THREE.Matrix4().makeScale(0, 0, 0);

export class FuelManager {
  constructor(physics, scene, field) {
    this.physics = physics;
    this.scene = scene;
    this.field = field;
    this.match = null;
    this.balls = [];
    this.hubQueue = [];
    this.outQueue = [];
    this.exitBusy = { blue: [0, 0, 0, 0], red: [0, 0, 0, 0] };
    this.chute = { blue: [], red: [] };

    const geo = new THREE.SphereGeometry(R, 16, 12);
    const mat = new THREE.MeshStandardMaterial({ color: 0xf4d23a, roughness: 0.78, metalness: 0 });
    this.mesh = new THREE.InstancedMesh(geo, mat, FUEL.total);
    this.mesh.castShadow = true;
    this.mesh.receiveShadow = true;
    this.mesh.instanceMatrix.setUsage(THREE.DynamicDrawUsage);
    this.mesh.frustumCulled = false;
    scene.add(this.mesh);

    const { world } = physics;
    for (let i = 0; i < FUEL.total; i++) {
      const body = world.createRigidBody(
        RAPIER.RigidBodyDesc.dynamic()
          .setTranslation(0, -10 - i * 0.2, 0)
          .setCcdEnabled(true)
          .setAngularDamping(FUEL.angularDamping)
          .setCanSleep(true),
      );
      const col = world.createCollider(
        RAPIER.ColliderDesc.ball(R)
          .setMass(FUEL.mass)
          .setRestitution(FUEL.restitution)
          .setFriction(FUEL.friction)
          .setCollisionGroups(NORMAL_GROUPS),
        body,
      );
      body.setEnabled(false);
      this.balls.push({
        id: i, body, col, state: 'off',
        pos: new THREE.Vector3(0, -10, 0), quat: new THREE.Quaternion(),
        prevVel: { x: 0, y: 0, z: 0 },
        inFlight: false, launch: null, hubFresh: false, ignoreUntil: 0, ignoring: false, inCorral: null,
      });
    }
  }

  // ------------------------------------------------------------------ staging
  // preload: FUEL preloaded in the player's robot (0..8). Remaining FUEL goes to the NEUTRAL ZONE.
  stage(preload) {
    this.hubQueue = [];
    this.outQueue = [];
    this.exitBusy = { blue: [0, 0, 0, 0], red: [0, 0, 0, 0] };
    for (const b of this.balls) { this._disable(b, 'off'); b.captor = null; b.hop = null; }
    let i = 0;
    const next = () => this.balls[i++];
    const neutralCount = Math.min(FUEL.neutralMax, FUEL.total - 2 * FUEL.perDepot - 2 * FUEL.perChute - preload);
    for (const s of this.field.neutralSlots(neutralCount)) this._place(next(), s.x, s.y, s.z);
    for (const a of [BLUE, RED]) for (const s of this.field.depotSlots(a)) this._place(next(), s.x, s.y, s.z);
    this.chute = { blue: [], red: [] };
    for (const a of [BLUE, RED]) {
      for (let k = 0; k < FUEL.perChute; k++) {
        const b = next();
        b.state = 'chute';
        b.chuteAlliance = a;
        this.chute[a].push(b);
      }
    }
    const pre = [];
    for (let k = 0; k < preload; k++) {
      const b = next();
      b.state = 'robot';
      pre.push(b);
    }
    // any leftovers (only if counts change) go to the NEUTRAL ZONE edges
    while (i < this.balls.length) this._place(next(), rand(-0.5, 0.5), R + 0.01, rand(-3.4, 3.4));
    this._chuteWorld = null;
    return pre;
  }

  _place(b, x, y, z) {
    b.state = 'field';
    b.inFlight = false;
    b.launch = null;
    b.hubFresh = false;
    b.inCorral = null;
    b.body.setEnabled(true);
    b.body.setTranslation({ x, y, z }, false);
    b.body.setLinvel({ x: 0, y: 0, z: 0 }, false);
    b.body.setAngvel({ x: 0, y: 0, z: 0 }, false);
    b.body.setRotation({ x: 0, y: 0, z: 0, w: 1 }, false);
    b.col.setCollisionGroups(NORMAL_GROUPS);
    b.ignoring = false;
    b.body.sleep();
    b.pos.set(x, y, z);
    b.prevVel = { x: 0, y: 0, z: 0 };
  }

  _disable(b, state) {
    b.state = state;
    b.body.setLinvel({ x: 0, y: 0, z: 0 }, false);
    b.body.setEnabled(false);
    b.inFlight = false;
    b.inCorral = null;
  }

  toRobot(b) {
    this._disable(b, 'robot');
    b.hubFresh = false;
    b.launch = null;
  }

  // Put a ball into play at pos with velocity vel
  launch(b, pos, vel, info = {}) {
    b.state = 'field';
    b.body.setEnabled(true);
    b.body.setTranslation({ x: pos.x, y: pos.y, z: pos.z }, true);
    b.body.setLinvel({ x: vel.x, y: vel.y, z: vel.z }, true);
    b.body.setAngvel({ x: 0, y: 0, z: 0 }, true);
    b.body.wakeUp();
    b.pos.set(pos.x, pos.y, pos.z);
    b.prevVel = { x: vel.x, y: vel.y, z: vel.z };
    b.inFlight = true;
    b.hubFresh = !!info.hubFresh;
    b.inCorral = null;
    b.launch = info.by ? { by: info.by, alliance: info.alliance, legal: info.legal, t: info.t } : null;
    // thrown in from the OUTPOST AREA (behind the ALLIANCE WALL): not "out" until it has entered
    b.fromOutside = info.fromOutside ? (info.t ?? 0) : null;
    if (info.ignoreRobot) {
      b.col.setCollisionGroups(NO_ROBOT_GROUPS);
      b.ignoring = true;
      b.ignoreUntil = (info.t ?? 0) + info.ignoreRobot;
    } else {
      b.col.setCollisionGroups(NORMAL_GROUPS);
      b.ignoring = false;
    }
  }

  // ------------------------------------------------------------------ per physics step
  preStep(dt) {
    const k = FUEL.dragK;
    for (const b of this.balls) {
      if (b.state !== 'field') continue;
      const body = b.body;
      if (body.isSleeping()) continue;
      const v = body.linvel();
      let vx = v.x, vy = v.y, vz = v.z;
      const sp = Math.hypot(vx, vy, vz);
      if (sp > 0.3) {
        const f = Math.max(0, 1 - k * sp * dt);
        vx *= f; vy *= f; vz *= f;
      }
      const grounded = b.pos.y < R + 0.012 && Math.abs(vy) < 0.35;
      if (grounded) {
        // carpet rolling resistance: slow the roll and the spin together. (Forcing the spin to
        // v/R here would fight the contact solver, which rolls the ball about its slightly
        // sunken contact point, and bleed off speed every step.)
        const hs = Math.hypot(vx, vz);
        const dec = FUEL.rollDecel * dt;
        const f = hs <= dec + 0.01 ? 0 : (hs - dec) / hs;
        vx *= f; vz *= f;
        const w = body.angvel();
        body.setAngvel({ x: w.x * f, y: w.y * f, z: w.z * f }, false);
      }
      body.setLinvel({ x: vx, y: vy, z: vz }, false);
    }
  }

  postStep(dt, t) {
    for (const b of this.balls) {
      if (b.state !== 'field') continue;
      const body = b.body;
      if (body.isSleeping()) continue;
      const p = body.translation();
      b.pos.set(p.x, p.y, p.z);
      const v = body.linvel();
      if (b.inFlight || b.hubFresh) {
        // detect contact with anything: velocity deviating from free flight
        const ex = b.prevVel.x, ey = b.prevVel.y - 9.81 * dt, ez = b.prevVel.z;
        const dev = Math.hypot(v.x - ex, v.y - ey, v.z - ez);
        const sp = Math.hypot(v.x, v.y, v.z);
        if (dev > 0.35 + 0.03 * sp * sp * dt * 60 || p.y < R + 0.015) {
          b.inFlight = false;
          b.hubFresh = false;
        }
      }
      b.prevVel = { x: v.x, y: v.y, z: v.z };
      if (b.ignoring && t >= b.ignoreUntil) {
        b.col.setCollisionGroups(NORMAL_GROUPS);
        b.ignoring = false;
      }
    }
  }

  // ------------------------------------------------------------------ per frame
  update(dt, t) {
    const m = this.match;
    for (const b of this.balls) {
      if (b.state !== 'field') continue;
      const p = b.pos;
      // HUB entry (FUEL that passes through the top opening)
      if (p.y > 1.0 && p.y < HUB.funnelBottomY + R + 0.12) {
        for (const a of [BLUE, RED]) {
          if (this.field.isInsideHubOpening(a, p.x, p.y, p.z)) {
            this._enterHub(b, a, t);
            break;
          }
        }
        if (b.state !== 'field') continue;
      }
      // Corral (behind the OUTPOST base opening) — still on the carpet but off the FIELD
      b.inCorral = null;
      if (Math.abs(p.x) > HALF_L + 0.02) {
        for (const a of [BLUE, RED]) {
          const c = this.field.outposts[a].corral;
          if (p.x > c.x0 - 0.05 && p.x < c.x1 + 0.05 && p.z > c.z0 - 0.05 && p.z < c.z1 + 0.05 && p.y < 0.5) b.inCorral = a;
        }
      }
      if (b.inCorral) continue;
      if (b.fromOutside !== null && b.fromOutside !== undefined) {
        const inside = Math.abs(p.x) < HALF_L && Math.abs(p.z) < HALF_W;
        if (inside) b.fromOutside = null;
        else if (t - b.fromOutside < 3) continue;
      }
      // left the FIELD
      const out = Math.abs(p.x) > HALF_L + 0.08 || Math.abs(p.z) > HALF_W + 0.08 || p.y < -0.5;
      if (out) {
        if (b.inFlight && b.launch && b.launch.by === 'robot' && m) {
          m.addFoul(b.launch.alliance, 'minor', 'G405', 'FUEL launched out of the FIELD');
        }
        const rx = clamp(p.x, -HALF_L + 0.35, HALF_L - 0.35);
        const rz = clamp(p.z, -HALF_W + 0.35, HALF_W - 0.35);
        this._disable(b, 'out');
        this.outQueue.push({ b, at: t + rand(3, 6), x: rx, z: rz });
      }
    }

    // HUB processing -> exits into the NEUTRAL ZONE
    for (let i = this.hubQueue.length - 1; i >= 0; i--) {
      const q = this.hubQueue[i];
      if (t < q.at) continue;
      const busy = this.exitBusy[q.alliance];
      let e = Math.floor(Math.random() * 4);
      let tries = 0;
      while (busy[e] > t && tries < 4) { e = (e + 1) % 4; tries++; }
      if (busy[e] > t) continue;
      busy[e] = t + 0.11;
      this.hubQueue.splice(i, 1);
      this._exitHub(q.b, q.alliance, e, t);
    }
    // field staff returning FUEL that left the FIELD
    for (let i = this.outQueue.length - 1; i >= 0; i--) {
      const q = this.outQueue[i];
      if (t < q.at) continue;
      this.outQueue.splice(i, 1);
      this.launch(q.b, { x: q.x, y: R + 0.02, z: q.z }, { x: 0, y: 0, z: 0 }, {});
      q.b.inFlight = false;
    }
  }

  _enterHub(b, alliance, t) {
    this._disable(b, 'hub');
    if (this.match) this.match.fuelEnteredHub(alliance, b, t);
    b.launch = null;
    this.hubQueue.push({ b, alliance, at: t + rand(0.45, 1.3) });
  }

  _exitHub(b, alliance, e, t) {
    const c = Field.hubCenter(alliance);
    const s = alliance === BLUE ? 1 : -1; // direction toward the NEUTRAL ZONE
    const off = HUB.exitOffsets[e];
    const x = c.x + s * (HUB.size / 2 + R + 0.03);
    const z = c.z + s * off;
    const base = Math.sign(off) * (Math.abs(off) > 0.3 ? 0.55 : 0.18);
    const ang = base + rand(-0.4, 0.4);
    const sp = rand(HUB.exitSpeed[0], HUB.exitSpeed[1]);
    // leaves the exit ramp level or slightly downward (no pop-up), already rolling
    this.launch(b, { x, y: HUB.exitHeight + R, z }, { x: s * Math.cos(ang) * sp, y: rand(-0.3, 0), z: s * Math.sin(ang) * sp }, { hubFresh: true, t });
    b.body.setAngvel({ x: (s * Math.sin(ang) * sp) / R, y: 0, z: -(s * Math.cos(ang) * sp) / R }, true);
    b.inFlight = false;
  }

  // ------------------------------------------------------------------ chute helpers
  chuteCount(a) { return this.chute[a].length; }

  takeFromChute(a) { return this.chute[a].shift() || null; }

  addToChute(a, b) {
    this._disable(b, 'chute');
    b.chuteAlliance = a;
    this.chute[a].push(b);
  }

  corralBalls(a) { return this.balls.filter((b) => b.state === 'field' && b.inCorral === a); }

  // ------------------------------------------------------------------ rendering
  sync() {
    const mesh = this.mesh;
    const m = new THREE.Matrix4();
    const one = new THREE.Vector3(1, 1, 1);
    for (const b of this.balls) {
      if (b.state === 'field') {
        if (!b.body.isSleeping()) {
          const r = b.body.rotation();
          b.quat.set(r.x, r.y, r.z, r.w);
        }
        m.compose(b.pos, b.quat, one);
        mesh.setMatrixAt(b.id, m);
      } else if (b.state !== 'chute') {
        mesh.setMatrixAt(b.id, HIDDEN);
      }
    }
    // FUEL waiting in the OUTPOST CHUTES
    for (const a of [BLUE, RED]) {
      const slots = this.field.outposts[a].chuteSlots;
      const list = this.chute[a];
      for (let k = 0; k < list.length; k++) {
        const sl = slots[k % slots.length];
        const w = sl.chute.localToWorld(sl.local.clone());
        m.makeTranslation(w.x, w.y, w.z);
        mesh.setMatrixAt(list[k].id, m);
      }
    }
    mesh.instanceMatrix.needsUpdate = true;
  }

  counts() {
    const c = { field: 0, robot: 0, hub: 0, chute: 0, out: 0, off: 0, corral: 0 };
    for (const b of this.balls) {
      c[b.state] = (c[b.state] || 0) + 1;
      if (b.inCorral) c.corral++;
    }
    return c;
  }
}
