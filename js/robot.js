import * as THREE from 'three';
import { RAPIER, yawQuat } from './physics.js';
import {
  BLUE, HALF_L, HALF_W, HUB, FUEL, TOWER, GROUP, groups, IN,
} from './constants.js';
import { BUMPER_T } from './robotConfigs.js';
import { buildRobotModel, addClimberVisual, BUMP_Y1 } from './robotModels.js';
import { Hopper } from './hopper.js';
import { ShotTable, solveMovingShot, trajectoryPoints } from './ballistics.js';
import { Field } from './field.js';
import { obstacleAt } from './nav.js';
import { clamp, wrapAngle, approach, approachAngle, gauss, DEG, rand } from './util.js';

const WHEEL_R = 0.05;
const R = FUEL.radius;
// FUEL the intake has grabbed is held by the rollers: it still hits field structures, but not
// this robot, other FUEL, or the floor features the robot is driving over
const CAPTURED_GROUPS = groups(GROUP.BALL, GROUP.STATIC);
const INTAKE_SPEED = 3.5; // m/s the rollers pull FUEL in
const FEED_SPEED = 7;     // m/s up the feed path into the shooter
const CLIMB_LIFT = [0, 0.16, 0.74, 1.2]; // body lift to satisfy LEVEL 1/2/3 criteria

export class Robot {
  constructor({ cfg, alliance, physics, scene, field, fuel, match, climber }) {
    this.cfg = cfg;
    this.alliance = alliance;
    this.physics = physics;
    this.scene = scene;
    this.field = field;
    this.fuel = fuel;
    this.match = match;
    this.climberCfg = climber;

    const T = BUMPER_T;
    this.halfL = cfg.frame.length / 2 + T;
    this.halfW = cfg.frame.width / 2 + T;
    this.height = cfg.height;

    // ---- model
    this.model = buildRobotModel(cfg, alliance);
    if (climber) addClimberVisual(this.model, cfg);
    this.visual = new THREE.Group();
    this.visual.add(this.model.root);
    scene.add(this.visual);

    // ---- shot tables (hub + pass), built from the drag model
    const sh = cfg.shooter;
    const h0 = sh.type === 'fixed' ? sh.exit.y : sh.exitY;
    this.hubTable = new ShotTable({
      h0, Ht: HUB.targetHeight, hoodMin: sh.hoodMin, hoodMax: sh.hoodMax, speedMax: sh.speedMax, mode: 'hub',
      clearDist: HUB.size / 2 + FUEL.radius + 0.02, clearHeight: HUB.rimFront + FUEL.radius + 0.04,
    });
    this.passTable = new ShotTable({ h0, Ht: FUEL.radius + 0.02, hoodMin: sh.hoodMin, hoodMax: sh.hoodMax, speedMax: sh.speedMax, mode: 'pass', passTheta: 55 });

    this.hopper = new Hopper(cfg.bay);
    this._createBody();
    this.reset();
  }

  _createBody() {
    const { world } = this.physics;
    const cfg = this.cfg;
    const desc = RAPIER.RigidBodyDesc.dynamic()
      .enabledRotations(false, true, false)
      .setCcdEnabled(true)
      .setCanSleep(false)
      .setLinearDamping(0)
      .setAngularDamping(0);
    this.body = world.createRigidBody(desc);
    const robotGroups = groups(GROUP.ROBOT, GROUP.STATIC | GROUP.TERRAIN | GROUP.BALL | GROUP.ROBOT_BARRIER | GROUP.ROBOT);
    const y0 = 0.055, y1 = 0.16;
    const bumper = RAPIER.ColliderDesc.cuboid(this.halfL, (y1 - y0) / 2, this.halfW)
      .setTranslation(0, (y0 + y1) / 2, 0)
      .setMass(cfg.mass * 0.55)
      .setFriction(0.05).setFrictionCombineRule(RAPIER.CoefficientCombineRule.Min)
      .setRestitution(0.1)
      .setCollisionGroups(robotGroups);
    world.createCollider(bumper, this.body);
    const sL = cfg.frame.length / 2, sW = cfg.frame.width / 2;
    const upper = RAPIER.ColliderDesc.cuboid(sL, (this.height - y1) / 2, sW)
      .setTranslation(0, (this.height + y1) / 2, 0)
      .setMass(cfg.mass * 0.4)
      .setFriction(0.05).setFrictionCombineRule(RAPIER.CoefficientCombineRule.Min)
      .setRestitution(0.1)
      .setCollisionGroups(robotGroups);
    world.createCollider(upper, this.body);
    const inset = 0.075;
    for (const sx of [1, -1]) {
      for (const sz of [1, -1]) {
        const w = RAPIER.ColliderDesc.ball(WHEEL_R)
          .setTranslation(sx * (sL - inset), WHEEL_R, sz * (sW - inset))
          .setMass(cfg.mass * 0.0125)
          .setFriction(0).setFrictionCombineRule(RAPIER.CoefficientCombineRule.Min)
          .setRestitution(0)
          .setCollisionGroups(groups(GROUP.WHEEL, GROUP.STATIC | GROUP.TERRAIN | GROUP.ROBOT_BARRIER));
        world.createCollider(w, this.body);
      }
    }
    // extending hopper section: a real collider that slides out with the hopper
    const st = this.cfg.storage;
    this.hopperCollider = null;
    if (st.extLen && !st.inside) {
      this.hopperHalfH = (this.height - 0.17) / 2;
      this.hopperCollider = world.createCollider(
        RAPIER.ColliderDesc.cuboid(st.extLen / 2, this.hopperHalfH, cfg.frame.width / 2 - 0.02)
          .setTranslation(cfg.frame.length / 2 - st.extLen / 2, 0.17 + this.hopperHalfH, 0)
          .setMass(0.5)
          .setFriction(0.05).setRestitution(0.1)
          .setCollisionGroups(0),
        this.body,
      );
    }
    // deployed intake roller (pushes FUEL it cannot swallow)
    const ic = this.cfg.intake;
    this.intakeCollider = world.createCollider(
      RAPIER.ColliderDesc.cuboid(0.03, 0.05, ic.width / 2)
        .setTranslation(this.halfL + ic.reach - 0.05, 0.1, 0)
        .setMass(0.1)
        .setFriction(0.1).setRestitution(0.05)
        .setCollisionGroups(0),
      this.body,
    );
  }

  reset() {
    this.stored = [];
    this.hopper.clear();
    this.captured = [];
    this.lastT = 0;
    this.prevVel = null;
    this.prevOmega = 0;
    this.intakeDeploy = 0;
    this.hopperDeploy = 0;
    this.intakeSpeed = 0;
    this.flywheel = 0;
    this.hoodDeg = this.cfg.shooter.hoodMin;
    this.turretYaw = 0;
    this.feedTimer = 0;
    this.lane = 0;
    this.intakeTokens = 0;
    this.outtakeTimer = 0;
    this.enabled = false;
    this.cmd = { vx: 0, vz: 0, omega: 0, intake: false, outtake: false, shoot: false, pass: false };
    this.aimOverride = null;
    this.passTarget = null;
    this.shot = null;
    this.status = 'Idle';
    this.feeding = 0;
    this.climbState = 'none'; // none | aligning | climbing | hanging | descending
    this.climbLift = 0;
    this.climbTarget = this.climberCfg ? this.climberCfg.maxLevel : 0;
    this.climbLevel = 0;
    this.climbTime = 0;
    this.stats = { shots: 0, intaked: 0, passes: 0, dropped: 0 };
    this.lastInZone = false;
    this.preview = null;
  }

  spawn(x, z, yaw) {
    this.body.setBodyType(RAPIER.RigidBodyType.Dynamic, true);
    this.body.setTranslation({ x, y: 0.003, z }, true);
    this.body.setRotation(yawQuat(yaw), true);
    this.body.setLinvel({ x: 0, y: 0, z: 0 }, true);
    this.body.setAngvel({ x: 0, y: 0, z: 0 }, true);
    this.yaw = yaw;
    this.pos = new THREE.Vector3(x, 0, z);
    this.vel = new THREE.Vector3();
    this.omega = 0;
    this._syncVisual(0);
  }

  destroy() {
    this.hopper.clear();
    this.physics.world.removeRigidBody(this.body);
    this.scene.remove(this.visual);
  }

  // ------------------------------------------------------------------ geometry helpers
  forward() { return new THREE.Vector3(Math.cos(this.yaw), 0, -Math.sin(this.yaw)); }
  right() { return new THREE.Vector3(Math.sin(this.yaw), 0, Math.cos(this.yaw)); }

  localToWorld(lx, ly, lz) {
    const c = Math.cos(this.yaw), s = Math.sin(this.yaw);
    return new THREE.Vector3(this.pos.x + lx * c + lz * s, this.pos.y + ly, this.pos.z - lx * s + lz * c);
  }

  worldToLocal(x, z) {
    const dx = x - this.pos.x, dz = z - this.pos.z;
    const c = Math.cos(this.yaw), s = Math.sin(this.yaw);
    return { x: dx * c - dz * s, z: dx * s + dz * c };
  }

  corners() {
    return [[1, 1], [1, -1], [-1, 1], [-1, -1]].map(([a, b]) => this.localToWorld(a * this.halfL, 0, b * this.halfW));
  }

  // G407: BUMPERS partially or fully within the ALLIANCE ZONE
  inAllianceZone(alliance = this.alliance) {
    return this.corners().some((c) => Field.inAllianceZone(alliance, c.x));
  }

  velocityAt(p) {
    return { x: this.vel.x + this.omega * (p.z - this.pos.z), z: this.vel.z - this.omega * (p.x - this.pos.x) };
  }

  // how much FUEL fits right now (an extending hopper holds more when it's out)
  capacity() {
    const st = this.cfg.storage;
    if (!st.extLen) return st.capacity;
    return Math.floor(st.retracted + (st.capacity - st.retracted) * this.hopperDeploy + 1e-6);
  }

  maxCapacity() { return this.cfg.storage.capacity; }

  // front of the hopper right now (an extending hopper moves it forward)
  bayFront() {
    const st = this.cfg.storage, ext = st.extLen || 0;
    return st.inside ? this.cfg.bay.x1 - ext * (1 - this.hopperDeploy) : this.cfg.bay.x1 + ext * this.hopperDeploy;
  }

  // preloaded FUEL, dropped loosely into the hopper
  loadFuel(balls) {
    this.hopper.front = this.bayFront();
    for (const b of balls) { this.fuel.toRobot(b); this.stored.push(b); }
    this.hopper.fill(balls);
  }

  worldToLocal3(p) {
    const l = this.worldToLocal(p.x, p.z);
    return new THREE.Vector3(l.x, p.y - this.pos.y, l.z);
  }

  localToWorldVec(x, z) {
    const c = Math.cos(this.yaw), s = Math.sin(this.yaw);
    return { x: x * c + z * s, z: -x * s + z * c };
  }

  _hopper(dt, on) {
    const st = this.cfg.storage;
    if (!st.extLen) return;
    // 'latched' hoppers come out with the intake at the start and stay out; 'intake' hoppers
    // follow the intake. Either way the hopper can't close on FUEL that needs the room.
    let want = st.extend === 'latched' ? (this.hopperDeploy > 0.02 || this.intakeDeploy > 0.3 ? 1 : 0) : this.intakeDeploy;
    const need = Math.min(1, Math.max(0, (this.stored.length - st.retracted) / (st.capacity - st.retracted)));
    want = Math.max(want, need);
    if (on || want < this.hopperDeploy) this.hopperDeploy = approach(this.hopperDeploy, want, dt / 0.45);
    if (!this.hopperCollider) return;
    // collider follows the sliding section; it only hits field structures once clear of them
    const out = this.hopperDeploy * st.extLen;
    const fx = this.cfg.frame.length / 2 - st.extLen / 2 + out;
    this.hopperCollider.setTranslationWrtParent({ x: fx, y: 0.17 + this.hopperHalfH, z: 0 });
    let g = 0;
    if (this.hopperDeploy > 0.05) {
      const tip = this.cfg.frame.length / 2 + out, hw = this.cfg.frame.width / 2;
      const clear = [[tip, hw], [tip, -hw], [tip, 0]].every(([lx, lz]) => { const p = this.localToWorld(lx, 0, lz); return !obstacleAt(p.x, p.z, 0); });
      g = groups(GROUP.ROBOT, GROUP.BALL | GROUP.ROBOT | GROUP.ROBOT_BARRIER | (clear ? GROUP.STATIC : 0));
    }
    this.hopperCollider.setCollisionGroups(g);
  }

  // ------------------------------------------------------------------ fixed-step control (before world.step)
  preStep(dt) {
    if (this.climbState === 'climbing' || this.climbState === 'hanging' || this.climbState === 'descending') {
      this._climbStep(dt);
      return;
    }
    const d = this.cfg.drive;
    let tvx = 0, tvz = 0, tw = 0;
    if (this.enabled) {
      tvx = this.cmd.vx;
      tvz = this.cmd.vz;
      tw = this.cmd.omega;
      if (this.climbState === 'aligning') {
        const a = this._alignCommand();
        tvx = a.vx; tvz = a.vz; tw = a.omega;
      } else if (this.aimOverride !== null) {
        tw = this.aimOverride;
      }
    }
    const sp = Math.hypot(tvx, tvz);
    if (sp > d.maxSpeed) { tvx *= d.maxSpeed / sp; tvz *= d.maxSpeed / sp; }
    tw = clamp(tw, -d.maxOmega, d.maxOmega);
    const lv = this.body.linvel();
    let dvx = tvx - lv.x, dvz = tvz - lv.z;
    const dm = Math.hypot(dvx, dvz), maxDv = d.maxAccel * dt;
    if (dm > maxDv) { dvx *= maxDv / dm; dvz *= maxDv / dm; }
    this.body.setLinvel({ x: lv.x + dvx, y: lv.y, z: lv.z + dvz }, true);
    const av = this.body.angvel();
    const w = approach(av.y, tw, d.maxAlpha * dt);
    this.body.setAngvel({ x: 0, y: w, z: 0 }, true);
    this._guideCaptured(dt);
  }

  // Where FUEL crosses into the hopper: over the bumper, then in at the front of the hopper
  _intakePath(z) {
    const front = this.bayFront();
    // just over the bumper (an extended hopper already reaches out over it)
    const entryX = Math.min(front - R - 0.02, this.halfL + 0.02);
    const entryY = this.hopper.floorAt(entryX, z) + R + 0.015;
    const lipX = this.halfL + 0.04;
    return { lipX, entryX, entryY, liftY: Math.max(entryY, BUMP_Y1 + R + 0.025) };
  }

  // The intake rollers drag grabbed FUEL up over the bumper and into the hopper
  _guideCaptured(dt) {
    if (!this.captured.length) return;
    const tr = this.body.translation(), lv = this.body.linvel(), w = this.body.angvel().y;
    const c = Math.cos(this.yaw), sn = Math.sin(this.yaw);
    for (const cap of this.captured) {
      const b = cap.b;
      const p = b.body.translation();
      const dx = p.x - tr.x, dz = p.z - tr.z;
      const lx = dx * c - dz * sn, lz = dx * sn + dz * c, ly = p.y - tr.y;
      const path = this._intakePath(cap.z);
      // up the intake arm to just over the bumper, then into the hopper
      if (!cap.over && ly >= path.liftY - 0.03) cap.over = true;
      const up = !cap.over;
      const tx = up ? path.lipX : path.entryX, ty = up ? path.liftY : path.entryY;
      let vx = tx - lx, vy = ty - ly, vz = cap.z - lz;
      const dist = Math.hypot(vx, vy, vz) || 1;
      const sp = Math.min(INTAKE_SPEED, dist / (2 * dt));
      vx *= sp / dist; vy *= sp / dist; vz *= sp / dist;
      // robot point velocity + the pull, in world axes
      b.body.setLinvel({
        x: lv.x + w * dz + vx * c + vz * sn,
        y: vy + 9.81 * dt,
        z: lv.z - w * dx - vx * sn + vz * c,
      }, true);
    }
  }

  // ------------------------------------------------------------------ after world.step
  postStep(dt, t) {
    const tr = this.body.translation();
    const q = this.body.rotation();
    this.pos.set(tr.x, tr.y, tr.z);
    this.yaw = 2 * Math.atan2(q.y, q.w);
    const lv = this.body.linvel();
    this.vel.set(lv.x, 0, lv.z);
    this.omega = this.body.angvel().y;

    const on = this.enabled && this.climbState === 'none';
    this.lastT = t;
    this._intake(dt, t, on);
    this._shooter(dt, t, on);
    this._stepHopper(dt);
  }

  _stepHopper(dt) {
    // the robot's acceleration and turn rate, felt by the FUEL inside
    let ax = 0, az = 0, alpha = 0;
    if (this.prevVel) {
      ax = clamp((this.vel.x - this.prevVel.x) / dt, -25, 25);
      az = clamp((this.vel.z - this.prevVel.z) / dt, -25, 25);
      alpha = clamp((this.omega - this.prevOmega) / dt, -60, 60);
    }
    this.prevVel = { x: this.vel.x, z: this.vel.z };
    this.prevOmega = this.omega;
    const acc = this.worldToLocalVec(ax, az);
    const f = this.cfg.bay.feed;
    this.hopper.front = this.bayFront();
    this.hopper.step(dt, {
      acc, w: this.omega, alpha,
      feeding: this.feeding > 0,
      intaking: this.intakeSpeed > 0,
      feedPoint: new THREE.Vector3(f.x, 0, f.z ?? 0),
    });
  }

  _intake(dt, t, on) {
    const ic = this.cfg.intake;
    let want = on && this.cmd.intake;
    if (ic.latched && this.intakeDeploy >= 1) want = true; // latched down for the whole match
    if (this.forceDeploy) want = true;
    // Re•Blitz retracts the intake while shooting to compress FUEL into the shooter
    if (this.cfg.key === '2910' && on && (this.cmd.shoot || this.cmd.pass) && !this.cmd.intake) want = false;
    if (on && this.cmd.outtake) want = true;
    this.intakeDeploy = approach(this.intakeDeploy, want ? 1 : 0, dt / ic.deployTime);
    this._hopper(dt, on);
    const deployed = this.intakeDeploy > 0.85;
    // An intake that deploys into a structure (e.g. 4414 at the Hub start) would jam the robot,
    // so it only collides with field structures once it is clear of them.
    let groupsNow = 0;
    if (deployed) {
      const tip = this.halfL + ic.reach, hw = ic.width / 2;
      const clear = [[tip, hw], [tip, -hw], [tip, 0], [this.halfL + 0.05, hw], [this.halfL + 0.05, -hw]]
        .every(([lx, lz]) => { const p = this.localToWorld(lx, 0, lz); return !obstacleAt(p.x, p.z, 0); });
      groupsNow = groups(GROUP.INTAKE, GROUP.BALL | GROUP.ROBOT_BARRIER | (clear ? GROUP.STATIC : 0));
    }
    this.intakeCollider.setCollisionGroups(groupsNow);

    const running = on && this.cmd.intake && deployed;
    this.intakeSpeed = on && this.cmd.outtake ? -1 : running ? 1 : 0;
    if (running && this.stored.length + this.captured.length < this.capacity()) {
      this.intakeTokens = Math.min(4, this.intakeTokens + ic.rate * dt);
      const front = this.halfL - 0.04;
      const reach = this.halfL + ic.reach + R + 0.02;
      const hw = ic.width / 2 + 0.02;
      const balls = this.fuel.balls;
      for (let i = 0; i < balls.length && this.intakeTokens >= 1; i++) {
        const b = balls[i];
        if (b.state !== 'field' || b.captor) continue;
        const p = b.pos;
        if (Math.abs(p.x - this.pos.x) > 1.3 || Math.abs(p.z - this.pos.z) > 1.3) continue;
        if (p.y - this.pos.y > 0.33) continue;
        const l = this.worldToLocal(p.x, p.z);
        if (l.x < front || l.x > reach || Math.abs(l.z) > hw) continue;
        if (b.hubFresh) this.match.addFoul(this.alliance, 'minor', 'G408', 'Caught FUEL released by the HUB before it touched the carpet');
        // grabbed by the rollers: it stops colliding with this robot and gets pulled in
        b.captor = this;
        b.hubFresh = false;
        b.inFlight = false;
        b.launch = null;
        b.ignoring = false;
        b.col.setCollisionGroups(CAPTURED_GROUPS);
        b.body.wakeUp();
        const hz = this.cfg.bay.hw - R - 0.01;
        this.captured.push({ b, t0: t, z: clamp(l.z, -Math.min(hz, ic.width / 2 - R), Math.min(hz, ic.width / 2 - R)) });
        this.intakeTokens -= 1;
        if (this.stored.length + this.captured.length >= this.capacity()) break;
      }
    } else {
      this.intakeTokens = 0;
    }
    this._settleCaptured(t, running);
    // outtake: FUEL goes back out over the intake
    if (on && this.cmd.outtake && this.stored.length) {
      this.outtakeTimer += dt;
      while (this.outtakeTimer >= 0.08) {
        this.outtakeTimer -= 0.08;
        if (!this._startOuttake()) break;
      }
    } else this.outtakeTimer = 0;
  }

  // Grabbed FUEL that reached the hopper joins it; FUEL the intake lets go of is released
  _settleCaptured(t, running) {
    for (let i = this.captured.length - 1; i >= 0; i--) {
      const cap = this.captured[i];
      const b = cap.b;
      const l = this.worldToLocal3(b.pos);
      const path = this._intakePath(cap.z);
      const lost = b.state !== 'field' || t - cap.t0 > 0.8;
      if (!lost && l.x <= path.entryX + 0.02 && l.y >= path.entryY - 0.04) {
        this.captured.splice(i, 1);
        const v = b.body.linvel();
        const pv = this.velocityAt(b.pos);
        const lv = this.worldToLocalVec(v.x - pv.x, v.z - pv.z);
        this.fuel.toRobot(b);
        b.captor = null;
        this.stored.push(b);
        this.stats.intaked++;
        this.hopper.add(b, l, new THREE.Vector3(lv.x, v.y, lv.z));
      } else if (lost || (!running && !cap.over)) {
        // not over the bumper yet: it drops back onto the carpet
        this.captured.splice(i, 1);
        b.captor = null;
        this.stats.dropped++;
        if (b.state === 'field') { b.ignoring = true; b.ignoreUntil = t + 0.25; }
      }
    }
  }

  _startOuttake() {
    const front = this.bayFront();
    let best = null, bd = Infinity;
    for (const e of this.hopper.list) {
      if (e.tr) continue;
      const d = (front - e.p.x) + 0.5 * e.p.y;
      if (d < bd) { bd = d; best = e; }
    }
    if (!best) return false;
    const z = clamp(best.p.z, -this.cfg.intake.width / 2 + 0.08, this.cfg.intake.width / 2 - 0.08);
    const path = this._intakePath(z);
    const out = new THREE.Vector3(Math.max(this.halfL, front) + 0.2, 0.18, z);
    this.hopper.startTransit(best, [new THREE.Vector3(path.entryX, path.entryY, z), new THREE.Vector3(path.lipX, path.liftY, z)], () => out, 5, (e) => {
      this.hopper.remove(e);
      this._unstore(e.b);
      const p = this.localToWorld(e.p.x, e.p.y, e.p.z);
      const f = this.forward();
      const v = this.velocityAt(p);
      this.fuel.launch(e.b, p, { x: v.x + f.x * 2.8, y: 0.6, z: v.z + f.z * 2.8 }, { by: 'robot', alliance: this.alliance, legal: this.inAllianceZone(), t: this.lastT, ignoreRobot: 0.3 });
    });
    return true;
  }

  _unstore(b) {
    const i = this.stored.indexOf(b);
    if (i >= 0) this.stored.splice(i, 1);
  }

  // Target selection: HUB when legal (in own ALLIANCE ZONE), otherwise pass into own ALLIANCE ZONE
  _target() {
    const inZone = this.inAllianceZone();
    const passMode = this.cmd.pass || !inZone;
    if (!passMode) {
      const c = Field.hubCenter(this.alliance);
      return { mode: 'hub', x: c.x, z: c.z, table: this.hubTable, inZone };
    }
    // an AI can pick where its passes land (feeding its own zone); otherwise the nearest
    // corner of our ALLIANCE ZONE (away from the HUB)
    if (this.passTarget) return { mode: 'pass', x: this.passTarget.x, z: this.passTarget.z, table: this.passTable, inZone };
    const s = this.alliance === BLUE ? 1 : -1;
    // aimed well inside the corner so long lobs that scatter or bounce stay on the FIELD (G405)
    const x = s * (-HALF_L + 1.7);
    const zc = [HALF_W - 1.8, -HALF_W + 1.8];
    const z = Math.abs(this.pos.z - zc[0]) < Math.abs(this.pos.z - zc[1]) ? zc[0] : zc[1];
    return { mode: 'pass', x, z, table: this.passTable, inZone };
  }

  _exitPoint(laneZ = 0) {
    const sh = this.cfg.shooter;
    if (sh.type === 'fixed') return this.localToWorld(sh.exit.x, sh.exit.y, laneZ);
    return this.localToWorld(sh.turretPos.x, sh.exitY, sh.turretPos.z);
  }

  _shooter(dt, t, on) {
    const sh = this.cfg.shooter;
    const wantShoot = on && (this.cmd.shoot || this.cmd.pass);
    const has = this.stored.length > 0;
    this.aimOverride = null;
    this.feeding = 0;
    this.shot = null;
    this.preview = null;
    let setpoint = 0;
    let ready = false;
    let status = has ? 'Holding FUEL' : 'Empty';

    const tgt = this._target();
    const inZoneNow = tgt.inZone;
    this.lastInZone = inZoneNow;
    const prespin = on && has && inZoneNow;
    if ((wantShoot || prespin) && has) {
      let exit, lv;
      if (sh.type === 'fixed') {
        // The drum sits on the robot's centerline, so once aimed the shot line passes through the
        // robot center. Solve from a point that does not move when the chassis rotates.
        const dx = tgt.x - this.pos.x, dz = tgt.z - this.pos.z;
        const dd = Math.hypot(dx, dz) || 1;
        exit = new THREE.Vector3(this.pos.x + (dx / dd) * sh.exit.x, this.pos.y + sh.exit.y, this.pos.z + (dz / dd) * sh.exit.x);
        lv = { x: this.vel.x, z: this.vel.z };
      } else {
        // FUEL leaves the hood exitRadius in front of the turret axis, along the shot line
        const c = this._exitPoint(0);
        const dx = tgt.x - c.x, dz = tgt.z - c.z;
        const dd = Math.hypot(dx, dz) || 1;
        exit = new THREE.Vector3(c.x + (dx / dd) * sh.exitRadius, c.y, c.z + (dz / dd) * sh.exitRadius);
        lv = this.velocityAt(exit);
      }
      const sol = solveMovingShot(tgt.table, exit, lv, tgt);
      if (sol) {
        this.shot = { ...sol, mode: tgt.mode, target: tgt, exit, lv };
        setpoint = sol.v;
        this.hoodDeg = approach(this.hoodDeg, sol.theta / DEG, 260 * dt);
        let aimErr;
        if (sh.type === 'fixed') {
          aimErr = wrapAngle(sol.psi - this.yaw);
          if (wantShoot) {
            // chassis heading controller: time-optimal profile (no overshoot) + feed-forward on
            // how fast the aim direction moves while driving
            const d = this.cfg.drive;
            const ff = this._lastPsi !== undefined ? clamp(wrapAngle(sol.psi - this._lastPsi) / dt, -3, 3) : 0;
            const a = Math.abs(aimErr);
            const w = Math.min(d.maxOmega, Math.sqrt(2 * 0.7 * d.maxAlpha * a), 7 * a);
            this.aimOverride = clamp(Math.sign(aimErr) * w + ff, -d.maxOmega, d.maxOmega);
          }
        } else {
          let rel = wrapAngle(sol.psi - this.yaw);
          const lim = sh.turretRange * DEG;
          // choose the equivalent angle closest to the current turret position within limits
          const cands = [rel, rel + 2 * Math.PI, rel - 2 * Math.PI].filter((a) => Math.abs(a) <= lim);
          let goal = cands.length ? cands.reduce((a, b) => (Math.abs(b - this.turretYaw) < Math.abs(a - this.turretYaw) ? b : a)) : clamp(rel, -lim, lim);
          this.turretYaw = approach(this.turretYaw, goal, sh.turretRate * DEG * dt);
          aimErr = wrapAngle(sol.psi - (this.yaw + this.turretYaw));
        }
        this._lastPsi = sol.psi;
        const tol = tgt.mode === 'hub' ? Math.max(0.6 * DEG, Math.atan2(0.14, sol.dist)) : 4 * DEG;
        const spinOk = Math.abs(this.flywheel - sol.v) / sol.v < 0.025;
        const hoodOk = Math.abs(this.hoodDeg - sol.theta / DEG) < 1.5;
        const aimOk = Math.abs(aimErr) < tol;
        ready = spinOk && hoodOk && aimOk;
        status = !spinOk ? 'Spinning up' : !aimOk ? 'Aiming' : !hoodOk ? 'Hood' : 'READY';
        if (tgt.mode === 'pass') status = ready ? 'PASS READY' : 'Pass: ' + status.toLowerCase();
        this.preview = { exit, lv, sol };
      } else {
        status = tgt.mode === 'hub' ? 'Out of range' : 'Pass out of range';
      }
    } else if (!inZoneNow && has) {
      status = 'Outside ALLIANCE ZONE';
    }
    if (!this.shot) this._lastPsi = undefined;
    if (!on) status = this.enabled ? status : 'Disabled';

    // keep the flywheel at its last shot speed for a moment after the trigger is released, so
    // stop-and-go shooting (or passing) doesn't have to spin up from scratch every time
    if (setpoint > 0) { this.spinHold = setpoint; this.spinHoldT = t; }
    else if (on && this.spinHold && t - this.spinHoldT < 1.5) setpoint = this.spinHold;
    // flywheel dynamics: torque-limited spin-up (0 -> max in spinTau*2), fast closed-loop
    // settle near the setpoint, slow coast-down
    const target = Math.min(setpoint, sh.speedMax);
    const err = target - this.flywheel;
    if (err > 0) this.flywheel += Math.min(err * (1 - Math.exp(-dt / 0.06)), (sh.speedMax / (2 * sh.spinTau)) * dt);
    else this.flywheel += Math.max(err * (1 - Math.exp(-dt / 0.06)), -(sh.speedMax / 3) * dt);

    // feed
    const period = 1 / sh.bps;
    this.feedTimer = Math.min(this.feedTimer + dt, period * 1.5);
    if (wantShoot && ready && has && this.shot) {
      // same rate as ever: one FUEL starts up the feed path every period
      while (this.feedTimer >= period && this._startFeed()) this.feedTimer -= period;
      this.feeding = 1;
    }
    if (this.hopper.list.some((e) => e.tr && e.tr.feed && e.tr.t < e.tr.T)) this.feeding = 1;
    this.status = status;
    this.ready = ready;
  }

  // The FUEL nearest the feed point starts up the feed path (indexer, ramp, turret) and leaves
  // the shooter when it gets there.
  _startFeed() {
    const sh = this.cfg.shooter, f = this.cfg.bay.feed;
    let laneZ = f.z ?? 0;
    if (sh.type === 'fixed') laneZ = sh.lanes[this.lane % sh.lanes.length];
    const fp = new THREE.Vector3(f.x, this.hopper.floorAt(f.x, laneZ) + R, laneZ);
    let best = null, bd = Infinity;
    for (const e of this.hopper.list) {
      if (e.tr) continue;
      const d = e.p.distanceToSquared(fp);
      if (d < bd) { bd = d; best = e; }
    }
    if (!best) return false;
    if (sh.type === 'fixed') this.lane++;
    const via = f.via.map(([x, y, z]) => new THREE.Vector3(x, y, z ?? laneZ));
    const end = sh.type === 'fixed'
      ? () => new THREE.Vector3(sh.exit.x, sh.exit.y, laneZ)
      : () => new THREE.Vector3(sh.turretPos.x + Math.cos(this.turretYaw) * sh.exitRadius, sh.exitY, sh.turretPos.z - Math.sin(this.turretYaw) * sh.exitRadius);
    this.hopper.startTransit(best, via, end, FEED_SPEED, (e) => {
      // a FUEL that reaches the wheels while the shot isn't lined up waits there
      if (this.ready && this.shot && this.enabled && (this.cmd.shoot || this.cmd.pass)) this._fire(e, this.shot.mode);
    });
    best.tr.feed = true;
    return true;
  }

  _fire(e, mode) {
    const sh = this.cfg.shooter;
    const b = e.b;
    this.hopper.remove(e);
    this._unstore(b);
    const exit = this.localToWorld(e.p.x, e.p.y, e.p.z);
    let psi = sh.type === 'fixed' ? this.yaw : this.yaw + this.turretYaw;
    const k = this.noiseScale ?? 1; // AI skill: extra scatter for weaker drivers
    psi += gauss() * sh.yawSigma * k * DEG;
    const th = this.hoodDeg * DEG + gauss() * sh.angleSigma * k * DEG;
    const v = this.flywheel * (1 + gauss() * sh.speedSigma * k);
    const lv = this.velocityAt(exit);
    const vel = {
      x: Math.cos(psi) * Math.cos(th) * v + lv.x,
      y: Math.sin(th) * v,
      z: -Math.sin(psi) * Math.cos(th) * v + lv.z,
    };
    this.fuel.launch(b, exit, vel, { by: 'robot', alliance: this.alliance, legal: this.lastInZone, t: this.lastT, ignoreRobot: 0.35, spin: true });
    this.flywheel *= 1 - sh.shotDrop;
    this.stats.shots++;
    if (mode === 'pass') this.stats.passes++;
  }

  // ------------------------------------------------------------------ climbing (optional add-on)
  // The add-on climber is on the back of the robot, so it backs up to the TOWER's RUNGS
  climbPose() {
    const s = this.alliance === BLUE ? 1 : -1;
    const ladderX = -HALF_L + TOWER.uprightFx + TOWER.uprightDeep / 2;
    const x = ladderX + this.halfL + 0.03;
    const z = HALF_W - TOWER.fy;
    return { x: s * x, z: s * z, yaw: this.alliance === BLUE ? 0 : Math.PI };
  }

  canStartClimb() {
    if (!this.climberCfg || this.climbState !== 'none') return false;
    const p = this.climbPose();
    return Math.hypot(this.pos.x - p.x, this.pos.z - p.z) < 1.4;
  }

  requestClimb() {
    if (!this.climberCfg) return 'No climber installed';
    if (this.climbState === 'none') {
      if (!this.canStartClimb()) return 'Drive to the front of your TOWER to climb';
      this.climbState = 'aligning';
      return null;
    }
    if (this.climbState === 'aligning') { this.climbState = 'none'; return null; }
    if (this.climbState === 'hanging' && this.climbTarget > this.climbLevel) { this.climbState = 'climbing'; return null; }
    this.climbState = 'descending';
    return null;
  }

  _alignCommand() {
    const p = this.climbPose();
    const ex = p.x - this.pos.x, ez = p.z - this.pos.z;
    const eyaw = wrapAngle(p.yaw - this.yaw);
    const dist = Math.hypot(ex, ez);
    if (dist < 0.03 && Math.abs(eyaw) < 2 * DEG && this.vel.length() < 0.12) {
      this.climbState = 'climbing';
      this.climbTime = 0;
      this.body.setBodyType(RAPIER.RigidBodyType.KinematicPositionBased, true);
      this.climbBase = { x: this.pos.x, y: this.pos.y, z: this.pos.z };
      return { vx: 0, vz: 0, omega: 0 };
    }
    const sp = Math.min(2.2, dist * 3);
    return { vx: (ex / (dist || 1)) * sp, vz: (ez / (dist || 1)) * sp, omega: clamp(eyaw * 6, -4, 4) };
  }

  _climbStep(dt) {
    const c = this.climberCfg;
    const target = CLIMB_LIFT[this.climbTarget];
    if (this.climbState === 'climbing') {
      // time-based profile between levels
      const lvl = this.climbLevelFromLift(this.climbLift);
      const nextLvl = Math.min(this.climbTarget, lvl + 1);
      const segT = Math.max(0.3, (c.times[nextLvl] ?? 2) - (c.times[nextLvl - 1] ?? 0));
      const segH = CLIMB_LIFT[nextLvl] - CLIMB_LIFT[nextLvl - 1];
      this.climbLift = Math.min(target, this.climbLift + (segH / segT) * dt);
      if (this.climbLift >= target - 1e-4) this.climbState = 'hanging';
    } else if (this.climbState === 'descending') {
      this.climbLift = Math.max(0, this.climbLift - 0.45 * dt);
      if (this.climbLift <= 0) {
        this.climbState = 'none';
        this.body.setBodyType(RAPIER.RigidBodyType.Dynamic, true);
        this.body.setLinvel({ x: 0, y: 0, z: 0 }, true);
      }
    }
    this.climbLevel = this.climbState === 'hanging' || this.climbState === 'climbing' || this.climbState === 'descending'
      ? this.climbLevelFromLift(this.climbLift) : 0;
    if (this.climbState !== 'none') {
      const b = this.climbBase;
      this.body.setNextKinematicTranslation({ x: b.x, y: b.y + this.climbLift, z: b.z });
    }
  }

  climbLevelFromLift(lift) {
    let l = 0;
    for (let i = 1; i <= 3; i++) if (lift >= CLIMB_LIFT[i] - 1e-3) l = i;
    return l;
  }

  // ------------------------------------------------------------------ visuals
  update(dt) {
    this._syncVisual(dt);
  }

  _syncVisual(dt) {
    const v = this.visual;
    const f = this.field;
    // visual pitch/roll from the terrain under the wheels (physics body stays level)
    let pitch = 0, roll = 0;
    if (this.climbState === 'none' && f) {
      const L = this.cfg.frame.length / 2 - 0.075, W = this.cfg.frame.width / 2 - 0.075;
      const h = (lx, lz) => { const p = this.localToWorld(lx, 0, lz); return f.terrainHeight(p.x, p.z); };
      const hf = Math.max(h(L, W), h(L, -W)), hb = Math.max(h(-L, W), h(-L, -W));
      const hr = Math.max(h(L, W), h(-L, W)), hl = Math.max(h(L, -W), h(-L, -W));
      pitch = Math.atan2(hf - hb, 2 * L);
      roll = Math.atan2(hr - hl, 2 * W);
    } else if (this.climbState !== 'none') {
      pitch = -Math.min(0.12, this.climbLift * 0.15);
    }
    v.position.copy(this.pos);
    if (this.climbState === 'none' && f) {
      const L = this.cfg.frame.length / 2 - 0.075, W = this.cfg.frame.width / 2 - 0.075;
      const hs = [[L, W], [L, -W], [-L, W], [-L, -W]].map(([a, b]) => { const p = this.localToWorld(a, 0, b); return f.terrainHeight(p.x, p.z); });
      const hmax = Math.max(...hs), havg = (hs[0] + hs[1] + hs[2] + hs[3]) / 4;
      v.position.y = havg + Math.max(0, this.pos.y - hmax);
    }
    v.rotation.set(0, 0, 0);
    v.rotateY(this.yaw);
    v.rotateZ(pitch);
    v.rotateX(-roll);
    // stored FUEL
    const m = this.model;
    const list = this.hopper.list;
    const n = Math.min(list.length, m.storedMesh.instanceMatrix.count);
    const mat = new THREE.Matrix4();
    for (let i = 0; i < n; i++) {
      mat.makeTranslation(list[i].p.x, list[i].p.y, list[i].p.z);
      m.storedMesh.setMatrixAt(i, mat);
    }
    m.storedMesh.count = n;
    m.storedMesh.instanceMatrix.needsUpdate = true;
    // mechanisms
    const sh = this.cfg.shooter;
    m.anim({
      intakeDeploy: this.intakeDeploy,
      intakeSpeed: this.intakeSpeed,
      hopperDeploy: this.hopperDeploy,
      flywheel: this.flywheel,
      feeding: this.feeding,
      hoodDeg: this.hoodDeg,
      turretYaw: this.turretYaw,
    }, dt);
    // swerve module steering to match the motion
    const lv = this.worldToLocalVec(this.vel.x, this.vel.z);
    for (const mod of m.modules) {
      const p = mod.pivot.parent.position;
      const vx = lv.x + this.omega * p.z;
      const vz = lv.z - this.omega * p.x;
      if (Math.hypot(vx, vz) > 0.05) mod.pivot.rotation.y = -Math.atan2(vz, vx);
      mod.wheel.rotation.y -= Math.hypot(vx, vz) * dt / WHEEL_R;
    }
    if (m.climber) {
      m.climber.hook.position.y = 0.55 + (this.climbState === 'aligning' ? 0.25 : this.climbState !== 'none' ? Math.max(0, 0.25 - this.climbLift) : 0);
    }
  }

  worldToLocalVec(x, z) {
    const c = Math.cos(this.yaw), s = Math.sin(this.yaw);
    return { x: x * c - z * s, z: x * s + z * c };
  }

  previewPoints() {
    if (!this.preview) return null;
    const { exit, lv, sol } = this.preview;
    const psi = this.cfg.shooter.type === 'fixed' ? this.yaw : this.yaw + this.turretYaw;
    const v = this.flywheel > 0.5 ? Math.max(this.flywheel, 0) : sol.v;
    const th = this.hoodDeg * DEG;
    return trajectoryPoints(exit, {
      x: Math.cos(psi) * Math.cos(th) * v + lv.x,
      y: Math.sin(th) * v,
      z: -Math.sin(psi) * Math.cos(th) * v + lv.z,
    });
  }
}
