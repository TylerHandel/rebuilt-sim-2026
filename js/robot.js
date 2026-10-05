import * as THREE from 'three';
import { RAPIER, yawQuat } from './physics.js';
import {
  BLUE, RED, HALF_L, HALF_W, HUB, FUEL, TOWER, TRENCH, GROUP, groups, IN,
} from './constants.js';
import { BUMPER_T, BUMPER_Y0, BUMPER_Y1, fitsTrench, foldTop, frameShape, offsetShape, modulePoints } from './robotConfigs.js';
import { buildRobotModel, addClimberVisual, BUMP_Y1 } from './robotModels.js';
import { Hopper, measureCapacity, intakePush, fuelForce } from './hopper.js';
import { ShotTable, solveMovingShot, trajectoryPoints } from './ballistics.js';
import { Field } from './field.js';
import { TRENCH_ARMS } from './nav.js';
import { Drivetrain } from './drivetrain.js';
import { clamp, wrapAngle, approach, approachAngle, gauss, DEG, rand } from './util.js';

const WHEEL_R = 0.05;
const TIP_RATE = 2.5; // rad/s: the most a robot pitches or rolls (see preStep)
const FLOOR_QUERY = groups(GROUP.WHEEL, GROUP.STATIC | GROUP.TERRAIN); // what the wheels drive on
const R = FUEL.radius;
// FUEL the intake has grabbed is held by the rollers: it still hits field structures, but not
// this robot, other FUEL, or the floor features the robot is driving over
const CAPTURED_GROUPS = groups(GROUP.BALL, GROUP.STATIC);
const INTAKE_SPEED = 3.5; // m/s the rollers pull FUEL in (cfg.intake.pull overrides)
// A FUEL goes in only once the one before it is out of the way: while one is within BLOCK_D of
// the entry, the next waits in the mouth HOLD_GAP out from it and pushes it on in. The rollers
// stall once one has waited STALL_T (the hopper is full, for now), and it sits in the roller
// (sticking HOLD_OUT past the front of the intake) until the load gives way.
const BLOCK_D = 2 * R * 0.85;
const HOLD_GAP = 2 * R * 0.9;
const HOLD_OUT = 0.06;
const STALL_T = 0.35;
const FULL_T = 0.75; // s stalled: full
const FEED_SPEED = 7;     // m/s up the feed path into the shooter
const CLIMB_LIFT = [0, 0.16, 0.74, 1.2]; // body lift to satisfy LEVEL 1/2/3 criteria

let robotCount = 0;

const _q = new THREE.Quaternion(), _v = new THREE.Vector3(), _v2 = new THREE.Vector3();

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
    this.uid = ++robotCount; // tells robots apart (two can share an alliance and a model)
    this.halfL = cfg.frame.length / 2 + T;
    this.halfW = cfg.frame.width / 2 + T;
    this.height = cfg.height;

    // ---- model
    this.model = buildRobotModel(cfg, alliance);
    if (climber && !cfg.climber) addClimberVisual(this.model, cfg); // a built-in climber is in the model
    this.visual = new THREE.Group();
    this.visual.add(this.model.root);
    scene.add(this.visual);

    // ---- shot tables (hub + pass), built from the drag model
    const sh = cfg.shooter;
    // a fixed shooter fires along the robot's heading, or straight out the back (2910)
    this.aimOffset = sh.type === 'fixed' && sh.facing === 'back' ? Math.PI : 0;
    const h0 = sh.type === 'fixed' ? sh.exit.y : sh.exitY;
    this.hubTable = new ShotTable({
      h0, Ht: HUB.targetHeight, hoodMin: sh.hoodMin, hoodMax: sh.hoodMax, speedMax: sh.speedMax, mode: 'hub',
      clearDist: HUB.size / 2 + FUEL.radius + 0.02, clearHeight: HUB.rimFront + FUEL.radius + 0.04,
    });
    this.passTable = new ShotTable({ h0, Ht: FUEL.radius + 0.02, hoodMin: sh.hoodMin, hoodMax: sh.hoodMax, speedMax: sh.speedMax, mode: 'pass', passTheta: 55 });

    // turret shooters: one or more turrets (971 has two), each aiming itself
    this.turrets = sh.type === 'turret' ? (sh.turrets || [sh.turretPos]) : [];
    this.hopper = new Hopper(cfg.bay);
    // the drivetrain: force vs speed from its motors, gearing, battery and tires
    const [mx, mz] = modulePoints(cfg)[0];
    this.drivetrain = new Drivetrain(cfg.drive, cfg.mass, cfg.frame.length, cfg.frame.width, Math.hypot(mx, mz));
    const ext = cfg.storage.extLen || 0;
    // retracted: hopper in and any lid (1678's, on the climber) down
    this.intakePush = intakePush(cfg); // N: how hard the intake shoves FUEL into the hopper
    this.geoCap = { retracted: measureCapacity(cfg.bay, cfg.bay.x1, 0, this.intakePush), extended: measureCapacity(cfg.bay, cfg.bay.x1 + ext, 1, this.intakePush) };
    // a compacting intake folding in crams the load harder than the roller packs it (compactPush):
    // what the hopper holds without its extension, packed that hard
    if (cfg.intake.compacts) this.geoCap.compacted = Math.max(this.geoCap.retracted, measureCapacity(cfg.bay, cfg.bay.x1, 0, cfg.intake.compactPush ?? this.intakePush));
    this._createBody();
    this.reset();
  }

  _createBody() {
    const { world } = this.physics;
    const cfg = this.cfg;
    // free to pitch and roll: it rides up BUMPS, over DEPOT barriers and onto jammed FUEL on
    // its wheels (the drive only commands the horizontal velocity and the yaw rate)
    const desc = RAPIER.RigidBodyDesc.dynamic()
      .setCcdEnabled(true)
      .setCanSleep(false)
      .setLinearDamping(0)
      .setAngularDamping(4);
    this.body = world.createRigidBody(desc);
    const robotGroups = groups(GROUP.ROBOT, GROUP.STATIC | GROUP.TERRAIN | GROUP.BALL | GROUP.ROBOT_BARRIER | GROUP.ROBOT | GROUP.INTAKE);
    const y0 = BUMPER_Y0, y1 = BUMPER_Y1;
    // Bumpers: straight up and down, so two robots meet face to face and push level (a fully
    // rounded top lets one ride up the other and lever it over); the bottom edge is chamfered, so a
    // pile of FUEL can still lift it rather than stop it dead, and the top edge a little (the pool
    // noodles are round), so a robot that lands on another's bumper slides off rather than
    // perching there. Rounded corners in plan.
    // The hull follows the frame's own shape (frameShape: 4414's cut corners, 4946's round back),
    // BUMPER_T out, with rounded corners in plan.
    const bc = 0.03, cr = 0.04, pts = [];
    const shape = frameShape(cfg);
    for (const [y, inset] of [[y0, bc], [y0 + bc, 0], [y1 - 0.025, 0], [y1, 0.025]]) {
      for (const [x, z] of offsetShape(shape, BUMPER_T - inset - cr)) {
        for (let k = 0; k < 8; k++) { const a = (k / 8) * Math.PI * 2; pts.push(x + cr * Math.cos(a), y, z + cr * Math.sin(a)); }
      }
    }
    const bumper = RAPIER.ColliderDesc.convexHull(new Float32Array(pts))
      .setMass(cfg.mass * 0.15)
      .setFriction(0.05).setFrictionCombineRule(RAPIER.CoefficientCombineRule.Min)
      .setRestitution(0.1)
      .setCollisionGroups(robotGroups);
    world.createCollider(bumper, this.body);
    const sL = cfg.frame.length / 2, sW = cfg.frame.width / 2;
    // the frame and superstructure above the bumpers: solid to the field and to other robots (a
    // robot coming down off a BUMP onto another lands on it, not through it); level robots only
    // meet bumper to bumper, since the frame sits inside the bumpers
    // (its lower edge chamfered, so a frame that ends up on another robot's bumper slides off)
    const up = [];
    for (const [x, z] of offsetShape(shape, -0.05)) up.push(x, y1, z);
    for (const [x, z] of shape) up.push(x, y1 + 0.05, z, x, this.height, z);
    const upper = RAPIER.ColliderDesc.convexHull(new Float32Array(up))
      .setMass(cfg.mass * 0.3)
      .setFriction(0.05).setFrictionCombineRule(RAPIER.CoefficientCombineRule.Min)
      .setRestitution(0.1)
      .setCollisionGroups(groups(GROUP.ROBOT, GROUP.STATIC | GROUP.TERRAIN | GROUP.BALL | GROUP.ROBOT_BARRIER | GROUP.ROBOT | GROUP.INTAKE));
    world.createCollider(upper, this.body);
    // most of a robot's weight is low (drivetrain, battery, motors): mass only, it touches nothing
    world.createCollider(RAPIER.ColliderDesc.cuboid(sL, 0.03, sW).setTranslation(0, 0.05, 0).setMass(cfg.mass * 0.5).setCollisionGroups(0), this.body);
    this.wheelPts = [];
    for (const [mx, mz] of modulePoints(cfg)) {
      {
        this.wheelPts.push({ x: mx, z: mz });
        const w = RAPIER.ColliderDesc.ball(WHEEL_R)
          .setTranslation(mx, WHEEL_R, mz)
          .setMass(cfg.mass * 0.0125)
          .setFriction(0).setFrictionCombineRule(RAPIER.CoefficientCombineRule.Min)
          .setRestitution(0)
          .setCollisionGroups(groups(GROUP.WHEEL, GROUP.STATIC | GROUP.TERRAIN | GROUP.ROBOT_BARRIER | GROUP.BALL));
        world.createCollider(w, this.body);
      }
    }
    // extending hopper section: a real collider that slides out with the hopper
    const st = this.cfg.storage;
    this.hopperCollider = null;
    // (not where the extension is the space over a folding intake, 8793's and 341's: the arm is
    // its own collider)
    if (st.extLen && !cfg.intake.fold) {
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
    // What sticks up out of the top is solid too, where it really is (no invisible slabs): 1678's
    // lid and 8793's folding intake arm (rigid: they stop the robot at a TRENCH arm), and the FUEL
    // a load pushes up under a net (the net lies over it; a sphere per FUEL above the frame's top,
    // moved with it each step, for other robots). A TRENCH arm meets that FUEL in the hopper
    // instead, where it's soft: pushed hard enough, the load squashes down and goes under.
    this.topY = this.topLoad = this.topStowed = this.height;
    this.growTop = this.loadTop = this.loadHighT = 0;
    const bay = cfg.bay;
    const solid = groups(GROUP.ROBOT, GROUP.STATIC | GROUP.ROBOT | GROUP.INTAKE);
    this.loadBalls = [];
    // the FUEL under a net meets robots here; a TRENCH arm it meets in the hopper, where the load
    // gives (FUEL is soft): Hopper.step pushes it down out of the arm and the robot feels that
    this.loadBallGroups = groups(GROUP.ROBOT, GROUP.ROBOT | GROUP.INTAKE);
    this.lidCollider = null;
    if (bay.lift) {
      const x1 = bay.slope ? bay.slope.x : bay.x1;
      this.lid = { x: (bay.x0 + x1) / 2 };
      this.lidCollider = world.createCollider(
        RAPIER.ColliderDesc.cuboid((x1 - bay.x0) / 2, 0.006, bay.hw).setTranslation(this.lid.x, bay.top, 0)
          .setDensity(0).setFriction(0.05).setRestitution(0.1).setCollisionGroups(solid),
        this.body,
      );
    }
    const ic = this.cfg.intake;
    // Intakes are solid to the field and other robots as they come out, so one deployed against a
    // wall or a robot pushes this robot back rather than going through. 8793's arm is its own
    // shape (its side outline across its width), turning about its pivot; the others are a block
    // from the bumper out to the roller that slides out as they deploy.
    const intakeGroups = groups(GROUP.INTAKE, GROUP.STATIC | GROUP.ROBOT | GROUP.INTAKE | GROUP.ROBOT_BARRIER);
    if (ic.fold) {
      const pts = [];
      for (const [x, y] of ic.fold.hull) for (const z of [-ic.width / 2 - 0.01, ic.width / 2 + 0.01]) pts.push(x, y, z);
      this.armCollider = world.createCollider(
        RAPIER.ColliderDesc.convexHull(new Float32Array(pts)).setTranslation(ic.fold.pivot[0], ic.fold.pivot[1], 0)
          .setDensity(0).setFriction(0.1).setRestitution(0.05).setCollisionGroups(intakeGroups),
        this.body,
      );
    } else {
      this.intakeBlock = { hx: ic.reach / 2 + 0.02, y0: 0.03, y1: 0.19 };
      this.armCollider = world.createCollider(
        RAPIER.ColliderDesc.cuboid(this.intakeBlock.hx, (this.intakeBlock.y1 - this.intakeBlock.y0) / 2, ic.width / 2)
          .setTranslation(this.halfL - this.intakeBlock.hx, (this.intakeBlock.y0 + this.intakeBlock.y1) / 2, 0)
          .setDensity(0).setFriction(0.1).setRestitution(0.05).setCollisionGroups(0),
        this.body,
      );
    }
    // deployed intake roller (pushes FUEL it cannot swallow)
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
    this.pivotLoad = 0;
    this.pivotMoving = false;
    this.hopperDeploy = 0;
    this.intakeSpeed = 0;
    this.flywheel = 0;
    this.hoodDeg = this.cfg.shooter.hoodMin;
    this.turretYaw = 0;
    // each turret starts at the middle of its travel (971's point back and out to their sides)
    this.turretYaws = this.turrets.map((t) => (t.center ?? 0) * DEG);
    this.turretYaw = this.turretYaws[0] ?? 0;
    // more than one turret: each has its own flywheel and hood (and knows whether it's on target)
    this.tws = this.turrets.length > 1 ? this.turrets.map(() => ({ fly: 0, hood: this.cfg.shooter.hoodMin, holdV: 0, holdT: 0, ok: false, sol: null })) : [];
    this.feedTimer = 0;
    this.lane = 0;
    this.intakeTokens = 0;
    this.full = false; // the intake has stalled against the load
    this.stalled = false;
    this.rollerSpeed = 1;
    this.wallPush = 0; // N: something outside pushing FUEL in the mouth on in (a wall)
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
    this.climbClaim = false; // an AI has picked this robot to climb for its alliance
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
    this.quat = new THREE.Quaternion().setFromAxisAngle(new THREE.Vector3(0, 1, 0), yaw);
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

  // how much FUEL fits right now: what the modeled hopper holds (measured from its geometry,
  // not the team's stated number), more once an extending hopper is out
  capacity() {
    const g = this.geoCap;
    return Math.floor(g.retracted + (g.extended - g.retracted) * this.hopperDeploy + 1e-6);
  }

  maxCapacity() { return this.geoCap.extended; }

  // how far forward the retracting intake arm reaches into the hopper (its roller), at a deploy
  // fraction; a compacting intake only sweeps FUEL back as far as intake.compactMin. While the
  // roller is still down below the hopper floor (out on the carpet, under a box pushed out over
  // it) it isn't in the FUEL's way at all
  _compactorX(dep = this.intakeDeploy) {
    const a = this.cfg.intake.arm;
    const deg = a.stowDeg + (a.deployDeg - a.stowDeg) * dep;
    const x = a.x + a.len * Math.cos(deg * DEG), y = a.y + a.len * Math.sin(deg * DEG);
    if (y < this.hopper.floorAt(Math.min(x, this.hopper.front - 0.01), 0)) return Infinity;
    return Math.max(x, this.cfg.intake.compactMin ?? -Infinity);
  }

  // How much FUEL fits behind a compacting intake whose roller is at x: all of it up to the
  // hopper's front; between the fixed front (x1) and there, from the compacted capacity up; past
  // x1 (2910's arm stows inside the hopper), in proportion to the room left
  _capBehind(x) {
    const b = this.cfg.bay, g = this.geoCap, front = this.bayFront(), full = this.capacity();
    const c = Math.min(full, g.compacted ?? g.retracted);
    if (x >= front) return full;
    if (x >= b.x1) return c + ((full - c) * (x - b.x1)) / Math.max(0.01, front - b.x1);
    return (c * Math.max(0, x - b.x0)) / Math.max(0.01, b.x1 - b.x0);
  }

  // front of the hopper right now (an extending hopper moves it forward)
  bayFront() { return this.cfg.bay.x1 + (this.cfg.storage.extLen || 0) * this.hopperDeploy; }

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
    const gc = this.geoCap;
    // (a compacting intake crams the load into the fixed part harder: gc.compacted)
    const inFixed = gc.compacted ?? gc.retracted;
    const need = Math.min(1, Math.max(0, (this.stored.length - inFixed) / Math.max(1, gc.extended - inFixed)));
    want = Math.max(want, need);
    if (on || want < this.hopperDeploy) this.hopperDeploy = approach(this.hopperDeploy, want, dt / 0.45);
    if (!this.hopperCollider) return;
    // collider follows the sliding section
    const out = this.hopperDeploy * st.extLen;
    const fx = this.cfg.frame.length / 2 - st.extLen / 2 + out;
    this.hopperCollider.setTranslationWrtParent({ x: fx, y: 0.17 + this.hopperHalfH, z: 0 });
    // solid to the field and to robots once it's out (out against a wall, it pushes the robot back)
    const g = this.hopperDeploy > 0.05 ? groups(GROUP.ROBOT, GROUP.BALL | GROUP.ROBOT | GROUP.ROBOT_BARRIER | GROUP.STATIC | GROUP.INTAKE) : 0;
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
    // driving and turning share the wheels (swerve desaturation): a wheel can't go faster than
    // the top speed, so turning hard at full speed takes some of the speed
    const wheel = Math.hypot(tvx, tvz) + Math.abs(tw) * this.drivetrain.r;
    if (wheel > d.maxSpeed) { const k = d.maxSpeed / wheel; tvx *= k; tvz *= k; tw *= k; }
    // the wheels only push on what they're touching: the drive has as much grip as the share of
    // wheels on the floor (carpet, BUMPS, DEPOT barriers). Up on FUEL or another robot, it
    // coasts instead of climbing further.
    const grip = this.traction = this._wheelsDown() / this.wheelPts.length;
    // Tires and frame flex soak up the kick from a sharp edge (a DEPOT barrier at full speed, a
    // bumper hit), which rigid wheels in the physics don't: pitching and rolling are capped at
    // TIP_RATE, and the angular damping bleeds them off, so a hit rocks a robot but doesn't flip it.
    {
      const av = this.body.angvel();
      const h = Math.hypot(av.x, av.z);
      if (h > TIP_RATE) this.body.setAngvel({ x: (av.x * TIP_RATE) / h, y: av.y, z: (av.z * TIP_RATE) / h }, true);
    }
    if (grip > 0) {
      const lv = this.body.linvel();
      let dvx = tvx - lv.x, dvz = tvz - lv.z;
      // how much the speed can change this step (drivetrain.js): speeding up, the motors' force
      // falls off with speed (back-EMF, battery sag, current limits); slowing down, they brake
      const v = Math.hypot(lv.x, lv.z);
      const dm = Math.hypot(dvx, dvz), maxDv = this.drivetrain.maxDv(v, Math.hypot(tvx, tvz) > v - 0.05, dt) * grip;
      if (dm > maxDv) { dvx *= maxDv / dm; dvz *= maxDv / dm; }
      this.body.setLinvel({ x: lv.x + dvx, y: lv.y, z: lv.z + dvz }, true);
      const av = this.body.angvel();
      const w = approach(av.y, tw, d.maxAlpha * dt * grip);
      this.body.setAngvel({ x: av.x, y: w, z: av.z }, true); // pitch and roll are the physics'
    }
    this._guideCaptured(dt);
  }

  // How many wheels touch the floor: a short ray down from each wheel's center finds the carpet,
  // a BUMP or a DEPOT barrier (not FUEL, not robots) within its radius (plus a little give)
  _wheelsDown() {
    const world = this.physics.world;
    const q = this.body.rotation(), p = this.body.translation();
    const qv = new THREE.Quaternion(q.x, q.y, q.z, q.w);
    const v = new THREE.Vector3();
    let n = 0;
    for (const w of this.wheelPts) {
      v.set(w.x, WHEEL_R, w.z).applyQuaternion(qv);
      const ray = new RAPIER.Ray({ x: p.x + v.x, y: p.y + v.y, z: p.z + v.z }, { x: 0, y: -1, z: 0 });
      if (world.castRay(ray, WHEEL_R + 0.02, true, undefined, FLOOR_QUERY, undefined, this.body)) n++;
    }
    return n;
  }

  // Where FUEL crosses into the hopper: over the bumper, then in at the front of the hopper
  _intakePath(z) {
    const front = this.bayFront();
    // in at the front of the hopper (a hopper pushed out over the intake takes it in at its front)
    const lip = this.cfg.intake.lip;
    const entryX = front - R - 0.02;
    // it goes in at the bottom, on the hopper floor at the front: under whatever FUEL is already
    // there, which it has to push back and lift (the rollers push it in under the pile)
    const entryY = this.hopper.floorAt(entryX, z) + R + 0.005;
    const lipX = lip ? lip.x : this.halfL + 0.04, lipY = lip ? lip.y : BUMP_Y1 + R + 0.025;
    return { lipX, lipY, entryX, entryY, liftY: Math.max(entryY, lipY) };
  }

  // Where a FUEL that can't get in yet waits (cap.slot), and the way it goes in from there: in the
  // mouth, a ball's width out from the entry toward the lip (or at the lip, if that's farther),
  // pushing on what's in the way; once the rollers have stalled, back down in the roller at the
  // front of the intake, on the carpet, sticking out a little past the intake, so a wall can squash it
  _holdPoint(path, slot) {
    let x, y;
    if (slot === 'roller') { x = this.halfL + this.cfg.intake.reach + 0.04 - R + HOLD_OUT; y = R + 0.01; }
    else {
      let dx = path.lipX - path.entryX, dy = path.liftY - path.entryY;
      const len = Math.hypot(dx, dy);
      if (len > 0.01) { dx /= len; dy /= len; } else { dx = 1; dy = 0; }
      const k = Math.max(len, HOLD_GAP);
      x = path.entryX + dx * k; y = path.entryY + dy * k;
    }
    const dx = path.entryX - x, dy = path.entryY - y, len = Math.hypot(dx, dy) || 1;
    return { x, y, ix: dx / len, iy: dy / len };
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
      // up the intake arm to just over the bumper, then into the hopper; while the way in is
      // blocked, the rollers hold it in the mouth (cap.wait, the hold point)
      if (!cap.over && ly >= path.liftY - 0.03) cap.over = true;
      const up = !cap.over;
      let tx = up ? path.lipX : path.entryX, ty = up ? path.liftY : path.entryY;
      if (cap.wait) {
        const h = this._holdPoint(path, cap.slot);
        tx = h.x; ty = h.y;
        if (cap.slot === 'roller') cap.over = false; // back down to the roller: up over the bumper again
      }
      let vx = tx - lx, vy = ty - ly, vz = cap.z - lz;
      const dist = Math.hypot(vx, vy, vz) || 1;
      const sp = Math.min(this.cfg.intake.pull ?? INTAKE_SPEED, dist / (2 * dt));
      vx *= sp / dist; vy *= sp / dist; vz *= sp / dist;
      // robot point velocity + the pull, in world axes
      cap.vs = {
        x: lv.x + w * dz + vx * c + vz * sn,
        y: vy + 9.81 * dt,
        z: lv.z - w * dx - vx * sn + vz * c,
      };
      b.body.setLinvel(cap.vs, true);
    }
  }

  // ------------------------------------------------------------------ after world.step
  postStep(dt, t) {
    const tr = this.body.translation();
    const q = this.body.rotation();
    this.pos.set(tr.x, tr.y, tr.z);
    this.quat.set(q.x, q.y, q.z, q.w);
    // heading of the (possibly tilted) body's forward axis
    const fx = 1 - 2 * (q.y * q.y + q.z * q.z), fz = 2 * (q.x * q.z - q.w * q.y);
    this.yaw = Math.atan2(-fz, fx);
    const lv = this.body.linvel();
    this.vel.set(lv.x, 0, lv.z);
    this.omega = this.body.angvel().y;

    const on = this.enabled && this.climbState === 'none';
    this.lastT = t;
    this._intake(dt, t, on);
    this._shooter(dt, t, on);
    this._stepHopper(dt);
  }

  // TRENCH arms near the robot, as oriented boxes in its frame, for the FUEL it holds
  _armsNear() {
    const out = [];
    const inv = _q.set(-this.quat.x, -this.quat.y, -this.quat.z, this.quat.w);
    for (const a of TRENCH_ARMS) {
      if (Math.abs(a.x - this.pos.x) > a.hx + 1 || Math.abs(a.z - this.pos.z) > a.hz + 1) continue;
      out.push({
        c: new THREE.Vector3(a.x - this.pos.x, a.y - this.pos.y, a.z - this.pos.z).applyQuaternion(inv),
        ax: [new THREE.Vector3(1, 0, 0).applyQuaternion(inv), new THREE.Vector3(0, 1, 0).applyQuaternion(inv), new THREE.Vector3(0, 0, 1).applyQuaternion(inv)],
        h: [a.hx, a.hy, a.hz],
        // the arm's velocity seen from the robot (it's the robot that moves)
        v: new THREE.Vector3(-this.vel.x, 0, -this.vel.z).applyQuaternion(inv),
      });
    }
    return out;
  }

  // the push a TRENCH arm gave the FUEL (squashing it down into the load) comes back on the robot
  _applyBoxForce(dt) {
    const f = this.hopper.boxForce;
    if (!f.lengthSq()) return;
    const w = _v.copy(f).applyQuaternion(this.quat).multiplyScalar(dt);
    const at = _v2.copy(this.hopper.boxAt).applyQuaternion(this.quat).add(this.pos);
    this.body.applyImpulseAtPoint({ x: w.x, y: w.y, z: w.z }, { x: at.x, y: at.y, z: at.z }, true);
  }

  // Move the colliders for what sticks up out of the top to where it is now (see _createBody)
  _stepTopColliders() {
    const bay = this.cfg.bay, ic = this.cfg.intake;
    // a sphere (FUEL plus the net over it) for each held FUEL above the frame's top
    let n = 0;
    for (const e of this.hopper.list) {
      if (e.tr || e.p.y + R < this.height - 0.01) continue;
      let c = this.loadBalls[n];
      if (!c) {
        c = this.physics.world.createCollider(RAPIER.ColliderDesc.ball(R + 0.004).setDensity(0).setFriction(0.05).setRestitution(0.1).setCollisionGroups(0), this.body);
        this.loadBalls.push(c);
      }
      c.setTranslationWrtParent({ x: e.p.x, y: e.p.y, z: e.p.z });
      c.setCollisionGroups(this.loadBallGroups);
      n++;
    }
    for (let k = n; k < this.loadBalls.length; k++) this.loadBalls[k].setCollisionGroups(0);
    if (this.lidCollider) this.lidCollider.setTranslationWrtParent({ x: this.lid.x, y: bay.top + bay.lift.h * this.hopper.liftScale + 0.006, z: 0 });
    if (ic.fold) {
      const a = (1 - this.intakeDeploy) * ic.fold.stowDeg * DEG;
      this.armCollider.setRotationWrtParent({ x: 0, y: 0, z: Math.sin(a / 2), w: Math.cos(a / 2) });
    } else {
      // the block slides out from inside the bumper to the roller as the intake deploys
      const b = this.intakeBlock, out = this.intakeDeploy * (ic.reach + 0.04);
      this.armCollider.setTranslationWrtParent({ x: this.halfL - b.hx + out - 0.02, y: (b.y0 + b.y1) / 2, z: 0 });
      this.armCollider.setCollisionGroups(this.intakeDeploy > 0.05 ? groups(GROUP.INTAKE, GROUP.STATIC | GROUP.ROBOT | GROUP.INTAKE | GROUP.ROBOT_BARRIER) : 0);
    }
  }

  _stepHopper(dt) {
    // A stretchy net over the top bulges as far as the load pushes it; nothing presses it back
    // down (a TRENCH arm stops the robot instead). A lid that rises with the hopper (1678's, on
    // the climber) comes down with it, but not onto FUEL that's piled up under it.
    const bay = this.cfg.bay, lift = bay.lift;
    let loadTop = 0;
    for (const e of this.hopper.list) if (!e.tr) loadTop = Math.max(loadTop, e.p.y + R);
    // (the lid rides up with the intake, 1678's: its climber lifts it while the intake is down)
    if (lift) this.hopper.liftScale = Math.max(this.intakeDeploy, clamp((loadTop - 0.01 - bay.top) / lift.h, 0, 1));
    // topLoad: the top with the intake down; topY: the true top, with a folded intake as it is now
    // growTop: the part that grows with the load (a net it bulges up, a lid it holds up), 0 if none
    this.growTop = lift ? bay.top + lift.h * this.hopper.liftScale + 0.01 : bay.dome ? loadTop + 0.005 : 0;
    this.topLoad = Math.max(this.height, this.growTop);
    // loadTop: the top of the FUEL itself; topStowed: the top once the intake (and a hopper that
    // rides out with it) is back in, so a lid that lifts with the hopper (1678's) is down on
    // whatever FUEL holds it up: what has to fit under a TRENCH arm
    this.loadTop = loadTop;
    this.loadHighT = (bay.dome || lift) && loadTop > TRENCH.clearHeight - 0.035 ? (this.loadHighT || 0) + dt : 0;
    const held = lift ? bay.top + lift.h * clamp((loadTop - 0.01 - bay.top) / lift.h, 0, 1) + 0.01 : this.growTop;
    this.topStowed = Math.max(this.height, held);
    const fold = foldTop(this.cfg.intake, this.intakeDeploy);
    this.topY = Math.max(this.topLoad, fold);
    this._stepTopColliders();
    // the robot's acceleration and turn rate, felt by the FUEL inside
    let ax = 0, az = 0, alpha = 0;
    if (this.prevVel) {
      ax = clamp((this.vel.x - this.prevVel.x) / dt, -25, 25);
      az = clamp((this.vel.z - this.prevVel.z) / dt, -25, 25);
      alpha = clamp((this.omega - this.prevOmega) / dt, -60, 60);
    }
    this.prevVel = { x: this.vel.x, z: this.vel.z };
    this.prevOmega = this.omega;
    // the robot's acceleration and gravity, in the (tilted) robot frame
    const qi = this.quat.clone().invert();
    const acc = new THREE.Vector3(ax, 0, az).applyQuaternion(qi);
    const g = new THREE.Vector3(0, -9.81, 0).applyQuaternion(qi);
    const f = this.cfg.bay.feed;
    this.hopper.front = this.bayFront();
    this.hopper.step(dt, {
      acc, g, w: this.omega, alpha,
      feeding: this.feeding > 0,
      intaking: this.intakeSpeed > 0,
      feedPoint: new THREE.Vector3(f.x, 0, f.z ?? 0),
      boxes: this._armsNear(),
    });
    this._applyBoxForce(dt);
  }

  _intake(dt, t, on) {
    const ic = this.cfg.intake;
    let want = on && this.cmd.intake;
    if (ic.latched && this.intakeDeploy >= 1) want = true; // latched down for the whole match
    if (this.forceDeploy) want = true;
    if (on && this.cmd.lower) want = true; // down without running (to get under the TRENCH)
    // Re•Blitz retracts the intake while shooting to compact FUEL into the indexer
    if (ic.compacts && on && (this.cmd.shoot || this.cmd.pass) && !this.cmd.intake) want = false;
    if (on && this.cmd.outtake) want = true;
    // a folding intake whose rollers hold FUEL (8793's, 341's: the space over the deployed arm is
    // part of the ball path) stays down while that FUEL has nowhere else to go
    if (ic.fold && !ic.compacts && this.cfg.storage.extend === 'intake' && this.stored.length > this.geoCap.retracted) want = true;
    const next = approach(this.intakeDeploy, want ? 1 : 0, dt / (want ? ic.deployTime : ic.retractTime ?? ic.deployTime));
    // a compacting intake pushes on the load as it comes in, and stalls while it can't squeeze more
    // folding in, it pushes the FUEL in front of it back into the hopper; it stalls where the load
    // behind it is packed as hard as it can squeeze (compactPush)
    // and it can't push harder than its motor allows: once the packed load pushes back on it
    // that hard (Hopper.wallPressure), it stalls where it is instead of crushing the FUEL
    const squeezing = ic.compacts && next < this.intakeDeploy;
    const limit = ic.compactPush ?? this.intakePush;
    const full = squeezing && this.stored.length > this._capBehind(this._compactorX(next));
    const hard = squeezing && this.hopper.wallPressure > limit;
    // how hard the intake's pivot motor works (0 idle .. 1 stalled against the load), for its sound
    this.pivotLoad = squeezing ? (full || hard ? 1 : Math.min(1, this.hopper.wallPressure / limit)) : 0;
    this.pivotMoving = Math.abs(next - this.intakeDeploy) > 1e-6 && !(full || hard);
    if (!(full || hard)) this.intakeDeploy = next;
    this.hopper.wall = ic.compacts ? this._compactorX() : Infinity;
    this._hopper(dt, on);
    const deployed = this.intakeDeploy > 0.85;
    // the roller only pushes FUEL (the intake's own collider meets walls and robots)
    const groupsNow = deployed ? groups(GROUP.INTAKE, GROUP.BALL | GROUP.ROBOT_BARRIER) : 0;
    this.intakeCollider.setCollisionGroups(groupsNow);

    const running = on && this.cmd.intake && deployed;
    this.intakeSpeed = on && this.cmd.outtake ? -1 : running ? 1 : 0;
    // full: the rollers have stalled against the load (nothing more gets in until it gives way);
    // there's no count limit, so a load packed harder (a wall shoving FUEL in) holds more
    // (stalled for a while: a moment's stall while the load shifts isn't full)
    this.full = this.stallT > FULL_T;
    // the rollers hold about a row of FUEL across
    const row = Math.max(2, Math.floor(ic.width / (2 * R)));
    if (running && !this.stalled && this.captured.length < row && this.stored.length < this.maxCapacity() * 2 + 10) {
      this.intakeTokens = Math.min(Math.max(4, 2 * ic.rate * dt), this.intakeTokens + ic.rate * dt);
      const front = this.halfL - 0.04;
      const reach = this.halfL + ic.reach + R + 0.02;
      const hw = ic.width / 2 + 0.02;
      const balls = this.fuel.balls;
      for (let i = 0; i < balls.length && this.intakeTokens >= 1 && this.captured.length < row; i++) {
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
      }
    } else {
      this.intakeTokens = 0;
    }
    this._settleCaptured(t, dt, running);
    // outtake: FUEL goes back out over the intake
    if (on && this.cmd.outtake && this.stored.length) {
      this.outtakeTimer += dt;
      while (this.outtakeTimer >= 0.08) {
        this.outtakeTimer -= 0.08;
        if (!this._startOuttake()) break;
      }
    } else this.outtakeTimer = 0;
  }

  // Grabbed FUEL that reached the hopper joins it; FUEL the intake lets go of is released.
  // A FUEL only goes in once the entry is clear. Until then the rollers hold it in the mouth and
  // push it into the FUEL in the way, as hard as they can grip (intakePush), and that one into the
  // load: it goes in if the load gives way, and the rollers stall if it doesn't. Something outside
  // pushing the held FUEL in (driving it into a wall) adds its push, as hard as the FUEL is squashed.
  _settleCaptured(t, dt, running) {
    const deployed = this.intakeDeploy > 0.85;
    let waited = 0, outside = 0;
    const pushed = new Set(); // a FUEL in the way takes one roller's push, however many FUEL wait behind it
    for (let i = this.captured.length - 1; i >= 0; i--) {
      const cap = this.captured[i];
      const b = cap.b;
      const l = this.worldToLocal3(b.pos);
      const path = this._intakePath(cap.z);
      // what's in the way at the entry
      const block = [];
      for (const e of this.hopper.list) {
        if (e.tr) continue;
        const dx = e.p.x - path.entryX, dy = e.p.y - path.entryY, dz = e.p.z - cap.z;
        if (dx * dx + dy * dy + dz * dz < BLOCK_D * BLOCK_D) block.push(e);
      }
      // (8793's single-file ball path isn't packed by pushing: it holds what its path fits)
      const shut = this.cfg.bay.pack === false && this.stored.length >= this.capacity();
      const lost = b.state !== 'field' || (!cap.waitT && t - cap.t0 > 0.8) || (cap.waitT && !deployed);
      if (!lost && !block.length && !shut && l.x <= path.entryX + 0.02 && l.y >= path.entryY - 0.04) {
        this.captured.splice(i, 1);
        const v = b.body.linvel();
        const pv = this.velocityAt(b.pos);
        const lv = this.worldToLocalVec(v.x - pv.x, v.z - pv.z);
        this.fuel.toRobot(b);
        b.captor = null;
        this.stored.push(b);
        this.stats.intaked++;
        const e = this.hopper.add(b, l, new THREE.Vector3(lv.x, v.y, lv.z));
        e.push = this.intakePush; // the roller shoves it on into the load
        e.pushT = 0.4;
      } else if (lost || (!running && !cap.over && !cap.waitT)) {
        // not over the bumper yet (or let go of): it drops back onto the carpet
        this.captured.splice(i, 1);
        b.captor = null;
        this.stats.dropped++;
        if (b.state === 'field') { b.ignoring = true; b.ignoreUntil = t + 0.25; }
      } else if (block.length || shut) {
        // waiting, pushing on what's in the way
        cap.wait = true;
        cap.t0 = t;
        cap.waitT = (cap.waitT || 0) + dt;
        // once the rollers stall, the load pushes it back down the intake into the stalled roller
        cap.slot = cap.waitT > STALL_T ? 'roller' : 'mouth';
        const h = this._holdPoint(path, cap.slot);
        const hx = l.x - h.x, hy = l.y - h.y;
        if (Math.hypot(hx, hy, l.z - cap.z) < 0.02) cap.held = true;
        // pushed in past where it's held by something outside (it didn't move back out as fast as
        // the rollers moved it: something's in the way): it's squashed that much, and pushes on
        // the FUEL ahead of it (and back on the robot) as hard
        const c = Math.cos(this.yaw), sn = Math.sin(this.yaw);
        const v = b.body.linvel(), vs = cap.vs || v;
        const blocked = (vs.x - v.x) * -h.ix * c + (vs.z - v.z) * h.ix * sn + (vs.y - v.y) * -h.iy > 0.15;
        // (in the roller it's pressed straight back; in the mouth, along the way in)
        const into = cap.slot === 'roller' ? -hx : hx * h.ix + hy * h.iy;
        const sq = cap.held && blocked ? Math.max(0, into - 0.005) : 0;
        const f = fuelForce(sq);
        if (f > 0) {
          outside += f;
          const J = f * dt;
          this.body.applyImpulse({ x: (h.ix * c) * J, y: 0, z: (-h.ix * sn) * J }, true);
        }
        // it pushes what's in the way out of the entry (back into the load, and up under the
        // weight of the pile over it), as hard as the rollers grip it plus whatever's pushing it
        // in from outside
        const F = (running ? this.intakePush : 0) + f;
        // (it comes in from the front and below: what's in its way goes back and up)
        for (const e of block) {
          if (pushed.has(e)) continue;
          pushed.add(e);
          const dx = e.p.x - path.entryX - R, dy = e.p.y - path.entryY + 0.5 * R, dz = e.p.z - cap.z;
          const d = Math.hypot(dx, dy, dz) || 1;
          const k = F / block.length / d;
          e.ext = e.ext || new THREE.Vector3();
          e.ext.x += dx * k; e.ext.y += dy * k; e.ext.z += dz * k;
        }
        waited = Math.max(waited, cap.waitT);
      } else {
        // the way in just opened: it heads in (still counting as waiting until it's in, so a load
        // that keeps pushing back into the way still stalls the rollers)
        cap.wait = false;
        if (cap.waitT) { cap.waitT += dt; waited = Math.max(waited, cap.waitT); }
      }
    }
    this.wallPush = outside;
    this.stalled = waited > STALL_T;
    this.stallT = this.stalled ? (this.stallT || 0) + dt : 0;
    // the rollers slow while FUEL waits in them, and stop when they've stalled
    this.rollerSpeed = this.stalled ? 0 : waited > 0 ? 0.35 : 1;
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

  // where a FUEL launched now would come down is on our half of the FIELD, clear of the walls,
  // and it doesn't drop into a HUB on the way (that would be a G407 from outside the zone)
  _passLands(ti = 0) {
    const sh = this.cfg.shooter;
    const psi = sh.type === 'fixed' ? this.yaw + this.aimOffset : this.yaw + this.turretYaws[ti];
    // from where it really leaves, moving with the (maybe turning) robot
    const exit = sh.type === 'fixed'
      ? this._exitPoint(sh.lanes[this.lane % sh.lanes.length])
      : this.localToWorld(...this._turretExit(ti));
    const lv = this.velocityAt(exit);
    const w = this.tws[ti];
    const th = (w ? w.hood : this.hoodDeg) * DEG, v = w ? w.fly : this.flywheel;
    const pts = trajectoryPoints(exit, { x: Math.cos(psi) * Math.cos(th) * v + lv.x, y: Math.sin(th) * v, z: -Math.sin(psi) * Math.cos(th) * v + lv.z });
    const x = pts[pts.length - 3], z = pts[pts.length - 1], s = this.alliance === BLUE ? 1 : -1;
    if (!(Math.abs(x) < HALF_L - 1.2 && Math.abs(z) < HALF_W - 1.2 && s * x < -0.5)) return false;
    for (const a of [BLUE, RED]) {
      const c = Field.hubCenter(a);
      for (let i = 0; i < pts.length; i += 3) {
        if (pts[i + 1] < HUB.rimFront + 0.6 && Math.hypot(pts[i] - c.x, pts[i + 2] - c.z) < HUB.size / 2 + 0.3) return false;
      }
    }
    return true;
  }

  _exitPoint(laneZ = 0, ti = 0) {
    const sh = this.cfg.shooter;
    if (sh.type === 'fixed') return this.localToWorld(sh.exit.x, sh.exit.y, laneZ);
    const t = this.turrets[ti];
    return this.localToWorld(t.x, sh.exitY, t.z);
  }

  // where FUEL leaves turret ti's hood right now (robot frame)
  _turretExit(ti) {
    const sh = this.cfg.shooter, t = this.turrets[ti], a = this.turretYaws[ti];
    return [t.x + Math.cos(a) * sh.exitRadius, sh.exitY, t.z - Math.sin(a) * sh.exitRadius];
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
    // spin up ahead of time: in our zone, or when the driver (an AI about to dump a pass) asks
    const prespin = on && has && (inZoneNow || !!this.cmd.prespin);
    const twin = this.turrets.length > 1;
    if (twin) ({ ready, status } = this._twinTurrets(dt, t, tgt, (wantShoot || prespin) && has, wantShoot, status));
    else if ((wantShoot || prespin) && has) {
      let exit, lv;
      if (sh.type === 'fixed') {
        // The drum sits on the robot's centerline, so once aimed the shot line passes through the
        // robot center. Solve from a point that does not move when the chassis rotates: the exit's
        // distance along the shot line (a back-facing drum's exit is toward the target).
        const dx = tgt.x - this.pos.x, dz = tgt.z - this.pos.z;
        const dd = Math.hypot(dx, dz) || 1;
        const along = this.aimOffset ? -sh.exit.x : sh.exit.x;
        exit = new THREE.Vector3(this.pos.x + (dx / dd) * along, this.pos.y + sh.exit.y, this.pos.z + (dz / dd) * along);
        lv = { x: this.vel.x, z: this.vel.z };
      } else {
        // FUEL leaves the hood exitRadius in front of the turret axis, along the shot line
        const c = this._exitPoint(0, 0);
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
          aimErr = wrapAngle(sol.psi - this.yaw - this.aimOffset);
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
          // measured from the middle of the turret's travel (its hard stops are +-lim from there)
          const lim = sh.turretRange * DEG, mid = (this.turrets[0].center ?? 0) * DEG;
          const rel = wrapAngle(sol.psi - this.yaw - mid);
          // choose the equivalent angle closest to the current turret position within limits
          const cands = [rel, rel + 2 * Math.PI, rel - 2 * Math.PI].filter((a) => Math.abs(a) <= lim);
          const cur = this.turretYaw - mid;
          const goal = cands.length ? cands.reduce((a, b) => (Math.abs(b - cur) < Math.abs(a - cur) ? b : a)) : clamp(rel, -lim, lim);
          this.turretYaw = this.turretYaws[0] = mid + approach(cur, goal, sh.turretRate * DEG * dt);
          aimErr = wrapAngle(sol.psi - (this.yaw + this.turretYaw));
        }
        this._lastPsi = sol.psi;
        // a pass (shuttling FUEL back) doesn't have to be perfect: instead of waiting for a clean
        // shot at the aim point, it goes as soon as, with the flywheel, hood and heading as they
        // are right now, the FUEL would come down on our half of the FIELD (clear of the walls)
        const pass = tgt.mode === 'pass';
        const tol = pass ? 4 * DEG : Math.max(0.6 * DEG, Math.atan2(0.14, sol.dist));
        const spinOk = pass ? this.flywheel > 0.75 * sol.v : Math.abs(this.flywheel - sol.v) / sol.v < 0.025;
        const hoodOk = Math.abs(this.hoodDeg - sol.theta / DEG) < 1.5;
        const aimOk = Math.abs(aimErr) < tol;
        ready = pass ? spinOk && this._passLands() : spinOk && hoodOk && aimOk;
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
    if (!twin) this._flywheelStep(dt, t, on, setpoint);

    // feed
    const period = 1 / sh.bps;
    // a fixed shooter's lanes each go when their FUEL gets there: the gaps vary (random, 0.5-1.5x
    // the period, the same rate on average); a single stream keeps its beat
    if (!this.nextPeriod) this.nextPeriod = period;
    this.feedTimer = Math.min(this.feedTimer + dt, period * 2);
    if (wantShoot && ready && has && this.shot) {
      // one FUEL starts up the feed path every period
      while (this.feedTimer >= this.nextPeriod && this._startFeed()) {
        this.feedTimer -= this.nextPeriod;
        this.nextPeriod = sh.type === 'fixed' ? period * (0.5 + Math.random()) : period;
      }
      this.feeding = 1;
    }
    if (this.hopper.list.some((e) => e.tr && e.tr.feed && e.tr.t < e.tr.T)) this.feeding = 1;
    this.status = status;
    this.ready = ready;
  }

  // Flywheel dynamics (w: an object with fly / holdV / holdT; the robot itself for one shooter).
  // It keeps its last shot speed for a moment after the trigger is released, so stop-and-go
  // shooting (or passing) doesn't have to spin up from scratch every time. Torque-limited spin-up
  // (0 -> max in spinTau*2), fast closed-loop settle near the setpoint, slow coast-down.
  _spin(w, dt, t, on, setpoint) {
    const sh = this.cfg.shooter;
    if (setpoint > 0) { w.holdV = setpoint; w.holdT = t; }
    else if (on && w.holdV && t - w.holdT < 1.5) setpoint = w.holdV;
    const target = Math.min(setpoint, sh.speedMax);
    const err = target - w.fly;
    if (err > 0) w.fly += Math.min(err * (1 - Math.exp(-dt / 0.06)), (sh.speedMax / (2 * sh.spinTau)) * dt);
    else w.fly += Math.max(err * (1 - Math.exp(-dt / 0.06)), -(sh.speedMax / 3) * dt);
  }

  _flywheelStep(dt, t, on, setpoint) {
    const w = { fly: this.flywheel, holdV: this.spinHold, holdT: this.spinHoldT };
    this._spin(w, dt, t, on, setpoint);
    this.flywheel = w.fly; this.spinHold = w.holdV; this.spinHoldT = w.holdT;
  }

  // More than one turret (971): each is its own shooter, with its own flywheel and hood, solving
  // its own shot from where it sits and turning within its own travel. Any turret that's on target
  // can fire (the feed takes turns among them). If none can reach the target, the chassis turns
  // to bring the nearest one into range.
  _twinTurrets(dt, t, tgt, active, wantShoot, status) {
    const sh = this.cfg.shooter, on = this.enabled && this.climbState === 'none';
    const lim = sh.turretRange * DEG, margin = 8 * DEG, pass = tgt.mode === 'pass';
    let need = Infinity, show = -1, anyReach = false, anySol = false, spun = false, aimed = false;
    this.turrets.forEach((tt, i) => {
      const w = this.tws[i];
      w.ok = false;
      w.sol = null;
      let setpoint = 0;
      if (active) {
        // FUEL leaves the hood exitRadius out from the turret axis, along the shot line
        const c = this._exitPoint(0, i);
        const dx = tgt.x - c.x, dz = tgt.z - c.z, dd = Math.hypot(dx, dz) || 1;
        const exit = new THREE.Vector3(c.x + (dx / dd) * sh.exitRadius, c.y, c.z + (dz / dd) * sh.exitRadius);
        const lv = this.velocityAt(exit);
        const sol = solveMovingShot(tgt.table, exit, lv, tgt);
        if (sol) {
          anySol = true;
          // measured from the middle of this turret's travel (its hard stops are +-lim from there)
          const mid = (tt.center ?? 0) * DEG;
          const rel = wrapAngle(sol.psi - this.yaw - mid);
          const cands = [rel, rel + 2 * Math.PI, rel - 2 * Math.PI].filter((a) => Math.abs(a) <= lim);
          const cur = this.turretYaws[i] - mid;
          const goal = cands.length ? cands.reduce((a, b) => (Math.abs(b - cur) < Math.abs(a - cur) ? b : a)) : clamp(rel, -lim, lim);
          this.turretYaws[i] = mid + approach(cur, goal, sh.turretRate * DEG * dt);
          const o = rel - clamp(rel, margin - lim, lim - margin);
          if (Math.abs(o) < Math.abs(need)) need = o;
          if (cands.length) {
            anyReach = true;
            setpoint = sol.v;
            w.hood = approach(w.hood, sol.theta / DEG, 260 * dt);
            w.sol = { ...sol, exit, lv };
            const err = Math.abs(wrapAngle(sol.psi - (this.yaw + this.turretYaws[i])));
            const tol = pass ? 4 * DEG : Math.max(0.6 * DEG, Math.atan2(0.14, sol.dist));
            const spinOk = pass ? w.fly > 0.75 * sol.v : Math.abs(w.fly - sol.v) / sol.v < 0.025;
            const hoodOk = Math.abs(w.hood - sol.theta / DEG) < 1.5;
            spun ||= spinOk; aimed ||= err < tol;
            w.ok = spinOk && err < tol && (pass || hoodOk);
            if (show < 0 || (w.ok && !this.tws[show].ok)) show = i;
          }
        }
      }
      this._spin(w, dt, t, on, setpoint);
    });
    if (active && wantShoot && !anyReach && Number.isFinite(need) && need) {
      const d = this.cfg.drive, a = Math.abs(need);
      this.aimOverride = Math.sign(need) * Math.min(d.maxOmega, Math.sqrt(2 * 0.7 * d.maxAlpha * a), 7 * a);
    }
    // what the HUD, the preview and the anims show: the turret that's firing (or about to)
    const k = show >= 0 ? show : 0, w = this.tws[k];
    this.turretYaw = this.turretYaws[k];
    this.flywheel = w.fly;
    this.hoodDeg = w.hood;
    if (pass) for (let i = 0; i < this.tws.length; i++) if (this.tws[i].ok && !this._passLands(i)) this.tws[i].ok = false;
    const ready = this.tws.some((x) => x.ok);
    if (w.sol) {
      this.shot = { ...w.sol, mode: tgt.mode, target: tgt };
      this.preview = { exit: w.sol.exit, lv: w.sol.lv, sol: w.sol };
      status = ready ? 'READY' : !spun ? 'Spinning up' : !aimed ? 'Aiming' : 'Hood';
      if (pass) status = ready ? 'PASS READY' : 'Pass: ' + status.toLowerCase();
    } else if (active) status = anySol ? 'Turning to aim' : tgt.mode === 'hub' ? 'Out of range' : 'Pass out of range';
    else if (!tgt.inZone && this.stored.length) status = 'Outside ALLIANCE ZONE';
    return { ready, status };
  }

  // The FUEL nearest the feed point starts up the feed path (indexer, ramp, turret) and leaves
  // the shooter when it gets there.
  _startFeed() {
    const sh = this.cfg.shooter, f = this.cfg.bay.feed;
    let laneZ = f.z ?? 0;
    let ti = 0;
    if (this.turrets.length > 1) {
      // the next turret in turn that's on target
      const n = this.turrets.length;
      ti = -1;
      for (let k = 0; k < n && ti < 0; k++) { const i = (this.lane + k) % n; if (this.tws[i].ok) { ti = i; this.lane = i; } }
      if (ti < 0) return false;
    }
    // a fixed shooter's lanes don't take turns in a perfect order: whichever lane's FUEL gets to
    // its wheels first goes (random, never the same lane twice running)
    if (sh.type === 'fixed') {
      const n = sh.lanes.length, prev = this.lane % n;
      this.lane = n > 1 ? (prev + 1 + Math.floor(Math.random() * (n - 1))) % n : 0;
      laneZ = sh.lanes[this.lane];
    }
    else if (f.zs) laneZ = f.zs[ti]; // twin turrets: each has its own side of the separator
    const fp = new THREE.Vector3(f.x, this.hopper.floorAt(f.x, laneZ) + R, laneZ);
    let best = null, bd = Infinity;
    for (const e of this.hopper.list) {
      if (e.tr) continue;
      const d = e.p.distanceToSquared(fp);
      if (d < bd) { bd = d; best = e; }
    }
    if (!best) return false;
    // a single-file indexer only takes the FUEL that's got to it (the conveyor brings the rest)
    if (f.reach && bd > f.reach * f.reach) return false;
    if (sh.type !== 'fixed') this.lane++;
    const via = (f.vias ? f.vias[ti] : f.via).map(([x, y, z]) => new THREE.Vector3(x, y, z ?? laneZ));
    const end = sh.type === 'fixed'
      ? () => new THREE.Vector3(sh.exit.x, sh.exit.y, laneZ)
      : () => new THREE.Vector3(...this._turretExit(ti));
    this.hopper.startTransit(best, via, end, FEED_SPEED, (e) => {
      // a FUEL that reaches the wheels while the shot isn't lined up waits there
      // (and with more than one turret, its own turret has to be on target)
      const onTarget = this.turrets.length < 2 || this.tws[ti].ok;
      if (this.ready && onTarget && this.shot && this.enabled && (this.cmd.shoot || this.cmd.pass)) this._fire(e, this.shot.mode);
    });
    best.tr.feed = true;
    best.tr.turret = ti;
    return true;
  }

  _fire(e, mode) {
    const sh = this.cfg.shooter;
    const b = e.b;
    this.hopper.remove(e);
    this._unstore(b);
    const exit = this.localToWorld(e.p.x, e.p.y, e.p.z);
    const ti = e.tr?.turret ?? 0, w = this.tws[ti]; // w: that turret's own shooter (971)
    let psi = sh.type === 'fixed' ? this.yaw + this.aimOffset : this.yaw + this.turretYaws[ti];
    const k = this.noiseScale ?? 1; // AI skill: extra scatter for weaker drivers
    psi += gauss() * sh.yawSigma * k * DEG;
    const th = (w ? w.hood : this.hoodDeg) * DEG + gauss() * sh.angleSigma * k * DEG;
    const v = (w ? w.fly : this.flywheel) * (1 + gauss() * sh.speedSigma * k);
    const lv = this.velocityAt(exit);
    const vel = {
      x: Math.cos(psi) * Math.cos(th) * v + lv.x,
      y: Math.sin(th) * v,
      z: -Math.sin(psi) * Math.cos(th) * v + lv.z,
    };
    this.fuel.launch(b, exit, vel, { by: 'robot', alliance: this.alliance, legal: this.lastInZone, t: this.lastT, ignoreRobot: 0.35, spin: true });
    if (w) w.fly *= 1 - sh.shotDrop; else this.flywheel *= 1 - sh.shotDrop;
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
      this.body.setRotation(yawQuat(this.yaw), true);
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
    // the model follows the body, tilt and all (climbing leans it back a little on the hooks)
    v.position.copy(this.pos);
    v.quaternion.copy(this.quat);
    if (this.climbState !== 'none') v.rotateZ(-Math.min(0.12, this.climbLift * 0.15));
    // stored FUEL
    const m = this.model;
    const list = this.hopper.list;
    const n = Math.min(list.length, m.storedMesh.instanceMatrix.count);
    const mat = new THREE.Matrix4();
    // squeezed FUEL is drawn flattened where it's pressed (against other FUEL and the walls)
    const tc = m.storedTouch || [], patches = this._touch || (this._touch = new Float32Array(16));
    for (let i = 0; i < n; i++) {
      const k = this.hopper.touchPatches(list[i], patches);
      for (let j = 0; j < tc.length; j++) {
        const a = tc[j].array, o = 4 * i;
        if (j < k) { a[o] = patches[4 * j]; a[o + 1] = patches[4 * j + 1]; a[o + 2] = patches[4 * j + 2]; a[o + 3] = patches[4 * j + 3]; }
        else { a[o] = 0; a[o + 1] = 1; a[o + 2] = 0; a[o + 3] = 1; } // no patch: a plane beyond the ball
      }
      mat.makeTranslation(list[i].p.x, list[i].p.y, list[i].p.z);
      m.storedMesh.setMatrixAt(i, mat);
    }
    for (const a of tc) a.needsUpdate = true;
    m.storedMesh.count = n;
    m.storedMesh.instanceMatrix.needsUpdate = true;
    // mechanisms
    const sh = this.cfg.shooter;
    m.anim({
      intakeDeploy: this.intakeDeploy,
      intakeSpeed: this.intakeSpeed * (this.intakeSpeed > 0 ? this.rollerSpeed ?? 1 : 1),
      hopperDeploy: this.hopperDeploy,
      flywheel: this.flywheel,
      feeding: this.feeding,
      hoodDeg: this.hoodDeg,
      turretYaw: this.turretYaw,
      turretYaws: this.turretYaws,
      climbLift: this.climbLift,
      climbState: this.climbState,
      rotorAngle: this.hopper.finAngle,
      load: this.hopper.list,
      lift: this.hopper.liftScale,
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

  // whether it fits under a TRENCH arm right now (a load bulging its net, or a raised lid, doesn't)
  fitsTrenchNow() { return fitsTrench(this.cfg) && this.topY <= TRENCH.clearHeight - 0.005; }
  // fits once a folded intake is lowered (the AI lowers it on its way under)
  fitsTrenchLowered() { return fitsTrench(this.cfg) && this.topStowed <= TRENCH.clearHeight - 0.005; }
  // the load is nearly up to a TRENCH arm: more FUEL would make it too tall to get under
  // (only where the load can push the top up: under a net or a lid; a fixed hopper top already fits)
  // (after it's stayed that high a moment: a FUEL tumbling in on top doesn't count)
  loadNearTrench() { return this.loadHighT > 0.3; }
  // deploying the intake raises the top (1678's lid rides up with the hopper extension), so it
  // stays in near a TRENCH arm
  get deployRaisesTop() { return !!this.cfg.bay.lift; }

  worldToLocalVec(x, z) {
    const c = Math.cos(this.yaw), s = Math.sin(this.yaw);
    return { x: x * c - z * s, z: x * s + z * c };
  }

  previewPoints() {
    if (!this.preview) return null;
    const { exit, lv, sol } = this.preview;
    const psi = this.cfg.shooter.type === 'fixed' ? this.yaw + this.aimOffset : this.yaw + this.turretYaw;
    const v = this.flywheel > 0.5 ? Math.max(this.flywheel, 0) : sol.v;
    const th = this.hoodDeg * DEG;
    return trajectoryPoints(exit, {
      x: Math.cos(psi) * Math.cos(th) * v + lv.x,
      y: Math.sin(th) * v,
      z: -Math.sin(psi) * Math.cos(th) * v + lv.z,
    });
  }
}
