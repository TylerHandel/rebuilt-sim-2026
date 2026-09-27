// Robot definitions, based on each team's published 2026 information.
//  - 2910 Jack in the Bot "Re•Blitz" (tech binder): 27.5x27in swerve (MK5n R1), front slap-down
//    over-bumper intake (4 FUEL wide), one-piece hopper (holds 50+, fits under the TRENCH),
//    4-FUEL-wide drum shooter with adjustable hood fixed to the chassis, firing out the back (away
//    from the intake) — the whole robot turns to aim; 30+ BPS (34 sustained). No climber ("Won't: Climb").
//  - 4414 HighTide "RIPCURRENT" (tech binder): 25x32in swerve (7.67:1), over-bumper intake,
//    extending hopper (85+ under the TRENCH), a Dye Rotor (pocketed rotor, as in a paintball
//    loader) feeding a single-stream turret shooter
//    (3in flywheel, adjustable hood), precomputed shoot-on-the-move. No climber listed.
//  - 8793 Pumpkin Bots (team CAD): hopperless — Intake V3 -> Conveyor V2 -> Turret -> Shooter.
//    Low profile for the TRENCH. No climber in the assembly.
//  - 971 Spartan Robotics "Mixtape" (public Onshape CAD + technical documentation): 25x30in
//    swerve (MK5), 4-bar ground intake that pushes the polycarbonate hopper out, roller floor to a
//    powered omni-wheel separator that splits FUEL into two paths, kickers and ramps up into two
//    independently powered turrets (~210deg of travel each, 4in flywheels, lead-screw hood), one
//    stage telescoping L1 climber.
//  - 1678 Citrus Circuits "Limestone" (public Onshape CAD + robot page): 27x27in swerve (MK5n),
//    full-width slapdown intake with a horizontal hopper extension, roller floor of flex wheels to
//    a ball tunnel, full-width 3.5in drum with three hood rollers and an articulating hood fixed to
//    the chassis (fires out the back), a corrugated lid on the climber that lifts to make the
//    hopper taller (the vertical extension), L1 climb.
//  - 1690 Orbit "Kepler" (Onshape views + X_T release, reveal and CAD threads): swerve, over-bumper
//    intake with passive deploy, expanding hopper with a lattice frame over a powered roller floor
//    (50-55 FUEL open-topped), vertical kicker into a compact gear-driven turret on an 8in bearing,
//    shoot-on-the-move while intaking.
//  Nets: all three carry netting over the open parts of their hoppers (the physics lets the load
//  bulge it up, bay.dome; 1678's closes the sides of the lifted lid).
import { IN } from './constants.js';

export const BUMPER_T = 3.25 * IN; // bumper thickness incl. backing

export const ROBOTS = {
  2910: {
    key: '2910',
    team: 2910,
    teamName: 'Jack in the Bot',
    robotName: 'Re•Blitz',
    archetype: 'Dumper',
    blurb: 'Huge hopper and a 4-wide drum shooter fixed to the chassis — the whole robot rotates to aim. Unloads 30+ FUEL per second.',
    frame: { length: 27.0 * IN, width: 27.5 * IN },
    height: 21.5 * IN,
    mass: 61,
    drive: { maxSpeed: 4.3, maxAccel: 9.5, maxOmega: 8.5, maxAlpha: 32 },
    // slap-down intake from 2910's CAD: its 2in roller reaches ~7.8in past the bumper
    // rate: fast enough that driving through FUEL is the only limit; pull: roller surface speed
    // arm: the CAD intake's pivot and its 2in roller's reach and angle, stowed (as exported) and
    // down on the carpet. While shooting it retracts slowly and pushes the load back into the
    // indexer (compacts), stalling against the FUEL until there's room.
    intake: {
      width: 25.5 * IN, reach: 7.8 * IN, rate: 200, pull: 5, deployTime: 0.35, retractTime: 1.6, side: 'front', latched: false,
      compacts: true, arm: { x: 0.273, y: 0.17, len: 0.338, stowDeg: 112.5, deployDeg: -15.5 },
    },
    // one-piece hopper (their CAD): its panels sit over the shooter at the start and slide out
    // along slotted rails with the first intake, then stay out for the match. capacity /
    // retracted are the team's stated numbers; how much it really holds is measured from the
    // hopper (bay) below (hopper.js measureCapacity)
    storage: { capacity: 58, retracted: 40, extLen: 0.25, extend: 'latched' },
    // the hopper as the FUEL sees it (robot frame: x forward, y up, z to the side; meters).
    // Powered floor rollers slope down to the back, where compliant indexer wheels lift FUEL
    // into the drum.
    bay: {
      x0: -0.143, x1: 0.323, hw: 0.33, top: 0.53, extTop: 0.5, // under the hopper top (their CAD)
      floor: { a: 0.1, b: 0.266, lo: 0.09, hi: 0.17 },
      drive: 'floor', driveSpeed: 1.8,
      // up the roller ramp at the back of the hopper, under the hood to the drum (their CAD)
      feed: { x: -0.09, via: [[-0.1, 0.25], [-0.12, 0.38], [-0.2, 0.44]] },
    },
    shooter: {
      type: 'fixed',
      facing: 'back', // the drum at the back fires away from the intake
      lanes: [-0.19, -0.063, 0.063, 0.19],
      exit: { x: -0.34, y: 0.5 }, // where FUEL leaves the hood, over the back of the drum (their CAD)
      bps: 32,
      hoodMin: 42, hoodMax: 74,
      speedMax: 17,
      spinTau: 0.33,
      shotDrop: 0.002,
      speedSigma: 0.016, angleSigma: 0.8, yawSigma: 0.8,
    },
    climber: null,
    stats: { 'Shot rate': '32 BPS', Aiming: 'Chassis', 'Top speed': '14.1 ft/s', Trench: 'Yes' },
    colors: { frame: 0xb9bec5, accent: 0x5c6168, trim: 0xc6cbd1 }, // raw aluminum, grey plates (their CAD)
  },
  4414: {
    key: '4414',
    team: 4414,
    teamName: 'HighTide',
    robotName: 'RIPCURRENT',
    archetype: 'Dye Rotor',
    blurb: 'Extending hopper holds 88 FUEL, about 110 with its net stretched. A Dye Rotor single-streams FUEL into a fast turret shooter with precomputed shoot-on-the-move.',
    frame: { length: 25.0 * IN, width: 32.0 * IN },
    height: 21.75 * IN,
    mass: 60,
    drive: { maxSpeed: 4.0, maxAccel: 9.0, maxOmega: 8.0, maxAlpha: 30 },
    // the intake is a box that slides out on racks (extLen) with its roller at the lip
    // rate: fast enough that driving through FUEL is the only limit; pull: roller surface speed
    // lip: FUEL rides up the box's ramp to the ridge over the front bumper here, then drops in
    // behind it at entry (robot frame, m)
    intake: { width: 30 * IN, reach: 0.27 + 0.035 - 3.25 * IN, rate: 200, pull: 5, deployTime: 0.4, side: 'front', latched: true, lip: { x: 0.4, y: 0.29, entry: 0.35 } },
    // the intake box is also the hopper's extension (stated: 58 FUEL retracted, 88 out; the sim
    // measures what the bay holds)
    storage: { capacity: 88, retracted: 58, extLen: 0.27, extend: 'latched' },
    // Netted hopper over the Dye Rotor: printed stadium pieces funnel FUEL onto the rotor, which
    // spins, carrying it round (its Dolphin Fin sweeps the load) into a hook of passive rollers
    // that steers it to the feeder at the center column; the feeder wheels lift it up a ramp into
    // the turret. Chamfered back corners. The walls run all the way up (wallTop), where the top
    // plate carries the turret; a net covers the rest of the top, and it stretches: past the
    // stated 88 the load bulges it up (dome: its height at the middle; pinned at the edges, the
    // top plate from x0 back, and round the turret), as far as there's room over the robot.
    bay: {
      x0: -0.305, x1: 0.317, hw: 0.376, top: 0.53, chamfer: 0.12, wallTop: 0.53,
      dome: { h: 0.27, x0: -0.165, cx: -0.02, cz: 0, rHole: 0.2 },
      floor: { a: 0.105, b: 0, lo: 0.105, hi: 0.105 },
      funnel: { slope: 0.4, cap: 0.1 },
      // the intake box's ramp: up from the main floor to a ridge over the front bumper, then down
      // to the roller at the front of the box (lift: ball center over a steep ramp)
      ramp: { x: 0.4, y: 0.19, fwd: 1.05, lo: 0.02, lift: 0.03 },
      column: { x: -0.02, z: 0, r: 0.12, y1: 0.41 },
      obstacles: [
        { x: -0.02, z: 0, r: 0.12, y0: 0.1, y1: 1 }, // center column
        { x: -0.02, z: 0, r: 0.16, y0: 0.42, y1: 1 }, // the shooter, down inside the turret ring
      ],
      // drag: how the spinning platter carries FUEL along; grip: the Dolphin Fin (rim, from finR0)
      drive: 'rotor', rotor: { x: -0.02, z: 0, y: 0.105, r: 0.285, grip: 40, drag: 4, finR0: 0.175, spin: 14.1, idle: -0.6 },
      feed: { x: -0.02, z: 0.21, via: [[0.0, 0.25, 0.2], [-0.04, 0.34, 0.17], [-0.02, 0.46, 0]] },
      // hook over the rotor: from r0 at the rim (th0 upstream of the feed point) in to r1 by the
      // column (th1 just past it), y0..y1 over the rotor, r its half thickness
      hook: { r0: 0.27, r1: 0.15, th0: -1.15, th1: 0.25, y0: 0.1, y1: 0.2, r: 0.014 },
    },
    shooter: {
      type: 'turret',
      turretPos: { x: -0.02, z: 0.0 }, // on the Dye Rotor's center column
      exitRadius: 0.1,
      exitY: 0.53,
      turretRange: 200, turretRate: 720,
      bps: 18,
      hoodMin: 44, hoodMax: 82,
      speedMax: 16,
      spinTau: 0.25,
      shotDrop: 0.005,
      speedSigma: 0.013, angleSigma: 0.6, yawSigma: 0.6,
    },
    climber: null,
    stats: { 'Shot rate': '18 BPS', Aiming: 'Turret', 'Top speed': '13.1 ft/s', Trench: 'Yes' },
    colors: { frame: 0x0fa3b1, accent: 0x0fa3b1, trim: 0x2b2f36 }, // teal drive rails and truss, black / carbon above
  },
  8793: {
    key: '8793',
    team: 8793,
    teamName: 'Pumpkin Bots',
    robotName: 'Hopperless',
    archetype: 'Hopperless',
    blurb: 'No hopper: a wide intake feeds a conveyor straight into a turret shooter. Holds only what fits in the ball path — intake and shoot at the same time.',
    frame: { length: 27.5 * IN, width: 27.5 * IN },
    height: 19.5 * IN,
    mass: 52,
    drive: { maxSpeed: 4.5, maxAccel: 10.0, maxOmega: 9.0, maxAlpha: 34 },
    intake: { width: 26 * IN, reach: 10 * IN, rate: 14, deployTime: 0.3, side: 'front', latched: false },
    storage: { capacity: 12 },
    // no hopper: two lanes on the conveyor that climbs from the intake to the turret
    bay: {
      x0: -0.13, x1: 0.3, hw: 0.16, above: 0.26,
      floor: { a: 0.24, b: -0.543, lo: 0.115, hi: 0.28 },
      drive: 'belt', driveSpeed: 1.4,
      feed: { x: -0.08, via: [[-0.12, 0.42]] },
    },
    shooter: {
      type: 'turret',
      turretPos: { x: -0.12, z: 0.0 },
      exitRadius: 0.08,
      exitY: 0.47,
      turretRange: 185, turretRate: 600,
      bps: 13,
      hoodMin: 45, hoodMax: 80,
      speedMax: 15,
      spinTau: 0.28,
      shotDrop: 0.007,
      speedSigma: 0.017, angleSigma: 0.9, yawSigma: 0.9,
    },
    climber: null,
    stats: { 'Shot rate': '13 BPS', Aiming: 'Turret', 'Top speed': '14.8 ft/s', Trench: 'Yes' },
    colors: { frame: 0x1f2126, accent: 0xf07a1a, trim: 0xf07a1a },
  },
  971: {
    key: '971',
    team: 971,
    teamName: 'Spartan Robotics',
    robotName: 'Mixtape',
    archetype: 'Twin turrets',
    blurb: 'Two independently aimed turrets fed by a roller floor and a powered separator that splits FUEL into two streams. The hopper is pushed out by its 4-bar intake, and a net holds the load in over the top. Climbs Level 1.',
    // their CAD: bumpers +-0.394 x +-0.457 m
    frame: { length: 24.5 * IN, width: 29.5 * IN },
    height: 22 * IN,
    mass: 60,
    drive: { maxSpeed: 4.4, maxAccel: 9.5, maxOmega: 8.5, maxAlpha: 32 },
    // 4-bar ground intake under the hopper's front (their CAD: the beater tube folds up inside the
    // hopper, and deploying it pushes the hopper out)
    intake: { width: 29 * IN, reach: 0.18, rate: 200, pull: 5, deployTime: 0.35, side: 'front', latched: false },
    storage: { extLen: 0.28, extend: 'latched' },
    // the hopper as the FUEL sees it (their CAD): polycarbonate walls to 0.54 m, 0.7 m apart; the
    // roller floor slopes down to the back wall, under which the separator and kicker take FUEL
    // to the two ramps. The net over the open top bulges up with the load.
    bay: {
      x0: 0.0, x1: 0.34, hw: 0.345, top: 0.54,
      dome: { h: 0.15, x0: 0.0, cx: 0, cz: 0, rHole: -1 },
      floor: { a: 0.14, b: 0.12, lo: 0.14, hi: 0.22 },
      drive: 'floor', driveSpeed: 2.0,
      // separator in the middle at the back wall: each side goes up its own ramp into its turret
      feed: {
        x: 0.02, z: 0, zs: [-0.12, 0.12],
        vias: [
          [[-0.06, 0.24, -0.17], [-0.15, 0.34, -0.21], [-0.19, 0.46, -0.21]],
          [[-0.06, 0.24, 0.17], [-0.15, 0.34, 0.21], [-0.19, 0.46, 0.21]],
        ],
      },
    },
    shooter: {
      type: 'turret',
      // two 10in X-contact bearings on the platform behind the hopper (their CAD)
      turrets: [{ x: -0.148, z: -0.21 }, { x: -0.148, z: 0.21 }],
      turretPos: { x: -0.148, z: -0.21 },
      exitRadius: 0.1,
      exitY: 0.54,
      turretRange: 105, turretRate: 600, // ~210deg of travel between hard stops
      bps: 20,
      hoodMin: 35, hoodMax: 85, // 50deg of hood travel on the lead screw
      speedMax: 17,
      spinTau: 0.28,
      shotDrop: 0.004,
      speedSigma: 0.014, angleSigma: 0.7, yawSigma: 0.7,
    },
    climber: { maxLevel: 1, times: [0, 2.0] }, // one-stage telescoping arm
    stats: { 'Shot rate': '20 BPS (2 turrets)', Aiming: 'Twin turrets', 'Top speed': '14.4 ft/s', Trench: 'Yes', Climb: 'Level 1' },
    colors: { frame: 0xb4b9c1, accent: 0xc62828, trim: 0x2a2c31 }, // raw aluminum, red, black hoods
  },
  1678: {
    key: '1678',
    team: 1678,
    teamName: 'Citrus Circuits',
    robotName: 'Limestone',
    archetype: 'Drum + lift',
    blurb: 'Milstein Division champion. A full-width drum with three hood rollers fires out the back, fed by a roller floor and ball tunnel. The hopper grows out with the intake and up with the climber, with netting around the lifted lid. Climbs Level 1.',
    frame: { length: 27.0 * IN, width: 27.0 * IN },
    height: 21.6 * IN,
    mass: 62,
    drive: { maxSpeed: 4.5, maxAccel: 9.8, maxOmega: 8.8, maxAlpha: 33 },
    // full-width slapdown: 2in silicone roller + 1.25in kicker bar, pivot at the front
    intake: { width: 25 * IN, reach: 0.1, rate: 200, pull: 5, deployTime: 0.3, side: 'front', latched: false },
    // the horizontal extension rides out with the intake (a slanted slot in its side plates)
    storage: { extLen: 0.2, extend: 'intake' },
    // their CAD: roller floor (dead-axle rollers, then flex wheels) sloping down to the ball
    // tunnel at the back; the lid (corrugated plastic on the climber tubes) sits at 0.52 m and
    // lifts lift.h with the climber (it comes back down under the TRENCH), netting on the sides
    bay: {
      x0: -0.09, x1: 0.33, hw: 0.3, top: 0.52,
      lift: { h: 0.2 },
      floor: { a: 0.157, b: 0.3, lo: 0.12, hi: 0.23 },
      drive: 'floor', driveSpeed: 2.4,
      // up the ball tunnel (active backing rollers) to the drum
      feed: { x: -0.09, via: [[-0.14, 0.2], [-0.17, 0.34], [-0.2, 0.47]] },
    },
    shooter: {
      type: 'fixed',
      facing: 'back', // the drum at the back throws FUEL back over itself
      lanes: [-0.21, -0.07, 0.07, 0.21],
      exit: { x: -0.33, y: 0.56 }, // over the drum (their CAD)
      bps: 26,
      hoodMin: 38, hoodMax: 76,
      speedMax: 18,
      spinTau: 0.3,
      shotDrop: 0.002,
      speedSigma: 0.015, angleSigma: 0.75, yawSigma: 0.75,
    },
    climber: { maxLevel: 1, times: [0, 1.6] },
    stats: { 'Shot rate': '26 BPS', Aiming: 'Chassis', 'Top speed': '14.8 ft/s', Trench: 'Yes', Climb: 'Level 1' },
    colors: { frame: 0x1d1f24, accent: 0x5fd13a, trim: 0x2a2c31 }, // black anodized, lime
  },
  1690: {
    key: '1690',
    team: 1690,
    teamName: 'Orbit',
    robotName: 'Kepler',
    archetype: 'Compact turret',
    blurb: 'A compact gear-driven turret on an 8in bearing shoots on the move while the intake keeps running. The hopper expands forward under a lattice frame, and netting over the top stops FUEL bouncing out.',
    frame: { length: 25.0 * IN, width: 29.0 * IN },
    height: 21.5 * IN,
    mass: 58,
    // they geared down and went to spiked wheels mid-season for the BUMP, TRENCH and defense
    drive: { maxSpeed: 4.0, maxAccel: 10.5, maxOmega: 8.5, maxAlpha: 34 },
    // over-bumper: motorized main roller with light compression over an aluminum bottom roller,
    // deployed by surgical tubing (it stays down)
    intake: { width: 27 * IN, reach: 0.2, rate: 200, pull: 5, deployTime: 0.3, side: 'front', latched: true },
    storage: { extLen: 0.25, extend: 'latched' },
    // Onshape views: the powered roller floor covers the front of the robot; the turret sits at the
    // back left and the electronics box at the back right. A vertical kicker in front of the
    // turret lifts FUEL into it. Netting over the lattice frame on top.
    bay: {
      x0: -0.02, x1: 0.31, hw: 0.34, top: 0.45,
      dome: { h: 0.16, x0: -0.02, cx: -0.12, cz: -0.17, rHole: 0.16 },
      floor: { a: 0.13, b: 0.08, lo: 0.12, hi: 0.17 },
      drive: 'floor', driveSpeed: 2.0,
      feed: { x: -0.02, z: -0.17, via: [[-0.05, 0.26, -0.17], [-0.09, 0.4, -0.17]] },
    },
    shooter: {
      type: 'turret',
      turretPos: { x: -0.12, z: -0.17 },
      exitRadius: 0.09,
      exitY: 0.52,
      turretRange: 200, turretRate: 720,
      bps: 12,
      hoodMin: 40, hoodMax: 80,
      speedMax: 16,
      spinTau: 0.3, // no added flywheel mass
      shotDrop: 0.006,
      speedSigma: 0.012, angleSigma: 0.55, yawSigma: 0.55,
    },
    climber: null,
    stats: { 'Shot rate': '12 BPS', Aiming: 'Turret', 'Top speed': '13.1 ft/s', Trench: 'Yes' },
    colors: { frame: 0x2a2d33, accent: 0x2d6fd6, trim: 0x9aa0a8 }, // black lattice, blue, aluminum
  },
};

export const ROBOT_ORDER = ['2910', '4414', '8793', '971', '1678', '1690'];

// Optional add-on climbers for robots without one (971 and 1678 have their own, cfg.climber)
export const CLIMBER_OPTIONS = {
  none: null,
  l1: { maxLevel: 1, times: [0, 1.6] },
  l3: { maxLevel: 3, times: [0, 1.6, 4.0, 7.0] },
};

export const AUTO_ROUTINES = {
  best: { name: 'Best for this robot', desc: 'The highest-scoring AUTO found for the selected robot (its own start position, NEUTRAL ZONE sweeps and DEPOT runs).' },
  none: { name: 'No auto', desc: 'Robot sits still.' },
  preload: { name: 'Score preload', desc: 'Spin up and shoot the preloaded FUEL from the start line.' },
  depot: { name: 'Preload + Depot', desc: 'Shoot the preload, collect the 24 FUEL in the DEPOT, drive back and shoot.' },
  sweep: { name: 'Neutral Zone sweep', desc: 'Through the TRENCH to the NEUTRAL ZONE, sweep the FUEL line, return over the BUMP shooting on the move.' },
  sweep2: { name: 'Double sweep', desc: 'Two NEUTRAL ZONE sweeps (2910-style), shooting on the move each time back in the ALLIANCE ZONE.' },
  climb: { name: 'Preload + Climb L1', desc: 'Shoot the preload then climb LEVEL 1 for 15 points (needs a climber add-on).' },
  defendStage: { name: 'Preload + defensive position', desc: 'Shoot the preload, then drive over the BUMP to the middle of the field (own side of the CENTER LINE) to start TELEOP on defense.' },
};
