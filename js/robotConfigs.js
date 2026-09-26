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
    intake: { width: 25.5 * IN, reach: 7.8 * IN, rate: 26, deployTime: 0.35, side: 'front', latched: false },
    // one-piece hopper whose front panels expand out over the intake as it deploys
    // (40 FUEL stowed, 58 deployed)
    storage: { capacity: 58, retracted: 40, extLen: 0.2, extend: 'intake' },
    // the hopper as the FUEL sees it (robot frame: x forward, y up, z to the side; meters).
    // Powered floor rollers slope down to the back, where compliant indexer wheels lift FUEL
    // into the drum.
    bay: {
      x0: -0.143, x1: 0.323, hw: 0.33, top: 0.53, extTop: 0.47, // under the lids
      floor: { a: 0.1, b: 0.266, lo: 0.09, hi: 0.17 },
      drive: 'floor', driveSpeed: 1.8,
      // indexer, up the front of the drum, over the top under the hood
      feed: { x: -0.09, via: [[-0.16, 0.25], [-0.16, 0.42], [-0.26, 0.5]] },
    },
    shooter: {
      type: 'fixed',
      facing: 'back', // the drum at the back fires away from the intake
      lanes: [-0.19, -0.063, 0.063, 0.19],
      exit: { x: -0.35, y: 0.46 }, // where FUEL leaves the hood, at the back of the drum
      bps: 32,
      hoodMin: 42, hoodMax: 74,
      speedMax: 17,
      spinTau: 0.33,
      shotDrop: 0.002,
      speedSigma: 0.016, angleSigma: 0.8, yawSigma: 0.8,
    },
    climber: null,
    stats: { Capacity: 58, 'Shot rate': '32 BPS', Aiming: 'Chassis', 'Top speed': '14.1 ft/s', Trench: 'Yes' },
    colors: { frame: 0xb9bec5, accent: 0x5c6168, trim: 0xc6cbd1 }, // raw aluminum, grey plates (their CAD)
  },
  4414: {
    key: '4414',
    team: 4414,
    teamName: 'HighTide',
    robotName: 'RIPCURRENT',
    archetype: 'Dye Rotor',
    blurb: 'Extending hopper holds 85+ FUEL. A Dye Rotor single-streams FUEL into a fast turret shooter with precomputed shoot-on-the-move.',
    frame: { length: 25.0 * IN, width: 32.0 * IN },
    height: 21.75 * IN,
    mass: 60,
    drive: { maxSpeed: 4.0, maxAccel: 9.0, maxOmega: 8.0, maxAlpha: 30 },
    // the intake is a box that slides out on racks (extLen) with its roller at the lip
    intake: { width: 30 * IN, reach: 0.27 + 0.035 - 3.25 * IN, rate: 30, deployTime: 0.4, side: 'front', latched: true },
    // the intake box is also the hopper's extension (58 FUEL retracted, 88 out)
    storage: { capacity: 88, retracted: 58, extLen: 0.27, extend: 'latched' },
    // Netted hopper over the Dye Rotor: printed stadium pieces funnel FUEL onto the floor, where
    // the rotating Dolphin Fin sweeps it round to the center column; a ramp of passive rollers
    // climbs the column to the feeder wheels and the turret on top. Chamfered back corners.
    bay: {
      x0: -0.305, x1: 0.317, hw: 0.376, top: 0.53, extTop: 0.47, chamfer: 0.12,
      floor: { a: 0.105, b: 0, lo: 0.105, hi: 0.105 },
      funnel: { slope: 0.4, cap: 0.1 },
      column: { x: -0.02, z: 0, r: 0.12, y1: 0.41 },
      obstacles: [
        { x: -0.02, z: 0, r: 0.12, y0: 0.1, y1: 1 }, // center column and turret
      ],
      drive: 'rotor', rotor: { x: -0.02, z: 0, y: 0.105, r: 0.285, grip: 40, spin: 14.1, idle: -0.6 },
      feed: { x: -0.02, z: 0.21, via: [[0.0, 0.25, 0.2], [-0.04, 0.34, 0.17], [-0.02, 0.46, 0]] },
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
    stats: { Capacity: 88, 'Shot rate': '18 BPS', Aiming: 'Turret', 'Top speed': '13.1 ft/s', Trench: 'Yes' },
    colors: { frame: 0x22262d, accent: 0x0fa3b1, trim: 0x2b2f36 }, // black / carbon with the teal truss
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
    stats: { Capacity: 12, 'Shot rate': '13 BPS', Aiming: 'Turret', 'Top speed': '14.8 ft/s', Trench: 'Yes' },
    colors: { frame: 0x1f2126, accent: 0xf07a1a, trim: 0xf07a1a },
  },
};

export const ROBOT_ORDER = ['2910', '4414', '8793'];

// Optional add-on climbers (none of the three real robots climbed)
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
