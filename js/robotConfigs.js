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
//  - 4946 The Alpha Dogs "Moto Moto" (public Onshape CAD, view only, + engineering report): round
//    robot (half-circle drivetrain, 30in front edge), 35in hopper with clear walls on the bumpers
//    round a Dye Rotor tray, 11in turret on the center of rotation, sliding ground intake. Too tall
//    for the TRENCH.
//  - 3928 Team Neutrino (public Onshape CAD "26"): 27x27in swerve (MK5n), tall sheet hopper walls,
//    a 5-spoke spindexer with a center cone, a tower in the back corner up to a turret on top, a
//    slide-out intake box. Too tall for the TRENCH.
//  - 341 Miss Daisy XXIV (public Onshape CAD): 27x27in swerve (WCP X2i), triple-roller slapdown,
//    a one-layer serializer tray under the turret table (three flat 6in omni wheels), L1 climber.
//  - 4930 Electric Mayhem "Floyd 2" (public Onshape CAD + Open Alliance thread): 28.5x26.5in swerve
//    (MK4i/MK4n), belt floor to an updexer, three hooded flywheels on one shaft fixed to the chassis
//    and firing forward over the intake. Too tall for the TRENCH.
//  Nets: 971, 1690 and 3928 carry netting over the open tops of their hoppers (the physics lets the
//  load bulge it up, bay.dome); 1678, 4946 and 4930 stretch a net diagonally from the hopper's top
//  edge down to the front of the intake (bay.slope). None of the teams' CAD models its nets.
import { IN, TRENCH } from './constants.js';
import { Drivetrain } from './drivetrain.js';

export const BUMPER_T = 3.25 * IN; // bumper thickness incl. backing
// bumper height off the carpet: 1.25in up (clear of the 1.125in DEPOT barrier) and 5in tall, like real FRC bumpers (two pool
// noodles), so they meet FUEL near its middle and push it rather than wedging it underneath
export const BUMPER_Y0 = 0.032, BUMPER_Y1 = 0.16;

// The frame perimeter in plan (x forward, z to the side, a closed polygon starting at the front):
// a rectangle, one with its back corners cut off (frame.chamfer, 4414's), or a half circle at the
// back with straight sides to a flat front edge (frame.round, 4946's). The bumpers, the drawn
// frame and the physics all follow it.
export function frameShape(cfg) {
  const f = cfg.frame, L = f.length, W = f.width;
  if (f.round) {
    const { r, cx, front } = f.round, pts = [[L / 2, front / 2]];
    for (let i = 0; i <= 24; i++) { const a = Math.PI / 2 + (i / 24) * Math.PI; pts.push([cx + r * Math.cos(a), r * Math.sin(a)]); }
    pts.push([L / 2, -front / 2]);
    return pts;
  }
  const c = f.chamfer || 0, pts = [[L / 2, W / 2], [L / 2, -W / 2]];
  if (c > 0) pts.push([-L / 2 + c, -W / 2], [-L / 2, -W / 2 + c], [-L / 2, W / 2 - c], [-L / 2 + c, W / 2]);
  else pts.push([-L / 2, -W / 2], [-L / 2, W / 2]);
  return pts;
}

// the polygon moved out by d (in by -d), each edge parallel to the original
export function offsetShape(pts, d) {
  const n = pts.length;
  let area = 0;
  for (let i = 0; i < n; i++) { const a = pts[i], b = pts[(i + 1) % n]; area += a[0] * b[1] - b[0] * a[1]; }
  const s = area > 0 ? 1 : -1; // outward normal of edge (dx, dz) is s * (dz, -dx) / len
  const nrm = (a, b) => { const dx = b[0] - a[0], dz = b[1] - a[1], l = Math.hypot(dx, dz) || 1; return [(s * dz) / l, (-s * dx) / l]; };
  return pts.map((p, i) => {
    const n0 = nrm(pts[(i - 1 + n) % n], p), n1 = nrm(p, pts[(i + 1) % n]);
    const mx = n0[0] + n1[0], mz = n0[1] + n1[1], k = d / Math.max(0.2, (mx * n0[0] + mz * n0[1]));
    return [p[0] + mx * k, p[1] + mz * k];
  });
}

// where the swerve modules sit (x, z): 3in in from the corners, or inside a round frame
export function modulePoints(cfg) {
  const f = cfg.frame, inset = 0.075;
  if (f.round) {
    const { r, cx } = f.round, d = (r - 0.1) * Math.SQRT1_2;
    return [[cx + d, d], [cx + d, -d], [cx - d, d], [cx - d, -d]];
  }
  return [[1, 1], [1, -1], [-1, 1], [-1, -1]].map(([sx, sz]) => [sx * (f.length / 2 - inset), sz * (f.width / 2 - inset)]);
}

// whether a robot fits under the TRENCH arm (4946 doesn't: it goes over the BUMPS)
export const fitsTrench = (cfg) => cfg.height <= TRENCH.clearHeight - 0.005;

// Top of an intake that folds up over the robot to stow (intake.fold, 8793's): its side outline
// swung up from deployed by (1 - deploy) x stowDeg about the pivot. 0 for other intakes.
export function foldTop(ic, deploy) {
  const f = ic.fold;
  if (!f) return 0;
  const a = (1 - deploy) * f.stowDeg * Math.PI / 180, c = Math.cos(a), s = Math.sin(a);
  let top = -Infinity;
  for (const [x, y] of f.hull) top = Math.max(top, x * s + y * c);
  return f.pivot[1] + top;
}

export const ROBOTS = {
  2910: {
    key: '2910',
    team: 2910,
    teamName: 'Jack in the Bot',
    robotName: 'Re•Blitz',
    archetype: 'Dumper',
    blurb: 'Huge hopper and a 4-wide drum shooter fixed to the chassis — the whole robot rotates to aim. Unloads 45 FUEL per second.',
    frame: { length: 27.0 * IN, width: 27.5 * IN },
    height: 21.5 * IN,
    mass: 64, // robot (up to 115 lb) + bumpers + battery
    // SDS MK5n R1 (7.03:1, 4in wheels), Kraken X60s (see drivetrain.js)
    drive: { ratio: 7.03 },
    // slap-down intake from 2910's CAD: its 2in roller reaches ~7.8in past the bumper
    // rate: fast enough that driving through FUEL is the only limit; pull: roller surface speed
    // arm: the CAD intake's pivot and its 2in roller's reach and angle, stowed (as exported) and
    // down on the carpet. While shooting it retracts slowly and pushes the load back into the
    // indexer (compacts), stalling against the FUEL until there's room.
    intake: {
      width: 25.5 * IN, reach: 7.8 * IN, rate: 200, pull: 5, deployTime: 0.35, retractTime: 1.6, side: 'front', latched: false,
      compacts: true, arm: { x: 0.273, y: 0.17, len: 0.338, stowDeg: 112.5, deployDeg: -15.5 },
      // retracting over the load it crams it into the indexer, far harder than its roller pushes
      // FUEL in (hopper.js intakePush): it stalls once the load pushes back this hard (N, estimate)
      compactPush: 180,
    },
    // one-piece hopper (their CAD): its panels sit over the shooter at the start and slide out
    // along slotted rails with the first intake, then stay out for the match. capacity /
    // retracted are the team's stated numbers; how much it really holds is measured from the
    // hopper (bay) below (hopper.js measureCapacity)
    storage: { capacity: 58, retracted: 40, extLen: 0.27, extend: 'latched' },
    // the hopper as the FUEL sees it (robot frame: x forward, y up, z to the side; meters).
    // Powered floor rollers slope down to the back, where compliant indexer wheels lift FUEL
    // into the drum.
    bay: {
      x0: -0.143, x1: 0.323, hw: 0.33, top: 0.54, extTop: 0.51, // under the hopper top (their CAD)
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
      bps: 45,
      hoodMin: 42, hoodMax: 74,
      speedMax: 17,
      spinTau: 0.33,
      shotDrop: 0.002,
      speedSigma: 0.016, angleSigma: 0.8, yawSigma: 0.8,
    },
    climber: null,
    stats: { 'Shot rate': '45 BPS', Aiming: 'Chassis', 'Top speed': '14.1 ft/s', Trench: 'Yes' },
    colors: { frame: 0xb9bec5, accent: 0x5c6168, trim: 0xc6cbd1 }, // raw aluminum, grey plates (their CAD)
  },
  4414: {
    key: '4414',
    team: 4414,
    teamName: 'HighTide',
    robotName: 'RIPCURRENT',
    archetype: 'Dye Rotor',
    blurb: 'Extending hopper holds about 88 FUEL under a stretchy net. A Dye Rotor single-streams FUEL into a fast turret shooter with precomputed shoot-on-the-move.',
    frame: { length: 25.0 * IN, width: 32.0 * IN, chamfer: 0.12 }, // back corners cut off
    height: 21.75 * IN,
    mass: 64, // robot (up to 115 lb) + bumpers + battery
    // 7.67:1 on 4in wheels (tech binder), Kraken X60s
    drive: { ratio: 7.67 },
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
    blurb: 'No hopper: a 4-wide intake feeds a conveyor that funnels FUEL to single file, up through the turret and out a hooded flywheel. Holds only what fits in the ball path — intake and shoot at the same time.',
    frame: { length: 27.5 * IN, width: 27.5 * IN },
    height: 21.5 * IN, // top of the shooter (their CAD)
    mass: 58, // robot (up to 115 lb) + bumpers + battery
    // WCP Swerve X2t, geared for their stated 14.8 ft/s free speed (6.84:1 on 4in wheels)
    drive: { ratio: 6.84 },
    // Intake V3 (their CAD): three silicone rollers on an arm that swings down from a pivot over
    // the front of the frame. Stowed, it folds back up inside the frame perimeter and stands
    // 0.69 m tall, too tall for the TRENCH: it has to be down to drive under. fold.hull is the
    // arm's side outline around the pivot (m, x forward, y up) as exported, deployed.
    intake: {
      width: 26 * IN, reach: 10 * IN, rate: 14, deployTime: 0.3, side: 'front', latched: false,
      fold: {
        pivot: [0.292, 0.336], stowDeg: 145,
        hull: [[-0.026, -0.018], [0.161, -0.305], [0.176, -0.306], [0.298, -0.196], [0.328, -0.105], [0.191, 0.009], [0.005, 0.032], [-0.025, 0.02]],
      },
    },
    storage: { capacity: 12 },
    // no hopper, a ball path (their CAD). FUEL comes in 4 wide over the front of the frame (the
    // outer two ride the swerve covers) and Conveyor V2's overhead wheels (4in omnis on the sides
    // push in) carry it back down its polycarbonate floor, 2 wide, to the middle of the robot.
    // There the turret indexer's wall plates close in to one FUEL wide, and single file it runs
    // back along the J-shaped roller rails under the turntable, up their curve, and up through
    // the turret into the shooter, out forward over the flywheel under the hood.
    bay: {
      x0: -0.28, x1: 0.34, hw: 0.31, above: 0.2, pack: false, // poured, not packed
      // [x, half-width]; at the throat 5in (-z) and 3in (+z) omni wheels spin FUEL against each other
      taper: { pts: [[0.03, 0.087], [0.17, 0.165], [0.23, 0.31]], center: 60, spin: -1 },
      // [x, height of the FUEL's underside]: the conveyor floor down to the middle, the rails' J
      floor: { pts: [[-0.21, 0.165], [-0.12, 0.045], [0.03, 0.06], [0.34, 0.175]] },
      drive: 'belt', driveSpeed: 2.2, grip: 80, // compliant wheels grab FUEL hard: single file keeps up with the shooter
      feed: { x: -0.2, reach: 0.1, via: [[-0.19, 0.34], [-0.14, 0.42]] },
    },
    shooter: {
      type: 'turret',
      turretPos: { x: -0.127, z: 0.0 }, // 200T turntable (their CAD)
      exitRadius: 0.14, // over the flywheel, which sits 0.127 m out from the turret axis
      exitY: 0.49,
      turretRange: 185, turretRate: 600,
      bps: 10, // the team's measured rate
      hoodMin: 45, hoodMax: 80,
      speedMax: 15,
      spinTau: 0.28,
      shotDrop: 0.007,
      speedSigma: 0.017, angleSigma: 0.9, yawSigma: 0.9,
    },
    climber: null,
    stats: { 'Shot rate': '10 BPS', Aiming: 'Turret', 'Top speed': '14.8 ft/s', Trench: 'Intake down' },
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
    mass: 64, // robot (up to 115 lb) + bumpers + battery
    // SDS MK5 at 7.03:1 on 4in wheels (their stated 14.4 ft/s free speed)
    drive: { ratio: 7.03 },
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
      // two 10in X-contact bearings on the platform behind the hopper (their CAD). FUEL leaves
      // over the flywheel, on the opposite side from the hood. Each turret's ~210deg of travel is
      // centered pointing back and out to its own side (center, deg), so it shoots away from the
      // robot, and together they cover everything but straight ahead
      turrets: [{ x: -0.148, z: -0.21, center: 135 }, { x: -0.148, z: 0.21, center: -135 }],
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
    blurb: 'Milstein Division champion. A full-width drum with three hood rollers fires out the back, fed by a roller floor and ball tunnel. The hopper grows out with the intake and up with the climber, with a net stretched diagonally over the extension. Climbs Level 1.',
    frame: { length: 27.0 * IN, width: 27.0 * IN },
    height: 21.6 * IN,
    mass: 65, // robot (up to 115 lb) + bumpers + battery
    // SDS MK5n R1 (7.03:1, 4in wheels)
    drive: { ratio: 7.03 },
    // full-width slapdown: 2in silicone roller + 1.25in kicker bar, pivot at the front
    // a 2in silicone-covered carbon fiber roller (their reveal): rigid, so it squeezes FUEL a
    // little less than compliant wheels do (hopper.js intakePush)
    intake: { width: 25 * IN, reach: 0.1, rate: 200, pull: 5, deployTime: 0.3, side: 'front', latched: false, squeeze: 0.6 },
    // the horizontal extension rides out with the intake (a slanted slot in its side plates)
    storage: { extLen: 0.2, extend: 'intake' },
    // their CAD: roller floor (dead-axle rollers, then flex wheels) sloping down to the ball
    // tunnel at the back; the lid (corrugated plastic on the climber tubes) sits at 0.52 m and
    // lifts lift.h with the climber (it comes back down under the TRENCH); over the extension the
    // ceiling is the diagonal net
    bay: {
      x0: -0.09, x1: 0.33, hw: 0.3, top: 0.52,
      lift: { h: 0.2 },
      // the net from the lid's front edge down to the front of the extension
      slope: { x: 0.335, y: 0.5 },
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
    mass: 63, // robot (up to 115 lb) + bumpers + battery
    // they geared down and went to spiked wheels mid-season for the BUMP, TRENCH and defense
    // geared down (and spiked wheels) mid-season for the BUMP, TRENCH and defense: 13.1 ft/s free
    drive: { ratio: 7.73 },
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
  4946: {
    key: '4946',
    team: 4946,
    teamName: 'The Alpha Dogs',
    robotName: 'Moto Moto',
    archetype: 'Round Dye Rotor',
    blurb: 'A round robot: a 35in hopper around a turret on the center of rotation, fed by a Dye Rotor tray. Its intake slides out with a net stretched from the hopper down to it. Too tall for the TRENCH, it goes over the BUMPS.',
    // engineering report and Onshape's top view: a half-circle bellypan of 16.375in radius (its
    // center 3cm ahead of the middle, under the turret) whose sides run straight on to a 30in
    // front edge: 32.75in across, 30.3in front to back
    frame: { length: 0.77, width: 2 * 16.375 * IN, round: { r: 16.375 * IN, cx: 16.375 * IN - 0.77 / 2, front: 30 * IN } }, // cx: the back is at -length/2
    height: 29.5 * IN,
    mass: 66, // robot (up to 115 lb) + bumpers + battery
    // MK5n in its lowest gear, their stated 12.8 ft/s free speed
    drive: { ratio: 7.91 },
    // ground intake that slides out the front in two pieces (rollers with compliant wheels, passive
    // rollers behind that resist backpressure)
    intake: { width: 27 * IN, reach: 0.22, rate: 200, pull: 5, deployTime: 0.35, side: 'front', latched: false },
    // the intake box is part of the hopper while it's out
    storage: { extLen: 0.2, extend: 'intake' },
    // Onshape views: clear walls mounted on the bumpers, 1.125in out from the frame (a 35in hopper,
    // round at the back, then along the frame's sides), round a Dye Rotor tray (FUEL is funneled
    // onto it, and its spinner carries FUEL round and up into the turret at the center). The net
    // runs diagonally from the top of the hopper's front wall down to the front of the intake.
    bay: {
      x0: -0.415, x1: 0.37, hw: 0.445, top: 0.64,
      round: { x: 16.375 * IN - 0.77 / 2, r: 35 / 2 * IN },
      taper: { pts: [[0.031, 0.445], [0.385, 0.41]] },
      slope: { x: 0.37, y: 0.46 },
      floor: { a: 0.2, b: 0, lo: 0.2, hi: 0.2 },
      obstacles: [{ x: 0.02, z: 0, r: 0.15, y0: 0.15, y1: 1 }], // the turret's column
      drive: 'rotor', rotor: { x: 0.02, z: 0, y: 0.2, r: 0.4, grip: 30, drag: 6, finR0: 0.16, spin: 9, idle: -0.5 },
      feed: { x: 0.02, z: 0.2, via: [[0.02, 0.3, 0.15], [0.02, 0.5, 0.05], [0.02, 0.66, 0]] },
    },
    shooter: {
      type: 'turret',
      turretPos: { x: 0.02, z: 0 }, // on the center of rotation
      exitRadius: 0.1,
      exitY: 0.72,
      turretRange: 200, turretRate: 720,
      bps: 20,
      hoodMin: 40, hoodMax: 80, // worm-driven hood
      speedMax: 17,
      spinTau: 0.28,
      shotDrop: 0.004,
      speedSigma: 0.013, angleSigma: 0.6, yawSigma: 0.6,
    },
    climber: null,
    stats: { 'Shot rate': '20 BPS', Aiming: 'Turret', 'Top speed': '12.8 ft/s', Trench: 'No (BUMPS)' },
    colors: { frame: 0x9aa0a8, accent: 0xd32f2f, trim: 0x1d1f24 },
  },
  3928: {
    key: '3928',
    team: 3928,
    teamName: 'Team Neutrino',
    robotName: '', // not published; their CAD's top assembly is just "26"
    archetype: 'Spindexer tower',
    blurb: 'A 5-spoke spindexer with a cone in the middle turns the load round to a tower in the back corner, which lifts FUEL single file to a turret on top. Tall walls and a slide-out intake box make a deep hopper. Too tall for the TRENCH, it goes over the BUMPS.',
    // their CAD: 27x27in frame (rails at +-0.343 m), bumpers at +-0.433 m
    frame: { length: 27 * IN, width: 27 * IN },
    height: 29.3 * IN, // top of the hopper walls and the intake box (their CAD)
    mass: 64, // robot (up to 115 lb) + bumpers + battery
    // SDS MK5n; the ratio isn't in the CAD, so R1 (7.03:1) is assumed
    drive: { ratio: 7.03 },
    // the intake is a tall box (its sides are hopper walls) that slides 0.3 m out the front on
    // racks, with a 2in and a 1.5in roller at its lip; it stays out (their CAD, exported out)
    intake: { width: 26 * IN, reach: 0.19, rate: 200, pull: 5, deployTime: 0.4, side: 'front', latched: true },
    storage: { extLen: 0.3, extend: 'latched' },
    // their CAD: walls to 0.74 m round a spindexer at the front middle: a spoked wheel (r 0.14 m)
    // turning under the load with a printed cone over its hub, in a ring the floor wedges in the
    // corners slope down into. The tower fills the back left corner; FUEL the spokes bring round
    // to its foot rides a J-shaped ramp into its belts and up to the turret.
    bay: {
      x0: -0.33, x1: 0.33, hw: 0.33, top: 0.74,
      // a net over the open top, from the wall tops to the intake box's top crossbar, with a hole
      // round the turret; the load bulges it up
      dome: { h: 0.15, x0: -0.33, cx: -0.154, cz: 0.0985, rHole: 0.22 },
      floor: { a: 0.105, b: 0, lo: 0.105, hi: 0.105 },
      funnel: { slope: 0.35, cap: 0.3 },
      obstacles: [
        { x: 0.1125, z: -0.0175, r: 0.055, y0: 0.1, y1: 0.29 }, // the cone
        { box: [-0.33, -0.1, 0, 1, -0.02, 0.2] }, // the tower (its mouth faces the spindexer)
        { box: [-0.04, 0.11, 0.2, 0.52, 0.19, 0.33] }, // a plate on the spindexer's far side
      ],
      drive: 'rotor', rotor: { x: 0.1125, z: -0.0175, y: 0.105, r: 0.2, grip: 80, drag: 8, finR0: 0.06, fins: 5, spin: -15, idle: -0.5 },
      feed: { x: -0.05, z: 0.07, reach: 0.18, via: [[-0.09, 0.2, 0.09], [-0.15, 0.4, 0.09], [-0.15, 0.6, 0.095]] },
    },
    shooter: {
      type: 'turret',
      turretPos: { x: -0.154, z: 0.0985 }, // the bearing on top of the tower (their CAD)
      exitRadius: 0.1, // over the flywheels at the turret's front edge
      exitY: 0.76,
      turretRange: 180, turretRate: 600,
      bps: 12,
      hoodMin: 42, hoodMax: 80,
      speedMax: 16,
      spinTau: 0.3,
      shotDrop: 0.005,
      speedSigma: 0.014, angleSigma: 0.7, yawSigma: 0.7,
    },
    climber: null,
    stats: { 'Shot rate': '12 BPS', Aiming: 'Turret', 'Top speed': '', Trench: 'No (BUMPS)' },
    colors: { frame: 0x2a2c31, accent: 0xf08a1c, trim: 0xb9bec5 }, // black tube, orange plates
  },
  341: {
    key: '341',
    team: 341,
    teamName: 'Miss Daisy',
    robotName: 'Miss Daisy XXIV',
    archetype: 'Serializer',
    blurb: 'Almost no hopper: a triple-roller slapdown lifts FUEL onto a single-layer tray under the turret table, where three spinning 6in omni wheels serialize it into the uptake to the turret. Shoots as fast as it intakes. Climbs Level 1.',
    // their CAD: bumpers +-0.428 m, 27x27in frame
    frame: { length: 27 * IN, width: 27 * IN },
    height: 21.75 * IN, // the shooter on the table (their CAD)
    mass: 62, // robot (up to 115 lb) + bumpers + battery
    // WCP Swerve X2i in its low gearing (their CAD); ~7.1:1 assumed
    drive: { ratio: 7.13 },
    // "Main Slap Down (brontosaurus)" (their CAD): two silicone rollers at the bottom of an arm
    // that swings down from a pivot 0.34 m up, three plastic rollers over them lift FUEL over the
    // bumper. Stowed it stands up, too tall for the TRENCH: it has to be down to drive under.
    // fold.hull: the arm's side outline round the pivot (m, x forward, y up), deployed.
    intake: {
      width: 24 * IN, reach: 0.18, rate: 20, deployTime: 0.3, side: 'front', latched: false,
      fold: {
        pivot: [0.2545, 0.3365], stowDeg: 95,
        hull: [[-0.03, 0.035], [0.2, 0.03], [0.37, -0.09], [0.37, -0.13], [0.34, -0.2], [0.21, -0.3], [0.17, -0.305], [-0.02, -0.1]],
      },
    },
    storage: { capacity: 12 },
    // their CAD: a plate 0.175 m up across the front of the robot under the table the turret sits
    // on (0.375 m): room for one layer of FUEL. Three 6in omni wheels lying flat at the FUEL's
    // middle spin it round toward the uptake ramp at the back, which curls up into the turret.
    bay: {
      x0: -0.03, x1: 0.335, hw: 0.33, top: 0.37, pack: false,
      floor: { a: 0.175, b: 0, lo: 0.175, hi: 0.175 },
      // the omni wheels' hubs (r 0.076 m wheels: their rollers drive FUEL on, feed.pull, rather
      // than stopping it)
      obstacles: [
        { x: -0.0445, z: 0.0755, r: 0.03, y0: 0.22, y1: 0.28, wheel: 0.076 },
        { x: 0.108, z: 0.219, r: 0.03, y0: 0.22, y1: 0.28, wheel: 0.076 },
        { x: 0.1205, z: -0.2285, r: 0.03, y0: 0.22, y1: 0.28, wheel: 0.076 },
        { box: [-0.2, 0.095, 0, 1, -0.4, -0.148] }, // the wall beside the uptake, and the corner behind it
      ],
      drive: 'floor', driveSpeed: 1.6,
      feed: { x: -0.01, z: -0.075, reach: 0.14, pull: 15, via: [[-0.06, 0.3, -0.075], [-0.1, 0.42, -0.077]] },
    },
    shooter: {
      type: 'turret',
      turretPos: { x: -0.1015, z: -0.0765 }, // the 92T turret gear under the shooter (their CAD)
      exitRadius: 0.17, // the hood end of the shooter
      exitY: 0.52,
      turretRange: 180, turretRate: 600,
      bps: 12,
      hoodMin: 40, hoodMax: 80,
      speedMax: 16,
      spinTau: 0.3,
      shotDrop: 0.006,
      speedSigma: 0.014, angleSigma: 0.7, yawSigma: 0.7,
    },
    climber: { maxLevel: 1, times: [0, 1.8] }, // "L1 Climb": a telescoping tube in the back corner
    stats: { 'Shot rate': '12 BPS', Aiming: 'Turret', 'Top speed': '', Trench: 'Intake down', Climb: 'Level 1' },
    colors: { frame: 0xc3c8ce, accent: 0x2f5fb3, trim: 0xf2c318 }, // raw aluminum, blue, yellow
  },
  4930: {
    key: '4930',
    team: 4930,
    teamName: 'Electric Mayhem',
    robotName: 'Floyd 2',
    archetype: 'Triple shooter',
    blurb: 'Three hooded flywheel shooters side by side, fixed to the chassis and firing forward over the intake: the robot drives at the HUB to aim. A belt floor carries FUEL back to an updexer that lifts it into all three. Too tall for the TRENCH, it goes over the BUMPS.',
    // their build thread and CAD: 28.5x26.5in drivebase (wide), bumpers 0.928 x 0.851 m
    frame: { length: 26.5 * IN, width: 28.5 * IN },
    height: 25.8 * IN, // the hoods (their CAD)
    mass: 65, // robot (~120 lb stated before they took the climber off) + bumpers + battery
    // SDS MK4i / MK4n; L2 (6.75:1) assumed
    drive: { ratio: 6.75 },
    // "Intake V2" (their CAD): polycarbonate side plates and rollers on an arm pivoting at the
    // front of the frame, with its own belt floor. Stowed it swings up in front of the shooters.
    // fold.hull: its side outline round the pivot (m, x forward, y up), deployed.
    intake: {
      width: 28 * IN, reach: 0.2, rate: 200, pull: 5, deployTime: 0.35, side: 'front', latched: false,
      fold: {
        pivot: [0.2355, 0.2005], stowDeg: 90,
        hull: [[-0.03, 0.02], [0.2, 0.17], [0.37, 0.17], [0.39, 0.0], [0.39, -0.13], [0.35, -0.16], [0.02, -0.16], [-0.03, -0.05]],
      },
    },
    // the intake "drives an expanding hopper" (build thread): with the arm down, the space over it
    // is hopper too
    storage: { extLen: 0.22, extend: 'intake' },
    // their CAD: twenty belts make a floor sloping down from the intake to the updexer at the
    // back, whose belts lift FUEL up behind the three flywheels on one long shaft; each FUEL goes
    // round the back of its flywheel under the hood and out forward over the top.
    bay: {
      x0: -0.02, x1: 0.33, hw: 0.31, top: 0.55,
      // a net stretched from the front of the shooters' side plates down to the top of the
      // intake's side plates keeps the load in over the open front
      slope: { x: 0.12, y: 0.37 },
      floor: { a: 0.11, b: 0.385, lo: 0.11, hi: 0.23 },
      drive: 'floor', driveSpeed: 2.2,
      feed: { x: 0.0, via: [[-0.08, 0.2], [-0.19, 0.36], [-0.19, 0.52]] },
    },
    shooter: {
      type: 'fixed',
      facing: 'front', // over the top of the flywheels, forward over the intake
      lanes: [-0.205, 0, 0.205],
      exit: { x: -0.07, y: 0.68 }, // over the flywheels (their CAD)
      bps: 18,
      hoodMin: 40, hoodMax: 78,
      speedMax: 17,
      spinTau: 0.35,
      shotDrop: 0.003,
      speedSigma: 0.016, angleSigma: 0.8, yawSigma: 0.8,
    },
    climber: null, // taken off to make weight (build thread)
    stats: { 'Shot rate': '18 BPS (3 lanes)', Aiming: 'Chassis', 'Top speed': '', Trench: 'No (BUMPS)' },
    colors: { frame: 0xb4b9c1, accent: 0x3fa34d, trim: 0xc86bb5 }, // aluminum, green hoods, pink plates
  },
};

// Top speed, turn rate and acceleration come from each robot's drivetrain (drivetrain.js): its
// gear ratio, wheels and motors, current limits, battery and mass. The robot card shows the real
// top speed next to the gearing's free speed.
for (const cfg of Object.values(ROBOTS)) {
  const [mx, mz] = modulePoints(cfg)[0];
  const dt = new Drivetrain(cfg.drive, cfg.mass, cfg.frame.length, cfg.frame.width, Math.hypot(mx, mz));
  Object.assign(cfg.drive, dt.stats());
  const ft = (v) => (v / 0.3048).toFixed(1);
  if (cfg.stats) cfg.stats['Top speed'] = `${ft(dt.topSpeed)} ft/s (${ft(dt.freeSpeed)} free)`;
}

export const ROBOT_ORDER = ['2910', '4414', '8793', '971', '1678', '1690', '4946', '3928', '341', '4930'];

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
