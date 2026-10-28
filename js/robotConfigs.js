// Robot definitions, based on each team's published 2026 information.
//  - 2910 Jack in the Bot "Re•Blitz" (tech binder): 27.5x27in swerve (MK5n R1), front slap-down
//    over-bumper intake (4 FUEL wide), one-piece hopper (holds 50+, fits under the TRENCH),
//    4-FUEL-wide drum shooter with adjustable hood fixed to the chassis, firing out the back (away
//    from the intake) — the whole robot turns to aim; 23-25 BPS with the intakes saturated, 30 in
//    short volleys (the team, Chief Delphi reveal thread). No climber ("Won't: Climb").
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
//  - 1706 Ratchet Rockers "Mirage" (public Onshape CAD + CAD release thread): 25.2x29.5in swerve,
//    a deep hopper over two spindexers side by side, each feeding its own turret; the intake folds
//    up inside the hopper and pushes it out. Climber removed.
//  Nets: 971, 1690, 3928 and 1706 carry netting over the open tops of their hoppers (the physics lets the
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

// where the swerve modules sit (x, z): 3in in from the corners, or inside a round frame; at a cut
// corner (frame.chamfer) a module made to fit the diagonal (4414's "2 custom swerve modules"), its
// wheel just inside it
export function modulePoints(cfg) {
  const f = cfg.frame, inset = 0.075;
  const rw = (cfg.drive?.wheel ?? 4 * IN) / 2, cut = f.chamfer ? Math.max(inset, f.chamfer / 2 + (rw + 0.006) * Math.SQRT1_2) : inset;
  if (f.round) {
    const { r, cx } = f.round, d = (r - 0.1) * Math.SQRT1_2;
    return [[cx + d, d], [cx + d, -d], [cx - d, d], [cx - d, -d]];
  }
  return [[1, 1], [1, -1], [-1, 1], [-1, -1]].map(([sx, sz]) => { const d = sx < 0 ? cut : inset; return [sx * (f.length / 2 - d), sz * (f.width / 2 - d)]; });
}

// whether a robot fits under the TRENCH arm (4946 doesn't: it goes over the BUMPS)
export const fitsTrench = (cfg) => cfg.height <= TRENCH.clearHeight - 0.005;

// Top of an intake that folds up over the robot to stow (intake.fold, 8793's): its side outline
// swung up from deployed by (1 - deploy) x stowDeg about the pivot. 0 for other intakes.
// Where FUEL leaves a fixed shooter (robot frame, m) with the hood set for a shot thetaDeg up.
// A hood that swings round the drum (shooter.hood, 2910's) lets it go halfway across the gap
// between the drum and the hood, square to the shot; otherwise it's shooter.exit.
export function fixedExit(sh, thetaDeg) {
  if (!sh.hood) return sh.exit;
  const th = (thetaDeg * Math.PI) / 180, s = sh.facing === 'back' ? 1 : -1;
  return { x: sh.hood.x + s * sh.hood.r * Math.sin(th), y: sh.hood.y + sh.hood.r * Math.cos(th) };
}

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
    // built of its physical mechanisms (js/mech/r2910.js, phys2910.js): its FUEL go through its
    // CAD's ball path as bodies, moved by its rollers, under its own code's controls
    physical: '2910',
    blurb: 'Huge hopper and a 4-wide drum shooter fixed to the chassis — the whole robot rotates to aim. Unloads about 25 FUEL per second (30 in short volleys).',
    frame: { length: 27.0 * IN, width: 27.5 * IN },
    height: 21.5 * IN,
    mass: 64, // robot (up to 115 lb) + bumpers + battery
    // SDS MK5n R1 (7.03:1, 4in wheels), Kraken X60s (see drivetrain.js). Current limits (their code,
    // config/ReBlitz.java, Robot.configureTeleopCurrentLimits): 40 A supply; the stator limit is the
    // modules' slip current, 100 A, raised to 120 A when teleop starts
    drive: { ratio: 7.03, stator: 100, supply: 40, teleop: { stator: 120 } },
    // slap-down intake from 2910's CAD: its 2in roller reaches ~7.8in past the bumper
    // rate: fast enough that driving through FUEL is the only limit; pull: roller surface speed
    // arm: the CAD intake's pivot and its 2in roller's reach and angle, stowed (as exported) and
    // down on the bumper, its hardstop: their code's 114.3 deg of travel, which lands it on the
    // CAD's deployed pose (js/mech/r2910.js INTAKE). While shooting it retracts slowly and pushes the load back into the
    // indexer (compacts), stalling against the FUEL until there's room.
    intake: {
      width: 25.5 * IN, reach: 7.8 * IN, rate: 200, pull: 5, deployTime: 0.35, retractTime: 1.6, side: 'front', latched: false,
      compacts: true, arm: { x: 0.273, y: 0.17, len: 0.338, stowDeg: 112.5, deployDeg: -1.8 },
      // retracting over the load it crams it into the indexer, far harder than its roller pushes
      // FUEL in (hopper.js intakePush): it stalls once the load pushes back this hard (N, estimate)
      compactPush: 180,
    },
    // one-piece hopper (their CAD): its panels sit over the shooter at the start and slide out
    // 0.30 m along slotted rails with the first intake (the CAD's two poses), then stay out for
    // the match. capacity / retracted are the team's stated numbers; how much it really holds is
    // measured from the hopper (bay) below (hopper.js measureCapacity)
    storage: { capacity: 60, retracted: 40, extLen: 0.30, extend: 'latched' },
    // the hopper as the FUEL sees it (robot frame: x forward, y up, z to the side; meters).
    // Powered floor rollers slope down to the back, where compliant indexer wheels lift FUEL
    // into the drum.
    bay: {
      // walls 0.69 m apart, its front panel 0.34 m out when in and 0.61 m out; the top panel's
      // front edge bevels down to 0.41 m over the last 0.16 m (their CAD)
      x0: -0.143, x1: 0.335, hw: 0.345, top: 0.54, bevel: { dx: 0.16, y: 0.41 },
      floor: { a: 0.1, b: 0.266, lo: 0.09, hi: 0.17 },
      // The shot rate is these rollers' (shooter.feedPaced): the floor carries FUEL back at
      // driveSpeed, and each lane's indexer wheels take the next one that's within reach as soon
      // as the last has cleared them, lifting it at feed.speed. Tuned to what the team reports: a
      // full hopper dumps at 23-25 a second (the sim: ~24), short volleys at about 30 (~34 in the
      // first second, from the FUEL already sitting at the indexer).
      drive: 'floor', driveSpeed: 3.0,
      // up the roller ramp at the back of the hopper, over the indexer wheels, into the pinch
      // between the drum and the lower hood roller (where their CAD has a FUEL waiting)
      feed: { x: -0.09, via: [[-0.1, 0.25], [-0.12, 0.38], [-0.166, 0.418]], reach: 0.12, speed: 1.65 },
    },
    shooter: {
      type: 'fixed',
      facing: 'back', // the drum at the back fires away from the intake
      lanes: [-0.19, -0.063, 0.063, 0.19],
      // the hood (their CAD's 32-17 Hood Assembly): two rollers stacked in front of the drum, on
      // arms that pivot on the drum shaft. FUEL goes up between the drum and the rollers and
      // leaves past the top one, halfway across the gap: r from the drum axis (x, y), square to
      // the shot. Tilting the hood back over the drum flattens the shot and carries that point up
      // and back with it. cadDeg: the shot the CAD has it set for. Out of a shot it folds back
      // down (stowDeg), where its rollers are level with the top of the robot. See fixedExit.
      hood: { x: -0.283, y: 0.47, r: 0.1165, cadDeg: 79, stowDeg: 74 },
      // 23-25 typical with the intakes saturated (the team). Not a limit here: the rollers set
      // the rate (bay.feed); this is what they measure to, for the robot card and the AI
      bps: 25,
      feedPaced: true,
      hoodMin: 42, hoodMax: 74,
      speedMax: 17,
      spinTau: 0.33,
      shotDrop: 0.002,
      speedSigma: 0.016, angleSigma: 0.8, yawSigma: 0.8,
    },
    climber: null,
    stats: { 'Shot rate': '25 BPS', Aiming: 'Chassis', 'Top speed': '14.1 ft/s', Trench: 'Yes' },
    colors: { frame: 0xb9bec5, accent: 0x5c6168, trim: 0xc6cbd1 }, // raw aluminum, grey plates (their CAD)
  },
  4414: {
    key: '4414',
    team: 4414,
    teamName: 'HighTide',
    robotName: 'RIPCURRENT',
    archetype: 'Dye Rotor',
    blurb: 'Extending hopper holds about 88 FUEL under a stretchy net. A Dye Rotor single-streams FUEL into a turret shooter (about 14 FUEL per second) with precomputed shoot-on-the-move.',
    // its intake, Dye Rotor and shooter as physical mechanisms (js/mech/r4414.js, phys4414.js)
    physical: '4414',
    // back corners cut off, as little as keeps the frame perimeter within 110in ("chamfered corners
    // maximize hopper area and dye rotor diameter while staying within 110", the binder): 114in
    // round the 25x32in rectangle, each 45-degree cut saving (2 - sqrt2) x its legs, so 3.41in legs
    frame: { length: 25.0 * IN, width: 32.0 * IN, chamfer: (4 / (2 * (2 - Math.SQRT2))) * IN },
    height: 21.75 * IN,
    mass: 64, // robot (up to 115 lb) + bumpers + battery
    // 7.67:1 on 4in wheels (tech binder), Kraken X60s; custom modules made like WCP's X2i, inverted
    // (the motors hang under the module's plate beside the wheel, nothing up into the hopper), the
    // back two cut to fit the chamfered corners (the binder's "2 custom swerve modules")
    drive: { ratio: 7.67, inverted: true },
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
    // (the walls are the bumper backing: their faces 1in out past the frame perimeter, js/mech/r4414.js BAY)
    bay: {
      x0: -13.5 * IN, x1: 0.317, hw: 17 * IN, top: 0.53, chamfer: (4 / (2 * (2 - Math.SQRT2)) + 2 - Math.SQRT2) * IN, wallTop: 0.53,
      dome: { h: 0.27, x0: -0.165, cx: -0.02, cz: 0, rHole: 0.2 },
      floor: { a: 0.105, b: 0, lo: 0.105, hi: 0.105 },
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
      bps: 14, // gap-free 12-14 BPS (their tech binder; ~20 in theory from the rotor)
      hoodMin: 44, hoodMax: 82,
      speedMax: 16,
      spinTau: 0.25,
      shotDrop: 0.005,
      speedSigma: 0.013, angleSigma: 0.6, yawSigma: 0.6,
    },
    climber: null,
    stats: { 'Shot rate': '14 BPS', Aiming: 'Turret', 'Top speed': '13.1 ft/s', Trench: 'Yes' },
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
    // WCP Swerve X2t, geared for their stated 14.8 ft/s free speed (6.84:1 on 4in wheels); their
    // code's TunerConstants: 40 A supply, the stator limit the 120 A slip current
    drive: { ratio: 6.84, stator: 120, supply: 40 },
    // Intake V3 (their CAD): three silicone rollers on an arm that swings down from a pivot over
    // the front of the frame. Stowed, it folds back up inside the frame perimeter and stands
    // 0.69 m tall, too tall for the TRENCH: it has to be down to drive under. fold.hull is the
    // arm's side outline around the pivot (m, x forward, y up) as exported, deployed.
    // (deployTime: what their pivot's Motion Magic profile takes, 1 rotation/s and 2 rotations/s²
    // over the fold; js/mech/phys8793.js runs it, this is for the AI's planning)
    intake: {
      width: 26 * IN, reach: 10 * IN, rate: 14, deployTime: 0.9, side: 'front', latched: false,
      fold: {
        pivot: [0.292, 0.336], stowDeg: 145,
        hull: [[-0.026, -0.018], [0.161, -0.305], [0.176, -0.306], [0.298, -0.196], [0.328, -0.105], [0.191, 0.009], [0.005, 0.032], [-0.025, 0.02]],
      },
    },
    // the intake's rollers hold FUEL too: with the arm down, the ball path runs out over it
    // (it stays down while it's holding FUEL the path can't take)
    storage: { capacity: 12, extLen: 0.2, extend: 'intake' },
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
    // built of its physical mechanisms (js/mech/r8793.js, phys8793.js): its FUEL go through its
    // real rollers, wheels and CAD surfaces, and out of its shooter, instead of the bay above
    physical: '8793',
    shooter: {
      type: 'turret',
      // 200T turntable (their CAD); its travel is their soft limits, -200 to 270 deg
      // (TURRET_ROTATOR_MIN/MAX_ANGLE, counterclockwise from straight ahead): 35 +- 235, endpoint
      // to endpoint (no continuous rotation: past a limit it swings back the long way round). It
      // starts straight ahead.
      turretPos: { x: -0.127, z: 0.0, center: 35, start: 0 },
      // where FUEL leaves its wheels (tools/mechlab.mjs): 0.09 m out from the turret's axis, 0.57 up
      exitRadius: 0.09,
      exitY: 0.57,
      turretRange: 235, turretRate: 600,
      // what the AI plans a dump with: what its loads empty at in matches (3-6 FUEL/s: 0.1 s apart
      // when they flow, its ideal 8/s, less the unjams and the gaps waiting on the next FUEL).
      // The mechanisms set the real rate.
      bps: 5,
      hoodMin: 52, hoodMax: 72, // their rack's 18..38 deg (elevation = 90 - rack)
      speedMax: 9.2, // the fastest its shots were measured leaving (SHOT_TABLE, js/mech/r8793.js)
      spinTau: 0.28,
      shotDrop: 0.007,
      speedSigma: 0.017, angleSigma: 0.9, yawSigma: 0.9,
    },
    climber: null,
    stats: { 'Shot rate': '8 BPS (ideal)', Aiming: 'Turret', 'Top speed': '14.8 ft/s', Trench: 'Intake down' },
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
    // Folding back in, the beater sweeps the FUEL in the extension back into the hopper
    // (compacts) and stalls against the load when it can't squeeze more (compactPush, an
    // estimate). arm: the 4-bar's pivot and the beater's reach and angle, folded up (as exported)
    // and down (their code's pivot and 130deg travel); compactMin: the hopper's fixed front (past that the
    // beater is up over the load).
    intake: {
      width: 29 * IN, reach: 0.18, rate: 200, pull: 5, deployTime: 0.35, retractTime: 0.8, side: 'front', latched: false,
      compacts: true, compactPush: 180, compactMin: 0.34,
      arm: { x: 0.217, y: 0.143, len: 0.3065, stowDeg: 118.2, deployDeg: -9.4 },
    },
    storage: { extLen: 0.28, extend: 'latched' },
    // the hopper as the FUEL sees it (their CAD): polycarbonate walls to 0.54 m, 0.75 m apart
    // (the frame's full width); the roller floor is level at the back wall, under which the
    // separator and kicker take FUEL to the two ramps, and climbs over the front swerve modules.
    // The net over the open top bulges up with the load.
    bay: {
      x0: 0.0, x1: 0.34, hw: 0.37, top: 0.54,
      dome: { h: 0.15, x0: 0.0, cx: 0, cz: 0, rHole: -1 },
      floor: { pts: [[-0.1, 0.165], [0.09, 0.175], [0.14, 0.19], [0.19, 0.23], [0.24, 0.255], [1, 0.255]] },
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
      bps: 14, // both turrets together, 7 each (an estimate: the team says its indexing isn't fast; no published rate)
      hoodMin: 35, hoodMax: 85, // 50deg of hood travel on the lead screw
      speedMax: 17,
      spinTau: 0.28,
      shotDrop: 0.004,
      speedSigma: 0.014, angleSigma: 0.7, yawSigma: 0.7,
    },
    climber: { maxLevel: 1, times: [0, 2.0] }, // one-stage telescoping arm
    stats: { 'Shot rate': '2 x 7 BPS (est.)', Aiming: 'Twin turrets', 'Top speed': '14.4 ft/s', Trench: 'Yes', Climb: 'Level 1' },
    colors: { frame: 0xb4b9c1, accent: 0xc62828, trim: 0x2a2c31 }, // raw aluminum, red, black hoods
  },
  1678: {
    key: '1678',
    team: 1678,
    teamName: 'Citrus Circuits',
    robotName: 'Limestone',
    archetype: 'Drum + lift',
    blurb: 'Milstein Division champion. A full-width drum with three hood rollers fires out the back, fed by a roller floor and ball tunnel. The hopper grows out with the intake and up with the climber, with a net stretched diagonally over the extension. Climbs Level 1.',
    // built of its physical mechanisms (js/mech/r1678.js, phys1678.js): its FUEL go through its
    // CAD's ball path, moved by its rollers under its code's controls
    physical: '1678',
    frame: { length: 27.0 * IN, width: 27.0 * IN },
    height: 21.6 * IN,
    mass: 65, // robot (up to 115 lb) + bumpers + battery
    // SDS MK5n R1 (7.03:1, 4in wheels). Current limits (their code): TunerConstants' 80 A stator
    // (and slip current) and 40 A supply, then at teleopInit DriveConstants.getTeleConfig()'s 60 A
    // stator and 30 A supply
    drive: { ratio: 7.03, stator: 80, supply: 40, teleop: { stator: 60, supply: 30 } },
    // full-width slapdown: 2in silicone roller + 1.25in kicker bar, pivot at the front
    // a 2in silicone-covered carbon fiber roller (their reveal), set to squeeze FUEL 3/4in like
    // the compliant-wheel intakes (hopper.js intakePush)
    // Folding back in, it sweeps the FUEL in the extension back into the hopper (compacts), and
    // stalls against the load when it can't squeeze more (compactPush, an estimate). arm: its
    // pivot and the 2in roller's reach and angle, stowed (folded up inside the hopper, as exported)
    // and down (their code's pivot and 130deg travel); compactMin: it only pushes FUEL back as far as the front
    // of the frame (past that it's up over the load).
    intake: {
      width: 25 * IN, reach: 0.29, rate: 200, pull: 5, deployTime: 0.3, retractTime: 0.35, side: 'front', latched: false, squeeze: 0.75,
      compacts: true, compactPush: 180, compactMin: 0.33,
      arm: { x: 0.309, y: 0.169, len: 0.314, stowDeg: 132.3, deployDeg: 2.3 }, // 130deg (their code), crash bar square
      front: 0.639, // the crash bar's front, down (their CAD): where it meets a wall
    },
    // the horizontal extension (polycarbonate sides, a corrugated front) slides straight out: a pin
    // on the intake plate rides a slanted vertical slot in the side plates (their robot page and
    // CAD), so the intake swinging down pushes the extension forward: down 130deg, 0.303 m out
    // (their code: HOPPER_MAX_EXTENSION_DISTANCE), its front panel flush with the crash bar, and
    // folding the intake pulls it back in (their contractHopper; js/mech/r1678.js SLOT).
    storage: { extLen: 0.303, extend: 'latched' },
    // their CAD: roller floor (dead-axle rollers, then flex wheels) sloping down to the ball
    // tunnel at the back; the lid (corrugated plastic on the climber tubes) sits at 0.545 m and
    // lifts lift.h with the climber (it comes back down under the TRENCH); over the extension the
    // ceiling is the diagonal net
    bay: {
      x0: -0.09, x1: 0.33, hw: 0.34, top: 0.545, // walls at +-0.35 m, under the lid (their CAD)
      // (their climber's kMaxPosition, where expandHopper takes it: 8.5in on the robot they ran,
      // isEpsilon; ClimberConstants)
      lift: { h: 8.5 * IN },
      // the net from the lid's front edge down to the front of the extension
      slope: { x: 0.335, y: 0.5 },
      // and it stretches: the load bulges it up and out over the extension
      dome: { h: 0.25, x0: 0.335, cx: 0, cz: 0, rHole: -1 },
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
      bps: 13, // counted over a cycle in their reveal thread (the wheel recovers fast enough for ~20, the team says)
      hoodMin: 40, hoodMax: 80,
      speedMax: 16,
      spinTau: 0.3, // no added flywheel mass
      shotDrop: 0.006,
      speedSigma: 0.012, angleSigma: 0.55, yawSigma: 0.55,
    },
    climber: null,
    stats: { 'Shot rate': '13 BPS', Aiming: 'Turret', 'Top speed': '13.1 ft/s', Trench: 'Yes' },
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
      // the Dye Rotor (engineering report, CAD renders): a curved arm turning round the column from
      // its foot out to the tray wall (its tip leads by sweep), a scoop at its root steering FUEL
      // into a mouth in the column's foot that turns with it, and a powered vertical roller by the
      // mouth that drives FUEL in (pull, m/s in along the arm); the spoked platter turns with it
      drive: 'rotor', rotor: { x: 0.02, z: 0, y: 0.2, r: 0.38, grip: 30, drag: 6, finR0: 0.16, sweep: 0.6, pull: 1, spin: 9, idle: -0.5 },
      // the tray's ring of interlocking printed walls round the platter: its inner face slopes
      // steeply down from the hopper walls onto the platter's edge (their CAD render)
      funnel: { slope: 1.8, cap: 0.13 },
      // (at the arm's rest angle; it all turns with the arm) in the mouth, then up the column
      feed: { x: 0.217, z: -0.14, rotates: true, via: [[0.142, 0.3, -0.087], [0.061, 0.5, -0.029], [0.02, 0.66, 0]] },
    },
    shooter: {
      type: 'turret',
      turretPos: { x: 0.02, z: 0 }, // on the center of rotation
      exitRadius: 0.1,
      exitY: 0.72,
      turretRange: 200, turretRate: 720,
      bps: 14, // ~14 sustained, single stream (counted in their reveal thread)
      hoodMin: 40, hoodMax: 80, // worm-driven hood
      speedMax: 17,
      spinTau: 0.28,
      shotDrop: 0.004,
      speedSigma: 0.013, angleSigma: 0.6, yawSigma: 0.6,
    },
    climber: null,
    stats: { 'Shot rate': '14 BPS', Aiming: 'Turret', 'Top speed': '12.8 ft/s', Trench: 'No (BUMPS)' },
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
    // a hopper for about 80 FUEL (their CAD release notes)
    storage: { capacity: 80, extLen: 0.3, extend: 'latched' },
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
      drive: 'rotor', rotor: { x: 0.1125, z: -0.0175, y: 0.105, r: 0.2, grip: 160, drag: 35, finR0: 0.06, fins: 5, spin: -18, idle: -0.5 },
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
    // built of its physical mechanisms (js/mech/r341.js, phys341.js): its FUEL go through its
    // CAD's ball path, moved by its rollers under its code's controls
    physical: '341',
    blurb: 'Almost no hopper: a triple-roller slapdown lifts FUEL onto a single-layer tray under the turret table, where three spinning 6in omni wheels serialize it into the uptake to the turret. Shoots as fast as it intakes. Climbs Level 1.',
    // their CAD: bumpers +-0.428 m, 27x27in frame
    frame: { length: 27 * IN, width: 27 * IN },
    height: 21.75 * IN, // the shooter on the table (their CAD)
    mass: 62, // robot (up to 115 lb) + bumpers + battery
    // WCP Swerve X2i (their CAD); their code (BetaBotTunerConstants, the constants every mode uses):
    // 7.67:1 on a 1.8in wheel radius, the stator limit the 35 A slip current (ModuleIOTalonFX;
    // 55 A only while the driver holds defense), the supply limit Phoenix's default 70 A
    drive: { ratio: 7.67, wheel: 3.6 * IN, stator: 35, supply: 70 },
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
    // the slapdown holds FUEL too: with it down, the tray runs out over its rollers (it stays down
    // while it's holding FUEL the tray can't take). 8 in all (their spec sheet)
    storage: { capacity: 8, extLen: 0.22, extend: 'intake' },
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
    // Folding up, it sweeps the FUEL over the intake back into the hopper (compacts) and stalls
    // against the load when it can't squeeze more (compactPush, an estimate); arm: its roller's
    // reach and angle about the pivot, down and folded up.
    intake: {
      width: 28 * IN, reach: 0.2, rate: 200, pull: 5, deployTime: 0.35, retractTime: 0.8, side: 'front', latched: false,
      compacts: true, compactPush: 180, compactMin: 0.33,
      arm: { x: 0.2355, y: 0.2005, len: 0.401, stowDeg: 67.3, deployDeg: -22.7 },
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
      // intake's side plates keeps the load in over the open front; it stretches, so the load
      // bulges it up over the slope
      slope: { x: 0.12, y: 0.37 },
      dome: { h: 0.3, x0: 0.12, cx: 0, cz: 0, rHole: -1 },
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
  1706: {
    key: '1706',
    team: 1706,
    teamName: 'Ratchet Rockers',
    robotName: 'Mirage',
    archetype: 'Twin spindexers',
    blurb: 'Seeing double: two spindexers side by side under a deep hopper, each feeding its own turret. The intake folds up inside the hopper and pushes it out as it deploys, and a net over the top stretches as the load grows. 45-2 in 2026.',
    // built of its physical mechanisms (js/mech/r1706.js, phys1706.js): its FUEL go through its
    // CAD's ball path, moved by its rollers, spindexers, kickers and shooters
    physical: '1706',
    // their CAD: frame 0.64 x 0.75 m (25.2 x 29.5in), bumpers 0.797 x 0.914 m. The team calls the
    // turret end the front; the sim's front is the intake end.
    frame: { length: 25.2 * IN, width: 29.5 * IN },
    height: 21.6 * IN, // top of the hopper walls (their CAD)
    mass: 64, // robot (up to 115 lb) + bumpers + battery
    // Thrifty Swerve; the ratio isn't in the CAD, 6.75:1 assumed
    drive: { ratio: 6.75 },
    // rollers on an arm that folds up inside the hopper (as exported) and swings out over the
    // bumper about a pivot low in the frame; deploying it pushes the hopper box out
    // Folding back in, the rollers sweep the FUEL over the bumper back into the hopper (compacts)
    // and stall against the load when it can't squeeze more (compactPush, an estimate). arm: the
    // pivot (the hex shaft through the arm plates) and the bottom roller's reach and angle, folded
    // up (as exported) and down (their CAD: 104deg, the arm 2 cm off the carpet, its front 0.23 m
    // past the bumper).
    intake: {
      width: 26 * IN, reach: 0.2, rate: 200, pull: 5, deployTime: 0.35, retractTime: 0.8, side: 'front', latched: false,
      compacts: true, compactPush: 180, compactMin: 0.318,
      arm: { x: 0.262, y: 0.168, len: 0.36, stowDeg: 86.5, deployDeg: -17.5 },
    },
    // the arm swinging out pushes the hopper box's front wall out ahead of it (0.34 m in their CAD,
    // and the box's side plates have slots long enough); the 12in extension limit stops it at 0.3 m
    storage: { extLen: 0.3, extend: 'intake' },
    // their CAD: walls to 0.548 m round two spindexers side by side (flat discs, r 0.138 m, 0.16 m
    // up) with a hub over each; the floor slopes down into them from the corners. Each spindexer
    // carries FUEL round to its back edge, where a ramp takes it up into the turret behind it.
    bay: {
      x0: -0.14, x1: 0.318, hw: 0.37, top: 0.548,
      // a net over the open top (their CAD doesn't model it; the robot has one: js/mech/r1706.js
      // NET), slack cord tied to the walls' tops, the back panel's top tube and the hopper box's top
      // edges, riding out with the box; the load lifts it until it's taut (the turrets are behind
      // the back panel, no hole)
      dome: { h: 0.15, x0: -0.01, cx: 0, cz: 0, rHole: -1 },
      floor: { a: 0.16, b: 0, lo: 0.16, hi: 0.16 },
      funnel: { slope: 0.45, cap: 0.2 },
      obstacles: [
        { x: 0.0865, z: -0.1735, r: 0.05, y0: 0.2, y1: 0.32 },
        { x: 0.0865, z: 0.1735, r: 0.05, y0: 0.2, y1: 0.32 },
        // above the spindexers the back wall is a polycarbonate panel in front of the turrets
        // (their CAD): at x -0.025 from 0.33 m up, slanting back to -0.11 m in the middle
        { box: [-0.5, -0.025, 0.33, 1.5, -0.5, -0.07] },
        { box: [-0.5, -0.025, 0.33, 1.5, 0.07, 0.5] },
        { box: [-0.5, -0.07, 0.43, 1.5, -0.07, 0.07] },
        { box: [-0.5, -0.11, 0.33, 0.43, -0.07, 0.07] },
      ],
      drive: 'rotor',
      rotors: [
        { x: 0.0885, z: -0.1775, y: 0.16, r: 0.138, grip: 70, drag: 60, finR0: 0, spin: 20, idle: -0.5, dir: 1 },
        { x: 0.0885, z: 0.1775, y: 0.16, r: 0.138, grip: 70, drag: 60, finR0: 0, spin: 20, idle: -0.5, dir: -1 },
      ],
      feed: {
        x: -0.03, zs: [-0.19, 0.19], reach: 0.2,
        vias: [
          [[-0.09, 0.28, -0.2], [-0.15, 0.42, -0.22]],
          [[-0.09, 0.28, 0.2], [-0.15, 0.42, 0.22]],
        ],
      },
    },
    shooter: {
      type: 'turret',
      // on the turret rings behind the hopper (their CAD); each one's travel is centered pointing out
      // to its own side, so they can't shoot across each other
      turrets: [{ x: -0.1651, z: -0.222, center: 90 }, { x: -0.1651, z: 0.222, center: -90 }],
      turretPos: { x: -0.1651, z: -0.222 },
      // where FUEL leave over the 3in accelerator wheels, and how steep and fast: the hood swings
      // 45 deg about the accelerators' axle (their CAD's mates), FUEL leaving at 84 to 38 deg, at up
      // to ~8.6 m/s (tools/mechlab1706.mjs shooter)
      exitRadius: 0.06,
      exitY: 0.62,
      turretRange: 170, turretRate: 600,
      bps: 12.5, // both turrets together: "about 12-13 bps" (the team, their CAD release thread); its physical ball path sets the rate
      hoodMin: 38, hoodMax: 84,
      speedMax: 8.5,
      spinTau: 0.3,
      shotDrop: 0.004,
      speedSigma: 0.013, angleSigma: 0.6, yawSigma: 0.6,
    },
    climber: null, // taken off (remnants in the CAD)
    stats: { 'Shot rate': '12-13 BPS (2 turrets)', Aiming: 'Twin turrets', 'Top speed': '', Trench: 'Yes' },
    colors: { frame: 0xc3c8ce, accent: 0x2a4fb5, trim: 0x2a2d33 },
  },
  581: {
    key: '581',
    team: 581,
    teamName: 'Blazing Bulldogs',
    robotName: 'Rubble',
    archetype: 'Dumper',
    blurb: 'Hopper Division finalist. Rebuilt mid-season from a turret into a dumper: a linear intake slides out on titanium racks and carries the front of the hopper with it, a roller floor that pivots up to compact the load (ramps on the intake lift it as it comes in) feeds a drum at the back, and an adjustable hood throws FUEL out over the back. Fits under the TRENCH.',
    // built of its physical mechanisms (js/mech/r581.js, phys581.js): its FUEL go through its
    // CAD's ball path, moved by its rollers under its code's controls
    physical: '581',
    // their CAD ("2026 Dumper Champs Bot") and release post: 27 x 28.5in frame perimeter
    frame: { length: 27 * IN, width: 28.5 * IN },
    height: 21.7 * IN, // top of the hopper walls and the shooter tower (their CAD)
    mass: 64, // robot (up to 115 lb) + bumpers + battery
    // WCP X2i / X2c modules, X1 gearing with 10T pinions: 7.67:1 (their code), 4in wheels. Current
    // limits (their code, CompTunerConstants): 40 A supply, the stator limit the 120 A slip current.
    // (Their PowerManager's swerve supply limits reach only the practice bot: in
    // Swerve.applyCurrentLimits the comp bot's branch of the ?: re-applies the 40 A.)
    drive: { ratio: 7.674, stator: 120, supply: 40 },
    // linear deploy on titanium racks (their CAD's slider: 0.306 m, tilted 5.5deg up toward the
    // robot; their code runs it to 11.8in), a 3in main roller and a 2in top roller (21.8in of grip
    // tape), the 3in's front 0.28 m past the frame; the intake's front plate is the front wall of
    // the hopper's extension, so the hopper comes out with it
    intake: { width: 22 * IN, reach: 0.28, rate: 200, pull: 5, deployTime: 0.3, retractTime: 0.5, side: 'front', latched: false, compacts: true },
    storage: { extLen: 0.305, extend: 'intake' },
    // their CAD: a roller floor sloping up from the shooter tower to the front (it pivots up at the
    // tower to compact the load), smoked polycarbonate walls on the bumpers, a net over the top from
    // the walls' top tubes down to the intake's front plate; FUEL goes up the tower's rollers to the
    // drum at the back
    bay: {
      x0: -0.09, x1: 0.33, hw: 0.33, top: 0.55,
      slope: { x: 0.33, y: 0.49 },
      dome: { h: 0.15, x0: -0.09, cx: 0, cz: 0, rHole: -1 },
      floor: { a: 0.148, b: 0.25, lo: 0.125, hi: 0.205 },
      drive: 'floor', driveSpeed: 2.4,
      feed: { x: -0.09, via: [[-0.14, 0.2], [-0.2, 0.33], [-0.25, 0.47]] },
    },
    shooter: {
      type: 'fixed',
      facing: 'back', // their code: the shooter faces 180deg from the robot's front
      lanes: [-0.2, -0.067, 0.067, 0.2],
      // where FUEL leave the drum and the hood's two 2in rollers, over the 4in drum (their CAD;
      // tools/mechlab581.mjs shooter: -0.21..-0.27, 0.63..0.68 over their hood angles)
      exit: { x: -0.24, y: 0.65 },
      // their alpha bot shot 15/s; the Champs robot scored 252 FUEL a match in the Hopper Division
      // (Statbotics), up from 145 at their first event, against 1678's 283 and 2910's 304 at Champs:
      // on the same scale, about 23/s (its physical ball path dumps a full hopper at about 24/s:
      // tools/mechlab581.mjs dump)
      bps: 23,
      // hood: their 13-45 (ShooterHoodConfig), turning the hood about the drum's axle, opens up for
      // longer shots: FUEL leave at 75 to 43 deg (the CAD's geometry; tools/mechlab581.mjs)
      hoodMin: 43, hoodMax: 76,
      // their table tops out at 2500 RPM (feeding), where FUEL leave at ~8.3 m/s
      speedMax: 9,
      spinTau: 0.25, // four X60s 1:1 on the drum, and a 4in steel inertia wheel geared up 3:1
      shotDrop: 0.002,
      speedSigma: 0.013, angleSigma: 0.65, yawSigma: 0.65,
    },
    climber: null,
    stats: { 'Shot rate': '23 BPS (Champs)', Aiming: 'Chassis', 'Top speed': '', Trench: 'Yes' },
    colors: { frame: 0x1d1f24, accent: 0xc62828, trim: 0x2a2c31 },
  },
  1114: {
    key: '1114',
    team: 1114,
    teamName: 'Simbotics',
    robotName: 'Simbot Tim',
    archetype: 'Box dumper',
    // built of its physical mechanisms (js/mech/r1114.js, phys1114.js): its FUEL go through its
    // CAD's ball path, moved by its rollers, elevator and flywheel, and its code runs them
    physical: '1114',
    blurb: 'Archimedes Division finalist, 10-0 in qualifications, and Ontario champion. A tall clear hopper box slides out over the intake on racks, under a red "magic blanket" top cover (after 1124\'s 2008 claw). A roller floor feeds a ball elevator up the tower at the back to a hooded flywheel that fires forward over the hopper. Too tall for the TRENCH, it goes over the BUMPS.',
    // their CAD ("S26-A000"): 24.5 x 30in frame, 30in across the front
    frame: { length: 24.5 * IN, width: 30 * IN },
    height: 27.6 * IN, // top of the hopper box (their CAD)
    mass: 64, // robot (up to 115 lb) + bumpers + battery (they took the climber off for weight)
    // WCP X2 modules, X1 gearing with 10T pinions: 7.67:1 (their code), 4in wheels
    drive: { ratio: 7.674 },
    // the intake rollers are on the front of the hopper box, which slides out 0.298 m on racks
    // (their CAD's mate, tilted 8.7deg up toward the robot); a kicker bar hangs in front of the
    // rollers on arms that swing up 90deg
    intake: { width: 25 * IN, reach: 0.2, rate: 200, pull: 5, deployTime: 0.35, retractTime: 0.5, side: 'front', latched: false },
    storage: { extLen: 0.2945, extend: 'intake' },
    // their CAD: the box is the hopper (clear walls to 0.70 m, the blanket over the top); a roller
    // floor slopes down to the foot of the tower, where rollers and an "active wall" take FUEL
    // into the ball elevator
    bay: {
      x0: 0.0, x1: 0.311, hw: 0.36, top: 0.7,
      dome: { h: 0.1, x0: 0.0, cx: 0, cz: 0, rHole: -1 }, // the blanket sags over the load and rides up on it
      floor: { a: 0.077, b: 0.33, lo: 0.06, hi: 0.19 },
      drive: 'floor', driveSpeed: 2.4,
      feed: { x: 0.0, via: [[-0.05, 0.15], [-0.09, 0.3], [-0.09, 0.48], [-0.04, 0.6]] },
    },
    shooter: {
      type: 'fixed',
      facing: 'front', // their code turns the intake end at the HUB: over the top of the flywheel, forward
      lanes: [-0.16, 0, 0.16],
      // the hood (their CAD): three 1.5in rollers on an arc 0.1893 m round the 3in flywheel's
      // axis, on arms that pivot on its shaft, behind and over it. FUEL comes up the elevator
      // behind the flywheel, wraps up over it under the rollers and leaves past the last one,
      // halfway across the gap (0.104 m from the axis), square to the shot; swinging the hood up
      // and back steepens the shot and carries that point down and back with it. The CAD has it
      // set for 45.6 degrees, its flat end (their code: 42-74), where it rests out of a shot.
      // See fixedExit.
      hood: { x: -0.0176, y: 0.5715, r: 0.1039, cadDeg: 45.6, stowDeg: 45.6 },
      // not published: they scored 288-320 FUEL a match at the Ontario championship and Champs
      // (Statbotics), against 1678's 283 and 2910's 304 at Champs: on the same scale, about 26/s
      bps: 26,
      hoodMin: 42, hoodMax: 74, // their code's hood limits (30deg of travel in their CAD)
      speedMax: 10.5, // (its physical flywheel: FUEL leave at no more than ~10.5 m/s, tools/mechlab1114.mjs shooter)
      spinTau: 0.3,
      shotDrop: 0.003,
      speedSigma: 0.012, angleSigma: 0.55, yawSigma: 0.55, // 95-98% after the grip tape (their thread)
    },
    climber: null, // taken off for weight
    stats: { 'Shot rate': '~14 BPS a full box, ~21 best second (physical)', Aiming: 'Chassis', 'Top speed': '', Trench: 'No (BUMPS)' },
    colors: { frame: 0xb4b9c1, accent: 0xc0272d, trim: 0x2a2c31 },
  },
  604: {
    key: '604',
    team: 604,
    teamName: 'Quixilver',
    robotName: 'Toploader',
    archetype: 'Dye Rotor',
    blurb: 'Pinnacles winner, Half Moon Bay finalist and 4th seed in the Newton Division (the Autonomous Award). One driver, fully automated scoring: a turret that shoots on the move from anywhere in its zone and feeds from outside it, fed single file by its "Dynamic Point of Capture" serializer, a dye rotor whose hook sweeps FUEL round a plywood bucket to omni wheels that push them up a ramp into a corrugated plastic tube to the turret. A full-width intake on independent polycarbonate arms pushes the sliding hopper out. Goes over the BUMPS (too tall for the TRENCH).',
    // built of its physical mechanisms (js/mech/r604.js, phys604.js): its FUEL go through its
    // CAD's ball path, moved by its rollers under its code's controls
    physical: '604',
    // their CAD ("Toploader Assembly") and release thread: a 27in square chassis ("to maximize area
    // available for the serializer circular bucket")
    frame: { length: 27 * IN, width: 27 * IN },
    height: 29.5 * IN, // the top of the superstructure's coroplast and the turret (their CAD: 0.749 m)
    mass: 64, // their CAD's BOM: 61.9 kg with the battery and the bumpers' wood; the bumpers' foam and covers
    // WCP Swerve X2 (their CAD): (54/10)(16/40)(45/15) = 6.48:1 on 3.91in wheels, their drive
    // motors' 120 A stator and 40 A supply limits (20 A while launching: phys604.js)
    drive: { ratio: 6.48, wheel: 3.91 * IN, stator: 120, supply: 40 },
    // two 2in rollers in silicone tubing (21in) on polycarbonate arms pivoting on the gearboxes' arm
    // shaft 0.3048 m forward and 0.2064 m up (their Viz3d's 12in and 8.125in); the front roller's
    // 0.175 m past the bumpers deployed (their -5 deg); stowed (their 80, 98 at boot) the arms
    // stand up inside the frame. fold.hull: the arm's side outline round the pivot, deployed.
    intake: {
      width: 21 * IN, reach: 0.175, rate: 200, pull: 5, deployTime: 0.45, retractTime: 0.9, side: 'front', latched: false,
      fold: {
        pivot: [0.3048, 0.2064], stowDeg: 103,
        hull: [[-0.026, 0.108], [-0.042, -0.076], [0.057, -0.166], [0.137, -0.173], [0.297, -0.107], [0.317, 0.129], [0.240, 0.136]],
      },
    },
    // the hopper box slides out 0.2921 m over the intake as the arms come down (their CAD's slider;
    // the arms' retainer plates push it out, surgical tubing pulls it back)
    storage: { extLen: 0.2921, extend: 'intake' },
    // their CAD: corrugated plastic walls to 0.69 m (the box out front, the superstructure's at the
    // back) over the DPC serializer: a plywood bucket (floor 0.154 m up, a balustrade round it, 0.62 m
    // across) on the rotor's axis, 1in behind the center, where its hook sweeps FUEL round to the
    // capture point by the central column, whose omni wheels push them up the ramp into the tube
    // to the turret
    bay: {
      x0: -0.343, x1: 0.343, hw: 0.336, top: 0.69,
      // the bucket's floor, then up over the intake's gearboxes and Ball Bar behind the bumper to the
      // arms under the box out front (0.3 m up)
      floor: { a: 0.154, b: 0, lo: 0.154, hi: 0.3, pts: [[-0.343, 0.154], [0.25, 0.154], [0.31, 0.255], [0.36, 0.3], [0.66, 0.3]] },
      column: { x: -0.0254, z: 0, r: 0.0815, y1: 0.524 },
      obstacles: [
        { x: -0.0254, z: 0, r: 0.0815, y0: 0.35, y1: 1 }, // the tube
        { x: -0.0254, z: 0, r: 0.12, y0: 0.15, y1: 0.35 }, // the ramp and the up-feed round its foot
        { x: -0.0254, z: 0, r: 0.18, y0: 0.53, y1: 1 }, // the turret
        // (the Dyeper plates in the back corners, outside the bucket)
        { box: [-0.343, -0.2, 0, 0.262, -0.4, -0.2] },
        { box: [-0.343, -0.2, 0, 0.262, 0.2, 0.4] },
      ],
      drive: 'rotor', rotor: { x: -0.0254, z: 0, y: 0.154, r: 0.29, grip: 40, drag: 4, finR0: 0.11, spin: 15.1, idle: 0 },
      feed: { x: -0.0254, z: 0.0, via: [[0.0, 0.25, 0.0], [-0.025, 0.42, 0], [-0.025, 0.6, 0]] },
      hook: { r0: 0.28, r1: 0.12, th0: -1.2, th1: 0.4, y0: 0.2, y1: 0.256, r: 0.014 },
    },
    shooter: {
      type: 'turret',
      // on the rotor's axis (their CAD's revolutes), 50 to 310 deg round from straight ahead (their
      // soft limits: it can't point within 50 deg of the intake), facing back at rest (180)
      turretPos: { x: -0.0254, z: 0.0, center: 180, start: 180 },
      exitRadius: 0.02, // the nip between the main and hood rollers, about over the axis
      exitY: 0.68,
      turretRange: 130, turretRate: 573, // their Motion Magic's 10 rad/s
      // "Prototype serializer showed ~15BPS single stream" (their release thread)
      bps: 15,
      // their hood (15 to 35 deg) launches at 90 less it: 55 to 75 deg; their table tops out at
      // 9.9 m/s (8 m), their roller lookup at 12 (600 rad/s)
      hoodMin: 55, hoodMax: 75,
      speedMax: 12,
      spinTau: 0.3,
      shotDrop: 0.004,
      speedSigma: 0.013, angleSigma: 0.6, yawSigma: 0.6,
    },
    climber: null, // none in their CAD
    stats: { 'Shot rate': '15 BPS', Aiming: 'Turret', 'Top speed': '', Trench: 'No (BUMPS)' },
    // their CAD: gold printed parts and bumpers, black corrugated plastic, raw aluminum
    colors: { frame: 0xc3c8ce, accent: 0xfab508, trim: 0x1d1e21 },
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

export const ROBOT_ORDER = ['2910', '4414', '8793', '971', '1678', '1690', '4946', '3928', '341', '4930', '1706', '581', '1114', '604'];

// Optional add-on climbers for robots without one (971 and 1678 have their own, cfg.climber)
export const CLIMBER_OPTIONS = {
  none: null,
  l1: { maxLevel: 1, times: [0, 1.6] },
  l3: { maxLevel: 3, times: [0, 1.6, 4.0, 7.0] },
};

export const AUTO_ROUTINES = {
  best: { name: 'Best for this robot', desc: 'The team\'s own AUTO where their code is public (8793, 2910, 1678: from the side it starts on), otherwise the highest-scoring AUTO found for the robot (its own start position, NEUTRAL ZONE sweeps and DEPOT runs).' },
  none: { name: 'No auto', desc: 'Robot sits still.' },
  preload: { name: 'Score preload', desc: 'Spin up and shoot the preloaded FUEL from the start line.' },
  depot: { name: 'Preload + Depot', desc: 'Shoot the preload, collect the 24 FUEL in the DEPOT, drive back and shoot.' },
  sweep: { name: 'Neutral Zone sweep', desc: 'Through the TRENCH to the NEUTRAL ZONE, sweep the FUEL line, return over the BUMP shooting on the move.' },
  sweep2: { name: 'Double sweep', desc: 'Two NEUTRAL ZONE sweeps (2910-style), shooting on the move each time back in the ALLIANCE ZONE.' },
  climb: { name: 'Preload + Climb L1', desc: 'Shoot the preload then climb LEVEL 1 for 15 points (needs a climber add-on).' },
  defendStage: { name: 'Preload + defensive position', desc: 'Shoot the preload, then drive over the BUMP to the middle of the field (own side of the CENTER LINE) to start TELEOP on defense.' },
};
