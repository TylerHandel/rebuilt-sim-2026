// 604's Toploader as physical mechanisms (js/mech/mechanism.js), from their CAD (Onshape "Toploader
// Assembly" through a STEP translation; its mates give the joints, its BOM the materials and
// masses), their CAD/code release thread on Chief Delphi and their code (frc604/2026-public,
// FRC-2026: Constants, the intake, serializer and turret subsystems, LaunchCalculator,
// TrackAndLaunchCommand, UnjamCommand, DeployIntake / RetractIntake, FollowQuikplan).
// Robot frame: x forward (the intake), y up, z to its right; the CAD's x to the robot's left, y back,
// z up.
//
// The ball path, single file from a full-width intake ("Prototype serializer showed ~15BPS single
// stream vs ~23BPS for full-width"; "Serialization was identified as the limiting factor for BPS"):
// on the intake's polycarbonate arms (on the gearboxes' arm shaft, 0.3048 m forward, 0.2064 up),
// the front 2in roller (polycarbonate in silicone tubing) pulls FUEL in off the carpet against the
// spring-loaded kicker bar at the bumper's foot and up the bumper's face; the second roller above it
// squeezes them past the "Compression Cross Tube" (2.5 cm of foam between them) up and back over
// the bumper and the gearboxes' "Ball Bar" into the hopper. The hopper's sliding coroplast box (the
// intake pushes it out 0.2921 m through its retainer plates; surgical tubing pulls it back in) sits
// over the Dynamic Point of Capture serializer: a plywood bucket 0.62 m across (its floor 0.154 m
// up, a balustrade round it), and in it the rotor (a dye rotor). The rotor's curved hook (between
// two plates 0.2 and 0.256 m up, out to 0.284 m from its axis) sweeps round the bucket, clockwise
// seen from above, gathering FUEL in its pocket and steering them along its inside to the capture
// point at its root, where three 2in omni wheels standing on end (the infeed) push them onto the
// rotor's curved ramp and up it, under the up-feed roller (a 2in flex wheel on printed 1.625in
// rollers), which pushes them up into the tube (corrugated plastic, 155 mm bore, "held in shape by
// 3D printed ends"), each new FUEL pushing the ones in the tube up into the turret on the same
// axis ("Successive fuel pushes existing fuel in the tube up into the turret"). In the turret the
// main roller (2in, silicone tubing over PETG) catches each on its way up and flings it between
// itself and the hood's 1.25in roller (turning the other way, 1.4x as fast), out at 90 deg less the
// hood's angle.
// Drives (the CAD's gearing and their code's ratios, current limits and loops):
//   intake rollers: two Kraken X60s (80 A stator, 40 A supply, one following the other), 12:24 then
//     24:24 belts (their rollerMotorRatio) onto both rollers
//   intake arm: a Kraken X44 on each side (independent, for impacts; 30 A stator, 20 A supply)
//     through 8:38, 18:38 and 12:36 (30.08:1), their Motion Magic to -5 deg (deployed), 10
//     (standby), 80 (stowed, slowly); 98 at boot (their startingAngle)
//   rotor: a Kraken X60 (80 A, 40 A) through a MAXPlanetary and a 10t spur (21.2:1, their
//     rotorMotorRatio), velocity loop
//   infeed and up-feed ("roller"): a Kraken X60 on the rotor's axis (100 A, 60 A) by #25 chain 10t
//     to a 16t idler on the rotor, 16t to the infeed's 16t (1.6:1), 15t bevels 1:1 to the up-feed
//   launcher: two Kraken X60s (80 A, 40 A) belted 15:18 up (1.2x) to the main roller, which turns two
//     4in brass flywheels 1:1 on its belts (0.7 kg each in their BOM) and, through an 18t:18t belt
//     and 28t:20t gears, the hood roller the other way
//   hood: a Kraken X44 through 8:50 and 10:165 (103.1:1) onto its sector, about the main roller's
//     axle (their hoodMotorRatio), Motion Magic, 15 to 35 deg
//   turret: a Kraken X60 (the CAD's; their simulation says X44) through 10:135 and 10:24 (32.4:1),
//     Motion Magic (10 rad/s, 50 rad/s^2), 50 to 310 deg (0 is straight ahead: it can't point
//     within 50 deg of the intake)
import { IN } from '../constants.js';
import { Block, Motor, Shaft, Surface, Train, Wheel } from './mechanism.js';

// what each grips FUEL with (estimates, as 341's and 8793's): the rollers' silicone tubing ("high
// grip with fuel", their release thread); the up-feed's 60A flex wheel, compliant; the infeed's 80A
// omnis; the up-feed's printed 1.625in rollers (their material isn't in the BOM: printed TPU, as
// their plugs, an estimate)
export const MU = { silicone: 1.6, compliant: 1.6, omni: 0.6, printed: 0.8, polycarb: 0.5 };

// ------------------------------------------------------------------ the layout (robot frame)
export const HALF = 13.5 * IN; // the frame's half length and width (27in square, their thread)
// the DPC rotor's axis and the turret's (one axis: the CAD's revolutes, 0.0254 m behind the
// center); their code puts the turret 1in to the right (robotToTurretBaseT: (0, -1in)), the CAD 1in
// back: the CAD's is used here
export const AXIS = [-0.0254, 0];
export const PIVOT = [0.3048, 0.2064]; // the intake arm's (their Viz3d: 12in forward, 8.125in up)
export const INTAKE = {
  // the two 2in polycarbonate rollers in silicone tubing (the CAD's 54 mm over the tubing), 21in
  // long, as exported (their 0 deg)
  rollers: [[0.5826, 0.1524], [0.5553, 0.2381]], r: 0.027, width: 0.533,
  // their wrist angles (deg; positive up, 0 the CAD's pose): deployAngle -5 (also their minimum),
  // standbyAngle 10, stowAngle 80, startingAngle 98 (also their maximum)
  deploy: -5, standby: 10, stow: 80, start: 98, min: -5, max: 98,
  // Motion Magic (their maxWristVelocity 4 rad/s, slow 2; 45 rad/s^2), position loop (kP 1 V per
  // motor rotation, kV 0.12 V per motor rotation/s); within 30 deg it rolls (wristTolerance)
  mmV: 4, mmSlow: 2, mmA: 45, kP: 1, kV: 0.12, tol: 30,
  // the arm: polycarbonate plates, the two rollers and their plugs, the kicker bar, the cross tubes
  // and belts (their BOM: ~2 kg), its center of mass ~0.2 m out; their simulation's 0.059 kg m^2 a
  // side
  mass: 2.0, com: 0.2, J: 0.118,
};
// the hopper box slides out along x as the arm swings down (the arm's retainer plates push its
// front; surgical tubing pulls it back against them): 0.2921 m all the way out (the CAD's slider,
// exported out); its front retainer corner (as exported) and where the box is pushed
export const HOPPER = { out: 0.2921, retainer: [0.608, 0.362], front: 0.608 };
// the DPC: the bucket floor, the rotor's capture-point wheels (as exported, rotor at 0)
export const BUCKET = { floor: 0.154, r: 0.292 };
export const ROTOR = {
  // the infeed: three 2in 80A omni wheels standing on end (their axles up), 27.5 mm wide each
  omni: { at: [-0.1150, -0.0913], ys: [0.2051, 0.2326, 0.2601], r: 0.025, w: 0.0275 },
  // the up-feed roller: a 2in 60A flex wheel (1in wide) on the shaft over them, flanked by
  // printed 1.625in roller discs (their rollerWheelDiameter)
  upfeed: { at: [-0.1159, 0.3159], flex: { z: 0.004, r: 0.0255, w: 0.0257 }, printed: [{ z: -0.0341, w: 0.051 }, { z: 0.0231, w: 0.0129 }], pr: 0.0206 },
  // the rotor (plywood and aluminum plates, the hook, the ramp, the tube, the omnis and the
  // up-feed): their simulation's 0.03 kg m^2
  J: 0.03,
  // their effectiveRotorRadius (7.8in): where the rotor's speed is measured against the rollers'
  effR: 7.8 * IN,
};
// the turret's frame: about AXIS, facing forward (as exported it fires to the robot's right, a
// quarter turn round); the main roller (2in PETG in silicone tubing, 54 mm over it) and the hood
// roller (1.25in, 35 mm), 0.114 m long between the hood's plates (0.163 m apart)
export const SHOOTER = {
  main: { at: [0.0744, 0.5969], r: 0.027, w: 0.114 },
  hoodRoller: { at: [-0.0686, 0.6971], r: 0.01745, w: 0.115 },
  hoodPerMain: 28 / 20,
  // what turns with the main roller: the two 4in brass flywheels (0.7 kg, 9.2e-4 kg m^2 each), the
  // roller, pulleys and shafts (the motors' rotors are added)
  J: 1.95e-3,
  // their velocity loop (kP 0.2, kV 0.12 V per motor rotation/s)
  kP: 0.2, kV: 0.12,
  // their hood angles (deg): 15 to 35; the CAD has it at 35 (the nip between the rollers aims the
  // shot 55.0 deg up: their launch angle is 90 less the hood's)
  hoodMin: 15, hoodMax: 35, hoodCad: 35,
  // the hood's Motion Magic (2 rad/s, 20 rad/s^2: their hood motor config, in hood radians)
  hoodV: 2, hoodA: 20,
  // tolerances (TurretSubsystem / Constants.Turret): scoring 0.8 m/s, 3 deg, 5 deg of yaw; feeding
  // 2 m/s, 15 deg, 40 deg
  tolV: 0.8, tolAngle: 3, tolYaw: 5, feedTolV: 2.0, feedTolAngle: 15, feedTolYaw: 40,
  // the turret's travel (their minYawAngle, maxYawAngle, startingYawAngle) and Motion Magic
  yawMin: 50, yawMax: 310, yawStart: 180, yawV: 10, yawA: 50,
  // where FUEL leave about, at the CAD's hood (tools/mechlab604.mjs shooter)
  exit: [0.0, 0.68],
};
// their launch table (deploy/launch/continuousv0theta_accuracy_checked.csv: Monte-Carlo searched
// pairs least sensitive to error): turret-to-HUB-center distance (m) -> exit speed (m/s), launch
// angle (deg); and their measured exit speed -> roller speed (launchVelocityEntries, m/s ->
// rad/s of the main roller), which they trim on their dashboard (VelocityMultiplier, 0.95 by
// default)
export const LAUNCH = [
  [1.1, 5.647, 75.0], [1.2, 5.604, 74.65], [1.3, 5.57, 73.676], [1.4, 5.591, 71.896], [1.5, 5.634, 70.297], [1.6, 5.671, 69.58], [1.7, 5.728, 68.361],
  [1.8, 5.79, 67.543], [1.9, 5.894, 65.093], [2.0, 5.958, 64.39], [2.1, 6.035, 63.812], [2.2, 6.112, 63.277], [2.3, 6.17, 62.604], [2.4, 6.228, 61.951],
  [2.5, 6.291, 61.358], [2.6, 6.367, 60.949], [2.7, 6.445, 60.58], [2.8, 6.513, 60.115], [2.9, 6.62, 60.1], [3.0, 6.688, 60.0], [3.1, 6.732, 60.306],
  [3.2, 6.788, 60.435], [3.3, 6.855, 60.21], [3.4, 6.942, 60.107], [3.5, 7.034, 60.05], [3.6, 7.093, 60.0], [3.7, 7.165, 60.0], [3.8, 7.233, 60.0],
  [3.9, 7.289, 60.02], [4.0, 7.361, 60.02], [4.1, 7.446, 60.0], [4.2, 7.5, 60.0], [4.3, 7.572, 60.0], [4.4, 7.636, 60.0], [4.5, 7.699, 60.0],
  [4.6, 7.763, 60.0], [4.7, 7.833, 60.0], [4.8, 7.882, 60.0], [4.9, 7.957, 60.0], [5.0, 8.034, 60.02], [5.1, 8.098, 60.0], [5.2, 8.136, 60.0],
  [5.3, 8.246, 60.0], [5.4, 8.279, 60.0], [5.5, 8.359, 60.0], [5.6, 8.428, 60.0], [5.7, 8.5, 60.0], [5.8, 8.573, 60.0], [5.9, 8.642, 60.0],
  [6.0, 8.706, 60.0], [6.1, 8.726, 60.0], [6.2, 8.775, 60.0], [6.3, 8.841, 60.0], [6.4, 8.9, 60.0], [6.5, 8.96, 60.0], [6.6, 9.036, 60.0],
  [6.7, 9.116, 60.05], [6.8, 9.165, 60.0], [6.9, 9.226, 60.02], [7.0, 9.295, 60.02], [7.1, 9.336, 60.01], [7.2, 9.39, 60.0], [7.3, 9.45, 60.0],
  [7.4, 9.504, 60.0], [7.5, 9.573, 60.02], [7.6, 9.627, 60.0], [7.7, 9.68, 60.0], [7.8, 9.763, 60.0], [7.9, 9.82, 60.03], [8.0, 9.873, 60.02],
];
export const VELOCITY_TO_RAD = [[0, 0], [5.54, 220], [7.88, 360], [9.0, 480], [10.0, 510], [12.0, 600]];
// Their trim isn't applied here: their FUEL leave a little faster than the sim's at the same roller
// speed. Through the CAD's nip (the main roller and the hood's, the hood's printed guide behind)
// the sim's leave at 0.93 of the main roller's surface at their slow shots (their lookup says
// 0.93 too, without the trim) falling to 0.79 at 480 rad/s (their 0.69..0.74). Untrimmed, their
// table's shots come down within 0.3 m of the HUB's center out to 6 m (the far corner of the
// ALLIANCE ZONE; 0.5 m short at 7 m, 1.5 m short at 8, past their last 9 m/s: the sim's FUEL leave
// no faster than 9.7 m/s); at their 0.95, 0.15 to 0.55 m short (tools/mechlab604.mjs shooter)
export const VELOCITY_MULTIPLIER = 1.0, THEIR_MULTIPLIER = 0.95;
export const MAX_RAD = 600;
// feeding (their feed shots): a fixed 55 deg, the speed to land at the feed point from the
// turret's 27.5in (robotToTurretBaseT's height), solved ballistically
export const FEED = { angle: 55, height: 27.5 * IN };
const interp = (t, x, j = 1) => {
  if (x <= t[0][0]) return t[0][j];
  for (let k = 1; k < t.length; k++) if (x <= t[k][0]) return t[k - 1][j] + ((t[k][j] - t[k - 1][j]) * (x - t[k - 1][0])) / (t[k][0] - t[k - 1][0]);
  return t[t.length - 1][j];
};
// their LaunchCalculator's table row for a distance (clamped at its ends: InterpolatingTreeMap)
export function launchFor(d) { return { v: interp(LAUNCH, d, 1), angle: interp(LAUNCH, d, 2) }; }
// their feed speed (MathUtils.solveBallisticVelocity), or 0 when out of reach at 55 deg
export function feedSpeed(d) {
  const th = (FEED.angle * Math.PI) / 180, den = 2 * Math.cos(th) ** 2 * (d * Math.tan(th) + FEED.height);
  return den > 0 ? Math.sqrt((9.81 * d * d) / den) : 0;
}
// the main roller's speed (rad/s) they command for an exit speed (m/s): their lookup (x their trim)
export const radFor = (v) => interp(VELOCITY_TO_RAD, v) * VELOCITY_MULTIPLIER;
// and their reading back: the exit speed their lookup says a roller speed gives
export const velocityAt = (rad) => interp(VELOCITY_TO_RAD.map(([a, b]) => [b, a]), rad);

// the serializer's numbers (Constants.Serializer): the up-feed's speed is set as the FUEL's
// (rollerVelocity 3.6 m/s while scoring and feeding; -0.5 to unjam), which they take as half the
// 1.625in roller's surface speed ("Roller is only on one side of the ball"); the rotor follows at
// the rollers' measured speed / 1.2 at its effective radius (a 10-sample average)
export const SERIALIZER = { rollerR: 0.5 * 1.625 * IN, ballV: 3.6, unjamV: -0.5, rotorPerRoller: 1.2, kP: 0.1, kV: 0.125 };
// the roller's shaft speed (rad/s) for a FUEL speed (m/s), as their setRollerVelocity
export const rollerRad = (v) => v / (0.5 * SERIALIZER.rollerR);

// the preload: eight on the bucket floor round the rotor
export const PRELOAD = [0, 1, 2, 3, 4, 5, 6, 7].map((k) => {
  const a = (k / 8) * 2 * Math.PI + 0.3;
  return [AXIS[0] + 0.2 * Math.cos(a), 0.235, 0.2 * Math.sin(a)];
});

// ------------------------------------------------------------------ the drives
export function build604() {
  // the intake's rollers: positive pulls FUEL in (their bottoms run rearward, their backs up)
  const intake = new Train({ name: 'intake rollers', motor: new Motor('krakenX60', { count: 2, statorLimit: 80, supplyLimit: 40 }) });
  const k = (12 / 24) * (24 / 24);
  const front = intake.add(new Shaft({ name: 'intake front roller', inertia: 1.1e-4 }), -k);
  const back = intake.add(new Shaft({ name: 'intake second roller', inertia: 1.1e-4 }), -k);
  // the arm's pivot (positive swings it up): two X44s through 30.08:1
  const pivot = new Shaft({ name: 'intake arm', motor: new Motor('krakenX44', { count: 2, ratio: (38 / 8) * (38 / 18) * (36 / 12), statorLimit: 30, supplyLimit: 20 }), inertia: INTAKE.J, drag: 0.3 });
  // the rotor (positive turns it clockwise seen from above, its hook's tip leading)
  const rotor = new Shaft({ name: 'DPC rotor', motor: new Motor('krakenX60', { ratio: 212 / 10, statorLimit: 80, supplyLimit: 40 }), inertia: ROTOR.J, drag: 0.02 });
  // the infeed and the up-feed: positive feeds FUEL in, up the ramp and up the tube
  const roller = new Train({ name: 'infeed and up-feed', motor: new Motor('krakenX60', { statorLimit: 100, supplyLimit: 60 }) });
  const infeed = roller.add(new Shaft({ name: 'infeed omnis', inertia: 3e-5 }), 10 / 16);
  const upfeed = roller.add(new Shaft({ name: 'up-feed roller', inertia: 2e-5 }), 10 / 16);
  // the launcher: positive runs the main roller's back up (FUEL up over it), the hood roller the
  // other way
  const launcher = new Train({ name: 'launcher', motor: new Motor('krakenX60', { count: 2, ratio: 15 / 18, statorLimit: 80, supplyLimit: 40 }) });
  const main = launcher.add(new Shaft({ name: 'main roller and brass flywheels', inertia: SHOOTER.J }), -1);
  const hoodRoller = launcher.add(new Shaft({ name: 'hood roller', inertia: 1.5e-5 }), SHOOTER.hoodPerMain);
  return { intake, front, back, pivot, rotor, roller, infeed, upfeed, launcher, main, hoodRoller };
}

const Z = [0, 0, 1], Y = [0, 1, 0];
// the wheels (robot frame; the intake's posed with placeIntake, the rotor's with placeRotor, the
// shooter's with placeShooter)
export function wheels604(t) {
  const I = INTAKE;
  const intake = I.rollers.map(([x, y], i) => new Wheel({ name: i ? 'intake second roller (silicone)' : 'intake front roller (silicone)', shaft: i ? t.back : t.front, center: [x, y, 0], axis: Z, radius: I.r, width: I.width, mu: MU.silicone, tread: 20000 })
    .onPivot(t.pivot, [...PIVOT, 0], Z));
  intake.forEach((w) => { w.home = w.center.slice(); });
  const O = ROTOR.omni, U = ROTOR.upfeed;
  const rotor = [
    ...O.ys.map((y, i) => new Wheel({ name: `infeed 2in omni ${i + 1}`, shaft: t.infeed, kind: 'omni', center: [O.at[0], y, O.at[1]], axis: Y, radius: O.r, width: O.w, mu: MU.omni, tread: 30000 })),
    new Wheel({ name: 'up-feed 2in flex wheel', shaft: t.upfeed, center: [...U.at, U.flex.z], axis: Z, radius: U.flex.r, width: U.flex.w, mu: MU.compliant, tread: 4000 }),
    ...U.printed.map(({ z, w }) => new Wheel({ name: 'up-feed 1.625in printed roller', shaft: t.upfeed, center: [...U.at, z], axis: Z, radius: U.pr, width: w, mu: MU.printed, tread: 20000 })),
  ].map((w) => w.onPivot(t.rotor, [AXIS[0], 0, AXIS[1]], [0, -1, 0]));
  rotor.forEach((w) => { w.home = w.center.slice(); w.homeAxis = w.axis.slice(); });
  return { intake, rotor };
}
export function shooterWheels(t) {
  const S = SHOOTER;
  const w = [
    new Wheel({ name: 'main roller (silicone)', shaft: t.main, center: [0, 0, 0], axis: Z, radius: S.main.r, width: S.main.w, mu: MU.silicone, tread: 20000 }),
    new Wheel({ name: 'hood roller (silicone)', shaft: t.hoodRoller, center: [0, 0, 0], axis: Z, radius: S.hoodRoller.r, width: S.hoodRoller.w, mu: MU.silicone, tread: 20000 }),
  ];
  placeShooter(w, 0, SHOOTER.hoodCad);
  return w;
}
// The front bumper FUEL climb on the way in: the CAD's 2.5in of foam (0.370 to 0.433 m out, 0.038
// to 0.165 m up, 0.87 m across with its corners) on its 3/4in wood, as a solid block (a face alone,
// pressed hard by the rollers, lets a FUEL through into the frame, as 2910's did); soft (2.5in of
// foam: 20 kN/m, as 341's estimate)
export function bumper604() {
  return new Block({ name: 'bumper', x0: 0.3699, x1: 0.4334, y0: 0.0381, y1: 0.1651, hw: 0.87 / 2, mu: 0.6, tread: 20000 });
}

const rot = ([x, y], [cx, cy], a) => { const c = Math.cos(a), s = Math.sin(a), dx = x - cx, dy = y - cy; return [cx + dx * c - dy * s, cy + dx * s + dy * c]; };
// the arm's turn from the CAD's pose (rad; up positive) at their wrist angle (deg): 1:1 (the CAD is
// their 0)
export const armRot = (deg) => (deg * Math.PI) / 180;
export function placeIntake(wheels, deg) {
  const a = armRot(deg);
  for (const w of wheels) { const [x, y] = rot(w.home, PIVOT, a); w.center = [x, y, 0]; }
}
// the hopper box: how far out (m) with the arm at deg, and how fast it slides per radian of the
// arm (m/rad) there: its front follows the retainer plates' front corner
export function hopperExt(deg) {
  const [x] = rot(HOPPER.retainer, PIVOT, armRot(deg));
  return Math.max(0, Math.min(HOPPER.out, HOPPER.out + x - HOPPER.front));
}
export function hopperRate(deg) {
  const e0 = hopperExt(deg - 0.5), e1 = hopperExt(deg + 0.5);
  return ((e1 - e0) / 1) * (180 / Math.PI);
}
// a point (robot frame) turned with the rotor by phi (rad, clockwise from above) about its axis
export function rotorPoint([x, y, z], phi) {
  const dx = x - AXIS[0], dz = z - AXIS[1], c = Math.cos(phi), s = Math.sin(phi);
  return [AXIS[0] + dx * c - dz * s, y, AXIS[1] + dx * s + dz * c];
}
export function placeRotor(wheels, phi) {
  const c = Math.cos(phi), s = Math.sin(phi);
  for (const w of wheels) {
    w.center = rotorPoint(w.home, phi);
    const [ax, ay, az] = w.homeAxis;
    w.axis = [ax * c - az * s, ay, ax * s + az * c];
  }
}
// the hood's turn about the main roller's axle (rad; positive raises the shot) at their angle deg
export function hoodRot(deg) { return ((SHOOTER.hoodCad - Math.max(SHOOTER.hoodMin, Math.min(SHOOTER.hoodMax, deg))) * Math.PI) / 180; }
// the shot's elevation by the CAD's geometry: square to the line between the rollers' axles (55.0
// deg at the CAD's 35), which is 90 less their hood angle
export function hoodElevation(deg) {
  const [mx, my] = SHOOTER.main.at, [hx, hy] = SHOOTER.hoodRoller.at;
  return ((Math.atan2(mx - hx, hy - my) + hoodRot(deg)) * 180) / Math.PI;
}
// the shooter's wheels where the turret (yaw, rad, about up; 0 facing forward) and the hood put them
export function placeShooter([main, hood], yaw, hoodDeg) {
  const c = Math.cos(yaw), s = Math.sin(yaw), S = SHOOTER;
  const at = ([x, y]) => [AXIS[0] + x * c, y, AXIS[1] - x * s];
  main.center = at(S.main.at);
  const [hx, hy] = rot(S.hoodRoller.at, S.main.at, hoodRot(hoodDeg));
  hood.center = at([hx, hy]);
  for (const w of [main, hood]) w.axis = [s, 0, c];
}

// the ball path's surfaces from their CAD (js/mech/path604.js): the body; the intake's arm on its
// pivot; the hopper box (it slides with the arm); the rotor on its axis (it turns, and FUEL it
// touches push back on its motor); the turret and the hood (posed each step). (The bumper: its
// foam noodles, bumper604.)
// (FUEL's foam on the ball path's plywood, printed plastic and corrugated tube: 0.25, an estimate;
// at the 0.35 the turret and hood keep, the serializer's best second was 13 FUEL against their
// ~15 BPS, the infeed's motor at its current limit pushing the column up the tube; at 0.25, 15)
export function pathSurfaces604(mesh, t, { mu = 0.25, muShooter = 0.35 } = {}) {
  const s = (k, pos, m = mu) => new Surface({ name: `${k} (CAD)`, vertices: mesh[k].v.map((x) => x / 1000), indices: mesh[k].i, mu: m, pos });
  const body = s('body', [0, 0, 0]);
  const intake = s('intake', [...PIVOT, 0]).onShaft(t.pivot, [...PIVOT, 0], Z);
  // (the hopper slides as the arm turns: hopperRate m per radian of the arm, set by placeHopper)
  const cam = { k: 0, get omega() { return t.pivot.omega * this.k; }, J: Infinity, impulse(L) { t.pivot.impulse(L * this.k); } };
  const hopper = s('hopper', [0, 0, 0], 0.3).onSlide(cam, [1, 0, 0]);
  hopper.slideK = 1;
  hopper.cam = cam;
  const rotor = s('rotor', [AXIS[0], 0, AXIS[1]]).onShaft(t.rotor, [AXIS[0], 0, AXIS[1]], [0, -1, 0]);
  const turret = s('turret', [AXIS[0], 0, AXIS[1]], muShooter);
  const hood = s('hood', [AXIS[0] + SHOOTER.main.at[0], SHOOTER.main.at[1], AXIS[1]], muShooter);
  return { body, intake, hopper, rotor, turret, hood, bumper: bumper604() };
}
export function placeHopper(sf, deg) {
  sf.setPose([hopperExt(deg) - HOPPER.out, 0, 0]);
  sf.cam.k = hopperRate(deg);
}
// the rotor's surfaces at phi (clockwise from above: the surface's yaw is the other way)
export function placeRotorSurface(sf, phi) { sf.setPose([AXIS[0], 0, AXIS[1]], -phi, 0); }
export function placeTurretSurfaces({ turret, hood }, yaw, hoodDeg) {
  turret.setPose([AXIS[0], 0, AXIS[1]], yaw, 0);
  const c = Math.cos(yaw), s = Math.sin(yaw), mx = SHOOTER.main.at[0];
  hood.setPose([AXIS[0] + mx * c, SHOOTER.main.at[1], AXIS[1] - mx * s], yaw, hoodRot(hoodDeg));
}
