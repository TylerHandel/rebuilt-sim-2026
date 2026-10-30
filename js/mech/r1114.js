// 1114's Simbot Tim as physical mechanisms (js/mech/mechanism.js), from their CAD (Onshape
// "S26-A000" through a STEP translation, the box slid out) and their code (Simbotics/2026-Simbot-Tim:
// the intake, indexer and shooter subsystems' constants and IO, ShootMoveLUT and the intake,
// indexer and shooter commands). Robot frame: x forward (the intake), y up, z to its right; the
// CAD's x to the left, y back, z up.
//
// The ball path: the hopper box (clear polycarbonate walls, 0.75 m wide, its top 0.70 m up) slides
// 0.298 m out on racks rising 8.7 deg toward the robot, carrying the intake on its front: a 2in
// silicone roller that pulls FUEL in under it and up over the kicker bar (hanging on arms in front
// of the bumper, its "safety walk" grip) and the bumper. Inside, a floor of nine 1.25in rollers
// (grip tape, all belted together) slopes down from the front to a valley just in front of the
// tower and climbs its back, under a row of seven 3in 45A squish wheels at the tower's foot (the
// floor drives them); a 2in roller along each side of the floor's narrow channel (one X44 drives
// both, the right one by a belt across the robot) and two free 5/8in rollers standing up either
// side of the tower's front; the "ball elevator" up the tower: two columns
// of four 1.25in rollers (grip tape) 0.175 m apart, counter-rotating, two FUEL wide, lifting them
// to the 3in flywheel at the top (grip tape), which carries them up round its back under the
// hood's three 1.5in rollers (grip tape, driven off the flywheel the other way) and flings them
// forward over the hopper. In front of the elevator, three 1.25in rollers belted together (the
// "roller wall": the tower's face; the team "added the active wall in front of the shooter tower"
// after their first event: driven, its face running down, so the load leaning on it is carried
// down to the elevator's foot).
// Drives (the CAD's belts and gears, their code's limits; the Phoenix 6 default 70 A supply limit
// where their config only sets the stator's):
//   intake: two Kraken X60s on the box (85 A stator), 12t:30T (HTD) to the roller (0.4 of the motor)
//   slide: a Kraken X44 (60 A stator) 12t:36t to a cross shaft, 30t:36t up to each rack's 10t 10DP
//     pinion (1in pitch circle): 3.6:1, 22.2 mm of travel per motor turn, so their 13.5 "inches"
//     (SLIDE_INCHES_TO_ROTATIONS 1: motor turns) are the CAD's 0.298 m
//   floor ("hopper"): two X60s (70 A stator) 12t:24t onto the valley roller, the rest belted to it
//     20t:20t (0.5 of the motor); 18t:30t and 20t:44t from it to the squish wheels (0.136, the
//     other way)
//   side rollers: an X44 (70 A stator) 12t:64t (0.1875)
//   ball elevator ("ballevator"): an X60 (70 A stator) 12t:24t to the front column (belted 1:1 up
//     it), 24t:24t and 30t:26t to the back column (0.577, the other way); their velocity loop
//     (kP 0.3, kV 0.13 V per motor rotation/s) at the Talon's 1 kHz
//   shooter: four X60s (50 A stator) geared 50t:50t together, 36t:24t to a jackshaft, 24t:48t up to
//     the flywheel (0.75 of the motor; their RPM is the motors'), 36t:15t from the jackshaft to a
//     4in brass inertia flywheel (3.6 of the motor); the flywheel's 36t to the hood's 24t jackshaft
//     and 34t:34t to the middle hood roller, belted 24t:24t to the other two (1.5 of the flywheel,
//     the other way)
//   hood: an X44 (30 A stator) through 36:10, 36:15 and a pinion on its sector (2.43 deg per motor
//     turn: their HOOD_DEGREES_TO_ROTATIONS)
import { Block, BUMPER_MU, Motor, Shaft, Surface, Train, Wheel } from './mechanism.js';
import { FUEL } from '../constants.js';

const R = FUEL.radius;
// what each grips FUEL with (estimates, in the order 8793's team gave): the team's grip tape on
// nearly every roller (the floor, the elevator, the shooter: "skateboard grip tape"; the roller
// wall and side rollers "cat tongue"), the intake's silicone, the 45A squish wheels compliant
export const MU = { gripTape: 1.3, silicone: 1.0, compliant: 1.6 };

// ------------------------------------------------------------------ the layout (robot frame)
// the floor's nine 1.25in rollers (their sleeves 0.0333 m), front to back: down to the valley in
// front of the tower, then up its back to the elevator's foot
export const FLOOR = {
  rollers: [[0.2857, 0.1668], [0.2276, 0.1457], [0.1696, 0.1246], [0.1116, 0.1034], [0.0536, 0.0823], [-0.0326, 0.0719], [-0.1144, 0.1007], [-0.1751, 0.1626], [-0.2022, 0.2451]],
  r: 0.01665, width: 0.3937,
};
export const SQUISH = { at: [-0.0118, 0.2619], r: 1.5 * 0.0254, w: 0.0254, z: [-0.1524, -0.1016, -0.0508, 0, 0.0508, 0.1016, 0.1524] };
// the side rollers: 2in, along x either side of the floor's channel (their bottom ones)
export const SIDE = { x: 0.1129, y: 0.2246, z: 0.2725, r: 0.0254, len: 0.171 };
// the passive side rollers (the team: "the top 2 rollers are just passive and free spinning on both
// the left and right side"): two 5/8in rollers standing up either side of the tower's front, from
// 0.352 to 0.505 m up (their CAD's "Part 2" between its "Part 1" bosses), where the load used to
// "crystallize" on the hopper's sides (the team)
export const PASSIVE = { at: [[0.0483, 0.2883], [0.08, 0.3382]], y: 0.4286, r: 0.008, len: 0.153 };
// the ball elevator: its back and front columns, and the roller wall in front of it
export const ELEV = {
  back: [[-0.2096, 0.3224], [-0.2096, 0.3816], [-0.2096, 0.4408], [-0.2096, 0.5001]],
  front: [[-0.0343, 0.3224], [-0.0343, 0.3816], [-0.0343, 0.4408], [-0.0343, 0.5001]],
  wall: [[0.0134, 0.3266], [0.0195, 0.4134], [0.0255, 0.5002]],
  r: 0.01665, width: 0.3937,
};
// the flywheel's axis, which the hood turns about; its 3in tube with a 1/16in sleeve
export const FLY = [-0.0176, 0.5715];
export const SHOOTER = {
  fly: { r: 0.0402, width: 0.4064 },
  // the hood's three 1.5in rollers as exported (the CAD's pose: the shot leaves 45.6 deg up)
  hood: [[-0.2024, 0.6137], [-0.1841, 0.6621], [-0.1534, 0.7037]], hoodR: 0.01975, hoodWidth: 0.4064,
  flyPerMotor: (36 / 24) * (24 / 48), brassPerMotor: (36 / 24) * (36 / 15), hoodPerFly: 36 / 24,
  // inertia (kg m^2), each about its own axis from the CAD's solids (OpenCascade's volume
  // properties on the STEP) and the BOM's materials; README, *1114*, has the table. The flywheel:
  // its 3in 6061 tube (0.48 kg), the 2 mm sleeve over it (no BOM material: 1240, a print's),
  // the Onyx end hubs with their 48t and 36t pulleys, the hub rivets and stubs, and the 160T and
  // 45T belts at their pulleys' pitch radius. The brass: SDS's 4in brass flywheel (0.70 kg, the
  // 1.54 lb SDS publishes; 1.170e-3 against their 4.0 lb in^2), its 15t pulley and stub. The
  // jackshaft (1.5 of the motors): its 36t and two 24t pulleys, the 75T belt. The motors' output
  // shafts: four 50t gears, their hex adapters, the 36t pulley and the 90T belt. The hood's
  // drive (1.5 of the flywheel): the three 1.5in rollers (6061 tubes, 7075 shafts, the BOM's
  // carbon-fiber end pieces, grip tape), their 24t pulleys and 34t gears and the jackshaft that
  // drives them. (The motors' rotors are added by the Train.)
  flyJ: 1.36e-3, brassJ: 1.171e-3, jackJ: 5.06e-5, motorJ: 1.42e-4, hoodJ: 4.27e-4,
  jackPerMotor: 36 / 24,
  // their velocity loop (Slot0, VelocityVoltage, motor rotations/s): kV x target + kP x error
  kV: 0.14, kP: 0.25,
  // their ShooterIdle (SHOOTER_IDLE_VELOCITY, motor RPM): the shooter's default command in their
  // RobotContainer, commented out there; here it runs whenever it isn't aiming
  idle: 1000,
  // the four X60s' current limits (ShooterConstants.leadShooterConfigs, the followers take the
  // leader's configuration): a 50 A stator limit and nothing else, so Phoenix 6's defaults for the
  // rest: a 70 A supply limit (falling to 40 A once it's been at 70 for 1 s: never here, the
  // stator limit keeps the supply under 50), +-16 V peak output (the battery's the bound), no
  // closed-loop ramp; their VelocityVoltage has FOC off (trapezoidal). STATOR / SUPPLY in the
  // environment change them on tools/mechlab1114.mjs (spinup, dump), not in play
  stator: 50, supply: 70,
  // their hood angles (ShooterConstants): it homes against its stop at 74.484, steepest, and runs
  // 42 to 74; the CAD has it at 45.6 (the last roller's place round the flywheel)
  hoodCad: 45.6, hoodHome: 74.484, hoodMin: 42, hoodMax: 74,
  // their hood motor's degrees per motor turn (HOOD_DEGREES_TO_ROTATIONS) and its position loop
  // (kP 8, kI 5, kD 0.14 V per motor rotation)
  hoodDegPerRot: (10 / 36) * (15 / 36) * (Math.PI / 53.87831) * 360, hoodKP: 8,
};
// the elevator's velocity loop (their ballevator Slot0) and its shot speed (BALLEVATOR_SHOOT_VELOCITY)
export const BALLEVATOR = { kV: 0.13, kP: 0.3, shoot: 4500 };
// the intake roller, as exported (the box all the way out)
export const INTAKE = { at: [0.5457, 0.1663], r: 0.0262, width: 0.635 };
// the box's slide: all the way out as the CAD has it (0.298 m of rack, a parallel mate along
// (0, 0.988, 0.152) in the CAD); going in it moves back and up along DIR. pinion: its 10t 10DP
// pinion's pitch radius; perRot: m of travel per slide-motor turn; mass: what slides (the box's
// polycarbonate, side plates, two X60s, the roller, the kicker: ~6 kg, an estimate).
// pull: the volts their AgitateIntake pulls it in with while shooting, here their own
// SLIDE_FAST_PULL_VOLTAGE (-3.5, kept in their IntakeConstants for a commented-out "fast" mode),
// not the -2 their AgitateIntake runs (theirPull): at 2 V the X44 makes ~46 A at a stall, under its
// 60 A limit, ~170 N along the racks, and a full box's load pushes back 130-150 N, so the box crawled
// in at ~0.05 m/s (3.0 s to their 6.3in); at 3.5 V it's at its 60 A limit (~220 N) and comes in
// in 1.7 s; more volts give no more (tools/mechlab1114.mjs dump, AGITATE_V)
export const SLIDE = { out: 0.298, dir: [-0.988, 0.152], pinion: 0.0127, ratio: 3.6, mass: 6, perRot: (Math.PI * 0.0254) / 3.6, pull: -3.5, theirPull: -2 };
// their slide positions (IntakeConstants, motor turns: "inches")
export const SLIDE_POS = { drop: 13.55, zero: 13.5, out: 13.4, redeploy: 13.0, rollersOff: 10.8, agitateMin: 6.3, outCheck: 9.0, tol: 0.025 / 0.0254 };
export const slideTurns = (ext) => ext / SLIDE.perRot;
// the kicker bar: on arms on a pin on the box's side plates (0.4513, 0.196 as exported), its 1.25in
// tube 0.153 m under the pin, hanging with the box out; coming in, the bumper swings it up and
// forward (to 90 deg)
export const KICKER = { pin: [0.4513, 0.196], len: 0.153, barR: 0.016, max: Math.PI / 2 };
// the bumper: their CAD's 1/2in wood (0.317-0.329 m out, 0.048 to 0.172 m up) and 2.5in of foam on
// it, to the side bumpers' faces (0.399 m + 2.5in); the foam isn't in the CAD
export const BUMPER = { x0: 0.3294, x1: 0.3294 + 2.5 * 0.0254, y0: 0.0476, y1: 0.1721, hw: 0.399 + 2.5 * 0.0254 };
// the box's front panel: the front of its clear polycarbonate (S26-IN-P322, 1/16in), its inner face
// 0.6044 m out as exported, from its bottom edge 0.2692 m up (on the 5/8in bottom rail S26-IN-P306;
// under that the 5.3 cm slot down to the 1.25in bottom front tube S26-IN-P305, open in the CAD) to
// the box's top at 0.701 m, and across the box to its side walls' outer faces (0.3766 m either side)
export const PANEL = { x0: 0.6044, x1: 0.606, y0: 0.2692, y1: 0.701, hw: 0.3766 };
// the blanket: the box's top (S26-IN-P330, 0.115 to 0.598 m out as exported, 0.701 up, 0.74 m
// across), red plush tied round its edges; FUEL under it push it up (bulge, an estimate), and past
// that it gives as a taut cloth does (tread: 6 kN/m, an estimate; at the 1.5 kN/m of 1678's and
// 4414's nets the intake squeezed FUEL out over it)
export const BLANKET = { x0: 0.115, x1: 0.598, y: 0.701, hw: 0.37, bulge: 0.05, tread: 6000 };
// their hood CANrange (HOOD_CANRANGE: proximity 0.45 m, 10 deg field of view; not in the CAD):
// at the hood, looking down the elevator (an estimate: their RunIntakeBallevator "stores balls in
// the ballevator until the hood CANrange is triggered"): a FUEL up the elevator within its 0.45 m
// (its center over 0.33 m up, from the hood's ~0.72)
export const RANGE = { at: [-0.122, 0.33], half: 0.06 };
// the preload, eight with the box in, staged in the indexer as their robot's loaded (robot frame;
// they settle): two up the elevator side by side, where their RunIntakeBallevator stages FUEL
// (until the hood CANrange sees one; the elevator's rollers are 14.2 cm apart, so they're pressed
// 2 mm each side, as any FUEL there is), three on the floor's slope just in front of the squish
// wheels, staged against them (3 mm clear: they roll 3 cm on into the valley and rest on the
// wheels), and three on those, between them and the intake's roller (with the box in it stands in
// the hopper's front). (The three staged in the valley started 2-3 cm into the squish wheels; out
// of it, they take the room the three in front of them had.)
export const PRELOAD = [
  [-0.122, 0.4, -0.1], [-0.122, 0.4, 0.1],
  [0.082, 0.193, -0.153], [0.082, 0.193, 0], [0.082, 0.193, 0.153],
  [0.192, 0.3, -0.15], [0.192, 0.3, 0], [0.192, 0.3, 0.15],
];

const interp = (t, x) => {
  if (x <= t[0][0]) return t[0][1];
  for (let k = 1; k < t.length; k++) if (x <= t[k][0]) return t[k - 1][1] + ((t[k][1] - t[k - 1][1]) * (x - t[k - 1][0])) / (t[k][0] - t[k - 1][0]);
  return t[t.length - 1][1];
};
// their shot tables (ShootMoveLUT: distance from the robot's center to the target, m -> motor
// RPM, hood angle; interpolated, clamped at the ends); aimShooter asks for 50 RPM under the table.
// (In play the RPMs come from SIM_HUB and SIM_PASS, below)
export const HUB_LUT = [[1.583, 2150, 74.4], [1.713, 2225, 74.4], [1.915, 2325, 74.4], [2.115, 2300, 72.5], [2.29, 2360, 72.5], [2.353, 2375, 70.5], [2.53, 2425, 70.5],
  [2.73, 2450, 69.5], [2.93, 2520, 69.0], [3.02, 2565, 69.0], [3.22, 2570, 68.5], [3.35, 2620, 67.5], [3.5, 2675, 67.5], [3.67, 2720, 67.5], [3.8, 2770, 65.5],
  [3.9, 2820, 65.5], [4.02, 2845, 65.5], [4.07, 2860, 65.5], [4.11, 2870, 65.5], [4.25, 2875, 65.5], [4.46, 2920, 65.5], [4.8, 2985, 65.0], [4.95, 3025, 65.0],
  [5.15, 3100, 65.0], [5.35, 3150, 65.0]];
export const PASS_LUT = [[3.539, 1900, 55], [4.714, 2400, 55], [6.37, 2800, 55], [7.7, 3200, 55], [9.25, 3600, 55], [12.28, 4750, 50], [14.02, 5150, 50]];
// Their table re-found by shooting, as they found theirs ("Shooter Tuning", their CAD thread):
// their distances and hood angles, and the motor RPM at which the sim's FUEL come down through the
// HUB's rim height over its center (passes: on the carpet at the row's distance), from
// tools/mechlab1114.mjs tune. Their own RPMs put every shot 0.4-1.5 m short here: the sim's FUEL
// leave at 0.86 of the flywheel's surface speed (between it and the hood rollers' 0.74, as the
// CAD's gearing has them) where their table needs about 0.96 in the sim's drag-only flight (it
// has no Magnus lift for the backspin the slower hood rollers put on). So it asks 11-23% more up
// to 4.5 m; past that, 65 deg needs more than the flywheel's grip gives easily (the FUEL slip above
// ~3500 RPM and leave at no more than ~10 m/s): 4600-5850. Passes past 7.7 m are out of reach at
// their 50-55 deg: flat out (5800).
export const SIM_HUB = [[1.583, 2388, 74.4], [1.713, 2517, 74.4], [1.915, 2741, 74.4], [2.115, 2674, 72.5], [2.29, 2812, 72.5], [2.353, 2623, 70.5], [2.53, 2727, 70.5],
  [2.73, 2848, 69.5], [2.93, 2930, 69], [3.02, 2982, 69], [3.22, 3063, 68.5], [3.35, 3003, 67.5], [3.5, 3183, 67.5], [3.67, 3356, 67.5], [3.8, 3125, 65.5], [3.9, 3172, 65.5],
  [4.02, 3237, 65.5], [4.07, 3264, 65.5], [4.11, 3285, 65.5], [4.25, 3361, 65.5], [4.46, 3495, 65.5], [4.8, 4604, 65], [4.95, 4919, 65], [5.15, 5242, 65], [5.35, 5850, 65]];
export const SIM_PASS = [[3.539, 2193, 55], [4.714, 2606, 55], [6.37, 3155, 55], [7.7, 5238, 55], [9.25, 5850, 55], [12.28, 5850, 50], [14.02, 5850, 50]];
// what their aimShooter asks for at distance d (their table, or with THEIR_RPM=1 in the
// environment their own RPMs)
const THEIR_RPM = typeof process !== 'undefined' && !!process.env?.THEIR_RPM;
export function shotFor(kind, d) {
  const t = kind === 'pass' ? (THEIR_RPM ? PASS_LUT : SIM_PASS) : THEIR_RPM ? HUB_LUT : SIM_HUB;
  const rpm = interp(t.map((r) => [r[0], r[1]]), d), hood = interp(t.map((r) => [r[0], r[2]]), d);
  return { rpm: rpm - 50, hood: Math.max(SHOOTER.hoodMin, Math.min(SHOOTER.hoodMax, hood)) };
}

// ------------------------------------------------------------------ the drives
// (Phoenix 6's default supply limit, 70 A, where their configs set only the stator's)
const SUPPLY = 70;
export function build1114() {
  // the intake roller: positive turns it inward (its bottom runs rearward)
  const intake = new Train({ name: 'intake rollers', motor: new Motor('krakenX60', { count: 2, statorLimit: 85, supplyLimit: SUPPLY }) });
  const roller = intake.add(new Shaft({ name: 'intake 2in roller (silicone)', inertia: 3.2e-4 }), -12 / 30);
  // the floor: positive carries FUEL back along it and up the tower's foot (its rollers' tops run
  // rearward); the squish wheels the other way (their bottoms rearward, their fronts down)
  const floor = new Train({ name: 'floor (hopper)', motor: new Motor('krakenX60', { count: 2, statorLimit: 70, supplyLimit: SUPPLY }) });
  const rollers = FLOOR.rollers.map((_, i) => floor.add(new Shaft({ name: `floor roller ${i + 1}`, inertia: 5e-5 }), 12 / 24));
  const squish = floor.add(new Shaft({ name: 'squish wheels (3in 45A)', inertia: 3.5e-4 }), -(12 / 24) * (18 / 30) * (20 / 44));
  // the side rollers: positive turns their inner faces down, into the floor's channel (the
  // direction is an estimate)
  const sideT = new Train({ name: 'side rollers', motor: new Motor('krakenX44', { statorLimit: 70, supplyLimit: SUPPLY }) });
  const side = [sideT.add(new Shaft({ name: 'left side roller', inertia: 7e-5 }), 12 / 64), sideT.add(new Shaft({ name: 'right side roller', inertia: 7e-5 }), -12 / 64)];
  // the elevator: positive lifts FUEL between its columns
  const elev = new Train({ name: 'ball elevator', motor: new Motor('krakenX60', { statorLimit: 70, supplyLimit: SUPPLY }) });
  const front = ELEV.front.map((_, i) => elev.add(new Shaft({ name: `elevator front roller ${i + 1}`, inertia: 5e-5 }), -12 / 24));
  const back = ELEV.back.map((_, i) => elev.add(new Shaft({ name: `elevator back roller ${i + 1}`, inertia: 5e-5 }), (12 / 24) * (30 / 26)));
  // the roller wall: active (the team), belted together; the CAD shows nothing driving it, so here
  // a belt off the elevator's front column at half its
  // speed, its face down at 2.0 m/s while the elevator runs (an estimate: a full box dumps at 16
  // FUEL/s with its face at 1.6-2.4 m/s, 15 at 1 and 1.2, 14 at 2.9 and 3.9, 13 free)
  const wall = elev.add(new Shaft({ name: 'roller wall', inertia: 1.5e-4 }), -12 / 24 / 2);
  // the shooter: positive runs the flywheel's back up (FUEL up its back and forward over its top),
  // the hood rollers the other way
  const S = SHOOTER;
  // (their shooter's VelocityVoltage has FOC off: trapezoidal commutation)
  const shooter = new Train({ name: 'shooter', motor: new Motor('krakenX60Trap', { count: 4, statorLimit: S.stator, supplyLimit: S.supply }) });
  const fly = shooter.add(new Shaft({ name: 'flywheel (3in, grip tape)', inertia: S.flyJ }), -S.flyPerMotor);
  shooter.add(new Shaft({ name: '4in brass flywheel', inertia: S.brassJ }), S.brassPerMotor);
  // (the jackshaft's and the motors' gears' inertia only: the train's drag stays the three shafts'
  // it was tuned with)
  shooter.add(new Shaft({ name: 'jackshaft (36t, 24ts)', inertia: S.jackJ, drag: 0 }), S.jackPerMotor);
  shooter.add(new Shaft({ name: 'motors\' 50t gears and 36t', inertia: S.motorJ, drag: 0 }), 1);
  const hood = shooter.add(new Shaft({ name: 'hood rollers', inertia: S.hoodJ }), S.flyPerMotor * S.hoodPerFly);
  // the box's slide: the racks' pinions (positive runs the box out); what slides seen at the
  // pinion; the racks' friction (an estimate: enough that the box doesn't run out down its racks
  // on its own with the X44 coasting)
  const slide = new Racks({ name: 'slide pinions', motor: new Motor('krakenX44', { ratio: SLIDE.ratio, statorLimit: 60, supplyLimit: SUPPLY }), inertia: SLIDE.mass * SLIDE.pinion ** 2, drag: 0.05, friction: 0.15 });
  // the passive side rollers: free (their bearings' running friction an estimate)
  const passive = PASSIVE.at.flatMap(() => [-1, 1]).map((_, i) => new Shaft({ name: `passive side roller ${i + 1}`, inertia: 4e-6, drag: 2e-5, friction: 0.002 }));
  return { intake, roller, floor, rollers, squish, sideT, side, elev, front, back, wall, passive, shooter, fly, hood, slide };
}

// the wheels (robot frame, as exported; the intake's and hood's posed with placeIntake / placeHood)
export function wheels1114(t) {
  const Z = [0, 0, 1], X = [1, 0, 0];
  const floor = FLOOR.rollers.map(([x, y], i) => new Wheel({ name: `${t.rollers[i].name} (grip tape)`, shaft: t.rollers[i], center: [x, y, 0], axis: Z, radius: FLOOR.r, width: FLOOR.width, mu: MU.gripTape, tread: 40000 }));
  const squish = SQUISH.z.map((z) => new Wheel({ name: 'squish wheel (3in 45A)', shaft: t.squish, center: [...SQUISH.at, z], axis: Z, radius: SQUISH.r, width: SQUISH.w, mu: MU.compliant, tread: 5000 }));
  const side = [-1, 1].map((s, i) => new Wheel({ name: `${t.side[i].name} (cat tongue)`, shaft: t.side[i], center: [SIDE.x, SIDE.y, s * SIDE.z], axis: X, radius: SIDE.r, width: SIDE.len, mu: MU.gripTape, tread: 40000 }));
  const col = (pts, shafts, name) => pts.map(([x, y], i) => new Wheel({ name: `${name ?? shafts[i].name} (grip tape)`, shaft: shafts[i] ?? shafts, center: [x, y, 0], axis: Z, radius: ELEV.r, width: ELEV.width, mu: MU.gripTape, tread: 40000 }));
  const elev = [...col(ELEV.back, t.back), ...col(ELEV.front, t.front)];
  const wall = ELEV.wall.map(([x, y], i) => new Wheel({ name: `roller wall ${i + 1} (cat tongue)`, shaft: t.wall, center: [x, y, 0], axis: Z, radius: ELEV.r, width: ELEV.width, mu: MU.gripTape, tread: 40000 }));
  const passive = PASSIVE.at.flatMap(([x, z]) => [-1, 1].map((sg) => [x, sg * z])).map(([x, z], i) => new Wheel({ name: `${t.passive[i].name} (5/8in)`, shaft: t.passive[i], center: [x, PASSIVE.y, z], axis: [0, 1, 0], radius: PASSIVE.r, width: PASSIVE.len, mu: 0.5, tread: 40000 }));
  const intake = [new Wheel({ name: 'intake 2in roller (silicone)', shaft: t.roller, center: [...INTAKE.at, 0], axis: Z, radius: INTAKE.r, width: INTAKE.width, mu: MU.silicone, tread: 20000 })];
  intake.forEach((w) => { w.home = w.center.slice(); });
  const S = SHOOTER;
  const fly = new Wheel({ name: 'flywheel (3in, grip tape)', shaft: t.fly, center: [...FLY, 0], axis: Z, radius: S.fly.r, width: S.fly.width, mu: MU.gripTape, tread: 40000 });
  const hood = S.hood.map((h, i) => new Wheel({ name: `hood 1.5in roller ${i + 1} (grip tape)`, shaft: t.hood, center: [...h, 0], axis: Z, radius: S.hoodR, width: S.hoodWidth, mu: MU.gripTape, tread: 40000 }));
  hood.forEach((w) => { w.home = w.center.slice(); });
  return { floor, squish, side, passive, elev, wall, intake, fly, hood };
}
// rotate a point (x, y) about (cx, cy) by a (rad)
const rot = ([x, y], [cx, cy], a) => { const c = Math.cos(a), s = Math.sin(a), dx = x - cx, dy = y - cy; return [cx + dx * c - dy * s, cy + dx * s + dy * c]; };
// the box's offset from the CAD's pose (all the way out) with it ext m out along its racks
export function slideOffset(ext) { const d = SLIDE.out - ext; return [SLIDE.dir[0] * d, SLIDE.dir[1] * d]; }
// the racks' ends: with the box all the way out (or in) its pinions can't turn it on past there, so
// what the load pushes it with that way goes into the stop (set before each step: the box against
// its stop is as fixed as the robot, not a 6 kg box recoiling from the load within the step; that
// recoil let a hard knock press its load further into its front)
export function slideStops(shaft, ext) { shaft.lo = ext <= 1e-9 ? 0 : -Infinity; shaft.hi = ext >= SLIDE.out - 1e-9 ? 0 : Infinity; }
// the slide's pinions: a Shaft whose racks' friction holds it still against small pushes as the
// shared one's does, and its stops (lo, hi: slideStops) as well (the shared Shaft drops its
// friction against a stop)
class Racks extends Shaft {
  integrate(dt, V) {
    const { lo, hi } = this;
    this.lo = -Infinity; this.hi = Infinity;
    super.integrate(dt, V);
    Object.assign(this, { lo, hi });
    this._omega = Math.min(hi, Math.max(lo, this._omega));
  }
  impulse(L) {
    const { lo, hi } = this;
    this.lo = -Infinity; this.hi = Infinity;
    super.impulse(L);
    Object.assign(this, { lo, hi });
    this._omega = Math.min(hi, Math.max(lo, this._omega));
  }
}
export function placeIntake(wheels, ext) {
  const [dx, dy] = slideOffset(ext);
  for (const w of wheels) w.center = [w.home[0] + dx, w.home[1] + dy, w.home[2]];
}
// the kicker's swing (rad, up and forward from hanging) with the box ext m out: it hangs until its
// bar meets the bumper's face, then rides up it and over its top corner
export function kickerRot(ext) {
  const [dx, dy] = slideOffset(ext), px = KICKER.pin[0] + dx, py = KICKER.pin[1] + dy, K = KICKER.len;
  const xb = BUMPER.x1 + KICKER.barR, yc = BUMPER.y1 + KICKER.barR;
  if (px >= xb) return 0;
  const a1 = Math.asin(Math.min(1, (xb - px) / K)), a2 = Math.acos(Math.max(-1, Math.min(1, (py - yc) / K)));
  return Math.min(KICKER.max, a1, a2);
}
// the hood's turn about the flywheel's axis from the CAD's pose (rad, back and down positive) at
// their angle deg: 1:1 with their angle (their angle is the shot's elevation in their own sim)
export function hoodRot(deg) { return ((deg - SHOOTER.hoodCad) * Math.PI) / 180; }
export function placeHood(wheels, deg) {
  const a = hoodRot(deg);
  for (const w of wheels) { const [x, y] = rot(w.home, FLY, a); w.center = [x, y, 0]; }
}
// the shot's elevation at their angle deg by the CAD's geometry: square to the line from the
// flywheel's axis to the last hood roller (45.6 deg in the CAD's pose)
export function hoodElevation(deg) {
  const [hx, hy] = SHOOTER.hood[2];
  return (Math.atan2(hy - FLY[1], hx - FLY[0]) * 180) / Math.PI - 90 + (hoodRot(deg) * 180) / Math.PI;
}

// the ball path's surfaces from their CAD (js/mech/path1114.js): the body; the box on its racks (it
// slides, on the slide's pinions); the kicker on its pin (it rides the box, and swings up as the
// box comes in); the hood on the flywheel's axis; the blanket over the box (a soft sheet); and the
// bumper (solid foam: a Block). FUEL slide on the CAD's polycarbonate and aluminum at 0.25 (an
// estimate, 2910's and 604's: at 0.3 a full box dumped at 12 FUEL/s, at 0.2 at 15), and on the
// kicker's "safety walk" grip at 1.0
// The box's front panel is its own solid (FrontPanel), not the mesh's: the mesh's panel is two
// sheets 1 cm apart (its 1/16in snapped to the 1 cm grid), and a FUEL pressed far enough into it
// that its center passed their middle was pushed out the far side, out of the box (in play, the
// packed load thrown at it in a hard knock; on the bench one FUEL pressed on it with 800 N, or
// thrown at it at 20 m/s: tools/mechlab1114.mjs panel). So the mesh's panel (every triangle on
// those two sheets, 0.60 and 0.61 m out, from its bottom edge up) is left out here.
const panelTri = (V, I, t) => [0, 1, 2].every((k) => { const x = V[I[t * 3 + k] * 3], y = V[I[t * 3 + k] * 3 + 1]; return x >= 600 && x <= 610 && y >= 270; });
export function pathSurfaces1114(mesh, t, { mu = 0.25 } = {}) {
  const s = (k, pos, m = mu, keep = () => true) => {
    const I = mesh[k].i, n = I.length / 3, idx = [];
    for (let j = 0; j < n; j++) if (keep(mesh[k].v, I, j)) idx.push(I[j * 3], I[j * 3 + 1], I[j * 3 + 2]);
    return new Surface({ name: `${k} (CAD)`, vertices: mesh[k].v.map((x) => x / 1000), indices: idx, mu: m, pos });
  };
  const body = s('body', [0, 0, 0]);
  const slide = s('slide', [0, 0, 0], mu, (V, I, j) => !panelTri(V, I, j)).onSlide(t.slide, [-SLIDE.dir[0], -SLIDE.dir[1], 0]);
  slide.slideK = SLIDE.pinion;
  const panel = new FrontPanel({ mu }).onSlide(t.slide, [-SLIDE.dir[0], -SLIDE.dir[1], 0]);
  panel.slideK = SLIDE.pinion;
  const kicker = s('kicker', [...KICKER.pin, 0], 1.0);
  const hood = s('hood', [...FLY, 0]);
  const blanket = new Blanket();
  const bumper = new Block({ name: 'bumper', ...BUMPER, mu: BUMPER_MU, tread: 20000 });
  return { body, slide, panel, kicker, hood, blanket, bumper };
}
export function placeSlide(s, ext) {
  const [dx, dy] = slideOffset(ext);
  s.slide.setPose([dx, dy, 0]);
  s.panel.setPose([dx, dy, 0]);
  s.kicker.setPose([KICKER.pin[0] + dx, KICKER.pin[1] + dy, 0], 0, kickerRot(ext));
  s.blanket.place(dx, dy);
}
// The box's front panel (PANEL, in the box's frame as exported; it slides with the box): a solid
// sheet as thick as the CAD's polycarbonate that a FUEL can't pass through. Against its face a
// FUEL is pushed back to the side it came from, however far the load has pressed it in: each
// FUEL's side is remembered from step to step (as a collision engine's continuous detection
// does), so one whose center a hard knock has pressed past the sheet is still pushed back into
// the box (its dent deeper than its radius), not out the far side. Past its edges (under its
// bottom edge, along the slot over the bottom tube) it's a rounded edge like any other.
class FrontPanel extends Surface {
  constructor({ mu }) {
    const { x0, x1, y0, y1, hw } = PANEL, V = [], I = [];
    for (const z of [-hw, hw]) for (const [x, y] of [[x0, y0], [x1, y0], [x1, y1], [x0, y1]]) V.push(x, y, z);
    for (const [a, b, c, d] of [[1, 2, 6, 5], [2, 3, 7, 6], [0, 1, 5, 4], [0, 4, 7, 3], [0, 3, 2, 1], [4, 5, 6, 7]]) I.push(a, b, c, a, c, d);
    super({ name: 'front panel (CAD)', vertices: V, indices: I, mu });
    this.side = new WeakMap(); // FUEL (body) -> { s: -1 in the box, 1 out front, p: where it was }
  }
  touches(p, who) {
    const [x, y, z] = p, { x0, x1, y0, y1, hw } = PANEL;
    const span = y >= y0 && y <= y1 && Math.abs(z) <= hw;
    // (which side: the one it was on last step, if it was here then (not a FUEL that's been
    // elsewhere since: shot, or round the panel's edge), else the one its center's on)
    let s = x < (x0 + x1) / 2 ? -1 : 1;
    const m = who && this.side.get(who);
    if (span && m && Math.abs(x - m.p[0]) + Math.abs(y - m.p[1]) + Math.abs(z - m.p[2]) < 0.2) s = m.s;
    if (who) { if (m) { m.s = s; m.p[0] = x; m.p[1] = y; m.p[2] = z; } else this.side.set(who, { s, p: [x, y, z] }); }
    let dist, n;
    if (span) { dist = s < 0 ? x0 - x : x - x1; n = [s, 0, 0]; }
    else {
      const cx = Math.min(x1, Math.max(x0, x)), cy = Math.min(y1, Math.max(y0, y)), cz = Math.min(hw, Math.max(-hw, z));
      const dx = x - cx, dy = y - cy, dz = z - cz;
      dist = Math.hypot(dx, dy, dz);
      n = dist > 0 ? [dx / dist, dy / dist, dz / dist] : [s, 0, 0];
    }
    return dist < R ? [{ depth: R - dist, n }] : [];
  }
}
// The blanket: red plush tied round the box's top edges (their reveal, after 1124's 2008 "magic
// blanket"), soft: a FUEL under it presses it up, as far as BLANKET.bulge at its middle (an
// estimate), past that only as far as its tread gives, and nothing presses the load back down
// harder than that (as 1678's and 4414's nets). In the robot's frame, so its pose stays put.
class Blanket extends Surface {
  constructor() {
    super({ name: 'blanket', vertices: [0, 0, 0, 0.01, 0, 0, 0, 0, 0.01], indices: [0, 1, 2], mu: 0.5, tread: BLANKET.tread });
    this.place(0, 0);
  }
  place(dx, dy) { Object.assign(this, { x0: BLANKET.x0 + dx, x1: BLANKET.x1 + dx, y0: BLANKET.y + dy }); }
  at(x, z) {
    const L = this.x1 - this.x0, u = (x - this.x0) / L, v = (z + BLANKET.hw) / (2 * BLANKET.hw), B = BLANKET.bulge;
    const su = Math.sin(Math.PI * u), sv = Math.sin(Math.PI * v);
    return [this.y0 + B * su * sv, (B * Math.PI * Math.cos(Math.PI * u) * sv) / L, (B * su * Math.PI * Math.cos(Math.PI * v)) / (2 * BLANKET.hw)];
  }
  touches(p) {
    const [x, y, z] = p;
    if (x <= this.x0 || x >= this.x1 || Math.abs(z) >= BLANKET.hw) return [];
    const [h, hx, hz] = this.at(x, z), l = Math.hypot(hx, 1, hz), dist = (h - y) / l;
    if (dist >= R || dist < -R) return [];
    return [{ depth: R - dist, n: [hx / l, -1 / l, hz / l] }];
  }
}
