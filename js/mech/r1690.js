// 1690's Kepler as physical mechanisms (js/mech/mechanism.js), from their CAD: the X_T release their
// CAD thread on Chief Delphi links ("1690-26-Top Level Closed.x_t" and "...Open.x_t", in their public
// Drive folder), imported into Onshape and exported as STEP (cad/README.md). The X_T carries no mates,
// so the axes below are the parts' own cylinders, read with OpenCascade (tools/step-axes.py); the
// moves are the Closed and Open poses compared part by part; the surfaces FUEL touch are the CAD's
// (tools/path-mesh-1690.py -> js/mech/path1690.js). Every drive is traced from its motor through each
// gear, pulley and belt to what it turns: tooth counts from the BOM's names, from each belt's own
// length and wrap radii (its pulleys' pitch circles: the belt's length then checks to 0.1 mm), and from
// the gears' outside diameters and center distances (20 DP: (N1 + N2) / 40 in apart). The speeds are
// what the team posted in their CAD release thread, their motors' RPS (the reductions below put any
// mechanism at those RPS far past a Kraken's free speed): the hopper floor at 75 RPS and the top and
// bottom belts at 95 RPS scoring (85 / 100 passing), the vertical kicker at 60 RPS, the "feeder
// wheels" at 60% of the shooter wheel's surface speed, at least 5 m/s. Their code isn't public
// ("software is our competitive edge"). Everything marked is an estimate. Robot frame: x forward (the
// intake end), y up, z to its right; the CAD's X is the robot's left, Y forward, Z down.
//
// The ball path and its drives:
//   - intake (sub-assembly 200): a carriage that slides out of the frame on its black side bars,
//     0.3196 m and 12.5 deg down from "Closed" (inside the frame: travel) to "Open" (the two X_Ts'
//     carriage parts all moved by that one vector), driven by an X44 through a MAXPlanetary 4:1 and an
//     18T:28T 5M belt to pinions on a cross shaft. On it, two silicone-wrapped polycarbonate rollers
//     and the lattice U-frame that is the hopper's front and sides out over the intake: an X60 on the
//     carriage, 14T:28T (46T 5M belt) to the top roller and 24T:24T (68T 5M) on to the bottom one.
//     Under the rollers, the "kicker bar": an aluminum tube on two arms turning on a pin fixed to the
//     side plates, folded up behind the intake in Closed; in Open it has fallen 163.0 deg about the pin
//     (its tube at the same 0.200 m from it), held down by surgical tubing ("tangent to the intake
//     plate"), so FUEL are pinched between it and the rollers and climb the bumper.
//   - the hopper: its polycarbonate side and back panels (aluminum corner brackets) and a thin
//     polycarbonate lattice plate over the top, 0.55 m up, that the load bows up (PLATE)
//   - the roller floor (300): nine silicone-wrapped rollers sloping down from front to back (in Open
//     its front five are 2.1 deg lower, the floor's front hinged at the sixth). Its X60 (low at the back
//     right) drives up through 2:1 (their video; the CAD has a 16T:58T gear stage), 24T:24T (55T 5M) up to a shaft over the floor's
//     back, which drives the back top roller (28T:28T, 75T 5M) and, through 34T:34T gears (turning it
//     back), 18T:18T (62T 5M) down and 28T:28T (75T 5M) to the back floor roller; roller to roller by
//     60T 3M belts on 28T pulleys, but 30T on the second roller and three 29T:32T steps in front, so the
//     front rollers turn at 0.69 of the back one's speed: FUEL speed up toward the back. The two top
//     rollers over the floor's back turn the other way (28T:30T between them).
//   - the belts along the back: two 30 mm HTD belts (180T and 160T 5M) on 24T pulleys on fore-aft
//     axles, each its own X44 through 1.5:1 (their video: 8 m/s; the CAD has 16T:48T on 89T and 80T 3M belts). The bottom belt's top run (0.159
//     m up) and the top belt's bottom run (0.302 m) pinch FUEL between them, against the backing plate
//     behind (polycarbonate under PTFE film), and carry them to the robot's left.
//   - the vertical kicker: a 1in tube standing at the belts' left end in front of them (an X44,
//     16T:40T on a 120T 3M belt), "it spins clockwise (toward the belts)".
//   - the "bottom and top kicker" ("the polycarb ramp replaced by flex wheels"), the team's "feeder
//     wheels": off the belts' end, three stacks of 2.25in compliant wheels on fore-aft shafts step up
//     to the left, and two 2in Stealth wheels flank the turret's bore (left and right), squeezing FUEL
//     up into it. One X60: an 18T pulley on a 140T 5M belt round both top-kicker shafts' 28T pulleys
//     (and an idler); the right one turns the left one's way back through 28T:28T gears; the left one's
//     22T pulleys drive the three compliant stacks 22T:22T (50T 5M belts) one to the next.
//   - the turret (400): on an 8in bearing (its opening 0.176 m across), turned by an X60 low in the
//     frame: 12T:60T gears, 18T:28T 5M (a 45T belt fits the centers to 0.1 mm; the X_T leaves it out)
//     up a vertical shaft to a 14T 10 DP pinion on the turret's 90T ring: 50:1. FUEL come up under the
//     4in shooter wheel (custom, swerve tread, two X60s geared 28T:42T, no flywheel), which turns them
//     up its back face into the hood's wheels: 2in Stealth wheels with a 2in roller under them on the
//     hood's lattice arms, which turn about the wheel's axle (an X44 through a VersaPlanetary 4:1 and
//     19.333:1 gears, their thread). Those wheels have no motor: the wheel's 42T gear drives a 30T on
//     the hood's arm, its 21T pulley a 19T on the Stealth wheels' shaft (110T 5M), and 40T:40T (85T 3M)
//     the roller: 1.55 times the wheel's turns, the other way, 0.78 of its surface speed. The FUEL leave
//     the pinch along the wheel's face at the hood's angle ("the difference between hood angle and ball
//     exit angle was negligible"), between the "hollow hood" that guides them from the sides.
import { IN, FUEL } from '../constants.js';
import { Block, Motor, Shaft, Surface, Train, Wheel, BUMPER_MU } from './mechanism.js';
import PATH from './path1690.js';

const DEG = Math.PI / 180, R = FUEL.radius;
// what each grips FUEL with (estimates): silicone wraps on the intake and floor rollers (grippy), the
// compliant and Stealth wheels (35A), the shooter wheel's swerve tread, the belts' rubber, bare
// polycarbonate and aluminum, the PTFE film on the backing plate (slick: "to reduce friction as much
// as possible"), the printed hood (PLA), the net's cord
export const MU = { silicone: 1.2, compliant: 1.3, tread: 1.3, belt: 1.0, polycarb: 0.5, alu: 0.4, ptfe: 0.1, pla: 0.3, wall: 0.25, net: 0.25 };
// and how far each gives where a FUEL presses it (N/m; the belts ride on the CAD's 3 mm aluminum
// beds, 311-02A/04A and 312-07A/08A, so they give no more than a wrap on a tube: a free span's 3000
// let FUEL press through them onto the beds' edges, and drag there; BELTK=3000 for that)
export const TREAD = { silicone: 20000, compliant: 12000, tread: 30000, belt: Number(globalThis.process?.env?.BELTK ?? 20000), plate: 100000 }; // (the top plate where a FUEL presses it, its bow aside (PLATE): the FUEL's own foam gives)

// ------------------------------------------------------------------ the layout (robot frame, m: the CAD's)
export const HALF_L = 12.5 * IN, HALF_W = 14.5 * IN; // 25 x 29in frame
// the hopper: its side panels' inside faces (the bumper assembly's panels), the lattice frame's top
export const WALL = { left: -0.358, right: 0.358, top: 0.551 };
// the roller floor: [x, y] of each roller's axle, front to back (their 1.3in silicone-wrapped tubes),
// as the Open X_T has them (the front five 2.1 deg lower than in Closed); the back three stop short of
// the left side (the vertical kicker and the feeder are there): z from each one's left end to its
// right. Its X60: 2:1 to the drive shaft over the floor's back (ratio: the team's video; the CAD's 16T:58T,
// 3.6:1, is older, FLOORR=3.625 for it), which turns the back roller
// 1:1 (through 34T:34T, 18T:18T and 28T:28T); each roller's turns per the drive shaft's (k: the 60T 3M
// belts' 28T, 30T and 29T:32T pulleys)
export const FLOOR = {
  r: 0.0165,
  at: [[0.2810, 0.1817], [0.2335, 0.1749], [0.1860, 0.1681], [0.1422, 0.1618], [0.0984, 0.1556], [0.0546, 0.1493], [0.0071, 0.1425], [-0.0409, 0.1425], [-0.0874, 0.1425]],
  z0: [-0.322, -0.322, -0.322, -0.322, -0.322, -0.322, -0.132, -0.132, -0.132], z1: [0.31, 0.298, 0.298, 0.298, 0.298, 0.298, 0.298, 0.276, 0.276],
  ratio: 2,
  k: [28 / 30 * (29 / 32) ** 3, 28 / 30 * (29 / 32) ** 3, 28 / 30 * (29 / 32) ** 3, 28 / 30 * (29 / 32) ** 2, 28 / 30 * 29 / 32, 28 / 30, 28 / 30, 28 / 30, 1],
};
// the two rollers over the floor's back: the drive shaft turns the back one 28T:28T, which turns the
// front one 28T:30T; they turn the other way from the floor (k, per the drive shaft's turns)
export const TOP = { at: [[-0.0409, 0.3191], [-0.0874, 0.3183]], r: 0.016, z: [-0.066, 0.291], k: [-28 / 30, -1] };
// The foam rolling over the floor's rollers loses some of each turn (its hysteresis: rolling
// resistance, ROLL.rr times what presses it there: 0.08, an estimate, about twice the carpet's,
// FUEL.rollDecel 0.35 m/s^2 being 0.036 g, as the foam wraps the 33 mm rollers). Without it a FUEL resting between two rollers turning at the same speed (the front three,
// and the three before the back one: the CAD's 1:1 belts) spins in place like an idler gear meshed
// with them, its surface matching both, and never moves: every contact without slip. Resisting that
// spin, the rollers keep dragging its bottom back, and it goes (floorRoll, after each step)
export const ROLL = { rr: 0.08 };
// the belts along the back: across their 30 mm (x), each one's run against the FUEL (its outer
// face's height, z from pulley to pulley), the belt's outer radius round its 24T pulleys and their
// pitch radius (the belt's speed per radian); X44s through 1.5:1 (the team's video: 8 m/s, which their
// 100 RPS gives through 1.5:1, 7.6 m/s at the 95 they score at; the CAD's 16T:48T, 3:1, would take 200
// RPS, past an X44's free speed, so it's older: BELTR=3 for it)
export const BELT = {
  x: [-0.149, -0.119], r: 0.0203, pitch: (24 * 0.005) / (2 * Math.PI), ratio: 1.5,
  bottom: { y: 0.1586, z: [-0.1158, 0.2742] }, // its top run (the 180T)
  top: { y: 0.3022, z: [-0.0423, 0.2977] },    // its bottom run (the 160T)
};
// the backing plate behind the belts (its PTFE face), the electronics box's face
export const BACK = { x: -0.2105, box: -0.043 };
// the vertical kicker: a 1in tube standing in front of the belts' left end (an X44, 16T:40T)
export const VKICK = { at: [-0.0155, -0.1525], y: [0.088, 0.367], r: 0.0125, ratio: 40 / 16 };
// the bottom kicker (2.25in compliant wheels, two 1in wide on each fore-aft shaft, [z, y]) and the top
// kicker (2in Stealth wheels either side of the turret's bore): the team's feeder wheels, one X60
// through 18T:28T (the 140T belt), every shaft at that speed (the 22T:22T belts, the 28T:28T gears)
export const FEED = {
  x: -0.134, w: 0.05,
  wheels: [[-0.1806, 0.1600], [-0.2358, 0.2031], [-0.2721, 0.2629]], r: 0.0286,
  top: [[-0.2837, 0.3319], [-0.0909, 0.3319]], topR: 0.0254,
  ratio: 28 / 18,
};
// the turret: its axis (the 8in bearing's), the opening up through it (its rings' bore), where the
// bearing is, the deck it turns on; its travel (+-200 deg, as the sim's config has it)
export const TURRET = { at: [-0.1340, -0.1872], bore: { r: 0.088, y: [0.36, 0.38] }, deck: 0.378 };
// the turret as the CAD has it (its shot's yaw, three.js's sense) and the hood (its upper feeder wheel's
// angle about the axle, from the shot's direction)
export const YAW0 = -23.99 * DEG, HOOD0 = 172.47;
// the shooter, in the turret's frame (s along the shot, y up, z across): the 4in wheel's axle (2in
// wide), the hood's feeder wheels (2in Stealth wheels over a 2in roller, both 2in wide, on the hood's
// arms 0.2068 m from the axle: the roller 18.8 deg below), the hood's travel as the shot's elevation
// (its steep end is the CAD's pose: 82.5 deg; the low end isn't in the CAD: 35, an estimate), the
// wheel's gearing (28T:42T); the hood's wheels' turns per the shooter wheel's (k: its 42T gear to a
// 30T on the hood's arm, 21T:19T, 40T:40T; the other way round). What turns, from the CAD's solids
// (their volumes and second moments about each shaft's axis; the X_T names no materials, so each part's
// is by its kind and CAD color, estimates): on the wheel's axle (the axle itself a dead 20 mm tube on
// 20x27x4 bearings) the tread band (432-03A, 103 cc, rubber 1200 kg/m^3: 2.70e-4 kg m^2), the hub
// (432-01A, 84 cc, aluminum: 2.39e-4) and the two 42T gears (aluminum: 3.4e-5): 0.42 kg, J 5.43e-4;
// the hood's Stealth wheels' shaft (two 2in Stealth wheels, 38 cc each at 1100 kg/m^3, a 40T GT2 and
// the 19T (aluminum), the 3/8 hex (steel), 14T gears, printed spacers): 4.18e-5; its 2in roller (the
// aluminum tube and sleeve, two 40T GT2 pulleys; it turns on its own bearings round the hood
// actuator's cross shaft): 7.09e-5; the 30T/21T shaft on the hood's arm (k 42/30): 9.0e-6. The hood
// itself: its X44 rides on the hood's arm, a VersaPlanetary (4:1, their thread: the CAD has its ring
// gear, not its planets) and a 1:1 bevel pair (442-02A/09A) turning a cross shaft along the roller's
// axis with a 12T pinion each end (442-04A and a "Spur gear (12 teeth)": 11.85 mm tip radius, module
// ~1.69) on racks cut in the turret's side plates (431-06A/07A) round the wheel's axle (pitch radius
// ~0.197 m): 19.3:1, their "19.333:1"; 77:1 in all
export const SHOOTER = {
  axle: [0.070, 0.464], r: 0.0502, w: 0.05,
  feeder: { R: 0.2068, r: 0.0254, w: 0.05, lower: 18.82, k: (42 / 30) * (21 / 19) },
  elMin: 35, elMax: HOOD0 - 90, ratio: 1.5, J: 5.43e-4, hoodJ: [4.18e-5, 7.09e-5], armJ: 9.0e-6,
  // the hood's X44: 4:1 VersaPlanetary, a 1:1 bevel and 12T pinions on racks round the axle (19.333:1,
  // their thread; the CAD's geometry: 19.3); its profile (estimates)
  hoodRatio: 4 * 19.333, hoodV: 350, hoodA: 3500,
  // the wheel's velocity loop's gain; ready: within 3% of the wheel's target surface speed, 1.5 deg
  // of hood (estimates)
  kP: 0.25, tolV: 0.03, tolHood: 1.5,
};
// the intake: the carriage's travel out (Closed -> Open: its parts' move between the two X_Ts), the
// rollers on it as exported (in), their X60's belting (14T:28T, then 24T:24T), the U-frame's front and
// half width out (its plates' outside), the kicker bar: its arms' pin, its tube as exported (folded
// up) and how far it falls (deg: Closed -> Open)
export const INTAKE = {
  slide: [0.3120, -0.0691], width: 0.647,
  rollers: [[0.2473, 0.2301, 0.021], [0.1993, 0.3290, 0.021]], ratio: 28 / 14,
  frame: { front: 0.617, sides: 0.362, top: 0.482 },
  // (the kicker bar: a fixed aluminum tube (not a roller: the user) on its two arms, which swing on
  // their pin. What turns: the tube (Part 162, 87.7 cc of aluminum, 0.200 m out), the two outer arms
  // (Parts 154/159, 22 cc each) and the inner brackets (Part 157, 16 cc each): 0.44 kg, its center
  // 0.173 m out along the tube's line, 0.015 kg m^2 about the pin. The surgical tubing isn't in the
  // CAD: 1/4in latex tubing (1/16in wall, ~36 N per unit strain), one a side stretched ~40% at the
  // arms' stop, pulling at ~5 cm from the pin: 1.4 N m of preload holding them down, 1.2 N m more
  // per radian they're lifted (estimates). rigid (KICKRIGID=1, headless): the arms posed by the
  // carriage, as before, for comparison)
  kick: { pivot: [0.2723, 0.1703], tube: [0.1499, 0.3283, 0.0127], fall: -163.0, width: 0.675, mass: 0.44, com: 0.173, J: 0.015, tubing: { preload: 1.4, rate: 1.2 }, rigid: typeof process !== 'undefined' && process.env?.KICKRIGID === '1' },
};
// the carriage's drive: its X44 through a MAXPlanetary 4:1 and 18T:28T (45T 5M) to a cross shaft
// whose 28T 20 DP pinions (0.7in pitch radius) run in the side bars' racks; what it moves (the
// carriage, its rollers, their X60 and the U-frame: ~4 kg, an estimate from its parts' volumes); its
// travel (Closed -> Open)
export const SLIDE = { pinion: 0.7 * IN, ratio: (4 * 28) / 18, mass: 4, travel: Math.hypot(...INTAKE.slide), dir: [INTAKE.slide[0] / Math.hypot(...INTAKE.slide), INTAKE.slide[1] / Math.hypot(...INTAKE.slide), 0] };
// the preload, as it starts (the carriage in, its rollers and the kicker bar folded up behind them
// filling the hopper's front: the old slots there started FUEL 5-7 cm into them, and the back row's
// left one in the corner funnel's wall, Part 234, landing behind it): three staged in the belts'
// channel, three on the floor between the top rollers over its back and the folded kicker bar (clear
// of the vertical kicker), two on top of those (clear of the energy chain's guard round the turret
// and the electronics box)
export const PRELOAD = [
  ...[-0.06, 0.09, 0.24].map((z) => [-0.135, 0.23, z]),
  ...[-0.07, 0.08, 0.23].map((z) => [0.05, 0.27, z]),
  ...[0.03, 0.18].map((z) => [0.06, 0.42, z]),
];
// The corner funnel (Part 234, a printed diagonal wall at the floor's front left, leaning in as it
// rises: at 0.2 m up from (0.18, -0.36) to (0.01, -0.18), at its top, 0.342 m, from (0.103, -0.36)
// to (0.031, -0.275)) leaves a pocket behind it, against the left side panel and the plate before
// the kickers (Part 252, x -0.05), open at the top, too small for a FUEL (it would have to squeeze
// ~15%) but a FUEL pressed down by the load gets in and sits on the floor's front rollers. A lid over
// it at the wall's top (the sim's: the CAD has none) keeps them out
export const FUNNEL_LID = { y: 0.342, pts: [[0.105, -0.358], [-0.05, -0.358], [-0.05, -0.178]] };

// ------------------------------------------------------------------ the drives
// (current limits aren't public: 80 A stator, 40 A supply, estimates; the shooter's 30 A supply, as
// its channels' 30 A breakers: "we used 30A breakers for the channels with the 14awg")
export function build1690() {
  const lim = { statorLimit: 80, supplyLimit: 40 };
  const intake = new Train({ name: 'intake rollers', motor: new Motor('krakenX60', { ratio: INTAKE.ratio, ...lim }) });
  const rollers = INTAKE.rollers.map((_, i) => intake.add(new Shaft({ name: `intake roller ${i ? 'top' : 'bottom'}`, inertia: 1.5e-4 }), -1));
  // (the kicker bar's arms on their pin: no motor; the tubing and their weight its load, the
  // carriage and their stop its stops: phys1690's moveKick)
  const kickArm = new Shaft({ name: 'kicker bar arms (pin)', inertia: INTAKE.kick.J, drag: 0.02 });
  // (the carriage on its racks: + out; the bars' rollers' friction, an estimate)
  const slide = new Shaft({ name: 'carriage racks (pinions)', motor: new Motor('krakenX44', { ratio: SLIDE.ratio, statorLimit: 40, supplyLimit: 30 }), inertia: SLIDE.mass * SLIDE.pinion ** 2, drag: 0.005, friction: 0.05 });
  const floor = new Train({ name: 'roller floor', motor: new Motor('krakenX60', { ratio: FLOOR.ratio, ...lim }) });
  FLOOR.at.forEach((_, i) => floor.add(new Shaft({ name: `floor roller ${i + 1}`, inertia: 1.2e-4 }), FLOOR.k[i]));
  const top = TOP.at.map((_, i) => floor.add(new Shaft({ name: `top roller ${i + 1}`, inertia: 1.2e-4 }), TOP.k[i]));
  const belts = ['bottom', 'top'].map((k) => new Shaft({ name: `${k} belt`, motor: new Motor('krakenX44', { ratio: BELT.ratio, ...lim }), inertia: 5e-5, drag: 4e-4 }));
  const vkick = new Shaft({ name: 'vertical kicker', motor: new Motor('krakenX44', { ratio: VKICK.ratio, ...lim }), inertia: 6e-5 });
  const feed = new Train({ name: 'bottom and top kicker', motor: new Motor('krakenX60', { ratio: FEED.ratio, ...lim }) });
  const feedShafts = [...FEED.wheels, ...FEED.top].map((_, i) => feed.add(new Shaft({ name: i < 3 ? `bottom kicker ${i + 1}` : `top kicker ${i - 2}`, inertia: 6e-5 }), 1));
  const shooter = new Train({ name: 'shooter', motor: new Motor('krakenX60', { count: 2, ratio: SHOOTER.ratio, statorLimit: 80, supplyLimit: 30 }) });
  const wheel = shooter.add(new Shaft({ name: '4in shooter wheel', inertia: SHOOTER.J, drag: 2e-4 }), 1);
  // (the hood's wheels on the shooter's train: geared to its wheel)
  const hoodWheels = ['upper', 'lower'].map((k, i) => shooter.add(new Shaft({ name: `hood wheel ${k}`, inertia: SHOOTER.hoodJ[i], drag: 1e-4 }), SHOOTER.feeder.k));
  // (and the 30T/21T shaft between, on the hood's arm)
  shooter.add(new Shaft({ name: 'hood 30T/21T shaft', inertia: SHOOTER.armJ }), 42 / 30);
  return { intake, rollers, kickArm, slide, floor, top, belts, vkick, feed, feedShafts, shooter, wheel, hoodWheels };
}

const Z = [0, 0, 1];
// the wheels of the ball path (robot frame; the intake's posed by placeIntake)
export function wheels1690(t) {
  const K = INTAKE.kick;
  const intake = [
    ...INTAKE.rollers.map(([x, y, r], i) => new Wheel({ name: `intake roller ${i ? 'top' : 'bottom'} (silicone)`, shaft: t.rollers[i], center: [x, y, 0], axis: Z, radius: r, width: INTAKE.width, mu: MU.silicone, tread: TREAD.silicone })),
  ];
  intake.forEach((w) => { w.home = w.center.slice(); });
  // (the floor's top runs back: +z axes; the rollers over it turn the other way, their undersides
  // running back; the intake's rollers pull FUEL in under the bottom one)
  const floor = FLOOR.at.map(([x, y], i) => new Wheel({ name: `floor roller ${i + 1} (silicone)`, shaft: t.floor.shafts[i], center: [x, y, (FLOOR.z0[i] + FLOOR.z1[i]) / 2], axis: Z, radius: FLOOR.r, width: FLOOR.z1[i] - FLOOR.z0[i], mu: MU.silicone, tread: TREAD.silicone }));
  const top = TOP.at.map(([x, y], i) => new Wheel({ name: `top roller ${i + 1} (silicone)`, shaft: t.top[i], center: [x, y, (TOP.z[0] + TOP.z[1]) / 2], axis: Z, radius: TOP.r, width: TOP.z[1] - TOP.z[0], mu: MU.silicone, tread: TREAD.silicone }));
  // (clockwise from above: a -y axis)
  const vkick = new Wheel({ name: 'vertical kicker (aluminum)', shaft: t.vkick, center: [VKICK.at[0], (VKICK.y[0] + VKICK.y[1]) / 2, VKICK.at[1]], axis: [0, -1, 0], radius: VKICK.r, width: VKICK.y[1] - VKICK.y[0], mu: MU.alu, tread: 30000 });
  // (the bottom kicker's faces toward the FUEL run up and to the left: -x axes; the top kicker's
  // faces toward the bore run up: the left one's axis -x, the right one's +x)
  const feed = [
    ...FEED.wheels.map(([z, y], i) => new Wheel({ name: `bottom kicker ${i + 1} (2.25in compliant)`, shaft: t.feedShafts[i], center: [FEED.x, y, z], axis: [-1, 0, 0], radius: FEED.r, width: FEED.w, mu: MU.compliant, tread: TREAD.compliant })),
    ...FEED.top.map(([z, y], i) => new Wheel({ name: `top kicker ${i ? 'right' : 'left'} (2in Stealth)`, shaft: t.feedShafts[3 + i], center: [FEED.x, y, z], axis: [i ? 1 : -1, 0, 0], radius: FEED.topR, width: FEED.w, mu: MU.compliant, tread: TREAD.compliant })),
  ];
  return { intake, floor: [...floor, ...top], vkick, feed };
}
// the shooter wheel and the hood's feeder wheels (posed by placeTurret)
export function shooterWheel(t) {
  return new Wheel({ name: '4in shooter wheel (swerve tread)', shaft: t.wheel, center: [0, 0, 0], axis: Z, radius: SHOOTER.r, width: SHOOTER.w, mu: MU.tread, tread: TREAD.tread });
}
export function feederWheels(t) {
  const F = SHOOTER.feeder;
  return t.hoodWheels.map((sh, i) => new Wheel({ name: `hood wheel ${i ? 'lower (2in roller)' : 'upper (2in Stealth)'}`, shaft: sh, center: [0, 0, 0], axis: Z, radius: F.r, width: F.w, mu: MU.compliant, tread: TREAD.compliant }));
}
// a turret-frame point (s along the shot, y up, z across) where the turret is at yaw (rad, three.js
// sense: + turns it to the left)
export function turretPoint([s, y, z], yaw) {
  const c = Math.cos(yaw), sn = Math.sin(yaw), [tx, tz] = TURRET.at;
  return [tx + s * c + z * sn, y, tz - s * sn + z * c];
}
// the shot's elevation (deg) by the hood's geometry for its angle (deg about the axle, from the shot's
// direction to the upper feeder wheel: 90 + the elevation, the wheels' pinch's tangent), and back
export const hoodElevation = (deg) => deg - 90;
export const hoodFor = (el) => el + 90;

// the front bumper: solid foam (3.25in with its backing), 1.25in to 5in up, across the front and
// round to the side bumpers' faces; soft (20 kN/m, as the others')
export function bumper1690() {
  return new Block({ name: 'bumper', x0: HALF_L + 0.75 * IN, x1: HALF_L + 3.25 * IN, y0: 0.032, y1: 0.16, hw: HALF_W + 3.25 * IN, mu: BUMPER_MU, tread: 20000 });
}

// ------------------------------------------------------------------ the surfaces
function grid(nu, nv, f) {
  const v = [], i = [];
  for (let a = 0; a <= nu; a++) for (let b = 0; b <= nv; b++) v.push(...f(a / nu, b / nv));
  for (let a = 0; a < nu; a++) for (let b = 0; b < nv; b++) { const p = a * (nv + 1) + b, q = p + nv + 1; i.push(p, q, p + 1, q, q + 1, p + 1); }
  return { v, i };
}
function merge(...ms) {
  const v = [], i = [];
  for (const m of ms) { const o = v.length / 3; v.push(...m.v); for (const k of m.i) i.push(k + o); }
  return { v, i };
}
const lerp = (a, b, t) => a + (b - a) * t;
const level = (x0, x1, z0, z1, y) => grid(Math.max(1, Math.ceil(Math.abs(x1 - x0) / 0.06)), Math.max(1, Math.ceil(Math.abs(z1 - z0) / 0.06)), (u, v) => [lerp(x0, x1, u), y, lerp(z0, z1, v)]);
// the belts' runs (strips across their 30 mm, the FUEL's side; they slide along -z: the belt's surface
// moving toward the robot's left), with their ends rounded over the pulleys
function beltRun(y, z0, z1, up) {
  const [x0, x1] = BELT.x, r = BELT.r, s = up ? 1 : -1;
  const end = (zc, k) => grid(2, 4, (u, v) => { const a = (Math.PI / 2) * v; return [lerp(x0, x1, u), y - s * r + s * r * Math.cos(a), zc + k * r * Math.sin(a)]; });
  return merge(level(x0, x1, z0, z1, y), end(z0, -1), end(z1, 1));
}

// the ball path's surfaces from their CAD (js/mech/path1690.js): what doesn't move (at 1 cm; the belts'
// channel, the kickers and up to the bore at 5 mm); the backing plate's PTFE; the belts' runs; the
// intake's carriage (it slides: placeIntake) and the kicker bar's arms (they swing on their pin); the
// turret and its hood (posed each step); the bumper's foam; the net over the top
// (FUEL's foam on the CAD's aluminum and polycarbonate: 0.25, as the other CAD robots' ball paths; the
// turret's plates and prints, 0.3)
export function pathSurfaces1690(t, mesh = PATH) {
  const cad = (k, o = {}) => new Surface({ name: `${k} (CAD)`, vertices: mesh[k].v.map((x) => x / 1000), indices: mesh[k].i, mu: MU.wall, ...o });
  const s = (name, m, o = {}) => new Surface({ name, vertices: m.v, indices: m.i, mu: MU.wall, ...o });
  const body = cad('body'), bodyFine = cad('bodyFine');
  const ptfe = cad('ptfe', { mu: MU.ptfe });
  // the belts: their runs slide along -z as their shafts turn (the 24T pulleys' pitch radius, a radian)
  const belt = (k, m) => { const sf = s(`${k} belt`, m, { mu: MU.belt, tread: TREAD.belt }).onSlide(t.belts[k === 'bottom' ? 0 : 1], [0, 0, -1]); sf.slideK = BELT.pitch; return sf; };
  const bottom = belt('bottom', beltRun(BELT.bottom.y, ...BELT.bottom.z, true)), top = belt('top', beltRun(BELT.top.y, ...BELT.top.z, false));
  // (the carriage on its racks: what FUEL push on its U-frame pushes back on its X44)
  const carriage = cad('carriage').onSlide(t.slide, SLIDE.dir);
  carriage.slideK = SLIDE.pinion;
  // (the kicker bar's arms and its fixed aluminum tube, on their pin: what FUEL push on them swings
  // them against the tubing, phys1690's moveKick; posed by placeIntake)
  const K = INTAKE.kick, piv = [...K.pivot, 0];
  const kick = cad('kick', { pos: piv });
  const [tx, ty] = [K.tube[0] - K.pivot[0], K.tube[1] - K.pivot[1]];
  const kickTube = s('kicker bar tube (aluminum, fixed)', grid(16, 12, (u, v) => [tx + K.tube[2] * Math.cos(2 * Math.PI * u), ty + K.tube[2] * Math.sin(2 * Math.PI * u), (v - 0.5) * K.width]), { mu: MU.alu, pos: piv });
  if (!K.rigid) { kick.onShaft(t.kickArm, piv); kickTube.onShaft(t.kickArm, piv); }
  const L = FUNNEL_LID, funnelLid = s('corner funnel lid (the sim\'s)', { v: L.pts.flatMap(([x, z]) => [x, L.y, z]), i: [0, 1, 2] }, { mu: MU.pla });
  const turret = cad('turret', { mu: MU.pla, pos: [TURRET.at[0], 0, TURRET.at[1]] });
  const hood = cad('hood', { mu: MU.pla });
  return { body, bodyFine, ptfe, bottom, top, carriage, kick, kickTube, funnelLid, turret, hood, bumper: bumper1690(), net: new Net1690() };
}
// how far the kicker bar has fallen (deg) with the carriage ext of the way out (it falls once the
// carriage has cleared it, a third of the way: an estimate)
export const kickAt = (ext) => INTAKE.kick.fall * Math.min(1, Math.max(0, (ext - 0.2) / 0.25));
// the kicker bar's arms' load (N m about the pin, + lifting them back up) at angle deg (from folded,
// Closed; down is negative): their weight, and the tubing pulling them down toward their stop
export function floorRoll(contacts, dt) {
  for (const c of contacts) {
    if (!/^(floor|top) roller/.test(c.wheel.name) || !(c.force > 0)) continue;
    const b = c.ball, w = b.angvel(), dw = (ROLL.rr * c.force * R * dt) / (0.4 * FUEL.mass * R * R);
    // (about the rollers' axis, z: toward no spin, not past it)
    const wz = Math.sign(w.z) * Math.max(0, Math.abs(w.z) - dw);
    b.setAngvel({ x: w.x, y: w.y, z: wz }, true);
  }
}
export function kickLoad(deg) {
  const K = INTAKE.kick, a0 = Math.atan2(K.tube[1] - K.pivot[1], K.tube[0] - K.pivot[0]), a = a0 + deg * DEG;
  const gravity = -K.mass * 9.81 * K.com * Math.cos(a);
  const tubing = -(K.tubing.preload + K.tubing.rate * Math.max(0, (deg - K.fall) * DEG));
  return gravity + tubing;
}
// the intake ext (0 in, 1 out) of the way out: the carriage, its rollers, the kicker bar
export function placeIntake(sf, wheels, ext, kickDeg = kickAt(ext)) {
  const [dx, dy] = INTAKE.slide, K = INTAKE.kick, a = kickDeg * DEG, c = Math.cos(a), sn = Math.sin(a);
  sf.carriage.setPose([dx * ext, dy * ext, 0]);
  sf.kick.setPose([...K.pivot, 0], 0, a);
  sf.kickTube.setPose([...K.pivot, 0], 0, a);
  wheels.forEach((w, i) => {
    const [x, y, z] = w.home;
    if (i < 2) w.center = [x + dx * ext, y + dy * ext, z];
    else { const ddx = x - K.pivot[0], ddy = y - K.pivot[1]; w.center = [K.pivot[0] + ddx * c - ddy * sn, K.pivot[1] + ddx * sn + ddy * c, z]; }
  });
}
// the intake's front (m: its U-frame's front) ext of the way out
export const intakeFront = (ext) => INTAKE.frame.front - INTAKE.slide[0] * (1 - ext);
// the turret at yaw, its hood at hoodDeg: the turret, the hood, the shooter wheel (its face toward the
// FUEL coming up runs up) and the feeder wheels (counter-rotating: their faces toward the wheel run
// up too)
export function placeTurret(sf, wheels, yaw, hoodDeg) {
  const S = SHOOTER, F = S.feeder, [ax, ay] = S.axle;
  // (the CAD's turret and hood meshes are in the turret's frame: yaw turns them from 0; the hood's is as
  // exported, at HOOD0)
  sf.turret.setPose([TURRET.at[0], 0, TURRET.at[1]], yaw, 0);
  sf.hood.setPose(turretPoint([ax, ay, 0], yaw), yaw, (hoodDeg - HOOD0) * DEG);
  const lat = [Math.sin(yaw), 0, Math.cos(yaw)];
  wheels.wheel.center = turretPoint([ax, ay, 0], yaw);
  wheels.wheel.axis = lat.map((v) => -v);
  wheels.feeder.forEach((w, i) => {
    const a = (hoodDeg + (i ? F.lower : 0)) * DEG;
    w.center = turretPoint([ax + F.R * Math.cos(a), ay + F.R * Math.sin(a), 0], yaw);
    w.axis = lat;
  });
}

// The top: the hopper's top plate (the bumper assembly's Part 51), a 2.0 mm polycarbonate sheet (the
// CAD's: its two faces 0.549 and 0.551 m up) cut to a lattice (0.13 m^2 of its 0.6 x 0.77 m: open over
// the turret at the back left, its windows elsewhere narrower than a FUEL), its edges bent down at
// the sides and riveted to the side panels' tops; at the back it sits on the back panel (the right
// half: the turret's opening is the left), and its front edge (over the frame's front, x 0.316) is
// free once the carriage is out (the U-frame's lattice top is under it only when it's in). It flexes
// "quite a bit to allow more storage" (the user): the load bows it up, most at its free front edge's
// middle (bow, m: across between the side panels a half sine, back to front a quarter one from the
// electronics box's face), against its stiffness: a strip of the lattice across the 0.716 m between
// the side panels (E 2.4 GPa, about 40% of its width solid) is ~100 N/m pressed at its middle, and as
// it bows its riveted edges pull in and it stiffens (membrane: k3, so 3 cm takes ~20 N and 5 cm ~80
// N), both estimates. Past it, out over the U-frame, the net (the team: "the balls tend to bounce up
// and out ... we'll do some tests with netting, but it could cause some issues with trench
// crossing"; their CAD has none) from the plate's front edge down to the U-frame's top edges (it
// slides out with the carriage). NET.bulge: lifted further (a fill, the benches': the top held up out
// of the way while they settle). In the robot's frame: x0, the back of the bow (the electronics box's
// face); x1, the front (the U-frame's front, out); hw, half the span
export const NET = { x0: BACK.box, x1: INTAKE.frame.front, y: WALL.top, hw: 0.358, bulge: 0, hang: { sag: 0.02, sagOut: 0.06 } };
// (press: how far a TRENCH's arm riding over it can press it back down onto the load, the FUEL's
// foam giving: 2 cm, an estimate)
export const PLATE = { t: 0.002, k: 100, k3: 6e5, max: 0.08, tau: 0.05, press: 0.02, back: -0.284, openZ: 0.09 };
// the top's bow shape at (x, z): 1 at the plate's front edge's middle, 0 at its supports
export const bowShape = (x, z) => {
  const v = Math.min(1, Math.max(0, (z + NET.hw) / (2 * NET.hw))), u = Math.min(1, Math.max(0, (x - NET.x0) / (HALF_L - NET.x0)));
  return Math.sin(Math.PI * v) * Math.sin((Math.PI / 2) * u);
};
// the top's height at (x, z) with the plate bowed `bow` (m at its front edge's middle), and its slopes
function topY(x, z, x1, bow) {
  const L = x1 - NET.x0, W = 2 * NET.hw, B = NET.bulge;
  const u = Math.min(1, Math.max(0, (x - NET.x0) / L)), v = Math.min(1, Math.max(0, (z + NET.hw) / W));
  const lift = B * Math.sin(Math.PI * u) * Math.sin(Math.PI * v);
  if (x <= HALF_L) return NET.y + bow * bowShape(x, z) + lift;
  // (out over the U-frame, whose top is 0.07 m lower, the net slopes down to it from the plate's edge)
  const edge = NET.y + bow * bowShape(HALF_L, z);
  return edge - (edge - INTAKE.frame.top) * Math.min(1, (x - HALF_L) / 0.06) + lift;
}
export function netAt(x, z, x1 = NET.x1, bow = 0) {
  const e = 1e-3, y = topY(x, z, x1, bow);
  return [y, (topY(x + e, z, x1, bow) - y) / e, (topY(x, z + e, x1, bow) - y) / e];
}
export const netY = (x, z, x1, bow) => topY(x, z, x1 ?? NET.x1, bow ?? 0);
// the plate's force (N, at its front edge's middle) bowed b, and its stiffness there
export const plateForce = (b) => PLATE.k * b + PLATE.k3 * b ** 3;
const plateK = (b) => PLATE.k + 3 * PLATE.k3 * b * b;
class Net1690 extends Surface {
  constructor() {
    super({ name: 'top plate (polycarbonate) and net', vertices: [0, 0, 0, 0.01, 0, 0, 0, 0, 0.01], indices: [0, 1, 2], mu: MU.wall, tread: TREAD.plate });
    this.x1 = NET.x1;
    this.bow = 0; this.force = 0; this.cap = PLATE.max;
    this.seen = new Map();
  }
  // (the U-frame's front, where the carriage puts it)
  place(x1) { this.x1 = x1; }
  at(x, z) { return netAt(x, z, this.x1, this.bow); }
  touches(p, b) {
    const [x, y, z] = p;
    // (behind the bow, the plate flat over the hopper's back right to its back edge; the back left is
    // its opening for the turret)
    if (x >= this.x1 || Math.abs(z) >= NET.hw || (x <= NET.x0 && !(x > PLATE.back && z > PLATE.openZ))) return [];
    const [h, hx, hz] = this.at(x, z), l = Math.hypot(hx, 1, hz), dist = (h - y) / l;
    if (dist >= R || dist < -0.03) return [];
    // (how far the plate's bow moves the top here: out on the net, less toward the U-frame)
    if (b) this.seen.set(b, x <= HALF_L ? bowShape(x, z) : bowShape(HALF_L, z) * Math.max(0, 1 - (x - HALF_L) / 0.06));
    return [{ depth: R - dist, n: [hx / l, -1 / l, hz / l] }];
  }
  // after a step: the plate bowed as far as the load pressing on it (contacts: the step's) balances
  // its stiffness (a Newton step on the two, the FUEL's own give in series, eased over PLATE.tau)
  flex(dt, contacts) {
    let F = 0, K = 0;
    for (const c of contacts) {
      if (c.wheel !== this || !(c.force > 0)) continue;
      const w = (this.seen.get(c.ball) ?? 0) * Math.abs(c.n[1]);
      F += c.force * w;
      K += ((2 * c.force) / Math.max(c.depth, 1e-4)) * w * w;
    }
    this.seen.clear();
    this.force = F;
    const step = (F - plateForce(this.bow)) / (K + plateK(this.bow));
    // (cap: held down, a TRENCH's arm over it)
    this.bow = Math.min(this.cap, Math.max(0, this.bow + step * Math.min(1, dt / PLATE.tau)));
  }
}
export { Net1690 };
