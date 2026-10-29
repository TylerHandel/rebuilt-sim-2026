// 2910's Re•Blitz as physical mechanisms (js/mech/mechanism.js), from their CAD (Onshape "12 - 2026
// ReBlitz Top Level Assembly", linked from their 2026 CAD and tech binder release) and their code
// (FRCTeam2910/2026CompetitionRobot-Public: config/ReBlitz.java, config/Constants.java, the
// intake, hopper and shooter subsystems and SuperStructure). Robot frame: x forward (the intake),
// y up, z to its right; the CAD's x across, y back, z up.
//
// The ball path: the slap-down intake's 2in roller (grip) pulls FUEL in under it, and its two 1in
// rollers below lift them over the bumper (its "powered bottom roller", 4 cm off the carpet), into
// the hopper. Its floor is ten powered rollers, 1.25in silicone sleeves sloping down to the back
// and then thin aluminum ones, carrying FUEL back to the foot of the shooter, where two more go up
// the back wall: there FUEL are pinched between them and a shaft of nine 2.25in 35A compliant
// wheels and lifted into the pinch between the drum (4in, grip tape) and the hood's two 1.5in
// rollers (grip tape), which flings them up and back over the drum. All four FUEL wide.
// Drives (the CAD's gears, belts and chain):
//   intake: two Kraken X60s (IntakeLeft/RightRoller, 120 A stator, 100 A supply; in teleop 100 A
//     and 40 A), each 11t:~30t
//     (HTD 5mm) to the 2in roller; the left one also 24t:48t and 12t:12t (#25) to the 1in rollers
//   hopper floor: four X60s (80 A stator, 40 A supply), 40t:40t onto one roller, the rest chained
//     12t:12t (#25) to it (their code: HOPPER_GEAR_RATIO 1.0); the compliant wheels' shaft
//     chained 12t:12t up to a 16t gear meshing their 28t
//   shooter: four X60s (60 A stator, 35 A supply; FLYWHEEL_GEAR_RATIO 1.0, so their table's RPM
//     is the motors'), belted 24t to a ~38t pulley on the drum (its size in the CAD; with that,
//     their own shot table puts every shot from 1.3 to 5 m in the HUB: tools/mechlab2910.mjs);
//     the drum's 67t belts a 6in brass flywheel's 10t pulley (410 mm belt), and a double-sided
//     belt turns the hood rollers the other way at ~38t:21t (pulley sizes from the CAD's bounds)
//   intake pivot: an X60 (75 A stator, 30 A supply) through 3:1, 1:1, 4:1 and 30:9 (#35 chain)
//   hood: an X44 through 52:12, 18:12, 146:10, about the drum's axle
import { IN } from '../constants.js';
import { BUMPER_T, BUMPER_Y0, BUMPER_Y1 } from '../robotConfigs.js';
import { Block, Motor, Shaft, Surface, Train, Wheel, BUMPER_MU } from './mechanism.js';

// what each grips FUEL with (estimates, in the order 8793's team gave: compliant wheels most,
// grip tape, silicone and rubber, bare aluminum least). Bare aluminum (the floor's back rollers)
// 0.6, not 0.45: with the hopper's walls at 0.25 (pathSurfaces2910) a full hopper dumps at 31-32
// FUEL/s on average, as the team says it does, not 29 (tools/mechlab2910.mjs dump); an estimate
export const MU = { compliant: 1.6, gripTape: 1.5, silicone: 1.0, aluminum: 0.6 };

// ------------------------------------------------------------------ the layout (robot frame)
export const PIVOT = [0.273, 0.17];   // the intake's pivot (its bearings)
export const DRUM = [-0.283, 0.47];   // the drum's axle, which the hood pivots on
export const INTAKE = {
  // as the CAD has it, deployed (resting on the bumper, its hardstop): arm angle -1.5 deg
  main: { at: [0.610, 0.161], d: 0.054 },            // 2in roller in grip
  lower: [{ at: [0.448, 0.103], d: 0.0349 }, { at: [0.483, 0.042], d: 0.0349 }],
  halfWidth: 0.29,
  // their code's angles (degrees from stowed): deployed 114.3, retracted 10, stowed 0
  deploy: 114.3, retract: 10,
  cadDeg: -1.5, // the arm's angle (up from forward) at deploy, as the CAD has it
  mass: 8.1, com: 0.24, // kg, m from the pivot (the CAD: billet plates, rollers, motors)
};
export const FLOOR = [ // [x, y, diameter, grip]
  [0.256, 0.195, 0.0349, 'silicone'], [0.194, 0.179, 0.0349, 'silicone'], [0.132, 0.162, 0.0349, 'silicone'],
  [0.071, 0.146, 0.0349, 'silicone'], [0.009, 0.129, 0.0349, 'silicone'],
  [-0.053, 0.113, 0.0349, 'aluminum'], [-0.161, 0.113, 0.0349, 'aluminum'], [-0.251, 0.174, 0.0349, 'aluminum'],
  [-0.289, 0.261, 0.0349, 'silicone'], [-0.278, 0.356, 0.0349, 'silicone'],   // up the back wall
];
export const INDEXER = { at: [-0.107, 0.289], d: 2.25 * IN, z: [-0.254, -0.191, -0.127, -0.064, 0, 0.063, 0.127, 0.19, 0.254] };
export const SHOOTER = {
  drum: { d: 0.1048, width: 0.508 },
  hood: [{ at: [-0.081, 0.461], d: 0.0413 }, { at: [-0.084, 0.508], d: 0.0413 }], // at hood 0 (the CAD's)
  hoodWidth: 0.603,
  hoodMax: 37, // their HOOD_MAX_ANGLE: tilting the hood back over the drum this far flattens the shot
  drumPerMotor: 24 / 38, flywheelPerDrum: 67 / 10, hoodRollerPerDrum: 38 / 21,
  // inertia (kg m^2), each about its own axis from the CAD's solids (OpenCascade's volume
  // properties on the STEP) and the BOM's materials; README, *2910*, has the table. The drum: its
  // 4in 6061 tube (0.85 kg), the DKL grip tape (the BOM's silicone rubber, 1.6 mm), the Onyx end
  // caps with their 40t pulleys, the 67t PLA pulley, the 7075 shaft, the 40t that drives the hood's
  // belt, and the 68T, 410 mm and 170T belts at their pulleys' pitch radius. The brass flywheel with
  // its 10t pulley (88 cm3, 0.75 kg; it turns on a dead axle). The hood's two rollers (6061 tubes,
  // grip tape, Onyx pulleys, 6061 hex), 1.7e-4 each. The motors' 24t pulleys and 55T belts (at the
  // motors' speed). (The motors' rotors are added by the Train.)
  drumJ: 3.99e-3, flywheelJ: 2.48e-3, hoodJ: 3.43e-4, motorJ: 2.3e-5,
  // their velocity loop (Slot0, volts): kV x target + kP x error, motor rotations/s
  kV: 12 / (5800 / 60), kP: 0.35,
};
// their shot tables (ShooterInterpolation, PassingInterpolation): distance to the HUB's center
// (m) -> motor RPM and hood angle; passes at the flattest hood
export const SHOT_TABLE = [[1.3, 2100, 0], [2.0, 2250, 8], [3.0, 2400, 17], [4.0, 2600, 23], [5.0, 2900, 28]];
export const PASS_TABLE = [[2.0, 2000], [6.0, 3500], [10.0, 6000]];
const interp = (t, x, j) => {
  if (x <= t[0][0]) return t[0][j];
  for (let k = 1; k < t.length; k++) if (x <= t[k][0]) return t[k - 1][j] + ((t[k][j] - t[k - 1][j]) * (x - t[k - 1][0])) / (t[k][0] - t[k - 1][0]);
  return t[t.length - 1][j];
};
export const shotFor = (d) => ({ rpm: interp(SHOT_TABLE, d, 1), hood: interp(SHOT_TABLE, d, 2) });
export const passFor = (d) => ({ rpm: interp(PASS_TABLE, d, 1), hood: SHOOTER.hoodMax });
// their CANranges, on the right side plate looking across, tripped by anything within 0.57 m:
// the hopper's (full when FUEL are piled up to it) and the feeder's (one waiting at the lift)
export const RANGES = { hopper: { at: [-0.012, 0.399], z: -0.327 }, feeder: { at: [-0.229, 0.380], z: -0.327 }, reach: 0.57 };
// the preload, eight on the floor (robot frame)
export const PRELOAD = [
  [-0.17, 0.205, -0.19], [-0.17, 0.205, -0.063], [-0.17, 0.205, 0.063], [-0.17, 0.205, 0.19],
  [-0.02, 0.225, -0.19], [-0.02, 0.225, -0.063], [-0.02, 0.225, 0.063], [-0.02, 0.225, 0.19],
];

// ------------------------------------------------------------------ the drives
// the intake rollers' current limits (A, per motor): ReBlitz.java's for AUTO, and from teleop on
// what Robot.configureTeleopCurrentLimits sets (IntakeSubsystem.setCurrentLimits(100, 40))
export const INTAKE_LIMITS = { auto: { statorLimit: 120, supplyLimit: 100 }, teleop: { statorLimit: 100, supplyLimit: 40 } };
export function intakeLimits(t, teleop) { Object.assign(t.intake.motor, INTAKE_LIMITS[teleop ? 'teleop' : 'auto']); }
export function build2910() {
  // the intake's rollers: positive turns them inward (the 2in roller's bottom and the 1in rollers'
  // tops run rearward)
  const intake = new Train({ name: 'intake rollers', motor: new Motor('krakenX60', { count: 2, ...INTAKE_LIMITS.auto }) });
  const main = intake.add(new Shaft({ name: 'intake 2in roller', inertia: 3e-4 }), -11 / 30);
  const lower = INTAKE.lower.map((_, i) => intake.add(new Shaft({ name: `intake 1in roller ${i + 1}`, inertia: 5e-5 }), (24 / 48) * (12 / 12)));
  // the hopper's floor, up the back, and the indexer's compliant wheels: positive carries FUEL
  // back along the floor and up the back
  const floor = new Train({ name: 'hopper', motor: new Motor('krakenX60', { count: 4, statorLimit: 80, supplyLimit: 40 }) });
  const rollers = FLOOR.map(([x], i) => floor.add(new Shaft({ name: `hopper roller ${i + 1}`, inertia: 5e-5 }), 1));
  const indexer = floor.add(new Shaft({ name: 'indexer compliant wheels', inertia: 2e-4 }), -16 / 28);
  // the shooter: positive runs the drum's front up, the hood rollers' backs up
  const shooter = new Train({ name: 'shooter', motor: new Motor('krakenX60', { count: 4, statorLimit: 60, supplyLimit: 35 }) });
  const S = SHOOTER;
  const drum = shooter.add(new Shaft({ name: 'drum', inertia: S.drumJ }), S.drumPerMotor);
  shooter.add(new Shaft({ name: 'brass flywheel', inertia: S.flywheelJ }), S.drumPerMotor * S.flywheelPerDrum);
  const hood = shooter.add(new Shaft({ name: 'hood rollers', inertia: S.hoodJ }), -S.drumPerMotor * S.hoodRollerPerDrum);
  // (inertia only: the train's drag stays the three shafts' it was tuned with)
  shooter.add(new Shaft({ name: 'motors\' 24t pulleys', inertia: S.motorJ, drag: 0 }), 1);
  // the intake's pivot: an X60 through 40:1, its arm on it (I = m r^2, the arm's own spread
  // about doubling it)
  const pivot = new Shaft({ name: 'intake pivot', motor: new Motor('krakenX60', { ratio: 40, statorLimit: 75, supplyLimit: 30 }), inertia: 2 * INTAKE.mass * INTAKE.com * INTAKE.com, drag: 0.5 });
  return { intake, main, lower, floor, rollers, indexer, shooter, drum, hood, pivot };
}

// the wheels (robot frame; the intake's and hood's posed with placeIntake / placeHood)
export function wheels2910(t) {
  const Z = [0, 0, 1];
  const floor = FLOOR.map(([x, y, d, grip], i) => new Wheel({ name: t.rollers[i].name, shaft: t.rollers[i], center: [x, y, 0], axis: Z, radius: d / 2, width: 0.559, mu: MU[grip], tread: 20000 }));
  const indexer = INDEXER.z.map((z) => new Wheel({ name: 'indexer 2.25in 35A compliant', shaft: t.indexer, center: [...INDEXER.at, z], axis: Z, radius: INDEXER.d / 2, width: 1 * IN, mu: MU.compliant, tread: 3000 }));
  const intake = [
    new Wheel({ name: 'intake 2in roller', shaft: t.main, center: [...INTAKE.main.at, 0], axis: Z, radius: INTAKE.main.d / 2, width: 0.575, mu: MU.gripTape, tread: 40000 }),
    ...INTAKE.lower.map((w, i) => new Wheel({ name: t.lower[i].name, shaft: t.lower[i], center: [...w.at, 0], axis: Z, radius: w.d / 2, width: 0.586, mu: MU.silicone, tread: 20000 })),
  ];
  intake.forEach((w) => { w.home = w.center.slice(); });
  const drum = new Wheel({ name: 'drum (grip tape)', shaft: t.drum, center: [...DRUM, 0], axis: Z, radius: SHOOTER.drum.d / 2, width: SHOOTER.drum.width, mu: MU.gripTape, tread: 40000 });
  const hood = SHOOTER.hood.map((h, i) => new Wheel({ name: `hood roller ${i + 1} (grip tape)`, shaft: t.hood, center: [...h.at, 0], axis: Z, radius: h.d / 2, width: SHOOTER.hoodWidth, mu: MU.gripTape, tread: 40000 }));
  hood.forEach((w) => { w.home = w.center.slice(); });
  return { floor, indexer, intake, drum, hood };
}

// The front bumper, which the intake lifts FUEL over: solid foam (2.5in, from 1.25in up off the
// carpet to 5in, on a 3/4in backing: robotConfigs.js BUMPER_*) across the front, out to the side
// bumpers' faces, as a solid block, so a FUEL squeezed against it is pushed back out however far
// in it's got. The CAD has the bumper as a
// hollow shell of thin plates, and a FUEL on the carpet in front is the intake's (it passes through
// the robot's own collider) with nothing else to keep it out: the deploying arm landing its 1in
// rollers on one against the bumper (450 N, 6 cm into it), or one coming in under the arm's side
// plate, pushed it through the face plate into the hollow. From there it slid on under the frame
// and stayed, wedged up into the floor rollers by the CAD's belly pan and motors under them,
// rubbing on them (eight matches of 2910 v 8793: 16 FUEL, 1-4 a match, most riding there the rest
// of the match; the floor under half its speed through 70% of its feeding in the match with two
// there, 10-12% with none). With this and where()'s guard (js/mech/phys2910.js), none. (None ever
// fell through the floor from the hopper.)
export function bumper2910() {
  return new Block({ name: 'bumper', x0: 13.5 * IN + 0.75 * IN, x1: 13.5 * IN + BUMPER_T, y0: BUMPER_Y0, y1: BUMPER_Y1, hw: 13.75 * IN + BUMPER_T, mu: BUMPER_MU });
}
// rotate a point (x, y) about (cx, cy) by a (rad)
const rot = ([x, y], [cx, cy], a) => { const c = Math.cos(a), s = Math.sin(a), dx = x - cx, dy = y - cy; return [cx + dx * c - dy * s, cy + dx * s + dy * c]; };
// the intake's wheels with the arm at angle deg (their code's: 0 stowed, 114.3 deployed)
export function intakeRot(deg) { return ((INTAKE.deploy - deg) * Math.PI) / 180; }
export function placeIntake(wheels, deg) {
  const a = intakeRot(deg);
  for (const w of wheels) { const [x, y] = rot(w.home, PIVOT, a); w.center = [x, y, 0]; }
}
// the hood's rollers at hood angle deg (their code's: 0 to 37, tilting back over the drum)
export function placeHood(wheels, deg) {
  const a = (deg * Math.PI) / 180;
  for (const w of wheels) { const [x, y] = rot(w.home, DRUM, a); w.center = [x, y, 0]; }
}
// the shot's elevation at hood angle deg, as the CAD's geometry has it: square to the line from
// the drum's axle to the top hood roller (79 deg at hood 0), flatter as the hood tilts back
export function hoodElevation(deg) {
  const [hx, hy] = SHOOTER.hood[1].at;
  return 90 - (Math.atan2(hy - DRUM[1], hx - DRUM[0]) * 180) / Math.PI - deg;
}

// the ball path's surfaces from their CAD (js/mech/path2910.js): the body, the intake on its pivot
// (it swings, on the pivot's shaft) and the hood on the drum's axle. FUEL slide on them at 0.25
// (an estimate: their hopper is polycarbonate panels and lids on an aluminum frame, the FUEL's
// urethane skin slick on them; at the 0.35 every other robot's CAD gets, a full hopper dumps at 29
// FUEL/s on average, at 0.25 with the floor's aluminum rollers at 0.6, 31-32: the team's 32)
export function pathSurfaces2910(mesh, t, { mu = 0.25 } = {}) {
  const s = (k, pos) => new Surface({ name: `${k} (CAD)`, vertices: mesh[k].v.map((x) => x / 1000), indices: mesh[k].i, mu, pos });
  const body = s('body', [0, 0, 0]);
  const intake = s('intake', [PIVOT[0], PIVOT[1], 0]).onShaft(t.pivot, [PIVOT[0], PIVOT[1], 0]);
  const hood = s('hood', [DRUM[0], DRUM[1], 0]);
  return { body, intake, hood, bumper: bumper2910() };
}
