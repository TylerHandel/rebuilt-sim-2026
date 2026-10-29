// 341's Miss Daisy XXIV as physical mechanisms (js/mech/mechanism.js), from their CAD (Onshape
// "Miss Daisy XXIV"), their design binder (the gearings) and their code (Team341/FRC2026-Public:
// the intake, serializer, feeder, shooter, hood and turret subsystems and their tables). Robot
// frame: x forward (the intake), y up, z to its right; the CAD's x across to the left, y back, z up.
//
// The ball path, 4 wide to 1 ("the zipper"): on the slapdown intake, down, the front ("top")
// 1.25in silicone roller pulls FUEL in off the carpet to the bumper's foot, where the bottom one
// lifts them up its face; the top roller rolls them up it, and three 2in polycarbonate rollers
// (all of them driven) roll them over its top, squeezed against it, to the top roller on the
// arm's pivot (2.25in compliant wheels in the middle, mecanums pushing in from each side), which
// brings them in onto the serializer's floor. There the table's two top rollers (compliant
// wheels, mecanums pushing toward the uptake's side) and three 6in omni wheels lying flat (two
// free, one powered: the feeder) steer them into
// the uptake behind the turret, where two shafts of 1.625in flex wheels push them up into the
// turret: a 1in urethane wheel hands them to the shooter, whose 4in urethane flywheel wraps them
// round under the hood's two rows of 2in Stealth wheels and flings them out forward.
// Drives (the binder's and the CAD's gearings, their code's limits):
//   intake rollers: two Kraken X60s (90 A stator, 30 A supply), a 2:1 timing belt reduction
//   intake deploy: a Kraken X44 through 3.4:1, 48:14 and 48:16 (#25 chain), 35:1, onto the arm,
//     which swings on the top roller's shaft
//   top rollers and uptake: two X60s (90 A, 30 A), 2:1; the uptake 7:5 slower again (shared belts)
//   feeder (the powered omni): a Falcon 500, 6:1
//   shooter: two X60s (120 A stator), direct to the flywheel, the Stealth wheels and the 1in wheel
//   hood: an X44 through 44:11, 32:18 and its 188-tooth rack (133.7:1), about the flywheel's axle
//   turret: an X44 through 40:8 and 92:10 (46:1)
import { IN } from '../constants.js';
import { Motor, Shaft, Surface, Train, Wheel, BUMPER_MU } from './mechanism.js';

// what each grips FUEL with (estimates, as 8793's: compliant wheels most, then urethane and
// silicone, mecanums' and omnis' rollers, bare polycarbonate least)
export const MU = { compliant: 1.6, urethane: 1.3, stealth: 1.4, silicone: 1.0, mecanum: 0.8, omni: 0.6, polycarb: 0.5 };

// ------------------------------------------------------------------ the layout (robot frame)
export const HALF = 13.5 * IN; // the frame's half length and width (27in)
// The intake swings on the top roller's shaft. As the CAD has it, folded up (their 9 deg: below);
// down, their 149 (their DEPLOYMENT_ANGLE), turned forward and down.
export const PIVOT = [0.255, 0.337];
export const INTAKE = {
  // the two 1.25in polycarbonate rollers wrapped in silicone (bottom, top: the silicone's 34 mm
  // outside, the CAD's) and the three 2in polycarbonate rollers ("dead axle"), on the arm, as
  // exported (folded up; the 2in ones from the pivot out)
  bottom: { at: [0.291, 0.680], d: 0.034, w: 0.583 },
  top: { at: [0.119, 0.680], d: 0.034, w: 0.565 },
  dead: [[0.192, 0.397], [0.121, 0.468], [0.083, 0.574]], deadD: 2 * IN, deadW: 0.562,
  // on the pivot's shaft: three 2.25in 50A compliant wheels in the middle, seven 2in mecanums each
  // side pushing to the middle
  compliant: [-0.056, -0.018, 0.020], left: [-0.298, -0.263, -0.228, -0.193, -0.158, -0.123, -0.088], right: [0.052, 0.087, 0.122, 0.157, 0.192, 0.227, 0.262],
  down: 149, up: 44, stowed: 0,
  // the CAD's folded pose, in their angles. Down is where the arm rests on the bumper: their code
  // zeroes its encoder there (150 at boot, 149 on the operator's button) and holds 149; their
  // binder: the "plate for the bottom roller rest[s] against [the] vertical plane of [the] bumper".
  // That plate is the arm's side plates' straight back edge (the CAD's "Main Slap Down"): swung
  // 140 deg from the CAD's pose it stands vertical (within 0.3 deg) and lies on the face of a
  // 2.5in foam bumper (0.4275 out; the CAD has only its wood) from 0.05 to 0.19 m up, and the
  // plate's lowest corner clears the carpet by 34 mm. (149 from the CAD's pose would sink it 39 mm
  // into the foam; their 150, where it boots, sinks it 5 mm.) So the CAD's pose is their 9, and
  // down (their 149) is 140 from it. There the bottom roller sits at the bumper's foot, 4 mm in
  // front of its face, and FUEL climb the face squeezed 15 mm between it and the top roller
  // (their binder: ~0.5in was best, >1in caused problems), 27 and 20 mm under the third and second
  // 2in rollers round its top corner (the foam's corner gives too), 21 under the second over its
  // top, 19 under the first and 16 under the pivot's wheels over the serializer's floor. (With its
  // 2in rollers driven it takes in every FUEL the bench brings it, 17.6 a second, from 137 to 143
  // alike: tools/mechlab341.mjs rate.)
  cad: 9,
  // the arm (side plates, rollers, bash bar): an estimate of the binder's 16.5 lb intake less its
  // fixed gearboxes and plates
  mass: 3.0, com: 0.25, J: 0.2,
};
// the serializer's floor (top) and its three 6in omni wheels lying flat at FUEL height: the
// powered one (the feeder) nearest the uptake, two free
export const FLOOR_Y = 0.1747;
export const OMNI = { d: 6 * IN, y: 0.248, w: 0.037, feeder: [-0.044, 0.075], free: [[0.108, 0.219], [0.121, -0.229]] };
// the table's top rollers over the floor (x, the wheels' z): compliant wheels, and 2in printed
// mecanums pushing toward the uptake's side (the robot's left)
export const TABLE = {
  y: 0.337,
  a: { x: 0.093, compliant: [-0.098, -0.063, -0.029], right: [0.002, 0.031, 0.059, 0.088, 0.116, 0.145] },
  b: { x: 0.168, compliant: [-0.087, -0.043, 0.0, 0.043, 0.087], left: [-0.158, -0.129], right: [0.129, 0.158] },
};
// the uptake: two shafts of 1.625in flex wheels either side of the turret's axis, pushing FUEL up
export const UPTAKE = { back: [-0.187, 0.328], front: [-0.016, 0.328], z: [-0.064, -0.089], d: 1.625 * IN };
// the turret's axis (their ROBOT_TO_TURRET: 4in back, 3in left) and, in its frame (facing
// forward), the shooter: the flywheel (two 4in urethane rollers), the 1in urethane wheels under
// it, and on the hood, about the flywheel's axle, two rows of 2in Stealth wheels
export const TURRET = [-0.102, -0.076];
export const SHOOTER = {
  fly: { at: [0.112, 0.466], d: 4 * IN, w: 0.1 },
  feed: { at: [0.074, 0.410], d: 1 * IN, w: 0.1 },
  hood: [[-0.202, -0.002], [-0.196, 0.052]], hoodD: 2 * IN, hoodW: 0.05,
  // the flywheel and what turns with it (kg m^2), each part about its own axis from the CAD's solids
  // (OpenCascade's volume properties on the STEP) and the BOM's materials; README, *341*, has the
  // table: two McMaster 2477K37 4in urethane rollers (0.40 kg each) on their steel hubs and HDPE
  // inserts, the 7075 hex, the PLA pulleys, the belts and the motors' pulleys. The Stealth wheels'
  // two shafts and the reverser (the 30t gears) are Jstealth, the 1in wheels Jfeed
  J: 1.228e-3, Jstealth: 8.66e-5, Jfeed: 8.3e-6,
  // their velocity loop (VelocityVoltage, motor rotations/s): kV, kP; their RPM tolerance
  kV: 0.135, kP: 0.125, tol: 130,
  hoodMax: 31.3, // their soft limit, 11.62 motor rotations
  // the hood's turn about the flywheel's axle (deg) at their angle (deg): 1:1 by its rack
  hoodTurn: [[0, 0], [31.3, 31.3]],
};
// their tables: distance from the turret to the HUB's inner center (m) -> flywheel RPM, hood
// angle; and for ferrying (FEED: their hood's always 28 there)
const table = (d, a) => d.map((x, i) => [x, a[i]]);
export const REGRESSIONS = {
  rpm: table([1.51, 2.38, 3.07, 3.5, 4.24, 5.5, 14], [2000, 2060, 2025, 2100, 2200, 2425, 3245]),
  hood: table([1.51, 2.38, 3.03, 3.5, 4.24, 5.5, 14], [0, 6, 11, 13.5, 20, 29.5, 31]),
  feedRpm: table([5.5, 7.5, 10.5, 13.94, 16], [1800, 2000, 2650, 3100, 4720]),
  feedHood: 28,
};
const interp = (t, x) => {
  if (x <= t[0][0]) return t[0][1];
  for (let k = 1; k < t.length; k++) if (x <= t[k][0]) return t[k - 1][1] + ((t[k][1] - t[k - 1][1]) * (x - t[k - 1][0])) / (t[k][0] - t[k - 1][0]);
  return t[t.length - 1][1];
};
export function shotFor(kind, d) {
  return kind === 'ferry' ? { rpm: interp(REGRESSIONS.feedRpm, d), hood: REGRESSIONS.feedHood } : { rpm: interp(REGRESSIONS.rpm, d), hood: interp(REGRESSIONS.hood, d) };
}
// the preload, eight on the serializer's floor
export const PRELOAD = [
  [0.22, 0.25, -0.225], [0.22, 0.25, -0.075], [0.22, 0.25, 0.075], [0.22, 0.25, 0.225],
  [0.06, 0.25, -0.15], [0.06, 0.25, 0.0], [-0.1, 0.25, -0.077], [0.0, 0.25, 0.15],
];

// ------------------------------------------------------------------ the drives
export function build341() {
  // the intake's rollers: positive runs them inward (the bottom roller's front up, the top one's
  // back up; the 2in rollers' and the top roller on the pivot's bottoms rearward)
  // Their CAD's belts: the X60s' 12t pulleys onto a 20t, its shaft's 18t to the pivot shaft's 22t
  // (104T belt): 2.04:1 (their binder's "2:1"). From the pivot shaft a 20t-to-20t belt runs to
  // each roller on the arm in turn (55T to the first 2in roller, 60T to the second, 65T to the
  // third, 65T to the top roller; the belts' lengths match the rollers' spacing to the mm), and
  // from the top roller (60T) a pair of 36t spur gears turns the other way, belted (90T) to the
  // bottom roller. So the three 2in rollers are driven too ("dead axle": each tube turns on
  // bearings on stub shafts, which tie the side plates together), all at the pivot shaft's speed,
  // the bottom roller the other way. (Free, the 2in rollers left FUEL stalled where the top roller
  // lets go of them: between the bumper's face and the third, which pressed them down ~30 N, and
  // on the floor under the first, 8 cm short of the pivot's wheels.)
  const intake = new Train({ name: 'intake rollers', motor: new Motor('krakenX60', { count: 2, statorLimit: 90, supplyLimit: 30 }) });
  const k = (12 / 20) * (18 / 22);
  const bottom = intake.add(new Shaft({ name: 'intake bottom roller', inertia: 3e-5 }), k);
  const top = intake.add(new Shaft({ name: 'intake top roller', inertia: 3e-5 }), -k);
  const pivotRoller = intake.add(new Shaft({ name: 'intake top shaft (on the pivot)', inertia: 1.5e-4 }), -k);
  const dead = INTAKE.dead.map((_, i) => intake.add(new Shaft({ name: `intake 2in roller ${i + 1}`, inertia: 6e-5 }), -k));
  // the top rollers and the uptake: positive runs the top rollers' bottoms rearward and the
  // uptake's faces up
  // Their CAD's belts (each belt's length matches its pulleys' spacing to the tooth): both X60s'
  // 12t pulleys onto an intermediate shaft's 24t (50T belts: their binder's 2:1); from it a 20t to
  // the front top roller's 20t (75T, 15 mm), that one's 20t to the back top roller's (50T): both at
  // the intermediate's speed. The uptake is the binder's "7:5": the intermediate's 28t drives the
  // front uptake shaft's 20t (50T), 7:5 FASTER than the top rollers (8.8 m/s at their flex wheels'
  // rims, the top rollers' compliant wheels 8.7: FUEL handed from one to the other at the same
  // speed); its 24t, an 80T belt to a 24t behind and 36t:36t spur gears turn the back shaft the
  // other way at the same speed. (It was the other way up, the uptake at 5/7, half as fast: FUEL
  // waited at the uptake, and the zipper behind it had longer to lock up (wheels341): 9 of 24
  // tools/mechlab341.mjs path runs locked, 4.7 FUEL/s; with this and the floor's fix
  // (tools/path-mesh-341.py) 6, 5.8 FUEL/s, and at 0.5 m/s 8.6.)
  const serializer = new Train({ name: 'top rollers and uptake', motor: new Motor('krakenX60', { count: 2, statorLimit: 90, supplyLimit: 30 }) });
  const tableA = serializer.add(new Shaft({ name: 'table top roller (front)', inertia: 1e-4 }), -1 / 2);
  const tableB = serializer.add(new Shaft({ name: 'table top roller (back)', inertia: 1e-4 }), -1 / 2);
  const upBack = serializer.add(new Shaft({ name: 'uptake (back)', inertia: 3e-5 }), (1 / 2) * (7 / 5));
  const upFront = serializer.add(new Shaft({ name: 'uptake (front)', inertia: 3e-5 }), -(1 / 2) * (7 / 5));
  // the feeder: the powered omni (positive turns FUEL on its face toward the uptake). (Its
  // direction is their code's and their CAD's: their feeder runs at -3 V on TalonFXFactory's
  // CounterClockwise_Positive, so clockwise looking down at the Falcon's shaft, which points up;
  // one 10:60 spur stage under the omni turns the omni the other way, counterclockwise from
  // above: its front toward the robot's left, its left side rearward. Its current limits are
  // TalonFXFactory's 90 A stator, 30 A supply: FeederIOReal builds its own 100/20 and never
  // applies them.)
  const feederT = new Train({ name: 'feeder', motor: new Motor('falcon500', { statorLimit: 90, supplyLimit: 30 }) });
  const feeder = feederT.add(new Shaft({ name: 'feeder 6in omni', inertia: 6e-4 }), 1 / 6);
  const omniFree = OMNI.free.map((_, i) => new Shaft({ name: `free 6in omni ${i + 1}`, inertia: 6e-4, drag: 1e-4 }));
  // the shooter: positive runs the flywheel's back up (FUEL up its back and over it), the hood's
  // Stealth wheels' fronts up, and the 1in wheel's back up
  const shooter = new Train({ name: 'shooter', motor: new Motor('krakenX60', { count: 2, statorLimit: 120, supplyLimit: 200 }) });
  const fly = shooter.add(new Shaft({ name: 'flywheel (4in urethane)', inertia: SHOOTER.J }), -1);
  const stealth = shooter.add(new Shaft({ name: 'hood Stealth wheels', inertia: SHOOTER.Jstealth }), 1);
  const feed = shooter.add(new Shaft({ name: '1in urethane wheel', inertia: SHOOTER.Jfeed }), -1);
  // the intake's pivot (positive turns the arm down, out)
  const pivot = new Shaft({ name: 'intake pivot', motor: new Motor('krakenX44', { ratio: 3.4 * (48 / 14) * (48 / 16), statorLimit: 40, supplyLimit: 30 }), inertia: INTAKE.J, drag: 0.3 });
  return { intake, bottom, top, pivotRoller, dead, serializer, tableA, tableB, upBack, upFront, feederT, feeder, omniFree, shooter, fly, stealth, feed, pivot };
}

// a mecanum's hand: a FUEL under it (the shaft turning so its bottom runs rearward) is pushed
// toward dz (+1: the robot's right), as 8793's (js/mech/r8793.js)
const hand = (dz) => 45 * Math.sign(-dz);
// the wheels (robot frame; the intake's posed with placeIntake, the shooter's with placeShooter)
export function wheels341(t) {
  const Z = [0, 0, 1], Y = [0, 1, 0];
  const comp = (shaft, x, y, z, name) => new Wheel({ name, shaft, center: [x, y, z], axis: Z, radius: 2.25 * IN / 2, width: 1 * IN, mu: MU.compliant, tread: 5000 });
  const mec = (shaft, x, y, z, dz, name) => new Wheel({ name, shaft, kind: 'mecanum', mecanum: hand(dz), center: [x, y, z], axis: Z, radius: 2 * IN / 2 * 1.05, width: 0.9 * IN, mu: MU.mecanum, tread: 20000 });
  const [px, py] = PIVOT, I = INTAKE;
  const pivotRow = [
    ...I.compliant.map((z) => comp(t.pivotRoller, px, py, z, 'intake top shaft compliant')),
    ...I.left.map((z) => mec(t.pivotRoller, px, py, z, 1, 'intake top shaft mecanum')),
    ...I.right.map((z) => mec(t.pivotRoller, px, py, z, -1, 'intake top shaft mecanum')),
  ];
  const arm = [
    new Wheel({ name: 'intake bottom roller (silicone)', shaft: t.bottom, center: [...I.bottom.at, 0], axis: Z, radius: I.bottom.d / 2, width: I.bottom.w, mu: MU.silicone, tread: 20000 }),
    new Wheel({ name: 'intake top roller (silicone)', shaft: t.top, center: [...I.top.at, 0], axis: Z, radius: I.top.d / 2, width: I.top.w, mu: MU.silicone, tread: 20000 }),
    ...I.dead.map((at, i) => new Wheel({ name: t.dead[i].name, shaft: t.dead[i], center: [...at, 0], axis: Z, radius: I.deadD / 2, width: I.deadW, mu: MU.polycarb })),
  ];
  arm.forEach((w) => { w.home = w.center.slice(); });
  const T = TABLE;
  const table = [
    ...T.a.compliant.map((z) => comp(t.tableA, T.a.x, T.y, z, 'table front roller compliant')),
    ...T.a.right.map((z) => mec(t.tableA, T.a.x, T.y, z, -1, 'table front roller mecanum')),
    ...T.b.compliant.map((z) => comp(t.tableB, T.b.x, T.y, z, 'table back roller compliant')),
    ...T.b.left.map((z) => mec(t.tableB, T.b.x, T.y, z, 1, 'table back roller mecanum')),
    ...T.b.right.map((z) => mec(t.tableB, T.b.x, T.y, z, -1, 'table back roller mecanum')),
  ];
  const U = UPTAKE;
  const uptake = [
    ...U.z.map((z) => new Wheel({ name: 'uptake back flex wheel', shaft: t.upBack, center: [...U.back, z], axis: Z, radius: U.d / 2, width: 1 * IN, mu: MU.compliant, tread: 4000 })),
    ...U.z.map((z) => new Wheel({ name: 'uptake front flex wheel', shaft: t.upFront, center: [...U.front, z], axis: Z, radius: U.d / 2, width: 1 * IN, mu: MU.compliant, tread: 4000 })),
  ];
  // the omnis lie flat: axis up (the powered one's face turning toward the uptake, at -z)
  // Where the zipper can lock up: a FUEL from the right, pressed onto the powered omni's front by
  // the ones behind it (100-170 N), is rubbed toward the lane and spun, and pins the lane's FUEL
  // against the left wall, dragging it forward, where the front top roller's compliant wheels slip
  // on it. At their 3 V the omni slows to a third of its speed against that FUEL and grinds it
  // into the lane (tools/mechlab341.mjs path: 12 of 48 runs lock; none with the feeder stopped;
  // 3 at 6 V). Their unjam can't see it (nothing stalls), so in play the AI's driver lets go of
  // the trigger and presses again (js/mech/phys341.js letGo).
  const omnis = [
    new Wheel({ name: 'feeder 6in omni', shaft: t.feeder, kind: 'omni', center: [OMNI.feeder[0], OMNI.y, OMNI.feeder[1]], axis: Y, radius: OMNI.d / 2, width: OMNI.w, mu: MU.omni }),
    ...OMNI.free.map(([x, z], i) => new Wheel({ name: t.omniFree[i].name, shaft: t.omniFree[i], kind: 'omni', center: [x, OMNI.y, z], axis: Y, radius: OMNI.d / 2, width: OMNI.w, mu: MU.omni })),
  ];
  return { pivotRow, arm, table, uptake, omnis };
}
export function shooterWheels(t) {
  const S = SHOOTER, Z = [0, 0, 1];
  const w = [
    new Wheel({ name: 'flywheel (4in urethane)', shaft: t.fly, center: [0, 0, 0], axis: Z, radius: S.fly.d / 2, width: S.fly.w, mu: MU.urethane, tread: 40000 }),
    new Wheel({ name: '1in urethane wheel', shaft: t.feed, center: [0, 0, 0], axis: Z, radius: S.feed.d / 2, width: S.feed.w, mu: MU.urethane, tread: 20000 }),
    ...S.hood.map((_, i) => new Wheel({ name: `hood Stealth row ${i + 1}`, shaft: t.stealth, center: [0, 0, 0], axis: Z, radius: S.hoodD / 2, width: S.hoodW, mu: MU.stealth, tread: 30000 })),
  ];
  placeShooter(w, 0, 0);
  return w;
}

// the arm's turn from the CAD's pose (rad) at their angle deg (0 folded, 149 down: forward)
export function intakeRot(deg) { return (-(deg - INTAKE.cad) * Math.PI) / 180; }
const rot = ([x, y], [cx, cy], a) => { const c = Math.cos(a), s = Math.sin(a), dx = x - cx, dy = y - cy; return [cx + dx * c - dy * s, cy + dx * s + dy * c]; };
export function placeIntake(wheels, deg) {
  const a = intakeRot(deg);
  for (const w of wheels) { const [x, y] = rot(w.home, PIVOT, a); w.center = [x, y, 0]; }
}
// the hood's turn about the flywheel's axle (rad; over the top, forward) at their angle deg
const lerpT = (t, x) => { for (let k = 1; k < t.length; k++) if (x <= t[k][0]) return t[k - 1][1] + ((t[k][1] - t[k - 1][1]) * (x - t[k - 1][0])) / (t[k][0] - t[k - 1][0]); return t[t.length - 1][1]; };
export function hoodRot(deg) { return (-lerpT(SHOOTER.hoodTurn, Math.max(0, Math.min(SHOOTER.hoodMax, deg))) * Math.PI) / 180; }
// the shooter's wheels where the turret (yaw, rad, about up; 0 facing forward) and the hood
// (their angle) put them
export function placeShooter([fly, feed, ...hood], yaw, hoodDeg) {
  const c = Math.cos(yaw), s = Math.sin(yaw), S = SHOOTER;
  const at = ([x, y]) => [TURRET[0] + x * c, y, TURRET[1] - x * s];
  fly.center = at(S.fly.at); feed.center = at(S.feed.at);
  const h = hoodRot(hoodDeg);
  S.hood.forEach((p, i) => { const [hx, hy] = rot(p, [0, 0], h); hood[i].center = at([S.fly.at[0] + hx, S.fly.at[1] + hy]); });
  for (const w of [fly, feed, ...hood]) w.axis = [s, 0, c];
}
// where FUEL leave, about (turret frame, facing forward): over the flywheel, past the hood's last
// row of Stealth wheels, tangent to the flywheel; and the angle up (deg) the CAD's geometry gives
export function hoodElevation(deg) {
  const [hx, hy] = SHOOTER.hood[SHOOTER.hood.length - 1], th = Math.atan2(hy, hx) + hoodRot(deg);
  return (Math.atan2(-Math.cos(th), Math.sin(th)) * 180) / Math.PI;
}

// the CAD's surfaces: the body; the intake's arm on the pivot's shaft (it swings with it); the
// turret and the hood (posed each step)
// The front bumper, which FUEL climb on the way in (their CAD leaves it out): XLPE foam under
// fabric (their binder: stiffer and more even than pool noodles), its face and top. (Its tread,
// N/m, an estimate: a FUEL pressed into a 2.5in slab of foam that takes ~0.25 MPa at 25% squeeze
// sinks in about 5-8 mm at 20-50 N, 13-18 kN/m; far stiffer than the FUEL, which does most of the
// giving.)
export function bumperSurface({ mu = BUMPER_MU, tread = 20000 } = {}) {
  // (the CAD's bumper wood: 0.345-0.364 m out, 0.041-0.168 m up; 2.5in of foam on it)
  const x0 = HALF, x1 = 0.364 + 2.5 * IN, hw = x1, y0 = 0.041, y1 = 0.168;
  const V = [x1, y0, -hw, x1, y0, hw, x1, y1, hw, x1, y1, -hw, x0, y1, -hw, x0, y1, hw];
  return new Surface({ name: 'bumper', vertices: V, indices: [0, 1, 2, 0, 2, 3, 3, 2, 5, 3, 5, 4], mu, tread });
}
export function pathSurfaces341(mesh, t, { mu = 0.35 } = {}) {
  const s = (k, pos) => new Surface({ name: `${k} (CAD)`, vertices: mesh[k].v.map((x) => x / 1000), indices: mesh[k].i, mu, pos });
  const body = s('body', [0, 0, 0]), bumper = bumperSurface();
  // (positive on the pivot swings the arm down, clockwise looking from the robot's right: about -z)
  const intake = s('intake', [PIVOT[0], PIVOT[1], 0]).onShaft(t.pivot, [PIVOT[0], PIVOT[1], 0], [0, 0, -1]);
  const turret = s('turret', [TURRET[0], 0, TURRET[1]]);
  const hood = s('hood', [TURRET[0] + SHOOTER.fly.at[0], SHOOTER.fly.at[1], TURRET[1]]);
  return { body, bumper, intake, turret, hood };
}
export function placeTurretSurfaces({ turret, hood }, yaw, hoodDeg) {
  turret.setPose([TURRET[0], 0, TURRET[1]], yaw, 0);
  const c = Math.cos(yaw), s = Math.sin(yaw), fx = SHOOTER.fly.at[0];
  hood.setPose([TURRET[0] + fx * c, SHOOTER.fly.at[1], TURRET[1] - fx * s], yaw, hoodRot(hoodDeg));
}
