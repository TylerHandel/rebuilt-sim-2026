// 1678's Limestone as physical mechanisms (js/mech/mechanism.js), from their CAD (Onshape
// "1678-26c-0000 CAD Release") and their code (frc1678/C2026-Public: the roller, tunnel, feeder,
// shooter, hood and intake-deploy subsystems' constants, Superstructure and the shooting
// regressions). Robot frame: x forward (the intake), y up, z to its right; the CAD's x across,
// y back, z up.
//
// The ball path: the slapdown intake's 2in silicone roller, behind its crash bar, pulls FUEL in
// under it and pinches them against its 1.25in kicker bar (locked) at the front of the bumper,
// rolling them up over it and a free roller on the bumper's top; the arm's top, down, is the
// extension's floor. The roller floor (polycarbonate rollers, then three
// shafts of 2in flex wheels, then two more polycarbonate rollers) slopes down to the back, where
// six rollers curve up the back of the ball tunnel; against them, on the front, the feeder's 2in
// roller and two more tunnel rollers pinch FUEL three wide up to the drum (3.5in, silicone) and
// its hood's three silicone rollers, which fling them up and back over the drum. All three wide.
// Drives (the CAD's belts and gears, their code's gearings):
//   intake rollers: two Kraken X60s (80 A stator, 50 A supply), 16t:30t (HTD) to the roller
//   roller floor: one X60 (90 A, 45 A), 12t:36t onto the first flex-wheel shaft, the others
//     15t:15t from it; its 30t pulleys drive the polycarbonate rollers' ~18t ends
//   tunnel: two X60s (60 A, 10 A supply), 12t:21t onto the bottom roller, then each roller up the
//     back belted 21t:19t from the one below (their gearing (21/12)(19/21)^5: the top one turns
//     at 1/1.061 of the motors); the fifth drives the front two through a reversing gear pair
//   feeder: one X60 (50 A, 25 A), 12t:30t to its 2in roller
//   shooter: four X60s (80 A, 30 A), 18t:24t to the drum, whose 18t belts the hood's idler at
//     1:1, geared 28t:28t to the hood rollers (turning the other way)
//   intake pivot: an X60 through 62:8, 68:18 and 15:9 (#35 chain); 48.8:1
//   hood: an X44 through 48:8, 18:15 and 175:10, about the drum's axle
import { IN, FUEL } from '../constants.js';
import { Motor, Shaft, Surface, Train, Wheel } from './mechanism.js';

const R = FUEL.radius;
// what each grips FUEL with (estimates; the order 8793's team gave: compliant wheels most,
// silicone and rubber, bare polycarbonate least)
export const MU = { compliant: 1.6, silicone: 1.0, polycarb: 0.5 };

// ------------------------------------------------------------------ the layout (robot frame)
export const PIVOT = [0.305, 0.169]; // the intake's pivot tube
export const DRUM = [-0.288, 0.477]; // the drum's axle, which the hood turns about
export const INTAKE = {
  // as the CAD has it: folded up inside the hopper (their 130 deg; 0 is down)
  // the roller: the CAD's 2in tube, its silicone 1/4in thicker than the CAD's (2.53in over it) and
  // softer for it (tread, N/m) and grippier (mu). As the CAD has it, a FUEL resting against the
  // kicker bar is squeezed only 3 mm and gripped so lightly that, driving into the NEUTRAL ZONE's
  // FUEL, they pile up against the bar faster than it lifts them (tools/intake-test.mjs at 3 m/s:
  // 21-38 of the 52 in its path; 35-42 with this, and 45-52 of 52 at 1.5 m/s against 36-45).
  // Thicker still and the gap to the kicker bar closes up (2.78in: 22-37 at 1.5 m/s).
  main: { at: [0.143, 0.368], d: Number(globalThis.process?.env?.IN_D ?? 0.0643), mu: Number(globalThis.process?.env?.IN_MU ?? 1.3), tread: Number(globalThis.process?.env?.IN_TREAD ?? 10000) },
  // the 1.25in kicker bar at the front of the bumper when it's down: locked in place, not free to
  // turn (the team); the roller rolls FUEL up over it
  kicker: { at: [0.312, 0.322], d: 0.0326 },
  // free polycarbonate sleeves on dead shafts (one rides on the bumper's top when it's down)
  idlers: [{ at: [0.253, 0.247], d: 0.0188 }, { at: [0.131, 0.329], d: 0.0188 }],
  stowed: 130, partIn: 50,
  // the CAD's folded pose, in their angles. Their encoder is zeroed folded (130) and the arm comes
  // down until the free sleeve at its back sits on the bumper's top (0.146 m up): 2 deg past where
  // 130 deg from the CAD's pose would put it, which leaves that sleeve hovering 3 mm over the
  // bumper and the roller 4 mm out of reach of a FUEL resting against the kicker bar (a dead spot)
  cad: 132,
  // down: their 0 (the CAD's pose turned 132 deg). A FUEL on the carpet against the kicker bar is
  // then squeezed by the roller, which rolls it up over the bar (with the arm 2 deg higher, FUEL
  // sat on the bar out of the roller's reach)
  down: Number(globalThis.process?.env?.IN_DOWN ?? 0),
  // its arm (plates, roller, two X60s, crash bar): their sim's 0.222 kg m^2 about the pivot
  mass: 4.2, com: 0.22, comDeg: -1, J: 0.222,
};
export const FLOOR = [[0.323, 0.215], [0.262, 0.208], [0.214, 0.202], [-0.015, 0.126], [-0.075, 0.115]]; // 1in polycarbonate
export const FLEX = [[0.159, 0.179], [0.101, 0.160], [0.042, 0.142]]; // 16 2in 30A flex wheels each
// up the back of the tunnel, bottom to top; and the front's two
export const BACK = [[-0.147, 0.122], [-0.201, 0.154], [-0.247, 0.212], [-0.266, 0.284], [-0.266, 0.346], [-0.266, 0.408]];
export const FRONT = [[-0.090, 0.346], [-0.090, 0.408]];
export const FEEDER = { at: [-0.082, 0.294], d: 0.0516 };
export const TUNNEL_D = 0.0302;
export const SHOOTER = {
  drum: { d: 0.092, width: 0.563 },
  hood: [{ at: [-0.094, 0.448] }, { at: [-0.093, 0.490] }, { at: [-0.101, 0.536] }], // at hood 17.5 (the CAD's)
  hoodD: 0.0413, hoodWidth: 0.619,
  hoodMin: 17.5, hoodMax: 52.5, // their kMinAngle, kMaxAngle (17.5: the CAD's, its top roller 17.5 deg up about the drum)
  // how far the hood turns about the drum's axle (deg) at their angle: calibrated so their HUB
  // regression lands every shot (tools/mechlab1678.mjs shooter). The FUEL leave at a steady 0.71 of
  // the drum's surface speed, and their table only works if the shot flattens about 2.2 deg per
  // degree of their hood angle over 17.5-26.5 (a 1:1 turn leaves the 4.75 m shot a meter short);
  // the CAD doesn't show how the hood's linkage moves (its sector gear meshes a pinion 0.12 m
  // from the drum). Past that it's eased so their ferry angles (43-52.5) come out at 35-45 deg.
  hoodTurn: [[17.5, 0], [26.5, 20], [52.5, 37]],
  drumPerMotor: 18 / 24,
  // inertia (kg m^2), each about its own axis from the CAD's solids (OpenCascade's volume
  // properties on the STEP) and the BOM's materials; README, *1678*, has the table. The drum: its
  // 3.5in tube (the BOM's 1020 steel, 2.0 kg), the silicone over it, its 6061 end pieces and shaft,
  // the two 24t pulleys, the 18t to the hood and the 50t PLA pulley, and the belts. The inertia
  // flywheel the 50t drives 50t:15t (3.33 times the drum; it isn't in their sim): a 5in bronze ring
  // (the BOM's leaded bronze, 0.57 kg) round a 4in 6061 disk, 2.30e-3 about its axis, 0.0256 seen
  // at the drum. The drum and flywheel together are 0.0304 at the drum: their sim's 0.0167 is that
  // seen at the motors (0.0171), not at the drum, where its FlywheelSim puts it. The hood: the
  // three rollers (aluminum tubes and shafts, the silicone over them, 28t gears, 18t pulleys) and
  // the idler that drives them, 5.2e-4 at the drum's speed. The motors' 18t and 12t pulleys and
  // their 40T belts (at the motors' speed). (The motors' rotors are added by the Train.)
  drumJ: 4.85e-3, flywheelJ: 2.30e-3, flywheelPerDrum: 50 / 15, hoodJ: 5.18e-4, motorJ: 7.9e-6,
  // their velocity loop (Slot1, volts; drum rotations/s): kV x target + kP x error + kS
  kV: (12 / 100) * (24 / 18) * 0.995, kP: 0.8, kS: 0.32,
  slow: 2100, // their kSlow: where it idles between shots
};
export const TUNNEL = {
  // each roller's turns per turn of the motors, up the back (the top one's is 1 / their kGearing)
  backPerMotor: BACK.map((_, i) => (12 / 21) * (21 / 19) ** i),
  frontPerMotor: [(12 / 21) * (21 / 19) ** 4, (12 / 21) * (21 / 19) ** 5],
  // their velocity loop (Slot1; top roller rotations/s)
  kV: (12 / 100) * ((21 / 12) * (19 / 21) ** 5) * 1.02, kP: 1.75, kS: 0.36,
  up: 5000, slow: 1250, preload: 250, // RPM
};
const interp = (t, x) => {
  if (x <= t[0][0]) return t[0][1];
  for (let k = 1; k < t.length; k++) if (x <= t[k][0]) return t[k - 1][1] + ((t[k][1] - t[k - 1][1]) * (x - t[k - 1][0])) / (t[k][0] - t[k - 1][0]);
  return t[t.length - 1][1];
};
// their regressions (ShooterVelocityRegressions): distance to the target (m) -> drum RPM, hood
// angle, top tunnel roller RPM
const table = (d, rpm, hood, tunnel) => ({ rpm: d.map((x, i) => [x, rpm[i]]), hood: d.map((x, i) => [x, hood[i]]), tunnel: d.map((x, i) => [x, tunnel[i]]) });
export const REGRESSIONS = {
  hub: table([0, 1.7, 2.0, 2.215, 2.495, 2.888, 3.222, 3.57, 3.83, 4.15, 4.75],
    [1815, 1855, 1875, 1940, 2015, 2065, 2130, 2200, 2240, 2340, 2365],
    [17.5, 17.5, 18.5, 19.0, 19.5, 20.5, 21.5, 22.5, 23.0, 24.5, 26.5],
    [1400, 1425, 1439.6, 1450, 1500, 1550, 1650, 1750, 1800, 1950, 2150]),
  // (their neutral and alliance ferry tables are the same)
  ferry: table([0, 4.977, 5.976, 6.902, 7.912, 8.477, 9.029, 9.59, 10.061],
    [1800, 1800, 1850, 1900, 2050, 2100, 2200, 2400, 2550],
    [52.5, 52.5, 51.5, 50.5, 49.5, 48.5, 46.5, 45.5, 43.5],
    [1600, 1650, 1700, 1750, 1850, 1900, 2000, 2200, 2350]),
  far: table([0, 12.46, 13.133], [3100, 3150, 3500], [45, 45, 45], [1400, 1400, 1300]),
};
// Their ferry and far tables re-found by shooting (tools/mechlab1678.mjs tune): their distances,
// hood angles and tunnel speeds, and the drum RPM at which the sim's FUEL, fed up the tunnel as
// they are in play, come down on the carpet at the row's distance from the robot's center. Their
// own RPMs put every pass 0.7-5 m short here, for two reasons. Up to ~2500 RPM the FUEL leave at
// 0.71 of the drum's surface speed (the drum's and the hood rollers', geared 1:1 on a 3.5in drum
// and 1.63in rollers, averaged) and fly drag-only, with no Magnus lift for the backspin the slower
// hood rollers put on, so they need 10-30% more. Past that the FUEL slip on the drum: carried
// round its 0.12 m radius (drum and FUEL) at 8.5 m/s they need ~130 N toward it, about all the
// 2 cm squeeze between drum and hood rollers presses them with (the foam's measured, static curve,
// constants.js), so the hood side takes the load, the drum's grip goes, and they leave at no more
// than ~9 m/s however fast the drum turns: no more than ~8 m on the carpet, at any hood angle.
// Rows past that are flat out (4200 RPM: the drum gets to 4350 under its loop) and land short.
// THEIR_PASS=1 in the environment: their own RPMs. (The HUB's land as they are: the hood's turn,
// SHOOTER.hoodTurn, was fitted to them.)
export const SIM_PASS = {
  ferry: [1990, 1990, 2240, 2465, 4200, 4200, 4200, 4200, 4200],
  far: [4200, 4200, 4200],
};
const THEIR_PASS = typeof process !== 'undefined' && !!process.env?.THEIR_PASS;
export function shotFor(kind, d, { theirs = THEIR_PASS } = {}) {
  const t = REGRESSIONS[kind], sim = !theirs && SIM_PASS[kind];
  const rpm = sim ? t.rpm.map(([x], i) => [x, sim[i]]) : t.rpm;
  return { rpm: interp(rpm, d), hood: interp(t.hood, d), tunnel: interp(t.tunnel, d) };
}
// their CANranges: the tunnel's, on the left side plate looking across the tunnel (0.6 m), and the
// hopper's, at the back of the hopper looking forward along it (0.3 m, 27 deg wide)
export const RANGES = { tunnel: { at: [-0.178, 0.379], z: -0.327, reach: 0.6 }, hopper: { at: [-0.04, 0.45], z: -0.285, reach: 0.3, half: 13.5 } };
// the preload, eight on the floor (robot frame)
export const PRELOAD = [
  [0.21, 0.30, -0.225], [0.21, 0.30, -0.075], [0.21, 0.30, 0.075], [0.21, 0.30, 0.225],
  [0.05, 0.25, -0.225], [0.05, 0.25, -0.075], [0.05, 0.25, 0.075], [0.05, 0.25, 0.225],
];

// ------------------------------------------------------------------ the drives
export function build1678() {
  // the intake's roller: positive turns it inward (its bottom runs rearward, up its back); and
  // its free sleeves
  const intake = new Train({ name: 'intake roller', motor: new Motor('krakenX60', { count: 2, statorLimit: 80, supplyLimit: 50 }) });
  const main = intake.add(new Shaft({ name: 'intake 2in roller', inertia: 1.6e-4 }), -16 / 30);
  // (the kicker bar: locked, it never turns)
  const kicker = new Shaft({ name: 'intake kicker bar (locked)', inertia: Infinity });
  const idlers = INTAKE.idlers.map((w, i) => new Shaft({ name: `intake free roller ${i + 1}`, inertia: 1.5e-5, drag: 2e-5 }));
  // the roller floor: positive carries FUEL back along it
  const floor = new Train({ name: 'roller floor', motor: new Motor('krakenX60', { statorLimit: 90, supplyLimit: 45 }) });
  const flex = FLEX.map((_, i) => floor.add(new Shaft({ name: `flex wheel shaft ${i + 1}`, inertia: 8e-5 }), 12 / 36));
  const rollers = FLOOR.map((_, i) => floor.add(new Shaft({ name: `floor roller ${i + 1}`, inertia: 2e-5 }), (12 / 36) * (30 / 18)));
  // the tunnel: positive lifts FUEL (the back rollers' fronts and the front rollers' backs run up)
  const tunnel = new Train({ name: 'tunnel', motor: new Motor('krakenX60', { count: 2, statorLimit: 60, supplyLimit: 10 }) });
  const back = BACK.map((_, i) => tunnel.add(new Shaft({ name: `tunnel back roller ${i + 1}`, inertia: 6e-5 }), TUNNEL.backPerMotor[i]));
  const front = FRONT.map((_, i) => tunnel.add(new Shaft({ name: `tunnel front roller ${i + 1}`, inertia: 6e-5 }), -TUNNEL.frontPerMotor[i]));
  const feederT = new Train({ name: 'feeder', motor: new Motor('krakenX60', { statorLimit: 50, supplyLimit: 25 }) });
  const feeder = feederT.add(new Shaft({ name: 'feeder 2in roller', inertia: 2.5e-4 }), -12 / 30);
  // the shooter: positive runs the drum's front up, the hood rollers' backs up
  const shooter = new Train({ name: 'shooter', motor: new Motor('krakenX60', { count: 4, statorLimit: 80, supplyLimit: 30 }) });
  const S = SHOOTER;
  const drum = shooter.add(new Shaft({ name: 'drum', inertia: S.drumJ }), S.drumPerMotor);
  // (the inertia flywheel and the motors' pulleys: inertia only, the train's drag stays the two
  // shafts' it was tuned with)
  shooter.add(new Shaft({ name: 'bronze inertia flywheel (50t:15t)', inertia: S.flywheelJ, drag: 0 }), S.drumPerMotor * S.flywheelPerDrum);
  shooter.add(new Shaft({ name: 'motors\' pulleys', inertia: S.motorJ, drag: 0 }), 1);
  const hood = shooter.add(new Shaft({ name: 'hood rollers', inertia: S.hoodJ }), -S.drumPerMotor);
  // the intake's pivot (positive turns the arm up, toward stowed)
  const pivot = new Shaft({ name: 'intake pivot', motor: new Motor('krakenX60', { ratio: (62 / 8) * (68 / 18) * (15 / 9), statorLimit: 60, supplyLimit: 40 }), inertia: INTAKE.J, drag: 0.5 });
  return { intake, main, kicker, idlers, floor, flex, rollers, tunnel, back, front, feederT, feeder, shooter, drum, hood, pivot };
}

// the wheels (robot frame; the intake's and hood's posed with placeIntake / placeHood)
export function wheels1678(t) {
  const Z = [0, 0, 1];
  const floor = [
    ...FLOOR.map(([x, y], i) => new Wheel({ name: t.rollers[i].name, shaft: t.rollers[i], center: [x, y, -0.013], axis: Z, radius: 0.0125, width: 0.529, mu: MU.polycarb })),
    ...FLEX.map(([x, y], i) => new Wheel({ name: `${t.flex[i].name} (2in 30A flex wheels)`, shaft: t.flex[i], center: [x, y, 0], axis: Z, radius: 1 * IN, width: 0.514, mu: MU.compliant, tread: 3000 })),
  ];
  const tunnel = [
    ...BACK.map(([x, y], i) => new Wheel({ name: t.back[i].name, shaft: t.back[i], center: [x, y, 0], axis: Z, radius: TUNNEL_D / 2, width: 0.55, mu: MU.silicone, tread: 20000 })),
    ...FRONT.map(([x, y], i) => new Wheel({ name: t.front[i].name, shaft: t.front[i], center: [x, y, 0], axis: Z, radius: TUNNEL_D / 2, width: 0.544, mu: MU.silicone, tread: 20000 })),
    new Wheel({ name: t.feeder.name, shaft: t.feeder, center: [...FEEDER.at, 0.012], axis: Z, radius: FEEDER.d / 2, width: 0.564, mu: MU.silicone, tread: 20000 }),
  ];
  const intake = [
    new Wheel({ name: 'intake 2in roller (silicone)', shaft: t.main, center: [...INTAKE.main.at, 0], axis: Z, radius: INTAKE.main.d / 2, width: 0.618, mu: INTAKE.main.mu, tread: INTAKE.main.tread }),
    new Wheel({ name: 'intake kicker bar (locked)', shaft: t.kicker, center: [...INTAKE.kicker.at, 0], axis: Z, radius: INTAKE.kicker.d / 2, width: 0.66, mu: Number(globalThis.process?.env?.KICK_MU ?? MU.polycarb) }),
    ...INTAKE.idlers.map((w, i) => new Wheel({ name: t.idlers[i].name, shaft: t.idlers[i], center: [...w.at, 0], axis: Z, radius: w.d / 2, width: 0.66, mu: MU.polycarb })),
  ];
  intake.forEach((w) => { w.home = w.center.slice(); });
  const drum = new Wheel({ name: 'drum (silicone)', shaft: t.drum, center: [...DRUM, -0.009], axis: Z, radius: SHOOTER.drum.d / 2, width: SHOOTER.drum.width, mu: MU.silicone });
  const hood = SHOOTER.hood.map((h, i) => new Wheel({ name: `hood roller ${i + 1} (silicone)`, shaft: t.hood, center: [...h.at, 0], axis: Z, radius: SHOOTER.hoodD / 2, width: SHOOTER.hoodWidth, mu: MU.silicone }));
  hood.forEach((w) => { w.home = w.center.slice(); });
  return { floor, tunnel, intake, drum, hood };
}
// rotate a point (x, y) about (cx, cy) by a (rad)
const rot = ([x, y], [cx, cy], a) => { const c = Math.cos(a), s = Math.sin(a), dx = x - cx, dy = y - cy; return [cx + dx * c - dy * s, cy + dx * s + dy * c]; };
// the arm's turn from the CAD's pose at their angle deg (0 down, 130 folded up)
export function intakeRot(deg) { return ((deg - INTAKE.cad) * Math.PI) / 180; }
export function placeIntake(wheels, deg) {
  const a = intakeRot(deg);
  for (const w of wheels) { const [x, y] = rot(w.home, PIVOT, a); w.center = [x, y, 0]; }
}
// the hood at their angle deg (17.5 to 52.5): its turn about the drum's axle from the CAD's pose
export function hoodRot(deg) { return (interp(SHOOTER.hoodTurn, deg) * Math.PI) / 180; }
export function placeHood(wheels, deg) {
  const a = hoodRot(deg);
  for (const w of wheels) { const [x, y] = rot(w.home, DRUM, a); w.center = [x, y, 0]; }
}
// the shot's elevation at hood angle deg: about square to the line from the drum's axle to the top
// hood roller, out the back (72.5 deg in the CAD's pose; the FUEL leave ~2 deg steeper)
export function hoodElevation(deg) { return 74 - interp(SHOOTER.hoodTurn, deg); }

// the ball path's surfaces from their CAD (js/mech/path1678.js): the body; the intake on its pivot
// (it swings, on the pivot's shaft); the hood on the drum's axle; the extension's sides and front
// (they slide out); the lid on the climber (it rises); and the net from the lid's front edge to
// the extension's front (it stretches: Net1678, placed with placeNet)
export function pathSurfaces1678(mesh, t, { mu = 0.35 } = {}) {
  const s = (k, pos) => new Surface({ name: `${k} (CAD)`, vertices: mesh[k].v.map((x) => x / 1000), indices: mesh[k].i, mu, pos });
  const body = s('body', [0, 0, 0]);
  const intake = s('intake', [PIVOT[0], PIVOT[1], 0]).onShaft(t.pivot, [PIVOT[0], PIVOT[1], 0]);
  const hood = s('hood', [DRUM[0], DRUM[1], 0]);
  const slide = s('slide', [0, 0, 0]);
  const lid = s('lid', [0, 0, 0]);
  const net = new Net1678();
  return { body, intake, hood, slide, lid, net };
}
// the net between the lid's front edge (lid raised lift) and the top of the extension's front
// panel (out ext), across hw either side; folded away while the extension's in
export const NET = { lidX: 0.335, lidY: 0.545, frontX: 0.335, frontY: 0.505, hw: 0.34, bulge: 0.12 };
// The extension rides on the intake: a pin on the arm's side plate, 0.173 m from the pivot, runs
// in an angled slot in the extension's side plate (which slides fore and aft on its rail), so the
// arm's angle sets how far out it is: all the way (0.30 m) with the arm down, in with it folded,
// halfway at their 80 deg (their contractHopper pulls it in as it lifts the intake). From the CAD.
export const SLOT = { pin: [0.158, 0.2597], at: [0.1543, 0.2697], dir: [-0.2079, 0.9782], mass: 2.5 };
const slotPin = (deg) => {
  const [px, py] = SLOT.pin, r = Math.hypot(px - PIVOT[0], py - PIVOT[1]);
  const f = Math.atan2(py - PIVOT[1], px - PIVOT[0]) + intakeRot(deg);
  return [PIVOT[0] + r * Math.cos(f), PIVOT[1] + r * Math.sin(f)];
};
// the slot's centerline x at height y (the extension's plate, extension in)
const slotX = (y) => SLOT.at[0] + ((y - SLOT.at[1]) * SLOT.dir[0]) / SLOT.dir[1];
const SLOT0 = SLOT.pin[0] - slotX(SLOT.pin[1]);
// how far out the extension is (m) with the arm at their angle deg
export function slotExt(deg) { const [x, y] = slotPin(deg); return Math.max(0, x - slotX(y) - SLOT0); }
// d(extension)/d(arm angle), m per radian (the arm's angle up)
export function slotRate(deg) { return ((slotExt(deg + 0.5) - slotExt(deg - 0.5)) * 180) / Math.PI; }
export function placeNet(net, ext, lift) { net.place(ext, lift); }
// The net: tied along the lid's front edge, the top of the extension's front panel and its side
// plates' (the diagonal), it stretches, and the load bulges it (the team), up to NET.bulge at its
// middle, tapering to its ties (as 4414's net dome does, js/mech/r4414.js netY): 0.12 m, about up
// to the lid's height there (the diagonal's 0.13 under it), an estimate. A FUEL under it presses
// it out to there, past it only as far as its soft tread gives (as 4414's); nothing pushes the load
// back down. In the robot's frame, so its pose stays put.
class Net1678 extends Surface {
  constructor() {
    super({ name: 'net', vertices: [0, 0, 0, 0.01, 0, 0, 0, 0, 0.01], indices: [0, 1, 2], mu: 0.3, tread: 1500 });
    this.place(0, 0);
  }
  place(ext, lift) {
    Object.assign(this, { ext, x0: NET.lidX, x1: NET.frontX + ext, y0: NET.lidY + lift, y1: NET.frontY });
  }
  // the stretched net's height at (x, z), and its slope along x and z
  at(x, z) {
    const L = this.x1 - this.x0, u = (x - this.x0) / L, v = (z + NET.hw) / (2 * NET.hw), B = NET.bulge;
    const su = Math.sin(Math.PI * u), sv = Math.sin(Math.PI * v);
    return [this.y0 + (this.y1 - this.y0) * u + B * su * sv, (this.y1 - this.y0) / L + (B * Math.PI * Math.cos(Math.PI * u) * sv) / L, (B * su * Math.PI * Math.cos(Math.PI * v)) / (2 * NET.hw)];
  }
  touches(p) {
    if (this.ext < 0.05) return [];
    const [x, y, z] = p;
    if (x <= this.x0 || x >= this.x1 || Math.abs(z) >= NET.hw) return [];
    const [h, hx, hz] = this.at(x, z), l = Math.hypot(hx, 1, hz), dist = (h - y) / l;
    // (under it, or pushed a little way through: not one that's got out over it)
    if (dist >= R || dist < -0.03) return [];
    return [{ depth: R - dist, n: [hx / l, -1 / l, hz / l] }];
  }
}
