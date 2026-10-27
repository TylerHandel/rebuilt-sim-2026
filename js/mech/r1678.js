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
import { IN } from '../constants.js';
import { Motor, Shaft, Surface, Train, Wheel } from './mechanism.js';

// what each grips FUEL with (estimates; the order 8793's team gave: compliant wheels most,
// silicone and rubber, bare polycarbonate least)
export const MU = { compliant: 1.6, silicone: 1.0, polycarb: 0.5 };

// ------------------------------------------------------------------ the layout (robot frame)
export const PIVOT = [0.305, 0.169]; // the intake's pivot tube
export const DRUM = [-0.288, 0.477]; // the drum's axle, which the hood turns about
export const INTAKE = {
  // as the CAD has it: folded up inside the hopper (their 130 deg; 0 is down)
  // 2in silicone-covered roller: the CAD's tube is 2.03in with no cover drawn; a 1/16in silicone
  // wall over it (an estimate) makes 2.16in
  main: { at: [0.143, 0.368], d: 0.0548 },
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
  // then squeezed 3 mm by the roller, which rolls it up over the bar; driving into the NEUTRAL
  // ZONE's FUEL at 3 m/s it takes in about 34 in 2 s (against 15 with the arm 2 deg higher and the
  // bare tube, when FUEL sat on the bar out of the roller's reach)
  down: 0,
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
  // the drum and what turns with it (their sim: 0.0167 kg m^2 at the drum)
  drumJ: 0.016,
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
export function shotFor(kind, d) {
  const t = REGRESSIONS[kind];
  return { rpm: interp(t.rpm, d), hood: interp(t.hood, d), tunnel: interp(t.tunnel, d) };
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
  const hood = shooter.add(new Shaft({ name: 'hood rollers', inertia: 3e-4 }), -S.drumPerMotor);
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
    new Wheel({ name: 'intake 2in roller (silicone)', shaft: t.main, center: [...INTAKE.main.at, 0], axis: Z, radius: INTAKE.main.d / 2, width: 0.618, mu: MU.silicone, tread: 20000 }),
    new Wheel({ name: 'intake kicker bar (locked)', shaft: t.kicker, center: [...INTAKE.kicker.at, 0], axis: Z, radius: INTAKE.kicker.d / 2, width: 0.66, mu: MU.polycarb }),
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
// the extension's front (a plane, placed with placeNet)
export function pathSurfaces1678(mesh, t, { mu = 0.35 } = {}) {
  const s = (k, pos) => new Surface({ name: `${k} (CAD)`, vertices: mesh[k].v.map((x) => x / 1000), indices: mesh[k].i, mu, pos });
  const body = s('body', [0, 0, 0]);
  const intake = s('intake', [PIVOT[0], PIVOT[1], 0]).onShaft(t.pivot, [PIVOT[0], PIVOT[1], 0]);
  const hood = s('hood', [DRUM[0], DRUM[1], 0]);
  const slide = s('slide', [0, 0, 0]);
  const lid = s('lid', [0, 0, 0]);
  // the net: 0.45 m back from its front edge (the front edge on the extension's front panel)
  const net = new Surface({ name: 'net', vertices: [0, 0, -0.34, 0, 0, 0.34, -0.45, 0, 0.34, -0.45, 0, -0.34], indices: [0, 1, 2, 0, 2, 3], mu: 0.3, pos: [0, -10, 0] });
  return { body, intake, hood, slide, lid, net };
}
// the net between the lid's front edge (lid raised lift) and the top of the extension's front
// panel (out ext); folded away while the extension's in
export const NET = { lidX: 0.335, lidY: 0.545, frontX: 0.335, frontY: 0.505 };
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
export function placeNet(net, ext, lift) {
  if (ext < 0.05) { net.setPose([0, -10, 0]); return; }
  const fx = NET.frontX + ext, dy = NET.lidY + lift - NET.frontY;
  net.setPose([fx, NET.frontY, 0], 0, -Math.atan2(dy, ext));
}
