// 1690's Kepler as physical mechanisms (js/mech/mechanism.js), from their CAD: the X_T release their
// CAD thread on Chief Delphi links ("1690-26-Top Level Closed.x_t", in their public Drive folder),
// imported into Onshape and exported as STEP (cad/README.md). The X_T carries no mates, so the axes
// below are the parts' own cylinders, read with OpenCascade, and the surfaces FUEL touch are the CAD's
// (tools/path-mesh-1690.py -> js/mech/path1690.js). The speeds are what the team posted in their CAD
// release thread: the hopper floor at 75 RPS and the top and bottom belts at 95 RPS scoring (85 / 100
// passing), the vertical kicker at 60 RPS, the shooter's feeder wheels at 60% of the shooter wheel's
// surface speed, at least 5 m/s. Their code isn't public ("software is our competitive edge").
// Everything marked is an estimate. Robot frame: x forward (the intake end), y up, z to its right; the
// CAD's X is the robot's left, Y forward, Z down.
//
// The ball path:
//   - intake (sub-assembly 200): a carriage that slides out of the frame and down 12 deg on its black
//     side bars (they run on rollers fixed to the side plates, driven by a pinion: an X44 and a
//     MAXPlanetary), 0.318 m from "Closed" (the X_T's pose, inside the frame: travel) to out. On it,
//     two silicone-wrapped polycarbonate rollers (an X60 belted 15t:31t) and the lattice U-frame that
//     is the hopper's front and sides out over the intake. Under the rollers, the "kicker bar": an
//     aluminum tube on two arms turning on a pin fixed to the side plates, stowed folded up behind
//     the intake; when the carriage comes out it falls and surgical tubing holds it down ("tangent to
//     the intake plate"), so FUEL are pinched between it and the rollers and climb the bumper.
//   - the hopper: its side and back panels and a lattice frame over the top, 0.55 m up
//   - the roller floor (300): nine silicone-wrapped rollers belted roller to roller, sloping down 5 cm
//     from front to back, with two more rollers 0.18 m over its back two that push FUEL down and
//     back into
//   - the belts along the back: two 30 mm HTD belts (180T and 160T 5M) on 22t pulleys on fore-aft
//     axles, each its own X44 (16t:48t). The bottom belt's top run (0.159 m up) and the top belt's
//     bottom run (0.302 m) pinch FUEL between them (0.144 m apart: 6 mm of squeeze), against the
//     backing plate behind (polycarbonate under PTFE film), and carry them to the robot's left.
//   - the vertical kicker: a 1in tube standing at the belts' left end in front of them (an X44,
//     16t:45t), "it spins clockwise (toward the belts)".
//   - the "bottom and top kicker" ("the polycarb ramp replaced by flex wheels"): off the belts' end,
//     three stacks of 2.25in compliant wheels on fore-aft shafts step up to the left, and two 2in
//     Stealth wheels flank the turret's bore (left and right), squeezing FUEL up into it (an X60).
//   - the turret (400): on an 8in bearing (its opening 0.176 m across). FUEL come up under the 4in
//     shooter wheel (custom, swerve tread, two X60s geared 28t:42t, no flywheel), which turns them up
//     its back face into the "feeder wheels" on the hood: 2in Stealth wheels with a 2in roller under
//     them, on the hood's lattice arms, which turn about the wheel's axle (an X44 through a
//     VersaPlanetary 4:1 and 19.333:1 gears). The FUEL leave the pinch along the wheel's face at the
//     hood's angle ("the difference between hood angle and ball exit angle was negligible"), between
//     the "hollow hood" that guides them from the sides.
import { IN, FUEL } from '../constants.js';
import { Block, Motor, Shaft, Surface, Train, Wheel, BUMPER_MU } from './mechanism.js';
import PATH from './path1690.js';

const DEG = Math.PI / 180, R = FUEL.radius;
// what each grips FUEL with (estimates): silicone wraps on the intake and floor rollers (grippy), the
// compliant and Stealth wheels (35A), the shooter wheel's swerve tread, the belts' rubber, bare
// polycarbonate and aluminum, the PTFE film on the backing plate (slick: "to reduce friction as much
// as possible"), the printed hood (PLA), the net's cord
export const MU = { silicone: 1.2, compliant: 1.3, tread: 1.3, belt: 1.0, polycarb: 0.5, alu: 0.4, ptfe: 0.1, pla: 0.3, wall: 0.25, net: 0.25 };
// and how far each gives where a FUEL presses it (N/m)
export const TREAD = { silicone: 20000, compliant: 12000, tread: 30000, belt: 3000, net: 10000 };

// ------------------------------------------------------------------ the layout (robot frame, m: the CAD's)
export const HALF_L = 12.5 * IN, HALF_W = 14.5 * IN; // 25 x 29in frame
// the hopper: its side panels' inside faces (the bumper assembly's panels), the lattice frame's top
export const WALL = { left: -0.358, right: 0.358, top: 0.551 };
// the roller floor: [x, y] of each roller's axle, front to back (their 1.3in silicone-wrapped tubes);
// the back three stop short of the left side (the vertical kicker and the feeder are there): z from
// each one's left end to its right; an X60, belted roller to roller (60T 3M belts on ~37t pulleys):
// its reduction isn't clear in the CAD (a gear stage, 21t:31t and 34t:34t stages): 1.5:1, an estimate
// (5 m/s at their 75 RPS)
export const FLOOR = {
  r: 0.0165,
  at: [[0.2797, 0.1899], [0.2324, 0.1814], [0.1852, 0.1729], [0.1417, 0.1650], [0.0982, 0.1572], [0.0546, 0.1493], [0.0071, 0.1425], [-0.0409, 0.1425], [-0.0874, 0.1425]],
  z0: [-0.322, -0.322, -0.322, -0.322, -0.322, -0.322, -0.132, -0.132, -0.132], z1: [0.31, 0.298, 0.298, 0.298, 0.298, 0.298, 0.298, 0.276, 0.276],
  ratio: 1.5,
};
// the two rollers over the floor's back (the floor's motor: 31t:31t from its second shaft, so they turn
// the other way at ~1.48 the floor's speed, an estimate from the CAD's pulleys)
export const TOP = { at: [[-0.0409, 0.3191], [-0.0874, 0.3183]], r: 0.016, z: [-0.066, 0.291], k: -1.48 };
// the belts along the back: across their 30 mm (x), each one's run against the FUEL (its outer
// face's height, z from pulley to pulley), the belt's outer radius round its 22t pulleys; X44s at
// 16t:48t (3:1)
export const BELT = {
  x: [-0.149, -0.119], r: 0.0203, ratio: 3,
  bottom: { y: 0.1586, z: [-0.1158, 0.2742] }, // its top run (the 180T)
  top: { y: 0.3022, z: [-0.0423, 0.2977] },    // its bottom run (the 160T)
};
// the backing plate behind the belts (its PTFE face), the electronics box's face
export const BACK = { x: -0.2105, box: -0.043 };
// the vertical kicker: a 1in tube standing in front of the belts' left end (an X44, 16t:45t)
export const VKICK = { at: [-0.0155, -0.1525], y: [0.088, 0.367], r: 0.0125, ratio: 45 / 16 };
// the bottom kicker (2.25in compliant wheels, two 1in wide on each fore-aft shaft, [z, y]) and the top
// kicker (2in Stealth wheels either side of the turret's bore); an X60 (its reduction isn't readable:
// 2.85:1, an estimate: 6 m/s at the belts' 95 RPS)
export const FEED = {
  x: -0.134, w: 0.05,
  wheels: [[-0.1806, 0.1600], [-0.2358, 0.2031], [-0.2721, 0.2629]], r: 0.0286,
  top: [[-0.2837, 0.3319], [-0.0909, 0.3319]], topR: 0.0254,
  ratio: 2.85,
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
// wheel's gearing (28t:42t), what turns (the custom wheel: 6e-4 kg m^2, an estimate); the feeder
// wheels' drive (an X44, 1:1, an estimate: their motor and gearing aren't clear in the CAD)
export const SHOOTER = {
  axle: [0.070, 0.464], r: 0.0502, w: 0.05,
  feeder: { R: 0.2068, r: 0.0254, w: 0.05, lower: 18.82, ratio: 1 },
  elMin: 35, elMax: HOOD0 - 90, ratio: 1.5, J: 6e-4,
  // the hood's X44: 4:1 VersaPlanetary and 19.333:1 gears (their thread); its profile (estimates)
  hoodRatio: 4 * 19.333, hoodV: 350, hoodA: 3500,
  // the wheel's velocity loop's gain; ready: within 3% of the wheel's target surface speed, 1.5 deg
  // of hood (estimates)
  kP: 0.25, tolV: 0.03, tolHood: 1.5,
};
// the intake: the carriage's travel out (Closed -> out), the rollers on it as exported (in), their X60's
// belting (15t:31t), the U-frame's front and half width out (its plates' outside), the kicker bar: its
// arms' pin, its tube as exported (folded up) and how far it falls (deg)
export const INTAKE = {
  slide: [0.311, -0.068], width: 0.647,
  rollers: [[0.2473, 0.2301, 0.021], [0.1993, 0.3290, 0.021]], ratio: 31 / 15,
  frame: { front: 0.617, sides: 0.362, top: 0.482 },
  kick: { pivot: [0.2723, 0.1703], tube: [0.1499, 0.3283, 0.0127], fall: -164.6, width: 0.675 },
};
export const PRELOAD = [0, 1, 2, 3, 4, 5, 6, 7].map((k) => [0.21 - 0.155 * Math.floor(k / 4), 0.36, -0.24 + 0.155 * (k % 4)]);

// ------------------------------------------------------------------ the drives
// (current limits aren't public: 80 A stator, 40 A supply, estimates; the shooter's 30 A supply, as
// its channels' 30 A breakers: "we used 30A breakers for the channels with the 14awg")
export function build1690() {
  const lim = { statorLimit: 80, supplyLimit: 40 };
  const intake = new Train({ name: 'intake rollers', motor: new Motor('krakenX60', { ratio: INTAKE.ratio, ...lim }) });
  const rollers = INTAKE.rollers.map((_, i) => intake.add(new Shaft({ name: `intake roller ${i ? 'top' : 'bottom'}`, inertia: 1.5e-4 }), -1));
  const tube = new Shaft({ name: 'kicker bar (free)', inertia: 4e-5, drag: 2e-4 });
  const floor = new Train({ name: 'roller floor', motor: new Motor('krakenX60', { ratio: FLOOR.ratio, ...lim }) });
  FLOOR.at.forEach((_, i) => floor.add(new Shaft({ name: `floor roller ${i + 1}`, inertia: 1.2e-4 }), 1));
  const top = TOP.at.map((_, i) => floor.add(new Shaft({ name: `top roller ${i + 1}`, inertia: 1.2e-4 }), TOP.k));
  const belts = ['bottom', 'top'].map((k) => new Shaft({ name: `${k} belt`, motor: new Motor('krakenX44', { ratio: BELT.ratio, ...lim }), inertia: 5e-5, drag: 4e-4 }));
  const vkick = new Shaft({ name: 'vertical kicker', motor: new Motor('krakenX44', { ratio: VKICK.ratio, ...lim }), inertia: 6e-5 });
  const feed = new Train({ name: 'bottom and top kicker', motor: new Motor('krakenX60', { ratio: FEED.ratio, ...lim }) });
  const feedShafts = [...FEED.wheels, ...FEED.top].map((_, i) => feed.add(new Shaft({ name: i < 3 ? `bottom kicker ${i + 1}` : `top kicker ${i - 2}`, inertia: 6e-5 }), 1));
  const shooter = new Train({ name: 'shooter', motor: new Motor('krakenX60', { count: 2, ratio: SHOOTER.ratio, statorLimit: 80, supplyLimit: 30 }) });
  const wheel = shooter.add(new Shaft({ name: '4in shooter wheel', inertia: SHOOTER.J, drag: 2e-4 }), 1);
  const hoodTrain = new Train({ name: 'feeder wheels (hood)', motor: new Motor('krakenX44', { ratio: SHOOTER.feeder.ratio, statorLimit: 60, supplyLimit: 30 }) });
  const hoodWheels = ['upper', 'lower'].map((k) => hoodTrain.add(new Shaft({ name: `feeder wheel ${k}`, inertia: 4e-5, drag: 1e-4 }), 1));
  return { intake, rollers, tube, floor, top, belts, vkick, feed, feedShafts, shooter, wheel, hoodTrain, hoodWheels };
}

const Z = [0, 0, 1];
// the wheels of the ball path (robot frame; the intake's posed by placeIntake)
export function wheels1690(t) {
  const K = INTAKE.kick;
  const intake = [
    ...INTAKE.rollers.map(([x, y, r], i) => new Wheel({ name: `intake roller ${i ? 'top' : 'bottom'} (silicone)`, shaft: t.rollers[i], center: [x, y, 0], axis: Z, radius: r, width: INTAKE.width, mu: MU.silicone, tread: TREAD.silicone })),
    new Wheel({ name: 'kicker bar (aluminum, free)', shaft: t.tube, center: [K.tube[0], K.tube[1], 0], axis: Z, radius: K.tube[2], width: K.width, mu: MU.alu }),
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
  return t.hoodWheels.map((sh, i) => new Wheel({ name: `feeder wheel ${i ? 'lower (2in roller)' : 'upper (2in Stealth)'}`, shaft: sh, center: [0, 0, 0], axis: Z, radius: F.r, width: F.w, mu: MU.compliant, tread: TREAD.compliant }));
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
  // the belts: their runs slide along -z as their shafts turn (the pulleys' pitch radius, 17.5 mm, a radian)
  const belt = (k, m) => { const sf = s(`${k} belt`, m, { mu: MU.belt, tread: TREAD.belt }).onSlide(t.belts[k === 'bottom' ? 0 : 1], [0, 0, -1]); sf.slideK = 0.0175; return sf; };
  const bottom = belt('bottom', beltRun(BELT.bottom.y, ...BELT.bottom.z, true)), top = belt('top', beltRun(BELT.top.y, ...BELT.top.z, false));
  const carriage = cad('carriage');
  const kick = cad('kick', { pos: [...INTAKE.kick.pivot, 0] });
  const turret = cad('turret', { mu: MU.pla, pos: [TURRET.at[0], 0, TURRET.at[1]] });
  const hood = cad('hood', { mu: MU.pla });
  return { body, bodyFine, ptfe, bottom, top, carriage, kick, turret, hood, bumper: bumper1690(), net: new Net1690() };
}
// how far the kicker bar has fallen (deg) with the carriage ext of the way out (it falls once the
// carriage has cleared it, a third of the way: an estimate)
export const kickAt = (ext) => INTAKE.kick.fall * Math.min(1, Math.max(0, (ext - 0.2) / 0.25));
// the intake ext (0 in, 1 out) of the way out: the carriage, its rollers, the kicker bar
export function placeIntake(sf, wheels, ext) {
  const [dx, dy] = INTAKE.slide, K = INTAKE.kick, a = kickAt(ext) * DEG, c = Math.cos(a), sn = Math.sin(a);
  sf.carriage.setPose([dx * ext, dy * ext, 0]);
  sf.kick.setPose([...K.pivot, 0], 0, a);
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

// The net over the top (the team: "the balls tend to bounce up and out ... we'll do some tests with
// netting, but it could cause some issues with trench crossing"; their CAD has none): slack cord tied
// along the lattice frame over the top (0.551 m) and out over the U-frame's top edges (it slides with
// the carriage). The load lifts it freely as far as NET.bulge over its ties in its middle (the shape a
// dome of cord takes, as 1706's), and there it goes taut (TREAD.net). NET.bulge, 0.01 m: the lattice's
// bars are under it, so it can't dome much (an estimate). In the robot's frame: x0, the back tie (the
// electronics box's face); x1, the front tie (the U-frame's front, out); hw, half its width
export const NET = { x0: BACK.box, x1: INTAKE.frame.front, y: WALL.top, hw: 0.355, bulge: 0.01, hang: { sag: 0.02, sagOut: 0.06 } };
export function netAt(x, z, x1 = NET.x1) {
  const L = x1 - NET.x0, W = 2 * NET.hw, B = NET.bulge;
  const u = Math.min(1, Math.max(0, (x - NET.x0) / L)), v = Math.min(1, Math.max(0, (z + NET.hw) / W));
  const su = Math.sin(Math.PI * u), sv = Math.sin(Math.PI * v);
  // (out over the U-frame, whose top is 0.07 m lower, it slopes down to it)
  const y0 = x > HALF_L ? NET.y - (NET.y - INTAKE.frame.top) * Math.min(1, (x - HALF_L) / 0.06) : NET.y;
  return [y0 + B * su * sv, (B * Math.PI * Math.cos(Math.PI * u) * sv) / L, (B * su * Math.PI * Math.cos(Math.PI * v)) / W];
}
export const netY = (x, z, x1) => netAt(x, z, x1)[0];
class Net1690 extends Surface {
  constructor() {
    super({ name: 'net', vertices: [0, 0, 0, 0.01, 0, 0, 0, 0, 0.01], indices: [0, 1, 2], mu: MU.net, tread: TREAD.net });
    this.x1 = NET.x1;
  }
  // (the U-frame's front, where the carriage puts it)
  place(x1) { this.x1 = x1; }
  at(x, z) { return netAt(x, z, this.x1); }
  touches(p) {
    const [x, y, z] = p;
    if (x <= NET.x0 || x >= this.x1 || Math.abs(z) >= NET.hw) return [];
    const [h, hx, hz] = this.at(x, z), l = Math.hypot(hx, 1, hz), dist = (h - y) / l;
    if (dist >= R || dist < -0.03) return [];
    return [{ depth: R - dist, n: [hx / l, -1 / l, hz / l] }];
  }
}
export { Net1690 };
