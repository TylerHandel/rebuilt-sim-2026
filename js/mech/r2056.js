// 2056 OP Robotics' "Desperate Measures" as physical mechanisms (js/mech/mechanism.js), from their 2026
// technical binder and CAD drawing package (2056.ca: OPR26-B-2056 Technical Binder, OPR26-B-2056
// Drawing Package; the Chief Delphi release thread, t/525183). They didn't release 3D CAD ("we do not
// have any plans to share the 3D CAD models or geometry"), so the ball path's surfaces are built in
// code (as 4414's), from the drawings' dimensions where the drawings give them. The binder thanks 1690
// Orbit for "the primary inspiration": "nearly all aspects of our robot's redesign were directly
// influenced by ideas we saw in Orbit's machine" (the same ball path: a roller floor back into two belts
// along the back, a vertical kicker, two 2in Stealth wheels up into a turret on an 8in bearing, a 4in
// wheel throwing FUEL against a wheel on the hood's arm, which turns about the shooter wheel's axle),
// so where the drawings settle nothing the layout follows 1690's (js/mech/r1690.js: their CAD), mirrored:
// 2056's turret is at the back RIGHT. Everything marked (est.) is an estimate.
//
// From the drawings (inches; OPR26-B-A00 sheet 2, a top view at 1:2 with its own dimensions; the
// A19 shooter's side view, scaled on the turret's 100T 10 DP gear, 10.2in across; part drawings):
//   - frame 25.5 long x 29.5 wide (the binder; the chassis drawing's rails 25.0 x 29.0 plus the bumper
//     brackets' 0.25), the swerve modules' axes +-9.875 fore and aft, +-11.875 across (A00); the
//     turret's axis 5.25 back and 7.25 right of center ("-X TURRET OFFSET", "-Y TURRET OFFSET", A00).
//   - the roller floor (A41 Hopper & Intake, its BOM and roller drawings): 1.5in polycarbonate tubes
//     under WCP silicone, 1-5/8in across; six 26.5in long and three 16.5in (the back three stop short of
//     the right side, the turret's), all on 24T 5mm pulleys and 43T belts (215 mm: 47.5 mm apart,
//     1:1); two more 18.6in rollers (A43): over the floor's back here, as 1690's two (est.: where they
//     sit isn't legible in the drawings). Its X60: 14:36 20 DP then 18:25 5mm HTD (the binder): 3.57:1.
//   - the "hopper guide" (P173, 11.9in): a diagonal wall from the third long roller's right end to the
//     short rollers' end (A41's top view), keeping the load off the turret's corner.
//   - the singulator (binder): "dual rubber coated timing belt drive ... rides in a 3D printed carry
//     way"; 2x X44, 12:36 5mm HTD, 24T pulleys (pitch radius 19.1 mm). The top conveyor's side plate 14in
//     (P163). Heights and ends (est.) as 1690's belts, mirrored.
//   - the side kicker (A11, "vertical kicker roller ... to sort fuel collisions at quarter pipe
//     infeed"): a 1.625in polyurethane sleeve 1.75in long on a 1/2in tube axle, its 24T pulley on an
//     X44's 18T (est.: A10's BOM has the 18T pulley).
//   - the quarter pipe (P157 "Shooter Feed Plate": polycarbonate bent UP 70 deg at R 6.75in) and its
//     "dual 2in Stealth wheel shooter feed" on an X60, 16:28 20 DP: the two Stealth wheels (AM-3155,
//     50A) on two shafts (P181 short, P182 long), "CENTRE WHEELS" either side of the turret's bore (A10).
//   - the turret (binder; A19): 8in ID X-contact bearing; 4in shooter wheel (OPR26-A11: a McMaster
//     2477K37 60A urethane drive roller, 4in x 1.92in, 0.97 lb, pressed on a 1.25in tube) on two X60s,
//     28:34 3mm GT2; the "backspin kicker", dual 2in Stealth wheels on an X44 32:32, on the hood's arm
//     8.16in from the wheel's axle (A19: the arm turns about it, 33.6 deg between its two drawn poses,
//     the binder's 34); the release angle 39.5 to 73.5 deg, the pinch's tangent: elevation = the arm's
//     angle - 90 (as 1690's); the axle 3.5in from the turret's axis toward the shot, 2.83in over the
//     turret gear's top. Hood: X44, 8:40 then 10:155 on a sector gear (77.5:1); turret: X44, 8:40 then
//     10:100 (50:1), +-185 deg on a printed energy chain.
//   - the intake (binder; A51; P512 rack, 22.6in with 16.8in of 8 DP teeth): two 2-1/8in rollers (2in
//     polycarbonate under polyurethane/silicone, 23.5in tubes) on an X60 12:30 5mm HTD; the side arms
//     are 1/4in polycarbonate racks driven by an X44 8:40 20 DP to 12T 8 DP pinions (1.5in pitch
//     circle); "a low stator current limit allowing the intake to retract like a spring"; the kicker
//     bar, a 0.75in Lexan tube (P557) on 7.3in arms (P555), "sprung loaded down with some surgical
//     tubing ... It retracts passively when the intake retracts against the bumper". The travel (0.33 m,
//     11.5 deg down: the rack's slope in A41's side view) and the rollers' and bar's places (est.) after
//     1690's, which their intake follows.
// Robot frame: x forward (the intake end), y up, z to its right.
import { IN, FUEL } from '../constants.js';
import { Block, Motor, Shaft, Surface, Train, Wheel, BUMPER_MU } from './mechanism.js';

const DEG = Math.PI / 180, R = FUEL.radius;
// what each grips FUEL with (estimates): silicone on the intake's and floor's rollers, the Stealth
// wheels (50A), the shooter wheel's 60A urethane, the belts' rubber coating, the side kicker's
// polyurethane, bare polycarbonate and aluminum, the printed carry way, the net's cord
export const MU = { silicone: 1.2, stealth: 1.3, urethane: 1.2, belt: 1.0, polycarb: 0.5, alu: 0.4, print: 0.3, wall: 0.25, net: 0.25 };
// how far each gives where a FUEL presses it (N/m; estimates, as 1690's: the belts on their printed
// carry way give no more than a wrap on a tube)
export const TREAD = { silicone: 20000, stealth: 12000, urethane: 30000, belt: 20000, net: 100000 };

// ------------------------------------------------------------------ the layout (robot frame, m)
export const HALF_L = 12.75 * IN, HALF_W = 14.75 * IN; // 25.5 x 29.5in (the binder)
// the hopper: its sides' inside faces (the intake's 27.25in hopper front, P581, and 26.5in rollers:
// 0.346 m each side), the side panels' top (est.: the hopper side lattice, P584/P586, ~10.5in tall
// over the floor plates)
export const WALL = { left: -0.346, right: 0.346, top: 0.545 };
// the roller floor: [x, y] of each roller's axle, front to back (est.: 1690's, which has the same
// 47.5 mm pitch, its tops kept where 1690's are), z from each one's left end to its right (long:
// 25.5in of sleeve, short: 15.5in); every roller 1:1 with the drive roller (24T pulleys round)
export const FLOOR = {
  r: (1.625 / 2) * IN,
  at: [[0.2810, 0.1776], [0.2335, 0.1708], [0.1860, 0.1640], [0.1422, 0.1577], [0.0984, 0.1515], [0.0546, 0.1452], [0.0071, 0.1384], [-0.0409, 0.1384], [-0.0874, 0.1384]],
  z0: Array(9).fill(-0.324), z1: [0.324, 0.324, 0.324, 0.324, 0.324, 0.324, 0.070, 0.070, 0.070],
  ratio: (36 / 14) * (25 / 18),
  k: Array(9).fill(1),
};
// the two 18.6in rollers (A43) over the floor's back (est.: 1690's place, mirrored, turning the other
// way, 1:1 off the floor's drive)
export const TOP = { at: [[-0.0409, 0.3191], [-0.0874, 0.3183]], r: (1.625 / 2) * IN, z: [-0.324, 0.06], k: [-1, -1] };
// the foam's rolling resistance on the floor's rollers (as 1690's: 0.08 of what presses it there;
// with every roller at one speed a FUEL resting between two would otherwise spin in place)
export const ROLL = { rr: 0.08 };
// the belts along the back (est. where: 1690's, mirrored; their runs carry FUEL to the right): across
// their width (x), each one's run against the FUEL (its outer face's height, z from pulley to
// pulley), the belt's outer radius round its 24T pulleys and their pitch radius; X44s through 12:36
export const BELT = {
  x: [-0.149, -0.119], r: 0.0203, pitch: (24 * 0.005) / (2 * Math.PI), ratio: 36 / 12,
  bottom: { y: 0.1586, z: [-0.2742, 0.1158] }, // its top run
  top: { y: 0.3022, z: [-0.2977, 0.0423] },    // its bottom run
};
// the backing behind the belts (est.: polycarbonate), the channel's front wall over the quarter pipe
export const BACK = { x: -0.2105, front: -0.056 };
// the turret: its axis (A00), the bore up through its bearing (the 8in ID: the turret's opening, est.
// 0.088 m, as 1690's), the turret gear's top (the deck; est.: the shooter uprights, 12.63in, on the
// frame); travel +-185 deg (the binder)
export const TURRET = { at: [-5.25 * IN, 7.25 * IN], bore: { r: 0.088, y: [0.385, 0.40] }, deck: 0.37, range: 185 };
// the side kicker: a 1.625in polyurethane sleeve 1.75in long, standing in front of the belts' channel
// at the quarter pipe's infeed (est. where), its X44's 18T on the axle's 24T (est.)
export const VKICK = { at: [-0.0415, 0.10], y: [0.23 - 0.875 * IN, 0.23 + 0.875 * IN], r: (1.625 / 2) * IN, ratio: 24 / 18 };
// the quarter pipe: the shooter feed plate's 6.75in bend (UP 70 deg, its flats and the frame's
// guides round it: est. 90 deg in all), its center so a FUEL off the bottom belt's run rides round it
// and comes up under the turret's axis (c: [z, y]); across the channel (x)
export const PIPE = { R: 6.75 * IN, c: [TURRET.at[1] - (6.75 * IN - R), 0.327], x: [-0.214, BACK.front], top: TURRET.bore.y[0] };
// the dual Stealth wheels (2in, 50A, est. 1in wide each), [z, y]: one at the pipe's center, driving the
// FUEL round it (its bottom where the top belt's run is), the other across the bore over the pipe's
// end; one X60, 16:28, both shafts at its speed (the second turned back 1:1)
export const FEED = {
  x: -0.134, w: 0.025, r: 1 * IN,
  wheels: [[PIPE.c[0], PIPE.c[1]], [TURRET.at[1] + (R + 1 * IN - 0.0044), 0.355]],
  ratio: 28 / 16,
};
// the shooter, in the turret's frame (s along the shot, y up, z across; A19): the 4in wheel's axle
// (1.92in wide), the backspin kicker (2in Stealth wheels on the hood's arm, R from the axle), the
// release angles (the hood's travel), the wheel's gearing (28:34), the kicker's (32:32); what turns
// (the urethane roller 0.441 kg, 4in OD on a 1.25in bore: 6.25e-4 kg m^2; its 1.25in tube and the
// pulley plugs (P101, P114): 2e-5; est. 3e-5 more for the 34T pulleys: 6.75e-4. The kicker: two
// Stealth wheels, its 3/8 hex and a 32T pulley: 4e-5, est.). The hood: X44 8:40, 10T on a 155T
// sector (77.5:1); its profile (est.)
export const SHOOTER = {
  axle: [3.5 * IN, TURRET.deck + 2.83 * IN], r: 2 * IN, w: 1.92 * IN,
  kicker: { R: 8.16 * IN, r: 1 * IN, w: 0.05, ratio: 1 },
  elMin: 39.5, elMax: 73.5, ratio: 34 / 28, J: 6.75e-4, kickJ: 4e-5,
  hoodRatio: 5 * 15.5, hoodV: 350, hoodA: 3500,
  // the wheel's velocity loop's gain; ready: within 3% of the wheel's target surface speed, 1.5 deg
  // of hood (est.)
  kP: 0.25, tolV: 0.03, tolHood: 1.5,
};
// the intake: the carriage's travel out (est. 0.33 m, 11.5 deg down: A41's side view), the rollers on
// it as it is retracted ([x, y, r]), their X60's 12:30, the intake's front (the bash bar and the hopper
// front panel, out: its lower edge, with the bash bar, over the top roller so no FUEL is pinched
// between them, est.), its sides (the intake's hopper sides, P583), the front panel's top; the kicker bar
// on its arms (pivot on the carriage, retracted; the 0.75in tube; its arms' fall from folded, deg)
export const INTAKE = {
  slide: [0.33 * Math.cos(11.5 * DEG), -0.33 * Math.sin(11.5 * DEG)], width: 23.5 * IN,
  // (the rollers where 1690's are against the bumper with the intake out, the bottom one 6 mm higher for
  // its larger radius, the top one 8 mm further back, reaching FUEL that stop on the bumper's front
  // edge: est.)
  rollers: [[0.5657 - 0.33 * Math.cos(11.5 * DEG), 0.167 + 0.33 * Math.sin(11.5 * DEG), (2.125 / 2) * IN], [0.5097 - 0.33 * Math.cos(11.5 * DEG), 0.2599 + 0.33 * Math.sin(11.5 * DEG), (2.125 / 2) * IN]], ratio: 30 / 12,
  frame: { front: 0.635, sides: 0.352, top: 0.482, panel: 0.33, lip: [0.55, 0.305] },
  // (the bar's arms run forward and down from their pin on the intake's side plates, just behind the
  // bumper's top when it's out, to the tube just in front of the bumper's face, as 1690's (est.: the
  // drawings don't dimension it); FUEL pushing up under the tube swing it up and back over the bumper,
  // lifting them; coming in, the bumper folds it up and back: a0 55 deg folded, -35.2 out)
  kick: { pivot: [0.2787 - 0.33 * Math.cos(11.5 * DEG), 0.1703 + 0.33 * Math.sin(11.5 * DEG)], len: 0.186, a0: 55, fall: -90.2, r: 0.375 * IN, width: 27.81 * IN, mass: 0.25, J: 0.008, tubing: { preload: 1.0, rate: 1.0 } },
};
// the racks' drive: an X44 through 8:40 to the 12T 8 DP pinions (0.75in pitch radius); what slides
// (the intake, its rollers and X60, the bash bar and hopper front: ~4.5 kg, est.)
export const SLIDE = { pinion: 0.75 * IN, ratio: 40 / 8, mass: 4.5, travel: Math.hypot(...INTAKE.slide), dir: [INTAKE.slide[0] / Math.hypot(...INTAKE.slide), INTAKE.slide[1] / Math.hypot(...INTAKE.slide), 0] };
// the preload (8): three in the belts' channel, three on the floor, two on top of them
export const PRELOAD = [
  ...[-0.22, -0.07, 0.02].map((z) => [-0.135, 0.23, z]),
  ...[-0.24, -0.09, 0.06].map((z) => [0.13, 0.25, z]),
  ...[-0.17, -0.02].map((z) => [0.12, 0.40, z]),
];
// the hopper guide: in plan, from the third long roller's right end to the short rollers' end
export const GUIDE = { from: [0.186, WALL.right], to: [0.007, FLOOR.z1[6]] };

// ------------------------------------------------------------------ the drives
// (their mechanisms' current limits aren't published: "current limits were required on every
// mechanism this year to manage current draw"; 80 A stator, 40 A supply, est.; the intake's racks low,
// 20 A stator, "the intake just collapses in an impact")
export function build2056() {
  const lim = { statorLimit: 80, supplyLimit: 40 };
  const intake = new Train({ name: 'intake rollers', motor: new Motor('krakenX60', { ratio: INTAKE.ratio, ...lim }) });
  const rollers = INTAKE.rollers.map((_, i) => intake.add(new Shaft({ name: `intake roller ${i ? 'top' : 'bottom'}`, inertia: 2e-4 }), -1));
  const kickArm = new Shaft({ name: 'kicker bar arms (pin)', inertia: INTAKE.kick.J, drag: 0.02 });
  const slide = new Shaft({ name: 'intake racks (pinions)', motor: new Motor('krakenX44', { ratio: SLIDE.ratio, statorLimit: 20, supplyLimit: 20 }), inertia: SLIDE.mass * SLIDE.pinion ** 2, drag: 0.005, friction: 0.05 });
  const floor = new Train({ name: 'roller floor', motor: new Motor('krakenX60', { ratio: FLOOR.ratio, ...lim }) });
  FLOOR.at.forEach((_, i) => floor.add(new Shaft({ name: `floor roller ${i + 1}`, inertia: 1.5e-4 }), FLOOR.k[i]));
  const top = TOP.at.map((_, i) => floor.add(new Shaft({ name: `top roller ${i + 1}`, inertia: 1.5e-4 }), TOP.k[i]));
  const belts = ['bottom', 'top'].map((k) => new Shaft({ name: `${k} belt`, motor: new Motor('krakenX44', { ratio: BELT.ratio, ...lim }), inertia: 5e-5, drag: 4e-4 }));
  const vkick = new Shaft({ name: 'side kicker', motor: new Motor('krakenX44', { ratio: VKICK.ratio, ...lim }), inertia: 3e-5 });
  const feed = new Train({ name: 'shooter feed (Stealth wheels)', motor: new Motor('krakenX60', { ratio: FEED.ratio, ...lim }) });
  const feedShafts = FEED.wheels.map((_, i) => feed.add(new Shaft({ name: `shooter feed ${i ? 'outer' : 'inner'}`, inertia: 4e-5 }), 1));
  const shooter = new Train({ name: 'shooter', motor: new Motor('krakenX60', { count: 2, ratio: SHOOTER.ratio, statorLimit: 80, supplyLimit: 40 }) });
  const wheel = shooter.add(new Shaft({ name: '4in shooter wheel', inertia: SHOOTER.J, drag: 2e-4 }), 1);
  // (the backspin kicker on its own X44, 1:1)
  const kicker = new Shaft({ name: 'backspin kicker', motor: new Motor('krakenX44', { ratio: SHOOTER.kicker.ratio, statorLimit: 60, supplyLimit: 30 }), inertia: SHOOTER.kickJ, drag: 1e-4 });
  return { intake, rollers, kickArm, slide, floor, top, belts, vkick, feed, feedShafts, shooter, wheel, kicker };
}

const Z = [0, 0, 1];
// the wheels of the ball path (robot frame; the intake's posed by placeIntake)
export function wheels2056(t) {
  const intake = INTAKE.rollers.map(([x, y, r], i) => new Wheel({ name: `intake roller ${i ? 'top' : 'bottom'} (silicone)`, shaft: t.rollers[i], center: [x, y, 0], axis: Z, radius: r, width: INTAKE.width, mu: MU.silicone, tread: TREAD.silicone }));
  intake.forEach((w) => { w.home = w.center.slice(); });
  // (the floor's top runs back: +z axes; the rollers over it turn the other way)
  const floor = FLOOR.at.map(([x, y], i) => new Wheel({ name: `floor roller ${i + 1} (silicone)`, shaft: t.floor.shafts[i], center: [x, y, (FLOOR.z0[i] + FLOOR.z1[i]) / 2], axis: Z, radius: FLOOR.r, width: FLOOR.z1[i] - FLOOR.z0[i], mu: MU.silicone, tread: TREAD.silicone }));
  const top = TOP.at.map(([x, y], i) => new Wheel({ name: `top roller ${i + 1} (silicone)`, shaft: t.top[i], center: [x, y, (TOP.z[0] + TOP.z[1]) / 2], axis: Z, radius: TOP.r, width: TOP.z[1] - TOP.z[0], mu: MU.silicone, tread: TREAD.silicone }));
  // (the side kicker: its face toward the channel runs right, with the FUEL: a +y axis)
  const vkick = new Wheel({ name: 'side kicker (polyurethane)', shaft: t.vkick, center: [VKICK.at[0], (VKICK.y[0] + VKICK.y[1]) / 2, VKICK.at[1]], axis: [0, 1, 0], radius: VKICK.r, width: VKICK.y[1] - VKICK.y[0], mu: MU.urethane, tread: 30000 });
  // (the feed wheels' faces toward the FUEL run up and round: the inner one's axis -x, the outer's +x)
  const feed = FEED.wheels.map(([z, y], i) => new Wheel({ name: `shooter feed ${i ? 'outer' : 'inner'} (2in Stealth)`, shaft: t.feedShafts[i], center: [FEED.x, y, z], axis: [i ? 1 : -1, 0, 0], radius: FEED.r, width: FEED.w, mu: MU.stealth, tread: TREAD.stealth }));
  return { intake, floor: [...floor, ...top], vkick, feed };
}
// the shooter wheel and the backspin kicker (posed by placeTurret)
export function shooterWheel(t) {
  return new Wheel({ name: '4in shooter wheel (60A urethane)', shaft: t.wheel, center: [0, 0, 0], axis: Z, radius: SHOOTER.r, width: SHOOTER.w, mu: MU.urethane, tread: TREAD.urethane });
}
export function kickerWheel(t) {
  const K = SHOOTER.kicker;
  return new Wheel({ name: 'backspin kicker (2in Stealth)', shaft: t.kicker, center: [0, 0, 0], axis: Z, radius: K.r, width: K.w, mu: MU.stealth, tread: TREAD.stealth });
}
// a turret-frame point (s along the shot, y up, z across) where the turret is at yaw (rad, three.js
// sense: + turns it to the left)
export function turretPoint([s, y, z], yaw) {
  const c = Math.cos(yaw), sn = Math.sin(yaw), [tx, tz] = TURRET.at;
  return [tx + s * c + z * sn, y, tz - s * sn + z * c];
}
// the shot's elevation (deg) by the hood's geometry for its arm's angle (deg about the axle, from the
// shot's direction to the kicker: the pinch's tangent), and back
export const hoodElevation = (deg) => deg - 90;
export const hoodFor = (el) => el + 90;

// the front bumper: solid foam (3.25in with its backing), across the front and round to the sides
export function bumper2056() {
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
const N = (d) => Math.max(1, Math.ceil(Math.abs(d) / 0.06));
// flat panels: level (y), across x at z, across z at x, and a vertical one along a line in plan
const level = (x0, x1, z0, z1, y) => grid(N(x1 - x0), N(z1 - z0), (u, v) => [lerp(x0, x1, u), y, lerp(z0, z1, v)]);
const wallZ = (z, x0, x1, y0, y1) => grid(N(x1 - x0), N(y1 - y0), (u, v) => [lerp(x0, x1, u), lerp(y0, y1, v), z]);
const wallX = (x, z0, z1, y0, y1) => grid(N(z1 - z0), N(y1 - y0), (u, v) => [x, lerp(y0, y1, v), lerp(z0, z1, u)]);
const wallLine = ([x0, z0], [x1, z1], y0, y1) => grid(N(Math.hypot(x1 - x0, z1 - z0)), N(y1 - y0), (u, v) => [lerp(x0, x1, u), lerp(y0, y1, v), lerp(z0, z1, u)]);
// the belts' runs (strips across their width, the FUEL's side; they slide along +z, to the right),
// their ends rounded over the pulleys
function beltRun(y, z0, z1, up) {
  const [x0, x1] = BELT.x, r = BELT.r, s = up ? 1 : -1;
  const end = (zc, k) => grid(2, 4, (u, v) => { const a = (Math.PI / 2) * v; return [lerp(x0, x1, u), y - s * r + s * r * Math.cos(a), zc + k * r * Math.sin(a)]; });
  return merge(level(x0, x1, z0, z1, y), end(z0, -1), end(z1, 1));
}
// the quarter pipe: the plate round its center from under the bottom belt's end (where its arc meets
// the run) to straight up, then straight on up to the bore
function quarterPipe() {
  const [zc, yc] = PIPE.c, Rq = PIPE.R, [x0, x1] = PIPE.x;
  const a0 = Math.asin(Math.min(1, (BELT.bottom.z[1] - 0.01 - zc) / Rq));
  const arc = grid(4, 14, (u, v) => { const a = lerp(a0, Math.PI / 2, v); return [lerp(x0, x1, u), yc - Rq * Math.cos(a), zc + Rq * Math.sin(a)]; });
  return merge(arc, wallZ(zc + Rq, x0, x1, yc, PIPE.top + 0.02));
}
// the guard round the turret's sweep over its deck (A19's 15in envelope; A10's "shooter guard" and
// "top visor ring"), on the hopper's side, up to the hopper's top: the load stays out of the turret
export const GUARD = { r: 7.5 * IN + 0.01 };
function guard() {
  // (round the front from the backing on the left to behind it on the right)
  const [tx, tz] = TURRET.at;
  return grid(24, 3, (u, v) => { const a = lerp(-Math.PI / 2 - 0.6, Math.PI / 2 + 0.9, u); return [tx + GUARD.r * Math.cos(a), lerp(TURRET.deck, WALL.top, v), tz + GUARD.r * Math.sin(a)]; });
}
// the bore up through the turret's bearing (a ring), fixed
function bore() {
  const [tx, tz] = TURRET.at, B = TURRET.bore;
  return grid(28, 2, (u, v) => [tx + B.r * Math.cos(2 * Math.PI * u), lerp(B.y[0], B.y[1], v), tz + B.r * Math.sin(2 * Math.PI * u)]);
}
// what doesn't move: the hopper's sides (to the frame's front) and the backing behind the belts; the
// plates under the floor's back and the belts; the hopper guide and the walls closing the turret's
// corner off (the short rollers' end, the channel's front over the quarter pipe); the quarter pipe and
// its side guides (the backing, the front wall); the bore
function body() {
  const W = WALL, yb = 0.10, x1 = HALF_L;
  const [fx, fz] = GUIDE.from, [gx, gz] = GUIDE.to;
  return merge(
    wallZ(W.left, BACK.x, x1, yb, W.top), wallZ(W.right, gx, x1, yb, W.top),
    wallX(BACK.x, W.left, W.right, yb, PIPE.top + 0.02),
    // (under the floor's back roller to the bottom belt, and behind the bottom belt to the backing)
    level(BELT.x[1], FLOOR.at[8][0] - FLOOR.r + 0.004, W.left, FLOOR.z1[8], 0.150),
    level(BACK.x, BELT.x[0], W.left, PIPE.c[0], 0.152),
    // (the hopper guide, the short rollers' end, the channel's front wall over the quarter pipe: up to
    // the turret's deck; over it the guard round the turret's sweep)
    wallLine([fx, fz], [gx, gz], 0.12, W.top), wallZ(gz, BACK.front, gx, 0.12, TURRET.deck), wallX(BACK.front, gz, W.right, 0.12, TURRET.deck),
    guard(),
    // (over the top conveyor's beam, the hopper's back slopes down to the floor, so no FUEL rest on
    // the beam: est.)
    grid(3, N(W.left - GUIDE.to[1]), (u, v) => [lerp(BACK.x, BELT.x[1] + 0.004, u), lerp(0.43, BELT.top.y + 2 * BELT.r + 0.008, u), lerp(W.left, GUIDE.to[1], v)]),
    quarterPipe(),
  );
}
// the turret's guides (its frame: s along the shot, y up, z across): its side plates either side of
// the FUEL up from the bore to past the pinch, and a back plate behind the bore up to under the
// kicker's lowest
function turret() {
  const zw = R + 0.01, S = SHOOTER, G = GUIDE_ARC, [ax, ay] = S.axle;
  return merge(
    ...[-zw, zw].map((z) => grid(6, 6, (u, v) => [lerp(-0.13, 0.20, u), lerp(TURRET.deck, 0.70, v), z])),
    grid(4, 3, (u, v) => [-0.105, lerp(TURRET.deck, 0.40, v), lerp(-zw, zw, u)]),
    // (the fixed shooter skis, A19 item 19: round the wheel's back below the hood's arm)
    grid(10, 2, (u, v) => { const d = lerp(G.fixed[0], G.fixed[1], u), a = d * DEG, rr = G.r + G.flare * Math.max(0, (d - G.fixed[2]) / (G.fixed[1] - G.fixed[2])); return [ax + rr * Math.cos(a), ay + rr * Math.sin(a), lerp(-G.w, G.w, v)]; }),
  );
}
// the hood's guide (on its arm, in the arm's frame: about the axle, 0 deg toward the kicker): from
// just past the kicker round the wheel's back to the skis
function hoodGuide() {
  const G = GUIDE_ARC;
  return grid(12, 2, (u, v) => { const a = lerp(G.arm[0], G.arm[1], u) * DEG; return [G.r * Math.cos(a), G.r * Math.sin(a), lerp(-G.w, G.w, v)]; });
}
// FUEL held to the wheel's back between the bore and the kicker (est.: the drawings show the hood's
// side plates and the fixed "shooter skis", not their profiles): concentric guides at G.r from the
// wheel's axle (a FUEL on the wheel squeezed ~6 mm), on the arm from 9 to 70 deg past the kicker, fixed
// from 190 to 222 deg from the shot's direction (a FUEL up the bore meets the wheel at 225), flaring
// out 2 cm from 207 deg to its lower end to lead it in; +-G.w across
export const GUIDE_ARC = { r: SHOOTER.r + 2 * R - 0.006, arm: [9, 70], fixed: [190, 222, 207], flare: 0.02, w: 0.04 };
// the intake (its frame as retracted; placeIntake slides it): the front panel with the bash bar (the
// front, 0.22 m up to its top), the hopper sides (P583) out over the bumper
function carriage() {
  const F = INTAKE.frame, [dx, dy] = INTAKE.slide, f = F.front - dx, top = F.top - dy;
  return merge(
    wallX(f, -F.sides, F.sides, F.panel - dy, top),
    // (its lower part bent back over the top roller to just above it: est., P581's bends)
    grid(2, N(2 * F.sides), (u, v) => [lerp(f, F.lip[0] - dx, u), lerp(F.panel, F.lip[1], u) - dy, lerp(-F.sides, F.sides, v)]),
    ...[-F.sides, F.sides].map((z) => wallZ(z, f - 0.33, f, 0.20 - dy, top)),
  );
}

export function pathSurfaces2056(t) {
  const s = (name, m, o = {}) => new Surface({ name, vertices: m.v, indices: m.i, mu: MU.wall, ...o });
  const bodyS = s('body: hopper sides, guide, quarter pipe (polycarbonate)', body());
  const boreS = s('turret bore (aluminum)', bore(), { mu: MU.alu });
  const belt = (k, m) => { const sf = s(`${k} belt`, m, { mu: MU.belt, tread: TREAD.belt }).onSlide(t.belts[k === 'bottom' ? 0 : 1], [0, 0, 1]); sf.slideK = BELT.pitch; return sf; };
  const bottom = belt('bottom', beltRun(BELT.bottom.y, ...BELT.bottom.z, true)), top = belt('top', beltRun(BELT.top.y, ...BELT.top.z, false));
  const car = s('intake (front panel, hopper sides)', carriage(), { mu: MU.polycarb }).onSlide(t.slide, SLIDE.dir);
  car.slideK = SLIDE.pinion;
  // (the kicker bar's tube on its arms, which swing on their pin on the intake: posed by placeIntake)
  const K = INTAKE.kick;
  const kickTube = s('kicker bar tube (0.75in Lexan)', grid(16, 12, (u, v) => [K.len + K.r * Math.cos(2 * Math.PI * u), K.r * Math.sin(2 * Math.PI * u), (v - 0.5) * K.width]), { mu: MU.polycarb, pos: [...K.pivot, 0] });
  kickTube.onShaft(t.kickArm, [...K.pivot, 0]);
  const turretS = s('turret side plates, skis', turret(), { mu: MU.alu, pos: [TURRET.at[0], 0, TURRET.at[1]] });
  const hood = s('hood guide', hoodGuide(), { mu: MU.print });
  return { body: bodyS, bore: boreS, bottom, top, carriage: car, kickTube, turret: turretS, hood, bumper: bumper2056(), net: new Net2056() };
}
// how far the kicker bar has fallen (deg) with the intake ext of the way out: it folds against the
// bumper coming in and falls once the intake's clear of it (est.: past half its travel)
export const kickAt = (ext) => INTAKE.kick.fall * Math.min(1, Math.max(0, (ext - 0.3) / 0.35));
// the kicker bar's arms' load (N m about the pin, + counterclockwise seen from the robot's left) at
// deg (from folded): their weight (0.25 kg at 0.12 m, est.), the tubing pulling them toward their stop
// (the way they fall: sign of fall)
export function kickLoad(deg) {
  const K = INTAKE.kick, a = (K.a0 + deg) * DEG, s = Math.sign(K.fall);
  return -K.mass * 9.81 * 0.12 * Math.cos(a) + s * (K.tubing.preload + K.tubing.rate * Math.max(0, (K.fall - deg) * s * DEG));
}
// the foam's rolling resistance on the floor's rollers (as 1690's floorRoll)
export function floorRoll(contacts, dt) {
  for (const c of contacts) {
    if (!/^(floor|top) roller/.test(c.wheel.name) || !(c.force > 0)) continue;
    const b = c.ball, w = b.angvel(), dw = (ROLL.rr * c.force * R * dt) / (0.4 * FUEL.mass * R * R);
    const wz = Math.sign(w.z) * Math.max(0, Math.abs(w.z) - dw);
    b.setAngvel({ x: w.x, y: w.y, z: wz }, true);
  }
}
// the intake ext (0 in, 1 out) of the way out, the kicker bar's arms kickDeg from folded: the intake's
// front panel and sides, its rollers, the bar
export function placeIntake(sf, wheels, ext, kickDeg = kickAt(ext)) {
  const [dx, dy] = INTAKE.slide, K = INTAKE.kick, piv = [K.pivot[0] + dx * ext, K.pivot[1] + dy * ext, 0];
  sf.carriage.setPose([dx * ext, dy * ext, 0]);
  sf.kickTube.setPose(piv, 0, (K.a0 + kickDeg) * DEG);
  sf.kickTube.pivot = piv;
  wheels.forEach((w) => { const [x, y, z] = w.home; w.center = [x + dx * ext, y + dy * ext, z]; });
}
// the intake's front (m) ext of the way out
export const intakeFront = (ext) => INTAKE.frame.front - INTAKE.slide[0] * (1 - ext);
// the turret at yaw, the hood's arm at hoodDeg: the side plates, the shooter wheel (its face toward the
// FUEL coming up runs up) and the backspin kicker (counter-rotating: its face toward the wheel runs up
// too)
export function placeTurret(sf, wheels, yaw, hoodDeg) {
  const S = SHOOTER, K = S.kicker, [ax, ay] = S.axle;
  sf.turret.setPose([TURRET.at[0], 0, TURRET.at[1]], yaw, 0);
  const lat = [Math.sin(yaw), 0, Math.cos(yaw)];
  wheels.wheel.center = turretPoint([ax, ay, 0], yaw);
  wheels.wheel.axis = lat.map((v) => -v);
  const a = hoodDeg * DEG;
  sf.hood.setPose(turretPoint([ax, ay, 0], yaw), yaw, a);
  wheels.kicker.center = turretPoint([ax + K.R * Math.cos(a), ay + K.R * Math.sin(a), 0], yaw);
  wheels.kicker.axis = lat;
}

// The net over the hopper ("2025 barge net top to prevent fuel from escaping the hopper", the
// binder): cord netting pinned along the hopper's side panels and back, and out over the intake to
// its front panel's top. The load lifts it, most at the middle (a half sine across and fore and aft),
// against its stretch: est. 40 N/m pressed at its middle, stiffening as it tightens (k3), up to 0.12
// m. NET.bulge: lifted further (a fill, the benches').
export const NET = { x0: BACK.x, x1: INTAKE.frame.front, y: WALL.top, hw: WALL.right, bulge: 0 };
// (press: how far a TRENCH's arm riding over it can press it back down onto the load: 2 cm, est.)
export const PLATE = { k: 40, k3: 2e5, max: 0.12, tau: 0.05, press: 0.02 };
// the net's lift shape at (x, z): 1 at its middle, 0 at its pinned edges
export const bowShape = (x, z) => {
  const v = Math.min(1, Math.max(0, (z + NET.hw) / (2 * NET.hw))), u = Math.min(1, Math.max(0, (x - NET.x0) / (HALF_L - NET.x0)));
  return Math.sin(Math.PI * v) * Math.sin(Math.PI * Math.min(1, u * 0.85 + 0.15 * u * u));
};
function topY(x, z, x1, bow) {
  const B = NET.bulge, u = Math.min(1, Math.max(0, (x - NET.x0) / (x1 - NET.x0))), v = Math.min(1, Math.max(0, (z + NET.hw) / (2 * NET.hw)));
  const lift = B * Math.sin(Math.PI * u) * Math.sin(Math.PI * v);
  if (x <= HALF_L) return NET.y + bow * bowShape(x, z) + lift;
  // (out over the intake, down to its front panel's top)
  const edge = NET.y + bow * bowShape(HALF_L, z);
  return edge - (edge - INTAKE.frame.top) * Math.min(1, (x - HALF_L) / Math.max(0.06, x1 - HALF_L)) + lift;
}
export function netAt(x, z, x1 = NET.x1, bow = 0) {
  const e = 1e-3, y = topY(x, z, x1, bow);
  return [y, (topY(x + e, z, x1, bow) - y) / e, (topY(x, z + e, x1, bow) - y) / e];
}
export const netY = (x, z, x1, bow) => topY(x, z, x1 ?? NET.x1, bow ?? 0);
export const plateForce = (b) => PLATE.k * b + PLATE.k3 * b ** 3;
const plateK = (b) => PLATE.k + 3 * PLATE.k3 * b * b;
class Net2056 extends Surface {
  constructor() {
    super({ name: 'net', vertices: [0, 0, 0, 0.01, 0, 0, 0, 0, 0.01], indices: [0, 1, 2], mu: MU.net, tread: TREAD.net });
    this.x1 = NET.x1;
    this.bow = 0; this.force = 0; this.cap = PLATE.max;
    this.seen = new Map();
  }
  place(x1) { this.x1 = x1; }
  at(x, z) { return netAt(x, z, this.x1, this.bow); }
  touches(p, b) {
    const [x, y, z] = p;
    // (not over the turret's corner, which the walls close off)
    if (x >= this.x1 || x <= NET.x0 || Math.abs(z) >= NET.hw || (x < BACK.front && z > GUIDE.to[1])) return [];
    const [h, hx, hz] = this.at(x, z), l = Math.hypot(hx, 1, hz), dist = (h - y) / l;
    if (dist >= R || dist < -0.03) return [];
    if (b) this.seen.set(b, x <= HALF_L ? bowShape(x, z) : bowShape(HALF_L, z) * Math.max(0, 1 - (x - HALF_L) / 0.06));
    return [{ depth: R - dist, n: [hx / l, -1 / l, hz / l] }];
  }
  // after a step: lifted as far as the load pressing on it balances its stretch (as 1690's plate)
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
    this.bow = Math.min(this.cap, Math.max(0, this.bow + step * Math.min(1, dt / PLATE.tau)));
  }
}
export { Net2056 };
