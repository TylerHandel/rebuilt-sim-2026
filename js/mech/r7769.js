// 7769 The CREW's CHUNK as physical mechanisms (js/mech/mechanism.js), from their CAD (Onshape "Full
// Robot", assembly "Chunk", 692fc43a78732f76f6a7fc01: its STEP and glTF exports and its bill of
// materials) and their Chief Delphi thread ("FRC 7769 - CAD & Tech Slides : CHUNK", t/521008: the
// team's answers there). There is no public code and their tech slides are gone (the team: deleted,
// no backup), so the controls, speeds and some sizes are estimates: each one says so. Robot frame: x
// forward (the intake), y up, z to its right; the CAD's x to the robot's left, y back, z up.
//
// The ball path: the intake slides out on two polycarbonate racks (the team's "linear intake"), its
// 3in polycarbonate roller wrapped in cat-tongue grip tape pulling FUEL in under it and up over the
// kick bar (a 1/2in hex hanging on arms in front of the bumper) and the bumper; the hopper's front and
// moving sides ride out with it, the stationary polycarbonate sides stay, a net over the top. The floor
// is ten 15mm timing belts (105T, the team: "regular 105T x 15mm wide timing belts from WCP/
// Thriftybot") on 18T pulleys, sloping down toward the back, past a row of stationary rollers to the
// ramp: a static curved plate up the back to under the shooter's wheels (the team: "top rollers
// powered and the lower surface being static", after their week-4 rebuild). The feeder's shaft runs
// across in front of the ramp: eight 2.25in 35A compliant wheels and two 2in ION wheels in the middle,
// four 2in "vectored" wheels at each end (print-in-place PETG, their rollers at 45 deg, after a 2910
// design: they push FUEL in toward the middle) pinch FUEL against the ramp and roll them up it, two
// abreast, into two stacks of six 4in 50A Stealth wheels, which carry them up round their front under
// the fixed hood and the adjustable hood (no rollers: "a static surface on the hood") and throw them
// up and out over the back. The chamber is 16.5in wide, about 2.75 FUEL (the team's "2.75 width
// shooter").
// Drives (the CAD's gears, belts and chain; motors and ratios from the BOM's part names):
//   intake roller: two Kraken X60s on the sliding intake, each 12t (HTD 5mm) to the roller's 30T
//     printed hub (55t belts): 2.5:1
//   slide: a Kraken X44, 12t (SplineXS) to 42t (20 DP) on a cross shaft, 18t:18t #25 chain to each
//     side's 10t 10 DP pinion on its rack (1in pitch circle): 3.5:1, 22.8 mm of travel per motor turn;
//     the team: a "dynamic deploy current limit ... from 45A when deploying to 5A at the deployed state"
//   floor: a Kraken X60 through a MAXPlanetary 3:1, 18T:18T (45t belt) to the front belt shaft: 3:1
//   feeder: a Kraken X60, 18t (20 DP) to 42t, 18t:18t #25 chain to the wheels' shaft: 2.33:1
//   shooter: four Kraken X60s, each 18T:18T (45t belts) to the wheels' 18.75in shaft: 1:1
//   hood: a Kraken X44, 12t:36t (20 DP), 12t:32t #25 chain to the hood's plate sprockets on the
//     wheels' shaft: 8:1
//   (the drive: SDS MK5n, Kraken X60 drive and X44 steer; the climber: an X60 through a 27:1
//     MAXPlanetary to a winch, an L1 hook they took off at Worlds)
import { FUEL, IN } from '../constants.js';
import { Block, BUMPER_MU, Motor, Shaft, Surface, Train, Wheel } from './mechanism.js';

const R = FUEL.radius, DEG = Math.PI / 180;
// what each grips FUEL with (estimates, in the order 8793's team gave: compliant wheels most, grip
// tape, rubber, bare plastic least): the cat-tongue grip tape on the intake roller, the 35A compliant
// and 40A ION wheels, the 50A Stealth wheels, the timing belts' smooth backs, the PETG vectored
// wheels' rollers, the polycarbonate and aluminum of the walls, ramp and hood
export const MU = { gripTape: 1.4, compliant: 1.6, ion: 1.5, stealth: 1.3, belt: 0.8, petg: 0.5, wall: 0.25, hood: 0.3 };
// how far each gives where a FUEL presses it (N/m; estimates, as 2910's and 2056's)
export const TREAD = { gripTape: 40000, compliant: 3000, ion: 4000, stealth: 12000, belt: 20000, petg: 40000 };

// ------------------------------------------------------------------ the layout (robot frame)
// the frame perimeter (their CAD's "Limits": 25.5 x 29.5in), the bumpers (the CAD's: 0.409 m out
// and 0.46 to the side, 0.038 to 0.152 m up; 3/4in backing on the frame, foam to the face)
export const HALF_L = 0.3239, HALF_W = 0.3747;
export const BUMPER = { x0: HALF_L + 0.75 * IN, x1: 0.409, y0: 0.038, y1: 0.152, hw: 0.46 };
// the intake's 3in roller (the CAD's "Intake Roller" under "Cat Tongue": 0.078 m across), as exported
export const INTAKE = { at: [0.5779, 0.1746], r: 0.039, width: 0.566 };
// the slide: as exported it's all the way out (the stop plates on the arms' backs against the
// sliders). Its racks slope 9.2 deg down toward the front (the CAD), so coming in it goes back and
// up along DIR. out: 0.305 m of travel (an estimate: 12in, which brings the hopper's front from 0.626
// m out back to the frame perimeter, the rack's teeth still on the pinion; the CAD has no other pose).
// pinion: the 10t 10 DP pinion's pitch radius; mass: what slides (the polycarbonate arms and racks,
// the hopper's front and moving sides, the cross tube, the roller and its two X60s: ~4.5 kg, an
// estimate from the BOM's parts)
const SLOPE = 9.2 * DEG;
export const SLIDE = { out: 0.305, dir: [-Math.cos(SLOPE), Math.sin(SLOPE)], pinion: 0.5 * IN, ratio: 42 / 12, mass: 4.5 };
export const SLIDE_CURRENT = { deploy: 45, held: 5 }; // (A, stator: the team's)
// the kick bar: its 1/2in hex 0.152 m below the pin on the intake's arms (0.4873, 0.1915 as
// exported), hanging just behind it (the CAD), 4.3 cm off the carpet in front of the bumper; coming
// in, the bumper swings it up and forward onto its rest plates on the frame (the CAD's "Kicker Rest
// Plate", level with the bumper's top)
export const KICK = { pin: [0.4873, 0.1915], bar: [0.455, 0.043], r: 0.007, max: Math.PI / 2 };
KICK.len = Math.hypot(KICK.bar[0] - KICK.pin[0], KICK.bar[1] - KICK.pin[1]);
KICK.b0 = Math.atan2(KICK.bar[0] - KICK.pin[0], KICK.pin[1] - KICK.bar[1]); // (from straight down, forward positive)
// the floor: ten 105T belts on 18T pulleys (28.6 mm pitch circle) between the front shaft and the back
// one, 15 mm wide from 0.035 to 0.269 m either side of the middle (the CAD), the run's outer face
// 1.6 mm over the pitch line
export const FLOOR = { front: [0.2045, 0.2030], back: [-0.0075, 0.1560], pitch: (18 * 0.005) / (2 * Math.PI), r: 0.016, hw: 0.27 };
FLOOR.len = Math.hypot(FLOOR.front[0] - FLOOR.back[0], FLOOR.front[1] - FLOOR.back[1]);
FLOOR.dir = [(FLOOR.back[0] - FLOOR.front[0]) / FLOOR.len, (FLOOR.back[1] - FLOOR.front[1]) / FLOOR.len, 0]; // (the run's way)
// the feeder's shaft: compliant wheels (2.25in 35A, 1in wide) from 6 to 146 mm either side of the
// middle, an ION wheel (2in 40A) at 165 mm, and four vectored wheels (2in) from 184 to 295 mm
export const FEED = { at: [-0.1061, 0.3175], compliant: { r: 2.25 * IN / 2, z: 0.076, w: 0.14 }, ion: { r: IN, z: 0.165, w: 0.0127 }, vector: { r: IN, z: 0.2395, w: 0.111 } };
// the shooter's wheels: two stacks of six 4in 50A Stealth wheels (1in each) either side of the
// middle plate, 19 to 171 mm out, on one shaft (the CAD's 18.75in hex)
export const AXLE = [-0.2286, 0.4953];
export const SHOOTER = {
  wheel: { r: 2 * IN, z: 0.0952, w: 0.152 },
  // inertia (kg m^2) about the shaft: twelve 4in Stealth wheels (AndyMark's 0.25 lb each, as a disk
  // with a hub: ~0.5 m r^2), the 18.75in hex, two 18t pulleys (an estimate) (the motors' rotors are
  // added by the Train); 1:1
  J: 1.6e-3,
  // the hood's travel up and back about the wheels' shaft from the CAD's pose (down: an estimate,
  // 0 to 35 deg; the CAD has it down, its top 0.555 m, under the TRENCH's 22.25in)
  hoodMin: 0, hoodMax: 35,
  // FUEL leave round the wheels' front at the hood's top edge, 14.7 deg over level from the shaft as
  // exported: going up and back at 75.3 deg (tangent to their path round the shaft), flatter as the
  // hood swings up and back (tools/mechlab7769.mjs shooter measures it)
  exitDeg: 14.7, exitR: 0.123,
  // the four X60s' current limits (estimates: 60 A stator, 40 A supply each)
  stator: 60, supply: 40,
  // its velocity loop (estimates: volts per motor rotation/s): kV x target + kP x error
  kV: 12 / (5800 / 60), kP: 0.4,
};
// the hood's motor: an X44 through 8:1 (its position loop, an estimate: it moves at most 7530 RPM /
// 8 = 5650 deg/s; held here to 400 deg/s)
export const HOOD = { ratio: 8, rate: 400, tol: 1.5 };
// the net over the hopper (the team's "hopper net"; not in the CAD): from the fixed hood's top (0.555
// m, x -0.02) over the stationary sides' tops (0.552 m, to 0.309 m out) down to the hopper front's
// top (0.494 m) wherever the intake has it; FUEL under it lift it as far as bulge (an estimate), past
// that it gives as a stretched net does (tread, an estimate)
export const NET = { x0: -0.02, y0: 0.552, xs: 0.309, yFront: 0.494, hw: 0.35, bulge: 0.02, tread: 3000 };
// the hopper's front face (the CAD's "Hopper Front": its inner face 0.6224 m out as exported)
export const FRONT_X = 0.6224;
// the preload, eight on the belts (robot frame; they settle)
const floorY = (x) => FLOOR.back[1] + ((x - FLOOR.back[0]) / (FLOOR.front[0] - FLOOR.back[0])) * (FLOOR.front[1] - FLOOR.back[1]) + FLOOR.r + R + 0.004;
export const PRELOAD = [0.02, 0.17].flatMap((x) => [-0.235, -0.08, 0.08, 0.235].map((z) => [x, floorY(x), z]));

// ------------------------------------------------------------------ the drives
export function build7769() {
  // the intake roller: positive turns it inward (its bottom runs rearward)
  const intake = new Train({ name: 'intake rollers', motor: new Motor('krakenX60', { count: 2, statorLimit: 60, supplyLimit: 40 }) });
  const roller = intake.add(new Shaft({ name: 'intake 3in roller (cat tongue)', inertia: 6e-4 }), -12 / 30);
  // the floor: positive runs the belts' top run back (down to the ramp)
  const floor = new Train({ name: 'floor belts', motor: new Motor('krakenX60', { ratio: 3, statorLimit: 60, supplyLimit: 40 }) });
  const belts = floor.add(new Shaft({ name: 'floor belts', inertia: 2e-4 }), 1);
  // the feeder: positive turns the wheels' backs up (FUEL rolled up the ramp behind them)
  const feedT = new Train({ name: 'feeder', motor: new Motor('krakenX60', { statorLimit: 60, supplyLimit: 40 }) });
  const feed = feedT.add(new Shaft({ name: 'feeder wheels', inertia: 3e-4 }), -18 / 42);
  // the shooter: positive turns the wheels' fronts up (FUEL up round their front and out over the back)
  const S = SHOOTER;
  const shooter = new Train({ name: 'shooter', motor: new Motor('krakenX60', { count: 4, statorLimit: S.stator, supplyLimit: S.supply }) });
  const wheels = shooter.add(new Shaft({ name: 'shooter wheels (4in Stealth)', inertia: S.J }), 1);
  // the slide's pinions (positive runs the intake out); what slides seen at the pinion; the racks'
  // friction (an estimate: enough that it doesn't run out down its racks with the X44 coasting)
  const slide = new Racks({ name: 'slide pinions', motor: new Motor('krakenX44', { ratio: SLIDE.ratio, statorLimit: SLIDE_CURRENT.deploy, supplyLimit: 40 }), inertia: SLIDE.mass * SLIDE.pinion ** 2, drag: 0.05, friction: 0.1 });
  return { intake, roller, floor, belts, feedT, feed, shooter, wheels, slide };
}
// the slide's pinions: a Shaft whose racks' friction holds it still against small pushes, and its
// stops (lo, hi: slideStops) as well (as 1114's)
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
export function slideStops(shaft, ext) { shaft.lo = ext <= 1e-9 ? 0 : -Infinity; shaft.hi = ext >= SLIDE.out - 1e-9 ? 0 : Infinity; }

// the wheels (robot frame, as exported; the intake's posed with placeIntake)
export function wheels7769(t, { vector = 45 } = {}) {
  const Z = [0, 0, 1];
  const intake = [new Wheel({ name: 'intake 3in roller (cat tongue)', shaft: t.roller, center: [...INTAKE.at, 0], axis: Z, radius: INTAKE.r, width: INTAKE.width, mu: MU.gripTape, tread: TREAD.gripTape })];
  intake.forEach((w) => { w.home = w.center.slice(); });
  const F = FEED, at = F.at;
  const feed = [
    ...[-1, 1].map((s) => new Wheel({ name: '2.25in 35A compliant wheels', shaft: t.feed, center: [...at, s * F.compliant.z], axis: Z, radius: F.compliant.r, width: F.compliant.w, mu: MU.compliant, tread: TREAD.compliant })),
    ...[-1, 1].map((s) => new Wheel({ name: '2in 40A ION wheel', shaft: t.feed, center: [...at, s * F.ion.z], axis: Z, radius: F.ion.r, width: F.ion.w, mu: MU.ion, tread: TREAD.ion })),
    // (the vectored wheels: their rollers at 45 deg, handed so the push along them is in toward the
    // middle, LH on the left and RH on the right; tools/mechlab7769.mjs vector checks the hand)
    ...[-1, 1].map((s) => new Wheel({ name: `2in vectored wheels (${s < 0 ? 'LH' : 'RH'})`, shaft: t.feed, center: [...at, s * F.vector.z], axis: Z, radius: F.vector.r, width: F.vector.w, kind: 'mecanum', mecanum: s * vector, mu: MU.petg, tread: TREAD.petg })),
  ];
  const S = SHOOTER.wheel;
  const shooter = [-1, 1].map((s) => new Wheel({ name: '4in 50A Stealth wheels', shaft: t.wheels, center: [...AXLE, s * S.z], axis: Z, radius: S.r, width: S.w, mu: MU.stealth, tread: TREAD.stealth }));
  return { intake, feed, shooter };
}
// the intake's offset from the CAD's pose (all the way out) with it ext m out along its racks
export function slideOffset(ext) { const d = SLIDE.out - ext; return [SLIDE.dir[0] * d, SLIDE.dir[1] * d]; }
export function placeIntake(wheels, ext) {
  const [dx, dy] = slideOffset(ext);
  for (const w of wheels) w.center = [w.home[0] + dx, w.home[1] + dy, w.home[2]];
}
// the kick bar's swing (rad, forward and up from hanging) with the intake ext m out: it hangs until
// its bar meets the bumper's face, then rides up it and over its top corner onto the rest plates
export function kickRot(ext) {
  const [dx, dy] = slideOffset(ext), px = KICK.pin[0] + dx, py = KICK.pin[1] + dy, L = KICK.len;
  const xb = BUMPER.x1 + KICK.r, yc = BUMPER.y1 + KICK.r;
  if (px + L * Math.sin(KICK.b0) >= xb) return 0;
  const b1 = Math.asin(Math.max(-1, Math.min(1, (xb - px) / L))), b2 = Math.acos(Math.max(-1, Math.min(1, (py - yc) / L)));
  return Math.min(KICK.max, Math.max(0, Math.min(b1, b2) - KICK.b0));
}
// the hood's turn about the wheels' shaft from the CAD's pose (rad, up and back positive) at deg
export const hoodRot = (deg) => deg * DEG;
// the shot's elevation (deg over level, going back) with the hood at deg, by the CAD's geometry:
// tangent to the FUEL's path round the shaft at the hood's top edge
export const hoodElevation = (deg) => 90 - SHOOTER.exitDeg - deg;

// the ball path's surfaces from their CAD (js/mech/path7769.js): the body (and its finer parts: the
// ramp and its stationary rollers; the fixed hood, which the shooter's Mechanism has, with the
// adjustable hood, so FUEL are pinched between them and the wheels in one solve); the intake on its racks (it slides, on the
// slide's pinions); the kick bar on its pin (it rides the intake, and swings up as it comes in); the
// adjustable hood on the wheels' shaft; the floor's belts (moving surfaces on the floor's shaft); the
// net; and the bumper (solid foam: a Block). FUEL slide on the CAD's polycarbonate and aluminum at
// MU.wall, on the hoods at MU.hood (estimates)
// (path: everything but the two hoods; shooter: the hoods)
export const SHOOTER_SURFACES = ['fixedHood', 'hood'];
export function pathSurfaces7769(mesh, t) {
  const s = (k, pos, mu = MU.wall) => new Surface({ name: `${k} (CAD)`, vertices: mesh[k].v.map((x) => x / 1000), indices: mesh[k].i, mu, pos });
  const body = s('body', [0, 0, 0]);
  const fine = s('bodyFine', [0, 0, 0], MU.hood);
  const fixedHood = s('fixedHood', [0, 0, 0], MU.hood);
  const slide = s('slide', [0, 0, 0]).onSlide(t.slide, [-SLIDE.dir[0], -SLIDE.dir[1], 0]);
  slide.slideK = SLIDE.pinion;
  const kick = s('kick', [...KICK.pin, 0]);
  const hood = s('hood', [...AXLE, 0], MU.hood);
  const belts = beltRun(t);
  const net = new Net();
  const bumper = new Block({ name: 'bumper', ...BUMPER, mu: BUMPER_MU, tread: 20000 });
  return { body, fine, slide, kick, belts, net, bumper, fixedHood, hood };
}
// the belts' top run (one strip across their width, the FUEL's side, sliding back along it at the
// belts' pitch-line speed), its ends rounded over the pulleys
function beltRun(t) {
  const F = FLOOR, V = [], I = [], NZ = 6, NA = 4;
  const [fx, fy] = F.front, [bx, by] = F.back, d = F.dir;
  // (the run's outward normal: up)
  const up = d[0] < 0 ? [-d[1], d[0]] : [d[1], -d[0]];
  // (the profile, front to back: over the front pulley's top, the run, over the back pulley's top)
  const prof = [];
  for (let a = NA; a >= 0; a--) { const th = (a / NA) * (Math.PI / 2); prof.push([fx - d[0] * F.r * Math.sin(th) + up[0] * F.r * Math.cos(th), fy - d[1] * F.r * Math.sin(th) + up[1] * F.r * Math.cos(th)]); }
  for (let a = 0; a <= NA; a++) { const th = (a / NA) * (Math.PI / 2); prof.push([bx + d[0] * F.r * Math.sin(th) + up[0] * F.r * Math.cos(th), by + d[1] * F.r * Math.sin(th) + up[1] * F.r * Math.cos(th)]); }
  const n = prof.length;
  for (let k = 0; k <= NZ; k++) for (const [x, y] of prof) V.push(x, y, -F.hw + (2 * F.hw * k) / NZ);
  for (let k = 0; k < NZ; k++) for (let j = 0; j < n - 1; j++) {
    const a = k * n + j, b = a + 1, c = a + n, e = c + 1;
    I.push(a, c, b, b, c, e);
  }
  const sf = new Surface({ name: 'floor belts', vertices: V, indices: I, mu: MU.belt, tread: TREAD.belt }).onSlide(t.belts, F.dir);
  sf.slideK = F.pitch;
  return sf;
}
// The net over the hopper (in the robot's frame): a sheet from the fixed hood's top over the
// stationary sides to the hopper front's top edge, wherever the intake has it; FUEL under it press it
// up as far as NET.bulge at its middle (an estimate), past that only as far as its tread gives, and
// nothing presses the load back down harder than that (as 1114's blanket, 1678's and 4414's nets)
class Net extends Surface {
  constructor() {
    super({ name: 'net', vertices: [0, 0, 0, 0.01, 0, 0, 0, 0, 0.01], indices: [0, 1, 2], mu: 0.5, tread: NET.tread });
    this.place(SLIDE.out);
  }
  place(ext) { this.x1 = FRONT_X + slideOffset(ext)[0]; this.yF = NET.yFront + slideOffset(ext)[1]; }
  // its height and slopes (d/dx, d/dz) at (x, z)
  at(x, z) {
    const N = NET, x1 = this.x1, xs = Math.min(N.xs, x1 - 0.01);
    let h, hx;
    if (x <= xs) { h = N.y0; hx = 0; } else { hx = (this.yF - N.y0) / (x1 - xs); h = N.y0 + (x - xs) * hx; }
    const L = x1 - N.x0, u = (x - N.x0) / L, v = (z + N.hw) / (2 * N.hw), B = N.bulge;
    const su = Math.sin(Math.PI * u), sv = Math.sin(Math.PI * v);
    return [h + B * su * sv, hx + (B * Math.PI * Math.cos(Math.PI * u) * sv) / L, (B * su * Math.PI * Math.cos(Math.PI * v)) / (2 * N.hw)];
  }
  touches(p) {
    const [x, y, z] = p;
    if (x <= NET.x0 || x >= this.x1 || Math.abs(z) >= NET.hw) return [];
    const [h, hx, hz] = this.at(x, z), l = Math.hypot(hx, 1, hz), dist = (h - y) / l;
    if (dist >= R || dist < -R) return [];
    return [{ depth: R - dist, n: [hx / l, -1 / l, hz / l] }];
  }
}
export function placeSlide(s, ext) {
  const [dx, dy] = slideOffset(ext);
  s.slide.setPose([dx, dy, 0]);
  s.kick.setPose([KICK.pin[0] + dx, KICK.pin[1] + dy, 0], 0, kickRot(ext));
  s.net.place(ext);
}
