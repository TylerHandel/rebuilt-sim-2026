// 6328's Darwin as physical mechanisms (js/mech/mechanism.js), from their CAD and their code
// (Mechanical-Advantage/RobotCode2026Public: Slamtake, Slam, Hopper, Kicker, Flywheel, Hood,
// LaunchCalculator, RollerSystem, CurrentLimits, CompactingCommands, RobotContainer, and the roboRIO's
// C++ IO layer, src/main/cpp/Robot.cpp and src/main/cpp/subsystems/*IOTalonFX.cpp). Robot frame: x
// forward (the intake), y up, z to its right.
//
// The CAD: their public Onshape document ("6328-26C-0000", 2da573df9d139d7a1e0f08a8) is view-only
// (the API serves its BOM and shaded views, no export), so the geometry is the AdvantageScope model
// their code repository ships (ascope_assets/Robot_Darwin: the same assembly's glTF export, their
// 0100 Drivebase, 0410 Superdexer, 3100 Full Wide Launcher, 2300 Intake and hopper). Its launcher,
// floor and drivebase match the Onshape document's shaded views (the drum's axle, the inertia wheel,
// the kickers, the motors and pulleys within a few mm); its intake is the arm they ran before their
// last revision (the binder's final one adds a static bottom roller under the powered one and sits
// ~2 cm higher, its slot pin in the same place), and its hopper still has the roof they replaced
// with a net for Champs (their build thread, Chief Delphi 509595 #891; the binder's "Net" variant):
// the roof is left out and the net drawn over the walls (NET below).
//
// The ball path: the slap-down intake's 2in roller (polycarbonate under WCP silicone) pulls FUEL in
// under it and rolls them up its ramp plate, which stands in front of the bumper, into the hopper's
// front. The hopper's front (its front wall and two dynamic walls) slides out with the intake: a pin
// on the intake's side plate runs in a slot angled 70 deg in the walls (their DarwinMechanism3d's
// geometry), all the way out (0.31 m) with the intake down, in with it up. The Superdexer (their
// "hopper" rollers): nine 1.5in polycarbonate rollers under silicone tubing, the floor sloping down
// to the back, then the launcher: three 2in rollers (the back kicker) curve up the back of the
// tunnel, the 2.5in front kicker roller on its front pinches FUEL up into the 4in drum, and the hood
// (carbon-fiber tubes on an arc 0.186 m round the drum's axle, and a 2in roller at its end) wraps
// them up over the front of the drum and throws them out over the back. All full width.
// Drives (the code's IO configs, the CAD's belts and gears, the binder's ratios):
//   intake roller: two Kraken X60s (Robot.cpp IntakeRoller 18 + follower 6, 70 A supply, no stator
//     limit), 12t:24t (the CAD's 24T intake roller pulley; their SensorToMechanismRatio 2.0, the
//     binder's "geared 2:1"); 13 V in (clamped to the battery), -6 V out
//   slam (the arm's pivot): an X60 through 52.5:1 (their SensorToMechanismRatio; the binder says
//     42:1), 70 A stator, 50 A supply, PositionVoltage kP 80 V per rotation; 17.4 deg down
//     (slamMinAngle, its hard stop) to 145.8 up (slamMaxAngle)
//   floor (HopperRoller 15 + follower 16): two X60s, 40 A supply, each 12t onto the live axle's 24t
//     (the CAD; the binder says 1.5:1), the rollers belted 1:1 (40T belts); 12 V while launching
//   kickers: one X60 each through 3:1 (their SensorToMechanismRatio 3.0, the CAD's 36t on the front
//     roller's shaft; kV 0.056 and 0.0575 V per rad/s of roller confirm it), 30 A supply, velocity
//     loops at their surface speed (6 m/s, 3 from 4 m out) over the front roller's 1.25in and the
//     back rollers' 0.79in radius
//   flywheel: four X60s 1:1 to the drum (the binder; the CAD's 24t:24t #25 chains, the motors chained
//     12t:12t in pairs), 40 A supply each, no stator limit; the drum's 30T belts a 6in steel inertia
//     wheel at 15t (2:1, the binder) and the hood's jackshaft at 30t, which turns the hood's 2in
//     roller the other way 34t:34t (1:1: half the drum's surface speed, the binder)
//   hood: an X44 through 12:36, 18:48 and a 10t pinion on a 9.25in-radius rack about the drum's
//     axle (148:1, their SensorToMechanismRatio), 10 to 41.2 deg
import { FUEL, IN } from '../constants.js';
import { Block, Motor, Shaft, Surface, Train, Wheel, BUMPER_MU } from './mechanism.js';

const R = FUEL.radius, TAU = 2 * Math.PI, DEG = Math.PI / 180;
// what each grips FUEL with (estimates, in the order 8793's team gave: compliant wheels most,
// silicone and rubber, bare polycarbonate least). The intake and floor rollers are under WCP silicone
// tubing (the binder); the kickers, drum and hood roller are polycarbonate and aluminum in the BOM
// with no sleeve named: shooters' rollers are wrapped, so silicone there too (an estimate)
export const MU = { silicone: 1.0, polycarb: 0.5, cf: 0.3, net: 0.25 };
const SIL_TREAD = 20000; // silicone sleeve over a tube (N/m; 1678's)

// ------------------------------------------------------------------ the layout (robot frame)
export const PIVOT = [0.317, 0.1855];   // the intake's pivot (their robotToPivotX 12.48in, 0.186 up)
export const DRUM = [-0.2639, 0.4861];  // the drum's axle, which the hood turns about
// their launcher's position (LauncherConstants.robotToLauncher: 0.151 m behind the center, facing
// back): their tables' distances are from here to the HUB's center
export const LAUNCHER_X = -0.151;
export const INTAKE = {
  // as exported: folded up, its slot pin 147.5 deg round from forward about the pivot (their angles
  // are that pin's: DarwinMechanism3d, "determined from CAD defined as angle to washer")
  cad: 147.5, down: 17.4, up: 145.8, // Slam.slamMinAngle, slamMaxAngle
  roller: { at: [0.1652, 0.4308], r: 0.0264, width: 0.547 }, // 2in tube under silicone (0.0527 m over it)
  // the arm (side plates, roller, its two X60s, bash bar, spring steel, ramp: ~3.2 kg, its center
  // 0.22 m out 20 deg behind the pin's line; 0.18 kg m^2 about the pivot): estimates from the BOM's
  // masses and the CAD's layout
  mass: 3.2, com: 0.222, comDeg: -20, J: 0.18,
  pin: 7.204 * IN, // the slot pin's distance from the pivot (DarwinMechanism3d pivotToWasherNorm)
};
// the hopper's sliding front (front wall, dynamic walls and their angles: 4.6 lb in the BOM) where
// their DarwinMechanism3d puts it for the pin at deg, from the CAD's (as exported, in)
export const SLIDE_MASS = 2.1;
export function hopX(deg) {
  const t = deg * DEG;
  return (12.48 + 7.204 * Math.cos(t) + (7.204 * Math.sin(t) - 2.393) / Math.tan(70 * DEG) + 6.789) * IN;
}
export const slideExt = (deg) => hopX(deg) - hopX(INTAKE.cad);
export const slideRate = (deg) => ((slideExt(deg + 0.5) - slideExt(deg - 0.5)) * 180) / Math.PI; // m per rad of arm (up)
// the floor: the Superdexer's seven and the launcher's two, back to front; 1.5in polycarbonate
// under silicone (0.0401 m over it), belted 1:1
export const FLOOR = [[-0.1587, 0.1905], [-0.1114, 0.1779], [-0.0620, 0.1780], [-0.0140, 0.1840], [0.0350, 0.1900], [0.0835, 0.1960], [0.1320, 0.2020], [0.1810, 0.2080], [0.2290, 0.2130]];
export const FLOOR_R = 0.02, FLOOR_W = 0.538;
// the back kicker's three rollers up the back of the tunnel, bottom to top (their 0.79in radius)
export const BACK = [[-0.2059, 0.2186], [-0.2510, 0.2816], [-0.2664, 0.3703]];
export const BACK_R = 0.79 * IN, BACK_W = 0.51;
// the front kicker roller (2.5in polycarbonate, their 1.25in radius)
export const KICK = { at: [-0.0789, 0.3572], r: 1.25 * IN, width: 0.559 };
export const SHOOTER = {
  drum: { r: 0.0508, width: 0.533 },
  // the hood's 2in polycarbonate roller at the hood's end, as exported (their 10 deg)
  roller: { at: [-0.0638, 0.5214], r: 0.0254, width: 0.544 },
  hoodMin: 10, hoodMax: 41.2, hoodCad: 10, // Hood.minAngle, maxAngle; the model's export
  // inertia (kg m^2) about each axis, from the BOM's masses and the CAD's sizes: the drum (6061, 3.14
  // lb, 4in OD: a 1/8in wall) 3.4e-3, its PLA caps, center shaft, sprockets and pulleys ~6e-4; the
  // inertia wheel the binder's "6in steel flywheel (2.5 lbs)" as a disk, 3.3e-3 at 2x the drum's speed
  // (its 0.0132 at the drum: their sim's 0.02 for the whole flywheel, Flywheel.java, is about the
  // two); the hood's roller, jackshaft and gears ~1e-4 (estimates)
  drumJ: 4.0e-3, flyJ: 3.3e-3, flyPerDrum: 30 / 15, hoodJ: 1.0e-4,
  // Flywheel.java: their feedforward (V; rad/s of the motors, which turn with the drum) and PID
  // (VelocityVoltage, kP per rotation/s), the setpoint profile's limits, the bang-bang's boost
  kS: 0.28, kV: 0.0192, kA: 0.007, kP: 0.6, maxAccel: 350, bangBang: 2.0, bangMin: 3.25, pidOffset: 4.0,
  supplyTeleop: 160, supplyAuto: 60, eff: 0.8, idle: 80, // (A, all four; rad/s)
};
// the kickers' velocity loops (Kicker.java: kP per rotation/s of the roller, kS, kV per rad/s)
export const KICKER = { front: { kP: 1.5, kS: 0.48, kV: 0.056 }, back: { kP: 3.0, kS: 0.4, kV: 0.0575 }, blearghh: 6.0 };

const interp = (t, x) => {
  if (x <= t[0][0]) return t[0][1];
  for (let k = 1; k < t.length; k++) if (x <= t[k][0]) return t[k - 1][1] + ((t[k][1] - t[k - 1][1]) * (x - t[k - 1][0])) / (t[k][0] - t[k - 1][0]);
  return t[t.length - 1][1];
};
// LaunchCalculator's maps: the launcher's distance to the HUB (m) -> hood angle (deg), flywheel
// (rad/s), kicker surface speed (m/s), time of flight (s); and the passing maps
export const HUB_MAP = {
  hood: [[1.12, 10], [1.52, 20], [2.01, 23], [2.73, 26], [3.51, 29], [3.93, 32], [4.40, 34], [4.68, 36], [4.90, 38], [5.01, 38]],
  fly: [[1.12, 145], [1.52, 145], [2.01, 153], [2.73, 160], [3.51, 175], [3.93, 188], [4.40, 192], [4.68, 195], [4.90, 195], [5.01, 195]],
  kicker: [[0, 6], [3, 6], [4, 3], [5, 3]],
  tof: [[1.63, 1.017], [2.40, 0.967], [3.25, 1.19], [4.15, 1.18], [4.875, 1.25]],
};
export const PASS_MAP = {
  hood: [[5.46, 38], [17.16, 38]],
  fly: [[5.46, 160], [6.62, 180], [7.80, 200], [17.16, 400]],
  kicker: [[0, 6]],
  tof: [[5.46, 1.27], [6.62, 1.39], [7.8, 1.49], [11.0, 1.75], [13.0, 1.76], [17.16, 2.5]],
};
// their limits (LaunchCalculator): HUB shots from 0.6 to 6 m, passes from 3 to 17.16 m
export const RANGE = { hub: [0.6, 6.0], pass: [3.0, 17.16] };
// The sim's FUEL leave this robot's drum differently from theirs (tools/mechlab6328.mjs tune: what
// leaves at each row of their map, and where it comes down): SIM_FLY (null: theirs) re-finds each
// row's flywheel speed so its FUEL, fed as in play at the row's hood angle, come down through the
// HUB's opening from the row's distance (rad/s), as 1678's SIM_PASS. THEIR_FLY=1 for their own.
export const SIM_FLY = { hub: null, pass: null };
const THEIR = typeof process !== 'undefined' && !!process.env?.THEIR_FLY;
export function shotFor(kind, d, { theirs = THEIR } = {}) {
  const m = kind === 'pass' ? PASS_MAP : HUB_MAP, sim = !theirs && SIM_FLY[kind];
  const fly = sim ? interp(m.fly.map(([x], i) => [x, sim[i]]), d) : interp(m.fly, d);
  return { fly, hood: interp(m.hood, d), kicker: interp(m.kicker, d), tof: interp(m.tof, d) };
}
// the preload: eight on the floor (robot frame)
export const PRELOAD = [
  [0.19, 0.30, -0.225], [0.19, 0.30, -0.075], [0.19, 0.30, 0.075], [0.19, 0.30, 0.225],
  [0.03, 0.28, -0.225], [0.03, 0.28, -0.075], [0.03, 0.28, 0.075], [0.03, 0.28, 0.225],
];

// ------------------------------------------------------------------ the drives
export function build6328() {
  // the intake roller: positive turns it inward (its bottom runs rearward, its back up the ramp)
  const intake = new Train({ name: 'intake roller', motor: new Motor('krakenX60', { count: 2, statorLimit: Infinity, supplyLimit: 70 }) });
  const roller = intake.add(new Shaft({ name: 'intake 2in roller', inertia: 1.8e-4 }), -12 / 24);
  // the floor: positive carries FUEL back along it (the rollers' tops run rearward)
  const floor = new Train({ name: 'Superdexer floor', motor: new Motor('krakenX60', { count: 2, statorLimit: Infinity, supplyLimit: 40 }) });
  const rollers = FLOOR.map((_, i) => floor.add(new Shaft({ name: `floor roller ${i + 1}`, inertia: 3e-5 }), 12 / 24));
  // the back kicker: positive lifts FUEL (the rollers' fronts run up)
  const backT = new Train({ name: 'back kicker', motor: new Motor('krakenX60', { statorLimit: Infinity, supplyLimit: 30 }) });
  const back = BACK.map((_, i) => backT.add(new Shaft({ name: `back kicker roller ${i + 1}`, inertia: 3e-5 }), 1 / 3));
  // the front kicker: positive lifts FUEL (its back runs up)
  const frontT = new Train({ name: 'front kicker', motor: new Motor('krakenX60', { statorLimit: Infinity, supplyLimit: 30 }) });
  const kick = frontT.add(new Shaft({ name: 'front kicker 2.5in roller', inertia: 2.5e-4 }), -1 / 3);
  // the flywheel: positive runs the drum's front up, the hood roller's back (toward the drum) up
  const S = SHOOTER;
  const flywheel = new Train({ name: 'flywheel', motor: new Motor('krakenX60', { count: 4, statorLimit: Infinity, supplyLimit: 40 }) });
  const drum = flywheel.add(new Shaft({ name: 'drum', inertia: S.drumJ }), 1);
  flywheel.add(new Shaft({ name: '6in steel inertia wheel (30T:15T)', inertia: S.flyJ, drag: 0 }), S.flyPerDrum);
  const hood = flywheel.add(new Shaft({ name: 'hood 2in roller', inertia: S.hoodJ }), -1);
  // the slam (the arm's pivot): positive turns the arm up, toward stowed
  const pivot = new Shaft({ name: 'slam pivot', motor: new Motor('krakenX60', { ratio: 52.5, statorLimit: 70, supplyLimit: 50 }), inertia: INTAKE.J, drag: 0.5 });
  return { intake, roller, floor, rollers, backT, back, frontT, kick, flywheel, drum, hood, pivot };
}

// the wheels (robot frame; the intake's and the hood's posed with placeIntake / placeHood)
export function wheels6328(t) {
  const Z = [0, 0, 1];
  const floor = FLOOR.map(([x, y], i) => new Wheel({ name: t.rollers[i].name, shaft: t.rollers[i], center: [x, y, 0], axis: Z, radius: FLOOR_R, width: FLOOR_W, mu: MU.silicone, tread: SIL_TREAD }));
  const kick = [
    ...BACK.map(([x, y], i) => new Wheel({ name: t.back[i].name, shaft: t.back[i], center: [x, y, 0], axis: Z, radius: BACK_R, width: BACK_W, mu: MU.silicone, tread: SIL_TREAD })),
    new Wheel({ name: t.kick.name, shaft: t.kick, center: [...KICK.at, 0], axis: Z, radius: KICK.r, width: KICK.width, mu: MU.silicone, tread: SIL_TREAD }),
  ];
  const I = INTAKE.roller;
  const intake = [new Wheel({ name: 'intake 2in roller (silicone)', shaft: t.roller, center: [...I.at, 0], axis: Z, radius: I.r, width: I.width, mu: MU.silicone, tread: SIL_TREAD })];
  intake.forEach((w) => { w.home = w.center.slice(); });
  const S = SHOOTER;
  const drum = new Wheel({ name: 'drum', shaft: t.drum, center: [...DRUM, 0], axis: Z, radius: S.drum.r, width: S.drum.width, mu: MU.silicone, tread: SIL_TREAD });
  const hood = [new Wheel({ name: 'hood 2in roller', shaft: t.hood, center: [...S.roller.at, 0], axis: Z, radius: S.roller.r, width: S.roller.width, mu: MU.silicone, tread: SIL_TREAD })];
  hood.forEach((w) => { w.home = w.center.slice(); });
  return { floor, kick, intake, drum, hood };
}
// rotate a point (x, y) about (cx, cy) by a (rad)
const rot = ([x, y], [cx, cy], a) => { const c = Math.cos(a), s = Math.sin(a), dx = x - cx, dy = y - cy; return [cx + dx * c - dy * s, cy + dx * s + dy * c]; };
// the arm's turn from the CAD's pose at their angle deg
export const intakeRot = (deg) => (deg - INTAKE.cad) * DEG;
export function placeIntake(wheels, deg) {
  const a = intakeRot(deg);
  for (const w of wheels) { const [x, y] = rot(w.home, PIVOT, a); w.center = [x, y, 0]; }
}
// the hood at their angle deg: it turns about the drum's axle, over the top toward the back, from
// the CAD's pose (their DarwinMechanism3d: pitch -angle about the launcher, 1:1)
export const hoodRot = (deg) => (deg - SHOOTER.hoodCad) * DEG;
export function placeHood(wheels, deg) {
  const a = hoodRot(deg);
  for (const w of wheels) { const [x, y] = rot(w.home, DRUM, a); w.center = [x, y, 0]; }
}
// the shot's elevation at hood angle deg: square to the line from the drum's axle to the hood's end
// (its 2in roller, 9.9 deg up from forward about the axle as exported), out the back: 90 less the
// hood's angle, about (tools/mechlab6328.mjs shooter measures what leaves; EL_OFFSET is the
// difference it finds)
export const EL_OFFSET = 0;
export function hoodElevation(deg) {
  const [hx, hy] = SHOOTER.roller.at, a = Math.atan2(hy - DRUM[1], hx - DRUM[0]) / DEG + (deg - SHOOTER.hoodCad);
  return 90 - a + EL_OFFSET;
}

// The hopper's walls and top. Static walls (polycarbonate, "Static Wall Left/Right" in the BOM;
// their size and place from the Onshape document's side view, an estimate to a cm): inside the
// dynamic walls, from the launcher's front to the frame's front, 0.2 to 0.556 m up. The net
// (the binder's "stretched net over polycarbonate walls that flexes upward to hold more fuel ...
// ~75 fuel capacity at peak"): tied across the launcher's top at the back and along the top of the
// sliding front wall, riding out with it, and along the walls; slack, the load lifting it up to
// NET.bulge at its middle (as 581's, js/mech/r581.js NET), then taut, giving NET.tread: estimates,
// sized to the binder's ~75
export const WALLS = { x0: -0.035, x1: 0.35, y0: 0.2, y1: 0.556, hw: 0.334 };
export const NET = { backX: 0.0, backY: 0.555, frontX: 0.346, frontY: 0.51, hw: 0.334, bulge: Number(globalThis.process?.env?.NET_BULGE ?? 0.08), tread: 8000 };
export function netAt(x, z, ext) {
  const fx = NET.frontX + ext, L = Math.max(0.01, fx - NET.backX), B = NET.bulge;
  const u = Math.min(1, Math.max(0, (x - NET.backX) / L)), v = Math.min(1, Math.max(0, (z + NET.hw) / (2 * NET.hw)));
  const su = Math.sin(Math.PI * u), sv = Math.sin(Math.PI * v), slope = (NET.frontY - NET.backY) / L;
  return [NET.backY + slope * L * u + B * su * sv, slope + (B * Math.PI * Math.cos(Math.PI * u) * sv) / L, (B * su * Math.PI * Math.cos(Math.PI * v)) / (2 * NET.hw)];
}
class Net6328 extends Surface {
  constructor() {
    super({ name: 'net', vertices: [0, 0, 0, 0.01, 0, 0, 0, 0, 0.01], indices: [0, 1, 2], mu: MU.net, tread: NET.tread });
    this.place(0);
  }
  place(ext) { this.ext = ext; this.x1 = NET.frontX + ext; }
  touches(p) {
    const [x, y, z] = p;
    if (x <= NET.backX || x >= this.x1 || Math.abs(z) >= NET.hw) return [];
    const [h, hx, hz] = netAt(x, z, this.ext), l = Math.hypot(hx, 1, hz), dist = (h - y) / l;
    if (dist >= R || dist < -0.03) return [];
    return [{ depth: R - dist, n: [hx / l, -1 / l, hz / l] }];
  }
}
// the front bumper (solid foam under fabric: js/mech/mechanism.js Block), where FUEL climb out of the
// intake: their frame 26.5in long (DriveConstants), 3.25in of bumper to their 33in
export const HALF_L = 26.5 * IN / 2, HALF_W = 27.5 * IN / 2;
export function bumperSurface() {
  return new Block({ name: 'bumper', x0: HALF_L + 0.75 * IN, x1: HALF_L + 3.25 * IN, y0: 0.032, y1: 0.16, hw: HALF_W + 3.25 * IN, mu: BUMPER_MU, tread: 20000 });
}

// the ball path's surfaces: from their CAD (js/mech/path6328.js) the body, the intake on its pivot
// (it swings, on the slam's shaft), the sliding front (it rides on the arm through the pin and slot)
// and the hood on the drum's axle; and the static walls, the net and the bumper
export function pathSurfaces6328(mesh, t, { mu = 0.35 } = {}) {
  const s = (k, pos) => new Surface({ name: `${k} (CAD)`, vertices: mesh[k].v.map((x) => x / 1000), indices: mesh[k].i, mu, pos });
  const body = s('body', [0, 0, 0]);
  const intake = s('intake', [PIVOT[0], PIVOT[1], 0]).onShaft(t.pivot, [PIVOT[0], PIVOT[1], 0]);
  const slide = s('slide', [0, 0, 0]).onSlide(t.pivot, [1, 0, 0]);
  const hood = s('hood', [DRUM[0], DRUM[1], 0]);
  const W = WALLS, V = [];
  for (const z of [-W.hw, W.hw]) V.push(W.x0, W.y0, z, W.x1, W.y0, z, W.x1, W.y1, z, W.x0, W.y1, z);
  const walls = new Surface({ name: 'static walls (polycarbonate)', vertices: V, indices: [0, 1, 2, 0, 2, 3, 4, 5, 6, 4, 6, 7], mu: MU.polycarb });
  return { body, intake, slide, hood, walls, net: new Net6328(), bumper: bumperSurface() };
}
export function placeNet(net, ext) { net.place(ext); }
