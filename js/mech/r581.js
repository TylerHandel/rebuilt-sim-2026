// 581's Rubble as physical mechanisms (js/mech/mechanism.js), from their CAD (Onshape "581 2026
// Rubble", the "2026 Dumper Champs Bot" assembly through the document's own STEP) and their code
// (team581/frc-2026, comp-bot: the deploy, intake, conveyor, feeder, shooter and shooter hood
// subsystems' configs and states, HopperManager, RobotManager, PowerManager, AimParameterUtil).
// Robot frame: x forward (the intake), y up, z to its right; the CAD's x back, y right, z up.
//
// The ball path: the intake rides out 0.306 m on titanium racks (rising 5.5 deg toward the robot),
// its 3in roller (grip tape) out front pulling FUEL in under it, its 2in roller (grip tape) above
// and behind carrying them on up over the kicker bars (two 3/8in tubes on plates hanging from a
// pivot, pushed back against the bumper) and the bumper's top; the intake's side panels and front
// plate are the front of the hopper. The roller floor (eleven 1in rollers, one belt run) slopes
// down to the back, where FUEL go onto the shooter tower's curved plate; against it on the front,
// three 1.5in rollers lift them to the drum (4in) and the hood's two 2in rollers, which fling them
// up and back over the drum. All about four wide. The floor pivots on its powered back shaft, and
// 2in followers at its front corners ride ramps on the intake's side plates: pulling the intake in
// lifts the floor's front (the cam below), which is how their code compacts the load (the deploy's
// SCORE_COMPACTION).
// Drives (the CAD's belts and gears, their code's ratios and current limits):
//   intake: two Kraken X60s (75 A stator, 25 A supply; PowerManager's IDLE, 80 A in auto), each
//     12t:20t (HTD 15mm) to the 2in roller, 20t:30t from it to the 3in
//   deploy: two Kraken X44s (a differential pair: 50 A stator, 18 A supply, torque current
//     +100/-35 A) through 8:40 (their 40/8) to a 10t 10DP pinion on each rack (1in pitch circle:
//     their 1/(pi x 1in) per turn); position in inches from all the way in (0 to their 11.8)
//   conveyor (the floor): two X60s locked together 20t:20t (50 A stator, their PowerManager's 10-30 A
//     supply), 12t:18t onto the floor's back shaft, the rest 18t:18t and 23t:23t from it (1:1)
//   feeder (the tower): two X60s locked together 20t:20t (50 A, 10-30 A supply), 12t:18t onto the
//     low 1.5in roller, then 18t:18t and 23t:23t up the tower (1:1)
//   shooter: four X60s (150 A stator, 50 A supply scoring, 200 A torque current) belted 1:1 to the
//     drum (two direct 18t:18t, two 20t:20t onto those; their RPM is the motors' and the drum's);
//     the drum's 45t belts a 4in steel inertia flywheel (WCP-2414) at 15t (3:1), and its 18t a
//     jackshaft on the hood at 15t, geared 30t:22t to the hood rollers (turning the other way,
//     1.64 times the drum)
//   hood: an X44 through 8:62 and a 10t pinion on the hood plates' 154t sectors about the drum's
//     axle (119.4:1)
import { FUEL, IN } from '../constants.js';
import { Motor, Shaft, Surface, Train, Wheel } from './mechanism.js';

// what each grips FUEL with (estimates, in the order 8793's team gave): the intake rollers' grip
// tape ("Cat Tongue" in the CAD); the drum, hood and tower rollers are drawn as bare tubes, but
// shooters are wrapped, so grip tape there too (an estimate); the floor's 1in tubes are black in
// the CAD (anodized or sleeved; an estimate); the net's cord (NET below: bungee cord in a braided
// sheath, slick and round, as 1706's net's cord: 0.25, js/mech/r1706.js)
export const MU = { gripTape: 1.3, floor: 0.6, polycarb: 0.5, net: 0.25 };

// ------------------------------------------------------------------ the layout (robot frame)
export const DRUM = [-0.2883, 0.4788];   // the drum's axle, which the hood turns about
export const PIVOT = [-0.0701, 0.1105];  // the floor's pivot: its powered back shaft
// the intake's slide: all the way out as the CAD has it (0.3061 m of rack from all the way in);
// going in it moves back and up along DIR (5.5 deg)
export const SLIDE = { out: 0.3061, dir: [-0.9954, 0.0960], pinion: 0.5 * IN, mass: 5.5 };
export const INTAKE = {
  // as exported (all the way out); r: radius over the grip tape
  big: { at: [0.5794, 0.1781], r: 0.0397, width: 0.5525 },   // 3in polycarbonate tube, grip tape
  small: { at: [0.5307, 0.2345], r: 0.0286, width: 0.5588 }, // 2in
};
// the floor's eleven 1in rollers, back (its powered shaft, on the pivot) to front, down (as
// exported)
export const FLOOR = {
  rollers: [[-0.0701, 0.1105], [-0.0300, 0.1220], [0.0102, 0.1335], [0.0503, 0.1450], [0.0904, 0.1564], [0.1306, 0.1679],
    [0.1707, 0.1794], [0.2123, 0.1823], [0.2540, 0.1852], [0.2938, 0.1975], [0.3237, 0.1684]],
  r: 0.5 * IN, width: 0.508,
  // its front's lift (deg, up) with the intake pulled in m from all the way out: its 2in followers
  // riding the intake's ramps (the CAD's ramp plate and follower; a section through them, the
  // floor resting down until a follower touches). It's down for the first 8 cm; their compaction
  // (5in) lifts it 13.5 deg, all the way in 30.6
  cam: [[0, 0], [0.083, 0], [0.089, 0.8], [0.101, 1.8], [0.110, 2.4], [0.119, 3.6], [0.129, 5.4], [0.141, 7.2], [0.150, 8.9],
    [0.159, 10.6], [0.171, 12.2], [0.181, 13.8], [0.190, 15.4], [0.199, 16.9], [0.211, 18.4], [0.220, 19.7], [0.230, 21.1],
    [0.239, 22.4], [0.251, 23.7], [0.260, 25.0], [0.269, 26.3], [0.279, 27.6], [0.291, 28.8], [0.3061, 30.6]],
  // the floor (two plates, eleven tubes and their shafts, belts): ~2.5 kg, its center ~0.2 m out
  // (estimates)
  mass: 2.5, com: 0.2,
};
// the tower's three 1.5in rollers, bottom to top (in front of the FUEL; the curved plate behind)
export const TOWER = { rollers: [[-0.1142, 0.2789], [-0.1027, 0.3454], [-0.0911, 0.4119]], r: 0.019, width: 0.553 };
export const SHOOTER = {
  drum: { r: 0.051, width: 0.553, z: -0.0105 },
  // the hood's 2in rollers as exported (their 11 deg, the hood's hard stop)
  hood: [[-0.0820, 0.4615], [-0.0865, 0.5245]], hoodR: 0.0254, hoodWidth: 0.552,
  hoodPerDrum: (18 / 15) * (30 / 22), flywheelPerDrum: 45 / 15,
  // inertia (kg m^2), each part about its own axis from the CAD's solids (OpenCascade's volume
  // properties on the STEP; their BOM carries no materials, so by name and appearance: the
  // tubes, shafts and WCP pulleys aluminum, the dark custom hubs and pulleys prints, PLA);
  // README, *581*, has the table. The drum: its 4in tube (0.74 kg), the hubs, the 45t and 18t
  // pulleys, the ThunderHex and the belts; the WCP-2414 steel inertia flywheel (0.44 kg) with
  // its 15t pulley and stub; the hood's two 2in rollers (tubes, hubs, ThunderHex, the 22t gear
  // and 20t pulleys) and the jackshaft that drives them, at the rollers' speed
  drumJ: 2.21e-3, flywheelJ: 7.88e-4, hoodJ: 5.11e-4,
  // their hood angles (ShooterHoodConfig): 11 is its hard stop (homing), where the CAD has it;
  // 13 to 45 in play
  hoodCad: 11, hoodMin: 13, hoodMax: 45, hoodIdle: 13,
  // how far the hood turns about the drum's axle (deg) from the CAD's pose at their angle (their
  // MechanismVisualizer turns it 1:1, angle - 11)
  hoodTurn: [[11, 0], [45, 34]],
  // their velocity loop (Slot0, torque current, motor rotations/s): kS + kV x target + kP x error,
  // amps, plus their feeder-current feedforward
  kS: 6.4, kV: 0.13, kP: 14,
  idle: 750, // their IDLE_RPM
  rpmTol: 50, rpmTolShooting: 300, rpmTolFeeding: 500,
};
const interp = (t, x) => {
  if (x <= t[0][0]) return t[0][1];
  for (let k = 1; k < t.length; k++) if (x <= t[k][0]) return t[k - 1][1] + ((t[k][1] - t[k - 1][1]) * (x - t[k - 1][0])) / (t[k][0] - t[k - 1][0]);
  return t[t.length - 1][1];
};
// their tables (ShooterConfig, ShooterHoodConfig; their regression flag is off, so these
// interpolate, clamped at the ends): distance from the robot's center to the target (m) -> drum RPM
// and hood angle (clamped to 13-45)
export const SCORE_RPM = [[1.42, 1350], [2.79, 1560], [3.46, 1610], [4.92, 1900]];
export const SCORE_HOOD = [[1.42, 12.5], [2.38, 20], [3.46, 29], [4.92, 33]];
export const FEED_RPM = [[6.0, 1500], [8.71, 2500], [13.6, 2500]];
export const FEED_HOOD = [[6.0, 35], [8.71, 40], [13.6, 40]];
// their feeder current (A, the two tower motors' average stator) -> extra shooter torque current
// (A): FEEDER_CURRENT_TO_SHOOTER_FEED_FORWARD; and the first moments of a shot: at least 20 A for
// 0.7 s with the hopper full, 15 A for 0.1 s otherwise
export const FEEDER_FF = [[0, 0], [20, 0], [30, 15], [50, 22]];
export const clampHood = (deg) => Math.max(SHOOTER.hoodMin, Math.min(SHOOTER.hoodMax, deg));
export function shotFor(kind, d) {
  const feed = kind === 'feed';
  return { rpm: Math.min(6000, interp(feed ? FEED_RPM : SCORE_RPM, d)), hood: clampHood(interp(feed ? FEED_HOOD : SCORE_HOOD, d)) };
}
export const feederFF = (amps) => interp(FEEDER_FF, amps);
// the deploy's positions (their DeployState, inches out from all the way in) and its config
export const DEPLOY = {
  INTAKE: 11.8, STOW: 5.0, SAFE_KICKER_STOW: 7.915, SCORE_COMPACTION_WAITING: 11.8 - 3.5, SCORE_COMPACTION: 1.0, FEED_COMPACTION: 6.915,
  BEAST_MODE_COMPACTION: 5.0, min: 5, max: 11.8, homeOut: 11.9,
  // their Slot0 (torque current per inch of error), Motion Magic (in/s, in/s^2), homing (4 V out)
  kP: 8, mmV: 200, mmA: 50, homeV: 4, tol: 0.25,
};
// their sensors: the hopper's CANrange (on the tower's front at the top right, looking forward
// along the hopper; 6.75 deg wide; their thresholds: 6in HIGH, 8in MEDIUM) and the tower's beam
// break (DIO 9; not in the CAD: across the tower between its low and mid rollers, an estimate)
export const RANGES = { hopper: { at: [-0.014, 0.418], z: 0.289, reach: 0.6, half: 3.4, high: 6 * IN, medium: 8 * IN }, tower: { at: [-0.19, 0.31] } };
// the preload, eight dropped into the hopper (robot frame; they settle on the floor)
export const PRELOAD = [
  [0.0, 0.33, -0.225], [0.0, 0.33, -0.075], [0.0, 0.33, 0.075], [0.0, 0.33, 0.225],
  [0.16, 0.42, -0.225], [0.16, 0.42, -0.075], [0.16, 0.42, 0.075], [0.16, 0.42, 0.225],
];

// ------------------------------------------------------------------ the drives
export function build581() {
  // the intake's rollers: positive turns them inward (their bottoms run rearward)
  const intake = new Train({ name: 'intake rollers', motor: new Motor('krakenX60', { count: 2, statorLimit: 75, supplyLimit: 25 }) });
  const small = intake.add(new Shaft({ name: 'intake 2in roller', inertia: 1.2e-4 }), -12 / 20);
  const big = intake.add(new Shaft({ name: 'intake 3in roller', inertia: 4.5e-4 }), -(12 / 20) * (20 / 30));
  // the floor: positive carries FUEL back along it (the rollers' tops run rearward)
  const floor = new Train({ name: 'conveyor (floor)', motor: new Motor('krakenX60', { count: 2, statorLimit: 50, supplyLimit: 10 }) });
  const rollers = FLOOR.rollers.map((_, i) => floor.add(new Shaft({ name: `floor roller ${i + 1}`, inertia: 2.5e-5 }), 12 / 18));
  // the tower: positive lifts FUEL (the rollers' backs run up)
  const feederT = new Train({ name: 'feeder (tower)', motor: new Motor('krakenX60', { count: 2, statorLimit: 50, supplyLimit: 10 }) });
  const tower = TOWER.rollers.map((_, i) => feederT.add(new Shaft({ name: `tower 1.5in roller ${i + 1}`, inertia: 9e-5 }), -12 / 18));
  // the shooter: positive runs the drum's front up, the hood rollers' backs up
  const shooter = new Train({ name: 'shooter', motor: new Motor('krakenX60', { count: 4, statorLimit: 150, supplyLimit: 50 }) });
  const S = SHOOTER;
  const drum = shooter.add(new Shaft({ name: 'drum', inertia: S.drumJ }), 1);
  shooter.add(new Shaft({ name: 'inertia flywheel', inertia: S.flywheelJ }), S.flywheelPerDrum);
  const hood = shooter.add(new Shaft({ name: 'hood rollers', inertia: S.hoodJ }), -S.hoodPerDrum);
  // the deploy: the racks' pinions (positive runs the intake out); what slides (the intake's
  // plates, rollers, two X60s, panels: ~5.5 kg, an estimate) seen at the pinion; the racks'
  // friction (an estimate: it damps their position loop, which has no kD)
  const deploy = new Shaft({ name: 'deploy pinions', motor: new Motor('krakenX44', { count: 2, ratio: 40 / 8, statorLimit: 50, supplyLimit: 18 }), inertia: SLIDE.mass * SLIDE.pinion ** 2, drag: 0.05 });
  return { intake, small, big, floor, rollers, feederT, tower, shooter, drum, hood, deploy };
}

// the wheels (robot frame, as exported; the intake's, floor's and hood's posed with place*)
export function wheels581(t) {
  const Z = [0, 0, 1];
  const floor = FLOOR.rollers.map(([x, y], i) => new Wheel({ name: t.rollers[i].name, shaft: t.rollers[i], center: [x, y, -0.003], axis: Z, radius: FLOOR.r, width: FLOOR.width, mu: MU.floor, tread: 40000 }));
  floor.forEach((w) => { w.home = w.center.slice(); });
  const tower = TOWER.rollers.map(([x, y], i) => new Wheel({ name: t.tower[i].name, shaft: t.tower[i], center: [x, y, -0.0025], axis: Z, radius: TOWER.r, width: TOWER.width, mu: MU.gripTape, tread: 40000 }));
  const I = INTAKE;
  const intake = [
    new Wheel({ name: 'intake 3in roller (grip tape)', shaft: t.big, center: [...I.big.at, -0.0025], axis: Z, radius: I.big.r, width: I.big.width, mu: MU.gripTape, tread: 40000 }),
    new Wheel({ name: 'intake 2in roller (grip tape)', shaft: t.small, center: [...I.small.at, -0.0025], axis: Z, radius: I.small.r, width: I.small.width, mu: MU.gripTape, tread: 40000 }),
  ];
  intake.forEach((w) => { w.home = w.center.slice(); });
  const S = SHOOTER;
  const drum = new Wheel({ name: 'drum (grip tape)', shaft: t.drum, center: [...DRUM, S.drum.z], axis: Z, radius: S.drum.r, width: S.drum.width, mu: MU.gripTape, tread: 40000 });
  const hood = S.hood.map((h, i) => new Wheel({ name: `hood 2in roller ${i + 1} (grip tape)`, shaft: t.hood, center: [...h, -0.001], axis: Z, radius: S.hoodR, width: S.hoodWidth, mu: MU.gripTape, tread: 40000 }));
  hood.forEach((w) => { w.home = w.center.slice(); });
  return { floor, tower, intake, drum, hood };
}
// rotate a point (x, y) about (cx, cy) by a (rad)
const rot = ([x, y], [cx, cy], a) => { const c = Math.cos(a), s = Math.sin(a), dx = x - cx, dy = y - cy; return [cx + dx * c - dy * s, cy + dx * s + dy * c]; };
// the intake's offset from the CAD's pose (all the way out) with it ext m out along its racks
export function slideOffset(ext) { const d = SLIDE.out - ext; return [SLIDE.dir[0] * d, SLIDE.dir[1] * d]; }
export function placeIntake(wheels, ext) {
  const [dx, dy] = slideOffset(ext);
  for (const w of wheels) w.center = [w.home[0] + dx, w.home[1] + dy, w.home[2]];
}
// the floor's lift (rad) with the intake ext m out, and how fast it lifts as the intake comes in
// (rad per m in)
export function floorRot(ext) { return (interp(FLOOR.cam, SLIDE.out - ext) * Math.PI) / 180; }
export function floorRate(ext) { return (floorRot(ext - 0.0025) - floorRot(ext + 0.0025)) / 0.005; }
export function placeFloor(wheels, ext) {
  const a = floorRot(ext);
  for (const w of wheels) { const [x, y] = rot(w.home, PIVOT, a); w.center = [x, y, w.home[2]]; }
}
// the hood at their angle deg: its turn about the drum's axle from the CAD's pose
export function hoodRot(deg) { return (interp(SHOOTER.hoodTurn, deg) * Math.PI) / 180; }
export function placeHood(wheels, deg) {
  const a = hoodRot(deg);
  for (const w of wheels) { const [x, y] = rot(w.home, DRUM, a); w.center = [x, y, w.home[2]]; }
}
// the shot's elevation at their hood angle deg, by the CAD's geometry: square to the line from the
// drum's axle to the top hood roller, out the back (77.2 deg at their 11, the CAD's pose)
export function hoodElevation(deg) {
  const [hx, hy] = SHOOTER.hood[1];
  return 90 - (Math.atan2(hy - DRUM[1], hx - DRUM[0]) * 180) / Math.PI - interp(SHOOTER.hoodTurn, deg);
}

// the ball path's surfaces from their CAD (js/mech/path581.js): the body; the intake on its racks
// (it slides, on the deploy's pinions); the floor on its pivot (it turns as the intake's ramps lift
// it: on the deploy too, through the cam); the hood on the drum's axle; and the net from the
// hopper's polycarbonate top down to the intake's front plate (NET below, placed with placeNet)
export function pathSurfaces581(mesh, t, { mu = 0.35 } = {}) {
  const s = (k, pos) => new Surface({ name: `${k} (CAD)`, vertices: mesh[k].v.map((x) => x / 1000), indices: mesh[k].i, mu, pos });
  const body = s('body', [0, 0, 0]);
  const slide = s('slide', [0, 0, 0]).onSlide(t.deploy, [-SLIDE.dir[0], -SLIDE.dir[1], 0]);
  slide.slideK = SLIDE.pinion;
  // (the floor turns as the deploy's pinions do through the cam: k radians of floor per radian of
  // pinion, set by placeFloorSurface)
  const cam = { k: 0, get omega() { return t.deploy.omega * this.k; }, J: Infinity, impulse(L) { t.deploy.impulse(L * this.k); } };
  const floor = s('floor', [PIVOT[0], PIVOT[1], 0]).onShaft(cam, [PIVOT[0], PIVOT[1], 0]);
  floor.cam = cam;
  const hood = s('hood', [DRUM[0], DRUM[1], 0]);
  return { body, slide, floor, hood, net: new Net581() };
}
export function placeFloorSurface(sf, ext) {
  sf.setPose([PIVOT[0], PIVOT[1], 0], 0, floorRot(ext));
  // (the pinion turning out lowers the floor: d(floor)/d(pinion) = -floorRate x pinion radius)
  sf.cam.k = -floorRate(ext) * SLIDE.pinion;
}
// The net. Their CAD release thread: a "trucker net added to the top of the hopper that spans each
// side, mounted to the top 1x1 tubes by zip-ties, extends to front intake plate"; the CAD has the
// polycarbonate top over the fixed part (0.55 m up, from the shooter tower's front at x -0.06 to
// its front edge at 0.302, the full width), so the net covers only what's left: from the top's front
// edge (topX, topY) down to the top of the intake's front plate (frontX, frontY with the intake all
// the way out; it rides out and in with it), across the hopper between the intake's side panels
// (hw: their inside faces, 0.34 m out; the panels' tops, 0.49-0.50 m, stand within 4 cm of the
// net's sides, under a FUEL's radius). Tied along its four edges.
// A trucker net is bungee cord, and it's tied on loose: the load lifts it freely (it weighs next to
// nothing) until its slack is taken up, as far as NET.bulge over the line between its ties at its
// middle, tapering to them (the shape a dome of cord takes: 1706's net, js/mech/r1706.js), and
// there it goes taut: a FUEL pressing on it then gives as far as the stretching bungee lets it
// (NET.tread, N/m). Nothing pushes the load back down below that. Both are estimates, sized to the
// reveal's photo of the Champs robot full (Chief Delphi, "581 Blazing Bulldogs 2026 CAD and Code
// Release", Rubble_2): its load standing ~4-5 cm over the side rails near the front, 0.59-0.60 m
// up. With 0.05 m of slack (about 2in of rise at its middle; as much hanging empty, as the net on
// their earlier robot hangs between its zip ties in the same thread) and 8 kN/m (bungee: softer
// than 1706's braided cord, 10 kN/m, stiffer than 4414's stretchy net, 1.5, which let the load 2-5
// cm past it), the load the intake crams in stands 0.58-0.60 m (tools/mechlab581.mjs intakefull);
// 0.04-0.08 m and 4-15 kN/m all hold the same 69-72 there, the slack only moves how tall it stands
export const NET = { topX: 0.302, topY: 0.55, frontX: 0.643, frontY: 0.49, hw: 0.34, bulge: 0.05, tread: 8000 };
const R = FUEL.radius;
// the net's front tie with the intake ext m out
export function netFront(ext) { const [dx, dy] = slideOffset(ext); return [NET.frontX + dx, NET.frontY + dy]; }
// the net's height at (x, z) with its slack all taken up, the intake ext m out; and its slope
// (d/dx, d/dz)
export function netAt(x, z, ext) {
  const [fx, fy] = netFront(ext), L = Math.max(0.01, fx - NET.topX), W = 2 * NET.hw, B = NET.bulge;
  const u = Math.min(1, Math.max(0, (x - NET.topX) / L)), v = Math.min(1, Math.max(0, (z + NET.hw) / W));
  const su = Math.sin(Math.PI * u), sv = Math.sin(Math.PI * v), slope = (fy - NET.topY) / L;
  return [NET.topY + slope * L * u + B * su * sv, slope + (B * Math.PI * Math.cos(Math.PI * u) * sv) / L, (B * su * Math.PI * Math.cos(Math.PI * v)) / W];
}
// (its cord: MU.net; it gives NET.tread once taut)
class Net581 extends Surface {
  constructor() {
    super({ name: 'net', vertices: [0, 0, 0, 0.01, 0, 0, 0, 0, 0.01], indices: [0, 1, 2], mu: MU.net, tread: NET.tread });
    this.place(SLIDE.out);
  }
  place(ext) { this.ext = ext; this.x1 = netFront(ext)[0]; }
  under(x, z) { return x > NET.topX && x < this.x1 && Math.abs(z) < NET.hw; }
  at(x, z) { return netAt(x, z, this.ext); }
  touches(p) {
    const [x, y, z] = p;
    if (!this.under(x, z)) return [];
    const [h, hx, hz] = this.at(x, z), l = Math.hypot(hx, 1, hz), dist = (h - y) / l;
    if (dist >= R || dist < -0.03) return [];
    return [{ depth: R - dist, n: [hx / l, -1 / l, hz / l] }];
  }
}
export function placeNet(net, ext) { net.place(ext); }
