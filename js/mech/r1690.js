// 1690's Kepler as physical mechanisms (js/mech/mechanism.js). Orbit's Onshape document ("FRC 1690
// 2026 TopLevel", d09a47dbfc1b125f63a78ac7, linked from their CAD release thread on Chief Delphi) is
// view-only: its export, assembly definition, mass properties, bounding boxes and tessellation are
// refused to the API, so nothing here is their geometry itself. What it gives: its BOM (the COTS parts:
// belts, pulleys, wheels, motors per subassembly) and Onshape's own scaled orthographic renders of
// each subassembly from any direction (shadedviews with a pixelSize: the image's middle is the view's
// origin, and the top level's origin is on the carpet under the robot's middle; cad/README.md). The
// layout below is measured off those renders (0.3-1 mm a pixel; good to a few mm), the speeds are
// what the team posted (their CAD release thread: the hopper floor at 75 RPS and the top and bottom
// belts at 95 RPS scoring, 85 / 100 passing, the vertical kicker at 60 RPS, the feeder wheels at 60%
// of the shooter wheel's speed, at least 5 m/s; the BehindTheBumpers walkthrough: the belts at 8 m/s);
// everything marked is an estimate. Their code isn't public ("software is our competitive edge").
// Robot frame: x forward (the intake end), y up, z to its right; the CAD's X is the robot's left, Y
// up, Z forward.
//
// The ball path:
//   - intake (subassembly 200): an arm on each side, pivoting low in the frame (0.172 m forward,
//     0.186 m up), carries two silicone-wrapped polycarbonate rollers at its end (an X60) and the
//     lattice U-frame that is the hopper's front, out over the intake. Under the rollers, an aluminum
//     tube on its own swing arm, which surgical tubing pulls down and which stays down ("it's just an
//     aluminum roller", passive): FUEL are pinched between it and the rollers and climb the
//     bumper's face into the hopper.
//   - the roller floor (300): nine polycarbonate rollers with silicone wraps (an X60, belted roller to
//     roller), sloping down 4 cm from front to back, carrying FUEL back to
//   - the belts along the back: two 30 mm HTD belts (the CAD's 180T and 160T 5M 30 mm), each round
//     two pulleys on fore-aft axles (an X44 each: "we prototyped them on one belt ... separate motors
//     make it easier to remove them"). The bottom belt's top run (0.167 m up) and the top belt's bottom
//     run (0.303 m) pinch FUEL between them (0.136 m apart: 14 mm of squeeze), against the backing
//     plate behind (polycarbonate under PTFE film), and carry them to the robot's left at 8 m/s.
//   - the vertical kicker: a polycarbonate roller standing up at the belts' left end, in front of
//     them where the diagonal deflector over the floor's left side ends; "it spins clockwise (toward
//     the belts)" seen from above, kicking what comes along the deflector into the belts.
//   - the feeder (the "S-curve" up to the turret): off the belts' end, the corridor turns up to the
//     left over three stacks of 2.25in compliant wheels on fore-aft shafts stepping up to the left
//     ("the polycarb ramp replaced by flex wheels", so the FUEL climb without spinning up) under a
//     guide that continues the top belt's run, and a top wheel at the corridor's end that turns them
//     up through the turret's 8in bearing (an X60).
//   - the turret (400): the FUEL comes up into it under the shooter wheel (a custom 4in aluminum
//     traction wheel with swerve tread on a dead axle, geared to two X60s, no added flywheel) and is
//     thrown up its front face and over it, against the hood (printed, pivoting on the wheel's axle,
//     an X44 through a VersaPlanetary 4:1 and 19.333:1 gears): it leaves along the hood's end, at the
//     hood's angle ("the difference between hood angle and ball exit angle was negligible").
import { IN, FUEL } from '../constants.js';
import { Block, Motor, Shaft, Surface, Train, Wheel, BUMPER_MU } from './mechanism.js';

const DEG = Math.PI / 180, R = FUEL.radius;
// what each grips FUEL with (estimates): silicone wraps on the intake and floor rollers (grippy), the
// compliant wheels (35A), the shooter wheel's swerve tread, the belts' rubber, bare polycarbonate
// and aluminum, the PTFE film on the backing plate and the corridor's walls (slick: "to reduce
// friction as much as possible"), the printed hood (PLA), the net's cord
export const MU = { silicone: 1.2, compliant: 1.3, tread: 1.3, belt: 1.0, polycarb: 0.5, alu: 0.4, ptfe: 0.1, pla: 0.3, wall: 0.25, net: 0.25 };
// and how far each gives where a FUEL presses it (N/m)
export const TREAD = { silicone: 20000, compliant: 12000, tread: 30000, belt: 3000, net: 10000 };

// ------------------------------------------------------------------ the layout (robot frame, m)
export const HALF_L = 12.5 * IN, HALF_W = 14.5 * IN; // 25 x 29in frame ("29 is the intake side", the team)
// the hopper's walls: the floor's side plates (the left one inside the deflector, the right one by
// the belts' pulleys), to the lattice frame's top
export const WALL = { left: -0.335, right: 0.315, top: 0.482 };
// the roller floor: [x, y] of each roller's axle (the floor's side plates), front to back; 1.25in
// polycarbonate with silicone wraps; the back three stop short of the left wall (the deflector and
// the corridor are there): z from each one's left end to the right wall
export const FLOOR = {
  r: 0.0175,
  at: [[0.282, 0.182], [0.232, 0.176], [0.186, 0.168], [0.142, 0.162], [0.098, 0.158], [0.054, 0.150], [0.006, 0.146], [-0.040, 0.146], [-0.088, 0.144]],
  z0: [-0.325, -0.325, -0.325, -0.325, -0.325, -0.325, -0.135, -0.07, -0.07], z1: 0.305,
  // an X60, belted roller to roller (60T 3M belts); the reduction isn't readable (16t motor pulleys,
  // 22t and 24t HTD pulleys in the BOM): 1.5:1, an estimate (5 m/s at their 75 RPS)
  ratio: 1.5,
};
// the belts along the back: each one's run against the FUEL (x across its 30 mm width, its face's
// height, z from end to end), the 22t HTD5 pulleys' pitch radius; X44s at 1.3:1 (their 95 RPS gives
// the 8 m/s of the BehindTheBumpers walkthrough)
export const BELT = {
  x: [-0.15, -0.115], r: 0.0175, ratio: 1.3,
  bottom: { y: 0.167, z: [-0.13, 0.25] }, // its top run (the 160T: 0.38 m end to end)
  top: { y: 0.303, z: [-0.09, 0.31] },     // its bottom run (the 180T: 0.43 m)
};
// the backing plate behind the belts (FUEL slide on it), the housing's top over the top belt
// (and the electronics box's face over the right half above the housing, with a shelf between: the CAD's box)
export const BACK = { x: -0.205, housing: -0.11, top: 0.34, box: -0.045, shelf: 0.375 };
// the deflector over the floor's left side (a diagonal wall from the left wall's front to the
// vertical kicker) and the cover from it to the left wall
export const DEFLECT = { from: [0.19, -0.335], to: [0.03, -0.15], y: [0.17, 0.342] };
// the vertical kicker: a 1.25in polycarbonate roller standing in front of the belts' left end (an X44,
// 1:1 assumed: 6 m/s at their 60 RPS)
export const VKICK = { at: [-0.013, -0.155], y: [0.2, 0.37], r: 0.016, ratio: 1 };
// the feeder: the corridor (PTFE walls front and back), its three 2.25in compliant wheel stacks (two
// 1in wheels each, on fore-aft shafts), stepping up to the left, and the top wheel at the end ([z, y]);
// an X60 at 1.25:1 (estimate)
export const FEED = {
  x: -0.134, w: 0.05, front: -0.054, back: -0.205,
  wheels: [[-0.184, 0.162], [-0.238, 0.204], [-0.272, 0.266]], r: 0.0286,
  top: { at: [-0.284, 0.334], r: 0.025 },
  ratio: 1.25,
};
// the guide over the feeder's wheels: on from the top belt's run, rising along the wheels' line,
// 2R + r - 15 mm off it (the wheels' squeeze), to where the turret's bore opens
const LINE = (() => {
  const [a, c] = [FEED.wheels[0], FEED.wheels[2]], d = [c[0] - a[0], c[1] - a[1]], l = Math.hypot(...d);
  return { a, u: [d[0] / l, d[1] / l], n: [d[1] / l, -d[0] / l].map((v) => -v) };
})();
export const GUIDE = { off: FEED.r + 2 * R - 0.015 };
// the turret: its axis, its 8in bearing's bore (fixed), the deck it turns on; its travel (+-200 deg,
// as the sim's config had it: their code isn't public)
export const TURRET = { at: [-0.14, -0.20], bore: { r: 0.1016, y: [0.33, 0.40] }, deck: 0.375, guardR: 0.19, guardY: [0.375, 0.51] };
// the shooter, in the turret's frame (s along the shot, y up, z across): the 4in wheel's dead axle,
// where FUEL come up into it (on the turret's axis), the side plates; the hood: an arc about the
// axle, 2R + r - 22 mm out (the squeeze), `span` deg long, its end where the shot leaves: the hood's
// angle (roll) is 90 deg + the shot's elevation by the geometry; its travel (elevations 35-78 deg,
// estimates); the gearing (the wheel geared straight to the X60s: 1:1, an estimate; recovery
// "around 50 ms at top speed" with no added flywheel), what turns (the custom wheel and its gears:
// 6e-4 kg m^2, estimate)
export const SHOOTER = {
  axle: [0.11, 0.525], r: 2 * IN, w: 0.05, entry: [0, 0.454], plateZ: 0.082,
  hoodR: 2 * IN + 2 * R - 0.022, span: 40, elMin: 35, elMax: 78, ratio: 1, J: 6e-4,
  // the hood's X44: 4:1 VersaPlanetary and 19.333:1 gears (their thread); its profile (estimates)
  hoodRatio: 4 * 19.333, hoodV: 350, hoodA: 3500,
  // the lead-in under the hood (fixed on the turret): the same arc, from below the entry to the hood's
  // lowest reach
  lead: [-20, 5],
  // the wheel's velocity loop's gain; ready: within 3% of the wheel's target surface speed, 1.5 deg
  // of hood (estimates)
  kP: 0.25, tolV: 0.03, tolHood: 1.5,
};
// the intake arm, deployed (the CAD's pose as exported, "Floating"): the pivot, the two rollers at
// its end (the bottom one 2.5 cm in from the U-frame's front panel), the aluminum tube under them on
// its own arm (deployed by surgical tubing, it stays down), the U-frame (front panel and sides); how
// far up it folds in "travel" mode (an estimate); the rollers' X60 at 2:1 (estimate)
export const INTAKE = {
  pivot: [0.172, 0.186], width: 0.66,
  rollers: [[0.558, 0.165, 0.025], [0.558, 0.226, 0.025]], tube: [0.436, 0.048, 0.0127],
  frame: { front: 0.60, sides: 0.355, x0: 0.27, top: 0.482, bottom: [0.298, 0.223] },
  stow: 70, ratio: 2,
};
export const PRELOAD = [0, 1, 2, 3, 4, 5, 6, 7].map((k) => [0.21 - 0.155 * Math.floor(k / 4), 0.36, -0.24 + 0.155 * (k % 4)]);

// ------------------------------------------------------------------ the drives
// (current limits aren't public: 80 A stator, 40 A supply, estimates; the shooter's 30 A supply, as
// its channels' 30 A breakers: "we used 30A breakers for the channels with the 14awg")
export function build1690() {
  const lim = { statorLimit: 80, supplyLimit: 40 };
  const intake = new Train({ name: 'intake rollers', motor: new Motor('krakenX60', { ratio: INTAKE.ratio, ...lim }) });
  const rollers = INTAKE.rollers.map((_, i) => intake.add(new Shaft({ name: `intake roller ${i ? 'top' : 'bottom'}`, inertia: 1.5e-4 }), -1));
  const tube = new Shaft({ name: 'intake tube (free)', inertia: 4e-5, drag: 2e-4 });
  const floor = new Train({ name: 'roller floor', motor: new Motor('krakenX60', { ratio: FLOOR.ratio, ...lim }) });
  FLOOR.at.forEach((_, i) => floor.add(new Shaft({ name: `floor roller ${i + 1}`, inertia: 1.2e-4 }), 1));
  const belts = ['bottom', 'top'].map((k) => new Shaft({ name: `${k} belt`, motor: new Motor('krakenX44', { ratio: BELT.ratio, ...lim }), inertia: 5e-5, drag: 4e-4 }));
  const vkick = new Shaft({ name: 'vertical kicker', motor: new Motor('krakenX44', { ratio: VKICK.ratio, ...lim }), inertia: 6e-5 });
  const feed = new Train({ name: 'feeder', motor: new Motor('krakenX60', { ratio: FEED.ratio, ...lim }) });
  const feedShafts = [...FEED.wheels, FEED.top.at].map((_, i) => feed.add(new Shaft({ name: i < 3 ? `feeder wheels ${i + 1}` : 'feeder top wheel', inertia: 6e-5 }), 1));
  const shooter = new Train({ name: 'shooter', motor: new Motor('krakenX60', { count: 2, ratio: SHOOTER.ratio, statorLimit: 80, supplyLimit: 30 }) });
  const wheel = shooter.add(new Shaft({ name: '4in shooter wheel', inertia: SHOOTER.J, drag: 2e-4 }), 1);
  return { intake, rollers, tube, floor, belts, vkick, feed, feedShafts, shooter, wheel };
}

const X = [1, 0, 0], Z = [0, 0, 1];
// the wheels of the ball path (robot frame; the intake's posed by placeIntake)
export function wheels1690(t) {
  const intake = [
    ...INTAKE.rollers.map(([x, y, r], i) => new Wheel({ name: `intake roller ${i ? 'top' : 'bottom'} (silicone)`, shaft: t.rollers[i], center: [x, y, 0], axis: Z, radius: r, width: INTAKE.width, mu: MU.silicone, tread: TREAD.silicone })),
    new Wheel({ name: 'intake tube (aluminum, free)', shaft: t.tube, center: [INTAKE.tube[0], INTAKE.tube[1], 0], axis: Z, radius: INTAKE.tube[2], width: INTAKE.width, mu: MU.alu }),
  ];
  intake.forEach((w) => { w.home = w.center.slice(); });
  // (the floor's top runs back: +z axes; the intake's bottom roller pulls FUEL in under it, its top
  // one over it: their faces toward the bumper running up)
  const floor = FLOOR.at.map(([x, y], i) => new Wheel({ name: `floor roller ${i + 1} (silicone)`, shaft: t.floor.shafts[i], center: [x, y, (FLOOR.z0[i] + FLOOR.z1) / 2], axis: Z, radius: FLOOR.r, width: FLOOR.z1 - FLOOR.z0[i], mu: MU.silicone, tread: TREAD.silicone }));
  // (clockwise from above: a -y axis)
  const vkick = new Wheel({ name: 'vertical kicker (polycarbonate)', shaft: t.vkick, center: [VKICK.at[0], (VKICK.y[0] + VKICK.y[1]) / 2, VKICK.at[1]], axis: [0, -1, 0], radius: VKICK.r, width: VKICK.y[1] - VKICK.y[0], mu: MU.polycarb, tread: 30000 });
  // (their faces toward the FUEL running up and to the left: -x axes)
  const feed = [
    ...FEED.wheels.map(([z, y], i) => new Wheel({ name: `feeder 2.25in compliant wheels ${i + 1}`, shaft: t.feedShafts[i], center: [FEED.x, y, z], axis: [-1, 0, 0], radius: FEED.r, width: FEED.w, mu: MU.compliant, tread: TREAD.compliant })),
    new Wheel({ name: 'feeder top wheel', shaft: t.feedShafts[3], center: [FEED.x, FEED.top.at[1], FEED.top.at[0]], axis: [-1, 0, 0], radius: FEED.top.r, width: FEED.w, mu: MU.compliant, tread: TREAD.compliant }),
  ];
  return { intake, floor, vkick, feed };
}
// the shooter wheel (posed by placeShooter)
export function shooterWheel(t) {
  return new Wheel({ name: '4in shooter wheel (swerve tread)', shaft: t.wheel, center: [0, 0, 0], axis: Z, radius: SHOOTER.r, width: SHOOTER.w, mu: MU.tread, tread: TREAD.tread });
}
// a turret-frame point (s along the shot, y up, z across) where the turret is at yaw (rad, three.js
// sense: + turns it to the left)
export function turretPoint([s, y, z], yaw) {
  const c = Math.cos(yaw), sn = Math.sin(yaw), [tx, tz] = TURRET.at;
  return [tx + s * c + z * sn, y, tz - s * sn + z * c];
}
// the shooter wheel where the turret's yaw puts it (its face toward the FUEL coming up runs up)
export function placeShooter(wheel, yaw) {
  wheel.center = turretPoint([SHOOTER.axle[0], SHOOTER.axle[1], 0], yaw);
  wheel.axis = [-Math.sin(yaw), 0, -Math.cos(yaw)];
}
// the shot's elevation (deg) by the hood's geometry for its angle (deg about the axle, from the shot's
// direction: 90 + the elevation), and back
export const hoodElevation = (deg) => deg - 90;
export const hoodFor = (el) => el + 90;

// the front bumper: solid foam (3.25in with its backing), 1.25in to 5in up, across the front and
// round to the side bumpers' faces; soft (20 kN/m, as the others')
export function bumper1690() {
  return new Block({ name: 'bumper', x0: HALF_L + 0.75 * IN, x1: HALF_L + 3.25 * IN, y0: 0.032, y1: 0.16, hw: HALF_W + 3.25 * IN, mu: BUMPER_MU, tread: 20000 });
}

// ------------------------------------------------------------------ the surfaces (built here: no CAD export)
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
// a vertical sheet along a polyline [[x, z], ...] from y0 to y1
const wall = (pts, y0, y1) => merge(...pts.slice(1).map((p, k) => {
  const q = pts[k], L = Math.hypot(p[0] - q[0], p[1] - q[1]), nu = Math.max(1, Math.ceil(L / 0.06)), nv = Math.max(1, Math.ceil(Math.abs(y1 - y0) / 0.08));
  return grid(nu, nv, (u, v) => [lerp(q[0], p[0], u), lerp(y0, y1, v), lerp(q[1], p[1], u)]);
}));
// a level sheet over x0..x1, z0..z1 at y (or y(x, z))
const level = (x0, x1, z0, z1, y) => grid(Math.max(1, Math.ceil(Math.abs(x1 - x0) / 0.06)), Math.max(1, Math.ceil(Math.abs(z1 - z0) / 0.06)), (u, v) => { const x = lerp(x0, x1, u), z = lerp(z0, z1, v); return [x, typeof y === 'function' ? y(x, z) : y, z]; });
// a vertical cylinder's wall round (cx, cz), radius r, y0..y1, angles a0..a1 (from +x toward +z)
const cyl = (cx, cz, r, y0, y1, a0 = 0, a1 = 2 * Math.PI) => grid(Math.max(4, Math.ceil(((a1 - a0) * r) / 0.03)), 2, (u, v) => { const a = lerp(a0, a1, u); return [cx + r * Math.cos(a), lerp(y0, y1, v), cz + r * Math.sin(a)]; });
// a flat ring round (cx, cz) at y, r0..r1
const ring = (cx, cz, r0, r1, y) => grid(48, 2, (u, v) => { const a = u * 2 * Math.PI, r = lerp(r0, r1, v); return [cx + r * Math.cos(a), y, cz + r * Math.sin(a)]; });

// what doesn't move: the floor's side walls, the frame under the floor, the backing plate and the
// belts' housing, the deflector and its cover, the feeder's corridor (its front and back walls, the
// guide over its wheels, the floor under them), the turret's bearing bore and the deck round it, the
// guard round the turret's front (the energy chain's track) and the electronics box's face
function body() {
  const { left, right, top } = WALL, B = BELT, [tx, tz] = TURRET.at;
  // the guide over the feeder: from the top belt's left end, along the wheels' line, to the bore
  const g = (s) => [LINE.a[0] + LINE.u[0] * s + LINE.n[0] * GUIDE.off, LINE.a[1] + LINE.u[1] * s + LINE.n[1] * GUIDE.off];
  const sAt = (zz) => (zz - LINE.a[0] - LINE.n[0] * GUIDE.off) / LINE.u[0];
  const s0 = sAt(B.top.z[0]), s1 = sAt(tz + TURRET.bore.r);
  const guide = grid(4, 3, (u, v) => { const [zz, yy] = g(lerp(s0, s1, u)); return [lerp(FEED.back, FEED.front, v), yy, zz]; });
  const arc = [];
  for (let k = 0; k <= 12; k++) { const b = (135 * DEG * k) / 12; arc.push([tx + TURRET.guardR * Math.sin(b), tz + TURRET.guardR * Math.cos(b)]); }
  return merge(
    // the side walls
    wall([[BACK.x - 0.01, left], [HALF_L, left]], 0.10, top), wall([[BACK.x - 0.01, right], [HALF_L, right]], 0.10, top),
    // under the floor (the frame), the belts' housing under the bottom belt's run (beside it, 2.7 cm under
    // its top: the belt stands up out of it, what the FUEL ride on)
    level(-0.12, HALF_L, left, right, 0.11), level(BACK.x, -0.10, BELT.bottom.z[1], right, 0.150), level(BACK.x, B.x[0], B.bottom.z[0], right, 0.140), level(B.x[1], -0.10, B.bottom.z[0], right, 0.140),
    // the housing's front over the top belt, its top (the backing plate: PTFE, below)
    wall([[BACK.housing, B.top.z[0]], [BACK.housing, right]], B.top.y + 0.035, BACK.shelf), level(BACK.x, BACK.housing, B.top.z[0], right, B.top.y + 0.035), level(BACK.housing, BACK.box, tz + TURRET.guardR, right, BACK.shelf),
    // the deflector and its cover
    wall([DEFLECT.from, DEFLECT.to], DEFLECT.y[0], DEFLECT.y[1]), level(FEED.front, DEFLECT.to[0], left, VKICK.at[1] - VKICK.r, DEFLECT.y[1]), grid(4, 4, (u, v) => { const x = lerp(DEFLECT.to[0], DEFLECT.from[0], u), zd = lerp(DEFLECT.to[1], DEFLECT.from[1], u); return [x, DEFLECT.y[1], lerp(zd, left, v)]; }),
    // the corridor: a low wall under the deflector's end (its PTFE front wall and the backing plate
    // behind it are the `ptfe` surface, below), the guide over its wheels, the floor under them
    wall([[FEED.front, VKICK.at[1] - 0.02], [DEFLECT.to[0], DEFLECT.to[1]]], 0.11, 0.17),
    guide, level(FEED.back, FEED.front, left, B.bottom.z[0], 0.11),
    // the turret's bore (its bearing) and the deck round it, the guard round its front, the box's face
    cyl(tx, tz, TURRET.bore.r, TURRET.bore.y[0], TURRET.bore.y[1]), ring(tx, tz, TURRET.bore.r, TURRET.guardR, TURRET.deck),
    wall(arc, TURRET.guardY[0], TURRET.guardY[1]), wall([[tx, tz + TURRET.guardR], [BACK.box, tz + TURRET.guardR], [BACK.box, right]], BACK.shelf, TURRET.guardY[1]),
  );
}
// the belts' runs (strips 30 mm wide, the FUEL's side; they slide along -z: the belt's surface moving
// toward the robot's left), with their ends rounded over the pulleys
function beltRun(y, z0, z1, up) {
  const [x0, x1] = BELT.x, r = BELT.r, s = up ? 1 : -1;
  // (round each end's pulley, a quarter turn away from the FUEL's side)
  const end = (zc, k) => grid(2, 4, (u, v) => { const a = (Math.PI / 2) * v; return [lerp(x0, x1, u), y - s * r + s * r * Math.cos(a), zc + k * r * Math.sin(a)]; });
  return merge(level(x0, x1, z0, z1, y), end(z0, -1), end(z1, 1));
}
// the intake arm (it turns about its pivot: posed with placeIntake): the lattice U-frame (the
// hopper's front, out over the intake) and the arms' plates down the sides, in the arm's frame (the
// pivot at its origin, as deployed)
function intakeArm() {
  const F = INTAKE.frame, [px, py] = INTAKE.pivot, s = F.sides, rel = ([x, y, z]) => [x - px, y - py, z];
  return merge(
    grid(2, 12, (u, v) => rel([F.front, lerp(F.bottom[1], F.top, u), lerp(-s, s, v)])),
    ...[-1, 1].map((k) => grid(6, 4, (u, v) => { const x = lerp(F.x0, F.front, u); return rel([x, lerp(lerp(F.bottom[0], F.bottom[1], u), F.top, v), k * s]); })),
    ...[-1, 1].map((k) => grid(6, 2, (u, v) => { const x = lerp(px, F.front, u); return rel([x, lerp(py - 0.03, lerp(F.bottom[0], F.bottom[1], (x - F.x0) / (F.front - F.x0)) + 0.01, v), k * (s + 0.004)]); })),
  );
}
// the turret (it turns: posed with placeTurret), in its frame (s along the shot, y up, z across): its
// side plates and the lead-in under the hood (the hood's arc, below its lowest reach)
function turret() {
  const S = SHOOTER, [ax, ay] = S.axle, z = S.plateZ;
  const arcAt = (phi, rr = S.hoodR) => [ax - rr * Math.cos(phi * DEG), ay + rr * Math.sin(phi * DEG)];
  return merge(
    ...[-1, 1].map((k) => grid(8, 6, (u, v) => [lerp(-0.12, 0.20, u), lerp(TURRET.deck + 0.005, 0.56, v), k * z])),
    grid(6, 4, (u, v) => { const [s, y] = arcAt(lerp(S.lead[0], S.lead[1], u)); return [s, y, lerp(-z, z, v)]; }),
    // (and from its low end down and out in front of the bore, then straight down to the deck: the FUEL
    // come up behind it into the pinch)
    ...(() => { const [s0, y0] = arcAt(S.lead[0]), sw = -(R + 0.007), yw = y0 - 0.035; return [
      grid(2, 4, (u, v) => [lerp(s0, sw, u), lerp(y0, yw, u), lerp(-z, z, v)]),
      grid(2, 4, (u, v) => [sw, lerp(yw, TURRET.deck + 0.005, u), lerp(-z, z, v)]),
    ]; })(),
    // (round behind the wheel, so a FUEL thrown short doesn't fall down its back: a guard)
    grid(6, 4, (u, v) => { const [s, y] = arcAt(lerp(-60, 60, u) + 180, S.r + 0.012); return [s, y, lerp(-z, z, v)]; }),
  );
}
// the hood (it turns about the wheel's axle: posed with placeHood): its arc, in its own frame, the
// end the shot leaves at 0 rad, back toward the front
function hood() {
  const S = SHOOTER;
  return grid(10, 4, (u, v) => { const a = lerp(0, S.span, u) * DEG; return [S.hoodR * Math.cos(a), S.hoodR * Math.sin(a), lerp(-S.plateZ, S.plateZ, v)]; });
}

export function pathSurfaces1690(t) {
  const s = (name, m, o = {}) => new Surface({ name, vertices: m.v, indices: m.i, mu: MU.wall, ...o });
  const bodyS = s('hopper, belts housing, deflector, corridor, bore (built from the renders)', body());
  // (the PTFE: the backing plate and the corridor's walls, on their own so they're slick)
  const ptfe = s('backing plate and corridor walls (PTFE)', merge(wall([[BACK.x, WALL.left], [BACK.x, WALL.right]], 0.12, BELT.top.y + 0.04), wall([[FEED.front, VKICK.at[1] - VKICK.r - 0.005], [FEED.front, WALL.left]], 0.11, TURRET.deck)), { mu: MU.ptfe });
  // the belts: their runs slide along -z as their shafts turn (BELT.r m a radian)
  const belt = (k, m) => { const sf = s(`${k} belt`, m, { mu: MU.belt, tread: TREAD.belt }).onSlide(t.belts[k === 'bottom' ? 0 : 1], [0, 0, -1]); sf.slideK = BELT.r; return sf; };
  const bottom = belt('bottom', beltRun(BELT.bottom.y, ...BELT.bottom.z, true)), top = belt('top', beltRun(BELT.top.y, ...BELT.top.z, false));
  const intake = s('intake U-frame and arms', intakeArm(), { pos: [...INTAKE.pivot, 0] });
  const turretS = s('turret plates and lead-in', turret(), { mu: MU.pla, pos: [TURRET.at[0], 0, TURRET.at[1]] });
  const hoodS = s('hood (PLA)', hood(), { mu: MU.pla });
  return { body: bodyS, ptfe, bottom, top, intake, turret: turretS, hood: hoodS, bumper: bumper1690(), net: new Net1690() };
}
// the intake arm turned up by deg from deployed: its surfaces and its wheels
export function placeIntake(sf, wheels, deg) {
  const a = deg * DEG, [px, py] = INTAKE.pivot, c = Math.cos(a), sn = Math.sin(a);
  sf.setPose([px, py, 0], 0, a);
  for (const w of wheels) { const [x, y, z] = w.home, dx = x - px, dy = y - py; w.center = [px + dx * c - dy * sn, py + dx * sn + dy * c, z]; }
}
export function placeTurret(sf, wheel, yaw, hoodDeg) {
  sf.turret.setPose([TURRET.at[0], 0, TURRET.at[1]], yaw, 0);
  sf.hood.setPose(turretPoint([SHOOTER.axle[0], SHOOTER.axle[1], 0], yaw), yaw, hoodDeg * DEG);
  placeShooter(wheel, yaw);
}

// The net over the open top (the team: "the balls tend to bounce up and out ... we'll do some tests
// with netting, but it could cause some issues with trench crossing"; their CAD has none): slack cord
// tied along the walls' tops, the guard round the turret and the box's face at the back, and the
// U-frame's top edges out front (it swings with the intake). The load lifts it freely as far as
// NET.bulge over its ties in its middle (the shape a dome of cord takes, as 1706's), and there it goes
// taut (TREAD.net). NET.bulge, 0.08 m: as much as keeps a full load under a TRENCH arm's 22.25in
// (0.565 m), which they go under (an estimate). In the robot's frame: x0, the back tie; x1, the front
// tie (the U-frame's front, deployed); hw, half its width
export const NET = { x0: BACK.box, x1: INTAKE.frame.front, y: WALL.top, hw: 0.345, bulge: 0.08, hang: { sag: 0.06, sagOut: 0.06 } };
export function netAt(x, z, x1 = NET.x1) {
  const L = x1 - NET.x0, W = 2 * NET.hw, B = NET.bulge;
  const u = Math.min(1, Math.max(0, (x - NET.x0) / L)), v = Math.min(1, Math.max(0, (z + NET.hw) / W));
  const su = Math.sin(Math.PI * u), sv = Math.sin(Math.PI * v);
  return [NET.y + B * su * sv, (B * Math.PI * Math.cos(Math.PI * u) * sv) / L, (B * su * Math.PI * Math.cos(Math.PI * v)) / W];
}
export const netY = (x, z, x1) => netAt(x, z, x1)[0];
class Net1690 extends Surface {
  constructor() {
    super({ name: 'net', vertices: [0, 0, 0, 0.01, 0, 0, 0, 0, 0.01], indices: [0, 1, 2], mu: MU.net, tread: TREAD.net });
    this.x1 = NET.x1;
  }
  // (the U-frame's front, as the intake's angle puts it)
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
