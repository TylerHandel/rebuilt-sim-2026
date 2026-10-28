// 1706's Mirage as physical mechanisms (js/mech/mechanism.js), from their CAD (Onshape "Mirage Public
// Release", 23ba2ed5b3893ab97929c8b6, through a STEP translation; its mates give the joints and their
// limits, its BOM the materials) and their CAD release thread on Chief Delphi ("1706 Ratchet Rockers
// 2026 CAD Release - Mirage (Champs Version)": the shooters' front and rear wheels at different
// speeds, the sloped plate out of the intake, the bumpers' foam, about 12-13 BPS). Their 2026 code
// isn't public (github.com/rr1706 has their 2025 code, konshu, whose motor conventions fill in what
// the CAD can't say: marked). Robot frame: x forward (the intake end; the team calls the turret end
// the front), y up, z to its right; the CAD's x back, y to the right, z up.
//
// The ball path, two streams from one intake: the intake's arm (on a hex shaft through its plates,
// 0.262 m forward, 0.168 m up) folds up inside the hopper and swings 113.4 deg out and down over the
// bumper (its mate's limit; it pushes into the bumper's foam a little there, as the team says). Down,
// a full-width shaft of 28 1in urethane wheels sits at the bumper's face, its front running up: it
// lifts FUEL off the carpet against the face, and the two 2in polycarbonate rollers above and in
// front of it (their backs running up) carry them up the face and over the bumper, and the top one
// throws them back up the sloped plate into the hopper. Swinging out, the arm pushes the hopper box
// (polycarbonate sides and front, no floor) 0.30 m out ahead of it on its slider (0.3048 m), and
// folding in it brings it back, sweeping what's out over it back in. Under the hopper, two
// spindexers side by side: a 10in polypropylene disc in a polycarbonate bowl (open at the back), a
// printed hub up its shaft. The disc carries the FUEL on it round to the bowl's back, where it goes
// off onto a curved polycarbonate ramp that rises under that side's turret, into the pinch of the
// kicker: two pairs of 2in urethane wheels either side of the turret's throat, counter-rotating,
// which push it up through the turret bearing into the shooter. In the turret, the FUEL is caught
// between the 3in urethane "accelerator" wheels (rear, the team's word) and two shafts of 1in
// urethane wheels on the hood (geared 2:1 off the accelerators, so their faces run at 2/3 of the
// accelerators': "we run the front wheel different speeds from the rear accelerator wheels"), and
// flung out over the accelerators. The hood's plates swing about the accelerators' axle: 45 deg
// (their mates' limits), from a shot 88 deg up (as exported) to 43. A 5in aluminum flywheel rides
// beside the path on the same belt as the accelerators.
// Drives (the CAD's gearing; current limits and loops, their 2025 code's conventions: marked):
//   intake rollers: two Kraken X60s on the arm, 10t:14t #25 chain each to the top roller; 14t:24t
//     chain to the bottom roller; its 18t gear through a 16t (reversing) and 24t:20t HTD belt to the
//     1in wheels' shaft
//   intake arm: a Kraken X60 with a MAXPlanetary (its stages aren't in the CAD: 16:1, an estimate)
//     and 22t:32t #25 chain onto the arm's plate sprockets
//   spindexers: a Kraken X44 each, 10t:72t #25 chain onto the disc's sprocket (WCP-0972: 72t)
//   kickers: a Kraken X44 each, 16t:24t onto the back stack's shaft and a 24t:24t reversing pair
//     whose other shaft belts 21t:21t to the front stack
//   shooters: a Kraken X60 each, 18t:18t then 22t:24t HTD (one 100T belt) to the 5in flywheel and to
//     a shaft whose 28t gear turns the accelerators' 28t the other way; the accelerators' other 28t
//     turns a 28t on the hood (riding round it as the hood swings) whose 30t belts to a 15t on the
//     upper 1in shaft (83T belt), the lower one off it through an 18t:26t:18t idler
//   hood: a Kraken X44 (8t onto a 34t, then belts; the rest isn't modeled: a profile, an estimate)
//   turrets: a Kraken X44 each through a printed 8DP ring gear (their thread; the aim follows the
//     robot's turret model)
import { IN } from '../constants.js';
import { Block, Motor, Shaft, Surface, Train, Wheel, BUMPER_MU } from './mechanism.js';

const DEG = Math.PI / 180;
// what each grips FUEL with (estimates): the solid 45A urethane wheels (intake's 1in, the kickers'
// 2in, the shooters' 3in and 1in: grippy and fairly firm), the bare polycarbonate rollers (their BOM:
// no tubing or tape in the CAD), the polypropylene disc (smooth), the printed hub (PLA)
export const MU = { urethane: 1.4, polycarb: 0.6, disc: 0.45, pla: 0.3 };
export const TREAD = { urethane: 25000, polycarb: 30000, bowl: Infinity };

// ------------------------------------------------------------------ the layout (robot frame)
export const HALF_L = 12.6 * IN, HALF_W = 14.75 * IN; // 25.2 x 29.5in frame (their CAD)
export const PIVOT = [0.262, 0.168]; // the intake arm's: the hex shaft through its plates (its mate)
export const INTAKE = {
  // folded up inside the hopper as exported (the CAD's pose, its mate's 0): the 2in polycarbonate
  // rollers (the top one, the bottom one: deployed, the bottom one is the front and the top one
  // over the bumper), the shaft of 28 1in urethane wheels nearest the pivot (deployed, at the
  // bumper's face), across what they span
  top: { at: [0.0715, 0.3805], r: 0.0254, z: [-0.327, 0.336] },
  bottom: { at: [0.1500, 0.4485], r: 0.0254, z: [-0.334, 0.314] },
  wheels: { at: [0.2955, 0.3440], r: 0.0127, z: [-0.351, 0.335] },
  // its swing (deg, out and down from folded): its mate's 0 to 113.4 (1.98 rad); deployed at the
  // limit (its 1in wheels 4 mm into the bumper's foam: "the intake clips through the bumper slightly
  // so in real life it just pushes itself into the foam", the team)
  out: 113.4, max: 113.4,
  // within 20 deg of out the rollers run (an estimate)
  tol: 20,
  // Motion Magic (estimates: their 2025 arm's MotionMagicVoltage, the speeds here): cruise 9 rad/s,
  // 60 rad/s^2; a position loop of kP 2 V per motor rotation, kV 0.12 V per motor rotation/s
  mmV: 9, mmA: 60, kP: 2, kV: 0.12,
  // the arm (its 3/8in plates, two rollers, the bash bar, the 1in wheels' shaft and the two X60s on
  // it: ~4.5 kg, its center ~0.22 m from the pivot toward the motors; estimates from the CAD's parts)
  mass: 4.5, com: [-0.12, 0.185], J: 0.25,
};
// the hopper box: as far out (m) as the arm keeps it at each degree, 0 to 113 (tools/path-mesh-1706.py
// from the CAD's parts: the arm's furthest-forward part between the box's walls), on its slider
// (0.3048 m: its mate's limit); its front as exported (in)
export const BOX = { out: 0.3048, front: 0.318, side: 0.374, bottom: 0.218, top: 0.548 };
// the spindexers (the right one, z < 0, then the left): their axes, the disc's top (10in
// polypropylene, r 0.127)
export const SPIN = { axes: [[0.0889, -0.1778], [0.0889, 0.1778]], top: 0.1611, r: 0.127, ratio: 72 / 10, J: 2.2e-3 };
// the turrets' axes (their 8in bearings') and the kickers under them: two stacks of two 2in
// urethane wheels either side of the throat, 0.30 m up (the right turret's; the left mirrors it)
export const TURRET = [[-0.1651, -0.2220], [-0.1651, 0.2220]];
export const KICKER = { front: [-0.0718, 0.30, -0.2502], back: [-0.2583, 0.30, -0.1943], axis: [-0.2973, 0, -0.9548], r: 0.0254, w: 0.0508, ratio: 24 / 16 };
// each turret's shooter, in its own frame (origin on the turret's axis, x along the shot, y up): the
// two 3in accelerator wheels, the hood's two shafts of two 1in wheels (as exported, the hood at its
// steep end), the hood's axle (the accelerators')
export const SHOOTER = {
  accel: { at: [0.1053, 0.4607], r: 0.0381, w: 0.0508 },
  hood: [{ at: [-0.0770, 0.4928], r: 0.0127, w: 0.0508 }, { at: [-0.0787, 0.4404], r: 0.0127, w: 0.0508 }],
  // the gearing: the accelerators 22/24 of the X60 (and the 5in flywheel, on the same belt), the 1in
  // wheels twice the accelerators the other way
  kAccel: 22 / 24, kHood: 2,
  // what turns with them: the 5in aluminum flywheel (0.43 kg, 8.7e-4 kg m^2: its CAD solid), the
  // accelerators (two 3in urethane wheels, their shaft and gears: 1.5e-4), the 1in wheels (2e-5)
  Jfly: 8.7e-4, Jaccel: 1.5e-4, Jhood: 2e-5,
  // the hood's swing about the accelerators' axle (deg, up and over: the CAD's mates' 45 deg, from
  // the steep end as exported); the shot's elevation by the CAD's geometry at 0 is 88 deg (the nip,
  // 178 deg round from the shot about the accelerators' axle)
  hoodMax: 45,
  // the hood's profile (an estimate: an X44 through a big reduction), deg/s and deg/s^2
  hoodV: 300, hoodA: 3000,
  // their velocity loop (estimates: a VelocityVoltage with the X60's own kV, 0.124 V per rotation/s,
  // and kP 0.3), on the Talon at its 1 kHz
  kP: 0.3,
  // ready: within 0.3 m/s of the exit speed, 1.5 deg of hood, 3 deg of yaw (estimates)
  tolV: 0.3, tolHood: 1.5, tolYaw: 3,
};
// the elevation (deg) a hood swing h (deg) gives by the CAD's geometry: square to the line from the
// accelerators' axle to the nip between the hood's two 1in wheels
export function hoodElevation(h) {
  const [ax, ay] = SHOOTER.accel.at, [u, l] = SHOOTER.hood;
  const ang = (q) => (Math.atan2(q.at[1] - ay, q.at[0] - ax) + 2 * Math.PI) % (2 * Math.PI);
  const a = (ang(u) + ang(l)) / 2 / DEG; // (~178)
  return a - 90 - h;
}
export const PRELOAD = [0, 1].flatMap((s) => [0, 1, 2, 3].map((k) => {
  const [x, z] = SPIN.axes[s], a = (k / 4) * 2 * Math.PI + 0.4;
  return [x + 0.112 * Math.cos(a), SPIN.top + 0.08, z + 0.112 * Math.sin(a)];
}));

// ------------------------------------------------------------------ the drives
// (current limits: their 2025 code's, rr1706/konshu: the intake 80 A stator, 40 A supply; the funnel's
// rollers 40 A, 40 A; the arm 50 A, 30 A. The shooters' X60s: 80 A, 40 A, as their intake's)
export function build1706() {
  // the intake's rollers: positive takes FUEL in (the rollers' backs and the 1in wheels' fronts
  // running up, deployed)
  const intake = new Train({ name: 'intake rollers', motor: new Motor('krakenX60', { count: 2, statorLimit: 80, supplyLimit: 40 }) });
  const top = intake.add(new Shaft({ name: 'intake top roller', inertia: 1.2e-4 }), -10 / 14);
  const bottom = intake.add(new Shaft({ name: 'intake bottom roller', inertia: 1.2e-4 }), -(10 / 14) * (14 / 24));
  const wheels = intake.add(new Shaft({ name: 'intake 1in wheels', inertia: 4e-5 }), (10 / 14) * (14 / 24) * (18 / 16) * (24 / 20));
  // the arm's pivot (positive swings it out and down)
  const pivot = new Shaft({ name: 'intake arm', motor: new Motor('krakenX60', { ratio: 16 * (32 / 22), statorLimit: 50, supplyLimit: 30 }), inertia: INTAKE.J, drag: 0.3 });
  // the spindexers (positive carries FUEL round to the bowl's back the way it feeds: SPIN_DIR)
  const spin = [0, 1].map((i) => new Shaft({ name: `spindexer ${i ? 'left' : 'right'}`, motor: new Motor('krakenX44', { ratio: SPIN.ratio, statorLimit: 40, supplyLimit: 40 }), inertia: SPIN.J, drag: 0.01 }));
  // the kickers (positive lifts FUEL up the throat)
  const kicker = [0, 1].map((i) => {
    const tr = new Train({ name: `kicker ${i ? 'left' : 'right'}`, motor: new Motor('krakenX44', { ratio: KICKER.ratio, statorLimit: 40, supplyLimit: 40 }) });
    return { train: tr, front: tr.add(new Shaft({ name: 'kicker front wheels', inertia: 3e-5 }), 1), back: tr.add(new Shaft({ name: 'kicker back wheels', inertia: 3e-5 }), 1) };
  });
  // the shooters (positive flings FUEL up between the wheels)
  const shooter = [0, 1].map((i) => {
    const tr = new Train({ name: `shooter ${i ? 'left' : 'right'}`, motor: new Motor('krakenX60', { statorLimit: 80, supplyLimit: 40 }) });
    tr.add(new Shaft({ name: '5in flywheel', inertia: SHOOTER.Jfly }), SHOOTER.kAccel);
    return { train: tr, accel: tr.add(new Shaft({ name: '3in accelerator wheels', inertia: SHOOTER.Jaccel }), SHOOTER.kAccel), hood: tr.add(new Shaft({ name: 'hood 1in wheels', inertia: SHOOTER.Jhood }), SHOOTER.kAccel * SHOOTER.kHood) };
  });
  return { intake, top, bottom, wheels, pivot, spin, kicker, shooter };
}

const Z = [0, 0, 1];
// which way each spindexer turns to feed, seen from above (+1: counterclockwise, the right one's
// FUEL along its outer side toward the back; the left one mirrors it). Not in their CAD: the way that
// feeds on the bench (tools/mechlab1706.mjs spin)
export const SPIN_DIR = [1, -1];
// the wheels (robot frame; the intake's posed with placeIntake, the shooters' with placeShooter)
export function wheels1706(t) {
  const I = INTAKE, w = (name, shaft, { at, r, z }, mu, tread) => new Wheel({ name, shaft, center: [at[0], at[1], (z[0] + z[1]) / 2], axis: Z, radius: r, width: z[1] - z[0], mu, tread }).onPivot(t.pivot, [...PIVOT, 0], [0, 0, -1]);
  const intake = [
    w('intake top roller (polycarbonate)', t.top, I.top, MU.polycarb, TREAD.polycarb),
    w('intake bottom roller (polycarbonate)', t.bottom, I.bottom, MU.polycarb, TREAD.polycarb),
    w('intake 1in urethane wheels', t.wheels, I.wheels, MU.urethane, TREAD.urethane),
  ];
  intake.forEach((x) => { x.home = x.center.slice(); });
  // the kickers: each wheel's axis turned so its face toward the throat runs up when its shaft turns
  // positive
  const kicker = [0, 1].flatMap((i) => {
    const s = i ? -1 : 1, K = KICKER, ax = [K.axis[0], 0, K.axis[2] * s];
    return ['front', 'back'].map((k) => {
      const c = [K[k][0], K[k][1], K[k][2] * s], [tx, tz] = TURRET[i], n = [tx - c[0], 0, tz - c[2]];
      const up = ax[2] * n[0] - ax[0] * n[2]; // ((axis x n).y)
      return new Wheel({ name: `kicker ${k} 2in urethane wheels`, shaft: t.kicker[i][k], center: c, axis: ax.map((v) => v * Math.sign(up)), radius: K.r, width: K.w, mu: MU.urethane, tread: TREAD.urethane });
    });
  });
  return { intake, kicker };
}
// the shooters' wheels (both turrets), posed with placeShooter
export function shooterWheels(t) {
  const S = SHOOTER;
  return [0, 1].map((i) => [
    new Wheel({ name: '3in accelerator wheels (urethane)', shaft: t.shooter[i].accel, center: [0, 0, 0], axis: Z, radius: S.accel.r, width: S.accel.w, mu: MU.urethane, tread: TREAD.urethane }),
    ...S.hood.map((h, k) => new Wheel({ name: `hood ${k ? 'lower' : 'upper'} 1in wheels (urethane)`, shaft: t.shooter[i].hood, center: [0, 0, 0], axis: Z, radius: h.r, width: h.w, mu: MU.urethane, tread: TREAD.urethane })),
  ]);
}
// The bumper's foam FUEL climb on the way in: their bumpers are 1/8in aluminum U-channel backing
// with ~2.75-3in of PE foam (their thread), the CAD's foam 0.340 to 0.398 m out (the backing from
// 0.321), 0.033 to 0.154 m up, round the frame: a solid block, soft (20 kN/m, as 604's)
export function bumper1706() {
  return new Block({ name: 'bumper', x0: 0.321, x1: 0.398, y0: 0.033, y1: 0.154, hw: 0.457, mu: BUMPER_MU, tread: 20000 });
}

const rot = ([x, y], [cx, cy], a) => { const c = Math.cos(a), s = Math.sin(a), dx = x - cx, dy = y - cy; return [cx + dx * c - dy * s, cy + dx * s + dy * c]; };
// the arm's turn from the CAD's pose (rad; counterclockwise seen from the right: out and down is
// negative) at its swing deg
export const armRot = (deg) => -deg * DEG;
export function placeIntake(wheels, deg) {
  const a = armRot(deg);
  for (const w of wheels) { const [x, y] = rot(w.home, PIVOT, a); w.center = [x, y, w.home[2]]; }
}
// the hopper box: how far out (m) at the arm's swing deg, and how fast it slides per radian of the
// arm's swing (m/rad)
export function boxExt(ext, deg) {
  const u = Math.max(0, Math.min(ext.length - 1, deg)), i = Math.min(ext.length - 2, Math.floor(u)), f = u - i;
  return Math.min(BOX.out, ext[i] + (ext[i + 1] - ext[i]) * f);
}
export function boxRate(ext, deg) { return ((boxExt(ext, deg + 0.5) - boxExt(ext, deg - 0.5)) * 180) / Math.PI; }
// a turret-frame point (x along the shot, y up, z) where turret i is at yaw (rad, three.js sense)
export function turretPoint(i, [x, y, z], yaw) {
  const c = Math.cos(yaw), s = Math.sin(yaw), [tx, tz] = TURRET[i];
  return [tx + x * c + z * s, y, tz - x * s + z * c];
}
// the hood's 1in wheels' centers (turret frame) at its swing h (deg)
export function hoodWheelAt(k, h) { return rot(SHOOTER.hood[k].at, SHOOTER.accel.at, -h * DEG); }
// turret i's shooter wheels where its yaw (rad) and hood swing (deg) put them
export function placeShooter(i, [accel, upper, lower], yaw, h) {
  accel.center = turretPoint(i, [...SHOOTER.accel.at, 0], yaw);
  upper.center = turretPoint(i, [...hoodWheelAt(0, h), 0], yaw);
  lower.center = turretPoint(i, [...hoodWheelAt(1, h), 0], yaw);
  // (their axes across the shot, turned so the accelerators' face toward the FUEL (behind them) and
  // the 1in wheels' (in front of them) run up when the shaft turns positive)
  const s = Math.sin(yaw), c = Math.cos(yaw), ax = [s, 0, c];
  accel.axis = ax.map((v) => -v);
  upper.axis = ax; lower.axis = ax;
}

// the ball path's surfaces from their CAD (js/mech/path1706.js): the body; the intake's arm on its
// pivot; the hopper box (it slides with the arm); each spindexer's disc and hubs (they turn, and FUEL
// they touch push back on their motors); each turret and its hood (posed each step); the bumper's
// foam; the net over the top
// (FUEL's foam on the CAD's polycarbonate, aluminum and printed parts: 0.3, an estimate, as 604's)
export function pathSurfaces1706(mesh, t, { mu = 0.3 } = {}) {
  const s = (k, pos, m = mu) => new Surface({ name: `${k} (CAD)`, vertices: mesh[k].v.map((x) => x / 1000), indices: mesh[k].i, mu: m, pos });
  const body = s('body', [0, 0, 0]);
  // (the spindexers' bowls: 1/8in polycarbonate bands, TREAD.bowl: how far they give where a FUEL
  // presses them)
  const bowl = s('bowl', [0, 0, 0]);
  bowl.tread = TREAD.bowl;
  const intake = s('intake', [...PIVOT, 0]).onShaft(t.pivot, [...PIVOT, 0], [0, 0, -1]);
  // (the box slides as the arm turns: boxRate m per radian of the arm, set by placeBox)
  const cam = { k: 0, get omega() { return t.pivot.omega * this.k; }, J: Infinity, impulse(L) { t.pivot.impulse(L * this.k); } };
  const hopper = s('hopper', [0, 0, 0]).onSlide(cam, [1, 0, 0]);
  hopper.slideK = 1;
  hopper.cam = cam;
  const spin = ['spinA', 'spinB'].map((k, i) => {
    const sf = s(k, [SPIN.axes[i][0], 0, SPIN.axes[i][1]], MU.disc).onShaft(t.spin[i], [SPIN.axes[i][0], 0, SPIN.axes[i][1]], [0, SPIN_DIR[i], 0]);
    sf.name = `spindexer ${i ? 'left' : 'right'} (CAD)`;
    return sf;
  });
  const turret = [0, 1].map((i) => s('turret', [TURRET[i][0], 0, TURRET[i][1]]));
  const hood = [0, 1].map((i) => s('hood', [TURRET[i][0], 0, TURRET[i][1]]));
  return { body, bowl, intake, hopper, spin, turret, hood, bumper: bumper1706(), net: new Net1706() };
}
export function placeBox(sf, ext, deg) {
  sf.setPose([boxExt(ext, deg), 0, 0]);
  sf.cam.k = boxRate(ext, deg); // (the arm's shaft turns positive swinging out: the box goes out)
}
export function placeSpin(sf, i, phi) { sf.setPose([SPIN.axes[i][0], 0, SPIN.axes[i][1]], SPIN_DIR[i] * phi, 0); }
export function placeTurret(sf, i, yaw, h) {
  sf.turret[i].setPose([TURRET[i][0], 0, TURRET[i][1]], yaw, 0);
  sf.hood[i].setPose(turretPoint(i, [...SHOOTER.accel.at, 0], yaw), yaw, -h * DEG);
}
// The net over the open top (their CAD doesn't model one: the simplified model's, js/robotConfigs.js
// bay.dome): pinned along the walls' tops (0.548 m), the back panel's top edge and the box's front,
// it stretches, and the load bulges it (as 4414's net dome, js/mech/r4414.js netY): up to NET.bulge at
// its middle, tapering to its ties. A FUEL under it presses it out to there, past it only as far as
// its soft tread gives; nothing pushes the load back down. In the robot's frame.
export const NET = { x0: -0.01, y: 0.548, hw: 0.374, bulge: 0.15 };
class Net1706 extends Surface {
  constructor() {
    super({ name: 'net', vertices: [0, 0, 0, 0.01, 0, 0, 0, 0, 0.01], indices: [0, 1, 2], mu: 0.3, tread: 1500 });
    this.place(0);
  }
  place(ext) { this.x1 = BOX.front + ext; }
  at(x, z) {
    const L = this.x1 - NET.x0, u = (x - NET.x0) / L, v = (z + NET.hw) / (2 * NET.hw), B = NET.bulge;
    const su = Math.sin(Math.PI * u), sv = Math.sin(Math.PI * v);
    return [NET.y + B * su * sv, (B * Math.PI * Math.cos(Math.PI * u) * sv) / L, (B * su * Math.PI * Math.cos(Math.PI * v)) / (2 * NET.hw)];
  }
  touches(p) {
    const [x, y, z] = p;
    if (x <= NET.x0 || x >= this.x1 || Math.abs(z) >= NET.hw) return [];
    const [h, hx, hz] = this.at(x, z), l = Math.hypot(hx, 1, hz), dist = (h - y) / l;
    if (dist >= 0.075 || dist < -0.03) return [];
    return [{ depth: 0.075 - dist, n: [hx / l, -1 / l, hz / l] }];
  }
}
export { Net1706 };
