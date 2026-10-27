// 8793 (Pumpkin Bots) built as a physical mechanism (js/mech/mechanism.js): their drive chains as
// the team describes them, the rollers, wheels and their places from their CAD (Onshape
// "8793-2026-A-0000 Robot": Intake V3, Conveyor V2, Turret, Shooter), in the robot's frame (x
// forward, y up, z to its right, from the frame's center on the floor).
//
// Horizontal rollers, all on one Kraken X60, slowing stage by stage toward the intake (so FUEL
// speeds up, and spreads out, on its way to the turret): X60 > 18:33 > conveyor roller 1 > 25:24 >
// roller 2 > 24:43 > roller 3 (the intake arm's pivot shaft) > 23:20 > the arm's first roller >
// 20:20 > second > 20:20 > third > 20:28 > the last roller, at the arm's tip. (25:24 speeds up a
// little.)
// Vertical-axis rollers (one omni wheel, one compliant wheel, which still lets FUEL down past
// it): each on its own Kraken X44 on a 5:1 MAXPlanetary > 28:18 (speeds up, to match the rpm) >
// a 1:1 90-degree gearbox, beside the second roller from the X60. The gearboxes lock the X44s and
// the X60 together (the team): every roller turns in step, and the three motors share the load.
// From their code (github.com/Pumpkin-Bots/2026_Robot, Constants.GroundIntakeConstants and
// GroundIntakeSubsystem): the roller motor (CAN 22) runs at 95% duty cycle with a 70 A stator
// limit, the two vertical-roller motors (CAN 23, 24) at 95% x 0.9244 (INDEXER_TO_ROLLER_RATIO),
// mirrored, with 50 A limits (and 40 A from the supply). Then the turret indexer and the shooter,
// further down.
import { IN } from '../constants.js';
import { BUMPER_T, BUMPER_Y0, BUMPER_Y1 } from '../robotConfigs.js';
import { Motor, Shaft, Surface, Train, Wheel } from './mechanism.js';

// the gear chains: [name, belt/gear ratio from the shaft before (driver:driven teeth), roller
// diameter (m), friction against FUEL, what's on it]. Conveyor 3 is the intake's pivot shaft (it's
// in the intake assembly); the intake's rollers are the four on the arm, pivot outward.
// (Aluminum on foam grips far less than a compliant wheel or grip tape: estimates to tune.)
// what each grips FUEL with (estimates; the order is the team's: compliant wheels grip far more
// than mecanums' rollers or bare aluminum, and it's what carries a centered FUEL all the way through)
// An omni wheel touches a FUEL only on the small hard rollers round its rim, where a compliant
// wheel wraps it on a broad patch: far less grip (the team). 0.6 halves the auto-unjams in
// matches against 1.0 (the rollers slip on FUEL waiting at the turret instead of stalling the
// X60) and FUEL still go through two abreast (tools/mechlab.mjs funnel, robot).
export const MU = { compliant: 1.6, gripTape: 1.3, omni: 0.6, mecanum: 0.8, aluminum: 0.45 };
// a 4in 45A compliant wheel's give at its rim (N/m; an estimate)
export const COMPLIANT_TREAD = 12000;
export const HORIZONTAL = [
  ['conveyor 1', [18, 33], 2 * IN, MU.mecanum, '2in compliant wheel in the middle, two 2in mecanums each side, all pushing to the middle'],
  ['conveyor 2', [25, 24], 2 * IN, MU.mecanum, 'eight 2in mecanums, four each side, pushing to the middle'],
  ['conveyor 3 (pivot)', [24, 43], 2 * IN, MU.mecanum, 'twelve 2in mecanums: four on one side, eight on the other, pushing toward each other'],
  ['intake 1', [23, 20], 2 * IN, MU.aluminum, '2in aluminum roller'],
  ['intake 2', [20, 20], 2 * IN, MU.aluminum, '2in aluminum roller'],
  ['intake 3', [20, 20], 2 * IN, MU.aluminum, '2in aluminum roller'],
  ['intake 4 (tip)', [20, 28], 2 * IN, MU.gripTape, '2in aluminum roller in cat-tongue grip tape'],
];
// beside conveyor 2, at the side walls: one 4in omni wheel, and 4in 45A compliant wheels
// perX60: each X44's turns per turn of the X60 they're locked to. Their code runs the X44s at
// 0.9244 (INDEXER_TO_ROLLER_RATIO) of the X60's duty cycle, which matches the two motors' free
// speeds when an X44 turns 1.2 times the X60 (0.9244 x 7530 / 5800 = 1.200)
export const VERTICAL = { ratio: [[1, 5], [28, 18], [1, 1]], diameter: 4 * IN, perX60: 1.2 };
// At the neck, where the conveyor's floor ramps down to the turret indexer's rails and FUEL
// go single file: a vertical-axis wheel each side, 4in on both (the team: the CAD's 5in and 3in
// are out of date), an omni on the left and a 45A compliant on the right, like the vertical
// rollers; on the CAD's axles (a 0.139 m gap: a FUEL squeezes 11 mm through), each belted from the
// vertical roller on its side (the CAD's 18t:18t on the left, 18t:28t on the right), so the two
// faces run at different speeds and turn a pair of FUEL arriving abreast instead of wedging them.
export const NECK = {
  x: 0.065, y: 0.237,
  left: { z: -0.122, d: 4 * IN, belt: [18, 18], kind: 'omni' },
  right: { z: 0.119, d: 4 * IN, belt: [18, 28], kind: 'compliant' },
};
// what their code commands while intaking (duty cycle)
export const DUTY = { rollers: 0.95, vertical: 0.95 * 0.9244 };

// the trains: each shaft's k = the turns it makes per turn of the motor
export function build8793() {
  const horizontal = new Train({ name: 'horizontal rollers', motor: new Motor('krakenX60', { statorLimit: 70, supplyLimit: 40 }) });
  let k = 1;
  const rollers = HORIZONTAL.map(([name, [a, b], d, mu, what]) => {
    k *= a / b;
    const s = horizontal.add(new Shaft({ name, inertia: 6e-5 }), k);
    s.diameter = d; s.mu = mu; s.what = what;
    return s;
  });
  const vk = VERTICAL.ratio.reduce((p, [a, b]) => (p * a) / b, 1);
  // the two X44s are geared into the same chain through their 90-degree gearboxes, so every
  // roller, horizontal and vertical, turns locked together and all three motors share the load:
  // each X44 at VERTICAL.perX60 times the X60's speed; the vertical rollers on them (vk), and the
  // neck's omni wheels belted to those. (Negative: the train turns backward while intaking; this
  // keeps each vertical roller's face at the FUEL running rearward.)
  const x44 = ['left', 'right'].map(() => horizontal.couple(new Motor('krakenX44', { statorLimit: 50, supplyLimit: 40 }), VERTICAL.perX60));
  const vshafts = ['left', 'right'].map((side) => {
    const s = horizontal.add(new Shaft({ name: `vertical roller, ${side}`, inertia: 1e-4 }), -VERTICAL.perX60 * vk);
    s.diameter = VERTICAL.diameter;
    const nk = NECK[side];
    s.neck = horizontal.add(new Shaft({ name: `neck ${nk.d / IN}in ${nk.kind === 'omni' ? 'omni' : 'compliant 45A'}, ${side}`, inertia: 6e-5 }), (-VERTICAL.perX60 * vk * nk.belt[0]) / nk.belt[1]);
    s.neck.diameter = nk.d;
    return s;
  });
  // the turret indexer (a positive command: the wheels' bottoms run rearward)
  const indexer = new Train({ name: 'turret indexer', motor: new Motor('krakenX60', { ratio: INDEXER.ratio, statorLimit: 60, supplyLimit: 40 }) });
  const ishafts = [indexer.add(new Shaft({ name: 'turret indexer', inertia: 4e-5 }), -1)];
  // the shooter: the 4in wheels and, geared to them, the 2in wheels over them turning the other way
  // (a positive command turns the 4in wheels so FUEL leave over them)
  const shooter = new Train({ name: 'shooter', motor: new Motor('krakenX60', { statorLimit: 160, supplyLimit: 60 }) });
  const large = shooter.add(new Shaft({ name: 'shooter 4in wheels', inertia: SHOOTER.moi }), -1 / SHOOTER.ratio);
  const small = shooter.add(new Shaft({ name: 'shooter 2in wheels', inertia: 6e-5 }), SHOOTER.smallPerLarge / SHOOTER.ratio);
  return { horizontal, rollers, x44, vshafts, indexer, ishafts, shooter, large, small };
}

// run the intake as their code does (on = false: everything off; dir -1: reversed, at full, for
// an unjam or to spit FUEL out). x44 = false leaves the X44s unpowered (coasting): the vertical
// rollers still turn, driven through the gearboxes by the X60.
export function intake8793({ horizontal, x44 }, on = true, { x44: powered = true, dir = 1 } = {}) {
  // (the rollers' bottoms run rearward: the train turns backward)
  horizontal.cmd = !on ? 0 : dir > 0 ? -DUTY.rollers : 1;
  for (const m of x44) m.cmd = !on ? 0 : !powered ? null : dir > 0 ? -DUTY.vertical : 1;
}

// ------------------------------------------------------------------ the conveyor's layout
// Robot frame (x forward, y up, z to the robot's right). From their CAD: the shafts' x and
// height (conveyor 1 x 0.122 y 0.315, conveyor 2 x 0.217 y 0.34, the pivot x 0.292 y 0.336) over
// the polycarbonate floor (0.183 up, ramping down at x 0.153 to the indexer's rails: it's one of
// the CAD surfaces, pathSurfaces below), and the vertical rollers at x 0.217, 0.24 m each side,
// centered 0.241 up. The mecanums sit as the team says: conveyor 1's five wheels where the CAD has
// them (a compliant wheel in the middle, two mecanums each side, all pushing to the middle),
// conveyor 2's eight and the pivot's twelve spread evenly across the shaft's width (estimates):
// conveyor 2 four and four pushing to the middle, the pivot the four on the robot's right pushing
// left and the eight on its left pushing right. Vertical rollers: the omni wheel on the robot's
// left, the compliant (45A) on its right, their inner faces running rearward.
export const CONVEYOR = {
  shafts: [ // [roller index in HORIZONTAL, x, y, the wheels' z (or the half width they spread over)]
    [0, 0.122, 0.315, [-0.077, -0.039, -0.001, 0.037, 0.075]],
    // (conveyor 2's eight spread evenly over what FUEL touch: between the vertical rollers' faces)
    [1, 0.217, 0.34, 0.24 - 2 * IN],
    [2, 0.292, 0.336, 0.31],
  ],
  vertical: { x: 0.217, y: 0.241, z: 0.24 },
};

// the wheels, on the trains from build8793() (the horizontal train turning so the rollers' bottoms
// run rearward: its speed is negative)
export function conveyorWheels({ rollers, vshafts }) {
  const wheels = [];
  const w = 1 * IN, flex = { tread: 5000 }, mec = { tread: 20000 };
  // a mecanum's hand: a FUEL under it is pushed toward dz (+1: the robot's right) while the shaft
  // turns at sign s (contact below the axle: grip direction c t1 + s t2, t1 = rearward/forward)
  const hand = (dz, s = -1) => 45 * Math.sign(dz * s);
  const spread = (n, hw) => (Array.isArray(hw) ? hw : Array.from({ length: n }, (_, i) => -hw + w / 2 + ((2 * hw - w) * i) / (n - 1)));
  for (const [ri, x, y, hw] of CONVEYOR.shafts) {
    const s = rollers[ri], base = { shaft: s, axis: [0, 0, 1], radius: s.diameter / 2, width: w, mu: s.mu };
    const at = (z) => [x, y, z];
    // the bare 1/2in hex shaft between the wheels (FUEL can't pass up through it)
    wheels.push(new Wheel({ shaft: s, name: `${s.name} shaft`, center: at(0), axis: [0, 0, 1], radius: 0.29 * IN, width: 0.62, mu: 0.3 }));
    if (ri === 0) {
      // compliant in the middle, two mecanums each side, all pushing to the middle
      spread(5, hw).forEach((z, i) => wheels.push(i === 2
        ? new Wheel({ ...base, ...flex, name: 'conveyor 1 compliant', center: at(z), mu: MU.compliant })
        : new Wheel({ ...base, ...mec, name: 'conveyor 1 mecanum', kind: 'mecanum', mecanum: hand(-Math.sign(z)), center: at(z) })));
    } else if (ri === 1) {
      spread(8, hw).forEach((z) => wheels.push(new Wheel({ ...base, ...mec, name: 'conveyor 2 mecanum', kind: 'mecanum', mecanum: hand(-Math.sign(z)), center: at(z) })));
    } else {
      // twelve: the four toward the robot's right (+z) push left, the eight on its left push right
      spread(12, hw).forEach((z, i) => wheels.push(new Wheel({ ...base, ...mec, name: 'pivot mecanum', kind: 'mecanum', mecanum: hand(i >= 8 ? -1 : 1), center: at(z) })));
    }
  }
  const v = CONVEYOR.vertical, [vl, vr] = vshafts, r = vl.diameter / 2;
  // two 1in-wide wheels stacked on each, 1in apart, around the FUEL's center height
  for (const dy of [-0.6 * IN, 0.6 * IN]) {
    wheels.push(new Wheel({ shaft: vl, name: 'vertical omni', kind: 'omni', center: [v.x, v.y + dy, -v.z], axis: [0, -1, 0], radius: r, width: w, mu: MU.omni }));
    wheels.push(new Wheel({ shaft: vr, name: 'vertical compliant 45A', center: [v.x, v.y + dy, v.z], axis: [0, 1, 0], radius: r, width: w, mu: MU.compliant, tread: COMPLIANT_TREAD }));
  }
  for (const [i, side] of [[0, 'left'], [1, 'right']]) {
    const nk = NECK[side], sh = vshafts[i].neck;
    const how = nk.kind === 'omni' ? { kind: 'omni', mu: MU.omni } : { mu: MU.compliant, tread: COMPLIANT_TREAD };
    wheels.push(new Wheel({ shaft: sh, name: sh.name, ...how, center: [NECK.x, NECK.y, nk.z], axis: [0, side === 'left' ? -1 : 1, 0], radius: nk.d / 2, width: w }));
  }
  return wheels;
}

// ------------------------------------------------------------------ the intake arm, deployed
// Its four rollers from their CAD (Intake V3, exported down on the carpet, relative to the
// pivot at x 0.292 y 0.336): pivot outward (0.0835, -0.0059), (0.1834, -0.0152),
// (0.2593, -0.0803), (0.2764, -0.1823). Down, the tip roller pinches a FUEL on the carpet, the
// next drives it up the bumper's face, and the first two carry it over the bumper to the pivot.
export const ARM = {
  pivot: [0.292, 0.336],
  rollers: [[0.0835, -0.0059], [0.1834, -0.0152], [0.2593, -0.0803], [0.2764, -0.1823]],
  halfWidth: 0.31,
};
export function armWheels({ rollers }) {
  const [px, py] = ARM.pivot;
  return ARM.rollers.map(([dx, dy], i) => {
    const s = rollers[3 + i];
    return new Wheel({ name: s.name, shaft: s, center: [px + dx, py + dy, 0], axis: [0, 0, 1], radius: s.diameter / 2, width: 2 * ARM.halfWidth, mu: s.mu });
  });
}
// swing armWheels' rollers up by roll (rad) about the pivot: folding up to stow
export function placeArm(wheels, roll) {
  const [px, py] = ARM.pivot, c = Math.cos(roll), s = Math.sin(roll);
  wheels.forEach((w, i) => { const [dx, dy] = ARM.rollers[i]; w.center = [px + dx * c - dy * s, py + dx * s + dy * c, 0]; });
}
// Where 8793 holds a preload with its arm stowed (they settle there, tools/mechlab.mjs): two on
// the indexer's rails, one on the ramp down to them, two on the floor behind the pivot's rollers
// (in front of those, the rollers' mecanums let a FUEL roll out over the bumper)
export const PRELOAD = [[-0.06, 0.135, 0], [-0.14, 0.26, 0], [0.066, 0.21, 0], [0.21, 0.255, -0.085], [0.21, 0.255, 0.085]];

// ------------------------------------------------------------------ the turret indexer
// Under the turntable (their CAD, Turret assembly): a shaft of three 2in 30A flex wheels, 1in
// wide, at x -0.039, 0.226 up. A Kraken X60 (CAN 28, 60 A stator, 40 A supply) turns it direct
// (the team; the CAD shows a 9:1 MAXPlanetary); their code runs it at 65% duty while
// shooting (TurretConstants.TURRET_INDEXER_SPEED). (The CAD's second shaft of flex wheels behind
// the turret's axis was a concept, never built.) Single file off the conveyor, FUEL ride a pair of
// roller rails (two plates 0.108 m apart, a FUEL's width less a bit, so a FUEL sits in them
// 0.055 m over their edges), flat under the wheels, then up a ramp behind them: the wheels push
// FUEL back along the rails and up the ramp, and nothing but their own momentum carries them on
// up through the turntable until the shooter's wheels catch them. (The rails are CAD surfaces.)
export const INDEXER = {
  ratio: 1,
  shafts: [[-0.039, 0.226]],
  wheelZ: [-0.025, 0.001, 0.026],
};
export const DUTY_INDEXER = 0.65;
export function indexerWheels({ ishafts }) {
  const wheels = [];
  INDEXER.shafts.forEach(([x, y], i) => {
    for (const z of INDEXER.wheelZ) {
      wheels.push(new Wheel({ name: ishafts[i].name, shaft: ishafts[i], center: [x, y, z], axis: [0, 0, 1], radius: 1 * IN, width: 1 * IN, mu: MU.compliant, tread: 5000 }));
    }
    // its bare 1/2in hex shaft, out to the bearings (z -0.19 to 0.09), turning with them
    wheels.push(new Wheel({ name: `${ishafts[i].name} shaft`, shaft: ishafts[i], center: [x, y, -0.05], axis: [0, 0, 1], radius: 0.29 * IN, width: 0.28, mu: 0.3 }));
  });
  return wheels;
}

// ------------------------------------------------------------------ the shooter
// On the turret (their CAD, Shooter assembly; turret frame: x along the shot, from the turret's
// axis): a 4in 45A solid roller wheel on a shaft 0.127 m out from the axis and 0.411 m up, with a
// flywheel half an SDS 4in flywheel's (the team; the CAD shows two whole ones), and three 2in 80A
// Stealth wheels on a shaft 0.203 m from it, geared to it through 28t:28t spur gears so they turn
// the other way. A FUEL coming up through the turntable is caught by the 4in wheel, carried
// between them (0.9in narrower than a FUEL) and leaves square to the line between their centers: the rack (the "hood") swings the 2in wheels around the 4in ones to set the launch
// elevation (their code: rack 18..38 deg, elevation = 90 - rack). From their code
// (ShooterConstants): one Kraken X60 (CAN 27; 160 A stator, 60 A supply), 28:18 to the 4in
// wheels, the 2in wheels at 18/28 of the 4in wheels' speed (so FUEL leave with backspin);
// velocity control in volts, kV 0.125 and kP 0.4 per motor rotation/s, up to 90 rotations/s. The
// 4in shaft's inertia: half the SDS flywheel's 0.00117 kg·m² (Onshape's mass properties: 0.70 kg),
// the 4in solid roller wheel (~0.3 kg, ~0.00035) and the shaft and pulleys.
export const SHOOTER = {
  ratio: 28 / 18, smallPerLarge: 18 / 28, moi: 0.00097,
  large: { x: 0.127, y: 0.411, r: 2 * IN }, small: { r: 1 * IN, d: 0.203 },
  elevation: [52, 72],
  kV: 0.125, kP: 0.4, maxRps: 90,
  // friction against FUEL, set so that their old gearing (both shafts at the motor's speed)
  // comes near the ~0.205 m/s per motor rotation/s their code says they measured with it (it
  // can't be the 28:18 gearing's: that'd be half again the wheels' mean surface speed). Past 1.7
  // the FUEL no longer slips and it levels off at ~0.19, the flywheel's sag taking the rest
  // (tools/mechlab.mjs calibrate)
  mu: { roller: 1.7, stealth: 1.7 },
};
// the 2in shaft's center, turret frame, at launch elevation e (deg)
export function shooterSmallAt(e) {
  const a = (e * Math.PI) / 180, { large: L, small: S } = SHOOTER;
  return [L.x - S.d * Math.sin(a), L.y + S.d * Math.cos(a)];
}
// the wheels, in the robot's frame with the turret (axis at turretX) facing yaw (rad, 0 forward)
export function shooterWheels({ large, small }, e, { turretX = -0.127, yaw = 0, mu = SHOOTER.mu } = {}) {
  const L = SHOOTER.large;
  const w = [
    new Wheel({ name: 'shooter 4in 45A roller', shaft: large, center: [0, 0, 0], axis: [0, 0, 1], radius: L.r, width: 1.5 * IN, mu: mu.roller, tread: 20000 }),
    new Wheel({ name: 'shooter 2in 80A Stealth', shaft: small, center: [0, 0, 0], axis: [0, 0, 1], radius: SHOOTER.small.r, width: 3 * IN, mu: mu.stealth, tread: 40000 }),
  ];
  placeShooter(w, e, yaw, turretX);
  return w;
}
// move shooterWheels' wheels to the turret's yaw and the hood's elevation e (deg)
export function placeShooter([lw, sw], e, yaw = 0, turretX = -0.127) {
  const c = Math.cos(yaw), s = Math.sin(yaw);
  const at = (x, y, z) => [turretX + x * c + z * s, y, -x * s + z * c];
  const [sx, sy] = shooterSmallAt(Math.max(SHOOTER.elevation[0], Math.min(SHOOTER.elevation[1], e))), L = SHOOTER.large;
  lw.center = at(L.x, L.y, 0); sw.center = at(sx, sy, 0);
  lw.axis = sw.axis = [s, 0, c];
}
// What leaves the shooter for a motor speed (rotations/s -> m/s; tools/mechlab.mjs shooter, the
// average over its elevations), a FUEL fed in slowly (1.5 m/s): the grip runs out past ~80
// rotations/s
export const EXIT_TABLE = [[0, 0], [15, 2.07], [20, 2.71], [30, 4.05], [40, 5.16], [50, 6.11], [60, 7.14], [70, 8.04], [80, 8.39], [90, 8.52]];
const interp = (t, x, i, j) => {
  for (let k = 1; k < t.length; k++) if (x <= t[k][i]) return t[k - 1][j] + ((t[k][j] - t[k - 1][j]) * (x - t[k - 1][i])) / (t[k][i] - t[k - 1][i]);
  return t[t.length - 1][j];
};
// ...and in play, fed by the direct-drive turret indexer up its ramp, a FUEL comes in faster and
// leaves faster: the median exit speed (relative to the robot) of 8793's shots in AI matches, at
// the flywheel speed it was commanded to (so the droop of a stream of shots is in it, as their
// FlywheelDroopCompensator learns it). Past 80 rotations/s, unmeasured: it stops there.
export const SHOT_TABLE = [[0, 0], [15, 2.07], [20, 2.71], [30, 4.15], [40, 5.4], [50, 6.29], [60, 7.21], [70, 8.26], [80, 9.21]];
export const exitSpeed = (rps) => interp(SHOT_TABLE, Math.max(0, rps), 0, 1);
export const rpsFor = (v) => interp(SHOT_TABLE, Math.max(0, v), 1, 0);
// their velocity loop (VelocityVoltage: kV x target + kP x error, in volts) toward targetRps,
// motor rotations per second; returns the duty cycle at battery V
export function flywheelDuty({ shooter }, targetRps, V = 12) {
  const rps = shooter.omega / (2 * Math.PI);
  const volts = SHOOTER.kV * targetRps + SHOOTER.kP * (targetRps - rps);
  return Math.max(-1, Math.min(1, volts / V));
}
// the exit speed their aim solver assumes for a motor speed (m/s): the two wheels' surface
// speeds averaged (FLYWHEEL_EFFECTIVE_DIAMETER_METERS)
export const codeExitSpeed = (rps) => Math.PI * ((4 * IN + 2 * IN * SHOOTER.smallPerLarge) / 2 / SHOOTER.ratio) * rps;

// ------------------------------------------------------------------ the ball path's surfaces
// Everything FUEL can touch between the intake and the shooter, from their CAD itself
// (tools/path-mesh.py -> js/mech/path8793.js: the polycarbonate floor and its ramp down to
// the indexer's rails, the rails, the tower's plates, the swerve shields, the arm's side plates,
// the turntable and the shooter's frame), the wheels the mechanism turns cut out: mechanism
// Surfaces (soft, through the foam), the turret's turned to yaw (rad; set surfaces.turret.yaw to
// turn it later).
export function pathSurfaces(mesh, { yaw = 0, turretX = -0.127, armPivot = ARM.pivot, mu = 0.3 } = {}) {
  const pos = { body: [0, 0, 0], arm: [armPivot[0], armPivot[1], 0], turret: [turretX, 0, 0] };
  const out = {};
  for (const [k, { v, i }] of Object.entries(mesh)) {
    out[k] = new Surface({ name: `${k} (CAD)`, vertices: v.map((x) => x / 1000), indices: i, mu, pos: pos[k], yaw: k === 'turret' ? yaw : 0 });
  }
  out.bumper = bumperSurface();
  out.guides = guideSurface();
  return out;
}
// What keeps FUEL where the team says they always are, where the CAD leaves it open: no FUEL ever
// gets between the indexer's rails and the turret structure's side plates (guides along the rails,
// 0.085 m out, from the neck to the back of the turret, up to the turntable's plate), or down
// beside the conveyor's floor into the front corners, where the swerve modules are (they aren't
// in the CAD): a cover there level with the floor.
export function guideSurface() {
  const V = [], I = [];
  const quad = (a, b, c, d) => { const o = V.length / 3; V.push(...a, ...b, ...c, ...d); I.push(o, o + 1, o + 2, o, o + 2, o + 3); };
  for (const zs of [1, -1]) {
    const z = 0.085 * zs;
    quad([0.0, 0.05, z], [-0.33, 0.05, z], [-0.33, 0.27, z], [0.0, 0.27, z]);
    // (the floor is 0.18 up out to 0.165 each side: the cover's just outside it, a hair lower)
    quad([0.15, 0.178, 0.165 * zs], [0.335, 0.178, 0.165 * zs], [0.335, 0.178, 0.335 * zs], [0.15, 0.178, 0.335 * zs]);
  }
  return new Surface({ name: 'guides', vertices: V, indices: I, mu: 0.3 });
}
// The front bumper, which FUEL climb over on the way in: square foam under fabric (the team; mu:
// fabric on foam, an estimate), its face and top. 8793's frame: 27.5in. The foam gives with a FUEL
// pressed into it (tread, N/m: an estimate, about 2 N per mm), so the arm's bare aluminum rollers
// squeeze FUEL against it lightly (the tip roller's 39 mm of overlap with the bumper's corner is
// mostly the bumper giving) and slip on one that's held up rather than stall the X60 (the team):
// 28-39 auto-unjams a match against 34-91 with a rigid bumper.
export function bumperSurface({ halfL = 13.75 * IN, halfW = 13.75 * IN + BUMPER_T, mu = 0.6, tread = 2000 } = {}) {
  const x1 = halfL + BUMPER_T;
  const V = [
    x1, BUMPER_Y0, -halfW, x1, BUMPER_Y0, halfW, x1, BUMPER_Y1, halfW, x1, BUMPER_Y1, -halfW, // face
    halfL, BUMPER_Y1, -halfW, halfL, BUMPER_Y1, halfW,                                         // top, back edge
  ];
  const I = [0, 1, 2, 0, 2, 3, 3, 2, 5, 3, 5, 4];
  return new Surface({ name: 'bumper', vertices: V, indices: I, mu, tread });
}
