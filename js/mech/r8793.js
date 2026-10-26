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
// a 1:1 90-degree gearbox, beside the second roller from the X60.
// From their code (github.com/Pumpkin-Bots/2026_Robot, Constants.GroundIntakeConstants and
// GroundIntakeSubsystem): the roller motor (CAN 22) runs at 95% duty cycle with a 70 A stator
// limit, the two vertical-roller motors (CAN 23, 24) at 95% x 0.9244 (INDEXER_TO_ROLLER_RATIO),
// mirrored, with 50 A limits (and 40 A from the supply). Then the turret indexer and the shooter,
// further down.
import { IN } from '../constants.js';
import { Motor, Shaft, Train, Wheel } from './mechanism.js';

// the gear chains: [name, belt/gear ratio from the shaft before (driver:driven teeth), roller
// diameter (m), friction against FUEL, what's on it]. Conveyor 3 is the intake's pivot shaft (it's
// in the intake assembly); the intake's rollers are the four on the arm, pivot outward.
// (Aluminum on foam grips far less than a compliant wheel or grip tape: estimates to tune.)
export const HORIZONTAL = [
  ['conveyor 1', [18, 33], 2 * IN, 1.0, '2in compliant wheel in the middle, two 2in mecanums each side, all pushing to the middle'],
  ['conveyor 2', [25, 24], 2 * IN, 1.0, 'eight 2in mecanums, four each side, pushing to the middle'],
  ['conveyor 3 (pivot)', [24, 43], 2 * IN, 1.0, 'twelve 2in mecanums: four on one side, eight on the other, pushing toward each other'],
  ['intake 1', [23, 20], 2 * IN, 0.45, '2in aluminum roller'],
  ['intake 2', [20, 20], 2 * IN, 0.45, '2in aluminum roller'],
  ['intake 3', [20, 20], 2 * IN, 0.45, '2in aluminum roller'],
  ['intake 4 (tip)', [20, 28], 2 * IN, 1.3, '2in aluminum roller in cat-tongue grip tape'],
];
// beside conveyor 2, at the side walls: one 4in omni wheel, and 4in 45A compliant wheels
export const VERTICAL = { ratio: [[1, 5], [28, 18], [1, 1]], diameter: 4 * IN };
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
  // the left one (the omni) and the right one (compliant), each on its own X44; a positive
  // command turns each one's face at the FUEL rearward
  const [left, right] = ['left', 'right'].map((side) => {
    const t = new Train({ name: `vertical roller, ${side}`, motor: new Motor('krakenX44', { statorLimit: 50, supplyLimit: 40 }) });
    const s = t.add(new Shaft({ name: `vertical roller, ${side}`, inertia: 1e-4 }), vk);
    s.diameter = VERTICAL.diameter;
    return [t, s];
  });
  // the turret indexer: the front shaft off the gearbox, the rear one belted to it turning the
  // other way (a positive command: the front wheels' bottoms run rearward)
  const indexer = new Train({ name: 'turret indexer', motor: new Motor('krakenX60', { ratio: INDEXER.ratio, statorLimit: 60, supplyLimit: 40 }) });
  const ishafts = INDEXER.shafts.map(([x], i) => indexer.add(new Shaft({ name: `turret indexer, ${i ? 'rear' : 'front'}`, inertia: 4e-5 }), i ? 1 : -1));
  // the shooter: the 4in wheels and, geared to them, the 2in wheels over them turning the other way
  // (a positive command turns the 4in wheels so FUEL leave over them)
  const shooter = new Train({ name: 'shooter', motor: new Motor('krakenX60', { statorLimit: 160, supplyLimit: 60 }) });
  const large = shooter.add(new Shaft({ name: 'shooter 4in wheels', inertia: SHOOTER.moi }), -1 / SHOOTER.ratio);
  const small = shooter.add(new Shaft({ name: 'shooter 2in wheels', inertia: 6e-5 }), SHOOTER.smallPerLarge / SHOOTER.ratio);
  return { horizontal, rollers, vertical: [left[0], right[0]], vshafts: [left[1], right[1]], indexer, ishafts, shooter, large, small };
}

// run the intake as their code does (on = false: everything off)
export function intake8793({ horizontal, vertical }, on = true, { verticalRollers = true } = {}) {
  horizontal.cmd = on ? -DUTY.rollers : 0; // the rollers' bottoms run rearward
  for (const t of vertical) t.cmd = on && verticalRollers ? DUTY.vertical : 0;
}

// ------------------------------------------------------------------ the conveyor's layout
// Robot frame (x forward, y up, z to the robot's right). From their CAD: the shafts' x and
// height (conveyor 1 x 0.122 y 0.315, conveyor 2 x 0.217 y 0.34, the pivot x 0.292 y 0.336), the
// vertical rollers at x 0.217, 0.24 m each side, centered 0.241 up, and the channel from the
// hopper model (bay.taper): full width (±0.31) at the pivot, narrowing from x 0.23 to ±0.165 at
// x 0.17 and to one FUEL (±0.087) at x 0.03. FUEL go under the rollers, which press them onto a
// floor; the floor is set so each roller squeezes a FUEL about SQUEEZE (to check against the
// robot). The mecanums are spread evenly across each shaft's width of the channel, each
// handed so it pushes the way the team says: conveyor 1 a compliant wheel between two mecanums
// each side, all pushing to the middle; conveyor 2 four and four pushing to the middle; the
// pivot twelve, the four on the robot's right pushing left and the eight on its left pushing
// right. Vertical rollers: the omni wheel on the robot's left, the compliant (45A) on its right,
// their inner faces running rearward.
export const SQUEEZE = 0.3 * IN;
export const CONVEYOR = {
  shafts: [ // [roller index in HORIZONTAL, x, y, half width of the channel there]
    [0, 0.122, 0.315, 0.138],
    [1, 0.217, 0.34, 0.28],
    [2, 0.292, 0.336, 0.31],
  ],
  taper: [[0.45, 0.31], [0.23, 0.31], [0.17, 0.165], [0.03, 0.087], [-0.15, 0.087]],
  vertical: { x: 0.217, y: 0.241, z: 0.24 },
};
const R_FUEL = (5.91 * IN) / 2;
// the floor's height at x: SQUEEZE under each roller, straight between, dropping after conveyor 1
// to the turret's pickup (the hopper model's floor)
export function conveyorFloor(x) {
  const pts = CONVEYOR.shafts.map(([, sx, sy]) => [sx, sy - IN - 2 * R_FUEL + SQUEEZE]).sort((a, b) => a[0] - b[0]);
  const all = [[-0.15, 0.045], [0.03, 0.06], ...pts, [0.45, pts[pts.length - 1][1]]];
  for (let i = 0; i + 1 < all.length; i++) {
    const [x0, y0] = all[i], [x1, y1] = all[i + 1];
    if (x <= x1) return y0 + ((y1 - y0) * Math.max(0, x - x0)) / (x1 - x0);
  }
  return all[all.length - 1][1];
}

// the wheels, on the trains from build8793() (the horizontal train turning so the rollers' bottoms
// run rearward: its speed is negative)
export function conveyorWheels({ rollers, vshafts }) {
  const wheels = [];
  const w = 1 * IN, flex = { tread: 5000 }, mec = { tread: 20000 };
  // a mecanum's hand: a FUEL under it is pushed toward dz (+1: the robot's right) while the shaft
  // turns at sign s (contact below the axle: grip direction c t1 + s t2, t1 = rearward/forward)
  const hand = (dz, s = -1) => 45 * Math.sign(dz * s);
  const spread = (n, hw) => Array.from({ length: n }, (_, i) => -hw + w / 2 + ((2 * hw - w) * i) / (n - 1));
  for (const [ri, x, y, hw] of CONVEYOR.shafts) {
    const s = rollers[ri], base = { shaft: s, axis: [0, 0, 1], radius: s.diameter / 2, width: w, mu: s.mu };
    const at = (z) => [x, y, z];
    // the bare 1/2in hex shaft between the wheels (FUEL can't pass up through it)
    wheels.push(new Wheel({ shaft: s, name: `${s.name} shaft`, center: at(0), axis: [0, 0, 1], radius: 0.29 * IN, width: 2 * hw, mu: 0.3 }));
    if (ri === 0) {
      // compliant in the middle, two mecanums each side, all pushing to the middle
      spread(5, hw).forEach((z, i) => wheels.push(i === 2
        ? new Wheel({ ...base, ...flex, name: 'conveyor 1 compliant', center: at(z) })
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
    wheels.push(new Wheel({ shaft: vl, name: 'vertical omni', kind: 'omni', center: [v.x, v.y + dy, -v.z], axis: [0, -1, 0], radius: r, width: w, mu: 1.0 }));
    wheels.push(new Wheel({ shaft: vr, name: 'vertical compliant 45A', center: [v.x, v.y + dy, v.z], axis: [0, 1, 0], radius: r, width: w, mu: 1.1, tread: 12000 }));
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

// ------------------------------------------------------------------ the turret indexer
// Under the turntable (their CAD, Turret assembly): two shafts of three 2in 30A flex wheels,
// 1in wide, at x -0.039 and -0.215, 0.226 up, straddling the turret's axis (x -0.127). A Kraken
// X60 (CAN 28, 60 A stator, 40 A supply) drives the front one through a MAXPlanetary 3:1 x 3:1
// and a MAX 90-degree gearbox; an 18t:18t belt takes it to the rear one. Their code runs it at
// 65% duty while shooting (TurretConstants.TURRET_INDEXER_SPEED). Single file off the conveyor,
// FUEL ride a pair of roller rails (two plates 0.108 m apart, a FUEL's width less a bit, so a
// FUEL sits in them 0.055 m over their edges), flat under the front wheels, then up a ramp to
// the rear shaft (the rails hang from it). The front wheels push FUEL back along the rails and up
// the ramp into the gap between the two shafts, 2.5 cm narrower than a FUEL, where both wheels'
// faces run up (so the shafts must turn opposite ways: the belt is routed to reverse it) and
// squeeze it straight up the turret's axis into the shooter.
export const INDEXER = {
  ratio: 9,
  shafts: [[-0.039, 0.226], [-0.215, 0.226]],
  wheelZ: [-0.025, 0.001, 0.026],
  // the rails' top edge (x, y), from the conveyor's end back up the ramp to the rear shaft
  rail: [[0.03, 0.083], [-0.06, 0.083], [-0.08, 0.087], [-0.1, 0.095], [-0.12, 0.106], [-0.14, 0.121], [-0.16, 0.139], [-0.17, 0.159], [-0.18, 0.18], [-0.19, 0.2]],
  railZ: 0.054, railT: 0.006,
};
export const DUTY_INDEXER = 0.65;
export function indexerWheels({ ishafts }) {
  const wheels = [];
  INDEXER.shafts.forEach(([x, y], i) => {
    for (const z of INDEXER.wheelZ) {
      wheels.push(new Wheel({ name: ishafts[i].name, shaft: ishafts[i], center: [x, y, z], axis: [0, 0, 1], radius: 1 * IN, width: 1 * IN, mu: 1.1, tread: 5000 }));
    }
  });
  return wheels;
}

// ------------------------------------------------------------------ the shooter
// On the turret (their CAD, Shooter assembly; turret frame: x along the shot, from the turret's
// axis): a 4in 45A solid roller wheel between two SDS 4in flywheels on one shaft, 0.127 m out from
// the axis and 0.411 m up, and three 2in 80A Stealth wheels on a shaft 0.203 m from it, geared
// to it through 28t:28t spur gears so they turn the other way. A FUEL coming up the turret's axis
// is caught between them (0.9in narrower than a FUEL) and leaves square to the line between their
// centers: the rack (the "hood") swings the 2in wheels around the 4in ones to set the launch
// elevation (their code: rack 18..38 deg, elevation = 90 - rack). From their code
// (ShooterConstants): one Kraken X60 (CAN 27; 160 A stator, 60 A supply), 28:18 to the 4in
// wheels, the 2in wheels at 18/28 of the 4in wheels' speed (so FUEL leave with backspin), the
// 4in shaft's inertia 0.0013 kg·m² (their placeholder); velocity control in volts, kV 0.125 and
// kP 0.4 per motor rotation/s, up to 90 rotations/s.
export const SHOOTER = {
  ratio: 28 / 18, smallPerLarge: 18 / 28, moi: 0.0013,
  large: { x: 0.127, y: 0.411, r: 2 * IN }, small: { r: 1 * IN, d: 0.203 },
  elevation: [52, 72],
  kV: 0.125, kP: 0.4, maxRps: 90,
  // friction against FUEL, set so that their old gearing (both shafts at the motor's speed)
  // shoots the ~0.205 m/s per motor rotation/s their code says they measured with it
  // (tools/mechlab.mjs shooter)
  mu: { roller: 1.4, flywheel: 1.4, stealth: 1.4 },
};
// the 2in shaft's center, turret frame, at launch elevation e (deg)
export function shooterSmallAt(e) {
  const a = (e * Math.PI) / 180, { large: L, small: S } = SHOOTER;
  return [L.x - S.d * Math.sin(a), L.y + S.d * Math.cos(a)];
}
// the wheels, in the robot's frame with the turret (axis at turretX) facing yaw (rad, 0 forward)
export function shooterWheels({ large, small }, e, { turretX = -0.127, yaw = 0, mu = SHOOTER.mu } = {}) {
  const c = Math.cos(yaw), s = Math.sin(yaw);
  const at = (x, y, z) => [turretX + x * c + z * s, y, -x * s + z * c];
  const axis = [s, 0, c];
  const [sx, sy] = shooterSmallAt(e), L = SHOOTER.large;
  return [
    new Wheel({ name: 'shooter 4in 45A roller', shaft: large, center: at(L.x, L.y, 0), axis, radius: L.r, width: 1.5 * IN, mu: mu.roller, tread: 20000 }),
    ...[-1, 1].map((sg) => new Wheel({ name: 'shooter SDS 4in flywheel', shaft: large, center: at(L.x, L.y, sg * 0.038), axis, radius: L.r, width: 0.75 * IN, mu: mu.flywheel })),
    new Wheel({ name: 'shooter 2in 80A Stealth', shaft: small, center: at(sx, sy, 0), axis, radius: SHOOTER.small.r, width: 3 * IN, mu: mu.stealth, tread: 40000 }),
  ];
}
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
