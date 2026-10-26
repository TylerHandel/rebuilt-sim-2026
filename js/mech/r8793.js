// 8793 (Pumpkin Bots) built as a physical mechanism (js/mech/mechanism.js): their drive chains as
// the team describes them, the roller sizes from their CAD (Onshape "8793-2026-A-0000 Robot":
// Intake V3, Conveyor V2). Positions come later, with the robot.
//
// Horizontal rollers, all on one Kraken X60, slowing stage by stage toward the intake (so FUEL
// speeds up, and spreads out, on its way to the turret): X60 > 18:33 > conveyor roller 1 > 25:24 >
// roller 2 > 24:43 > roller 3 > 23:20 > the intake's first roller (on the arm's pivot) > 20:20 >
// second > 20:20 > third > 20:28 > the last roller, at the arm's tip. (25:24 speeds up a little.)
// Vertical-axis rollers (one omni wheel, one compliant wheel, which still lets FUEL down past
// it): a Kraken X44 on a 5:1 MAXPlanetary > 28:18 (speeds up, to match the rpm) > a 1:1 90-degree
// gearbox, beside the second-to-last horizontal roller.
import { IN } from '../constants.js';
import { Motor, Shaft, Train } from './mechanism.js';

// the gear chains: [name, belt/gear ratio from the shaft before (driver:driven teeth), roller
// diameter (m), what's on it]
export const HORIZONTAL = [
  ['conveyor 1', [18, 33], 2 * IN, '2in flex wheels (30A)'],
  ['conveyor 2', [25, 24], 2 * IN, '2in flex wheels (30A)'],
  ['conveyor 3', [24, 43], 2 * IN, '2in flex / WCP 2in mecanum wheels'],
  ['intake 1 (pivot)', [23, 20], 1.5 * IN, 'silicone roller (OD assumed 1.5in)'],
  ['intake 2', [20, 20], 1.5 * IN, 'silicone roller'],
  ['intake 3', [20, 20], 1.5 * IN, 'silicone roller'],
  ['intake 4 (tip)', [20, 28], 1.5 * IN, 'silicone roller'],
];
export const VERTICAL = { ratio: [[1, 5], [28, 18], [1, 1]], diameter: 4 * IN };

// the trains: each shaft's k = the turns it makes per turn of the motor
export function build8793() {
  const horizontal = new Train({ name: 'horizontal rollers', motor: new Motor('krakenX60', { statorLimit: 60 }) });
  let k = 1;
  const rollers = HORIZONTAL.map(([name, [a, b], d, what]) => {
    k *= a / b;
    const s = horizontal.add(new Shaft({ name, inertia: 6e-5 }), k);
    s.diameter = d; s.what = what;
    return s;
  });
  const vk = VERTICAL.ratio.reduce((p, [a, b]) => (p * a) / b, 1);
  const vertical = new Train({ name: 'vertical rollers', motor: new Motor('krakenX44', { statorLimit: 40 }) });
  const vshaft = vertical.add(new Shaft({ name: 'vertical rollers', inertia: 1e-4 }), vk);
  vshaft.diameter = VERTICAL.diameter;
  return { horizontal, rollers, vertical, vshaft };
}
