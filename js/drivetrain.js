// Swerve drivetrain model: how hard a robot can accelerate at a given speed, from its motors, gear
// ratio, wheels, current limits, battery and tires, instead of a fixed acceleration up to the
// gearing's free speed. What it gives:
//   - off the line: the stator current limit (or the tires) sets the push;
//   - speeding up: the motors' back-EMF and the battery sagging under four motors' current (and
//     the supply current limit) take force away, so acceleration tapers off;
//   - top speed: where what's left balances drag (rolling resistance and the modules' own
//     friction), ~85-90% of the free speed the gearing is quoted at (calibrated to TOP_FRACTION).
// Units: SI. Motor constants: Kraken X60 (WCP / CTR Electronics datasheet).
import { GRAVITY, IN } from './constants.js';

export const MOTORS = {
  // FOC commutation: 5800 rpm free, 9.37 N·m / 483 A stall at 12 V
  krakenX60: { free: (5800 / 60) * 2 * Math.PI, stallTorque: 9.37, stallCurrent: 483, freeCurrent: 2, V: 12 },
};
const V_REST = 12.6;     // a charged battery, resting (V)
const R_BATTERY = 0.02;  // battery internal resistance plus main breaker and wiring (ohm)
const OTHER_LOAD = 25;   // everything else drawing from the battery while driving (A)
const ETA = 0.93;        // gearbox and module efficiency on torque
const TOP_FRACTION = 0.88; // real top speed / the gearing's free speed at 12 V: what teams see

// Defaults for what a team doesn't state: 4 Kraken X60s on 4in wheels, CTRE's usual 70 A supply
// limit with an 80 A stator limit, tread on carpet
const DEFAULTS = { motor: 'krakenX60', motors: 4, wheel: 4 * IN, stator: 80, supply: 70, mu: 1.3 };

export class Drivetrain {
  // d: cfg.drive ({ ratio, wheel, motor, stator, supply, mu }), m: robot mass (kg),
  // L, W: frame (m), r: the modules' distance from the center (m)
  constructor(d, m, L, W, r) {
    Object.assign(this, DEFAULTS, d);
    this.m = m;
    this.r = r;
    const mo = MOTORS[this.motor];
    this.mo = mo;
    this.Rm = mo.V / mo.stallCurrent;             // winding resistance
    this.kT = mo.stallTorque / mo.stallCurrent;   // N·m per A
    this.kV = mo.free / (mo.V - mo.freeCurrent * this.Rm); // rad/s per volt of back-EMF
    this.rw = this.wheel / 2;
    this.freeSpeed = (mo.free / this.ratio) * this.rw; // at 12 V, what the gearing is quoted at
    this.grip = this.mu * GRAVITY * m;              // most the tires push with
    // drag: rolling resistance plus the modules' friction, growing with speed; calibrated so the
    // robot tops out at TOP_FRACTION of its free speed
    this.drag0 = 0.02 * m * GRAVITY;
    const vTop = TOP_FRACTION * this.freeSpeed;
    this.dragK = Math.max(0, (this.force(vTop).F - this.drag0) / vTop);
    this.topSpeed = vTop;
    this.accel0 = (this.force(0).F - this.drag0) / m;
    // the force curve, tabulated for play (every 5 cm/s up to past the free speed)
    this.step = 0.05;
    this.table = Float32Array.from({ length: Math.ceil((1.2 * this.freeSpeed) / this.step) + 2 }, (_, i) => this.force(i * this.step).F);
    // spinning: the same wheels, at r from the center; I ~ a uniform slab with the bumpers
    this.Iz = (m * ((L + 0.17) ** 2 + (W + 0.17) ** 2)) / 12;
  }

  // The motors' total push at speed v (m/s, >= 0), full throttle: { F (N), V (battery volts),
  // I (battery amps) }. The battery sags with the current drawn and the current depends on the
  // voltage, so find the voltage where they agree (bisection: more voltage draws more current).
  force(v) {
    const n = this.motors;
    const back = ((v / this.rw) * this.ratio) / this.kV; // back-EMF (V)
    const draw = (Vb) => {
      // stator current: the voltage left after back-EMF through the winding, up to the limit
      let Is = Math.max(0, Math.min(this.stator, (Vb - back) / this.Rm));
      // supply current from power in = power out + copper loss; the supply limit caps it
      let Isup = (Is * (back + Is * this.Rm)) / Vb;
      if (Isup > this.supply) { Is *= this.supply / Isup; Isup = this.supply; }
      return { Is, I: n * Isup + OTHER_LOAD };
    };
    let lo = 4, hi = V_REST;
    for (let k = 0; k < 30; k++) {
      const mid = (lo + hi) / 2;
      if (V_REST - R_BATTERY * draw(mid).I > mid) lo = mid; else hi = mid;
    }
    const Vb = (lo + hi) / 2, { Is, I } = draw(Vb);
    const F = (n * this.kT * Is * this.ratio * ETA) / this.rw;
    return { F: Math.min(F, this.grip), V: Vb, I };
  }

  drag(v) { return this.drag0 + this.dragK * v; }

  // force(v).F from the table
  forceAt(v) {
    const t = this.table, x = Math.max(0, v) / this.step, i = Math.min(t.length - 2, Math.floor(x)), f = Math.min(1, x - i);
    return t[i] + (t[i + 1] - t[i]) * f;
  }

  // Most the speed can change this step, toward a target: speeding up takes what the motors have
  // left over drag; slowing down has the motors braking (up to the tires) plus drag.
  maxDv(v, speedingUp, dt) {
    if (speedingUp) return Math.max(0, (this.forceAt(v) - this.drag(v)) / this.m) * dt;
    return ((this.grip + this.drag(v)) / this.m) * dt;
  }

  // Stated numbers for the robot card and the AI: top speed, turn rate, spin-up
  stats() {
    const vT = this.topSpeed;
    return {
      maxSpeed: vT,
      maxAccel: this.accel0,
      maxOmega: vT / this.r,
      maxAlpha: (this.grip * this.r) / this.Iz,
    };
  }
}
