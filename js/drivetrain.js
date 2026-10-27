// Swerve drivetrains: a robot's motors, gear ratio, wheels, current limits, battery and tires
// (Drivetrain: its stated numbers, for the robot cards and the AI), and the four modules that
// drive it in play (SwerveDrive, below: steering, motors and tires on the robot's rigid body).
// Units: SI. Motor constants: Kraken X60 (WCP / CTR Electronics datasheet).
import { GRAVITY, IN } from './constants.js';

export const MOTORS = {
  // FOC commutation: 5800 rpm free, 9.37 N·m / 483 A stall at 12 V
  krakenX60: { free: (5800 / 60) * 2 * Math.PI, stallTorque: 9.37, stallCurrent: 483, freeCurrent: 2, V: 12, rotorInertia: 6.5e-5 },
  // the small Kraken (WCP), FOC: ~7530 rpm free, ~4.1 N·m / ~280 A stall at 12 V (WCP's published
  // curve, approximate); rotor inertias estimated from the motors' size
  krakenX44: { free: (7530 / 60) * 2 * Math.PI, stallTorque: 4.1, stallCurrent: 280, freeCurrent: 1.5, V: 12, rotorInertia: 2.5e-5 },
  // the Falcon 500 (VEX's published curve): 6380 rpm free, 4.69 N·m / 257 A stall at 12 V
  falcon500: { free: (6380 / 60) * 2 * Math.PI, stallTorque: 4.69, stallCurrent: 257, freeCurrent: 1.5, V: 12, rotorInertia: 5e-5 },
};
const V_REST = 12.6;     // a charged battery, resting (V)
const R_BATTERY = 0.02;  // battery internal resistance plus main breaker and wiring (ohm)
const OTHER_LOAD = 25;   // everything else drawing from the battery while driving (A)
const ETA = 0.93;        // gearbox and module efficiency on torque
const TOP_FRACTION = 0.88; // the top speed teams command / the gearing's free speed at 12 V

// Defaults for what a team doesn't state: 4 Kraken X60s on 4in wheels, CTRE's usual 70 A supply
// limit with an 80 A stator limit, tread on carpet
const DEFAULTS = { motor: 'krakenX60', motors: 4, wheel: 4 * IN, stator: 80, supply: 70, mu: 1.3 };

export class Drivetrain {
  // d: cfg.drive ({ ratio, wheel, motor, stator, supply, mu, teleop: { stator, supply } }), m: robot mass (kg),
  // L, W: frame (m), r: the modules' distance from the center (m)
  constructor(d, m, L, W, r) {
    Object.assign(this, DEFAULTS, d);
    this.base = { stator: this.stator, supply: this.supply };
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
    this.topSpeed = TOP_FRACTION * this.freeSpeed;
    this.accel0 = this.force(0).F / m;
    // spinning: the same wheels, at r from the center; I ~ a uniform slab with the bumpers
    this.Iz = (m * ((L + 0.17) ** 2 + (W + 0.17) ** 2)) / 12;
  }

  // the current limits for the match's phase: cfg.drive's own until teleop, then cfg.drive.teleop's
  // over them (what a team's teleopInit applies)
  limitsFor(teleop) { if (this.teleop) Object.assign(this, this.base, teleop ? this.teleop : null); }

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

// ------------------------------------------------------------------ swerve modules
// The drive as four swerve modules on the robot's rigid body, instead of setting its velocity.
// Each module steers its wheel toward where it's commanded: a Kraken X60 through the steering
// gearing, so a 90-degree turn takes a few hundredths of a second, and a module never turns more
// than 90 degrees (past that it runs its wheel backwards). Each drives its wheel through its own
// motor and gearing toward the commanded speed (the teams' velocity loops are stiff), as hard as
// the motor can push or brake at that speed: back-EMF, the stator and supply current limits, and
// the battery sagging under all four. The wheels push on the carpet through the tires' grip, within
// the friction circle (mu x its share of the weight), along the wheel and sideways (a wheel won't
// slide sideways until the tire gives), solved for all four at once against the robot's mass and
// spin inertia, and put on the body as forces for the world's step. So a robot follows its
// wheels: it pushes only the way they point, a change of direction waits on the modules turning,
// and robots shoving each other meet with their motors' torque and their tires' grip.
const STEER = {
  ratio: 150 / 7,  // MK4i-style steering reduction (teams' common choice; X2t's is close)
  accel: 1500,     // rad/s²: how hard the steering loop swings a module (its profile)
  roll: 0.015,     // rolling resistance on carpet, as a share of the weight
  friction: 2,     // N per m/s of wheel speed, all four modules: gears and bearings
};
export class SwerveDrive {
  // drive: the robot's Drivetrain (its motors, gearing, tires); pts: the modules' [x, z] from the
  // robot's center (x forward, z to its right)
  constructor(drive, pts) {
    this.d = drive;
    this.pts = pts;
    this.steerRate = 0.85 * (drive.mo.free / STEER.ratio);
    this.modules = pts.map(() => ({ angle: 0, rate: 0, w: 0, I: 0, slip: false }));
    this.V = 12.4;      // the battery's voltage
    this.current = 0;   // the drive's battery current (A)
  }

  reset() { for (const m of this.modules) Object.assign(m, { angle: 0, rate: 0, w: 0, I: 0, slip: false }); }

  // the motor's torque at the wheel at wheel speed w, flat out (dir +1 forward, -1 back) at battery
  // V, within the stator and supply limits
  _flatOut(w, dir, V) {
    const d = this.d, Eb = (w * d.ratio) / d.kV, I = (dir * V - Eb) / d.Rm;
    const e = Math.sign(I) * Eb;
    const cap = Math.min(d.stator, (-e + Math.sqrt(e * e + 4 * d.Rm * d.supply * V)) / (2 * d.Rm));
    return d.kT * (Math.abs(I) > cap ? Math.sign(I) * cap : I) * d.ratio * ETA;
  }
  // the battery current for wheel torque tau at wheel speed w (negative: braking charges it)
  _supply(w, tau, V) {
    const d = this.d, Ic = tau / (d.kT * d.ratio * ETA), Eb = (w * d.ratio) / d.kV;
    return (Ic * (Eb + Ic * d.Rm)) / V;
  }

  // One step. body: the robot's Rapier body; yaw: its heading; (vx, vz, omega): the wanted
  // field velocity and yaw rate; down: [bool] which wheels are on the floor. Puts the tires'
  // push on the body as a force and a torque for the world's step.
  step(body, yaw, vx, vz, omega, down, dt) {
    const d = this.d, rw = d.rw, n = this.pts.length;
    const c = Math.cos(yaw), s = Math.sin(yaw);
    // robot frame (x forward, z right) <-> field: forward is (cos yaw, -sin yaw)
    const toField = (x, z) => [x * c + z * s, -x * s + z * c];
    const toRobot = (x, z) => [x * c - z * s, x * s + z * c];
    const [rvx, rvz] = toRobot(vx, vz);
    // the modules steer toward their targets
    const want = [];
    for (let i = 0; i < n; i++) {
      const m = this.modules[i], [px, pz] = this.pts[i];
      // the wheel's wanted velocity: the robot's, plus the spin's at the module (w x r, about +y)
      const ux = rvx + omega * pz, uz = rvz - omega * px;
      let sp = Math.hypot(ux, uz);
      m.angle = Math.atan2(Math.sin(m.angle), Math.cos(m.angle));
      const target = sp > 0.02 ? Math.atan2(uz, ux) : m.angle;
      let err = wrap(target - m.angle);
      if (Math.abs(err) > Math.PI / 2) { err = wrap(err + Math.PI); sp = -sp; }
      const goal = m.angle + err;
      const stopRate = Math.sqrt(2 * STEER.accel * Math.abs(err)) * Math.sign(err);
      const rate = Math.max(-this.steerRate, Math.min(this.steerRate, stopRate));
      m.rate += Math.max(-STEER.accel * dt, Math.min(STEER.accel * dt, rate - m.rate));
      const turn = m.rate * dt;
      if (Math.abs(turn) >= Math.abs(err)) { m.angle = goal; m.rate = 0; } else m.angle += turn;
      // the wheel's wanted speed, scaled by how far the module still has to turn (one pointing
      // the wrong way waits)
      want.push(sp * Math.max(0, Math.cos(goal - m.angle)));
    }
    // the tires: each wheel drives the robot toward its wanted speed along it (as hard as its
    // motor can, either way) and holds it from sliding sideways, all within its friction circle
    const lv = body.linvel(), av = body.angvel(), com = body.worldCom(), p = body.translation();
    const M = body.mass(), Iz = body.principalInertia().y;
    let v = [lv.x, lv.z], wy = av.y;
    const nDown = down.filter(Boolean).length;
    const V = this.V;
    const C = [];
    for (let i = 0; i < n; i++) {
      const m = this.modules[i];
      if (!down[i]) { m.slip = false; m.I = 0; continue; }
      const [px, pz] = this.pts[i];
      const [fx, fz] = toField(px, pz);
      const r = [p.x + fx - com.x, p.z + fz - com.z];
      const [lx, lz] = toField(Math.cos(m.angle), Math.sin(m.angle)); // along the wheel
      const k = [[lx, lz], [-lz, lx]].map(([ux, uz]) => {
        const ru = r[1] * ux - r[0] * uz; // r x u about +y
        return { u: [ux, uz], ru, m: 1 / (1 / M + (ru * ru) / Iz), lam: 0 };
      });
      // the motor's reach either way at the wheel's speed, less the drag on it
      const fwd = (this._flatOut(m.w, 1, V) * dt) / rw, back = (this._flatOut(m.w, -1, V) * dt) / rw;
      const drag = ((STEER.roll * d.m * GRAVITY + STEER.friction * Math.abs(m.w * rw)) / n) * dt;
      C.push({ m, r, k, want: want[i], lo: back - drag, hi: fwd + drag, lim: (d.mu * M * GRAVITY * dt) / nDown });
    }
    for (let it = 0; it < 10; it++) {
      for (const ct of C) {
        const vpx = v[0] + wy * ct.r[1], vpz = v[1] - wy * ct.r[0]; // the body's velocity there
        const [a, b] = ct.k;
        let l1 = a.lam - (vpx * a.u[0] + vpz * a.u[1] - ct.want) * a.m;
        let l2 = b.lam - (vpx * b.u[0] + vpz * b.u[1]) * b.m;
        l1 = Math.max(ct.lo, Math.min(ct.hi, l1));
        const mag = Math.hypot(l1, l2);
        ct.slip = mag > ct.lim;
        if (ct.slip) { l1 *= ct.lim / mag; l2 *= ct.lim / mag; }
        for (const [kk, l] of [[a, l1], [b, l2]]) {
          const dl = l - kk.lam;
          kk.lam = l;
          v = [v[0] + (kk.u[0] * dl) / M, v[1] + (kk.u[1] * dl) / M];
          wy += (kk.ru * dl) / Iz;
        }
      }
    }
    // the wheels: rolling with the carpet, or (pushing past their grip) spinning up toward the
    // speed they're driven at; and what their motors draw
    let Isum = 0;
    for (const ct of C) {
      const m = ct.m, [a] = ct.k;
      const ground = (v[0] + wy * ct.r[1]) * a.u[0] + (v[1] - wy * ct.r[0]) * a.u[1];
      const tau = (a.lam * rw) / dt;
      m.w = (ct.slip && Math.abs(ct.want) > Math.abs(ground) ? ground + 0.5 * (ct.want - ground) : ground) / rw;
      m.slip = ct.slip;
      m.I = this._supply(m.w, tau, V);
      Isum += m.I;
    }
    this.current = Isum;
    // the battery sags under the draw (eased, so a step's draw doesn't swing the next)
    this.V += 0.5 * (Math.max(6, V_REST - R_BATTERY * (Isum + OTHER_LOAD)) - this.V);
    // the tires' push, as forces the world's step applies along with everything touching the
    // robot (a robot it's pushing, a wall), so neither side of a shove gets to go first
    body.resetForces(true);
    body.resetTorques(true);
    body.addForce({ x: ((v[0] - lv.x) * M) / dt, y: 0, z: ((v[1] - lv.z) * M) / dt }, true);
    body.addTorque({ x: 0, y: ((wy - av.y) * Iz) / dt, z: 0 }, true);
  }
}
const wrap = (a) => Math.atan2(Math.sin(a), Math.cos(a));
