// A turret on its motor: the motor model (js/mech/mechanism.js Motor: voltage, back-EMF, stator and
// supply limits) through the turret's gearing, under the team's position loop, against the turret's
// inertia, its bearings' and gears' friction and its energy chain (js/echain.js chainLoad*). The
// physical robots' turretStep (js/mech/phys8793.js, phys4414.js) runs one, so the turret gets where
// the aim asks as fast as its motor can take it there, instead of at a set rate (js/robot.js).
import { Motor } from './mechanism.js';
import { TrackChain, SpoolChain, SwivelChain, chainLoad8793, chainLoad4414, chainLoad1690, SWIVEL1690, LINK } from '../echain.js';

const DEG = Math.PI / 180, TURN = 2 * Math.PI;
const sgn = (x) => (x > 0 ? 1 : x < 0 ? -1 : 0);

export class TurretDrive {
  // motor: a Motor (its ratio the turret's); J: what turns with the turret (kg m^2; the motor's rotor
  // is added); friction: its bearings' and gears' (N·m, Coulomb); viscous (N·m s/rad);
  // gains: their Slot0 (volts per motor rotation, rotation/s): kP, kD, kV, kS (kS on the sign the loop
  // drives), ff(yaw, rateRef): an arbitrary feedforward (V); maxRate: the rate feedforward's bound
  // (rad/s); limits: their soft limits [lo, hi] (rad); chain: (yaw, omega) -> { torque, friction,
  // viscous } or null; sub: control steps per step (a Talon closes its loop at 1 kHz)
  constructor({ motor, J, friction = 0, viscous = 0, gains, maxRate = Infinity, limits = [-Infinity, Infinity], chain = null, sub = 8 }) {
    Object.assign(this, { motor, friction, viscous, gains, maxRate, limits, chain, sub });
    this.J = J + motor.J;
    this.yaw = null; this.omega = 0; this.goal = null; this.current = 0; this.load = 0;
  }
  // one of the robot's steps (dt s, at time t): from cur (rad: where the robot has the turret) toward
  // goal (rad); returns where it is after. (Put somewhere else, or not asked for a while: it starts
  // from rest there.)
  step(cur, goal, dt, t = null) {
    const lapse = t !== null && this.t !== undefined && this.t !== null && t - this.t > 1.5 * dt;
    if (this.yaw === null || Math.abs(cur - this.yaw) > 1e-9 || lapse) { this.yaw = cur; this.omega = 0; this.goal = null; }
    this.t = t;
    // the rate the command sweeps at (their aim's lookahead): a jump to a new target isn't a rate
    let rate = 0;
    if (this.goal !== null) { const r = (goal - this.goal) / dt; if (Math.abs(r) <= this.maxRate * 1.5) rate = Math.max(-this.maxRate, Math.min(this.maxRate, r)); }
    this.goal = goal;
    const { kP, kD = 0, kV = 0, kS = 0, ff = null } = this.gains, k = this.motor.ratio / TURN, h = dt / this.sub;
    const [lo, hi] = this.limits;
    for (let i = 0; i < this.sub; i++) {
      const e = (goal - this.yaw) * k, vRef = rate * k, v = this.omega * k;
      const pid = kP * e + kD * (vRef - v);
      let volts = pid + kV * vRef + (Math.abs(e) > 5e-4 ? kS * sgn(pid) : 0) + (ff ? ff(this.yaw, rate) : 0);
      let duty = Math.max(-1, Math.min(1, volts / 12));
      if ((this.yaw >= hi && duty > 0) || (this.yaw <= lo && duty < 0)) duty = 0; // (the soft limits)
      const tm = this.motor.torque(this.omega, duty).tau;
      const L = this.chain ? this.chain(this.yaw, this.omega) : null;
      const Fc = this.friction + (L ? L.friction : 0);
      const ta = tm + (L ? L.torque : 0) - (this.viscous + (L ? L.viscous : 0)) * this.omega;
      // Coulomb friction: it holds still against anything less; moving, it takes up to Fc
      if (this.omega === 0) {
        if (Math.abs(ta) > Fc) this.omega = (h * (ta - sgn(ta) * Fc)) / this.J;
      } else {
        const w = this.omega + (h * (ta - sgn(this.omega) * Fc)) / this.J;
        this.omega = sgn(w) !== sgn(this.omega) ? 0 : w;
      }
      this.yaw += this.omega * h;
      // (its hard stops, a few degrees past the soft ones)
      if (this.yaw > hi + 5 * DEG) { this.yaw = hi + 5 * DEG; this.omega = Math.min(0, this.omega); }
      if (this.yaw < lo - 5 * DEG) { this.yaw = lo - 5 * DEG; this.omega = Math.max(0, this.omega); }
      this.current = this.motor.current; this.load = L ? L.torque - sgn(this.omega) * L.friction : 0;
    }
    return this.yaw;
  }
}

// ---------------------------------------------------------------- 8793
// Their code (Pumpkin-Bots/2026_Robot, ShooterSubsystem): a Kraken X60 (FOC) through 20:200
// (TURRET_ROTATOR_GEAR_RATIO; the CAD's pinion is a 21t), PositionVoltage with kP 4 (their comment:
// "optimal is 12, but too violent, need stronger wiring chain"), kD 0.25, kV 0.25, kS 0.65 V, the
// commanded angle's own rate fed forward (bounded at 360 deg/s), stator 40 A, supply 30 A, soft
// limits -200..270 deg. What turns: the shooter on the 200T turntable, about 5 kg (estimate:
// J 0.06 kg m^2). Friction: their kS, tuned "until the turret just begins to break away from a
// small error", is 0.65 V at a stall: 26 A, 4.6 N·m at the turret, the chain's ~0.3 of it; so the
// bearings and gears ~4.0 N·m once moving (an estimate from their kS, a turret on V-rollers round
// a support ring and a 200T mesh).
export const X60_FOC = 'krakenX60';
export function drive8793({ chain = true } = {}) {
  const ch = new TrackChain();
  return new TurretDrive({
    motor: new Motor(X60_FOC, { ratio: 10, statorLimit: 40, supplyLimit: 30, eff: 0.9 }),
    J: 0.06, friction: 4.0, viscous: 0.02,
    gains: { kP: 4, kD: 0.25, kV: 0.25, kS: 0.65 },
    maxRate: 360 * DEG, limits: [-200 * DEG, 270 * DEG],
    chain: chain ? (yaw) => chainLoad8793(ch, yaw) : null,
  });
}

// ---------------------------------------------------------------- 4414
// Their binder: "55.7:1 overall ratio powered by 1 Kraken X44", "44t idler gear", "Compact bearing
// stack using WCP 21/32in ultra low profile shoulder bolts". Their code isn't public: the loop is an
// estimate like 8793's (kP 1.5 V per motor rotation: ~4 N·m a degree; kD 0.05; kV the X44's 1/Kv
// 0.095; kS 0.09 V for ~1.5 N·m of bearings and gears, estimate) and a feedforward against the
// chain's springs by the turret's angle, as a team measures one into a table (their pull at the arm
// the chain leaves the guides with: 4.1-6.7 N·m, 0.24-0.39 V); stator 40 A, supply 30 A
// (estimates); what turns, the A-frame shooter with its four X44s, copper flywheel and hood (~6 kg,
// J 0.07, estimate); travel +-200 deg (the game's config).
export function drive4414({ chain = true } = {}) {
  const ch = new SpoolChain();
  const motor = new Motor('krakenX44', { ratio: 55.7, statorLimit: 40, supplyLimit: 30, eff: 0.9 });
  const vPerNm = motor.Rm / (motor.ratio * motor.eff * motor.kT);
  return new TurretDrive({
    motor, J: 0.07, friction: 1.5, viscous: 0.02,
    gains: { kP: 1.5, kD: 0.05, kV: 0.095, kS: 0.09, ff: chain ? (yaw) => -chainLoad4414(ch, yaw).torque * vPerNm : null },
    maxRate: 360 * DEG, limits: [-200 * DEG, 200 * DEG],
    chain: chain ? (yaw) => chainLoad4414(ch, yaw) : null,
  });
}

// ---------------------------------------------------------------- 1690
// Their code isn't public. Their CAD release thread: the turret on position control, Motion Magic
// when it wraps, a feedforward of the target's tracking rate less the robot's own; a REV Through Bore
// encoder geared to it, the Kraken's own encoder for the loop. Its CAD (their X_T release): an X60 low
// in the frame (fixed), a 12T steel pinion into a 60T gear (78.8 mm across: 20 DP; their centers 45.8
// mm apart, 72 teeth's worth), an 18T 5M pulley on that shaft, 28T on a vertical shaft beside the
// turret (54.4 mm apart: a 45T belt fits to 0.1 mm; the belt isn't in the X_T), up to a 14T 10 DP
// pinion (40.6 mm across) 0.132 m from the turret's axis on its 90T ring (233.6 mm across): 5 x 1.556
// x 6.43 = 50:1 (~720 deg/s free). The loop as 8793's (an X60): kP 3 V per motor rotation, kD 0.1, kV
// the X60's 1/Kv (0.124), kS 0.15 V, the commanded angle's own rate fed forward (bounded at 360
// deg/s); stator 40 A, supply 30 A; what turns: the shooter on its plates with its two X60s, the
// hood's X44 and the camera (~6 kg, J 0.07); its thin-section 8in bearing ("tight tolerance on its
// slots made it significantly stiffer so we had to scrape it quite a lot until it got smooth"): 1.0
// N m. All estimates; travel +-200 deg (the sim's config). Its chain: js/echain.js SWIVEL1690.
export function drive1690({ chain = true } = {}) {
  const ch = new SwivelChain(SWIVEL1690, LINK);
  return new TurretDrive({
    motor: new Motor('krakenX60', { ratio: 50, statorLimit: 40, supplyLimit: 30, eff: 0.9 }),
    J: 0.07, friction: 1.0, viscous: 0.02,
    gains: { kP: 3, kD: 0.1, kV: 0.124, kS: 0.15 },
    maxRate: 360 * DEG, limits: [-200 * DEG, 200 * DEG],
    chain: chain ? (yaw) => chainLoad1690(ch, yaw) : null,
  });
}

// ---------------------------------------------------------------- 2056
// Their binder: an 8in ID X-contact bearing (Lilly Bearing), a 1/4in 10 DP turret gear, "1x Kraken X44
// on turret rotation, 8:40 - 10:100 reduction" (50:1, ~900 deg/s free), "+/- 185 deg of rotation with
// custom printed energy chain", homed on a hall effect sensor and tracked on the X44's own encoder
// (their thread); Motion Magic to setpoints "with standard PID for continuous targeting". The loop
// as 1690's (kP 3 V per motor rotation, kD 0.1, the X44's 1/Kv 0.096, kS 0.15 V), stator 40 A, supply
// 30 A; what turns (the shooter's plates, its two X60s, the hood's and kicker's X44s: J 0.07), the
// bearing's and chain's friction 1.0 N m: all estimates. The printed chain's own pull isn't modeled.
export function drive2056() {
  return new TurretDrive({
    motor: new Motor('krakenX44', { ratio: 50, statorLimit: 40, supplyLimit: 30, eff: 0.9 }),
    J: 0.07, friction: 1.0, viscous: 0.02,
    gains: { kP: 3, kD: 0.1, kV: 0.096, kS: 0.15 },
    maxRate: 360 * DEG, limits: [-185 * DEG, 185 * DEG],
  });
}
