// What a robot's code knows of where it is: a pose estimate, the one its aim works from (Robot.aimPt,
// estVel), between the truth (the physics) and every aim (Robot._shooter, js/mech/phys*.js). Real
// robots don't know their pose: they integrate wheel odometry with a gyro's heading, correct it with
// AprilTag vision at the cameras' rate and latency, blend the two in a filter, and act on that
// estimate a loop later. Each step here does the same from the simulated hardware:
//   - Odometry: the swerve modules' own wheel speeds and angles (drivetrain.js SwerveDrive: rolling
//     with the carpet, or spinning past their grip, and blind to the chassis sliding sideways when
//     it's pushed or bumps something, or a wheel off the floor on the BUMP), through a wheel radius
//     off by tread wear and module angle offsets, forward kinematics (least squares), and the gyro.
//   - Gyro (a Pigeon 2.0 for all of them): the true heading plus a bias drift, a random walk, a scale
//     error (per turn) and noise.
//   - Vision: each camera, at its frame rate, sees the field's AprilTags (constants.js APRILTAGS,
//     WPILib's 2026 welded layout) that are inside its field of view (mount, FOV; on a turret, turning
//     with it), facing it, big enough in the image and not hidden behind a robot or a HUB. Each tag's
//     bearing and range are measured with pixel noise (more with motion blur, and a rolling
//     shutter's skew on an LL3), a mount yaw and lens-scale calibration error, and the tags' own
//     placement tolerance; then the robot's pose is solved from them the way that team's pipeline
//     does: MegaTag2 (position, the heading given by the robot's estimate), PnP / multi-tag (full
//     pose), or 2910's own (one tag's pixel height for distance, its tx and the gyro). The result
//     reaches the robot code its latency later, timestamped (with jitter).
//   - Filter: what that team's code does with it: the closed-form Kalman blend of WPILib's /
//     AdvantageKit's pose estimators, K = q / (q + sqrt(q r)) per axis, with its own odometry (q)
//     and vision std devs (r, from distance and tag count) and rejections, applied at the frame's
//     timestamp against the odometry buffer; or 2910's translation reset to the vision pose
//     (stale: no latency compensation, as their own TODO says).
//   - Actuation: the aim uses the estimate as it was ACT_DELAY ago (a 50 Hz loop's estimate,
//     sent over CAN to the motor controllers), with the velocity the wheels measure (for
//     shoot-on-the-move).
// The aim is done from the truth with the target moved: the point that sits from the robot's true
// pose where the target sits from its estimated pose (Robot.aimPt), and the velocity as measured, in
// the true frame. That's exact for everything the aims compute (their geometry is relative to the
// robot), and keeps every aim's own code as it was.
//
// AIMTRUTH=1 (or ?aimtruth in the page's URL) turns it off: the aims see the truth, as before (for
// the benches and tools that fit shot tables: those must be fit against truth).
//
// Deterministic: each robot draws from its own seeded generator (the match's seed, SIM_SEED, set by
// tools/headless.mjs, its slot and team), never Math.random, so the match's own random stream (shot
// scatter, FUEL bounces) is the same with it on or off.
//
// Calibration (the noise is physical, not fit to an accuracy):
//   - Corner noise 0.2-0.4 px (global-shutter OV9281/OV2311 cameras, an LL4's; the LL3's rolling-
//     shutter sensor noisier): PhotonVision's calibration docs take a mean reprojection error under
//     ~0.5 px as a good calibration; the AprilTag 3 detector refines edges to sub-pixel. Range from a
//     tag's size then has sd ~ r^2 px / (f s): ~2-4 cm at 4 m on an LL4 (f 736 px), 8-15 cm at 6 m
//     with one tag, as teams' own distance-squared std devs say (6328 0.01 d^2/n^2, 341 0.025, 581
//     0.01 d^0.8 for MegaTag2) and Abbas et al., "Analysis and Improvements in AprilTag Based State
//     Estimation" (Sensors 2019): error grows with distance and the view's obliqueness.
//   - Mount yaw 0.25 deg, lens scale 0.5% (calibration), tags placed to 5 mm (the field's welded
//     tolerance), turret angle 0.3 deg and its timing 3 ms (341 buffers its turret angle by timestamp).
//   - Latency (capture to robot code) 28-45 ms (Limelight's own tl + cl; PhotonVision/Northstar
//     similar), timestamp jitter 2 ms (1678's CameraIOConfig: latencyStdDevs 5 ms).
//   - Odometry: wheel radius 0.5% (tread wears 1-2% over an event; teams re-measure), module angle
//     offsets 0.3 deg, wheel speed noise 0.02 m/s.
//   - Gyro: bias 0.1 deg/min, random walk 0.05 deg/sqrt(min), scale 0.1% (after the Pigeon 2's
//     boot calibration; teams report it "really low drift", Chief Delphi "Gyro drift", 2025; these
//     are estimates).
// Sources for each team's setup are at its profile below. Links: Abbas et al. 2019,
// https://pmc.ncbi.nlm.nih.gov/articles/PMC6960891/ ; PhotonVision multi-tag and calibration docs,
// https://docs.photonvision.org/en/latest/docs/apriltag-pipelines/multitag.html ; Limelight's
// MegaTag2 docs (docs.limelightvision.io, "MegaTag2": its setVisionMeasurementStdDevs(.7, .7, 9999999)
// example); the gyro thread, https://www.chiefdelphi.com/t/gyro-drift/503380 ; std-dev tuning
// discussions, https://www.chiefdelphi.com/t/vision-filtering/520684 .
import { APRILTAGS, TAG_SIZE, HALF_L, HALF_W, HUB } from './constants.js';
import { Field } from './field.js';

const DEG = Math.PI / 180;
const wrap = (a) => Math.atan2(Math.sin(a), Math.cos(a));
const env = globalThis.process?.env;
// (read when each robot is made, so a tool can set it before it builds its game)
export const aimTruth = () => globalThis.process?.env?.AIMTRUTH === '1' || (typeof location !== 'undefined' && new URLSearchParams(location.search).has('aimtruth'));
const LOOP = 0.02;          // robot code's loop (s)
const AIR_T = 0.3;          // s off the floor that a wheel spins at its setpoint (a BUMP's crest)
const SPIN_T = 0.25;        // s barely moving with its wheels driven: stalled, its wheels spinning
const ACT_DELAY = Number(env?.AIM_DELAY ?? 2); // physics steps between the estimate the aim uses and now (~17 ms)

function mulberry32(seed) {
  let a = seed >>> 0;
  return () => {
    a = (a + 0x6d2b79f5) >>> 0;
    let t = a;
    t = Math.imul(t ^ (t >>> 15), t | 1);
    t ^= t + Math.imul(t ^ (t >>> 7), t | 61);
    return ((t ^ (t >>> 14)) >>> 0) / 4294967296;
  };
}
const hash = (...xs) => xs.reduce((h, x) => { for (const c of String(x)) h = Math.imul(h ^ c.charCodeAt(0), 16777619); return h >>> 0; }, 2166136261);

// The tags in the estimator's frame (X = world x, Y = -world z: a right-handed plane with headings
// CCW from +X, the sim's yaw)
const TAGS = APRILTAGS.map(([id, fx, fy, h, deg]) => ({ id, X: fx - HALF_L, Y: fy - HALF_W, h, th: deg * DEG }));

// ---------------------------------------------------------------- cameras
// f: focal length (px), fov (deg), fps, lat: capture to robot code (s), px: corner noise (px), exp:
// exposure (s, for blur), rs: rolling-shutter readout (s; 0 global shutter), minPx: smallest tag found
const CAMS = {
  // Limelight 4: 1280x800 global shutter, 82 x 56.2 deg (Limelight's spec)
  ll4: { f: 736, hfov: 82, vfov: 56.2, fps: 50, lat: 0.028, px: 0.25, exp: 0.0012, rs: 0, minPx: 14 },
  // Limelight 3 at 1280x960 (2910's own pixel-height table implies f ~1050 px), rolling shutter
  ll3: { f: 1050, hfov: 62.5, vfov: 48.9, fps: 22, lat: 0.045, px: 0.4, exp: 0.0015, rs: 0.03, minPx: 16 },
  // 6328's Northstar: OV2311 at 1800x1200, 75 deg (their VisionConstants), and 1280-wide at 90 deg
  ns1800: { f: 1173, hfov: 75, vfov: 54, fps: 50, lat: 0.035, px: 0.2, exp: 0.0018, rs: 0, minPx: 14 },
  ns1280: { f: 640, hfov: 90, vfov: 64, fps: 50, lat: 0.035, px: 0.2, exp: 0.0018, rs: 0, minPx: 14 },
  // OV9281 1280x800 on PhotonVision (604's intrinsics: f 917 px)
  ov9281: { f: 917, hfov: 70, vfov: 47, fps: 40, lat: 0.032, px: 0.25, exp: 0.0015, rs: 0, minPx: 14 },
  // 1678's side cameras: 800x600, 82 deg (their SimulatedCameraIOConfig)
  ov800: { f: 460, hfov: 82, vfov: 66, fps: 40, lat: 0.032, px: 0.3, exp: 0.0015, rs: 0, minPx: 12 },
};

// ---------------------------------------------------------------- each team's localization
// cams: [{ cam, x, y (left), h, yaw, pitch (deg, up), turret (index: on that turret, x/y from its axis
// along the shot) }], mode: 'mt2' | 'pnp' | 'tagdist' (2910); fuse: 'kalman' | 'reset'; q: odometry
// std devs [x, y, theta]; std(d, n, single): [xy, theta] for a vision pose; reject(o): its own filters
const CTRE_Q = [0.1, 0.1, 0.1]; // CTRE SwerveDrivetrain's default odometry std devs
const AK_Q = [0.003, 0.003, 0.002]; // AdvantageKit's RobotState (6328, 341)
const LL_DOCS_STD = () => [0.7, 9999999]; // the Limelight docs' MegaTag2 example, what most copy
const PROFILES = {
  // 2910 (FRCTeam2910/2026CompetitionRobot-Public): one Limelight (ReBlitz: 13.25in back, 15.5in up,
  // 15 deg), VisionIOLimelight: the primary tag's pixel height through their VisionInterpolation table
  // for distance, its tx (/1.21) plus the gyro heading for direction, from that tag's pose; and
  // SuperStructure resets the drive's translation to it every loop it sees one, unless latched (once
  // ready to score with an observation, until it isn't): "doesn't compensate for the latency" (their
  // TODO). Heading: the Pigeon alone.
  '2910': { cams: [{ cam: 'll3', x: -0.34, y: 0, h: 0.39, yaw: 180, pitch: 15 }], mode: 'tagdist', fuse: 'reset', q: CTRE_Q },
  // 6328 (Mechanical-Advantage/RobotCode2026Public, VisionConstants DARWIN): Northstar on the back
  // (1800x1200, 75 deg, 22.5 deg up) and both sides (1280, 90 deg, 70 deg out); the slam's camera only
  // if the back one is out. Vision: xy 0.01 d^2 / n^2, theta 0.03 d^2 / n^2 from multi-tag only;
  // RobotState: odometry std devs 0.003/0.003/0.002, its translation scaled down as the robot tilts
  // (1 - tilt / 25 deg: the BUMP).
  '6328': {
    cams: [
      { cam: 'ns1800', x: -0.324, y: 0, h: 0.351, yaw: 180, pitch: 22.5 },
      { cam: 'ns1280', x: -0.171, y: 0.317, h: 0.435, yaw: 70, pitch: 12 },
      { cam: 'ns1280', x: -0.158, y: -0.317, h: 0.435, yaw: -70, pitch: 12 },
    ],
    mode: 'pnp', fuse: 'kalman', q: AK_Q, odoTilt: true, maxAmb: 0.4,
    std: (d, n) => [0.01 * d * d / (n * n), n > 1 ? 0.03 * d * d / (n * n) : Infinity],
  },
  // 341 (Team341/FRC2026-Public): "limelight-turret" on the turret (8.25in out along it), Vision:
  // MegaTag2 rejected, MegaTag1 single tags rejected: multi-tag MegaTag1 only, xy 0.025 d^2 / n^2,
  // theta never; RobotState odometry 0.003/0.003/0.002; the camera's pose from the turret's angle at
  // the frame's timestamp.
  '341': { cams: [{ cam: 'll4', turret: 0, x: 0.21, y: 0, h: 0.62, yaw: 0, pitch: 28 }], mode: 'pnp', fuse: 'kalman', q: AK_Q, minTags: 2, std: (d, n) => [0.025 * d * d / (n * n), Infinity] },
  // 1678 (frc1678/C2026-Public, CamerasConstants): a Limelight on the back (12.9in back, 14.2in up,
  // 20 deg, MegaTag2) and two 800x600 PhotonVision cameras out the sides (multi-tag, else
  // trig-solve on one tag with the heading; tags past 5.5 m rejected); std devs (0.3, 0.3, -)
  // x the average tag distance into CTRE's estimator; vision only while the pitch is stable.
  '1678': {
    cams: [
      { cam: 'll4', x: -0.328, y: 0, h: 0.36, yaw: 180, pitch: 20, mode: 'mt2' },
      { cam: 'ov800', x: -0.064, y: 0.342, h: 0.334, yaw: 90, pitch: 0, mode: 'pnp1h', maxTag: 5.5 },
      { cam: 'ov800', x: -0.064, y: -0.342, h: 0.334, yaw: -90, pitch: 0, mode: 'pnp1h', maxTag: 5.5 },
    ],
    mode: 'mt2', fuse: 'kalman', q: CTRE_Q, pitchGate: 3, std: (d) => [0.3 * d, Infinity],
  },
  // 8793 (Pumpkin-Bots/2026_Robot, VisionConstants): four PhotonVision cameras at the corners (45 and
  // 135 deg out, 11 deg), multi-tag on the coprocessor, else the lowest-ambiguity single tag (<= 0.2);
  // the alliance walls' tags ignored, past 6 m rejected; std devs multi (0.5, 0.5, 1.0), single (4, 4,
  // 8), x (1 + d^2 / 30), into CTRE's estimator (theta too).
  '8793': {
    cams: [
      { cam: 'ov9281', x: -0.305, y: 0.178, h: 0.340, yaw: 135, pitch: 11 },
      { cam: 'ov9281', x: -0.305, y: -0.178, h: 0.340, yaw: -135, pitch: 11 },
      { cam: 'ov9281', x: 0.165, y: -0.314, h: 0.381, yaw: -45, pitch: 11 },
      { cam: 'ov9281', x: 0.165, y: 0.314, h: 0.381, yaw: 45, pitch: 11 },
    ],
    mode: 'pnp', fuse: 'kalman', q: CTRE_Q, maxAmb: 0.2, maxDist: 6, ignore: [13, 14, 15, 16, 29, 30, 31, 32],
    std: (d, n) => { const k = 1 + d * d / 30; return n > 1 ? [0.5 * k, 1.0 * k] : [4 * k, 8 * k]; },
  },
  // 581 (team581/frc-2026, comp-bot CameraConfigs, Limelight.java): three Limelight 4s, MegaTag2: the
  // shooter's (back, 20 deg up) and one out each side (15 deg); xy 0.01 d^0.8 (theta from MegaTag1
  // only up close: not modeled), rejected spinning past 100 deg/s; CTRE's estimator.
  '581': {
    cams: [
      { cam: 'll4', x: -0.321, y: 0, h: 0.410, yaw: 180, pitch: 20 },
      { cam: 'll4', x: -0.191, y: 0.353, h: 0.422, yaw: 90, pitch: 15 },
      { cam: 'll4', x: -0.191, y: -0.353, h: 0.422, yaw: -90, pitch: 15 },
    ],
    mode: 'mt2', fuse: 'kalman', q: CTRE_Q, maxOmega: 100, std: (d) => [0.01 * d ** 0.8, Infinity],
  },
  // 604 (frc604/2026-public, Constants.Cameras): four 1280x800 cameras (left, back-left 150, back-right
  // -150, right; 26in up, level) into their coprocessor's factor-graph localizer (QuixSwerveLocalizer:
  // tag corners and odometry, smoothed): modeled as a tight Kalman blend weighting multi-tag poses like
  // 6328's.
  '604': {
    cams: [
      { cam: 'ov9281', x: -0.216, y: 0.289, h: 0.660, yaw: 90, pitch: 0 },
      { cam: 'ov9281', x: -0.330, y: 0.025, h: 0.648, yaw: 150, pitch: 0 },
      { cam: 'ov9281', x: -0.330, y: -0.025, h: 0.648, yaw: -150, pitch: 0 },
      { cam: 'ov9281', x: -0.216, y: -0.289, h: 0.660, yaw: -90, pitch: 0 },
    ],
    mode: 'pnp', fuse: 'kalman', q: AK_Q, maxAmb: 0.3, std: (d, n) => [0.01 * d * d / (n * n), n > 1 ? 0.03 * d * d / (n * n) : Infinity],
  },
  // 4414 (its binder: a Limelight 4 on the turret, just ahead of the flywheel, looking along the shot
  // and a little up): MegaTag2 at the Limelight docs' std devs, CTRE's estimator.
  '4414': { cams: [{ cam: 'll4', turret: 0, x: 0.15, y: 0, h: 0.62, yaw: 0, pitch: 15 }], mode: 'mt2', fuse: 'kalman', q: CTRE_Q, std: LL_DOCS_STD },
  // 1690 and 2056: a Limelight on the turret (their models': 1690's, 2056's a Limelight 4), the same.
  '1690': { cams: [{ cam: 'll4', turret: 0, x: 0.1, y: 0, h: 0.75, yaw: 0, pitch: 17 }], mode: 'mt2', fuse: 'kalman', q: CTRE_Q, std: LL_DOCS_STD },
  '2056': { cams: [{ cam: 'll4', turret: 0, x: 0.1, y: 0, h: 0.85, yaw: 0, pitch: 14 }], mode: 'mt2', fuse: 'kalman', q: CTRE_Q, std: LL_DOCS_STD },
};
// Anyone else (1114, 1706, 7769, ...: their localization not looked up): two Limelight 4s on
// MegaTag2, one looking where the shooter does (a turret's: back), one the other way, 20 deg up, at
// the Limelight docs' std devs into CTRE's estimator: the common 2026 setup.
function defaultProfile(cfg) {
  const sh = cfg.shooter, back = sh.type !== 'fixed' || sh.facing === 'back';
  const L = cfg.frame.length / 2 - 0.05;
  return {
    cams: [
      { cam: 'll4', x: back ? -L : L, y: 0, h: 0.4, yaw: back ? 180 : 0, pitch: 20 },
      { cam: 'll4', x: back ? L : -L, y: 0, h: 0.4, yaw: back ? 0 : 180, pitch: 20 },
    ],
    mode: 'mt2', fuse: 'kalman', q: CTRE_Q, std: LL_DOCS_STD,
  };
}
export const profileFor = (cfg) => PROFILES[cfg.physical ?? cfg.key] || defaultProfile(cfg);

const SIGMA = {
  mountYaw: 0.25 * DEG, scale: 0.005, tag: 0.005, turretAng: 0.3 * DEG, turretT: 0.003, tsJitter: 0.002, latJitter: 0.004,
  wheelR: 0.005, modAngle: 0.3 * DEG, wheelV: 0.02,
  gyroBias: (0.1 * DEG) / 60, gyroWalk: (0.05 * DEG) / Math.sqrt(60), gyroScale: 0.001, gyroNoise: 0.01 * DEG,
  start: 0.02, startYaw: 0.5 * DEG, // placed on its starting line by hand; the auto resets to its start pose
};

// ---------------------------------------------------------------- 2D poses (X, Y, th)
const compose = (a, b) => ({ X: a.X + Math.cos(a.th) * b.X - Math.sin(a.th) * b.Y, Y: a.Y + Math.sin(a.th) * b.X + Math.cos(a.th) * b.Y, th: wrap(a.th + b.th) });
const between = (a, b) => { // a^-1 b
  const dx = b.X - a.X, dy = b.Y - a.Y, c = Math.cos(a.th), s = Math.sin(a.th);
  return { X: c * dx + s * dy, Y: -s * dx + c * dy, th: wrap(b.th - a.th) };
};

export class Localizer {
  constructor(robot) {
    this.r = robot;
    this.truth = aimTruth(); // (and a robot built alone, never init'ed by a game: the truth)
    this.prof = profileFor(robot.cfg);
    this.ready = false;
  }

  // once the match's robots are made (game.js createGame): their seeded generator and calibration
  init(robots, slot) {
    const r = this.r, seed = hash(globalThis.SIM_SEED ?? (globalThis.process ? 0 : Date.now()), slot, r.cfg.key, r.alliance);
    this.rng = mulberry32(seed);
    this.others = robots.filter((o) => o !== r);
    const g = () => this.gauss();
    this.cal = {
      cams: this.prof.cams.map((c) => ({ yaw: g() * SIGMA.mountYaw, scale: g() * SIGMA.scale, turret: g() * SIGMA.turretAng, phase: this.rng() })),
      tags: TAGS.map(() => ({ X: g() * SIGMA.tag, Y: g() * SIGMA.tag })),
      wheelR: 1 + g() * SIGMA.wheelR,
      mod: r.swerve.modules.map(() => g() * SIGMA.modAngle),
      gyroBias: g() * SIGMA.gyroBias, gyroScale: g() * SIGMA.gyroScale,
      loop: this.rng() * LOOP,
    };
    this.reset();
    this.ready = true;
  }

  gauss() {
    let u = 0, v = 0;
    while (u === 0) u = this.rng();
    v = this.rng();
    return Math.sqrt(-2 * Math.log(u)) * Math.cos(2 * Math.PI * v);
  }

  _truth() { const r = this.r; return { X: r.pos.x, Y: -r.pos.z, th: r.yaw }; }

  // (spawned: the robot's code is told its start pose, as an auto's reset does, to its placement's
  // tolerance)
  reset() {
    if (!this.rng) return;
    const T = this._truth(), g = () => this.gauss();
    this.est = { X: T.X + g() * SIGMA.start, Y: T.Y + g() * SIGMA.start, th: wrap(T.th + g() * SIGMA.startYaw) };
    this.odo = { ...this.est };
    this.gyroErr = this.est.th - T.th;
    this.lastYaw = T.th;
    this.buf = [];            // odometry poses by time, for vision at its timestamp
    this.hist = [];           // what the aim reads, ACT_DELAY steps late
    this.pending = [];        // vision results on their way to the robot code
    this.camT = this.prof.cams.map((c, i) => this.cal.cams[i].phase / (CAMS[c.cam].fps));
    this.v = { f: 0, l: 0, w: 0 };
    this.latched = false;
    this.air = this.r.swerve.modules.map(() => 0);
    this.lastTurret = this.r.turretYaws?.slice() ?? [];
    this.t = null;
    this.stats = { frames: 0, used: 0 };
  }

  // each physics step, after the robot's pose is synced (Robot.postStep)
  step(dt, t) {
    if (!this.ready || this.truth) return;
    if (this.t === null) this.t = t;
    const r = this.r, T = this._truth();
    // ---- gyro
    const dYaw = wrap(T.th - this.lastYaw);
    this.lastYaw = T.th;
    const c = this.cal;
    this.gyroErr += c.gyroBias * dt + c.gyroScale * dYaw + this.gauss() * SIGMA.gyroWalk * Math.sqrt(dt);
    const gyro = wrap(T.th + this.gyroErr + this.gauss() * SIGMA.gyroNoise);
    const dth = wrap(gyro - (this.gyroPrev ?? gyro));
    this.gyroPrev = gyro;
    // ---- odometry: the modules' wheel speeds and angles, forward kinematics (robot frame: x forward,
    // z right in the sim's; out as forward / left)
    const sw = r.swerve;
    let a11 = 0, a12 = 0, a13 = 0, a22 = 0, a23 = 0, a33 = 0, b1 = 0, b2 = 0, b3 = 0;
    // A wheel past its grip (drivetrain.js: its tire saturated) reads faster than the carpet goes by:
    // while the robot moves, by a tire's slip at peak traction (up to ~7% of the speed, and 0.05 m/s);
    // stalled against something (pushing: barely moving for SPIN_T with its wheels driven), scrubbing
    // round at up to 0.25 m/s. (The sim's own wheel speed past grip, half way to what's asked, is
    // for the motors' current: as odometry it ran off meters a minute)
    const sp = Math.hypot(r.vel.x, r.vel.z);
    const driven = sw.modules.some((m) => Math.abs(m.w * sw.d.rw) > 1);
    this.stallT = sp < 0.3 && driven ? (this.stallT ?? 0) + dt : 0;
    const cy = Math.cos(r.yaw), sy = Math.sin(r.yaw);
    const rvx = r.vel.x * cy - r.vel.z * sy, rvz = r.vel.x * sy + r.vel.z * cy; // robot frame (x fwd, z right)
    sw.modules.forEach((m, i) => {
      const [px, pz] = sw.pts[i];
      let w = m.w * sw.d.rw;
      const ground = (rvx + r.omega * pz) * Math.cos(m.angle) + (rvz - r.omega * px) * Math.sin(m.angle);
      const air = (this.air[i] = sw.down && !sw.down[i] ? this.air[i] + dt : 0);
      // off the floor (over a BUMP's crest, landing): spun freely to its setpoint. (Longer than
      // AIR_T it isn't a wheel in the air but the sim's floor test missing it under a robot riding
      // on its frame: read as rolling)
      if (air > 0) w = air < AIR_T ? sw.want[i] : ground;
      else if (m.slip) {
        const cap = 0.07 * Math.abs(ground) + (this.stallT < SPIN_T ? 0.05 : 0.25);
        w = ground + Math.max(-cap, Math.min(cap, w - ground));
      }
      const a = m.angle + c.mod[i], s = w * c.wheelR + this.gauss() * SIGMA.wheelV;
      const ux = s * Math.cos(a), uz = s * Math.sin(a);
      // rows [1 0 pz] -> ux, [0 1 -px] -> uz
      a11 += 1; a13 += pz; a22 += 1; a23 -= px; a33 += pz * pz + px * px;
      b1 += ux; b2 += uz; b3 += pz * ux - px * uz;
    });
    const [vx, vz] = solve3(a11, a12, a13, a22, a23, a33, b1, b2, b3);
    let vf = vx, vl = -vz;
    this.v = { f: vf, l: vl, w: dt > 0 ? dth / dt : 0 };
    // (6328's: translation scaled down as the robot tilts, 1 - tilt / 25 deg)
    if (this.prof.odoTilt) { const k = 1 - Math.min(1, this._tilt() / (25 * DEG)); vf *= k; vl *= k; }
    const step = (p) => {
      const hm = p.th + dth / 2;
      p.X += (Math.cos(hm) * vf - Math.sin(hm) * vl) * dt;
      p.Y += (Math.sin(hm) * vf + Math.cos(hm) * vl) * dt;
      p.th = wrap(p.th + dth);
    };
    step(this.odo); step(this.est);
    this.buf.push({ t, ...this.odo });
    while (this.buf.length && this.buf[0].t < t - 1.5) this.buf.shift();
    // ---- cameras
    this.prof.cams.forEach((cm, i) => {
      const C = CAMS[cm.cam];
      this.camT[i] -= dt;
      if (this.camT[i] > 0) return;
      this.camT[i] += 1 / C.fps;
      const m = this._frame(cm, i, t, T, dt);
      if (m) this.pending.push(m);
    });
    // ---- vision results reaching the robot code
    if (this.pending.length) {
      this.pending.sort((a, b) => a.due - b.due);
      while (this.pending.length && this.pending[0].due <= t) this._apply(this.pending.shift(), t);
    }
    // 2910: latched while it's ready to score (SuperStructure isReadyToScore with an observation)
    this.latched = !!(r.ready && r.wantShoot);
    // ---- what the aim reads, ACT_DELAY steps late
    this.hist.push({ X: this.est.X, Y: this.est.Y, th: this.est.th, f: this.v.f, l: this.v.l, w: this.v.w });
    if (this.hist.length > ACT_DELAY + 1) this.hist.shift();
    this.lastTurret = r.turretYaws?.slice() ?? [];
  }

  _tilt() { const q = this.r.quat; return Math.acos(Math.min(1, 1 - 2 * (q.x * q.x + q.z * q.z))); }

  // one camera frame: the tags it sees and the pose its pipeline makes of them, or null
  _frame(cm, ci, t, T, dt) {
    const r = this.r, C = CAMS[cm.cam], P = this.prof, cal = this.cal.cams[ci];
    this.stats.frames++;
    // the camera's pose in the robot frame (forward, left), on a turret along its shot
    let cx = cm.x, cy = cm.y, cyaw = cm.yaw * DEG, tRate = 0;
    if (cm.turret !== undefined) {
      const tt = r.turrets[cm.turret], ty = r.turretYaws[cm.turret] ?? 0;
      tRate = dt > 0 ? wrap(ty - (this.lastTurret[cm.turret] ?? ty)) / dt : 0;
      cx = tt.x + Math.cos(ty) * cm.x - Math.sin(ty) * cm.y;
      cy = -tt.z + Math.sin(ty) * cm.x + Math.cos(ty) * cm.y;
      cyaw = ty + cm.yaw * DEG;
    }
    const camWorld = (p, yawErr = 0) => ({ X: p.X + Math.cos(p.th) * cx - Math.sin(p.th) * cy, Y: p.Y + Math.sin(p.th) * cx + Math.cos(p.th) * cy, th: p.th + cyaw + yawErr });
    const cam = camWorld(T);
    const tilt = this._tilt();
    if (P.pitchGate && tilt > P.pitchGate * DEG) return null;
    const wCam = Math.abs(r.omega + tRate);
    const sp = Math.hypot(r.vel.x, r.vel.z);
    const obs = [];
    for (let k = 0; k < TAGS.length; k++) {
      const tg = TAGS[k];
      if (P.ignore?.includes(tg.id)) continue;
      const tX = tg.X + this.cal.tags[k].X, tY = tg.Y + this.cal.tags[k].Y;
      const dx = tX - cam.X, dy = tY - cam.Y, d = Math.hypot(dx, dy);
      if (d < 0.3) continue;
      // facing the camera (within 70 deg of square on)
      const view = wrap(Math.atan2(-dy, -dx) - tg.th);
      if (Math.abs(view) > 70 * DEG) continue;
      const b = wrap(Math.atan2(dy, dx) - cam.th);
      if (Math.abs(b) > (C.hfov / 2 - 2) * DEG) continue;
      const el = Math.atan2(tg.h - cm.h, d) - cm.pitch * DEG;
      if (Math.abs(el) > (C.vfov / 2 - 2) * DEG) continue;
      const tagPx = (C.f * TAG_SIZE) / d, edgePx = tagPx * Math.sqrt(Math.max(0.05, Math.cos(view)));
      if (edgePx < C.minPx) continue;
      if (cm.maxTag && Math.hypot(tg.X - T.X, tg.Y - T.Y) > cm.maxTag) continue;
      if (this._hidden(cam, cm.h, tg, tX, tY, d)) continue;
      if (this.rng() > 0.97) continue; // a missed detection
      // pixel noise: blur from the camera turning and the tag sweeping across the image
      const blur = C.exp * C.f * (wCam + sp / d);
      const px = Math.hypot(C.px, 0.3 * blur);
      const sb = px / C.f, sr = (d * d * px) / (C.f * TAG_SIZE);
      // a rolling shutter's skew: rows read out over rs, the tag's row against the frame's middle
      const skew = C.rs ? -(r.omega + tRate) * C.rs * (0.5 - (0.5 - el / (C.vfov * DEG))) : 0;
      const bm = b + cal.yaw + skew + this.gauss() * sb;
      const dm = d * (1 + cal.scale) + this.gauss() * sr;
      // one tag alone: its own tilt in the image gives the heading (PnP), poorly far off and square on
      const cue = edgePx * (0.15 + Math.abs(Math.sin(view)));
      const sphi = (2 * px) / cue;
      const phim = wrap(tg.th + Math.PI - cam.th) + this.gauss() * sphi;
      obs.push({ k, d, b: bm, r: dm, sb, sr, phi: phim, sphi, amb: Math.min(1, (6 * px) / cue) });
    }
    if (!obs.length) return null;
    // the turret's angle as the code knows it at the frame's time (encoder, timing)
    const turretErr = cm.turret !== undefined ? cal.turret + tRate * this.gauss() * SIGMA.turretT : 0;
    const mode = cm.mode ?? P.mode;
    if ((P.minTags ?? 1) > obs.length) return null;
    if (P.maxOmega && Math.abs(r.omega) > P.maxOmega * DEG) return null;
    // the camera's pose as the code believes it (nominal mount, its turret angle)
    const camNominal = (p) => camWorld(p, -turretErr);
    let pose, single = obs.length === 1;
    if (mode === 'mt2' || mode === 'tagdist' || (mode === 'pnp1h' && single)) {
      // the heading given (the estimate's, sent to the camera / used by their code), position solved
      const use = mode === 'tagdist' ? [obs.reduce((a, o) => (o.d < a.d ? o : a))] : obs;
      const th = this.est.th;
      const cn = camNominal({ X: 0, Y: 0, th });
      let I11 = 0, I12 = 0, I22 = 0, v1 = 0, v2 = 0;
      for (const o of use) {
        const tg = TAGS[o.k], a = cn.th + o.b, ux = Math.cos(a), uy = Math.sin(a);
        // the robot's position from this tag: the tag less the ray, less the camera's offset
        const pX = tg.X - o.r * ux - cn.X, pY = tg.Y - o.r * uy - cn.Y;
        const wA = 1 / (o.sr * o.sr), wC = 1 / ((o.r * o.sb) ** 2);
        const i11 = wA * ux * ux + wC * uy * uy, i12 = (wA - wC) * ux * uy, i22 = wA * uy * uy + wC * ux * ux;
        I11 += i11; I12 += i12; I22 += i22; v1 += i11 * pX + i12 * pY; v2 += i12 * pX + i22 * pY;
      }
      const det = I11 * I22 - I12 * I12;
      pose = { X: (I22 * v1 - I12 * v2) / det, Y: (I11 * v2 - I12 * v1) / det, th };
      single = use.length === 1;
    } else {
      if (single && obs[0].amb > (P.maxAmb ?? 0.3)) return null;
      pose = this._pnp(obs, camNominal, T);
    }
    // a tilted robot (the BUMP, a hard stop) read as level: the camera's height leans its pose
    if (tilt > 1e-3) {
      const q = r.quat, upX = 2 * (q.x * q.y - q.w * q.z), upZ = 2 * (q.y * q.z + q.w * q.x);
      pose.X += cm.h * upX; pose.Y -= cm.h * upZ;
    }
    const n = single ? 1 : obs.length, avgD = obs.reduce((s, o) => s + o.d, 0) / obs.length;
    if (P.maxDist && avgD > P.maxDist) return null;
    if (pose.X < -HALF_L - 0.5 || pose.X > HALF_L + 0.5 || pose.Y < -HALF_W - 0.5 || pose.Y > HALF_W + 0.5) return null;
    const lat = C.lat + Math.abs(this.gauss()) * SIGMA.latJitter;
    return { stamp: t + this.gauss() * SIGMA.tsJitter, due: t + lat, pose, n, d: avgD, mode };
  }

  // a tag hidden: a HUB (other than the one it's on) or a robot between, below the line of sight
  _hidden(cam, ch, tg, tX, tY, d) {
    const seg = (x, y, rad, top) => {
      const ex = tX - cam.X, ey = tY - cam.Y;
      const u = Math.max(0, Math.min(1, ((x - cam.X) * ex + (y - cam.Y) * ey) / (d * d)));
      if (u <= 0 || u >= 0.98) return false;
      const qx = cam.X + u * ex - x, qy = cam.Y + u * ey - y;
      return Math.hypot(qx, qy) < rad && ch + u * (tg.h - ch) < top;
    };
    for (const o of this.others) if (seg(o.pos.x, -o.pos.z, Math.max(o.halfL, o.halfW) * 0.9, 0.55)) return true;
    for (const a of ['blue', 'red']) {
      const hc = Field.hubCenter(a);
      if (Math.hypot(tg.X - hc.x, tg.Y + hc.z) < HUB.size) continue; // its own HUB
      if (seg(hc.x, -hc.z, HUB.size / 2, 2)) return true;
    }
    return false;
  }

  // full pose from the tags (Gauss-Newton on bearings, ranges and each tag's own tilt, from the truth
  // as a start: the solvers converge there)
  _pnp(obs, camNominal, T) {
    let p = { X: T.X, Y: T.Y, th: T.th };
    const res = (q) => {
      const cn = camNominal(q), out = [];
      for (const o of obs) {
        const tg = TAGS[o.k], dx = tg.X - cn.X, dy = tg.Y - cn.Y;
        out.push(wrap(Math.atan2(dy, dx) - cn.th - o.b) / o.sb, (Math.hypot(dx, dy) - o.r) / o.sr, wrap(wrap(tg.th + Math.PI - cn.th) - o.phi) / o.sphi);
      }
      return out;
    };
    for (let it = 0; it < 3; it++) {
      const r0 = res(p), J = [];
      const h = [1e-4, 1e-4, 1e-5];
      for (let j = 0; j < 3; j++) {
        const q = { ...p };
        q[['X', 'Y', 'th'][j]] += h[j];
        J.push(res(q).map((v, i) => (v - r0[i]) / h[j]));
      }
      // normal equations
      const A = [[0, 0, 0], [0, 0, 0], [0, 0, 0]], g = [0, 0, 0];
      for (let i = 0; i < r0.length; i++) for (let a = 0; a < 3; a++) { g[a] += J[a][i] * r0[i]; for (let b = 0; b < 3; b++) A[a][b] += J[a][i] * J[b][i]; }
      const dx = solve3(A[0][0], A[0][1], A[0][2], A[1][1], A[1][2], A[2][2], -g[0], -g[1], -g[2]);
      p = { X: p.X + dx[0], Y: p.Y + dx[1], th: wrap(p.th + dx[2]) };
    }
    return p;
  }

  // a vision pose reaching the robot code
  _apply(m, t) {
    const P = this.prof;
    this.stats.used++;
    if (P.fuse === 'reset') {
      // 2910: the translation set to the vision pose as it is (stale), unless latched
      if (!this.latched) { this.est.X = m.pose.X; this.est.Y = m.pose.Y; }
      return;
    }
    const [sxy, sth] = P.std(m.d, m.n, m.n === 1);
    const r = [sxy * sxy, sxy * sxy, sth * sth];
    const K = P.q.map((s, i) => { const q = s * s; return Number.isFinite(r[i]) ? q / (q + Math.sqrt(q * r[i])) : 0; });
    // the odometry pose at the frame's time (AdvantageKit's RobotState / WPILib's PoseEstimator)
    const at = this._odoAt(m.stamp);
    if (!at) return;
    const est0 = compose(this.est, between(this.odo, at)); // the estimate shifted back to then
    const d = between(est0, m.pose);
    const corr = { X: K[0] * d.X, Y: K[1] * d.Y, th: K[2] * d.th };
    const est1 = compose(est0, corr);
    this.est = compose(est1, between(at, this.odo));
  }

  _odoAt(ts) {
    const b = this.buf;
    if (!b.length || ts < b[0].t) return null;
    for (let i = b.length - 1; i > 0; i--) {
      if (b[i - 1].t <= ts) {
        const a = b[i - 1], c = b[i], u = c.t > a.t ? Math.min(1, (ts - a.t) / (c.t - a.t)) : 0;
        return { X: a.X + u * (c.X - a.X), Y: a.Y + u * (c.Y - a.Y), th: wrap(a.th + u * wrap(c.th - a.th)) };
      }
    }
    return { ...b[b.length - 1] };
  }

  // ---------------------------------------------------------------- what the aim reads
  _aimState() { return this.hist.length ? this.hist[0] : null; }

  // world point p as the aim sees it: where it sits from the true pose as p sits from the estimate
  aimPt(p) {
    const s = !this.truth && this.ready && this._aimState();
    if (!s) return p;
    const r = this.r, T = this._truth();
    const rel = between({ X: s.X, Y: s.Y, th: s.th }, { X: p.x, Y: -p.z, th: 0 });
    const w = compose(T, rel);
    return { ...p, x: w.X, z: -w.Y };
  }

  // the chassis velocity the aim uses (world x / z), measured by the wheels, in the true frame
  vel() {
    const s = !this.truth && this.ready && this._aimState();
    if (!s) return null;
    const th = this.r.yaw, c = Math.cos(th), sn = Math.sin(th);
    const X = c * s.f - sn * s.l, Y = sn * s.f + c * s.l;
    return { x: X, z: -Y };
  }

  omega() { const s = !this.truth && this.ready && this._aimState(); return s ? s.w : null; }

  // how far off the estimate the aim uses is (m, rad), and where it puts the HUB (m)
  error() {
    const s = !this.truth && this.ready && this._aimState();
    if (!s) return null;
    // live: the estimate now against the truth now; hub: where the aim (the estimate ACT_DELAY ago,
    // against the truth now) puts the HUB's center; hubLive: where the estimate now would
    const T = this._truth(), h = Field.hubCenter(this.r.alliance), a = this.aimPt(h);
    const rel = between(this.est, { X: h.x, Y: -h.z, th: 0 }), w = compose(T, rel);
    return { live: Math.hypot(this.est.X - T.X, this.est.Y - T.Y), th: wrap(this.est.th - T.th), hub: Math.hypot(a.x - h.x, a.z - h.z), hubLive: Math.hypot(w.X - h.x, w.Y + h.z) };
  }
}

// symmetric 3x3 solve (a11 a12 a13 / . a22 a23 / . . a33) x = b
function solve3(a11, a12, a13, a22, a23, a33, b1, b2, b3) {
  const c11 = a22 * a33 - a23 * a23, c12 = a13 * a23 - a12 * a33, c13 = a12 * a23 - a13 * a22;
  const det = a11 * c11 + a12 * c12 + a13 * c13;
  if (Math.abs(det) < 1e-12) return [0, 0, 0];
  const c22 = a11 * a33 - a13 * a13, c23 = a12 * a13 - a11 * a23, c33 = a11 * a22 - a12 * a12;
  return [(c11 * b1 + c12 * b2 + c13 * b3) / det, (c12 * b1 + c22 * b2 + c23 * b3) / det, (c13 * b1 + c23 * b2 + c33 * b3) / det];
}
