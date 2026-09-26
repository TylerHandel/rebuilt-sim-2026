// All dimensions are from the 2026 FRC Game Manual (REBUILT, Section 5 "ARENA")
// and the official 2026 welded-field AprilTag layout. Units: meters, seconds, kg.
//
// Coordinate systems
//   Field frame (WPILib): origin at the right corner of the BLUE alliance wall (from the
//   blue drivers' view). fx runs toward the red wall, fy runs to the blue drivers' left.
//   World frame (three.js / Rapier): Y is up, field centered at the origin.
//     X = fx - L/2          (blue wall at -X, red wall at +X)
//     Z = W/2 - fy          (blue drivers' right is +Z)
//   The red half of the field is the blue half rotated 180 degrees about the Y axis,
//   i.e. world (x, z) -> (-x, -z).

export const IN = 0.0254;
export const LB = 0.45359237;

export const FIELD_L = 16.541; // 651.2in
export const FIELD_W = 8.069;  // 317.7in
export const HALF_L = FIELD_L / 2;
export const HALF_W = FIELD_W / 2;

export const BLUE = 'blue';
export const RED = 'red';
export const other = (a) => (a === BLUE ? RED : BLUE);
// Side sign: blue half is -X, red half is +X
export const sideSign = (a) => (a === BLUE ? 1 : -1);

// Blue-side field coords (fx, fy) -> world (x, z) for the requested alliance's half
export function fw(fx, fy, alliance = BLUE) {
  const x = fx - HALF_L;
  const z = HALF_W - fy;
  return alliance === BLUE ? { x, z } : { x: -x, z: -z };
}

export const GRAVITY = 9.81;

// ---------------------------------------------------------------- FUEL
export const FUEL = {
  radius: (5.91 * IN) / 2, // 5.91in diameter high-density foam ball
  mass: 0.215,             // 0.448-0.500 lb
  total: 504,
  perDepot: 24,
  perChute: 24,
  maxPreload: 8,
  neutralMax: 408,
  // quadratic drag coefficient k = 0.5*rho*Cd*A/m  (Cd~0.47 sphere)
  dragK: (0.5 * 1.225 * 0.47 * Math.PI * ((5.91 * IN) / 2) ** 2) / 0.215,
  rollDecel: 0.35,         // carpet rolling resistance (m/s^2)
  restitution: 0.45,
  friction: 0.35,          // foam on foam / field elements (low so FUEL slides off walls and rolls on)
  angularDamping: 0.05,
};

// ---------------------------------------------------------------- ZONES
export const ALLIANCE_ZONE_DEPTH = 158.6 * IN; // 4.028 m, ROBOT STARTING LINE is its edge

// ---------------------------------------------------------------- HUB
export const HUB = {
  size: 47 * IN,                    // 47in x 47in rectangular prism
  fx: 158.6 * IN + (47 * IN) / 2,   // center, 158.6in from ALLIANCE WALL to near face
  fy: HALF_W,
  rimFront: 72 * IN,                // front edge of the opening is 72in off the carpet
  rimBack: 80 * IN,                 // "extended opening" rises toward the back
  hexR: (41.7 * IN) / 2,            // 41.7in hexagonal opening (circumradius)
  funnelBottomY: 1.42,
  funnelBottomR: 0.36,
  netTop: 3.25,                     // net structure in the back of the HUB
  netLean: 0.55,
  exitOffsets: [-0.46, -0.16, 0.16, 0.46], // 4 exits into the NEUTRAL ZONE
  exitHeight: 0.10,                 // bottom of the FUEL as it leaves an exit (m above the carpet)
  exitSpeed: [1.5, 3.0],            // m/s off the exit ramps
  scoreGrace: 3.0,                  // FUEL assessed up to 3s after deactivation
  targetHeight: 1.95,               // aim point (center of opening)
};

// ---------------------------------------------------------------- BUMP
export const BUMP = {
  width: 73.0 * IN,   // along fy
  depth: 44.4 * IN,   // along fx
  height: 6.513 * IN,
};

// ---------------------------------------------------------------- TRENCH
export const TRENCH = {
  width: 65.65 * IN,
  depth: 47.0 * IN,
  height: 40.25 * IN,
  clearWidth: 50.34 * IN,
  clearHeight: 22.25 * IN,
  armThick: 3.0 * IN,
};

// ---------------------------------------------------------------- TOWER
export const TOWER = {
  fy: (3.7457126 + 4.1775126) / 2, // centered between TOWER WALL AprilTags 31/32
  width: 49.25 * IN,
  depth: 45.0 * IN,
  height: 78.25 * IN,
  baseWidth: 39.0 * IN,
  baseDepth: 45.18 * IN,
  baseThick: 0.25 * IN,
  uprightHeight: 72.1 * IN,
  uprightThick: 1.5 * IN,           // along the rung direction
  uprightDeep: 3.5 * IN,
  uprightGap: 32.25 * IN,
  rungR: (1.66 * IN) / 2,
  rungExt: 5.875 * IN,
  rungs: [27.0 * IN, 45.0 * IN, 63.0 * IN], // LOW, MID, HIGH centers
  supportLow: 28.4 * IN,
  supportHigh: 43.38 * IN,
};
TOWER.uprightFx = TOWER.baseDepth - TOWER.uprightDeep / 2 - 0.02;

// ---------------------------------------------------------------- DEPOT
export const DEPOT = {
  fy: 6.0,
  width: 42.0 * IN,
  depth: 27.0 * IN,
  barrierW: 3.0 * IN,
  barrierH: 1.125 * IN,
};

// ---------------------------------------------------------------- OUTPOST
export const OUTPOST = {
  fy: (0.6659626 + 1.0977626) / 2, // centered between OUTPOST AprilTags 29/30
  upperW: 31.8 * IN,
  upperH: 7.0 * IN,
  upperY: 28.1 * IN,
  baseW: 32.0 * IN,
  baseH: 7.0 * IN,
  baseY: 1.88 * IN,
  corralW: 35.8 * IN,
  corralD: 37.6 * IN,
  corralWallH: 8.13 * IN,
  chuteCapacity: 25,
  chuteSlopeDeg: 15,
  structW: 1.35,
};

// ---------------------------------------------------------------- WALLS
export const GUARDRAIL_H = 20.0 * IN;
export const DS_BASE_H = 36.8 * IN;
export const DS_GLASS_H = 42.0 * IN;
export const ALLIANCE_WALL_H = DS_BASE_H + DS_GLASS_H;

// Driver station centers along the alliance wall (blue fy), left/center/right from the drivers' view
export const DRIVER_STATIONS = [
  { name: 'Left (DS1)', fy: 7.2 },
  { name: 'Center (DS2)', fy: 5.45 },
  { name: 'Right (DS3)', fy: 2.35 },
];

// ---------------------------------------------------------------- MATCH TIMING
export const TIMING = {
  preMatch: 3,
  auto: 20,
  autoGap: 3,
  teleop: 140,
  transition: 10,
  shift: 25,
  endgame: 30,
  post: 3,
};

// ---------------------------------------------------------------- POINTS
export const POINTS = {
  fuel: 1,
  towerAutoL1: 15,
  tower: [0, 10, 20, 30],
  minorFoul: 5,
  majorFoul: 15,
};

// ---------------------------------------------------------------- COLLISION GROUPS
export const GROUP = {
  STATIC: 0x0001,
  BALL: 0x0002,
  ROBOT: 0x0004,
  WHEEL: 0x0008,
  ROBOT_BARRIER: 0x0010, // invisible, blocks robots only
  INTAKE: 0x0020,        // deployed intake bar: hits FUEL and walls, rides over TERRAIN
  TERRAIN: 0x0040,       // low floor features (BUMPS, DEPOT barriers, TOWER BASE)
};
export const groups = (member, filter) => ((member & 0xffff) << 16) | (filter & 0xffff);

export const PHYSICS_DT = 1 / 120;
