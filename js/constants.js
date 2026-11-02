// The field's dimensions are from FIRST's official field CAD (Onshape "FE-2026: REBUILT Playing
// Field", the welded perimeter) and its drawings (2026-field-dimension-dwgs.pdf, the FE-2026 sheets;
// 2026-field-dwg-game-specific.pdf, the GE-26xxx element drawings), as cited at each element, and the
// 2026 FRC Game Manual (REBUILT, Section 5 "ARENA"). Units: meters, seconds, kg.
//
// Coordinate systems
//   Field frame (WPILib): origin at the right corner of the BLUE alliance wall (from the
//   blue drivers' view). fx runs toward the red wall, fy runs to the blue drivers' left.
//   World frame (three.js / Rapier): Y is up, field centered at the origin.
//     X = fx - L/2          (blue wall at -X, red wall at +X)
//     Z = W/2 - fy          (blue drivers' right is +Z)
//   The red half of the field is the blue half rotated 180 degrees about the Y axis,
//   i.e. world (x, z) -> (-x, -z), except the TRENCHES: both FIXED ones are on the scoring-table
//   side (world +z), both HINGED ones on the other (TRENCH.fixedZ).

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
  // Foam: squeezed, it gives softly at first and firms up the harder it's pressed, as the cells
  // close up: F(x) = k0 * xc * (e^(x / xc) - 1) (hopper.js fuelForce), stiffness k0 to start,
  // growing e-fold every xc of squeeze. Set so that over the first inch it averages 33 lb/in: the
  // FUEL played with. AndyMark's 2026 Scoring Element Testing Report has the batch's nominal 20
  // lb/in when made (summer 2025), and the same balls retested at the season's start 26-38, 33 on
  // average, every one at least 10% firmer ("all the fuel used on official fields would be
  // similarly aged"; teams found the Kit of Parts FUEL firmer than AndyMark's). The curve's shape,
  // the fresh balls' scaled up: 19.8 lb/in to start, 34 lb at 1in, 69 lb at 1.5in, 127 lb at 2in.
  // (It was the fresh 20 lb/in, 12 to start: a shooter's FUEL, pressed toward its hood as they go
  // round the wheel, pulled off the wheel at ~8-10 m/s, and no robot passed past ~8 m.)
  // Two FUEL pressed together squash the same (each contact is twice as stiff, in series).
  // (FOAM0=12: the fresh foam's curve, a trial)
  springRate0: (Number(globalThis.process?.env?.FOAM0 ?? 19.8) * 4.448) / 0.0254, // N/m, uncompressed
  stiffen: 1.0 * 0.0254,               // m of squeeze per e-fold of stiffness
  springRate: (33 * 4.448) / 0.0254,  // N/m, nominal (the average over the first inch)
  // how hard an intake roller shoves FUEL into a hopper when nothing else says (hopper.js
  // intakePush works it out per robot: 3/4in of squeeze -> ~100 N)
  intakePush: 100,         // N
  total: 504,
  perDepot: 24,
  perChute: 24,
  maxPreload: 8,
  neutralMax: 408,
  // quadratic drag coefficient k = 0.5*rho*Cd*A/m  (Cd~0.47 sphere)
  dragK: (0.5 * 1.225 * 0.47 * Math.PI * ((5.91 * IN) / 2) ** 2) / 0.215,
  // Magnus lift on a spinning FUEL: F = 0.5 rho A C_L v^2 across its flight, toward (spin x
  // velocity), so a shooter's backspin lifts it. liftK = 0.5 rho A / m (per m); C_L from the spin
  // ratio S = R w / v (the surface's speed from the spin over the flight speed): S / (1 + 2 S), the
  // shape measured for spinning spheres (about S at low spin, 0.25 at S = 0.5, 0.33 at S = 1;
  // ballistics.js liftCoef). The spin itself barely decays in flight (aerodynamic torque on a light
  // foam sphere: under 2%/s at 10 m/s), so it's held as it left.
  liftK: (0.5 * 1.225 * Math.PI * ((5.91 * IN) / 2) ** 2) / 0.215,
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
  // The top (field CAD): a polycarbonate hexagonal funnel standing proud of the box. Its rim is
  // level at 72in, 41.7in across the flats, corners toward the ALLIANCE WALL and the NEUTRAL ZONE
  // (just past the 47in faces); its walls slope in (0.68 out per 1 up) to a level hexagon 1.52m
  // off the carpet. The box's alliance-colored deck round it slopes down toward the NEUTRAL ZONE.
  rimFront: 72 * IN,                // the rim, 72in off the carpet all round
  hexR: (41.7 * IN) / (2 * Math.cos(Math.PI / 6)), // circumradius of the 41.7in opening
  funnelSlope: 0.68,                // funnel wall: radius lost per meter down
  funnelLowY: 1.52,                 // the funnel's bottom edge
  deckY0: 1.52, deckY1: 1.415,      // deck at the ALLIANCE WALL and NEUTRAL ZONE faces
  funnelBottomY: 1.40,              // FUEL below this inside the box has dropped through the funnel
  // NET behind the HUB (GE-26300 sheet 4): 58.41in wide, 49.75in to 120.36in off the carpet,
  // 10.26in behind the HUB's NEUTRAL ZONE face
  netTop: 120.36 * IN,
  netBottom: 49.75 * IN,
  netOut: 10.26 * IN,
  netWidth: 58.41 * IN,
  // FUEL leaves through one opening in the NEUTRAL ZONE face: 47in - 2 x 5.72in wide, its bottom
  // edge 30.13in off the carpet (top of the 30in rear bottom panel, GE-26306), off the HUB ramps
  exitWidth: (47 - 2 * 5.72) * IN,
  exitTop: 38.88 * IN,
  exitHeight: 30.13 * IN,           // bottom of the FUEL as it leaves the opening (m above the carpet)
  // Inside, FUEL drops out of the funnel onto a ramp (rampRise higher at the back than at the
  // exit) and rolls out; a roof slopes up and back from the top of the exit, so the load narrows
  // to one layer at the opening. FUEL inside is slick (fuelFriction) and pressed down a little
  // harder (extraGravity, m/s^2), so the load keeps sliding out instead of locking into an arch.
  rampRise: 0.4,
  roofRun: 0.45, roofRise: 0.2,
  fuelFriction: 0.02,
  extraGravity: 4.9,
  scoreGrace: 3.0,                  // FUEL assessed up to 3s after deactivation
  targetHeight: 1.95,               // aim point (center of opening)
  // LED light diffusers around the top of the HUB box (field CAD, GE-26309/10/11)
  lightY0: 1.275,
  lightY1: 1.51,
};

// ---------------------------------------------------------------- BUMP
export const BUMP = {
  width: 73.0 * IN,   // along fy
  depth: 44.4 * IN,   // along fx
  height: 6.513 * IN,
};

// ---------------------------------------------------------------- TRENCH
// From the official field CAD (FE-2026, GE-26200 TRENCH) and its drawings (FE-2026 sheets 4 and 8,
// sections A-A and C-C; GE-26201 hinged and GE-26202 fixed assemblies). Each ALLIANCE has two, not
// alike: the FIXED one on the scoring-table side (fy small: world +z for both ALLIANCES, as the field
// is mirrored about the CENTER LINE there, not turned) and the HINGED one on the far side, whose arm
// swings up for field reset and is pinned down in a match ("hinged TRENCHES to be measured in the
// closed and locked state"). Both stand on the same skirt on the BUMP side: 47in x 12in, its side
// plates (GE-26211/26212) under covers (GE-26213) that ramp up at 30 degrees from 8.0in at the ends to
// 18.85in, 4.7in either side of the middle; a riser in its middle (6in along fx); and the arm across
// the opening, a square-edged aluminum tube whose underside is 22.25in (+-0.25in, controlled: FE-2026
// sheet 8) off the carpet, flat all the way across, and its top 26.25in. The FIXED arm (GE-26224) is
// 6in wide (along fx) and runs from the skirt's BUMP side to 8.66in past the guardrail, where a
// leveling mount sits on the guardrail's top rail. The HINGED arm (GE-26214) is a 4in tube from 3.25in
// past the guardrail to 43.07in in from it, then an 8in cross tube (43.07in-47.07in) and a fork of two
// 3in tubes 4in-7in either side of the middle, from 43.07in back over the skirt to its pivot on a riser
// that stands 29.25in. On each arm, 25.37in in from the guardrail (133.47in off the center line), an
// AprilTag mount (GE-26203) 11.5in wide reaching 40.25in, its tags centered 35in up. The opening is
// 50.34in (+-2, controlled: the guardrail to the skirt). FE-2026 sheet 4 gives 50.67in for the fixed
// side's: measured at its section A-A, where the guardrail is a field gate; at the TRENCH the guardrail's
// plastic is 158.84in off the center line on both sides and the skirt's face 108.50in.
export const TRENCH = {
  width: 62.34 * IN,       // the guardrail to the BUMP side of the skirt (158.84in - 96.50in)
  depth: 47.0 * IN,        // along fx, centered on the HUB
  height: 40.25 * IN,      // top of the AprilTag mount
  clearWidth: 50.34 * IN,  // the opening, the guardrail to the skirt
  clearHeight: 22.25 * IN, // the arms' underside
  armTop: 26.25 * IN,
  skirtEndH: 8.0 * IN,
  skirtTopH: 18.85 * IN,
  skirtSlopeDeg: 30,
  riserW: 6.0 * IN,        // both risers, along fx
  hingedRiserTop: 29.25 * IN,
  tagWidth: 11.5 * IN,
  tagThick: 2.94 * IN,     // the mount (2.84in) and a tag on each face
  tagIn: 25.37 * IN,       // the mount's middle, in from the guardrail
  tagY: 35.0 * IN,         // the tags' centers
  fixedZ: 1,               // the FIXED TRENCHES are on world +z (fy < FIELD_W / 2), the HINGED ones on -z
};
// The parts of a TRENCH's arm, all at clearHeight..armTop: u0..u1 across it (along fx, from its
// middle), g0..g1 in from the guardrail's face (negative: past it, over the guardrail's top rail)
export function trenchArmParts(fixed) {
  const skirtIn = TRENCH.width - 0.19 * IN; // (the skirt's inner side plate)
  if (fixed) return [{ u0: -3 * IN, u1: 3 * IN, g0: -8.66 * IN, g1: skirtIn }];
  return [
    { u0: -2 * IN, u1: 2 * IN, g0: -3.25 * IN, g1: 43.07 * IN },  // main tube
    { u0: -4 * IN, u1: 4 * IN, g0: 43.07 * IN, g1: 47.07 * IN },  // cross tube
    { u0: -7 * IN, u1: -4 * IN, g0: 43.07 * IN, g1: 61.37 * IN }, // the fork, back to the pivot
    { u0: 4 * IN, u1: 7 * IN, g0: 43.07 * IN, g1: 61.37 * IN },
  ];
}

// ---------------------------------------------------------------- TOWER
// From the field CAD (GE-26500 TOWER) and drawing GE-26500 sheet 2
export const TOWER = {
  fy: 3.7457126, // centered on TOWER WALL AprilTag 31 (official field CAD, FE-2026)
  width: 49.25 * IN,               // the TOWER WALL's plastic (GE-26505/26506)
  depth: 45.0 * IN,
  height: 78.25 * IN,              // its top rail
  baseWidth: 39.0 * IN,            // base plate GE-26501-01
  baseDepth: 45.06 * IN,           // from the ALLIANCE WALL
  baseThick: 0.28 * IN,            // its top off the carpet (a 0.16in plate on hook tape)
  uprightHeight: 72.125 * IN,
  uprightThick: 1.5 * IN,          // along the rung direction
  uprightDeep: 3.51 * IN,
  uprightGap: 32.25 * IN,
  rungR: (1.66 * IN) / 2,
  rungExt: 5.875 * IN,
  rungs: [27.0 * IN, 45.0 * IN, 63.0 * IN], // LOW, MID, HIGH centers (+-0.25in)
  // the 1.75in tube (GE-26510) from each upright back to the TOWER WALL, 35.125in to 36.875in up; the
  // gussets at its wall end (GE-26509), their lower edge rising from 28.37in at the wall to the tube
  // 8.05in out; a gusset on each upright over the tube (GE-26508, 6.5in, up to 43.375in); and two 2in
  // braces (GE-26507, 35in-37in up) from the wall beside the TOWER's middle out to the tubes
  supportY: 36.0 * IN,
  supportTube: 1.75 * IN,
  gussetLow: 28.37 * IN,
  gussetDeep: 8.05 * IN,
  uprightGusset: 6.5 * IN,
  braceY: 36.0 * IN,
  braceTube: 2.0 * IN,
  braceIn: 2.97 * IN,              // a brace's centerline: from the wall 2.97in off the middle
  braceOut: 13.03 * IN,            // to the support tube's side, 13.03in out and 16in off the middle
  braceSide: 16.0 * IN,
  tagStep: 17.0 * IN,              // AprilTag 32 is 17in from 31 (on the middle) toward fy+
};
TOWER.uprightFx = 40.0 * IN + TOWER.uprightDeep / 2; // the uprights' centers: 40in from the TOWER WALL to their faces (GE-26500)

// ---------------------------------------------------------------- DEPOT
// Field CAD GE-26600: a weldment of 3in x 1in tubes on hook tape (top 1.12in), 42in x 27in outside
export const DEPOT = {
  fy: 5.965, // center, from the official field CAD (FE-2026)
  width: 42.0 * IN,
  depth: 27.0 * IN,
  barrierW: 3.0 * IN,
  barrierH: 1.125 * IN,
};

// ---------------------------------------------------------------- OUTPOST
// Field CAD GE-26000 OUTPOST: a 49.25in wide stretch of the ALLIANCE WALL (lower panel GE-26002, upper
// GE-26003, on frame GE-26001, 1.94in deep) from the guardrail's corner to DRIVER STATION 3, with two
// 32in openings on its middle: the base opening 1.87in-8.87in up (a 1.66in round tube up its middle)
// and the CHUTE's 28.12in-35.12in. The CHUTE (GE-26005) rises away from the FIELD at 15 degrees, 31.67in
// wide inside and 32in long. Behind the base opening, the CORRAL's border (GE-26010, 8in tall) is
// 35.75in wide inside, its back 37.58in behind the wall's face, with a ramp (GE-26009) down from the
// opening's lip (1.87in) to the floor 27.95in back.
export const OUTPOST = {
  fy: 0.6659626, // centered on OUTPOST AprilTag 29 (official field CAD, FE-2026)
  upperW: 31.67 * IN,
  upperH: 7.0 * IN,
  upperY: 28.12 * IN,
  baseW: 32.0 * IN,
  baseH: 7.0 * IN,
  baseY: 1.87 * IN,
  corralW: 35.75 * IN,
  corralD: 37.58 * IN,     // the ALLIANCE WALL's face to the CORRAL's back
  corralWallH: 8.12 * IN,
  rampFrom: 1.94 * IN,     // the ramp starts at the frame's back, behind the wall's face
  rampLen: 27.95 * IN,
  chuteCapacity: 25,
  chuteSlopeDeg: 15,
  chuteLen: 32.0 * IN,
  structW: 49.25 * IN,
  tagStep: 17.0 * IN,      // AprilTag 30 is 17in from 29 (on the middle) toward fy+
};

// ---------------------------------------------------------------- WALLS
// Guardrail (FE-2026-02): polycarbonate 0-19in on its inside face (158.84in off the center line), the
// rail just behind it topped at 20.10in
export const GUARDRAIL_H = 20.1 * IN;
// DRIVER STATION end panel (FE-00043): diamond plate to 36.12in under a cap at 36.87in, acrylic above
// it, the top rail (FE-00007) at 76.25in-78.25in
export const DS_BASE_H = 36.87 * IN;
export const ALLIANCE_WALL_H = 78.25 * IN;
export const DS_GLASS_H = ALLIANCE_WALL_H - DS_BASE_H;

// Driver station centers along the alliance wall (blue fy), left/center/right from the drivers' view.
// Each is 72in wide (field CAD): DS 3 from the OUTPOST (50.84in) to the TOWER WALL (122.84in), DS 2
// from the TOWER WALL (172.09in) to 244.09in, DS 1 from there to 316.09in.
export const DRIVER_STATIONS = [
  { name: 'Left (DS1)', fy: 280.09 * IN },
  { name: 'Center (DS2)', fy: 208.09 * IN },
  { name: 'Right (DS3)', fy: 86.84 * IN },
];
export const DS_SPLIT_FY = 244.09 * IN; // where DS 1 meets DS 2

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
  HELD: 0x0080,          // FUEL in a physical robot's ball path: their touching each other is its
                         // mechanism's to work out (foam on foam), not the world's
  TRENCH_ARM: 0x0100,    // the TRENCH arms over the openings: met by whatever meets the field (groups()
                         // adds it wherever STATIC is), but an intake arm the TRENCH arm pushes down
                         // on its pivot instead (8793's: js/mech/phys8793.js trenchArm)
};
export const groups = (member, filter) => ((member & 0xffff) << 16) | ((filter & GROUP.STATIC ? filter | GROUP.TRENCH_ARM : filter) & 0xffff);

export const PHYSICS_DT = 1 / 120;

// ---------------------------------------------------------------- APRILTAGS
// The 2026 field's AprilTags (36h11, 6.5in black square), as WPILib ships them for the welded field
// (allwpilib v2026.2.1, apriltag/.../2026-rebuilt-welded.json; FE-2026 sheet 11): [id, fx, fy, height,
// facing], field frame (m), facing the way the tag looks out (deg, CCW from +fx). What a robot's vision
// sees to find itself (js/localize.js). World: x = fx - L/2, z = W/2 - fy.
export const TAG_SIZE = 6.5 * IN;
export const APRILTAGS = [
  [1, 11.8780, 7.4248, 0.8890, 180], [2, 11.9154, 4.6380, 1.1240, 90], [3, 11.3119, 4.3902, 1.1240, 180],
  [4, 11.3119, 4.0346, 1.1240, 180], [5, 11.9154, 3.4312, 1.1240, -90], [6, 11.8780, 0.6445, 0.8890, 180],
  [7, 11.9529, 0.6445, 0.8890, 0], [8, 12.2710, 3.4312, 1.1240, -90], [9, 12.5192, 3.6790, 1.1240, 0],
  [10, 12.5192, 4.0346, 1.1240, 0], [11, 12.2710, 4.6380, 1.1240, 90], [12, 11.9529, 7.4248, 0.8890, 0],
  [13, 16.5333, 7.4033, 0.5524, 180], [14, 16.5333, 6.9715, 0.5524, 180], [15, 16.5330, 4.3236, 0.5524, 180],
  [16, 16.5330, 3.8918, 0.5524, 180], [17, 4.6631, 0.6445, 0.8890, 0], [18, 4.6256, 3.4312, 1.1240, -90],
  [19, 5.2292, 3.6790, 1.1240, 0], [20, 5.2292, 4.0346, 1.1240, 0], [21, 4.6256, 4.6380, 1.1240, 90],
  [22, 4.6631, 7.4248, 0.8890, 0], [23, 4.5882, 7.4248, 0.8890, 180], [24, 4.2700, 4.6380, 1.1240, 90],
  [25, 4.0219, 4.3902, 1.1240, 180], [26, 4.0219, 4.0346, 1.1240, 180], [27, 4.2700, 3.4312, 1.1240, -90],
  [28, 4.5882, 0.6445, 0.8890, 180], [29, 0.0077, 0.6660, 0.5524, 0], [30, 0.0077, 1.0978, 0.5524, 0],
  [31, 0.0081, 3.7457, 0.5524, 0], [32, 0.0081, 4.1775, 0.5524, 0],
];
