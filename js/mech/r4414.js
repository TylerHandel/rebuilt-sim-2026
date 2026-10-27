// 4414's RIPCURRENT as physical mechanisms (js/mech/mechanism.js). Their CAD and code aren't public:
// this is built from their 2026 technical binder (2026.team4414.com: its text and CAD renders) and
// what the team said in its Chief Delphi thread ("Team 4414 HighTide 2026 Tech Binder:
// RIPCURRENT"), with 604's Toploader (js/mech/r604.js, whose CAD and code are public) as the
// reference for how a Dye Rotor's parts work together. Every dimension below the binder doesn't
// give is read off its renders, or is an estimate (marked).
// Robot frame: x forward (the intake), y up, z to its right.
//
// The binder's numbers:
//   - drive: 25 x 32in swerve, 7.67:1; chamfered back corners; the bumper backing is the hopper's
//     walls ("~1in extra hopper space per side")
//   - Dye Rotor: "2x Kraken X60 for rollers and 1x Kraken X60 for rotation", "2.5:1 reduction on
//     feeder wheels", "36.4:1 reduction for rotation"; a "Dolphin Fin" ramp on the outside ("rotor
//     constantly spins in the opposite direction slowly to agitate balls and reduce packing
//     pressure"); the "Shrink Wrap" ("allow balls to fall down smoothly around the rotor and reduce
//     agitation while firing"); "passive plastic rollers on hook and ramp coupled with bearings
//     decrease current draw and allow balls to push each other through"; a vertical 3in omni wheel
//     over the 3in TTB urethane wheels; horizontal 1.25in GoBilda omni wheels onto the ramp.
//     The team: "We spin the rotor around 2.25rps which is about 20bps theoretical" and, gap-free,
//     12-14 BPS; "The feeder wheels on the shooter are the main bps regulator of the system", "the
//     rotor wheels run at much higher surface speeds, so balls briefly treadmill". (A moving floor,
//     "to actively pull balls toward the hook faster than a static floor", was in their rotor
//     prototypes V4, V5, V7 and V8, driven by a 20DP gear round its rim; the rotor that played, V9
//     and V10, has its big 10DP gear at the middle, under the hook.)
//   - intake: a primary 3in polycarbonate roller (two X60s, 3.45:1) and a bottom 1.5in roller (an
//     X44, 1.5:1), deployed at the start of the match and latched; the hopper box extends on
//     polycarbonate 10DP racks (an X44 a side, 3.33:1 at a 10t pinion): "holds 85+ balls under the
//     trench, 100+ with net over the bump"
//   - shooter: "4x Kraken X44 geared 1:1 to a 3in flywheel", copper mass disks; "3x 1in sushi wheel
//     feeder on one side geared down 16:14 with .5in compression"; "1in sushi wheel hood roller at
//     50% surface speed with 1in compression"; ".9in and .8in compression on hood backing 3d
//     prints"; the hood "coaxial with the shooter wheels" (the team), on a sector at 85.3:1 (an X44)
//   - turret: 55.7:1 (an X44)
//
// How they're put together here (the renders, and 604's): the Dye Rotor turns (36.4:1, clockwise
// from above) on the hopper's middle over a fixed floor sunk in a bowl: the stadium pieces funnel
// FUEL down to a wall standing straight up round the floor's rim (the clear ring in the renders),
// and the hook's tip runs flush against it. The hook, its face a row of little rollers standing up
// on bearings ("passive plastic rollers on hook ... coupled with bearings"), curves gradually from the rim in to the rotor's hub, its tip leading, and sweeps the
// FUEL on the floor along its face in to the hub's side. There, on the inside of its curve across
// the way in from the hook's end, is the one powered stack (RotorV10's render: the 3in TTB urethane
// wheels on a shaft standing up, the 3in omni over them, driven through the hub: "rotor wheels"),
// with the 1.25in omnis on their shaft across the entrance over the FUEL coming in (a row of small
// omni wheels, bevel-geared off the top of the stack): the stack drives FUEL in
// through the hub's side, under the omnis and up the ramp (rollers on bearings too, lying across
// it) into the hub's bore, each pushing the one
// ahead up into the turret on the same axis. Nothing else turns at the hook's root. The hook's back
// is the Dolphin Fin, a ramp FUEL ride up over when the rotor turns back; the hub's skin is the
// Shrink Wrap, a skirt FUEL slide off. In the turret the sushi feeder (geared off the same
// two X60s: "the main bps regulator") pushes them up into the flywheel, which flings them round
// under the hood and out past the hood roller.
import { IN, FUEL } from '../constants.js';
import { Motor, Shaft, Surface, Train, Wheel } from './mechanism.js';

const R = FUEL.radius;
// what each grips FUEL with (estimates, as 604's and 341's): the TTB urethane wheels, the stealth
// wheels on the flywheel, the sushi wheels (rubber), the intake's polycarbonate tubes, the omnis'
// rollers, the passive rollers (free: almost nothing along them), and plates and the hook's fixed
// standoffs
export const MU = { urethane: 1.1, stealth: 1.6, sushi: 1.0, polycarb: 0.5, omni: 0.6, rollers: 0.06, plate: 0.3 };

// ------------------------------------------------------------------ the layout (robot frame)
export const HALF_L = 12.5 * IN, HALF_W = 16 * IN; // the frame (25 x 32in)
// the Dye Rotor's axis, and the turret's on it (the renders: just behind the center)
export const AXIS = [-0.02, 0];
export const FLOOR = 0.105;      // the hopper's floor under the rotor (the renders: on the two cross rails)
export const BUCKET = { r: 0.285 }; // the floor's round middle the rotor sweeps
// the hopper: the bumper backing as walls, chamfered at the back corners, open at the front into
// the box; the top plate over the back (to the turret's bearing) and the net over the rest
// (the bumper backing is the hopper's wall: "~1in extra hopper space per side", the binder)
// (the frame's back corners cut 3.41in each way, as little as keeps its perimeter within 110in; the
// bumper backing 1in out from it, its cut (2 - sqrt2) in longer each way)
// The net stretches: pinned along the walls, the top plate's front edge, the box's front and round
// the turret's ring, the load can push it up into a dome as far as dome m over the hopper's top at
// its middle (robotConfigs' bay.dome: as far as there's room over the robot), and nothing presses
// the load back down (the binder: "holds 85+ balls under the trench, 100+ with net over the bump")
export const BAY = { x0: -(HALF_L + IN), hw: HALF_W + IN, chamfer: (4 / (2 * (2 - Math.SQRT2)) + 2 - Math.SQRT2) * IN, top: 0.535, net: 0.565, hole: 0.25, dome: 0.235 };
// the bowl round the rotor and the printed "stadium" pieces that funnel FUEL into it: the floor
// out to r, a wall standing straight up round its rim, the hook's tip running flush against it
// (wall high at the back and sides, fading to wallF at the front, where the stadium has to stay
// well under the bumper's top, BUMPER_TOP, or FUEL slosh forward over it into the box), and the
// stadium from its top up (slope) toward the hopper's walls, to cap over the floor (estimates: the
// renders' clear ring round the floor)
export const STADIUM = { r: 0.292, wall: 0.07, wallF: 0, slope: 0.25, cap: 0.1 };
export const BUMPER_TOP = 0.168;
// the rotor's hub (its "Shrink Wrap" skin) round the axis, and the skirt that sheds FUEL off it
// (estimates: the renders)
export const COLUMN = { r: 0.105, top: 0.36, skirt: [0.12, 0.345] };
// the rotor's frame (it turns: at phi = 0, as here): its channel from the hook's root in to the hub,
// along CH (rad from +x toward +z, the way the rotor turns: clockwise from above); u out along it,
// w the way it travels
export const CH = Math.PI / 2;
export const FEED = {
  w: 0.078,               // the channel's half width (FUEL's radius and a little)
  // the stack: two 3in TTB urethane wheels on a shaft standing up on the channel's leading side (the
  // inside of the hook's curve), u out along it, just outside the hub, and the 3in omni over them
  // ("to allow balls to fall down smoothly into the 3in TTB urethane wheels"), 0.8in into the FUEL
  // against the hook's end (estimates: the renders)
  ttb: { u: 0.105, r: 1.5 * IN, ys: [0.16, 0.197], omniY: 0.234, wid: 0.024, squeeze: 0.028 },
  // the entrance: under the 1.25in omnis on their shaft across it (driven: their undersides run in),
  // round the bend up into the hub
  omni: { r: 0.625 * IN, squeeze: 0.003, wid: 0.15 },
};
// FUEL's center along the channel, and the bend's center (u, y)
export const YB = FLOOR + R + 0.002;
export const BEND = { u: R + FEED.omni.r - FEED.omni.squeeze, y: YB + R + FEED.omni.r - FEED.omni.squeeze };
export const BORE = { r: R + 0.004, top: 0.345 };
// the ramp round the bend: its face (FUEL's side) RO m from the bend's center, a row of n rollers
// lying across it, each free on its bearings (r, J: estimates, the renders), the plate behind them
export const RAMP = { RO: BEND.u + R + 0.004, rollers: { n: 9, r: 0.0125, J: 2e-6 } };
// the hook: its face curves gradually from its tip, flush with the bowl's wall (tipA rad round from
// the channel, ahead), back and in to the hub's side on the channel's trailing side, coming in along
// the channel there (a cubic: lead m on along the wall from the tip, root m out along the channel
// from the hub); from just off the floor to y1, over a FUEL's middle and a little over the wall; its
// back the Dolphin Fin (finBack m behind its top edge, at the floor) (estimates: the renders)
// The rollers: n of them standing up along the face out to s = end (from the tip), each free on its
// bearings (r, and its inertia J: a plastic tube; estimates: the renders), their fronts on the
// curve FUEL ride along (hookCurve) and the plate behind them; past them, where it comes in to the
// hub, no rollers (the team), the plate itself the face
export const HOOK = { tipR: STADIUM.r - 0.003, tipA: 0.45, lead: 0.12, root: 0.1, y0: FLOOR + 0.006, y1: FLOOR + 0.1, finBack: 0.09, rollers: { n: 17, end: 0.75, r: 0.011, J: 1e-6 } };
// the rotor: the hook, the hub, the feeder wheels and their gears (estimate)
export const ROTOR = { J: 0.04 };

// the turret's frame: about AXIS, x along the shot; the flywheel (3in) FLY.x out from the axis,
// the FUEL going up the column meeting its back; .9in of squeeze between it and the backing low
// down (half each), .8in under the hood, 1in at the hood roller
export const FLY = { r: 1.5 * IN, y: 0.4, w: 0.12 };
const SQ = { low: 0.9 * IN, hood: 0.8 * IN, roller: 1.0 * IN, feeder: 0.5 * IN };
export const SHOOTER = {
  // FUEL's center round the flywheel's axle: the flywheel squeezing it by half the low backing's
  path: FLY.r + R - SQ.low / 2,
  // the axle's place: FUEL coming straight up the column meet the flywheel's lower back, ENTRY deg
  // round its axle (so they wrap it from there round to where the hood lets them go)
  entry: 210,
  get x() { return -this.path * Math.cos((this.entry * Math.PI) / 180); },
  // the backings (their 3D prints) round the axle: the fixed one low down (fixedA, deg),
  // the hood's past it (HOOD.span deg from where it lets go)
  fixedR: FLY.r + 2 * R - SQ.low, fixedA: [150, 200],
  hoodR: FLY.r + 2 * R - SQ.hood,
  // the hood roller (1in sushi wheel, 50% of the flywheel's surface speed) just inside the hood's end
  roller: { r: 0.5 * IN, at: 4, w: 0.1 },
  // the sushi feeder: three 1in wheels on the column's front side under the flywheel, into the
  // FUEL against the column's back wall
  feeder: { r: 0.5 * IN, ys: [0.29, 0.315, 0.34], w: 0.06 },
  // what turns with the flywheel: four stealth wheels with copper and mass disks between (estimate:
  // six 3in x 1/4in copper disks, 0.26 kg each, and the wheels), the rotors are added
  J: 1.25e-3,
  // the hood: where it lets FUEL go, deg round the axle from forward (90 is straight up over it):
  // a shot leaves about 90 below that (the tangent); its travel (estimate, from the renders), its
  // speed (an X44 through 85.3:1)
  hoodMin: 125, hoodMax: 160, hoodStow: 125, hoodSpan: 45, hoodV: 7, hoodA: 60,
  // the side plates either side of the FUEL
  plateZ: R + 0.006,
};
export const HOOD = SHOOTER;
// the intake, at the box's full extension (it slides out BOX.ext on its racks); xF its front
export const BOX = { ext: 0.27, front: HALF_L + 3.25 * IN + 0.2224, side: HALF_W + IN + 0.005, chin: 0.31 };
export const INTAKE = {
  // No ramp, the box's bottom open (the renders, from above; the team): the primary 3in
  // polycarbonate roller, the kicker, low just in front of the bumper's face, its back running up
  // it, squeezes a FUEL against the bumper (0.8in) and rolls it up the face; the 1.5in roller
  // sitting on top of the bumper, its top running in, takes it over into the hopper (dx back from
  // the box's front at its full extension; their places are estimates from the renders). The small
  // roller is dark, in short segments in the renders: compliant, gripping as rubber does (mu, tread)
  primary: { dx: -0.07, y: 0.155, r: 1.5 * IN },
  bar: { dx: -0.23, y: 0.192, r: 0.75 * IN, mu: 1.0, tread: 20000 },
  width: 0.8,
};
// the preload: eight on the floor round the hub, clear of the hook
export const PRELOAD = [0, 1, 2, 3, 4, 5, 6, 7].map((k) => {
  const a = CH + 0.65 + (k / 7) * (2 * Math.PI - 1.3);
  return [AXIS[0] + 0.19 * Math.cos(a), FLOOR + R + 0.01, AXIS[1] + 0.19 * Math.sin(a)];
});

// ------------------------------------------------------------------ the drives
export function build4414() {
  // the intake: the primary roller, the kicker (positive rolls FUEL up the bumper's face), and the
  // 1.5in roller on top of the bumper (positive takes them in over it)
  const primary = new Shaft({ name: 'intake primary 3in roller', motor: new Motor('krakenX60', { count: 2, ratio: 3.45, statorLimit: 60, supplyLimit: 40 }), inertia: 3e-4 });
  const bar = new Shaft({ name: 'intake bottom 1.5in roller', motor: new Motor('krakenX44', { ratio: 1.5, statorLimit: 40, supplyLimit: 30 }), inertia: 4e-5 });
  // the rotor (positive turns it clockwise from above, the hook's tip leading)
  const rotor = new Shaft({ name: 'Dye Rotor', motor: new Motor('krakenX60', { ratio: 36.4, statorLimit: 80, supplyLimit: 60 }), inertia: ROTOR.J, drag: 0.02 });
  // the rollers: the TTB wheels and the omni over them (2.5:1; positive feeds FUEL in)
  const rollers = new Train({ name: 'Dye Rotor rollers', motor: new Motor('krakenX60', { count: 2, ratio: 2.5, statorLimit: 60, supplyLimit: 40 }) });
  const ttb = rollers.add(new Shaft({ name: 'TTB feeder wheels', inertia: 2.4e-4 }), 1);
  // the 1.25in omnis over the entrance: bevel-geared off the top of the stack (the renders; the team:
  // they spin more than twice as fast), 3:1.25 so their surface runs with the 3in wheels' (estimate)
  const bend = rollers.add(new Shaft({ name: '1.25in omnis', inertia: 1e-5, drag: 1e-5 }), 3 / 1.25);
  // the launcher: four X44s 1:1 on the flywheel; the hood roller at half its surface speed
  const launcher = new Train({ name: 'launcher', motor: new Motor('krakenX44', { count: 4, ratio: 1, statorLimit: 60, supplyLimit: 40 }) });
  const fly = launcher.add(new Shaft({ name: '3in flywheel and copper', inertia: SHOOTER.J }), 1);
  const hoodRoller = launcher.add(new Shaft({ name: 'hood sushi roller', inertia: 5e-6 }), (0.5 * FLY.r) / SHOOTER.roller.r);
  // the sushi feeder: geared down 16:14 off the flywheel (its gears down the turret's side, in the
  // renders), so it feeds as fast as the shot's speed lets it: "the main bps regulator"
  const feeder = launcher.add(new Shaft({ name: 'shooter sushi feeder', inertia: 1e-5 }), 14 / 16);
  return { primary, bar, rotor, rollers, ttb, feeder, bend, launcher, fly, hoodRoller };
}

const X = [1, 0, 0], Y = [0, 1, 0], Z = [0, 0, 1];
// the channel's frame -> the rotor's (about the axis, at phi = 0): (u out along it, y, w ahead)
export function chanL(u, y, w) {
  const c = Math.cos(CH), s = Math.sin(CH);
  return [u * c - w * s, y, u * s + w * c];
}
// and the robot's
export function chan(u, y, w) { const p = chanL(u, y, w); return [AXIS[0] + p[0], y, AXIS[1] + p[2]]; }
const chanDir = (u, w) => { const c = Math.cos(CH), s = Math.sin(CH); return [u * c - w * s, 0, u * s + w * c]; };

// the wheels (robot frame; the box's posed with placeIntake, the turret's with placeShooter)
export function wheels4414(t) {
  const F = FEED, T = F.ttb;
  const tw = R + T.r - T.squeeze; // the stack's shaft's distance ahead of the channel's middle
  // (on the rotor: they turn with it about the axis, and FUEL they touch push back on its motor;
  // the stack's face toward the channel runs in toward the hub, the one toward the bowl back
  // toward the hook)
  const feed = [
    ...T.ys.map((y, i) => new Wheel({ name: `TTB 3in urethane ${i + 1}`, shaft: t.ttb, center: chan(T.u, y, tw), axis: [0, 1, 0], radius: T.r, width: T.wid, mu: MU.urethane, tread: 15000 })),
    new Wheel({ name: 'TTB 3in omni', shaft: t.ttb, kind: 'omni', center: chan(T.u, T.omniY, tw), axis: [0, 1, 0], radius: T.r, width: T.wid, mu: MU.omni, tread: 30000 }),
    // (the hook's rollers, each on its own bearings: FUEL roll along the hook almost freely, but
    // not up or down it)
    ...hookRollerAt().map((c, i) => new Wheel({ name: 'hook rollers', shaft: new Shaft({ name: `hook roller ${i + 1}`, inertia: HOOK.rollers.J, drag: 1e-6 }), center: c, axis: [0, 1, 0], radius: HOOK.rollers.r, width: HOOK.y1 - HOOK.y0 - 0.006, mu: MU.plate })),
    // (the ramp's rollers, each on its own bearings, lying across the channel)
    ...rampRollerAt().map((c, i) => new Wheel({ name: 'ramp rollers', shaft: new Shaft({ name: `ramp roller ${i + 1}`, inertia: RAMP.rollers.J, drag: 1e-6 }), center: c, axis: chanDir(0, 1), radius: RAMP.rollers.r, width: 2 * F.w - 0.01, mu: MU.plate })),
    // (the entrance's omnis: their shaft across the channel, turning so their undersides run in)
    new Wheel({ name: '1.25in omnis', shaft: t.bend, kind: 'omni', center: chan(BEND.u, BEND.y, 0), axis: chanDir(0, -1), radius: F.omni.r, width: F.omni.wid, mu: MU.omni, tread: 30000 }),
  ].map((w) => w.onPivot(t.rotor, [AXIS[0], 0, AXIS[1]], [0, -1, 0]));
  feed.forEach((w) => { w.home = w.center.slice(); w.homeAxis = w.axis.slice(); });
  const intake = [
    new Wheel({ name: 'intake bottom 1.5in roller', shaft: t.bar, center: [0, 0, 0], axis: Z, radius: INTAKE.bar.r, width: INTAKE.width, mu: INTAKE.bar.mu ?? MU.polycarb, tread: INTAKE.bar.tread ?? 60000 }),
    new Wheel({ name: 'intake primary 3in roller', shaft: t.primary, center: [0, 0, 0], axis: [0, 0, -1], radius: INTAKE.primary.r, width: INTAKE.width, mu: MU.polycarb, tread: 60000 }),
  ];
  placeIntake(intake, BOX.ext);
  return { feed, intake };
}
// the hook's rollers' centers (robot frame, the rotor at phi = 0): their fronts on the curve
export function hookRollerAt() {
  const n = HOOK.rollers.n, y = (HOOK.y0 + HOOK.y1) / 2;
  return Array.from({ length: n }, (_, i) => { const { r, a } = hookBack(((i + 0.5) / n) * HOOK.rollers.end, HOOK.rollers.r); return [AXIS[0] + r * Math.cos(a), y, AXIS[1] + r * Math.sin(a)]; });
}
// the ramp's rollers' centers (robot frame, the rotor at phi = 0): their faces where the ramp's face
// is, from under the bend up to the bore's far side
export function rampRollerAt() {
  const n = RAMP.rollers.n, rc = RAMP.RO + RAMP.rollers.r;
  return Array.from({ length: n }, (_, i) => { const al = -Math.PI / 2 - ((i + 0.5) / n) * (Math.PI / 2); return chan(BEND.u + rc * Math.cos(al), BEND.y + rc * Math.sin(al), 0); });
}
// a point (robot frame) turned with the rotor by phi (rad, clockwise from above) about its axis
export function rotorPoint([x, y, z], phi) {
  const dx = x - AXIS[0], dz = z - AXIS[1], c = Math.cos(phi), s = Math.sin(phi);
  return [AXIS[0] + dx * c - dz * s, y, AXIS[1] + dx * s + dz * c];
}
export function placeRotor(wheels, phi) {
  const c = Math.cos(phi), s = Math.sin(phi);
  for (const w of wheels) {
    w.center = rotorPoint(w.home, phi);
    const [ax, ay, az] = w.homeAxis;
    w.axis = [ax * c - az * s, ay, ax * s + az * c];
  }
}
// the intake's rollers with the box out ext (m)
export function placeIntake([bar, primary], ext) {
  const xF = BOX.front - BOX.ext + ext, I = INTAKE;
  bar.center = [xF + I.bar.dx, I.bar.y, 0];
  primary.center = [xF + I.primary.dx, I.primary.y, 0];
}
export function shooterWheels(t) {
  const S = SHOOTER;
  const w = [
    new Wheel({ name: '3in flywheel (stealth wheels)', shaft: t.fly, center: [0, 0, 0], axis: [0, 0, -1], radius: FLY.r, width: FLY.w, mu: MU.stealth, tread: 30000 }),
    new Wheel({ name: 'hood 1in sushi roller', shaft: t.hoodRoller, center: [0, 0, 0], axis: Z, radius: S.roller.r, width: S.roller.w, mu: MU.sushi, tread: 15000 }),
    ...S.feeder.ys.map((y, i) => new Wheel({ name: `1in sushi feeder ${i + 1}`, shaft: t.feeder, center: [0, y, 0], axis: [0, 0, -1], radius: S.feeder.r, width: S.feeder.w, mu: MU.sushi, tread: 15000 })),
  ];
  w.forEach((x) => { x.home = x.center.slice(); });
  placeShooter(w, 0, S.hoodStow);
  return w;
}
// where the hood lets FUEL go (deg round the flywheel's axle) -> the shot's elevation (deg): the
// tangent there
export const hoodElevation = (deg) => deg - 90;
// the shooter's wheels where the turret (yaw, rad; 0 facing forward) and the hood (deg) put them
export function placeShooter([fly, roller, ...feeders], yaw, hoodDeg) {
  const c = Math.cos(yaw), s = Math.sin(yaw), S = SHOOTER;
  const at = ([x, y]) => [AXIS[0] + x * c, y, AXIS[1] - x * s];
  fly.center = at([S.x, FLY.y]);
  const b = ((hoodDeg + S.roller.at) * Math.PI) / 180, rr = S.hoodR + S.roller.r - (SQ.roller - SQ.hood);
  roller.center = at([S.x + rr * Math.cos(b), FLY.y + rr * Math.sin(b)]);
  const fx = -BORE.r + 2 * R - SQ.feeder + S.feeder.r; // into the FUEL against the back wall
  feeders.forEach((f) => { f.center = at([fx, f.home[1]]); });
  // (the hood roller turns the other way from the flywheel and the feeder: their surfaces all carry
  // FUEL along)
  for (const w of [fly, roller, ...feeders]) { const k = w === roller ? 1 : -1; w.axis = [k * s, 0, k * c]; }
}
// the bumper's foam: two 2.5in noodles across the front, as 604's
export function bumper4414() {
  const shaft = new Shaft({ name: 'bumper', inertia: Infinity });
  const r = 1.25 * IN, x = HALF_L + 3.25 * IN - r;
  return [0.032 + r, 0.16 - r].map((y) => new Wheel({ name: 'bumper', shaft, center: [x, y, 0], axis: Z, radius: r, width: 2 * HALF_W, mu: 0.6, tread: 20000 }));
}

// ------------------------------------------------------------------ the surfaces (built here: no CAD)
// a grid surface: f(u, v) -> [x, y, z], u in 0..1 over nu cells, v over nv
function grid(nu, nv, f) {
  const v = [], i = [];
  for (let a = 0; a <= nu; a++) for (let b = 0; b <= nv; b++) v.push(...f(a / nu, b / nv));
  for (let a = 0; a < nu; a++) for (let b = 0; b < nv; b++) { const p = a * (nv + 1) + b, q = p + nv + 1; i.push(p, q, p + 1, q, q + 1, p + 1); }
  return { v, i };
}
function merge(...ms) {
  const v = [], i = [];
  for (const m of ms) { const o = v.length / 3; v.push(...m.v); for (const k of m.i) i.push(k + o); }
  return { v, i };
}
const lerp = (a, b, t) => a + (b - a) * t;
// a vertical sheet along a polyline [[x, z], ...] from y0 to y1
const wall = (pts, y0, y1, n = 2) => merge(...pts.slice(1).map((p, k) => {
  const q = pts[k], L = Math.hypot(p[0] - q[0], p[1] - q[1]), nu = Math.max(1, Math.ceil(L / 0.06));
  return grid(nu, n, (u, v) => [lerp(q[0], p[0], u), lerp(y0, y1, v), lerp(q[1], p[1], u)]);
}));
// a cylinder's wall round (cx, cz), radius r, y0..y1, angles a0..a1 (rad, from +x toward +z)
const cyl = (cx, cz, r, y0, y1, a0 = 0, a1 = 2 * Math.PI) => grid(Math.max(4, Math.ceil(((a1 - a0) * r) / 0.03)), 2, (u, v) => {
  const a = lerp(a0, a1, u);
  return [cx + r * Math.cos(a), lerp(y0, y1, v), cz + r * Math.sin(a)];
});

// the hopper's floor: flat under the rotor, the bowl's wall round it, the stadium pieces from its top
// out to the walls, the front over the bumper's top (the rotor's hub in the middle): its height at
// (x, z)
export function stadiumY(x, z) {
  const S = STADIUM, d = Math.hypot(x - AXIS[0], z - AXIS[1]);
  // (the wall lower toward the front, wallF there: under the bumper's top, so FUEL coming over it
  // roll down into the bowl and none roll out of it into the box)
  const c = Math.max(0, (x - AXIS[0]) / (d || 1)), wall = lerp(S.wall, S.wallF, c * c);
  let y = d < S.r ? FLOOR : FLOOR + Math.min(S.cap, wall + (d - S.r) * S.slope);
  // (over the front rail and the bumper's top, at its height)
  if (x > HALF_L - 0.02) y = Math.max(y, lerp(y, BUMPER_TOP, Math.min(1, (x - HALF_L + 0.02) / 0.035)));
  return y;
}
function hopperFloor() {
  const S = STADIUM, x0 = BAY.x0, x1 = HALF_L + 3.25 * IN, hw = BAY.hw, na = 120, nr = 24;
  // (polar, so the rim is a true circle the hook's tip runs round: the floor inside it, the stadium
  // out from it to the walls and the box, finer near the rim)
  const out = (a) => {
    const c = Math.cos(a), s = Math.sin(a);
    return Math.min(c > 1e-6 ? (x1 - AXIS[0]) / c : c < -1e-6 ? (x0 - AXIS[0]) / c : Infinity, Math.abs(s) > 1e-6 ? (hw - Math.sign(s) * AXIS[1]) / Math.abs(s) : Infinity);
  };
  const floor = grid(na, 2, (u, v) => { const a = u * 2 * Math.PI, r = lerp(COLUMN.r - 0.01, S.r, v); return [AXIS[0] + r * Math.cos(a), FLOOR, AXIS[1] + r * Math.sin(a)]; });
  const stadium = grid(na, nr, (u, v) => {
    const a = u * 2 * Math.PI, r = lerp(S.r + 1e-4, out(a), v * v), x = AXIS[0] + r * Math.cos(a), z = AXIS[1] + r * Math.sin(a);
    return [x, stadiumY(x, z), z];
  });
  // (the bowl's wall, up to the stadium's edge all round)
  const wall = grid(na, 2, (u, v) => { const a = u * 2 * Math.PI, x = AXIS[0] + S.r * Math.cos(a), z = AXIS[1] + S.r * Math.sin(a); return [x, lerp(FLOOR - 0.005, stadiumY(x + 1e-4 * Math.cos(a), z + 1e-4 * Math.sin(a)), v), z]; });
  return merge(floor, stadium, wall);
}
// The rotor's parts, in its own frame (about the axis, at phi = 0).
// the hook's face (passive rollers): s 0 at its tip, 1 at its root
export function hookCurve(s) {
  const H = HOOK, ta = H.tipA;
  // (in the channel's frame (u, w): from the tip on round the wall, the way it goes back, to the hub's
  // side coming in along the channel; never out past the wall)
  const P = [[H.tipR * Math.cos(ta), H.tipR * Math.sin(ta)], null, [COLUMN.r + H.root, -FEED.w], [COLUMN.r, -FEED.w]];
  P[1] = [P[0][0] + H.lead * Math.sin(ta), P[0][1] - H.lead * Math.cos(ta)];
  const q = 1 - s, k = [q * q * q, 3 * q * q * s, 3 * q * s * s, s * s * s];
  const u = k.reduce((a, c, i) => a + c * P[i][0], 0), w = k.reduce((a, c, i) => a + c * P[i][1], 0);
  return { r: Math.min(H.tipR, Math.hypot(u, w)), a: CH + Math.atan2(w, u) };
}
// the curve at s, d m back from it (behind, away from the FUEL it sweeps)
export function hookBack(s, d) {
  const p = (q) => { const { r, a } = hookCurve(Math.min(1, Math.max(0, q))); return [r * Math.cos(a), r * Math.sin(a), a]; };
  const [x, z, a] = p(s), [x0, z0] = p(s - 0.005), [x1, z1] = p(s + 0.005);
  let nx = -(z1 - z0), nz = x1 - x0; const l = Math.hypot(nx, nz) || 1; nx /= l; nz /= l;
  // (the normal facing ahead, the way the rotor turns: +a)
  if (nx * -Math.sin(a) + nz * Math.cos(a) < 0) { nx = -nx; nz = -nz; }
  const bx = x - d * nx, bz = z - d * nz;
  return { r: Math.hypot(bx, bz), a: Math.atan2(bz, bx) };
}
// the hook's plate: behind its rollers, then (past them, over 0.05 of its length) the face itself
export const hookPlate = (s) => { const R0 = HOOK.rollers; return hookBack(s, (2 * R0.r + 0.003) * Math.min(1, Math.max(0, (R0.end + 0.05 - s) / 0.05))); };
const polar = (r, a, y) => [r * Math.cos(a), y, r * Math.sin(a)];
function hook() {
  const H = HOOK;
  const face = grid(32, 3, (u, v) => { const { r, a } = hookPlate(u); return polar(r, a, lerp(H.y0, H.y1, v)); });
  // the Dolphin Fin: from the plate's top edge down behind it to the floor
  const fin = grid(32, 2, (u, v) => { const { r, a } = hookPlate(u); return polar(r, a - (v * H.finBack) / r, lerp(H.y1, FLOOR + 0.006, v)); });
  return merge(face, fin);
}
// the channel, the bend and the hub's bore
function feedPath() {
  const F = FEED, w = F.w, B = BEND, RO = RAMP.RO + 0.004; // (the plate a little behind the rollers' faces)
  const parts = [];
  // the trailing wall in the hub (the hook's side, out to its end at the hub's side)
  parts.push(grid(6, 4, (u, v) => chanL(lerp(-B.u, COLUMN.r, u), lerp(FLOOR, BORE.top, v), -w)));
  // the hub's floor under the channel
  parts.push(grid(4, 2, (u, v) => chanL(lerp(B.u, COLUMN.r + 0.005, u), FLOOR + 0.001, lerp(-w, w, v))));
  // the ramp's plate round the bend, behind its rollers: the outside of the bend, from under the
  // channel up to the bore's far side
  parts.push(grid(12, 3, (u, v) => {
    const al = lerp(-Math.PI / 2, -Math.PI, u);
    return chanL(B.u + RO * Math.cos(al), B.y + RO * Math.sin(al), lerp(-w, w, v));
  }));
  return merge(...parts);
}
// the bore up the hub, and the channel's leading side in the hub (plain walls: the sushi feeder rolls
// FUEL along them; against free rollers it'd only spin them)
function bore() {
  const B = BEND, w = FEED.w, parts = [cyl(0, 0, BORE.r, B.y, BORE.top)];
  // (the leading wall in the hub; out of it, the stack)
  parts.push(grid(6, 4, (u, v) => chanL(lerp(-B.u, COLUMN.r, u), lerp(FLOOR, BORE.top, v), w)));
  return merge(...parts);
}
// the hub's skin (Shrink Wrap) and the skirt off its top, the channel's slot cut in its side
function column() {
  const C = COLUMN, slot = Math.asin(Math.min(1, (FEED.w + 0.005) / C.r));
  const [sr, sy] = C.skirt;
  return merge(
    cyl(0, 0, C.r, FLOOR, C.top, CH + slot, CH + 2 * Math.PI - slot),
    cyl(0, 0, C.r, YB + R + 0.01, C.top, CH - slot, CH + slot),
    // the skirt: down and out from the hub's top to over the hook's root
    grid(48, 3, (u, v) => polar(lerp(C.r, sr, v), u * 2 * Math.PI, lerp(C.top, sy, v))),
  );
}
// the walls, the top plate and the net
function hopperWalls() {
  const { x0, hw, chamfer } = BAY, xf = HALF_L + 0.01;
  const pts = [[xf, -hw], [x0 + chamfer, -hw], [x0, -hw + chamfer], [x0, hw - chamfer], [x0 + chamfer, hw], [xf, hw]];
  return wall(pts, FLOOR - 0.01, BAY.net + 0.02, 6);
}
// (the drop of triangles whose middles are inside a circle round the axis)
function holed(m, r) {
  const keep = [];
  for (let t = 0; t < m.i.length; t += 3) {
    const cx = (m.v[m.i[t] * 3] + m.v[m.i[t + 1] * 3] + m.v[m.i[t + 2] * 3]) / 3, cz = (m.v[m.i[t] * 3 + 2] + m.v[m.i[t + 1] * 3 + 2] + m.v[m.i[t + 2] * 3 + 2]) / 3;
    if (Math.hypot(cx - AXIS[0], cz - AXIS[1]) > r) keep.push(m.i[t], m.i[t + 1], m.i[t + 2]);
  }
  return { v: m.v, i: keep };
}
function topPlate() {
  // the plate over the back to the turret's bearing (a hard top); the net over the rest (soft),
  // open round the turret (its hood stands up through it and shots go out over it)
  return grid(6, 12, (u, v) => [lerp(BAY.x0, -0.21, u), BAY.top, lerp(-BAY.hw, BAY.hw, v)]);
}
// the net's height at (x, z), stretched all the way: pinned at its edges and round the turret's
// ring, up between
const NET_X = [-0.21, BOX.front + 0.01];
export function netY(x, z) {
  const u = Math.min(1, Math.max(0, (x - NET_X[0]) / (NET_X[1] - NET_X[0]))), v = Math.min(1, Math.max(0, (z + BOX.side) / (2 * BOX.side)));
  const d = Math.hypot(x - AXIS[0], z - AXIS[1]), ring = Math.min(1, Math.max(0, (d - BAY.hole) / 0.12));
  return BAY.net + BAY.dome * Math.sin(Math.PI * u) * Math.sin(Math.PI * v) * ring * ring * (3 - 2 * ring);
}
function net() {
  return holed(grid(28, 32, (u, v) => { const x = lerp(NET_X[0], NET_X[1], u), z = lerp(-BOX.side, BOX.side, v); return [x, netY(x, z), z]; }), BAY.hole);
}
// the box (it slides out): its front panel over the intake and its sides, its bottom open (at its
// full extension; placed with placeBox)
function box() {
  const xF = BOX.front, s = BOX.side;
  return merge(
    grid(2, 14, (u, v) => [xF, lerp(BOX.chin, BAY.net + 0.02, u), lerp(-s, s, v)]),
    // (the chin: bent back at the front panel's foot)
    grid(1, 14, (u, v) => [lerp(xF, xF - 0.06, u), BOX.chin, lerp(-s, s, v)]),
    ...[-1, 1].map((k) => grid(6, 4, (u, v) => [lerp(HALF_L - 0.02, xF, u), lerp(0.03, BAY.net + 0.02, v), k * s])),
  );
}
// the turret (it turns): the column's top where FUEL come up (its back wall), the fixed backing,
// the side plates
function turret() {
  const S = SHOOTER, B = BORE, F = FLY;
  const back = grid(3, 4, (u, v) => { const a = lerp(Math.PI / 2 + 0.6, 1.5 * Math.PI - 0.6, v); return [B.r * Math.cos(a), lerp(B.top, F.y - 0.01, u), B.r * Math.sin(a)]; });
  const fixed = grid(8, 4, (u, v) => { const a = (lerp(S.fixedA[0], S.fixedA[1], u) * Math.PI) / 180; return [S.x + S.fixedR * Math.cos(a), F.y + S.fixedR * Math.sin(a), lerp(-S.plateZ, S.plateZ, v)]; });
  const plates = [-1, 1].map((k) => grid(8, 6, (u, v) => [lerp(-0.1, 0.32, u), lerp(B.top - 0.01, 0.64, v), k * S.plateZ]));
  return merge(back, fixed, ...plates);
}
// the hood's backing (it turns about the flywheel's axle): its local frame there, the end where it
// lets FUEL go at 0 rad (posed with roll = that end's angle)
function hood() {
  const S = SHOOTER;
  return grid(10, 4, (u, v) => { const a = (u * S.hoodSpan * Math.PI) / 180; return [S.hoodR * Math.cos(a), S.hoodR * Math.sin(a), lerp(-S.plateZ, S.plateZ, v)]; });
}

export function pathSurfaces4414(t) {
  const s = (name, m, o = {}) => new Surface({ name, vertices: m.v, indices: m.i, mu: MU.plate, ...o });
  const body = s('hopper (floor, walls, top plate)', merge(hopperFloor(), hopperWalls(), topPlate()));
  const netS = s('net', net(), { mu: 0.4, tread: 1500 });
  // (the rotor's: it turns, and FUEL they touch push back on its motor)
  const A = [AXIS[0], 0, AXIS[1]], onRotor = (sf) => sf.onShaft(t.rotor, A, [0, -1, 0]);
  const hookS = onRotor(s('hook and Dolphin Fin', hook(), { mu: MU.plate, pos: A }));
  const feedS = onRotor(s('channel and ramp (passive rollers)', feedPath(), { mu: MU.rollers, pos: A }));
  const boreS = onRotor(s('bore', bore(), { pos: A }));
  const columnS = onRotor(s('hub and skirt', column(), { mu: MU.rollers, pos: A }));
  const boxS = s('box and intake ramp', box());
  const turretS = s('turret', turret(), { pos: [AXIS[0], 0, AXIS[1]] });
  const hoodS = s('hood backing', hood(), { pos: [AXIS[0] + SHOOTER.x, FLY.y, AXIS[1]] });
  return { body, net: netS, hook: hookS, feed: feedS, bore: boreS, column: columnS, box: boxS, turret: turretS, hood: hoodS };
}
// the rotor's surfaces at phi (clockwise from above: a surface's yaw is the other way)
export function placeRotorSurfaces({ hook, feed, bore, column }, phi) { for (const sf of [hook, feed, bore, column]) sf.setPose([AXIS[0], 0, AXIS[1]], -phi, 0); }
export function placeBox(sf, ext) { sf.setPose([ext - BOX.ext, 0, 0]); }
export function placeTurretSurfaces({ turret, hood }, yaw, hoodDeg) {
  turret.setPose([AXIS[0], 0, AXIS[1]], yaw, 0);
  const c = Math.cos(yaw), s = Math.sin(yaw);
  hood.setPose([AXIS[0] + SHOOTER.x * c, FLY.y, AXIS[1] - SHOOTER.x * s], yaw, (hoodDeg * Math.PI) / 180);
}
