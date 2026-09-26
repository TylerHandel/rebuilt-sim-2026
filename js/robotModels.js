// Robot 3D models built from primitives. Local frame: +X = robot front (intake side),
// +Z = robot right, Y = up, origin on the floor at the robot center.
//
// Each builder returns { root, anim(state, dt), stored[], modules[], extLen }:
//   stored   - FUEL positions shown in the robot (filled in order); entries flagged ext:true sit
//              in the extending part of the hopper and slide out with it
//   modules  - swerve modules { pivot, wheel } animated by robot.js
//   extLen   - how far (m) the hopper extension slides out at full deploy (0 if none)
import * as THREE from 'three';
import { RoundedBoxGeometry } from 'three/addons/geometries/RoundedBoxGeometry.js';
import { IN, FUEL } from './constants.js';
import { BUMPER_T } from './robotConfigs.js';
import { ALLIANCE_COLOR_CSS } from './field.js';

export const BUMP_Y0 = 0.055, BUMP_Y1 = 0.16;
const lerp = THREE.MathUtils.lerp;

// ------------------------------------------------------------------ materials
// knotted net: one texture tile per meter of UV (shape geometry UVs are in meters), 2in cells
function netTexture() {
  const c = document.createElement('canvas');
  c.width = c.height = 256;
  const g = c.getContext('2d');
  g.clearRect(0, 0, 256, 256);
  g.strokeStyle = '#ffffff';
  g.lineWidth = 2;
  const n = 20;
  for (let i = 0; i <= n; i++) {
    const p = (i * 256) / n;
    g.beginPath(); g.moveTo(p, 0); g.lineTo(p, 256); g.stroke();
    g.beginPath(); g.moveTo(0, p); g.lineTo(256, p); g.stroke();
  }
  const t = new THREE.CanvasTexture(c);
  t.wrapS = t.wrapT = THREE.RepeatWrapping;
  t.anisotropy = 4;
  return t;
}

function std(color, rough = 0.55, metal = 0.2, extra = {}) {
  return new THREE.MeshStandardMaterial({ color, roughness: rough, metalness: metal, ...extra });
}
const M = {
  alu: std(0xb4b9c1, 0.32, 0.85),
  aluDark: std(0x4a4f58, 0.4, 0.75),
  anodBlack: std(0x1d1f24, 0.45, 0.55),
  black: std(0x131417, 0.7, 0.15),
  tread: std(0x0e0e10, 0.95, 0.0),
  hubGrey: std(0x6b7078, 0.4, 0.7),
  kraken: std(0x22252b, 0.35, 0.6),
  krakenRing: std(0xc9ccd2, 0.3, 0.9),
  // polycarbonate: lightly tinted so you can tell it's there
  poly: new THREE.MeshPhysicalMaterial({ color: 0xa9c4dd, transparent: true, opacity: 0.26, roughness: 0.05, depthWrite: false, side: THREE.DoubleSide }),
  polyTint: new THREE.MeshPhysicalMaterial({ color: 0x8aa3bb, transparent: true, opacity: 0.32, roughness: 0.08, depthWrite: false, side: THREE.DoubleSide }),
  compliant: std(0x2b2b2e, 0.9, 0.0),
  green: std(0x3fae49, 0.6, 0.1),
  orangeWheel: std(0xe0782a, 0.7, 0.05),
  blueWheel: std(0x2d6fd6, 0.7, 0.05),
  copper: std(0xb87333, 0.35, 0.9),
  battery: std(0x1a1a1a, 0.8, 0.05),
  red: std(0xc62828, 0.5, 0.1),
  rioGrey: std(0x9ea3aa, 0.4, 0.5),
  radioWhite: std(0xe8e8e8, 0.6, 0.0),
  lens: std(0x1e2b3c, 0.1, 0.8, { emissive: 0x0b1a33 }),
  led: std(0x39ff6a, 0.3, 0, { emissive: 0x39ff6a, emissiveIntensity: 1.1 }),
  polyBelt: std(0x3a3d42, 0.8, 0.0),
  net: new THREE.MeshStandardMaterial({ map: netTexture(), transparent: true, alphaTest: 0.35, side: THREE.DoubleSide, roughness: 0.9, color: 0x1c1c1c }),
};

function mesh(geo, mat, parent, x = 0, y = 0, z = 0) {
  const m = new THREE.Mesh(geo, mat);
  m.position.set(x, y, z);
  m.castShadow = true;
  m.receiveShadow = true;
  parent.add(m);
  return m;
}
const bx = (w, h, d, mat, parent, x, y, z) => mesh(new THREE.BoxGeometry(w, h, d), mat, parent, x, y, z);
const rbx = (w, h, d, r, mat, parent, x, y, z) => mesh(new RoundedBoxGeometry(w, h, d, 2, r), mat, parent, x, y, z);
// cylinder with its axis along local Z
function cylZ(r, len, mat, parent, x, y, z, segs = 20) {
  const m = mesh(new THREE.CylinderGeometry(r, r, len, segs), mat, parent, x, y, z);
  m.rotation.x = Math.PI / 2;
  return m;
}
function cylX(r, len, mat, parent, x, y, z, segs = 20) {
  const m = mesh(new THREE.CylinderGeometry(r, r, len, segs), mat, parent, x, y, z);
  m.rotation.z = Math.PI / 2;
  return m;
}
const cylY = (r, len, mat, parent, x, y, z, segs = 20) => mesh(new THREE.CylinderGeometry(r, r, len, segs), mat, parent, x, y, z);

// ------------------------------------------------------------------ parts library
// 2x1 aluminum tube between two points in the XZ plane at height y (runs along X or Z)
function tube(parent, x0, z0, x1, z1, y, mat = M.alu, w = 1 * IN, h = 2 * IN) {
  const len = Math.hypot(x1 - x0, z1 - z0);
  const m = rbx(len, h, w, 0.003, mat, parent, (x0 + x1) / 2, y, (z0 + z1) / 2);
  m.rotation.y = -Math.atan2(z1 - z0, x1 - x0);
  return m;
}

// flat plate with round lightening pockets, lying in the XY plane (thickness along Z)
function pocketPlate(parent, w, h, t, mat, holes = [], x = 0, y = 0, z = 0) {
  const s = new THREE.Shape();
  const r = Math.min(0.012, w / 6, h / 6);
  s.moveTo(-w / 2 + r, -h / 2);
  s.lineTo(w / 2 - r, -h / 2); s.quadraticCurveTo(w / 2, -h / 2, w / 2, -h / 2 + r);
  s.lineTo(w / 2, h / 2 - r); s.quadraticCurveTo(w / 2, h / 2, w / 2 - r, h / 2);
  s.lineTo(-w / 2 + r, h / 2); s.quadraticCurveTo(-w / 2, h / 2, -w / 2, h / 2 - r);
  s.lineTo(-w / 2, -h / 2 + r); s.quadraticCurveTo(-w / 2, -h / 2, -w / 2 + r, -h / 2);
  for (const [hx, hy, hr] of holes) { const p = new THREE.Path(); p.absarc(hx, hy, hr, 0, Math.PI * 2, true); s.holes.push(p); }
  const g = new THREE.ExtrudeGeometry(s, { depth: t, bevelEnabled: false, curveSegments: 12 });
  g.translate(0, 0, -t / 2);
  return mesh(g, mat, parent, x, y, z);
}

// Kraken X60 (or X44 when small) motor, axis along Y by default
function kraken(parent, x, y, z, small = false, axis = 'y') {
  const g = new THREE.Group();
  g.position.set(x, y, z);
  parent.add(g);
  const r = small ? 0.026 : 0.03, len = small ? 0.055 : 0.07;
  cylY(r, len, M.kraken, g, 0, 0, 0, 18);
  cylY(r * 1.02, 0.008, M.krakenRing, g, 0, len / 2 - 0.004, 0, 18);
  bx(0.014, len * 0.8, 0.02, M.kraken, g, r, 0, 0); // controller bump
  if (axis === 'x') g.rotation.z = Math.PI / 2;
  if (axis === 'z') g.rotation.x = Math.PI / 2;
  return g;
}

// MK5n-style swerve module: square top plate, two motors on top, 4in wheel underneath
function swerveModule(parent, x, z) {
  const g = new THREE.Group();
  g.position.set(x, 0, z);
  parent.add(g);
  rbx(0.118, 0.008, 0.118, 0.004, M.anodBlack, g, 0, 0.158, 0);
  cylY(0.056, 0.035, M.aluDark, g, 0, 0.135, 0, 28); // steering bearing housing
  kraken(g, -0.028, 0.2, 0.028);
  kraken(g, 0.028, 0.2, -0.028);
  bx(0.028, 0.012, 0.028, M.red, g, 0.035, 0.168, 0.035); // CANcoder
  const pivot = new THREE.Group();
  g.add(pivot);
  // fork
  for (const s of [-1, 1]) bx(0.05, 0.075, 0.008, M.aluDark, pivot, 0, 0.09, s * 0.03);
  const wheel = new THREE.Group();
  wheel.position.set(0, 0.0508, 0);
  pivot.add(wheel);
  // robot.js spins the wheel with wheel.rotation.y, so the axle is this group's local Y
  const spin = new THREE.Group();
  spin.rotation.x = Math.PI / 2;
  const tire = new THREE.Mesh(new THREE.CylinderGeometry(0.0508, 0.0508, 0.038, 24), M.tread);
  tire.castShadow = true;
  const hub = new THREE.Mesh(new THREE.CylinderGeometry(0.03, 0.03, 0.04, 16), M.hubGrey);
  spin.add(tire, hub);
  // tread nubs so the spin is visible
  for (let i = 0; i < 6; i++) {
    const n = new THREE.Mesh(new THREE.BoxGeometry(0.008, 0.04, 0.006), M.hubGrey);
    const a = (i * Math.PI) / 3;
    n.position.set(Math.cos(a) * 0.02, 0, Math.sin(a) * 0.02);
    spin.add(n);
  }
  wheel.add(spin);
  return { pivot, wheel: spin };
}

// Frame outline (frame perimeter) in robot XZ, counter-clockwise seen from above
function frameOutline(L, W, chamferBack = 0) {
  const c = chamferBack;
  const pts = [[L / 2, W / 2], [L / 2, -W / 2]];
  if (c > 0) pts.push([-L / 2 + c, -W / 2], [-L / 2, -W / 2 + c], [-L / 2, W / 2 - c], [-L / 2 + c, W / 2]);
  else pts.push([-L / 2, -W / 2], [-L / 2, W / 2]);
  return pts;
}

function bumperTexture(number, alliance) {
  const c = document.createElement('canvas');
  c.width = 512;
  c.height = 128;
  const g = c.getContext('2d');
  g.clearRect(0, 0, c.width, c.height);
  g.fillStyle = '#ffffff';
  g.font = '900 104px "Arial Black", Arial, sans-serif';
  g.textAlign = 'center';
  g.textBaseline = 'middle';
  g.fillText(String(number), 256, 68);
  const t = new THREE.CanvasTexture(c);
  t.colorSpace = THREE.SRGBColorSpace;
  return t;
}

// One continuous, round-edged bumper around the whole frame perimeter (chamfers included),
// with the team number on every straight side
function addBumpers(root, cfg, alliance, chamferBack = 0) {
  const L = cfg.frame.length, W = cfg.frame.width, T = BUMPER_T;
  const h = BUMP_Y1 - BUMP_Y0;
  const bevel = Math.min(0.02, h / 2 - 0.004);
  const outline = frameOutline(L, W, chamferBack);
  // shape space: (u, v) = (x, -z); after rotateX(-90deg) shape v -> world -z, depth -> +y
  const P = outline.map(([x, z]) => new THREE.Vector2(x, -z));
  let area = 0;
  for (let i = 0; i < P.length; i++) { const a = P[i], b = P[(i + 1) % P.length]; area += a.x * b.y - b.x * a.y; }
  if (area < 0) P.reverse();
  const n = P.length;
  const rOut = T - bevel; // the bevel adds the rest of the thickness
  const shape = new THREE.Shape();
  for (let i = 0; i < n; i++) {
    const p = P[i], a = P[(i - 1 + n) % n], b = P[(i + 1) % n];
    const d0 = new THREE.Vector2().subVectors(p, a).normalize(), d1 = new THREE.Vector2().subVectors(b, p).normalize();
    let a0 = Math.atan2(-d0.x, d0.y), a1 = Math.atan2(-d1.x, d1.y); // outward normals (CCW polygon)
    if (a1 < a0) a1 += Math.PI * 2;
    const sx = p.x + rOut * Math.cos(a0), sy = p.y + rOut * Math.sin(a0);
    if (i === 0) shape.moveTo(sx, sy); else shape.lineTo(sx, sy);
    shape.absarc(p.x, p.y, rOut, a0, a1, false);
  }
  shape.closePath();
  const hole = new THREE.Path();
  const inset = 0.004;
  const HP = P.map((p) => p.clone().multiplyScalar(1 - inset / Math.max(L, W)));
  hole.moveTo(HP[0].x, HP[0].y);
  for (let i = HP.length - 1; i >= 1; i--) hole.lineTo(HP[i].x, HP[i].y);
  hole.closePath();
  shape.holes.push(hole);
  const geo = new THREE.ExtrudeGeometry(shape, {
    depth: h - 2 * bevel, bevelEnabled: true, bevelThickness: bevel, bevelSize: bevel, bevelSegments: 4, curveSegments: 10,
  });
  geo.rotateX(-Math.PI / 2);
  geo.translate(0, BUMP_Y0 + bevel, 0);
  geo.computeVertexNormals();
  const fabric = std(ALLIANCE_COLOR_CSS[alliance], 0.88, 0.0);
  const ring = mesh(geo, fabric, root, 0, 0, 0);
  ring.name = 'bumper';
  // team numbers on each straight side
  const tex = bumperTexture(cfg.team, alliance);
  const numMat = new THREE.MeshBasicMaterial({ map: tex, transparent: true, depthWrite: false, polygonOffset: true, polygonOffsetFactor: -2 });
  const yc = (BUMP_Y0 + BUMP_Y1) / 2;
  const sides = [
    { x: L / 2 + T + 0.002, z: 0, ry: Math.PI / 2, len: W },
    { x: -L / 2 - T - 0.002, z: 0, ry: -Math.PI / 2, len: W - 2 * chamferBack },
    { x: -chamferBack / 2, z: W / 2 + T + 0.002, ry: 0, len: L - chamferBack },
    { x: -chamferBack / 2, z: -W / 2 - T - 0.002, ry: Math.PI, len: L - chamferBack },
  ];
  for (const sd of sides) {
    const w = Math.min(0.42, sd.len * 0.8), hh = w / 4;
    const pl = new THREE.Mesh(new THREE.PlaneGeometry(w, Math.min(hh, h * 0.85)), numMat);
    pl.position.set(sd.x, yc, sd.z);
    pl.rotation.y = sd.ry;
    root.add(pl);
  }
  return ring;
}

function addDrivebase(root, cfg, chamferBack = 0) {
  const L = cfg.frame.length, W = cfg.frame.width;
  const frameMat = std(cfg.colors.frame, 0.4, 0.65);
  const y = 0.085;
  const outline = frameOutline(L, W, chamferBack);
  for (let i = 0; i < outline.length; i++) {
    const [x0, z0] = outline[i], [x1, z1] = outline[(i + 1) % outline.length];
    // pull each tube 0.5in inside the perimeter
    const cx = (x0 + x1) / 2, cz = (z0 + z1) / 2, k = 0.0127 / Math.max(1e-6, Math.hypot(cx, cz));
    tube(root, x0 - x0 * k * 0.9, z0 - z0 * k * 0.9, x1 - x1 * k * 0.9, z1 - z1 * k * 0.9, y, frameMat);
  }
  // cross tubes + bellypan
  tube(root, -L / 2 + 0.16, -W / 2 + 0.03, -L / 2 + 0.16, W / 2 - 0.03, y, frameMat);
  tube(root, L / 2 - 0.16, -W / 2 + 0.03, L / 2 - 0.16, W / 2 - 0.03, y, frameMat);
  rbx(L - 0.05, 0.004, W - 0.05, 0.002, M.aluDark, root, 0, 0.057, 0);
  const modules = [];
  const inset = 0.075;
  for (const sx of [1, -1]) for (const sz of [1, -1]) modules.push(swerveModule(root, sx * (L / 2 - inset), sz * (W / 2 - inset)));
  return modules;
}

// battery, roboRIO, PDH, radio on the bellypan
function addElectronics(parent, x, z, rot = 0) {
  const g = new THREE.Group();
  g.position.set(x, 0.06, z);
  g.rotation.y = rot;
  parent.add(g);
  rbx(0.18, 0.17, 0.077, 0.006, M.battery, g, 0, 0.085, 0);
  cylY(0.006, 0.02, M.red, g, 0.05, 0.18, 0.02, 8);
  cylY(0.006, 0.02, M.black, g, -0.05, 0.18, 0.02, 8);
  rbx(0.14, 0.03, 0.1, 0.005, M.rioGrey, g, 0, 0.015, 0.1); // roboRIO
  rbx(0.14, 0.04, 0.09, 0.005, M.anodBlack, g, 0, 0.02, -0.1); // PDH
  bx(0.02, 0.006, 0.06, M.red, g, 0, 0.042, -0.1);
  rbx(0.1, 0.03, 0.06, 0.008, M.radioWhite, g, 0.12, 0.015, 0.1); // radio
  return g;
}

function limelight(parent, x, y, z, ry = 0) {
  const g = new THREE.Group();
  g.position.set(x, y, z);
  g.rotation.y = ry;
  parent.add(g);
  rbx(0.03, 0.06, 0.1, 0.006, M.anodBlack, g, 0, 0, 0);
  const lens = mesh(new THREE.CircleGeometry(0.014, 16), M.lens, g, 0.0155, 0, 0);
  lens.rotation.y = Math.PI / 2;
  for (const s of [-1, 1]) { const l = mesh(new THREE.CircleGeometry(0.006, 10), M.led, g, 0.0155, 0, s * 0.03); l.rotation.y = Math.PI / 2; }
  return g;
}

// polycarbonate panel with aluminum angle along its top edge; lies in the XY plane
function polyWall(parent, w, h, x, y, z, ry = 0, mat = M.poly, edgeMat = M.alu) {
  const g = new THREE.Group();
  g.position.set(x, y, z);
  g.rotation.y = ry;
  parent.add(g);
  const p = mesh(new THREE.PlaneGeometry(w, h), mat, g, 0, 0, 0);
  p.castShadow = false;
  rbx(w, 0.016, 0.016, 0.002, edgeMat, g, 0, h / 2, 0);
  rbx(0.016, h, 0.012, 0.002, edgeMat, g, -w / 2, 0, 0);
  rbx(0.016, h, 0.012, 0.002, edgeMat, g, w / 2, 0, 0);
  return g;
}

// Over-bumper intake arm from its pivot to the roller: the roller reaches the intake's rated
// reach past the bumper (where the physics grabs FUEL) and sits at FUEL height when deployed.
function intakeArm(cfg, px, py, rollerR) {
  const tx = cfg.frame.length / 2 + BUMPER_T + cfg.intake.reach - rollerR;
  const ty = Math.max(rollerR + 0.012, 0.05);
  return { armLen: Math.hypot(tx - px, py - ty), deploy: -Math.atan2(py - ty, tx - px) };
}

// row of shooter wheels on a shaft along Z
function wheelStack(parent, len, r, n, wheelMat, x, y, z, width = 0.02) {
  const g = new THREE.Group();
  g.position.set(x, y, z);
  parent.add(g);
  cylZ(0.006, len, M.alu, g, 0, 0, 0, 8);
  for (let i = 0; i < n; i++) {
    const zz = -len / 2 + (i + 0.5) * (len / n);
    cylZ(r, width, wheelMat, g, 0, 0, zz, 20);
  }
  return g;
}

function storedGrid(xs, zs, ys, ext = false) {
  const out = [];
  for (const y of ys) for (const x of xs) for (const z of zs) { const v = new THREE.Vector3(x, y, z); v.ext = ext; out.push(v); }
  return out;
}

// ============================================================ 2910 Re•Blitz
// Dumper: huge one-piece hopper over most of the robot, a 4-wide drum shooter with an
// adjustable hood across the back (fixed to the chassis), slap-down intake at the front.
// The hopper front wall slides forward with the intake to open up more room.
function build2910(cfg, alliance) {
  const root = new THREE.Group();
  const L = cfg.frame.length, W = cfg.frame.width, H = cfg.height;
  addBumpers(root, cfg, alliance);
  const modules = addDrivebase(root, cfg);
  addElectronics(root, 0.02, 0, 0);
  const plate = std(0x8f949c, 0.35, 0.85);
  const purple = std(cfg.colors.accent, 0.45, 0.35);
  const green = std(cfg.colors.trim, 0.5, 0.2);
  const extLen = cfg.storage.extLen;

  // ---- shooter tower at the back: pocketed billet side plates, drum + hood
  const tower = new THREE.Group();
  tower.position.set(-L / 2 + 0.1, 0, 0);
  root.add(tower);
  const holes = [[-0.04, -0.12, 0.025], [0.04, -0.12, 0.025], [-0.04, 0.0, 0.02], [0.05, 0.02, 0.018], [0, 0.12, 0.018]];
  for (const s of [-1, 1]) {
    pocketPlate(tower, 0.19, H - 0.14, 0.0127, plate, holes, 0, 0.14 + (H - 0.14) / 2, s * (W / 2 - 0.03));
    kraken(tower, -0.02, 0.44, s * (W / 2 + 0.01), false, 'z');
  }
  const drum = wheelStack(tower, W - 0.09, 0.0508, 4, M.green, -0.02, 0.44, 0, (W - 0.12) / 4);
  const hood = new THREE.Group();
  hood.position.set(-0.02, 0.44, 0);
  tower.add(hood);
  const hoodShell = new THREE.Mesh(new THREE.CylinderGeometry(0.125, 0.125, W - 0.1, 24, 1, true, Math.PI * 0.05, Math.PI * 0.62), std(0x2a2d33, 0.45, 0.55, { side: THREE.DoubleSide }));
  hoodShell.rotation.x = Math.PI / 2;
  hood.add(hoodShell);
  for (let i = 0; i < 5; i++) {
    const rib = new THREE.Mesh(new THREE.TorusGeometry(0.125, 0.004, 4, 20, Math.PI * 0.62), purple);
    rib.rotation.z = Math.PI * 0.05;
    rib.position.z = -W / 2 + 0.06 + i * ((W - 0.12) / 4);
    hood.add(rib);
  }
  const hoodRollers = [];
  for (let i = 0; i < 3; i++) {
    const a = 0.45 + i * 0.5;
    hoodRollers.push(cylZ(0.018, W - 0.12, M.alu, hood, -Math.cos(a) * 0.1, Math.sin(a) * 0.1, 0, 12));
  }
  // compliant indexer wheels feeding the drum + inclined powered floor
  const indexer = wheelStack(tower, W - 0.1, 0.03, 8, M.compliant, 0.07, 0.3, 0, 0.022);
  const feeder = new THREE.Group();
  feeder.position.set(0.06, 0.1, 0);
  feeder.rotation.z = 0.26;
  root.add(feeder);
  const feedRollers = [];
  for (let i = 0; i < 9; i++) feedRollers.push(cylZ(0.016, W - 0.1, M.alu, feeder, 0.32 - i * 0.05, 0, 0, 10));
  mesh(new THREE.PlaneGeometry(0.46, W - 0.1), M.polyBelt, feeder, 0.1, -0.017, 0).rotation.x = -Math.PI / 2;

  // ---- one-piece hopper (tinted polycarbonate on an aluminum frame)
  const hopper = new THREE.Group();
  root.add(hopper);
  const hY = 0.17, hH = H - hY - 0.01;
  const hBack = -L / 2 + 0.2, hFront = L / 2 - 0.02;
  const hMid = (hBack + hFront) / 2, hLen = hFront - hBack;
  for (const s of [-1, 1]) polyWall(hopper, hLen, hH, hMid, hY + hH / 2, s * (W / 2 - 0.012), 0, M.polyTint, green);
  // sliding front section: telescoping side panels + front wall ride out with the intake
  const front = new THREE.Group();
  root.add(front);
  for (const s of [-1, 1]) {
    polyWall(front, extLen + 0.04, hH * 0.85, hFront - extLen / 2 + 0.02, hY + hH * 0.85 / 2, s * (W / 2 - 0.028), 0, M.polyTint, M.alu);
    tube(root, hFront - 0.28, s * (W / 2 - 0.05), hFront - 0.02, s * (W / 2 - 0.05), hY + hH - 0.02, M.aluDark, 0.012, 0.012); // slide rail
  }
  polyWall(front, W - 0.06, hH * 0.85, hFront + 0.02, hY + hH * 0.85 / 2, 0, Math.PI / 2, M.polyTint, M.alu);
  limelight(root, hFront - 0.3, H + 0.01, W / 2 - 0.03);
  limelight(root, -L / 2 + 0.02, H - 0.03, -W / 2 + 0.12, Math.PI);

  // ---- slap-down intake: pivots at the front of the frame, arms fold up over the hopper
  const intake = new THREE.Group();
  intake.position.set(L / 2 - 0.035, 0.26, 0);
  root.add(intake);
  const { armLen, deploy } = intakeArm(cfg, L / 2 - 0.035, 0.26, 0.03);
  for (const s of [-1, 1]) {
    pocketPlate(intake, armLen + 0.04, 0.055, 0.0095, plate, [[-0.08, 0, 0.012], [0.04, 0, 0.012]], armLen / 2, 0, s * (cfg.intake.width / 2 + 0.012));
    kraken(intake, 0.02, 0, s * (cfg.intake.width / 2 + 0.04), true, 'z');
  }
  const intakeRollers = [
    wheelStack(intake, cfg.intake.width, 0.03, 8, M.orangeWheel, armLen, 0, 0, 0.03),
    wheelStack(intake, cfg.intake.width, 0.022, 6, M.compliant, armLen - 0.09, -0.035, 0, 0.025),
  ];
  rbx(0.05, 0.05, cfg.intake.width + 0.04, 0.004, purple, intake, armLen - 0.02, 0.05, 0);

  // ---- FUEL inside: main hopper, then the extension, then the tower
  const stored = storedGrid([-0.13, 0.02, 0.17, 0.3], [-0.225, -0.075, 0.075, 0.225], [0.2, 0.34, 0.47]);
  stored.push(...storedGrid([0.28, 0.43], [-0.225, -0.075, 0.075, 0.225], [0.22, 0.36], true).map((v) => { v.x -= extLen; return v; }));
  for (const z of [-0.19, -0.063, 0.063, 0.19]) stored.push(new THREE.Vector3(-0.2, 0.3, z));

  const anim = (st, dt) => {
    // intake: 0 = stowed (arms up, leaning back over the hopper), 1 = deployed over the bumper
    intake.rotation.z = lerp(2.0, deploy, st.intakeDeploy);
    for (const r of intakeRollers) r.rotation.z += st.intakeSpeed * dt * 40;
    front.position.x = st.hopperDeploy * extLen; // slides out by extLen
    drum.rotation.z -= st.flywheel * dt * 6;
    for (const r of hoodRollers) r.rotation.y += st.flywheel * dt * 8;
    indexer.rotation.z -= st.feeding * dt * 25;
    for (const r of feedRollers) r.rotation.y -= st.feeding * dt * 30;
    hood.rotation.z = (st.hoodDeg - 58) * Math.PI / 180 * 0.6;
  };
  return { root, anim, stored, modules, extLen };
}

// ============================================================ 4414 RIPCURRENT
// Dye Rotor: a wide, chamfer-backed robot whose polycarbonate hopper sits on the frame; the
// front section telescopes out 12in with the latched over-bumper intake. The Dye Rotor, a
// pocketed rotor like a paintball loader's, single-streams FUEL into a turret with a hooded
// 3in flywheel.
function build4414(cfg, alliance) {
  const root = new THREE.Group();
  const L = cfg.frame.length, W = cfg.frame.width, H = cfg.height;
  const chamfer = 0.12;
  const extLen = cfg.storage.extLen;
  addBumpers(root, cfg, alliance, chamfer);
  const modules = addDrivebase(root, cfg, chamfer);
  const teal = std(cfg.colors.accent, 0.4, 0.45);
  const orange = std(cfg.colors.trim, 0.45, 0.3);
  const plate = std(0x9aa0a8, 0.35, 0.85);
  addElectronics(root, -L / 2 + 0.2, W / 2 - 0.14, Math.PI / 2);

  // ---- fixed hopper walls on the frame (angled at the chamfers)
  const hY = 0.17, hH = H - hY - 0.02;
  const wallZ = W / 2 - 0.012;
  for (const s of [-1, 1]) polyWall(root, L - chamfer - 0.02, hH, chamfer / 2 - 0.01, hY + hH / 2, s * wallZ, 0, M.poly, teal);
  polyWall(root, W - 2 * chamfer, hH, -L / 2 + 0.012, hY + hH / 2, 0, Math.PI / 2, M.poly, teal);
  for (const s of [-1, 1]) {
    const cw = Math.SQRT2 * chamfer;
    polyWall(root, cw, hH, -L / 2 + chamfer / 2, hY + hH / 2, s * (W / 2 - chamfer / 2), -s * Math.PI / 4, M.poly, teal);
  }
  // ---- telescoping front extension: nested walls on rails
  const ext = new THREE.Group();
  root.add(ext);
  for (const s of [-1, 1]) {
    polyWall(ext, extLen + 0.06, hH * 0.9, L / 2 - extLen / 2 - 0.03, hY + hH * 0.45, s * (wallZ - 0.018), 0, M.polyTint, M.alu);
    tube(root, L / 2 - 0.34, s * (wallZ - 0.035), L / 2 - 0.02, s * (wallZ - 0.035), hY + hH - 0.03, M.aluDark, 0.014, 0.014);
  }
  polyWall(ext, W - 0.07, hH * 0.55, L / 2, hY + hH * 0.28, 0, Math.PI / 2, M.polyTint, M.alu);
  rbx(0.02, 0.02, W - 0.05, 0.003, orange, ext, L / 2, hY + hH * 0.56, 0);

  // ---- net over the hopper (open around the turret), and over the extension
  const topY = hY + hH;
  const netShape = new THREE.Shape();
  // shape space (u, v) = (x, -z); rotateX(-90deg) maps v -> -z
  [[-L / 2 + chamfer, -W / 2], [L / 2, -W / 2], [L / 2, W / 2], [-L / 2 + chamfer, W / 2], [-L / 2, W / 2 - chamfer], [-L / 2, -W / 2 + chamfer]]
    .forEach(([x, z], i) => (i ? netShape.lineTo(x, -z) : netShape.moveTo(x, -z)));
  const hole = new THREE.Path();
  hole.absarc(cfg.shooter.turretPos.x, -cfg.shooter.turretPos.z, 0.165, 0, Math.PI * 2, true);
  netShape.holes.push(hole);
  const netGeo = new THREE.ShapeGeometry(netShape, 24);
  netGeo.rotateX(-Math.PI / 2);
  mesh(netGeo, M.net, root, 0, topY, 0).castShadow = false;
  const extShape = new THREE.Shape();
  extShape.moveTo(L / 2 - extLen - 0.02, -(W / 2 - 0.035)); extShape.lineTo(L / 2, -(W / 2 - 0.035));
  extShape.lineTo(L / 2, W / 2 - 0.035); extShape.lineTo(L / 2 - extLen - 0.02, W / 2 - 0.035);
  const extNetGeo = new THREE.ShapeGeometry(extShape);
  extNetGeo.rotateX(-Math.PI / 2);
  mesh(extNetGeo, M.net, ext, 0, topY - hH * 0.1, 0).castShadow = false;
  tube(ext, L / 2, -(W / 2 - 0.035), L / 2, W / 2 - 0.035, topY - hH * 0.1, M.aluDark, 0.008, 0.008);

  // ---- Dye Rotor (after the paintball loader): a low, wide rotor whose curved fins form
  // pockets. FUEL drops straight into the pockets, rides around to the Dolphin Fin at the back
  // and rolls up a ramp of passive rollers to the feeder wheels under the turret. Printed
  // "stadium" terraces fill the corners and funnel FUEL down into the rotor.
  const bay = cfg.bay, rs = bay.rotor;
  const white = std(0xe9ebef, 0.5, 0.1);
  const rotor = new THREE.Group();
  rotor.position.set(rs.x, rs.y, rs.z);
  root.add(rotor);
  mesh(new THREE.CylinderGeometry(rs.r, rs.r, 0.012, 56), white, rotor, 0, -0.006, 0);
  mesh(new THREE.SphereGeometry(0.06, 20, 8, 0, Math.PI * 2, 0, Math.PI / 2), white, rotor, 0, 0, 0);
  mesh(new THREE.TorusGeometry(rs.r - 0.004, 0.006, 6, 56), teal, rotor, 0, 0.004, 0).rotation.x = Math.PI / 2;
  const finH = 0.075;
  for (let i = 0; i < rs.pockets; i++) {
    const a0 = (i * 2 * Math.PI) / rs.pockets;
    const th = (r) => a0 + 0.7 * (r - 0.06) / (rs.r - 0.06); // curved, trailing fins
    const rr = [0.06, 0.13, 0.2, rs.r - 0.008];
    for (let k = 0; k < rr.length - 1; k++) {
      const p0 = [Math.cos(th(rr[k])) * rr[k], Math.sin(th(rr[k])) * rr[k]];
      const p1 = [Math.cos(th(rr[k + 1])) * rr[k + 1], Math.sin(th(rr[k + 1])) * rr[k + 1]];
      const len = Math.hypot(p1[0] - p0[0], p1[1] - p0[1]);
      const fin = mesh(new THREE.BoxGeometry(len + 0.006, finH, 0.008), teal, rotor, (p0[0] + p1[0]) / 2, finH / 2, (p0[1] + p1[1]) / 2);
      fin.rotation.y = -Math.atan2(p1[1] - p0[1], p1[0] - p0[0]);
    }
  }
  kraken(root, rs.x, 0.07, rs.z, true); // rotor drive under the disc
  // stadium terraces: the funnel floor around the rotor, in printed steps
  {
    const cell = 0.035, step = 0.021, pos = [];
    const box = (x0, x1, y0, y1, z0, z1) => {
      const q = [[x0, y1, z0], [x1, y1, z0], [x1, y1, z1], [x0, y1, z1], [x0, y0, z0], [x1, y0, z0], [x1, y0, z1], [x0, y0, z1]];
      for (const f of [[0, 3, 2, 1], [0, 1, 5, 4], [1, 2, 6, 5], [2, 3, 7, 6], [3, 0, 4, 7]]) {
        const [a, b, c, d] = f.map((i) => q[i]);
        pos.push(...a, ...b, ...c, ...a, ...c, ...d);
      }
    };
    for (let x = bay.x0; x < bay.x1 - 1e-6; x += cell) {
      for (let z = -bay.hw; z < bay.hw - 1e-6; z += cell) {
        const cx = x + cell / 2, cz = z + cell / 2;
        const d = Math.hypot(cx - rs.x, cz - rs.z) - rs.r;
        if (d < 0.01) continue;
        if ((cx - bay.x0) - Math.abs(cz) + bay.hw - bay.chamfer < 0) continue;
        const h = Math.min(bay.funnel.cap, d * bay.funnel.slope);
        const top = rs.y + Math.max(step, Math.ceil(h / step) * step);
        box(x, Math.min(x + cell, bay.x1), rs.y - 0.01, top, z, Math.min(z + cell, bay.hw));
      }
    }
    const g = new THREE.BufferGeometry();
    g.setAttribute('position', new THREE.Float32BufferAttribute(pos, 3));
    g.computeVertexNormals();
    mesh(g, std(0xdfe2e6, 0.7, 0.05), root, 0, 0, 0);
  }
  // Dolphin Fin at the back of the rotor lifts each FUEL out of its pocket onto the ramp
  const finShape = new THREE.Shape();
  finShape.moveTo(0, 0); finShape.quadraticCurveTo(0.05, 0.02, 0.07, 0.11); finShape.quadraticCurveTo(0.03, 0.07, -0.03, 0.06); finShape.lineTo(0, 0);
  const dolphin = mesh(new THREE.ExtrudeGeometry(finShape, { depth: 0.012, bevelEnabled: false }), teal, root, rs.x - rs.r + 0.02, rs.y, -0.006);
  dolphin.rotation.y = Math.PI;
  dolphin.position.z = 0.006;
  // ramp of passive rollers up to the feeder wheels, under a printed cover
  const ramp = new THREE.Group();
  ramp.position.set(-0.265, 0.19, 0);
  ramp.rotation.z = 1.18; // rises toward the turret
  root.add(ramp);
  const rampLen = 0.24;
  const rampRollers = [];
  for (let i = 0; i < 7; i++) rampRollers.push(cylZ(0.012, 0.14, M.alu, ramp, 0.015 + i * (rampLen / 7), -0.01, 0, 10));
  for (const sgn of [-1, 1]) bx(rampLen, 0.05, 0.006, plate, ramp, rampLen / 2, 0.01, sgn * 0.078);
  const cover = mesh(new THREE.CylinderGeometry(0.09, 0.09, rampLen, 20, 1, true, -Math.PI / 2, Math.PI), std(0xf2f2ee, 0.6, 0.05, { side: THREE.DoubleSide }), ramp, rampLen / 2, 0.0, 0);
  cover.rotation.z = -Math.PI / 2;
  const kicker = wheelStack(root, 0.14, 0.028, 2, M.compliant, -0.19, 0.4, 0, 0.05);

  // ---- turret with the hooded 3in flywheel
  const turret = new THREE.Group();
  turret.position.set(cfg.shooter.turretPos.x, 0.42, cfg.shooter.turretPos.z);
  root.add(turret);
  mesh(new THREE.CylinderGeometry(0.135, 0.135, 0.025, 40), std(0x2b2f36, 0.4, 0.6), turret, 0, 0, 0);
  mesh(new THREE.TorusGeometry(0.135, 0.006, 6, 48), orange, turret, 0, 0.004, 0).rotation.x = Math.PI / 2; // gear ring
  const body = new THREE.Group();
  turret.add(body);
  for (const s of [-1, 1]) pocketPlate(body, 0.24, 0.13, 0.008, plate, [[-0.05, 0.0, 0.022], [0.06, 0.0, 0.018]], 0, 0.08, s * 0.068);
  const fly = wheelStack(body, 0.12, 0.038, 3, M.blueWheel, -0.02, 0.07, 0, 0.03);
  const hood = new THREE.Group();
  hood.position.set(-0.02, 0.07, 0);
  body.add(hood);
  const hoodShell = new THREE.Mesh(new THREE.CylinderGeometry(0.095, 0.095, 0.13, 18, 1, true, 0, Math.PI * 0.55), std(0x1e2126, 0.45, 0.45, { side: THREE.DoubleSide }));
  hoodShell.rotation.x = Math.PI / 2;
  hood.add(hoodShell);
  for (let i = 0; i < 4; i++) kraken(body, -0.1, 0.07, -0.045 + i * 0.03, true, 'z').scale.setScalar(0.7);
  limelight(body, 0.1, 0.14, 0, 0);

  // ---- over-bumper intake (deploys at match start and latches down)
  const intake = new THREE.Group();
  intake.position.set(L / 2 - 0.02, 0.25, 0);
  root.add(intake);
  const { armLen, deploy } = intakeArm(cfg, L / 2 - 0.02, 0.25, 0.038);
  for (const s of [-1, 1]) pocketPlate(intake, armLen + 0.04, 0.05, 0.008, M.alu, [[-0.1, 0, 0.012], [0.05, 0, 0.012]], armLen / 2, 0, s * (cfg.intake.width / 2 + 0.02));
  const intakeRollers = [
    wheelStack(intake, cfg.intake.width, 0.038, 1, std(0xcfe8ff, 0.3, 0.1, { transparent: true, opacity: 0.8 }), armLen, 0, 0, cfg.intake.width - 0.02),
    wheelStack(intake, cfg.intake.width, 0.024, 8, M.compliant, armLen - 0.1, -0.06, 0, 0.025),
  ];
  rbx(0.07, 0.03, cfg.intake.width + 0.05, 0.004, M.alu, intake, armLen - 0.02, 0.05, 0);
  kraken(intake, 0.03, 0, cfg.intake.width / 2 + 0.05, true, 'z');

  // ---- FUEL: on the rotor, then in the extension
  const stored = [];
  for (const y of [0.18, 0.32, 0.46]) {
    for (let ix = 0; ix < 5; ix++) {
      for (let iz = 0; iz < 6; iz++) {
        const x = -0.27 + ix * 0.15, z = -0.375 + iz * 0.15;
        if (Math.hypot(x - cfg.shooter.turretPos.x, z) < 0.17 && y > 0.3) continue;
        if (x < -L / 2 + 0.05 && Math.abs(z) > W / 2 - chamfer) continue;
        stored.push(new THREE.Vector3(x, y, z));
      }
    }
  }
  stored.sort((a, b) => a.y - b.y || Math.hypot(a.x, a.z) - Math.hypot(b.x, b.z));
  stored.push(...storedGrid([L / 2 - extLen + 0.08, L / 2 - extLen + 0.22], [-0.375, -0.225, -0.075, 0.075, 0.225, 0.375], [0.2, 0.34], true));

  const anim = (st, dt) => {
    intake.rotation.z = lerp(2.05, deploy, st.intakeDeploy);
    for (const r of intakeRollers) r.rotation.z += st.intakeSpeed * dt * 35;
    ext.position.x = st.hopperDeploy * extLen;
    // ~2.25 rev/s when feeding; otherwise turns slowly backward to agitate the load
    rotor.rotation.y += (st.feeding > 0 ? cfg.bay.rotor.spin : cfg.bay.rotor.idle) * dt;
    kicker.rotation.z -= st.feeding * dt * 30;
    for (const r of rampRollers) r.rotation.y -= st.feeding * dt * 20;
    turret.rotation.y = st.turretYaw;
    fly.rotation.z -= st.flywheel * dt * 10;
    hood.rotation.z = (st.hoodDeg - 62) * Math.PI / 180 * 0.8;
  };
  return { root, anim, stored, modules, extLen };
}

// ============================================================ 8793 Hopperless
// Low, open robot: wide multi-roller intake -> funnel conveyor of compliant wheels -> big
// bearing-ring turret carrying a hooded flywheel shooter. Holds only the FUEL in its path.
function build8793(cfg, alliance) {
  const root = new THREE.Group();
  const L = cfg.frame.length, W = cfg.frame.width;
  addBumpers(root, cfg, alliance);
  const modules = addDrivebase(root, cfg);
  const orange = std(cfg.colors.accent, 0.45, 0.35);
  const plate = std(0x2a2c31, 0.5, 0.6);
  addElectronics(root, -0.12, -W / 2 + 0.2, Math.PI / 2);

  // electronics deck
  rbx(L - 0.12, 0.008, W - 0.12, 0.003, std(0x1b1c20, 0.6, 0.4), root, -0.02, 0.172, 0);
  for (const s of [-1, 1]) rbx(L * 0.45, 0.01, 0.16, 0.003, orange, root, -0.14, 0.18, s * (W / 2 - 0.14));

  // conveyor: funnel of compliant wheels from the intake back to the turret
  const conv = new THREE.Group();
  conv.position.set(0.1, 0.19, 0);
  root.add(conv);
  const convWheels = [];
  for (let row = 0; row < 3; row++) {
    const w = W - 0.18 - row * 0.1;
    const n = 6 - row;
    const stack = wheelStack(conv, w, 0.035, n, M.compliant, 0.12 - row * 0.1, 0.02 + row * 0.03, 0, 0.03);
    convWheels.push(stack);
  }
  for (const s of [-1, 1]) pocketPlate(conv, 0.36, 0.12, 0.006, orange, [[-0.1, 0, 0.025], [0.02, 0, 0.025], [0.13, 0, 0.02]], 0.02, 0.05, s * (W / 2 - 0.08));
  kraken(conv, 0.02, 0.05, W / 2 - 0.05, true, 'z');

  // turret: big bearing ring at the back with the hooded shooter
  const turret = new THREE.Group();
  turret.position.set(cfg.shooter.turretPos.x, 0.25, cfg.shooter.turretPos.z);
  root.add(turret);
  mesh(new THREE.CylinderGeometry(0.22, 0.22, 0.02, 48), std(0x2a2d33, 0.45, 0.5), turret, 0, 0, 0);
  mesh(new THREE.TorusGeometry(0.215, 0.009, 8, 60), M.alu, turret, 0, 0.012, 0).rotation.x = Math.PI / 2;
  const body = new THREE.Group();
  turret.add(body);
  for (const s of [-1, 1]) pocketPlate(body, 0.3, 0.2, 0.008, orange, [[-0.09, 0.02, 0.03], [0.02, -0.03, 0.03], [0.1, 0.04, 0.025]], 0, 0.11, s * 0.1);
  const fly = wheelStack(body, 0.16, 0.05, 2, M.orangeWheel, 0.02, 0.1, 0, 0.05);
  const hood = new THREE.Group();
  hood.position.set(0.02, 0.1, 0);
  body.add(hood);
  const hoodShell = new THREE.Mesh(new THREE.CylinderGeometry(0.12, 0.12, 0.19, 20, 1, true, 0, Math.PI * 0.6), std(0x1b1d22, 0.45, 0.5, { side: THREE.DoubleSide }));
  hoodShell.rotation.x = Math.PI / 2;
  hood.add(hoodShell);
  for (let i = 0; i < 5; i++) cylZ(0.022, 0.03, M.compliant, hood, -Math.cos(0.5 + i * 0.3) * 0.12, Math.sin(0.5 + i * 0.3) * 0.12, 0, 10);
  for (const s of [-1, 1]) kraken(body, 0.02, 0.1, s * 0.13, false, 'z');
  for (let i = 0; i < 7; i++) mesh(new THREE.SphereGeometry(0.007, 8, 6), M.led, body, 0.09 + i * 0.012, 0.16, 0.108);
  limelight(body, -0.15, 0.2, 0, Math.PI);

  // Intake V3: multi-roller over-bumper intake
  const intake = new THREE.Group();
  intake.position.set(L / 2 - 0.03, 0.28, 0);
  root.add(intake);
  const { armLen, deploy } = intakeArm(cfg, L / 2 - 0.03, 0.28, 0.024);
  for (const s of [-1, 1]) pocketPlate(intake, armLen + 0.04, 0.07, 0.008, plate, [[-0.1, 0, 0.018], [0.02, 0, 0.018], [0.12, 0, 0.015]], armLen / 2, -0.01, s * (cfg.intake.width / 2 + 0.02));
  const intakeRollers = [];
  for (let i = 0; i < 3; i++) intakeRollers.push(wheelStack(intake, cfg.intake.width, 0.024, 1, M.black, armLen - i * 0.075, -0.02 + i * 0.03, 0, cfg.intake.width - 0.02));
  wheelStack(intake, cfg.intake.width - 0.06, 0.035, 7, M.compliant, armLen - 0.19, 0.03, 0, 0.025);
  kraken(intake, 0.03, 0, cfg.intake.width / 2 + 0.05, true, 'z');

  // FUEL in the ball path (intake -> conveyor -> turret)
  const stored = [];
  const path = [[0.33, 0.13], [0.26, 0.16], [0.19, 0.2], [0.12, 0.24], [0.05, 0.28], [-0.02, 0.32]];
  for (let i = path.length - 1; i >= 0; i--) for (const z of [-0.08, 0.08]) stored.push(new THREE.Vector3(path[i][0] - 0.1, path[i][1] + 0.06, z));

  const anim = (st, dt) => {
    intake.rotation.z = lerp(2.0, deploy, st.intakeDeploy);
    for (const r of intakeRollers) r.rotation.z += st.intakeSpeed * dt * 40;
    for (const w of convWheels) w.rotation.z += (st.intakeSpeed + st.feeding) * dt * 25;
    turret.rotation.y = st.turretYaw;
    fly.rotation.z -= st.flywheel * dt * 8;
    hood.rotation.z = (st.hoodDeg - 62) * Math.PI / 180 * 0.8;
  };
  return { root, anim, stored, modules, extLen: 0 };
}

export function buildRobotModel(cfg, alliance) {
  let m;
  if (cfg.key === '2910') m = build2910(cfg, alliance);
  else if (cfg.key === '4414') m = build4414(cfg, alliance);
  else m = build8793(cfg, alliance);
  // stored FUEL visual (instanced)
  const r = FUEL.radius;
  const geo = new THREE.SphereGeometry(r, 14, 10);
  const mat = new THREE.MeshStandardMaterial({ color: 0xf4d23a, roughness: 0.75 });
  const inst = new THREE.InstancedMesh(geo, mat, Math.max(m.stored.length, cfg.storage.capacity + 8));
  inst.instanceMatrix.setUsage(THREE.DynamicDrawUsage);
  inst.frustumCulled = false;
  inst.castShadow = true;
  inst.count = 0;
  m.root.add(inst);
  m.storedMesh = inst;
  m.root.traverse((o) => {
    if (o.isMesh && o.material !== M.poly && o.material !== M.polyTint) o.castShadow = true;
  });
  return m;
}

export function addClimberVisual(model, cfg) {
  const g = new THREE.Group();
  const L = cfg.frame.length;
  g.position.set(-L / 2 + 0.06, 0, 0);
  const mat = std(0x9aa0a8, 0.35, 0.85);
  for (const s of [-1, 1]) bx(0.04, 0.5, 0.04, mat, g, 0, 0.3, s * 0.2);
  const hook = new THREE.Group();
  hook.position.set(0, 0.55, 0);
  g.add(hook);
  bx(0.04, 0.04, 0.5, mat, hook, 0, 0, 0);
  for (const s of [-1, 1]) bx(0.08, 0.02, 0.03, mat, hook, 0.04, 0.02, s * 0.2);
  model.root.add(g);
  model.climber = { group: g, hook };
}
