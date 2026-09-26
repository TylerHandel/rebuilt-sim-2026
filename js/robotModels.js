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
// truss plate (the teal frame): triangles cut out of a strip
function trussTexture() {
  const c = document.createElement('canvas');
  c.width = 256; c.height = 64;
  const g = c.getContext('2d');
  g.fillStyle = '#ffffff';
  g.fillRect(0, 0, 256, 64);
  g.globalCompositeOperation = 'destination-out';
  for (let i = 0; i < 8; i++) {
    const x = i * 32;
    g.beginPath(); g.moveTo(x + 6, 54); g.lineTo(x + 16, 12); g.lineTo(x + 26, 54); g.closePath(); g.fill();
    g.beginPath(); g.moveTo(x + 22, 10); g.lineTo(x + 42, 10); g.lineTo(x + 32, 50); g.closePath(); g.fill();
  }
  const t = new THREE.CanvasTexture(c);
  t.wrapS = THREE.RepeatWrapping;
  t.repeat.set(4, 1);
  return t;
}

// rotor plate: pocketed spokes, seen from above
function rotorTexture() {
  const c = document.createElement('canvas');
  c.width = c.height = 256;
  const g = c.getContext('2d');
  g.fillStyle = '#9aa0a8';
  g.fillRect(0, 0, 256, 256);
  g.fillStyle = '#2a2d33';
  for (let i = 0; i < 12; i++) {
    const a0 = (i * Math.PI) / 6 + 0.05, a1 = a0 + Math.PI / 6 - 0.1;
    for (const [r0, r1] of [[34, 78], [86, 122]]) {
      g.beginPath();
      g.arc(128, 128, r1, a0, a1);
      g.arc(128, 128, r0, a1, a0, true);
      g.closePath();
      g.fill();
    }
  }
  return new THREE.CanvasTexture(c);
}

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
  green: std(0x8ccf6a, 0.6, 0.1), // 2910's drum wheels
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

// stowed intakes stand nearly upright just in front of the hopper
const STOWED = 1.4;

// Parts exported from a team's own CAD (cad/robots/*.glb, see cad/README.md), loaded in the
// browser only; the drawn part stays until (and unless) the CAD arrives.
function cadPart(parent, file, onLoad) {
  if (typeof window === 'undefined') return;
  import('three/addons/loaders/GLTFLoader.js')
    .then(({ GLTFLoader }) => new GLTFLoader().loadAsync('cad/' + file))
    .then((gltf) => {
      gltf.scene.traverse((o) => { if (o.isMesh) { o.castShadow = true; o.receiveShadow = true; } });
      parent.add(gltf.scene);
      onLoad(gltf.scene);
    })
    .catch((err) => console.warn('robot CAD part failed to load:', file, err));
}

// 2910's slap-down intake from their Onshape CAD (Re•Blitz top level assembly, "Pivoting Intake
// Assembly"): pivot position in the robot frame, and how far it swings from stowed (as exported)
// until its 2in roller is down at FUEL height, ~7.8in past the bumper
const INTAKE_2910 = { pivot: [0.273, 0.17], swing: -2.234 };

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
  // expanding front section: telescoping side panels + front wall ride out over the intake as it
  // deploys (the stowed intake stands just behind the front wall)
  const front = new THREE.Group();
  root.add(front);
  for (const s of [-1, 1]) {
    polyWall(front, extLen + 0.04, hH * 0.85, hFront - extLen / 2 + 0.02, hY + hH * 0.85 / 2, s * (W / 2 - 0.028), 0, M.polyTint, M.alu);
    tube(root, hFront - 0.28, s * (W / 2 - 0.05), hFront - 0.02, s * (W / 2 - 0.05), hY + hH - 0.02, M.aluDark, 0.012, 0.012); // slide rail
  }
  // front wall stops short of the floor: the intake feeds FUEL in through the slot under it
  const slot = 0.2, fwH = hH * 0.85 - slot;
  polyWall(front, W - 0.06, fwH, hFront + 0.02, hY + slot + fwH / 2, 0, Math.PI / 2, M.polyTint, M.alu);
  // clear lids: one over the fixed hopper, and one on the expanding section, nested just under it
  const lid = (parent, x0, x1, y, w) => {
    const m = mesh(new THREE.PlaneGeometry(x1 - x0, w), M.polyTint, parent, (x0 + x1) / 2, y, 0);
    m.rotation.x = -Math.PI / 2;
    m.castShadow = false;
    for (const s of [-1, 1]) rbx(x1 - x0, 0.012, 0.016, 0.002, M.alu, parent, (x0 + x1) / 2, y, s * w / 2);
    rbx(0.016, 0.012, w, 0.002, M.alu, parent, x1, y, 0);
    return m;
  };
  lid(hopper, hBack, hFront, hY + hH, W - 0.03);
  lid(front, hFront - extLen + 0.02, hFront + 0.02, hY + hH * 0.85, W - 0.06);
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
  const cadIntake = new THREE.Group();
  cadIntake.position.set(INTAKE_2910.pivot[0], INTAKE_2910.pivot[1], 0);
  root.add(cadIntake);
  cadPart(cadIntake, 'robots/2910-intake.glb', () => { intake.visible = false; });

  // ---- FUEL inside: main hopper, then the extension, then the tower
  const stored = storedGrid([-0.13, 0.02, 0.17, 0.3], [-0.225, -0.075, 0.075, 0.225], [0.2, 0.34, 0.47]);
  stored.push(...storedGrid([0.28, 0.43], [-0.225, -0.075, 0.075, 0.225], [0.22, 0.36], true).map((v) => { v.x -= extLen; return v; }));
  for (const z of [-0.19, -0.063, 0.063, 0.19]) stored.push(new THREE.Vector3(-0.2, 0.3, z));

  const anim = (st, dt) => {
    // intake: 0 = stowed (arms up in front of the hopper), 1 = deployed over the bumper
    intake.rotation.z = lerp(STOWED, deploy, st.intakeDeploy);
    for (const r of intakeRollers) r.rotation.z += st.intakeSpeed * dt * 40;
    cadIntake.rotation.z = lerp(0, INTAKE_2910.swing, st.intakeDeploy);
    front.position.x = st.hopperDeploy * extLen; // the hopper expands with the intake
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

  // ---- hopper: smoked panels on the frame, chamfered at the back, with the teal truss frame
  // along the top and at the back corners (as in the tech binder renders)
  const hY = 0.17, hH = H - hY - 0.02;
  const wallZ = W / 2 - 0.012;
  const smoke = new THREE.MeshPhysicalMaterial({ color: 0x3a4048, transparent: true, opacity: 0.5, roughness: 0.25, depthWrite: false, side: THREE.DoubleSide });
  for (const s of [-1, 1]) polyWall(root, L - chamfer - 0.02, hH, chamfer / 2 - 0.01, hY + hH / 2, s * wallZ, 0, smoke, M.anodBlack);
  polyWall(root, W - 2 * chamfer, hH, -L / 2 + 0.012, hY + hH / 2, 0, Math.PI / 2, smoke, M.anodBlack);
  const cw = Math.SQRT2 * chamfer;
  for (const s of [-1, 1]) polyWall(root, cw, hH, -L / 2 + chamfer / 2, hY + hH / 2, s * (W / 2 - chamfer / 2), -s * Math.PI / 4, smoke, M.anodBlack);
  const topY = hY + hH;
  const truss = new THREE.MeshStandardMaterial({ color: cfg.colors.accent, map: trussTexture(), alphaTest: 0.4, side: THREE.DoubleSide, roughness: 0.45, metalness: 0.4 });
  const rail = (len, x, z, ry) => { const m = mesh(new THREE.PlaneGeometry(len, 0.07), truss, root, x, topY - 0.035, z); m.rotation.y = ry; m.castShadow = false; return m; };
  rail(W - 2 * chamfer, -L / 2 + 0.012, 0, Math.PI / 2);
  for (const s of [-1, 1]) {
    rail(cw, -L / 2 + chamfer / 2, s * (W / 2 - chamfer / 2), -s * Math.PI / 4);
    rail(L - chamfer - 0.04, chamfer / 2 - 0.02, s * wallZ, 0);
    for (const [x, z] of [[-L / 2 + 0.012, s * (W / 2 - chamfer)], [-L / 2 + chamfer, s * wallZ]]) {
      const post = mesh(new THREE.PlaneGeometry(0.05, hH), truss, root, x, hY + hH / 2, z);
      post.rotation.y = x < -L / 2 + 0.05 ? Math.PI / 2 : 0;
      post.castShadow = false;
    }
  }

  // ---- the intake: a box that slides out on rack-and-pinion rails and latches down at the
  // start of the match (it's also the hopper's extension)
  const ext = new THREE.Group();
  root.add(ext);
  const carbon = std(0x24272c, 0.5, 0.35);
  const boxBack = L / 2 - extLen - 0.04, boxLen = extLen + 0.06;
  const side = new THREE.Shape();
  // side plate profile (x forward, y up): tall at the back, a lower nose around the roller
  side.moveTo(0, 0.02); side.lineTo(boxLen - 0.02, 0.02); side.lineTo(boxLen + 0.02, 0.07); side.lineTo(boxLen + 0.02, 0.2);
  side.lineTo(boxLen - 0.05, 0.26); side.lineTo(boxLen - 0.08, hH * 0.9 + hY - 0.06); side.lineTo(0, hH * 0.9 + hY - 0.06); side.lineTo(0, 0.02);
  const sideGeo = new THREE.ExtrudeGeometry(side, { depth: 0.006, bevelEnabled: false });
  for (const s of [-1, 1]) mesh(sideGeo, carbon, ext, boxBack, 0, s * (wallZ - 0.015) - 0.003);
  // front panel above the intake opening (cut out low for capacity), impact guards, racks
  const fwY = 0.27, fwH = hY + hH * 0.9 - 0.06 - fwY;
  polyWall(ext, W - 0.06, fwH, L / 2 - 0.05, fwY + fwH / 2, 0, Math.PI / 2, smoke, M.anodBlack);
  for (const s of [-1, 1]) {
    const guard = bx(0.012, 0.2, 0.09, M.alu, ext, L / 2 + 0.03, 0.14, s * (wallZ - 0.02));
    guard.rotation.y = s * 0.5;
    for (let i = 0; i < 14; i++) bx(0.01, 0.012, 0.012, M.anodBlack, root, L / 2 - 0.36 + i * 0.026, 0.215, s * (wallZ - 0.035)); // rack teeth
    bx(0.38, 0.012, 0.014, M.aluDark, root, L / 2 - 0.18, 0.205, s * (wallZ - 0.035)); // rack
  }
  const intakeRollers = [
    cylZ(0.035, W - 0.05, std(0xe8eaec, 0.4, 0.05), ext, L / 2 + 0.005, 0.07, 0, 24), // big front roller
    wheelStack(ext, W - 0.09, 0.024, 9, M.compliant, L / 2 - 0.06, 0.17, 0, 0.025),
  ];
  kraken(ext, L / 2 - 0.1, 0.24, W / 2 - 0.07, true, 'z');

  // ---- net over the hopper (open around the turret), and over the intake box
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
  extShape.moveTo(boxBack, -(W / 2 - 0.035)); extShape.lineTo(L / 2 - 0.05, -(W / 2 - 0.035));
  extShape.lineTo(L / 2 - 0.05, W / 2 - 0.035); extShape.lineTo(boxBack, W / 2 - 0.035);
  const extNetGeo = new THREE.ShapeGeometry(extShape);
  extNetGeo.rotateX(-Math.PI / 2);
  mesh(extNetGeo, M.net, ext, 0, hY + hH * 0.9 - 0.06, 0).castShadow = false;

  // ---- Dye Rotor: a low, wide floor inside a ring wall. FUEL falls onto it, and the Dolphin
  // Fin sweeps round over it, pushing FUEL to the center column, where a ramp of passive rollers
  // climbs to the feeder wheels and the turret on top. Printed "stadium" pieces fill the corners
  // and funnel FUEL down onto the floor.
  const bay = cfg.bay, rs = bay.rotor;
  const dark = std(0x33373d, 0.55, 0.4);
  // the floor stays still; the Dolphin Fin on its arm is what turns
  const plateMat = new THREE.MeshStandardMaterial({ color: 0xb4bac2, map: rotorTexture(), roughness: 0.5, metalness: 0.3 });
  const floorPlate = mesh(new THREE.CylinderGeometry(rs.r, rs.r, 0.01, 64), [dark, plateMat, dark], root, rs.x, rs.y - 0.005, rs.z);
  floorPlate.receiveShadow = true;
  const rotor = new THREE.Group();
  rotor.position.set(rs.x, rs.y, rs.z);
  root.add(rotor);
  mesh(new THREE.CylinderGeometry(0.05, 0.05, 0.03, 24), dark, rotor, 0, 0.015, 0);
  bx(rs.r - 0.06, 0.025, 0.02, dark, rotor, (rs.r + 0.04) / 2, 0.0125, 0); // arm
  // Dolphin Fin at the outer end of the arm: a curved blade that sweeps FUEL round and up
  const finShape = new THREE.Shape();
  const f0 = bay.column.r + 0.015; // the blade runs from just outside the column to the rim
  finShape.moveTo(f0, 0); finShape.lineTo(rs.r - 0.005, 0); finShape.lineTo(rs.r - 0.005, 0.1);
  finShape.quadraticCurveTo(rs.r - 0.03, 0.16, rs.r - 0.07, 0.14); finShape.quadraticCurveTo(f0 + 0.04, 0.06, f0, 0);
  const dolphin = mesh(new THREE.ExtrudeGeometry(finShape, { depth: 0.012, bevelEnabled: true, bevelThickness: 0.003, bevelSize: 0.003, bevelSegments: 2 }), teal, rotor, 0, 0, -0.006);
  dolphin.castShadow = true;
  // fixed ring wall around the rotor, on standoffs
  const ringMat = new THREE.MeshPhysicalMaterial({ color: 0x2e3238, transparent: true, opacity: 0.55, roughness: 0.3, depthWrite: false, side: THREE.DoubleSide });
  mesh(new THREE.CylinderGeometry(rs.r + 0.01, rs.r + 0.01, 0.1, 64, 1, true), ringMat, root, rs.x, rs.y + 0.05, rs.z).castShadow = false;
  for (let i = 0; i < 12; i++) {
    const a = (i * Math.PI) / 6;
    mesh(new THREE.CylinderGeometry(0.005, 0.005, 0.1, 6), M.anodBlack, root, rs.x + Math.cos(a) * (rs.r + 0.012), rs.y + 0.05, rs.z + Math.sin(a) * (rs.r + 0.012));
  }
  kraken(root, rs.x + 0.16, 0.07, rs.z + 0.25, true); // rotor drive
  // center column carrying the turret, wrapped in the "shrink wrap" sheet
  const col = bay.column;
  mesh(new THREE.CylinderGeometry(col.r, col.r, col.y1 - rs.y, 32, 1, true), std(0x2a2d33, 0.5, 0.3, { side: THREE.DoubleSide }), root, col.x, (rs.y + col.y1) / 2, col.z);
  mesh(new THREE.CylinderGeometry(col.r + 0.035, col.r + 0.035, 0.012, 40), dark, root, col.x, col.y1, col.z);
  const wrap = mesh(new THREE.CylinderGeometry(col.r + 0.06, col.r + 0.015, 0.16, 32, 1, true, Math.PI * 0.2, Math.PI * 1.1), ringMat, root, col.x, rs.y + 0.1, col.z);
  wrap.castShadow = false;
  // ramp of passive rollers climbing the column, then the feeder wheels
  const rampRollers = [];
  const up = new THREE.Vector3(0, 1, 0);
  for (let i = 0; i < 8; i++) {
    // each roller lies across the ramp (pointing out from the column), climbing as it wraps round
    const a = -0.2 + i * 0.28, y = rs.y + 0.1 + i * 0.035;
    const dir = new THREE.Vector3(Math.sin(a), 0, Math.cos(a));
    const rr = mesh(new THREE.CylinderGeometry(0.011, 0.011, 0.09, 10), M.alu, root, col.x + dir.x * (col.r + 0.05), y, col.z + dir.z * (col.r + 0.05));
    rr.quaternion.setFromUnitVectors(up, dir);
    rampRollers.push(rr);
  }
  const omniV = cylZ(0.038, 0.03, M.compliant, root, col.x + 0.02, rs.y + 0.33, col.z + col.r - 0.02, 20);
  omniV.rotation.y = Math.PI / 2;
  const kicker = wheelStack(root, 0.1, 0.016, 3, M.compliant, col.x - 0.01, rs.y + 0.12, col.z + col.r + 0.03, 0.03);
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
        if (d < 0.02) continue;
        if ((cx - bay.x0) - Math.abs(cz) + bay.hw - bay.chamfer < 0) continue;
        const h = Math.min(bay.funnel.cap, d * bay.funnel.slope);
        const top = rs.y + Math.max(step, Math.ceil(h / step) * step);
        box(x, Math.min(x + cell, bay.x1), rs.y - 0.01, top, z, Math.min(z + cell, bay.hw));
      }
    }
    const g = new THREE.BufferGeometry();
    g.setAttribute('position', new THREE.Float32BufferAttribute(pos, 3));
    g.computeVertexNormals();
    mesh(g, std(0x3b3f46, 0.7, 0.05), root, 0, 0, 0);
  }

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
    // the intake box slides out on its racks (and stays out)
    ext.position.x = st.hopperDeploy * extLen;
    intakeRollers[0].rotation.y -= st.intakeSpeed * dt * 35; // cylinder along z: spin about its axis
    intakeRollers[1].rotation.z -= st.intakeSpeed * dt * 35;
    // the Dolphin Fin: ~2.25 rev/s when feeding, otherwise slowly backward to agitate the load
    // (its angle comes from the hopper physics, so the fin and the FUEL it pushes agree)
    rotor.rotation.y = st.rotorAngle ?? 0;
    kicker.rotation.z -= st.feeding * dt * 30;
    omniV.rotation.x -= st.feeding * dt * 30;
    for (const r of rampRollers) r.rotateY(-st.feeding * dt * 20);
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
    intake.rotation.z = lerp(STOWED, deploy, st.intakeDeploy);
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
