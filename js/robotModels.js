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
import { hookPath } from './hopper.js';

export const BUMP_Y0 = 0.055, BUMP_Y1 = 0.16;
const lerp = THREE.MathUtils.lerp;

// ------------------------------------------------------------------ materials
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

// frame tubes, bellypan and swerve modules (frame: false when the team's CAD brings its own frame)
function addDrivebase(root, cfg, chamferBack = 0, frame = true) {
  const L = cfg.frame.length, W = cfg.frame.width;
  const modules = [];
  const inset = 0.075;
  for (const sx of [1, -1]) for (const sz of [1, -1]) modules.push(swerveModule(root, sx * (L / 2 - inset), sz * (W / 2 - inset)));
  if (!frame) return modules;
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
      gltf.scene.traverse((o) => {
        if (!o.isMesh) return;
        o.castShadow = true;
        o.receiveShadow = true;
        // polycarbonate (see-through in the CAD, or marked "poly" by tools/extract-parts.mjs)
        const m = o.material;
        if (m.transparent || m.name === 'poly') { o.material = M.poly; o.castShadow = false; }
      });
      parent.add(gltf.scene);
      onLoad(gltf.scene);
    })
    .catch((err) => console.warn('robot CAD part failed to load:', file, err));
}

// 2910's slap-down intake from their Onshape CAD (Re•Blitz top level assembly, "Pivoting Intake
// Assembly"): pivot position in the robot frame, and how far it swings from stowed (as exported)
// until its 2in roller is down at FUEL height, ~7.8in past the bumper (cfg.intake.arm)
const intake2910 = (cfg) => { const a = cfg.intake.arm; return { pivot: [a.x, a.y], swing: (a.deployDeg - a.stowDeg) * Math.PI / 180 }; };

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

  // ---- shooter tower at the back: pocketed billet side plates, drum + hood. It fires out the
  // back, away from the intake: FUEL comes up the front of the drum from the indexer, rides over
  // the top under the hood and leaves at the back.
  const tower = new THREE.Group();
  tower.position.set(-L / 2 + 0.1, 0, 0);
  root.add(tower);
  const holes = [[-0.04, -0.12, 0.025], [0.04, -0.12, 0.025], [-0.04, 0.0, 0.02], [0.05, 0.02, 0.018], [0, 0.12, 0.018]];
  const drumY = 0.37, hoodR = 0.17; // hood clears a FUEL (5.9in) squeezed on the 4in drum
  for (const s of [-1, 1]) {
    pocketPlate(tower, 0.19, H - 0.14, 0.0127, plate, holes, 0, 0.14 + (H - 0.14) / 2, s * (W / 2 - 0.03));
    kraken(tower, -0.02, drumY, s * (W / 2 + 0.01), false, 'z');
  }
  const drum = wheelStack(tower, W - 0.09, 0.0508, 4, M.green, -0.02, drumY, 0, (W - 0.12) / 4);
  const hood = new THREE.Group();
  hood.position.set(-0.02, drumY, 0);
  tower.add(hood);
  // the hood only covers the exit side: a polycarbonate arc on ribs from just past the top of the
  // drum (70deg) back to where FUEL leaves (139deg), so the drum stays in view
  const arc0 = 1.22, arcLen = 1.21;
  const hoodShell = new THREE.Mesh(new THREE.CylinderGeometry(hoodR, hoodR, W - 0.1, 20, 1, true, arc0 + Math.PI / 2, arcLen), M.polyTint);
  hoodShell.rotation.x = Math.PI / 2;
  hood.add(hoodShell);
  for (let i = 0; i < 5; i++) {
    const rib = new THREE.Mesh(new THREE.TorusGeometry(hoodR + 0.004, 0.004, 4, 24, arcLen), purple);
    rib.rotation.z = arc0;
    rib.position.z = -W / 2 + 0.06 + i * ((W - 0.12) / 4);
    hood.add(rib);
  }
  const hoodRollers = [];
  for (const a of [1.45, 1.95]) hoodRollers.push(cylZ(0.016, W - 0.12, M.alu, hood, Math.cos(a) * (hoodR - 0.02), Math.sin(a) * (hoodR - 0.02), 0, 12));
  // compliant indexer wheels feeding the drum + inclined powered floor
  const indexer = wheelStack(tower, W - 0.1, 0.03, 8, M.compliant, 0.085, 0.25, 0, 0.022);
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
  const rails = [];
  for (const s of [-1, 1]) {
    polyWall(front, extLen + 0.04, hH * 0.85, hFront - extLen / 2 + 0.02, hY + hH * 0.85 / 2, s * (W / 2 - 0.028), 0, M.polyTint, M.alu);
    rails.push(tube(root, hFront - 0.28, s * (W / 2 - 0.05), hFront - 0.02, s * (W / 2 - 0.05), hY + hH - 0.02, M.aluDark, 0.012, 0.012)); // slide rail
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
  // their "R2 Hopper" assembly: side, top and front panels that sit over the shooter when stowed
  // and slide out along the slotted rails as the intake deploys; replaces the drawn hopper
  const drawnFront = [...front.children];
  cadPart(front, 'robots/2910-hopper.glb', () => {
    hopper.visible = false;
    for (const c of [...drawnFront, ...rails]) c.visible = false;
  });
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
  const INTAKE_2910 = intake2910(cfg);
  cadIntake.position.set(INTAKE_2910.pivot[0], INTAKE_2910.pivot[1], 0);
  root.add(cadIntake);
  cadPart(cadIntake, 'robots/2910-intake.glb', () => { intake.visible = false; });
  // their "Shooter & Feeder" assembly: powered floor, the roller ramp that indexes FUEL up under
  // the rollered hood, and the drum at the back; it replaces the drawn tower and floor
  cadPart(root, 'robots/2910-shooter.glb', () => { tower.visible = false; feeder.visible = false; });

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
    hood.rotation.z = -(st.hoodDeg - 58) * Math.PI / 180 * 0.4; // a higher shot tips the exit up
  };
  return { root, anim, stored, modules, extLen };
}

// ============================================================ 4414 RIPCURRENT
// Modeled on 4414's tech binder CAD renders (2026.team4414.com; their CAD isn't public):
//  - chamfer-backed hopper of smoked polycarbonate with the teal truss along the back, and a
//    pocketed top plate that runs from the back truss to the turret bearing
//  - the intake box (also the hopper's extension) slides out on racks at the start of the
//    match: a front panel with an under-roller, a hinged ramp of passive rollers up to the
//    bumper, and a star roller that drives FUEL over it
//  - the Dye Rotor: a pocketed platter that spins, carrying FUEL round (the Dolphin Fin ramp on
//    its outside sweeps the load), into a fixed hook of passive rollers that steers it to the
//    feeder at the center column; the feeder wheels lift it up the ramp into the turret
//  - a netted top that stays folded down in the starting configuration and springs up once
//    the hopper is out (it squashes under the TRENCH arm)

// carbon / SRPP weave
function carbonTexture() {
  const c = document.createElement('canvas');
  c.width = c.height = 64;
  const g = c.getContext('2d');
  g.fillStyle = '#7d8289';
  g.fillRect(0, 0, 64, 64);
  for (let y = 0; y < 64; y += 4) {
    for (let x = 0; x < 64; x += 4) {
      g.fillStyle = ((x + y) / 4) % 2 ? '#8f949b' : '#6c7178';
      g.fillRect(x, y, 4, 4);
    }
  }
  const t = new THREE.CanvasTexture(c);
  t.wrapS = t.wrapT = THREE.RepeatWrapping;
  t.repeat.set(12, 12);
  return t;
}

// knotless netting, 2in square mesh of thin cord: one tile = 0.1 m (two cells)
function fineNetTexture() {
  const c = document.createElement('canvas');
  c.width = c.height = 128;
  const g = c.getContext('2d');
  g.clearRect(0, 0, 128, 128);
  g.strokeStyle = 'rgba(255,255,255,0.95)';
  g.lineWidth = 2.2;
  for (const p of [1, 65]) {
    g.beginPath(); g.moveTo(p, 0); g.lineTo(p, 128); g.stroke();
    g.beginPath(); g.moveTo(0, p); g.lineTo(128, p); g.stroke();
  }
  g.fillStyle = 'rgba(255,255,255,1)';
  for (const x of [1, 65]) for (const y of [1, 65]) g.fillRect(x - 2.5, y - 2.5, 5, 5); // knots
  const t = new THREE.CanvasTexture(c);
  t.wrapS = t.wrapT = THREE.RepeatWrapping;
  t.repeat.set(10, 10);
  t.anisotropy = 8;
  return t;
}

// strip with triangular cutouts (the teal truss), len along x, w along y, t thick (centered on z)
function trussStrip(len, w, t, mat, parent) {
  const s = new THREE.Shape();
  s.moveTo(0, 0); s.lineTo(len, 0); s.lineTo(len, w); s.lineTo(0, w); s.closePath();
  const m = 0.006, n = Math.max(1, Math.floor((len - m) / (w * 0.9)));
  const p = (len - m) / n;
  for (let i = 0; i < n; i++) {
    const x0 = m + i * p, x1 = x0 + p - m;
    if (x1 - x0 < 0.01) continue;
    const h = new THREE.Path();
    if (i % 2) { h.moveTo(x0, w - m); h.lineTo(x1, w - m); h.lineTo((x0 + x1) / 2, m); }
    else { h.moveTo(x0, m); h.lineTo(x1, m); h.lineTo((x0 + x1) / 2, w - m); }
    h.closePath();
    s.holes.push(h);
  }
  const g = new THREE.ExtrudeGeometry(s, { depth: t, bevelEnabled: false });
  g.translate(0, 0, -t / 2);
  return mesh(g, mat, parent);
}

// button-head bolts, all in one instanced mesh: add(position, outward normal)
function boltSet(parent, mat, r = 0.004) {
  const pts = [];
  return {
    add(p, n) { pts.push([p.clone(), n.clone()]); },
    done() {
      const geo = new THREE.CylinderGeometry(r, r * 1.1, 0.003, 8);
      geo.translate(0, 0.0015, 0);
      const im = new THREE.InstancedMesh(geo, mat, Math.max(1, pts.length));
      const up = new THREE.Vector3(0, 1, 0), q = new THREE.Quaternion(), m4 = new THREE.Matrix4(), one = new THREE.Vector3(1, 1, 1);
      pts.forEach(([p, n], i) => { q.setFromUnitVectors(up, n); m4.compose(p, q, one); im.setMatrixAt(i, m4); });
      im.count = pts.length;
      parent.add(im);
      return im;
    },
  };
}

// flat part from an outline in the XZ plane (robot frame, points [x, z]), top face at y
function flatPart(outline, holes, t, mat, parent, y) {
  const s = new THREE.Shape(outline.map(([x, z]) => new THREE.Vector2(x, -z)));
  for (const h of holes) s.holes.push(new THREE.Path(h.map(([x, z]) => new THREE.Vector2(x, -z))));
  const g = new THREE.ExtrudeGeometry(s, { depth: t, bevelEnabled: false, curveSegments: 24 });
  g.rotateX(-Math.PI / 2);
  return mesh(g, mat, parent, 0, y - t, 0);
}

const arcPts = (cx, cz, r, a0, a1, n) => Array.from({ length: n + 1 }, (_, i) => { const a = a0 + ((a1 - a0) * i) / n; return [cx + r * Math.cos(a), cz - r * Math.sin(a)]; });

// spur gear lying flat (axis along y), top at y = 0
function spurGear(r, teeth, t, mat, parent, holes = 0) {
  const s = new THREE.Shape();
  const td = r * 0.06;
  for (let i = 0; i < teeth; i++) {
    const a = (i / teeth) * Math.PI * 2, da = (Math.PI * 2) / teeth;
    const pts = [[r - td, a], [r + td, a + da * 0.2], [r + td, a + da * 0.45], [r - td, a + da * 0.65]];
    pts.forEach(([rr, aa], k) => { const x = rr * Math.cos(aa), y = rr * Math.sin(aa); if (i === 0 && k === 0) s.moveTo(x, y); else s.lineTo(x, y); });
  }
  s.closePath();
  for (let i = 0; i < holes; i++) {
    const a0 = (i / holes) * Math.PI * 2 + 0.12, a1 = ((i + 1) / holes) * Math.PI * 2 - 0.12;
    const h = new THREE.Path();
    h.absarc(0, 0, r * 0.78, a0, a1, false);
    h.absarc(0, 0, r * 0.3, a1, a0, true);
    s.holes.push(h);
  }
  const g = new THREE.ExtrudeGeometry(s, { depth: t, bevelEnabled: false, curveSegments: 6 });
  g.rotateX(-Math.PI / 2);
  g.translate(0, -t, 0);
  return mesh(g, mat, parent);
}

// the Dolphin Fin: a wedge on the rotor's outside rim, tall at its leading face and sloping down
// behind it (rotor frame, it leads at angle 0 and trails back to -span)
function dolphinFinGeometry(r0, r1, span, hMax) {
  const NU = 14, NV = 5;
  const pos = [], idx = [];
  const hAt = (u, v) => hMax * (0.35 + 0.65 * v) * Math.pow(1 - u, 1.4) * (1 - 0.35 * Math.pow(u, 0.3) * (1 - v));
  const P = (u, v, top) => {
    const a = -u * span - 0.18 * (1 - v) * (1 - v); // leading edge swept back toward the column
    const r = r0 + v * (r1 - r0);
    return [r * Math.cos(a), top ? hAt(u, v) : 0, -r * Math.sin(a)];
  };
  const quad = (a, b, c, d) => { const i = pos.length / 3; pos.push(...a, ...b, ...c, ...d); idx.push(i, i + 1, i + 2, i, i + 2, i + 3); };
  for (let i = 0; i < NU; i++) {
    for (let j = 0; j < NV; j++) {
      const u0 = i / NU, u1 = (i + 1) / NU, v0 = j / NV, v1 = (j + 1) / NV;
      quad(P(u0, v0, 1), P(u1, v0, 1), P(u1, v1, 1), P(u0, v1, 1));
    }
    const u0 = i / NU, u1 = (i + 1) / NU;
    quad(P(u0, 1, 0), P(u1, 1, 0), P(u1, 1, 1), P(u0, 1, 1)); // outer side
    quad(P(u0, 0, 1), P(u1, 0, 1), P(u1, 0, 0), P(u0, 0, 0)); // inner side
  }
  for (let j = 0; j < NV; j++) quad(P(0, j / NV, 0), P(0, j / NV, 1), P(0, (j + 1) / NV, 1), P(0, (j + 1) / NV, 0)); // leading face
  const g = new THREE.BufferGeometry();
  g.setAttribute('position', new THREE.Float32BufferAttribute(pos, 3));
  g.setIndex(idx);
  g.computeVertexNormals();
  return g;
}

// thin band following a curve of [x, z] points at height y0..y1 (vertical ribbon)
function ribbon(points, y0, y1, mat, parent) {
  const pos = [], idx = [];
  points.forEach(([x, z], i) => {
    pos.push(x, y0, z, x, y1, z);
    if (i) { const k = 2 * i; idx.push(k - 2, k, k + 1, k - 2, k + 1, k - 1); }
  });
  const g = new THREE.BufferGeometry();
  g.setAttribute('position', new THREE.Float32BufferAttribute(pos, 3));
  g.setIndex(idx);
  g.computeVertexNormals();
  return mesh(g, mat, parent);
}

// flat curved strip following [x, z] points, width w, at height y
function flatRibbon(points, w, y, mat, parent) {
  const pos = [], idx = [];
  points.forEach(([x, z], i) => {
    const [ax, az] = points[Math.max(0, i - 1)], [bx2, bz] = points[Math.min(points.length - 1, i + 1)];
    let tx = bx2 - ax, tz = bz - az;
    const l = Math.hypot(tx, tz) || 1;
    tx /= l; tz /= l;
    pos.push(x - tz * w / 2, y, z + tx * w / 2, x + tz * w / 2, y, z - tx * w / 2);
    if (i) { const k = 2 * i; idx.push(k - 2, k, k + 1, k - 2, k + 1, k - 1); }
  });
  const g = new THREE.BufferGeometry();
  g.setAttribute('position', new THREE.Float32BufferAttribute(pos, 3));
  g.setIndex(idx);
  g.computeVertexNormals();
  return mesh(g, mat, parent);
}

// cylinder between two points
function rod(parent, a, b, r, mat, segs = 8) {
  const d = new THREE.Vector3().subVectors(b, a);
  const m = mesh(new THREE.CylinderGeometry(r, r, d.length(), segs), mat, parent);
  m.position.copy(a).add(b).multiplyScalar(0.5);
  m.quaternion.setFromUnitVectors(new THREE.Vector3(0, 1, 0), d.normalize());
  return m;
}

// A net stretched over an open hopper top: pinned along its edges (and round a hole, e.g. a
// turret), draped over whatever FUEL pokes up above the pins, taut everywhere else. An extending
// hopper can carry its front edge out: the part past xFixed stretches to follow it.
//   x0..x1: back and (stowed) front edge; x1Out: the front edge with the hopper out; hw: half width;
//   y: height of the pinned edges (yFront: the front edge's, for a net stretched diagonally down
//   to an intake). drape(load, ex, at): ex is how far the front has moved out; at { y, yFront }
//   moves the pinned heights (a lid that lifts).
function hopperNet(parent, { x0, x1, x1Out = x1, xFixed = x1, hw, y, yFront = y, hole = null, NX = 36, NZ = 28, color = 0x15171a }) {
  const mat = new THREE.MeshStandardMaterial({ map: fineNetTexture(), transparent: true, depthWrite: false, side: THREE.DoubleSide, roughness: 0.9, color });
  const nv = (NX + 1) * (NZ + 1);
  const base = new Float32Array(nv), pin = new Uint8Array(nv), hNow = new Float32Array(nv), hMin = new Float32Array(nv);
  const pos = new Float32Array(nv * 3), uv = new Float32Array(nv * 2), idx = [];
  const inHole = (x, z, pad) => hole && Math.hypot(x - hole.x, z - hole.z) < hole.r + pad;
  for (let i = 0; i <= NX; i++) {
    for (let j = 0; j <= NZ; j++) {
      const k = i * (NZ + 1) + j, x = x0 + ((x1Out - x0) * i) / NX, z = -hw + (2 * hw * j) / NZ;
      base[k] = x;
      pin[k] = i === 0 || i === NX || j === 0 || j === NZ || inHole(x, z, 0.01) ? 1 : 0;
      hNow[k] = y;
      pos.set([x, y, z], k * 3);
      uv.set([x, z], k * 2);
    }
  }
  for (let i = 0; i < NX; i++) {
    for (let j = 0; j < NZ; j++) {
      const k = i * (NZ + 1) + j;
      if (inHole(x0 + ((x1Out - x0) * (i + 0.5)) / NX, -hw + (2 * hw * (j + 0.5)) / NZ, 0)) continue;
      idx.push(k, k + 1, k + NZ + 2, k, k + NZ + 2, k + NZ + 1);
    }
  }
  const geo = new THREE.BufferGeometry();
  geo.setAttribute('position', new THREE.BufferAttribute(pos, 3));
  geo.setAttribute('uv', new THREE.BufferAttribute(uv, 2));
  geo.setIndex(idx);
  geo.computeVertexNormals();
  const m = mesh(geo, mat, parent);
  m.castShadow = false;
  m.frustumCulled = false;
  m.userData.net = true;
  const R = FUEL.radius;
  const hPin = new Float32Array(nv);
  const drape = (load, ex = 0, at = null) => {
    const kx = x1Out > xFixed ? (x1 + ex - xFixed) / (x1Out - xFixed) : 1;
    const yb = at?.y ?? y, yf = at?.yFront ?? yFront, xf = x1 + ex;
    for (let k = 0; k < nv; k++) {
      const x = base[k] <= xFixed ? base[k] : xFixed + (base[k] - xFixed) * kx;
      pos[k * 3] = x;
      hPin[k] = yb + (yf - yb) * Math.min(1, Math.max(0, (x - x0) / Math.max(0.01, xf - x0)));
      hMin[k] = hPin[k];
    }
    // held up by the FUEL poking above the pins
    for (const e of load || []) {
      const p = e.p;
      if (p.y + R < Math.min(yb, yf)) continue;
      for (let k = 0; k < nv; k++) {
        if (pin[k]) continue;
        const dx = pos[k * 3] - p.x, dz = pos[k * 3 + 2] - p.z, d2 = dx * dx + dz * dz;
        if (d2 < R * R) hMin[k] = Math.max(hMin[k], p.y + Math.sqrt(R * R - d2) + 0.004);
      }
    }
    // a taut sheet: each point sits at its neighbors' average unless the FUEL holds it up
    for (let it = 0; it < 10; it++) {
      for (let i = 1; i < NX; i++) {
        for (let j = 1; j < NZ; j++) {
          const k = i * (NZ + 1) + j;
          if (pin[k]) continue;
          hNow[k] = Math.max(hMin[k], (hNow[k - 1] + hNow[k + 1] + hNow[k - NZ - 1] + hNow[k + NZ + 1]) / 4);
        }
      }
    }
    for (let k = 0; k < nv; k++) pos[k * 3 + 1] = pin[k] ? hPin[k] : hNow[k];
    geo.attributes.position.needsUpdate = true;
    geo.computeVertexNormals();
  };
  return { mesh: m, drape };
}

function build4414(cfg, alliance) {
  const root = new THREE.Group();
  const L = cfg.frame.length, W = cfg.frame.width;
  const bay = cfg.bay, rs = bay.rotor, col = bay.column;
  const chamfer = bay.chamfer;
  const extLen = cfg.storage.extLen;
  const wallTop = bay.wallTop; // top of the hopper walls and the top plate
  const hY = 0.15;             // walls start just under the bumper top
  const wallZ = W / 2 - 0.012;
  const R_FUEL = FUEL.radius;
  addBumpers(root, cfg, alliance, chamfer);
  const modules = addDrivebase(root, cfg, chamfer);
  addElectronics(root, -L / 2 + 0.2, W / 2 - 0.14, Math.PI / 2); // battery out at the edge

  const teal = std(cfg.colors.accent, 0.4, 0.45);
  const blackAl = std(0x2a2e35, 0.42, 0.55);
  const darkPlate = std(0x50565f, 0.45, 0.5);
  const greyAl = std(0x5c6169, 0.38, 0.7);
  const plastic = std(0x3a3e45, 0.7, 0.05);
  const carbon = new THREE.MeshStandardMaterial({ map: carbonTexture(), color: 0xffffff, roughness: 0.4, metalness: 0.3 });
  const smoke = new THREE.MeshPhysicalMaterial({ color: 0x2c3036, transparent: true, opacity: 0.58, roughness: 0.2, depthWrite: false, side: THREE.DoubleSide });
  const clearSmoke = new THREE.MeshPhysicalMaterial({ color: 0x59606a, transparent: true, opacity: 0.22, roughness: 0.15, depthWrite: false, side: THREE.DoubleSide });
  const bolts = boltSet(root, std(0xc4c8ce, 0.3, 0.9));
  const up = new THREE.Vector3(0, 1, 0);

  // ---- hopper walls: smoked polycarbonate, chamfered back corners
  const back = -L / 2 + 0.012, cz = W / 2 - chamfer, cx = -L / 2 + chamfer;
  const wallPts = [[L / 2 - 0.005, -wallZ], [cx, -wallZ], [back, -cz], [back, cz], [cx, wallZ], [L / 2 - 0.005, wallZ]];
  const wallSegs = [];
  for (let i = 0; i < wallPts.length - 1; i++) {
    const [x0, z0] = wallPts[i], [x1, z1] = wallPts[i + 1];
    const len = Math.hypot(x1 - x0, z1 - z0), ry = -Math.atan2(z1 - z0, x1 - x0);
    const p = mesh(new THREE.PlaneGeometry(len, wallTop - hY), smoke, root, (x0 + x1) / 2, (hY + wallTop) / 2, (z0 + z1) / 2);
    p.rotation.y = ry;
    p.castShadow = false;
    // outward normal (the polygon runs clockwise seen from above)
    const n = new THREE.Vector3(-(z1 - z0) / len, 0, (x1 - x0) / len);
    if (n.x * (x0 + x1) + n.z * (z0 + z1) < 0) n.negate();
    wallSegs.push({ x0, z0, x1, z1, len, ry, n });
    // rivets along the top and bottom edges
    for (const y of [wallTop - 0.012, hY + 0.012]) {
      const k = Math.max(2, Math.round(len / 0.07));
      for (let j = 0; j <= k; j++) {
        const t = (j + 0.5) / (k + 1);
        bolts.add(new THREE.Vector3(x0 + (x1 - x0) * t + n.x * 0.001, y, z0 + (z1 - z0) * t + n.z * 0.001), n);
      }
    }
    // thin bent lip along the top edge
    const lip = bx(len, 0.004, 0.018, blackAl, root, (x0 + x1) / 2 - n.x * 0.009, wallTop - 0.002, (z0 + z1) / 2 - n.z * 0.009);
    lip.rotation.y = ry;
  }
  // the teal truss: an angle along the top of the back and chamfer walls (flange on top, web on
  // the wall), and truss columns down the four back corners
  for (const sgm of wallSegs.slice(1, 4)) {
    const { x0, z0, x1, z1, len, ry, n } = sgm;
    const g = new THREE.Group();
    g.position.set(x0, 0, z0);
    g.rotation.y = ry;
    root.add(g);
    // which way is out in the group's frame
    const zs = new THREE.Vector3(0, 0, 1).applyAxisAngle(up, ry).dot(n) > 0 ? 1 : -1;
    trussStrip(len, 0.05, 0.004, teal, g).position.set(0, wallTop - 0.05, zs * 0.004);
    const fl = trussStrip(len, 0.042, 0.004, teal, g);
    fl.rotation.x = zs > 0 ? -Math.PI / 2 : Math.PI / 2;
    fl.position.set(0, wallTop + 0.002, zs * 0.004);
    const k = Math.round(len / 0.06);
    for (let j = 1; j < k; j++) {
      const t = j / k;
      bolts.add(new THREE.Vector3(x0 + (x1 - x0) * t + n.x * 0.007, wallTop - 0.025, z0 + (z1 - z0) * t + n.z * 0.007), n);
    }
  }
  for (const i of [1, 2, 3, 4]) {
    const [x, z] = wallPts[i];
    const n = wallSegs[i - 1].n.clone().add(wallSegs[i].n).normalize();
    const g = new THREE.Group();
    g.position.set(x + n.x * 0.007, hY - 0.01, z + n.z * 0.007);
    g.rotation.y = Math.atan2(n.x, n.z);
    root.add(g);
    const c = trussStrip(wallTop - hY + 0.01, 0.045, 0.004, teal, g);
    c.rotation.z = Math.PI / 2;
    c.position.x = 0.0225;
  }

  // ---- top plate: pocketed, from the back truss forward to the turret bearing (a keyhole)
  const tx = cfg.shooter.turretPos.x, tz = cfg.shooter.turretPos.z;
  const xs = -0.165, ro = 0.2, riT = 0.158;
  {
    const aJ = 2.3; // where the ring meets the strip
    const outline = [
      [back, -cz], [cx, -wallZ], [xs, -wallZ], [xs, -0.2],
      ...arcPts(tx, tz, ro, aJ, -aJ, 40),
      [xs, 0.2], [xs, wallZ], [cx, wallZ], [back, cz],
    ];
    const holes = [arcPts(tx, tz, riT, 0, Math.PI * 2, 48)];
    // truss pockets across the back strip
    const px0 = back + 0.018, px1 = xs - 0.016;
    const zs = [];
    for (let z = -wallZ + 0.03; z < wallZ - 0.03; z += 0.062) zs.push(z);
    zs.forEach((z, i) => {
      const z1 = z + 0.052;
      if (Math.abs(z + 0.026) < 0.2 + 0.03) {
        // the neck: pockets between the strip and the ring
        return;
      }
      const inChamfer = (x, zz) => (x - (-L / 2)) + (W / 2 - Math.abs(zz)) < chamfer + 0.03;
      const tri = i % 2 ? [[px0, z], [px0, z1], [px1, (z + z1) / 2]] : [[px1, z], [px1, z1], [px0, (z + z1) / 2]];
      if (tri.some(([x, zz]) => inChamfer(x, zz))) return;
      holes.push(tri);
    });
    for (const s of [-1, 1]) holes.push([[-0.225, s * 0.035], [-0.225, s * 0.17], [-0.285, s * 0.1]]);
    flatPart(outline, holes, 0.005, blackAl, root, wallTop);
    for (let i = 0; i < 24; i++) {
      const a = (i / 24) * Math.PI * 2;
      bolts.add(new THREE.Vector3(tx + Math.cos(a) * 0.18, wallTop, tz - Math.sin(a) * 0.18), up);
    }
  }

  // ---- intake box: slides out on its racks (at the start of the match) and stays out
  const ext = new THREE.Group();
  root.add(ext);
  const xF = L / 2 + BUMPER_T + cfg.intake.reach - extLen; // front face, stowed
  // the box's side panels and racks run outboard of the hopper walls and the pinion strips
  const sideZ = W / 2 + 0.02, rackZ = W / 2 + 0.012, boxBack = xF - 0.33, bw = sideZ + 0.002;
  const chinY = 0.245, rollerY = 0.165;
  {
    // front panel (SRPP), with a bent chin at the bottom over the roller
    mesh(new THREE.BoxGeometry(0.004, wallTop - chinY, 2 * bw), carbon, ext, xF - 0.006, (wallTop + chinY) / 2, 0);
    const chin = bx(0.004, 0.05, 2 * bw - 0.02, carbon, ext, xF - 0.022, chinY - 0.021, 0);
    chin.rotation.z = -0.72;
    bx(0.006, 0.005, 2 * bw - 0.01, M.alu, ext, xF - 0.006, chinY, 0); // bend line
    for (let j = 0; j < 14; j++) for (const y of [wallTop - 0.012, chinY + 0.014]) bolts.add(new THREE.Vector3(xF - 0.003, y, -bw + 0.03 + j * (2 * bw - 0.06) / 13), new THREE.Vector3(1, 0, 0));
    // side panels: tall, cut away at the bottom to clear the bumper
    const sp = new THREE.Shape();
    sp.moveTo(0, 0.17); sp.lineTo(0.12, 0.17);
    sp.quadraticCurveTo(0.2, 0.17, 0.235, 0.06);
    sp.lineTo(0.33, 0.035); sp.lineTo(0.33, wallTop); sp.lineTo(0, wallTop); sp.closePath();
    const spGeo = new THREE.ExtrudeGeometry(sp, { depth: 0.004, bevelEnabled: false });
    for (const s of [-1, 1]) {
      mesh(spGeo, smoke, ext, boxBack, 0, s * sideZ - 0.002).castShadow = false;
      // bent aluminum impact guard wrapping the front corner
      bx(0.004, wallTop - 0.07, 0.06, greyAl, ext, xF + 0.001, (wallTop + 0.07) / 2, s * (bw - 0.03));
      bx(0.07, wallTop - 0.07, 0.004, greyAl, ext, xF - 0.034, (wallTop + 0.07) / 2, s * (bw + 0.001));
      for (let j = 0; j < 7; j++) {
        bolts.add(new THREE.Vector3(xF + 0.003, 0.09 + j * 0.045, s * (bw - 0.03)), new THREE.Vector3(1, 0, 0));
        bolts.add(new THREE.Vector3(xF - 0.034, 0.09 + j * 0.045, s * (bw + 0.003)), new THREE.Vector3(0, 0, s));
      }
      // rack along the bottom of the box (it runs back through the pinion plate)
      bx(0.52, 0.012, 0.01, blackAl, ext, xF - 0.3, 0.182, s * rackZ);
      const teeth = new THREE.InstancedMesh(new THREE.BoxGeometry(0.006, 0.008, 0.012), blackAl, 60);
      const m4 = new THREE.Matrix4();
      for (let j = 0; j < 60; j++) { m4.makeTranslation(xF - 0.555 + j * 0.0085, 0.192, s * rackZ); teeth.setMatrixAt(j, m4); }
      ext.add(teeth);
      // roller drive: Kraken X44 on the side panel, belt down to the roller
      kraken(ext, xF - 0.07, 0.3, s * (sideZ - 0.04), true, 'z');
      bx(0.012, 0.15, 0.006, M.polyBelt, ext, xF - 0.05, 0.23, s * (sideZ - 0.022)).rotation.z = 0.25;
    }
  }
  // under-roller at the front lip, white with big printed flanged hubs
  const intakeRoller = new THREE.Group();
  intakeRoller.position.set(xF - 0.035, rollerY, 0);
  ext.add(intakeRoller);
  cylZ(0.03, 2 * sideZ - 0.03, std(0xe6e8ea, 0.45, 0.05), intakeRoller, 0, 0, 0, 24);
  for (const s of [-1, 1]) for (const zz of [0.06, 0.3]) cylZ(0.036, 0.012, plastic, intakeRoller, 0, 0, s * (sideZ - zz), 18);
  for (let i = 0; i < 6; i++) bx(0.004, 0.061, 2 * sideZ - 0.08, std(0xd0d3d7, 0.5, 0.05), intakeRoller, 0, 0, 0).rotation.z = (i * Math.PI) / 6; // ribs
  // hinged ramp of passive rollers from the roller up to the bumper (it swings down as the box
  // comes out and lies on the bumper when it's out)
  const ramp = new THREE.Group();
  ramp.position.set(xF - 0.06, 0.02, 0);
  ext.add(ramp);
  const ridge = bay.ramp; // top of the ramp, just over the front bumper
  const rampLen = Math.hypot(xF + extLen - 0.06 - ridge.x, ridge.y - 0.02);
  for (const s of [-1, 1]) bx(rampLen, 0.02, 0.008, blackAl, ramp, -rampLen / 2, 0, s * (wallZ - 0.02));
  const rampRollers = [];
  for (let i = 0; i < 6; i++) rampRollers.push(cylZ(0.011, 2 * wallZ - 0.05, std(0x2e3137, 0.55, 0.1), ramp, -0.02 - i * (rampLen - 0.03) / 5, 0.006, 0, 10));
  cylZ(0.004, 2 * wallZ - 0.02, M.alu, ramp, 0, 0, 0, 6);
  // star roller (hook shaft) over the bumper that drives FUEL up and over it
  const star = new THREE.Group();
  star.position.set(L / 2, 0.345, 0);
  root.add(star);
  cylZ(0.006, 2 * wallZ - 0.01, M.alu, star, 0, 0, 0, 8);
  for (let i = 0; i < 22; i++) {
    const z = -wallZ + 0.04 + i * (2 * wallZ - 0.08) / 21;
    const w = cylZ(0.018, 0.012, M.compliant, star, 0, 0, z, 6);
    w.rotation.y = i * 0.5;
  }
  // pinion strips (SRPP) on the outside of the side walls: the rack's pinion, driven by a Kraken
  // X44 inside the wall
  const stripX0 = 0.085, stripX1 = 0.195;
  for (const s of [-1, 1]) {
    const pp = new THREE.Shape();
    pp.moveTo(0, 0); pp.lineTo(stripX1 - stripX0, 0); pp.lineTo(stripX1 - stripX0, wallTop - hY);
    pp.lineTo(0, wallTop - hY); pp.lineTo(0, 0.04); pp.quadraticCurveTo(0, 0, 0.04, 0);
    const h = new THREE.Path(); h.absarc(0.055, 0.058, 0.01, 0, Math.PI * 2, true); pp.holes.push(h);
    mesh(new THREE.ExtrudeGeometry(pp, { depth: 0.005, bevelEnabled: false }), carbon, root, stripX0, hY, s * (W / 2 + 0.0035) - 0.0025);
    for (let j = 0; j < 8; j++) for (const x of [stripX0 + 0.012, stripX1 - 0.012]) bolts.add(new THREE.Vector3(x, hY + 0.05 + j * 0.03, s * (W / 2 + 0.006)), new THREE.Vector3(0, 0, s));
    const pin = spurGear(0.018, 12, 0.01, M.copper, root);
    pin.rotation.x = Math.PI / 2;
    pin.position.set(stripX0 + 0.055, 0.206, s * (rackZ + 0.005));
    kraken(root, stripX0 + 0.055, 0.206, s * (W / 2 - 0.04), true, 'z');
  }
  for (const s of [-1, 1]) bx(0.03, 0.04, 0.012, blackAl, root, L / 2, 0.345, s * (wallZ - 0.006)); // star roller bearing blocks

  // ---- Dye Rotor
  const rotor = new THREE.Group();
  rotor.position.set(rs.x, rs.y, rs.z);
  root.add(rotor);
  // the platter: pocketed spokes, a center hole round the column, skinned with thin smoked poly
  {
    const s = new THREE.Shape();
    s.absarc(0, 0, rs.r, 0, Math.PI * 2, false);
    const hub = new THREE.Path(); hub.absarc(0, 0, 0.078, 0, Math.PI * 2, true); s.holes.push(hub);
    const n = 12;
    for (let i = 0; i < n; i++) {
      for (const [r0, r1] of [[0.095, 0.17], [0.183, rs.r - 0.03]]) {
        const gap = 0.008;
        const a0 = (i / n) * Math.PI * 2 + gap / r0 + (r0 > 0.15 ? Math.PI / n : 0), a1 = a0 + (Math.PI * 2) / n - 2 * gap / r0;
        const h = new THREE.Path();
        h.absarc(0, 0, r1, a0 + gap / r1 - gap / r0, a1 - gap / r1 + gap / r0, false);
        h.absarc(0, 0, r0, a1, a0, true);
        s.holes.push(h);
      }
    }
    const g = new THREE.ExtrudeGeometry(s, { depth: 0.006, bevelEnabled: false, curveSegments: 10 });
    g.rotateX(-Math.PI / 2);
    g.translate(0, -0.006, 0);
    mesh(g, darkPlate, rotor);
    const skin = mesh(new THREE.RingGeometry(0.08, rs.r - 0.004, 64, 1), clearSmoke, rotor, 0, 0.0015, 0);
    skin.rotation.x = -Math.PI / 2;
    skin.castShadow = false;
    // rim band and bolts
    mesh(new THREE.CylinderGeometry(rs.r, rs.r, 0.012, 72, 1, true), blackAl, rotor, 0, 0.0, 0);
    const rb = boltSet(rotor, std(0x9ea3aa, 0.35, 0.8), 0.0045);
    for (let i = 0; i < 36; i++) { const a = (i / 36) * Math.PI * 2; rb.add(new THREE.Vector3(Math.cos(a) * (rs.r - 0.015), 0.002, Math.sin(a) * (rs.r - 0.015)), up); }
    for (let i = 0; i < 12; i++) { const a = (i / 12) * Math.PI * 2; rb.add(new THREE.Vector3(Math.cos(a) * 0.088, 0.002, Math.sin(a) * 0.088), up); }
    rb.done();
    // big spur gear under the center (36.4:1 from the motors)
    spurGear(0.12, 72, 0.008, blackAl, rotor, 8).position.y = -0.008;
    // the Dolphin Fin, on the rim
    mesh(dolphinFinGeometry(0.175, rs.r - 0.004, 0.95, 0.085), std(0x7a818b, 0.45, 0.3, { side: THREE.DoubleSide }), rotor, 0, 0.001, 0);
  }
  // base plate and the fixed ring wall round the rotor, on standoffs
  flatPart(arcPts(rs.x, rs.z, rs.r + 0.03, 0, Math.PI * 2, 48), [], 0.005, darkPlate, root, rs.y - 0.03);
  const ringWall = mesh(new THREE.CylinderGeometry(rs.r + 0.012, rs.r + 0.012, 0.03, 72, 1, true), smoke, root, rs.x, rs.y + 0.005, rs.z);
  ringWall.castShadow = false;
  for (let i = 0; i < 16; i++) {
    const a = (i / 16) * Math.PI * 2;
    cylY(0.004, 0.06, blackAl, root, rs.x + Math.cos(a) * (rs.r + 0.02), rs.y - 0.01, rs.z + Math.sin(a) * (rs.r + 0.02), 6);
  }
  // rotor motors (Kraken X60s) standing at the back corner, belted to the gear train
  for (const [x, z] of [[-0.235, 0.255], [-0.175, 0.305], [-0.115, 0.335]]) {
    kraken(root, x, 0.13, z);
    bx(0.03, 0.006, 0.02, M.led, root, x + 0.02, 0.166, z);
  }
  // printed "stadium" pieces: terraces that funnel FUEL down onto the rotor
  {
    const NX = 64, NZ = 76, x0 = bay.x0, x1 = bay.x1, z0 = -bay.hw, z1 = bay.hw;
    const f = bay.funnel, step = 0.018;
    const hAt = (x, z) => {
      const d = Math.hypot(x - rs.x, z - rs.z) - rs.r - 0.012;
      if (d < 0) return -1;
      const h = Math.min(f.cap, d * f.slope);
      const k = Math.floor(h / step), fr = h / step - k;
      return rs.y - 0.005 + (k + THREE.MathUtils.smoothstep(fr, 0.75, 1)) * step;
    };
    const inside = (x, z) => (x - x0) - Math.abs(z) + bay.hw - chamfer >= -0.005;
    const pos = [], idx = [], map = new Map();
    const vid = (i, j) => {
      const key = i * 1000 + j;
      if (map.has(key)) return map.get(key);
      const x = x0 + ((x1 - x0) * i) / NX, z = z0 + ((z1 - z0) * j) / NZ;
      pos.push(x, Math.max(rs.y - 0.005, hAt(x, z)), z);
      map.set(key, pos.length / 3 - 1);
      return pos.length / 3 - 1;
    };
    for (let i = 0; i < NX; i++) {
      for (let j = 0; j < NZ; j++) {
        const xa = x0 + ((x1 - x0) * (i + 0.5)) / NX, za = z0 + ((z1 - z0) * (j + 0.5)) / NZ;
        if (hAt(xa, za) < 0 || !inside(xa, za)) continue;
        const a = vid(i, j), b = vid(i + 1, j), c = vid(i + 1, j + 1), d = vid(i, j + 1);
        idx.push(a, d, c, a, c, b);
      }
    }
    const g = new THREE.BufferGeometry();
    g.setAttribute('position', new THREE.Float32BufferAttribute(pos, 3));
    g.setIndex(idx);
    g.computeVertexNormals();
    mesh(g, std(0x40454d, 0.75, 0.05), root);
  }
  // center column: a cage of four legs under the turret bearing, wrapped low down in the
  // "shrink wrap" sheet so FUEL slides off it onto the rotor
  const colTop = wallTop - 0.012;
  flatPart(arcPts(col.x, col.z, col.r + 0.02, 0, Math.PI * 2, 40), [arcPts(col.x, col.z, col.r - 0.005, 0, Math.PI * 2, 40)], 0.008, blackAl, root, colTop);
  for (let i = 0; i < 4; i++) {
    const a = Math.PI / 4 + (i * Math.PI) / 2;
    const top = new THREE.Vector3(col.x + Math.cos(a) * (col.r + 0.005), colTop - 0.008, col.z + Math.sin(a) * (col.r + 0.005));
    const bot = new THREE.Vector3(col.x + Math.cos(a) * 0.062, rs.y - 0.03, col.z + Math.sin(a) * 0.062);
    const leg = rod(root, top, bot, 0.012, blackAl, 6);
    leg.scale.set(1, 1, 0.5);
  }
  const wrap = mesh(new THREE.CylinderGeometry(col.r + 0.005, col.r + 0.075, 0.2, 40, 1, true, 0.75, Math.PI * 2 - 1.5), clearSmoke, root, col.x, rs.y + 0.105, col.z);
  wrap.castShadow = false;
  // the hook: a fixed curved guide of passive rollers over the rotor, from the rim in to the
  // feeder (FUEL riding round on the rotor runs into it and slides in along it)
  const feed = bay.feed, hk = bay.hook;
  const hookPts = hookPath(bay, 20);
  const hookRollers = [];
  for (const y of [rs.y + hk.y0, rs.y + hk.y1]) flatRibbon(hookPts, 2 * hk.r, y, blackAl, root);
  for (let i = 1; i < hookPts.length; i += 2) {
    const [x, z] = hookPts[i];
    const pr = cylY(0.011, hk.y1 - hk.y0 - 0.005, std(0x3b3f45, 0.5, 0.1), root, x, rs.y + (hk.y0 + hk.y1) / 2, z, 10);
    hookRollers.push(pr);
  }
  for (let i = 0; i < hookPts.length; i += 5) {
    const [x, z] = hookPts[i];
    cylY(0.004, hk.y1 - hk.y0, M.alu, root, x, rs.y + (hk.y0 + hk.y1) / 2, z, 6);
  }
  // horizontal 1.25in omni wheels on a shaft along the hook's inner end: FUEL slides onto the ramp
  const omniShaft = new THREE.Group();
  let omniSpin;
  {
    const [ax, az] = hookPts[12], [bx2, bz] = hookPts[20];
    omniShaft.position.set((ax + bx2) / 2, rs.y + 0.21, (az + bz) / 2);
    omniShaft.rotation.y = -Math.atan2(bz - az, bx2 - ax);
    root.add(omniShaft);
    const len = Math.hypot(bx2 - ax, bz - az);
    omniSpin = new THREE.Group();
    omniShaft.add(omniSpin);
    cylX(0.005, len, M.alu, omniSpin, 0, 0, 0, 6);
    for (let i = 0; i < 7; i++) cylX(0.016, 0.012, M.compliant, omniSpin, -len / 2 + (i + 0.5) * len / 7, 0, 0, 8);
  }
  // feeder: vertical 3in omni wheel and 3in TTB urethane wheels on the column side of the ramp,
  // passive-roller backing on the outside; the ramp climbs to the turret
  const fr = new THREE.Vector3(feed.x, rs.y + R_FUEL, feed.z);
  const via = feed.via.map(([x, y, z]) => new THREE.Vector3(x, y, z));
  const path = [fr, ...via];
  const feederWheels = [];
  for (const [i, t, big] of [[0, 0.5, true], [1, 0.3, false], [1, 0.8, false]]) {
    const p = path[i].clone().lerp(path[i + 1], t);
    const toC = new THREE.Vector3(col.x - p.x, 0, col.z - p.z).normalize();
    const c = p.clone().addScaledVector(toC, R_FUEL + 0.03);
    const w = new THREE.Group();
    w.position.copy(c);
    w.rotation.y = Math.atan2(toC.x, toC.z);
    root.add(w);
    const spin = new THREE.Group();
    w.add(spin);
    cylX(0.038, big ? 0.03 : 0.025, big ? M.compliant : std(0x1d6fd0, 0.6, 0.05), spin, 0, 0, 0, 20);
    cylX(0.012, 0.035, M.copper, spin, 0, 0, 0, 10);
    if (big) for (let k = 0; k < 8; k++) { const a = (k * Math.PI) / 4; cylX(0.008, 0.02, M.compliant, spin, 0, Math.cos(a) * 0.036, Math.sin(a) * 0.036, 6).rotation.set(a, 0, 0); }
    feederWheels.push(spin);
  }
  kraken(root, col.x + 0.05, 0.3, col.z + 0.08, true, 'x');
  // backing ramp outside the path
  {
    const pts = [];
    for (let i = 0; i < path.length - 1; i++) for (let k = 0; k < 4; k++) pts.push(path[i].clone().lerp(path[i + 1], k / 4));
    pts.push(path[path.length - 1].clone());
    const bk = [];
    for (const p of pts) {
      const out = new THREE.Vector3(p.x - col.x, 0, p.z - col.z).normalize();
      bk.push(p.clone().addScaledVector(out, R_FUEL + 0.015));
    }
    for (let i = 0; i < bk.length - 1; i++) {
      if (bk[i].y > wallTop + 0.04) break;
      const a = bk[i], b = bk[i + 1];
      const plate = rod(root, a, b, 0.004, blackAl, 4);
      plate.scale.set(12, 1, 1);
      if (i % 2 === 0) {
        const mid = a.clone().add(b).multiplyScalar(0.5);
        const out = new THREE.Vector3(mid.x - col.x, 0, mid.z - col.z).normalize();
        const t = new THREE.Vector3().crossVectors(up, out);
        rod(root, mid.clone().addScaledVector(t, -0.04).addScaledVector(out, -0.012), mid.clone().addScaledVector(t, 0.04).addScaledVector(out, -0.012), 0.009, std(0x3b3f45, 0.5, 0.1), 8);
      }
    }
  }

  // ---- turret: bearing ring in the top plate. The shooter hangs down inside the ring, with only
  // the hood roller at the top of its A-frame above the plate: the 3in flywheel low at the back,
  // the hood over it, and FUEL out past the apex roller toward the front (+x)
  const turret = new THREE.Group();
  turret.position.set(tx, wallTop, tz);
  root.add(turret);
  {
    const ring = flatPart(arcPts(0, 0, 0.19, 0, Math.PI * 2, 48), [arcPts(0, 0, 0.14, 0, Math.PI * 2, 48)], 0.006, darkPlate, turret, 0.008);
    ring.castShadow = true;
    const tb = boltSet(turret, std(0xc4c8ce, 0.3, 0.9));
    for (let i = 0; i < 16; i++) { const a = (i / 16) * Math.PI * 2; tb.add(new THREE.Vector3(Math.cos(a) * 0.165, 0.008, Math.sin(a) * 0.165), up); }
    tb.done();
    // drum down into the column
    mesh(new THREE.CylinderGeometry(0.118, 0.118, 0.16, 36, 1, true), std(0x23262b, 0.5, 0.4, { side: THREE.DoubleSide }), turret, 0, -0.075, 0);
  }
  const body = new THREE.Group();
  body.position.y = -0.11;
  body.rotation.y = Math.PI; // built with the flywheel at +x; it sits at the back of the shot
  turret.add(body);
  // A-frame side plates: a pocketed truss triangle each side
  const aframe = new THREE.Shape();
  aframe.moveTo(-0.15, 0); aframe.lineTo(0.15, 0); aframe.lineTo(0.018, 0.13); aframe.lineTo(-0.018, 0.13); aframe.closePath();
  const tri = (pts) => { const h = new THREE.Path(); h.moveTo(...pts[0]); for (const p of pts.slice(1)) h.lineTo(...p); h.closePath(); aframe.holes.push(h); };
  tri([[-0.122, 0.01], [-0.052, 0.01], [-0.087, 0.045]]);
  tri([[-0.042, 0.012], [-0.075, 0.052], [-0.009, 0.052]]);
  tri([[-0.065, 0.062], [-0.012, 0.062], [-0.02, 0.105]]);
  tri([[0.122, 0.01], [0.052, 0.01], [0.087, 0.045]]);
  tri([[0.042, 0.012], [0.075, 0.052], [0.009, 0.052]]);
  tri([[0.065, 0.062], [0.012, 0.062], [0.02, 0.105]]);
  const afGeo = new THREE.ExtrudeGeometry(aframe, { depth: 0.006, bevelEnabled: false });
  afGeo.translate(0, 0.012, -0.003);
  for (const s of [-1, 1]) mesh(afGeo, blackAl, body, 0, 0, s * 0.078);
  // 3in flywheel low down, hood backing over it, hood roller at the apex
  const fly = wheelStack(body, 0.13, 0.038, 3, M.compliant, 0.075, 0.055, 0, 0.034);
  for (let i = 0; i < 3; i++) cylZ(0.021, 0.036, M.copper, fly, 0, 0, -0.043 + i * 0.043, 14);
  const hood = new THREE.Group();
  hood.position.set(0.075, 0.055, 0);
  body.add(hood);
  const hoodShell = new THREE.Mesh(new THREE.CylinderGeometry(0.075, 0.075, 0.145, 20, 1, true, Math.PI / 2 + 0.5, 1.75), std(0x3a3f47, 0.45, 0.45, { side: THREE.DoubleSide }));
  hoodShell.rotation.x = Math.PI / 2;
  hood.add(hoodShell);
  const hoodRoller = cylZ(0.019, 0.145, M.compliant, body, 0.0, 0.123, 0, 14);
  for (const s of [-1, 1]) {
    // belt from the flywheel up the front leg to the hood roller, with its pulleys
    const a = new THREE.Vector3(0.075, 0.055, s * 0.07), b = new THREE.Vector3(0.0, 0.123, s * 0.07);
    rod(body, a, b, 0.006, M.polyBelt, 4).scale.set(1, 1, 0.35);
    for (const p of [a, b]) cylZ(0.016, 0.01, M.alu, body, p.x, p.y, p.z, 12);
    cylZ(0.008, 0.012, M.copper, body, 0.075, 0.055, s * 0.088, 8); // bearing
  }
  // motors stacked vertically under the ring, inside the drum
  kraken(body, -0.06, -0.06, 0.035, false);
  kraken(body, -0.06, -0.06, -0.035, true);
  limelight(turret, 0.11, 0.036, 0.11, 0); // on the ring, looking where it shoots
  // cable chain to the turret
  const chain = mesh(new THREE.TorusGeometry(0.17, 0.008, 4, 24, Math.PI * 0.8), std(0x111214, 0.8, 0.05), turret, 0, 0.018, 0);
  chain.rotation.x = Math.PI / 2;
  chain.rotation.z = Math.PI * 0.6;

  // ---- net over the top: pinned to the wall tops, the top plate's front edge and the turret
  // ring. It drapes over the load and stretches up where the FUEL bulges it (the hopper physics
  // lets the load bulge it as far as bay.dome allows)
  const net = hopperNet(root, {
    x0: bay.dome.x0 - 0.02, x1: xF - 0.006, x1Out: xF - 0.006 + extLen, xFixed: L / 2,
    hw: wallZ, y: wallTop + 0.004, hole: { x: tx, z: tz, r: bay.dome.rHole - 0.005 },
  });
  bolts.done();

  const anim = (st, dt) => {
    // the intake box slides out on its racks (and stays out)
    const ex = st.hopperDeploy * extLen;
    ext.position.x = ex;
    intakeRoller.rotation.z += st.intakeSpeed * dt * 45; // pulls FUEL in under the chin
    star.rotation.z += st.intakeSpeed * dt * 35;
    for (const r of rampRollers) r.rotation.y += st.intakeSpeed * dt * 20;
    // the ramp hangs from the roller end and lies back on the bumper once the box is out
    const dxr = xF - 0.06 + ex - ridge.x;
    ramp.rotation.z = -Math.min(Math.PI / 2 - 0.06, Math.atan2(ridge.y - 0.02, Math.max(1e-3, dxr)));
    // the Dye Rotor turns (its angle comes from the hopper physics, so it and the FUEL agree)
    rotor.rotation.y = st.rotorAngle ?? 0;
    for (const r of hookRollers) r.rotation.y += st.feeding * dt * 25;
    omniSpin.rotation.x += st.feeding * dt * 30;
    for (const w of feederWheels) w.rotation.x -= st.feeding * dt * 40;
    turret.rotation.y = st.turretYaw;
    fly.rotation.z -= st.flywheel * dt * 10;
    hoodRoller.rotation.y -= st.flywheel * dt * 5;
    hood.rotation.z = (st.hoodDeg - 62) * Math.PI / 180 * 0.5;
    net.drape(st.load, ex);
  };
  return { root, anim, stored: [], modules, extLen };
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

// ============================================================ 971 Mixtape
// From 971's public Onshape CAD ("2026 971 Robot Mixtape Public Release"), split into parts by
// tools/extract-parts.mjs (cad/robots/971.json): the body (drivetrain, roller floor, separator,
// kicker, ramps, turret platform, fixed hopper walls), the sliding front of the hopper, the 4-bar
// ground intake (stowed folded up inside the hopper, as exported) and a turret (drawn twice). A
// net covers the open top of the hopper.
const INTAKE_971 = { pivot: [0.217, 0.143], swing: -2.227 }; // beater from folded up to on the carpet
function build971(cfg, alliance) {
  const root = new THREE.Group();
  const L = cfg.frame.length, W = cfg.frame.width, bay = cfg.bay, sh = cfg.shooter;
  const extLen = cfg.storage.extLen;
  addBumpers(root, cfg, alliance);
  const modules = addDrivebase(root, cfg, 0, false);
  const blackAl = std(0x2a2e35, 0.42, 0.55);
  const top = bay.top, hw = bay.hw + 0.005;
  // drawn stand-ins until the CAD arrives
  const drawn = new THREE.Group();
  root.add(drawn);
  addDrivebase(drawn, cfg, 0, true).forEach((md) => md.pivot.parent.removeFromParent());
  for (const s of [-1, 1]) polyWall(drawn, 0.27, top - 0.05, 0.15, 0.05 + (top - 0.05) / 2, s * hw);
  polyWall(drawn, 2 * hw, top - 0.17, 0.03, 0.17 + (top - 0.17) / 2, 0, Math.PI / 2);
  const floorRollers = new THREE.Group();
  drawn.add(floorRollers);
  for (let x = 0.04; x < 0.3; x += 0.055) cylZ(0.016, 2 * hw - 0.02, M.alu, floorRollers, x, bay.floor.a + bay.floor.b * x - 0.018, 0, 10);
  rbx(0.32, 0.008, W - 0.02, 0.003, M.alu, drawn, -0.15, 0.28, 0); // turret platform
  // the hopper's sliding front: telescoping side plates and the front wall
  const slide = new THREE.Group();
  root.add(slide);
  const slideDrawn = new THREE.Group();
  slide.add(slideDrawn);
  const xFront = 0.606; // front wall, hopper out (as exported)
  for (const s of [-1, 1]) polyWall(slideDrawn, 0.47, top - 0.12, xFront - 0.235, 0.12 + (top - 0.12) / 2, s * (hw + 0.012));
  polyWall(slideDrawn, 2 * hw + 0.03, top - 0.12, xFront, 0.12 + (top - 0.12) / 2, 0, Math.PI / 2);
  // 4-bar ground intake
  const intake = new THREE.Group();
  intake.position.set(INTAKE_971.pivot[0], INTAKE_971.pivot[1], 0);
  root.add(intake);
  const intakeDrawn = new THREE.Group();
  intake.add(intakeDrawn);
  const beater = cylZ(0.028, cfg.intake.width, std(0x2b2b2e, 0.8, 0.1), intakeDrawn, -0.145, 0.27, 0, 14);
  for (const s of [-1, 1]) rod(intakeDrawn, new THREE.Vector3(0, 0, s * (cfg.intake.width / 2 + 0.01)), new THREE.Vector3(-0.145, 0.27, s * (cfg.intake.width / 2 + 0.01)), 0.012, blackAl);
  // two turrets
  const turrets = sh.turrets.map((t) => {
    const g = new THREE.Group();
    g.position.set(t.x, 0.327, t.z); // the bearings on the platform (the CAD floor is 0.043 m under its origin)
    root.add(g);
    const d = new THREE.Group();
    g.add(d);
    mesh(new THREE.TorusGeometry(0.13, 0.012, 8, 40), M.alu, d, 0, 0.005, 0).rotation.x = Math.PI / 2;
    for (const s of [-1, 1]) pocketPlate(d, 0.24, 0.2, 0.006, blackAl, [[-0.05, 0, 0.03], [0.05, 0.02, 0.03]], -0.02, 0.11, s * 0.075);
    wheelStack(d, 0.1, 0.05, 2, M.black, -0.06, 0.15, 0, 0.045);
    cadPart(g, 'robots/971-turret.glb', () => { d.visible = false; });
    return g;
  });
  cadPart(root, 'robots/971-body.glb', () => { drawn.visible = false; });
  cadPart(slide, 'robots/971-hopper.glb', () => { slideDrawn.visible = false; });
  cadPart(intake, 'robots/971-intake.glb', () => { intakeDrawn.visible = false; });
  // net over the open top, from the back wall to the sliding front wall
  const net = hopperNet(root, { x0: 0.035, x1: xFront - extLen - 0.01, x1Out: xFront - 0.01, xFixed: 0.27, hw, y: top + 0.004 });

  const anim = (st, dt) => {
    const ex = st.hopperDeploy * extLen;
    slide.position.x = ex - extLen; // exported with the hopper out
    intake.rotation.z = st.intakeDeploy * INTAKE_971.swing;
    beater.rotation.y += st.intakeSpeed * dt * 40;
    for (const r of floorRollers.children) r.rotation.y -= st.feeding * dt * 30;
    // the CAD's shooter fires out its back (over the flywheel, away from the hood): at yaw pi
    turrets.forEach((g, i) => { g.rotation.y = (st.turretYaws ? st.turretYaws[i] : st.turretYaw) - Math.PI; });
    net.drape(st.load, ex);
  };
  return { root, anim, stored: [], modules, extLen };
}

// ============================================================ 1678 Limestone
// From 1678's public Onshape CAD ("1678-26c-0000 CAD Release"), split by tools/extract-parts.mjs
// (cad/robots/1678.json): the body (drivetrain with its polycarbonate walls on the bumper mounts,
// roller floor, ball tunnel and drum shooter), the slapdown intake (exported folded up), the
// polycarbonate side plates of the horizontal extension (they slide out with the intake), and the
// climber with the corrugated lid, which lifts to make the hopper taller (its side skirts ride over
// the walls). A net stretches diagonally from the lid's front edge down to the extension's front.
const INTAKE_1678 = { pivot: [0.254, 0.194], swing: -2.905 };
function build1678(cfg, alliance) {
  const root = new THREE.Group();
  const L = cfg.frame.length, W = cfg.frame.width, bay = cfg.bay, sh = cfg.shooter;
  const extLen = cfg.storage.extLen;
  addBumpers(root, cfg, alliance);
  const modules = addDrivebase(root, cfg, 0, false);
  const blackAl = std(0x23262b, 0.42, 0.55);
  const lime = std(cfg.colors.accent, 0.5, 0.2);
  const drawn = new THREE.Group();
  root.add(drawn);
  addDrivebase(drawn, cfg, 0, true).forEach((md) => md.pivot.parent.removeFromParent());
  const wallTop = 0.495, hw = W / 2 - 0.01;
  for (const s of [-1, 1]) polyWall(drawn, L - 0.02, wallTop - 0.15, 0, 0.15 + (wallTop - 0.15) / 2, s * hw);
  // ball tunnel and drum across the back
  for (const s of [-1, 1]) pocketPlate(drawn, 0.22, 0.42, 0.006, blackAl, [[0, 0.05, 0.05], [0, -0.1, 0.04]], -0.22, 0.3, s * 0.318);
  const drum = wheelStack(drawn, 0.6, 0.045, 1, M.black, -0.288, 0.477, 0, 0.58);
  for (let i = 0; i < 3; i++) cylZ(0.022, 0.6, M.black, drawn, -0.09 - i * 0.012, 0.43 + i * 0.035, 0, 12);
  for (let x = -0.05; x < 0.32; x += 0.06) cylZ(0.013, 0.58, M.black, drawn, x, bay.floor.a + bay.floor.b * x - 0.02, 0, 10);
  // horizontal extension: polycarbonate side plates that slide out with the intake
  const slide = new THREE.Group();
  root.add(slide);
  // lid on the climber
  const lift = new THREE.Group();
  root.add(lift);
  const lidDrawn = rbx(0.38, 0.01, 2 * hw, 0.003, std(0xe8e8e2, 0.8, 0), lift, 0.15, 0.54, 0);
  for (const s of [-1, 1]) rbx(0.02, 0.45, 0.02, 0.003, M.alu, lift, -0.02, 0.32, s * (hw - 0.015));
  // slapdown intake
  const intake = new THREE.Group();
  intake.position.set(INTAKE_1678.pivot[0], INTAKE_1678.pivot[1], 0);
  root.add(intake);
  const intakeDrawn = new THREE.Group();
  intake.add(intakeDrawn);
  const roller = cylZ(0.028, cfg.intake.width, lime, intakeDrawn, -0.156, 0.207, 0, 14);
  for (const s of [-1, 1]) rod(intakeDrawn, new THREE.Vector3(0, 0, s * 0.33), new THREE.Vector3(-0.156, 0.207, s * 0.33), 0.012, blackAl);
  cadPart(root, 'robots/1678-body.glb', () => { drawn.visible = false; });
  cadPart(slide, 'robots/1678-slide.glb', () => {});
  cadPart(lift, 'robots/1678-lift.glb', () => { lidDrawn.visible = false; });
  cadPart(intake, 'robots/1678-intake.glb', () => { intakeDrawn.visible = false; });
  // a net stretched diagonally from the front edge of the lid down to the front of the
  // extension (it rides out with it, and up with the lid)
  const lidX1 = bay.slope.x, H = bay.lift.h, lidY0 = 0.54;
  const net = hopperNet(root, { x0: lidX1, x1: lidX1 + 0.01, x1Out: lidX1 + extLen, xFixed: lidX1, hw, y: lidY0, yFront: bay.slope.y, NX: 12, NZ: 24 });

  const anim = (st, dt) => {
    slide.position.x = st.hopperDeploy * extLen;
    intake.rotation.z = st.intakeDeploy * INTAKE_1678.swing;
    roller.rotation.y += st.intakeSpeed * dt * 40;
    drum.rotation.z += st.flywheel * dt * 8;
    // the lid rides up on the climber tubes (and all the way up to reach the rung for a climb)
    const up = st.climbState && st.climbState !== 'none' ? 0.26 : (st.lift ?? 0) * H;
    lift.position.y = up;
    const ex = st.hopperDeploy * extLen;
    net.mesh.visible = ex > 0.02;
    net.drape(st.load, Math.max(0, ex - 0.01), { y: lidY0 + up - 0.01 });
  };
  return { root, anim, stored: [], modules, extLen };
}

// ============================================================ 1690 Kepler
// Orbit's CAD blocks export, so this is drawn from Onshape's own renders of it (scaled
// orthographic views), their X_T release and photos: a black lattice-sided hopper over a powered
// roller floor, an over-bumper intake held down by surgical tubing, an electronics box at the back
// right, and at the back left a vertical kicker of green compliant wheels feeding a compact turret
// (A-frame lattice side plates, green flywheel wheels under a hood) on an 8in bearing ringed with
// printed guide fins. The hopper expands forward and netting over its top keeps FUEL from
// bouncing out.
function build1690(cfg, alliance) {
  const root = new THREE.Group();
  const L = cfg.frame.length, W = cfg.frame.width, bay = cfg.bay, sh = cfg.shooter;
  const extLen = cfg.storage.extLen;
  addBumpers(root, cfg, alliance);
  const modules = addDrivebase(root, cfg);
  const lattice = std(cfg.colors.frame, 0.45, 0.55);
  const blue = std(cfg.colors.accent, 0.45, 0.3);
  const green = std(0x62c94a, 0.7, 0.05);
  const fin = std(0x9fd3ea, 0.5, 0.1);
  const bolts = boltSet(root, std(0xc4c8ce, 0.3, 0.9));
  const top = bay.top, hw = bay.hw + 0.008, hY = 0.14;
  const t = sh.turretPos;

  // lattice plate: an outline with X-braced triangular pockets, in the XY plane (thickness z)
  const latticePlate = (parent, w, h, cells, x, y, z, ry = 0) => {
    const g = new THREE.Group();
    g.position.set(x, y, z);
    g.rotation.y = ry;
    parent.add(g);
    const s = new THREE.Shape();
    s.moveTo(-w / 2, -h / 2); s.lineTo(w / 2, -h / 2); s.lineTo(w / 2, h / 2); s.lineTo(-w / 2, h / 2); s.closePath();
    const cw = w / cells, b = 0.014;
    for (let i = 0; i < cells; i++) {
      const xa = -w / 2 + i * cw + b, xb = xa + cw - 2 * b, ya = -h / 2 + b, yb = h / 2 - b, xm = (xa + xb) / 2, ym = (ya + yb) / 2;
      const d = b * 0.7;
      // four triangles between the X's arms
      for (const tri of [[[xa, ya + d], [xa, yb - d], [xm - d, ym]], [[xb, ya + d], [xb, yb - d], [xm + d, ym]], [[xa + d, ya], [xb - d, ya], [xm, ym - d]], [[xa + d, yb], [xb - d, yb], [xm, ym + d]]]) {
        const p = new THREE.Path();
        p.moveTo(...tri[0]); p.lineTo(...tri[1]); p.lineTo(...tri[2]); p.closePath();
        s.holes.push(p);
      }
    }
    const geo = new THREE.ExtrudeGeometry(s, { depth: 0.005, bevelEnabled: false });
    geo.translate(0, 0, -0.0025);
    mesh(geo, lattice, g);
    return g;
  };

  // ---- hopper: lattice side walls over clear polycarbonate, from the back wall to the front
  const xB = bay.x0, xF = bay.x1;
  for (const s of [-1, 1]) {
    latticePlate(root, xF - xB, top - hY, 3, (xB + xF) / 2, (hY + top) / 2, s * (hw + 0.004));
    polyWall(root, xF - xB, top - hY, (xB + xF) / 2, (hY + top) / 2, s * (hw - 0.002), 0, M.poly, lattice);
  }
  // back wall behind the right half (the electronics box is behind it); the kicker takes FUEL
  // in at the left
  polyWall(root, hw + 0.05, top - hY, xB, (hY + top) / 2, hw / 2 - 0.02, Math.PI / 2, M.poly, lattice);
  // powered roller floor
  const floorRollers = [];
  for (let x = xB + 0.03; x < xF - 0.01; x += 0.047) floorRollers.push(cylZ(0.021, 2 * hw - 0.03, std(0x2a2c30, 0.6, 0.3), root, x, bay.floor.a + bay.floor.b * x - 0.022, 0, 12));
  for (const s of [-1, 1]) pocketPlate(root, xF - xB, 0.07, 0.006, lattice, [], (xB + xF) / 2, 0.12, s * (hw - 0.012));

  // ---- expanding front: lattice side panels and front frame slide out with the hopper
  const ext = new THREE.Group();
  root.add(ext);
  for (const s of [-1, 1]) latticePlate(ext, extLen + 0.03, top - hY - 0.04, 1, xF - extLen / 2 + 0.01, (hY + top) / 2 + 0.02, s * (hw + 0.016));
  const fx = xF + 0.012;
  for (const y of [top, 0.27]) rbx(0.018, 0.018, 2 * hw + 0.04, 0.003, lattice, ext, fx, y, 0);
  for (const s of [-1, 1]) rbx(0.018, top - 0.27, 0.018, 0.003, lattice, ext, fx, (top + 0.27) / 2, s * (hw + 0.012));
  rod(ext, new THREE.Vector3(fx, 0.27, -hw), new THREE.Vector3(fx, top, hw), 0.006, lattice);
  rod(ext, new THREE.Vector3(fx, top, -hw), new THREE.Vector3(fx, 0.27, hw), 0.006, lattice);

  // ---- top frame: lattice rails round the top, curving round the turret
  for (const s of [-1, 1]) rbx(xF - xB + 0.03, 0.012, 0.03, 0.003, lattice, root, (xB + xF) / 2, top + 0.006, s * (hw + 0.004));
  {
    const arc = new THREE.Mesh(new THREE.TorusGeometry(0.17, 0.012, 6, 30, Math.PI * 0.9), lattice);
    arc.rotation.x = Math.PI / 2;
    arc.rotation.z = -Math.PI * 0.45;
    arc.position.set(t.x, top + 0.006, t.z);
    root.add(arc);
  }

  // ---- over-bumper intake, held down by surgical tubing
  const intake = new THREE.Group();
  const ipx = L / 2 - 0.02, ipy = 0.24;
  intake.position.set(ipx, ipy, 0);
  root.add(intake);
  const { armLen, deploy } = intakeArm(cfg, ipx, ipy, 0.03);
  for (const s of [-1, 1]) latticePlate(intake, armLen + 0.05, 0.07, 3, armLen / 2, 0, s * (cfg.intake.width / 2 + 0.012));
  const intakeRollers = [
    cylZ(0.03, cfg.intake.width, std(0x2b2b2e, 0.85, 0.05), intake, armLen, 0, 0, 16),
    cylZ(0.02, cfg.intake.width, M.alu, intake, armLen - 0.07, -0.04, 0, 12),
  ];
  kraken(intake, 0.03, 0, cfg.intake.width / 2 + 0.05, true, 'z');
  const tubing = std(0xc9a26b, 0.7, 0);
  for (const s of [-1, 1]) rod(root, new THREE.Vector3(ipx - 0.12, 0.3, s * (cfg.intake.width / 2 + 0.03)), new THREE.Vector3(ipx + 0.03, 0.18, s * (cfg.intake.width / 2 + 0.03)), 0.004, tubing);

  // ---- electronics box, back right
  {
    const g = new THREE.Group();
    g.position.set(-0.17, 0, 0.19);
    root.add(g);
    for (const s of [-1, 1]) latticePlate(g, 0.22, 0.24, 2, 0, 0.28, s * 0.13);
    latticePlate(g, 0.26, 0.24, 2, -0.11, 0.28, 0, Math.PI / 2);
    addElectronics(g, 0, 0, Math.PI / 2);
  }

  // ---- vertical kicker in front of the turret: a column of green compliant wheels
  const kicker = new THREE.Group();
  kicker.position.set(-0.035, 0, t.z);
  root.add(kicker);
  for (const s of [-1, 1]) pocketPlate(kicker, 0.12, 0.3, 0.006, lattice, [[0, 0.06, 0.02], [0, -0.06, 0.02]], 0, 0.3, s * 0.09);
  const kickWheels = [];
  for (const y of [0.2, 0.28, 0.36]) kickWheels.push(wheelStack(kicker, 0.16, 0.035, 3, green, 0.03, y, 0, 0.03));

  // ---- turret: 8in bearing ring with the printed guide fins, shooter on top
  const ringY = 0.4;
  const base = new THREE.Group();
  base.position.set(t.x, ringY, t.z);
  root.add(base);
  mesh(new THREE.TorusGeometry(0.115, 0.012, 8, 44), M.alu, base, 0, 0, 0).rotation.x = Math.PI / 2;
  for (let i = 0; i < 20; i++) { const a = (i / 20) * Math.PI * 2; bolts.add(new THREE.Vector3(t.x + Math.cos(a) * 0.115, ringY + 0.012, t.z - Math.sin(a) * 0.115), new THREE.Vector3(0, 1, 0)); }
  for (const a of [0.3, 0.9, 1.5, 2.1, 2.7]) {
    const f = bx(0.012, 0.07, 0.03, fin, base, Math.cos(a) * 0.14, 0.03, -Math.sin(a) * 0.14);
    f.rotation.y = a;
    f.rotation.z = 0.35;
  }
  const turret = new THREE.Group();
  turret.position.set(t.x, ringY, t.z);
  root.add(turret);
  mesh(new THREE.CylinderGeometry(0.108, 0.108, 0.012, 40), std(0x33373e, 0.5, 0.5), turret, 0, 0.008, 0);
  const plateShape = new THREE.Shape();
  plateShape.moveTo(-0.11, 0); plateShape.lineTo(0.1, 0); plateShape.lineTo(0.02, 0.15); plateShape.lineTo(-0.07, 0.15); plateShape.closePath();
  for (const [hx, hy] of [[-0.05, 0.05], [0.03, 0.05], [-0.02, 0.11]]) { const p = new THREE.Path(); p.absarc(hx, hy, 0.022, 0, Math.PI * 2, true); plateShape.holes.push(p); }
  for (const s of [-1, 1]) {
    const g = new THREE.ExtrudeGeometry(plateShape, { depth: 0.006, bevelEnabled: false });
    g.translate(0, 0.012, s * 0.07 - 0.003);
    mesh(g, lattice, turret);
  }
  const fly = wheelStack(turret, 0.12, 0.048, 4, green, -0.02, 0.1, 0, 0.026);
  const hood = new THREE.Group();
  hood.position.set(-0.02, 0.1, 0);
  turret.add(hood);
  const hoodShell = new THREE.Mesh(new THREE.CylinderGeometry(0.1, 0.1, 0.14, 20, 1, true, -0.2, 1.7), std(0x1b1d22, 0.45, 0.5, { side: THREE.DoubleSide }));
  hoodShell.rotation.x = Math.PI / 2;
  hood.add(hoodShell);
  for (const s of [-1, 1]) kraken(turret, -0.08, 0.1, s * 0.1, false, 'z');
  limelight(turret, -0.1, 0.13, 0, Math.PI);
  bolts.done();

  // net over the top: pinned to the frame round the hopper, round the turret, and to the front
  // frame as it slides out
  const net = hopperNet(root, {
    x0: xB + 0.005, x1: xF + 0.006, x1Out: xF + 0.006 + extLen, xFixed: xF - 0.02, hw, y: top + 0.012,
    hole: { x: t.x, z: t.z, r: bay.dome.rHole },
  });

  const anim = (st, dt) => {
    const ex = st.hopperDeploy * extLen;
    ext.position.x = ex;
    intake.rotation.z = lerp(STOWED, deploy, st.intakeDeploy);
    for (const r of intakeRollers) r.rotation.y += st.intakeSpeed * dt * 40;
    for (const r of floorRollers) r.rotation.y -= (st.feeding + 0.3 * Math.max(0, st.intakeSpeed)) * dt * 30;
    for (const w of kickWheels) w.rotation.z += st.feeding * dt * 30;
    turret.rotation.y = st.turretYaw;
    fly.rotation.z -= st.flywheel * dt * 9;
    hood.rotation.z = (st.hoodDeg - 60) * Math.PI / 180 * 0.5;
    net.drape(st.load, ex);
  };
  return { root, anim, stored: [], modules, extLen };
}

// ============================================================ 4946 Moto Moto
// 4946's Onshape document is view-only (no export), so this is drawn from Onshape's scaled renders
// of it and their engineering report: a round robot (half-circle drivetrain, 30in front edge) with
// clear hopper walls on its bumpers, a Dye Rotor tray whose spinner carries FUEL round to the
// center, the turret on the center of rotation (a tall column with a flared shroud and the shooter
// in a bearing ring on top), a gantry of perforated tube over the front with the beacon, and an
// intake box that slides out the front. A net stretches diagonally from the top of the hopper's
// front wall down to the front of the intake box.
function build4946(cfg, alliance) {
  const root = new THREE.Group();
  const L = cfg.frame.length, W = cfg.frame.width, bay = cfg.bay, sh = cfg.shooter;
  const extLen = cfg.storage.extLen, ch = bay.chamfer;
  addBumpers(root, cfg, alliance, ch);
  const modules = addDrivebase(root, cfg, ch);
  const grey = std(0x9aa0a8, 0.4, 0.7);
  const dark = std(0x24262b, 0.5, 0.45);
  const trayMat = std(0xa9bccb, 0.5, 0.4);
  const green = std(0x5fb94a, 0.75, 0.05);
  const bolts = boltSet(root, std(0xc4c8ce, 0.3, 0.9));
  const top = bay.top, y0 = 0.17;
  const t = sh.turretPos;

  // ---- clear walls on the bumpers, following the frame round the back
  const out = 0.025;
  const pts = frameOutline(L + 2 * out, W + 2 * out, ch + out * 0.4);
  const xFront = bay.x1 + 0.005;
  pts[0][0] = pts[1][0] = xFront;
  for (let i = 0; i < pts.length; i++) {
    const [xa, za] = pts[i], [xb, zb] = pts[(i + 1) % pts.length];
    const len = Math.hypot(xb - xa, zb - za);
    const g = new THREE.Group();
    g.position.set((xa + xb) / 2, 0, (za + zb) / 2);
    g.rotation.y = -Math.atan2(zb - za, xb - xa);
    root.add(g);
    const pane = mesh(new THREE.PlaneGeometry(len, top - y0), M.poly, g, 0, (y0 + top) / 2, 0);
    pane.castShadow = false;
    rbx(len, 0.014, 0.014, 0.002, grey, g, 0, top, 0);
    rbx(0.014, top - y0, 0.014, 0.002, grey, g, len / 2, (y0 + top) / 2, 0);
  }

  // ---- Dye Rotor tray: a round plate under the load, with the spinner's vanes on it
  const rs = bay.rotor;
  mesh(new THREE.CylinderGeometry(rs.r + 0.01, rs.r + 0.01, 0.012, 48), trayMat, root, rs.x, rs.y - 0.006, rs.z);
  for (let i = 0; i < 16; i++) {
    const a = (i / 16) * Math.PI * 2;
    bolts.add(new THREE.Vector3(rs.x + Math.cos(a) * (rs.r - 0.02), rs.y + 0.001, rs.z - Math.sin(a) * (rs.r - 0.02)), new THREE.Vector3(0, 1, 0));
  }
  const rotor = new THREE.Group();
  rotor.position.set(rs.x, rs.y, rs.z);
  root.add(rotor);
  for (let i = 0; i < 8; i++) {
    const a = (i / 8) * Math.PI * 2;
    const v = bx(rs.r - rs.finR0, 0.07, 0.008, dark, rotor, Math.cos(a) * (rs.r + rs.finR0) / 2, 0.035, -Math.sin(a) * (rs.r + rs.finR0) / 2);
    v.rotation.y = a;
  }
  mesh(new THREE.CylinderGeometry(rs.finR0, rs.finR0, 0.04, 32), dark, rotor, 0, 0.02, 0);

  // ---- the turret's column and, on top, the turret itself
  cylY(0.15, 0.27, dark, root, t.x, 0.2 + 0.135, t.z, 40);
  for (let i = 0; i < 8; i++) {
    const a = (i / 8) * Math.PI * 2;
    const slot = bx(0.012, 0.05, 0.02, std(0x0e0f11, 0.8, 0), root, t.x + Math.cos(a) * 0.152, 0.4, t.z - Math.sin(a) * 0.152);
    slot.rotation.y = a;
  }
  const turret = new THREE.Group();
  turret.position.set(t.x, 0.47, t.z);
  root.add(turret);
  const shroud = mesh(new THREE.CylinderGeometry(0.175, 0.15, 0.2, 40, 1, true), std(0x2c2f35, 0.5, 0.4, { side: THREE.DoubleSide }), turret, 0, 0.1, 0);
  shroud.castShadow = true;
  for (let i = 0; i < 10; i++) {
    const a = (i / 10) * Math.PI * 2;
    const w = bx(0.004, 0.03, 0.022, std(0x0e0f11, 0.8, 0), turret, Math.cos(a) * 0.165, 0.12, -Math.sin(a) * 0.165);
    w.rotation.y = a;
  }
  mesh(new THREE.TorusGeometry(0.19, 0.012, 8, 48), grey, turret, 0, 0.24, 0).rotation.x = Math.PI / 2;
  mesh(new THREE.RingGeometry(0.15, 0.2, 40), std(0x6b7078, 0.4, 0.6, { side: THREE.DoubleSide }), turret, 0, 0.245, 0).rotation.x = -Math.PI / 2;
  // shooter in the ring: the flywheel across it, the hood on the exit side (+x, the shot direction)
  const fly = wheelStack(turret, 0.2, 0.05, 4, green, -0.02, 0.2, 0, 0.035);
  const hood = new THREE.Group();
  hood.position.set(-0.02, 0.2, 0);
  turret.add(hood);
  const hoodShell = new THREE.Mesh(new THREE.CylinderGeometry(0.1, 0.1, 0.22, 20, 1, true, 0.1, 1.6), std(0x1b1d22, 0.45, 0.5, { side: THREE.DoubleSide }));
  hoodShell.rotation.x = Math.PI / 2;
  hood.add(hoodShell);
  for (const s of [-1, 1]) kraken(turret, -0.08, 0.13, s * 0.12, false, 'z');

  // ---- gantry over the front of the hopper: perforated tube, beacon on top
  const gx = xFront - 0.06, gz = W / 2 - 0.02, gy = 0.74;
  const tubeMat = std(0xb8bdc4, 0.35, 0.85);
  for (const sz of [-1, 1]) rbx(0.025, gy - 0.12, 0.025, 0.003, tubeMat, root, gx, 0.12 + (gy - 0.12) / 2, sz * gz);
  rbx(0.025, 0.025, 2 * gz + 0.025, 0.003, tubeMat, root, gx, gy, 0);
  for (let z = -gz + 0.04; z < gz; z += 0.035) bolts.add(new THREE.Vector3(gx + 0.0126, gy, z), new THREE.Vector3(1, 0, 0));
  cylY(0.02, 0.04, std(0xff8a1f, 0.4, 0.1, { emissive: 0x6a2c00 }), root, gx, gy + 0.03, -gz + 0.03, 14);
  limelight(root, gx, gy - 0.08, 0, 0);

  // ---- intake box: slides out the front
  const box = new THREE.Group();
  root.add(box);
  const iw = cfg.intake.width / 2 + 0.012;
  const xF = L / 2 + BUMPER_T + cfg.intake.reach - extLen; // front of the box, stowed
  const bx0 = xF - 0.26; // back of its side plates
  const plateShape = new THREE.Shape();
  plateShape.moveTo(bx0, 0.05); plateShape.lineTo(xF, 0.05); plateShape.lineTo(xF, 0.44); plateShape.lineTo(xF - 0.07, bay.slope.y + 0.02); plateShape.lineTo(bx0, top - 0.03); plateShape.closePath();
  const hole = new THREE.Path();
  hole.moveTo(bx0 + 0.05, 0.2); hole.lineTo(xF - 0.06, 0.2); hole.lineTo(xF - 0.06, 0.38); hole.lineTo(bx0 + 0.05, top - 0.1); hole.closePath();
  plateShape.holes.push(hole);
  for (const sz of [-1, 1]) {
    const g = new THREE.ExtrudeGeometry(plateShape, { depth: 0.006, bevelEnabled: false });
    g.translate(0, 0, sz * iw - 0.003);
    mesh(g, grey, box);
  }
  // spun about their axles (z): the two over the FUEL turn with their undersides moving in,
  // the low one it rides over turns the other way
  const star = wheelStack(box, 2 * iw - 0.02, 0.032, 10, green, xF - 0.03, 0.26, 0, 0.022);
  const upper = cylZ(0.022, 2 * iw - 0.02, M.black, box, xF - 0.02, 0.19, 0, 12);
  const under = cylZ(0.02, 2 * iw - 0.02, M.alu, box, xF - 0.1, 0.08, 0, 12);
  rbx(0.03, 0.03, 2 * iw, 0.004, tubeMat, box, xF - 0.015, 0.44, 0); // top crossbar
  kraken(box, xF - 0.12, 0.3, iw + 0.04, true, 'z');
  bolts.done();

  // the net from the top of the front wall down to the intake box's top crossbar
  const net = hopperNet(root, { x0: xFront, x1: xF - 0.02, x1Out: xF - 0.02 + extLen, xFixed: xFront, hw: iw - 0.01, y: top, yFront: 0.45, NX: 14, NZ: 24 });

  const anim = (st, dt) => {
    const ex = st.hopperDeploy * extLen;
    box.position.x = ex;
    const spin = st.intakeSpeed * dt * 40;
    star.rotation.z -= spin; // a group: its axle is its z
    upper.rotation.y -= spin; // cylZ meshes are tipped onto z, so their own y is the axle
    under.rotation.y += spin;
    rotor.rotation.y = st.rotorAngle ?? 0;
    turret.rotation.y = st.turretYaw;
    fly.rotation.z -= st.flywheel * dt * 9;
    hood.rotation.z = (st.hoodDeg - 60) * Math.PI / 180 * 0.4;
    net.drape(st.load, ex);
  };
  return { root, anim, stored: [], modules, extLen };
}

export function buildRobotModel(cfg, alliance) {
  let m;
  if (cfg.key === '2910') m = build2910(cfg, alliance);
  else if (cfg.key === '4414') m = build4414(cfg, alliance);
  else if (cfg.key === '971') m = build971(cfg, alliance);
  else if (cfg.key === '1678') m = build1678(cfg, alliance);
  else if (cfg.key === '1690') m = build1690(cfg, alliance);
  else if (cfg.key === '4946') m = build4946(cfg, alliance);
  else m = build8793(cfg, alliance);
  // stored FUEL visual (instanced)
  const r = FUEL.radius;
  const geo = new THREE.SphereGeometry(r, 20, 14);
  // squeezed FUEL flattens where it's pressed: up to four contact patches per ball (touch0..3:
  // the patch's normal and its distance from the center, Hopper.touchPatches). The sphere is cut
  // flat at each one and the patch shades flat, so only the contact edges squash.
  const MAX = 260; // room for whatever the hopper holds
  const touch = [0, 1, 2, 3].map((j) => {
    const a = new THREE.InstancedBufferAttribute(new Float32Array(4 * MAX), 4);
    for (let i = 0; i < MAX; i++) { a.array[4 * i + 1] = 1; a.array[4 * i + 3] = 1; }
    a.setUsage(THREE.DynamicDrawUsage);
    geo.setAttribute('touch' + j, a);
    return a;
  });
  const mat = new THREE.MeshStandardMaterial({ color: 0xf4d23a, roughness: 0.75 });
  mat.onBeforeCompile = (sh) => {
    sh.vertexShader = sh.vertexShader
      .replace('#include <common>', `#include <common>
attribute vec4 touch0; attribute vec4 touch1; attribute vec4 touch2; attribute vec4 touch3;
void squash(vec4 c, inout vec3 p, inout vec3 nrm) {
  float s = dot(p, c.xyz) - c.w;
  if (s > 0.0) { p -= c.xyz * s; nrm = c.xyz; }
}`)
      .replace('#include <beginnormal_vertex>', `#include <beginnormal_vertex>
vec3 sqP = position;
squash(touch0, sqP, objectNormal); squash(touch1, sqP, objectNormal);
squash(touch2, sqP, objectNormal); squash(touch3, sqP, objectNormal);`)
      .replace('#include <begin_vertex>', 'vec3 transformed = sqP;');
  };
  const inst = new THREE.InstancedMesh(geo, mat, MAX);
  inst.instanceMatrix.setUsage(THREE.DynamicDrawUsage);
  inst.frustumCulled = false;
  inst.castShadow = true;
  inst.count = 0;
  m.root.add(inst);
  m.storedMesh = inst;
  m.storedTouch = touch;
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
