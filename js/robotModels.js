// Robot 3D models built from primitives. Local frame: +X = robot front (intake side),
// +Z = robot right, Y = up, origin on the floor at the robot center.
import * as THREE from 'three';
import { RoundedBoxGeometry } from 'three/addons/geometries/RoundedBoxGeometry.js';
import { IN, FUEL } from './constants.js';
import { BUMPER_T } from './robotConfigs.js';
import { ALLIANCE_COLOR_CSS } from './field.js';

const BUMP_Y0 = 0.055, BUMP_Y1 = 0.16;

function std(color, rough = 0.55, metal = 0.2, extra = {}) {
  return new THREE.MeshStandardMaterial({ color, roughness: rough, metalness: metal, ...extra });
}
const M = {
  alu: std(0xaeb3bb, 0.35, 0.85),
  darkAlu: std(0x3a3e46, 0.45, 0.7),
  black: std(0x15161a, 0.7, 0.2),
  wheel: std(0x101012, 0.9, 0.0),
  tread: std(0x1c1c1c, 0.95, 0.0),
  poly: new THREE.MeshPhysicalMaterial({ color: 0xcfe3ff, transparent: true, opacity: 0.22, roughness: 0.08, depthWrite: false, side: THREE.DoubleSide }),
  tintPoly: new THREE.MeshPhysicalMaterial({ color: 0x8fa3b8, transparent: true, opacity: 0.3, roughness: 0.1, depthWrite: false, side: THREE.DoubleSide }),
  green: std(0x39b54a, 0.6, 0.1),
  compliant: std(0x2a2a2a, 0.9, 0.0),
  yellowWheel: std(0xd8c21a, 0.6, 0.1),
  copper: std(0xb87333, 0.35, 0.9),
  led: std(0xffffff, 0.3, 0, { emissive: 0xffffff, emissiveIntensity: 1.2 }),
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
// cylinder whose axis is along local Z
function cylZ(r, len, mat, parent, x, y, z, segs = 18) {
  const m = mesh(new THREE.CylinderGeometry(r, r, len, segs), mat, parent, x, y, z);
  m.rotation.x = Math.PI / 2;
  return m;
}

function bumperTexture(number, alliance, lengthM) {
  const c = document.createElement('canvas');
  c.width = 512;
  c.height = 96;
  const g = c.getContext('2d');
  g.fillStyle = ALLIANCE_COLOR_CSS[alliance];
  g.fillRect(0, 0, c.width, c.height);
  // fabric texture
  for (let i = 0; i < 700; i++) {
    g.fillStyle = `rgba(0,0,0,${Math.random() * 0.08})`;
    g.fillRect(Math.random() * 512, Math.random() * 96, 2, 2);
  }
  g.fillStyle = '#ffffff';
  const fontPx = Math.min(78, 78 * (lengthM / 0.7));
  g.font = `900 ${fontPx}px "Arial Black", Arial, sans-serif`;
  g.textAlign = 'center';
  g.textBaseline = 'middle';
  g.fillText(String(number), 256, 52);
  const t = new THREE.CanvasTexture(c);
  t.colorSpace = THREE.SRGBColorSpace;
  return t;
}

function addBumpers(root, cfg, alliance, chamferBack = 0) {
  const L = cfg.frame.length, W = cfg.frame.width, T = BUMPER_T;
  const h = BUMP_Y1 - BUMP_Y0, yc = (BUMP_Y0 + BUMP_Y1) / 2;
  const plain = std(ALLIANCE_COLOR_CSS[alliance], 0.85, 0.0);
  const seg = (len, x, z, rotY, withText) => {
    const geo = new RoundedBoxGeometry(len, h, T, 3, 0.03);
    const mats = [plain, plain, plain, plain, plain, plain];
    if (withText) {
      const tm = std(0xffffff, 0.85, 0, { map: bumperTexture(cfg.team, alliance, len) });
      mats[4] = tm; // +z face
      mats[5] = tm; // -z face (seen mirrored from inside; fine)
    }
    const m = new THREE.Mesh(geo, mats);
    m.position.set(x, yc, z);
    m.rotation.y = rotY;
    m.castShadow = true;
    m.receiveShadow = true;
    root.add(m);
  };
  const outerW = W + 2 * T;
  // front and back (run along Z)
  seg(outerW, L / 2 + T / 2, 0, Math.PI / 2, true);
  seg(outerW - 2 * chamferBack, -L / 2 - T / 2, 0, -Math.PI / 2, true);
  // sides (run along X)
  seg(L - chamferBack, -chamferBack / 2, W / 2 + T / 2, 0, true);
  seg(L - chamferBack, -chamferBack / 2, -W / 2 - T / 2, Math.PI, true);
  if (chamferBack > 0) {
    for (const s of [-1, 1]) {
      const len = Math.SQRT2 * chamferBack + T * 0.6;
      const geo = new RoundedBoxGeometry(len, h, T, 3, 0.03);
      const m = new THREE.Mesh(geo, plain);
      m.position.set(-L / 2 + chamferBack / 2 - T * 0.35, yc, s * (W / 2 - chamferBack / 2 + T * 0.35));
      m.rotation.y = s * Math.PI / 4;
      m.castShadow = true;
      root.add(m);
    }
  }
}

function addDrivebase(root, cfg, chamferBack = 0) {
  const L = cfg.frame.length, W = cfg.frame.width;
  const frameMat = std(cfg.colors.frame, 0.45, 0.6);
  const t = 0.05;
  // perimeter tubes (2x1)
  bx(L, 0.025, t, frameMat, root, -chamferBack / 2 * 0, 0.075, W / 2 - t / 2);
  bx(L, 0.025, t, frameMat, root, 0, 0.075, -W / 2 + t / 2);
  bx(t, 0.025, W - 2 * chamferBack, frameMat, root, L / 2 - t / 2, 0.075, 0);
  bx(t, 0.025, W - 2 * chamferBack, frameMat, root, -L / 2 + t / 2, 0.075, 0);
  // bellypan
  bx(L - 0.04, 0.006, W - 0.04, M.darkAlu, root, 0, 0.05, 0);
  // swerve modules
  const mods = [];
  const inset = 0.075;
  for (const sx of [1, -1]) {
    for (const sz of [1, -1]) {
      const g = new THREE.Group();
      g.position.set(sx * (L / 2 - inset), 0, sz * (W / 2 - inset));
      root.add(g);
      bx(0.1, 0.05, 0.1, M.darkAlu, g, 0, 0.115, 0);
      const wheelPivot = new THREE.Group();
      g.add(wheelPivot);
      const wheel = cylZ(0.0508, 0.038, M.wheel, wheelPivot, 0, 0.0508, 0, 20);
      const motor = mesh(new THREE.CylinderGeometry(0.03, 0.03, 0.08, 14), M.black, g, 0.0, 0.18, 0.0);
      motor.castShadow = true;
      mods.push({ pivot: wheelPivot, wheel });
    }
  }
  return mods;
}

function storedGrid(xs, zs, ys) {
  const out = [];
  for (const y of ys) for (const x of xs) for (const z of zs) out.push(new THREE.Vector3(x, y, z));
  return out;
}

// ============================================================ 2910 Re•Blitz
function build2910(cfg, alliance) {
  const root = new THREE.Group();
  const L = cfg.frame.length, W = cfg.frame.width;
  addBumpers(root, cfg, alliance);
  const modules = addDrivebase(root, cfg);
  const plate = std(0x8e939b, 0.35, 0.85);
  const purple = std(cfg.colors.accent, 0.5, 0.3);

  // Shooter tower at the back: thick billet side plates, 4" drum + hood rollers, launching forward
  const tower = new THREE.Group();
  tower.position.set(-L / 2 + 0.11, 0, 0);
  root.add(tower);
  for (const s of [-1, 1]) {
    const sp = bx(0.2, 0.46, 0.015, plate, tower, 0, 0.3, s * (W / 2 - 0.03));
    sp.castShadow = true;
  }
  const drum = cylZ(0.0508, W - 0.08, M.alu, tower, -0.02, 0.44, 0, 24);
  const hood = new THREE.Group();
  hood.position.set(-0.02, 0.44, 0);
  tower.add(hood);
  const hoodShell = new THREE.Mesh(new THREE.CylinderGeometry(0.13, 0.13, W - 0.1, 20, 1, true, Math.PI * 0.05, Math.PI * 0.6), std(0x2a2d33, 0.5, 0.5, { side: THREE.DoubleSide }));
  hoodShell.rotation.x = Math.PI / 2;
  hood.add(hoodShell);
  const hoodRollers = [];
  for (let i = 0; i < 3; i++) {
    const a = 0.35 + i * 0.5;
    hoodRollers.push(cylZ(0.019, W - 0.1, M.alu, hood, -Math.cos(a) * 0.1, Math.sin(a) * 0.1, 0, 12));
  }
  // flywheel (overdriven mass) on one side
  const fly = cylZ(0.076, 0.03, M.copper, tower, -0.02, 0.44, W / 2 - 0.06, 24);
  // backing board and inclined powered feeder floor (15 deg) with rollers
  bx(0.01, 0.3, W - 0.08, std(0xdddddd, 0.3, 0.1), tower, -0.09, 0.25, 0);
  const feeder = new THREE.Group();
  feeder.position.set(0.05, 0.1, 0);
  feeder.rotation.z = 0.26;
  root.add(feeder);
  const feedRollers = [];
  for (let i = 0; i < 10; i++) feedRollers.push(cylZ(0.016, W - 0.08, M.alu, feeder, 0.3 - i * 0.05, 0, 0, 10));
  // compliant indexing wheels
  for (let i = 0; i < 9; i++) cylZ(0.028, 0.02, M.compliant, tower, 0.05, 0.3, -W / 2 + 0.08 + i * ((W - 0.16) / 8), 12);

  // Hopper: one-piece light-tinted polycarbonate hopper, slides forward on deploy
  const hopper = new THREE.Group();
  root.add(hopper);
  const hL = 0.62, hH = 0.36, hY = 0.2;
  const hx0 = 0.05;
  for (const s of [-1, 1]) bx(hL, hH, 0.006, M.tintPoly, hopper, hx0, hY + hH / 2, s * (W / 2 - 0.01));
  bx(0.006, hH, W - 0.02, M.tintPoly, hopper, hx0 + hL / 2, hY + hH / 2, 0);
  bx(hL, 0.006, W - 0.02, M.tintPoly, hopper, hx0, hY + hH, 0); // closed top
  bx(hL, 0.02, 0.025, M.alu, hopper, hx0, hY + hH, W / 2 - 0.01);
  bx(hL, 0.02, 0.025, M.alu, hopper, hx0, hY + hH, -W / 2 + 0.01);
  bx(0.025, 0.013, W, M.alu, hopper, hx0 + hL / 2, hY + hH, 0);

  // Slap-down intake: pivot at the front top of the frame
  const intake = new THREE.Group();
  intake.position.set(L / 2 - 0.04, 0.24, 0);
  root.add(intake);
  const armLen = 0.36;
  for (const s of [-1, 1]) {
    const arm = bx(armLen, 0.05, 0.015, plate, intake, armLen / 2, 0, s * (cfg.intake.width / 2 + 0.01));
  }
  const intakeRollers = [
    cylZ(0.0254, cfg.intake.width, std(0x9ad0ff, 0.3, 0.1, { transparent: true, opacity: 0.85 }), intake, armLen, 0, 0, 16),
    cylZ(0.016, cfg.intake.width, M.alu, intake, armLen - 0.07, -0.05, 0, 12),
    cylZ(0.016, cfg.intake.width, M.alu, intake, armLen - 0.13, -0.08, 0, 12),
  ];
  bx(0.05, 0.05, cfg.intake.width + 0.04, M.darkAlu, intake, armLen - 0.02, 0.05, 0); // 2x2 bash bar
  // Limelight 4 on the front
  bx(0.06, 0.05, 0.1, M.black, root, L / 2 - 0.05, 0.5, 0);
  mesh(new THREE.CircleGeometry(0.018, 16), std(0x223344, 0.1, 0.6, { emissive: 0x113355 }), root, L / 2 - 0.019, 0.5, 0).rotation.y = Math.PI / 2;

  const stored = storedGrid([-0.14, 0.01, 0.16, 0.31], [-0.225, -0.075, 0.075, 0.225], [0.2, 0.34, 0.47]);
  // plus the deployed hopper extension row
  for (const y of [0.24, 0.38]) for (const z of [-0.225, -0.075, 0.075, 0.225]) stored.push(new THREE.Vector3(0.44, y, z));
  // plus balls pre-staged in the tower
  for (const z of [-0.19, -0.063, 0.063, 0.19]) stored.push(new THREE.Vector3(-0.19, 0.3, z));

  const anim = (st, dt) => {
    // intake pivot: 0 = stowed (up), 1 = deployed (down, over the bumper)
    intake.rotation.z = THREE.MathUtils.lerp(2.3, -0.42, st.intakeDeploy);
    const roll = st.intakeSpeed * dt * 40;
    for (const r of intakeRollers) r.rotation.y += roll;
    hopper.position.x = THREE.MathUtils.lerp(-0.1, 0.02, st.hopperDeploy);
    drum.rotation.y -= st.flywheel * dt * 6;
    fly.rotation.y -= st.flywheel * dt * 6;
    for (const r of hoodRollers) r.rotation.y += st.flywheel * dt * 8;
    for (const r of feedRollers) r.rotation.y -= st.feeding * dt * 30;
    hood.rotation.z = (st.hoodDeg - 58) * Math.PI / 180 * 0.6;
  };
  return { root, anim, stored, modules, shooterLocal: null };
}

// ============================================================ 4414 RIPCURRENT
function build4414(cfg, alliance) {
  const root = new THREE.Group();
  const L = cfg.frame.length, W = cfg.frame.width;
  const chamfer = 0.12;
  addBumpers(root, cfg, alliance, chamfer);
  const modules = addDrivebase(root, cfg, 0);
  const teal = std(cfg.colors.accent, 0.4, 0.4);
  const plate = std(0x9aa0a8, 0.35, 0.85);

  // Hopper walls rise from the structural bumpers; the front section extends with the intake
  const hH = 0.4, hY = 0.16;
  const back = new THREE.Group();
  root.add(back);
  for (const s of [-1, 1]) {
    bx(L - chamfer, hH, 0.004, M.poly, back, -chamfer / 2, hY + hH / 2, s * (W / 2 + 0.01));
    bx(L - chamfer, 0.02, 0.02, teal, back, -chamfer / 2, hY + hH, s * (W / 2 + 0.01));
  }
  bx(0.004, hH, W - 2 * chamfer, M.poly, back, -L / 2 - 0.01, hY + hH / 2, 0);
  const ext = new THREE.Group();
  root.add(ext);
  const extL = 0.3;
  for (const s of [-1, 1]) {
    bx(extL, hH, 0.004, M.poly, ext, L / 2 + extL / 2 - 0.05, hY + hH / 2, s * (W / 2 + 0.03));
    bx(extL, 0.02, 0.02, teal, ext, L / 2 + extL / 2 - 0.05, hY + hH, s * (W / 2 + 0.03));
  }
  bx(0.004, hH * 0.55, W + 0.06, M.poly, ext, L / 2 + extL - 0.05, hY + hH * 0.72, 0);
  bx(0.02, 0.02, W + 0.06, teal, ext, L / 2 + extL - 0.05, hY + hH, 0);

  // Dye Rotor: large rotating disc with fins, "dolphin fin" ramp and center shrink-wrap cone
  const rotor = new THREE.Group();
  rotor.position.set(-0.04, 0.1, 0);
  root.add(rotor);
  const rotorR = Math.min(L, W) / 2 - 0.03;
  mesh(new THREE.CylinderGeometry(rotorR, rotorR, 0.012, 40), std(0xe8e8e8, 0.5, 0.1), rotor, 0, 0, 0);
  mesh(new THREE.ConeGeometry(0.13, 0.22, 24), std(0xdadfe6, 0.4, 0.2), rotor, 0, 0.12, 0);
  for (let i = 0; i < 6; i++) {
    const a = (i * Math.PI) / 3;
    const fin = bx(rotorR - 0.12, 0.08, 0.012, teal, rotor, Math.cos(a) * (rotorR / 2 + 0.05), 0.05, Math.sin(a) * (rotorR / 2 + 0.05));
    fin.rotation.y = -a;
  }
  // Kicker / feeder stack at the back of the rotor
  bx(0.08, 0.3, 0.12, plate, root, -L / 2 + 0.1, 0.3, 0);
  cylZ(0.038, 0.1, std(0xdddd55, 0.6), root, -L / 2 + 0.1, 0.36, 0, 16);

  // Turret with hooded 3" flywheel shooter (4x Kraken X44)
  const turret = new THREE.Group();
  turret.position.set(cfg.shooter.turretPos.x, 0.4, cfg.shooter.turretPos.z);
  root.add(turret);
  mesh(new THREE.CylinderGeometry(0.13, 0.13, 0.03, 32), std(0x2b2f36, 0.4, 0.6), turret, 0, 0, 0);
  const body = new THREE.Group();
  turret.add(body);
  for (const s of [-1, 1]) {
    const side = bx(0.24, 0.13, 0.012, plate, body, 0, 0.08, s * 0.07);
  }
  const fly = cylZ(0.038, 0.12, M.copper, body, -0.02, 0.07, 0, 20);
  const hood = new THREE.Group();
  hood.position.set(-0.02, 0.07, 0);
  body.add(hood);
  const hoodShell = new THREE.Mesh(new THREE.CylinderGeometry(0.1, 0.1, 0.13, 16, 1, true, 0, Math.PI * 0.55), std(0x1e2126, 0.5, 0.4, { side: THREE.DoubleSide }));
  hoodShell.rotation.x = Math.PI / 2;
  hood.add(hoodShell);
  for (let i = 0; i < 4; i++) {
    const m = mesh(new THREE.CylinderGeometry(0.018, 0.018, 0.05, 12), M.black, body, -0.1, 0.07, -0.045 + i * 0.03);
  }
  // Intake: over-bumper, deploys at match start and latches down
  const intake = new THREE.Group();
  intake.position.set(L / 2 - 0.02, 0.2, 0);
  root.add(intake);
  const armLen = 0.36;
  for (const s of [-1, 1]) bx(armLen, 0.045, 0.012, M.alu, intake, armLen / 2, 0, s * (cfg.intake.width / 2 + 0.02));
  const intakeRollers = [
    cylZ(0.038, cfg.intake.width, std(0xcfe8ff, 0.3, 0.1, { transparent: true, opacity: 0.8 }), intake, armLen, 0, 0, 18),
    cylZ(0.019, cfg.intake.width, M.alu, intake, armLen - 0.09, -0.07, 0, 12),
  ];
  bx(0.07, 0.03, cfg.intake.width + 0.05, std(0xaeb3bb, 0.3, 0.9), intake, armLen - 0.02, 0.05, 0);

  const stored = [];
  for (const y of [0.2, 0.34, 0.48]) {
    for (let ix = 0; ix < 6; ix++) {
      for (let iz = 0; iz < 6; iz++) {
        const x = -0.28 + ix * 0.15, z = -0.375 + iz * 0.15;
        if (Math.hypot(x - cfg.shooter.turretPos.x, z) < 0.16 && y > 0.3) continue;
        if (x < -L / 2 + 0.05 && Math.abs(z) > W / 2 - chamfer) continue;
        stored.push(new THREE.Vector3(x, y, z));
      }
    }
  }
  stored.sort((a, b) => a.y - b.y || Math.hypot(a.x, a.z) - Math.hypot(b.x, b.z));

  const anim = (st, dt) => {
    intake.rotation.z = THREE.MathUtils.lerp(2.3, -0.4, st.intakeDeploy);
    for (const r of intakeRollers) r.rotation.y += st.intakeSpeed * dt * 35;
    ext.position.x = THREE.MathUtils.lerp(-0.26, 0, st.hopperDeploy);
    rotor.rotation.y += (st.feeding > 0 ? 7 : 0.6) * dt; // agitates slowly, spins fast when firing
    turret.rotation.y = st.turretYaw;
    fly.rotation.y -= st.flywheel * dt * 10;
    hood.rotation.z = (st.hoodDeg - 62) * Math.PI / 180 * 0.8;
  };
  return { root, anim, stored, modules };
}

// ============================================================ 8793 Hopperless
function build8793(cfg, alliance) {
  const root = new THREE.Group();
  const L = cfg.frame.length, W = cfg.frame.width;
  addBumpers(root, cfg, alliance);
  const modules = addDrivebase(root, cfg);
  const orange = std(cfg.colors.accent, 0.45, 0.35);
  const plate = std(0x2a2c31, 0.5, 0.6);

  // electronics / top plate
  bx(L - 0.1, 0.01, W - 0.1, std(0x1b1c20, 0.6, 0.4), root, -0.02, 0.17, 0);
  for (const s of [-1, 1]) bx(L * 0.5, 0.012, 0.18, orange, root, -0.12, 0.178, s * (W / 2 - 0.14));

  // Conveyor V2: funnel of compliant wheels from the intake back to the turret
  const conv = new THREE.Group();
  conv.position.set(0.1, 0.19, 0);
  root.add(conv);
  const convWheels = [];
  for (let row = 0; row < 3; row++) {
    const w = W - 0.18 - row * 0.1;
    const n = 6 - row;
    for (let i = 0; i < n; i++) {
      const z = -w / 2 + (i + 0.5) * (w / n);
      const m = cylZ(0.035, 0.03, M.compliant, conv, 0.12 - row * 0.1, 0.02 + row * 0.03, z, 14);
      convWheels.push(m);
    }
    cylZ(0.008, w, M.alu, conv, 0.12 - row * 0.1, 0.02 + row * 0.03, 0, 8);
  }
  for (const s of [-1, 1]) bx(0.34, 0.1, 0.008, plate, conv, 0.02, 0.05, s * (W / 2 - 0.08));

  // Turret: big round bearing plate at the back with the hooded flywheel shooter
  const turret = new THREE.Group();
  turret.position.set(cfg.shooter.turretPos.x, 0.25, cfg.shooter.turretPos.z);
  root.add(turret);
  mesh(new THREE.CylinderGeometry(0.22, 0.22, 0.025, 40), std(0x2a2d33, 0.45, 0.5), turret, 0, 0, 0);
  mesh(new THREE.TorusGeometry(0.215, 0.008, 6, 40), M.alu, turret, 0, 0.014, 0).rotation.x = Math.PI / 2;
  const body = new THREE.Group();
  turret.add(body);
  for (const s of [-1, 1]) {
    const sp = bx(0.3, 0.2, 0.012, orange, body, 0.0, 0.11, s * 0.1);
  }
  const fly = cylZ(0.05, 0.16, M.yellowWheel, body, 0.02, 0.1, 0, 22);
  const hood = new THREE.Group();
  hood.position.set(0.02, 0.1, 0);
  body.add(hood);
  const hoodShell = new THREE.Mesh(new THREE.CylinderGeometry(0.12, 0.12, 0.19, 18, 1, true, 0, Math.PI * 0.6), std(0x1b1d22, 0.45, 0.5, { side: THREE.DoubleSide }));
  hoodShell.rotation.x = Math.PI / 2;
  hood.add(hoodShell);
  for (let i = 0; i < 5; i++) cylZ(0.022, 0.03, M.compliant, hood, -Math.cos(0.5 + i * 0.3) * 0.12, Math.sin(0.5 + i * 0.3) * 0.12, 0, 10);
  // LED strip on the turret
  for (let i = 0; i < 7; i++) mesh(new THREE.SphereGeometry(0.008, 8, 6), M.led, body, 0.09 + i * 0.012, 0.16, 0.107);
  bx(0.05, 0.06, 0.08, M.black, body, -0.14, 0.2, 0); // camera

  // Intake V3: multi-roller over-bumper intake at the front
  const intake = new THREE.Group();
  intake.position.set(L / 2 - 0.03, 0.2, 0);
  root.add(intake);
  const armLen = 0.33;
  for (const s of [-1, 1]) {
    const arm = bx(armLen, 0.07, 0.012, plate, intake, armLen / 2, -0.01, s * (cfg.intake.width / 2 + 0.02));
  }
  const intakeRollers = [];
  for (let i = 0; i < 3; i++) intakeRollers.push(cylZ(0.024, cfg.intake.width, M.black, intake, armLen - i * 0.075, -0.02 + i * 0.03, 0, 14));
  for (let i = 0; i < 7; i++) cylZ(0.035, 0.025, M.compliant, intake, armLen - 0.19, 0.03, -cfg.intake.width / 2 + 0.05 + i * ((cfg.intake.width - 0.1) / 6), 12);

  // FUEL held in the ball path (intake -> conveyor -> turret)
  const stored = [];
  const path = [[0.33, 0.13], [0.26, 0.16], [0.19, 0.2], [0.12, 0.24], [0.05, 0.28], [-0.02, 0.32]];
  for (let i = path.length - 1; i >= 0; i--) for (const z of [-0.08, 0.08]) stored.push(new THREE.Vector3(path[i][0] - 0.1, path[i][1] + 0.06, z));

  const anim = (st, dt) => {
    intake.rotation.z = THREE.MathUtils.lerp(2.2, -0.35, st.intakeDeploy);
    for (const r of intakeRollers) r.rotation.y += st.intakeSpeed * dt * 40;
    for (const w of convWheels) w.rotation.y += (st.intakeSpeed + st.feeding) * dt * 25;
    turret.rotation.y = st.turretYaw;
    fly.rotation.y -= st.flywheel * dt * 8;
    hood.rotation.z = (st.hoodDeg - 62) * Math.PI / 180 * 0.8;
  };
  return { root, anim, stored, modules };
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
  const inst = new THREE.InstancedMesh(geo, mat, Math.max(1, m.stored.length));
  inst.castShadow = true;
  inst.count = 0;
  m.root.add(inst);
  m.storedMesh = inst;
  m.root.traverse((o) => {
    if (o.isMesh) o.castShadow = true;
  });
  // add a subtle climber visual if equipped
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
