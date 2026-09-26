import * as THREE from 'three';
import {
  IN, FIELD_L, FIELD_W, HALF_L, HALF_W, BLUE, RED, HUB, BUMP, TRENCH, TOWER, DEPOT, OUTPOST,
  GUARDRAIL_H, DS_BASE_H, DS_GLASS_H, ALLIANCE_WALL_H, ALLIANCE_ZONE_DEPTH, FUEL, GROUP, groups,
} from './constants.js';
import { mulQuat, yawQuat } from './physics.js';
import { inHexagon } from './util.js';

export const ALLIANCE_COLOR = { blue: 0x1f5fe0, red: 0xd92b2b };
export const ALLIANCE_COLOR_CSS = { blue: '#2f6ff0', red: '#e03434' };

const mats = {};
function initMaterials() {
  if (mats.ready) return;
  mats.ready = true;
  mats.black = new THREE.MeshStandardMaterial({ color: 0x1b1d22, roughness: 0.7, metalness: 0.2 });
  mats.dark = new THREE.MeshStandardMaterial({ color: 0x2b2e35, roughness: 0.6, metalness: 0.3 });
  mats.alu = new THREE.MeshStandardMaterial({ color: 0xb8bcc4, roughness: 0.35, metalness: 0.85 });
  mats.steel = new THREE.MeshStandardMaterial({ color: 0x8d9299, roughness: 0.45, metalness: 0.7 });
  mats.diamond = new THREE.MeshStandardMaterial({ color: 0x9aa0a8, roughness: 0.3, metalness: 0.9 });
  mats.white = new THREE.MeshStandardMaterial({ color: 0xeeeeee, roughness: 0.5 });
  mats.poly = new THREE.MeshPhysicalMaterial({
    color: 0xaec7de, transparent: true, opacity: 0.24, roughness: 0.05, metalness: 0, depthWrite: false, side: THREE.DoubleSide, // lightly tinted
  });
  mats.funnel = new THREE.MeshStandardMaterial({
    color: 0xf2f4f7, transparent: true, opacity: 0.72, roughness: 0.4, side: THREE.DoubleSide,
  });
  mats.net = new THREE.MeshStandardMaterial({
    map: makeNetTexture(), color: 0xffffff, transparent: true, roughness: 1, side: THREE.DoubleSide, depthWrite: false,
  });
  mats.tag = new THREE.MeshStandardMaterial({ map: makeTagTexture(), roughness: 0.6 });
  for (const a of [BLUE, RED]) {
    mats[a] = new THREE.MeshStandardMaterial({ color: ALLIANCE_COLOR[a], roughness: 0.55, metalness: 0.1 });
    mats[a + 'Tower'] = new THREE.MeshStandardMaterial({ color: ALLIANCE_COLOR[a], roughness: 0.4, metalness: 0.3 });
    mats[a + 'Bump'] = new THREE.MeshStandardMaterial({ color: ALLIANCE_COLOR[a], roughness: 0.85, metalness: 0.0 });
  }
}

function makeNetTexture() {
  const c = document.createElement('canvas');
  c.width = c.height = 256;
  const g = c.getContext('2d');
  g.fillStyle = 'rgba(20,20,24,0.18)';
  g.fillRect(0, 0, 256, 256);
  g.strokeStyle = 'rgba(15,15,18,0.95)';
  g.lineWidth = 3;
  for (let i = -256; i < 512; i += 32) {
    g.beginPath(); g.moveTo(i, 0); g.lineTo(i + 256, 256); g.stroke();
    g.beginPath(); g.moveTo(i + 256, 0); g.lineTo(i, 256); g.stroke();
  }
  const t = new THREE.CanvasTexture(c);
  t.wrapS = t.wrapT = THREE.RepeatWrapping;
  t.repeat.set(3, 4);
  t.colorSpace = THREE.SRGBColorSpace;
  return t;
}

function makeTagTexture() {
  const c = document.createElement('canvas');
  c.width = c.height = 64;
  const g = c.getContext('2d');
  g.fillStyle = '#fff';
  g.fillRect(0, 0, 64, 64);
  g.fillStyle = '#000';
  g.fillRect(6, 6, 52, 52);
  g.fillStyle = '#fff';
  for (let i = 0; i < 4; i++) for (let j = 0; j < 4; j++) if ((i * 7 + j * 3 + i * j) % 3 === 0) g.fillRect(14 + i * 9, 14 + j * 9, 9, 9);
  const t = new THREE.CanvasTexture(c);
  t.colorSpace = THREE.SRGBColorSpace;
  t.magFilter = THREE.NearestFilter;
  return t;
}

function box(w, h, d, mat, x, y, z, parent, shadow = true) {
  const m = new THREE.Mesh(new THREE.BoxGeometry(w, h, d), mat);
  m.position.set(x, y, z);
  m.castShadow = shadow;
  m.receiveShadow = true;
  parent.add(m);
  return m;
}

function cyl(r, len, mat, parent, segs = 16) {
  const m = new THREE.Mesh(new THREE.CylinderGeometry(r, r, len, segs), mat);
  m.castShadow = true;
  m.receiveShadow = true;
  parent.add(m);
  return m;
}

// Geometry of the hexagonal rim (flat sides facing the ALLIANCE WALL and the NEUTRAL ZONE).
// u = local axis toward the NEUTRAL ZONE, v = lateral axis.
function hexVerts(r) {
  const out = [];
  for (let k = 0; k < 6; k++) {
    const a = Math.PI / 2 + (k * Math.PI) / 3;
    out.push({ u: r * Math.cos(a), v: r * Math.sin(a) });
  }
  return out;
}

export class Field {
  constructor(scene, physics) {
    this.scene = scene;
    this.physics = physics;
    this.group = new THREE.Group();
    scene.add(this.group);
    this.hubs = {};
    this.outposts = {};
    initMaterials();
  }

  build() {
    this._buildFloor();
    this._buildGuardrails();
    for (const a of [BLUE, RED]) this._buildSide(a);
    this._buildArena();
  }

  // ------------------------------------------------------------------ helpers
  static hubCenter(alliance) {
    const x = HUB.fx - HALF_L;
    return alliance === BLUE ? { x, z: 0 } : { x: -x, z: 0 };
  }

  // x-coordinate of the alliance zone line (world)
  static allianceLineX(alliance) {
    const x = -HALF_L + ALLIANCE_ZONE_DEPTH;
    return alliance === BLUE ? x : -x;
  }

  static inAllianceZone(alliance, x) {
    const lx = Field.allianceLineX(alliance);
    return alliance === BLUE ? x <= lx : x >= lx;
  }

  // Height of the carpet surface (the BUMPS are the only raised driving surface)
  terrainHeight(x, z) {
    const hx = Math.abs(HUB.fx - HALF_L);
    const ax = Math.abs(x);
    const dx = Math.abs(ax - hx);
    if (dx > BUMP.depth / 2) return 0;
    const az = Math.abs(z);
    if (az < HUB.size / 2 || az > HUB.size / 2 + BUMP.width) return 0;
    return BUMP.height * (1 - dx / (BUMP.depth / 2));
  }

  isInsideHubOpening(alliance, x, y, z) {
    const c = Field.hubCenter(alliance);
    return y < HUB.funnelBottomY + FUEL.radius + 0.12 && y > 1.0 && inHexagon(x - c.x, z - c.z, HUB.funnelBottomR + 0.03);
  }

  // Hub light states: 'active' | 'warning' | 'off' | 'chase'
  setHubLights(alliance, mode, t) {
    const h = this.hubs[alliance];
    if (!h) return;
    const col = new THREE.Color(ALLIANCE_COLOR[alliance]);
    let intensity = 0;
    let color = col;
    if (mode === 'active') intensity = 2.2;
    else if (mode === 'warning') intensity = 0.4 + 1.8 * (0.5 + 0.5 * Math.sin(t * 14));
    else if (mode === 'chase') {
      intensity = 2.0;
      color = col.clone().lerp(new THREE.Color(0xffffff), 0.5 + 0.5 * Math.sin(t * 9));
    } else if (mode === 'purple') { color = new THREE.Color(0x9b30ff); intensity = 1.5; }
    else if (mode === 'green') { color = new THREE.Color(0x22ff66); intensity = 1.5; }
    else if (mode === 'white') { color = new THREE.Color(0xffffff); intensity = 1.8; }
    h.lightMat.emissive.copy(color);
    h.lightMat.emissiveIntensity = intensity;
    h.lightMat.color.copy(intensity > 0 ? color : new THREE.Color(0x222222));
  }

  // ------------------------------------------------------------------ floor & carpet
  _buildFloor() {
    const P = this.physics;
    // Floor collider (also under the OUTPOST CORRALS behind the walls)
    P.box(0, -0.5, 0, HALF_L + 3, 0.5, HALF_W + 3, { friction: 0.8, restitution: 0.35 });

    const carpet = new THREE.Mesh(
      new THREE.PlaneGeometry(FIELD_L, FIELD_W),
      new THREE.MeshStandardMaterial({ map: this._carpetTexture(), roughness: 0.95, metalness: 0 }),
    );
    carpet.rotation.x = -Math.PI / 2;
    carpet.receiveShadow = true;
    carpet.userData.floor = true;
    this.group.add(carpet);

    const outer = new THREE.Mesh(
      new THREE.PlaneGeometry(60, 40),
      new THREE.MeshStandardMaterial({ color: 0x23252b, roughness: 1 }),
    );
    outer.rotation.x = -Math.PI / 2;
    outer.position.y = -0.005;
    outer.receiveShadow = true;
    outer.userData.floor = true;
    this.group.add(outer);
  }

  _carpetTexture() {
    const W = 4096, H = Math.round((4096 * FIELD_W) / FIELD_L);
    const c = document.createElement('canvas');
    c.width = W;
    c.height = H;
    const g = c.getContext('2d');
    g.fillStyle = '#62666d';
    g.fillRect(0, 0, W, H);
    // carpet noise
    const img = g.getImageData(0, 0, W, H);
    for (let i = 0; i < img.data.length; i += 4) {
      const n = (Math.random() - 0.5) * 18;
      img.data[i] += n;
      img.data[i + 1] += n;
      img.data[i + 2] += n;
    }
    g.putImageData(img, 0, 0);
    const sx = W / FIELD_L, sy = H / FIELD_W;
    // canvas x = fx, canvas y = fy mirrored so that the texture maps to world (PlaneGeometry v up = -Z)
    const rect = (fx0, fy0, fx1, fy1, col) => {
      g.fillStyle = col;
      g.fillRect(fx0 * sx, (FIELD_W - fy1) * sy, (fx1 - fx0) * sx, (fy1 - fy0) * sy);
    };
    // subtle alliance zone tint
    rect(0, 0, ALLIANCE_ZONE_DEPTH, FIELD_W, 'rgba(47,111,240,0.06)');
    rect(FIELD_L - ALLIANCE_ZONE_DEPTH, 0, FIELD_L, FIELD_W, 'rgba(224,52,52,0.04)');
    const tape = 2 * IN;
    // ROBOT STARTING LINES (edge of each ALLIANCE ZONE)
    rect(ALLIANCE_ZONE_DEPTH - tape, 0, ALLIANCE_ZONE_DEPTH, FIELD_W, '#2f6ff0');
    rect(FIELD_L - ALLIANCE_ZONE_DEPTH, 0, FIELD_L - ALLIANCE_ZONE_DEPTH + tape, FIELD_W, '#e03434');
    // CENTER LINE
    rect(FIELD_L / 2 - tape / 2, 0, FIELD_L / 2 + tape / 2, FIELD_W, '#f2f2f2');
    // DEPOT outlines
    for (const [fx0, fx1, fyc, col] of [
      [0, DEPOT.depth, DEPOT.fy, '#2f6ff0'],
      [FIELD_L - DEPOT.depth, FIELD_L, FIELD_W - DEPOT.fy, '#e03434'],
    ]) {
      g.strokeStyle = col;
      g.lineWidth = 6;
      g.strokeRect(fx0 * sx, (FIELD_W - (fyc + DEPOT.width / 2)) * sy, (fx1 - fx0) * sx, DEPOT.width * sy);
    }
    const t = new THREE.CanvasTexture(c);
    t.colorSpace = THREE.SRGBColorSpace;
    t.anisotropy = 8;
    return t;
  }

  // ------------------------------------------------------------------ guardrails
  _buildGuardrails() {
    const P = this.physics;
    const th = 0.05;
    for (const s of [1, -1]) {
      const z = s * (HALF_W + th / 2);
      // 20in guardrail: FUEL launched above it leaves the FIELD
      P.box(0, GUARDRAIL_H / 2, z, HALF_L + 0.3, GUARDRAIL_H / 2, th / 2, { restitution: 0.45, friction: 0.3 });
      // visuals: polycarbonate panels + aluminum rails/posts
      const panel = new THREE.Mesh(new THREE.BoxGeometry(FIELD_L, GUARDRAIL_H, 0.01), mats.poly);
      panel.position.set(0, GUARDRAIL_H / 2, z);
      this.group.add(panel);
      box(FIELD_L, 0.04, 0.05, mats.alu, 0, GUARDRAIL_H, z, this.group);
      box(FIELD_L, 0.05, 0.05, mats.alu, 0, 0.025, z, this.group);
      const n = 14;
      for (let i = 0; i <= n; i++) {
        const x = -HALF_L + (i * FIELD_L) / n;
        box(0.05, GUARDRAIL_H, 0.05, mats.alu, x, GUARDRAIL_H / 2, z, this.group);
      }
    }
    // Invisible ceiling-high catch walls above the guardrails are NOT added: FUEL can leave the field.
  }

  // ------------------------------------------------------------------ one alliance's half
  _buildSide(alliance) {
    const P = this.physics;
    const s = alliance === BLUE ? 1 : -1;
    const T = (x, z) => ({ x: s * x, z: s * z });
    const q180 = yawQuat(Math.PI);
    const rotQ = (q) => (s === 1 ? q : mulQuat(q180, q));
    const grp = new THREE.Group();
    if (alliance === RED) grp.rotation.y = Math.PI;
    this.group.add(grp);
    const col = mats[alliance];

    const boxC = (cx, cy, cz, hx, hy, hz, opts) => {
      const p = T(cx, cz);
      return P.box(p.x, cy, p.z, hx, hy, hz, opts);
    };
    const hullC = (pts, opts) => {
      const out = [];
      for (let i = 0; i < pts.length; i += 3) out.push(s * pts[i], pts[i + 1], s * pts[i + 2]);
      return P.hull(out, opts);
    };

    const wallX = -HALF_L;
    const zOf = (fy) => HALF_W - fy;

    // ================================================================ HUB
    const hx = HUB.fx - HALF_L;
    const hs = HUB.size / 2;
    const deckY = HUB.rimFront;
    {
      const hub = new THREE.Group();
      hub.position.set(hx, 0, 0);
      grp.add(hub);
      // body: solid below the funnel, thin side panels up to the deck so the funnel stays visible
      box(HUB.size, HUB.funnelBottomY, HUB.size, mats.black, 0, HUB.funnelBottomY / 2, 0, hub);
      const sideH = deckY - HUB.funnelBottomY - 0.004;
      const sideY = HUB.funnelBottomY + sideH / 2;
      box(0.03, sideH, HUB.size, mats.black, hs - 0.015, sideY, 0, hub);
      box(0.03, sideH, HUB.size, mats.black, -hs + 0.015, sideY, 0, hub);
      box(HUB.size, sideH, 0.03, mats.black, 0, sideY, hs - 0.015, hub);
      box(HUB.size, sideH, 0.03, mats.black, 0, sideY, -hs + 0.015, hub);
      // alliance-colored lower trim and REBUILT panels
      for (const [px, pz, ry] of [[-hs - 0.002, 0, -Math.PI / 2], [0, hs + 0.002, 0], [0, -hs - 0.002, Math.PI]]) {
        const p = new THREE.Mesh(new THREE.PlaneGeometry(HUB.size * 0.8, 0.55), new THREE.MeshStandardMaterial({ color: 0x3f7fa8, roughness: 0.6 }));
        p.position.set(px, 0.85, pz);
        p.rotation.y = ry;
        hub.add(p);
        const b = p.clone();
        b.material = col;
        b.scale.set(1.25, 0.18, 1);
        b.position.y = 0.12;
        hub.add(b);
      }
      // AprilTags on the hub faces
      for (const [px, pz, ry] of [[-hs - 0.004, -0.18, -Math.PI / 2], [-hs - 0.004, 0.18, -Math.PI / 2], [hs + 0.004, -0.18, Math.PI / 2], [hs + 0.004, 0.18, Math.PI / 2], [0.18, hs + 0.004, 0], [-0.18, -hs - 0.004, Math.PI]]) {
        const tag = new THREE.Mesh(new THREE.PlaneGeometry(0.2, 0.2), mats.tag);
        tag.position.set(px, 1.124, pz);
        tag.rotation.y = ry;
        hub.add(tag);
      }
      // exit opening in the NEUTRAL ZONE face, with the white HDPE ramp lip FUEL rolls off
      const exH = HUB.exitTop - HUB.exitHeight;
      box(0.02, exH, HUB.exitWidth, mats.dark, hs + 0.004, HUB.exitHeight + exH / 2, 0, hub, false);
      box(0.1, 0.012, HUB.exitWidth, mats.white, hs + 0.04, HUB.exitHeight - 0.006, 0, hub, false);
      for (const sz of [-1, 1]) box(0.03, exH + 0.03, 0.03, mats.alu, hs + 0.01, HUB.exitHeight + exH / 2, sz * (HUB.exitWidth / 2 + 0.015), hub, false);
      box(0.03, 0.03, HUB.exitWidth + 0.06, mats.alu, hs + 0.01, HUB.exitTop + 0.015, 0, hub, false);

      // ---- funnel / deck (trimesh used for both rendering and collision)
      const rim = hexVerts(HUB.hexR);
      const bot = hexVerts(HUB.funnelBottomR);
      const inr = HUB.hexR * Math.cos(Math.PI / 6);
      const rimY = (u) => HUB.rimFront + ((HUB.rimBack - HUB.rimFront) * (u + inr)) / (2 * inr);
      const verts = [];
      const idx = [];
      const addV = (u, y, v) => (verts.push(u, y, v), verts.length / 3 - 1);
      const rimI = rim.map((p) => addV(p.u, rimY(p.u), p.v));
      const deckI = rim.map((p) => addV(p.u, deckY, p.v));
      const botI = bot.map((p) => addV(p.u, HUB.funnelBottomY, p.v));
      for (let k = 0; k < 6; k++) {
        const k2 = (k + 1) % 6;
        idx.push(rimI[k], botI[k], botI[k2], rimI[k], botI[k2], rimI[k2]); // funnel wall
        idx.push(deckI[k], rimI[k], rimI[k2], deckI[k], rimI[k2], deckI[k2]); // lip
      }
      // deck ring between the hexagon and the 47in square
      const sq = [[hs, hs], [-hs, hs], [-hs, -hs], [hs, -hs]].map(([u, v]) => ({ u, v, a: Math.atan2(v, u) }));
      const proj = rim.map((p) => {
        const a = Math.atan2(p.v, p.u);
        const t = hs / Math.max(Math.abs(Math.cos(a)), Math.abs(Math.sin(a)));
        return { u: t * Math.cos(a), v: t * Math.sin(a), a };
      });
      const projI = proj.map((p) => addV(p.u, deckY, p.v));
      const sqI = sq.map((p) => addV(p.u, deckY, p.v));
      const norm = (a) => (a < 0 ? a + 2 * Math.PI : a);
      for (let k = 0; k < 6; k++) {
        const k2 = (k + 1) % 6;
        let a0 = norm(proj[k].a), a1 = norm(proj[k2].a);
        if (a1 <= a0) a1 += 2 * Math.PI;
        const poly = [deckI[k], deckI[k2], projI[k2]];
        const corners = [];
        for (let c = 0; c < 4; c++) {
          let ac = norm(sq[c].a);
          if (ac < a0) ac += 2 * Math.PI;
          if (ac > a0 && ac < a1) corners.push({ i: sqI[c], a: ac });
        }
        corners.sort((p, q) => q.a - p.a);
        for (const c of corners) poly.push(c.i);
        poly.push(projI[k]);
        for (let j = 1; j < poly.length - 1; j++) idx.push(poly[0], poly[j], poly[j + 1]);
      }
      // drop zero-area triangles (the lip has no height along the front edge)
      const clean = [];
      for (let i = 0; i < idx.length; i += 3) {
        const [a, b, c] = [idx[i] * 3, idx[i + 1] * 3, idx[i + 2] * 3];
        const e1 = new THREE.Vector3(verts[b] - verts[a], verts[b + 1] - verts[a + 1], verts[b + 2] - verts[a + 2]);
        const e2 = new THREE.Vector3(verts[c] - verts[a], verts[c + 1] - verts[a + 1], verts[c + 2] - verts[a + 2]);
        if (e1.cross(e2).length() > 1e-6) clean.push(idx[i], idx[i + 1], idx[i + 2]);
      }
      idx.length = 0;
      idx.push(...clean);
      const geo = new THREE.BufferGeometry();
      geo.setAttribute('position', new THREE.Float32BufferAttribute(verts, 3));
      geo.setIndex(idx);
      geo.computeVertexNormals();
      const top = new THREE.Mesh(geo, mats.funnel);
      top.castShadow = true;
      hub.add(top);
      // physics trimesh in alliance world coords
      const wv = [];
      for (let i = 0; i < verts.length; i += 3) {
        const p = T(hx + verts[i], verts[i + 2]);
        wv.push(p.x, verts[i + 1], p.z);
      }
      P.trimesh(wv, idx, { restitution: 0.35, friction: 0.35 });

      // lower solid body + perimeter walls up to the deck
      boxC(hx, HUB.funnelBottomY / 2, 0, hs, HUB.funnelBottomY / 2, hs, { restitution: 0.4 });
      const wallH = (deckY - HUB.funnelBottomY) / 2;
      const wy = HUB.funnelBottomY + wallH;
      boxC(hx + hs - 0.03, wy, 0, 0.03, wallH, hs, {});
      boxC(hx - hs + 0.03, wy, 0, 0.03, wallH, hs, {});
      boxC(hx, wy, hs - 0.03, hs, wallH, 0.03, {});
      boxC(hx, wy, -hs + 0.03, hs, wallH, 0.03, {});

      // light bars along the top edges
      const lightMat = new THREE.MeshStandardMaterial({ color: 0x222222, emissive: ALLIANCE_COLOR[alliance], emissiveIntensity: 0, roughness: 0.4 });
      for (const [w, d, px, pz] of [[HUB.size, 0.04, 0, hs], [HUB.size, 0.04, 0, -hs], [0.04, HUB.size, hs, 0], [0.04, HUB.size, -hs, 0]]) {
        box(w + 0.01, 0.05, d + 0.01, lightMat, px, deckY - 0.03, pz, hub, false);
      }
      // the real LED diffusers wrap the HUB below its roof (GE-26309/10/11); with the field CAD
      // shown these glow on top of it
      const dy = (HUB.lightY0 + HUB.lightY1) / 2, dh = HUB.lightY1 - HUB.lightY0, o = hs + 0.012;
      for (const [w, d, px, pz] of [[HUB.size - 0.02, 0.012, 0, o], [HUB.size - 0.02, 0.012, 0, -o], [0.012, HUB.size - 0.02, o, 0], [0.012, HUB.size - 0.02, -o, 0]]) {
        const m = box(w, dh, d, lightMat, px, dy, pz, hub, false);
        m.userData.keepWithCad = true;
        m.userData.cadOnly = true; // shown by cadModels.js when the field CAD loads
        m.visible = false;
      }
      // angled top light bars following the rim
      for (let k = 0; k < 6; k++) {
        const p0 = rim[k], p1 = rim[(k + 1) % 6];
        const a = new THREE.Vector3(p0.u, rimY(p0.u) + 0.01, p0.v);
        const b = new THREE.Vector3(p1.u, rimY(p1.u) + 0.01, p1.v);
        const len = a.distanceTo(b);
        const bar = new THREE.Mesh(new THREE.BoxGeometry(len, 0.025, 0.025), lightMat);
        bar.position.copy(a).add(b).multiplyScalar(0.5);
        bar.quaternion.setFromUnitVectors(new THREE.Vector3(1, 0, 0), b.clone().sub(a).normalize());
        hub.add(bar);
      }

      // NET in the back of the HUB (toward the NEUTRAL ZONE): a vertical back sheet on a U frame
      // with triangular side sheets up to the leaning poles
      const nx = hs + HUB.netOut, nw = HUB.netWidth / 2;
      const nH = HUB.netTop - HUB.netBottom, nY = HUB.netBottom + nH / 2;
      const net = new THREE.Mesh(new THREE.PlaneGeometry(HUB.netWidth, nH), mats.net);
      net.position.set(nx, nY, 0);
      net.rotation.y = Math.PI / 2;
      hub.add(net);
      const tube = (a, b) => {
        const len = a.distanceTo(b);
        const c = cyl(0.016, len, mats.alu, hub, 8);
        c.position.copy(a).add(b).multiplyScalar(0.5);
        c.quaternion.setFromUnitVectors(new THREE.Vector3(0, 1, 0), b.clone().sub(a).normalize());
      };
      const V = (x, y, z) => new THREE.Vector3(x, y, z);
      tube(V(nx, HUB.netTop, -nw), V(nx, HUB.netTop, nw));
      for (const sz of [-1, 1]) {
        const z = sz * nw;
        tube(V(nx, HUB.netBottom, z), V(nx, HUB.netTop, z));
        tube(V(0, HUB.netBottom, z), V(nx, HUB.netBottom, z)); // U frame
        tube(V(-hs * 0.3, deckY, sz * (hs - 0.05)), V(nx, HUB.netTop, z)); // leaning pole
        const g = new THREE.BufferGeometry().setFromPoints([V(0, HUB.netBottom, z), V(nx, HUB.netBottom, z), V(nx, HUB.netTop, z)]);
        g.setAttribute('uv', new THREE.Float32BufferAttribute([0, 0, 1, 0, 1, 1], 2));
        g.computeVertexNormals();
        const side = new THREE.Mesh(g, mats.net);
        hub.add(side);
      }
      tube(V(nx, HUB.netBottom, -nw), V(nx, HUB.netBottom, nw));
      const nc = T(hx + nx, 0);
      P.box(nc.x, nY, nc.z, 0.02, nH / 2, nw, { restitution: 0.15, friction: 0.6 });

      this.hubs[alliance] = { center: Field.hubCenter(alliance), lightMat, group: hub };
    }

    // ================================================================ BUMPS
    for (const sz of [1, -1]) {
      const z0 = sz * hs, z1 = sz * (hs + BUMP.width);
      const zl = Math.min(z0, z1), zh = Math.max(z0, z1);
      const d = BUMP.depth / 2, H = BUMP.height;
      const shape = new THREE.Shape();
      shape.moveTo(-d, 0);
      shape.lineTo(0, H);
      shape.lineTo(d, 0);
      shape.lineTo(-d, 0);
      const geo = new THREE.ExtrudeGeometry(shape, { depth: zh - zl, bevelEnabled: false });
      const m = new THREE.Mesh(geo, mats[alliance + 'Bump']);
      m.position.set(hx, 0, zl);
      m.castShadow = m.receiveShadow = true;
      grp.add(m);
      // aluminum edge rails
      box(BUMP.depth, 0.02, 0.03, mats.alu, hx, 0.01, zl, grp);
      box(BUMP.depth, 0.02, 0.03, mats.alu, hx, 0.01, zh, grp);
      const pts = [];
      for (const z of [zl, zh]) pts.push(hx - d, 0, z, hx, H, z, hx + d, 0, z, hx - d, -0.02, z, hx + d, -0.02, z);
      hullC(pts, { friction: 0.9, restitution: 0.2, groups: P.terrainGroups });
    }

    // ================================================================ TRENCHES
    for (const sz of [1, -1]) {
      const zGuard = sz * HALF_W;
      const zOpenEnd = sz * (HALF_W - TRENCH.clearWidth);
      const zPostEnd = sz * (HALF_W - TRENCH.width);
      const postW = Math.abs(zOpenEnd - zPostEnd);
      const postZ = (zOpenEnd + zPostEnd) / 2;
      const td = TRENCH.depth / 2;
      // bump-side post (full height)
      box(TRENCH.depth, TRENCH.height, postW, mats.dark, hx, TRENCH.height / 2, postZ, grp);
      box(TRENCH.depth + 0.01, 0.1, postW + 0.01, col, hx, TRENCH.height - 0.05, postZ, grp);
      boxC(hx, TRENCH.height / 2, postZ, td, TRENCH.height / 2, postW / 2, { restitution: 0.4 });
      // guardrail-side upright
      box(0.08, TRENCH.height, 0.08, mats.alu, hx, TRENCH.height / 2, zGuard - sz * 0.04, grp);
      // arm spanning the opening at 22.25in - 40.25in
      const armH = TRENCH.height - TRENCH.clearHeight;
      const armZ = (zGuard + zOpenEnd) / 2;
      box(TRENCH.armThick, armH, TRENCH.clearWidth, col, hx, TRENCH.clearHeight + armH / 2, armZ, grp);
      box(TRENCH.armThick + 0.02, 0.03, TRENCH.clearWidth, mats.alu, hx, TRENCH.clearHeight + 0.015, armZ, grp);
      boxC(hx, TRENCH.clearHeight + armH / 2, armZ, TRENCH.armThick / 2, armH / 2, TRENCH.clearWidth / 2, { restitution: 0.4 });
      // floor rails (feet) of the trench, too low to matter for driving
      for (const fx of [-td + 0.02, td - 0.02]) box(0.04, 0.012, TRENCH.clearWidth, mats.alu, hx + fx, 0.006, armZ, grp, false);
      // AprilTags on the arm
      for (const sx of [-1, 1]) {
        const tag = new THREE.Mesh(new THREE.PlaneGeometry(0.2, 0.2), mats.tag);
        tag.position.set(hx + sx * (TRENCH.armThick / 2 + 0.003), 0.889, armZ);
        tag.rotation.y = sx * Math.PI / 2;
        grp.add(tag);
      }
    }

    // ================================================================ ALLIANCE WALL
    const wallT = 0.12;
    const wx = wallX - wallT / 2;
    const outZc = zOf(OUTPOST.fy);
    const outEdge = zOf(OUTPOST.fy + OUTPOST.structW / 2 + 0.03); // DS side edge of outpost
    const towerZc = zOf(TOWER.fy);
    const towerZ0 = towerZc - TOWER.width / 2, towerZ1 = towerZc + TOWER.width / 2;
    // DS / tower-wall segments (z ranges), from -HALF_W to outEdge
    const segs = [
      { z0: -HALF_W, z1: zOf(6.33), kind: 'ds' },
      { z0: zOf(6.33), z1: towerZ0, kind: 'ds' },
      { z0: towerZ0, z1: towerZ1, kind: 'tower' },
      { z0: towerZ1, z1: outEdge, kind: 'ds' },
    ];
    for (const sg of segs) {
      const w = sg.z1 - sg.z0, zc = (sg.z0 + sg.z1) / 2;
      boxC(wx, ALLIANCE_WALL_H / 2, zc, wallT / 2, ALLIANCE_WALL_H / 2, w / 2, { restitution: 0.4 });
      if (sg.kind === 'ds') {
        box(wallT, DS_BASE_H, w - 0.02, mats.diamond, wx, DS_BASE_H / 2, zc, grp);
        const glass = new THREE.Mesh(new THREE.BoxGeometry(0.012, DS_GLASS_H, w - 0.04), mats.poly);
        glass.position.set(wx, DS_BASE_H + DS_GLASS_H / 2, zc);
        grp.add(glass);
        box(0.06, 0.05, w, col, wx, ALLIANCE_WALL_H, zc, grp);
        box(0.06, ALLIANCE_WALL_H, 0.05, mats.alu, wx, ALLIANCE_WALL_H / 2, sg.z0 + 0.025, grp);
        // shelf behind the glass
        box(0.31, 0.02, Math.min(1.75, w - 0.1), mats.alu, wx - 0.2, DS_BASE_H, zc, grp);
      } else {
        box(wallT, TOWER.height, w, mats.black, wx, TOWER.height / 2, zc, grp);
        const banner = new THREE.Mesh(new THREE.PlaneGeometry(w * 0.8, 0.8), new THREE.MeshStandardMaterial({ color: 0x3f7fa8, roughness: 0.6 }));
        banner.position.set(wallX + 0.002, 1.45, zc);
        banner.rotation.y = Math.PI / 2;
        grp.add(banner);
        for (const dz of [-0.2159, 0.2159]) {
          const tag = new THREE.Mesh(new THREE.PlaneGeometry(0.2, 0.2), mats.tag);
          tag.position.set(wallX + 0.003, 0.5525, zc + dz);
          tag.rotation.y = Math.PI / 2;
          grp.add(tag);
        }
      }
    }

    // ================================================================ TOWER
    {
      const baseX = wallX + TOWER.baseDepth / 2;
      box(TOWER.baseDepth, TOWER.baseThick, TOWER.baseWidth, mats.black, baseX, TOWER.baseThick / 2, towerZc, grp);
      boxC(baseX, TOWER.baseThick / 2, towerZc, TOWER.baseDepth / 2, TOWER.baseThick / 2, TOWER.baseWidth / 2, { friction: 0.8, groups: P.terrainGroups });
      const ux = wallX + TOWER.uprightFx;
      const half = TOWER.uprightGap / 2 + TOWER.uprightThick / 2;
      const tmat = mats[alliance + 'Tower'];
      for (const sz of [-1, 1]) {
        const uz = towerZc + sz * half;
        box(TOWER.uprightDeep, TOWER.uprightHeight, TOWER.uprightThick, tmat, ux, TOWER.uprightHeight / 2, uz, grp);
        boxC(ux, TOWER.uprightHeight / 2, uz, TOWER.uprightDeep / 2, TOWER.uprightHeight / 2, TOWER.uprightThick / 2, { restitution: 0.3 });
        // support structure from the upright back to the TOWER WALL
        const sLen = ux - wallX;
        box(sLen, 0.05, 0.04, mats.steel, wallX + sLen / 2, TOWER.supportLow, uz, grp);
        boxC(wallX + sLen / 2, TOWER.supportLow, uz, sLen / 2, 0.025, 0.02, {});
        const dLen = Math.hypot(sLen, TOWER.supportHigh - TOWER.supportLow);
        const diag = new THREE.Mesh(new THREE.BoxGeometry(dLen, 0.04, 0.035), mats.steel);
        diag.position.set(wallX + sLen / 2, (TOWER.supportLow + TOWER.supportHigh) / 2, uz);
        diag.rotation.z = -Math.atan2(TOWER.supportHigh - TOWER.supportLow, sLen);
        diag.castShadow = true;
        grp.add(diag);
      }
      const rungHalf = half + TOWER.uprightThick / 2 + TOWER.rungExt;
      for (const ry of TOWER.rungs) {
        const r = cyl(TOWER.rungR, rungHalf * 2, tmat, grp, 14);
        r.rotation.x = Math.PI / 2;
        r.position.set(ux, ry, towerZc);
        const p1 = T(ux, towerZc - rungHalf), p2 = T(ux, towerZc + rungHalf);
        P.pipe(p1.x, ry, p1.z, p2.x, p2.z, TOWER.rungR, { restitution: 0.3 });
      }
    }

    // ================================================================ DEPOT
    {
      const zc = zOf(DEPOT.fy);
      const bw = DEPOT.barrierW, bh = DEPOT.barrierH;
      const fxEnd = wallX + DEPOT.depth;
      const bars = [
        [fxEnd - bw / 2, zc, bw, DEPOT.width],
        [wallX + DEPOT.depth / 2, zc - DEPOT.width / 2 + bw / 2, DEPOT.depth, bw],
        [wallX + DEPOT.depth / 2, zc + DEPOT.width / 2 - bw / 2, DEPOT.depth, bw],
      ];
      for (const [x, z, w, d] of bars) {
        box(w, bh, d, col, x, bh / 2, z, grp);
        boxC(x, bh / 2, z, w / 2, bh / 2, d / 2, { friction: 0.6, groups: P.terrainGroups });
      }
    }

    // ================================================================ OUTPOST
    {
      const zc = outZc;
      const ow = OUTPOST.structW;
      const zA = HALF_W; // guardrail side
      const zB = outEdge;
      const H = ALLIANCE_WALL_H;
      const baseHalf = OUTPOST.baseW / 2, upHalf = OUTPOST.upperW / 2;
      const baseTop = OUTPOST.baseY + OUTPOST.baseH;
      const upTop = OUTPOST.upperY + OUTPOST.upperH;
      // wall pieces around the two openings
      const pieces = [
        // [z0, z1, y0, y1]
        [zc + baseHalf, zA, 0, H],             // guardrail side of openings
        [zB, zc - baseHalf, 0, H],             // DS side
        [zc - baseHalf, zc + baseHalf, 0, OUTPOST.baseY],        // lip under base opening
        [zc - baseHalf, zc + baseHalf, baseTop, OUTPOST.upperY], // between openings
        [zc - baseHalf, zc + baseHalf, upTop, H],                 // above upper opening
      ];
      const face = new THREE.MeshStandardMaterial({ color: 0x9aa0a8, roughness: 0.35, metalness: 0.8 });
      for (const [z0, z1, y0, y1] of pieces) {
        const zl = Math.min(z0, z1), zh = Math.max(z0, z1);
        if (zh - zl < 1e-3 || y1 - y0 < 1e-3) continue;
        box(wallT, y1 - y0, zh - zl, face, wx, (y0 + y1) / 2, (zl + zh) / 2, grp);
        boxC(wx, (y0 + y1) / 2, (zl + zh) / 2, wallT / 2, (y1 - y0) / 2, (zh - zl) / 2, { restitution: 0.35 });
      }
      box(0.06, 0.05, Math.abs(zA - zB), col, wx, H, (zA + zB) / 2, grp);
      // divider pipe in the base opening
      const pipe = cyl(0.021, OUTPOST.baseH, mats.steel, grp, 10);
      pipe.position.set(wallX - 0.02, OUTPOST.baseY + OUTPOST.baseH / 2, zc);
      boxC(wallX - 0.02, OUTPOST.baseY + OUTPOST.baseH / 2, zc, 0.021, OUTPOST.baseH / 2, 0.021, {});
      // robot-only barrier across the base opening (FUEL passes, ROBOTS cannot enter)
      const bp = T(wallX + 0.01, zc);
      P.box(bp.x, 0.3, bp.z, 0.01, 0.3, baseHalf, { groups: groups(GROUP.ROBOT_BARRIER, GROUP.ROBOT | GROUP.WHEEL | GROUP.INTAKE) });
      // AprilTags
      for (const dz of [-0.2159, 0.2159]) {
        const tag = new THREE.Mesh(new THREE.PlaneGeometry(0.2, 0.2), mats.tag);
        tag.position.set(wallX + 0.003, 0.5525, zc + dz);
        tag.rotation.y = Math.PI / 2;
        grp.add(tag);
      }
      // CORRAL behind the base opening
      const cx0 = wx - wallT / 2, cx1 = cx0 - OUTPOST.corralD;
      const cz0 = zc - OUTPOST.corralW / 2, cz1 = zc + OUTPOST.corralW / 2;
      const cwh = OUTPOST.corralWallH;
      for (const [x, z, w, d] of [
        [cx1 - 0.01, zc, 0.02, OUTPOST.corralW],
        [(cx0 + cx1) / 2, cz0 - 0.01, OUTPOST.corralD, 0.02],
        [(cx0 + cx1) / 2, cz1 + 0.01, OUTPOST.corralD, 0.02],
      ]) {
        const pm = new THREE.Mesh(new THREE.BoxGeometry(w, cwh, d), mats.poly);
        pm.position.set(x, cwh / 2, z);
        grp.add(pm);
        boxC(x, cwh / 2, z, w / 2, cwh / 2, d / 2, { restitution: 0.3 });
      }
      // CHUTE: sloped tunnel rising away from the field behind the upper opening
      const slope = (OUTPOST.chuteSlopeDeg * Math.PI) / 180;
      const chuteLen = 1.0;
      const chute = new THREE.Group();
      chute.position.set(wx - wallT / 2, OUTPOST.upperY, zc);
      chute.rotation.z = -slope;
      grp.add(chute);
      const floorMesh = new THREE.Mesh(new THREE.BoxGeometry(chuteLen, 0.01, OUTPOST.upperW), mats.poly);
      floorMesh.position.set(-chuteLen / 2, 0, 0);
      chute.add(floorMesh);
      for (const sz2 of [-1, 1]) {
        const side = new THREE.Mesh(new THREE.BoxGeometry(chuteLen, OUTPOST.upperH + 0.05, 0.01), mats.poly);
        side.position.set(-chuteLen / 2, OUTPOST.upperH / 2, sz2 * OUTPOST.upperW / 2);
        chute.add(side);
      }
      box(0.05, 0.05, OUTPOST.upperW + 0.04, mats.alu, -chuteLen, -0.04, 0, chute);
      // CHUTE DOOR (HDPE arm on a pivot)
      const door = new THREE.Mesh(new THREE.BoxGeometry(0.02, OUTPOST.upperH + 0.02, OUTPOST.upperW), new THREE.MeshStandardMaterial({ color: 0xf5f5f5, roughness: 0.7 }));
      door.position.set(-0.03, OUTPOST.upperH / 2, 0);
      chute.add(door);
      door.userData.keepWithCad = true; // shows the chute state
      // chute slots for the 24/25 staged FUEL (5 wide x 5 deep)
      const slots = [];
      const r = FUEL.radius;
      for (let row = 0; row < 5; row++) {
        for (let c = 0; c < 5; c++) {
          const lx = -0.09 - row * (2 * r + 0.004);
          const lz = (c - 2) * (2 * r + 0.004);
          const local = new THREE.Vector3(lx, r + 0.006, lz);
          slots.push({ local, chute });
        }
      }
      // HUMAN PLAYER standing in the OUTPOST AREA
      const hp = new THREE.Group();
      const shirt = new THREE.MeshStandardMaterial({ color: ALLIANCE_COLOR[alliance], roughness: 0.8 });
      const skin = new THREE.MeshStandardMaterial({ color: 0xd6a57c, roughness: 0.8 });
      const legs = new THREE.MeshStandardMaterial({ color: 0x2a2f3a, roughness: 0.9 });
      const torso = new THREE.Mesh(new THREE.CapsuleGeometry(0.17, 0.45, 4, 10), shirt);
      torso.position.y = 1.2;
      const head = new THREE.Mesh(new THREE.SphereGeometry(0.11, 14, 10), skin);
      head.position.y = 1.62;
      const leg = new THREE.Mesh(new THREE.CapsuleGeometry(0.12, 0.7, 4, 8), legs);
      leg.position.y = 0.5;
      const arm = new THREE.Mesh(new THREE.CapsuleGeometry(0.05, 0.5, 4, 8), shirt);
      arm.position.set(0.12, 1.35, 0.2);
      arm.rotation.z = -0.6;
      for (const m of [torso, head, leg, arm]) { m.castShadow = true; hp.add(m); }
      hp.position.set(wallX - 1.25, 0, zc - 0.55);
      hp.rotation.y = 0;
      hp.userData.keepWithCad = true; // not in the field CAD
      grp.add(hp);

      this.outposts[alliance] = {
        chuteSlots: slots,
        door,
        hpArm: arm,
        hp,
        // world-space info
        upperOpening: { ...T(wallX + 0.06, zc), y: OUTPOST.upperY + FUEL.radius + 0.01, halfW: upHalf - FUEL.radius },
        corral: (() => {
          const a = T(cx1, cz0), b = T(cx0, cz1);
          return { x0: Math.min(a.x, b.x), x1: Math.max(a.x, b.x), z0: Math.min(a.z, b.z), z1: Math.max(a.z, b.z) };
        })(),
        hpThrowPos: (() => { const p = T(wallX - 1.1, zc - 0.45); return { x: p.x, y: 2.05, z: p.z }; })(),
        intoField: s, // +x direction multiplier into the field
      };
    }
  }

  // ------------------------------------------------------------------ arena dressing
  _buildArena() {
    // bleachers / backdrop blocks for depth cues
    const standMat = new THREE.MeshStandardMaterial({ color: 0x1a1d24, roughness: 1 });
    for (const s of [1, -1]) {
      for (let i = 0; i < 4; i++) {
        box(FIELD_L + 8, 0.6, 1.2, standMat, 0, 0.3 + i * 0.6, s * (HALF_W + 3.2 + i * 1.2), this.group, false).userData.keepWithCad = true;
      }
    }
    // scoring table
    box(4, 0.8, 0.8, new THREE.MeshStandardMaterial({ color: 0x33363d, roughness: 0.8 }), 0, 0.4, HALF_W + 1.4, this.group);
  }

  // --------------------------------------------------------------- staging positions
  neutralSlots(count) {
    const r = FUEL.radius;
    const cols = 34, rows = 12;
    const dz = 0.1535, dx = 2 * r + 0.004;
    const slots = [];
    for (let row = 0; row < rows; row++) {
      const side = row < 6 ? -1 : 1;
      const k = row % 6;
      const x = side * (0.0254 + r + 0.002 + k * dx);
      for (let c = 0; c < cols; c++) {
        const z = (c - (cols - 1) / 2) * dz;
        slots.push({ x, y: r + 0.002, z, edge: Math.min(c, cols - 1 - c) + Math.min(k, 5 - k) * 0.5 });
      }
    }
    // remove from the outer corners first when fewer than 408 are staged
    slots.sort((a, b) => a.edge - b.edge);
    return slots.slice(slots.length - count);
  }

  depotSlots(alliance) {
    const r = FUEL.radius;
    const zc = HALF_W - DEPOT.fy;
    const slots = [];
    for (let i = 0; i < 4; i++) {
      for (let j = 0; j < 6; j++) {
        const x = -HALF_L + 0.004 + r + i * (2 * r + 0.001);
        const z = zc + (j - 2.5) * (2 * r + 0.002);
        slots.push(alliance === BLUE ? { x, y: r + 0.002, z } : { x: -x, y: r + 0.002, z: -z });
      }
    }
    return slots;
  }
}
