// Drawing the energy chains (js/echain.js) on 8793's and 4414's models: every link one instance of
// one mesh (WCP's link, js/echainLink.js), placed on the pins the chain's kinematics put out for the
// turret's yaw, and the pieces that hold its ends.
import * as THREE from 'three';
import { LINK_POS, LINK_IDX } from './echainLink.js';
import { TrackChain, SpoolChain, LINK } from './echain.js';

const DEG = Math.PI / 180;

// the link's geometry; `width` (m, along its pins) other than the STEP's 24.2 mm moves its side
// plates apart (a taller chain of the same family: the cross bars longer)
const linkGeos = new Map();
export function linkGeometry(width = LINK.width) {
  const key = width.toFixed(5);
  if (linkGeos.has(key)) return linkGeos.get(key);
  const extra = (width - LINK.width) / 2, pos = new Float32Array(LINK_POS.length);
  for (let i = 0; i < LINK_POS.length; i += 3) {
    const y = LINK_POS[i + 1] / 1e5;
    pos[i] = LINK_POS[i] / 1e5;
    pos[i + 1] = y + Math.sign(y) * extra;
    pos[i + 2] = LINK_POS[i + 2] / 1e5;
  }
  let g = new THREE.BufferGeometry();
  g.setAttribute('position', new THREE.BufferAttribute(pos, 3));
  g.setIndex(new THREE.BufferAttribute(LINK_IDX, 1));
  g = g.toNonIndexed(); // (flat faces: a molded part's crisp edges)
  g.computeVertexNormals();
  linkGeos.set(key, g);
  return g;
}

// n links, one InstancedMesh; place(pins, y) puts link k between pins k and k+1 ([x, w] pairs, w =
// -z), its middle at height y
export function chainMesh(n, width, material) {
  const mesh = new THREE.InstancedMesh(linkGeometry(width), material, n);
  mesh.name = 'echain links';
  mesh.castShadow = true; mesh.receiveShadow = true;
  mesh.frustumCulled = false;
  const m4 = new THREE.Matrix4(), q = new THREE.Quaternion(), v = new THREE.Vector3(), one = new THREE.Vector3(1, 1, 1), up = new THREE.Vector3(0, 1, 0);
  const place = (pins, y, from = 0, ys = null) => {
    for (let k = from; k < n; k++) {
      const x0 = pins[2 * k], w0 = pins[2 * k + 1], x1 = pins[2 * k + 2], w1 = pins[2 * k + 3];
      q.setFromAxisAngle(up, Math.atan2(w1 - w0, x1 - x0));
      v.set((x0 + x1) / 2, ys ? ys(k) : y, -(w0 + w1) / 2);
      m4.compose(v, q, one);
      mesh.setMatrixAt(k, m4);
    }
    mesh.instanceMatrix.needsUpdate = true;
  };
  return { mesh, place };
}

const box = (w, h, d, mat, parent, x = 0, y = 0, z = 0) => { const m = new THREE.Mesh(new THREE.BoxGeometry(w, h, d), mat); m.position.set(x, y, z); m.castShadow = true; m.receiveShadow = true; parent.add(m); return m; };
const cyl = (r, h, mat, parent, x = 0, y = 0, z = 0, seg = 12) => { const m = new THREE.Mesh(new THREE.CylinderGeometry(r, r, h, seg), mat); m.position.set(x, y, z); m.castShadow = true; parent.add(m); return m; };

// An end bracket of the chain's family, as a group in its own frame: x along the chain (the pin at
// 0, the bracket's body from x0 to x1), y along the pins, z across (+ the side it's fastened on: a
// plate there, with two screws through it)
function endBracket(width, x0, x1, mat, screwMat, { plate = 'z' } = {}) {
  const g = new THREE.Group(), h = LINK.height, t = 0.0025, len = x1 - x0, mid = (x0 + x1) / 2;
  // the side plates either side (round round the pin) and a floor across
  for (const s of [-1, 1]) {
    box(len, t, h, mat, g, mid, s * (width / 2 - t / 2), 0);
    const end = new THREE.Mesh(new THREE.CylinderGeometry(h / 2, h / 2, t, 16), mat);
    end.position.set(0, s * (width / 2 - t / 2), 0);
    g.add(end);
  }
  if (plate === 'z') {
    box(len, width, t, mat, g, mid, 0, h / 2 - t / 2);
    for (const f of [0.3, 0.75]) { const sc = cyl(0.0035, 0.003, screwMat, g, x0 + len * f, 0, h / 2 + 0.0015, 10); sc.rotation.x = Math.PI / 2; }
  } else {
    box(len, t, h, mat, g, mid, width / 2 + t / 2, 0);
    for (const f of [0.3, 0.75]) cyl(0.0035, 0.003, screwMat, g, x0 + len * f, width / 2 + t + 0.0015, 0, 10);
  }
  box(len * 0.35, width - 2 * t, h * 0.7, mat, g, x0 + len * 0.2, 0, 0);
  return g;
}

// ---------------------------------------------------------------- 8793
// The chain in its track (cad/robots/8793-track.glb, fixed): parent is the robot's root, the track's
// axis at axis [x, z]. The fixed end's bracket is screwed through the outer ring at the wedge's step,
// the moving end's hangs on a block under the back-right corner of the shooter's base plate.
// update(yaw) moves the links.
export function chain8793(parent, axis, { linkMat, bracketMat, screwMat } = {}) {
  const ch = new TrackChain(), o = ch.o;
  const g = new THREE.Group();
  g.name = 'echain 8793';
  g.position.set(axis[0], 0, axis[1]);
  parent.add(g);
  linkMat ||= new THREE.MeshStandardMaterial({ color: 0x1c1d21, roughness: 0.62, metalness: 0.05 });
  bracketMat ||= linkMat;
  screwMat ||= new THREE.MeshStandardMaterial({ color: 0xb8bcc2, roughness: 0.35, metalness: 0.85 });
  const y = o.floor + LINK.width / 2;
  const links = chainMesh(ch.n, LINK.width, linkMat);
  g.add(links.mesh);
  // the fixed end: from its pin back along the outer wall to the wedge's step, fastened to the wall
  const fixed = endBracket(LINK.width, -o.ringBracket, 0.01, bracketMat, screwMat);
  fixed.name = 'echain 8793 ring mount';
  g.add(fixed);
  {
    const a = ch.aF, x = ch.ro * Math.cos(a), w = ch.ro * Math.sin(a), hd = a + Math.PI / 2;
    fixed.position.set(x, y, -w);
    fixed.rotation.y = hd;
    // the screws' heads outside the outer ring (drilled after the fact)
    for (const f of [0.35, 0.8]) {
      const aa = a - (f * o.ringBracket) / ch.ro, r = o.rOut + 0.0135;
      const s = cyl(0.0042, 0.003, screwMat, g, r * Math.cos(aa), y, -r * Math.sin(aa), 10);
      s.rotation.z = Math.PI / 2; s.rotation.y = aa;
    }
  }
  // the moving end: forward of its pin along the inner wall, its block up to the base plate
  const moving = new THREE.Group();
  moving.name = 'echain 8793 base-plate mount';
  g.add(moving);
  {
    const br = endBracket(LINK.width, -0.01, 0.036, bracketMat, screwMat, { plate: 'y' });
    moving.add(br);
    // the block from the bracket's top to the plate's underside, and two bolts through the plate
    const top = LINK.width / 2 + 0.0025, gap = o.plate - (y + top);
    box(0.03, gap, 0.016, bracketMat, moving, 0.013, top + gap / 2, 0);
    for (const dx of [0.004, 0.022]) cyl(0.0042, 0.003, screwMat, moving, dx, o.plate - y + 0.005 + 0.0015, 0, 10);
  }
  let last = null;
  const update = (yaw) => {
    if (last !== null && Math.abs(yaw - last) < 1e-5) return;
    last = yaw;
    const P = ch.solve(yaw), n = ch.n;
    links.place(P, y);
    const hd = Math.atan2(P[2 * n + 1] - P[2 * n - 1], P[2 * n] - P[2 * n - 2]);
    moving.position.set(P[2 * n], y, -P[2 * n + 1]);
    moving.rotation.y = hd;
  };
  update(0);
  return { group: g, chain: ch, update };
}

// ---------------------------------------------------------------- 4414
// The chain under the top plate's back strip and round the shooter (js/echain.js SPOOL4414): parent
// the robot's root, turret the turret's group (rotating, at the top plate's level: wallTop). The
// tensioner (its 1.625in flex wheel on a slider along the strip, two constant-force springs pulling
// it right), the 3in compliant wheel behind the turret on the left, the fixed end on the left, the
// corner guides round the shooter and the end's bracket on them. update(yaw) moves the chain and the
// tensioner.
export function chain4414(parent, turret, wallTop, M) {
  const ch = new SpoolChain(), o = ch.o, link = ch.link;
  const g = new THREE.Group();
  g.name = 'echain 4414';
  parent.add(g);
  const linkMat = new THREE.MeshStandardMaterial({ color: 0x1c1d21, roughness: 0.62, metalness: 0.05 });
  const dark = new THREE.MeshStandardMaterial({ color: 0x2a2d33, roughness: 0.55, metalness: 0.35 });
  const print = new THREE.MeshStandardMaterial({ color: 0x24272c, roughness: 0.8, metalness: 0.02 });
  const print2 = new THREE.MeshStandardMaterial({ color: 0x24272c, roughness: 0.8, metalness: 0.02, side: THREE.DoubleSide });
  const steel = new THREE.MeshStandardMaterial({ color: 0xc9ccd1, roughness: 0.3, metalness: 0.9 });
  const y = o.y, top = wallTop - 0.005, wid = link.width;
  const links = chainMesh(ch.n, wid, linkMat);
  g.add(links.mesh);
  const flex = M?.compliant || new THREE.MeshStandardMaterial({ color: 0x3f8f5a, roughness: 0.85 });
  // the fixed end, by the left wall under the strip, the chain heading right
  {
    const br = endBracket(wid, -0.035, 0.01, print, steel, { plate: 'y' });
    br.position.set(ch.xA, y, -o.entry[1]);
    br.rotation.y = -Math.PI / 2; // (heading -w: +z)
    g.add(br);
    box(0.03, top - (y + wid / 2), 0.03, print, g, ch.xA, (top + y + wid / 2) / 2, -o.entry[1] + 0.015);
  }
  // the tensioner: rails along the strip under it, the slider, the flex wheel hanging from it, the
  // springs' drums at the right end and their bands to the slider
  const xT = ch.xA + ch.rhoT, zR = [-o.tension[1] - 0.03, -o.tension[0] + 0.05];
  for (const dx of [-0.022, 0.022]) {
    const r = new THREE.Mesh(new THREE.CylinderGeometry(0.004, 0.004, zR[1] - zR[0], 10), steel);
    r.rotation.x = Math.PI / 2; r.position.set(xT + dx, top - 0.008, (zR[0] + zR[1]) / 2); g.add(r);
  }
  for (const z of zR) box(0.06, 0.012, 0.008, dark, g, xT, top - 0.006, z);
  const slider = new THREE.Group();
  g.add(slider);
  box(0.062, 0.006, 0.05, dark, slider, 0, top - 0.008 - (y), 0);
  for (const dx of [-0.022, 0.022]) for (const dz of [-0.016, 0.016]) cyl(0.007, 0.012, M?.copper || steel, slider, dx, top - 0.008 - y, dz, 10).rotation.x = Math.PI / 2; // (the thrust bushings' stacks)
  cyl(0.004, top - 0.008 - y + wid / 2, steel, slider, 0, (top - 0.008 - y) / 2 - wid / 4, 0, 8);
  {
    const wheel = cyl(o.tensionR, wid * 0.9, flex, slider, 0, 0, 0, 20);
    wheel.name = 'echain 4414 tensioner wheel';
  }
  const bands = [];
  for (const dx of [-0.012, 0.012]) {
    const drum = cyl(0.016, 0.012, steel, g, xT + dx * 2.2, top - 0.02, zR[1] - 0.012, 18);
    drum.rotation.x = 0;
    const band = box(0.0025, 0.009, 1, steel, g, xT + dx, top - 0.02, 0);
    bands.push(band);
  }
  // the compliant wheel, on its axle hung from the plate
  {
    const [cx, cw] = ch.C;
    cyl(o.wheel.r, wid * 0.85, flex, g, cx, y, -cw, 28).name = 'echain 4414 compliant wheel';
    cyl(0.005, top - y + wid / 2, steel, g, cx, (top + y) / 2, -cw, 8);
    box(0.05, 0.006, 0.05, dark, g, cx, top - 0.003, -cw);
  }
  // on the turret: the guides round the shooter's corners (printed, lipped top and bottom), and the
  // end's bracket on the box's face where the chain's end is fixed
  const guides = new THREE.Group();
  guides.name = 'echain 4414 corner guides';
  turret.add(guides);
  {
    const b = o.box, r = b.r, gy = y - wallTop;
    const cx = [b.x[0] + r, b.x[1] - r], cz = [b.z[0] + r, b.z[1] - r];
    const corners = [[cx[1], cz[1], 0], [cx[1], cz[0], 1], [cx[0], cz[0], 2], [cx[0], cz[1], 3]];
    for (const [x, z, k] of corners) {
      // (three's cylinders run theta from +z toward +x: an outward direction at angle a in [x, w] is
      // theta pi/2 + a; the corner's quarter, normals from -90 + 90k, and a little more each way)
      const t0 = Math.PI / 2 + (-Math.PI / 2 + (k * Math.PI) / 2) - 0.25, tl = Math.PI / 2 + 0.5;
      const shell = new THREE.Mesh(new THREE.CylinderGeometry(r, r, wid + 0.006, 12, 1, true, t0, tl), print2);
      shell.position.set(x, gy, z); guides.add(shell);
      for (const s of [-1, 1]) {
        const lip = new THREE.Mesh(new THREE.CylinderGeometry(r + 0.012, r + 0.012, 0.003, 12, 1, false, t0, tl), print);
        lip.position.set(x, gy + s * (wid / 2 + 0.003), z); guides.add(lip);
      }
      // (the block behind it, on the shooter's frame)
      box(0.03, wid + 0.006, 0.03, print, guides, x - Math.sign(x) * 0.008, gy, z - Math.sign(z) * 0.008);
    }
  }
  const endB = endBracket(wid, -0.01, 0.035, print, steel);
  endB.name = 'echain 4414 shooter mount';
  endB.rotation.order = 'YXZ';
  turret.add(endB);
  let last = null;
  const update = (yaw) => {
    if (last !== null && Math.abs(yaw - last) < 2e-4) return;
    last = yaw;
    const P = ch.solve(yaw), n = ch.n;
    links.place(P, y);
    slider.position.set(xT, y, -ch.wT);
    for (const band of bands) { const z0 = -ch.wT, z1 = zR[1] - 0.012; band.scale.z = Math.max(0.001, z1 - z0); band.position.z = (z0 + z1) / 2; }
    // the end's bracket in the turret's frame: from the last pin on along the chain
    const hd = Math.atan2(P[2 * n + 1] - P[2 * n - 1], P[2 * n] - P[2 * n - 2]) - yaw;
    const dx = P[2 * n] - o.axis[0], dw = P[2 * n + 1] - o.axis[1], c = Math.cos(-yaw), s = Math.sin(-yaw);
    endB.position.set(c * dx - s * dw, y - wallTop, -(s * dx + c * dw));
    endB.rotation.y = hd;
    endB.rotation.x = Math.PI; // (its plate on the box's side: the chain's left)
  };
  update(0);
  return { group: g, chain: ch, update };
}
