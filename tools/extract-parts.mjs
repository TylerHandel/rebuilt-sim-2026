// Pull parts out of a robot CAD export (GLB from tools/onshape-export.sh) as light, flat-shaded
// GLBs in the robot model's frame (x forward, y up, z to the side), each with its origin on a
// pivot or axis so the game can move it. A recipe lists the outputs:
//   npm i --no-save @gltf-transform/core @gltf-transform/extensions @gltf-transform/functions meshoptimizer
//   node tools/extract-parts.mjs robot.glb cad/robots/971.json
// Recipe: { "front": "+y" | "-y" (which CAD axis the robot's front is; CAD is Z up),
//   "parts": [{ "out": "cad/robots/971-turret.glb", "include": ["shooter assembly <1>"],
//               "exclude": ["regex", ...], "origin": [x, y, z] (model frame), "ratio": 0.08 }] }
// include: node names (prefix match) whose subtrees are kept; exclude: regexes on any node name
// on the way down (dropped with their subtrees). Small hardware is always dropped. Optional
// filters on what's left: "skipBox" drops parts whose center (model frame) is in one of the
// boxes [x0, x1, y0, y1, z0, z1]; "onlyName" / "onlyBox" keep just the parts under a node
// matching one of the regexes or centered in one of the boxes. "glass": regexes for parts that are
// polycarbonate but aren't see-through in the CAD (they get a clear material named "poly").
// "like": { "glb": ref.glb, "node": "name" } keeps only the parts that match (by name and
// position, within 5 mm) a part under that node in another export of the same robot, and
// copies which of them are see-through; for picking a subassembly out of a STEP conversion,
// which flattens the tree (tools/step2glb.mjs). "smooth": keep the surface normals and the
// source's own tessellation instead of simplifying (for STEP conversions: round parts stay
// round), quantized to keep the file small.
import { NodeIO, getBounds } from '@gltf-transform/core';
import { ALL_EXTENSIONS } from '@gltf-transform/extensions';
import { prune, dedup, weld, simplify, flatten, join, quantize } from '@gltf-transform/functions';
import { MeshoptSimplifier } from 'meshoptimizer';
import fs from 'node:fs';
import path from 'node:path';

const [src, recipePath] = process.argv.slice(2);
if (!recipePath) { console.log('usage: node tools/extract-parts.mjs robot.glb recipe.json'); process.exit(1); }
const recipe = JSON.parse(fs.readFileSync(recipePath, 'utf8'));
const HW = /screw|\bnut\b|washer|\brivets?\b|shcs|bhcs|fhcs|rivnut|insert|retaining|zip tie|^9\d{4}a|^fuel\b|reference cube|origin cube/i;
const io = new NodeIO().registerExtensions(ALL_EXTENSIONS);
await MeshoptSimplifier.ready;

// CAD (Z up) -> model frame, column-major
const AXES = {
  '+y': [0, 0, 1, 0, 1, 0, 0, 0, 0, 1, 0, 0], // model = (y, z, x)
  '-y': [0, 0, -1, 0, -1, 0, 0, 0, 0, 1, 0, 0], // model = (-y, z, -x)
};

const center = (n) => { const b = getBounds(n); return [0, 1, 2].map((i) => (b.min[i] + b.max[i]) / 2); };
const see = (n) => n.getMesh().listPrimitives().some((p) => { const m = p.getMaterial(); return m && (m.getAlphaMode() === 'BLEND' || m.getBaseColorFactor()[3] < 0.99); });
const refs = new Map();
const refParts = async (like) => {
  const k = like.glb + '|' + like.node;
  if (!refs.has(k)) {
    const ref = await io.read(path.resolve(path.dirname(src), like.glb)); // next to the source
    const top = ref.getRoot().listNodes().find((n) => n.getName().startsWith(like.node));
    if (!top) throw new Error('no node ' + like.node + ' in ' + like.glb);
    const list = [];
    top.traverse((n) => { if (n.getMesh()) list.push({ name: n.getName(), c: center(n), clear: see(n) }); });
    refs.set(k, list);
  }
  return refs.get(k);
};

for (const part of recipe.parts) {
  const doc = await io.read(src);
  const like = part.like ? await refParts(part.like) : null;
  const root = doc.getRoot();
  const scene = root.listScenes()[0];
  const inc = (part.include || []).map((n) => n.toLowerCase());
  const exc = (part.exclude || []).map((r) => new RegExp(r, 'i'));
  // keep a mesh node when an ancestor (or itself) is included and nothing on the way is excluded
  const keep = new Set();
  const M0 = AXES[recipe.front];
  const toModel = (p) => [0, 1, 2].map((i) => M0[i] * p[0] + M0[4 + i] * p[1] + M0[8 + i] * p[2]);
  const inBox = (c, b) => c[0] >= b[0] && c[0] <= b[1] && c[1] >= b[2] && c[1] <= b[3] && c[2] >= b[4] && c[2] <= b[5];
  const onlyName = (part.onlyName || []).map((r) => new RegExp(r, 'i'));
  const glassRe = (part.glass || []).map((r) => new RegExp(r, 'i'));
  const glass = new Set();
  const walk = (n, on, named, clear) => {
    const name = n.getName();
    if (HW.test(name) || exc.some((r) => r.test(name))) return;
    const hit = on || !inc.length || inc.some((p) => name.toLowerCase().startsWith(p));
    const nm = named || onlyName.some((r) => r.test(name));
    const cl = clear || glassRe.some((r) => r.test(name));
    if (cl && n.getMesh()) glass.add(n);
    if (hit && n.getMesh()) {
      const cc = center(n);
      const c = toModel(cc);
      let ok = !(part.skipBox || []).some((bx) => inBox(c, bx));
      if (ok && like) {
        const m = like.find((q) => q.name === name && Math.hypot(q.c[0] - cc[0], q.c[1] - cc[1], q.c[2] - cc[2]) < 0.005);
        ok = !!m;
        if (m && m.clear) glass.add(n);
      }
      if (ok && (part.onlyName || part.onlyBox)) ok = nm || (part.onlyBox || []).some((bx) => inBox(c, bx));
      if (ok) keep.add(n);
    }
    for (const c of n.listChildren()) walk(c, hit, nm, cl);
  };
  for (const c of scene.listChildren()) walk(c, false, false, false);
  if (glass.size) {
    const poly = doc.createMaterial('poly').setBaseColorFactor([0.66, 0.76, 0.87, 0.3]).setAlphaMode('BLEND').setDoubleSided(true);
    for (const n of glass) if (keep.has(n)) for (const p of n.getMesh().listPrimitives()) p.setMaterial(poly);
  }
  for (const n of root.listNodes()) if (n.getMesh() && !keep.has(n)) n.setMesh(null);
  await doc.transform(prune(), flatten());
  const M = AXES[recipe.front];
  const o = part.origin || [0, 0, 0];
  const pivot = doc.createNode('pivot').setMatrix([...M, -o[0], -o[1], -o[2], 1]);
  for (const c of [...scene.listChildren()]) { scene.removeChild(c); pivot.addChild(c); }
  scene.addChild(pivot);
  if (part.smooth) {
    await doc.transform(prune(), dedup(), weld(), flatten(), join({ keepNamed: false }), prune(), quantize());
  } else {
    // flat CAD surfaces: drop the split normals so vertices weld and the simplifier can work
    for (const m of root.listMeshes()) for (const p of m.listPrimitives()) { p.setAttribute('NORMAL', null); p.setAttribute('TEXCOORD_0', null); }
    await doc.transform(prune(), dedup(), weld({ tolerance: 0.0002 }), simplify({ simplifier: MeshoptSimplifier, ratio: part.ratio ?? 0.08, error: part.error ?? 0.02 }), flatten(), join({ keepNamed: false }), prune());
  }
  let tris = 0;
  root.listMeshes().forEach((m) => m.listPrimitives().forEach((p) => { tris += (p.getIndices() || p.getAttribute('POSITION')).getCount() / 3; }));
  await io.write(part.out, doc);
  console.log(`wrote ${part.out}: ${keep.size} parts, ${tris} triangles, ${(fs.statSync(part.out).size / 1024).toFixed(0)} KB`);
}
