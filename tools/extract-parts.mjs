// Pull parts out of a robot CAD export (GLB from tools/onshape-export.sh) as light, flat-shaded
// GLBs in the robot model's frame (x forward, y up, z to the side), each with its origin on a
// pivot or axis so the game can move it. A recipe lists the outputs:
//   npm i --no-save @gltf-transform/core @gltf-transform/extensions @gltf-transform/functions meshoptimizer
//   node tools/extract-parts.mjs robot.glb cad/robots/971.json
// Recipe: { "front": "+y" | "-y" | "-x" (which CAD axis the robot's front is; CAD is Z up),
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
// round), only lightly simplified (ratio 0.5, error 0.001 by default) and quantized.
// "bom" (top level, found next to the source like "like"'s glb): the parts' materials from the
// Onshape bill of materials, as tools/aluminum-colors.py writes them (<team>-materials.json).
// Parts are merged by color, so a plastic part in the aluminum's color (8793's cable carrier and
// 3D prints in its plates' slate) would be drawn as aluminum; with a "bom", the parts of the BOM's
// aluminum end up in a copy of their color's material named "alu" and those of its other
// materials in one named "not-alu", while the geometry comes out exactly as without it
// (js/robotModels.js cadPart draws "alu" as aluminum, "not-alu" as it is, and the rest by the
// color rule, as for a GLB made without a "bom"). Parts are matched to BOM rows by
// name (without Onshape's instance suffix " <3>", letters and digits only, which also gets past
// STEP's mangling of quotes and such); a name that several parts share is narrowed by the part
// id (a glTF export's node carries it, PTC_onshape_metadata; a STEP part takes its "like" match's)
// and then by placement (the node sits on the part studio's origin where the instance is, 1 mm).
// A part matched to rows that still disagree, or to none, or to a row with no material, keeps its
// color. EXTRACT_LOG=file.json writes each kept part's name, color and verdict.
import { NodeIO, getBounds } from '@gltf-transform/core';
import { ALL_EXTENSIONS } from '@gltf-transform/extensions';
import { prune, dedup, weld, simplify, flatten, join, quantize, compactPrimitive } from '@gltf-transform/functions';
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
  '-x': [-1, 0, 0, 0, 0, 0, 1, 0, 0, 1, 0, 0], // model = (-x, z, y)
};

const center = (n) => { const b = getBounds(n); return [0, 1, 2].map((i) => (b.min[i] + b.max[i]) / 2); };
const see = (n) => n.getMesh().listPrimitives().some((p) => { const m = p.getMaterial(); return m && (m.getAlphaMode() === 'BLEND' || m.getBaseColorFactor()[3] < 0.99); });
// Onshape's part id on each node of a glTF export (PTC_onshape_metadata, an extension the reader
// drops), by node, read from the file's JSON (its nodes come in the same order as listNodes)
const partIds = (file, doc) => {
  const b = fs.readFileSync(file);
  if (b.readUInt32LE(0) !== 0x46546c67) return new Map();
  const j = JSON.parse(b.subarray(20, 20 + b.readUInt32LE(12)).toString('utf8'));
  const nodes = doc.getRoot().listNodes();
  if (!j.nodes || j.nodes.length !== nodes.length || j.nodes.some((n, i) => (n.name ?? '') !== nodes[i].getName())) return new Map();
  return new Map(nodes.map((n, i) => [n, j.nodes[i].extensions?.PTC_onshape_metadata?.id?.[0] || '']));
};
const refs = new Map();
const refParts = async (like) => {
  const k = like.glb + '|' + like.node;
  if (!refs.has(k)) {
    const file = path.resolve(path.dirname(src), like.glb); // next to the source
    const ref = await io.read(file);
    const pid = partIds(file, ref);
    const top = ref.getRoot().listNodes().find((n) => n.getName().startsWith(like.node));
    if (!top) throw new Error('no node ' + like.node + ' in ' + like.glb);
    const list = [];
    top.traverse((n) => { if (n.getMesh()) list.push({ name: n.getName(), c: center(n), clear: see(n), pid: pid.get(n) || '', t: n.getWorldTranslation() }); });
    refs.set(k, list);
  }
  return refs.get(k);
};

// the BOM's verdict on a part: 'alu', 'not-alu' or null (no match, no material, or rows that disagree)
const AL = /alumin|6061|7075|5052|6063|2024/i;
const nameKey = (s) => s.replace(/\s*<\d+>$/, '').normalize('NFKD').toLowerCase().replace(/[^a-z0-9]+/g, '');
const bomRows = new Map();
if (recipe.bom) {
  for (const r of JSON.parse(fs.readFileSync(path.resolve(path.dirname(src), recipe.bom), 'utf8')).parts) {
    const k = nameKey(r.name);
    if (!bomRows.has(k)) bomRows.set(k, []);
    bomRows.get(k).push(r);
  }
}
const kind = (r) => (!r.material ? null : AL.test(r.material) ? 'alu' : 'not-alu');
const agree = (rows) => new Set(rows.map(kind)).size === 1;
const verdict = (name, pid, places) => {
  let rows = bomRows.get(nameKey(name));
  if (!rows) return null;
  if (!agree(rows) && pid) { const f = rows.filter((r) => r.partId === pid); if (f.length) rows = f; }
  for (const t of places) {
    if (agree(rows)) break;
    const f = rows.filter((r) => r.at.some((a) => Math.hypot(a[0] - t[0], a[1] - t[1], a[2] - t[2]) < 0.001));
    if (f.length) rows = f;
  }
  return agree(rows) ? kind(rows[0]) : null;
};
// splits each merged primitive by its triangles' _BOM into one per verdict, in the same mesh (so
// quantize's per-mesh grid is unchanged): aluminum ones get a copy of the color's material named
// "alu", the BOM's other materials one named "not-alu"; then drops _BOM
const splitBom = (doc) => {
  const copies = new Map();
  const variant = (m, c) => {
    if (!m || !c) return m;
    if (!copies.has(m)) copies.set(m, {});
    return (copies.get(m)[c] ??= m.clone().setName(c === 1 ? 'alu' : 'not-alu'));
  };
  for (const mesh of doc.getRoot().listMeshes()) {
    for (const p of mesh.listPrimitives()) {
      const tag = p.getAttribute('_BOM');
      if (!tag) continue;
      const t = tag.getArray(), idx = p.getIndices().getArray();
      const groups = new Map();
      for (let i = 0; i < idx.length; i += 3) {
        const c = t[idx[i]];
        if (!groups.has(c)) groups.set(c, []);
        groups.get(c).push(idx[i], idx[i + 1], idx[i + 2]);
      }
      const list = [...groups].sort((a, b) => a[0] - b[0]);
      if (list.length === 1) { p.setAttribute('_BOM', null); p.setMaterial(variant(p.getMaterial(), list[0][0])); continue; }
      const prims = list.map((_, i) => (i === 0 ? p : p.clone()));
      list.forEach(([c, ids], i) => {
        const q = prims[i];
        if (i) mesh.addPrimitive(q);
        q.setIndices(doc.createAccessor().setType('SCALAR').setArray(new Uint32Array(ids)).setBuffer(p.getIndices().getBuffer()));
        q.setMaterial(variant(p.getMaterial(), c));
      });
      for (const q of prims) { compactPrimitive(q); q.setAttribute('_BOM', null); }
    }
  }
};
const toSRGB = (c) => Math.round(255 * (c <= 0.0031308 ? 12.92 * c : 1.055 * c ** (1 / 2.4) - 0.055));
const log = [];

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
  const refOf = new Map(); // kept node -> its "like" match
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
        if (m) refOf.set(n, m);
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
  if (recipe.bom) {
    // each kept part's verdict rides along as a vertex attribute (_BOM: 0 by color, 1 aluminum,
    // 2 not), through the welding, joining and simplifying, which go on exactly as without a BOM
    // (all of a part's vertices carry the same value, and a triangle never spans two parts), and
    // splits the merged meshes by material at the end (splitBom)
    const pid = partIds(src, doc);
    const code = { alu: 1, 'not-alu': 2 };
    const tagged = new Map(); // mesh -> its verdict
    const tally = { alu: 0, 'not-alu': 0, null: 0 };
    const tag = (mesh, c) => {
      for (const p of mesh.listPrimitives()) {
        const count = p.getAttribute('POSITION').getCount();
        p.setAttribute('_BOM', doc.createAccessor().setType('SCALAR').setArray(new Float32Array(count).fill(c)).setBuffer(root.listBuffers()[0]));
      }
      tagged.set(mesh, c);
    };
    for (const n of keep) {
      const ref = refOf.get(n);
      const v = glass.has(n) ? null : verdict(n.getName(), pid.get(n) || ref?.pid, [n.getWorldTranslation(), ...(ref ? [ref.t] : [])]);
      if (!glass.has(n)) tally[v]++;
      const m0 = n.getMesh().listPrimitives()[0]?.getMaterial();
      if (process.env.EXTRACT_LOG && !glass.has(n)) log.push({ out: part.out, name: n.getName(), rgb: m0 ? m0.getBaseColorFactor().slice(0, 3).map(toSRGB) : null, bom: v });
      const c = code[v] ?? 0, mesh = n.getMesh();
      if (!tagged.has(mesh)) tag(mesh, c);
      else if (tagged.get(mesh) !== c) {
        // (instances of a part share its mesh; one whose verdict differs gets its own)
        const own = doc.createMesh(mesh.getName());
        for (const p of mesh.listPrimitives()) own.addPrimitive(p.clone());
        n.setMesh(own);
        tag(own, c);
      }
    }
    console.log(`  BOM: ${tally.alu} parts aluminum, ${tally['not-alu']} not, ${tally.null} by color`);
  }
  for (const n of root.listNodes()) if (n.getMesh() && !keep.has(n)) n.setMesh(null);
  await doc.transform(prune(), flatten());
  const M = AXES[recipe.front];
  const o = part.origin || [0, 0, 0];
  const pivot = doc.createNode('pivot').setMatrix([...M, -o[0], -o[1], -o[2], 1]);
  for (const c of [...scene.listChildren()]) { scene.removeChild(c); pivot.addChild(c); }
  scene.addChild(pivot);
  if (part.smooth) {
    // light: hard edges keep their split normals, so the simplifier leaves them alone
    await doc.transform(prune(), dedup(), weld(), flatten(), join({ keepNamed: false }), simplify({ simplifier: MeshoptSimplifier, ratio: part.ratio ?? 0.5, error: part.error ?? 0.001 }), prune());
    splitBom(doc);
    await doc.transform(prune(), quantize());
  } else {
    // flat CAD surfaces: drop the split normals so vertices weld and the simplifier can work
    for (const m of root.listMeshes()) for (const p of m.listPrimitives()) { p.setAttribute('NORMAL', null); p.setAttribute('TEXCOORD_0', null); }
    await doc.transform(prune(), dedup(), weld({ tolerance: 0.0002 }), simplify({ simplifier: MeshoptSimplifier, ratio: part.ratio ?? 0.08, error: part.error ?? 0.02 }), flatten(), join({ keepNamed: false }), prune());
    splitBom(doc);
    await doc.transform(prune());
  }
  let tris = 0;
  root.listMeshes().forEach((m) => m.listPrimitives().forEach((p) => { tris += (p.getIndices() || p.getAttribute('POSITION')).getCount() / 3; }));
  await io.write(part.out, doc);
  console.log(`wrote ${part.out}: ${keep.size} parts, ${tris} triangles, ${(fs.statSync(part.out).size / 1024).toFixed(0)} KB`);
}
if (process.env.EXTRACT_LOG) fs.writeFileSync(process.env.EXTRACT_LOG, JSON.stringify(log, null, 0).replace(/\},\{/g, '},\n{'));
