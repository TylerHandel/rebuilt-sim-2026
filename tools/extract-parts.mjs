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
// and then by placement (the node sits on the part studio's origin where the instance is, 1 mm);
// a name the BOM lacks (STEP calls some parts "id2590") goes by placement alone. A part matched
// to rows that still disagree, or to none, or to a row with no material, keeps its color. EXTRACT_LOG=file.json writes each kept part's name, center, color and verdict.
// "bomKinds" (top level, with a "bom"): finer names for the BOM's other materials, first match
// wins: [{ "kind": "coroplast", "material": "polypropylene", "recolor": [[[77, 77, 77], [28, 28, 30]]] }]
// puts the parts whose BOM material matches the regex in a copy of their color's material named
// for the kind (js/robotModels.js cadMaterial draws coroplast, plywood, print, rubber and steel;
// cadPart draws "poly" as clear polycarbonate, as for "glass"), repainted where its color (sRGB)
// is one of recolor's [from, to] pairs' first (a CAD's dark gray for what is black on the robot);
// the BOM's other non-aluminum parts stay "not-alu". Rows that agree on aluminum but not on the
// kind make the part "not-alu". An entry with "name" (a regex) instead takes the parts so named,
// whatever the BOM says (ones it gives no material: a battery in the aluminum's gray).
// --retag: leave each output's geometry as it is and only split its materials by the BOM, each
// triangle taking the verdict of the nearest kept part of its color (and each connected piece its
// triangles' majority): for outputs made from an older state of the CAD than a new export can
// reproduce exactly (parts dragged since, a turret turned), where re-extracting would move things.
import { NodeIO, getBounds } from '@gltf-transform/core';
import { ALL_EXTENSIONS } from '@gltf-transform/extensions';
import { prune, dedup, weld, simplify, flatten, join, quantize, compactPrimitive } from '@gltf-transform/functions';
import { MeshoptSimplifier } from 'meshoptimizer';
import fs from 'node:fs';
import path from 'node:path';

const [src, recipePath] = process.argv.slice(2);
if (!recipePath) { console.log('usage: node tools/extract-parts.mjs robot.glb recipe.json [--retag]'); process.exit(1); }
const RETAG = process.argv.includes('--retag');
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
const KINDS = (recipe.bomKinds || []).map((k) => ({ ...k, re: k.material && new RegExp(k.material, 'i'), nameRe: k.name && new RegExp(k.name, 'i') }));
// the verdicts by code (0: by color, so none; a "bomKinds" entry's is '#' and its index), their
// materials' names and each code's repaint
const VERDICTS = [null, 'alu', 'not-alu', ...KINDS.map((_, i) => '#' + i)];
const NAMES = [null, 'alu', 'not-alu', ...KINDS.map((k) => k.kind)];
const RECOLOR = [null, null, null, ...KINDS.map((k) => k.recolor || null)];
const coarse = (r) => (!r.material ? null : AL.test(r.material) ? 'alu' : 'not-alu');
const kind = (r) => {
  const i = coarse(r) === 'not-alu' ? KINDS.findIndex((k) => k.re && k.re.test(r.material)) : -1;
  return i < 0 ? coarse(r) : '#' + i;
};
const agree = (rows, f = kind) => new Set(rows.map(f)).size === 1;
const allRows = [...bomRows.values()].flat();
const placedAt = (rows, t) => rows.filter((r) => r.at.some((a) => Math.hypot(a[0] - t[0], a[1] - t[1], a[2] - t[2]) < 0.001));
const verdict = (name, pid, places) => {
  const byName = KINDS.findIndex((k) => k.nameRe && k.nameRe.test(name));
  if (byName >= 0) return '#' + byName;
  let rows = bomRows.get(nameKey(name));
  // (a name the BOM doesn't know, like the "id2590" a STEP export gives some parts: the rows of
  // the parts placed there, if they agree)
  if (!rows) rows = places.map((t) => placedAt(allRows, t)).find((f) => f.length);
  if (!rows) return null;
  if (!agree(rows) && pid) { const f = rows.filter((r) => r.partId === pid); if (f.length) rows = f; }
  for (const t of places) {
    if (agree(rows)) break;
    const f = placedAt(rows, t);
    if (f.length) rows = f;
  }
  return agree(rows) ? kind(rows[0]) : agree(rows, coarse) ? coarse(rows[0]) : null;
};
// splits each primitive by its triangles' verdicts (code(prim, i): 0 by color, 1 aluminum, 2 not,
// 3 on a "bomKinds" kind; null for a primitive to leave alone) into one per verdict, in the same
// mesh (so quantize's per-mesh grid is unchanged): aluminum ones get a copy of the color's
// material named "alu", the BOM's other materials one named "not-alu" or for their kind
const toLinear = (v) => (v / 255 <= 0.04045 ? v / 255 / 12.92 : ((v / 255 + 0.055) / 1.055) ** 2.4);
const splitByVerdict = (doc, code) => {
  const copies = new Map();
  const variant = (m, c) => {
    if (!m || !c) return m;
    if (!copies.has(m)) copies.set(m, {});
    if (!copies.get(m)[c]) {
      const v = m.clone().setName(NAMES[c]), f = m.getBaseColorFactor();
      const to = (RECOLOR[c] || []).find(([from]) => from.every((x, i) => Math.abs(toSRGB(f[i]) - x) <= 2))?.[1];
      if (to) v.setBaseColorFactor([...to.map(toLinear), f[3]]);
      copies.get(m)[c] = v;
    }
    return copies.get(m)[c];
  };
  for (const mesh of doc.getRoot().listMeshes()) {
    for (const p of [...mesh.listPrimitives()]) {
      const idx = p.getIndices()?.getArray();
      if (!idx || code(p, 0) === null) continue;
      const groups = new Map();
      for (let i = 0; i < idx.length; i += 3) {
        const c = code(p, i / 3);
        if (!groups.has(c)) groups.set(c, []);
        groups.get(c).push(idx[i], idx[i + 1], idx[i + 2]);
      }
      const list = [...groups].sort((a, b) => a[0] - b[0]);
      if (list.length === 1) { p.setMaterial(variant(p.getMaterial(), list[0][0])); continue; }
      const prims = list.map((_, i) => (i === 0 ? p : p.clone()));
      list.forEach(([c, ids], i) => {
        const q = prims[i];
        if (i) mesh.addPrimitive(q);
        q.setIndices(doc.createAccessor().setType('SCALAR').setArray(new Uint32Array(ids)).setBuffer(p.getIndices().getBuffer()));
        q.setMaterial(variant(p.getMaterial(), c));
      });
      for (const q of prims) compactPrimitive(q);
    }
  }
};
// by the _BOM vertex attribute (all of a triangle's corners carry its part's), then dropped
const splitBom = (doc) => {
  splitByVerdict(doc, (p, t) => {
    const tag = p.getAttribute('_BOM');
    return tag ? tag.getArray()[p.getIndices().getArray()[3 * t]] : null;
  });
  for (const m of doc.getRoot().listMeshes()) for (const p of m.listPrimitives()) p.setAttribute('_BOM', null);
};
// --retag: the verdicts put on part.out as it is (its geometry untouched), each triangle taking
// that of the nearest kept part of its color (its centroid within 8 mm of the part's surface,
// in the output's frame), for when the CAD has moved on since the output was made
const retag = async (file, parts) => {
  const out = await io.read(file);
  if (out.getRoot().listMaterials().some((m) => m.getName() !== 'poly' && NAMES.includes(m.getName()))) throw new Error(file + ' is tagged already');
  const ck = (m) => (m ? m.getBaseColorFactor().map((x) => Math.round(x * 255)).join(',') : '-');
  const CELL = 0.01, R = 0.008, grid = new Map();
  for (const { tris, color, c } of parts) {
    for (const t of tris) {
      const lo = [0, 1, 2].map((d) => Math.floor((Math.min(t[0][d], t[1][d], t[2][d]) - R) / CELL));
      const hi = [0, 1, 2].map((d) => Math.floor((Math.max(t[0][d], t[1][d], t[2][d]) + R) / CELL));
      for (let i = lo[0]; i <= hi[0]; i++) for (let j = lo[1]; j <= hi[1]; j++) for (let k = lo[2]; k <= hi[2]; k++) {
        const key = `${color}|${i},${j},${k}`;
        if (!grid.has(key)) grid.set(key, []);
        grid.get(key).push([t, c]);
      }
    }
  }
  const codes = new Map(), tally = NAMES.map(() => 0);
  let far = 0; // triangles with no kept part of their color near
  for (const n of out.getRoot().listNodes()) {
    if (!n.getMesh()) continue;
    const m = n.getWorldMatrix();
    for (const p of n.getMesh().listPrimitives()) {
      const color = ck(p.getMaterial()), pos = p.getAttribute('POSITION'), idx = p.getIndices(), e = [0, 0, 0];
      if (p.getMaterial()?.getName() === 'poly' || !idx) continue;
      const w = (i) => { pos.getElement(i, e); return [0, 1, 2].map((r) => m[r] * e[0] + m[4 + r] * e[1] + m[8 + r] * e[2] + m[12 + r]); };
      const cs = new Uint8Array(idx.getCount() / 3);
      for (let t = 0; t < cs.length; t++) {
        const a = w(idx.getScalar(3 * t)), b = w(idx.getScalar(3 * t + 1)), c = w(idx.getScalar(3 * t + 2));
        const q = [0, 1, 2].map((d) => (a[d] + b[d] + c[d]) / 3);
        let best = R;
        for (const [s, code] of grid.get(`${color}|${q.map((x) => Math.floor(x / CELL)).join(',')}`) || []) {
          const d = pointTri(q, s[0], s[1], s[2]);
          if (d < best) { best = d; cs[t] = code; }
        }
        if (best === R) far++;
      }
      // a connected piece of the mesh is one part (or a piece of one): it takes its triangles'
      // majority, so no part ends up half one material where it touches another
      const up = new Int32Array(pos.getCount()).map((_, i) => i);
      const find = (i) => { while (up[i] !== i) i = up[i] = up[up[i]]; return i; };
      for (let k = 0; k < idx.getCount(); k += 3) { const a = find(idx.getScalar(k)); up[find(idx.getScalar(k + 1))] = a; up[find(idx.getScalar(k + 2))] = a; }
      const votes = new Map();
      for (let t = 0; t < cs.length; t++) { const r = find(idx.getScalar(3 * t)); if (!votes.has(r)) votes.set(r, NAMES.map(() => 0)); votes.get(r)[cs[t]]++; }
      for (let t = 0; t < cs.length; t++) { const v = votes.get(find(idx.getScalar(3 * t))); cs[t] = v.indexOf(Math.max(...v)); tally[cs[t]]++; }
      codes.set(p, cs);
    }
  }
  splitByVerdict(out, (p, t) => (codes.has(p) ? codes.get(p)[t] : null));
  await out.transform(prune());
  await io.write(file, out);
  console.log(`retagged ${file}: ${tally[1]} triangles aluminum, ${tally[2]} not, ${KINDS.map((k, i) => `${tally[3 + i]} ${k.kind}, `).join('')}${tally[0]} by color (${far} with no kept part of their color near)`);
};
// distance from p to triangle (a, b, c) (Ericson, Real-Time Collision Detection 5.1.5)
const sub = (a, b) => [a[0] - b[0], a[1] - b[1], a[2] - b[2]];
const dot = (a, b) => a[0] * b[0] + a[1] * b[1] + a[2] * b[2];
function pointTri(p, a, b, c) {
  const ab = sub(b, a), ac = sub(c, a), ap = sub(p, a), d1 = dot(ab, ap), d2 = dot(ac, ap);
  if (d1 <= 0 && d2 <= 0) return Math.hypot(...ap);
  const bp = sub(p, b), d3 = dot(ab, bp), d4 = dot(ac, bp);
  if (d3 >= 0 && d4 <= d3) return Math.hypot(...bp);
  const vc = d1 * d4 - d3 * d2;
  if (vc <= 0 && d1 >= 0 && d3 <= 0) { const v = d1 / (d1 - d3); return Math.hypot(...sub(ap, [v * ab[0], v * ab[1], v * ab[2]])); }
  const cp = sub(p, c), d5 = dot(ab, cp), d6 = dot(ac, cp);
  if (d6 >= 0 && d5 <= d6) return Math.hypot(...cp);
  const vb = d5 * d2 - d1 * d6;
  if (vb <= 0 && d2 >= 0 && d6 <= 0) { const w = d2 / (d2 - d6); return Math.hypot(...sub(ap, [w * ac[0], w * ac[1], w * ac[2]])); }
  const va = d3 * d6 - d5 * d4;
  if (va <= 0 && d4 - d3 >= 0 && d5 - d6 >= 0) { const w = (d4 - d3) / (d4 - d3 + (d5 - d6)); return Math.hypot(...sub(bp, [w * (c[0] - b[0]), w * (c[1] - b[1]), w * (c[2] - b[2])])); }
  const den = 1 / (va + vb + vc), v = vb * den, w = vc * den;
  return Math.hypot(...sub(ap, [ab[0] * v + ac[0] * w, ab[1] * v + ac[1] * w, ab[2] * v + ac[2] * w]));
}
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
  const at = new Map(); // kept node -> its center (model frame), for the log
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
      if (ok) { keep.add(n); at.set(n, c); }
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
    // 2 not, 3 on a "bomKinds" kind), through the welding, joining and simplifying, which go on exactly as without a BOM
    // (all of a part's vertices carry the same value, and a triangle never spans two parts), and
    // splits the merged meshes by material at the end (splitBom)
    const pid = partIds(src, doc);
    const code = Object.fromEntries(VERDICTS.map((n, i) => [n, i]).filter(([n]) => n));
    const tagged = new Map(); // mesh -> its verdict
    const tally = Object.fromEntries(VERDICTS.map((n) => [n, 0]));
    const codeOf = new Map(); // kept node -> its verdict's code
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
      if (process.env.EXTRACT_LOG && !glass.has(n)) log.push({ out: part.out, name: n.getName(), at: at.get(n).map((x) => Math.round(x * 1000)), rgb: m0 ? m0.getBaseColorFactor().slice(0, 3).map(toSRGB) : null, bom: NAMES[code[v] ?? 0] });
      const c = code[v] ?? 0, mesh = n.getMesh();
      codeOf.set(n, c);
      if (!tagged.has(mesh)) tag(mesh, c);
      else if (tagged.get(mesh) !== c) {
        // (instances of a part share its mesh; one whose verdict differs gets its own)
        const own = doc.createMesh(mesh.getName());
        for (const p of mesh.listPrimitives()) own.addPrimitive(p.clone());
        n.setMesh(own);
        tag(own, c);
      }
    }
    console.log(`  BOM: ${tally.alu} parts aluminum, ${tally['not-alu']} not, ${KINDS.map((k, i) => `${tally['#' + i]} ${k.kind}, `).join('')}${tally.null} by color`);
    if (RETAG) {
      // each kept part's triangles in the output's frame (model frame less the origin), by color
      const M = AXES[recipe.front], o = part.origin || [0, 0, 0];
      const list = [];
      for (const [n, c] of codeOf) {
        const w = n.getWorldMatrix(), e = [0, 0, 0];
        for (const p of n.getMesh().listPrimitives()) {
          const pos = p.getAttribute('POSITION'), idx = p.getIndices(), V = [];
          for (let i = 0; i < pos.getCount(); i++) {
            pos.getElement(i, e);
            const q = [0, 1, 2].map((r) => w[r] * e[0] + w[4 + r] * e[1] + w[8 + r] * e[2] + w[12 + r]);
            V.push([0, 1, 2].map((r) => M[r] * q[0] + M[4 + r] * q[1] + M[8 + r] * q[2] - o[r]));
          }
          const tris = [];
          for (let k = 0; k < (idx ? idx.getCount() : V.length); k += 3) tris.push([0, 1, 2].map((j) => V[idx ? idx.getScalar(k + j) : k + j]));
          const m = p.getMaterial();
          list.push({ tris, c, color: m ? m.getBaseColorFactor().map((x) => Math.round(x * 255)).join(',') : '-' });
        }
      }
      await retag(part.out, list);
      continue;
    }
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
