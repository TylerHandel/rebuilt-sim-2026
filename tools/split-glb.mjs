// Split a moving part out of a robot CAD part file, by matching its triangles against that part
// in a glTF export of the same robot. Where the part file came through a STEP conversion
// (tools/step2glb.py: smooth, but the subassembly tree flattened and merged), this finds the
// part's triangles by where they are: on the surface of one of the reference parts.
//   npm i --no-save @gltf-transform/core @gltf-transform/extensions @gltf-transform/functions
//   node tools/split-glb.mjs part.glb robot-export.glb <front> "<node prefix>[|<prefix>...]" \
//        <pivot x,y,z> rest-out.glb part-out.glb
// front: which CAD axis is the robot's front ('+y', '-y', '-x', as in tools/extract-parts.mjs);
// the prefixes name the reference subassemblies (their whole subtrees); pivot (model frame):
// the part's origin in part-out.glb, so the game can turn it about there. part.glb is in the
// model frame (x forward, y up, z to the side) with its origin at the robot center.
import { NodeIO } from '@gltf-transform/core';
import { ALL_EXTENSIONS } from '@gltf-transform/extensions';
import { compactPrimitive, prune } from '@gltf-transform/functions';

const [src, refPath, front, prefixArg, pivotArg, outRest, outPart] = process.argv.slice(2);
if (!outPart) { console.log('usage: node tools/split-glb.mjs part.glb robot-export.glb <front> "<prefix>|..." x,y,z rest.glb part.glb'); process.exit(1); }
const prefixes = prefixArg.split('|').map((s) => s.trim().toLowerCase());
const pivot = pivotArg.split(',').map(Number);
const TOL = 0.002; // how close to a reference surface a triangle has to be (m)
const io = new NodeIO().registerExtensions(ALL_EXTENSIONS);

// CAD (Z up) -> model frame (as tools/extract-parts.mjs)
const AXES = {
  '+y': (p) => [p[1], p[2], p[0]],
  '-y': (p) => [-p[1], p[2], -p[0]],
  '-x': (p) => [-p[0], p[2], p[1]],
};
const toModel = AXES[front];
if (!toModel) throw new Error('front must be one of ' + Object.keys(AXES).join(' '));

const xf = (m, p) => [
  m[0] * p[0] + m[4] * p[1] + m[8] * p[2] + m[12],
  m[1] * p[0] + m[5] * p[1] + m[9] * p[2] + m[13],
  m[2] * p[0] + m[6] * p[1] + m[10] * p[2] + m[14],
];
// every triangle of a primitive in world (document) coordinates
function* triangles(node, prim) {
  const m = node.getWorldMatrix();
  const pos = prim.getAttribute('POSITION'), idx = prim.getIndices();
  const n = idx ? idx.getCount() : pos.getCount();
  const v = [0, 0, 0];
  const at = (k) => xf(m, pos.getElement(idx ? idx.getScalar(k) : k, v));
  for (let k = 0; k + 2 < n; k += 3) yield [at(k), at(k + 1), at(k + 2)];
}

// ---- reference: the part's triangles in the export, model frame, binned on a 1 cm grid
const ref = await io.read(refPath);
const CELL = 0.01, grid = new Map();
const key = (i, j, k) => `${i},${j},${k}`;
let nRef = 0;
const under = (n) => { for (let p = n; p; p = p.getParentNode()) if (prefixes.some((q) => p.getName().toLowerCase().startsWith(q))) return true; return false; };
for (const node of ref.getRoot().listNodes()) {
  if (!node.getMesh() || !under(node)) continue;
  for (const prim of node.getMesh().listPrimitives()) {
    for (const t of triangles(node, prim)) {
      const tri = t.map(toModel);
      const lo = [0, 1, 2].map((d) => Math.floor((Math.min(tri[0][d], tri[1][d], tri[2][d]) - TOL) / CELL));
      const hi = [0, 1, 2].map((d) => Math.floor((Math.max(tri[0][d], tri[1][d], tri[2][d]) + TOL) / CELL));
      for (let i = lo[0]; i <= hi[0]; i++) for (let j = lo[1]; j <= hi[1]; j++) for (let k = lo[2]; k <= hi[2]; k++) {
        const c = key(i, j, k);
        if (!grid.has(c)) grid.set(c, []);
        grid.get(c).push(tri);
      }
      nRef++;
    }
  }
}
if (!nRef) throw new Error('no reference triangles under ' + prefixArg);

// distance from p to triangle (a, b, c) (Ericson, Real-Time Collision Detection 5.1.5)
const sub = (a, b) => [a[0] - b[0], a[1] - b[1], a[2] - b[2]];
const dot = (a, b) => a[0] * b[0] + a[1] * b[1] + a[2] * b[2];
function dist(p, a, b, c) {
  const ab = sub(b, a), ac = sub(c, a), ap = sub(p, a);
  const d1 = dot(ab, ap), d2 = dot(ac, ap);
  let q;
  if (d1 <= 0 && d2 <= 0) q = a;
  else {
    const bp = sub(p, b), d3 = dot(ab, bp), d4 = dot(ac, bp);
    if (d3 >= 0 && d4 <= d3) q = b;
    else {
      const vc = d1 * d4 - d3 * d2;
      if (vc <= 0 && d1 >= 0 && d3 <= 0) { const v = d1 / (d1 - d3); q = [a[0] + v * ab[0], a[1] + v * ab[1], a[2] + v * ab[2]]; }
      else {
        const cp = sub(p, c), d5 = dot(ab, cp), d6 = dot(ac, cp);
        if (d6 >= 0 && d5 <= d6) q = c;
        else {
          const vb = d5 * d2 - d1 * d6;
          if (vb <= 0 && d2 >= 0 && d6 <= 0) { const w = d2 / (d2 - d6); q = [a[0] + w * ac[0], a[1] + w * ac[1], a[2] + w * ac[2]]; }
          else {
            const va = d3 * d6 - d5 * d4;
            if (va <= 0 && d4 - d3 >= 0 && d5 - d6 >= 0) { const w = (d4 - d3) / (d4 - d3 + (d5 - d6)); q = [b[0] + w * (c[0] - b[0]), b[1] + w * (c[1] - b[1]), b[2] + w * (c[2] - b[2])]; }
            else { const den = 1 / (va + vb + vc), v = vb * den, w = vc * den; q = [a[0] + ab[0] * v + ac[0] * w, a[1] + ab[1] * v + ac[1] * w, a[2] + ab[2] * v + ac[2] * w]; }
          }
        }
      }
    }
  }
  const d = sub(p, q);
  return Math.sqrt(dot(d, d));
}
const near = (p) => {
  const cell = grid.get(key(Math.floor(p[0] / CELL), Math.floor(p[1] / CELL), Math.floor(p[2] / CELL)));
  if (!cell) return false;
  for (const t of cell) if (dist(p, t[0], t[1], t[2]) < TOL) return true;
  return false;
};

// ---- which triangles of the part file are the part: its center and corners on the reference
const pick = async () => {
  const doc = await io.read(src);
  const sel = new Map(); // primitive -> [keep in part?] per triangle
  let nPart = 0, nAll = 0;
  for (const node of doc.getRoot().listNodes()) {
    if (!node.getMesh()) continue;
    for (const prim of node.getMesh().listPrimitives()) {
      const flags = [];
      for (const [a, b, c] of triangles(node, prim)) {
        const ctr = [(a[0] + b[0] + c[0]) / 3, (a[1] + b[1] + c[1]) / 3, (a[2] + b[2] + c[2]) / 3];
        const on = near(ctr) && near(a) && near(b) && near(c);
        flags.push(on);
        nPart += on; nAll++;
      }
      sel.set(prim, flags);
    }
  }
  return { doc, sel, nPart, nAll };
};

// keep only the triangles whose flag is `want`, as indexed primitives
function keepOnly(doc, sel, want) {
  for (const [prim, flags] of sel) {
    const idx = prim.getIndices(), pos = prim.getAttribute('POSITION');
    const n = idx ? idx.getCount() : pos.getCount();
    const out = [];
    for (let k = 0, t = 0; k + 2 < n; k += 3, t++) {
      if (flags[t] !== want) continue;
      for (let q = 0; q < 3; q++) out.push(idx ? idx.getScalar(k + q) : k + q);
    }
    if (!out.length) { prim.dispose(); continue; }
    const arr = pos.getCount() > 65535 ? new Uint32Array(out) : new Uint16Array(out);
    const acc = doc.createAccessor().setType('SCALAR').setArray(arr).setBuffer(doc.getRoot().listBuffers()[0]);
    prim.setIndices(acc);
    compactPrimitive(prim); // (drops the vertices no triangle uses any more)
  }
}

const a = await pick();
console.log(`reference: ${nRef} triangles; ${a.nPart} of ${a.nAll} triangles are the part`);
keepOnly(a.doc, a.sel, false);
await a.doc.transform(prune());
await io.write(outRest, a.doc);

const b = await pick();
keepOnly(b.doc, b.sel, true);
// origin on the pivot
const scene = b.doc.getRoot().listScenes()[0];
const holder = b.doc.createNode('pivot').setTranslation([-pivot[0], -pivot[1], -pivot[2]]);
for (const c of [...scene.listChildren()]) { scene.removeChild(c); holder.addChild(c); }
scene.addChild(holder);
await b.doc.transform(prune());
await io.write(outPart, b.doc);
console.log(`wrote ${outRest} and ${outPart} (origin at ${pivot.join(', ')})`);
