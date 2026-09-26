// Slim a CAD GLB for the browser: drop FUEL (the game simulates its own), the carpet and tape (the
// game draws them), and small hardware; then dedup, simplify and merge meshes by material.
//   npm i --no-save @gltf-transform/core @gltf-transform/extensions @gltf-transform/functions meshoptimizer
//   node tools/slim-glb.mjs in.glb cad/field/field.glb [--ratio 0.2] [--error 0.002]
// --error is the simplifier's allowed deviation relative to each mesh's size.
import { NodeIO } from '@gltf-transform/core';
import { ALL_EXTENSIONS } from '@gltf-transform/extensions';
import { prune, dedup, weld, simplify, flatten, join } from '@gltf-transform/functions';
import { MeshoptSimplifier } from 'meshoptimizer';

const argv = process.argv.slice(2);
const opt = (k, d) => (argv.includes(k) ? +argv[argv.indexOf(k) + 1] : d);
const [src, dst] = argv.filter((a, i) => !a.startsWith('--') && !argv[i - 1]?.startsWith('--'));
if (!src || !dst) { console.log('usage: node tools/slim-glb.mjs in.glb out.glb [--ratio 0.2] [--error 0.002]'); process.exit(1); }
const DROP = /fuel|carpet|gaffer tape|seaming tape|rivet|\bpem\b|bolt|nut\b|screw|washer|cable tie|gear|pulley|bearing|\brev-|wheel|hex hub|spacer|standoff|\bbelt\b|shaft|collar|spring|tread|shoulder|4138T/i;

const io = new NodeIO().registerExtensions(ALL_EXTENSIONS);
const doc = await io.read(src);
const root = doc.getRoot();
const tris = () => root.listNodes().reduce((s, n) => s + (n.getMesh()?.listPrimitives() ?? []).reduce((a, p) => a + (p.getIndices() ?? p.getAttribute('POSITION')).getCount() / 3, 0), 0);
const before = tris();
let removed = 0;
for (const n of root.listNodes()) {
  if (!n.isDisposed() && DROP.test(n.getName())) { n.traverse((c) => c !== n && c.dispose()); n.dispose(); removed++; }
}
await MeshoptSimplifier.ready;
await doc.transform(
  prune(), dedup(), weld(),
  simplify({ simplifier: MeshoptSimplifier, ratio: opt('--ratio', 0.2), error: opt('--error', 0.002) }),
  flatten(), join({ keepNamed: false }), prune(),
);
const draws = root.listNodes().reduce((s, n) => s + (n.getMesh()?.listPrimitives().length ?? 0), 0);
await io.write(dst, doc);
console.log(`${src}: ${before} triangles -> ${dst}: ${tris()} triangles, ${draws} meshes (${removed} parts dropped)`);
