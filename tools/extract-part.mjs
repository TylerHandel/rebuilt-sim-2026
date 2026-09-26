// Pull one subassembly out of a robot CAD export (GLB from tools/onshape-export.sh) as a
// light, flat-shaded GLB in the robot model's frame: x forward, y up, z to the side, with the
// origin on a pivot. The CAD is Z-up with the robot's front toward -Y (as 2910's is).
//   npm i --no-save @gltf-transform/core @gltf-transform/extensions @gltf-transform/functions meshoptimizer
//   node tools/extract-part.mjs robot.glb out.glb "Pivoting Intake Assembly" <pivot y> <pivot z>
// 2910's intake: "Pivoting Intake Assembly", pivot -0.273 0.170 -> cad/robots/2910-intake.glb
import { NodeIO } from '@gltf-transform/core';
import { ALL_EXTENSIONS } from '@gltf-transform/extensions';
import { prune, dedup, weld, simplify, flatten, join } from '@gltf-transform/functions';
import { MeshoptSimplifier } from 'meshoptimizer';

const [src, dst, name, py, pz] = process.argv.slice(2);
if (!pz) { console.log('usage: node tools/extract-part.mjs robot.glb out.glb "<subassembly name>" <pivot y> <pivot z>'); process.exit(1); }
const DROP = /screw|bolt|nut\b|washer|rivet|spacer|chain|belt|tensioner|bearing|gear|sprocket|kraken|motor|plug|collar|pin\b|hub|insert|pulley|retaining|wcp-0982|^9\d{4}a/i;
const io = new NodeIO().registerExtensions(ALL_EXTENSIONS);
const doc = await io.read(src);
const root = doc.getRoot();
const scene = root.listScenes()[0];
const target = root.listNodes().find((n) => n.getName().startsWith(name));
if (!target) { console.log('no node named', name); process.exit(1); }
const world = target.getWorldMatrix();
const all = [];
target.traverse((n) => { if (n !== target) all.push(n); });
for (const n of all) if (!n.isDisposed() && DROP.test(n.getName())) { n.traverse((c) => c !== n && c.dispose()); n.dispose(); }
for (const c of [...scene.listChildren()]) scene.removeChild(c);
target.setMatrix(world);
// model (x, y, z) = (-yc, zc, -xc), minus the pivot (column-major matrix)
const T = [0, 0, -1, 0, -1, 0, 0, 0, 0, 1, 0, 0, +py, -pz, 0, 1];
scene.addChild(doc.createNode('pivot').setMatrix(T).addChild(target));
// flat CAD surfaces: drop the split normals so vertices weld and the simplifier can work
for (const m of root.listMeshes()) for (const p of m.listPrimitives()) { p.setAttribute('NORMAL', null); p.setAttribute('TEXCOORD_0', null); }
await MeshoptSimplifier.ready;
await doc.transform(prune(), dedup(), weld({ tolerance: 0.0002 }), simplify({ simplifier: MeshoptSimplifier, ratio: 0.08, error: 0.02 }), flatten(), join({ keepNamed: false }), prune());
let tris = 0;
root.listMeshes().forEach((m) => m.listPrimitives().forEach((p) => { tris += (p.getIndices() || p.getAttribute('POSITION')).getCount() / 3; }));
await io.write(dst, doc);
console.log(`wrote ${dst}: ${tris} triangles`);
