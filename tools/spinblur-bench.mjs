// How long js/spinBlur.js takes to build the blurred copy of a part (a 100k-triangle torus knot
// and a few big plates), spun in node:  node tools/spinblur-bench.mjs [level]
import * as THREE from 'three';
globalThis.window = globalThis.window || {};
const sb = await import('../js/spinBlur.js');
sb.setSpinBlur(+(process.argv[2] || 1));
const root = new THREE.Group();
const part = new THREE.Group();
root.add(part);
const mat = new THREE.MeshStandardMaterial({ color: 0x808080 });
part.add(new THREE.Mesh(new THREE.TorusKnotGeometry(0.08, 0.02, 1000, 50), mat));
for (let i = 0; i < 4; i++) { const p = new THREE.Mesh(new THREE.BoxGeometry(0.5, 0.01, 0.3), mat); p.position.z = i * 0.05; part.add(p); }
const model = {};
for (let f = 0; f < 30; f++) { sb.spinBlurBegin(); part.rotation.z += 0.8; sb.spinBlurEnd(model, root); }
const t = [...model._spinBlur.tracks.values()][0];
console.log(`builds ${sb.spinBlurStats.builds} in ${sb.spinBlurStats.ms.toFixed(1)} ms; blur ${t.w.toFixed(2)}; its triangles ${t.ov.geometry.attributes.position.count / 3}`);
// (again, warm)
for (let k = 0; k < 3; k++) {
  sb.setSpinBlur(0); sb.setSpinBlur(+(process.argv[2] || 1));
  const b0 = sb.spinBlurStats.ms;
  for (let f = 0; f < 1e6 && !(t.ov && t.ov.geometry); f++) { sb.spinBlurBegin(); part.rotation.z += 0.8; sb.spinBlurEnd(model, root); }
  console.log(`warm build ${(sb.spinBlurStats.ms - b0).toFixed(1)} ms`);
}
