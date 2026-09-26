// Real CAD in the game: GLB models listed in cad/manifest.json are loaded on top of (or
// instead of) the drawn field. Physics colliders always stay the ones in field.js, which are
// built from the game manual dimensions. See cad/README.md.
import * as THREE from 'three';
import { GLTFLoader } from 'three/addons/loaders/GLTFLoader.js';

const D = Math.PI / 180;

export async function loadCadModels(scene, field) {
  let manifest;
  try {
    const res = await fetch('cad/manifest.json', { cache: 'no-cache' });
    if (!res.ok) return [];
    manifest = await res.json();
  } catch { return []; }
  const loader = new GLTFLoader();
  const loaded = [];
  for (const e of manifest.models || []) {
    if (e.enabled === false || e.kind === 'robot') continue;
    try {
      const gltf = await loader.loadAsync('cad/' + e.file);
      const obj = gltf.scene;
      obj.userData.cad = e.file;
      // CAD is usually Z-up in mm; the game is Y-up in m
      const s = e.scale ?? 0.001;
      obj.scale.setScalar(s);
      const [rx, ry, rz] = e.rotationDeg ?? [-90, 0, 0];
      obj.rotation.set(rx * D, ry * D, rz * D);
      const [x, y, z] = e.position ?? [0, 0, 0];
      obj.position.set(x, y, z);
      obj.traverse((o) => {
        if (o.isMesh) {
          o.castShadow = true;
          o.receiveShadow = true;
          if (o.material && e.color) o.material = new THREE.MeshStandardMaterial({ color: e.color, roughness: 0.6, metalness: 0.2 });
          else if (o.material && o.material.transparent && o.material.opacity < 0.7) tintClear(o.material);
        }
      });
      scene.add(obj);
      loaded.push(e.file);
      // a whole-field model replaces the drawn field (the carpet, bleachers, HUMAN PLAYERS, HUB lights and OUTPOST doors stay)
      if (e.kind === 'field') hideDrawnField(field.group);
    } catch (err) {
      console.warn('CAD model failed to load:', e.file, err);
    }
  }
  return loaded;
}

function hideDrawnField(group) {
  const keep = (o) => { for (; o && o !== group; o = o.parent) if (o.userData.floor || o.userData.keepWithCad) return true; return false; };
  group.traverse((o) => { if ((o.isMesh || o.isSprite) && !keep(o)) o.visible = false; });
}

// clear CAD parts (polycarbonate) get a light tint so they read as panels, not holes
function tintClear(m) {
  m.color.lerp(new THREE.Color(0x9fbbd6), 0.5);
  m.opacity = Math.max(m.opacity, 0.22);
  m.depthWrite = false;
}
