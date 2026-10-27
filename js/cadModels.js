// Real CAD in the game: GLB models listed in cad/manifest.json are loaded on top of (or
// instead of) the drawn field. Physics colliders always stay the ones in field.js, which are
// built from the game manual dimensions. See cad/README.md.
import * as THREE from 'three';
import { GLTFLoader } from 'three/addons/loaders/GLTFLoader.js';
import { registerPoly, powderCoat } from './robotModels.js';
import { driverStationGlass } from './field.js';

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
          else if (o.material && o.material.transparent && o.material.opacity < 0.7 && DS_GLASS.test(o.name)) {
            // the ALLIANCE WALL's glass: clear on every preset, so the field reads through it from
            // the Driver Station (not the HUB's sheet), and it casts no shadow on the carpet
            o.material = driverStationGlass();
            o.castShadow = false;
          } else if (o.material && o.material.transparent && o.material.opacity < 0.7) {
            tintClear(o.material);
            // (polycarbonate, not the HUB's black net: glossy sheet on High and up, a fifth of its
            // light blue-gray showing so the HUB's big panels read)
            if (o.material.color.getHSL({}).l > 0.4) {
              if (!POLY.has(o.material)) POLY.set(o.material, registerPoly(o.material, 0xd3e0ea, 0.2));
              o.material = POLY.get(o.material);
            }
          } else if (o.material && e.kind === 'field' && isFieldAluminum(o.material)) {
            o.material = FIELD_ALU.get(o.material.color.getHex()) ?? FIELD_ALU.set(o.material.color.getHex(), powderCoat(o.material.color.getHex())).get(o.material.color.getHex());
          }
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

// The field's aluminum: the color (230, 230, 230) most of whose parts are aluminum in the field's
// Onshape bill of materials (107 of 123: the HUB's frames), powder-coated
const POLY = new Map(), FIELD_ALU = new Map();
// the field CAD's alliance wall glass (the Driver Stations' panels, round to the OUTPOST): one
// merged part for both ends
const DS_GLASS = /Outpost_Exchange_Border/;
function isFieldAluminum(m) {
  if (!m.color || m.transparent) return false;
  const c = m.color.getHex(), r = (c >> 16) & 255, g = (c >> 8) & 255, b = c & 255;
  return Math.abs(r - 230) <= 2 && Math.abs(g - 230) <= 2 && Math.abs(b - 230) <= 2;
}

function hideDrawnField(group) {
  const keep = (o) => { for (; o && o !== group; o = o.parent) if (o.userData.floor || o.userData.keepWithCad) return true; return false; };
  group.traverse((o) => {
    if ((o.isMesh || o.isSprite) && !keep(o)) o.visible = false;
    if (o.userData.cadOnly) o.visible = true;
  });
}

// clear CAD parts (polycarbonate) get a light tint so they read as panels, not holes
// (real polycarbonate lets ~90% of the light through: faint, so the field reads clearly
// through the alliance wall from the Driver Station)
function tintClear(m) {
  m.color.lerp(new THREE.Color(0xc4d6e8), 0.35);
  m.opacity = Math.min(Math.max(m.opacity, 0.07), 0.1);
  m.depthWrite = false;
}
