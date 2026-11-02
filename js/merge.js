// Fewer draw calls: a robot's (or the field's) parts that don't move are drawn as one mesh per
// material instead of one each. A 3v3's robots are ~1,000 meshes (plus the field's ~600), every one
// a draw call twice a frame (the shadow map's pass and the view's); merged, a few dozen a robot.
//
// What moves isn't listed anywhere (each robot's model animates its own parts), so it's found by
// watching: a root is merged once it's been drawn a while; every frame each merged part's place is
// checked (its own and its parents' position, rotation, scale, visibility, material and geometry,
// up to the root), and a batch any of them changed in is put back as it was before that frame is
// drawn, those parts marked as moving and left out when the rest are merged again a moment later.
// (A merged part stays where it is in the scene, only off every camera's layers: anything that
// looks it up still finds it.)
// So nothing is ever drawn where it isn't: a part that moves for the first time at the end of a
// match (a climber) comes apart from its batch the frame it moves.
//
// Merged: opaque meshes with one material, no instancing, morphs or custom depth/render hooks,
// grouped by material, shadow flags, layers and render order, baked into the root's frame (a
// mirrored part's triangles turned back round). The material is the same object, so a color that
// changes (a light, an LED) changes on the merged mesh too. ?nomerge in the URL turns it off.
import * as THREE from 'three';
import { mergeGeometries } from 'three/addons/utils/BufferGeometryUtils.js';

const NOOP = THREE.Object3D.prototype.onBeforeRender;
const SETTLE = 1.0; // s (wall clock): how long a root is drawn before it's merged, and after something in it moves
const now = () => performance.now();

export class StaticMerger {
  constructor() {
    this.roots = []; // { root, at (when to merge, performance.now ms; 0: done), moving: Set<Object3D>, batches: [] }
    this.on = true;
    this.stats = { batches: 0, sources: 0, unmerged: 0 };
    this._inv = new THREE.Matrix4();
    this._m = new THREE.Matrix4();
  }

  // root: an Object3D in the scene whose still parts to merge (once it's been drawn SETTLE s)
  track(root) {
    if (this.roots.some((r) => r.root === root)) return;
    this.roots.push({ root, at: now() + SETTLE * 1000, moving: new Set(), batches: [] });
  }

  // each frame, before drawing: put back any batch whose parts moved, merge what's been still
  update(dt) {
    for (let i = this.roots.length - 1; i >= 0; i--) {
      const R = this.roots[i];
      // (a root taken out of the scene, a robot of a match that's over: its merged meshes freed)
      if (!R.root.parent) { for (const b of R.batches) this._unmerge(R, b, false); this.roots.splice(i, 1); continue; }
      for (let k = R.batches.length - 1; k >= 0; k--) {
        const b = R.batches[k], moved = this._moved(b);
        // (or the merged mesh itself changed by something that went through the scene: the CAD
        // hiding the drawn field's parts but those kept with it, js/cadModels.js. Its parts, put
        // back, are as that left them; they merge again in a moment)
        if (moved.length || this._touched(b)) {
          for (const o of moved) R.moving.add(o);
          this._unmerge(R, b, true);
          R.batches.splice(k, 1);
          R.at = now() + SETTLE * 1000;
          this.stats.unmerged++;
        }
      }
      if (!this.on) { for (const b of R.batches) this._unmerge(R, b, true); R.batches = []; R.at = now() + SETTLE * 1000; continue; }
      if (R.at && now() >= R.at) { R.at = 0; this._merge(R); }
    }
    this.stats.batches = this.roots.reduce((n, R) => n + R.batches.length, 0);
    this.stats.sources = this.roots.reduce((n, R) => n + R.batches.reduce((m, b) => m + b.sources.length, 0), 0);
  }

  // a node's state, as checked
  _state(o) {
    const p = o.position, q = o.quaternion, s = o.scale;
    const g = o.geometry, pa = g?.attributes?.position, na = g?.attributes?.normal;
    const m = o.material && !Array.isArray(o.material) ? o.material : null;
    const M = o.matrixAutoUpdate ? null : o.matrix.elements.slice();
    return [p.x, p.y, p.z, q.x, q.y, q.z, q.w, s.x, s.y, s.z, o.visible, o.material, g, pa ? pa.version : 0, na ? na.version : 0, g ? g.drawRange.count : 0, o.parent, o.layers.mask,
      o.castShadow, o.receiveShadow, o.matrixAutoUpdate, M, ...matState(m)];
  }

  // the merged mesh as it was made: shown, on its layers, casting, in its place
  _meshState(m) { return [m.visible, m.layers.mask, m.castShadow, m.receiveShadow, m.parent, m.material, m.renderOrder]; }
  _touched(b) { const s = b.meshSnap, m = b.mesh; return m.visible !== s[0] || m.layers.mask !== s[1] || m.castShadow !== s[2] || m.receiveShadow !== s[3] || m.parent !== s[4] || m.material !== s[5] || m.renderOrder !== s[6]; }

  _moved(b) {
    const out = [];
    for (let i = 0; i < b.nodes.length; i++) {
      const o = b.nodes[i], s = b.snap[i], p = o.position, q = o.quaternion, sc = o.scale;
      // (a source is off every layer while merged: anything else on its layers is a change)
      const g = o.geometry, pa = g?.attributes?.position, na = g?.attributes?.normal;
      if (p.x !== s[0] || p.y !== s[1] || p.z !== s[2] || q.x !== s[3] || q.y !== s[4] || q.z !== s[5] || q.w !== s[6]
        || sc.x !== s[7] || sc.y !== s[8] || sc.z !== s[9] || o.visible !== s[10] || o.material !== s[11] || g !== s[12]
        || (pa ? pa.version : 0) !== s[13] || (na ? na.version : 0) !== s[14] || (g ? g.drawRange.count : 0) !== s[15] || o.parent !== s[16]
        || o.layers.mask !== (b.isSource[i] ? 0 : s[17]) || o.castShadow !== s[18] || o.receiveShadow !== s[19] || o.matrixAutoUpdate !== s[20]
        || (s[21] && !sameMatrix(o.matrix.elements, s[21])) || (b.watchMat && b.isSource[i] && matChanged(o.material, s, 22))) out.push(o);
    }
    return out;
  }

  _eligible(o) {
    if (!o.isMesh || o.isInstancedMesh || o.isSkinnedMesh || !o.visible || o.userData.glow || o.layers.mask !== 1) return false;
    const m = o.material, g = o.geometry;
    if (!m || Array.isArray(m) || m.transparent || !g || !g.attributes.position) return false;
    if (Object.keys(g.morphAttributes).length || o.customDepthMaterial || o.customDistanceMaterial || o.onBeforeRender !== NOOP || !o.frustumCulled) return false;
    if (g.drawRange.start !== 0 || (g.drawRange.count !== Infinity && g.drawRange.count < (g.index ? g.index.count : g.attributes.position.count))) return false;
    return true;
  }

  // the root's still parts, and each moving part's own still parts (a turret's, an intake's: merged
  // into one mesh that moves with it)
  _merge(R) {
    const merged = new Set(R.batches.flatMap((b) => b.sources));
    R.root.updateMatrixWorld(true);
    const under = (o) => { for (let p = o; p; p = p.parent) if (p === R.root) return true; return false; };
    for (const S of [R.root, ...[...R.moving].filter((o) => o !== R.root && under(o))]) this._mergeUnder(R, S, merged);
  }

  _mergeUnder(R, root, merged) {
    this._inv.copy(root.matrixWorld).invert();
    const groups = new Map();
    // (only meshes whose every parent up to this root is visible and not moving: a moving one is a
    // root of its own)
    const walk = (o, ok) => {
      if (o !== root && R.moving.has(o)) return;
      if (o !== root && !o.visible) ok = false;
      if (o.userData.noMerge) return;
      if (ok && o !== root && this._eligible(o) && !merged.has(o)) {
        const g = o.geometry;
        const attrs = Object.keys(g.attributes).sort().map((k) => `${k}${g.attributes[k].itemSize}${g.attributes[k].normalized ? 'n' : ''}${g.attributes[k].array.constructor.name}`).join();
        // (materials alike but for their color merge too, the color in the vertices)
        const key = `${matKey(o.material)}|${o.castShadow}|${o.receiveShadow}|${o.layers.mask}|${o.renderOrder}|${!!g.index}|${attrs}`;
        if (!groups.has(key)) groups.set(key, []);
        groups.get(key).push(o);
      }
      for (const c of o.children) walk(c, ok);
    };
    walk(root, true);
    for (const list of groups.values()) {
      if (list.length < 2) continue;
      // one material, or several alike but for their color: a copy of the first, white, colored by
      // its vertices (the same color: the shader multiplies the material's by the vertices')
      const mats = new Set(list.map((o) => o.material));
      const tint = mats.size > 1 && !list.every((o) => o.material.color && o.material.color.equals(list[0].material.color));
      // (the merged mesh goes in the parts' nearest common parent: whatever goes through the scene
      // by what a mesh is in finds it where they are)
      const up = [];
      for (let n = list[0].parent; n; n = n.parent) { up.push(n); if (n === root) break; }
      let host = 0;
      for (const o of list.slice(1)) {
        const mine = new Set();
        for (let n = o.parent; n; n = n.parent) { mine.add(n); if (n === root) break; }
        let i = 0;
        while (i < up.length - 1 && !mine.has(up[i])) i++;
        host = Math.max(host, i);
      }
      const parent = up[host] ?? root;
      this._inv.copy(parent.matrixWorld).invert();
      const geos = list.map((o) => {
        this._m.multiplyMatrices(this._inv, o.matrixWorld);
        const g = o.geometry.clone();
        g.clearGroups(); // (one material: its groups draw the whole of it)
        for (const k of Object.keys(g.attributes)) if (!['position', 'normal', 'uv', 'uv1', 'uv2', 'color', 'tangent'].includes(k)) g.deleteAttribute(k);
        if (tint) {
          const c = o.material.color, n = g.attributes.position.count, a = new Float32Array(3 * n);
          for (let i = 0; i < n; i++) { a[3 * i] = c.r; a[3 * i + 1] = c.g; a[3 * i + 2] = c.b; }
          g.setAttribute('color', new THREE.BufferAttribute(a, 3));
        }
        g.applyMatrix4(this._m);
        // (a mirrored part: its triangles' winding turned back, as the renderer would for its matrix)
        if (this._m.determinant() < 0) {
          if (g.index) { const a = g.index.array; for (let t = 0; t + 2 < a.length; t += 3) { const x = a[t + 1]; a[t + 1] = a[t + 2]; a[t + 2] = x; } }
          else for (const k of Object.keys(g.attributes)) {
            const at = g.attributes[k], n = at.itemSize, a = at.array;
            for (let t = 0; t + 2 < at.count; t += 3) for (let j = 0; j < n; j++) { const x = a[(t + 1) * n + j]; a[(t + 1) * n + j] = a[(t + 2) * n + j]; a[(t + 2) * n + j] = x; }
          }
        }
        return g;
      });
      let geo = null;
      try { geo = mergeGeometries(geos, false); } catch { geo = null; }
      for (const g of geos) g.dispose();
      if (!geo) continue;
      const src = list[0];
      let material = src.material;
      if (tint) {
        material = src.material.clone();
        material.color.setRGB(1, 1, 1);
        material.vertexColors = true;
        // (what the field's wall fade and the shadow presets added to the shader: js/wallCutaway.js)
        material.onBeforeCompile = src.material.onBeforeCompile;
        if (Object.prototype.hasOwnProperty.call(src.material, 'customProgramCacheKey')) material.customProgramCacheKey = src.material.customProgramCacheKey;
      }
      const mesh = new THREE.Mesh(geo, material);
      mesh.name = `merged (${list.length})`;
      mesh.castShadow = src.castShadow; mesh.receiveShadow = src.receiveShadow;
      mesh.layers.mask = src.layers.mask; mesh.renderOrder = src.renderOrder;
      mesh.userData.glow = src.userData.glow; mesh.userData.merged = true; mesh.userData.noMerge = true;
      mesh.matrixAutoUpdate = false;
      parent.add(mesh);
      mesh.updateMatrixWorld(true);
      // what to watch: the sources and every parent between them and the root
      const nodes = [], seen = new Set(), isSource = [];
      for (const o of list) {
        for (let n = o; n && n !== root; n = n.parent) {
          if (seen.has(n)) continue;
          seen.add(n); nodes.push(n); isSource.push(n === o);
        }
      }
      const snap = nodes.map((n) => this._state(n));
      for (const o of list) o.layers.mask = 0;
      // (several materials drawn as one: a change to any of them is a change; one shared changes the merged mesh too)
      R.batches.push({ mesh, sources: list, nodes, snap, isSource, ownMat: tint, watchMat: mats.size > 1, meshSnap: this._meshState(mesh) });
    }
  }

  // put a batch's parts back on their layers (the ones they were on, unless something's set others)
  _unmerge(R, b, restore) {
    if (restore) for (let i = 0; i < b.nodes.length; i++) if (b.isSource[i] && b.nodes[i].layers.mask === 0) b.nodes[i].layers.mask = b.snap[i][17];
    b.mesh.removeFromParent();
    b.mesh.geometry.dispose();
    if (b.ownMat) b.mesh.material.dispose();
  }
}

// what's watched of a merged part's material: its color, glow, opacity, visibility (anything that
// changes at run time; the rest of a material is set once)
function sameMatrix(a, b) { for (let i = 0; i < 16; i++) if (a[i] !== b[i]) return false; return true; }
function matState(m) {
  if (!m) return [0, 0, 0, 0, 0, 0, 0, 0, true];
  const c = m.color, e = m.emissive;
  return [c ? c.r : 0, c ? c.g : 0, c ? c.b : 0, e ? e.r : 0, e ? e.g : 0, e ? e.b : 0, m.emissiveIntensity ?? 0, m.opacity, m.visible];
}
function matChanged(m, s, k) {
  const c = m.color, e = m.emissive;
  return (c ? c.r : 0) !== s[k] || (c ? c.g : 0) !== s[k + 1] || (c ? c.b : 0) !== s[k + 2] || (e ? e.r : 0) !== s[k + 3] || (e ? e.g : 0) !== s[k + 4]
    || (e ? e.b : 0) !== s[k + 5] || (m.emissiveIntensity ?? 0) !== s[k + 6] || m.opacity !== s[k + 7] || m.visible !== s[k + 8];
}
// a material's identity but for its color: its kind and every other setting (textures and
// functions by identity), so two that draw alike but for the color group together
const FN = new Map();
const SKIP = new Set(['uuid', 'name', 'id', 'color', 'version', 'userData', '_listeners', 'needsUpdate', 'type']);
const KEYS = new WeakMap();
function matKey(m) {
  // (one whose vertices carry colors of their own merges only with others of the same material)
  if (m.vertexColors) return m.uuid;
  const had = KEYS.get(m);
  if (had && had.v === m.version) return had.key;
  const parts = [m.type];
  for (const k of Object.keys(m).sort()) {
    if (SKIP.has(k)) continue;
    const v = m[k];
    if (v === null || v === undefined || typeof v !== 'object' && typeof v !== 'function') parts.push(`${k}=${v}`);
    else if (typeof v === 'function') { if (!FN.has(v)) FN.set(v, FN.size); parts.push(`${k}=f${FN.get(v)}`); }
    else if (v.isColor) parts.push(`${k}=${v.r},${v.g},${v.b}`);
    else if (v.isTexture) parts.push(`${k}=${v.uuid}`);
    else if (v.isVector2 || v.isVector3 || v.isEuler) parts.push(`${k}=${v.toArray()}`);
    else { try { parts.push(`${k}=${JSON.stringify(v)}`); } catch { parts.push(`${k}=${m.uuid}`); } }
  }
  const key = parts.join('|');
  KEYS.set(m, { key, v: m.version });
  return key;
}
