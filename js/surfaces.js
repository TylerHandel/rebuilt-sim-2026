// Surface detail: small procedural maps (drawn once on a canvas, seamless, tiled at real scale)
// and a few cheap shader touches, so the field and robots read as carpet, fabric, metal and
// sheet instead of flat colors. Nothing here adds a render pass: a normal map and a roughness map
// on a material it already draws, or a few lines in its shader.
//   setSurfaceDetail(level, aniso): 0 = none (Lowest: the flat look), 1 = small maps (Low,
//   Medium), 2 = full (High and up). Materials register once (detail(m, kind, ...)) and follow.
import * as THREE from 'three';

// ------------------------------------------------------------------ seamless noise
// value noise on a lattice that wraps every `per` cells (so the maps tile)
function hash2(x, y, s) {
  let h = (x * 374761393 + y * 668265263 + s * 2147483647) | 0;
  h = Math.imul(h ^ (h >>> 13), 1274126177);
  return ((h ^ (h >>> 16)) >>> 0) / 4294967296;
}
function vnoise(x, y, per, s) {
  const xi = Math.floor(x), yi = Math.floor(y), fx = x - xi, fy = y - yi;
  const u = fx * fx * (3 - 2 * fx), v = fy * fy * (3 - 2 * fy);
  const m = (a) => ((a % per) + per) % per;
  const a = hash2(m(xi), m(yi), s), b = hash2(m(xi + 1), m(yi), s), c = hash2(m(xi), m(yi + 1), s), d = hash2(m(xi + 1), m(yi + 1), s);
  return a + (b - a) * u + (c - a) * v + (a - b - c + d) * u * v;
}
// a few octaves: x, y in [0, 1) of the tile, base lattice `per` cells across
function fbm(x, y, per, oct, s) {
  let f = 0, amp = 0.5, tot = 0;
  for (let o = 0; o < oct; o++) { f += amp * vnoise(x * per, y * per, per, s + o * 17); tot += amp; amp *= 0.5; per *= 2; }
  return f / tot;
}

// ------------------------------------------------------------------ height fields (0..1, size x size, wrapping)
const HEIGHT = {
  // short loop-pile carpet: rows of tufts (64 per tile across, alternate rows offset half a tuft),
  // each loop a rounded bump with its own height, over a fine fiber noise and a slow mottle
  carpet(S) {
    const h = new Float32Array(S * S), N = 64;
    for (let y = 0; y < S; y++) for (let x = 0; x < S; x++) {
      const u = x / S, v = y / S;
      const ry = v * N, row = Math.floor(ry);
      const rx = u * N + (row & 1) * 0.5, col = Math.floor(rx);
      const fx = rx - col - 0.5, fy = ry - row - 0.5;
      const jit = hash2(((col % N) + N) % N, row % N, 3);
      const loop = Math.max(0, 1 - (fx * fx * 3.2 + fy * fy * 2.6) * 4) * (0.7 + 0.3 * jit);
      h[y * S + x] = 0.65 * Math.sqrt(loop) + 0.25 * fbm(u, v, 128, 2, 7) + 0.1 * fbm(u, v, 8, 3, 11);
    }
    return h;
  },
  // plain-weave nylon (bumper fabric): 32 threads each way per tile, over and under in turn
  weave(S) {
    const h = new Float32Array(S * S), N = 32;
    for (let y = 0; y < S; y++) for (let x = 0; x < S; x++) {
      const u = x / S * N, v = y / S * N, i = Math.floor(u), j = Math.floor(v);
      const fu = u - i, fv = v - j;
      const warp = Math.sin(Math.PI * fu) ** 0.6, weft = Math.sin(Math.PI * fv) ** 0.6; // the threads' round backs
      const over = (i + j) & 1;
      // the thread on top at this crossing, rising to the crossing's middle along its length
      const up = over ? warp * (0.55 + 0.45 * Math.sin(Math.PI * fv)) : weft * (0.55 + 0.45 * Math.sin(Math.PI * fu));
      h[y * S + x] = up * (0.9 + 0.1 * vnoise(u * 4, v * 4, N * 4, 5));
    }
    return h;
  },
};

// a normal map from a wrapping height field (strength: slope per texel scale)
function normalTexture(h, S, strength) {
  const c = document.createElement('canvas');
  c.width = c.height = S;
  const g = c.getContext('2d'), img = g.createImageData(S, S), d = img.data;
  for (let y = 0; y < S; y++) for (let x = 0; x < S; x++) {
    const l = h[y * S + (x + S - 1) % S], r = h[y * S + (x + 1) % S];
    const t = h[((y + S - 1) % S) * S + x], b = h[((y + 1) % S) * S + x];
    // (canvas y runs down, texture v up: the y slope flips)
    let nx = (l - r) * strength, ny = (b - t) * strength, nz = 1;
    const k = 1 / Math.hypot(nx, ny, nz);
    const i = 4 * (y * S + x);
    d[i] = (nx * k * 0.5 + 0.5) * 255; d[i + 1] = (ny * k * 0.5 + 0.5) * 255; d[i + 2] = (nz * k * 0.5 + 0.5) * 255; d[i + 3] = 255;
  }
  g.putImageData(img, 0, 0);
  return finish(new THREE.CanvasTexture(c));
}
// a roughness map (in the green channel, as three.js reads it): lo..hi over a slow seamless mottle
function roughTexture(S, lo, hi, per, seed) {
  const c = document.createElement('canvas');
  c.width = c.height = S;
  const g = c.getContext('2d'), img = g.createImageData(S, S), d = img.data;
  for (let y = 0; y < S; y++) for (let x = 0; x < S; x++) {
    const n = fbm(x / S, y / S, per, 4, seed), v = (lo + (hi - lo) * Math.min(1, Math.max(0, (n - 0.25) * 2))) * 255;
    const i = 4 * (y * S + x);
    d[i] = d[i + 1] = d[i + 2] = v; d[i + 3] = 255;
  }
  g.putImageData(img, 0, 0);
  return finish(new THREE.CanvasTexture(c));
}
function finish(t) {
  t.wrapS = t.wrapT = THREE.RepeatWrapping;
  t.colorSpace = THREE.NoColorSpace;
  return t;
}

// the maps by kind and level (cached): size, how steep, and the roughness mottle where there is one
const KINDS = {
  carpet: { size: [0, 256, 512], strength: 2.6, rough: [0.72, 1.0, 6, 31] },
  weave: { size: [0, 128, 256], strength: 1.6 },
};
const CACHE = new Map();
function maps(kind, level) {
  const key = kind + level;
  if (!CACHE.has(key)) {
    const K = KINDS[kind], S = K.size[level];
    const m = { normal: normalTexture(HEIGHT[kind](S), S, K.strength * S / 256) };
    if (K.rough) m.rough = roughTexture(Math.min(S, 256), ...K.rough);
    CACHE.set(key, m);
  }
  return CACHE.get(key);
}

// ------------------------------------------------------------------ registered materials
const REG = [];
let LEVEL = 0, ANISO = 1;
// m: the material; kind: a KINDS map; repeat: [u, v] tiles over the material's uv range (the
// normal map's), roughRepeat: the roughness map's (a different, non-multiple scale, so the two
// never line up into a visible repeat); normalScale: how deep
export function detail(m, kind, { repeat = [1, 1], roughRepeat = null, normalScale = 1 } = {}) {
  REG.push({ m, kind, repeat, roughRepeat, normalScale, rough0: m.roughness, sheen0: m.sheen ?? 0 });
  applyDetail(REG[REG.length - 1]);
  return m;
}
function applyDetail(e) {
  const { m, kind } = e;
  if (!LEVEL) {
    m.normalMap = null; m.roughnessMap = null; m.roughness = e.rough0; if (e.sheen0) m.sheen = 0; // (a cloth sheen with the detail only)
  } else {
    const t = maps(kind, LEVEL);
    // (each material its own texture objects over the shared image: its own repeat)
    const own = (tex, rep, slot) => {
      const c = (e[slot] && e[slot].source === tex.source) ? e[slot] : (e[slot] = tex.clone());
      c.repeat.set(...rep);
      c.anisotropy = ANISO;
      c.needsUpdate = true;
      return c;
    };
    m.normalMap = own(t.normal, e.repeat, 'nTex');
    m.normalScale.set(e.normalScale, e.normalScale);
    if (e.sheen0) m.sheen = e.sheen0;
    if (t.rough && e.roughRepeat) { m.roughnessMap = own(t.rough, e.roughRepeat, 'rTex'); m.roughness = 1; }
  }
  m.needsUpdate = true;
}
export function setSurfaceDetail(level, aniso = 1) {
  if (level === LEVEL && aniso === ANISO) return;
  LEVEL = level; ANISO = aniso;
  for (const e of REG) applyDetail(e);
  for (const m of SHADER_MATS) { setDefine(m, 'SURF_DETAIL', LEVEL ? '' : null); }
}
export const surfaceLevel = () => LEVEL;

// ------------------------------------------------------------------ shader touches
// Materials whose detail is drawn in the shader (no texture coordinates on CAD parts), on while
// the detail is (SURF_DETAIL)
const SHADER_MATS = [];
function setDefine(m, k, v) {
  m.defines = m.defines || {};
  if (v === null) delete m.defines[k]; else m.defines[k] = v;
  m.needsUpdate = true;
}
function chain(m, fn, key) {
  const prev = m.onBeforeCompile, prevKey = m.customProgramCacheKey?.bind(m);
  m.onBeforeCompile = (sh, r) => { if (prev) prev(sh, r); fn(sh); };
  const k0 = prevKey ? prevKey() : '';
  m.customProgramCacheKey = () => k0 + '|' + key;
  SHADER_MATS.push(m);
  if (LEVEL) setDefine(m, 'SURF_DETAIL', '');
}
const WORLD_POS_VS = (sh) => {
  sh.vertexShader = sh.vertexShader
    .replace('#include <common>', '#include <common>\nvarying vec3 vSurfW;')
    .replace('#include <project_vertex>', `#include <project_vertex>
  { vec4 sw = vec4(transformed, 1.0);
  #ifdef USE_INSTANCING
    sw = instanceMatrix * sw;
  #endif
    vSurfW = (modelMatrix * sw).xyz; }`);
};

// Diamond plate (tread plate) where `region` (GLSL, of the world position w and the world normal
// wn) holds: raised lozenges ~1 in apart, alternating 90 deg, drawn as a bump in the shader on the
// part's flat faces (the CAD has no texture coordinates), metal and glossy on the raised tops so
// they catch the overhead lights; faded where the pattern is finer than the pixels
export function diamondPlate(m, region) {
  chain(m, (sh) => {
    WORLD_POS_VS(sh);
    sh.fragmentShader = sh.fragmentShader
      .replace('#include <common>', `#include <common>
varying vec3 vSurfW;
float dpLug(vec2 q) {
  // a 25.4 mm lattice turned 45 deg; each cell's lug long one way or the other, in a checker
  vec2 r = vec2(q.x + q.y, q.x - q.y) * (0.70710678 / 0.0254);
  vec2 c = floor(r), f = fract(r) - 0.5;
  float flip = mod(c.x + c.y, 2.0);
  vec2 g = flip > 0.5 ? f.yx : f;
  float d = (g.x * g.x) / 0.09 + (g.y * g.y) / 0.0064; // a lozenge ~0.6 x 0.16 of the cell
  return 1.0 - smoothstep(0.55, 1.0, d);
}`)
      .replace('#include <normal_fragment_maps>', `#include <normal_fragment_maps>
#ifdef SURF_DETAIL
  float dpMask = 0.0;
  {
    vec3 w = vSurfW;
    vec3 wn = normalize((vec4(normal, 0.0) * viewMatrix).xyz);
    if (${region}) {
      // the plane's two directions in world space, then its bump's slope along each
      vec3 an = abs(wn);
      vec3 t1 = an.y > 0.7 ? vec3(1.0, 0.0, 0.0) : (an.x > an.z ? vec3(0.0, 0.0, 1.0) : vec3(1.0, 0.0, 0.0));
      vec3 t2 = normalize(cross(wn, t1));
      t1 = cross(t2, wn);
      vec2 q = vec2(dot(w, t1), dot(w, t2));
      float e = 0.0012, h0 = dpLug(q);
      vec2 grad = vec2(dpLug(q + vec2(e, 0.0)) - h0, dpLug(q + vec2(0.0, e)) - h0) / e;
      float fade = clamp(1.6 - length(fwidth(q)) / 0.004, 0.0, 1.0);
      grad *= 0.0016 * fade; // (the lugs ~1.6 mm high)
      vec3 nw = normalize(wn - grad.x * t1 - grad.y * t2);
      normal = normalize((viewMatrix * vec4(nw, 0.0)).xyz);
      dpMask = h0 * fade;
      // (roughness and metalness are read before the normal: set here, before the lighting)
      roughnessFactor = mix(roughnessFactor, 0.22, dpMask);
      metalnessFactor = mix(metalnessFactor, 0.85, dpMask);
    }
  }
#endif`);
  }, 'dp:' + region);
  return m;
}

// Powder coat (and other painted metal): the slight orange peel of the coat, a soft bump in its
// normal and its roughness, in the part's own space (no texture coordinates on CAD parts)
export function orangePeel(m) {
  chain(m, (sh) => {
    sh.vertexShader = sh.vertexShader
      .replace('#include <common>', '#include <common>\nvarying vec3 vPeelP;')
      .replace('#include <begin_vertex>', '#include <begin_vertex>\n  vPeelP = position * length(modelMatrix[0].xyz); // (the part own space in meters: moves with it)');
    sh.fragmentShader = sh.fragmentShader
      .replace('#include <common>', `#include <common>
varying vec3 vPeelP;
float peelH(vec3 p) { return fract(sin(dot(floor(p), vec3(12.9898, 78.233, 37.719))) * 43758.5453); }
float peelN(vec3 p) {
  vec3 i = floor(p), f = fract(p); f = f * f * (3.0 - 2.0 * f);
  return mix(mix(mix(peelH(i), peelH(i + vec3(1,0,0)), f.x), mix(peelH(i + vec3(0,1,0)), peelH(i + vec3(1,1,0)), f.x), f.y),
             mix(mix(peelH(i + vec3(0,0,1)), peelH(i + vec3(1,0,1)), f.x), mix(peelH(i + vec3(0,1,1)), peelH(i + vec3(1,1,1)), f.x), f.y), f.z);
}`)
      .replace('#include <normal_fragment_maps>', `#include <normal_fragment_maps>
#ifdef SURF_DETAIL
  {
    vec3 p = vPeelP * 900.0; // ~1 mm cells
    float fade = clamp(1.5 - length(fwidth(p)) * 0.6, 0.0, 1.0);
    float peel = peelN(p) - 0.5;
    vec3 g = vec3(peelN(p + vec3(0.5, 0.0, 0.0)), peelN(p + vec3(0.0, 0.5, 0.0)), peelN(p + vec3(0.0, 0.0, 0.5))) - (peel + 0.5);
    normal = normalize(normal + 0.06 * fade * g); // (a random bump: its frame doesn't matter)
    roughnessFactor = clamp(roughnessFactor + peel * fade * 0.12, 0.04, 1.0);
  }
#endif`);
  }, 'peel');
  return m;
}

// Clear sheet (polycarbonate, the Driver Station glass) on the presets with reflections, without
// transmission's extra render of the scene: alpha blended for what it lets through, its
// reflections added on top at full strength (premultiplied alpha: a reflection doesn't fade with
// the sheet's opacity, as it does when blended), and more of its tint showing toward the edges
// (seen at a grazing angle, light crosses more of the sheet)
export function clearSheet(m) {
  if (m.userData.sheet) return m;
  m.userData.sheet = true;
  chain(m, (sh) => {
    sh.fragmentShader = sh.fragmentShader.replace('#include <opaque_fragment>', `
#if defined( SURF_DETAIL ) && defined( SHEET_ON )
  {
    float cosV = clamp(dot(normal, normalize(vViewPosition)), 0.0, 1.0);
    float fr = pow(1.0 - cosV, 3.0);
    float a = clamp(diffuseColor.a + (1.0 - diffuseColor.a) * 0.55 * fr, 0.0, 1.0);
    vec3 dif = reflectedLight.directDiffuse + reflectedLight.indirectDiffuse;
    vec3 spc = reflectedLight.directSpecular + reflectedLight.indirectSpecular;
    // (clear sheet scatters little of the light that crosses it: its own color mostly dims what's behind)
    gl_FragColor = vec4((dif * a * 0.25 + spc) / max(a, 0.002), a);
  }
#else
  #include <opaque_fragment>
#endif`);
  }, 'sheet');
  return m;
}
// the sheet look on or off for one material (with the scene's reflections or without)
export function sheetOn(m, on) {
  setDefine(m, 'SHEET_ON', on ? '' : null);
  m.premultipliedAlpha = !!on;
}
