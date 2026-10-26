// Graphics presets: from "Lowest" (half resolution, no shadows, no CAD) for weak laptops up to
// "Max" for a desktop RTX card. Everything here only changes how the game is drawn, never the
// physics.
//
// The NVIDIA side: a browser can't reach DLSS, Reflex or hardware ray tracing (those need a
// native app and NVIDIA's driver SDKs; WebGL/WebGPU don't expose them). What it can run are
// NVIDIA's real-time rendering techniques, done in shaders:
//   - FXAA (Timothy Lottes, NVIDIA 2009): the fast anti-aliasing on Low;
//   - PCSS (Randima Fernando, NVIDIA 2005, "Percentage-Closer Soft Shadows"): shadows that are
//     sharp where an object meets the carpet and soften with distance, on Ultra and Max;
//   - horizon-based ambient occlusion, the idea behind NVIDIA's HBAO+: here as GTAO (the
//     ground-truth refinement of HBAO), darkening creases, the inside of hoppers, under bumpers;
//   - dynamic resolution, the idea behind DLSS's "render low, show high": the game drops its
//     render resolution while it can't hold 60 fps and brings it back when it can (no AI
//     upscaler: the browser scales the frame up).
// "Auto" reads the GPU's name (WEBGL_debug_renderer_info) and picks a preset: GeForce RTX ->
// Ultra (RTX 4080/4090/50-series -> Max), other discrete GPUs -> High, integrated -> Medium,
// software rendering -> Lowest.
import * as THREE from 'three';
import { EffectComposer } from 'three/addons/postprocessing/EffectComposer.js';
import { RenderPass } from 'three/addons/postprocessing/RenderPass.js';
import { SSAARenderPass } from 'three/addons/postprocessing/SSAARenderPass.js';
import { GTAOPass } from 'three/addons/postprocessing/GTAOPass.js';
import { UnrealBloomPass } from 'three/addons/postprocessing/UnrealBloomPass.js';
import { OutputPass } from 'three/addons/postprocessing/OutputPass.js';
import { ShaderPass } from 'three/addons/postprocessing/ShaderPass.js';
import { SMAAPass } from 'three/addons/postprocessing/SMAAPass.js';
import { FXAAShader } from 'three/addons/shaders/FXAAShader.js';
import { RoomEnvironment } from 'three/addons/environments/RoomEnvironment.js';

// scale: render resolution (x the screen's, capped at maxDpr device pixels per CSS pixel)
// aa: 'none' | 'fxaa' | 'msaa' (the canvas's own, or the composer's 4x target) | 'smaa' | 'ssaa'
// shadow: shadow map size (0: none), soft: 'basic' | 'pcf' | 'pcfsoft' | 'pcss'
// ao: 0 off, 1 GTAO, 2 GTAO high quality; bloom; env: image-based reflections (intensity)
// fuelSeg: FUEL sphere segments; aniso: texture filtering; cad: load the CAD models
export const PRESETS = {
  potato: { name: 'Lowest', scale: 0.5, maxDpr: 1, aa: 'none', shadow: 0, soft: 'basic', ao: 0, bloom: false, env: 0, fuelSeg: 8, aniso: 1, cad: false,
    desc: 'Half resolution, no shadows, simple FUEL and the drawn robots and field instead of the CAD models (the CAD part takes effect after a reload). For old or school laptops.' },
  low: { name: 'Low', scale: 0.75, maxDpr: 1, aa: 'fxaa', shadow: 1024, soft: 'pcf', ao: 0, bloom: false, env: 0, fuelSeg: 12, aniso: 2, cad: true,
    desc: '75% resolution, small shadow map, NVIDIA FXAA for the edges.' },
  medium: { name: 'Medium', scale: 1, maxDpr: 1.5, aa: 'msaa', shadow: 2048, soft: 'pcf', ao: 0, bloom: false, env: 0, fuelSeg: 16, aniso: 4, cad: true,
    desc: 'Full resolution, multisampled edges, medium shadows. For integrated graphics.' },
  high: { name: 'High', scale: 1, maxDpr: 2, aa: 'msaa', shadow: 4096, soft: 'pcfsoft', ao: 0, bloom: true, env: 0.12, fuelSeg: 16, aniso: 8, cad: true,
    desc: 'Sharp on high-DPI screens, big soft shadows, glowing HUB lights (bloom) and reflections on the metal.' },
  ultra: { name: 'Ultra', scale: 1, maxDpr: 2, aa: 'msaa', shadow: 4096, soft: 'pcss', ao: 1, bloom: true, env: 0.15, fuelSeg: 20, aniso: 16, cad: true,
    desc: 'High, plus NVIDIA PCSS contact-hardening soft shadows and HBAO+-style ambient occlusion (GTAO). For a gaming GPU (GeForce RTX).' },
  max: { name: 'Max', scale: 1, maxDpr: 2, aa: 'ssaa', shadow: 8192, soft: 'pcss', ao: 2, bloom: true, env: 0.18, fuelSeg: 28, aniso: 16, cad: true,
    desc: 'Everything on: 4x supersampling, an 8K PCSS shadow map, full-quality ambient occlusion, rounder FUEL. Overkill on purpose: an RTX 4080/4090 or better.' },
};
export const PRESET_ORDER = ['potato', 'low', 'medium', 'high', 'ultra', 'max'];

// Selective bloom: only what glows (emissive: the HUB lights, LEDs) goes on this layer, is drawn
// on its own, blurred and added over the frame. (A brightness threshold can't tell a lit white
// panel from a light: under the arena lights both are well over 1.)
const GLOW_LAYER = 11;
const MIX_SHADER = {
  uniforms: { tDiffuse: { value: null }, tGlow: { value: null }, strength: { value: 1 } },
  vertexShader: 'varying vec2 vUv; void main() { vUv = uv; gl_Position = projectionMatrix * modelViewMatrix * vec4(position, 1.0); }',
  fragmentShader: 'uniform sampler2D tDiffuse, tGlow; uniform float strength; varying vec2 vUv; void main() { gl_FragColor = texture2D(tDiffuse, vUv) + strength * vec4(texture2D(tGlow, vUv).rgb, 0.0); }',
};

// ------------------------------------------------------------------ GPU detection
export function detectGPU(renderer) {
  let name = '';
  try {
    const gl = renderer.getContext();
    const ext = gl.getExtension('WEBGL_debug_renderer_info');
    name = ext ? gl.getParameter(ext.UNMASKED_RENDERER_WEBGL) : gl.getParameter(gl.RENDERER);
  } catch { /* no context info */ }
  name = String(name || '');
  const n = name.toLowerCase();
  let preset = 'medium', vendor = '';
  if (/swiftshader|llvmpipe|software|microsoft basic/.test(n)) preset = 'potato';
  else if (/nvidia|geforce|quadro|rtx/.test(n)) {
    vendor = 'NVIDIA';
    const m = n.match(/rtx\s*(\d{2})(\d{2})/);
    if (m && (+m[1] >= 50 || (+m[1] === 40 && +m[2] >= 80))) preset = 'max';
    else if (/rtx/.test(n)) preset = 'ultra';
    else preset = 'high'; // GTX and older GeForce
  } else if (/radeon|amd/.test(n)) {
    vendor = 'AMD';
    preset = /rx\s*[6-9]\d{3}/.test(n) ? 'ultra' : /vega|graphics\b/.test(n) && !/rx/.test(n) ? 'medium' : 'high';
  } else if (/apple/.test(n)) preset = /max|ultra|pro/.test(n) ? 'high' : 'medium';
  else if (/intel/.test(n)) preset = /arc/.test(n) ? 'high' : 'low';
  else if (/mali|adreno|powervr/.test(n)) preset = 'low';
  return { name, vendor, preset };
}

// the preset chosen in the menu before the game starts (for what has to be decided up front)
export function storedPreset() {
  try { return JSON.parse(localStorage.getItem('rebuiltSim.settings') || '{}').graphics || 'auto'; } catch { return 'auto'; }
}

// ------------------------------------------------------------------ PCSS
// NVIDIA's Percentage-Closer Soft Shadows for the sun's (orthographic) shadow map: find the
// average depth of what blocks the light round this point, size the penumbra from how far the
// point is below it (a light of SUN_ANGLE across), then filter over that width. Poisson-disk
// samples, rotated per pixel so the pattern doesn't band.
const PCSS_GLSL = /* glsl */`
#ifdef USE_PCSS
#define PCSS_N 20
const vec2 PCSS_DISK[PCSS_N] = vec2[](
  vec2(-0.613392, 0.617481), vec2(0.170019, -0.040254), vec2(-0.299417, 0.791925), vec2(0.645680, 0.493210),
  vec2(-0.651784, 0.717887), vec2(0.421003, 0.027070), vec2(-0.817194, -0.271096), vec2(-0.705374, -0.668203),
  vec2(0.977050, -0.108615), vec2(0.063326, 0.142369), vec2(0.203528, 0.214331), vec2(-0.667531, 0.326090),
  vec2(-0.098422, -0.295755), vec2(-0.885922, 0.215369), vec2(0.566637, 0.605213), vec2(0.039766, -0.396100),
  vec2(0.751946, 0.453352), vec2(0.078707, -0.715323), vec2(-0.075838, -0.529344), vec2(0.724479, -0.580798));
float pcssShadow(sampler2D map, vec4 sc) {
  float ang = fract(sin(dot(sc.xy, vec2(12.9898, 78.233))) * 43758.5453) * 6.2831853;
  mat2 rot = mat2(cos(ang), sin(ang), -sin(ang), cos(ang));
  // blockers: what's nearer the light within the search radius
  float zR = sc.z, sum = 0.0, n = 0.0;
  for (int i = 0; i < PCSS_N; i++) {
    float d = unpackRGBAToDepth(texture2D(map, sc.xy + rot * PCSS_DISK[i] * PCSS_SEARCH));
    if (d < zR) { sum += d; n += 1.0; }
  }
  if (n < 0.5) return 1.0;
  // penumbra: the receiver's distance below the blockers (m) x the sun's angular size, in uv
  float pen = clamp((zR - sum / n) * PCSS_DEPTH * PCSS_ANGLE * PCSS_UV, PCSS_MIN, PCSS_MAX);
  float lit = 0.0;
  for (int i = 0; i < PCSS_N; i++) lit += step(zR, unpackRGBAToDepth(texture2D(map, sc.xy + rot * PCSS_DISK[i] * pen)));
  return lit / float(PCSS_N);
}
#endif
`;
let pcssPatched = false;
function patchPCSS() {
  if (pcssPatched) return;
  pcssPatched = true;
  const C = THREE.ShaderChunk;
  C.shadowmap_pars_fragment = C.shadowmap_pars_fragment
    .replace('float getShadow( sampler2D shadowMap,', PCSS_GLSL + '\n\tfloat getShadow( sampler2D shadowMap,')
    .replace('\t\t#if defined( SHADOWMAP_TYPE_PCF )', '\t\t#if defined( USE_PCSS )\n\t\t\tshadow = pcssShadow( shadowMap, shadowCoord );\n\t\t#elif defined( SHADOWMAP_TYPE_PCF )');
}

// ------------------------------------------------------------------ the graphics system
export class Graphics {
  // sun: the shadow-casting light; shadowBox: its shadow camera's half sizes (m) and depth range
  constructor({ renderer, scene, camera, sun, fuel, field }) {
    Object.assign(this, { renderer, scene, camera, sun, fuel, field });
    this.gpu = detectGPU(renderer);
    this.key = null;
    this.composer = null;
    this.dynScale = 1;     // dynamic resolution's share of the preset's scale
    this.dynamic = false;
    this.frameT = 1 / 60;  // smoothed frame time
    this.adjustT = 0;
    this.fps = 60;
    this.envTex = null;
  }

  resolve(key) { return key === 'auto' || !PRESETS[key] ? this.gpu.preset : key; }

  // switch to a preset (key: one of PRESETS or 'auto'); dynamic: dynamic resolution on
  apply(key, { dynamic = false } = {}) {
    const k = this.resolve(key);
    const p = PRESETS[k];
    const changed = k !== this.key;
    this.key = k;
    this.p = p;
    this.dynamic = dynamic;
    if (!dynamic) this.dynScale = 1;
    if (!changed && this.composer !== undefined && this._built) { this._size(); return; }
    this._built = true;
    const r = this.renderer, sun = this.sun;
    // shadows
    if (p.soft === 'pcss') patchPCSS();
    r.shadowMap.enabled = p.shadow > 0;
    r.shadowMap.type = p.soft === 'basic' ? THREE.BasicShadowMap : p.soft === 'pcfsoft' ? THREE.PCFSoftShadowMap : THREE.PCFShadowMap;
    sun.castShadow = p.shadow > 0;
    if (p.shadow > 0) {
      const sz = Math.min(p.shadow, r.capabilities.maxTextureSize);
      sun.shadow.mapSize.set(sz, sz / 2);
      if (sun.shadow.map) { sun.shadow.map.dispose(); sun.shadow.map = null; }
      sun.shadow.radius = p.soft === 'pcf' ? 2 : 1;
      // (bias in shadow-map texels: a coarser map needs more, or it streaks)
      sun.shadow.bias = -0.0004 * (4096 / sz);
      sun.shadow.normalBias = 0.02 * (4096 / sz);
    }
    const sc = sun.shadow.camera;
    const defs = p.soft === 'pcss' ? {
      USE_PCSS: '', PCSS_DEPTH: (sc.far - sc.near).toFixed(3), PCSS_ANGLE: '0.035',
      PCSS_UV: (2 / (sc.right - sc.left)).toFixed(5), PCSS_SEARCH: '0.004', PCSS_MIN: '0.0002', PCSS_MAX: '0.006',
    } : null;
    // every material compiles again with the new shadow code
    this.scene.traverse((o) => {
      if (!o.material) return;
      for (const m of Array.isArray(o.material) ? o.material : [o.material]) this._material(m, defs);
    });
    this.pcssDefs = defs;
    // image-based reflections (aluminum, polycarbonate, bumpers)
    if (p.env > 0) {
      if (!this.envTex) {
        const pm = new THREE.PMREMGenerator(r);
        this.envTex = pm.fromScene(new RoomEnvironment(), 0.04).texture;
        pm.dispose();
      }
      this.scene.environment = this.envTex;
      this.scene.environmentIntensity = p.env;
    } else this.scene.environment = null;
    // FUEL: rounder on the higher presets
    if (this.fuel && this.fuel.mesh) {
      const R = this.fuel.mesh.geometry.parameters.radius;
      this.fuel.mesh.geometry.dispose();
      this.fuel.mesh.geometry = new THREE.SphereGeometry(R, p.fuelSeg, Math.round(p.fuelSeg * 0.75));
    }
    // robots built from here on skip their CAD parts on Lowest (window.__gfxNoCad, robotModels.js)
    window.__gfxNoCad = !p.cad;
    this.refresh();
    this._buildComposer();
  }

  // new materials (robots built after the preset was set, CAD that loads later) get the current shadow code
  _material(m, defs) {
    m.defines = m.defines || {};
    for (const k of ['USE_PCSS', 'PCSS_DEPTH', 'PCSS_ANGLE', 'PCSS_UV', 'PCSS_SEARCH', 'PCSS_MIN', 'PCSS_MAX']) delete m.defines[k];
    if (defs) Object.assign(m.defines, defs);
    m.needsUpdate = true;
  }

  _aniso() {
    const a = Math.min(this.p.aniso, this.renderer.capabilities.getMaxAnisotropy());
    this.scene.traverse((o) => {
      if (!o.material) return;
      for (const m of Array.isArray(o.material) ? o.material : [o.material]) {
        if (m.map && m.map.anisotropy !== a) { m.map.anisotropy = a; m.map.needsUpdate = true; }
      }
    });
  }

  // things added to the scene after apply() (CAD that loads later, a new match's robots)
  refresh() {
    if (!this.p) return;
    this.scene.traverse((o) => {
      if (!o.material) return;
      // what glows goes on the glow layer too (emissive, and bright enough to read as a light)
      const m0 = Array.isArray(o.material) ? o.material[0] : o.material;
      if (o.isMesh && m0.emissive && (m0.emissiveIntensity >= 1 || o.userData.glow) && m0.emissive.getHex() !== 0) { o.layers.enable(GLOW_LAYER); o.userData.glow = true; }
      for (const m of Array.isArray(o.material) ? o.material : [o.material]) {
        const has = !!(m.defines && 'USE_PCSS' in m.defines);
        if (has !== !!this.pcssDefs) this._material(m, this.pcssDefs);
      }
    });
    this._aniso();
  }

  _pixelRatio() {
    const dpr = Math.min(window.devicePixelRatio || 1, this.p.maxDpr);
    return Math.max(0.35, dpr * this.p.scale * this.dynScale);
  }

  _buildComposer() {
    for (const c of [this.composer, this.glow?.composer]) {
      if (!c) continue;
      for (const ps of c.passes) ps.dispose?.();
      c.dispose();
    }
    this.composer = null;
    this.glow = null;
    const p = this.p, r = this.renderer;
    r.setPixelRatio(this._pixelRatio());
    const post = p.aa === 'fxaa' || p.aa === 'smaa' || p.aa === 'ssaa' || p.ao || p.bloom;
    if (!post) { this._size(); return; } // straight to the canvas (its own MSAA if it has it)
    const w = window.innerWidth, h = window.innerHeight;
    const target = new THREE.WebGLRenderTarget(w * r.getPixelRatio(), h * r.getPixelRatio(), {
      type: THREE.HalfFloatType, samples: p.aa === 'msaa' ? 4 : 0,
    });
    const c = new EffectComposer(r, target);
    if (p.aa === 'ssaa') {
      const ss = new SSAARenderPass(this.scene, this.camera, 0x0b0d12, 1);
      ss.sampleLevel = 2; // 4 jittered renders per frame
      ss.unbiased = true;
      c.addPass(ss);
    } else c.addPass(new RenderPass(this.scene, this.camera));
    if (p.ao) {
      const ao = new GTAOPass(this.scene, this.camera, w, h);
      ao.blendIntensity = 0.85;
      ao.updateGtaoMaterial({ radius: 0.35, distanceExponent: 1.5, thickness: 1, scale: 1, samples: p.ao > 1 ? 24 : 12 });
      ao.updatePdMaterial({ lumaPhi: 10, depthPhi: 2, normalPhi: 3, radius: p.ao > 1 ? 8 : 4, rings: 2, samples: p.ao > 1 ? 16 : 8 });
      c.addPass(ao);
      this.aoPass = ao;
    }
    this.glow = null;
    if (p.bloom) {
      // the glowing parts alone, blurred (UnrealBloom with no threshold), added over the frame
      const gc = new EffectComposer(r, new THREE.WebGLRenderTarget(w, h, { type: THREE.HalfFloatType }));
      gc.renderToScreen = false;
      // everything in black first (so what's in front of a light hides its glow), then the
      // glowing parts over it in their own colors
      gc.addPass(new RenderPass(this.scene, this.camera, new THREE.MeshBasicMaterial({ color: 0x000000 })));
      const lit = new RenderPass(this.scene, this.camera);
      lit.clear = false;
      const draw = lit.render.bind(lit);
      lit.render = (...a) => { const cam = this.camera, m = cam.layers.mask; cam.layers.set(GLOW_LAYER); draw(...a); cam.layers.mask = m; };
      gc.addPass(lit);
      gc.addPass(new UnrealBloomPass(new THREE.Vector2(w, h), 0.55, 0.2, 0));
      const mix = new ShaderPass(new THREE.ShaderMaterial(MIX_SHADER), 'tDiffuse');
      mix.material.uniforms.tGlow.value = gc.renderTarget2.texture;
      c.addPass(mix);
      this.glow = { composer: gc, mix };
    }
    c.addPass(new OutputPass());
    if (p.aa === 'fxaa') { this.fxaa = new ShaderPass(FXAAShader); c.addPass(this.fxaa); } else this.fxaa = null;
    if (p.aa === 'smaa') c.addPass(new SMAAPass(w * r.getPixelRatio(), h * r.getPixelRatio()));
    this.composer = c;
    this._size();
  }

  _size() {
    const r = this.renderer, w = window.innerWidth, h = window.innerHeight;
    r.setPixelRatio(this._pixelRatio());
    r.setSize(w, h);
    if (this.composer) {
      this.composer.setPixelRatio(r.getPixelRatio());
      this.composer.setSize(w, h);
      if (this.glow) {
        this.glow.composer.setPixelRatio(r.getPixelRatio() * 0.5); // (a blur: half resolution is plenty)
        this.glow.composer.setSize(w, h);
        this.glow.mix.material.uniforms.tGlow.value = this.glow.composer.readBuffer.texture;
      }
      if (this.fxaa) this.fxaa.material.uniforms.resolution.value.set(1 / (w * r.getPixelRatio()), 1 / (h * r.getPixelRatio()));
    }
  }

  resize() { if (this.p) this._size(); }

  // dt: the frame's time, for the FPS readout and dynamic resolution
  render(dt) {
    if (dt > 0) {
      this.frameT += (dt - this.frameT) * 0.05;
      this.fps = 1 / this.frameT;
    }
    if (this.dynamic && dt > 0) {
      // keep ~60 fps: drop the resolution 10% while it's slower, take it back once there's room
      this.adjustT += dt;
      if (this.adjustT > 0.75) {
        this.adjustT = 0;
        const s = this.dynScale;
        if (this.frameT > 1 / 55) this.dynScale = Math.max(0.5, s * 0.9);
        else if (this.frameT < 1 / 62 && s < 1) this.dynScale = Math.min(1, s / 0.92);
        if (this.dynScale !== s) this._size();
      }
    }
    if (this.glow) {
      // the glow, on black, without fog
      const s = this.scene, bg = s.background, fog = s.fog;
      s.background = null; s.fog = null;
      const gc = this.glow.composer;
      gc.render(dt);
      this.glow.mix.material.uniforms.tGlow.value = gc.readBuffer.texture;
      s.background = bg; s.fog = fog;
    }
    if (this.composer) this.composer.render(dt);
    else this.renderer.render(this.scene, this.camera);
  }

  // for the menu: what the current choice does
  describe(key) {
    const k = this.resolve(key), p = PRESETS[k];
    if (key === 'auto') {
      const gpu = this.gpu.name ? `${this.gpu.name.replace(/^ANGLE \((.*)\)$/, '$1')}` : 'your GPU';
      return `Picked from ${gpu}: ${p.name}. ${p.desc}`;
    }
    return p.desc;
  }

  // the render resolution right now (for the FPS readout)
  get resolution() {
    const pr = this.renderer.getPixelRatio();
    return `${Math.round(window.innerWidth * pr)}×${Math.round(window.innerHeight * pr)}`;
  }
}
