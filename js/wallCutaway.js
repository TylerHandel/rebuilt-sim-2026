// The alliance wall fades out when it's about to hide your robot: as the line from the camera to
// the robot comes down toward the top of an ALLIANCE WALL's solid part (a Driver Station's base
// under its glass, the TOWER WALL, the OUTPOST: field.wallSpans), that whole end of the field (the
// wall and everything behind it, from just in front of its face) fades away, over a short ramp:
// starting when the line passes FADE_START over the solid part's top, gone by FADE_END, so it's
// all the way out by the time it would block anything. A screen-door fade (fragments dropped in a
// fine ordered pattern) on the field's materials (the drawn field and its CAD; never the robots or
// the FUEL), so the CAD's big merged opaque meshes don't need sorting as transparent ones would.
import * as THREE from 'three';
import { HALF_L } from './constants.js';

const FADE_START = 0.35; // m the line to the robot passes over the solid part's top: starts to fade
const FADE_END = 0.1;    // and gone
const BAND = 0.3;        // what fades: from this far in front of the wall's face back
const RATE = 6;          // how fast it follows (1/s)

export class WallCutaway {
  constructor(renderer, field) {
    this.field = field;
    // shared by every material it's on
    this.uniforms = { uCutFade: { value: 0 }, uCutSide: { value: 1 }, uCutX: { value: HALF_L - BAND } };
    this.mats = new Set();
    this.fade = 0;
  }

  // the field's materials under root fade with it (the floor and the carpet are left alone)
  register(root) {
    root.traverse((o) => {
      if (!o.isMesh || o.userData.floor) return;
      for (const m of [].concat(o.material)) {
        if (!m || this.mats.has(m)) continue;
        this.mats.add(m);
        const prev = m.onBeforeCompile, U = this.uniforms;
        m.onBeforeCompile = (sh, r) => {
          if (prev) prev(sh, r);
          Object.assign(sh.uniforms, U);
          sh.vertexShader = sh.vertexShader
            .replace('#include <common>', '#include <common>\nvarying vec3 vCutWorld;')
            .replace('#include <project_vertex>', `#include <project_vertex>
  {
    vec4 cw = vec4(transformed, 1.0);
    #ifdef USE_INSTANCING
      cw = instanceMatrix * cw;
    #endif
    vCutWorld = (modelMatrix * cw).xyz;
  }`);
          sh.fragmentShader = sh.fragmentShader
            .replace('#include <common>', `#include <common>
varying vec3 vCutWorld;
uniform float uCutFade, uCutSide, uCutX;
float cutBayer(vec2 p) {
  // a 4x4 ordered-dither threshold
  ivec2 q = ivec2(mod(p, 4.0));
  int i = q.x + q.y * 4;
  int b[16] = int[16](0, 8, 2, 10, 12, 4, 14, 6, 3, 11, 1, 9, 15, 7, 13, 5);
  return (float(b[i]) + 0.5) / 16.0;
}`)
            .replace('#include <clipping_planes_fragment>', `#include <clipping_planes_fragment>
  if (uCutFade > 0.0 && uCutSide * vCutWorld.x > uCutX && cutBayer(gl_FragCoord.xy) < uCutFade) discard;`);
        };
        const key = m.customProgramCacheKey?.bind(m);
        m.customProgramCacheKey = () => (key ? key() : '') + '|wallfade';
        m.needsUpdate = true;
      }
    });
  }

  // how far below the top of a wall's solid part the line from the camera to the robot crosses it
  // (m; negative: over it), and which end
  _block(C, R) {
    let best = -Infinity, side = 0;
    for (const y of [0.12, 0.3, 0.5]) {
      for (const sp of this.field.wallSpans) {
        const dx = R.x - C.x;
        if (Math.abs(dx) < 1e-6) continue;
        const t = (sp.x - C.x) / dx;
        if (t <= 0 || t >= 1) continue;
        const z = C.z + (R.z - C.z) * t, yy = C.y + (y - C.y) * t;
        if (z < sp.z0 || z > sp.z1) continue;
        if (sp.h - yy > best) { best = sp.h - yy; side = Math.sign(sp.x); }
      }
    }
    return { b: best, side };
  }

  update(cam, robot, dt = 1 / 60) {
    let want = 0;
    if (robot) {
      const { b, side } = this._block(cam.position, robot.pos);
      if (side) {
        // (b = -FADE_START: just starting; b = -FADE_END: gone)
        want = THREE.MathUtils.clamp((b + FADE_START) / (FADE_START - FADE_END), 0, 1);
        if (want > 0) this.uniforms.uCutSide.value = side;
      }
    }
    this.fade += (want - this.fade) * Math.min(1, RATE * dt);
    if (this.fade < 0.01 && want === 0) this.fade = 0;
    this.uniforms.uCutFade.value = this.fade;
  }
}
