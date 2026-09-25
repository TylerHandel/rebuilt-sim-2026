// Loaded with `node --import` before the game modules so they run headless in Node:
// resolves "rapier" and stubs the canvas API the field/robot textures use.
import { register } from 'node:module';

register('./resolve-hook.mjs', import.meta.url);

const ctx2d = new Proxy({}, {
  get(t, k) {
    if (k in t) return t[k];
    if (k === 'getImageData') return () => ({ data: new Uint8ClampedArray(4) });
    if (k === 'measureText') return () => ({ width: 0 });
    if (k === 'createLinearGradient' || k === 'createRadialGradient' || k === 'createPattern') return () => ({ addColorStop() {} });
    return () => {};
  },
  set(t, k, v) { t[k] = v; return true; },
});
globalThis.document ??= { createElement: () => ({ width: 0, height: 0, style: {}, getContext: () => ctx2d }) };
