// What the game's modules expect of a page, for the physics thread (js/simWorker.js), imported
// before them: a canvas to draw textures on (an OffscreenCanvas: the worker's own copies of the
// field's and robots' textures are never shown) and localStorage (the page's custom autos, sent
// over when the worker starts: simWorker.js setStorage).
const store = new Map();
export function setStorage(entries) { store.clear(); for (const [k, v] of Object.entries(entries || {})) store.set(k, String(v)); }
globalThis.localStorage ??= {
  getItem: (k) => (store.has(k) ? store.get(k) : null),
  setItem: (k, v) => { store.set(k, String(v)); },
  removeItem: (k) => { store.delete(k); },
};
globalThis.document ??= {
  createElement: (tag) => {
    if (tag !== 'canvas') return { style: {} };
    const c = new OffscreenCanvas(1, 1);
    c.style = {};
    return c;
  },
  createElementNS: (ns, tag) => globalThis.document.createElement(tag),
};
