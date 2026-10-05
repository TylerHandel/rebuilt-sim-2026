// The neural networks a player can pick: your own latest training on this computer
// (js/nn/driver.json, written by the trainers) and the ones shared to the project
// (js/nn/shared/*.json, uploaded with nn-share.bat). The published site lists the shared ones
// in js/nn/shared/index.json (made when it's deployed); a local server shows the folder.
export const NN_LOCAL = 'js/nn/driver.json';
export const NN_SHARED = 'js/nn/shared/';

// [{ key, name, note }] — key: 'local' or a file in js/nn/shared/
export const nnLibrary = { list: [], ready: false };

const titleOf = (file) => file.replace(/\.json$/, '').replace(/[-_]+/g, ' ');

export async function refreshLibrary() {
  const out = [];
  try {
    const r = await fetch(NN_LOCAL, { method: 'HEAD', cache: 'no-store' });
    if (r.ok) out.push({ key: 'local', name: 'Your training (this computer)', note: 'The network your trainer last saved on this computer.' });
  } catch { /* not served */ }
  let shared = [];
  try {
    const r = await fetch(NN_SHARED + 'index.json', { cache: 'no-store' });
    if (r.ok) shared = ((await r.json()).networks || []).map((n) => ({ key: n.file, name: n.name || titleOf(n.file), note: n.note || '' }));
  } catch { /* no index */ }
  if (!shared.length) {
    try { // a local server's folder listing
      const r = await fetch(NN_SHARED, { cache: 'no-store' });
      if (r.ok) {
        const html = await r.text();
        const files = [...html.matchAll(/href="([^"/?]+\.json)"/g)].map((m) => decodeURIComponent(m[1])).filter((f) => f !== 'index.json');
        shared = [...new Set(files)].map((f) => ({ key: f, name: titleOf(f), note: '' }));
      }
    } catch { /* none */ }
  }
  nnLibrary.list = [...out, ...shared];
  nnLibrary.ready = true;
  return nnLibrary.list;
}

// which network a setting means ('auto': your own training if there is one, else the newest shared)
export function resolveNet(choice) {
  const L = nnLibrary.list;
  const hit = L.find((n) => n.key === choice);
  return hit || L[0] || null;
}

export function netUrl(entry) {
  return entry.key === 'local' ? NN_LOCAL : NN_SHARED + encodeURIComponent(entry.key);
}
