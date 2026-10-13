// Node module hook: the browser maps the bare specifier "rapier" with an import map; do the
// same here, pointing at the ES build of the npm package (which has no "type": "module").
// (A copy installed before the physics engine was upgraded still has the old package: it runs on
// that, a little slower, until "npm install" (nn-setup.bat) fetches the new one.)
import { createRequire } from 'node:module';
import { pathToFileURL } from 'node:url';

const require = createRequire(import.meta.url);
const find = (f) => { try { return f(); } catch { return null; } };
// (its ES build sits beside the CommonJS one the package exports for require)
const RAPIER_ES = find(() => pathToFileURL(require.resolve('@dimforge/rapier3d-simd-compat').replace(/\.cjs$/, '.mjs')).href)
  || find(() => pathToFileURL(require.resolve('@dimforge/rapier3d-compat/rapier.es.js')).href);
if (!RAPIER_ES) throw new Error('The physics engine is not installed: run "npm install" in the repo folder (nn-setup.bat does it).');

export async function resolve(specifier, context, next) {
  if (specifier === 'rapier') return { url: RAPIER_ES, format: 'module', shortCircuit: true };
  return next(specifier, context);
}

export async function load(url, context, next) {
  if (url === RAPIER_ES) return next(url, { ...context, format: 'module' });
  return next(url, context);
}
