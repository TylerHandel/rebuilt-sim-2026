// Node module hook: the browser maps the bare specifier "rapier" with an import map; do the
// same here, pointing at the ES build of the npm package (which has no "type": "module").
import { createRequire } from 'node:module';
import { pathToFileURL } from 'node:url';

const require = createRequire(import.meta.url);
const RAPIER_ES = pathToFileURL(require.resolve('@dimforge/rapier3d-compat/rapier.es.js')).href;

export async function resolve(specifier, context, next) {
  if (specifier === 'rapier') return { url: RAPIER_ES, format: 'module', shortCircuit: true };
  return next(specifier, context);
}

export async function load(url, context, next) {
  if (url === RAPIER_ES) return next(url, { ...context, format: 'module' });
  return next(url, context);
}
