// Training worker: plays the matches it is sent (each in a fresh physics world).
import { parentPort } from 'node:worker_threads';
import { runMatch } from './headless.mjs';

parentPort.on('message', async ({ id, a, b, allianceA, seed }) => {
  try {
    parentPort.postMessage({ id, result: await runMatch(a, b, { allianceA, seed }) });
  } catch (e) {
    parentPort.postMessage({ id, error: String((e && e.stack) || e) });
  }
});
parentPort.postMessage({ ready: true });
