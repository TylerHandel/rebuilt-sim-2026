// Worker for tools/auto-search.mjs: plays the AUTO-only runs it is sent
import { parentPort } from 'node:worker_threads';
import { runAuto } from './auto-eval.mjs';
import { BLUE, RED } from '../js/constants.js';

parentPort.on('message', async ({ id, a, b, seed }) => {
  try {
    parentPort.postMessage({ id, result: await runAuto(a, b, { seed, allianceA: seed % 2 ? BLUE : RED }) });
  } catch (e) {
    parentPort.postMessage({ id, error: String((e && e.stack) || e) });
  }
});
parentPort.postMessage({ ready: true });
