// Training worker: plays the matches it is sent (each in a fresh physics world).
//   1v1: { a, b, allianceA, seed }            -> runMatch's result
//   3v3: { game: 3v3 settings, allianceA, seed } -> { margin, a: { total, foulPtsGiven, ... }, b }
import { parentPort } from 'node:worker_threads';
import { runMatch, runGame, BLUE, RED } from './headless.mjs';

parentPort.on('message', async ({ id, a, b, game, allianceA, seed }) => {
  try {
    let result;
    if (game) {
      const r = await runGame(game, { seed });
      const A = allianceA === RED ? r.red : r.blue, B = allianceA === RED ? r.blue : r.red;
      result = { margin: A.total - B.total, a: A, b: B };
    } else result = await runMatch(a, b, { allianceA, seed });
    parentPort.postMessage({ id, result });
  } catch (e) {
    parentPort.postMessage({ id, error: String((e && e.stack) || e) });
  }
});
parentPort.postMessage({ ready: true });
void BLUE;
