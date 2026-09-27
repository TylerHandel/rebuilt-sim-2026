// Measure every robot's hopper capacity (packed the way its intake packs it, hopper.js) and write
// the table the game uses: node --import ./tools/node-env.mjs tools/capacity.mjs
import fs from 'node:fs';
import { measureCapacity, capacityKey, intakePush } from '../js/hopper.js';
import { ROBOTS, ROBOT_ORDER } from '../js/robotConfigs.js';

const table = {};
for (const key of ROBOT_ORDER) {
  const c = ROBOTS[key], push = intakePush(c), ext = c.storage.extLen || 0;
  const out = [];
  for (const [front, lift] of [[c.bay.x1, 0], [c.bay.x1 + ext, 1]]) {
    const n = measureCapacity(c.bay, front, lift, push);
    table[capacityKey(c.bay, front, lift, push)] = n;
    out.push(n);
  }
  console.log(`${key}: intake push ${push.toFixed(0)} N, holds ${out[0]} -> ${out[1]}`);
}
const lines = Object.entries(table).map(([k, n]) => `  '${k}': ${n},`).join('\n');
fs.writeFileSync(new URL('../js/capacities.js', import.meta.url), `// Hopper capacities measured ahead of time by tools/capacity.mjs (packing a hopper takes a second
// or two), keyed by hopper.js capacityKey. A hopper that's changed since is measured live.
export const CAPACITY_TABLE = {
${lines}
};
`);
console.log('wrote js/capacities.js');
