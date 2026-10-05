// Lists the shared neural networks (js/nn/shared/*.json) in js/nn/shared/index.json, newest
// first, so the published game can offer them. Run by the GitHub Pages deploy.
//   node tools/nn/share-index.mjs [folder]
import fs from 'node:fs';
import path from 'node:path';

const dir = process.argv[2] || 'js/nn/shared';
const nets = [];
for (const file of fs.existsSync(dir) ? fs.readdirSync(dir) : []) {
  if (!file.endsWith('.json') || file === 'index.json') continue;
  try {
    const d = JSON.parse(fs.readFileSync(path.join(dir, file), 'utf8'));
    if (d.format !== 'rebuilt-nn-policy') continue;
    const sh = d.share || {}, info = d.info || {};
    nets.push({ file, name: sh.name || file.replace(/\.json$/, '').replace(/[-_]+/g, ' '), note: sh.note || '', by: sh.by || '',
      robots: sh.robots || info.robots || null, date: sh.date || info.date || '', hours: info.hours ?? null, steps: info.steps ?? null });
  } catch (e) {
    console.warn(`skipping ${file}: ${e.message}`);
  }
}
nets.sort((a, b) => String(b.date).localeCompare(String(a.date)));
fs.mkdirSync(dir, { recursive: true });
fs.writeFileSync(path.join(dir, 'index.json'), JSON.stringify({ networks: nets }, null, 1));
console.log(`${nets.length} shared networks listed`);
