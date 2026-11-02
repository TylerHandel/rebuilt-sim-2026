// Does a speed-up change how matches play? Runs a set of headless matches (tools/perf3v3.mjs --hash:
// every robot's and FUEL's state hashed every 0.5 s of the match) in parallel and saves, or
// compares against, the hashes. Bit-identical play prints "same" for every match.
//   node tools/samecheck.mjs --save DIR [--root REPO] [--jobs 4] [--quick]
//   node tools/samecheck.mjs --compare DIR [--root REPO] [--jobs 4] [--quick]
// --root: the copy of the game to run (default this one), e.g. a worktree of the commit before a
// change; --quick: 30 s of each match instead of the whole match. The first differing sample says
// when in the match the play first parted.
import { spawn } from 'node:child_process';
import { mkdirSync, readFileSync, existsSync, writeFileSync } from 'node:fs';
import { dirname, join, resolve } from 'node:path';
import { fileURLToPath } from 'node:url';

const args = process.argv.slice(2);
const arg = (k, d) => { const i = args.indexOf('--' + k); return i >= 0 ? args[i + 1] : d; };
const here = dirname(fileURLToPath(import.meta.url));
const root = resolve(arg('root', join(here, '..')));
const dir = resolve(arg('save') || arg('compare'));
const jobs = +arg('jobs', 4);
const secs = args.includes('--quick') ? ['--secs', '30'] : [];
mkdirSync(dir, { recursive: true });

const CASES = (arg('cases') ? arg('cases').split(';') : [
  '3v3 2910,1678,4414:6328,7769,1690 1',
  '3v3 8793,341,2056:604,581,1114 1',
  '1v1 2910:4414 1', '1v1 2910:4414 2', '1v1 2910:4414 3',
  '1v1 1678:8793 1', '1v1 341:1690 1', '1v1 7769:6328 1', '1v1 2056:604 1', '1v1 581:1706 1', '1v1 1114:4414 2',
]).map((c) => { const [mode, teams, seed] = c.split(' '); return { mode, teams, seed, name: `${mode}_${teams.replace(/[:,]/g, '-')}_s${seed}` }; });

const run = (c) => new Promise((ok) => {
  const out = join(dir, c.name + (arg('compare') ? '.new' : '') + '.txt');
  const p = spawn(process.execPath, ['--import', './tools/node-env.mjs', join(here, 'perf3v3.mjs'), '--' + c.mode, c.teams, '--seed', c.seed, '--noprof', '--hash', '--hashfile', out, ...secs], { cwd: root });
  let log = '';
  p.stdout.on('data', (d) => { log += d; });
  p.stderr.on('data', (d) => { log += d; });
  p.on('close', () => ok({ c, out, log }));
});

const queue = [...CASES], results = [];
await Promise.all(Array.from({ length: jobs }, async () => { while (queue.length) results.push(await run(queue.shift())); }));
let bad = 0;
for (const { c, out, log } of results.sort((a, b) => CASES.indexOf(a.c) - CASES.indexOf(b.c))) {
  const score = (log.match(/score: .*/) || ['(no score: ' + log.slice(-300) + ')'])[0];
  const speed = (log.match(/real-time factor [\d.]+x/) || [''])[0];
  if (arg('save')) { writeFileSync(join(dir, c.name + '.log'), log); console.log(`${c.name}: saved  ${speed}  ${score}`); continue; }
  const ref = join(dir, c.name + '.txt');
  if (!existsSync(ref) || !existsSync(out)) { console.log(`${c.name}: MISSING`); bad++; continue; }
  const A = readFileSync(ref, 'utf8').trim().split('\n'), B = readFileSync(out, 'utf8').trim().split('\n');
  const k = A.findIndex((h, i) => h !== B[i]);
  if (k < 0 && A.length === B.length) console.log(`${c.name}: same  ${speed}  ${score}`);
  else { bad++; console.log(`${c.name}: DIFFERENT from ${k < 0 ? 'the end' : ((k + 1) * 0.5).toFixed(1) + ' s'}  ${speed}  ${score}`); }
}
process.exit(bad ? 1 : 0);
