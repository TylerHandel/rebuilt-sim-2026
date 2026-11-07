// Same picture? Renders one moment of a 3v3 twice in Chromium (seeded, stepped with __sim.advance,
// so both runs play out alike) with two URL options (e.g. '' and 'nomerge'), saves both PNGs and
// prints how many pixels differ (tools/imgdiff.py), per graphics preset.
//   node tools/shotdiff.mjs --a "" --b nomerge [--presets medium,high] [--secs 8] [--size 960x540]
//        [--out dir] [--camera driver]
import { spawn, spawnSync } from 'node:child_process';
import { createRequire } from 'node:module';
import { mkdirSync } from 'node:fs';
import { dirname, join } from 'node:path';
import { fileURLToPath } from 'node:url';

const require = createRequire(import.meta.url);
const { chromium } = require(process.env.PLAYWRIGHT_MODULE || '/opt/node22/lib/node_modules/playwright');
const args = process.argv.slice(2);
const arg = (k, d) => { const i = args.indexOf('--' + k); return i >= 0 ? args[i + 1] : d; };
const here = dirname(fileURLToPath(import.meta.url)), root = join(here, '..');
const port = +arg('port', 8792), secs = +arg('secs', 8), out = arg('out', '/tmp/shotdiff');
const [W, H] = arg('size', '960x540').split('x').map(Number);
mkdirSync(out, { recursive: true });
const srv = spawn('python3', ['serve.py', String(port)], { cwd: root, stdio: 'ignore' });
await new Promise((r) => setTimeout(r, 1200));
const browser = await chromium.launch({ args: ['--enable-unsafe-swiftshader'] });
const shot = async (preset, q, file) => {
  const page = await browser.newPage({ viewport: { width: W, height: H } });
  page.on('pageerror', (e) => console.log('page error:', String(e)));
  const slots = ['2910', '1678', '4414', '6328', '7769', '1690'].map((robot, i) => ({ robot, auto: 'best', start: ['leftTrench', 'hub', 'rightBump'][i % 3], driver: 'scorer', skill: 'champs' }));
  await page.addInitScript(([s]) => {
    localStorage.setItem('rebuiltSim.settings', JSON.stringify(s));
    let a = 12345; window.__reseed = (s) => { a = s; }; Math.random = () => { a = (a + 0x6d2b79f5) >>> 0; let t = a; t = Math.imul(t ^ (t >>> 15), t | 1); t ^= t + Math.imul(t ^ (t >>> 7), t | 61); return ((t ^ (t >>> 14)) >>> 0) / 4294967296; };
  }, [{ matchMode: '3v3', slots, graphics: preset, fps: 'off', dynRes: 'off', camera: arg('camera', 'driver'), sound: 0, preview: 'off', physThread: 'main' }]);
  await page.goto(`http://127.0.0.1:${port}/index.html${q ? '?' + q : ''}`, { waitUntil: 'load' });
  await page.waitForFunction(() => window.__sim && window.__sim.ui, null, { timeout: 120000 });
  // (the CAD loads in the background)
  await page.waitForTimeout(5000);
  if (preset !== 'potato') await page.waitForFunction(() => window.__sim.scene.children.some((o) => o.userData.cad), null, { timeout: 90000 }).catch(() => console.log('(no field CAD)'));
  await page.evaluate((s) => { const S = window.__sim; S.hold = true; window.__reseed(777); globalThis.SIM_SEED = 777; S.startMatch(); S.advance(s, 60); }, secs);
  // (time for merging, which waits a second of the wall clock, then a frame to draw it)
  await page.waitForTimeout(1500);
  await page.evaluate(() => window.__sim.advance(1 / 60, 60));
  await page.waitForTimeout(1500);
  await page.evaluate(() => window.__sim.advance(1 / 60, 60));
  const info = await page.evaluate(() => ({ calls: window.__sim.renderer.info.render.calls, merged: window.__sim.merger?.stats, pos: window.__sim.game.robots.map((r) => r.pos.x.toFixed(6)).join(","), cam: window.__sim.camera.position.toArray().map((v) => v.toFixed(5)).join(",") }));
  await page.screenshot({ path: file });
  await page.close();
  return info;
};
for (const preset of arg('presets', 'medium,high').split(',')) {
  const fa = join(out, `${preset}-a.png`), fb = join(out, `${preset}-b.png`);
  const ia = await shot(preset, arg('a', ''), fa), ib = await shot(preset, arg('b', 'nomerge'), fb);
  const d = spawnSync('python3', [join(here, 'imgdiff.py'), fa, fb, join(out, `${preset}-diff.png`)], { encoding: 'utf8' });
  console.log(`${preset}: a(${arg('a', '')}) draw calls ${ia.calls} merged ${JSON.stringify(ia.merged)}; b(${arg("b", "nomerge")}) draw calls ${ib.calls}; robots ${ia.pos === ib.pos ? "same" : ia.pos + " vs " + ib.pos}; camera ${ia.cam === ib.cam ? "same" : ia.cam + " vs " + ib.cam}; ${d.stdout.trim()}${d.stderr.trim()}`);
}
await browser.close();
srv.kill();
