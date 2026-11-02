// In-browser frame time of a 3v3 (Playwright + Chromium): starts serve.py, sets up a 3v3 of AIs on
// a graphics preset, lets it play, and reads the Show FPS breakdown (js/perf.js: physics, game,
// render CPU, GPU where measurable) plus the renderer's draw calls, triangles and shadow passes.
//   node tools/browserperf.mjs [--preset medium] [--secs 20] [--size 1280x720] [--port 8790]
//        [--teams 2910,1678,4414:6328,7769,1690] [--headed] [--shot out.png] [--query worker]
// Headless Chromium draws with SwiftShader (the GPU's work on the CPU): its GPU number isn't the
// user's, the CPU-side numbers (physics, game, three.js's own render work) are comparable.
import { spawn } from 'node:child_process';
import { createRequire } from 'node:module';
import { dirname, join } from 'node:path';
import { fileURLToPath } from 'node:url';

const require = createRequire(import.meta.url);
const { chromium } = require(process.env.PLAYWRIGHT_MODULE || '/opt/node22/lib/node_modules/playwright');
const args = process.argv.slice(2);
const arg = (k, d) => { const i = args.indexOf('--' + k); return i >= 0 ? args[i + 1] : d; };
const root = join(dirname(fileURLToPath(import.meta.url)), '..');
const port = +arg('port', 8790), preset = arg('preset', 'medium'), secs = +arg('secs', 20);
const [W, H] = arg('size', '1280x720').split('x').map(Number);
const [blue, red] = arg('teams', '2910,1678,4414:6328,7769,1690').split(':').map((s) => s.split(','));

const srv = spawn('python3', ['serve.py', String(port)], { cwd: root, stdio: 'ignore' });
await new Promise((r) => setTimeout(r, 1200));
const browser = await chromium.launch({ headless: !args.includes('--headed'), args: ['--enable-unsafe-swiftshader', '--ignore-gpu-blocklist'] });
try {
  const page = await browser.newPage({ viewport: { width: W, height: H } });
  const errors = [];
  page.on('pageerror', (e) => errors.push(String(e)));
  page.on('console', (m) => { if (m.type() === 'error' && !/404|Failed to load resource/.test(m.text())) errors.push(m.text()); });
  const starts = ['leftTrench', 'hub', 'rightBump'];
  const slots = [...blue, ...red].map((robot, i) => ({ robot, auto: 'best', start: starts[i % 3], driver: 'scorer', skill: 'champs' }));
  await page.addInitScript(([s]) => { localStorage.setItem('rebuiltSim.settings', JSON.stringify(s)); }, [{ matchMode: '3v3', slots, graphics: preset, fps: 'on', dynRes: 'off', camera: 'driver', sound: 0 }]);
  const q = arg('query') ? '?' + arg('query') : '';
  await page.goto(`http://127.0.0.1:${port}/index.html${q}`, { waitUntil: 'load' });
  await page.waitForFunction(() => window.__sim && window.__sim.ui, null, { timeout: 120000 });
  await page.evaluate(() => window.__sim.startMatch());
  // (the CAD loads in the background: give it time, then measure)
  await page.waitForTimeout(6000);
  const t0 = Date.now();
  const samples = [];
  while (Date.now() - t0 < secs * 1000) {
    await page.waitForTimeout(1000);
    samples.push(await page.evaluate(() => {
      const s = window.__sim, st = s.stats?.last ?? null, info = s.renderer.info;
      return { merged: s.merger ? JSON.stringify(s.merger.stats) : "", st, calls: info.render.calls, tris: info.render.triangles, geos: info.memory.geometries, tex: info.memory.textures, t: s.game?.match?.t ?? 0, prof: s.profRows ? s.profRows() : null };
    }));
  }
  const ok = samples.filter((x) => x.st);
  const avg = (f) => ok.reduce((a, x) => a + f(x), 0) / ok.length;
  const matchT = samples.length ? samples[samples.length - 1].t - samples[0].t : 0;
  console.log(`${preset} ${W}x${H}, 3v3 ${blue.join(',')} v ${red.join(',')}: ${avg((x) => x.st.fps).toFixed(1)} fps, frame ${avg((x) => x.st.frame).toFixed(1)} ms = physics ${avg((x) => x.st.physics).toFixed(1)} (${avg((x) => x.st.steps).toFixed(0)} steps/s) + game ${avg((x) => x.st.game).toFixed(1)} + render CPU ${avg((x) => x.st.render).toFixed(1)}; GPU ${ok.some((x) => x.st.gpu !== null) ? avg((x) => x.st.gpu ?? 0).toFixed(1) + ' ms' : 'n/a'}; match clock ran ${(matchT / ((samples.length - 1) || 1)).toFixed(2)} s per s`);
  console.log(`draw calls ${avg((x) => x.calls).toFixed(0)}, triangles ${(avg((x) => x.tris) / 1e3).toFixed(0)}k per frame (last render pass); geometries ${samples.at(-1)?.geos}, textures ${samples.at(-1)?.tex}`);
  if (samples.at(-1)?.st?.parts) console.log("sim parts, self ms a frame: " + samples.at(-1).st.parts.map((p) => `${p.label} ${p.ms.toFixed(2)}`).join(", "));
  console.log(`merged: ${samples.at(-1)?.merged}`);
  // --census: what's in the scene, by top-level object: visible meshes (draw calls before culling),
  // triangles, shadow casters
  if (args.includes('--census')) {
    const rows = await page.evaluate(() => {
      const s = window.__sim, out = [];
      for (const top of s.scene.children) {
        const r = { name: top.name || top.type, ud: Object.keys(top.userData || {}).join(','), meshes: 0, tris: 0, casters: 0, casterTris: 0, instanced: 0, mats: new Set(), sig: new Set(), sigc: new Set(), transp: 0 };
        const sig = (m) => [m.type, m.side, m.transparent, m.opacity, m.map?.uuid, m.roughness, m.metalness, m.emissive?.getHex(), m.emissiveIntensity, m.flatShading, m.vertexColors, m.wireframe, m.depthWrite, m.alphaTest, m.envMapIntensity, m.normalMap?.uuid].join('|');
        top.traverseVisible((o) => {
          if (!o.isMesh || o.layers.mask === 0) return;
          const g = o.geometry, n = (g.index ? g.index.count : g.attributes.position?.count ?? 0) / 3 * (o.isInstancedMesh ? o.count : 1);
          r.meshes++; r.tris += n; if (o.castShadow) { r.casters++; r.casterTris += n; } if (o.isInstancedMesh) r.instanced++;
          r.mats.add(Array.isArray(o.material) ? o.material.map((m) => m.uuid).join() : o.material.uuid);
          if (!Array.isArray(o.material)) { r.sig.add(sig(o.material)); r.sigc.add(sig(o.material) + o.material.color?.getHex()); if (o.material.transparent) r.transp++; }
        });
        r.mats = r.mats.size; r.sig = r.sig.size; r.sigc = r.sigc.size;
        if (r.meshes) out.push(r);
      }
      return out.sort((a, b) => b.meshes - a.meshes);
    });
    for (const r of rows.slice(0, 25)) console.log(`${(r.name + ' ' + r.ud).slice(0, 40).padEnd(40)} meshes ${String(r.meshes).padStart(5)} tris ${(r.tris / 1000).toFixed(0).padStart(6)}k casters ${String(r.casters).padStart(5)} (${(r.casterTris / 1000).toFixed(0)}k) materials ${r.mats} (${r.sigc} distinct, ${r.sig} but color; ${r.transp} transparent)${r.instanced ? ' instanced ' + r.instanced : ''}`);
  }
  if (arg('shot')) await page.screenshot({ path: arg('shot') });
  if (errors.length) console.log('page errors:', errors.slice(0, 5).join('\n'));
} finally {
  await browser.close();
  srv.kill();
}
