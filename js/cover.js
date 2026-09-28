// Cover-art renderer (open the game with ?cover). Stages a moment of a 3v3 match (three robots in
// the blue ALLIANCE ZONE shooting into their HUB), renders it on this computer's GPU with nicer
// lighting than the game uses (a studio environment for reflections, arena spotlights), motion blur
// on the flying FUEL only, and the title, and exports it as a PNG (1920x1080, 4K, or GitHub's
// 1280x640 social preview).
//
// Motion blur: a sharp base frame with the moving FUEL hidden, then the moving FUEL alone at a few
// points along its path within the shutter (everything else drawn depth-only, so it still hides
// FUEL behind it), in color and as a white matte; averaged in linear light and laid over the base.
import { ROBOTS, ROBOT_ORDER } from './robotConfigs.js';

const CAMERAS = {
  hero: { name: 'Low, looking up at the HUB', p: [-7.7, 0.32, -3.1], at: [-5.2, 1.5, -0.6], fov: 62 },
  sideline: { name: 'Sideline', p: [-6.0, 1.3, -3.8], at: [-4.9, 1.4, 0.2], fov: 55 },
  behind: { name: 'Behind the alliance', p: [-8.0, 2.6, -1.2], at: [-4.6, 1.2, -0.6], fov: 50 },
  high: { name: 'High corner', p: [-7.8, 3.8, -3.6], at: [-4.9, 0.8, -0.6], fov: 50 },
};
// where the three blue robots stand (a row across the hero camera's view, 2-3.5 m from the HUB)
const SPOTS = [[-6.75, -0.55], [-5.8, -1.35], [-4.85, -2.15]];
const SIZES = { '1920x1080': [1920, 1080], '3840x2160': [3840, 2160], '1280x640': [1280, 640] };
const DEFAULTS = {
  blue: ['4414', '2910', '971'], red: ['1678', '1690', '4946'],
  camera: 'hero', height: 0, fov: 62, moment: 1.3, shutter: 8, samples: 8,
  size: '1920x1080', ss: 2, title: true,
  kicker: 'FRC 2026', name: 'REBUILT', accent: 'SIM', tagline: 'All 504 FUEL  ·  real team CAD  ·  play in your browser',
};

const lin = new Float32Array(256).map((_, i) => { const c = i / 255; return c <= 0.04045 ? c / 12.92 : ((c + 0.055) / 1.055) ** 2.4; });
const toSrgb = (v) => { v = Math.max(0, Math.min(1, v)); return 255 * (v <= 0.0031308 ? v * 12.92 : 1.055 * v ** (1 / 2.4) - 0.055); };
const sleep = (ms) => new Promise((r) => setTimeout(r, ms));
const frameWait = () => new Promise((r) => requestAnimationFrame(() => r()));

export async function startCover(sim) {
  const { THREE, renderer, scene, fuel } = sim;
  sim.hold = true;
  const o = { ...DEFAULTS, ...JSON.parse(localStorage.getItem('rebuiltSim.cover') || '{}') };
  const ui = buildUI(o);
  let staged = false, sceneImg = null; // the composite without the title (ImageData)

  // ---------------------------------------------------------------- looks
  const link = document.createElement('link');
  link.rel = 'stylesheet';
  link.href = 'https://fonts.googleapis.com/css2?family=Teko:wght@600;700&family=Rajdhani:wght@600;700&display=swap';
  document.head.appendChild(link);
  const { RoomEnvironment } = await import('three/addons/environments/RoomEnvironment.js');
  scene.environment = new THREE.PMREMGenerator(renderer).fromScene(new RoomEnvironment(), 0.04).texture;
  scene.environmentIntensity = 0.25;
  renderer.toneMappingExposure = 0.95;
  scene.traverse((l) => {
    if (l.isHemisphereLight) l.intensity *= 0.6;
    if (l.isDirectionalLight) l.intensity *= 0.85;
    if (l.isDirectionalLight && l.castShadow) l.shadow.radius = 3;
  });
  for (const [x, z, c, k] of [[-5.2, -3.4, 0xfff1dc, 22], [-3.2, 2.2, 0xdfe8ff, 16], [-7.6, 1.5, 0xffffff, 12]]) {
    const L = new THREE.SpotLight(c, k, 16, 0.75, 0.6, 1.2);
    L.position.set(x, 7.5, z);
    L.target.position.set(-5.2, 0.4, -0.9);
    scene.add(L, L.target);
  }

  // ---------------------------------------------------------------- staging
  async function waitForModels(msg) {
    // CAD parts load in the background: wait until the scene stops gaining meshes
    let last = -1, still = 0;
    for (let t = 0; t < 180 && still < 5; t++) {
      let n = 0;
      scene.traverse((m) => { if (m.isMesh) n++; });
      still = n === last ? still + 1 : 0;
      last = n;
      ui.status(`${msg} (${n} parts)`);
      await sleep(500);
    }
  }

  async function stage() {
    ui.status('Starting the match…');
    const s = sim.settings;
    s.matchMode = '3v3';
    s.slots = [...o.blue, ...o.red].map((robot, i) => ({ robot, auto: 'best', start: ['hub', 'leftTrench', 'rightBump'][i % 3], driver: 'scorer', skill: 'champs' }));
    sim.ui.openPage('3v3');
    sim.startMatch();
    await waitForModels('Loading the robots');
    // run through AUTO to the start of TELEOP (both HUBs active)
    ui.status('Playing AUTO…');
    for (let i = 0; i < 20 && sim.game.match.t < 26.6; i++) { sim.advance(Math.min(2, 26.6 - sim.game.match.t), 60); await frameWait(); }
    // the blue robots: off their AIs, into a row in the ALLIANCE ZONE, hoppers full, shooting
    const g = sim.game, hx = -3.66;
    const idle = { label: 'Cover', strategy: 'scorer', skillKey: 'champs', update() {} };
    if (g.driverAI) g.driverAI = idle;
    const blue = g.units.filter((u) => u.robot.alliance === 'blue');
    blue.forEach((u, i) => {
      const r = u.robot;
      u.ai = idle;
      for (const b of r.stored.splice(0)) fuel._disable(b, 'off');
      r.hopper.clear();
      const [x, z] = SPOTS[i % SPOTS.length];
      let yaw = Math.atan2(z, hx - x); // turrets face the HUB
      if (r.cfg.shooter.type === 'fixed' && r.cfg.shooter.facing !== 'front') yaw += Math.PI; // 2910's and 1678's drums fire out the back
      r.spawn(x, z, yaw);
      r.hopperDeploy = 1;
      r.intakeDeploy = 1;
      r.loadFuel(fuel.balls.filter((b) => b.state === 'field' && b.pos.x > -2.5).slice(0, r.maxCapacity()));
      Object.assign(r.cmd, { vx: 0, vz: 0, omega: 0, intake: false, outtake: false, shoot: true, pass: false, lower: true });
    });
    ui.status('Shooting…');
    sim.advance(o.moment, 60);
    staged = true;
  }

  // ---------------------------------------------------------------- rendering
  const HID = new THREE.Matrix4().makeScale(0, 0, 0);
  const white = new THREE.MeshBasicMaterial({ color: 0xffffff, toneMapped: false });
  const tmp = new THREE.Matrix4(), one = new THREE.Vector3(1, 1, 1), pos = new THREE.Vector3();
  const moving = (b) => { if (b.state !== 'field') return false; const v = b.body.linvel(); return Math.hypot(v.x, v.y, v.z) > 1.5; };

  // pass: 'base' (everything but the moving FUEL), 'color' or 'matte' (only the moving FUEL, moved on
  // along its velocity by dt seconds)
  function renderPass(pass, dt, W, H, ctx) {
    const { camera } = sim;
    fuel.sync();
    for (const b of fuel.balls) {
      if (b.state !== 'field') continue;
      const mv = moving(b);
      if (pass === 'base' ? mv : !mv) { fuel.mesh.setMatrixAt(b.id, HID); continue; }
      if (mv) {
        const v = b.body.linvel();
        pos.set(b.pos.x + v.x * dt, b.pos.y + v.y * dt - 4.9 * dt * dt, b.pos.z + v.z * dt);
        fuel.mesh.setMatrixAt(b.id, tmp.compose(pos, b.quat, one));
      }
    }
    fuel.mesh.instanceMatrix.needsUpdate = true;
    const saved = new Map(), bg = scene.background, fog = scene.fog, mat = fuel.mesh.material;
    if (pass !== 'base') {
      scene.background = new THREE.Color(0x000000);
      scene.traverse((m) => {
        if (!m.isMesh || m === fuel.mesh) return;
        for (const x of [].concat(m.material)) { if (!saved.has(x)) saved.set(x, x.colorWrite); x.colorWrite = false; }
      });
      if (pass === 'matte') { fuel.mesh.material = white; scene.fog = null; }
    }
    renderer.render(scene, camera);
    ctx.clearRect(0, 0, W, H);
    ctx.drawImage(renderer.domElement, 0, 0, W, H);
    for (const [x, v] of saved) x.colorWrite = v;
    scene.background = bg; scene.fog = fog; fuel.mesh.material = mat;
    return ctx.getImageData(0, 0, W, H).data;
  }

  async function render() {
    if (!staged) await stage();
    const [W, H] = SIZES[o.size];
    ui.status('Rendering…');
    await frameWait();
    scene.traverse((m) => { if (m.isLine || m.isLineSegments) m.visible = false; }); // aim previews
    const c = CAMERAS[o.camera], cam = sim.camera;
    sim.rig.mode = 'free';
    cam.position.set(c.p[0], c.p[1] + o.height, c.p[2]);
    cam.up.set(0, 1, 0);
    cam.lookAt(...c.at);
    cam.fov = o.fov;
    cam.aspect = W / H;
    cam.updateProjectionMatrix();
    const prevRatio = renderer.getPixelRatio();
    renderer.setPixelRatio(Math.max(1, Math.min(o.ss, 4000 / W))); // 2x for 1080p, 1x for 4K (GPU limits)
    renderer.setSize(W, H, false);
    const work = document.createElement('canvas');
    work.width = W; work.height = H;
    const ctx = work.getContext('2d', { willReadFrequently: true });
    ctx.imageSmoothingQuality = 'high';

    const n = W * H;
    const base = renderPass('base', 0, W, H, ctx);
    const col = new Float32Array(n * 3), alpha = new Float32Array(n);
    const S = o.shutter > 0 ? Math.max(2, o.samples) : 1, shutter = o.shutter / 1000;
    for (let k = 0; k < S; k++) {
      const dt = S > 1 ? (k / (S - 1) - 0.5) * shutter : 0; // centered on the moment
      const cd = renderPass('color', dt, W, H, ctx), md = renderPass('matte', dt, W, H, ctx);
      for (let i = 0, j = 0; i < n; i++, j += 4) {
        col[i * 3] += lin[cd[j]]; col[i * 3 + 1] += lin[cd[j + 1]]; col[i * 3 + 2] += lin[cd[j + 2]];
        alpha[i] += md[j] / 255;
      }
      ui.status(`Rendering… ${k + 1}/${S}`);
      await frameWait();
    }
    const out = new ImageData(W, H);
    for (let i = 0, j = 0; i < n; i++, j += 4) {
      const a = alpha[i] / S;
      out.data[j] = toSrgb(col[i * 3] / S + lin[base[j]] * (1 - a));
      out.data[j + 1] = toSrgb(col[i * 3 + 1] / S + lin[base[j + 1]] * (1 - a));
      out.data[j + 2] = toSrgb(col[i * 3 + 2] / S + lin[base[j + 2]] * (1 - a));
      out.data[j + 3] = 255;
    }
    renderer.setPixelRatio(prevRatio);
    renderer.setSize(window.innerWidth, window.innerHeight);
    sceneImg = out;
    await compose();
  }

  // the title over the rendered scene
  async function compose() {
    if (!sceneImg) return;
    const W = sceneImg.width, H = sceneImg.height, k = Math.min(W / 1920, H / 1080);
    const cv = ui.canvas;
    cv.width = W; cv.height = H;
    const ctx = cv.getContext('2d');
    ctx.putImageData(sceneImg, 0, 0);
    if (o.title) {
      await Promise.all([document.fonts.load('700 100px Teko'), document.fonts.load('600 40px Rajdhani')]).catch(() => {});
      // a soft dark wash behind the title
      const g = ctx.createRadialGradient(0, 0, 0, 0, 0, 1);
      g.addColorStop(0, 'rgba(8,10,16,0.85)'); g.addColorStop(0.55, 'rgba(8,10,16,0.55)'); g.addColorStop(1, 'rgba(8,10,16,0)');
      ctx.save();
      ctx.scale(1250 * k, 560 * k);
      ctx.fillStyle = g;
      ctx.fillRect(0, 0, 1, 1);
      ctx.restore();
      const x = 82 * k, y = 60 * k;
      const shadow = (b) => { ctx.shadowColor = 'rgba(0,0,0,0.65)'; ctx.shadowBlur = b * k; ctx.shadowOffsetY = 4 * k; };
      ctx.textBaseline = 'top';
      shadow(10);
      ctx.fillStyle = '#f5c630';
      ctx.font = `600 ${46 * k}px Rajdhani, "Barlow Condensed", sans-serif`;
      ctx.fillText(o.kicker, x, y);
      ctx.font = `700 ${200 * k}px Teko, "Barlow Condensed", sans-serif`;
      ctx.fillStyle = '#f5f7fa';
      ctx.fillText(o.name, x - 4 * k, y + 40 * k);
      const w = ctx.measureText(o.name + ' ').width;
      ctx.fillStyle = '#f5c630';
      ctx.fillText(o.accent, x - 4 * k + w, y + 40 * k);
      ctx.shadowColor = 'transparent';
      const sy = y + 250 * k;
      ctx.fillStyle = '#286eff'; ctx.fillRect(x, sy, 150 * k, 8 * k);
      ctx.fillStyle = '#eb2d2d'; ctx.fillRect(x + 158 * k, sy, 150 * k, 8 * k);
      shadow(8);
      ctx.fillStyle = '#f5f7fa';
      ctx.font = `600 ${40 * k}px Rajdhani, "Barlow Condensed", sans-serif`;
      ctx.fillText(o.tagline, x, sy + 24 * k);
      ctx.shadowColor = 'transparent';
    }
    ui.status(`Done: ${W} × ${H}. Download it, or change something and render again.`);
    ui.done(true);
  }

  // ---------------------------------------------------------------- export
  const blobOf = (cv) => new Promise((r) => cv.toBlob(r, 'image/png'));
  function download(blob, name) {
    const a = document.createElement('a');
    a.href = URL.createObjectURL(blob);
    a.download = name;
    a.click();
    setTimeout(() => URL.revokeObjectURL(a.href), 5000);
  }
  // GitHub's social preview (2:1): the middle band of the image, at 1280 x 640
  function socialCanvas() {
    const src = ui.canvas, W = src.width, H = src.height;
    const cv = document.createElement('canvas');
    cv.width = 1280; cv.height = 640;
    const h = Math.min(H, W / 2);
    const ctx = cv.getContext('2d');
    ctx.imageSmoothingQuality = 'high';
    ctx.drawImage(src, 0, (H - h) / 2, W, h, 0, 0, 1280, 640);
    return cv;
  }

  ui.on({
    change(key, value, restage) {
      o[key] = value;
      localStorage.setItem('rebuiltSim.cover', JSON.stringify(o));
      if (key === 'camera') { o.fov = CAMERAS[value].fov; ui.set('fov', o.fov); }
      if (restage) staged = false;
      if (['title', 'kicker', 'name', 'accent', 'tagline'].includes(key)) compose();
    },
    async render() {
      ui.busy(true); ui.done(false);
      try { await render(); } catch (e) { ui.status('Render failed: ' + e.message); console.error(e); }
      ui.busy(false);
    },
    async png() { download(await blobOf(ui.canvas), 'rebuilt-sim-cover.png'); },
    async social() { download(await blobOf(socialCanvas()), 'rebuilt-sim-social.png'); },
    async copy() {
      try { await navigator.clipboard.write([new ClipboardItem({ 'image/png': await blobOf(ui.canvas) })]); ui.status('Copied to the clipboard.'); } catch { ui.status("This browser won't copy images; use Download."); }
    },
    reset() { localStorage.removeItem('rebuiltSim.cover'); location.reload(); },
  });
  ui.status('Press Render to make the cover. It runs on this computer; a few seconds to a minute.');
}

// ------------------------------------------------------------------ the page
function buildUI(o) {
  const css = `
  #cover{position:fixed;inset:0;z-index:1000;background:#07090d;display:flex;font-family:Inter,system-ui,sans-serif;color:#e8ecf2}
  #cover .cv-view{flex:1;display:flex;align-items:center;justify-content:center;padding:16px;min-width:0}
  #cover canvas{max-width:100%;max-height:100%;box-shadow:0 8px 40px rgba(0,0,0,.6);background:#11141b}
  #cover .cv-panel{width:330px;flex:none;overflow:auto;background:#10131a;border-left:1px solid #232835;padding:18px 18px 28px}
  #cover h1{font:700 30px "Barlow Condensed",sans-serif;margin:0 0 4px;letter-spacing:.5px}
  #cover h2{font:600 13px Inter,sans-serif;text-transform:uppercase;letter-spacing:.08em;color:#8b94a7;margin:18px 0 8px}
  #cover label{display:grid;grid-template-columns:128px 1fr;align-items:center;gap:8px;margin:6px 0;font-size:13px;color:#c3cad6}
  #cover select,#cover input[type=text]{width:100%;background:#1a1f2a;color:#e8ecf2;border:1px solid #2b3242;border-radius:6px;padding:6px 8px;font-size:13px;box-sizing:border-box}
  #cover input[type=range]{width:100%}
  #cover .cv-row{display:flex;gap:8px;flex-wrap:wrap;margin-top:8px}
  #cover button{flex:1;min-width:120px;background:#2a3142;color:#fff;border:0;border-radius:7px;padding:10px 12px;font:600 13px Inter,sans-serif;cursor:pointer}
  #cover button.primary{background:#f5c630;color:#141414}
  #cover button:disabled{opacity:.45;cursor:default}
  #cover .cv-status{font-size:13px;color:#aeb6c4;margin-top:12px;min-height:36px;line-height:1.4}
  #cover .cv-val{font-variant-numeric:tabular-nums;color:#8b94a7;font-size:12px}
  #cover a{color:#8fb4ff}
  @media (max-width:760px){#cover{flex-direction:column}#cover .cv-panel{width:auto;border-left:0;border-top:1px solid #232835}}`;
  const st = document.createElement('style');
  st.textContent = css;
  document.head.appendChild(st);
  const robotOpts = (sel) => ROBOT_ORDER.map((k) => `<option value="${k}" ${k === sel ? 'selected' : ''}>${ROBOTS[k].team} ${ROBOTS[k].robotName || ''}</option>`).join('');
  const root = document.createElement('div');
  root.id = 'cover';
  root.innerHTML = `
    <div class="cv-view"><canvas width="1920" height="1080"></canvas></div>
    <div class="cv-panel">
      <h1>Cover render</h1>
      <div style="font-size:13px;color:#8b94a7">Renders a moment of a match on this computer and exports it. <a href="./">Back to the game</a></div>
      <div class="cv-row"><button class="primary" data-act="render">Render</button></div>
      <div class="cv-status"></div>
      <div class="cv-row">
        <button data-act="png" disabled>Download PNG</button>
        <button data-act="social" disabled>GitHub preview 1280×640</button>
        <button data-act="copy" disabled>Copy</button>
      </div>
      <h2>Shot</h2>
      <label>Camera<select data-k="camera">${Object.entries(CAMERAS).map(([k, c]) => `<option value="${k}" ${k === o.camera ? 'selected' : ''}>${c.name}</option>`).join('')}</select></label>
      <label><span>Height <span class="cv-val" data-v="height"></span></span><input type="range" data-k="height" min="-0.2" max="2" step="0.05" value="${o.height}"></label>
      <label><span>Zoom (FOV) <span class="cv-val" data-v="fov"></span></span><input type="range" data-k="fov" min="30" max="80" step="1" value="${o.fov}"></label>
      <label><span>Motion blur <span class="cv-val" data-v="shutter"></span></span><input type="range" data-k="shutter" min="0" max="50" step="1" value="${o.shutter}"></label>
      <label><span>Moment <span class="cv-val" data-v="moment"></span></span><input type="range" data-k="moment" data-restage="1" min="0.5" max="3" step="0.05" value="${o.moment}"></label>
      <h2>Robots</h2>
      ${o.blue.map((r, i) => `<label>Blue ${i + 1}<select data-k="blue" data-i="${i}" data-restage="1">${robotOpts(r)}</select></label>`).join('')}
      ${o.red.map((r, i) => `<label>Red ${i + 1}<select data-k="red" data-i="${i}" data-restage="1">${robotOpts(r)}</select></label>`).join('')}
      <h2>Title</h2>
      <label>Show title<select data-k="title"><option value="1" ${o.title ? 'selected' : ''}>Yes</option><option value="" ${o.title ? '' : 'selected'}>No</option></select></label>
      <label>Small line<input type="text" data-k="kicker" value="${o.kicker}"></label>
      <label>Title<input type="text" data-k="name" value="${o.name}"></label>
      <label>Accent<input type="text" data-k="accent" value="${o.accent}"></label>
      <label>Tagline<input type="text" data-k="tagline" value="${o.tagline}"></label>
      <h2>Output</h2>
      <label>Size<select data-k="size">${Object.keys(SIZES).map((s) => `<option ${s === o.size ? 'selected' : ''}>${s}</option>`).join('')}</select></label>
      <label>Quality<select data-k="ss"><option value="1" ${o.ss == 1 ? 'selected' : ''}>Fast</option><option value="2" ${o.ss == 2 ? 'selected' : ''}>Smooth edges (2×)</option></select></label>
      <div class="cv-row"><button data-act="reset">Reset settings</button></div>
    </div>`;
  document.body.appendChild(root);
  const $ = (q) => root.querySelector(q);
  const fmt = { height: (v) => `${v > 0 ? '+' : ''}${(+v).toFixed(2)} m`, fov: (v) => `${v}°`, shutter: (v) => (+v ? `1/${Math.round(1000 / v)} s` : 'off'), moment: (v) => `${(+v).toFixed(2)} s` };
  const showVal = (k, v) => { const e = $(`[data-v="${k}"]`); if (e) e.textContent = fmt[k] ? fmt[k](v) : v; };
  for (const k of Object.keys(fmt)) showVal(k, o[k]);
  let handlers = {};
  root.addEventListener('input', (e) => {
    const el = e.target, k = el.dataset.k;
    if (!k) return;
    let v = el.type === 'range' ? +el.value : el.value;
    if (k === 'title') v = !!el.value;
    if (k === 'ss') v = +el.value;
    if (k === 'blue' || k === 'red') { const arr = [...o[k]]; arr[+el.dataset.i] = el.value; v = arr; }
    showVal(k, v);
    handlers.change?.(k, v, !!el.dataset.restage);
  });
  root.addEventListener('click', (e) => { const a = e.target.dataset?.act; if (a && !e.target.disabled) handlers[a]?.(); });
  return {
    canvas: $('canvas'),
    on(h) { handlers = h; },
    status(t) { $('.cv-status').textContent = t; },
    busy(v) { $('[data-act="render"]').disabled = v; },
    done(v) { for (const b of root.querySelectorAll('[data-act="png"],[data-act="social"],[data-act="copy"]')) b.disabled = !v; },
    set(k, v) { const el = $(`[data-k="${k}"]`); if (el) el.value = v; showVal(k, v); },
  };
}
