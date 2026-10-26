// Where each frame's time goes, for the Show FPS readout: physics (the Rapier steps), the game
// (robots, AI, FUEL, rules, HUD), rendering on the CPU (scene traversal, draw calls), and the
// GPU's own time for the frame when the browser can measure it (EXT_disjoint_timer_query_webgl2;
// the GPU works alongside the CPU, so its share is shown on its own bar). Averaged over half a
// second, as a share of the frame time (vsync's wait included: what's left is headroom).
export class FrameStats {
  constructor(renderer) {
    this.gl = renderer.getContext();
    this.ext = null;
    try { this.ext = this.gl.getExtension('EXT_disjoint_timer_query_webgl2'); } catch { /* none */ }
    this.pending = [];
    this.active = null;
    this._reset();
    this.last = null; // the latest averages
  }

  _reset() { this.acc = { frame: 0, physics: 0, game: 0, render: 0, gpu: 0, gpuN: 0, steps: 0, n: 0 }; this.t = 0; }

  add(cat, ms) { this.acc[cat] += ms; }
  step() { this.acc.steps++; }

  // a frame ended, frameMs after the one before
  frame(frameMs) {
    this.acc.frame += frameMs;
    this.acc.n++;
    this._pollGPU();
    this.t += frameMs;
    if (this.t >= 500 && this.acc.n) {
      const a = this.acc, n = a.n;
      this.last = {
        fps: (1000 * n) / a.frame, frame: a.frame / n, physics: a.physics / n, game: a.game / n, render: a.render / n,
        gpu: a.gpuN ? a.gpu / a.gpuN : null, steps: (1000 * a.steps) / a.frame,
      };
      this._reset();
    }
  }

  // around the frame's draw calls
  gpuBegin() {
    if (!this.ext || this.active || this.pending.length > 4) return;
    const q = this.gl.createQuery();
    this.gl.beginQuery(this.ext.TIME_ELAPSED_EXT, q);
    this.active = q;
  }
  gpuEnd() {
    if (!this.active) return;
    this.gl.endQuery(this.ext.TIME_ELAPSED_EXT);
    this.pending.push(this.active);
    this.active = null;
  }
  _pollGPU() {
    const gl = this.gl, ext = this.ext;
    while (this.pending.length) {
      const q = this.pending[0];
      if (!gl.getQueryParameter(q, gl.QUERY_RESULT_AVAILABLE)) break;
      if (!gl.getParameter(ext.GPU_DISJOINT_EXT)) { this.acc.gpu += gl.getQueryParameter(q, gl.QUERY_RESULT) / 1e6; this.acc.gpuN++; }
      gl.deleteQuery(q);
      this.pending.shift();
    }
  }

  // the readout: a line of totals, a stacked bar of the CPU's frame and one of the GPU's
  html(head) {
    const s = this.last;
    if (!s) return head;
    const pct = (ms) => Math.min(100, (100 * ms) / s.frame);
    const free = Math.max(0, s.frame - s.physics - s.game - s.render);
    const row = (cls, name, ms) => `<div class="pr"><span><i class="${cls}"></i>${name}</span><b>${ms.toFixed(1)} ms</b><b>${Math.round(pct(ms))}%</b></div>`;
    const seg = (cls, ms) => `<span class="${cls}" style="width:${pct(ms).toFixed(1)}%"></span>`;
    return `<div class="ph">${head}</div>`
      + `<div class="pbar">${seg('pp', s.physics)}${seg('pg', s.game)}${seg('pc', s.render)}</div>`
      + row('pp', `Physics <small>${Math.round(s.steps)} steps/s</small>`, s.physics)
      + row('pg', 'Game logic', s.game)
      + row('pc', 'Render (CPU)', s.render)
      + row('pf', s.gpu !== null ? 'Idle' : 'Idle / GPU wait', free)
      + (s.gpu !== null ? `<div class="pbar gpu">${seg('pu', s.gpu)}</div>${row('pu', 'GPU', s.gpu)}` : '<div class="pr muted">GPU time: not measurable in this browser</div>');
  }
}
