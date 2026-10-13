// The neural-network driver's policy, run in plain JavaScript (no libraries), so the same code
// drives in the browser and in the Node training workers. Two kinds:
//   * the MLP (arch 'mlp', the default): weights from tools/nn/train.py as JSON ({ obsMean, obsStd,
//     hidden, logStd, weights: base64 float32 }); `weights` is each hidden layer W (out x in,
//     row-major) then b, then the movement head (3 means) and the button head (4 logits);
//   * attention over the objects on the field plus memory (arch 'attn-gru', tools/nn/models.py):
//     the weights are named (json.params: [name, shape] in order). It remembers between decisions,
//     so each robot it drives keeps its own memory: newState() makes one, act(obs, { state })
//     updates it. tools/nn/policy-parity.py checks this matches PyTorch.
import { OBS_DIM, OBS_VERSION, ACT_CONT, ACT_BIN, ACT_DIM } from './obs.js';

function b64ToFloat32(s) {
  let bytes;
  if (typeof Buffer !== 'undefined') bytes = Buffer.from(s, 'base64');
  else {
    const bin = atob(s);
    bytes = new Uint8Array(bin.length);
    for (let i = 0; i < bin.length; i++) bytes[i] = bin.charCodeAt(i);
  }
  // copy: a Node Buffer's offset into its pool isn't always 4-byte aligned
  return new Float32Array(bytes.buffer.slice(bytes.byteOffset, bytes.byteOffset + bytes.byteLength));
}

const act = {
  tanh: Math.tanh,
  relu: (v) => (v > 0 ? v : 0),
  elu: (v) => (v > 0 ? v : Math.expm1(v)),
};
const sigmoid = (v) => 1 / (1 + Math.exp(-v));

// y = W x + b (W out x in, row-major), optionally through f
function dense(W, b, x, n, m, y, f) {
  for (let r = 0; r < n; r++) {
    let s = b[r];
    const off = r * m;
    for (let k = 0; k < m; k++) s += W[off + k] * x[k];
    y[r] = f ? f(s) : s;
  }
  return y;
}

// Attention over the objects (other robots, FUEL map cells, nearest FUEL) + a GRU memory. Same math
// as AttnGruActor.step in tools/nn/models.py.
class AttnGruNet {
  constructor(json, w) {
    const P = {};
    let o = 0;
    for (const [name, shape] of json.params) {
      const n = shape.reduce((a, b) => a * b, 1);
      P[name] = { d: w.subarray(o, o + n), shape };
      o += n;
    }
    if (o !== w.length) throw new Error(`neural driver weights: expected ${o} floats, got ${w.length}`);
    this.P = P;
    this.cfg = json.cfg;
    this.idx = json.idx;
    this.field = json.field;
    this.H = json.cfg.heads; this.dh = json.cfg.dh; this.G = json.cfg.gru;
    this.ctxLayers = [];
    for (let i = 0; P[`ctx.${i}.weight`]; i += 2) this.ctxLayers.push(P[`ctx.${i}.weight`], P[`ctx.${i}.bias`]);
    this.headLayers = [];
    for (let i = 0; P[`head.${i}.weight`]; i += 2) this.headLayers.push(P[`head.${i}.weight`], P[`head.${i}.bias`]);
    this.C = this.ctxLayers.length ? this.ctxLayers[this.ctxLayers.length - 2].shape[0] : this.idx.ctx.length;
    this.kinds = ['robot', 'cell', 'near'];
    const gx = json.idx.gx, gz = json.idx.gz;
    this.cellC = new Float32Array(2 * gx * gz);
    for (let i = 0; i < gx; i++) for (let j = 0; j < gz; j++) { this.cellC[2 * (i * gz + j)] = ((i + 0.5) / gx) * 2 - 1; this.cellC[2 * (i * gz + j) + 1] = ((j + 0.5) / gz) * 2 - 1; }
  }

  newState() { return new Float32Array(this.G); }

  // x: normalized observation, raw: as built; h: memory (updated in place) -> { mu, logits }
  forward(x, raw, h) {
    const { P, H, dh, G, idx } = this;
    // context
    let v = Float32Array.from(idx.ctx, (i) => x[i]);
    for (let l = 0; l < this.ctxLayers.length; l += 2) {
      const W = this.ctxLayers[l], b = this.ctxLayers[l + 1];
      v = dense(W.d, b.d, v, W.shape[0], W.shape[1], new Float32Array(W.shape[0]), Math.tanh);
    }
    const ctx = v;
    const q = dense(P['q.weight'].d, P['q.bias'].d, ctx, H * dh, this.C, new Float32Array(H * dh));
    const sc = 1 / Math.sqrt(dh);
    for (let i = 0; i < q.length; i++) q[i] *= sc;
    // the objects: [features..] per token and whether it's there
    const toks = { robot: [], cell: [], near: [] };
    const rw = P['remb.weight'], rb = P['remb.bias'];
    idx.robots.forEach((ri, k) => {
      if (!(raw[idx.robotOn[k]] > 0.5)) return;
      const f = new Float32Array(ri.length + 1);
      ri.forEach((j, n) => { f[n] = x[j]; });
      f[ri.length] = idx.mate[k];
      toks.robot.push(dense(rw.d, rb.d, f, rw.shape[0], rw.shape[1], new Float32Array(rw.shape[0]), Math.tanh));
    });
    const HL = this.field.halfL, HW = this.field.halfW;
    const sx = raw[0], sz = raw[1], c = raw[2], sn = raw[3];
    const nCell = idx.gx * idx.gz;
    for (let i = 0; i < nCell; i++) {
      const val = raw[idx.field + i];
      if (!(val > 0)) continue;
      const cx = this.cellC[2 * i], cz = this.cellC[2 * i + 1];
      const dxm = (cx - sx) * HL, dzm = (cz - sz) * HW;
      const lx = dxm * c - dzm * sn, lz = dxm * sn + dzm * c;
      toks.cell.push(Float32Array.of(cx, cz, lx / 8, lz / 8, Math.sqrt(lx * lx + lz * lz + 1e-9) / 8, val));
    }
    for (let i = 0; i < idx.nearN; i++) {
      const a = raw[idx.near + 2 * i], b = raw[idx.near + 2 * i + 1];
      if (a === 1 && b === 0) continue;
      toks.near.push(Float32Array.of(a, b, Math.sqrt(a * a + b * b + 1e-9)));
    }
    // attention scores: q.k_i = (Wk^T q).t_i + q.bk, per head
    const per = this.kinds.map((k) => {
      const wk = P[`wk_${k}`], bk = P[`bk_${k}`].d, F = wk.shape[2], ts = toks[k];
      const s = new Float32Array(H * ts.length);
      for (let hh = 0; hh < H; hh++) {
        const qk = new Float32Array(F);
        let cc = 0;
        for (let d = 0; d < dh; d++) {
          const qv = q[hh * dh + d], base = (hh * dh + d) * F;
          for (let f = 0; f < F; f++) qk[f] += qv * wk.d[base + f];
          cc += qv * bk[hh * dh + d];
        }
        ts.forEach((t, i) => { let z = cc; for (let f = 0; f < F; f++) z += qk[f] * t[f]; s[hh * ts.length + i] = z; });
      }
      return { k, F, ts, s };
    });
    const o = new Float32Array(H * dh);
    for (let hh = 0; hh < H; hh++) {
      let sNull = 0;
      for (let d = 0; d < dh; d++) sNull += q[hh * dh + d] * P.k_null.d[hh * dh + d];
      let mx = sNull;
      for (const g of per) for (let i = 0; i < g.ts.length; i++) mx = Math.max(mx, g.s[hh * g.ts.length + i]);
      let tot = Math.exp(sNull - mx);
      for (const g of per) for (let i = 0; i < g.ts.length; i++) tot += Math.exp(g.s[hh * g.ts.length + i] - mx);
      const aNull = Math.exp(sNull - mx) / tot;
      for (let d = 0; d < dh; d++) o[hh * dh + d] = aNull * P.v_null.d[hh * dh + d];
      for (const g of per) {
        const tbar = new Float32Array(g.F);
        let m = 0;
        for (let i = 0; i < g.ts.length; i++) {
          const a = Math.exp(g.s[hh * g.ts.length + i] - mx) / tot;
          m += a;
          for (let f = 0; f < g.F; f++) tbar[f] += a * g.ts[i][f];
        }
        const wv = P[`wv_${g.k}`].d, bv = P[`bv_${g.k}`].d;
        for (let d = 0; d < dh; d++) {
          let z = bv[hh * dh + d] * m;
          const base = (hh * dh + d) * g.F;
          for (let f = 0; f < g.F; f++) z += wv[base + f] * tbar[f];
          o[hh * dh + d] += z;
        }
      }
    }
    const ow = P['out.weight'];
    const att = dense(ow.d, P['out.bias'].d, o, ow.shape[0], ow.shape[1], new Float32Array(ow.shape[0]));
    // memory (a GRU cell, PyTorch's gate order r, z, n)
    const inp = new Float32Array(ctx.length + att.length);
    inp.set(ctx); inp.set(att, ctx.length);
    const wi = P['gru.weight_ih'], wh = P['gru.weight_hh'];
    const gi = dense(wi.d, P['gru.bias_ih'].d, inp, 3 * G, inp.length, new Float32Array(3 * G));
    const gh = dense(wh.d, P['gru.bias_hh'].d, h, 3 * G, G, new Float32Array(3 * G));
    for (let j = 0; j < G; j++) {
      const r = sigmoid(gi[j] + gh[j]), z = sigmoid(gi[G + j] + gh[G + j]);
      const n = Math.tanh(gi[2 * G + j] + r * gh[2 * G + j]);
      h[j] = (1 - z) * n + z * h[j];
    }
    // head
    let y = new Float32Array(G + ctx.length);
    y.set(h); y.set(ctx, G);
    for (let l = 0; l < this.headLayers.length; l += 2) {
      const W = this.headLayers[l], b = this.headLayers[l + 1];
      y = dense(W.d, b.d, y, W.shape[0], W.shape[1], new Float32Array(W.shape[0]), Math.tanh);
    }
    const mw = P['mu.weight'], lw = P['lg.weight'];
    return { mu: dense(mw.d, P['mu.bias'].d, y, mw.shape[0], mw.shape[1], new Float32Array(mw.shape[0])),
      logits: dense(lw.d, P['lg.bias'].d, y, lw.shape[0], lw.shape[1], new Float32Array(lw.shape[0])) };
  }
}

export class Policy {
  constructor(json) {
    // observation blocks are only ever appended: an older network reads the start it knows
    if (json.obsVersion < 2 || json.obsVersion > OBS_VERSION || json.obsDim > OBS_DIM) {
      throw new Error(`neural driver was trained on observation v${json.obsVersion}/${json.obsDim}, this build uses v${OBS_VERSION}/${OBS_DIM}`);
    }
    this.inDim = json.obsDim;
    this.info = json.info || {};
    this.version = json.version || 0;
    this.f = act[json.activation || 'tanh'];
    this.mean = Float32Array.from(json.obsMean);
    this.std = Float32Array.from(json.obsStd);
    this.logStd = Float32Array.from(json.logStd);
    const w = b64ToFloat32(json.weights);
    this.arch = json.arch || 'mlp';
    this.recurrent = this.arch === 'attn-gru';
    this.x = new Float32Array(this.inDim);
    if (this.recurrent) { this.net = new AttnGruNet(json, w); return; }
    if (this.arch !== 'mlp') throw new Error(`neural driver: unknown network kind '${this.arch}' (update the game)`);
    let o = 0;
    const take = (n) => { const a = w.subarray(o, o + n); o += n; return a; };
    this.layers = [];
    let inDim = this.inDim;
    for (const h of json.hidden) {
      this.layers.push({ W: take(h * inDim), b: take(h), n: h, m: inDim });
      inDim = h;
    }
    this.mu = { W: take(ACT_CONT * inDim), b: take(ACT_CONT), n: ACT_CONT, m: inDim };
    this.lg = { W: take(ACT_BIN.length * inDim), b: take(ACT_BIN.length), n: ACT_BIN.length, m: inDim };
    if (o !== w.length) throw new Error(`neural driver weights: expected ${o} floats, got ${w.length}`);
    this.bufs = [new Float32Array(this.inDim), ...json.hidden.map((h) => new Float32Array(h))];
  }

  static async load(url) {
    const res = await fetch(url, { cache: 'no-store' });
    if (!res.ok) throw new Error(`${url}: HTTP ${res.status}`);
    return new Policy(await res.json());
  }

  static _dense(L, x, y, f) {
    const { W, b, n, m } = L;
    for (let r = 0; r < n; r++) {
      let s = b[r];
      const off = r * m;
      for (let k = 0; k < m; k++) s += W[off + k] * x[k];
      y[r] = f ? f(s) : s;
    }
    return y;
  }

  // memory for one robot (null for the MLP, which has none)
  newState() { return this.recurrent ? this.net.newState() : null; }

  // forward pass: { mu: Float32Array(3), logits: Float32Array(4) }. state: the robot's memory
  // (attn-gru; updated in place; without one it decides from this moment alone)
  forward(obs, state = null) {
    if (this.recurrent) {
      const x = this.x;
      for (let k = 0; k < this.inDim; k++) {
        const v = (obs[k] - this.mean[k]) / this.std[k];
        x[k] = v > 10 ? 10 : v < -10 ? -10 : v;
      }
      return this.net.forward(x, obs, state || this.net.newState());
    }
    const x0 = this.bufs[0];
    for (let k = 0; k < this.inDim; k++) {
      const v = (obs[k] - this.mean[k]) / this.std[k];
      x0[k] = v > 10 ? 10 : v < -10 ? -10 : v;
    }
    let x = x0;
    this.layers.forEach((L, j) => { x = Policy._dense(L, x, this.bufs[j + 1], this.f); });
    return { mu: Policy._dense(this.mu, x, new Float32Array(ACT_CONT)), logits: Policy._dense(this.lg, x, new Float32Array(ACT_BIN.length)) };
  }

  // stochastic: sample everything (training). Otherwise drive with the mean movement and, with
  // sampleButtons, press each button with its learned probability (a button the network wants
  // 40% of the time would otherwise never be pressed).
  act(obs, { stochastic = false, sampleButtons = false, rng = Math.random, state = null } = {}) {
    const { mu, logits } = this.forward(obs, state);
    const a = new Float32Array(ACT_DIM);
    let logp = 0;
    for (let k = 0; k < ACT_CONT; k++) {
      if (stochastic) {
        const sd = Math.exp(this.logStd[k]);
        const u1 = Math.max(1e-12, rng()), u2 = rng();
        const g = Math.sqrt(-2 * Math.log(u1)) * Math.cos(2 * Math.PI * u2);
        a[k] = mu[k] + sd * g;
        logp += -0.5 * g * g - this.logStd[k] - 0.9189385332046727;
      } else a[k] = mu[k];
    }
    for (let k = 0; k < ACT_BIN.length; k++) {
      const l = logits[k];
      const p = 1 / (1 + Math.exp(-l));
      const on = stochastic || sampleButtons ? rng() < p : p > 0.5;
      a[ACT_CONT + k] = on ? 1 : 0;
      // log sigmoid, numerically stable
      const ls = (v) => (v >= 0 ? -Math.log1p(Math.exp(-v)) : v - Math.log1p(Math.exp(v)));
      logp += on ? ls(l) : ls(-l);
    }
    return { action: a, logp };
  }
}
