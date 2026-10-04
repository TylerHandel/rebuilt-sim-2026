// The neural-network driver's policy: a small MLP run in plain JavaScript (no libraries), so the
// same code drives in the browser and in the Node training workers.
//
// Weights come from tools/nn/train.py as JSON ({ obsMean, obsStd, hidden, logStd, weights:
// base64 float32 }). Layout of `weights`: each hidden layer W (out x in, row-major) then b, then
// the movement head (3 means) and the button head (4 logits).
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

  // forward pass: { mu: Float32Array(3), logits: Float32Array(4) }
  forward(obs) {
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
  act(obs, { stochastic = false, sampleButtons = false, rng = Math.random } = {}) {
    const { mu, logits } = this.forward(obs);
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
