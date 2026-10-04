// Records your driving for the neural network: what it would have seen (js/nn/obs.js) and what
// you did, 10 times a second, for the whole match. Matches are kept in the browser (IndexedDB)
// until you download them from the AI TUNING screen for `tools/nn/train.py --bc <folder>`.
import { buildObs, cmdToAction, OBS_DIM, ACT_DIM, OBS_VERSION, DECISION_DT } from './obs.js';

const DB = 'rebuiltSim.nn', STORE = 'demos';

function db() {
  return new Promise((res, rej) => {
    const r = indexedDB.open(DB, 1);
    r.onupgradeneeded = () => r.result.createObjectStore(STORE, { keyPath: 'id', autoIncrement: true });
    r.onsuccess = () => res(r.result);
    r.onerror = () => rej(r.error);
  });
}

async function tx(mode, fn) {
  const d = await db();
  return new Promise((res, rej) => {
    const t = d.transaction(STORE, mode);
    const out = fn(t.objectStore(STORE));
    t.oncomplete = () => { d.close(); res(out && 'result' in out ? out.result : undefined); };
    t.onerror = () => { d.close(); rej(t.error); };
  });
}

export const NNDemos = {
  async all() { try { return (await tx('readonly', (s) => s.getAll())) || []; } catch { return []; } },
  async add(rec) { try { await tx('readwrite', (s) => s.add(rec)); return true; } catch { return false; } },
  async clear() { try { await tx('readwrite', (s) => s.clear()); } catch { /* storage unavailable */ } },
  async summary() {
    const all = await this.all();
    const n = all.reduce((s, r) => s + r.n, 0);
    return { matches: all.length, samples: n, minutes: (n * DECISION_DT) / 60 };
  },
  // one JSON file for tools/nn/train.py --bc
  async exportText() {
    const all = await this.all();
    const n = all.reduce((s, r) => s + r.n, 0);
    const obs = new Float32Array(n * OBS_DIM), act = new Float32Array(n * ACT_DIM);
    let o = 0;
    for (const r of all) {
      obs.set(new Float32Array(r.obs), o * OBS_DIM);
      act.set(new Float32Array(r.act), o * ACT_DIM);
      o += r.n;
    }
    const b64 = (a) => {
      const u = new Uint8Array(a.buffer);
      let s = '';
      for (let i = 0; i < u.length; i += 0x8000) s += String.fromCharCode.apply(null, u.subarray(i, i + 0x8000));
      return btoa(s);
    };
    return JSON.stringify({
      format: 'rebuilt-nn-demo', obsVersion: OBS_VERSION, obsDim: OBS_DIM, actDim: ACT_DIM,
      matches: all.map((r) => ({ date: r.date, robot: r.robot, n: r.n, score: r.score })),
      obs: b64(obs), act: b64(act),
    });
  },
};

export class NNRecorder {
  constructor(game) {
    this.g = game;
    this.t = DECISION_DT;
    this.obs = [];
    this.act = [];
    this.buf = new Float32Array(OBS_DIM);
  }

  step(dt) {
    const g = this.g, r = g.robot;
    if (!g.match.robotEnabled) return;
    this.t += dt;
    if (this.t < DECISION_DT - 1e-9) return;
    this.t = 0;
    this.obs.push(buildObs(r, g.opp ? g.opp.robot : null, g.match, g.fuel, new Float32Array(OBS_DIM)));
    this.act.push(cmdToAction(r, r.cmd));
  }

  // save the match (only full matches: aborted ones would teach it to stop early)
  async finish() {
    const n = this.obs.length;
    if (n < 1200) return 0;
    const obs = new Float32Array(n * OBS_DIM), act = new Float32Array(n * ACT_DIM);
    this.obs.forEach((o, i) => obs.set(o, i * OBS_DIM));
    this.act.forEach((a, i) => act.set(a, i * ACT_DIM));
    const g = this.g, m = g.match;
    await NNDemos.add({ date: new Date().toISOString(), robot: g.robot.cfg.key, n, obs: obs.buffer, act: act.buffer, score: [m.total(g.robot.alliance), m.total(g.robot.alliance === 'blue' ? 'red' : 'blue')] });
    return n;
  }
}
