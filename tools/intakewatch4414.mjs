// 4414's intake under load (tools/mechlab4414.mjs and tools/intake-test.mjs, INTAKELOG=1): each
// step the primary roller's and the kick wheel/black roller's speed against free, their motors'
// current and whether they're at a current limit; and what loads the primary: each FUEL touching it,
// the torque it takes out of the primary, where round the roller it touches (deg: 0 ahead, 90 on
// top, 180 behind toward the bumper, 270 under it) and what else holds that FUEL (the bumper, the
// kick wheel, the black roller, the guard tubes, the box, the carpet, other FUEL). Summed over the
// window the intake is working (from the first FUEL on the primary to the last one in).
import { FUEL } from '../js/constants.js';

const R = FUEL.radius;
const KIND = [['bumper', 'bumper'], ['kick wheel', 'kick'], ['black roller', 'bar'], ['guard tube', 'guard'], ['box', 'box'], ['hopper', 'hopper']];
const kind = (name) => (KIND.find(([k]) => name.includes(k)) || [0, name])[1];
const q = (a, f) => { const b = [...a].sort((x, y) => x - y); return b.length ? b[Math.floor(f * (b.length - 1))] : NaN; };

// path: the Mechanism; t: the drives (build4414); primary: its Wheel; local(body) -> [x, y, z] in the
// robot's frame; onCarpet(body): the FUEL's resting on the carpet; count(): how many it's taken in
// (the window ends with the last one in)
export function intakeWatch({ path, t, primary, local, onCarpet, count, every = 0.25 }) {
  const steps = [], load = new Map(), at = new Map(), series = [], barLoad = new Map();
  let first = null, last = null, tt = 0, win = [], nIn = 0;
  const free = { p: t.primary.motor.freeSpeed, b: t.bar.motor.freeSpeed };
  return {
    check(dt, balls) {
      tt += dt;
      const C = path.contacts, on = C.filter((c) => c.wheel === primary);
      const n = count();
      if (on.length) first ??= tt;
      if (first !== null && n > nIn) last = tt;
      nIn = Math.max(nIn, n);
      const s = { t: tt, pf: t.primary.omega / free.p, pA: Math.abs(t.primary.motor.current), pLim: !!t.primary.motor.limited, bf: t.bar.omega / free.b, bA: Math.abs(t.bar.motor.current), bLim: !!t.bar.motor.limited, n: on.length, top: '' };
      const pc = primary.center, P = balls.map((b) => local(b));
      let worst = 0;
      for (const c of on) {
        const tq = -c.torque * Math.sign(t.primary.omega || 1); // (what it takes out of the primary)
        const i = balls.indexOf(c.ball), p = i >= 0 ? P[i] : local(c.ball);
        const ang = ((Math.atan2(p[1] - pc[1], p[0] - pc[0]) * 180) / Math.PI + 360) % 360;
        const who = new Set(C.filter((d) => d.ball === c.ball && d.wheel !== primary).map((d) => kind(d.wheel.name)));
        if (onCarpet(c.ball)) who.add('carpet');
        if (P.some((o, j) => j !== i && Math.hypot(o[0] - p[0], o[1] - p[1], o[2] - p[2]) < 2 * R - 0.002)) who.add('FUEL');
        const key = [...who].sort().join('+') || 'nothing else';
        load.set(key, (load.get(key) || 0) + tq * dt);
        const bin = `${Math.floor(ang / 45) * 45}-${Math.floor(ang / 45) * 45 + 45}`;
        at.set(bin, (at.get(bin) || 0) + tq * dt);
        if (tq > worst) { worst = tq; s.top = `${key} at ${ang.toFixed(0)} deg ${tq.toFixed(2)} N·m (${c.force.toFixed(0)} N, dent ${(1000 * c.depth).toFixed(0)} mm)`; }
      }
      // (and the kick wheel's and the black roller's: by which of them and what else holds the FUEL)
      for (const c of C) {
        if (c.wheel.shaft !== t.bar || !c.wheel.radius) continue;
        const tq = -c.torque * Math.sign(t.bar.omega || 1), i = balls.indexOf(c.ball), p = i >= 0 ? P[i] : local(c.ball);
        const who = new Set(C.filter((d) => d.ball === c.ball && d.wheel !== c.wheel).map((d) => (d.wheel === primary ? 'primary' : kind(d.wheel.name))));
        if (onCarpet(c.ball)) who.add('carpet');
        if (P.some((o, j) => j !== i && Math.hypot(o[0] - p[0], o[1] - p[1], o[2] - p[2]) < 2 * R - 0.002)) who.add('FUEL');
        const key = `${kind(c.wheel.name)}: ${[...who].sort().join('+') || 'nothing else'}`;
        barLoad.set(key, (barLoad.get(key) || 0) + tq * dt);
      }
      s.load = on.reduce((a, c) => a - c.torque * Math.sign(t.primary.omega || 1), 0);
      steps.push(s); win.push(s);
      if (win.length * dt >= every - 1e-9) {
        const m = (k) => win.reduce((a, x) => a + x[k], 0) / win.length;
        const top = win.reduce((a, x) => (x.load > a.load ? x : a), win[0]);
        series.push(`${tt.toFixed(2)} s: primary ${(100 * m('pf')).toFixed(0)}% (min ${(100 * Math.min(...win.map((x) => x.pf))).toFixed(0)}) ${m('pA').toFixed(0)} A, limited ${(100 * win.filter((x) => x.pLim).length / win.length).toFixed(0)}%, ${m('n').toFixed(1)} FUEL on it, load ${m('load').toFixed(2)} N·m; bar ${(100 * m('bf')).toFixed(0)}% ${m('bA').toFixed(0)} A lim ${(100 * win.filter((x) => x.bLim).length / win.length).toFixed(0)}%${top.top ? `; worst: ${top.top}` : ''}`);
        win = [];
      }
    },
    report() {
      const W = steps.filter((s) => first !== null && s.t >= first && s.t <= last);
      if (!W.length) return { text: 'no FUEL touched the primary' };
      const pf = W.map((s) => s.pf), bf = W.map((s) => s.bf), dt = W.length > 1 ? (W[W.length - 1].t - W[0].t) / (W.length - 1) : 0;
      const tot = [...load.values()].reduce((a, v) => a + Math.max(0, v), 0) || 1;
      const share = (m, T = tot) => [...m.entries()].filter(([, v]) => v > 0.01 * T).sort((a, b) => b[1] - a[1]).map(([k, v]) => `${k} ${(100 * v / T).toFixed(0)}%`).join(', ');
      const o = {
        pMin: Math.min(...pf), pMed: q(pf, 0.5), p10: q(pf, 0.1), pLim: W.filter((s) => s.pLim).length / W.length, pLimS: W.filter((s) => s.pLim).length * dt, pA: q(W.map((s) => s.pA), 0.5), pA90: q(W.map((s) => s.pA), 0.9),
        bMin: Math.min(...bf), bMed: q(bf, 0.5), bLim: W.filter((s) => s.bLim).length / W.length, span: W.length * dt,
      };
      o.text = `over ${o.span.toFixed(2)} s from the first FUEL on the primary to the last one in: primary min ${(100 * o.pMin).toFixed(0)}%, 10th pct ${(100 * o.p10).toFixed(0)}%, median ${(100 * o.pMed).toFixed(0)}% of free; its X60s median ${o.pA.toFixed(0)} A, 90th ${o.pA90.toFixed(0)} A (both), at a current limit ${(100 * o.pLim).toFixed(0)}% (${o.pLimS.toFixed(2)} s); ` +
        `kick wheel/black roller min ${(100 * o.bMin).toFixed(0)}%, median ${(100 * o.bMed).toFixed(0)}%, limited ${(100 * o.bLim).toFixed(0)}%\n     the primary's load (torque x time) by what else holds the FUEL: ${share(load)}\n     by where round the primary: ${share(at)}\n     the kick wheel's and black roller's load: ${share(barLoad, [...barLoad.values()].reduce((a, v) => a + Math.max(0, v), 0) || 1)}`;
      o.series = series;
      return o;
    },
  };
}
