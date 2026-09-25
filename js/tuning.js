// AI Tuning screen: every brain value on a slider next to the hand-tuned value, the shipped
// trained value and what your driving measured; learning settings; export/import.
import { BRAIN_SPEC, DEFAULT_BRAIN } from './opponent.js';
import { TRAINED_BRAIN } from './trainedBrain.js';
import { KEYS, EXPLORE_LEVELS, IMITATE_LEVELS } from './learning.js';

const $ = (id) => document.getElementById(id);

const BUTTONS = [
  ['demo', 'Blend toward my driving (50%)'],
  ['shipped', 'Reset to shipped Trained AI'],
  ['default', 'Reset to hand-tuned (Champs)'],
  ['export', 'Export trainedBrain.js (for the main version)'],
  ['json', 'Copy JSON to clipboard'],
  ['import', 'Import JSON / trainedBrain.js…'],
  ['clearDemo', 'Clear my driving data'],
  ['clearStats', 'Reset training record'],
  ['done', 'Done'],
];
const ROWS = [...KEYS, 'explore', 'imitate', ...BUTTONS.map((b) => b[0])];

export function fmtValue(k, v) {
  if (v === null || v === undefined) return '—';
  const u = BRAIN_SPEC[k].unit;
  return u === '%' ? `${Math.round(v * 100)}%` : `${(+v).toFixed(2)}${u ? ' ' + u : ''}`;
}

export class TuningScreen {
  constructor({ learner, onClose }) {
    this.L = learner;
    this.onClose = onClose;
    this.row = 0;
    this.msg = '';
    this.file = document.createElement('input');
    this.file.type = 'file';
    this.file.accept = '.js,.json,text/plain,application/json,text/javascript';
    this.file.onchange = async () => {
      const f = this.file.files[0];
      if (!f) return;
      this.msg = this.L.importText(await f.text()) ? `Imported ${f.name}.` : `${f.name} doesn't contain brain values.`;
      this.file.value = '';
      this.render();
    };
  }

  show() { this.open = true; this.msg = ''; this.render(); }
  close() { this.open = false; this.onClose(); }

  handleInput(inp) {
    const p = inp.pressed, n = inp.nav;
    if (p.b || p.start) { this.close(); return; }
    if (n.up) { this.row = (this.row - 1 + ROWS.length) % ROWS.length; this.render(); }
    if (n.down) { this.row = (this.row + 1) % ROWS.length; this.render(); }
    const fine = inp.held.ls ? 0.25 : 1;
    if (n.left) this._change(ROWS[this.row], -fine);
    if (n.right) this._change(ROWS[this.row], fine);
    if (p.lb || p.rb) this._change(ROWS[this.row], p.lb ? -4 : 4);
    if (p.a) this._activate(ROWS[this.row]);
  }

  _change(row, dir) {
    if (BRAIN_SPEC[row]) {
      const sp = BRAIN_SPEC[row];
      this.L.set(row, this.L.s.brain[row] + dir * 0.025 * (sp.max - sp.min));
    } else if (row === 'explore' || row === 'imitate') {
      const list = row === 'explore' ? EXPLORE_LEVELS : IMITATE_LEVELS;
      const i = list.findIndex((x) => x[0] === this.L.s[row]);
      this.L.s[row] = list[Math.max(0, Math.min(list.length - 1, (i < 0 ? 0 : i) + Math.sign(dir)))][0];
      this.L.save();
    } else return;
    this.render();
  }

  async _activate(row) {
    const L = this.L;
    switch (row) {
      case 'demo': {
        const c = L.applyDemo(0.5);
        this.msg = c.length ? `Moved ${c.length} values halfway toward your driving.` : 'Not enough of your driving recorded yet — play a few matches first.';
        break;
      }
      case 'shipped': L.setBrain(TRAINED_BRAIN); this.msg = 'Reset to the shipped Trained AI.'; break;
      case 'default': L.setBrain(DEFAULT_BRAIN); this.msg = 'Reset to the hand-tuned Champs brain.'; break;
      case 'export': {
        download('trainedBrain.js', L.fileText(), 'text/javascript');
        download('rebuilt-ai-brain.json', L.jsonText(), 'application/json');
        this.msg = 'Downloaded trainedBrain.js — copy it over js/trainedBrain.js to make it the shipped Trained AI (and a .json copy for re-importing or tools/train.mjs --from).';
        break;
      }
      case 'json':
        try { await navigator.clipboard.writeText(L.jsonText()); this.msg = 'Brain JSON copied to the clipboard.'; } catch { this.msg = 'Clipboard not available — use Export instead.'; }
        break;
      case 'import': {
        const text = window.prompt('Paste brain JSON or the contents of a trainedBrain.js file.\n(Leave empty and press OK to choose a file instead.)', '');
        if (text === null) return;
        if (!text.trim()) { this.file.click(); return; }
        this.msg = L.importText(text) ? 'Imported.' : 'That text has no brain values in it.';
        break;
      }
      case 'clearDemo': L.clearDemo(); this.msg = 'Your recorded driving was cleared.'; break;
      case 'clearStats': L.resetStats(); this.msg = 'Training record reset (the brain is unchanged).'; break;
      case 'done': this.close(); return;
      default: this._change(row, 1); return;
    }
    this.render();
  }

  render() {
    if (!this.open) return;
    const L = this.L, S = L.s;
    const rec = S.record.score, d = S.record.defense;
    let html = `<div class="tuStats">Training matches: <b>${S.matches}</b> · vs you (Score role): <b>${rec.w}-${rec.l}${rec.t ? '-' + rec.t : ''}</b> (your W-L) · defense drills: <b>${d.n}</b>${d.n ? `, AI averaged <b>${L.defenseAverage()}</b> pts (best hold ${d.best})` : ''}</div>`;
    html += `<div class="tuHead"><span></span><span>Your trained AI</span><span class="muted">Hand-tuned · Shipped · <span class="you">You drove</span></span></div>`;
    let group = '';
    ROWS.forEach((k, i) => {
      const f = this.row === i ? 'focus' : '';
      const sp = BRAIN_SPEC[k];
      if (sp) {
        if (sp.group !== group) { group = sp.group; html += `<div class="tuGroup">${group}</div>`; }
        const v = S.brain[k];
        const pct = (100 * (v - sp.min)) / (sp.max - sp.min);
        const mark = (x, cls) => (x === null || x === undefined ? '' : `<i class="${cls}" style="left:${(100 * (x - sp.min)) / (sp.max - sp.min)}%"></i>`);
        const e = L.demoEstimate(k);
        const you = e ? (e.value === null ? `${e.n} sample${e.n > 1 ? 's' : ''}…` : `${fmtValue(k, e.value)} (${e.n})`) : '—';
        html += `<div class="tuRow ${f}" data-row="${k}" title="${sp.desc}">
          <span class="tuLabel">${sp.label}</span>
          <span class="tuSlider"><span class="tuFill" style="width:${pct}%"></span>${mark(DEFAULT_BRAIN[k], 'mDef')}${mark(TRAINED_BRAIN[k], 'mShip')}${e && e.value !== null ? mark(e.value, 'mYou') : ''}<b>${fmtValue(k, v)}</b></span>
          <span class="tuRef muted">${fmtValue(k, DEFAULT_BRAIN[k])} · ${fmtValue(k, TRAINED_BRAIN[k])} · <span class="you">${you}</span></span></div>`;
      } else if (k === 'explore' || k === 'imitate') {
        if (k === 'explore') html += '<div class="tuGroup">Learning</div>';
        const list = k === 'explore' ? EXPLORE_LEVELS : IMITATE_LEVELS;
        const cur = (list.find((x) => x[0] === S[k]) || list[0])[1];
        const label = k === 'explore' ? 'Exploration (how different each Training match is)' : 'Copy my style when I do better';
        html += `<div class="opt ${f}" data-row="${k}"><span class="ol">${label}</span><span class="ov"><span class="arr">◀</span>${cur}<span class="arr">▶</span></span></div>`;
      } else {
        if (k === BUTTONS[0][0]) html += `<div class="tuMsg">${this.msg}</div><div class="tuBtns">`;
        const label = BUTTONS.find((b) => b[0] === k)[1];
        html += `<div class="btn ${k === 'done' ? '' : 'secondary'} small ${f}" data-row="${k}">${label}</div>`;
        if (k === 'done') html += '</div>';
      }
    });
    $('tuningBody').innerHTML = html;
    for (const el of document.querySelectorAll('#tuningBody [data-row]')) {
      const k = el.dataset.row;
      el.onclick = (e) => {
        this.row = ROWS.indexOf(k);
        const sp = BRAIN_SPEC[k];
        const bar = el.querySelector('.tuSlider');
        if (sp && bar && bar.contains(e.target)) {
          const r = bar.getBoundingClientRect();
          this.L.set(k, sp.min + Math.min(1, Math.max(0, (e.clientX - r.left) / r.width)) * (sp.max - sp.min));
          this.render();
        } else if (k === 'explore' || k === 'imitate') this._change(k, e.offsetX < el.clientWidth / 2 ? -1 : 1);
        else if (!sp) this._activate(k);
        else this.render();
      };
    }
    const fe = document.querySelector('#tuningBody .focus');
    if (fe) fe.scrollIntoView({ block: 'nearest' });
  }
}

function download(name, text, type) {
  const a = document.createElement('a');
  a.href = URL.createObjectURL(new Blob([text], { type }));
  a.download = name;
  document.body.appendChild(a);
  a.click();
  setTimeout(() => { URL.revokeObjectURL(a.href); a.remove(); }, 1000);
}
