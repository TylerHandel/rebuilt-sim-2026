// Auto Editor: a simplified PathPlanner. Place waypoints on a top-down view of the blue half
// of the field, set what the robot does on the way to each one and when it gets there, and
// save the auto to pick it in the Auto routine menu. Works with the mouse or an Xbox controller.
import {
  FIELD_L, FIELD_W, HALF_L, HALF_W, ALLIANCE_ZONE_DEPTH, HUB, BUMP, TRENCH, TOWER, DEPOT, OUTPOST,
} from './constants.js';
import { ROBOTS, BUMPER_T } from './robotConfigs.js';
import { START_POSITIONS, START_ORDER } from './auto.js';
import {
  loadAutos, saveAutos, newAuto, newPoint, mirrorAuto, estimateTime, isCustom,
  CUSTOM_PREFIX, SHOOT_MODES, ARRIVE_ACTIONS, SPEEDS,
} from './customAutos.js';
import { clamp } from './util.js';

const $ = (id) => document.getElementById(id);
const VIEW_FX = FIELD_L / 2 + 1.3; // blue ALLIANCE WALL to just past the CENTER LINE
const PAD = 14;
const PT_HIT = 0.32; // m

const ROWS = [
  'auto', 'preload', 'start', 'wp', 'intake', 'shoot', 'speed', 'action',
  'new', 'dup', 'mirror', 'rename', 'delete', 'test', 'done',
];

export class AutoEditor {
  constructor({ settings, field, onClose, onTest }) {
    this.s = settings;
    this.field = field;
    this.onClose = onClose;
    this.onTest = onTest;
    this.canvas = $('edCanvas');
    this.ctx = this.canvas.getContext('2d');
    this.mode = 'field'; // field | panel
    this.row = 0;
    this.cursor = { fx: 6, fy: 2 };
    this.sel = -1; // selected waypoint (-1 = none)
    this.grab = null; // 'start' | point index
    this.drag = null;
    this.list = [];
    this.cur = null;
    // FUEL layout for reference (blue coords)
    const toF = (p) => ({ fx: p.x + HALF_L, fy: HALF_W - p.z });
    this.fuelPts = [...field.neutralSlots(400), ...field.depotSlots('blue')].map(toF);
    this._mouse();
    window.addEventListener('resize', () => { if (this.open) this._size(); });
  }

  // ------------------------------------------------------------------ open / close / storage
  show() {
    this.open = true;
    this.list = loadAutos();
    if (!this.list.length) { this.list.push(newAuto(this.list)); this._save(); }
    const id = isCustom(this.s.auto) ? this.s.auto.slice(CUSTOM_PREFIX.length) : null;
    this.cur = this.list.find((a) => a.id === id) || this.list[0];
    this.sel = this.cur.points.length ? 0 : -1;
    this.mode = 'field';
    this.grab = null;
    if (this.sel >= 0) this.cursor = { fx: this.cur.points[0].fx, fy: this.cur.points[0].fy };
    this._size();
    this.render();
  }

  close() {
    this._save();
    this.open = false;
    this.onClose();
  }

  _save() { saveAutos(this.list); }

  get robotCfg() { return ROBOTS[this.s.robot] || ROBOTS['2910']; }
  get halfL() { return this.robotCfg.frame.length / 2 + BUMPER_T; }
  get halfW() { return this.robotCfg.frame.width / 2 + BUMPER_T; }

  // ------------------------------------------------------------------ geometry
  _size() {
    const c = this.canvas;
    const wrap = c.parentElement;
    const w = Math.max(300, wrap.clientWidth);
    const h = Math.min(window.innerHeight * 0.72, (w - 2 * PAD) * (FIELD_W / VIEW_FX) + 2 * PAD);
    const scale = Math.min((w - 2 * PAD) / VIEW_FX, (h - 2 * PAD) / FIELD_W);
    const dpr = Math.min(2, window.devicePixelRatio || 1);
    c.style.width = w + 'px';
    c.style.height = h + 'px';
    c.width = Math.round(w * dpr);
    c.height = Math.round(h * dpr);
    this.dpr = dpr;
    this.scale = scale;
    this.ox = (w - scale * VIEW_FX) / 2;
    this.oy = (h - scale * FIELD_W) / 2;
  }

  px(fx) { return this.ox + fx * this.scale; }
  py(fy) { return this.oy + (FIELD_W - fy) * this.scale; }
  toField(x, y) { return { fx: (x - this.ox) / this.scale, fy: FIELD_W - (y - this.oy) / this.scale }; }

  clampPt(p) {
    p.fx = clamp(p.fx, this.halfL, VIEW_FX - 0.2);
    p.fy = clamp(p.fy, this.halfW, FIELD_W - this.halfW);
  }

  startFx() { return ALLIANCE_ZONE_DEPTH - 0.02 - this.halfL; }

  _hit(fx, fy) {
    const a = this.cur;
    let best = null, bd = PT_HIT;
    a.points.forEach((p, i) => {
      const d = Math.hypot(p.fx - fx, p.fy - fy);
      if (d < bd) { bd = d; best = i; }
    });
    if (best !== null) return best;
    if (Math.abs(fx - this.startFx()) < this.halfL && Math.abs(fy - a.startFy) < this.halfW) return 'start';
    return null;
  }

  _addPoint(fx, fy) {
    const a = this.cur;
    const base = this.sel >= 0 ? a.points[this.sel] : a.points[a.points.length - 1];
    const p = newPoint(fx, fy);
    if (base) { p.intake = base.intake; p.shoot = base.shoot; p.speed = base.speed; }
    this.clampPt(p);
    const at = this.sel >= 0 ? this.sel + 1 : a.points.length;
    a.points.splice(at, 0, p);
    this.sel = at;
    this._changed();
  }

  _deleteSel() {
    const a = this.cur;
    if (this.sel < 0 || !a.points.length) return;
    a.points.splice(this.sel, 1);
    this.sel = Math.min(this.sel, a.points.length - 1);
    this._changed();
  }

  _changed() {
    this._save();
    this.render();
  }

  // ------------------------------------------------------------------ mouse
  _mouse() {
    const c = this.canvas;
    const pos = (e) => {
      const r = c.getBoundingClientRect();
      return this.toField(e.clientX - r.left, e.clientY - r.top);
    };
    c.addEventListener('contextmenu', (e) => {
      e.preventDefault();
      const f = pos(e);
      const h = this._hit(f.fx, f.fy);
      if (typeof h === 'number') { this.sel = h; this._deleteSel(); }
    });
    c.addEventListener('mousedown', (e) => {
      if (e.button !== 0) return;
      const f = pos(e);
      this.mode = 'field';
      const h = this._hit(f.fx, f.fy);
      if (h === 'start') this.drag = 'start';
      else if (h !== null) { this.sel = h; this.drag = h; }
      else { this._addPoint(f.fx, f.fy); this.drag = this.sel; }
      this.cursor = f;
      this.render();
    });
    window.addEventListener('mousemove', (e) => {
      if (this.drag === null || !this.open) return;
      const f = pos(e);
      this._moveGrabbed(this.drag, f.fx, f.fy);
      this.cursor = f;
      this.render();
    });
    window.addEventListener('mouseup', () => {
      if (this.drag !== null) { this.drag = null; this._changed(); }
    });
  }

  _moveGrabbed(g, fx, fy) {
    if (g === 'start') this.cur.startFy = clamp(fy, this.halfW + 0.02, FIELD_W - this.halfW - 0.02);
    else if (typeof g === 'number' && this.cur.points[g]) {
      const p = this.cur.points[g];
      p.fx = fx; p.fy = fy;
      this.clampPt(p);
    }
  }

  // ------------------------------------------------------------------ controller / keyboard
  handleInput(inp, dt) {
    const p = inp.pressed, n = inp.nav;
    if (p.start) { this.close(); return; }
    if (p.y) { this.mode = this.mode === 'field' ? 'panel' : 'field'; this.grab = null; this.render(); return; }
    if (this.mode === 'panel') {
      if (n.up) { this.row = (this.row - 1 + ROWS.length) % ROWS.length; this.render(); }
      if (n.down) { this.row = (this.row + 1) % ROWS.length; this.render(); }
      if (n.left) this._rowChange(ROWS[this.row], -1);
      if (n.right) this._rowChange(ROWS[this.row], 1);
      if (p.a) this._rowActivate(ROWS[this.row]);
      if (p.b) { this.mode = 'field'; this.render(); }
      return;
    }
    // field mode: left stick moves the cursor (and whatever is grabbed)
    const sp = (inp.held.ls ? 1.2 : 3.6) * dt;
    const mx = inp.lx, my = -inp.ly;
    if (mx || my) {
      this.cursor.fx = clamp(this.cursor.fx + mx * sp, 0.2, VIEW_FX - 0.1);
      this.cursor.fy = clamp(this.cursor.fy + my * sp, 0.1, FIELD_W - 0.1);
      if (this.grab !== null) this._moveGrabbed(this.grab, this.cursor.fx, this.cursor.fy);
      this.render();
    }
    if (p.a) {
      if (this.grab !== null) { this.grab = null; this._changed(); }
      else {
        const h = this._hit(this.cursor.fx, this.cursor.fy);
        if (h === 'start') this.grab = 'start';
        else if (h !== null) { this.sel = h; this.grab = h; }
        else this._addPoint(this.cursor.fx, this.cursor.fy);
        this.render();
      }
    }
    if (p.x) { this.grab = null; this._deleteSel(); }
    if (p.lb || p.rb) {
      const N = this.cur.points.length;
      if (N) {
        this.sel = ((this.sel < 0 ? 0 : this.sel) + (p.rb ? 1 : -1) + N) % N;
        const q = this.cur.points[this.sel];
        this.cursor = { fx: q.fx, fy: q.fy };
        this.grab = null;
        this.render();
      }
    }
    // quick edits for the selected waypoint
    const q = this.cur.points[this.sel];
    if (q) {
      if (p.left || p.right) this._rowChange('shoot', p.left ? -1 : 1);
      if (p.up || p.down) this._rowChange('speed', p.up ? 1 : -1);
      if (p.rs) this._rowChange('intake', 1);
      if (p.back) this._rowChange('action', 1);
    }
    if (p.b) {
      if (this.grab !== null) { this.grab = null; this._changed(); }
      else this.close();
    }
  }

  _cycle(list, cur, dir) {
    const i = Math.max(0, list.findIndex((v) => v[0] === cur));
    return list[(i + dir + list.length) % list.length][0];
  }

  _rowChange(row, dir) {
    const a = this.cur;
    const q = a.points[this.sel];
    switch (row) {
      case 'auto': {
        const i = this.list.indexOf(a);
        this.cur = this.list[(i + dir + this.list.length) % this.list.length];
        this.sel = this.cur.points.length ? 0 : -1;
        break;
      }
      case 'preload': a.shootPreload = !a.shootPreload; break;
      case 'start': {
        // step through the named starting positions (drag the START box for anything in between)
        const named = START_ORDER.map((k) => START_POSITIONS[k].fy).sort((x, y) => x - y);
        const next = dir > 0 ? named.find((y) => y > a.startFy + 0.01) : [...named].reverse().find((y) => y < a.startFy - 0.01);
        a.startFy = clamp(next ?? a.startFy, this.halfW + 0.02, FIELD_W - this.halfW - 0.02);
        break;
      }
      case 'wp': {
        const N = a.points.length;
        if (N) this.sel = ((this.sel < 0 ? 0 : this.sel) + dir + N) % N;
        break;
      }
      case 'intake': if (q) q.intake = !q.intake; break;
      case 'shoot': if (q) q.shoot = this._cycle(SHOOT_MODES, q.shoot, dir); break;
      case 'speed': if (q) { const i = SPEEDS.indexOf(q.speed); q.speed = SPEEDS[clamp((i < 0 ? SPEEDS.length - 1 : i) + dir, 0, SPEEDS.length - 1)]; } break;
      case 'action': if (q) q.action = this._cycle(ARRIVE_ACTIONS, q.action, dir); break;
      default: return;
    }
    this._changed();
  }

  _rowActivate(row) {
    const a = this.cur;
    switch (row) {
      case 'new': this.cur = newAuto(this.list); this.list.push(this.cur); this.sel = this.cur.points.length - 1; break;
      case 'dup': this.cur = newAuto(this.list, a); this.list.push(this.cur); break;
      case 'mirror': mirrorAuto(a); break;
      case 'rename': {
        const name = window.prompt('Auto name', a.name);
        if (name && name.trim()) a.name = name.trim().slice(0, 40);
        break;
      }
      case 'delete': {
        if (!window.confirm(`Delete "${a.name}"?`)) return;
        this.list.splice(this.list.indexOf(a), 1);
        if (!this.list.length) this.list.push(newAuto(this.list));
        this.cur = this.list[0];
        this.sel = this.cur.points.length ? 0 : -1;
        break;
      }
      case 'test':
        this._save();
        this.open = false;
        this.onTest(CUSTOM_PREFIX + a.id);
        return;
      case 'done': this.close(); return;
      default: this._rowChange(row, 1); return;
    }
    this._changed();
  }

  // ------------------------------------------------------------------ rendering
  render() {
    if (!this.open || !this.cur) return;
    this._drawField();
    this._renderPanel();
  }

  _rect(g, fx0, fy0, fx1, fy1, fill, stroke) {
    const x = this.px(Math.min(fx0, fx1)), y = this.py(Math.max(fy0, fy1));
    const w = Math.abs(fx1 - fx0) * this.scale, h = Math.abs(fy1 - fy0) * this.scale;
    if (fill) { g.fillStyle = fill; g.fillRect(x, y, w, h); }
    if (stroke) { g.strokeStyle = stroke; g.strokeRect(x, y, w, h); }
  }

  _drawField() {
    const g = this.ctx, S = this.scale;
    const a = this.cur;
    g.setTransform(this.dpr, 0, 0, this.dpr, 0, 0);
    g.clearRect(0, 0, this.canvas.width, this.canvas.height);
    // carpet + zones
    this._rect(g, 0, 0, VIEW_FX, FIELD_W, '#2a2f38');
    this._rect(g, 0, 0, ALLIANCE_ZONE_DEPTH, FIELD_W, 'rgba(47,111,240,.18)');
    g.lineWidth = 2;
    g.strokeStyle = '#2f6ff0';
    g.beginPath(); g.moveTo(this.px(ALLIANCE_ZONE_DEPTH), this.py(0)); g.lineTo(this.px(ALLIANCE_ZONE_DEPTH), this.py(FIELD_W)); g.stroke();
    g.setLineDash([6, 5]);
    g.strokeStyle = 'rgba(255,255,255,.55)';
    g.beginPath(); g.moveTo(this.px(FIELD_L / 2), this.py(0)); g.lineTo(this.px(FIELD_L / 2), this.py(FIELD_W)); g.stroke();
    g.setLineDash([]);
    // BUMPS, TRENCHES, HUB
    const hs = HUB.size / 2;
    for (const sy of [1, -1]) {
      this._rect(g, HUB.fx - BUMP.depth / 2, HUB.fy + sy * hs, HUB.fx + BUMP.depth / 2, HUB.fy + sy * (hs + BUMP.width), 'rgba(47,111,240,.55)');
      const y0 = sy > 0 ? FIELD_W : 0;
      this._rect(g, HUB.fx - TRENCH.depth / 2, y0, HUB.fx + TRENCH.depth / 2, y0 - sy * TRENCH.clearWidth, 'rgba(255,255,255,.08)', 'rgba(47,111,240,.8)');
      this._rect(g, HUB.fx - TRENCH.depth / 2, y0 - sy * TRENCH.clearWidth, HUB.fx + TRENCH.depth / 2, y0 - sy * TRENCH.width, '#15171c');
    }
    this._rect(g, HUB.fx - hs, HUB.fy - hs, HUB.fx + hs, HUB.fy + hs, '#15171c', '#2f6ff0');
    // TOWER, DEPOT, OUTPOST
    this._rect(g, 0, TOWER.fy - TOWER.width / 2, TOWER.baseDepth, TOWER.fy + TOWER.width / 2, 'rgba(21,23,28,.9)', '#2f6ff0');
    this._rect(g, 0, DEPOT.fy - DEPOT.width / 2, DEPOT.depth, DEPOT.fy + DEPOT.width / 2, null, 'rgba(47,111,240,.9)');
    this._rect(g, 0, OUTPOST.fy - OUTPOST.structW / 2, 0.12, OUTPOST.fy + OUTPOST.structW / 2, '#2f6ff0');
    g.fillStyle = 'rgba(255,255,255,.7)';
    g.font = '600 11px Inter, sans-serif';
    g.fillText('HUB', this.px(HUB.fx) - 11, this.py(HUB.fy) + 4);
    g.fillText('TOWER', this.px(0.08), this.py(TOWER.fy) + 4);
    g.fillText('DEPOT', this.px(0.05), this.py(DEPOT.fy + DEPOT.width / 2) - 4);
    g.fillText('CENTER LINE', this.px(FIELD_L / 2) + 4, this.py(FIELD_W) + 14);
    // FUEL
    g.fillStyle = 'rgba(244,210,58,.55)';
    const r = Math.max(1.2, 0.075 * S);
    for (const f of this.fuelPts) { g.beginPath(); g.arc(this.px(f.fx), this.py(f.fy), r, 0, Math.PI * 2); g.fill(); }
    // start pose
    const sfx = this.startFx();
    this._rect(g, sfx - this.halfL, a.startFy - this.halfW, sfx + this.halfL, a.startFy + this.halfW, 'rgba(47,111,240,.45)', this.grab === 'start' || this.drag === 'start' ? '#fff' : '#9cc0ff');
    g.fillStyle = '#fff';
    g.fillText('START', this.px(sfx) - 16, this.py(a.startFy) + 4);
    // path
    let prev = { fx: sfx, fy: a.startFy };
    a.points.forEach((p) => {
      const col = p.intake && p.shoot !== 'off' ? '#ff9f1a' : p.intake ? '#37d67a' : p.shoot !== 'off' ? '#ffe066' : '#e8ecf3';
      g.strokeStyle = col;
      g.lineWidth = 3 + 2 * (p.speed - 0.3);
      if (p.shoot === 'always') g.setLineDash([8, 5]);
      g.beginPath(); g.moveTo(this.px(prev.fx), this.py(prev.fy)); g.lineTo(this.px(p.fx), this.py(p.fy)); g.stroke();
      g.setLineDash([]);
      prev = p;
    });
    // waypoints
    a.points.forEach((p, i) => {
      const x = this.px(p.fx), y = this.py(p.fy);
      const sel = i === this.sel;
      const bad = p.fx + this.halfL > FIELD_L / 2; // past the CENTER LINE (G403 risk)
      if (sel) {
        g.strokeStyle = 'rgba(255,255,255,.35)';
        g.lineWidth = 1;
        g.strokeRect(x - this.halfL * S, y - this.halfW * S, 2 * this.halfL * S, 2 * this.halfW * S);
      }
      g.beginPath();
      g.arc(x, y, sel ? 11 : 9, 0, Math.PI * 2);
      g.fillStyle = bad ? '#e03434' : sel ? '#fff' : '#1b1e25';
      g.fill();
      g.lineWidth = 2;
      g.strokeStyle = sel ? '#2f6ff0' : '#fff';
      g.stroke();
      g.fillStyle = sel ? '#111' : '#fff';
      g.font = '700 11px Inter, sans-serif';
      const label = String(i + 1);
      g.fillText(label, x - g.measureText(label).width / 2, y + 4);
      const tag = p.action === 'shoot' ? 'SHOOT' : p.action === 'stop' ? 'STOP' : p.action.startsWith('wait') ? `WAIT ${p.action.slice(4)}s` : '';
      if (tag) {
        g.font = '700 10px Inter, sans-serif';
        g.fillStyle = p.action === 'shoot' ? '#ffe066' : '#c7cdd8';
        g.fillText(tag, x + 12, y - 8);
      }
    });
    // controller cursor
    if (this.mode === 'field') {
      const x = this.px(this.cursor.fx), y = this.py(this.cursor.fy);
      g.strokeStyle = this.grab !== null ? '#37d67a' : '#fff';
      g.lineWidth = 1.5;
      g.beginPath(); g.moveTo(x - 10, y); g.lineTo(x + 10, y); g.moveTo(x, y - 10); g.lineTo(x, y + 10); g.stroke();
      g.beginPath(); g.arc(x, y, PT_HIT * S, 0, Math.PI * 2); g.stroke();
    }
  }

  _renderPanel() {
    const a = this.cur;
    const q = a.points[this.sel];
    const cfg = this.robotCfg;
    const est = estimateTime(a, cfg);
    const name = (list, v) => (list.find((x) => x[0] === v) || list[0])[1];
    const nearStart = START_ORDER.find((k) => Math.abs(START_POSITIONS[k].fy - a.startFy) < 0.05);
    const val = {
      auto: `${a.name} <span class="muted">(${this.list.indexOf(a) + 1}/${this.list.length})</span>`,
      preload: a.shootPreload ? 'Yes' : 'No',
      start: nearStart ? START_POSITIONS[nearStart].name : `${(a.startFy / 0.0254).toFixed(0)} in from right wall`,
      wp: q ? `#${this.sel + 1} of ${a.points.length}` : 'none',
      intake: q ? (q.intake ? 'On' : 'Off') : '—',
      shoot: q ? name(SHOOT_MODES, q.shoot) : '—',
      speed: q ? `${Math.round(q.speed * 100)}%` : '—',
      action: q ? name(ARRIVE_ACTIONS, q.action) : '—',
    };
    const labels = {
      auto: 'Auto', preload: 'Shoot preload first', start: 'Start position', wp: 'Waypoint',
      intake: '→ Intake on the way', shoot: '→ Shooting on the way', speed: '→ Max speed', action: 'At waypoint',
      new: 'New auto', dup: 'Duplicate', mirror: 'Mirror left ↔ right', rename: 'Rename…', delete: 'Delete auto',
      test: 'Test in a match ▶', done: 'Done',
    };
    const html = ROWS.map((k, i) => {
      const f = this.mode === 'panel' && this.row === i ? 'focus' : '';
      if (k in val) return `<div class="opt ${f}" data-row="${k}"><span class="ol">${labels[k]}</span><span class="ov"><span class="arr">◀</span>${val[k]}<span class="arr">▶</span></span></div>`;
      return `<div class="btn ${k === 'test' ? '' : 'secondary'} small ${f}" data-row="${k}">${labels[k]}</div>`;
    }).join('');
    const over = est > 20;
    $('edSide').innerHTML = `<div class="edEst ${over ? 'bad' : ''}">≈ ${est.toFixed(1)} s <span class="muted">of 20 s AUTO · ${cfg.team}</span></div>` + html;
    for (const el of document.querySelectorAll('#edSide [data-row]')) {
      el.onclick = (e) => {
        const k = el.dataset.row;
        this.row = ROWS.indexOf(k);
        if (k in val) this._rowChange(k, e.offsetX < el.clientWidth / 2 ? -1 : 1);
        else this._rowActivate(k);
      };
    }
    const focusEl = document.querySelector('#edSide .focus');
    if (focusEl) focusEl.scrollIntoView({ block: 'nearest' });
    $('edHelp').innerHTML = this.mode === 'field'
      ? '<b>Stick</b> cursor · <b>A</b> add / grab / drop · <b>X</b> delete · <b>LB/RB</b> prev/next point · <b>D-pad ◀▶</b> shooting · <b>D-pad ▲▼</b> speed · <b>RS click</b> intake · <b>View</b> at-waypoint action · <b>Y</b> settings panel · <b>B</b> done'
      + '<br><span class="muted">Mouse: click to add, drag to move, right-click to delete. Green = intake, yellow = shoot, orange = both, dashed = shoot/pass anywhere.</span>'
      : '<b>D-pad ▲▼</b> choose · <b>◀ ▶</b> change · <b>A</b> select · <b>Y / B</b> back to the field';
  }
}
