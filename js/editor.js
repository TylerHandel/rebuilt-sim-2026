// Auto Editor: a simplified PathPlanner. Place waypoints on a top-down view of the blue half
// of the field, set what the robot does on the way to each one and when it gets there, and
// save the auto to pick it in the Auto routine menu.
//
// Two areas, and whichever you touched last is active (it's outlined):
//   field     - the left stick / WASD moves the cursor, A adds / picks up / drops (or the mouse:
//               click to add, drag to move, right-click to delete)
//   inspector - the D-pad / arrow keys pick a setting on the right and change it (or click it)
// A bar along the bottom always shows the controls for what's active.
import {
  FIELD_L, FIELD_W, HALF_L, HALF_W, ALLIANCE_ZONE_DEPTH, HUB, BUMP, TRENCH, TOWER, DEPOT, OUTPOST,
} from './constants.js';
import { ROBOTS, ROBOT_ORDER, BUMPER_T } from './robotConfigs.js';
import { START_POSITIONS, START_ORDER } from './auto.js';
import {
  loadAutos, saveAutos, newAuto, newPoint, mirrorAuto, estimateTime, isCustom,
  CUSTOM_PREFIX, SHOOT_MODES, ARRIVE_ACTIONS, SPEEDS,
} from './customAutos.js';
import { saveSettings } from './ui.js';
import { clamp } from './util.js';

const $ = (id) => document.getElementById(id);
const VIEW_FX = FIELD_L / 2 + 1.3; // blue ALLIANCE WALL to just past the CENTER LINE
const PAD = 14;
const PT_HIT = 0.32; // m
const AUTO_S = 20;

const segColor = (p) => (p.intake && p.shoot !== 'off' ? '#ff9f1a' : p.intake ? '#37d67a' : p.shoot !== 'off' ? '#ffe066' : '#e8ecf3');
const nameOf = (list, v) => (list.find((x) => x[0] === v) || list[0])[1];
const chip = (k, t) => `<span class="kchip"><kbd>${k}</kbd>${t}</span>`;

export class AutoEditor {
  constructor({ settings, field, onClose, onTest }) {
    this.s = settings;
    this.field = field;
    this.onClose = onClose;
    this.onTest = onTest;
    this.canvas = $('edCanvas');
    this.ctx = this.canvas.getContext('2d');
    this.mode = 'field'; // field | panel: which area the controller / keyboard is working in
    this.rowId = 'auto'; // focused inspector row
    this.btn = 0; // focused button within a row of buttons
    this.cursor = { fx: 6, fy: 2 };
    this.hover = null; // mouse position over the field
    this.sel = -1; // selected waypoint (-1 = none)
    this.grab = null; // controller: 'start' | point index being carried
    this.drag = null; // mouse: same
    this.list = [];
    this.cur = null;
    this.source = 'keyboard';
    // FUEL layout for reference (blue coords)
    const toF = (p) => ({ fx: p.x + HALF_L, fy: HALF_W - p.z });
    this.fuelPts = [...field.neutralSlots(400), ...field.depotSlots('blue')].map(toF);
    this._mouse();
    window.addEventListener('resize', () => { if (this.open) { this._size(); this.render(); } });
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
    requestAnimationFrame(() => { this._size(); this.render(); });
    this._size();
    this.render();
  }

  close() {
    this._save();
    this.open = false;
    this.onClose();
  }

  test() {
    this._save();
    this.open = false;
    this.onTest(CUSTOM_PREFIX + this.cur.id);
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
    const hAvail = Math.max(240, wrap.clientHeight - ($('edLegend') ? $('edLegend').offsetHeight + 6 : 0));
    const scale = Math.min((w - 2 * PAD) / VIEW_FX, (hAvail - 2 * PAD) / FIELD_W);
    const h = scale * FIELD_W + 2 * PAD;
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

  _select(i) {
    const N = this.cur.points.length;
    if (!N) return;
    this.sel = (i + N) % N;
    const q = this.cur.points[this.sel];
    this.cursor = { fx: q.fx, fy: q.fy };
    this.grab = null;
    this.render();
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
      this.source = 'mouse';
      const h = this._hit(f.fx, f.fy);
      if (h === 'start') this.drag = 'start';
      else if (h !== null) { this.sel = h; this.drag = h; }
      else { this._addPoint(f.fx, f.fy); this.drag = this.sel; }
      this.cursor = f;
      this.render();
    });
    c.addEventListener('mousemove', (e) => {
      if (!this.open) return;
      this.hover = pos(e);
      this.source = 'mouse';
      if (this.drag === null) this.render();
    });
    c.addEventListener('mouseleave', () => { this.hover = null; if (this.open) this.render(); });
    window.addEventListener('mousemove', (e) => {
      if (this.drag === null || !this.open) return;
      const f = pos(e);
      this._moveGrabbed(this.drag, f.fx, f.fy);
      this.cursor = f;
      this.hover = f;
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

  // ------------------------------------------------------------------ inspector (right side)
  // Rows: { id, kind: 'opt' | 'btns' | 'pt' | 'sec' | 'sub' | 'note', ... }
  _rows() {
    const a = this.cur, q = a.points[this.sel], cfg = this.robotCfg;
    const nearStart = START_ORDER.find((k) => Math.abs(START_POSITIONS[k].fy - a.startFy) < 0.05);
    const rows = [
      { kind: 'sec', label: 'Auto' },
      { id: 'auto', kind: 'opt', label: 'Editing', value: `${a.name} <span class="muted">${this.list.indexOf(a) + 1}/${this.list.length}</span>`, desc: 'Which of your autos you are editing. They save as you go and show up (marked ✎) in every Auto routine menu.' },
      { id: 'robot', kind: 'opt', label: 'Robot', value: `${cfg.team} ${cfg.archetype}`, desc: 'The robot the time estimate and its size on the field are for (it is also your robot in Practice and 1 v 1).' },
      { id: 'file', kind: 'btns', btns: [['new', 'New'], ['dup', 'Copy'], ['mirror', 'Mirror'], ['rename', 'Rename'], ['delete', 'Delete']], desc: 'New starts a fresh auto, Copy duplicates this one, Mirror flips it to the other side of the field.' },
      { kind: 'sec', label: 'Start' },
      { id: 'start', kind: 'opt', label: 'Starting spot', value: nearStart ? START_POSITIONS[nearStart].name : `${(a.startFy / 0.0254).toFixed(0)} in from the right`, desc: 'Steps through the named spots; drag the START box on the field for anything in between.' },
      { id: 'preload', kind: 'opt', label: 'Shoot the preload first', value: a.shootPreload ? 'Yes' : 'No', desc: 'Spin up and shoot the preloaded FUEL from the start line before driving.' },
      { kind: 'sec', label: q ? `Waypoint ${this.sel + 1} <span class="muted">of ${a.points.length}</span>` : 'Waypoint' },
    ];
    if (q) {
      rows.push(
        { kind: 'sub', label: 'On the way here' },
        { id: 'intake', kind: 'opt', label: 'Intake', value: q.intake ? 'On' : 'Off', desc: 'Run the intake while driving to this waypoint.' },
        { id: 'shoot', kind: 'opt', label: 'Shoot', value: nameOf(SHOOT_MODES, q.shoot), desc: '"Once in our zone" shoots on the move after it gets back in the ALLIANCE ZONE; "Anywhere" also passes from outside it.' },
        { id: 'speed', kind: 'opt', label: 'Speed', value: `${Math.round(q.speed * 100)}%`, desc: 'Top speed on the way here (slower collects more of a FUEL line).' },
        { kind: 'sub', label: 'When it gets here' },
        { id: 'action', kind: 'opt', label: 'Then', value: nameOf(ARRIVE_ACTIONS, q.action), desc: 'Keep driving to the next waypoint, stop, shoot until empty, or wait.' },
        { id: 'wp', kind: 'btns', btns: [['prev', '◀ Previous'], ['next', 'Next ▶'], ['del', 'Delete']], desc: 'Step through the waypoints, or delete this one.' },
      );
    } else rows.push({ kind: 'note', label: 'No waypoints yet. Click the field (or move the cursor there and press A) to add the first one.' });
    rows.push({ kind: 'sec', label: 'Path' });
    a.points.forEach((p, i) => rows.push({
      id: 'pt' + i, kind: 'pt', i,
      value: `<span class="pdot" style="background:${segColor(p)}"></span><b>${i + 1}</b> ${p.intake ? 'Intake' : 'Drive'}${p.shoot !== 'off' ? ' + shoot' : ''} · ${Math.round(p.speed * 100)}%${p.action !== 'none' ? ' · then ' + nameOf(ARRIVE_ACTIONS, p.action).toLowerCase() : ''}${p.fx + this.halfL > FIELD_L / 2 ? ' <span class="bad">· past the CENTER LINE</span>' : ''}`,
      desc: 'Select this waypoint.',
    }));
    return rows;
  }

  _focusRows(rows = this._rows()) { return rows.filter((r) => r.id); }

  _change(id, dir) {
    const a = this.cur;
    const q = a.points[this.sel];
    switch (id) {
      case 'auto': {
        const i = this.list.indexOf(a);
        this.cur = this.list[(i + dir + this.list.length) % this.list.length];
        this.sel = this.cur.points.length ? 0 : -1;
        break;
      }
      case 'robot': {
        const i = ROBOT_ORDER.indexOf(this.s.robot);
        this.s.robot = ROBOT_ORDER[(i + dir + ROBOT_ORDER.length) % ROBOT_ORDER.length];
        saveSettings(this.s);
        break;
      }
      case 'preload': a.shootPreload = !a.shootPreload; break;
      case 'start': {
        const named = START_ORDER.map((k) => START_POSITIONS[k].fy).sort((x, y) => x - y);
        const next = dir > 0 ? named.find((y) => y > a.startFy + 0.01) : [...named].reverse().find((y) => y < a.startFy - 0.01);
        a.startFy = clamp(next ?? a.startFy, this.halfW + 0.02, FIELD_W - this.halfW - 0.02);
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

  _cycle(list, cur, dir) {
    const i = Math.max(0, list.findIndex((v) => v[0] === cur));
    return list[(i + dir + list.length) % list.length][0];
  }

  _button(b) {
    const a = this.cur;
    switch (b) {
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
      case 'prev': this._select(this.sel - 1); return;
      case 'next': this._select(this.sel + 1); return;
      case 'del': this._deleteSel(); return;
      case 'test': this.test(); return;
      case 'done': this.close(); return;
      default: return;
    }
    this._changed();
  }

  // A on the focused inspector row
  _activateRow(r) {
    if (!r) return;
    if (r.kind === 'opt') this._change(r.id, 1);
    else if (r.kind === 'btns') this._button(r.btns[Math.min(this.btn, r.btns.length - 1)][0]);
    else if (r.kind === 'pt') this._select(r.i);
  }

  // ------------------------------------------------------------------ controller / keyboard
  handleInput(inp, dt) {
    const p = inp.pressed, d = inp.dpad || {};
    // controls shown for what you're using: the controller / keyboard once you use it, the
    // mouse after a click or hover (set by the mouse handlers)
    const used = inp.lx || inp.ly || Object.values(d).some(Boolean) || Object.values(p).some(Boolean);
    if (used && inp.source !== this.source) { this.source = inp.source === 'gamepad' ? 'gamepad' : 'keyboard'; this.render(); }
    if (p.start) { this.close(); return; }
    if (p.y) { this.test(); return; }
    // LB / RB: previous / next waypoint, X: delete it (from either area)
    if (p.lb || p.rb) this._select((this.sel < 0 ? 0 : this.sel) + (p.rb ? 1 : -1));
    if (p.x) { this.grab = null; this._deleteSel(); }
    const stick = inp.lx || inp.ly;
    const dpad = d.up || d.down || d.left || d.right;
    // the D-pad works the inspector, the stick the field: whichever you use becomes active
    if (dpad && this.mode !== 'panel') { this.mode = 'panel'; this.grab = null; this.render(); return; }
    if (stick && this.mode !== 'field') { this.mode = 'field'; this.render(); }
    if (this.mode === 'panel') {
      const rows = this._focusRows();
      let i = Math.max(0, rows.findIndex((r) => r.id === this.rowId));
      const r = rows[i];
      if (d.up || d.down) {
        i = clamp(i + (d.down ? 1 : -1), 0, rows.length - 1);
        this.rowId = rows[i].id;
        this.btn = 0;
        if (rows[i].kind === 'pt') this._select(rows[i].i);
        this.render();
      }
      if ((d.left || d.right) && r) {
        const dir = d.left ? -1 : 1;
        if (r.kind === 'opt') this._change(r.id, dir);
        else if (r.kind === 'btns') { this.btn = clamp(this.btn + dir, 0, r.btns.length - 1); this.render(); }
      }
      if (p.a) this._activateRow(r);
      if (p.b) { this.mode = 'field'; this.render(); }
      return;
    }
    // field: the left stick (WASD) moves the cursor, and whatever it carries
    const sp = (inp.held.ls ? 1.2 : 3.6) * dt;
    if (stick) {
      this.cursor.fx = clamp(this.cursor.fx + inp.lx * sp, 0.2, VIEW_FX - 0.1);
      this.cursor.fy = clamp(this.cursor.fy - inp.ly * sp, 0.1, FIELD_W - 0.1);
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
    if (p.b) {
      if (this.grab !== null) { this.grab = null; this._changed(); }
      else this.close();
    }
  }

  // what A (or a click) does at the cursor right now
  _cursorAction(at = this.cursor) {
    if (this.grab !== null) return this.grab === 'start' ? 'Drop START here' : `Drop waypoint ${this.grab + 1}`;
    const h = this._hit(at.fx, at.fy);
    if (h === 'start') return 'Move START';
    if (h !== null) return `Pick up waypoint ${h + 1}`;
    return `Add waypoint ${this.sel >= 0 ? this.sel + 2 : this.cur.points.length + 1}`;
  }

  // ------------------------------------------------------------------ rendering
  render() {
    if (!this.open || !this.cur) return;
    this._drawField();
    this._renderTop();
    this._renderPanel();
    this._renderHelp();
  }

  _renderTop() {
    const est = estimateTime(this.cur, this.robotCfg);
    const over = est > AUTO_S;
    $('edTop').innerHTML = `<div class="heading">Auto Editor</div>
      <div class="edName">${this.cur.name}</div>
      <div class="edTime ${over ? 'bad' : ''}" title="A rough estimate: driving, intaking and shooting">
        <div class="edBar"><div style="width:${Math.min(100, (100 * est) / AUTO_S)}%"></div></div>
        <span>≈ ${est.toFixed(1)} s of ${AUTO_S} s${over ? ' — too long' : ''}</span></div>
      <div class="btn small" data-top="test">Test ▶ <kbd>Y</kbd></div>
      <div class="btn small secondary" data-top="done">Done <kbd>B</kbd></div>`;
    for (const el of document.querySelectorAll('#edTop [data-top]')) el.onclick = () => this._button(el.dataset.top);
  }

  _renderPanel() {
    const rows = this._rows();
    const focusRows = this._focusRows(rows);
    if (!focusRows.some((r) => r.id === this.rowId)) this.rowId = (focusRows[0] || {}).id;
    const active = this.mode === 'panel';
    let desc = '';
    const html = rows.map((r) => {
      const f = active && r.id === this.rowId ? 'focus' : '';
      if (f) desc = r.desc || '';
      const sel = r.kind === 'pt' && r.i === this.sel ? 'sel' : '';
      switch (r.kind) {
        case 'sec': return `<div class="edSec">${r.label}</div>`;
        case 'sub': return `<div class="edSub">${r.label}</div>`;
        case 'note': return `<div class="edNote">${r.label}</div>`;
        case 'opt': return `<div class="opt ${f}" data-row="${r.id}"><span class="ol">${r.label}</span><span class="ov"><span class="arr">◀</span>${r.value}<span class="arr">▶</span></span></div>`;
        case 'btns': return `<div class="edBtns ${f}" data-row="${r.id}">${r.btns.map(([k, t], j) => `<div class="btn small secondary ${f && j === this.btn ? 'bfocus' : ''}" data-btn="${k}" data-j="${j}">${t}</div>`).join('')}</div>`;
        case 'pt': return `<div class="edPt ${f} ${sel}" data-row="${r.id}">${r.value}</div>`;
        default: return '';
      }
    }).join('');
    const side = $('edSide');
    side.classList.toggle('active', active);
    side.innerHTML = html + `<div class="edDesc">${desc}</div>`;
    for (const el of side.querySelectorAll('[data-row]')) {
      el.onclick = (e) => {
        const r = rows.find((x) => x.id === el.dataset.row);
        this.mode = 'panel';
        this.rowId = r.id;
        const b = e.target.closest('[data-btn]');
        if (r.kind === 'btns') { if (b) { this.btn = +b.dataset.j; this._button(b.dataset.btn); } else this.render(); return; }
        if (r.kind === 'opt') { this._change(r.id, e.offsetX < el.clientWidth / 2 ? -1 : 1); return; }
        this._activateRow(r);
      };
    }
    const fe = side.querySelector('.focus');
    if (fe) fe.scrollIntoView({ block: 'nearest' });
  }

  // the controls bar: always visible, for whatever is active right now
  _renderHelp() {
    const pad = this.source === 'gamepad';
    const q = this.cur.points[this.sel];
    let html;
    if (this.mode === 'panel') {
      html = pad
        ? [chip('D-pad ▲▼', 'Choose a setting'), chip('D-pad ◀▶', 'Change it'), chip('A', 'Select'), chip('Left stick', 'Back to the field'), chip('LB / RB', 'Prev / next waypoint'), chip('Y', 'Test in a match'), chip('B', 'Back to the field')]
        : [chip('↑ ↓', 'Choose a setting'), chip('← →', 'Change it'), chip('Enter', 'Select'), chip('W A S D', 'Back to the field'), chip('F / R', 'Prev / next waypoint'), chip('H', 'Test in a match'), chip('Esc', 'Done')];
    } else {
      const act = this._cursorAction();
      html = pad
        ? [chip('Left stick', 'Move the cursor (hold LS click: fine)'), chip('A', act), q ? chip('X', `Delete waypoint ${this.sel + 1}`) : '', chip('LB / RB', 'Prev / next waypoint'), chip('D-pad', 'Edit the settings →'), chip('Y', 'Test in a match'), chip('B', this.grab !== null ? 'Drop' : 'Done')]
        : [chip('W A S D', 'Move the cursor'), chip('Enter', act), q ? chip('Delete', `Delete waypoint ${this.sel + 1}`) : '', chip('F / R', 'Prev / next waypoint'), chip('↑ ↓ ← →', 'Edit the settings →'), chip('H', 'Test in a match'), chip('Esc', 'Done')];
    }
    $('edHelp').innerHTML = `<div class="edKeys">${html.join('')}</div>`
      + `<div class="edMouse">🖱 Click the field to add a waypoint · drag waypoints or the START box to move them · right-click deletes · click a setting's ◀ / ▶ side to change it</div>`;
    $('edLegend').innerHTML = [['#e8ecf3', 'Drive'], ['#37d67a', 'Intake'], ['#ffe066', 'Shoot'], ['#ff9f1a', 'Intake + shoot']].map(([c, t]) => `<span><i style="background:${c}"></i>${t}</span>`).join('')
      + '<span><i class="dash"></i>Shoots / passes anywhere</span><span><i class="thick"></i>Thicker = faster</span><span><b class="red">●</b> Past the CENTER LINE (G403 risk)</span>';
    $('edField').classList.toggle('active', this.mode === 'field');
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
    g.fillText('TRENCH', this.px(HUB.fx) - 20, this.py(FIELD_W) + 14);
    g.fillText('TRENCH', this.px(HUB.fx) - 20, this.py(0) - 6);
    g.fillText('BUMP', this.px(HUB.fx) - 14, this.py(HUB.fy + hs + BUMP.width / 2) + 4);
    g.fillText('BUMP', this.px(HUB.fx) - 14, this.py(HUB.fy - hs - BUMP.width / 2) + 4);
    g.fillText('ALLIANCE ZONE', this.px(0.3), this.py(0) - 6);
    g.fillText('CENTER LINE', this.px(FIELD_L / 2) + 4, this.py(FIELD_W) + 14);
    // FUEL
    g.fillStyle = 'rgba(244,210,58,.55)';
    const r = Math.max(1.2, 0.075 * S);
    for (const f of this.fuelPts) { g.beginPath(); g.arc(this.px(f.fx), this.py(f.fy), r, 0, Math.PI * 2); g.fill(); }
    // start pose
    const sfx = this.startFx();
    const startHot = this.grab === 'start' || this.drag === 'start';
    this._rect(g, sfx - this.halfL, a.startFy - this.halfW, sfx + this.halfL, a.startFy + this.halfW, 'rgba(47,111,240,.45)', startHot ? '#fff' : '#9cc0ff');
    g.fillStyle = '#fff';
    g.font = '700 11px Inter, sans-serif';
    g.fillText('START', this.px(sfx) - 17, this.py(a.startFy) + 4);
    // path, with arrows showing the direction of travel
    let prev = { fx: sfx, fy: a.startFy };
    a.points.forEach((p) => {
      const col = segColor(p);
      g.strokeStyle = col;
      g.lineWidth = 3 + 3 * (p.speed - 0.3);
      if (p.shoot === 'always') g.setLineDash([8, 5]);
      const x0 = this.px(prev.fx), y0 = this.py(prev.fy), x1 = this.px(p.fx), y1 = this.py(p.fy);
      g.beginPath(); g.moveTo(x0, y0); g.lineTo(x1, y1); g.stroke();
      g.setLineDash([]);
      const L = Math.hypot(x1 - x0, y1 - y0);
      if (L > 40) {
        const ux = (x1 - x0) / L, uy = (y1 - y0) / L, mx = (x0 + x1) / 2, my = (y0 + y1) / 2;
        g.fillStyle = col;
        g.beginPath(); g.moveTo(mx + ux * 7, my + uy * 7); g.lineTo(mx - ux * 5 - uy * 6, my - uy * 5 + ux * 6); g.lineTo(mx - ux * 5 + uy * 6, my - uy * 5 - ux * 6); g.fill();
      }
      prev = p;
    });
    // waypoints
    a.points.forEach((p, i) => {
      const x = this.px(p.fx), y = this.py(p.fy);
      const sel = i === this.sel;
      const bad = p.fx + this.halfL > FIELD_L / 2; // past the CENTER LINE (G403 risk)
      if (sel) {
        // the robot's footprint at the selected waypoint
        g.strokeStyle = 'rgba(255,255,255,.4)';
        g.lineWidth = 1;
        g.setLineDash([4, 3]);
        g.strokeRect(x - this.halfL * S, y - this.halfW * S, 2 * this.halfL * S, 2 * this.halfW * S);
        g.setLineDash([]);
      }
      g.beginPath();
      g.arc(x, y, sel ? 12 : 9, 0, Math.PI * 2);
      g.fillStyle = bad ? '#e03434' : sel ? '#fff' : '#1b1e25';
      g.fill();
      g.lineWidth = 2;
      g.strokeStyle = this.grab === i || this.drag === i ? '#37d67a' : sel ? '#2f6ff0' : '#fff';
      g.stroke();
      g.fillStyle = sel && !bad ? '#111' : '#fff';
      g.font = '700 11px Inter, sans-serif';
      const label = String(i + 1);
      g.fillText(label, x - g.measureText(label).width / 2, y + 4);
      const tag = p.action === 'shoot' ? 'SHOOT' : p.action === 'stop' ? 'STOP' : p.action.startsWith('wait') ? `WAIT ${p.action.slice(4)}s` : '';
      if (tag) {
        g.font = '700 10px Inter, sans-serif';
        g.fillStyle = p.action === 'shoot' ? '#ffe066' : '#c7cdd8';
        g.fillText(tag, x + 13, y - 9);
      }
    });
    // cursor (controller / keyboard) and what A does there; the mouse gets the same hint
    const at = this.mode === 'field' && this.source !== 'mouse' ? this.cursor : this.hover;
    if (this.mode === 'field' && this.source !== 'mouse') {
      const x = this.px(this.cursor.fx), y = this.py(this.cursor.fy);
      g.strokeStyle = this.grab !== null ? '#37d67a' : '#fff';
      g.lineWidth = 1.5;
      g.beginPath(); g.moveTo(x - 10, y); g.lineTo(x + 10, y); g.moveTo(x, y - 10); g.lineTo(x, y + 10); g.stroke();
      g.beginPath(); g.arc(x, y, PT_HIT * S, 0, Math.PI * 2); g.stroke();
    }
    if (at && this.drag === null) {
      const x = this.px(at.fx), y = this.py(at.fy);
      const pad = this.source === 'gamepad', mouse = this.source === 'mouse';
      const act = this._cursorAction(at);
      const text = mouse ? (act.startsWith('Add') ? `Click: ${act.toLowerCase()}` : act.startsWith('Pick') || act === 'Move START' ? `Drag: ${act.replace('Pick up', 'move').toLowerCase()}` : act) : `${pad ? 'A' : 'Enter'}: ${act}`;
      g.font = '600 12px Inter, sans-serif';
      const w = g.measureText(text).width + 12;
      const tx = Math.min(x + 16, this.canvas.width / this.dpr - w - 4), ty = Math.max(y - 26, 4);
      g.fillStyle = 'rgba(10,12,18,.85)';
      g.fillRect(tx, ty, w, 20);
      g.fillStyle = '#fff';
      g.fillText(text, tx + 6, ty + 14);
    }
  }
}
