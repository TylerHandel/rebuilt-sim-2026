// Xbox controller via the Gamepad API ("standard" mapping), with a keyboard fallback.
//   Buttons: 0 A, 1 B, 2 X, 3 Y, 4 LB, 5 RB, 6 LT, 7 RT, 8 View, 9 Menu, 10 LS, 11 RS,
//            12 D-up, 13 D-down, 14 D-left, 15 D-right
//   Axes:    0 LX, 1 LY, 2 RX, 3 RY
const BTN = ['a', 'b', 'x', 'y', 'lb', 'rb', 'lt', 'rt', 'back', 'start', 'ls', 'rs', 'up', 'down', 'left', 'right'];

const KEYMAP = {
  Enter: 'a', Space: 'rt', ShiftLeft: 'lt', ShiftRight: 'lt', KeyF: 'lb', KeyR: 'rb',
  KeyC: 'a', KeyB: 'b', KeyG: 'x', KeyH: 'y', Escape: 'start', KeyP: 'start', Backspace: 'back',
  ArrowUp: 'up', ArrowDown: 'down', BracketLeft: 'left', BracketRight: 'right', KeyV: 'right',
  KeyT: 'rs', KeyX: 'ls',
};

function deadband(v, d = 0.1) {
  const a = Math.abs(v);
  if (a < d) return 0;
  return Math.sign(v) * ((a - d) / (1 - d));
}

export class Input {
  constructor() {
    this.keys = new Set();
    this.prevHeld = {};
    this.state = this._empty();
    this.lastSource = 'keyboard';
    this.padName = '';
    this.repeat = {};
    window.addEventListener('keydown', (e) => {
      if (e.target && (e.target.tagName === 'INPUT' || e.target.tagName === 'SELECT')) return;
      this.keys.add(e.code);
      this.lastSource = 'keyboard';
      if (['Space', 'ArrowUp', 'ArrowDown', 'ArrowLeft', 'ArrowRight', 'Backspace'].includes(e.code)) e.preventDefault();
    });
    window.addEventListener('keyup', (e) => this.keys.delete(e.code));
    window.addEventListener('blur', () => this.keys.clear());
    window.addEventListener('gamepadconnected', (e) => { this.padName = e.gamepad.id; });
    window.addEventListener('gamepaddisconnected', () => { this.padName = ''; });
  }

  _empty() {
    const held = {}, pressed = {};
    for (const b of BTN) { held[b] = false; pressed[b] = false; }
    return { lx: 0, ly: 0, rx: 0, ry: 0, lt: 0, rt: 0, held, pressed, nav: { up: false, down: false, left: false, right: false } };
  }

  _pad() {
    const pads = navigator.getGamepads ? navigator.getGamepads() : [];
    let best = null;
    for (const p of pads) {
      if (!p || !p.connected) continue;
      if (!best || (p.mapping === 'standard' && best.mapping !== 'standard')) best = p;
    }
    return best;
  }

  poll(dt) {
    const s = this._empty();
    const pad = this._pad();
    if (pad) {
      this.padName = pad.id;
      const ax = pad.axes;
      s.lx = deadband(ax[0] || 0);
      s.ly = deadband(ax[1] || 0);
      s.rx = deadband(ax[2] || 0);
      s.ry = deadband(ax[3] || 0);
      const b = pad.buttons;
      for (let i = 0; i < BTN.length && i < b.length; i++) s.held[BTN[i]] = !!(b[i] && (b[i].pressed || b[i].value > 0.5));
      s.lt = b[6] ? b[6].value : 0;
      s.rt = b[7] ? b[7].value : 0;
      if (Math.abs(s.lx) + Math.abs(s.ly) + Math.abs(s.rx) + s.lt + s.rt > 0.2 || BTN.some((k) => s.held[k])) this.lastSource = 'gamepad';
    }
    // keyboard (merged)
    const k = this.keys;
    const kx = (k.has('KeyD') ? 1 : 0) - (k.has('KeyA') ? 1 : 0);
    const ky = (k.has('KeyS') ? 1 : 0) - (k.has('KeyW') ? 1 : 0);
    const kr = (k.has('KeyL') || k.has('ArrowRight') || k.has('KeyE') ? 1 : 0) - (k.has('KeyJ') || k.has('ArrowLeft') || k.has('KeyQ') ? 1 : 0);
    if (kx || ky) {
      const m = Math.hypot(kx, ky);
      s.lx = kx / m;
      s.ly = ky / m;
    }
    if (kr) s.rx = kr;
    for (const [code, name] of Object.entries(KEYMAP)) if (k.has(code)) s.held[name] = true;
    if (s.held.lt) s.lt = Math.max(s.lt, 1);
    if (s.held.rt) s.rt = Math.max(s.rt, 1);
    s.held.lt = s.lt > 0.3;
    s.held.rt = s.rt > 0.3;
    // edges
    for (const b of BTN) {
      s.pressed[b] = s.held[b] && !this.prevHeld[b];
      this.prevHeld[b] = s.held[b];
    }
    // menu navigation (D-pad / left stick / arrows) with auto-repeat
    const dirs = {
      up: s.held.up || s.ly < -0.6 || k.has('KeyW'),
      down: s.held.down || s.ly > 0.6 || k.has('KeyS'),
      left: s.held.left || s.lx < -0.6 || k.has('ArrowLeft') || k.has('KeyA'),
      right: s.held.right || s.lx > 0.6 || k.has('ArrowRight') || k.has('KeyD'),
    };
    for (const d of Object.keys(dirs)) {
      const r = this.repeat[d] || { t: 0, on: false };
      if (dirs[d]) {
        if (!r.on) { s.nav[d] = true; r.t = 0.38; }
        else { r.t -= dt; if (r.t <= 0) { s.nav[d] = true; r.t = 0.12; } }
        r.on = true;
      } else r.on = false;
      this.repeat[d] = r;
    }
    this.state = s;
    s.source = this.lastSource;
    return s;
  }

  get hasPad() {
    return !!this._pad();
  }
}
