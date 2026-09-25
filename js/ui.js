// Menus (fully navigable with an Xbox controller) and the in-match HUD.
import { ROBOTS, ROBOT_ORDER, AUTO_ROUTINES } from './robotConfigs.js';
import { START_POSITIONS, START_ORDER } from './auto.js';
import { CAMERA_MODES, CAMERA_NAMES } from './cameras.js';
import { DRIVER_STATIONS, TIMING, BLUE, RED, other } from './constants.js';

const $ = (id) => document.getElementById(id);

export const DEFAULT_SETTINGS = {
  robot: '2910', alliance: BLUE, ds: 1, start: 'rightTrench', preload: 8, auto: 'sweep',
  hp: 'manual', climber: 'none', camera: 'driver', preview: 'on',
};

export function loadSettings() {
  try {
    const s = JSON.parse(localStorage.getItem('rebuiltSim.settings') || '{}');
    return { ...DEFAULT_SETTINGS, ...s };
  } catch { return { ...DEFAULT_SETTINGS }; }
}
export function saveSettings(s) {
  try { localStorage.setItem('rebuiltSim.settings', JSON.stringify(s)); } catch { /* storage unavailable */ }
}

const OPTIONS = [
  { key: 'alliance', label: 'Alliance', values: [[BLUE, 'Blue'], [RED, 'Red']] },
  { key: 'ds', label: 'Driver station', values: DRIVER_STATIONS.map((d, i) => [i, d.name]) },
  { key: 'start', label: 'Starting position', values: START_ORDER.map((k) => [k, START_POSITIONS[k].name]), desc: 'BUMPERS overlap the ROBOT STARTING LINE without touching a BUMP (G303).' },
  { key: 'preload', label: 'Preloaded FUEL', values: [0, 1, 2, 3, 4, 5, 6, 7, 8].map((n) => [n, String(n)]), desc: 'Up to 8 FUEL may be preloaded. Unused preload FUEL is staged in the NEUTRAL ZONE.' },
  { key: 'auto', label: 'Auto routine', values: Object.entries(AUTO_ROUTINES).map(([k, v]) => [k, v.name]), descFn: (s) => AUTO_ROUTINES[s.auto].desc + ' (G402: drivers cannot control the robot in AUTO.)' },
  { key: 'hp', label: 'Human player', values: [['manual', 'Manual (X / Y)'], ['auto', 'Auto-throw when HUB active']] },
  { key: 'climber', label: 'Climber add-on', values: [['none', 'None (as built)'], ['l1', 'Level 1 hook'], ['l3', 'Level 1-3 climber']], desc: 'None of these three robots climbed in 2026. Add a hypothetical climber to try the TOWER.' },
  { key: 'camera', label: 'Camera', values: CAMERA_MODES.map((m) => [m, CAMERA_NAMES[m]]) },
  { key: 'preview', label: 'Shot preview line', values: [['on', 'On'], ['off', 'Off']] },
];

const CONTROLS = [
  ['Drive (field-relative)', 'Left stick', 'W A S D'],
  ['Rotate', 'Right stick X', 'Q / E or ← →'],
  ['Shoot (auto-aim, shoot on the move)', 'RT', 'Space'],
  ['Intake', 'LT', 'Shift'],
  ['Pass FUEL to your ALLIANCE ZONE corner', 'RB (hold)', 'R'],
  ['Outtake / eject', 'LB', 'F'],
  ['Human player: open/close CHUTE DOOR', 'X', 'G'],
  ['Human player: throw FUEL at HUB', 'Y (hold)', 'H'],
  ['Climb / cancel / lower (add-on)', 'A', 'C'],
  ['Climb level up / down', 'D-pad ↑ / ↓', '↑ / ↓'],
  ['Camera', 'D-pad ← / →', '[ / ]'],
  ['Toggle field / robot-relative', 'B', 'B'],
  ['Slow mode (hold)', 'Left stick click', 'X'],
  ['Pause', 'Menu (☰)', 'Esc'],
  ['Restart match (hold 1s)', 'View (⧉)', 'Backspace'],
];

export class UI {
  constructor(settings, handlers) {
    this.s = settings;
    this.h = handlers;
    this.screen = 'menu';
    this.focus = 0;
    this.pauseFocus = 0;
    this.resFocus = 0;
    this.toastEls = [];
    this._buildControls();
    this.renderMenu();
    this._timelineBuilt = false;
  }

  // ------------------------------------------------------------------ screens
  show(screen) {
    this.screen = screen;
    $('menu').classList.toggle('hidden', screen !== 'menu');
    $('pause').classList.toggle('hidden', screen !== 'pause');
    $('results').classList.toggle('hidden', screen !== 'results');
    $('controls').classList.toggle('hidden', screen !== 'controls');
    $('hud').classList.toggle('hidden', screen === 'menu');
    if (screen === 'menu') this.renderMenu();
    if (screen === 'pause') this.renderPause();
  }

  _menuItems() {
    // 0 = robot row, 1..N options, N+1 start, N+2 controls
    return 1 + OPTIONS.length + 2;
  }

  renderMenu() {
    const s = this.s;
    const cards = ROBOT_ORDER.map((k) => {
      const r = ROBOTS[k];
      const stats = Object.entries(r.stats).map(([a, b]) => `<tr><td class="muted">${a}</td><td>${b}</td></tr>`).join('');
      return `<div class="rcard ${s.robot === k ? 'sel' : ''} ${this.focus === 0 && s.robot === k ? 'focus' : ''}" data-robot="${k}">
        <div class="num">${r.team}</div><div class="nm">${r.teamName} · ${r.robotName}</div>
        <div class="arch">${r.archetype}</div><div class="desc">${r.blurb}</div><table>${stats}</table></div>`;
    }).join('');
    $('robotCards').innerHTML = cards;
    const opts = OPTIONS.map((o, i) => {
      const cur = o.values.find((v) => v[0] === s[o.key]) || o.values[0];
      return `<div class="opt ${this.focus === i + 1 ? 'focus' : ''}" data-i="${i + 1}"><span class="ol">${o.label}</span><span class="ov"><span class="arr">◀</span>${cur[1]}<span class="arr">▶</span></span></div>`;
    }).join('');
    const fi = this.focus;
    let desc = '';
    if (fi >= 1 && fi <= OPTIONS.length) {
      const o = OPTIONS[fi - 1];
      desc = o.descFn ? o.descFn(s) : o.desc || '';
    } else if (fi === 0) desc = 'Choose a robot with ◀ ▶ (D-pad / stick).';
    const N = OPTIONS.length;
    $('options').innerHTML = opts +
      `<div class="optdesc">${desc}</div>` +
      `<div class="btn ${fi === N + 1 ? 'focus' : ''}" data-i="${N + 1}">START MATCH</div>` +
      `<div class="btn secondary ${fi === N + 2 ? 'focus' : ''}" data-i="${N + 2}">CONTROLS</div>`;
    // mouse support
    for (const el of document.querySelectorAll('.rcard')) el.onclick = () => { this.s.robot = el.dataset.robot; this.focus = 0; this._changed(); };
    for (const el of document.querySelectorAll('#options [data-i]')) {
      el.onclick = (e) => {
        const i = +el.dataset.i;
        this.focus = i;
        if (i >= 1 && i <= N) this._cycleOption(i - 1, e.offsetX < el.clientWidth / 2 ? -1 : 1);
        else this._activate();
        this.renderMenu();
      };
    }
  }

  _cycleOption(oi, dir) {
    const o = OPTIONS[oi];
    let idx = o.values.findIndex((v) => v[0] === this.s[o.key]);
    idx = (idx + dir + o.values.length) % o.values.length;
    this.s[o.key] = o.values[idx][0];
    this._changed();
  }

  _changed() {
    saveSettings(this.s);
    this.renderMenu();
  }

  _activate() {
    const N = OPTIONS.length;
    if (this.focus === N + 1) this.h.onStart();
    else if (this.focus === N + 2) { this.prevScreen = 'menu'; this.show('controls'); }
  }

  renderPause() {
    const items = ['Resume', 'Restart match', 'Controls', 'Main menu'];
    $('pauseItems').innerHTML = items.map((t, i) => `<div class="btn ${i === 0 ? '' : 'secondary'} ${this.pauseFocus === i ? 'focus' : ''}" data-i="${i}">${t}</div>`).join('');
    for (const el of document.querySelectorAll('#pauseItems [data-i]')) el.onclick = () => { this.pauseFocus = +el.dataset.i; this._pauseActivate(); };
  }

  _pauseActivate() {
    const i = this.pauseFocus;
    if (i === 0) this.h.onResume();
    else if (i === 1) this.h.onRestart();
    else if (i === 2) { this.prevScreen = 'pause'; this.show('controls'); }
    else this.h.onMenu();
  }

  _buildControls() {
    $('controlsBody').innerHTML = `<table class="ctable"><tr><th>Action</th><th>Xbox controller</th><th>Keyboard</th></tr>${CONTROLS.map((c) => `<tr><td>${c[0]}</td><td>${c[1]}</td><td><kbd>${c[2]}</kbd></td></tr>`).join('')}</table>
      <p class="muted" style="font-size:13px">Menus: D-pad / left stick to move, ◀ ▶ to change, A to select, B to go back. Shooting auto-targets your HUB when your BUMPERS are in your ALLIANCE ZONE; anywhere else RT lobs FUEL back into your ALLIANCE ZONE.</p>`;
  }

  // Returns true when a menu overlay is open (gameplay input suppressed)
  handleInput(inp) {
    const p = inp.pressed, n = inp.nav;
    if (this.screen === 'menu') {
      const N = this._menuItems();
      if (n.up) { this.focus = (this.focus - 1 + N) % N; this.renderMenu(); }
      if (n.down) { this.focus = (this.focus + 1) % N; this.renderMenu(); }
      if (n.left || n.right) {
        const d = n.left ? -1 : 1;
        if (this.focus === 0) {
          const i = ROBOT_ORDER.indexOf(this.s.robot);
          this.s.robot = ROBOT_ORDER[(i + d + ROBOT_ORDER.length) % ROBOT_ORDER.length];
          this._changed();
        } else if (this.focus <= OPTIONS.length) this._cycleOption(this.focus - 1, d);
      }
      if (p.a) {
        if (this.focus === 0) { this.focus = OPTIONS.length + 1; this.renderMenu(); }
        else if (this.focus > OPTIONS.length) this._activate();
        else this._cycleOption(this.focus - 1, 1);
      }
      if (p.start) this.h.onStart();
      return true;
    }
    if (this.screen === 'pause') {
      if (n.up) { this.pauseFocus = (this.pauseFocus + 3) % 4; this.renderPause(); }
      if (n.down) { this.pauseFocus = (this.pauseFocus + 1) % 4; this.renderPause(); }
      if (p.a) this._pauseActivate();
      else if (p.b || p.start) this.h.onResume();
      return true;
    }
    if (this.screen === 'controls') {
      if (p.b || p.start || p.a) this.show(this.prevScreen || 'menu');
      return true;
    }
    if (this.screen === 'results') {
      if (n.left || n.right) { this.resFocus = 1 - this.resFocus; this._renderResItems(); }
      if (p.a) (this.resFocus === 0 ? this.h.onRestart : this.h.onMenu)();
      if (p.b) this.h.onMenu();
      return true;
    }
    return false;
  }

  // ------------------------------------------------------------------ results
  showResults(game) {
    const m = game.match;
    const me = this.s.alliance;
    const col = (a) => {
      const s = m.score[a];
      return `<div class="rescol ${a}"><div class="big">${m.total(a)}</div><table>
        <tr><td>AUTO FUEL</td><td>${s.autoFuel}</td></tr>
        <tr><td>TELEOP FUEL</td><td>${s.teleFuel}</td></tr>
        <tr><td>TOWER</td><td>${s.autoTower + s.teleTower}</td></tr>
        <tr><td>FOUL points received</td><td>${s.foulPts}</td></tr></table></div>`;
    };
    const tb = m.total(BLUE), tr = m.total(RED);
    const win = tb === tr ? 'TIE' : (tb > tr ? 'BLUE' : 'RED') + ' WINS';
    const r = game.robot;
    const s = m.score[me];
    const fouls = m.score[me].fouls;
    $('resHeading').textContent = 'Match Results';
    $('resBody').innerHTML = `<div class="winner">${win}</div>
      <div class="resgrid">${col(BLUE)}<div class="vs">VS</div>${col(RED)}</div>
      <div class="stats">
        <div class="stat"><div class="sv">${s.autoFuel + s.teleFuel}</div><div class="sk">FUEL scored in active HUB</div></div>
        <div class="stat"><div class="sv">${s.inactiveFuel}</div><div class="sk">FUEL into inactive HUB (0 pts)</div></div>
        <div class="stat"><div class="sv">${r.stats.shots}</div><div class="sk">FUEL launched by robot</div></div>
        <div class="stat"><div class="sv">${r.stats.intaked}</div><div class="sk">FUEL intaked</div></div>
      </div>
      <div class="ptitle small">Your fouls (${fouls.length})</div>
      <div class="foullist">${fouls.length ? fouls.map((f) => `${f.rule} ${f.type.toUpperCase()} (+${f.pts} to ${other(me).toUpperCase()}): ${f.desc}`).join('<br>') : 'None — clean match!'}</div>`;
    this.resFocus = 0;
    this._renderResItems();
    this.show('results');
  }

  _renderResItems() {
    $('resItems').innerHTML = ['Rematch (A)', 'Main menu (B)'].map((t, i) => `<div class="btn ${i ? 'secondary' : ''} ${this.resFocus === i ? 'focus' : ''}" data-i="${i}" style="min-width:200px">${t}</div>`).join('');
    for (const el of document.querySelectorAll('#resItems [data-i]')) el.onclick = () => (+el.dataset.i === 0 ? this.h.onRestart : this.h.onMenu)();
  }

  // ------------------------------------------------------------------ HUD
  toast(text, kind = '') {
    const el = document.createElement('div');
    el.className = 'toast ' + kind;
    el.textContent = text;
    $('toasts').appendChild(el);
    this.toastEls.push(el);
    while (this.toastEls.length > 4) this.toastEls.shift().remove();
    setTimeout(() => { el.remove(); this.toastEls = this.toastEls.filter((e) => e !== el); }, 3200);
  }

  bigMessage(text, dur = 1.4) {
    const el = $('bigmsg');
    el.textContent = text;
    el.classList.add('show');
    clearTimeout(this._bigT);
    this._bigT = setTimeout(() => el.classList.remove('show'), dur * 1000);
  }

  camName(text) {
    $('camName').textContent = '📷 ' + text;
  }

  _buildTimeline() {
    const segs = [['AUTO', TIMING.auto], ['TRANS', TIMING.transition], ['SHIFT 1', TIMING.shift], ['SHIFT 2', TIMING.shift], ['SHIFT 3', TIMING.shift], ['SHIFT 4', TIMING.shift], ['END GAME', TIMING.endgame]];
    $('timeline').innerHTML = segs.map(([n, d], i) => `<div class="seg" data-i="${i}" style="width:${d * 3.3}px"><div class="prog"></div><span>${n}</span></div>`).join('');
    this.segEls = [...document.querySelectorAll('#timeline .seg')];
    this._timelineBuilt = true;
  }

  updateHUD(game) {
    const { match: m, robot: r, hp, settings: s, input } = game;
    if (!this._timelineBuilt) this._buildTimeline();
    const me = s.alliance;
    $('scoreBlue').textContent = m.total(BLUE);
    $('scoreRed').textContent = m.total(RED);
    $('phase').textContent = m.phaseLabel();
    $('clock').textContent = m.clockText();
    for (const a of [BLUE, RED]) {
      const el = $(a === BLUE ? 'hubBlue' : 'hubRed');
      const on = m.hubActive(a);
      const mode = m.hubLightMode(a);
      el.classList.toggle('on', on);
      el.classList.toggle('warn', mode === 'warning');
      el.textContent = on ? 'HUB ON' : 'HUB OFF';
    }
    // your hub
    const mh = $('myhub');
    const on = m.hubActive(me);
    const nc = m.phase === 'auto' ? null : m.hubNextChange(me);
    let sub = '';
    if (m.phase === 'pre') sub = 'Match starting';
    else if (m.phase === 'auto') sub = `AUTO — both HUBS active`;
    else if (m.phase === 'autoGap') sub = m.firstInactive ? '' : 'Waiting for FMS game data…';
    else if (m.phase === 'teleop') {
      if (nc !== null) sub = on ? `goes inactive in ${Math.ceil(nc)}s` : `active in ${Math.ceil(nc)}s`;
      else sub = on ? 'active until the end' : '';
      if (m.shiftIndex() === 0 && m.firstInactive) sub += m.firstInactive === me ? ' · you go inactive first' : ' · opponent goes inactive first';
    } else if (m.phase === 'post') sub = 'FUEL still being counted (3s)';
    $('myhubState').textContent = m.phase === 'pre' ? 'GET READY' : on ? 'YOUR HUB ACTIVE' : 'YOUR HUB INACTIVE';
    $('myhubSub').textContent = sub;
    mh.className = on ? 'active' : 'inactive';
    if (m.hubLightMode(me) === 'warning') mh.classList.add('warning');
    // timeline
    const autoFrac = m.phase === 'auto' ? m.phaseTime / TIMING.auto : m.phase === 'pre' ? 0 : 1;
    const tt = m.teleopElapsed();
    const bounds = [0, TIMING.transition, TIMING.transition + TIMING.shift, TIMING.transition + 2 * TIMING.shift, TIMING.transition + 3 * TIMING.shift, TIMING.transition + 4 * TIMING.shift, TIMING.teleop];
    this.segEls.forEach((el, i) => {
      let frac;
      if (i === 0) frac = autoFrac;
      else frac = Math.max(0, Math.min(1, (tt - bounds[i - 1]) / (bounds[i] - bounds[i - 1])));
      el.firstChild.style.width = frac * 100 + '%';
      el.classList.remove('mine', 'theirs');
      if (i >= 2 && i <= 5 && m.firstInactive) {
        const shiftOdd = (i - 1) % 2 === 1;
        const mineActive = shiftOdd ? me !== m.firstInactive : me === m.firstInactive;
        el.classList.add(mineActive ? 'mine' : 'theirs');
      } else if (i === 0 || i === 1 || i === 6) el.classList.add('mine');
    });
    // robot panel
    const cap = r.capacity();
    $('rpTeam').textContent = r.cfg.team;
    $('rpName').textContent = r.cfg.robotName === r.cfg.archetype ? `${r.cfg.teamName} · ${r.cfg.archetype}` : `${r.cfg.robotName} · ${r.cfg.archetype}`;
    $('fuelFill').style.width = (100 * r.stored.length) / cap + '%';
    $('fuelText').textContent = `${r.stored.length} / ${cap} FUEL`;
    const st = $('rpShooter');
    st.textContent = r.status;
    st.className = 'v ' + (r.ready ? 'ok' : /range|Outside|Disabled/.test(r.status) ? 'bad' : r.status === 'Empty' ? '' : 'warn');
    const z = $('rpZone');
    z.textContent = r.lastInZone ? 'ALLIANCE ZONE (can score)' : 'Outside — pass only';
    z.className = 'v ' + (r.lastInZone ? 'ok' : 'warn');
    $('rpIntake').textContent = r.intakeSpeed > 0 ? 'Intaking' : r.intakeSpeed < 0 ? 'Ejecting' : r.intakeDeploy > 0.5 ? 'Deployed' : 'Stowed';
    $('rpClimbRow').classList.toggle('hidden', !r.climberCfg);
    if (r.climberCfg) $('rpClimb').textContent = `${r.climbState === 'none' ? 'Ready' : r.climbState} · target L${r.climbTarget}${r.climbLevel ? ' · at L' + r.climbLevel : ''}`;
    $('rpDrive').textContent = (game.fieldRelative ? 'Field-relative' : 'Robot-relative') + (game.slow ? ' · SLOW' : '') + (m.isAuto ? ' · AUTO' : '');
    // human player
    $('hpChute').textContent = game.fuel.chuteCount(me);
    $('hpCorral').textContent = hp.corralCount();
    $('hpDoor').textContent = hp.doorOpen ? 'OPEN' : 'Closed';
    $('hpMode').textContent = hp.auto ? 'Auto-throw' : 'Manual';
    const f = m.score[me].fouls;
    $('foulCount').textContent = `${f.length} (${f.reduce((a, b) => a + b.pts, 0)} pts to opp.)`;
    // hint
    const pad = input.lastSource === 'gamepad';
    $('hint').innerHTML = pad
      ? '<b>RT</b> Shoot · <b>LT</b> Intake · <b>RB</b> Pass · <b>LB</b> Eject · <b>X</b> Chute door · <b>Y</b> HP throw · <b>A</b> Climb · <b>D-pad ◀▶</b> Camera · <b>☰</b> Pause'
      : '<b>WASD</b> Drive · <b>Q/E</b> Rotate · <b>Space</b> Shoot · <b>Shift</b> Intake · <b>R</b> Pass · <b>F</b> Eject · <b>G</b> Chute · <b>H</b> HP throw · <b>C</b> Climb · <b>[ ]</b> Camera · <b>Esc</b> Pause';
  }

  setPadStatus(name) {
    $('padStatus').textContent = name ? `🎮 Controller connected: ${name}` : 'Press any button on your Xbox controller to connect it (keyboard works too)';
  }
}
