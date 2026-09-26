// Menus (fully navigable with an Xbox controller) and the in-match HUD.
import { ROBOTS, ROBOT_ORDER, AUTO_ROUTINES } from './robotConfigs.js';
import { START_POSITIONS, START_ORDER } from './auto.js';
import { CAMERA_MODES, CAMERA_NAMES } from './cameras.js';
import { DRIVER_STATIONS, TIMING, BLUE, RED, other } from './constants.js';
import { loadAutos, isCustom, getCustom, CUSTOM_PREFIX } from './customAutos.js';
import { OPP_STRATEGIES, OPP_ORDER, OPP_SKILLS, SKILL_ORDER, BRAIN_SPEC } from './opponent.js';
import { PIN_LIMIT } from './rules.js';
import { fmtValue } from './tuning.js';
import { modelCapacity } from './hopper.js';
import { DEFAULT_SLOTS, SLOT_DRIVERS, slotAlliance } from './game.js';

const $ = (id) => document.getElementById(id);

export const DEFAULT_SETTINGS = {
  matchMode: '1v1', slots: DEFAULT_SLOTS,
  robot: '2910', alliance: BLUE, ds: 1, start: 'rightTrench', preload: 8, auto: 'best',
  hp: 'manual', climber: 'none', camera: 'driver', preview: 'on',
  opponent: 'scorer', oppRobot: '4414', oppSkill: 'regional', customSide: 'drawn',
  driver: 'human', driverSkill: 'trained', mode: 'normal', role: 'score',
};

export function loadSettings() {
  let s;
  try {
    s = { ...DEFAULT_SETTINGS, ...JSON.parse(localStorage.getItem('rebuiltSim.settings') || '{}') };
  } catch { s = { ...DEFAULT_SETTINGS }; }
  // a custom auto that was deleted falls back to the default routine
  const autoOk = (k) => (isCustom(k) ? !!getCustom(k) : !!AUTO_ROUTINES[k]);
  if (!autoOk(s.auto)) s.auto = DEFAULT_SETTINGS.auto;
  if (!['practice', '1v1', '3v3'].includes(s.matchMode)) s.matchMode = '1v1';
  if (s.opponent === 'off') s.opponent = 'scorer'; // "no opponent" is Practice now
  s.slots = DEFAULT_SLOTS.map((d, i) => {
    const sl = { ...d, ...((Array.isArray(s.slots) && s.slots[i]) || {}) };
    if (!autoOk(sl.auto)) sl.auto = d.auto;
    if (!ROBOTS[sl.robot]) sl.robot = d.robot;
    return sl;
  });
  return s;
}
export function saveSettings(s) {
  try { localStorage.setItem('rebuiltSim.settings', JSON.stringify(s)); } catch { /* storage unavailable */ }
}

// ------------------------------------------------------------------ menu options
// Each option: label, values(s) -> [[value, text]], get(s), set(s, v), desc(s)
const autoValues = () => [...Object.entries(AUTO_ROUTINES).map(([k, v]) => [k, v.name]), ...loadAutos().map((a) => [CUSTOM_PREFIX + a.id, '✎ ' + a.name])];
const autoDesc = (k) => (isCustom(k) ? 'Custom auto from the Auto Editor; it starts where you drew it.' : AUTO_ROUTINES[k].desc);
const robotName = (k) => `${ROBOTS[k].team} ${ROBOTS[k].archetype}`;
const robotValues = () => ROBOT_ORDER.map((k) => [k, robotName(k)]);
const opt = (key, label, values, desc = '') => ({
  label, values: typeof values === 'function' ? values : () => values,
  get: (s) => s[key], set: (s, v) => { s[key] = v; },
  desc: typeof desc === 'function' ? desc : () => desc,
});
const OPT = {
  robot: opt('robot', 'Robot', robotValues),
  alliance: opt('alliance', 'Alliance', [[BLUE, 'Blue'], [RED, 'Red']]),
  ds: opt('ds', 'Driver station', DRIVER_STATIONS.map((d, i) => [i, d.name]), 'Where you stand behind your ALLIANCE WALL (for the Driver Station camera).'),
  auto: opt('auto', 'Auto routine', autoValues, (s) => autoDesc(s.auto) + ' Drivers can\'t control the robot in AUTO (G402).'),
  start: {
    label: 'Starting position',
    values: (s) => (isCustom(s.auto) ? [['drawn', 'As drawn in editor'], ['mirror', 'Mirrored left ↔ right']] : START_ORDER.map((k) => [k, START_POSITIONS[k].name])),
    get: (s) => (isCustom(s.auto) ? s.customSide : s.start),
    set: (s, v) => { if (isCustom(s.auto)) s.customSide = v; else s.start = v; },
    desc: (s) => (isCustom(s.auto) ? 'Custom autos start where you placed them; mirror to run it from the other side.'
      : s.auto === 'best' ? '"Best for this robot" picks its own starting position; this only applies to the other routines.'
        : 'BUMPERS overlap the ROBOT STARTING LINE without touching a BUMP (G303).'),
  },
  preload: opt('preload', 'Preloaded FUEL', [0, 1, 2, 3, 4, 5, 6, 7, 8].map((n) => [n, String(n)]), 'Up to 8 FUEL per robot. Unused preload FUEL is staged in the NEUTRAL ZONE.'),
  preloadAll: opt('preload', 'Preloaded FUEL (each robot)', [0, 1, 2, 3, 4, 5, 6, 7, 8].map((n) => [n, String(n)]), 'Up to 8 FUEL per robot. Unused preload FUEL is staged in the NEUTRAL ZONE.'),
  oppRobot: opt('oppRobot', 'Robot', robotValues, 'The AI can drive any of the three robots.'),
  opponent: opt('opponent', 'Strategy', OPP_ORDER.filter((k) => k !== 'off').map((k) => [k, OPP_STRATEGIES[k].name]), (s) => OPP_STRATEGIES[s.opponent].desc),
  oppSkill: opt('oppSkill', 'Skill', SKILL_ORDER.map((k) => [k, OPP_SKILLS[k].name]), (s) => OPP_SKILLS[s.oppSkill].desc),
  mode: opt('mode', 'Match type', [['normal', 'Normal'], ['training', 'Training (AI learns)']], (s) => (s.mode === 'training'
    ? 'Each match the AI tries a variation of its strategy and keeps what works against you; when you out-drive it, it copies part of your style. See AI TUNING.'
    : 'A regular match. Your driving is still recorded (AI TUNING shows it) so the AI can learn from it later.')),
  role: opt('role', 'Your role', [['score', 'Score (win the match)'], ['defense', 'Defense drill']], (s) => (s.role === 'defense'
    ? 'The AI plays Scorer and your goal is to hold its score down (fouls you commit count as its points).'
    : 'Score more than the other alliance.')),
  driver: opt('driver', 'Your robot driven by', [['human', 'You'], ['scorer', 'AI Scorer (watch)'], ['defense', 'AI Defense (watch)'], ['hybrid', 'AI Hybrid (watch)']],
    'Watch mode: an AI drives your robot in TELEOP (AUTO still runs your auto routine).'),
  driverSkill: opt('driverSkill', 'Its AI skill', SKILL_ORDER.map((k) => [k, OPP_SKILLS[k].name]), (s) => 'Only used in watch mode. ' + OPP_SKILLS[s.driverSkill].desc),
  hp: opt('hp', 'Your human player', [['manual', 'Manual (X / Y)'], ['auto', 'Auto-throw when HUB active']], 'The human player at your OUTPOST: throw FUEL yourself, or let it throw whenever your HUB is active.'),
  climber: opt('climber', 'Climber add-on', [['none', 'None (as built)'], ['l1', 'Level 1 hook'], ['l3', 'Level 1-3 climber']], 'None of these three robots climbed in 2026. Add a hypothetical climber to your robot to try the TOWER.'),
  camera: opt('camera', 'Starting camera', CAMERA_MODES.map((m) => [m, CAMERA_NAMES[m]]), 'Change it any time in a match with D-pad ◀ ▶ ([ / ]); right stick click (T) turns it around.'),
  preview: opt('preview', 'Shot preview line', [['on', 'On'], ['off', 'Off']], 'While you hold shoot, a line shows where the shot goes (green once it will score).'),
};

// 3v3 slot options (slot i: blue 1-3 then red 1-3)
const SLOT_DRIVER = { you: 'You', scorer: 'AI Scorer', defense: 'AI Defense', hybrid: 'AI Hybrid', empty: 'Empty' };
const SLOT_ROLE_DESC = {
  you: 'You drive this robot. Only one slot can be you; pick none to watch the AIs play.',
  scorer: OPP_STRATEGIES.scorer.desc,
  defense: 'Blocks the other ALLIANCE\'s best-loaded robot on its way to its HUB and pushes it while it shoots. Backs off before a 3 s PIN (G418).',
  hybrid: 'Shift-aware: defends while only the other ALLIANCE\'s HUB is active and scores whenever its own is.', empty: 'No robot in this slot.',
};
function slotOpts(i) {
  const g = (s) => s.slots[i];
  const so = (field, label, values, desc) => ({ label, values, get: (s) => g(s)[field], set: (s, v) => { g(s)[field] = v; }, desc });
  const list = [
    {
      label: 'Driven by', values: () => SLOT_DRIVERS.map((k) => [k, SLOT_DRIVER[k]]), get: (s) => g(s).driver,
      set: (s, v) => {
        if (v === 'you') for (const sl of s.slots) if (sl.driver === 'you') sl.driver = 'scorer';
        g(s).driver = v;
      },
      desc: (s) => SLOT_ROLE_DESC[g(s).driver],
    },
    so('robot', 'Robot', robotValues, () => 'Which robot fills this slot.'),
    so('auto', 'Auto routine', autoValues, (s) => autoDesc(g(s).auto)),
    {
      label: 'Starting position', values: () => START_ORDER.map((k) => [k, START_POSITIONS[k].name]), get: (s) => g(s).start, set: (s, v) => { g(s).start = v; },
      show: (s) => g(s).auto !== 'best' && !isCustom(g(s).auto),
      desc: () => 'If two robots on an ALLIANCE want the same spot, the later one moves to the nearest free one.',
    },
    { ...so('skill', 'AI skill', () => SKILL_ORDER.map((k) => [k, OPP_SKILLS[k].name]), (s) => OPP_SKILLS[g(s).skill].desc), show: (s) => g(s).driver !== 'you' },
  ];
  // an empty slot only shows who drives it
  return list.map((x) => ({ ...x, show: x.show || (() => true), when: (s) => g(s).driver !== 'empty' || x.label === 'Driven by' }));
}

const MODES = {
  practice: { name: 'Practice', icon: '◎', line: 'Just your robot on the field', desc: 'Drive, collect and shoot with the field to yourself: learn a robot, try autos, work on your cycles.' },
  '1v1': { name: '1 v 1', icon: '⚔', line: 'You against an AI robot', desc: 'You against one AI-driven robot on the other ALLIANCE. Pick its robot, strategy and skill; train it or drill defense under More options.' },
  '3v3': { name: '3 v 3', icon: '⬢', line: 'Full alliances', desc: 'Two full ALLIANCES of three. Choose each robot, its auto and its role; drive one of them or watch the AIs play.' },
};
const MODE_ORDER = ['practice', '1v1', '3v3'];

// Menu pages: lists of items. Focusable: tiles, cards, opt, slot, toggle, btn. Not: section.
const sec = (label) => ({ type: 'section', label });
const o = (id) => ({ type: 'opt', opt: OPT[id] });
const btn = (label, act, extra = {}) => ({ type: 'btn', label, act, ...extra });
const hideForBest = (it) => ({ ...it, hidden: (s) => s.auto === 'best' });
const PAGES = {
  home: () => [
    { type: 'tiles' },
    btn('AUTO EDITOR', 'editor', { half: true, secondary: true }), btn('AI TUNING', 'tuning', { half: true, secondary: true }),
    btn('CONTROLS', 'controls', { half: true, secondary: true }), btn('SETTINGS', 'settings', { half: true, secondary: true }),
  ],
  practice: () => [
    sec('Robot'), { type: 'cards' },
    sec('Match'), o('auto'), hideForBest(o('start')), o('preload'), o('alliance'), o('ds'),
    btn('START MATCH', 'start'), btn('BACK', 'home', { secondary: true }),
  ],
  '1v1': (ui) => [
    sec('Your robot'), { type: 'cards' }, o('auto'), hideForBest(o('start')), o('preload'), o('alliance'),
    sec('AI opponent'), o('oppRobot'), o('opponent'), o('oppSkill'),
    { type: 'toggle', label: ui.adv ? 'Fewer options ▴' : 'More options ▾', desc: 'Driver station, Training mode, the defense drill and watch mode.' },
    ...(ui.adv ? [o('ds'), o('mode'), o('role'), o('driver'), o('driverSkill')] : []),
    btn('START MATCH', 'start'), btn('BACK', 'home', { secondary: true }),
  ],
  '3v3': (ui, s) => {
    const items = [];
    for (let i = 0; i < 6; i++) {
      if (i === 0) items.push(sec('Blue alliance'));
      if (i === 3) items.push(sec('Red alliance'));
      items.push({ type: 'slot', i });
      if (ui.openSlot === i) for (const so of slotOpts(i)) if (so.when(s) && so.show(s)) items.push({ type: 'opt', opt: so, sub: true });
    }
    items.push(sec('Match'), o('preloadAll'));
    items.push(btn('START MATCH', 'start'), btn('BACK', 'home', { secondary: true }));
    return items;
  },
  settings: () => [
    sec('Your robot'), o('hp'), o('climber'),
    sec('View'), o('camera'), o('preview'),
    btn('BACK', 'home', { secondary: true }),
  ],
};
// who drives a robot in a match, for the HUD and results
function unitDriver(u) {
  if (u.human) return 'You';
  const name = `AI ${OPP_STRATEGIES[u.ai.strategy].name}`;
  return u.entry.you ? `${name} (watch)` : `${name} · ${OPP_SKILLS[u.ai.skillKey] ? OPP_SKILLS[u.ai.skillKey].name : ''}`;
}

const PAGE_TITLE = { home: '', practice: 'Practice', '1v1': '1 v 1 vs AI', '3v3': '3 v 3 Alliances', settings: 'Settings' };
const focusable = (it) => it.type !== 'section';

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
  ['Turn camera around 180°', 'Right stick click', 'T'],
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
    this.page = 'home';
    this.tile = MODES[settings.matchMode] ? settings.matchMode : '1v1';
    this.adv = false;
    this.openSlot = null;
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
    $('editor').classList.toggle('hidden', screen !== 'editor');
    $('tuning').classList.toggle('hidden', screen !== 'tuning');
    $('hud').classList.toggle('hidden', screen === 'menu' || screen === 'editor' || screen === 'tuning');
    if (screen === 'tuning') this.tuning.show();
    if (screen === 'menu') this.renderMenu();
    if (screen === 'editor') this.editor.show();
    if (screen === 'pause') this.renderPause();
  }

  // ------------------------------------------------------------------ main menu
  openPage(page) {
    this.page = page;
    this.focus = 0;
    this.openSlot = null;
    if (MODES[page]) { this.s.matchMode = page; saveSettings(this.s); }
    this.renderMenu();
  }

  _items() { return PAGES[this.page](this, this.s).filter((it) => !(it.hidden && it.hidden(this.s))); }

  _focusables() { return this._items().filter(focusable); }

  renderMenu() {
    const s = this.s;
    if (!this.page) this.page = 'home';
    const items = this._items();
    const fitems = items.filter(focusable);
    this.focus = Math.max(0, Math.min(this.focus, fitems.length - 1));
    const cur = fitems[this.focus];
    let fi = 0;
    const html = items.map((it) => {
      const idx = focusable(it) ? fi++ : -1;
      const f = idx === this.focus ? 'focus' : '';
      const d = `data-f="${idx}"`;
      switch (it.type) {
        case 'section': return `<div class="msec">${it.label}</div>`;
        case 'tiles': return `<div class="tiles ${f}" ${d}>${MODE_ORDER.map((k) => `<div class="tile ${this.tile === k ? 'sel' : ''}" data-mode="${k}"><div class="ticon">${MODES[k].icon}</div><div class="tname">${MODES[k].name}</div><div class="tline">${MODES[k].line}</div></div>`).join('')}</div>`;
        case 'cards': return `<div class="cards ${f}" ${d}>${this._robotCards()}</div>`;
        case 'opt': {
          const vals = it.opt.values(s);
          const v = vals.find((x) => x[0] === it.opt.get(s)) || vals[0];
          return `<div class="opt ${it.sub ? 'sub' : ''} ${f}" ${d}><span class="ol">${it.opt.label}</span><span class="ov"><span class="arr">◀</span>${v[1]}<span class="arr">▶</span></span></div>`;
        }
        case 'slot': return this._slotRow(it.i, f, d);
        case 'toggle': return `<div class="toggle ${f}" ${d}>${it.label}</div>`;
        case 'btn': return `<div class="btn ${it.secondary ? 'secondary' : ''} ${it.half ? 'half' : ''} ${f}" ${d}>${it.label}</div>`;
        default: return '';
      }
    }).join('');
    const title = PAGE_TITLE[this.page];
    $('menuTitle').innerHTML = title ? `<span class="crumb" data-back="1">◀ Menu</span> ${title}` : '';
    $('menuTitle').classList.toggle('hidden', !title);
    $('menuBrand').classList.toggle('hidden', !!title);
    $('options').innerHTML = html + `<div class="optdesc">${this._desc(cur)}</div>`;
    // mouse
    const back = document.querySelector('#menuTitle [data-back]');
    if (back) back.onclick = () => this.openPage('home');
    for (const el of document.querySelectorAll('#options [data-f]')) {
      el.onclick = (e) => {
        const i = +el.dataset.f;
        if (i < 0) return;
        this.focus = i;
        const it = this._focusables()[i];
        if (it.type === 'tiles') { const m = e.target.closest('[data-mode]'); if (m) { this.tile = m.dataset.mode; this.openPage(m.dataset.mode); } return; }
        if (it.type === 'cards') { const c = e.target.closest('[data-robot]'); if (c) { this.s.robot = c.dataset.robot; this._changed(); } return; }
        if (it.type === 'opt') { this._cycle(it, e.offsetX < el.clientWidth / 2 ? -1 : 1); return; }
        if (it.type === 'slot') {
          const arr = e.target.closest('[data-arr]');
          if (arr) { this._slotRobot(it.i, +arr.dataset.arr); return; }
        }
        this._activate(it);
      };
    }
    const fe = document.querySelector('#options .focus');
    if (fe) fe.scrollIntoView({ block: 'nearest' });
  }

  _robotCards() {
    const s = this.s;
    return ROBOT_ORDER.map((k) => {
      const r = ROBOTS[k];
      // capacity: what the modeled hopper holds (measured), not the team's stated number
      const stats = Object.entries({ Capacity: modelCapacity(r), ...r.stats }).map(([a, b]) => `<tr><td class="muted">${a}</td><td>${b}</td></tr>`).join('');
      return `<div class="rcard ${s.robot === k ? 'sel' : ''}" data-robot="${k}">
        <div class="num">${r.team}</div><div class="nm">${r.teamName} · ${r.robotName}</div>
        <div class="arch">${r.archetype}</div><div class="desc">${r.blurb}</div><table>${stats}</table></div>`;
    }).join('');
  }

  _slotRow(i, f, d) {
    const sl = this.s.slots[i], a = slotAlliance(i);
    const empty = sl.driver === 'empty';
    const auto = isCustom(sl.auto) ? '✎ ' + (getCustom(sl.auto) || {}).name : AUTO_ROUTINES[sl.auto].name;
    const tag = `<span class="stag ${sl.driver}">${SLOT_DRIVER[sl.driver]}</span>`;
    const body = empty ? '<span class="muted">No robot</span>'
      : `<span class="arr" data-arr="-1">◀</span><b>${robotName(sl.robot)}</b><span class="arr" data-arr="1">▶</span><span class="muted">· ${auto}${sl.driver !== 'you' ? ' · ' + OPP_SKILLS[sl.skill].name : ''}</span>`;
    return `<div class="slot ${a} ${this.openSlot === i ? 'open' : ''} ${f}" ${d}><span class="sn">${a === BLUE ? 'B' : 'R'}${(i % 3) + 1}</span>${tag}<span class="sbody">${body}</span><span class="sedit">${this.openSlot === i ? 'Done ▴' : 'Edit ▾'}</span></div>`;
  }

  _desc(it) {
    const s = this.s;
    if (!it) return '';
    switch (it.type) {
      case 'tiles': return `${MODES[this.tile].desc} <span class="muted">◀ ▶ choose · A open</span>`;
      case 'cards': return 'Choose your robot with ◀ ▶.';
      case 'opt': return it.opt.desc(s);
      case 'slot': {
        const who = s.slots.some((x) => x.driver === 'you') ? '' : ' Nobody is set to You, so you\'ll watch the AIs play.';
        return (this.openSlot === it.i ? 'Change this slot below; A or B when done.' : '◀ ▶ change the robot · A edit its driver, auto and role.') + who;
      }
      case 'toggle': return it.desc;
      case 'btn': return it.act === 'start' ? this._summary() : '';
      default: return '';
    }
  }

  // one line describing the match START will begin
  _summary() {
    const s = this.s;
    if (this.page === 'practice') return `${robotName(s.robot)} on the ${s.alliance.toUpperCase()} ALLIANCE, field to yourself.`;
    if (this.page === '1v1') return `${robotName(s.robot)} (${s.driver === 'human' ? 'you' : 'AI'}) vs ${robotName(s.oppRobot)} (${OPP_STRATEGIES[s.opponent].name}, ${OPP_SKILLS[s.oppSkill].name}).`;
    const side = (a) => s.slots.filter((_, i) => slotAlliance(i) === a && s.slots[i].driver !== 'empty').map((sl) => `${ROBOTS[sl.robot].team}${sl.driver === 'you' ? ' (you)' : ''}`).join(', ') || 'nobody';
    return `BLUE ${side(BLUE)} vs RED ${side(RED)}.`;
  }

  _cycle(it, dir) {
    const o = it.opt;
    const vals = o.values(this.s);
    let idx = vals.findIndex((v) => v[0] === o.get(this.s));
    idx = (idx + dir + vals.length) % vals.length;
    o.set(this.s, vals[idx][0]);
    this._changed();
  }

  _slotRobot(i, dir) {
    const sl = this.s.slots[i];
    if (sl.driver === 'empty') return;
    const k = ROBOT_ORDER.indexOf(sl.robot);
    sl.robot = ROBOT_ORDER[(k + dir + ROBOT_ORDER.length) % ROBOT_ORDER.length];
    this._changed();
  }

  _changed() {
    saveSettings(this.s);
    if (this.screen === 'menu') this.renderMenu();
  }

  _activate(it) {
    switch (it.type) {
      case 'tiles': this.openPage(this.tile); return;
      case 'cards': this.focus++; this.renderMenu(); return;
      case 'opt': this._cycle(it, 1); return;
      case 'slot': this.openSlot = this.openSlot === it.i ? null : it.i; this.renderMenu(); return;
      case 'toggle': this.adv = !this.adv; this.renderMenu(); return;
      case 'btn':
        if (it.act === 'start') this.h.onStart();
        else if (it.act === 'home' || it.act === 'settings') this.openPage(it.act);
        else if (it.act === 'editor') this.show('editor');
        else if (it.act === 'tuning') this.show('tuning');
        else if (it.act === 'controls') { this.prevScreen = 'menu'; this.show('controls'); }
        return;
      default:
    }
  }

  // B / Esc: close an open slot, else back to the home page
  _menuBack() {
    if (this.openSlot !== null && this.openSlot !== undefined) {
      const i = this.openSlot;
      this.openSlot = null;
      this.focus = this._focusables().findIndex((x) => x.type === 'slot' && x.i === i);
      this.renderMenu();
    } else if (this.page !== 'home') {
      const from = this.page;
      this.openPage('home');
      if (MODES[from]) this.tile = from;
      this.renderMenu();
    }
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
      <p class="muted" style="font-size:13px">Menus: D-pad / left stick to move, ◀ ▶ to change, A to select, B to go back. Shooting auto-targets your HUB when your BUMPERS are in your ALLIANCE ZONE; anywhere else RT lobs FUEL back into your ALLIANCE ZONE. The Auto Editor lists its own controls at the bottom of its screen.</p>`;
  }

  // Returns true when a menu overlay is open (gameplay input suppressed)
  handleInput(inp, dt) {
    const p = inp.pressed, n = inp.nav;
    if (this.screen === 'editor') {
      this.editor.handleInput(inp, dt);
      return true;
    }
    if (this.screen === 'tuning') {
      this.tuning.handleInput(inp);
      return true;
    }
    if (this.screen === 'menu') {
      const F = this._focusables();
      const it = F[Math.min(this.focus, F.length - 1)];
      if (n.up) { this.focus = (this.focus - 1 + F.length) % F.length; this.renderMenu(); }
      if (n.down) { this.focus = (this.focus + 1) % F.length; this.renderMenu(); }
      if ((n.left || n.right) && it) {
        const d = n.left ? -1 : 1;
        if (it.type === 'tiles') { const k = MODE_ORDER.indexOf(this.tile); this.tile = MODE_ORDER[(k + d + MODE_ORDER.length) % MODE_ORDER.length]; this.renderMenu(); }
        else if (it.type === 'cards') { const k = ROBOT_ORDER.indexOf(this.s.robot); this.s.robot = ROBOT_ORDER[(k + d + ROBOT_ORDER.length) % ROBOT_ORDER.length]; this._changed(); }
        else if (it.type === 'opt') this._cycle(it, d);
        else if (it.type === 'slot') this._slotRobot(it.i, d);
      }
      if (p.a && it) this._activate(it);
      else if (p.b) this._menuBack();
      else if (p.start) {
        // controller Menu: open the chosen mode / start the match; keyboard Esc: back
        if (inp.source === 'keyboard') this._menuBack();
        else if (this.page === 'home') this.openPage(this.tile);
        else if (MODES[this.page]) this.h.onStart();
      }
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
    const me = game.me;
    const col = (a) => {
      const s = m.score[a];
      return `<div class="rescol ${a}"><div class="big">${m.total(a)}</div><table>
        <tr><td>AUTO FUEL</td><td>${s.autoFuel}</td></tr>
        <tr><td>TELEOP FUEL</td><td>${s.teleFuel}</td></tr>
        <tr><td>TOWER</td><td>${s.autoTower + s.teleTower}</td></tr>
        <tr><td>FOUL points received</td><td>${s.foulPts}</td></tr></table></div>`;
    };
    const tb = m.total(BLUE), tr = m.total(RED);
    const opp = other(me);
    const defense = game.role === 'defense' && game.opp;
    let win = tb === tr ? 'TIE' : (tb > tr ? 'BLUE' : 'RED') + ' WINS';
    if (defense) {
      const avg = game.defenseTarget;
      const held = m.total(opp);
      win = `Your defense held ${opp.toUpperCase()} to ${held} points` + (avg !== null && avg !== undefined ? ` <span class="${held < avg ? 'good' : 'bad'}">(${held < avg ? 'beat' : 'missed'} its average of ${avg})</span>` : '');
    }
    const T = game.learnSummary;
    const learnHtml = T ? `<div class="ptitle small">AI training</div><div class="learnbox">${T.lines.map((l) => `<div>${l}</div>`).join('') || '<div>No change this match.</div>'}
      ${T.changes.length ? `<div class="chg">${T.changes.slice(0, 6).map((c) => `${BRAIN_LABEL(c.k)} ${fmtValue(c.k, c.from)} → <b>${fmtValue(c.k, c.to)}</b>`).join(' · ')}</div>` : ''}</div>` : '';
    const s = m.score[me];
    const you = game.you && game.you.human;
    // every robot's match
    const rows = [BLUE, RED].flatMap((a) => game.units.filter((u) => u.robot.alliance === a).map((u) => {
      const r = u.robot;
      return `<tr class="${a}"><td><span class="dot ${a}"></span>${a === BLUE ? 'B' : 'R'}${u.entry.station + 1}</td><td><b>${r.cfg.team}</b> ${r.cfg.archetype}</td><td>${unitDriver(u)}</td>
        <td>${r.stats.shots}</td><td>${r.stats.passes}</td><td>${r.stats.intaked}</td></tr>`;
    })).join('');
    const foulList = (a) => m.score[a].fouls.length
      ? m.score[a].fouls.map((f) => `${f.rule} ${f.type.toUpperCase()} (+${f.pts} to ${other(a).toUpperCase()}): ${f.desc}`).join('<br>') : 'None';
    $('resHeading').textContent = 'Match Results';
    $('resBody').innerHTML = `<div class="winner">${win}</div>${learnHtml}
      <div class="resgrid">${col(BLUE)}<div class="vs">VS</div>${col(RED)}</div>
      <div class="stats">
        <div class="stat"><div class="sv">${s.autoFuel + s.teleFuel}</div><div class="sk">${me.toUpperCase()} FUEL scored in an active HUB</div></div>
        <div class="stat"><div class="sv">${s.inactiveFuel}</div><div class="sk">${me.toUpperCase()} FUEL into an inactive HUB (0 pts)</div></div>
        <div class="stat"><div class="sv">${m.score[BLUE].fouls.length}</div><div class="sk">BLUE fouls</div></div>
        <div class="stat"><div class="sv">${m.score[RED].fouls.length}</div><div class="sk">RED fouls</div></div>
      </div>
      <table class="restable"><tr><th></th><th>Robot</th><th>Driven by</th><th>Launched</th><th>Passed</th><th>Intaked</th></tr>${rows}</table>
      <div class="ptitle small">${you ? 'Your' : me.toUpperCase()} ALLIANCE fouls (${m.score[me].fouls.length})</div>
      <div class="foullist">${m.score[me].fouls.length ? foulList(me) : 'None — clean match!'}</div>
      <div class="ptitle small">${other(me).toUpperCase()} ALLIANCE fouls (${m.score[other(me)].fouls.length})</div>
      <div class="foullist">${foulList(other(me))}</div>`;
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
    const me = game.me;
    const lead = game.units.find((u) => u.robot === r);
    const watching = !lead || !lead.human || !!game.driverAI;
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
    const ai = game.driverAI || (lead && lead.ai);
    $('rpDrive').textContent = watching && ai
      ? `AI ${OPP_STRATEGIES[ai.strategy].name} · ${ai.label}`
      : (game.fieldRelative ? 'Field-relative' : 'Robot-relative') + (game.slow ? ' · SLOW' : '') + (m.isAuto ? ' · AUTO' : '');
    // human player
    $('hpChute').textContent = game.fuel.chuteCount(me);
    $('hpCorral').textContent = hp.corralCount();
    $('hpDoor').textContent = hp.doorOpen ? 'OPEN' : 'Closed';
    $('hpMode').textContent = hp.auto ? 'Auto-throw' : 'Manual';
    const f = m.score[me].fouls;
    $('foulCount').textContent = `${f.length} (${f.reduce((a, b) => a + b.pts, 0)} pts to opp.)`;
    // opponent + PIN status
    const op = game.opp;
    // every other robot on the field: what it's doing and what it holds
    const others = game.units.filter((u) => u.robot !== r);
    $('oppPanel').classList.toggle('hidden', !others.length);
    if (others.length) {
      $('oppTitle').textContent = others.length === 1 ? 'Opponent' : 'Other robots';
      const of = m.score[other(me)].fouls;
      $('oppStrat').textContent = `${other(me).toUpperCase()} fouls: ${of.length} (${of.reduce((a, b) => a + b.pts, 0)} pts to ${me.toUpperCase()})`;
      $('oppList').innerHTML = others.map((u) => {
        const o = u.robot;
        return `<div class="orow"><span class="dot ${o.alliance}"></span><b>${o.cfg.team}</b><span class="od">${u.human ? 'You' : u.ai.label}</span><span class="of">${o.stored.length}/${o.capacity()}</span></div>`;
      }).join('');
    }
    // PINS against any opponent (yours first)
    let pinTxt = '';
    if (m.robotEnabled && r) {
      for (const o of game.robots.filter((x) => x.alliance !== r.alliance)) {
        const mine = game.rules.pinState(r, o), theirs = game.rules.pinState(o, r);
        if (mine.t > 0.5) {
          pinTxt = mine.active
            ? `<span class="bad">PINNING ${o.cfg.team} ${mine.t.toFixed(1)} s</span> — back off 72 in (${PIN_LIMIT} s limit, G418)`
            : `Pin count ${mine.t.toFixed(1)} s — resets once you are 72 in away`;
          break;
        } else if (theirs.t > 0.5 && theirs.active) pinTxt = `<span class="warn">PINNED by ${o.cfg.team} ${theirs.t.toFixed(1)} s</span> — foul on them at ${PIN_LIMIT * (theirs.fouls + 1)} s`;
      }
    }
    const tb = $('trainBadge');
    let badge = '';
    if (game.training) badge = `TRAINING #${game.training.number} · AI variation ${game.training.cand.info.sign > 0 ? 'A' : game.training.cand.info.sign < 0 ? 'B' : '—'}`;
    if (game.role === 'defense' && op) badge += `${badge ? ' · ' : ''}DEFENSE: hold ${other(me).toUpperCase()} ${game.defenseTarget !== null && game.defenseTarget !== undefined ? `under ${game.defenseTarget}` : 'down'} — now ${m.total(other(me))}`;
    tb.textContent = badge;
    tb.classList.toggle('hidden', !badge);
    const pw = $('pinWarn');
    pw.innerHTML = pinTxt;
    pw.classList.toggle('hidden', !pinTxt);
    // hint
    const pad = input.lastSource === 'gamepad';
    $('hint').innerHTML = watching
      ? `Watching the AI drive · ${pad ? '<b>D-pad ◀▶</b> Camera · <b>☰</b> Pause' : '<b>[ ]</b> Camera · <b>Esc</b> Pause'}`
      : pad
      ? '<b>RT</b> Shoot · <b>LT</b> Intake · <b>RB</b> Pass · <b>LB</b> Eject · <b>X</b> Chute door · <b>Y</b> HP throw · <b>A</b> Climb · <b>D-pad ◀▶</b> Camera · <b>☰</b> Pause'
      : '<b>WASD</b> Drive · <b>Q/E</b> Rotate · <b>Space</b> Shoot · <b>Shift</b> Intake · <b>R</b> Pass · <b>F</b> Eject · <b>G</b> Chute · <b>H</b> HP throw · <b>C</b> Climb · <b>[ ]</b> Camera · <b>Esc</b> Pause';
  }

  setPadStatus(name) {
    $('padStatus').textContent = name ? `🎮 Controller connected: ${name}` : 'Press any button on your Xbox controller to connect it (keyboard works too)';
  }
}
