// Match flow and scoring per the 2026 Game Manual, Section 6 (Game Details).
//   AUTO 0:20 -> TELEOP 2:20 = TRANSITION SHIFT (10s) + SHIFT 1-4 (25s each) + END GAME (30s)
//   Both HUBS are active in AUTO, the TRANSITION SHIFT and END GAME. In SHIFTS 1-4 only one
//   HUB is active: the ALLIANCE that scored more FUEL in AUTO has its HUB inactive in SHIFT 1
//   (tie -> FMS picks randomly), then status alternates every SHIFT.
//   FUEL is assessed for up to 3 seconds after a HUB deactivates.
import { TIMING, POINTS, BLUE, RED, other, HUB } from './constants.js';
import { fmtClock } from './util.js';

const SHIFT_NAMES = ['TRANSITION SHIFT', 'SHIFT 1', 'SHIFT 2', 'SHIFT 3', 'SHIFT 4', 'END GAME'];

function emptyScore() {
  return { autoFuel: 0, teleFuel: 0, inactiveFuel: 0, autoTower: 0, teleTower: 0, foulPts: 0, fouls: [] };
}

export class Match {
  constructor({ playerAlliance = BLUE, onEvent = () => {} }) {
    this.player = playerAlliance;
    this.onEvent = onEvent;
    this.reset();
  }

  reset() {
    this.phase = 'pre';
    this.phaseTime = 0;
    this.t = 0; // simulation time since reset
    this.firstInactive = null;
    this.tiebreakRandom = false;
    this.score = { blue: emptyScore(), red: emptyScore() };
    this.lastDeactivate = { blue: -1e9, red: -1e9 };
    this.prevActive = { blue: false, red: false };
    this.lastShift = -1;
    this.foulLog = [];
    this.recentFoul = {};
    this.autoTowerDone = false;
  }

  // ------------------------------------------------------------------ timeline
  get robotEnabled() {
    return this.phase === 'auto' || this.phase === 'teleop';
  }
  get isAuto() { return this.phase === 'auto'; }
  get isTeleop() { return this.phase === 'teleop'; }
  get over() { return this.phase === 'done'; }

  teleopElapsed() { return this.phase === 'teleop' ? this.phaseTime : this.phase === 'post' || this.phase === 'done' ? TIMING.teleop : 0; }

  shiftIndex() {
    if (this.phase !== 'teleop') return -1;
    const tt = this.phaseTime;
    if (tt < TIMING.transition) return 0;
    if (tt >= TIMING.teleop - TIMING.endgame) return 5;
    return 1 + Math.floor((tt - TIMING.transition) / TIMING.shift);
  }

  shiftName() {
    const i = this.shiftIndex();
    return i >= 0 ? SHIFT_NAMES[i] : '';
  }

  hubActive(a) {
    switch (this.phase) {
      case 'auto':
        return true;
      case 'teleop': {
        const s = this.shiftIndex();
        if (s === 0 || s === 5) return true;
        const odd = s % 2 === 1; // SHIFT 1, 3
        return odd ? a !== this.firstInactive : a === this.firstInactive;
      }
      default:
        return false;
    }
  }

  // seconds until this hub's active status changes (null if it won't change again)
  hubNextChange(a) {
    if (this.phase === 'auto') return null;
    if (this.phase !== 'teleop') return null;
    const tt = this.phaseTime;
    const now = this.hubActive(a);
    const bounds = [TIMING.transition];
    for (let k = 1; k <= 4; k++) bounds.push(TIMING.transition + k * TIMING.shift);
    for (const b of bounds) {
      if (b <= tt + 1e-6) continue;
      // status at time b
      const save = this.phaseTime;
      this.phaseTime = b + 0.001;
      const st = this.hubActive(a);
      this.phaseTime = save;
      if (st !== now) return b - tt;
    }
    return now ? TIMING.teleop - tt : null;
  }

  // Light state per Table 5-3
  hubLightMode(a) {
    if (this.phase === 'pre') return 'off';
    if (this.phase === 'post') return 'white';
    if (this.phase === 'done') return 'green';
    if (this.phase === 'autoGap') return 'off';
    if (!this.hubActive(a)) return 'off';
    if (this.phase === 'teleop' && this.shiftIndex() === 0 && a === this.firstInactive) return 'chase';
    const nc = this.phase === 'auto' ? TIMING.auto - this.phaseTime : this.hubNextChange(a);
    if (nc !== null && nc <= 3) return 'warning';
    return 'active';
  }

  clockText() {
    switch (this.phase) {
      case 'pre': return fmtClock(TIMING.auto);
      case 'auto': return fmtClock(TIMING.auto - this.phaseTime);
      case 'autoGap': return fmtClock(TIMING.teleop);
      case 'teleop': return fmtClock(TIMING.teleop - this.phaseTime);
      default: return '0:00';
    }
  }

  phaseLabel() {
    switch (this.phase) {
      case 'pre': return 'GET READY';
      case 'auto': return 'AUTO';
      case 'autoGap': return 'AUTO COMPLETE';
      case 'teleop': return this.shiftName();
      case 'post': return 'SCORING…';
      case 'done': return 'MATCH OVER';
      default: return '';
    }
  }

  update(dt, robot) {
    this.t += dt;
    this.phaseTime += dt;
    const P = this.phase;
    if (P === 'pre' && this.phaseTime >= TIMING.preMatch) this._go('auto');
    else if (P === 'auto' && this.phaseTime >= TIMING.auto) {
      this._assessAutoTower(robot);
      this._go('autoGap');
    } else if (P === 'autoGap' && this.phaseTime >= TIMING.autoGap) {
      this._decideShifts();
      this._go('teleop');
    } else if (P === 'teleop' && this.phaseTime >= TIMING.teleop) {
      this._assessTeleopTower(robot);
      this._go('post');
    } else if (P === 'post' && this.phaseTime >= TIMING.post) {
      this._go('done');
    }
    // deactivation bookkeeping (for the 3s assessment window) + shift announcements
    for (const a of [BLUE, RED]) {
      const act = this.hubActive(a);
      if (this.prevActive[a] && !act) this.lastDeactivate[a] = this.t;
      this.prevActive[a] = act;
    }
    if (this.phase === 'teleop') {
      const s = this.shiftIndex();
      if (s !== this.lastShift) {
        this.lastShift = s;
        this.onEvent('shift', { name: SHIFT_NAMES[s], active: this.hubActive(this.player), index: s });
      }
    }
  }

  _go(phase) {
    this.phase = phase;
    this.phaseTime = 0;
    this.onEvent('phase', { phase });
  }

  _decideShifts() {
    const b = this.score.blue.autoFuel, r = this.score.red.autoFuel;
    if (b > r) this.firstInactive = BLUE;
    else if (r > b) this.firstInactive = RED;
    else {
      this.firstInactive = Math.random() < 0.5 ? BLUE : RED;
      this.tiebreakRandom = true;
    }
    this.onEvent('gamedata', { firstInactive: this.firstInactive, random: this.tiebreakRandom });
  }

  // ------------------------------------------------------------------ scoring
  fuelEnteredHub(a, ball, t) {
    const s = this.score[a];
    // G407: a ROBOT may only launch FUEL into its HUB with its BUMPERS in its ALLIANCE ZONE
    if (ball.launch && ball.launch.by === 'robot' && ball.launch.alliance === a && !ball.launch.legal) {
      this.addFoul(a, 'major', 'G407', 'Launched FUEL into the HUB from outside the ALLIANCE ZONE');
    }
    const active = this.hubActive(a);
    const grace = !active && this.t - this.lastDeactivate[a] <= HUB.scoreGrace;
    const counts = active || grace;
    const autoPeriod = this.phase === 'auto' || (this.phase === 'autoGap' && grace);
    if (this.phase === 'pre' || this.phase === 'done') return;
    if (counts) {
      if (autoPeriod) s.autoFuel += POINTS.fuel;
      else s.teleFuel += POINTS.fuel;
      this.onEvent('score', { alliance: a, pts: POINTS.fuel });
    } else {
      s.inactiveFuel++;
      this.onEvent('inactive', { alliance: a });
    }
  }

  addFoul(committedBy, type, rule, desc) {
    if (this.phase === 'pre' || this.phase === 'done') return;
    const pts = type === 'major' ? POINTS.majorFoul : POINTS.minorFoul;
    const opp = other(committedBy);
    this.score[opp].foulPts += pts;
    const f = { t: this.t, committedBy, type, rule, desc, pts };
    this.score[committedBy].fouls.push(f);
    this.foulLog.push(f);
    this.onEvent('foul', f);
  }

  _assessAutoTower(robot) {
    if (!robot) return;
    if (robot.climbLevel >= 1) {
      this.score[robot.alliance].autoTower += POINTS.towerAutoL1;
      this.onEvent('tower', { alliance: robot.alliance, pts: POINTS.towerAutoL1, level: 1, auto: true });
    }
  }

  _assessTeleopTower(robot) {
    if (!robot) return;
    const lvl = robot.climbLevel;
    if (lvl >= 1) {
      this.score[robot.alliance].teleTower += POINTS.tower[lvl];
      this.onEvent('tower', { alliance: robot.alliance, pts: POINTS.tower[lvl], level: lvl, auto: false });
    }
  }

  fuelPoints(a) { const s = this.score[a]; return s.autoFuel + s.teleFuel; }
  towerPoints(a) { const s = this.score[a]; return s.autoTower + s.teleTower; }
  total(a) { const s = this.score[a]; return s.autoFuel + s.teleFuel + s.autoTower + s.teleTower + s.foulPts; }
}
