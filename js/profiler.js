// Where the simulation's time goes, part by part: wraps the hot methods of the game's classes
// (on their prototypes, so it costs nothing until install() is called) with timers that keep
// inclusive and exclusive (self) time per label, and how many calls. Used by tools/perf3v3.mjs
// (headless) and by the Show FPS readout's breakdown (js/perf.js, js/main.js: ?prof in the URL).
// Wrapping changes no result: every wrapped call returns what it would have.
import { Robot } from './robot.js';
import { Physics } from './physics.js';
import { FuelManager } from './fuel.js';
import { RobotRules } from './rules.js';
import { Match } from './match.js';
import { HumanPlayer } from './humanPlayer.js';
import { OpponentAI } from './opponent.js';
import { AutoRunner } from './auto.js';
import { NavGrid } from './nav.js';
import { Localizer } from './localize.js';
import { Mechanism, Shaft, Train, Surface, Block } from './mech/mechanism.js';
import { PhysicalRobot } from './mech/physical.js';

const now = typeof performance !== 'undefined' ? () => performance.now() : () => Date.now();

export class Profiler {
  constructor() {
    this.stats = new Map(); // label -> { incl, self, n }
    this.stack = [];        // [label, start, childTime]
    this.robot = null;      // the robot whose mechanisms are stepping (labels by team)
    this.undo = [];
  }

  enter(label) { this.stack.push([label, now(), 0]); }
  exit() {
    const [label, t0, child] = this.stack.pop();
    const dt = now() - t0;
    let s = this.stats.get(label);
    if (!s) this.stats.set(label, (s = { incl: 0, self: 0, n: 0 }));
    s.incl += dt; s.self += dt - child; s.n++;
    if (this.stack.length) this.stack[this.stack.length - 1][2] += dt;
  }
  // time fn() under label (for code that isn't a method: rendering, the frame's loop)
  time(label, fn) { this.enter(label); try { return fn(); } finally { this.exit(); } }

  reset() { this.stats.clear(); }

  // wrap obj[name] (a prototype's method) so each call is timed under label(this, args) (a string,
  // or a function of the instance)
  wrap(obj, name, label) {
    const orig = obj[name];
    if (typeof orig !== 'function' || orig.__prof) return;
    const P = this;
    const f = function (...a) {
      P.enter(typeof label === 'function' ? label(this) : label);
      try { return orig.apply(this, a); } finally { P.exit(); }
    };
    f.__prof = true;
    obj[name] = f;
    this.undo.push(() => { obj[name] = orig; });
  }

  // the game's hot spots
  install() {
    const P = this;
    const team = () => (P.robot ? P.robot.cfg.key : '?');
    const mechKind = (m) => (P.robot && P.robot.phys ? (m === P.robot.phys.shooter ? 'shooter' : m === P.robot.phys.path ? 'path' : 'other') : 'other');
    this.wrap(Physics.prototype, 'step', 'physics.world.step');
    this.wrap(FuelManager.prototype, 'preStep', 'fuel.preStep');
    this.wrap(FuelManager.prototype, 'postStep', 'fuel.postStep');
    this.wrap(FuelManager.prototype, 'update', 'fuel.update(frame)');
    this.wrap(FuelManager.prototype, 'sync', 'fuel.sync(render)');
    this.wrap(RobotRules.prototype, 'update', 'rules');
    this.wrap(Match.prototype, 'update', 'match');
    this.wrap(HumanPlayer.prototype, 'update', 'humanPlayer(frame)');
    this.wrap(OpponentAI.prototype, 'update', 'ai');
    this.wrap(AutoRunner.prototype, 'update', 'auto');
    this.wrap(NavGrid.prototype, 'plan', 'nav.plan');
    this.wrap(NavGrid.prototype, 'los', 'nav.los');
    this.wrap(Localizer.prototype, 'step', 'loc');
    this.wrap(Localizer.prototype, '_frame', 'loc.camFrame');
    this.wrap(Localizer.prototype, '_hidden', 'loc.occlusion');
    // the robot's own steps, with the robot noted for the mechanisms' labels
    for (const name of ['preStep', 'postStep']) {
      const orig = Robot.prototype[name];
      if (orig.__prof) continue;
      const f = function (...a) {
        const prev = P.robot;
        P.robot = this;
        if (this.phys) P._wrapPhys(this.phys);
        P.enter('robot.' + name);
        try { return orig.apply(this, a); } finally { P.exit(); P.robot = prev; }
      };
      f.__prof = true;
      Robot.prototype[name] = f;
      this.undo.push(() => { Robot.prototype[name] = orig; });
    }
    this.wrap(Robot.prototype, '_wheelsDown', 'robot.wheelRays');
    this.wrap(Robot.prototype, '_shooter', 'robot.shooter');
    this.wrap(Robot.prototype, '_stepHopper', 'robot.hopper');
    this.wrap(Robot.prototype, '_guideCaptured', 'robot.guideCaptured');
    this.wrap(Robot.prototype, '_intakePhys', 'robot.intakePhys');
    this.wrap(Robot.prototype, 'update', 'robot.update(visual)');
    this.wrap(PhysicalRobot.prototype, 'stepMechanisms', () => `mech ${team()}`);
    this.wrap(PhysicalRobot.prototype, 'account', () => `mech ${team()} account`);
    this.wrap(Mechanism.prototype, 'step', (m) => `mech ${team()} ${mechKind(m)}`);
    this.wrap(Mechanism.prototype, '_contacts', (m) => `mech ${team()} ${mechKind(m)} contacts`);
    this.wrap(Mechanism.prototype, '_solve', (m) => `mech ${team()} ${mechKind(m)} solve`);
    this.wrap(Shaft.prototype, 'integrate', () => `mech ${team()} motors`);
    this.wrap(Train.prototype, 'integrate', () => `mech ${team()} motors`);
    return this;
  }

  // each physical robot's class (js/mech/phys*.js), the first time one steps
  _wrapPhys(phys) {
    const proto = Object.getPrototypeOf(phys);
    if (proto.__profWrapped) return;
    proto.__profWrapped = true;
    const team = () => (this.robot ? this.robot.cfg.key : '?');
    for (const name of ['preStep', 'postStep', 'watchFull', '_squash', '_mouthPressed', '_packed']) {
      if (Object.prototype.hasOwnProperty.call(proto, name)) this.wrap(proto, name, () => `mech ${team()} ${name}`);
    }
    this.undo.push(() => { delete proto.__profWrapped; });
  }

  uninstall() { while (this.undo.length) this.undo.pop()(); }

  // rows sorted by self time: { label, incl, self, n } (ms totals)
  rows() { return [...this.stats.entries()].map(([label, s]) => ({ label, ...s })).sort((a, b) => b.self - a.self); }
}

// (Surface and Block are exported for tools that want to wrap touches(): many calls, so the timer's
// own cost would swamp them)
export { Surface, Block };
