// One match: robots, human players, autos, AIs and rules, independent of rendering and input.
// Used by the browser game (main.js) and by the headless self-play trainer (tools/train.mjs).
//
// Any number of ROBOTS per ALLIANCE. settings.matchMode picks how they're set up:
//   'practice' - just your robot
//   '1v1'      - you (or an AI in watch mode) against one AI opponent (settings.opponent...)
//   '3v3'      - settings.slots: six { robot, auto, start, driver, skill }, blue 1-3 then red 1-3;
//                driver is 'you', an AI strategy ('scorer', 'defense', 'hybrid') or 'empty'
// Without a matchMode (the headless tools) it's '1v1', with no opponent if opponent is 'off'.
import { Robot } from './robot.js';
import { Match } from './match.js';
import { HumanPlayer } from './humanPlayer.js';
import { AutoRunner, startPose, customSelection, assignStarts } from './auto.js';
import { OpponentAI, opponentAuto } from './opponent.js';
import { RobotRules } from './rules.js';
import { ROBOTS, CLIMBER_OPTIONS } from './robotConfigs.js';
import { BLUE, RED, other } from './constants.js';

const IDLE_CMD = { vx: 0, vz: 0, omega: 0, intake: false, shoot: false, pass: false, outtake: false };

export const SLOT_DRIVERS = ['you', 'scorer', 'defense', 'hybrid', 'empty'];
export const DEFAULT_SLOTS = [
  { robot: '2910', auto: 'best', start: 'leftTrench', driver: 'you', skill: 'champs' },
  { robot: '4414', auto: 'best', start: 'hub', driver: 'scorer', skill: 'champs' },
  { robot: '8793', auto: 'defendStage', start: 'rightBump', driver: 'defense', skill: 'champs' },
  { robot: '4414', auto: 'best', start: 'leftTrench', driver: 'scorer', skill: 'champs' },
  { robot: '2910', auto: 'best', start: 'hub', driver: 'scorer', skill: 'champs' },
  { robot: '8793', auto: 'defendStage', start: 'rightBump', driver: 'defense', skill: 'champs' },
];
export const slotAlliance = (i) => (i < 3 ? BLUE : RED);

// The robots in the match: { alliance, station (0-2), robot, auto, start, customSide, driver
// ('human' or an AI strategy), skill, brain, plan, preload, you }
export function matchEntries(s) {
  const preload = s.preload ?? 8;
  if (s.matchMode === '3v3') {
    return (s.slots || DEFAULT_SLOTS).map((sl, i) => ({ sl, i })).filter(({ sl }) => sl.driver !== 'empty').map(({ sl, i }) => ({
      alliance: slotAlliance(i), station: i % 3, robot: sl.robot, auto: sl.auto, start: sl.start, customSide: 'drawn',
      driver: sl.driver === 'you' ? 'human' : sl.driver, skill: sl.skill, brain: sl.skill === 'mine' ? s.mineBrain : null,
      preload, you: sl.driver === 'you',
    }));
  }
  const out = [{
    alliance: s.alliance, station: s.ds ?? 1, robot: s.robot, auto: s.auto, start: s.start, customSide: s.customSide,
    driver: s.driver || 'human', skill: s.driverSkill, brain: s.driverBrain, plan: s.autoPlan, preload, you: true,
  }];
  if (s.matchMode !== 'practice' && s.opponent && s.opponent !== 'off') {
    const oa = opponentAuto(s.opponent);
    out.push({
      alliance: other(s.alliance), station: 1, robot: s.oppRobot || '4414', auto: oa.routine, start: oa.start,
      driver: s.opponent, skill: s.oppSkill, brain: s.oppBrain, plan: s.oppAutoPlan, preload: 8, you: false,
    });
  }
  return out;
}

// world: { physics, scene, field, fuel }. settings: the menu settings (see ui.js DEFAULT_SETTINGS),
// plus optional driverBrain / oppBrain overrides for training.
export function createGame(world, settings, { onEvent = () => {}, prev = null } = {}) {
  const { physics, scene, field, fuel } = world;
  if (prev) for (const r of prev.robots) r.destroy();
  const entries = matchEntries(settings);
  const youEntry = entries.find((e) => e.you) || null;
  const me = youEntry ? youEntry.alliance : entries[0] ? entries[0].alliance : BLUE; // the side the camera and HUD take
  const match = new Match({ playerAlliance: me, onEvent });
  fuel.match = match;
  const rules = new RobotRules(match);
  // starting spots, alliance by alliance, in driver station order
  for (const a of [BLUE, RED]) {
    const list = entries.filter((e) => e.alliance === a).sort((x, y) => x.station - y.station);
    assignStarts(list).forEach((st, k) => Object.assign(list[k], { startKey: st.start, plan: st.plan }));
  }
  const pre = fuel.stage(entries.reduce((n, e) => n + e.preload, 0));
  let k = 0;
  const units = entries.map((e) => {
    const robot = new Robot({ cfg: ROBOTS[e.robot] || ROBOTS['2910'], alliance: e.alliance, physics, scene, field, fuel, match, climber: e.you ? CLIMBER_OPTIONS[settings.climber] || null : null });
    const custom = customSelection(e.auto, e.customSide);
    const pose = startPose(e.startKey || e.start, robot, e.alliance, custom);
    robot.spawn(pose.x, pose.z, pose.yaw);
    robot.loadFuel(pre.slice(k, k + e.preload));
    k += e.preload;
    const auto = new AutoRunner(robot, e.auto, e.startKey || e.start, e.alliance, custom, e.plan);
    return { robot, auto, ai: null, entry: e, human: e.driver === 'human' };
  });
  const robots = units.map((u) => u.robot);
  for (const u of units) {
    if (u.human) continue;
    const a = u.robot.alliance;
    u.ai = new OpponentAI({
      robot: u.robot, foes: robots.filter((r) => r.alliance !== a), mates: robots.filter((r) => r.alliance === a && r !== u.robot),
      match, fuel, rules, strategy: u.entry.driver, skill: u.entry.skill, brain: u.entry.brain,
    });
  }
  const you = units.find((u) => u.entry.you) || null;
  const lead = you || units.find((u) => u.robot.alliance === me) || units[0];
  // a human player per ALLIANCE: yours throws when you tell it to (unless set to auto-throw
  // or nobody on your ALLIANCE is driving)
  const hps = {};
  for (const a of [BLUE, RED]) {
    hps[a] = new HumanPlayer({ alliance: a, field, fuel, match });
    hps[a].auto = !(you && you.human && a === me && settings.hp !== 'auto');
  }
  const foeUnit = units.find((u) => u.robot.alliance !== me) || null;
  return {
    match, units, robots, rules, fuel, settings, me, you,
    robot: lead ? lead.robot : null, auto: lead ? lead.auto : null, hp: hps[me], hps,
    // your robot driven by an AI (watch mode); 1v1 compatibility: the first opponent
    driverAI: you && !you.human ? you.ai : null,
    opp: foeUnit ? { robot: foeUnit.robot, hp: hps[other(me)], auto: foeUnit.auto, ai: foeUnit.ai } : null,
    fieldRelative: true, slow: false, driver: null, hpControls: {},
  };
}

// One fixed physics step
export function stepGame(game, world, dt) {
  const { match, robots, units } = game;
  const prevPhase = match.phase;
  match.update(dt, robots);
  for (const r of robots) {
    if (prevPhase === 'pre' && match.phase === 'auto' && r.cfg.intake.latched) r.forceDeploy = true;
    if (r.forceDeploy && r.intakeDeploy >= 1) r.forceDeploy = false;
    r.enabled = match.robotEnabled;
  }
  for (const u of units) {
    if (match.isAuto) {
      u.auto.update(dt);
      if (u.ai) u.ai.label = 'AUTO routine';
    } else if (u === game.you && game.driverAI && u.human) {
      // a test hook put an AI in the driver's seat
      if (match.isTeleop) game.driverAI.update(dt); else Object.assign(u.robot.cmd, IDLE_CMD);
    } else if (u.ai) u.ai.update(dt);
    else if (match.isTeleop && game.driver) Object.assign(u.robot.cmd, game.driver);
    else Object.assign(u.robot.cmd, IDLE_CMD);
  }
  for (const r of robots) r.preStep(dt);
  world.fuel.preStep(dt);
  world.physics.step();
  world.fuel.postStep(dt, match.t);
  for (const r of robots) r.postStep(dt, match.t);
  game.rules.update(dt, robots);
}

// Per-frame (not per physics step) updates: HUB processing, FUEL returns, human players
export function frameGame(game, dt) {
  const t = game.match.t;
  game.fuel.update(dt, t);
  for (const a of [BLUE, RED]) game.hps[a].update(dt, t, a === game.me ? game.hpControls : {});
}
