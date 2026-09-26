// One match: robots, human players, autos, AIs and rules, independent of rendering and input.
// Used by the browser game (main.js) and by the headless self-play trainer (tools/train.mjs).
import { Robot } from './robot.js';
import { Match } from './match.js';
import { HumanPlayer } from './humanPlayer.js';
import { AutoRunner, startPose, customSelection, BEST_AUTOS } from './auto.js';
import { OpponentAI, opponentAuto } from './opponent.js';
import { RobotRules } from './rules.js';
import { ROBOTS, CLIMBER_OPTIONS } from './robotConfigs.js';
import { other } from './constants.js';

const IDLE_CMD = { vx: 0, vz: 0, omega: 0, intake: false, shoot: false, pass: false, outtake: false };

// world: { physics, scene, field, fuel }. settings: the menu settings (see ui.js DEFAULT_SETTINGS),
// plus optional driverBrain / oppBrain overrides for training.
export function createGame(world, settings, { onEvent = () => {}, prev = null } = {}) {
  const { physics, scene, field, fuel } = world;
  if (prev) for (const r of prev.robots) r.destroy();
  const alliance = settings.alliance;
  const match = new Match({ playerAlliance: alliance, onEvent });
  fuel.match = match;
  const robot = new Robot({ cfg: ROBOTS[settings.robot] || ROBOTS['2910'], alliance, physics, scene, field, fuel, match, climber: CLIMBER_OPTIONS[settings.climber] || null });
  const custom = customSelection(settings.auto, settings.customSide);
  // 'best' brings its own start position (settings.autoPlan overrides the plan, for searching)
  const plan = settings.auto === 'best' ? settings.autoPlan || BEST_AUTOS[robot.cfg.key] : null;
  const startKey = plan ? plan.start : settings.start;
  const pose = startPose(startKey, robot, alliance, custom);
  robot.spawn(pose.x, pose.z, pose.yaw);
  const oppOn = settings.opponent && settings.opponent !== 'off';
  const pre = fuel.stage(settings.preload + (oppOn ? 8 : 0));
  robot.loadFuel(pre.slice(0, settings.preload));
  const aiDriven = settings.driver && settings.driver !== 'human';
  const hp = new HumanPlayer({ alliance, field, fuel, match });
  hp.auto = settings.hp === 'auto' || aiDriven;
  const auto = new AutoRunner(robot, settings.auto, startKey, alliance, custom, plan);
  const rules = new RobotRules(match);
  const robots = [robot];
  let opp = null;
  if (oppOn) {
    const oa = other(alliance);
    const orobot = new Robot({ cfg: ROBOTS[settings.oppRobot] || ROBOTS['4414'], alliance: oa, physics, scene, field, fuel, match, climber: null });
    const plan = opponentAuto(settings.opponent);
    const oplan = plan.routine === 'best' ? settings.oppAutoPlan || BEST_AUTOS[orobot.cfg.key] : null;
    if (oplan) plan.start = oplan.start;
    const op = startPose(plan.start, orobot, oa);
    orobot.spawn(op.x, op.z, op.yaw);
    orobot.loadFuel(pre.slice(settings.preload));
    const ohp = new HumanPlayer({ alliance: oa, field, fuel, match });
    ohp.auto = true;
    opp = {
      robot: orobot,
      hp: ohp,
      auto: new AutoRunner(orobot, plan.routine, plan.start, oa, null, oplan),
      ai: new OpponentAI({ robot: orobot, foe: robot, match, fuel, rules, strategy: settings.opponent, skill: settings.oppSkill, brain: settings.oppBrain }),
    };
    robots.push(orobot);
  }
  // watch mode / training: an AI drives the player's robot in TELEOP
  const driverAI = aiDriven
    ? new OpponentAI({ robot, foe: opp ? opp.robot : null, match, fuel, rules, strategy: settings.driver, skill: settings.driverSkill, brain: settings.driverBrain })
    : null;
  return {
    match, robot, robots, opp, rules, hp, auto, fuel, settings, driverAI,
    fieldRelative: true, slow: false, driver: null, hpControls: {},
  };
}

// One fixed physics step
export function stepGame(game, world, dt) {
  const { match, robot, robots, auto, opp } = game;
  const prevPhase = match.phase;
  match.update(dt, robots);
  for (const r of robots) {
    if (prevPhase === 'pre' && match.phase === 'auto' && r.cfg.intake.latched) r.forceDeploy = true;
    if (r.forceDeploy && r.intakeDeploy >= 1) r.forceDeploy = false;
    r.enabled = match.robotEnabled;
  }
  if (match.isAuto) auto.update(dt);
  else if (match.isTeleop && game.driverAI) game.driverAI.update(dt);
  else if (match.isTeleop && game.driver) Object.assign(robot.cmd, game.driver);
  else Object.assign(robot.cmd, IDLE_CMD);
  if (opp) {
    if (match.isAuto) { opp.auto.update(dt); opp.ai.label = 'AUTO routine'; }
    else opp.ai.update(dt);
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
  game.hp.update(dt, t, game.hpControls);
  if (game.opp) game.opp.hp.update(dt, t, {});
}
