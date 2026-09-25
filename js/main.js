import * as THREE from 'three';
import { Physics } from './physics.js';
import { Field } from './field.js';
import { FuelManager } from './fuel.js';
import { Robot } from './robot.js';
import { Match } from './match.js';
import { HumanPlayer } from './humanPlayer.js';
import { AutoRunner, startPose, customSelection } from './auto.js';
import { OpponentAI, opponentAuto } from './opponent.js';
import { RobotRules } from './rules.js';
import { AutoEditor } from './editor.js';
import { Input } from './input.js';
import { CameraRig, CAMERA_NAMES } from './cameras.js';
import { UI, loadSettings } from './ui.js';
import { ROBOTS, CLIMBER_OPTIONS } from './robotConfigs.js';
import { BLUE, RED, PHYSICS_DT, other } from './constants.js';

const $ = (id) => document.getElementById(id);

// ------------------------------------------------------------------ renderer / scene
const canvas = $('view');
const renderer = new THREE.WebGLRenderer({ canvas, antialias: true, powerPreference: 'high-performance' });
renderer.setPixelRatio(Math.min(window.devicePixelRatio, 2));
renderer.setSize(window.innerWidth, window.innerHeight);
renderer.shadowMap.enabled = true;
renderer.shadowMap.type = THREE.PCFSoftShadowMap;
renderer.toneMapping = THREE.ACESFilmicToneMapping;
renderer.toneMappingExposure = 1.05;
renderer.outputColorSpace = THREE.SRGBColorSpace;

const scene = new THREE.Scene();
scene.background = new THREE.Color(0x0b0d12);
scene.fog = new THREE.Fog(0x0b0d12, 28, 70);
const camera = new THREE.PerspectiveCamera(60, window.innerWidth / window.innerHeight, 0.05, 200);

scene.add(new THREE.HemisphereLight(0xdfe8ff, 0x30333a, 1.1));
const sun = new THREE.DirectionalLight(0xffffff, 2.1);
sun.position.set(-3, 18, 7);
sun.castShadow = true;
sun.shadow.mapSize.set(4096, 2048);
Object.assign(sun.shadow.camera, { left: -11, right: 11, top: 7, bottom: -7, near: 1, far: 40 });
sun.shadow.bias = -0.0004;
sun.shadow.normalBias = 0.02;
scene.add(sun);
const fill = new THREE.DirectionalLight(0xb9c8ff, 0.5);
fill.position.set(6, 10, -8);
scene.add(fill);

window.addEventListener('resize', () => {
  renderer.setSize(window.innerWidth, window.innerHeight);
  camera.aspect = window.innerWidth / window.innerHeight;
  camera.updateProjectionMatrix();
});

// ------------------------------------------------------------------ world
const physics = await Physics.create();
const field = new Field(scene, physics);
field.build();
const fuel = new FuelManager(physics, scene, field);
fuel.stage(8);
const input = new Input();
const rig = new CameraRig(camera);
const settings = loadSettings();

// shot preview line
const previewGeo = new THREE.BufferGeometry();
previewGeo.setAttribute('position', new THREE.Float32BufferAttribute(new Float32Array(3 * 400), 3));
const previewMat = new THREE.LineDashedMaterial({ color: 0xffe066, dashSize: 0.12, gapSize: 0.08, transparent: true, opacity: 0.9 });
const previewLine = new THREE.Line(previewGeo, previewMat);
previewLine.frustumCulled = false;
previewLine.visible = false;
scene.add(previewLine);

let game = null;
let simAcc = 0;
let resultsShown = false;
let backHeld = 0;
let lastInactiveToast = -10;

const ui = new UI(settings, {
  onStart: () => startMatch(),
  onResume: () => ui.show('hud'),
  onRestart: () => startMatch(),
  onMenu: () => ui.show('menu'),
});
ui.editor = new AutoEditor({
  settings, field,
  onClose: () => ui.show('menu'),
  onTest: (key) => { settings.auto = key; ui._changed(); startMatch(); },
});
ui.show('menu');
$('loading').classList.add('hidden');

function onEvent(type, d) {
  if (!game) return;
  const me = settings.alliance;
  switch (type) {
    case 'phase':
      if (d.phase === 'auto') ui.bigMessage('AUTO', 1.2);
      if (d.phase === 'autoGap') ui.toast('AUTO complete — scoring assessed', 'info');
      if (d.phase === 'teleop') ui.bigMessage('TELEOP', 1.2);
      if (d.phase === 'post') ui.bigMessage('MATCH OVER', 2);
      break;
    case 'gamedata': {
      const who = d.firstInactive === me ? 'Your' : 'Opponent';
      ui.toast(`FMS: ${who} HUB is inactive first${d.random ? ' (AUTO tie — random)' : ' (scored more FUEL in AUTO)'}`, 'info');
      break;
    }
    case 'shift':
      if (d.index > 0) {
        const act = game.match.hubActive(me);
        ui.bigMessage(`${d.name}`, 1.1);
        ui.toast(`${d.name}: your HUB is ${act ? 'ACTIVE' : 'INACTIVE'}`, act ? 'good' : 'foul');
      }
      break;
    case 'foul':
      if (d.committedBy === me) ui.toast(`${d.type === 'major' ? 'MAJOR' : 'MINOR'} FOUL ${d.rule}: ${d.desc} (+${d.pts} to ${other(me).toUpperCase()})`, 'foul');
      else ui.toast(`Opponent ${d.type === 'major' ? 'MAJOR' : 'MINOR'} FOUL ${d.rule}: ${d.desc} (+${d.pts} to you)`, 'good');
      break;
    case 'tower':
      ui.toast(`${d.alliance === me ? '' : 'Opponent '}TOWER LEVEL ${d.level}${d.auto ? ' in AUTO' : ''}: +${d.pts}`, d.alliance === me ? 'good' : 'info');
      break;
    case 'inactive':
      if (d.alliance === me && game.match.t - lastInactiveToast > 2.5) {
        lastInactiveToast = game.match.t;
        ui.toast('FUEL in an inactive HUB scores 0 points', '');
      }
      break;
    default:
      break;
  }
}

function startMatch() {
  if (game) for (const r of game.robots) r.destroy();
  const alliance = settings.alliance;
  const match = new Match({ playerAlliance: alliance, onEvent });
  fuel.match = match;
  const cfg = ROBOTS[settings.robot];
  const robot = new Robot({ cfg, alliance, physics, scene, field, fuel, match, climber: CLIMBER_OPTIONS[settings.climber] });
  const custom = customSelection(settings.auto, settings.customSide);
  const pose = startPose(settings.start, robot, alliance, custom);
  robot.spawn(pose.x, pose.z, pose.yaw);
  const oppOn = settings.opponent && settings.opponent !== 'off';
  const oppPreload = oppOn ? 8 : 0;
  const pre = fuel.stage(settings.preload + oppPreload);
  robot.stored.push(...pre.slice(0, settings.preload));
  const hp = new HumanPlayer({ alliance, field, fuel, match });
  hp.auto = settings.hp === 'auto';
  const auto = new AutoRunner(robot, settings.auto, settings.start, alliance, custom);
  const rules = new RobotRules(match);
  const robots = [robot];
  let opp = null;
  if (oppOn) {
    const oa = other(alliance);
    const orobot = new Robot({ cfg: ROBOTS[settings.oppRobot] || ROBOTS['4414'], alliance: oa, physics, scene, field, fuel, match, climber: null });
    const plan = opponentAuto(settings.opponent);
    const op = startPose(plan.start, orobot, oa);
    orobot.spawn(op.x, op.z, op.yaw);
    orobot.stored.push(...pre.slice(settings.preload));
    const ohp = new HumanPlayer({ alliance: oa, field, fuel, match });
    ohp.auto = true;
    opp = {
      robot: orobot,
      hp: ohp,
      auto: new AutoRunner(orobot, plan.routine, plan.start, oa),
      ai: new OpponentAI({ robot: orobot, player: robot, match, fuel, rules, strategy: settings.opponent, skill: settings.oppSkill }),
    };
    robots.push(orobot);
  }
  game = { match, robot, robots, opp, rules, hp, auto, fuel, settings, input, fieldRelative: true, slow: false, driver: null, hpControls: {} };
  rig.alliance = alliance;
  rig.ds = settings.ds;
  rig.setMode(settings.camera);
  ui.camName(CAMERA_NAMES[rig.mode]);
  resultsShown = false;
  simAcc = 0;
  ui.show('hud');
}

// ------------------------------------------------------------------ driver input -> robot command
function driverCommand(inp, dt) {
  const r = game.robot;
  const m = game.match;
  const p = inp.pressed;
  const d = r.cfg.drive;
  // translation: stick up = away from the driver station
  let up = -inp.ly, right = inp.lx;
  const mag = Math.min(1, Math.hypot(up, right));
  if (mag > 0) {
    up *= mag; right *= mag; // squared response for fine control
  }
  game.slow = inp.held.ls;
  const scale = game.slow ? 0.4 : 1;
  const s = settings.alliance === BLUE ? 1 : -1;
  let vx, vz;
  if (game.fieldRelative) {
    vx = s * up * d.maxSpeed * scale;
    vz = s * right * d.maxSpeed * scale;
  } else {
    const f = r.forward(), rt = r.right();
    vx = (f.x * up + rt.x * right) * d.maxSpeed * scale;
    vz = (f.z * up + rt.z * right) * d.maxSpeed * scale;
  }
  const rx = inp.rx;
  const omega = -Math.sign(rx) * rx * rx * d.maxOmega * (game.slow ? 0.5 : 1);
  return {
    vx, vz, omega,
    intake: inp.lt > 0.3,
    shoot: inp.rt > 0.3,
    pass: inp.held.rb,
    outtake: inp.held.lb,
  };
}

function handleGameInput(inp, dt) {
  const p = inp.pressed;
  const r = game.robot;
  if (p.start) { ui.show('pause'); return; }
  if (p.left) ui.camName(rig.cycle(-1));
  if (p.right) ui.camName(rig.cycle(1));
  if (p.b) { game.fieldRelative = !game.fieldRelative; ui.toast(game.fieldRelative ? 'Field-relative drive' : 'Robot-relative drive', 'info'); }
  if (r.climberCfg) {
    if (p.up) r.climbTarget = Math.min(r.climberCfg.maxLevel, r.climbTarget + 1);
    if (p.down) r.climbTarget = Math.max(1, r.climbTarget - 1);
  }
  if (p.a && game.match.isTeleop) {
    const msg = r.requestClimb();
    if (msg) ui.toast(msg, '');
  }
  game.hpControls = { toggleDoor: p.x, throwHeld: inp.held.y };
  game.driver = driverCommand(inp, dt);
  // hold View/Back for 1s to restart
  if (inp.held.back) {
    backHeld += dt;
    if (backHeld > 1) { backHeld = 0; startMatch(); }
  } else backHeld = 0;
}

const IDLE_CMD = { vx: 0, vz: 0, omega: 0, intake: false, shoot: false, pass: false, outtake: false };

function stepSim(dt) {
  const { match, robot, robots, auto, opp } = game;
  const prevPhase = match.phase;
  match.update(dt, robots);
  for (const r of robots) {
    if (prevPhase === 'pre' && match.phase === 'auto' && r.cfg.intake.latched) r.forceDeploy = true;
    if (r.forceDeploy && r.intakeDeploy >= 1) r.forceDeploy = false;
    r.enabled = match.robotEnabled;
  }
  if (match.isAuto) {
    auto.update(dt);
  } else if (match.isTeleop && game.autopilot) {
    game.autopilot.update(dt); // test hook: an AI drives the player's robot
  } else if (match.isTeleop && game.driver) {
    Object.assign(robot.cmd, game.driver);
  } else {
    Object.assign(robot.cmd, IDLE_CMD);
  }
  if (opp) {
    if (match.isAuto) opp.auto.update(dt);
    else opp.ai.update(dt);
  }
  for (const r of robots) r.preStep(dt);
  fuel.preStep(dt);
  physics.step();
  fuel.postStep(dt, match.t);
  for (const r of robots) r.postStep(dt, match.t);
  game.rules.update(dt, robots);
}

// ------------------------------------------------------------------ main loop
let last = performance.now();
let menuOrbit = 0;
let debugInput = null; // test hook: merged into the polled controller state
function frame(now) {
  requestAnimationFrame(frame);
  const dt = Math.min(0.05, (now - last) / 1000);
  last = now;
  tick(dt, true);
}

function tick(dt, render) {
  const inp = input.poll(dt);
  if (debugInput) {
    for (const [k, v] of Object.entries(debugInput)) {
      if (k === 'held' || k === 'pressed') Object.assign(inp[k], v);
      else inp[k] = v;
    }
    if (debugInput.pressed) debugInput.pressed = {};
  }
  if (ui.screen === 'menu') ui.setPadStatus(input.padName);
  const overlay = ui.handleInput(inp, dt);

  if (game && !overlay && ui.screen === 'hud') {
    handleGameInput(inp, dt);
    if (ui.screen === 'hud') {
      simAcc += dt;
      let n = 0;
      while (simAcc >= PHYSICS_DT && n < 8) {
        stepSim(PHYSICS_DT);
        simAcc -= PHYSICS_DT;
        n++;
      }
      if (n >= 8) simAcc = 0;
      const t = game.match.t;
      fuel.update(dt, t);
      game.hp.update(dt, t, game.hpControls);
      if (game.opp) game.opp.hp.update(dt, t, {});
      if (game.match.over && !resultsShown) {
        resultsShown = true;
        ui.showResults(game);
      }
    }
  }

  if (game) {
    for (const r of game.robots) r.update(dt);
    for (const a of [BLUE, RED]) field.setHubLights(a, game.match.hubLightMode(a), performance.now() / 1000);
    if (ui.screen === 'hud') ui.updateHUD(game);
    // shot preview
    const r = game.robot;
    const showPrev = settings.preview === 'on' && (r.cmd.shoot || r.cmd.pass) && r.preview;
    if (showPrev) {
      const pts = r.previewPoints();
      if (pts && pts.length >= 6) {
        const arr = previewGeo.attributes.position.array;
        const n = Math.min(pts.length, arr.length);
        arr.set(pts.slice(0, n));
        previewGeo.setDrawRange(0, n / 3);
        previewGeo.attributes.position.needsUpdate = true;
        previewLine.computeLineDistances();
        previewMat.color.set(r.ready ? 0x37d67a : 0xffe066);
        previewLine.visible = true;
      } else previewLine.visible = false;
    } else previewLine.visible = false;
  } else {
    for (const a of [BLUE, RED]) field.setHubLights(a, 'active', performance.now() / 1000);
  }
  fuel.sync();

  if (ui.screen === 'menu' || ui.screen === 'editor') {
    // slow orbit behind the menu
    menuOrbit += dt * 0.06;
    camera.position.set(Math.cos(menuOrbit) * 13, 7.5, Math.sin(menuOrbit) * 10);
    camera.up.set(0, 1, 0);
    camera.lookAt(0, 0, 0);
    if (camera.fov !== 50) { camera.fov = 50; camera.updateProjectionMatrix(); }
    rig.snap = true;
  } else {
    rig.update(dt, game ? game.robot : null);
  }
  if (render) renderer.render(scene, camera);
}
requestAnimationFrame(frame);

// debugging / test handle: __sim.advance(seconds) steps the game without animation frames
window.__sim = {
  get game() { return game; }, fuel, field, physics, settings, THREE, renderer, scene, camera, ui, rig, startMatch,
  setInput(v) { debugInput = v; },
  // test hook: let the AI drive the player's robot too (needs an opponent on the field)
  autopilot(strategy = 'scorer', skill = 'champs') {
    if (!game || !game.opp) return false;
    game.autopilot = new OpponentAI({ robot: game.robot, player: game.opp.robot, match: game.match, fuel, rules: game.rules, strategy, skill });
    return true;
  },
  advance(seconds, fps = 60) {
    const n = Math.round(seconds * fps);
    for (let i = 0; i < n; i++) tick(1 / fps, i === n - 1);
  },
};
