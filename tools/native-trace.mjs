// The native version's parity traces (github.com/TylerHandel/rebuilt-sim-native, PLAN.md): a
// scripted run of this game's own classes, and a digest of its state after each part of every step
// (stepGame's probe), for the native sim to match bit for bit.
//   node --import ./tools/node-env.mjs tools/native-trace.mjs <script.json> <out.json> [--dump <step>]
//
// A script: { name, seed, slots: [{ robot, alliance, pose: [x, z, yaw] }], phase?: 'auto' |
// 'teleop', humanPlayers?: false, steps, commands: [{ from, robot, cmd: { vx, vz, omega, intake, shoot, pass, outtake } }],
// launches: [{ step, ball, pos: [x, y, z], vel: [x, y, z], spin?: [x, y, z] }] }
//  - The robots are made in slot order (blue ones first, as createGame makes them), with no preload,
//    driven by the script ('external'), then placed at their poses.
//  - Math.random is mulberry32(seed) from there on (the game's construction draws aren't part of
//    it), and its draws are counted.
//  - phase: the match jumps straight to it (no pre-match wait), robots enabled.
//  - commands: each robot's command from step `from` on, until its next.
//  - launches: before step `step`, ball (fuel.balls index) is put at pos moving at vel, spinning at
//    spin (fuel.launch, by the HUMAN PLAYER's rules: { by: 'test' }).
//  - humanPlayers: false keeps both HUMAN PLAYERS from throwing at their HUBS (they still reload
//    their CHUTES from the CORRALS); true (the default) as the game has them, throwing while their
//    HUB is active.
//  - frameGame every second step, as the headless matches run it.
//  - aimTruth: true aims from the robots' true poses (AIMTRUTH=1: js/localize.js off), for scripts
//    the native sim plays before its port of the pose estimate.
// The digest (FNV-1a 64, little-endian bits), version 1:
//  - each FUEL in fuel.balls order: enabled (u8), translation, rotation (x, y, z, w), linvel, angvel
//    (f32), its state's name (UTF-8, then a 0);
//  - each robot in game.robots order: the same of its body;
//  - the random draws so far (u64).
// --dump <step>: that step's full state, readable, to stderr.
import { readFileSync, writeFileSync } from 'node:fs';
import { createWorld, mulberry32 } from './headless.mjs';
import { createGame, stepGame, frameGame } from '../js/game.js';
import { PHYSICS_DT } from '../js/constants.js';

const args = process.argv.slice(2);
const [scriptPath, outPath] = args.filter((a, i) => !a.startsWith('--') && args[i - 1] !== '--dump');
const dumpAt = args.includes('--dump') ? Number(args[args.indexOf('--dump') + 1]) : null;
const script = JSON.parse(readFileSync(scriptPath, 'utf8'));

if (script.aimTruth) process.env.AIMTRUTH = '1';
const world = await createWorld();
const slots = [0, 1, 2, 3, 4, 5].map(() => ({ driver: 'empty' }));
const blue = script.slots.filter((s) => s.alliance === 'blue'), red = script.slots.filter((s) => s.alliance === 'red');
if (blue.length > 3 || red.length > 3) throw new Error('three robots an alliance at most');
blue.forEach((s, i) => { slots[i] = { robot: s.robot, auto: 'best', start: 'hub', driver: 'external', skill: 'champs' }; });
red.forEach((s, i) => { slots[3 + i] = { robot: s.robot, auto: 'best', start: 'hub', driver: 'external', skill: 'champs' }; });
const game = createGame(world, { matchMode: '3v3', slots, preload: 0, climber: 'none', hp: 'auto' });
const order = [...blue, ...red];
game.robots.forEach((r, i) => { const [x, z, yaw] = order[i].pose; r.spawn(x, z, yaw); });

let draws = 0n;
const rnd = mulberry32(script.seed ?? 1);
Math.random = () => { draws++; return rnd(); };
globalThis.SIM_SEED = script.seed ?? 1;
if (script.humanPlayers === false) for (const hp of Object.values(game.hps)) hp.auto = false;
if (script.phase) { game.match._go(script.phase); for (const r of game.robots) r.enabled = game.match.robotEnabled; }

const MASK = (1n << 64n) - 1n, f32 = new Float32Array(13), bytes = new Uint8Array(f32.buffer), enc = new TextEncoder();
let h;
const eat = (u8) => { for (const x of u8) h = ((h ^ BigInt(x)) * 0x100000001b3n) & MASK; };
function body(b) {
  const t = b.translation(), q = b.rotation(), v = b.linvel(), w = b.angvel();
  f32.set([t.x, t.y, t.z, q.x, q.y, q.z, q.w, v.x, v.y, v.z, w.x, w.y, w.z]);
  eat(bytes);
}
function digest() {
  h = 0xcbf29ce484222325n;
  for (const b of world.fuel.balls) { eat([b.body.isEnabled() ? 1 : 0]); body(b.body); eat(enc.encode(b.state)); eat([0]); }
  for (const r of game.robots) body(r.body);
  const d = new DataView(new ArrayBuffer(8)); d.setBigUint64(0, draws, true); eat(new Uint8Array(d.buffer));
  return h.toString(16).padStart(16, '0');
}
function dump(step, phase) {
  const p = (b) => { const t = b.translation(), v = b.linvel(); return `(${t.x}, ${t.y}, ${t.z}) v (${v.x}, ${v.y}, ${v.z})`; };
  console.error(`--- step ${step} ${phase}: draws ${draws}`);
  world.fuel.balls.forEach((b, i) => { if (b.body.isEnabled()) console.error(`fuel ${i} ${b.state} ${p(b.body)}`); });
  game.robots.forEach((r, i) => console.error(`robot ${i} ${r.cfg.name ?? ''} ${p(r.body)}`));
}

const cmds = [...(script.commands ?? [])].sort((a, b) => a.from - b.from);
const launches = script.launches ?? [];
const steps = [];
for (let step = 0; step < script.steps; step++) {
  for (const c of cmds) if (c.from === step) Object.assign(game.robots[c.robot].cmd, { vx: 0, vz: 0, omega: 0, intake: false, shoot: false, pass: false, outtake: false }, c.cmd);
  for (const l of launches) {
    if (l.step !== step) continue;
    const b = world.fuel.balls[l.ball];
    world.fuel.launch(b, { x: l.pos[0], y: l.pos[1], z: l.pos[2] }, { x: l.vel[0], y: l.vel[1], z: l.vel[2] }, { by: 'test' });
    if (l.spin) b.body.setAngvel({ x: l.spin[0], y: l.spin[1], z: l.spin[2] }, true);
  }
  const phases = {};
  stepGame(game, world, PHYSICS_DT, (phase) => { phases[phase] = digest(); if (step === dumpAt) dump(step, phase); });
  if (step % 2 === 1) { frameGame(game, 2 * PHYSICS_DT); phases.frame = digest(); }
  steps.push(phases);
}
writeFileSync(outPath, JSON.stringify({ source: 'rebuilt-sim-2026 tools/native-trace.mjs', version: 1, script: script.name, steps }));
console.log(`${script.name}: ${script.steps} steps, ${game.robots.length} robots, ${draws} draws -> ${outPath}`);
process.exit(0);
