// How full the scripted AIs are when they head in to score: every collect -> score switch in
// TELEOP, with the load, the robot's capacity and whether its HUB was active. Also the points.
//   node --import ./tools/node-env.mjs tools/ai-trips.mjs [--robots 2910,4414] [--skill champs] [--seeds 2] [--teams 1]
import { runGame, matchSettings } from './headless.mjs';
import { ROBOT_ORDER } from '../js/robotConfigs.js';

const args = process.argv.slice(2);
const arg = (k, d) => { const i = args.indexOf('--' + k); return i >= 0 ? args[i + 1] : d; };
const robots = arg('robots', ROBOT_ORDER.join(',')).split(',');
const skill = arg('skill', 'champs');
const seeds = +arg('seeds', 1);
const teams = +arg('teams', 1);
const fmt = (xs) => xs.length ? (xs.reduce((a, b) => a + b, 0) / xs.length).toFixed(2) : '-';
const all = { act: [], off: [], pts: [] };
for (const key of robots) {
  const act = [], off = [], pts = [];
  for (let seed = 1; seed <= seeds; seed++) {
    const sl = { robot: key, auto: 'best', start: null, driver: 'scorer', skill };
    const settings = teams === 3
      ? { matchMode: '3v3', slots: Array.from({ length: 6 }, () => ({ ...sl })) }
      : { ...matchSettings({ strategy: 'scorer', skill, robot: key }, { strategy: 'scorer', skill, robot: key }) };
    const prev = new Map();
    const res = await runGame(settings, {
      seed,
      trace: (game) => {
        if (!game.match.isTeleop) return;
        for (const u of game.units) {
          const ai = u.ai;
          if (!ai || !ai.state) continue;
          const was = prev.get(u);
          if (was === 'collect' && ai.state === 'score') {
            const cap = u.robot.maxCapacity();
            (game.match.hubActive(u.robot.alliance) ? act : off).push(u.robot.stored.length / cap);
          }
          prev.set(u, ai.state);
        }
      },
    });
    pts.push(res.blue.total, res.red.total);
  }
  console.log(`${key.padEnd(5)} trips in active shift ${String(act.length).padStart(3)}, avg load ${fmt(act)} of capacity · staging trips ${String(off.length).padStart(3)}, avg load ${fmt(off)} · pts/match ${fmt(pts)}`);
  all.act.push(...act); all.off.push(...off); all.pts.push(...pts);
}
console.log(`all   trips in active shift ${all.act.length}, avg load ${fmt(all.act)} · staging trips ${all.off.length}, avg load ${fmt(all.off)} · pts/match ${fmt(all.pts)}`);
