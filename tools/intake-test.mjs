// Controlled intake check: each robot drives straight through the NEUTRAL ZONE FUEL with its
// intake running; prints FUEL collected, the count every half second, and when the intake stalled
// against the load (full): for a robot whose hopper is full when its intake can't take more
// (naturalFull), the most it held, how long its last 10 took and when it went full
//   node --import ./tools/node-env.mjs tools/intake-test.mjs [speed m/s] [--along] [--time s] [--settle s] [--robots 2910,4414]
// --along drives down the length of the FUEL line (the most FUEL per meter) instead of across it;
// --return (with --along): at the end of the line it turns round and comes back up the other half
// of the NEUTRAL ZONE's FUEL (twice the FUEL to fill from); --teleop: with the current limits
// their teleopInit sets (as most intaking is); --offset d starts it d m to the side (another run through the same FUEL); --settle holds still for s with the intake down first (an arm that takes a while to come down,
// 8793's, would otherwise land on the FUEL); the times and counts are from when it starts driving
// INTAKELOG=1 (4414): its primary roller's and kick wheel's speed, current and what loads them
// (tools/intakewatch4414.mjs); =2 each quarter second too
// --contacts: loose FUEL against the robot's own simplified colliders (bumpers, frame, hopper box,
// intake hull or block), where and what became of them (tools/fuel-contacts.mjs)
import { createWorld, mulberry32 } from './headless.mjs';
import { Robot } from '../js/robot.js';
import { Match } from '../js/match.js';
import { ROBOTS, ROBOT_ORDER } from '../js/robotConfigs.js';
import { PHYSICS_DT, BLUE, FUEL } from '../js/constants.js';
import { intakeWatch } from './intakewatch4414.mjs';
import { contactLog } from './fuel-contacts.mjs';

const speed = +(process.argv[2] || 1.5);
const along = process.argv.includes('--along'), RETURN = process.argv.includes('--return'), TELEOP = process.argv.includes('--teleop');
const arg = (name, def) => { const i = process.argv.indexOf(name); return i > 0 ? process.argv[i + 1] : def; };
const CONTACTS = process.argv.includes('--contacts');
const T = +arg('--time', 4), SETTLE = +arg('--settle', 0), OFF = +arg('--offset', 0);
for (const key of arg('--robots', ROBOT_ORDER.join(',')).split(',')) {
  for (const z of along ? [0] : [-1.5, 0.4]) {
    const world = await createWorld();
    Math.random = mulberry32(7);
    const { physics, scene, field, fuel } = world;
    const match = new Match({ playerAlliance: BLUE, onEvent: () => {} });
    fuel.match = match;
    // (--teleop: the match in teleop, so the robots' teleop current limits apply, as their
    // teleopInit sets them: most intaking is in teleop)
    if (TELEOP) match.phase = 'teleop';
    const r = new Robot({ cfg: ROBOTS[key], alliance: BLUE, physics, scene, field, fuel, match, climber: null });
    if (along) r.spawn(-0.45 + OFF, 3.4, Math.PI / 2); // facing down the line (-z)
    else r.spawn(-2.4, z + OFF, 0);
    fuel.stage(0);
    r.enabled = true;
    let t = 0, first = null, full = null; const curve = [];
    // (the count each step, and each time it went full and with how many: when it got to the most
    // it ever held, how long its last 10 took, and whether it went full and then took more after all)
    const counts = [], fulls = []; let wasFull = false;
    for (let i = 0; i < SETTLE / PHYSICS_DT; i++) {
      Object.assign(r.cmd, { vx: 0, vz: 0, omega: 0, intake: true, shoot: false, pass: false, outtake: false });
      r.preStep(PHYSICS_DT); fuel.preStep(PHYSICS_DT); physics.step(); t += PHYSICS_DT; fuel.postStep(PHYSICS_DT, t); r.postStep(PHYSICS_DT, t);
      if (i % 2) fuel.update(2 * PHYSICS_DT, t);
    }
    const t0 = t;
    // (INTAKELOG, 4414: its primary roller's speed, current and what loads it, tools/intakewatch4414.mjs)
    const ph = r.phys, iw = process.env.INTAKELOG && ph?.t?.primary ? intakeWatch({ path: ph.path, t: ph.t, primary: ph.w.intake[1], local: (b) => ph._local(b.translation()), onCarpet: (b) => b.translation().y < FUEL.radius + 0.002, count: () => r.stored.length }) : null;
    // (--contacts: loose FUEL against the robot's own simplified colliders, tools/fuel-contacts.mjs)
    const clog = CONTACTS ? contactLog(r, fuel, physics) : null;
    let leg = 'down';
    // (MOUTHLOG=s1,s2: at those times the loose FUEL near the front, robot frame, and what of the robot's mechanisms presses them)
    const mlog = process.env.MOUTHLOG?.split(',').map(Number) ?? [];
    // (BARAT=x,y,r: count the steps any loose FUEL touches the robot's mechanism surfaces within r of (x, y), robot frame, and how hard)
    const barAt = process.env.BARAT?.split(',').map(Number), bar = { steps: 0, fuel: new Set(), maxF: 0, sumF: 0 };
    for (let i = 0; i < T / PHYSICS_DT; i++) {
      // (--return: at the end of the line it turns round and comes back up the other half's)
      let cmd = { vx: along ? 0 : speed, vz: along ? -speed : 0, omega: 0 };
      if (along && RETURN) {
        if (leg === 'down' && r.pos.z < -2.9) leg = 'turn';
        const yawErr = Math.atan2(Math.sin(-Math.PI / 2 - r.yaw), Math.cos(-Math.PI / 2 - r.yaw));
        if (leg === 'turn') { cmd = { vx: Math.max(-1, Math.min(1, 2 * (0.45 + OFF - r.pos.x))), vz: 0, omega: 3 * Math.sign(yawErr) }; if (Math.abs(yawErr) < 0.05 && Math.abs(0.45 + OFF - r.pos.x) < 0.03) leg = 'up'; }
        if (leg === 'up') cmd = { vx: 2 * (0.45 + OFF - r.pos.x), vz: speed, omega: 3 * yawErr };
      }
      Object.assign(r.cmd, { ...cmd, intake: true, shoot: false, pass: false, outtake: false });
      r.preStep(PHYSICS_DT);
      fuel.preStep(PHYSICS_DT);
      physics.step();
      t += PHYSICS_DT;
      fuel.postStep(PHYSICS_DT, t);
      r.postStep(PHYSICS_DT, t);
      if (barAt && ph) for (const c of ph.path.contacts) { if (c.wheel.kind !== 'plain' || c.wheel.shaft?.name) {} const q = ph._local({ x: c.x[0], y: c.x[1], z: c.x[2] }); if (Math.hypot(q[0] - barAt[0], q[1] - barAt[1]) < barAt[2] && !c.wheel.radius) { bar.steps++; bar.fuel.add(c.ball); bar.maxF = Math.max(bar.maxF, c.force); bar.sumF += c.force; } }
      if (mlog.length && ph && mlog.some((m) => Math.abs(t - t0 - m) < PHYSICS_DT / 2)) {
        const near = fuel.balls.filter((b) => b.state === 'field').map((b) => [b, ph._local(b.pos)]).filter(([, p]) => p[0] > 0.2 && p[0] < 0.75 && Math.abs(p[2]) < 0.45 && p[1] < 0.5).sort((a, b) => b[1][0] - a[1][0]);
        console.log(`  t ${(t - t0).toFixed(2)} s: ${near.length} near the front; in it ${r.stored.length}; robot ${Math.hypot(r.vel.x, r.vel.z).toFixed(2)} m/s; arm ${ph.armDeg?.toFixed?.(1)} deg`);
        for (const [b, p] of near) { const cs = ph.path.contacts.filter((c) => c.ball === b.body && c.force > 2).map((c) => c.wheel.name.replace(/ \(.*\)/, '') + ' ' + c.force.toFixed(0)); console.log(`    ${p.map((v) => v.toFixed(2)).join(',')} ${b.owned ?? ''}: ${cs.join('; ')}`); }
      }
      if (i % 2) fuel.update(2 * PHYSICS_DT, t);
      iw?.check(PHYSICS_DT, ph.near.map((b) => b.body));
      clog?.step(t);
      if (first === null && r.stored.length) first = t - t0;
      if (full === null && r.full) full = t - t0;
      counts.push(r.stored.length);
      if (r.full && !wasFull) fulls.push({ t: t - t0, n: r.stored.length, i: counts.length - 1, by: r.phys?.fullWatch?.by });
      wasFull = r.full;
      if ((i + 1) % Math.round(0.5 / PHYSICS_DT) === 0) curve.push(r.stored.length);
    }
    // (held a quarter second, not a FUEL bobbing over the line for a step)
    const held = counts.map((_, i) => Math.min(...counts.slice(i, i + Math.round(0.25 / PHYSICS_DT))));
    const most = Math.max(...held), iMost = held.indexOf(most), i10 = held.findIndex((n) => n >= most - 10);
    const early = fulls.filter((f) => Math.max(...held.slice(f.i)) >= f.n + 2);
    const fullNote = !fulls.length ? '' : `; full at ${fulls.map((f) => `${f.t.toFixed(2)} s (${f.n}${f.by ? ', ' + f.by : ''})`).join(', ')}, ${((fulls[fulls.length - 1].t) - (iMost + 1) * PHYSICS_DT).toFixed(2)} s after it first held ${most}${early.length ? `, ${early.length} early (it took 2+ more after)` : ''}`;
    console.log(`  ${key}: the most it held ${most}, first at ${((iMost + 1) * PHYSICS_DT).toFixed(2)} s; its last 10 took ${((iMost - i10) * PHYSICS_DT).toFixed(2)} s${fullNote}`);
    console.log(`${key} z=${z}: collected ${r.stored.length}/${r.capacity()} (in rollers ${r.captured?.length}, dropped ${r.stats.dropped}), first after ${first?.toFixed(2)}s, ${full ? `stalled after ${full.toFixed(2)}s` : 'not stalled'}, every 0.5 s ${curve.join(' ')}, ended at x=${r.pos.x.toFixed(2)} z=${r.pos.z.toFixed(2)}`);
    if (clog) { clog.finish(t); console.log(clog.report()); }
    if (barAt) console.log(`  near (${barAt[0]}, ${barAt[1]}): ${bar.fuel.size} FUEL touched the surfaces there, ${bar.steps} contact-steps, up to ${bar.maxF.toFixed(0)} N (mean ${(bar.sumF / (bar.steps || 1)).toFixed(0)})`);
    if (iw) { const o = iw.report(); console.log('   ' + o.text); if (process.env.INTAKELOG === '2') console.log((o.series || []).map((x) => '     ' + x).join('\n')); }
    physics.world.free();
  }
}
