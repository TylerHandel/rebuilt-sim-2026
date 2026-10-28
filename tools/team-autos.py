#!/usr/bin/env python3
"""The teams' own AUTOs, from their robot code, as waypoints the sim's AutoRunner drives
(js/customAutos.js's format, plus each one's start, speeds in m/s and timed actions).
Writes js/teamAutos.js.

  python3 tools/team-autos.py [--8793 DIR] [--2910 DIR] [--1678 DIR] [--604 DIR] [--1114 DIR]

DIR: a checkout of the team's code (defaults: ../pumpkin-bots/2026_robot,
../frcteam2910/2026competitionrobot-public, ../frc1678/C2026-Public,
../frc604/2026-public/FRC-2026, ../Simbotics/2026-Simbot-Tim). All of them use WPILib's
blue-origin field coordinates (x from the blue ALLIANCE WALL, y from the blue drivers' right),
which are the sim's blue fx, fy.

- 8793 (PathPlanner): each .auto's command tree; each path's Bezier waypoints sampled, at its
  max velocity (and constraint zones). Their modes: TrenchMode (intake up, not shooting),
  ShuttleMode (intake, shuttling FUEL home from anywhere), ShooterMode (intake; shoots at the HUB
  in the ALLIANCE ZONE, shuttles from anywhere else).
- 2910 (AutoFactory, driveToPoint / driveToPointWhileIntaking to FieldConstants poses): their
  drive-to-point controller replayed (speed 3 x distance, capped at each command's 15 / 11 / 5
  ft/s, the next command once within its tolerance), so a big tolerance cuts the corner as on the
  robot. scoreStationary shoots until the intake has retracted (at most 4 s). Pose values are
  transcribed from their FieldConstants.java (the right side mirrors the left).
- 1678 (Choreo): each routine's trajectories' samples, at their speed there; driveOverBump (3 m/s straight back over the
  BUMP to where the next trajectory starts) and fireCommand (3 s) between them.
- 604 (Quikplan): every competition AUTO in their chooser (the Champs ones first, its default,
  Double Cycle Inwards, first of all; then the ones commented out from earlier events), each a
  sequence of FollowQuikplan partials (their time-optimal trajectories' 50 Hz samples, at their
  speed there, with the actions along them: launch on/off, intake down and rolling, stowed),
  waits and RetractIntake, from the right side and mirrored left as their isLeft does. A wait
  holds the robot where it is, launching if the last action left it launching.
- 1114 (PathPlanner): their chooser's first, Right / Left 1-1 Middle Sweep : Greedy Sweep (the
  left mirrored, as their chooser does), and Left Slow Centre Sweep Plus Depot. Each path's Bezier
  waypoints at its max velocity (and constraint zones), intaking over its IntakeDeploy zones; each
  FullShoot from x = 3.1 m (their autonXShootPose): aiming until aimed and stopped (at most its
  aiming time), then shooting for its timeout; the flywheel spun up from each AimShooterHub
  marker; no stop where an IntakeDeploy zone starts or ends.
"""
import argparse, json, math, os, re

HERE = os.path.dirname(os.path.abspath(__file__))
ROOT = os.path.dirname(HERE)
W = 8.069  # the field's width (m): the 2026 AprilTag layout's, as all three use
STEP = 0.35  # spacing of the waypoints along a path (m)


def pt(x, y, intake, shoot, v, action='none', t=None, spin=False):
    p = {'fx': round(x, 3), 'fy': round(y, 3), 'intake': intake, 'shoot': shoot, 'v': round(v, 2), 'action': action}
    if t is not None:
        p['t'] = t
    if spin:
        p['spin'] = True  # the flywheel up to its HUB shot on the way here (not shooting), intaking or not
    return p


def resample(xy, step=STEP):
    """points every step along a polyline (always keeping the last)"""
    out, acc = [xy[0]], 0.0
    for a, b in zip(xy, xy[1:]):
        seg = math.dist(a, b)
        acc += seg
        if acc >= step:
            out.append(b)
            acc = 0.0
    if out[-1] != xy[-1]:
        out.append(xy[-1])
    return out


# ------------------------------------------------------------------ 8793: PathPlanner
def bezier(p0, p1, p2, p3, n=40):
    return [tuple((1 - t) ** 3 * a + 3 * (1 - t) ** 2 * t * b + 3 * (1 - t) * t ** 2 * c + t ** 3 * d for a, b, c, d in zip(p0, p1, p2, p3)) for t in (i / n for i in range(n + 1))]


def pp_path(d, name, marker=None, onward=None):
    """(point, speed) every STEP along a path; with marker (an event's name), (point, speed, on):
    whether one of that event's zones (zoned event markers: running from its start to its end) covers it;
    with onward too (a point event's name), (point, speed, on, past): whether it's past that event
    (a command started there runs on to the path's end)"""
    p = json.load(open(os.path.join(d, 'paths', name + '.path')))
    wp = p['waypoints']
    xy, rel = [], []  # points, and each one's waypoint-relative position (for constraint zones)
    for i in range(len(wp) - 1):
        a, b = wp[i], wp[i + 1]
        P = lambda q: (q['x'], q['y'])
        seg = bezier(P(a['anchor']), P(a['nextControl'] or a['anchor']), P(b['prevControl'] or b['anchor']), P(b['anchor']))
        for k, q in enumerate(seg if i == 0 else seg[1:]):
            xy.append(q)
            rel.append(i + (k if i == 0 else k + 1) / (len(seg) - 1))
    vmax = p['globalConstraints']['maxVelocity']
    zones = [(z['minWaypointRelativePos'], z['maxWaypointRelativePos'], z['constraints']['maxVelocity']) for z in p.get('constraintZones', [])]
    speed = [min([vmax] + [zv for lo, hi, zv in zones if lo <= r <= hi]) for r in rel]
    if marker:
        ev = [(e['waypointRelativePos'], e.get('endWaypointRelativePos')) for e in p.get('eventMarkers', []) if e['name'] == marker]
        # (+0.01: zones end to end, as 1114's IntakeDeploys are 0.001 apart, run as one)
        on = [any(hi is not None and lo <= r <= hi + 0.01 for lo, hi in ev) for r in rel]
    past = [any(e['name'] == onward and r >= e['waypointRelativePos'] for e in p.get('eventMarkers', [])) for r in rel]
    # sample along it, a new point wherever the speed changes too (and the marker)
    out, acc = [(xy[0], speed[0])], 0.0
    for i in range(1, len(xy)):
        acc += math.dist(xy[i - 1], xy[i])
        if acc >= STEP or speed[i] != out[-1][1] or (marker and on[i] != on[i - 1]) or past[i] != past[i - 1] or i == len(xy) - 1:
            out.append((xy[i], speed[i]))
            acc = 0.0
            if marker: out[-1] += (on[i],)
            if onward: out[-1] += (past[i],)
    if marker: out[0] += (on[0],)
    if onward: out[0] += (past[0],)
    return out


MODES_8793 = {
    None: (False, 'off'),             # a path on its own: nothing running
    'TrenchMode': (False, 'off'),     # intake to the TRENCH position, rollers off, flywheel off
    'ShuttleMode': (True, 'always'),  # intake running, shuttling home from anywhere
    'ShooterMode': (True, 'always'),  # intake running; the HUB in our zone, shuttling elsewhere
}


def auto_8793(d, file):
    a = json.load(open(os.path.join(d, 'autos', file + '.auto')))
    pts, start = [], None
    for c in a['command']['data']['commands']:
        t, data = c['type'], c.get('data', {})
        kids = data.get('commands', []) if t in ('race', 'parallel', 'deadline', 'sequential') else [c]
        path = next((k['data']['pathName'] for k in kids if k['type'] == 'path'), None)
        mode = next((k['data']['name'] for k in kids if k['type'] == 'named'), None)
        wait = next((k['data']['waitTime'] for k in kids if k['type'] == 'wait'), None)
        intake, shoot = MODES_8793[mode]
        if path:
            seg = pp_path(d, path)
            if start is None:
                start = seg[0][0]
            for (x, y), v in seg[1:]:
                pts.append(pt(x, y, intake, shoot, v))
        elif mode and pts and wait is not None:
            # a mode on the spot for a while (ShooterMode: shooting what it holds)
            pts[-1].update(action='shoot', t=wait)
        elif mode and wait is not None:
            start_shoot = wait  # (before any path: from the start)
            pts.append({'preload': start_shoot})
        elif mode and pts:
            pts[-1].update(action='shoot', t=20)  # the rest of AUTO
    pre = next((p['preload'] for p in pts if 'preload' in p), None)
    pts = [p for p in pts if 'preload' not in p]
    return start, pts, pre


# ------------------------------------------------------------------ 1114: PathPlanner
# Their named commands (RobotContainer.addCommandsToPathPlanner, CommandFactory): IntakeDeploy
# (lowerIntake then RunIntakeBallevator), as zoned event markers along the paths, so the intake
# runs over each zone; AimShooterHub (the flywheel up to the shot, at the end of each path);
# FullShoot33 / FullShoot4 / FullShoot5, a timedShoot (aimingMilliseconds, timeoutSeconds) of
# (1000, 3.4) / (750, 2.25) / (750, 3.25) beside aimDriveHubAuto, which drives straight to
# x = DriveConstants.autonXShootPose (3.1 m) along y and turns to the HUB: its Shoot starts once
# the heading's within the drive's 2.75 deg and it's stopped (|vx| < 0.05), or once the aiming time
# is up, and runs for the timeout (it doesn't end when the hopper's empty), sitting there all that
# time; ShooterStop (the flywheel coasts). AimShooterHub (an AimShooterAuto, on the shooter alone,
# so it runs beside IntakeDeploy on the intake and indexer) spins the flywheel up to the HUB shot
# from its marker to the path's end ('spin'). PathPlanner runs the event zones beside the path:
# it doesn't stop where they start or end ('flow'). Their chooser's Left autos are the Right ones
# mirrored (PathPlannerAuto(name, true)).
SHOOT_1114 = {'FullShoot33': (1.0, 3.4), 'FullShoot4': (0.75, 2.25), 'FullShoot5': (0.75, 3.25)}
SHOOT_X_1114 = 3.1


def auto_1114(d, file, mirror):
    a = json.load(open(os.path.join(d, 'autos', file + '.auto')))
    ys = (lambda y: W - y) if mirror else (lambda y: y)
    pts, start = [], None
    for c in a['command']['data']['commands']:
        t, data = c['type'], c.get('data', {})
        if t == 'path':
            seg = pp_path(d, data['pathName'], 'IntakeDeploy', 'AimShooterHub')
            if start is None:
                start = (seg[0][0][0], ys(seg[0][0][1]))
            for (x, y), v, on, spin in seg[1:]:
                pts.append(pt(x, ys(y), on, 'off', v, spin=spin))
        elif t == 'named' and data['name'] in SHOOT_1114:
            aim, dur = SHOOT_1114[data['name']]
            last = pts[-1]
            if last['fx'] > SHOOT_X_1114:
                pts.append(pt(SHOOT_X_1114, last['fy'], False, 'off', 1.0, spin=True))
            pts[-1].update(action='shoot', t=dur, aim=aim, hold=True)
    return start, pts


# ------------------------------------------------------------------ 2910: drive to point
FT = 0.3048
FAST, MED, SLOW = 15 * FT, 11 * FT, 5 * FT
DEFAULT_TOL = 6 * 0.0254
P2910 = {  # BLUE, the left side (FieldConstants.java); the right is (x, W - y)
    'start': (4.5, 7.6), 'firstEntry': (9.9, 7.0), 'firstCenter': (8.1, 2.8), 'secondEntry': (7.2, 7.7),
    'secondCenter': (6.0, 3.3), 'sweepExit': (6.3, 5.3), 'trenchEnter': (2.8, 7.7), 'bumpScore': (2.6, 5.6),
    'curve': (8.3, 4.8),
    'center': (3.6, 4.0), 'depotRight': (0.65, 5.8), 'depotMiddle': (1.5, 6.0), 'depotLeft': (0.65, 6.2),
    'depotScoring': (1.4, 5.0), 'depotTrenchMid': (1.5, 6.0), 'depotTrenchEnter': (1.4, 7.7), 'depotNZ': (8.3, 7.7),
}
# their createLeftNeutralZoneAuto / createRightNeutralZoneAuto / createDepotAuto: (kind, pose,
# speed, tolerance, time limit). kind: 'in' driveToPointWhileIntaking, 'go' driveToPoint, 'shoot'
# scoreStationary, 'pause' (driving to a point at 0.01 m/s for t: holding still, intaking),
# 'wait' t s, 'until' t s into AUTO
SCORE = [('go', 'sweepExit', MED, 0.5), ('go', 'bumpScore', MED, 1.0), ('go', 'bumpScore', MED, DEFAULT_TOL), ('shoot', 4)]
LEFT_NZ = [
    ('in', 'firstEntry', FAST, 3.5), ('in', 'firstEntry', FAST, 2.5), ('in', 'firstCenter', FAST, 3.0), ('in', 'firstCenter', FAST, 1.0),
    *SCORE,
    ('in', 'trenchEnter', MED, 0.75, 2.0), ('in', 'secondEntry', FAST, 2.0), ('in', 'secondEntry', FAST, 1.0),
    ('in', 'secondCenter', MED, 1.5), ('pause', 0.5), ('in', 'curve', MED, 0.5),
    *SCORE,
    ('in', 'trenchEnter', MED, 0.75, 2.0), ('in', 'secondEntry', FAST, 2.0), ('in', 'secondEntry', FAST, 1.0),
    ('in', 'firstCenter', MED, DEFAULT_TOL),
]
RIGHT_NZ = [
    ('in', 'firstEntry', FAST, 2.5), ('in', 'firstCenter', FAST, 2.0), ('in', 'firstCenter', FAST, 1.0),
    *SCORE,
    ('in', 'trenchEnter', MED, 0.75, 2.0), ('in', 'secondEntry', FAST, 2.0), ('in', 'secondEntry', FAST, 1.0),
    ('in', 'secondCenter', MED, 1.5), ('pause', 0.5), ('in', 'curve', MED, 0.5),
    *SCORE,
    ('in', 'trenchEnter', MED, 0.75, 2.0), ('in', 'secondEntry', FAST, 2.0), ('in', 'secondEntry', FAST, 1.0),
    ('in', 'firstCenter', MED, DEFAULT_TOL),
]
DEPOT = [
    ('in', 'depotRight', SLOW, DEFAULT_TOL), ('wait', 1.0), ('in', 'depotMiddle', MED, DEFAULT_TOL),
    ('in', 'depotLeft', SLOW, DEFAULT_TOL), ('wait', 1.0), ('in', 'depotScoring', MED, DEFAULT_TOL), ('shoot', 4),
    ('in', 'depotTrenchMid', SLOW, DEFAULT_TOL), ('in', 'depotTrenchEnter', SLOW, DEFAULT_TOL), ('until', 15.0),
    ('in', 'depotNZ', MED, DEFAULT_TOL),
]


def auto_2910(seq, start, mirror):
    P = lambda k: (P2910[k][0], W - P2910[k][1]) if mirror else P2910[k]
    pos, vel, dt, accel = list(P(start)), [0.0, 0.0], 0.02, 6.0
    pts, trail = [], [tuple(pos)]
    flags = (False, 0.0)

    def flush(intake, v):
        for q in resample(trail)[1:]:
            pts.append(pt(q[0], q[1], intake, 'off', v))
        trail[:] = [tuple(pos)]

    for cmd in seq:
        kind = cmd[0]
        if kind in ('shoot', 'wait', 'until', 'pause'):
            if len(trail) > 1:
                flush(*flags)
            vel = [0.0, 0.0]
            if not pts:
                pts.append(pt(pos[0], pos[1], flags[0], 'off', flags[1] or MED))
            act = {'shoot': 'shoot', 'wait': 'wait', 'until': 'until', 'pause': 'wait'}[kind]
            if pts[-1]['action'] != 'none':  # (two in a row: a point of its own for the second)
                pts.append(dict(pts[-1], action='none'))
                pts[-1].pop('t', None)
            pts[-1].update(action=act, t=cmd[1])
            if kind == 'pause':
                pts[-1]['intake'] = True
            continue
        _, key, vmax, tol, *limit = cmd
        intake = kind == 'in'
        if (intake, vmax) != flags and len(trail) > 1:
            flush(*flags)
        flags = (intake, vmax)
        tgt, t = P(key), 0.0
        while math.dist(pos, tgt) >= tol and (not limit or t < limit[0]) and t < 8:
            dx, dy = tgt[0] - pos[0], tgt[1] - pos[1]
            dist = math.hypot(dx, dy)
            sp = min(3.0 * dist, vmax)
            want = (dx / dist * sp, dy / dist * sp)
            dvx, dvy = want[0] - vel[0], want[1] - vel[1]
            dv = math.hypot(dvx, dvy)
            k = min(1.0, accel * dt / dv) if dv > 0 else 0
            vel = [vel[0] + dvx * k, vel[1] + dvy * k]
            pos = [pos[0] + vel[0] * dt, pos[1] + vel[1] * dt]
            trail.append(tuple(pos))
            t += dt
    if len(trail) > 1:
        flush(*flags)
    return P(start), pts


# ------------------------------------------------------------------ 1678: Choreo
def choreo(d, name, mirror):
    t = json.load(open(os.path.join(d, 'src/main/deploy/choreo', name + '.traj')))
    s = t['trajectory']['samples']
    events = {e['name']: e['from']['targetTimestamp'] + (e['from']['offset']['val'] or 0) for e in t.get('events', [])}
    ys = (lambda y: W - y) if mirror else (lambda y: y)
    xy = [(q['x'], ys(q['y']), q['t'], math.hypot(q['vx'], q['vy'])) for q in s]
    return xy, events.get('intake', 0.0)


def auto_1678(d, trajs, mirror):
    pts, start = [], None
    over_bump = 3.0  # their OVER_BUMP_FORWARD_SPEED
    for i, name in enumerate(trajs):
        xy, intake_at = choreo(d, name, mirror)
        if start is None:
            start = xy[0][:2]
        elif pts:
            # driveOverBump from the last trajectory's end to here, then fireCommand
            a, b = (pts[-1]['fx'], pts[-1]['fy']), xy[0][:2]
            n = max(1, int(math.dist(a, b) / STEP))
            for k in range(1, n + 1):
                pts.append(pt(a[0] + (b[0] - a[0]) * k / n, a[1] + (b[1] - a[1]) * k / n, True, 'off', over_bump))
            pts[-1].update(action='shoot', t=3.0)
        last, acc = xy[0], 0.0
        for k, q in enumerate(xy[1:], 1):
            acc += math.dist(last[:2], q[:2])
            last = q
            if acc >= STEP or k == len(xy) - 1:
                # (its speed there in their trajectory, but not crawling: the sim's runner brakes for the turns)
                pts.append(pt(q[0], q[1], q[2] >= intake_at, 'off', max(1.0, q[3])))
                acc = 0.0
    return start, pts


# ------------------------------------------------------------------ 604: Quikplan partials
# Their FollowQuikplan replays a time-optimal trajectory (deploy/<name>.csv, 50 Hz: t, x, y, theta,
# vx, vy, omega, action, ...) and sets its mechanisms from the latest action so far (from nothing
# at the start of each partial): 1 launch on, 2 off, 3 intake down and rolling, 4 rollers off, 5
# stowed, 6 and 7 all on (6 falls through to 7), 8 and 9 all off (8 falls through), 10 off. What
# a partial leaves set stays set through the waits after it (WaitCommand; the launch flag is a
# static, and their intake holds its last command); RetractIntake stows the intake (its rollers off)
# and finishes within 30 deg of it. The isLeft autos read the same file mirrored across the field
# (y -> width - y); the left side's last leg is a file of its own (shoot_to_depot_left.csv).
def qp_state(state, action):
    deploy, roll, shoot = state
    if action == 1: shoot = True
    elif action == 2: shoot = False
    elif action == 3: deploy = roll = True
    elif action == 4: roll = False
    elif action == 5: deploy = roll = False
    elif action in (6, 7): deploy = roll = shoot = True
    elif action in (8, 9, 10): deploy = roll = shoot = False
    return (deploy, roll, shoot)


def qp_partial(d, name, flip):
    rows = [list(map(float, r.split(','))) for r in open(os.path.join(d, 'src/main/deploy', name + '.csv')).read().split('\n') if r.strip()]
    ys = (lambda y: W - y) if flip else (lambda y: y)
    state, out = (False, False, False), []
    for r in rows:
        if int(r[7]) != 0: state = qp_state(state, int(r[7]))
        out.append((r[1], ys(r[2]), r[0], math.hypot(r[4], r[5]), state))
    return out, rows[-1][0]


def auto_604(d, seq):
    """seq: ('qp', file, flip, endTimeExtension) | ('wait', s) | ('retract',). Returns start, points."""
    pts, start, state, hold, pos = [], None, (False, False, False), 0.0, None

    def flush_hold(final=False):
        nonlocal hold
        if hold <= 0 and not final: return
        intake, shoot = state[0] and state[1], state[2]
        if final and not shoot and hold <= 0: return
        # (shooting with its intake still rolling, for the whole wait even once it's empty: their
        # robot sits it out; launching to the end of AUTO when that's how it ends)
        p = pt(pos[0], pos[1], intake, 'always' if shoot else 'off', 1.0, 'shoot' if shoot else 'wait', round(20.0 if final and shoot else hold, 2))
        p['hold'] = True
        pts.append(p)
        hold = 0.0

    for item in seq:
        if item[0] == 'wait':
            hold += item[1]
            continue
        if item[0] == 'retract':
            state = (False, False, state[2])
            hold += 0.5  # (the arm up to within 30 deg of stowed, slowly)
            continue
        _, name, flip, ext = item
        samples, T = qp_partial(d, name, flip)
        if start is None:
            start = pos = samples[0][:2]
            flush_hold()  # (a wait before it moves: where it starts, nothing running yet)
        moved = math.dist(samples[0][:2], samples[-1][:2]) > 0.05 or any(math.dist(samples[0][:2], q[:2]) > 0.05 for q in samples)
        if moved:
            flush_hold()
            last, acc = samples[0], 0.0
            for k, q in enumerate(samples[1:], 1):
                acc += math.dist(last[:2], q[:2])
                last = q
                if acc >= STEP or k == len(samples) - 1 or q[4] != samples[k - 1][4]:
                    s = q[4]
                    # (their speed there in the trajectory, but not crawling: the sim's runner brakes for turns)
                    pts.append(pt(q[0], q[1], s[0] and s[1], 'always' if s[2] else 'off', max(1.0, q[3])))
                    acc = 0.0
            pos = samples[-1][:2]
        else:
            hold += T  # (turning on the spot: their turn_forward_transition)
        state = samples[-1][4]
        hold += ext
    flush_hold(final=True)
    return start, pts


# their autos (RobotContainer's chooser at the end of Champs first, its default first of all; then
# the ones it had commented out, from earlier events), each from the right side and mirrored left
CD = 0.5  # Constants.Auto.centerDelay / startingDelay (their dashboard's defaults)
DEPOT_R, DEPOT_L = ('qp', 'shoot_to_depot', False, 1.0), ('qp', 'shoot_to_depot_left', False, 1.0)
def A604(name, cls, right, left=None, champs=True):
    return {'name': name, 'cls': cls, 'right': right, 'left': left, 'champs': champs}
def both(*items):  # the same partials from both sides, the left mirrored
    return [tuple(i) for i in items], [(i[0], i[1], True, i[3]) if i[0] == 'qp' else i for i in items]
AUTOS_604 = []
for name, cls, items, champs in [
        ('Double Cycle Inwards', 'AutoDoubleCycleRightOutsideIn', [('qp', 'first_cycle', False, 0), ('qp', 'turn_forward_transition', False, 0), ('retract',), ('qp', 'second_cycle_outside_in', False, 0), ('wait', 2.0), ('retract',)], True),
        ('Double Cycle with XXXX', 'AutoDoubleCycleRightWithXXXX', [('qp', 'first_cycle_with_XXXX', False, 0), ('qp', 'turn_forward_transition', False, 0), ('retract',), ('qp', 'second_cycle_with_XXXX', False, 0), ('wait', 2.0), ('retract',)], True),
        ('Single Cycle Center Pause', 'AutoCenterPause', [('qp', 'center_pause', False, 0), ('wait', 0.25), ('qp', 'center_pause_sweep', False, 1.0), ('wait', 2.5), ('retract',)], True),
        ('Single Cycle Center Pause OPP', 'AutoCenterPauseOpp', [('qp', 'center_pause', False, 0), ('wait', 0.25), ('qp', 'center_pause_sweep_opp', False, 1.0), ('wait', 2.5), ('retract',)], True),
        ('Single Cycle Center Pause HUB', 'AutoCenterPauseHub', [('qp', 'center_pause', False, 0), ('wait', 0.25), ('qp', 'center_pause_sweep_hub', False, 1.0), ('wait', 2.5), ('retract',)], True),
        ('Fast Race no depot', 'AutoFastRaceNoDepot', [('qp', 'fast_race', False, 0), ('wait', CD), ('qp', 'fast_race_return', False, 1.0), ('wait', 3.0), ('retract',)], True),
        ('Slow Race no depot', 'AutoSlowRaceNoDepot', [('qp', 'slow_race', False, 1.0), ('wait', 3.0), ('retract',), ('wait', 5.0), ('qp', 'shoot_to_neutral', False, 1.0)], True),
        ('Go after partner + return same NO DEPOT', 'AutoGoAfterPartnerReturnSameSide{}NoDepot', [('wait', CD), ('qp', 'go_after_partner', False, 0), ('wait', CD), ('qp', 'after_partner_return', False, 1.0), ('wait', 3.0), ('retract',)], True),
        ('Go after partner + feed', 'AutoGoAfterPartnerFeed', [('wait', CD), ('qp', 'go_after_partner', False, 0), ('qp', 'after_partner_feed', False, 1.0)], True),
        ('Double Cycle', 'AutoDoubleCycleRight', [('qp', 'first_cycle', False, 0), ('wait', 3.0), ('retract',), ('qp', 'second_cycle', False, 0), ('wait', 2.0), ('retract',)], False),
        ('Double Cycle (no opp)', 'AutoDoubleCycleNoOppRight', [('qp', 'first_cycle_no_opp', False, 0), ('wait', 3.0), ('retract',), ('qp', 'second_cycle', False, 0), ('wait', 2.0), ('retract',)], False),
        ('Double Cycle Outwards', 'AutoDoubleCycleRightInsideOut', [('qp', 'first_cycle', False, 0), ('wait', 3.0), ('retract',), ('qp', 'second_cycle_inside_out', False, 0), ('wait', 2.0), ('retract',)], False),
        ('Single Cycle No Citrus', 'AutoSingleCycleNoCitrusRight', [('qp', 'first_cycle', False, 0), ('wait', 3.0), ('retract',), ('qp', 'second_cycle_no_citrus', False, 0)], False),
        ('vs XXXX', 'AutoVsXXXX', [('qp', 'first_cycle_vs_XXXX', False, 0), ('wait', 3.0), ('retract',), ('qp', 'second_cycle', False, 0)], False),
        ('HMB Finals', 'AutoHMBFinals', [('qp', 'vs_XXXX_XXXX_first_sweep', False, 0), ('qp', 'vs_XXXX_XXXX_second_sweep', False, 0)], False),
        ('Feed Short', 'AutoFeedShort', [('qp', 'feed_first_cycle_short', False, 0), ('wait', 3.0), ('retract',)], False),
        ('Feed Long', 'AutoFeedLong', [('qp', 'feed_first_cycle_long', False, 0), ('wait', 3.0), ('retract',)], False)]:
    r, l = both(*items)
    AUTOS_604.append(A604(name, cls, r, l, champs))
# the ones with a left and right class each, whose left side ends at the DEPOT by a file of its own
for name, cls, first, champs in [
        ('Center Hub + Depot', 'AutoCenterPauseSlow{}', [('qp', 'center_pause_slow', False, 0), ('wait', 2.0), ('qp', 'center_pause_slow_sweep_hub', False, 0)], True),
        ('Center Hub + Depot XXXX', 'AutoCenterPauseSlow{}XXXX', [('qp', 'center_pause_slow_XXXX', False, 0), ('wait', 2.0), ('qp', 'center_pause_slow_sweep_hub_XXXX', False, 0)], True),
        ('Fast Race', 'AutoFastRace{}', [('qp', 'fast_race', False, 0), ('wait', CD), ('qp', 'fast_race_return', False, 1.0)], True),
        ('Go after partner + return same', 'AutoGoAfterPartnerReturnSameSide{}', [('wait', CD), ('qp', 'go_after_partner', False, 0), ('wait', CD), ('qp', 'after_partner_return', False, 1.0)], True),
        ('Go after partner pause + return same', 'AutoGoAfterPartnerPauseReturnSameSide{}', [('wait', CD), ('qp', 'go_after_partner_with_pause', False, 0), ('wait', CD), ('qp', 'after_partner_return', False, 1.0)], True)]:
    r, l = both(*first)
    AUTOS_604.append(A604(name, cls, r + [DEPOT_R, ('wait', 1.0), ('retract',)], l + [DEPOT_L, ('wait', 1.0), ('retract',)], champs))
# (Cleanup Feed: one partial from the right only, feed_cleanup.csv, not mirrored)
AUTOS_604.append(A604('Cleanup Feed', 'AutoFeedCleanup', [('qp', 'feed_cleanup', False, 1.0)], None, False))
# (the order: their Champs chooser's, its default first)
ORDER_604 = ['Double Cycle Inwards', 'Double Cycle with XXXX', 'Single Cycle Center Pause', 'Single Cycle Center Pause OPP', 'Single Cycle Center Pause HUB',
             'Center Hub + Depot', 'Center Hub + Depot XXXX', 'Fast Race', 'Fast Race no depot', 'Slow Race no depot', 'Go after partner + return same',
             'Go after partner pause + return same', 'Go after partner + return same NO DEPOT', 'Go after partner + feed']
AUTOS_604.sort(key=lambda a: ORDER_604.index(a['name']) if a['name'] in ORDER_604 else len(ORDER_604))


# ------------------------------------------------------------------ all of it
COMPACT = ('604',)
def pack(p):
    f = (1 if p['intake'] else 0) | (2 if p['shoot'] != 'off' else 0) | (4 if p.get('hold') else 0)
    out = [p['fx'], p['fy'], p['v'], f]
    if p['action'] != 'none': out += [p['action'], p.get('t')]
    return out


def main():
    ap = argparse.ArgumentParser()
    up = os.path.dirname(ROOT)
    ap.add_argument('--8793', dest='t8793', default=os.path.join(up, 'pumpkin-bots/2026_robot'))
    ap.add_argument('--2910', dest='t2910', default=os.path.join(up, 'frcteam2910/2026competitionrobot-public'))
    ap.add_argument('--1678', dest='t1678', default=os.path.join(up, 'frc1678/C2026-Public'))
    ap.add_argument('--604', dest='t604', default=os.path.join(up, 'frc604/2026-public/FRC-2026'))
    ap.add_argument('--1114', dest='t1114', default=os.path.join(up, 'Simbotics/2026-Simbot-Tim'))
    a = ap.parse_args()
    autos = {}
    pp = os.path.join(a.t8793, 'src/main/deploy/pathplanner')
    for side, file, name in [('left', 'DEPOT Left Neutral Zone Shuttle Auto', 'Left Neutral Zone Shuttle + Depot'),
                             ('right', 'Right Neutral Zone Shuttle Auto', 'Right Neutral Zone Shuttle'),
                             ('center', 'Center + Depot Auto', 'Center + Depot')]:
        start, pts, pre = auto_8793(pp, file)
        autos.setdefault('8793', []).append({'id': f'8793-{side}', 'name': name, 'side': side, 'source': f'PathPlanner "{file}"',
                                             'startFx': round(start[0], 3), 'startFy': round(start[1], 3), 'shootPreload': pre or False, 'points': pts})
    for side, seq, start, mirror, name, src in [
            ('left', LEFT_NZ, 'start', False, 'Left Neutral Zone', 'createLeftNeutralZoneAuto'),
            ('right', RIGHT_NZ, 'start', True, 'Right Neutral Zone', 'createRightNeutralZoneAuto'),
            ('center', DEPOT, 'center', False, 'Depot', 'createDepotAuto')]:
        st, pts = auto_2910(seq, start, mirror)
        autos.setdefault('2910', []).append({'id': f'2910-{side}', 'name': name, 'side': side, 'source': f'AutoFactory.{src}',
                                             'startFx': round(st[0], 3), 'startFy': round(st[1], 3), 'shootPreload': False, 'points': pts})
    for side, mirror, name, src in [('left', False, 'Left Double Sweep', 'LeftSweep'), ('right', True, 'Right Double Sweep', 'RightSweep')]:
        st, pts = auto_1678(a.t1678, ['startToScore', 'scoreToSecondSweep', 'secondScoreToNeutral'], mirror)
        autos.setdefault('1678', []).append({'id': f'1678-{side}', 'name': name, 'side': side, 'source': f'autos/midline/{src}.java',
                                             'startFx': round(st[0], 3), 'startFy': round(st[1], 3), 'shootPreload': False, 'points': pts})
    for au in AUTOS_604:
        slug = re.sub(r'[^a-z0-9]+', '-', au['name'].lower()).strip('-')
        for side in ('right', 'left'):
            seq = au[side]
            if not seq: continue
            st, pts = auto_604(a.t604, seq)
            cls = au['cls'].format(side.capitalize()) if '{}' in au['cls'] else au['cls'] + (' (isLeft)' if side == 'left' else '')
            files = ' + '.join(i[1] + '.csv' for i in seq if i[0] == 'qp')
            autos.setdefault('604', []).append({'id': f'604-{slug}-{side}', 'name': f"{au['name']} {side.upper()}", 'side': side,
                                                'source': f'commands/autos/{cls}: {files}' + ('' if au['champs'] else ' (not in their Champs chooser)'),
                                                'startFx': round(st[0], 3), 'startFy': round(st[1], 3), 'shootPreload': False, 'points': pts})
    pp = os.path.join(a.t1114, 'src/main/deploy/pathplanner')
    for side, file, mirror, name in [('right', 'RightSideMiddleGreedySweep', False, 'Right 1-1 Middle Sweep : Greedy Sweep'),
                                     ('left', 'RightSideMiddleGreedySweep', True, 'Left 1-1 Middle Sweep : Greedy Sweep'),
                                     ('left', 'LeftCenterSweep254', False, 'Left Slow Centre Sweep Plus Depot')]:
        st, pts = auto_1114(pp, file, mirror)
        slug = re.sub(r'[^a-z0-9]+', '-', name.lower()).strip('-')
        autos.setdefault('1114', []).append({'id': f'1114-{slug}', 'name': name, 'side': side, 'source': f'PathPlanner "{file}"' + (' mirrored' if mirror else ''),
                                             'startFx': round(st[0], 3), 'startFy': round(st[1], 3), 'shootPreload': False, 'flow': True, 'spinPts': True, 'points': pts})
    out = os.path.join(ROOT, 'js', 'teamAutos.js')
    with open(out, 'w') as f:
        f.write('// The teams\' own AUTOs, from their robot code (tools/team-autos.py writes this file; see its\n')
        f.write('// header). Blue field coordinates; points as js/customAutos.js, speeds v in m/s, t: how long a\n')
        f.write('// shoot or wait lasts (s), or for until, the time into AUTO it waits for.\n')
        plain = {k: v for k, v in autos.items() if k not in COMPACT}
        f.write('export const TEAM_AUTOS = ' + json.dumps(plain, separators=(',', ':')) + ';\n')
        # (a team with many autos, 604's, as compact points, [fx, fy, v, flags] and [.., action, t]
        # (flags: 1 intake, 2 shooting, 4 hold the wait out), spelled out as above when it loads)
        for k in COMPACT:
            if k not in autos: continue
            comp = [{**{q: x[q] for q in x if q != 'points'}, 'pts': [pack(p) for p in x['points']]} for x in autos[k]]
            f.write(f"TEAM_AUTOS['{k}'] = " + json.dumps(comp, separators=(',', ':')) + '.map(({ pts, ...a }) => ({ ...a, points: pts.map(unpack) }));\n')
        f.write('function unpack([fx, fy, v, f, action = "none", t]) {\n'
                '  const p = { fx, fy, intake: !!(f & 1), shoot: f & 2 ? "always" : "off", v, action };\n'
                '  if (t !== undefined) p.t = t;\n'
                '  if (f & 4) p.hold = true;\n'
                '  return p;\n'
                '}\n')
    for k, v in autos.items():
        for x in v:
            print(k, x['id'], 'start', x['startFx'], x['startFy'], len(x['points']), 'points', [p['action'] + ' ' + str(p.get('t')) for p in x['points'] if p['action'] != 'none'])


main()
