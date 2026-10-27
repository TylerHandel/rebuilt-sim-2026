#!/usr/bin/env python3
"""The teams' own AUTOs, from their robot code, as waypoints the sim's AutoRunner drives
(js/customAutos.js's format, plus each one's start, speeds in m/s and timed actions).
Writes js/teamAutos.js.

  python3 tools/team-autos.py [--8793 DIR] [--2910 DIR] [--1678 DIR]

DIR: a checkout of the team's code (defaults: ../pumpkin-bots/2026_robot,
../frcteam2910/2026competitionrobot-public, ../frc1678/C2026-Public). All three use WPILib's
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
- 581 (team581/frc-2026 comp-bot: Left/RightNormalAuto, imperative state machines on their
  Trailblazer follower): the segments' points transcribed (in their RED coordinates, flipped to
  blue) and their PidPathFollower replayed (straight at each point, the next once within its
  transition tolerance and heading tolerance), cycle after cycle: intake out, back over the BUMP
  to SHOOT_X beside it, shoot 3.25 s from when it's off the BUMP. Their AutoSelection offers only
  these two (Left/RightSpecialAuto aren't selectable).
"""
import argparse, json, math, os

HERE = os.path.dirname(os.path.abspath(__file__))
ROOT = os.path.dirname(HERE)
W = 8.069  # the field's width (m): the 2026 AprilTag layout's, as all three use
STEP = 0.35  # spacing of the waypoints along a path (m)


def pt(x, y, intake, shoot, v, action='none', t=None):
    p = {'fx': round(x, 3), 'fy': round(y, 3), 'intake': intake, 'shoot': shoot, 'v': round(v, 2), 'action': action}
    if t is not None:
        p['t'] = t
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


def pp_path(d, name):
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
    # sample along it, a new point wherever the speed changes too
    out, acc = [(xy[0], speed[0])], 0.0
    for i in range(1, len(xy)):
        acc += math.dist(xy[i - 1], xy[i])
        if acc >= STEP or speed[i] != out[-1][1] or i == len(xy) - 1:
            out.append((xy[i], speed[i]))
            acc = 0.0
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


# ------------------------------------------------------------------ 581: Trailblazer
# LeftNormalAuto's segments (comp-bot autos/auto_state_machines; RightNormalAuto is the same
# mirrored across the field), in their RED coordinates (Point.ofRed: blue = rotated 180 degrees
# about the field's center). (x, y, heading deg, transition tolerance m, max m/s, max rad/s,
# angle tolerance deg); None: the segment's 4.5 m/s, 4 rotations/s. FieldUtil: the DEPOT TRENCH's
# center line (AprilTag 17's y, 2026 welded layout), the DEPOT BUMP's (48.94 + 12 + 73/2 in from
# the wall) less their BUMP_OFFSET (-0.13).
L581 = 16.541
TR581, BY581, SX581 = 0.6444996, (48.94 + 12 + 73 / 2) * 0.0254 + 0.13, 13.83
SEG4 = 4.0 * 2 * math.pi  # Units.rotationsToRadians(4.0)
INTAKE_FIRST = [  # their intakeFirstCycle (intake on from the start)
    (10.489, TR581, 90, 0.75), (9.010, 1.417, 120, 0.4), (8.553, 2.677, 85, 0.4), (8.670, 3.37, 30, 0.4),
    (9.6, 3.57, -90, 0.4), (9.7, BY581, -32, 0.3), (10.6, BY581, 0, 0.3), (SX581, BY581, 0, 0.3),
]
TURN_TO_TRENCH = [  # the first four points of their second and third cycles
    (13.2, TR581, -160, 2.0, 2.0, 4.0, 50), (13.2, TR581, 180, 0.2, 2.5, 4.0), (12.5, TR581, 180, 0.2, 3.0, 2.0),
    (11.5, TR581, 180, 0.2, 4.5, 0.5),
]
INTAKE_SECOND = TURN_TO_TRENCH + [  # their defaultIntakeSecondCycle
    (10.628, 0.87, 140, 0.4), (9.140, 1.417, 130, 0.4), (8.7, 2.477, 85, 0.4), (8.5, 3.76, 30, 0.4),
    (10.07, 3.77, -15, 0.5), (10.2, BY581, -50, 0.1), (SX581, BY581, 0, 0.3),
]
INTAKE_THIRD = TURN_TO_TRENCH + [  # their intakeThirdCycle
    (9.6, 1.717, 130, 0.4), (8.7, 2.477, 90, 0.3), (8.6, 3.76, 90, 0.3), (9.31, 3.77, -32, 0.3),
    (9.7, BY581, 0, 0.3), (10.6, BY581, 0, 0.3), (SX581, BY581, 0, 0.3),
]
# (state machine: each cycle drives its segment until it heads for the last point, SHOOT_X
# (READY_TO_CROSS_BUMP), then CROSS_BUMP_TO_SHOOT drives there over the BUMP, and SHOOT starts
# once it's flat again (their BumpCrossingTracker); SHOOT lasts 3.25 s at most (2 s once it
# isn't shooting). The intake runs from the start, and in the later cycles from START_INTAKE (the
# third point, under the TRENCH); the first cycle cancels it at the BUMP, the second keeps it on
# until SHOOT, the third's idleRequest stops it at the BUMP. Their cluster map's other second
# cycle (a lane further out) isn't modeled: the default one.)
CYCLES_581 = [(INTAKE_FIRST, 0, False), (INTAKE_SECOND, 2, True), (INTAKE_THIRD, 2, False)]
BUMP_CLEAR_581 = L581 - (11.915 + 44.4 / 2 * 0.0254) - 0.343  # (blue fx) its center a half frame past the BUMP


def auto_581(mirror):
    """their follower replayed (PidPathFollower: straight at the target point, at the speed left to
    the end of the segment within the point's constraints, 8 m/s^2; the next point once within its
    transition tolerance, heading included)"""
    def blue(p):
        x, y, h = L581 - p[0], W - p[1], p[2] + 180
        return (x, W - y, -h) if mirror else (x, y, h)
    start = blue((12.1, 0.47, 90))
    pos, speed, hdg, dt, t = [start[0], start[1]], 0.0, start[2], 0.02, 0.0
    pts, samples, anchor = [], [], [tuple(pos)]  # samples: (x, y, intake, shoot, v)

    def flush():
        # sampled along the way, a new point wherever the settings change
        groups, at = [], anchor[0]
        for s in samples:
            if not groups or groups[-1][0] != s[2:]:
                groups.append((s[2:], [at]))
            groups[-1][1].append(s[:2])
            at = s[:2]
        for (intake, shoot, v), xy in groups:
            for q in resample(xy)[1:]:
                pts.append(pt(q[0], q[1], intake, shoot, v))
        samples.clear()
        anchor[0] = tuple(pos)

    for n, (seg, intake_from, intake_after) in enumerate(CYCLES_581):
        P, last, i, landed = [blue(p) for p in seg], len(seg) - 1, 0, None
        while t < 60:
            p = seg[i]
            tol, vmax = p[3], p[4] if len(p) > 4 else 4.5
            wmax, atol = min(p[5] if len(p) > 5 else SEG4, SEG4), p[6] if len(p) > 6 else 100
            herr = (P[i][2] - hdg + 180) % 360 - 180
            d = math.dist(pos, P[i][:2])
            if i < last and d < tol and abs(herr) < atol:
                i += 1
                continue
            if i == last and d < 0.01:
                break
            # SHOOT (their prepareScoreRequest) once it's off the BUMP on the way to SHOOT_X
            if i == last and landed is None and pos[0] < BUMP_CLEAR_581:
                landed = t
            left = d + sum(math.dist(P[k][:2], P[k + 1][:2]) for k in range(i, last))
            sp = min(vmax, math.sqrt(2 * 8.0 * left))
            # (straight at the point: only the speed is rate limited)
            speed += max(-8.0 * dt, min(8.0 * dt, sp - speed))
            if d > 1e-6:
                pos = [pos[0] + (P[i][0] - pos[0]) / d * min(speed * dt, d), pos[1] + (P[i][1] - pos[1]) / d * min(speed * dt, d)]
            step = math.degrees(wmax) * dt
            hdg += max(-step, min(step, herr))
            t += dt
            shooting = landed is not None
            intake = i >= intake_from and (i < last or (intake_after and not shooting))
            samples.append((pos[0], pos[1], intake, 'hub' if shooting else 'off', vmax))
        if landed is None:
            landed = t
        flush()
        hold = round(3.25 - (t - landed), 2) if n < len(CYCLES_581) - 1 else 20
        pts[-1].update(action='shoot', t=hold)
        speed, t = 0.0, t + hold
    return start, pts


# ------------------------------------------------------------------ all of it
def main():
    ap = argparse.ArgumentParser()
    up = os.path.dirname(ROOT)
    ap.add_argument('--8793', dest='t8793', default=os.path.join(up, 'pumpkin-bots/2026_robot'))
    ap.add_argument('--2910', dest='t2910', default=os.path.join(up, 'frcteam2910/2026competitionrobot-public'))
    ap.add_argument('--1678', dest='t1678', default=os.path.join(up, 'frc1678/C2026-Public'))
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
    for side, mirror, name, src in [('left', False, 'Left Normal', 'LeftNormalAuto'), ('right', True, 'Right Normal', 'RightNormalAuto')]:
        st, pts = auto_581(mirror)
        autos.setdefault('581', []).append({'id': f'581-{side}', 'name': name, 'side': side, 'source': f'autos/auto_state_machines/{src}.java',
                                            'startFx': round(st[0], 3), 'startFy': round(st[1], 3), 'shootPreload': False, 'points': pts})
    out = os.path.join(ROOT, 'js', 'teamAutos.js')
    with open(out, 'w') as f:
        f.write('// The teams\' own AUTOs, from their robot code (tools/team-autos.py writes this file; see its\n')
        f.write('// header). Blue field coordinates; points as js/customAutos.js, speeds v in m/s, t: how long a\n')
        f.write('// shoot or wait lasts (s), or for until, the time into AUTO it waits for.\n')
        f.write('export const TEAM_AUTOS = ' + json.dumps(autos, separators=(',', ':')) + ';\n')
    for k, v in autos.items():
        for x in v:
            print(k, x['id'], 'start', x['startFx'], x['startFy'], len(x['points']), 'points', [p['action'] + ' ' + str(p.get('t')) for p in x['points'] if p['action'] != 'none'])


main()
