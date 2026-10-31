#!/usr/bin/env python3
"""6328's own AUTOs (Mechanical-Advantage/RobotCode2026Public: AutoBuilder, AutoCommands,
SalesmanCommands, DriveTrajectories), for js/teamAutos.js. Run on its own it writes 6328's entry
into js/teamAutos.js (tools/team-autos.py's main does the same along with the other teams'):

  python3 tools/autos6328.py

Their trajectories: their Vehicle Trajectory Service builds each one at build time from the path
requests in DriveTrajectories.java (waypoints, headings, stop points and each segment's
constraints: max velocity, max angular velocity, keep-in-lane, point-at) through the Choreo CLI,
time-optimal for their robot (4.0 m/s, 4 N m a wheel at 2in, 150 lb, COF 1.5). The .traj files
aren't in their repository (generated, git-ignored), and the Choreo solver isn't here, so:
- the requests are their own: tools/vts6328 compiles DriveTrajectories.java, FieldConstants and
  AutoFieldConstants against stand-ins for WPILib's geometry and Lombok's builders and dumps every
  path request to tools/vts6328/paths6328.json (see tools/vts6328/run.sh);
- each is time-parameterized here (gen): a smooth curve through the waypoints (centripetal
  Catmull-Rom; a straight line where a segment keeps in a lane), the heading turned evenly by
  distance between the waypoints that give one (point-at: the back toward its target), and the
  speed along it as fast as their robot's limits let it, as Choreo's would be: 4.0 m/s, 4.6 m/s^2
  (4 N m at each of four 2in wheels, 68 kg) along the path and across it in a curve, the modules'
  4.0 m/s shared between going and turning (0.391 m out), each segment's max velocity and max
  angular velocity, stopped where it stops. Choreo would round its corners its own way; the
  waypoints, constraints and timing are theirs.
The salesman (their real-time intaking, SalesmanSolver) and their drive-to-pose moves run live in
the sim (js/auto6328.js). Each AUTO is a program of their commands (below), from the left and
mirrored for the right as their VerticalFlipUtil does.
"""
import json, math, os, sys

HERE = os.path.dirname(os.path.abspath(__file__))
ROOT = os.path.dirname(HERE)
W = 8.069  # the field's width (their welded official AprilTag layout's)
VMAX, AMAX, RMOD = 4.0, 4.6, 0.391  # their trajectory speed, torque-limited acceleration, module radius
STEP = 0.02


def wrap(a): return (a + math.pi) % (2 * math.pi) - math.pi


def catmull(p0, p1, p2, p3, n):
    """centripetal Catmull-Rom from p1 to p2, n+1 points"""
    def tj(ti, a, b): return ti + max(1e-6, math.dist(a, b)) ** 0.5
    t0 = 0.0; t1 = tj(t0, p0, p1); t2 = tj(t1, p1, p2); t3 = tj(t2, p2, p3)
    out = []
    for k in range(n + 1):
        t = t1 + (t2 - t1) * k / n
        def lerp(a, b, ta, tb): return [a[i] * (tb - t) / (tb - ta) + b[i] * (t - ta) / (tb - ta) for i in range(2)]
        a1, a2, a3 = lerp(p0, p1, t0, t1), lerp(p1, p2, t1, t2), lerp(p2, p3, t2, t3)
        b1, b2 = lerp(a1, a2, t0, t2), lerp(a2, a3, t1, t3)
        out.append(lerp(b1, b2, t1, t2))
    return out


def gen(req, mirror=False):
    """a path request (paths6328.json) -> [[x, y, heading deg, v, t]] every ~0.1 s (blue field)"""
    wps, cons = [], []  # waypoints; each one's constraints (the segment it ends, from the one before)
    for seg in req['segments']:
        for w in seg['waypoints']:
            wps.append(dict(w)); cons.append(seg)
    if req['stopAtStart']: wps[0]['stopped'] = True
    if req['stopAtEnd']: wps[-1]['stopped'] = True
    P = [w['xy'] for w in wps]
    # the curve, interval by interval
    pts, wi = [P[0]], [0]  # points, and which interval each is on
    for i in range(len(P) - 1):
        a, b = P[i], P[i + 1]
        L = math.dist(a, b)
        n = max(1, int(L / STEP))
        straight = cons[i + 1]['keepInLaneWidth'] > 0 or L < 1e-6
        if straight:
            seg = [[a[0] + (b[0] - a[0]) * k / n, a[1] + (b[1] - a[1]) * k / n] for k in range(n + 1)]
        else:
            p0 = P[i - 1] if i > 0 and math.dist(P[i - 1], a) > 1e-6 else [2 * a[0] - b[0], 2 * a[1] - b[1]]
            p3 = P[i + 2] if i + 2 < len(P) and math.dist(P[i + 2], b) > 1e-6 else [2 * b[0] - a[0], 2 * b[1] - a[1]]
            seg = catmull(p0, a, b, p3, n)
        pts += seg[1:]; wi += [i] * n
    s = [0.0]
    for k in range(1, len(pts)): s.append(s[-1] + math.dist(pts[k - 1], pts[k]))
    # where each waypoint is along it
    sw, k = [0.0], 0
    for i in range(1, len(P)):
        while k + 1 < len(pts) and wi[k + 1] < i: k += 1
        k += 1
        while k + 1 < len(pts) and wi[k] == i - 1 and wi[k + 1] == i - 1: k += 1
        sw.append(s[min(k, len(s) - 1)])
    # headings: turned evenly by distance between the waypoints that give one (point-at overrides)
    known = [(sw[i], math.radians(w['deg'])) for i, w in enumerate(wps) if 'deg' in w]
    if not known: known = [(0.0, math.atan2(P[-1][1] - P[0][1], P[-1][0] - P[0][0]))]
    H = []
    for k, sk in enumerate(s):
        if sk <= known[0][0]: h = known[0][1]
        elif sk >= known[-1][0]: h = known[-1][1]
        else:
            j = max(q for q in range(len(known)) if known[q][0] <= sk)
            (s0, h0), (s1, h1) = known[j], known[j + 1]
            h = h0 + wrap(h1 - h0) * ((sk - s0) / (s1 - s0) if s1 > s0 else 1.0)
        c = cons[wi[k] + 1] if wi[k] + 1 < len(cons) else cons[-1]
        if c.get('pointAt'):
            t = c['pointAt']['target']
            h = math.atan2(t[1] - pts[k][1], t[0] - pts[k][0]) + (math.pi if c['pointAt']['flip'] else 0)
        H.append(h)
    # turning in place where the curve has no length (a waypoint's heading changed on the spot)
    # the speed limit at each point
    N = len(pts)
    vlim = [VMAX] * N
    for k in range(N):
        c = cons[wi[k] + 1] if wi[k] + 1 < len(cons) else cons[-1]
        if c['maxVelocity'] >= 0: vlim[k] = min(vlim[k], c['maxVelocity'])
        if 0 < k < N - 1:
            # curvature from the turn of the direction over the two neighbours
            d1 = math.atan2(pts[k][1] - pts[k - 1][1], pts[k][0] - pts[k - 1][0])
            d2 = math.atan2(pts[k + 1][1] - pts[k][1], pts[k + 1][0] - pts[k][0])
            ds = (s[k + 1] - s[k - 1]) / 2
            if ds > 1e-6:
                kap = abs(wrap(d2 - d1)) / ds
                if kap > 1e-6: vlim[k] = min(vlim[k], math.sqrt(AMAX / kap))
                dh = abs(wrap(H[k + 1] - H[k - 1])) / (2 * ds)
                vlim[k] = min(vlim[k], VMAX / (1 + dh * RMOD))
                if c['maxAngularVelocity'] >= 0 and dh > 1e-6: vlim[k] = min(vlim[k], c['maxAngularVelocity'] / dh)
    for i, w in enumerate(wps):
        if w.get('stopped'):
            k = min(range(N), key=lambda q: abs(s[q] - sw[i]))
            vlim[k] = 0.0
    v = vlim[:]
    v[0] = 0.0 if wps[0].get('stopped') else min(v[0], VMAX)
    for k in range(1, N): v[k] = min(v[k], math.sqrt(v[k - 1] ** 2 + 2 * AMAX * (s[k] - s[k - 1])))
    for k in range(N - 2, -1, -1): v[k] = min(v[k], math.sqrt(v[k + 1] ** 2 + 2 * AMAX * (s[k + 1] - s[k])))
    # time: along the curve, and at least what the turn takes at the modules' share (a turn on the spot)
    t, T = 0.0, [0.0]
    for k in range(1, N):
        ds = s[k] - s[k - 1]
        dt = 2 * ds / (v[k] + v[k - 1]) if v[k] + v[k - 1] > 1e-6 else 0.0
        dh = abs(wrap(H[k] - H[k - 1]))
        c = cons[wi[k] + 1] if wi[k] + 1 < len(cons) else cons[-1]
        wmax = VMAX / RMOD if c['maxAngularVelocity'] < 0 else min(VMAX / RMOD, c['maxAngularVelocity'])
        dt = max(dt, dh / (0.5 * wmax)) if ds < 1e-4 else max(dt, dh / wmax)
        t += dt; T.append(t)
    # thinned: every ~0.1 s, and where it stops
    out, last = [], -1.0
    for k in range(N):
        if k == 0 or k == N - 1 or T[k] - last >= 0.1 or v[k] == 0.0:
            if out and T[k] - last < 1e-4 and k != N - 1: continue
            x, y, h = pts[k][0], pts[k][1], H[k]
            if mirror: y, h = W - y, -h
            out.append([round(x, 3), round(y, 3), round(math.degrees(wrap(h)), 1), round(v[k], 2), round(T[k], 3)])
            last = T[k]
    return out


# ------------------------------------------------------------------ their commands
# The programs js/auto6328.js runs (blue field, left side; mirrored for the right):
#   ['traj', {name, pts, coastX?, aim?}]  followTrajectory (DriveTrajectory: their kP 8 m/s per m,
#       4 rad/s per rad on its samples); coastX: past that x (toward the field's center) the drive
#       coasts (Drive.CoastRequest.ALWAYS_COAST) and it rolls on, to the end of AUTO; aim: their
#       followTrajectoryWhileAiming, launching on the move once aim (an x it's back behind) says so
#   ['rush', {t, slow}]  rushToCenter: full speed (0.6 of it slow) toward the CENTER LINE for t s
#   ['salesman', {t, bounds}]  SalesmanCommands.autoIntake for t s inside its zoning limits
#       (bounds: 'dynamic', their DYNAMIC_ETHICAL; 'full', 'left', 'right': FULL/LEFT/RIGHT_CLOSE)
#   ['turn', {}]  salesmanTurn: turn into the corner of the FUEL pool (1 s at most)
#   ['return', {untilX}]  returnToDeterminedLaunchPose (left bump), until back past untilX
#   ['launch', {target, hasty, leaveAt}]  returnLaunchAndIndexMindfully: drive to their launch pose
#       (target: leftBump / leftTrench) until past the starting line less fullBaseRadius, then aimed
#       where it is, indexing; done once launchTime (3 s, 1.5 hasty) is up and (leaveAt) AUTO's
#       clock has reached it
#   ['launchAt', {pose}]  driveToPose to a pose, indexing once within 0.1 m and 5 deg (Monopoly,
#       Timid); to the end of AUTO
#   ['pass', {t}]  passingCommand (BOTH): drive to a stationary aimed pose on its side of the CENTER
#       LINE and pass for t s once within 0.5 m and 15 deg
#   ['wait', {t}]
L = {}  # field lines and poses (from paths6328.json)


def prog_kachow(P, start, first='CONSERVATIVE', second='DAVIS_FRIENDSHIP', coast='HUB'):
    """AutoBuilder.kachowSalesman, return LEFT (their selector's first answers but the sweeps, the
    binder's sample timeline: Conservative, Davis Friendship, Return to NZ)"""
    trench = start == 'trench'
    out = []
    if not trench: out.append(['rush', {'t': 1.5}])
    sweep = {'CONSERVATIVE': ('leftTrenchStartConservativeSweep', 'leftBumpConservativeSweep'),
             'NEUTRAL': ('leftTrenchStartNeutralSweep', 'leftBumpNeutralSweep')}[first]
    out.append(['traj', {'name': sweep[0] if trench else sweep[1]}])
    out.append(['return', {'untilX': L['starting'] - L['fullBaseRadius'] + 0.1}])
    # (the coast's earliest leave: 21 s less the coast trajectory's time)
    coast_name = {'HUB': 'launchLeftBumpToFarHubKachow', 'BUMP': 'launchLeftBumpToFarBumpKachow', 'TRENCH': 'launchLeftBumpToFarTrenchKachow'}[coast]
    out.append(['launch', {'target': 'leftBump', 'leaveAt': None}])
    second_name = {'DAVIS': 'launchLeftBumpDavisSweepKachow', 'DAVIS_FRIENDSHIP': 'launchLeftBumpDavisSweepFriendshipKachow',
                   'CORIOLIS': 'launchLeftBumpCoriolisSweepKachow'}[second]
    out.append(['traj', {'name': second_name}])
    out.append(['return', {'untilX': L['starting'] - L['fullBaseRadius'] + 0.1}])
    out.append(['launch', {'target': 'leftBump', 'leaveAt': 21.0 - P[coast_name][-1][4]}])
    out.append(['traj', {'name': coast_name, 'coastX': L['neutralZoneNear']}])
    return out


def prog_semple(P, start):
    """AutoBuilder.sempleSalesman (commented out of their selector): CONSERVATIVE, then the second and
    third sweeps the salesman's, out through the TRENCH launching on the move, return LEFT"""
    trench = start == 'trench'
    out = [] if trench else [['rush', {'t': 1.5}]]
    out.append(['traj', {'name': 'leftTrenchStartConservativeSweep' if trench else 'leftBumpConservativeSweep'}])
    for _ in range(2):
        out.append(['return', {'untilX': L['starting'] - L['fullApothemX']}])
        # (lotmThroughNeutralZoneSweep SALESMAN: out through the TRENCH launching on the move, then the
        # salesman 3 s; their shouldAim: once back past the TRENCH's hood bounds, an estimate: x 4.0)
        out.append(['traj', {'name': 'launchLeftBumpThroughTrench', 'aim': L['starting']}])
        out.append(['salesman', {'t': 3.0, 'bounds': 'dynamic'}])
    out.append(['return', {'untilX': L['starting'] - L['fullApothemX']}])
    out.append(['launch', {'target': 'leftBump', 'leaveAt': None}])
    return out


def prog_substantial(P, start):
    """AutoBuilder.substantialSalesman: first sweep CONSERVATIVE, home over the BUMP, launch, through
    the DEPOT and back to launch, then coast (return LEFT_BUMP, coast HUB: over the BUMP)"""
    trench = start == 'trench'
    out = [] if trench else [['rush', {'t': 1.5}]]
    out.append(['traj', {'name': 'leftTrenchStartConservativeSweep' if trench else 'leftBumpConservativeSweep'}])
    out.append(['return', {'untilX': L['starting'] - L['fullBaseRadius'] + 0.1}])
    out.append(['launch', {'target': 'leftBump', 'leaveAt': None}])
    out.append(['traj', {'name': 'launchLeftBumpThroughDepotToLaunchLeftBump'}])
    out.append(['launch', {'target': 'leftBump', 'hasty': True, 'leaveAt': 21.0 - 1.5}])
    out.append(['rush', {'t': 1.5, 'slow': True, 'coastX': L['neutralZoneNear']}])
    out.append(['toPose', {'pose': [L['neutralZoneFar'] - L['fullApothemX'], W / 2, -15], 'coastX': L['neutralZoneNear']}])
    return out


def prog_monopoly(P, start):
    """AutoBuilder.monopolySalesman, post-launch SALESMAN (its salesmanCycle: the salesman out of the
    BUMP's side, home and launch, again)"""
    out = [['traj', {'name': 'trenchLeftStartThroughDepot' if start == 'trench' else 'bumpLeftInnerThroughDepot'}],
           ['launchAt', {'pose': L['launchLeftTower'], 'until': 'empty'}]]
    for _ in range(2):
        out.append(['rush', {'t': 1.5}])
        out.append(['salesman', {'t': 3.0, 'bounds': 'dynamic'}])
        out.append(['return', {'untilX': L['starting'] - L['fullBaseRadius'] + 0.1}])
        out.append(['launch', {'target': 'leftBump', 'leaveAt': None}])
    return out


def prog_benevolent(P, start):
    """AutoBuilder.benevolentSalesman: the first sweep the salesman's, then passing home from the
    NEUTRAL ZONE and the salesman again, over and over"""
    trench = start == 'trench'
    out = [] if trench else [['rush', {'t': 1.5}]]
    if trench: out.append(['traj', {'name': 'trenchLeftStart'}])
    out.append(['salesman', {'t': 2.5, 'bounds': 'dynamic'}])
    for _ in range(6):
        out.append(['pass', {'t': 3.0}])
        out.append(['salesman', {'t': 3.0, 'bounds': 'dynamic'}])
    return out


def prog_mellonomics(P, start):
    """AutoBuilder.mellonomicsSalesman: the salesman, a pass home, the salesman, home and launch, then
    out launching on the move into the second sweep (DAVIS_FRIENDSHIP) and home"""
    trench = start == 'trench'
    out = [] if trench else [['rush', {'t': 1.5}]]
    if trench: out.append(['traj', {'name': 'trenchLeftStart'}])
    out += [['salesman', {'t': 2.5, 'bounds': 'dynamic'}], ['pass', {'t': 3.0}], ['salesman', {'t': 3.0, 'bounds': 'dynamic'}],
            ['return', {'untilX': L['starting'] - L['fullApothemX']}],
            ['traj', {'name': 'launchLeftBumpDavisSweepFriendship', 'aim': L['starting']}],
            ['return', {'untilX': L['starting'] - L['fullApothemX']}], ['launch', {'target': 'leftBump', 'leaveAt': None}]]
    return out


def prog_timid(P, start):
    """AutoBuilder.timidSalesman: back 1 m (and 0.5 toward the middle from a TRENCH) and launch"""
    fx, fy = START[start][0], START[start][1]
    off = {'trench': (-1.0, -0.5), 'bump': (-1.0, 0.0), 'center': (-1.0, 0.0)}[start]
    x, y = fx + off[0], fy + off[1]
    h = math.degrees(math.atan2(y - L['hubTop'][1], x - L['hubTop'][0]))
    return [['launchAt', {'pose': [round(x, 3), round(y, 3), round(h, 1)], 'until': 'end'}]]


def prog_forward(P, start):
    """AutoBuilder.driveForward1mSalesman: 1 m forward (their DriveForward1m, from where it starts)"""
    return [['traj', {'name': 'DriveForward1m', 'relative': True}]]


START = {}


def autos_6328(path_json=os.path.join(HERE, 'vts6328', 'paths6328.json')):
    d = json.load(open(path_json))
    L.update(d['lines']); L.update(d['poses'])
    L['fullApothemX'] = 33 * 0.0254 / 2
    L['fullBaseRadius'] = math.hypot(L['fullApothemX'], 34 * 0.0254 / 4)
    START.update({'trench': (*d['poses']['trenchLeftStart'], -90.0), 'bump': (*d['poses']['bumpLeftInner'], 0.0),
                  'center': (*d['poses']['hubCenterStart'], 180.0)})
    out = []
    specs = [
        ('kachow', 'Kachow Salesman', prog_kachow, ['trench', 'bump'], 'AutoBuilder.kachowSalesman (Conservative, Davis Friendship, coast to the far HUB)'),
        ('substantial', 'Substantial Salesman', prog_substantial, ['trench', 'bump'], 'AutoBuilder.substantialSalesman (Conservative, return over the BUMP, coast HUB)'),
        ('monopoly', 'Monopoly Salesman', prog_monopoly, ['trench', 'bump'], 'AutoBuilder.monopolySalesman (post-launch: salesman)'),
        ('timid', 'Timid Salesman', prog_timid, ['trench', 'bump', 'center'], 'AutoBuilder.timidSalesman'),
        ('semple', 'Semple Salesman', prog_semple, ['trench', 'bump'], 'AutoBuilder.sempleSalesman (not in their selector at the end)'),
        ('benevolent', 'Benevolent Salesman', prog_benevolent, ['trench', 'bump'], 'AutoBuilder.benevolentSalesman (not in their selector at the end)'),
        ('mellonomics', 'Mellonomics Salesman', prog_mellonomics, ['trench', 'bump'], 'AutoBuilder.mellonomicsSalesman (not in their selector at the end)'),
        ('forward', 'Drive Forward 1m Salesman', prog_forward, ['bump'], 'AutoBuilder.driveForward1mSalesman (a test)'),
    ]
    paths = d['paths']
    P = {k: gen(v) for k, v in paths.items()}
    for key, name, prog, starts, src in specs:
        for start in starts:
            # (Substantial and Monopoly go through the DEPOT, on the left only: their selector offers
            # them from the left starts only)
            sides = ['center'] if start == 'center' else ['left'] if key in ('substantial', 'monopoly') else ['left', 'right']
            for side in sides:
                mirror = side == 'right'
                program = prog(P, start)
                # trajectories mirrored as their followTrajectory's mirror (VerticalFlipUtil) does
                for k, args in program:
                    if k == 'traj':
                        args['pts'] = gen(paths[args['name']], mirror)
                    for q in ('pose',):
                        if q in args:
                            p = args[q]
                            args[q] = [p[0], round(W - p[1], 4), -p[2]] if mirror else p
                    if k in ('launch',) and mirror: args['target'] = args['target'].replace('left', 'right')
                sx, sy, syaw = START[start]
                # (Timid resets to its own poses, facing back at the HUB; Monopoly to its first
                # trajectory's start, followTrajectory's start = true)
                if key == 'timid': sx, syaw = (sx - 33 * 0.0254 if start == 'trench' else sx), 180.0
                if key == 'monopoly': sx, sy, syaw = P[program[0][1]['name']][0][:3]
                if mirror: sy, syaw = W - sy, -syaw
                where = {'trench': 'Trench', 'bump': 'Bump', 'center': 'Center'}[start]
                out.append({'id': f'6328-{key}-{side}-{start}', 'name': f'{name} ({side.capitalize() + " " if side != "center" else ""}{where})',
                            'side': side, 'source': src, 'startFx': round(sx, 3), 'startFy': round(sy, 3), 'startYaw': syaw,
                            'shootPreload': False,
                            'points': [{'fx': round(sx, 3), 'fy': round(sy, 3), 'intake': True, 'shoot': 'off', 'v': 0, 'action': 'none',
                                        'step': {'type': 'team6328', 'mirror': mirror, 'program': program}}]})
    return out


def write(autos):
    """6328's entry into js/teamAutos.js, in place (the rest of the file as tools/team-autos.py wrote it)"""
    path = os.path.join(ROOT, 'js', 'teamAutos.js')
    src = open(path).read()
    line = "TEAM_AUTOS['6328'] = " + json.dumps(autos, separators=(',', ':')) + ';\n'
    lines = [x for x in src.splitlines(keepends=True) if not x.startswith("TEAM_AUTOS['6328']")]
    i = next(k for k, x in enumerate(lines) if x.startswith('function unpack'))
    lines.insert(i, line)
    open(path, 'w').write(''.join(lines))


if __name__ == '__main__':
    a = autos_6328()
    write(a)
    for x in a:
        T = sum(args['pts'][-1][4] for k, args in x['points'][0]['step']['program'] if k == 'traj')
        print(x['id'], x['startFx'], x['startFy'], x['startYaw'], [k for k, _ in x['points'][0]['step']['program']], f'trajectories {T:.1f} s')
