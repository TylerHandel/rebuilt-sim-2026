#!/usr/bin/env python3
"""The teams' own AUTOs, from their robot code, as waypoints the sim's AutoRunner drives
(js/customAutos.js's format, plus each one's start, speeds in m/s and timed actions).
Writes js/teamAutos.js, and prints each PathPlanner path's length and time, and which of its
trajectories it ran (its ideal one, or one regenerated from the robot as it started it).

  python3 tools/team-autos.py [--8793 DIR] [--2910 DIR] [--1678 DIR] [--604 DIR] [--1114 DIR]

(581's are transcribed from their code, team581/frc-2026 comp-bot: see auto_581)

DIR: a checkout of the team's code (defaults: ../pumpkin-bots/2026_robot,
../frcteam2910/2026competitionrobot-public, ../frc1678/C2026-Public,
../frc604/2026-public/FRC-2026, ../Simbotics/2026-Simbot-Tim). All of them use WPILib's
blue-origin field coordinates (x from the blue ALLIANCE WALL, y from the blue drivers' right),
which are the sim's blue fx, fy.

- 8793 (PathPlanner): each .auto's command tree; each path's trajectory as PathPlannerLib
  2026.1.2 generates it (pp_path: a port of its Java, see the PathPlanner section: positions along
  the Bezier curves, the module-level speed profile with its constraints and constraint zones,
  the holonomic rotation cosine-interpolated by distance between the rotation targets, the speed
  slowed where that turns faster than the max angular velocity lets it), the ideal one or one
  generated from the robot's speed and rotation as their FollowPathCommand picks; chained paths on
  one clock; each state the sim needs, its time (dt), speed and rotation (rot). The AUTO starts at
  the first path's starting holonomic pose (resetOdom: -178.5, 8793 backward, on the starting
  line under the TRENCH's arm, its stowed intake clear of it). Their modes: TrenchMode (intake
  down, not shooting), ShuttleMode (intake, shuttling FUEL home from anywhere), ShooterMode
  (intake; shoots at the HUB in the ALLIANCE ZONE, shuttles from anywhere else).
- 2910 (AutoFactory, driveToPoint / driveToPointWhileIntaking to FieldConstants poses): their
  drive-to-point controller replayed (speed 3 x distance, capped at each command's 15 / 11 / 5
  ft/s, the next command once within its tolerance), so a big tolerance cuts the corner as on the
  robot. scoreStationary shoots until the intake has retracted (at most 4 s). Pose values are
  transcribed from their FieldConstants.java (the right side mirrors the left).
- 1678 (Choreo): each routine's trajectories' samples as they are (as many as the sim needs to
  keep within 1 cm and 0.5 deg of Choreo's own interpolation between them), at their speed,
  heading and time there; driveOverBump (3 m/s straight back over the BUMP to where the next
  trajectory starts, turned to its heading) and fireCommand (3 s) between them.
- 604 (Quikplan): every competition AUTO in their chooser (the Champs ones first, its default,
  Double Cycle Inwards, first of all; then the ones commented out from earlier events), each a
  sequence of FollowQuikplan partials (their time-optimal trajectories' 50 Hz samples, as many as
  the sim needs to keep within 1 cm and 0.5 deg of their reader's linear interpolation, at their
  speed, heading and time there, with the actions along them: launch on/off, intake down and
  rolling, stowed),
  waits and RetractIntake, from the right side and mirrored left as their isLeft does. A wait
  holds the robot where it is, launching if the last action left it launching.
- 1114 (PathPlanner): their chooser's first, Right / Left 1-1 Middle Sweep : Greedy Sweep (the
  left mirrored, as their chooser does), and Left Slow Centre Sweep Plus Depot. Each path's
  PathPlannerLib trajectory (as 8793's; their RobotConfig from Drive.PP_CONFIG), intaking over its
  IntakeDeploy zones; each
  FullShoot from x = 3.1 m (their autonXShootPose): aiming until aimed and stopped (at most its
  aiming time), then shooting for its timeout; the flywheel spun up from each AimShooterHub
  marker; no stop where an IntakeDeploy zone starts or ends.
- 581 (their Trailblazer state machines, LeftNormalAuto / RightNormalAuto): their follower
  replayed over their segments' poses into a time-parameterized path (auto_581).

The time-parameterized ones (PathPlanner, Choreo, Quikplan, 581's: timed()) run in the sim by their
trajectories' clock, as their followers do (js/auto.js, AutoRunner._timed): the robot chases where
the trajectory is by now with their controllers' P gains, and each path ends when its time is up,
wherever the robot is. 2910's (drive to point) give each point a time limit instead.
"""
import argparse, bisect, json, math, os, re

HERE = os.path.dirname(os.path.abspath(__file__))
ROOT = os.path.dirname(HERE)
W = 8.069  # the field's width (m): the 2026 AprilTag layout's, as all three use
STEP = 0.35  # spacing of the waypoints along a path (m)


def pt(x, y, intake, shoot, v, action='none', t=None, spin=False, dt=None, rot=None):
    p = {'fx': round(x, 3), 'fy': round(y, 3), 'intake': intake, 'shoot': shoot, 'v': round(v, 2), 'action': action}
    if t is not None:
        p['t'] = t
    if spin:
        p['spin'] = True  # the flywheel up to its HUB shot on the way here (not shooting), intaking or not
    if dt is not None:
        p['dt'] = round(dt, 3)  # (a timed path's: the time its trajectory takes from the point before to here)
    if rot is not None:
        p['rot'] = round(wrap_deg(rot), 1)  # (the chassis' heading there, deg, blue field frame: 0 toward the red ALLIANCE WALL, 90 to the left)
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


# ------------------------------------------------------------------ PathPlanner (8793, 1114)
# Their paths' trajectories as PathPlannerLib makes them: a port of its Java, 2026.1.2 (both teams'
# vendordep), so the sim runs the trajectory their robots run. PathPlannerPath.createPath: points
# about every 0.2 m along the Bezier curves (by waypoint-relative position, 0.05 at a time, then
# corrected toward the spacing), a point inserted where each rotation target is, and more where the
# curve's radius is under 0.5 m. PathPlannerTrajectory: each point's holonomic rotation from the
# robot's starting rotation through the rotation targets to the goal end state's, cosine
# interpolated between one and the next by the distance along the path (so the turn is fastest
# midway between targets); then the speed, worked out per swerve module (each module's own path
# over the ground, which the turning adds to): a forward pass speeding each module up as its motor
# (current-limited, torque loss at speed) and the wheels' grip (wheelCOF) let it, no faster than
# its grip lets it round its own curve, the four kept in step (the same time to the next point),
# the chassis speed and turn rate desaturated to the point's constraints (max velocity, and max
# angular velocity: so where the rotation turns fast the robot slows to let it); and a reverse
# pass slowing it to the goal end velocity the same way; the time between points from their
# distance and the speeds at each end. Rotation isn't profiled on its own: max angular
# acceleration only caps the angular part of each module's acceleration. Between states its
# sample() interpolates the speed linearly and the rotation linearly by time, which is how the sim's
# follower interpolates between the points it's given (js/auto.js _timed), so the points are the
# trajectory's states, thinned to those it needs to stay within 1 cm and 0.5 deg of them.
# FollowPathCommand: a path with an ideal starting state runs its ideal trajectory (generated from
# that state) if the robot is within 0.25 m/s and 30 deg of it as the path starts, else one
# generated from the robot's speed and rotation then.
PP_SPACING, PP_INCREMENT = 0.2, 0.05
GRAV = 9.8  # (PathPlannerLib's)


class PPMotor:
    """WPILib's DCMotor, one motor through a reduction"""
    def __init__(self, stall_torque, stall_current, free_current, free_rpm, reduction):
        self.R = 12.0 / stall_current
        free = free_rpm * 2 * math.pi / 60 / reduction
        self.Kv = free / (12.0 - self.R * free_current)
        self.Kt = stall_torque * reduction / stall_current

    def current(self, w, volts): return -w / self.Kv / self.R + volts / self.R
    def torque(self, amps): return self.Kt * amps


PP_MOTORS = {'krakenX60': (7.09, 366, 2, 6000), 'krakenX60FOC': (9.37, 483, 2, 5800)}


class PPConfig:
    """PathPlannerLib's RobotConfig (a swerve: four modules, one drive motor each)"""
    def __init__(self, mass, moi, wheel_r, max_v, cof, motor, current_limit, modules):
        self.mass, self.moi, self.wheel_r, self.max_v, self.modules = mass, moi, wheel_r, max_v, modules
        self.motor, self.current_limit = motor, current_limit
        self.torque_loss = max(motor.torque(min(motor.current(max_v / wheel_r, 12.0), current_limit)), 0.0)
        self.friction_force = cof * (mass / 4 * GRAV)
        self.max_torque_friction = self.friction_force * wheel_r
        self.pivot = [math.hypot(x, y) for x, y in modules]
        # (forward kinematics: the least-squares chassis speeds for the module vectors)
        A = [[4.0, 0.0, -sum(y for x, y in modules)], [0.0, 4.0, sum(x for x, y in modules)], [0.0, 0.0, sum(x * x + y * y for x, y in modules)]]
        A[2][0], A[2][1] = A[0][2], A[1][2]
        self.ata = A

    @staticmethod
    def gui(settings):
        """RobotConfig.fromGUISettings (their settings.json)"""
        s = json.load(open(settings))
        m = PP_MOTORS[s['driveMotorType']]
        mods = [(s['flModuleX'], s['flModuleY']), (s['frModuleX'], s['frModuleY']), (s['blModuleX'], s['blModuleY']), (s['brModuleX'], s['brModuleY'])]
        return PPConfig(s['robotMass'], s['robotMOI'], s['driveWheelRadius'], s['maxDriveSpeed'], s['wheelCOF'],
                        PPMotor(*m, s['driveGearing']), s['driveCurrentLimit'], mods)

    def to_modules(self, vx, vy, w):
        """robot-relative chassis speeds -> each module's (speed, angle)"""
        out = []
        for x, y in self.modules:
            mx, my = vx - w * y, vy + w * x
            out.append((math.hypot(mx, my), math.atan2(my, mx)))
        return out

    def to_chassis(self, mods):
        """[(speed, robot-relative angle)] -> robot-relative (vx, vy, omega)"""
        b = [0.0, 0.0, 0.0]
        for (x, y), (s, a) in zip(self.modules, mods):
            u, v = s * math.cos(a), s * math.sin(a)
            b[0] += u; b[1] += v; b[2] += -y * u + x * v
        return solve3(self.ata, b)


def solve3(A, b):
    (a, bb, c), (d, e, f), (g, h, i) = A
    det = a * (e * i - f * h) - bb * (d * i - f * g) + c * (d * h - e * g)
    x = (b[0] * (e * i - f * h) - bb * (b[1] * i - f * b[2]) + c * (b[1] * h - e * b[2])) / det
    y = (a * (b[1] * i - f * b[2]) - b[0] * (d * i - f * g) + c * (d * b[2] - b[1] * g)) / det
    z = (a * (e * b[2] - b[1] * h) - bb * (d * b[2] - b[1] * g) + b[0] * (d * h - e * g)) / det
    return x, y, z


def rwrap(a):
    return math.atan2(math.sin(a), math.cos(a))


def rlerp(a, b, t):
    """Rotation2d.interpolate: the short way round"""
    return rwrap(a + rwrap(b - a) * min(1.0, max(0.0, t)))


def rotate(x, y, a):
    c, s = math.cos(a), math.sin(a)
    return x * c - y * s, x * s + y * c


def curve_radius(a, b, c):
    """GeometryUtil.calculateRadius (signed; inf for a straight line)"""
    vba, vbc = (a[0] - b[0], a[1] - b[1]), (c[0] - b[0], c[1] - b[1])
    sign = 1 if vba[0] * vbc[1] - vba[1] * vbc[0] < 0 else -1
    ab, bc, ac = math.dist(a, b), math.dist(b, c), math.dist(a, c)
    p = (ab + bc + ac) / 2
    area = math.sqrt(abs(p * (p - ab) * (p - bc) * (p - ac)))
    return sign * ab * bc * ac / (4 * area) if area > 0 else math.inf


def pp_constraints(c):
    """(max velocity, max acceleration, max angular velocity, max angular acceleration (rad), nominal volts)"""
    if c.get('unlimited'):
        return (math.inf, math.inf, math.inf, math.inf, c['nominalVoltage'])
    return (c['maxVelocity'], c['maxAcceleration'], math.radians(c['maxAngularVelocity']), math.radians(c['maxAngularAcceleration']), c['nominalVoltage'])


class PPPath:
    """PathPlannerPath (from its .path file): its points (PathPoint: position, waypoint-relative
    position, rotation target, constraints, distance along the path)"""
    def __init__(self, p):
        if p.get('pointTowardsZones'): raise NotImplementedError('point towards zones')
        P = lambda q: (q['x'], q['y']) if q else None
        self.wp = [(P(w['prevControl']), P(w['anchor']), P(w['nextControl'])) for w in p['waypoints']]
        self.gc = pp_constraints(p['globalConstraints'])
        self.zones = [(z['minWaypointRelativePos'], z['maxWaypointRelativePos'], pp_constraints(z['constraints'])) for z in p.get('constraintZones', [])]
        self.targets = sorted(((t['waypointRelativePos'], math.radians(t['rotationDegrees'])) for t in p.get('rotationTargets', [])), key=lambda t: t[0])
        self.goal_v, self.goal_rot = p['goalEndState']['velocity'], math.radians(p['goalEndState']['rotation'])
        ideal = p.get('idealStartingState')
        self.ideal = (ideal['velocity'], math.radians(ideal['rotation'])) if ideal else None
        self.events = p.get('eventMarkers', [])
        self.points = self._create()
        self._precalc()

    def sample(self, pos):
        pos = min(max(pos, 0.0), len(self.wp) - 1.0)
        i = min(int(math.floor(pos)), len(self.wp) - 2)
        t = pos - i
        a, b, c, d = self.wp[i][1], self.wp[i][2], self.wp[i + 1][0], self.wp[i + 1][1]
        L = lambda p, q: (p[0] + (q[0] - p[0]) * t, p[1] + (q[1] - p[1]) * t)
        p0, p1 = L(L(a, b), L(b, c)), L(L(b, c), L(c, d))
        return L(p0, p1)

    def cons(self, pos):
        return next((c for lo, hi, c in self.zones if lo <= pos <= hi), self.gc)

    def _pt(self, pos, target=None, cons=None):
        return {'xy': self.sample(pos), 'rel': pos, 'target': target, 'cons': cons or self.cons(pos)}

    def _create(self):
        un, n = list(self.targets), len(self.wp) - 1
        pts = [self._pt(0.0)]
        pos = PP_INCREMENT
        while pos < n:
            xy = self.sample(pos)
            dist = math.dist(pts[-1]['xy'], xy)
            if dist <= 0.01:
                pos = min(pos + PP_INCREMENT, n)
                continue
            prev_pos = pos - PP_INCREMENT
            delta = dist - PP_SPACING
            if delta > PP_SPACING * 0.25:
                ci = PP_SPACING * PP_INCREMENT / dist
                pos = pos - PP_INCREMENT + ci
                xy = self.sample(pos)
                if math.dist(pts[-1]['xy'], xy) - PP_SPACING > PP_SPACING * 0.25:
                    pos -= ci * 0.5
                    xy = self.sample(pos)
            elif delta < -PP_SPACING * 0.25:
                ci = PP_SPACING * PP_INCREMENT / dist
                pos = pos - PP_INCREMENT + ci
                xy = self.sample(pos)
                if math.dist(pts[-1]['xy'], xy) - PP_SPACING < -PP_SPACING * 0.25:
                    pos += ci * 0.5
                    xy = self.sample(pos)
            target, prev = None, pts[-1]
            while un and prev_pos <= un[0][0] <= pos:
                if abs(un[0][0] - prev_pos) < 0.001:
                    prev['target'] = un.pop(0)
                elif abs(un[0][0] - pos) < 0.001:
                    target = un.pop(0)
                else:
                    t = un.pop(0)
                    pts.append(self._pt(t[0], t))
            pts.append({'xy': xy, 'rel': pos, 'target': target, 'cons': self.cons(pos)})
            pos = min(pos + PP_INCREMENT, n)
        # (the end point, tried until it's close enough to the one before)
        true_inc = n - (pos - PP_INCREMENT)
        pos = n
        while True:
            xy = self.sample(pos)
            dist = math.dist(pts[-1]['xy'], xy)
            if dist <= 0.01:
                if len(pts) < 2: pts.append(self._pt(pos))
                break
            prev_pos = pos - true_inc
            invalid = True
            if dist - PP_SPACING > PP_SPACING * 0.25:
                ci = PP_SPACING * true_inc / dist
                pos = pos - true_inc + ci
                true_inc = ci
                xy = self.sample(pos)
                if math.dist(pts[-1]['xy'], xy) - PP_SPACING > PP_SPACING * 0.25:
                    pos -= ci * 0.5
                    true_inc = ci * 0.5
                    xy = self.sample(pos)
            else:
                invalid = False
            if un and abs(un[0][0] - prev_pos) <= abs(un[0][0] - pos):
                pts[-1]['target'] = un.pop(0)
            pts.append({'xy': xy, 'rel': pos, 'target': None, 'cons': self.cons(pos)})
            pos = n
            if not invalid: break
        # (more points round a tight curve; as their Java loop does, over the list as it grows)
        i = 1
        while i < len(pts) - 1:
            r = curve_radius(pts[i - 1]['xy'], pts[i]['xy'], pts[i + 1]['xy'])
            if math.isfinite(r) and abs(r) < 0.5:
                c = pts[i]['cons']
                lerp = lambda a, b, t: a + (b - a) * t
                if abs(r) < 0.25:
                    b1, b2 = lerp(pts[i - 1]['rel'], pts[i]['rel'], 0.33), lerp(pts[i - 1]['rel'], pts[i]['rel'], 0.67)
                    a1, a2 = lerp(pts[i]['rel'], pts[i + 1]['rel'], 0.33), lerp(pts[i]['rel'], pts[i + 1]['rel'], 0.67)
                    pts[i:i + 1] = [self._pt(b1, cons=c), self._pt(b2, cons=c), pts[i], self._pt(a1, cons=c), self._pt(a2, cons=c)]
                    i += 4
                else:
                    b1, a1 = lerp(pts[i - 1]['rel'], pts[i]['rel'], 0.5), lerp(pts[i]['rel'], pts[i + 1]['rel'], 0.5)
                    pts[i:i + 1] = [self._pt(b1, cons=c), pts[i], self._pt(a1, cons=c)]
                    i += 2
            i += 1
        return pts

    def _precalc(self):
        d = 0.0
        for i, p in enumerate(self.points):
            if i: d += math.dist(self.points[i - 1]['xy'], p['xy'])
            p['dist'] = d
        self.points[-1]['target'] = (-1, self.goal_rot)


def pp_trajectory(path, cfg, v_robot, rot0):
    """PathPlannerTrajectory(path, startingSpeeds (robot-relative (vx, vy, omega)), startingRotation,
    config): its states, each {xy, rot, t, v (linear), vx, vy, omega (field), rel}"""
    pts, M = path.points, cfg.modules
    n = len(pts)
    # -- generateStates
    S = []
    prev_i, prev_rot = 0, rot0
    nxt = lambda s: next((k for k in range(s, n - 1) if pts[k]['target'] is not None), n - 1)
    next_i = nxt(0)
    next_rot = pts[next_i]['target'][1]
    for i, p in enumerate(pts):
        if i > next_i:
            prev_i, prev_rot = next_i, next_rot
            next_i = nxt(i)
            next_rot = pts[next_i]['target'][1]
        den = pts[next_i]['dist'] - pts[prev_i]['dist']
        t = (p['dist'] - pts[prev_i]['dist']) / den if den else math.nan
        t2 = (1.0 - math.cos(t * math.pi)) / 2.0 if not math.isnan(t) else 1.0
        rot = rlerp(prev_rot, next_rot, t2)
        st = {'xy': p['xy'], 'rot': rot, 'cons': p['cons'], 'rel': p['rel'], 'dpos': 0.0, 't': 0.0}
        if i != n - 1:
            hx, hy = pts[i + 1]['xy'][0] - p['xy'][0], pts[i + 1]['xy'][1] - p['xy'][1]
            st['heading'] = math.atan2(hy, hx) if math.hypot(hx, hy) > 1e-6 else 0.0
        else:
            st['heading'] = S[i - 1]['heading']
        if i: st['dpos'] = math.dist(p['xy'], S[i - 1]['xy'])
        st['mods'] = []
        for m, (mx, my) in enumerate(M):
            ox, oy = rotate(mx, my, rot)
            mod = {'pos': (p['xy'][0] + ox, p['xy'][1] + oy), 'speed': 0.0, 'dpos': 0.0}
            if i: mod['dpos'] = math.dist(mod['pos'], S[i - 1]['mods'][m]['pos'])
            st['mods'].append(mod)
        S.append(st)
    for i, st in enumerate(S):
        for m, mod in enumerate(st['mods']):
            if i != n - 1:
                q = S[i + 1]['mods'][m]['pos']
                dx, dy = q[0] - mod['pos'][0], q[1] - mod['pos'][1]
                mod['fang'] = math.atan2(dy, dx) if math.hypot(dx, dy) > 1e-6 else 0.0
            else:
                mod['fang'] = S[i - 1]['mods'][m]['fang']
            mod['ang'] = rwrap(mod['fang'] - st['rot'])
    # -- the start: the robot's speeds
    fvx, fvy = rotate(v_robot[0], v_robot[1], S[0]['rot'])
    for mod, (s, _) in zip(S[0]['mods'], cfg.to_modules(fvx, fvy, v_robot[2])):  # (their Java passes the field speeds here)
        mod['speed'] = s
    S[0]['f'] = (fvx, fvy, v_robot[2])
    S[0]['v'] = math.hypot(fvx, fvy)

    def accel_pass(i, prev, st, nxt_st, reverse):
        lf, tq = [0.0, 0.0], 0.0
        for m in range(4):
            last = (nxt_st if reverse else prev)['mods'][m]['speed']
            amps = min(cfg.motor.current(last / cfg.wheel_r, st['cons'][4]), cfg.current_limit)
            avail = cfg.motor.torque(amps) - (0.0 if reverse else cfg.torque_loss)
            avail = min(avail, cfg.max_torque_friction)
            force = avail / cfg.wheel_r
            fa = st['mods'][m]['fang'] + (math.pi if reverse else 0.0)
            fx, fy = force * math.cos(fa), force * math.sin(fa)
            lf[0] += fx; lf[1] += fy
            mp = st['mods'][m]['pos']
            to_mod = math.atan2(mp[1] - st['xy'][1], mp[0] - st['xy'][0])
            theta = (math.atan2(fy, fx) if math.hypot(fx, fy) > 1e-6 else 0.0) - to_mod
            tq += force * cfg.pivot[m] * math.sin(theta)
        amax_ang = st['cons'][3]
        ang_acc = min(max(tq / cfg.moi, -amax_ang), amax_ang)
        ax, ay = lf[0] / cfg.mass, lf[1] / cfg.mass
        a = math.hypot(ax, ay)
        if a > st['cons'][1]:
            ax, ay = ax * st['cons'][1] / a, ay * st['cons'][1] / a
        rax, ray = rotate(ax, ay, -st['rot'])
        acc_mods = cfg.to_modules(rax, ray, ang_acc)
        for m in range(4):
            ma = abs(acc_mods[m][0])
            mod = st['mods'][m]
            if not reverse:
                mod['speed'] = math.sqrt(abs(prev['mods'][m]['speed'] ** 2 + 2 * ma * mod['dpos']))
                r = curve_radius(prev['mods'][m]['pos'], mod['pos'], nxt_st['mods'][m]['pos'])
                if math.isfinite(r):
                    mod['speed'] = min(mod['speed'], math.sqrt(cfg.friction_force * abs(r) / (cfg.mass / 4)))
            else:
                vmax = math.sqrt(abs(nxt_st['mods'][m]['speed'] ** 2 + 2 * ma * nxt_st['mods'][m]['dpos']))
                mod['speed'] = min(vmax, mod['speed'])
        # (the four in step: each takes the slowest one's time to the next state, but one turning
        # 60 deg or more from the state before)
        before = prev if not reverse else S[i - 1]
        max_dt = real_dt = 0.0
        for m in range(4):
            sp = st['mods'][m]['speed']
            if sp == 0: continue
            dt = nxt_st['mods'][m]['dpos'] / sp
            if math.isfinite(dt):
                real_dt = max(dt, real_dt)
                if abs(math.degrees(rwrap(st['mods'][m]['ang'] - before['mods'][m]['ang']))) < 60:
                    max_dt = max(dt, max_dt)
        if max_dt == 0.0: max_dt = real_dt
        if max_dt > 0:
            for m in range(4):
                if abs(math.degrees(rwrap(st['mods'][m]['ang'] - before['mods'][m]['ang']))) >= 60: continue
                st['mods'][m]['speed'] = nxt_st['mods'][m]['dpos'] / max_dt
        des = cfg.to_chassis([(md['speed'], md['ang']) for md in st['mods']])
        vmax_c, wmax_c = st['cons'][0], st['cons'][2]
        if reverse:
            vmax_c = min(vmax_c, st['v'])
            wmax_c = min(wmax_c, abs(st['f'][2]))
        # (desaturateWheelSpeeds)
        real = max(abs(md['speed']) for md in st['mods'])
        if real != 0:
            tp = math.hypot(des[0], des[1]) / vmax_c if abs(vmax_c) > 1e-8 else 0.0
            rp = abs(des[2]) / abs(wmax_c) if abs(wmax_c) > 1e-8 else 0.0
            scale = min(1.0, cfg.max_v / real)
            if max(tp, rp) > 0: scale = min(scale, 1.0 / max(tp, rp))
            for md in st['mods']: md['speed'] *= scale
        cvx, cvy, cw = cfg.to_chassis([(md['speed'], md['ang']) for md in st['mods']])
        fx, fy = rotate(cvx, cvy, st['rot'])
        st['f'] = (fx, fy, cw)
        st['v'] = math.hypot(fx, fy)

    for i in range(1, n - 1):
        accel_pass(i, S[i - 1], S[i], S[i + 1], False)
    # -- the end: the goal end state's velocity
    end = S[-1]
    ex, ey = path.goal_v * math.cos(end['heading']), path.goal_v * math.sin(end['heading'])
    rx, ry = rotate(ex, ey, -end['rot'])
    for mod, (s, _) in zip(end['mods'], cfg.to_modules(rx, ry, 0.0)):
        mod['speed'] = s
    end['f'], end['v'] = (ex, ey, 0.0), path.goal_v
    for i in range(n - 2, 0, -1):
        accel_pass(i, None, S[i], S[i + 1], True)
    # -- the times
    for i in range(1, n):
        a, b = S[i - 1], S[i]
        sv = a['v'] + b['v']
        b['t'] = a['t'] if abs(sv) < 1e-6 or abs(b['dpos']) < 1e-6 else a['t'] + 2 * b['dpos'] / sv
    return S


def pp_follow(cfg, path, robot):
    """FollowPathCommand: the trajectory it runs for path, with the robot (rot, field vx, vy, omega)
    as it starts it: the ideal one if the robot's within 0.25 m/s and 30 deg of the ideal starting
    state, else one generated from the robot's speeds"""
    rot, vx, vy, w = robot
    if path.ideal and abs(math.hypot(vx, vy) - path.ideal[0]) <= 0.25 and abs(math.degrees(rwrap(rot - path.ideal[1]))) <= 30:
        # (getIdealTrajectory: the ideal velocity along the path's initial heading)
        h = math.atan2(path.points[1]['xy'][1] - path.points[0]['xy'][1], path.points[1]['xy'][0] - path.points[0]['xy'][0])
        sx, sy = rotate(path.ideal[0] * math.cos(h), path.ideal[0] * math.sin(h), -path.ideal[1])
        return pp_trajectory(path, cfg, (sx, sy, 0.0), path.ideal[1]), 'ideal'
    sx, sy = rotate(vx, vy, -rot)
    return pp_trajectory(path, cfg, (sx, sy, w), rot), 'regenerated'


def sim_interp(a, b, t):
    """where the sim's follower puts the setpoint at time t between points a and b (js/auto.js _timed):
    along the straight line at an even acceleration from a's speed to b's, the rotation turning evenly"""
    h = b['t'] - a['t']
    w = min(1.0, max(0.0, (t - a['t']) / h)) if h > 1e-6 else 1.0
    v0, v1 = a['v'], b['v']
    vm = (v0 + v1) / 2
    f = (v0 * w + 0.5 * (v1 - v0) * w * w) / vm if vm > 1e-6 else w
    return (a['xy'][0] + (b['xy'][0] - a['xy'][0]) * f, a['xy'][1] + (b['xy'][1] - a['xy'][1]) * f), a['rot'] + wrap_deg(b['rot'] - a['rot']) * w


def thin(S, keep=lambda i: False, exact=None, tol=0.01, tol_deg=0.5):
    """the states the sim needs (S: a trajectory's states or samples, each {xy, v, rot (deg), t}):
    from each kept one, on to the furthest that the sim's interpolation to it (sim_interp) keeps
    within tol (m) and tol_deg of the trajectory, every 0.02 s and at every state between. exact(t):
    the trajectory's own (xy, rot) at t (by default, its states interpolated as sim_interp does);
    keep(i): a state kept anyway. Returns their indices."""
    ts = [q['t'] for q in S]
    if exact is None:
        def exact(t):
            k = max(0, min(len(S) - 2, bisect.bisect_right(ts, t) - 1))
            return sim_interp(S[k], S[k + 1], t)
    out, i = [0], 0
    while i < len(S) - 1:
        j = i + 1
        while j + 1 < len(S) and not keep(j):
            k = j + 1
            at = [S[q]['t'] for q in range(i + 1, k)]
            t = S[i]['t'] + 0.02
            while t < S[k]['t']: at.append(t); t += 0.02
            ok = True
            for t in at:
                xy, r = sim_interp(S[i], S[k], t)
                sxy, sr = exact(t)
                if math.dist(xy, sxy) > tol or abs(wrap_deg(r - sr)) > tol_deg: ok = False; break
            if not ok: break
            j = k
        out.append(j)
        i = j
    return out


# PathPlannerTrajectory.sample()'s position between two states is their speed integrated from the
# first, 0.01 s at a time, and its loop leaves out the first 0.01 s (in their Java as in its Python):
# the target pose it hands the controller is where the trajectory was 0.01 s earlier, its speeds and
# rotation where they are now. The sim's follower does the same (posLag).
PP_POS_LAG = 0.01


PATH_TIMES = []  # (team's folder, path, how it ran (ideal / regenerated), its length and time, end speed)


def pp_path(d, name, cfg, robot, marker=None, onward=None):
    """The trajectory their FollowPathCommand runs for a path, starting with the robot as robot (rot
    (rad), field vx, vy, omega): its states the sim needs (each {xy, v, rot (deg), t: from the
    path's start}); with marker (an event's name), on: whether one of that event's zones (zoned
    event markers: from its start to its end) covers it; with onward (a point event's name), past:
    whether it's past that event (a command started there runs on to the path's end). Returns
    (states, the robot as it ends: its rotation and field speeds)."""
    path = PPPath(json.load(open(os.path.join(d, 'paths', name + '.path'))))
    S, how = pp_follow(cfg, path, robot)
    end = S[-1]
    PATH_TIMES.append((d, name, how, path.points[-1]['dist'], end['t'], end['v']))
    ev = [(e['waypointRelativePos'], e.get('endWaypointRelativePos')) for e in path.events if e['name'] == marker]
    for s in S:
        # (+0.01: zones end to end, as 1114's IntakeDeploys are 0.001 apart, run as one)
        s['on'] = bool(marker) and any(hi is not None and lo <= s['rel'] <= hi + 0.01 for lo, hi in ev)
        s['past'] = any(e['name'] == onward and s['rel'] >= e['waypointRelativePos'] for e in path.events)
    T = [dict(s, rot=math.degrees(s['rot'])) for s in S]
    idx = thin(T, keep=lambda i: T[i]['on'] != T[i - 1]['on'] or T[i]['past'] != T[i - 1]['past'])
    return [T[i] for i in idx], (end['rot'], end['f'][0], end['f'][1], end['f'][2])


def wrap_deg(a):
    return (a + 180.0) % 360.0 - 180.0


MODES_8793 = {
    None: (False, 'off'),             # a path on its own: nothing running
    'TrenchMode': (False, 'off'),     # intake to the TRENCH position, rollers off, flywheel off
    'ShuttleMode': (True, 'always'),  # intake running, shuttling home from anywhere
    'ShooterMode': (True, 'always'),  # intake running; the HUB in our zone, shuttling elsewhere
}


class Clock:
    """each point's dt from the trajectory's time there, rounded so their sum keeps to its clock"""
    def __init__(self): self.t = self.out = 0.0
    def to(self, t):
        self.t = t
        dt = round(t - self.out, 3)
        self.out += dt
        return dt
    def restart(self): self.t = self.out = 0.0


def auto_8793(d, file):
    a = json.load(open(os.path.join(d, 'autos', file + '.auto')))
    cfg = PPConfig.gui(os.path.join(d, 'settings.json'))
    pts, start, robot, clock, t0 = [], None, None, Clock(), 0.0
    for c in a['command']['data']['commands']:
        t, data = c['type'], c.get('data', {})
        kids = data.get('commands', []) if t in ('race', 'parallel', 'deadline', 'sequential') else [c]
        path = next((k['data']['pathName'] for k in kids if k['type'] == 'path'), None)
        mode = next((k['data']['name'] for k in kids if k['type'] == 'named'), None)
        wait = next((k['data']['waitTime'] for k in kids if k['type'] == 'wait'), None)
        intake, shoot = MODES_8793[mode]
        if path:
            if robot is None:
                # (resetOdom: the AUTO starts at the first path's starting holonomic pose, stopped)
                first = PPPath(json.load(open(os.path.join(d, 'paths', path + '.path'))))
                robot = (first.ideal[1] if first.ideal else 0.0, 0.0, 0.0, 0.0)
            # (each path as their FollowPathCommand runs it, from the robot as the last one left it;
            # one after another on the same clock)
            seg, robot = pp_path(d, path, cfg, robot)
            if start is None:
                start = (*seg[0]['xy'], seg[0]['rot'])
            for q in seg[1:]:
                pts.append(pt(*q['xy'], intake, shoot, q['v'], dt=clock.to(t0 + q['t']), rot=q['rot']))
            t0 = clock.t
            continue
        if robot: robot = (robot[0], 0.0, 0.0, 0.0)
        if mode and pts and wait is not None:
            # a mode on the spot for a while (ShooterMode: shooting what it holds)
            pts[-1].update(action='shoot', t=wait)
            clock.restart(); t0 = 0.0
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


# 1114's PathPlanner RobotConfig (Drive.PP_CONFIG, not their GUI settings): 74.088 kg, 6.883 kg m^2,
# 2in wheels, 4.02 m/s at 12 V, wheel COF 1.7, a Kraken X60 FOC through 7.6737:1 at their 50 A slip
# current, the modules 9.75in and 12.5in off the center (TunerConstants)
IN_M = 0.0254
CFG_1114 = PPConfig(74.088, 6.883, 2 * IN_M, 4.02, 1.7, PPMotor(*PP_MOTORS['krakenX60FOC'], 7.673684210526316), 50.0,
                    [(9.75 * IN_M, 12.5 * IN_M), (9.75 * IN_M, -12.5 * IN_M), (-9.75 * IN_M, 12.5 * IN_M), (-9.75 * IN_M, -12.5 * IN_M)])
HUB_XY = (4.6256, W / 2)  # (the blue HUB's center)


def auto_1114(d, file, mirror):
    a = json.load(open(os.path.join(d, 'autos', file + '.auto')))
    ys = (lambda y: W - y) if mirror else (lambda y: y)
    rs = (lambda r: -r) if mirror else (lambda r: r)
    pts, start, robot, clock, vend = [], None, None, Clock(), 0.0
    for c in a['command']['data']['commands']:
        t, data = c['type'], c.get('data', {})
        if t == 'path':
            if robot is None:
                # (no resetOdom: the robot placed at the first path's start, at its ideal rotation)
                first = PPPath(json.load(open(os.path.join(d, 'paths', data['pathName'] + '.path'))))
                robot = (first.ideal[1] if first.ideal else 0.0, 0.0, 0.0, 0.0)
            # (worked out unmirrored: a mirrored path's trajectory is its own mirrored, mirrorPath)
            seg, robot = pp_path(d, data['pathName'], CFG_1114, robot, 'IntakeDeploy', 'AimShooterHub')
            vend = math.hypot(robot[1], robot[2])
            if start is None:
                start = (seg[0]['xy'][0], ys(seg[0]['xy'][1]), rs(seg[0]['rot']))
            for q in seg[1:]:
                pts.append(pt(q['xy'][0], ys(q['xy'][1]), q['on'], 'off', q['v'], spin=q['past'], dt=clock.to(q['t']), rot=rs(q['rot'])))
            clock.restart()
        elif t == 'named' and data['name'] in SHOOT_1114:
            aim, dur = SHOOT_1114[data['name']]
            last = pts[-1]
            x, y = min(last['fx'], SHOOT_X_1114), ys(last['fy'])
            if last['fx'] > SHOOT_X_1114:
                # (aimDriveHubAuto's drive to x = 3.1 m, as a stretch of the path on from its end
                # speed to 1 m/s; it turns to the HUB as it aims, holding the path's heading till then)
                ds = last['fx'] - SHOOT_X_1114
                pts.append(pt(SHOOT_X_1114, last['fy'], False, 'off', 1.0, spin=True, dt=2 * ds / (vend + 1.0), rot=last['rot']))
            pts[-1].update(action='shoot', t=dur, aim=aim, hold=True)
            # (stopped there, turned to the HUB: the rotation the next path starts from)
            robot = (math.atan2(HUB_XY[1] - y, HUB_XY[0] - x), 0.0, 0.0, 0.0)
            vend = 0.0
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


# ------------------------------------------------------------------ 581: Trailblazer
# Their own path follower (shared/.../trailblazer, comp-bot Robot.java): an AUTO is a state machine
# (comp-bot autos/auto_state_machines/LeftNormalAuto, RightNormalAuto: their chooser's LEFT and
# RIGHT, after DO_NOTHING) running segments of poses. HeuristicPathTracker drives at the segment's
# current point (straight at it, turned to its rotation) and moves on to the next once within its
# transition tolerance (distance, and turned to within 100 deg, or 50 for each cycle's first point);
# PidPathFollower's speed is 3.5 x the distance left to the segment's last point through its points,
# slew-limited to the point's (else the segment's) max acceleration from what it commanded last (at
# least 0.5 m/s), no faster than its max velocity nor than stops it at the end at that acceleration;
# its turn rate a ProfiledPIDController (Swerve.ORIGINAL_HEADING_PID, kP 7.5) toward the point's
# rotation within its angular constraints. Replayed here at 50 Hz (the robot turning its velocity
# toward what it commands at up to 12 m/s², what its tires grip (the sim's 581 gets 12.75), and
# turning as commanded; its speed and turn rate scaled down together where they'd ask its modules
# for more than 3.2 m/s, 90% of the 3.54 m/s the sim's 581 tops out at: their 4.5 m/s is past what
# it reaches, and at its very top speed the robot can't turn too and the sim's follower can't catch
# up: falling behind, it met the BUMP late and turned, its intake on the TRENCH's skirt) into a
# time-parameterized path the sim's follower runs by the clock with their gains (3.5, 7.5). Their
# states: INTAKE_FIRST_CYCLE (intaking), CROSS_BUMP_TO_SHOOT (to the shot over the BUMP, intake off;
# BumpCrossingFollower drives it at 4 m/s along the field from the request until it's flat off the
# BUMP), SHOOT (from once it's flat off the BUMP, its back turned to the HUB by its aim (overriding
# the tracker's rotation), driving on to the shot: 2 s, on to 3.25 s while it's still shooting), the
# DEFAULT second cycle (intaking from its START_INTAKE_2 point; the cluster-map's far-lane variant
# and the collision offset are left out: they hang on what the robot sees), shoot, the third cycle,
# and shooting to the end. Poses are theirs, red, as Point.ofRed takes them (blue: turned 180 deg
# about the field's center, FieldUtil.pathflip); FieldUtil's centers from the 2026 welded AprilTag
# layout: the red DEPOT-side TRENCH's y (tag 17) 0.6445, BUMP's 2.475 (48.94 + 12 + 36.5in from the
# wall), BUMP_OFFSET -0.13. Right is Left mirrored (their points are; y -> W - y, rotation negated).
L581, W581 = 16.541, 8.069
T581, B581 = 0.6445, 2.475 + 0.13  # (the TRENCH lane, and the BUMP's center line less BUMP_OFFSET)
SHOOT_X581 = 13.83
ACCEL_581, VMAX_581 = 12.0, 3.2  # (the replayed robot: see above)
R4 = 4 * 2 * math.pi  # (4 rotations/s, rad/s)
# segments: (points, (max velocity, max acceleration), (max angular velocity, max angular
# acceleration), finish tolerance); points: (x, y, rot deg, transition tolerance (m, or (m, deg);
# deg 100 if not given), marker, linear constraints, angular constraints)
SEG_581 = {
    'intake1': ([(10.489, T581, 90, 0.75), (9.010, 1.417, 120, 0.4), (8.553, 2.677, 85, 0.4), (8.670, 3.37, 30, 0.4, 'PRIORITIZE_INTAKE'),
                 (9.6, 3.57, -90, 0.4), (9.7, B581, -32, 0.3), (10.6, B581, 0, 0.3), (SHOOT_X581, B581, 0, 0.3, 'READY_TO_CROSS_BUMP')],
                (4.5, 8), (R4, R4), 0.2),
    'intake2': ([(13.2, T581, -160, (2.0, 50), None, (2.0, 2.0), (4.0, 3.0)), (13.2, T581, 180, 0.2, None, (2.5, 8.0), (4.0, 3.0)),
                 (12.5, T581, 180, 0.2, 'START_INTAKE', (3.0, 8.0), (2.0, 1.0)), (11.5, T581, 180, 0.2, None, (4.5, 8.0), (0.5, 0.5)),
                 (10.628, 0.87, 140, 0.4), (9.140, 1.417, 130, 0.4), (8.7, 2.477, 85, 0.4), (8.5, 3.76, 30, 0.4), (10.07, 3.77, -15, 0.5),
                 (10.2, B581, -50, 0.1), (SHOOT_X581, B581, 0, 0.3, 'READY_TO_CROSS_BUMP')],
                (4.5, 8), (R4, R4), 0.2),
    'intake3': ([(13.2, T581, -160, (2.0, 50), None, (2.0, 2.0), (4.0, 3.0)), (13.2, T581, 180, 0.2, None, (2.5, 8.0), (4.0, 3.0)),
                 (12.5, T581, 180, 0.2, 'START_INTAKE', (3.0, 8.0), (2.0, 1.0)), (11.5, T581, 180, 0.2, None, (4.5, 8.0), (0.5, 0.5)),
                 (9.6, 1.717, 130, 0.4), (8.7, 2.477, 90, 0.3), (8.6, 3.76, 90, 0.3), (9.31, 3.77, -32, 0.3), (9.7, B581, 0, 0.3),
                 (10.6, B581, 0, 0.3), (SHOOT_X581, B581, 0, 0.3, 'READY_TO_CROSS_BUMP')],
                (4.5, 8), (R4, 3 * 2 * math.pi), 0.5),
    'cross': ([(SHOOT_X581, B581, 0, 0.1, None, (4.5, 8))], (4.5, 8), (R4 / 2, R4 / 2), 0.3),
}
START_581 = (12.1, 0.47, 90)
BUMP_X581 = 11.9155 + 0.564  # (the far edge of the red BUMP, from the HUB's x: off it from here on)


class Profile1D:
    """WPILib's ProfiledPIDController (P only, continuous input) on a TrapezoidProfile"""
    def __init__(self, kp): self.kp, self.pos, self.vel = kp, 0.0, 0.0
    def reset(self, pos, vel): self.pos, self.vel = pos, vel
    def calc(self, meas, goal, vmax, amax, dt):
        # (the goal and setpoint wrapped to within half a turn of the measurement)
        goal = meas + rwrap(goal - meas)
        self.pos = meas + rwrap(self.pos - meas)
        d = goal - self.pos
        # (a trapezoid's next step toward the goal from the setpoint)
        want = math.copysign(min(vmax, math.sqrt(2 * amax * abs(d))), d)
        self.vel += max(-amax * dt, min(amax * dt, want - self.vel))
        step = self.vel * dt
        if abs(step) >= abs(d): self.pos, self.vel = goal, 0.0
        else: self.pos += step
        return self.kp * rwrap(self.pos - meas)


def auto_581(mirror):
    """their Left (mirror: Right) NormalAuto replayed: (start (blue x, y, rot deg), points)"""
    # (blue, mirrored for the right: red -> blue turns it 180 about the center; the right side is
    # the left's mirror across the field's length)
    def blue(x, y, r):
        bx, by, br = L581 - x, W581 - y, wrap_deg(r + 180)
        return (bx, W581 - by, wrap_deg(-br)) if mirror else (bx, by, br)
    sx, sy, sr = blue(*START_581)
    pos, vel, yaw = [sx, sy], [0.0, 0.0], math.radians(sr)
    dt, t = 0.02, 0.0
    samples = [{'xy': tuple(pos), 't': 0.0, 'v': 0.0, 'rot': sr, 'intake': True, 'shoot': False}]
    pts = []
    rot_ctl = Profile1D(7.5)

    def run(seg, intake=True, intake_from=None, shoot_after_bump=False):
        nonlocal pos, vel, yaw, t
        points, lin, ang, fin = SEG_581[seg]
        P = [blue(*p[:3]) for p in points]
        k, last_cmd = 0, math.hypot(*vel)
        rot_ctl.reset(yaw, 0.0)
        if intake_from: intake = False
        while t < 20.0:
            p = points[k]
            tol = p[3] if isinstance(p[3], tuple) else (p[3], 100)
            if k < len(P) - 1 and math.dist(pos, P[k][:2]) <= tol[0] and abs(math.degrees(rwrap(yaw - math.radians(P[k][2])))) <= tol[1]:
                k += 1
                continue
            if intake_from and points[k][4:5] == (intake_from,): intake = True
            # (an intake cycle's last point, the shot, carries READY_TO_CROSS_BUMP: heading for it,
            # the AUTO moves on to crossing the BUMP to it)
            if seg != 'cross' and k == len(P) - 1: return
            tgt = P[k]
            dist = math.dist(pos, tgt[:2]) + sum(math.dist(P[i][:2], P[i + 1][:2]) for i in range(k, len(P) - 1))
            if k == len(P) - 1 and math.dist(pos, tgt[:2]) <= fin: return
            pl = points[k][5] if len(points[k]) > 5 and points[k][5] else lin
            pa = points[k][6] if len(points[k]) > 6 and points[k][6] else ang
            v = min(3.5 * dist, pl[0], VMAX_581, max(last_cmd + pl[1] * dt, 0.5), math.sqrt(2 * pl[1] * dist))
            # (BumpCrossingFollower: from the crossing request until it's flat off the BUMP, x at 4 m/s
            # toward the shot, keeping the follower's y speed and turn)
            bx = L581 - pos[0]
            on_bump = seg == 'cross' and bx < BUMP_X581 + 0.45
            last_cmd = v
            d = math.dist(pos, tgt[:2])
            want = [(tgt[0] - pos[0]) / d * v, (tgt[1] - pos[1]) / d * v] if d > 1e-6 else [0.0, 0.0]
            if on_bump:
                want[0] = math.copysign(min(4.0, VMAX_581), tgt[0] - pos[0])
                m = math.hypot(*want)
                if m > VMAX_581: want = [want[0] * VMAX_581 / m, want[1] * VMAX_581 / m]
            w = rot_ctl.calc(yaw, math.radians(tgt[2]), pa[0], pa[1], dt)
            # (its swerve modules share their top speed between driving and turning: 0.41 m out, as
            # the sim's 581 has them, both scaled down together where the sum is over it)
            m = math.hypot(*want) + abs(w) * 0.41
            if m > VMAX_581: want, w = [want[0] * VMAX_581 / m, want[1] * VMAX_581 / m], w * VMAX_581 / m
            dv = [want[0] - vel[0], want[1] - vel[1]]
            n = math.hypot(*dv)
            s = min(1.0, ACCEL_581 * dt / n) if n > 0 else 0.0
            vel = [vel[0] + dv[0] * s, vel[1] + dv[1] * s]
            pos = [pos[0] + vel[0] * dt, pos[1] + vel[1] * dt]
            yaw = rwrap(yaw + w * dt)
            t += dt
            shoot = shoot_after_bump and (L581 - pos[0]) > BUMP_X581 + 0.45
            samples.append({'xy': tuple(pos), 't': t, 'v': math.hypot(*vel), 'rot': math.degrees(yaw), 'intake': intake, 'shoot': shoot})

    def emit(action=None, hold=None):
        nonlocal samples
        if len(samples) > 1:
            S = samples
            clock = Clock()
            keep = lambda i: S[i]['intake'] != S[i - 1]['intake'] or S[i]['shoot'] != S[i - 1]['shoot']
            for i in thin(S, keep=keep, tol=0.03, tol_deg=1.0)[1:]:
                q = S[i]
                pts.append(pt(*q['xy'], q['intake'], 'hub' if q['shoot'] else 'off', q['v'], dt=clock.to(q['t'] - S[0]['t']), rot=q['rot']))
        if action:
            pts[-1].update(action='shoot', t=action)
        samples = [dict(samples[-1], t=t)]

    def shoot_until(t_end):
        # (SHOOT: driving on to the shot as it shoots, from once it's off the BUMP; at least 2 s,
        # at most 3.25 s, the rest of it sat at the shot)
        # As it shoots, its aim overrides the tracker's rotation (Trailblazer's rotation override): its
        # back (the drum shoots over it) turned to the HUB, the heading the next cycle sets off from
        nonlocal t, yaw
        start = next((s['t'] for s in samples if s['shoot']), t)
        emit(round(max(0.5, start + t_end - t), 2))
        t = max(t, start + t_end)
        yaw = rwrap(math.atan2(HUB_XY[1] - pos[1], HUB_XY[0] - pos[0]) + math.pi)
        samples[0].update(t=t, rot=math.degrees(yaw))

    run('intake1')
    emit()
    run('cross', intake=False, shoot_after_bump=True)
    shoot_until(3.25)
    run('intake2', intake_from='START_INTAKE')
    emit()
    run('cross', intake=False, shoot_after_bump=True)
    shoot_until(3.25)
    run('intake3', intake_from='START_INTAKE')
    emit()
    run('cross', intake=False, shoot_after_bump=True)
    emit(20.0)
    return (sx, sy, sr), pts


# ------------------------------------------------------------------ 1678: Choreo
def choreo(d, name):
    """a Choreo trajectory's samples (blue, unmirrored: {xy, t, v, rot (deg), and the rest of the
    sample}), the time its intake event fires, and its interpolation: Choreo's
    SwerveSample.interpolate, from the sample before integrating its velocity and acceleration (the
    heading too, from its omega and alpha: so across +-180 deg it turns the short way)"""
    t = json.load(open(os.path.join(d, 'src/main/deploy/choreo', name + '.traj')))
    s = t['trajectory']['samples']
    events = {e['name']: e['from']['targetTimestamp'] + (e['from']['offset']['val'] or 0) for e in t.get('events', [])}
    S = [dict(q, xy=(q['x'], q['y']), v=math.hypot(q['vx'], q['vy']), rot=math.degrees(q['heading'])) for q in s]
    ts = [q['t'] for q in S]

    def exact(t):
        q = S[max(0, bisect.bisect_right(ts, t) - 1)]
        u = min(t, S[-1]['t']) - q['t']
        return ((q['x'] + q['vx'] * u + 0.5 * q['ax'] * u * u, q['y'] + q['vy'] * u + 0.5 * q['ay'] * u * u),
                math.degrees(q['heading'] + q['omega'] * u + 0.5 * q['alpha'] * u * u))
    return S, events.get('intake', 0.0), exact


# driveOverBump's heading (CHOREO_THETA_CONTROLLER to it, blue, LeftSweep; RightSweep's the negative)
BUMP_ROT_1678 = [135.0, 141.15]


def auto_1678(d, trajs, mirror):
    pts, start = [], None
    over_bump = 3.0  # their OVER_BUMP_FORWARD_SPEED
    # (their mirroredTrajectory flips y and the heading)
    ys = (lambda y: W - y) if mirror else (lambda y: y)
    rs = (lambda r: -r) if mirror else (lambda r: r)
    for i, name in enumerate(trajs):
        S, intake_at, exact = choreo(d, name)
        if start is None:
            start = (S[0]['x'], ys(S[0]['y']), rs(S[0]['rot']))
        elif pts:
            # driveOverBump from the last trajectory's end to here, at 3 m/s turning to its heading,
            # then fireCommand
            a, b = (pts[-1]['fx'], pts[-1]['fy']), (S[0]['x'], ys(S[0]['y']))
            rot = BUMP_ROT_1678[min(i - 1, 1)] * (-1 if mirror else 1)
            n = max(1, int(math.dist(a, b) / STEP))
            for k in range(1, n + 1):
                pts.append(pt(a[0] + (b[0] - a[0]) * k / n, a[1] + (b[1] - a[1]) * k / n, True, 'off', over_bump, dt=math.dist(a, b) / n / over_bump, rot=rot))
            pts[-1].update(action='shoot', t=3.0)
        # (its samples as they are, as many as the sim needs to keep to it, each with its speed,
        # heading and time there: their AutoTrajectory runs by its clock)
        clock = Clock()
        on = lambda k: S[k]['t'] >= intake_at
        for k in thin(S, keep=lambda k: on(k) != on(k - 1), exact=exact)[1:]:
            q = S[k]
            pts.append(pt(q['x'], ys(q['y']), on(k), 'off', q['v'], dt=clock.to(q['t'] - S[0]['t']), rot=rs(q['rot'])))
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
        # (x, y, t, speed, mechanisms, heading in deg: their reader flips y and theta for the left)
        out.append((r[1], ys(r[2]), r[0], math.hypot(r[4], r[5]), state, math.degrees(-r[3] if flip else r[3])))
    return out, rows[-1][0]


def qp_thin(samples):
    """the rows of a partial the sim needs to keep within 1 cm and 0.5 deg of it, as their reader
    interpolates it (QuikPlanSwervePartialTrajectoryReader: x, y and theta each linear in time
    between rows; theta unwrapped, so it turns the way the file does), and every row where the
    mechanisms change"""
    S = [{'xy': q[:2], 't': q[2], 'v': q[3], 'rot': q[5]} for q in samples]
    ts = [q['t'] for q in S]

    def exact(t):
        k = max(0, min(len(S) - 2, bisect.bisect_right(ts, t) - 1))
        a, b = S[k], S[k + 1]
        u = min(1.0, max(0.0, (t - a['t']) / (b['t'] - a['t']))) if b['t'] > a['t'] else 1.0
        return (a['xy'][0] + (b['xy'][0] - a['xy'][0]) * u, a['xy'][1] + (b['xy'][1] - a['xy'][1]) * u), a['rot'] + (b['rot'] - a['rot']) * u
    return thin(S, keep=lambda k: samples[k][4] != samples[k - 1][4], exact=exact)


def auto_604(d, seq):
    """seq: ('qp', file, flip, endTimeExtension) | ('wait', s) | ('retract',). Returns start, points."""
    pts, start, state, hold, pos, rot = [], None, (False, False, False), 0.0, None, None

    def flush_hold(final=False):
        nonlocal hold
        if hold <= 0 and not final: return
        intake, shoot = state[0] and state[1], state[2]
        if final and not shoot and hold <= 0: return
        # (shooting with its intake still rolling, for the whole wait even once it's empty: their
        # robot sits it out; launching to the end of AUTO when that's how it ends; holding the
        # heading its last partial left it at, turning to it if that partial turned on the spot)
        p = pt(pos[0], pos[1], intake, 'always' if shoot else 'off', 1.0, 'shoot' if shoot else 'wait', round(20.0 if final and shoot else hold, 2), rot=rot)
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
            start = (*samples[0][:2], samples[0][5])
            pos, rot = samples[0][:2], samples[0][5]
            flush_hold()  # (a wait before it moves: where it starts, nothing running yet)
        moved = math.dist(samples[0][:2], samples[-1][:2]) > 0.05 or any(math.dist(samples[0][:2], q[:2]) > 0.05 for q in samples)
        if moved:
            flush_hold()
            clock = Clock()
            for k in qp_thin(samples)[1:]:
                q = samples[k]
                s = q[4]
                # (its speed, heading and time there in their trajectory: FollowQuikplan runs by its clock)
                pts.append(pt(q[0], q[1], s[0] and s[1], 'always' if s[2] else 'off', q[3], dt=clock.to(q[2] - samples[0][2]), rot=q[5]))
            pos = samples[-1][:2]
        else:
            hold += T  # (turning on the spot: their turn_forward_transition)
        rot = samples[-1][5]
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
    out = [p['fx'], p['fy'], p['v'], f, p.get('dt', 0), p.get('rot')]
    if p['action'] != 'none': out += [p['action'], p.get('t')]
    return out


def timed(start, kp, kr, lag=0.0):
    """An AUTO run by its trajectory's clock (PathPlanner's, Choreo's, Quikplan's followers): each
    point's dt (the time to it from the point before) and rot (the heading there), the robot chasing
    where the trajectory is by now with their controller's P gains (translation per m, rotation per
    rad), each path over once its time is up; starting at its first heading (startYaw, deg)."""
    out = {'startYaw': round(wrap_deg(start[2]), 1), 'timed': True, 'flow': True, 'gains': [kp, kr]}
    if lag: out['posLag'] = lag  # (the target position where the trajectory was lag s ago: PathPlanner's)
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
                                             'startFx': round(start[0], 3), 'startFy': round(start[1], 3), **timed(start, 5.0, 5.0, PP_POS_LAG), 'shootPreload': pre or False, 'points': pts})
    for side, seq, start, mirror, name, src in [
            ('left', LEFT_NZ, 'start', False, 'Left Neutral Zone', 'createLeftNeutralZoneAuto'),
            ('right', RIGHT_NZ, 'start', True, 'Right Neutral Zone', 'createRightNeutralZoneAuto'),
            ('center', DEPOT, 'center', False, 'Depot', 'createDepotAuto')]:
        st, pts = auto_2910(seq, start, mirror)
        autos.setdefault('2910', []).append({'id': f'2910-{side}', 'name': name, 'side': side, 'source': f'AutoFactory.{src}',
                                             'startFx': round(st[0], 3), 'startFy': round(st[1], 3), 'shootPreload': False, 'points': pts})
    for side, mirror, name, src in [('left', False, 'Left Normal', 'LeftNormalAuto'), ('right', True, 'Right Normal', 'RightNormalAuto')]:
        st, pts = auto_581(mirror)
        autos.setdefault('581', []).append({'id': f'581-{side}', 'name': name, 'side': side, 'source': f'comp-bot autos/auto_state_machines/{src}.java',
                                            'startFx': round(st[0], 3), 'startFy': round(st[1], 3), **timed(st, 3.5, 7.5), 'shootPreload': False, 'points': pts})
    for side, mirror, name, src in [('left', False, 'Left Double Sweep', 'LeftSweep'), ('right', True, 'Right Double Sweep', 'RightSweep')]:
        st, pts = auto_1678(a.t1678, ['startToScore', 'scoreToSecondSweep', 'secondScoreToNeutral'], mirror)
        autos.setdefault('1678', []).append({'id': f'1678-{side}', 'name': name, 'side': side, 'source': f'autos/midline/{src}.java',
                                             'startFx': round(st[0], 3), 'startFy': round(st[1], 3), **timed(st, 6.0, 4.0), 'shootPreload': False, 'points': pts})
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
                                                'startFx': round(st[0], 3), 'startFy': round(st[1], 3), **timed(st, 5.0, 4.0), 'shootPreload': False, 'points': pts})
    pp = os.path.join(a.t1114, 'src/main/deploy/pathplanner')
    for side, file, mirror, name in [('right', 'RightSideMiddleGreedySweep', False, 'Right 1-1 Middle Sweep : Greedy Sweep'),
                                     ('left', 'RightSideMiddleGreedySweep', True, 'Left 1-1 Middle Sweep : Greedy Sweep'),
                                     ('left', 'LeftCenterSweep254', False, 'Left Slow Centre Sweep Plus Depot')]:
        st, pts = auto_1114(pp, file, mirror)
        slug = re.sub(r'[^a-z0-9]+', '-', name.lower()).strip('-')
        autos.setdefault('1114', []).append({'id': f'1114-{slug}', 'name': name, 'side': side, 'source': f'PathPlanner "{file}"' + (' mirrored' if mirror else ''),
                                             'startFx': round(st[0], 3), 'startFy': round(st[1], 3), **timed(st, 3.5, 5.0, PP_POS_LAG), 'shootPreload': False, 'spinPts': True, 'points': pts})
    out = os.path.join(ROOT, 'js', 'teamAutos.js')
    with open(out, 'w') as f:
        f.write('// The teams\' own AUTOs, from their robot code (tools/team-autos.py writes this file; see its\n')
        f.write('// header). Blue field coordinates; points as js/customAutos.js, speeds v in m/s, t: how long a\n')
        f.write('// shoot or wait lasts (s), or for until, the time into AUTO it waits for. A timed one\'s points\n')
        f.write('// also give dt, the time its trajectory takes to them from the point before (s), and rot, the\n')
        f.write('// heading there (deg, blue: 0 toward the red ALLIANCE WALL, 90 to the left); startYaw, its start\'s.\n')
        plain = {k: v for k, v in autos.items() if k not in COMPACT}
        f.write('export const TEAM_AUTOS = ' + json.dumps(plain, separators=(',', ':')) + ';\n')
        # (a team with many autos, 604's, as compact points, [fx, fy, v, flags, dt, rot] and [.., action, t]
        # (flags: 1 intake, 2 shooting, 4 hold the wait out), spelled out as above when it loads)
        for k in COMPACT:
            if k not in autos: continue
            comp = [{**{q: x[q] for q in x if q != 'points'}, 'pts': [pack(p) for p in x['points']]} for x in autos[k]]
            f.write(f"TEAM_AUTOS['{k}'] = " + json.dumps(comp, separators=(',', ':')) + '.map(({ pts, ...a }) => ({ ...a, points: pts.map(unpack) }));\n')
        f.write('function unpack([fx, fy, v, f, dt, rot, action = "none", t]) {\n'
                '  const p = { fx, fy, intake: !!(f & 1), shoot: f & 2 ? "always" : "off", v, action, dt };\n'
                '  if (rot !== null) p.rot = rot;\n'
                '  if (t !== undefined) p.t = t;\n'
                '  if (f & 4) p.hold = true;\n'
                '  return p;\n'
                '}\n')
    seen = set()
    for d, name, how, L, T, v1 in PATH_TIMES:
        if (d, name, how, T) in seen: continue
        seen.add((d, name, how, T))
        print(f'  {d.split("/")[-5]} {name}: {L:.2f} m in {T:.2f} s, its {how} trajectory (ending at {v1:.2f} m/s)')
    for k, v in autos.items():
        for x in v:
            print(k, x['id'], 'start', x['startFx'], x['startFy'], len(x['points']), 'points', [p['action'] + ' ' + str(p.get('t')) for p in x['points'] if p['action'] != 'none'])


main()
