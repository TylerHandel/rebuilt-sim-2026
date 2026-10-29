# 1690's ball path as collision geometry, from their CAD: their public X_T release ("1690-26-Top Level
# Closed.x_t", in the Drive folder their CAD release thread links), imported into Onshape, exported as
# STEP and tessellated by tools/step2glb.py (cad/README.md). Every surface FUEL could touch from the
# intake to the shooter, simplified (vertices snapped to a grid: 1 cm for the frame and walls, 5 mm
# where FUEL pass close to them: the belts' channel, the feeder, the turret and its hood), without the
# rollers, belts and wheels js/mech/r1690.js turns. Writes js/mech/path1690.js: {body, bodyFine,
# ptfe, carriage, kick, turret, hood: {v, i}}, vertices in mm (int), in the robot's frame (x forward: the intake end; y up;
# z to its right):
#   body      what doesn't move: the chassis, the roller floor's side plates, the belts' housing and
#             backing plate, the vertical kicker's and the feeder's frames, the turret's fixed deck,
#             the intake's fixed side plates, the electronics and the back column
#             (body on the 1 cm grid; bodyFine on the 5 mm one, where FUEL pass close: the
#             corridor between the belts and the vertical kicker, the belts' channel)
#   ptfe      the belts' backing plate (313-10A), apart so it can be given its own friction
#   carriage  the intake's sliding carriage as exported (in: "Closed"): its U-frame of lattice plates
#             (the hopper's front and sides out over the intake), the black side bars it slides on,
#             the rollers' brackets. It slides SLIDE (m) out and down to deploy.
#   kick      the "kicker bar"'s arms (the aluminum tube under the rollers, on surgical tubing), about
#             their pivot (KICK_PIVOT), as exported (folded up behind the intake)
#   turret    the turret in its frame: origin on its axis, x along its shot (out over the 4in wheel),
#             without the hood
#   hood      the hood (its lattice arms, the printed "hollow hood" between them and the feeder
#             wheels' shafts' hardware) in the turret's frame about the 4in wheel's axle, as exported
#             (at its steep end)
# The X_T has no mates: the axes are its parts' cylinders (tools/step-axes.py, OpenCascade), the
# moves are what the parts' shapes allow (the carriage's black bars run on rollers fixed to the side
# plates; the kicker bar's arms turn on a pin fixed to them). Only what a FUEL can reach is kept (the
# space a FUEL's center fits in, flooded from points along the ball path with the intake out, and the
# surfaces within a FUEL's reach of it).
#   python3 -I tools/path-mesh-1690.py r1690-step.glb
import json, struct, sys, os, re
import numpy as np
from scipy import ndimage

ROOT = os.path.join(os.path.dirname(os.path.abspath(__file__)), '..')
SRC = sys.argv[1] if len(sys.argv) > 1 else 'r1690-step.glb'


def load(f):
    b = open(f, 'rb').read()
    n = struct.unpack('<I', b[12:16])[0]
    j = json.loads(b[20:20 + n])
    off = 20 + n
    bn = struct.unpack('<I', b[off:off + 4])[0]
    bin_ = b[off + 8:off + 8 + bn]
    CT = {5125: ('I', 4), 5123: ('H', 2), 5126: ('f', 4)}
    def acc(i):
        a = j['accessors'][i]; bv = j['bufferViews'][a['bufferView']]
        fmt, sz = CT[a['componentType']]
        nc = {'SCALAR': 1, 'VEC2': 2, 'VEC3': 3, 'VEC4': 4}[a['type']]
        start = bv.get('byteOffset', 0) + a.get('byteOffset', 0)
        return np.frombuffer(bin_, dtype='<' + fmt, count=a['count'] * nc, offset=start).reshape(a['count'], nc).astype(np.float64)
    def local(nd):
        if 'matrix' in nd: return np.array(nd['matrix']).reshape(4, 4).T
        return np.eye(4)
    out = []
    def walk(i, M, path):
        nd = j['nodes'][i]; W = M @ local(nd); p = path + [nd.get('name', '')]
        if 'mesh' in nd:
            for pr in j['meshes'][nd['mesh']]['primitives']:
                P = acc(pr['attributes']['POSITION']); P = (W[:3, :3] @ P.T).T + W[:3, 3]
                idx = acc(pr['indices']).astype(int).reshape(-1, 3)
                out.append((p, P[idx]))
        for c in nd.get('children', []): walk(c, W, p)
    for r in j['scenes'][j.get('scene', 0)]['nodes']: walk(r, np.eye(4), [])
    return out


def robot(T):  # CAD (X: the robot's left, Y forward, Z down) -> robot (x forward, y up, z right)
    return np.stack([T[..., 1], -T[..., 2], -T[..., 0]], -1)


# ---- where things are (robot frame, m), from the CAD's cylinders
TURRET = np.array([-0.1340, 0.0, -0.1872])       # the 8in bearing's axis
YAW0 = np.radians(-23.99)                        # the turret's shot as exported (three.js's sense: + to the left)
AXLE = np.array([0.0700, 0.4640])                # the 4in wheel's axle, turret frame (x along the shot, y up)
SLIDE = np.array([0.311, -0.068])                # the carriage's travel out (Closed -> Open)
KICK_PIVOT = np.array([0.2723, 0.1703])          # the kicker bar's arms' pin
KICK_OUT = -164.6                                # deg: folded (as exported) -> down on the carpet side
F = np.array([np.cos(YAW0), 0, -np.sin(YAW0)])   # the shot's direction
L = np.array([np.sin(YAW0), 0, np.cos(YAW0)])    # the turret's z (to the shot's right)


def to_turret(T):
    d = T - TURRET
    return np.stack([d @ F, d[..., 1], d @ L], -1)


def from_turret(T):
    return TURRET + T[..., :1] * F + T[..., 1:2] * [0, 1, 0] + T[..., 2:3] * L


HW = re.compile(r'screw|bolt|\bnut\b|locknut|washer|rivet|spacer|standoff|bearing|bushing|collar|belt|pulley|gear\b|spur gear|'
                r'\d+t |\d+T\b|radio|battery|roborio|pdp|pigeon|canivore|breaker|zip|wire|camera|limelight|encoder|signal light|'
                r'^id\d|^\*\d|planetary|217-2|constant force', re.I)
# what r1690.js turns (rollers, wheels, the belts' runs) and the FUEL in the CAD
TURNS = re.compile(r'^1690-26-321-(0[67]|1[0-5])|^1690-26-315-0[3-7]|^1690-26-314-0[2-8]|^1690-26-31[012]-01A|^1690-26-331-04A|'
                   r'^1690-26-332-0[5-7]|Compliant Wheel|Stealth Wheel|HTD5 22|Hex \(|Hex Shaft|^1690-26-253-|^1690-26-254-|'
                   r'^Part 16[12]$|^Part 156$|^1690-26-432-|^1690-26-443-08A|^Part 70$|^1690-26-313-09A')
CARRIAGE = re.compile(r'^1690-26-25[12]-|X60 Brushless Motor \(5\)|^Part 10[4-8]$|^Part 153$|^Part 157|^12-18-4|^motor pulley')
KICK = re.compile(r'^Part 15[49]$')
HOOD = re.compile(r'^1690-26-44\d|^1690-26-451-05A')
TURRET_RX = re.compile(r'^1690-26-43\d|^1690-26-451|Kraken X60 Brushless Motor \(3\)|X60 Brushless Motor-Mirrored|X44 Brushless Motor-Mirrored|'
                       r'^Part 1(1[7-9]|2\d|6[23]|99)|^Part 15$')

if __name__ == '__main__':
    parts = load(SRC)
    groups = {k: [] for k in ('body', 'ptfe', 'carriage', 'kick', 'turret', 'hood')}
    for p, T in parts:
        sub = p[1] if len(p) > 1 else ''
        leaf = p[-1]
        names = p[2:]
        # (the bumpers' foam, backing and corners: a solid Block, bumper1690; the rest of that assembly
        # is the hopper's side and back panels and the lattice frame over the top)
        if 'bumper' in sub.lower() and re.search(r'Bumper|Length|Width|^Part (1[7-9]|20|11[07])$|^\*|114-', names[0]): continue
        if any(HW.search(x) for x in names) or any(TURNS.search(x) for x in names): continue
        T = robot(T)
        if sub.startswith('1690-26-200'):
            k = 'carriage' if CARRIAGE.search(names[0]) else 'kick' if KICK.search(names[0]) else 'body'
        elif sub.startswith('1690-26-400'):
            n0 = names[0]
            k = 'hood' if HOOD.search(n0) else 'turret' if TURRET_RX.search(n0) else 'body'
        else:
            k = 'ptfe' if re.match(r'1690-26-313-10A', names[0]) else 'body'
        groups[k].append(T)
    G = {k: np.vstack(v) for k, v in groups.items()}
    for k, T in G.items(): print(k, len(T), 'triangles')

    def turn_kick(T, deg):
        a = np.radians(deg); c, s = np.cos(a), np.sin(a); d = T[..., :2] - KICK_PIVOT
        return np.stack([KICK_PIVOT[0] + d[..., 0] * c - d[..., 1] * s, KICK_PIVOT[1] + d[..., 0] * s + d[..., 1] * c, T[..., 2]], -1)

    VOX, R = 0.01, 0.075
    LO, HI = np.array([-0.40, 0.0, -0.45]), np.array([0.72, 0.80, 0.45])
    shape = tuple(np.ceil((HI - LO) / VOX).astype(int))
    g = LO[None, :] + (np.indices(shape).reshape(3, -1).T + 0.5) * VOX

    def samples(T, step=0.008):
        a, b, c = T[:, 0], T[:, 1], T[:, 2]
        n = np.clip(np.ceil(np.maximum(np.linalg.norm(b - a, axis=1), np.linalg.norm(c - a, axis=1)) / step).astype(int), 1, 80)
        out, tri = [], []
        for k in range(1, 81):
            m = n == k
            if not m.any(): continue
            u, v = np.meshgrid(np.arange(k + 1), np.arange(k + 1))
            keep = (u + v) <= k
            u, v = u[keep] / k, v[keep] / k
            P = a[m][:, None, :] + (b[m] - a[m])[:, None, :] * u[None, :, None] + (c[m] - a[m])[:, None, :] * v[None, :, None]
            out.append(P.reshape(-1, 3)); tri.append(np.repeat(np.nonzero(m)[0], len(u)))
        return np.vstack(out), np.concatenate(tri)

    def voxels(P):
        ijk = np.floor((P - LO) / VOX).astype(int)
        ok = np.all((ijk >= 0) & (ijk < shape), axis=1)
        return ijk, ok

    def mark(solid, P):
        ijk, ok = voxels(P); solid[tuple(ijk[ok].T)] = True

    def solid_cyl(solid, c, a, r, h):
        a = np.array(a, float) / np.linalg.norm(a); d = g - np.array(c, float); s = d @ a
        solid |= ((np.abs(s) < h) & (np.linalg.norm(d - np.outer(s, a), axis=1) < r)).reshape(shape)

    # the rollers, belts and wheels (r1690.js's): a FUEL can't be inside one
    FLOOR = [(0.2797, 0.1899), (0.2324, 0.1814), (0.1852, 0.1729), (0.1417, 0.1650), (0.0982, 0.1572), (0.0546, 0.1493),
             (0.0071, 0.1425), (-0.0409, 0.1425), (-0.0874, 0.1425)]
    def rollers(solid):
        for x, y in FLOOR: solid_cyl(solid, (x, y, 0), (0, 0, 1), 0.0165, 0.33)
        for x, y in [(-0.0409, 0.3191), (-0.0874, 0.3183)]: solid_cyl(solid, (x, y, 0.11), (0, 0, 1), 0.016, 0.18)
        for y, z0, z1 in [(0.1383, -0.1158, 0.2742), (0.3225, -0.0423, 0.2977)]:
            for z in np.arange(z0, z1 + 0.001, 0.005): solid_cyl(solid, (-0.134, y, z), (1, 0, 0), 0.0203, 0.016)
        solid_cyl(solid, (-0.0155, 0.228, -0.1525), (0, 1, 0), 0.0125, 0.14)
        for y, z in [(0.1600, -0.1806), (0.2031, -0.2358), (0.2629, -0.2721)]: solid_cyl(solid, (-0.134, y, z), (1, 0, 0), 0.0286, 0.026)
        for y, z in [(0.3319, -0.2837), (0.3319, -0.0909)]: solid_cyl(solid, (-0.134, y, z), (1, 0, 0), 0.0254, 0.026)
        for x, y in [(0.2473, 0.2301), (0.1993, 0.3290)]: solid_cyl(solid, (x + SLIDE[0], y + SLIDE[1], 0), (0, 0, 1), 0.021, 0.33)
        tube = turn_kick(np.array([[0.1499, 0.3283, 0.0]]), KICK_OUT)[0]; solid_cyl(solid, tube, (0, 0, 1), 0.0127, 0.34)
        solid_cyl(solid, from_turret(np.array([[AXLE[0], AXLE[1], 0.0]]))[0], L, 0.0502, 0.025)
        for ang in (172.47, 191.29):
            a = np.radians(ang); q = np.array([[AXLE[0] + 0.2068 * np.cos(a), AXLE[1] + 0.2068 * np.sin(a), 0.0]])
            solid_cyl(solid, from_turret(q)[0], L, 0.0254, 0.026)
        return solid

    turretW = G['turret']; hoodW = G['hood']
    # (walled off: beside the bumpers, out past the intake, behind the robot, over the load (except
    # round the turret's head), under the frame behind the intake)
    rT = np.hypot(g[:, 0] - TURRET[0], g[:, 2] - TURRET[2])
    outside = ((np.abs(g[:, 2]) > 0.44) | (g[:, 0] > 0.70) | (g[:, 0] < -0.33) | (g[:, 1] > 0.78)
               | ((np.abs(g[:, 2]) > 0.37) & (g[:, 1] > 0.17)) | ((g[:, 1] > 0.60) & (rT > 0.26))
               | ((g[:, 1] < 0.08) & (g[:, 0] < 0.30)))
    # (in the hopper and out over the intake, then along the ball path: over the floor's back, the belts'
    # channel, over the feeder's wheels, between its top wheels, up the turret's bore, between the 4in
    # wheel and the hood's wheels, over the hood)
    seeds = np.array([[0.62, 0.09, 0], [0.62, 0.09, 0.25], [0.62, 0.09, -0.25], [0.45, 0.30, 0], [0.2, 0.30, 0], [0.2, 0.30, 0.2], [0.2, 0.30, -0.2],
                      [-0.05, 0.23, 0.25], [-0.13, 0.23, 0.1], [-0.13, 0.23, -0.05], [-0.134, 0.26, -0.15], [-0.134, 0.29, -0.187],
                      [-0.134, 0.42, -0.187], [-0.15, 0.50, -0.20], [-0.05, 0.58, -0.2]])
    carP, carT = samples(G['carriage']); carP = carP + [SLIDE[0], SLIDE[1], 0]
    kickP, kickT = samples(G['kick']); kickP = turn_kick(kickP, KICK_OUT)
    fixed = {'body': G['body'], 'ptfe': G['ptfe'], 'turret': turretW, 'hood': hoodW}
    pts = {k: samples(T) for k, T in fixed.items()}
    solid = np.zeros(shape, bool)
    for P, _ in pts.values(): mark(solid, P)
    mark(solid, carP); mark(solid, kickP)
    solid[:, 0, :] = True
    solid |= outside.reshape(shape)
    solid = rollers(solid)
    free = ndimage.distance_transform_edt(~solid) * VOX >= R - 0.04
    lab, _ = ndimage.label(free)
    sk, ok = voxels(seeds)
    ids = set(lab[tuple(sk[ok].T)].tolist()) - {0}
    print('seeded regions', len(ids), 'of', lab.max(), [int(lab[tuple(s)]) for s in sk])
    reach = ndimage.distance_transform_edt(~np.isin(lab, list(ids))) * VOX <= R + 0.02

    def reached(P, tri, n):
        ijk, okk = voxels(P)
        hit = np.zeros(len(P), bool)
        hit[okk] = reach[tuple(ijk[okk].T)]
        k = np.zeros(n, bool)
        np.logical_or.at(k, tri, hit)
        return k
    keep = {k: reached(*pts[k], len(fixed[k])) for k in fixed}
    keep['carriage'] = reached(carP, carT, len(G['carriage']))
    keep['kick'] = reached(kickP, kickT, len(G['kick']))
    for k in keep: print(f'reachable {k}: {keep[k].sum()} of {len(keep[k])}')

    def simplify(T, grid):
        V = np.round(T.reshape(-1, 3) / grid).astype(np.int64)
        uniq, inv = np.unique(V, axis=0, return_inverse=True)
        I = inv.reshape(-1, 3)
        I = I[(I[:, 0] != I[:, 1]) & (I[:, 1] != I[:, 2]) & (I[:, 0] != I[:, 2])]
        I = np.unique(np.sort(I, axis=1), axis=0)
        used, I2 = np.unique(I, return_inverse=True)
        return np.round(uniq[used] * grid * 1000).astype(int), I2.reshape(-1, 3)
    # (the body near the belts, the kicker and the feeder at 5 mm, the rest at 1 cm)
    body = G['body'][keep['body']]
    c = body.mean(1)
    # (the feeder's corridor, up to the bore, and the belts' channel: x -0.22 to 0.03, below the deck)
    fine = ((c[:, 0] > -0.22) & (c[:, 0] < 0.03) & (c[:, 1] > 0.09) & (c[:, 1] < 0.40) & (c[:, 2] < -0.04)
            | (c[:, 0] > -0.22) & (c[:, 0] < -0.10) & (c[:, 1] > 0.10) & (c[:, 1] < 0.36))
    meshes = {
        'body': simplify(body[~fine], 0.01),
        'bodyFine': simplify(body[fine], 0.005),
        'carriage': simplify(G['carriage'][keep['carriage']], 0.005),
        'ptfe': simplify(G['ptfe'][keep['ptfe']], 0.005),
        'kick': simplify(G['kick'][keep['kick']] - [KICK_PIVOT[0], KICK_PIVOT[1], 0], 0.005),
        'turret': simplify(to_turret(turretW[keep['turret']]), 0.005),
        'hood': simplify(to_turret(hoodW[keep['hood']]) - [AXLE[0], AXLE[1], 0], 0.004),
    }
    res = {k: {'v': v.flatten().tolist(), 'i': i.flatten().tolist()} for k, (v, i) in meshes.items()}
    with open(f'{ROOT}/js/mech/path1690.js', 'w') as f:
        f.write("// 1690's ball path's surfaces from their CAD (generated by tools/path-mesh-1690.py; see js/mech/r1690.js)\n")
        f.write('export default ' + json.dumps(res, separators=(',', ':')) + ';\n')
    for k, (v, i) in meshes.items(): print(k, len(v), 'vertices', len(i), 'triangles')
    print(os.path.getsize(f'{ROOT}/js/mech/path1690.js') // 1024, 'KB')
