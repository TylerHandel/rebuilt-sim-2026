# 1706's ball path as collision geometry, from their CAD (Onshape "Mirage Public Release",
# 23ba2ed5b3893ab97929c8b6): every surface FUEL could touch from the intake to the two shooters,
# simplified (vertices snapped to a grid: 1 cm for the frame and walls, 5 mm where FUEL pass close to
# them: the intake, the turrets and their hoods; 2 mm on the spindexers' printed cones, whose small
# facets 5 mm would fold away) and with the rollers and wheels
# js/mech/r1706.js turns left out. Writes js/mech/path1706.js: {body, bowl, intake, hopper, spinA, spinB,
# turret, hood: {v, i}, ext}, vertices in mm (int), in the robot's frame (x forward: the intake end,
# which the team calls the back; y up; z to its right):
#   body    what doesn't move: the chassis and its swerve modules' tops, the bellypan under the
#           spindexers, the
#           sloped plate out of the intake ("to reduce the jamming when pulling the intake in", their
#           thread), the curved polycarbonate ramps from each bowl's back up under its turret, the
#           kicker brackets, the turret bearings' plates, the hopper's fixed walls and back panel,
#           the intake's fixed frame plates, the bumpers' backing (their foam is a solid Block,
#           bumper1706)
#   bowl    the two spindexer bowls: 1/8in polycarbonate walls round each disc (7.5in inside
#           radius, open at the back), a band from 0.094 to 0.259 m up
#   intake  the arm on its pivot (the hex shaft through its plates, 0.262 m forward, 0.168 m up), as
#           exported: folded up inside the hopper (its mate's 0; it swings 113.4 deg out and down,
#           1.98 rad, its mate's limit)
#   hopper  the hopper box (its polycarbonate sides, front and top corners), as exported: in (its
#           slider's 0; 0.3048 m out at its limit)
#   spinA, spinB  each spindexer about its axis (the right one, z < 0, then the left): the 10in
#           polypropylene disc, the printed hub under it and the printed hub up its shaft
#   turret  one shooter (both are the same subassembly) in its turret's frame: origin on the turret's
#           axis, x along its shot, without the hood
#   hood    the hood's two inner plates (they carry the 1in wheels) about the 3in accelerator
#           wheels' axle, in the turret's frame: as exported (the steep end of its 45 deg)
#   ext     how far the hopper box is out (m) at each degree of the arm's swing, 0 to 113: pushed
#           out by the arm's furthest-forward part between its side walls and over their bottom
#           edge (as 604's, tools/hopper-604.py), within its slider's 0.3048 m
# Only what a FUEL can reach is kept (the space a FUEL's center fits in, flooded from points along
# the ball path, and the surfaces within a FUEL's reach of it; the intake both folded and out, the
# spindexers turned to eight angles). The assembly's glTF export carries no subassemblies, so this
# reads a STEP translation (POST .../assemblies/d/<did>/w/<wid>/e/<eid>/translations
# {"formatName":"STEP","flattenAssemblies":false}, then the external data; cad/README.md),
# tessellated by tools/step2glb.py:
#   python tools/step2glb.py r1706.step r1706-step.glb --lin 0.0012 --ang 0.45 \
#     --skip "screw|bolt|\bnut\b|washer|rivet"
# then: python3 -I tools/path-mesh-1706.py r1706-step.glb
import json, struct, sys, os, re
import numpy as np
from scipy import ndimage

ROOT = os.path.join(os.path.dirname(os.path.abspath(__file__)), '..')
SRC = sys.argv[1] if len(sys.argv) > 1 else 'r1706-step.glb'

def load(f):
    b = open(f, 'rb').read()
    n = struct.unpack('<I', b[12:16])[0]
    j = json.loads(b[20:20 + n])
    off = 20 + n
    bn = struct.unpack('<I', b[off:off + 4])[0]
    bin_ = b[off + 8:off + 8 + bn]
    CT = {5125: ('I', 4, None), 5120: ('b', 1, 127.0), 5121: ('B', 1, 255.0), 5122: ('h', 2, 32767.0), 5123: ('H', 2, 65535.0), 5126: ('f', 4, None)}
    def acc(i):
        a = j['accessors'][i]; bv = j['bufferViews'][a['bufferView']]
        fmt, sz, norm = CT[a['componentType']]
        nc = {'SCALAR': 1, 'VEC2': 2, 'VEC3': 3, 'VEC4': 4}[a['type']]
        stride = bv.get('byteStride', sz * nc)
        start = bv.get('byteOffset', 0) + a.get('byteOffset', 0)
        raw = np.frombuffer(bin_, dtype=np.uint8, count=stride * (a['count'] - 1) + sz * nc, offset=start)
        arr = np.lib.stride_tricks.as_strided(raw, shape=(a['count'], nc * sz), strides=(stride, 1)).copy().view(np.dtype('<' + fmt)).reshape(a['count'], nc).astype(np.float64)
        if a.get('normalized') and norm: arr = np.maximum(arr / norm, -1.0)
        return arr
    def quat(q):
        x, y, z, w = q
        return np.array([[1 - 2 * (y * y + z * z), 2 * (x * y - z * w), 2 * (x * z + y * w)], [2 * (x * y + z * w), 1 - 2 * (x * x + z * z), 2 * (y * z - x * w)], [2 * (x * z - y * w), 2 * (y * z + x * w), 1 - 2 * (x * x + y * y)]])
    def local(nd):
        if 'matrix' in nd: return np.array(nd['matrix']).reshape(4, 4).T
        M = np.eye(4); M[:3, :3] = quat(nd.get('rotation', [0, 0, 0, 1])) @ np.diag(nd.get('scale', [1, 1, 1])); M[:3, 3] = nd.get('translation', [0, 0, 0])
        return M
    out = []  # (path, triangles)
    def walk(i, M, path):
        nd = j['nodes'][i]; W = M @ local(nd); p = path + [nd.get('name', '')]
        if 'mesh' in nd:
            for pr in j['meshes'][nd['mesh']]['primitives']:
                P = acc(pr['attributes']['POSITION']); P = (W[:3, :3] @ P.T).T + W[:3, 3]
                idx = acc(pr['indices']).astype(int).reshape(-1, 3) if 'indices' in pr else np.arange(len(P)).reshape(-1, 3)
                out.append((p, P[idx]))
        for c in nd.get('children', []): walk(c, W, p)
    for r in j['scenes'][j.get('scene', 0)]['nodes']: walk(r, np.eye(4), [])
    return out

# hardware, electronics, what moves on its own (chains, belts, gears, sprockets, pulleys), the
# rollers and wheels r1706.js turns (the intake's two 2in polycarbonate tubes, their end caps and the
# 1in urethane wheels' shaft; the kickers' 2in wheels; the shooters' 3in and 1in wheels and the 5in
# flywheel), their shafts, the FUEL in the CAD and the bumpers' foam (a solid Block, bumper1706)
SKIP = re.compile(r'screw|bolt|\bnut\b|locknut|washer|rivet|spacer|bearing|bushing|collar|chain|belt|sprocket|gear\b|spur gear|pulley|'
                  r'radio|battery|roborio|pdp|pigeon|canivore|breaker|grommet|shim|standoff|zip|wire|camera|limelight|encoder|cancoder|magnet|'
                  r'game piece|generic game|urethane wheel|flywheel|tube_roller|rounded hex|hex lite|-a-[\d.]+-tt|-b-[\d.]+-tt|-a-[\d.]+-xx|'
                  r'straight pin|retaining|adapter|energy chain', re.I)
# (in the intake: what stays on the frame, what slides, the pivot's drive, and the rollers' caps)
MOUNT = re.compile(r'PLATE_FRAME|SHEET_MOUNT_SLIDE|^Part 36|A-RB-20-109|RB-20-A-29|X60 Brushless Motor <2>|MAXPlanetary|^Part 22$')
BOX = re.compile(r'HOPPER_SIDE|HOPPER_FRONT|CORNER_HOPPER|^Part 4[123]')
SPIN = re.compile(r'RB-70-FOREHEAD|^Part 7$|^Part 1(-Mirrored)?$|3/8" Rounded Hex')
HOOD = re.compile(r'^RB-50-10[26]$')
BOWL = re.compile(r'^RB-70-101$|^Part 13$')
parts = load(SRC)
def robot(T):  # CAD (x back from the intake, y to the robot's right, z up) -> robot (x forward, y up, z right)
    return np.stack([-T[..., 0], T[..., 2], T[..., 1]], -1)

PIVOT = np.array([0.262, 0.168, 0.0])       # the intake arm's (the hex shaft through its plates; its mate)
ARM_OUT = 113.4                             # deg: its mate's 1.98 rad, down and out
BOX_OUT = 0.3048                            # its slider's travel
SPIN_AX = [np.array([0.0889, 0.0, -0.1778]), np.array([0.0889, 0.0, 0.1778])]  # the discs' axes (their 72t sprockets')
TURRET = [np.array([-0.1651, 0.0, -0.2220]), np.array([-0.1651, 0.0, 0.2220])]  # the turret bearings' axes
ACCEL = np.array([0.1053, 0.4607])          # the 3in wheels' axle in the turret's frame (x along the shot, y up)
BUMPER = re.compile(r'90-1XX <1> / Part 5$')

groups = {k: [] for k in ('body', 'bowl', 'intake', 'hopper', 'spinA', 'spinB', 'turretA', 'turretB', 'hoodA', 'hoodB')}
wheels3 = {0: [], 1: []}
for p, T in parts:
    name = ' / '.join(p)
    leaf = p[-1] if not re.match(r'^(Kraken|id\d)', p[-1]) else p[-2]
    T = robot(T)
    if 'SHOOTER <' in name and '3" Solid Urethane' in name: wheels3[0 if 'SHOOTER <1>' in name else 1].append(T.reshape(-1, 3).mean(0))
    if any(SKIP.search(x) for x in p[2:]) or BUMPER.search(name): continue
    if 'Thrifty Swerve' in name and T[..., 1].max() < 0.12: continue
    if 'C-RB-20-INTAKE' in name:
        groups['body' if MOUNT.search(leaf) else 'hopper' if BOX.search(leaf) else 'intake'].append(T)
    elif 'A-RB-70-Spindexer' in name and BOWL.search(leaf): groups['bowl'].append(T)
    elif 'A-RB-70-Spindexer' in name and SPIN.search(leaf):
        groups['spinA' if T[..., 2].mean() < 0 else 'spinB'].append(T)
    elif 'A-RB-50-SHOOTER <' in name:
        k = 'A' if 'SHOOTER <1>' in name else 'B'
        groups[('hood' if HOOD.search(leaf) else 'turret') + k].append(T)
    else: groups['body'].append(T)
G = {k: np.vstack(v) for k, v in groups.items()}

# each turret's frame as exported: x along its shot (out over the 3in wheels, which are 0.105 m from
# the axis the other way from the hood's 1in wheels), z = x turned a quarter to the right
def frame(k):
    c = np.mean(wheels3[k], 0) - TURRET[k]
    f = np.array([c[0], c[2]]) / np.hypot(c[0], c[2])
    return f, np.array([-f[1], f[0]])
def to_turret(T, k):
    f, r = frame(k); d = T - TURRET[k]
    return np.stack([d[..., 0] * f[0] + d[..., 2] * f[1], d[..., 1], d[..., 0] * r[0] + d[..., 2] * r[1]], -1)
def from_turret(T, k):
    f, r = frame(k)
    return np.stack([T[..., 0] * f[0] + T[..., 2] * r[0], T[..., 1], T[..., 0] * f[1] + T[..., 2] * r[1]], -1) + TURRET[k]
YAW = [np.degrees(np.arctan2(-frame(k)[0][1], frame(k)[0][0])) for k in (0, 1)]
print('turrets as exported: their shots at yaw', [f'{y:.1f}' for y in YAW], 'deg (0 straight ahead, the intake end; three.js sense)')

def turn_arm(T, deg):  # the arm swung deg out and down about its pivot
    a = -np.radians(deg); c, s = np.cos(a), np.sin(a); d = T - PIVOT
    return np.stack([PIVOT[0] + d[..., 0] * c - d[..., 1] * s, PIVOT[1] + d[..., 0] * s + d[..., 1] * c, T[..., 2]], -1)
def rot_y(T, a, c):  # turn about the vertical through c by a (rad; three.js's sense)
    d = T - c; ca, sa = np.cos(a), np.sin(a)
    return np.stack([d[..., 0] * ca + d[..., 2] * sa, d[..., 1], -d[..., 0] * sa + d[..., 2] * ca], -1) + c

# the box's travel at each degree of the arm: out to the arm's furthest-forward part between its side
# walls (|z| under 0.37) and between their bottom edge and top (0.218 to 0.548 m), less the box's
# front as exported (0.318 m), within its slider; no further out than at any steeper angle (a part
# coming up into the walls' height from below meets their bottom edge)
FRONT, WALL_Y = 0.318, (0.218, 0.548)
armP = G['intake'].reshape(-1, 3); armP = armP[np.abs(armP[:, 2]) < 0.37]
ext = []
for d in range(0, 114):
    P = turn_arm(armP, d); m = (P[:, 1] >= WALL_Y[0]) & (P[:, 1] <= WALL_Y[1])
    ext.append(float(np.clip(P[m, 0].max() - FRONT, 0, BOX_OUT)) if m.any() else 0.0)
ext = [round(max(ext[: i + 1]), 4) for i in range(len(ext))]
print('ext at 0, 30, 60, 90, 113 deg:', ext[0], ext[30], ext[60], ext[90], ext[113])

# Only the surfaces a FUEL can get to (as 604's): the space a FUEL's center fits in, allowing 1.5 cm of
# squash, flooded from points along the ball path, with the intake out (and the box) and folded in;
# the spindexers at eight angles; and the surfaces within a FUEL's reach of it. Walled off: beside
# the bumpers, out past the intake, behind the turrets, over the net.
VOX, R = 0.01, 0.075
LO, HI = np.array([-0.46, 0.0, -0.47]), np.array([0.76, 0.78, 0.47])
shape = tuple(np.ceil((HI - LO) / VOX).astype(int))
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
g = LO[None, :] + (np.indices(shape).reshape(3, -1).T + 0.5) * VOX
def mark(solid, P):
    ijk, ok = voxels(P); solid[tuple(ijk[ok].T)] = True
# (the rollers and wheels: a FUEL can't be inside one)
def solid_cyl(solid, c, a, r, h):
    a = np.array(a, float) / np.linalg.norm(a); d = g - np.array(c, float); s = d @ a
    solid |= ((np.abs(s) < h) & (np.linalg.norm(d - np.outer(s, a), axis=1) < r)).reshape(shape)
ROLLERS = [((0.0715, 0.3805), 0.0254), ((0.1500, 0.4485), 0.0254), ((0.2955, 0.3440), 0.0127)]  # stowed
def rollers(solid, deg):
    for (x, y), r in ROLLERS:
        c = turn_arm(np.array([[x, y, 0.0]]), deg)[0]; solid_cyl(solid, c, [0, 0, 1], r, 0.35)
    for k in (0, 1):
        s = -1 if k == 0 else 1
        for c, a in [((-0.072, 0.30, s * 0.25), (-0.297, 0, s * -0.955)), ((-0.258, 0.30, s * 0.194), (-0.297, 0, s * -0.955))]:
            solid_cyl(solid, c, a, 0.0254, 0.03)
        for (x, y), r in [(tuple(ACCEL), 0.0381), ((-0.0770, 0.4928), 0.0127), ((-0.0787, 0.4404), 0.0127)]:
            c = from_turret(np.array([[x, y, 0.0]]), k)[0]; f, rr = frame(k)
            solid_cyl(solid, c, [rr[0], 0, rr[1]], r, 0.03)
    return solid
turretW = {k: from_turret(to_turret(G['turret' + k], i), i) for i, k in enumerate('AB')}
hoodW = {k: G['hood' + k] for k in 'AB'}
outside = ((np.abs(g[:, 2]) > 0.465) | (g[:, 0] > 0.75) | (g[:, 0] < -0.45) | (g[:, 1] > 0.76)
           | ((np.abs(g[:, 2]) > 0.40) & (g[:, 1] > 0.16)) | ((g[:, 0] < -0.33) & (g[:, 1] > 0.12)))
seeds = np.array([[0.68, 0.08, 0], [0.68, 0.08, 0.25], [0.68, 0.08, -0.25], [0.47, 0.30, 0], [0.45, 0.45, 0.2], [0.2, 0.45, 0], [0.1, 0.45, -0.25],
                  [0.1, 0.25, 0.29], [0.1, 0.25, -0.29], [0.0, 0.25, 0.25], [0.0, 0.25, -0.25], [-0.1, 0.24, 0.2], [-0.1, 0.24, -0.2],
                  [-0.165, 0.30, 0.222], [-0.165, 0.30, -0.222], [-0.165, 0.40, 0.222], [-0.165, 0.40, -0.222], [-0.1, 0.65, 0.25], [-0.1, 0.65, -0.25]])
fixed = {'body': G['body'], 'bowl': G['bowl'], 'turretA': turretW['A'], 'turretB': turretW['B'], 'hoodA': hoodW['A'], 'hoodB': hoodW['B']}
pts = {k: samples(T) for k, T in fixed.items()}
movers = {'intake': samples(G['intake']), 'hopper': samples(G['hopper']), 'spinA': samples(G['spinA']), 'spinB': samples(G['spinB'])}
base = np.zeros(shape, bool)
for P, _ in pts.values(): mark(base, P)
base[:, 0, :] = True
base |= outside.reshape(shape)
keep = {k: np.zeros(len(T), bool) for k, T in G.items()}
def reached(reach, P, tri, n):
    ijk, ok = voxels(P)
    hit = np.zeros(len(P), bool)
    hit[ok] = reach[tuple(ijk[ok].T)]
    k = np.zeros(n, bool)
    np.logical_or.at(k, tri, hit)
    return k
for pose, (deg, out) in enumerate([(ARM_OUT, ext[-1]), (0.0, 0.0)]):
    for step in range(8):
        a = step * np.pi / 4
        solid = base.copy()
        armP_, armT = movers['intake']; boxP, boxT = movers['hopper']
        AP = turn_arm(armP_, deg); BP = boxP + [out, 0, 0]
        SA = rot_y(movers['spinA'][0], a, SPIN_AX[0]); SB = rot_y(movers['spinB'][0], -a, SPIN_AX[1])
        for P in (AP, BP, SA, SB): mark(solid, P)
        solid = rollers(solid, deg)
        free = ndimage.distance_transform_edt(~solid) * VOX >= R - 0.015
        lab, _ = ndimage.label(free)
        sk, ok = voxels(seeds)
        ids = set(lab[tuple(sk[ok].T)].tolist()) - {0}
        reach = ndimage.distance_transform_edt(~np.isin(lab, list(ids))) * VOX <= R + 0.02
        if step == 0:
            print(f'pose {pose}: seeded regions', len(ids), 'of', lab.max(), [int(lab[tuple(s)]) for s in sk])
            for k in fixed: keep[k] |= reached(reach, *pts[k], len(fixed[k]))
            keep['intake'] |= reached(reach, AP, armT, len(G['intake']))
            keep['hopper'] |= reached(reach, BP, boxT, len(G['hopper']))
        keep['spinA'] |= reached(reach, SA, movers['spinA'][1], len(G['spinA']))
        keep['spinB'] |= reached(reach, SB, movers['spinB'][1], len(G['spinB']))
for k, T in G.items(): print(f'reachable {k}: {keep[k].sum()} of {len(T)}')

def simplify(T, grid):
    V = np.round(T.reshape(-1, 3) / grid).astype(np.int64)
    uniq, inv = np.unique(V, axis=0, return_inverse=True)
    I = inv.reshape(-1, 3)
    I = I[(I[:, 0] != I[:, 1]) & (I[:, 1] != I[:, 2]) & (I[:, 0] != I[:, 2])]
    I = np.unique(np.sort(I, axis=1), axis=0)
    used, I2 = np.unique(I, return_inverse=True)
    return np.round(uniq[used] * grid * 1000).astype(int), I2.reshape(-1, 3)
# (one turret and one hood for both: the same subassembly; what either one's FUEL reach)
tur = np.vstack([to_turret(G['turretA'][keep['turretA']], 0), to_turret(G['turretB'][keep['turretB']], 1)])
hood = np.vstack([to_turret(G['hoodA'][keep['hoodA']], 0), to_turret(G['hoodB'][keep['hoodB']], 1)]) - [ACCEL[0], ACCEL[1], 0]
meshes = {
    'body': simplify(G['body'][keep['body']], 0.01),
    'bowl': simplify(G['bowl'][keep['bowl']], 0.005),
    'intake': simplify(G['intake'][keep['intake']] - PIVOT, 0.005),
    'hopper': simplify(G['hopper'][keep['hopper']], 0.01),
    # (all of each spindexer over its disc's underside: its cone's facets are small enough that the
    # flood's sampling misses some of them)
    'spinA': simplify(G['spinA'][G['spinA'][..., 1].max(1) > 0.158] - SPIN_AX[0], 0.002),
    'spinB': simplify(G['spinB'][G['spinB'][..., 1].max(1) > 0.158] - SPIN_AX[1], 0.002),
    'turret': simplify(tur, 0.005),
    'hood': simplify(hood, 0.005),
}
res = {k: {'v': v.flatten().tolist(), 'i': i.flatten().tolist()} for k, (v, i) in meshes.items()}
res['ext'] = ext
res['yaw'] = [round(y, 2) for y in YAW]
with open(f'{ROOT}/js/mech/path1706.js', 'w') as f:
    f.write('// 1706\'s ball path\'s surfaces from their CAD (generated by tools/path-mesh-1706.py; see js/mech/r1706.js)\n')
    f.write('export default ' + json.dumps(res, separators=(',', ':')) + ';\n')
for k, (v, i) in meshes.items(): print(k, len(v), 'vertices', len(i), 'triangles')
print(os.path.getsize(f'{ROOT}/js/mech/path1706.js') // 1024, 'KB')
