# 6328's ball path as collision geometry, from their CAD: the AdvantageScope model of Darwin their
# code repository ships (Mechanical-Advantage/RobotCode2026Public, ascope_assets/Robot_Darwin: their
# Onshape assembly "6328-26C-0000" exported to glTF, Y up, z forward, x to the robot's left; the
# public Onshape document, 2da573df9d139d7a1e0f08a8, is view-only and won't export). model.glb is
# the robot; model_0 the hood (as exported at their 10 deg, Hood.minAngle), model_1 the intake
# (exported folded up, its slot pin at 147.5 deg from its pivot), model_2 the hopper's sliding front
# (its front wall and the two dynamic walls, exported in), model_3 the roof they ran before the net
# (left out: js/mech/r6328.js NET). Every surface FUEL could touch from the intake to the drum,
# simplified (vertices snapped to a 1 cm grid), the rollers js/mech/r6328.js turns cut out. Writes
# js/mech/path6328.js: {body, intake, slide, hood: {v, i}}, vertices in mm (int), in the robot's
# frame (x forward, y up, z to its right): the body as it is; the intake about its pivot, as
# exported; the sliding front as exported (in); the hood about the drum's axle, as exported. Only
# what a FUEL can reach is kept: flooded with the intake down, the front out and the hood at 10,
# and again with the intake up, the front in and the hood at 41.2.
#   python3 -I tools/path-mesh-6328.py <dir with model.glb, model_0.glb, model_1.glb, model_2.glb>
import json, struct, sys, os, re, math
import numpy as np
from scipy import ndimage
from scipy.spatial import ConvexHull

ROOT = os.path.join(os.path.dirname(os.path.abspath(__file__)), '..')
GRID = 0.01
SRC = sys.argv[1]
IN = 0.0254

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
    out = []
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

def robot(T):  # the glTF (x left, y up, z forward) -> robot (x forward, y up, z right)
    return np.stack([T[..., 2], T[..., 1], -T[..., 0]], -1)

# hardware, drives and electronics: nothing a FUEL meets; and what js/mech/r6328.js turns (its
# rollers, the drum, the hood's back roller, the intake's roller), cut out by name here and by
# cylinder below. The bumpers (the model's boxes "Part 1".."Part 5" at the top level) are
# js/mech/r6328.js's Block. The roof variant's lift (the hopper's X44 gearbox, chain, jackshaft and
# sticks) is under the floor or outside the walls; the roof itself (model_3) isn't loaded.
SKIP = re.compile(r'screw|bolt|\bnut\b|locknut|washer|rivet|spacer|bearing|bushing|collar|chain|belt|sprocket|gear\b|spur gear|pulley|kraken|motor|'
                  r'radio|battery|roborio|pdp|pigeon|canivore|breaker|mac mini|signal light|mpm|standoff|press-fit|endcap|end plug|plug$|'
                  r'BASIC Roller|LIVE Axle|25-Roller|Assembly 1|Launcher Back Roller|Rear Hood Roller Cap|Drum$|Drum Cap|Hood Back Roller|'
                  r'2300 Roller|Inertia|Housing Template|Dart Adapter|WCP-0581|Hex \(|THUNDERHEX|Deadaxle|tensioner|Gearbox|Jack Shaft|swerve|MK5|'
                  r'wheel|module', re.I)
TOP_BUMPER = re.compile(r'^Part [1-5]$')
# Flat plates FUEL lean on, full of holes and pockets: each as one sheet over its outline (the convex
# hull in its plane), which 1 cm detail keeps
PLATES = re.compile(r'Front Wall$|Dynamic Wall$|Intake Side Plate$|Intake Ramp Plate$|Roller Plate$|Tower (Left|Right) Side Plate$|Hood (Left|Right) Plate$|Sandwich|^.*Ramp$', re.I)
def sheet(T):
    P = T.reshape(-1, 3)
    lo, hi = P.min(0), P.max(0)
    k = int(np.argmin(hi - lo)); u, v = [j for j in range(3) if j != k]
    h = ConvexHull(P[:, [u, v]]).vertices
    Q = np.zeros((len(h), 3)); Q[:, u] = P[h, u]; Q[:, v] = P[h, v]; Q[:, k] = (lo[k] + hi[k]) / 2
    return np.array([[Q[0], Q[i], Q[i + 1]] for i in range(1, len(Q) - 1)])
def gather(f, top=None):
    out = []
    for p, T in load(os.path.join(SRC, f)):
        if len(p) == 2 and TOP_BUMPER.search(p[1]): continue
        if any(SKIP.search(x) for x in p[1:]): continue
        name = ' / '.join(p)
        if top and not re.search(top, name): continue
        if PLATES.search(p[-1]): T = sheet(T)
        out.append((name, robot(T)))
    return out
body = [T for n, T in gather('model.glb') if 'Intake <1>' not in n or re.search(r'Hard ?[Ss]top|Pivot Gearbox|Pivot MAXSpline|Hardstop', n)]
body = np.vstack(body)
# (the intake's ramp plate, its mounting blocks, cross tubes and bash bar are the export's older
# revision's; js/mech/r6328.js INTAKE draws the final one's from the Onshape document's views)
intake = np.vstack([T for n, T in gather('model_1.glb') if not re.search(r'Ramp Plate|Ramp Mounting Block|Cross Steel Tube|Bash Bar', n)])
slide = np.vstack([T for n, T in gather('model_2.glb')])
hood = np.vstack([T for n, T in gather('model_0.glb')])

# (js/mech/r6328.js has the same layout; ROLLER is the export's, cut out with its roller)
PIVOT = (0.317, 0.1855)               # the intake's pivot (their robotToPivotX 12.48in, 0.186 up)
DRUM = (-0.2639, 0.4861)              # the drum's axle, which the hood turns about
FLOOR = [(-0.1587, 0.1905), (-0.1114, 0.1779), (-0.0620, 0.1780), (-0.0140, 0.1840), (0.0350, 0.1900), (0.0835, 0.1960), (0.1320, 0.2020), (0.1810, 0.2080), (0.2290, 0.2130)]
BACK = [(-0.2059, 0.2186), (-0.2510, 0.2816), (-0.2664, 0.3703)]
KICK = (-0.0789, 0.3572)
HOODROLL = (-0.0638, 0.5214)
ROLLER = (0.1652, 0.4308)             # the intake's 2in roller as exported
def hopX(deg):  # their DarwinMechanism3d: where the sliding front is with the intake's pin at deg
    t = math.radians(deg)
    return (12.48 + 7.204 * math.cos(t) + (7.204 * math.sin(t) - 2.393) / math.tan(math.radians(70)) + 6.789) * IN
EXPORT = 147.5

def cyl(c, r, h=0.31): return (np.array([c[0], c[1], 0.0]), np.array([0, 0, 1.0]), r, h)
def cut(T, cyls):
    c = T.mean(1)
    keep = np.ones(len(T), bool)
    for (p, a, r, h) in cyls:
        d = c - p; s = d @ a
        rad = np.linalg.norm(d - np.outer(s, a), axis=1)
        keep &= ~((np.abs(s) <= h) & (rad <= r))
    return T[keep]
BODY = cut(body, [cyl(p, 0.024) for p in FLOOR + BACK] + [cyl(KICK, 0.036), cyl(DRUM, 0.056)])
INTAKE = cut(intake, [cyl(ROLLER, 0.031)])
HOOD = cut(hood, [cyl(HOODROLL, 0.03), cyl(DRUM, 0.056)])
SLIDE = slide

def rot2(T, c, a):
    T = T.copy(); x, y = T[..., 0] - c[0], T[..., 1] - c[1]; co, si = math.cos(a), math.sin(a)
    T[..., 0] = c[0] + co * x - si * y; T[..., 1] = c[1] + si * x + co * y; return T
def posed(deg, hood):
    return {'body': BODY, 'intake': rot2(INTAKE, PIVOT, math.radians(deg - EXPORT)),
            'slide': SLIDE + np.array([hopX(deg) - hopX(EXPORT), 0, 0]), 'hood': rot2(HOOD, DRUM, math.radians(hood - 10))}

VOX, R = 0.01, 0.075
LO, HI = np.array([-0.40, 0.0, -0.40]), np.array([0.78, 0.74, 0.40])
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
def flood(groups, deg, hood, front):
    pts = {k: samples(T) for k, T in groups.items()}
    solid = np.zeros(shape, bool)
    for P, _ in pts.values():
        ijk, ok = voxels(P)
        solid[tuple(ijk[ok].T)] = True
    # (the rollers too: a FUEL can't be inside one)
    ang = math.radians(deg - EXPORT)
    rp = rot2(np.array([[ROLLER[0], ROLLER[1], 0.0]]), PIVOT, ang)[0]
    for (x, y), r in [*((p, 0.02) for p in FLOOR + BACK), (KICK, 0.032), (DRUM, 0.051), (tuple(rot2(np.array([[*HOODROLL, 0.0]]), DRUM, math.radians(hood - 10))[0][:2]), 0.0254), ((rp[0], rp[1]), 0.0264)]:
        solid |= ((np.hypot(g[:, 0] - x, g[:, 1] - y) < r) & (np.abs(g[:, 2]) < 0.3)).reshape(shape)
    solid[:, 0, :] = True
    # (the outside: beside the walls, behind the robot, out past the front and over the net)
    out = (np.abs(g[:, 2]) > 0.345) | (g[:, 0] > front) | (g[:, 0] < -0.37) | (g[:, 1] > 0.70)
    # (and the bumper and the frame under the floor: js/mech/r6328.js's Block and the frame)
    out |= (g[:, 0] > 0.34) & (g[:, 0] < 0.42) & (g[:, 1] < 0.16)
    solid |= out.reshape(shape)
    free = ndimage.distance_transform_edt(~solid) * VOX >= R - 0.015
    lab, _ = ndimage.label(free)
    seeds = np.array([[0.1, 0.32, 0], [0.2, 0.32, 0.2], [0.2, 0.32, -0.2], [0.0, 0.45, 0], [-0.16, 0.30, 0], [-0.17, 0.40, 0], [-0.17, 0.50, 0.1],
                      [0.55, 0.35, 0], [0.62, 0.075, 0], [0.62, 0.075, 0.2], [0.62, 0.075, -0.2], [0.5, 0.25, 0.15]])
    sk, ok = voxels(seeds)
    ids = set(lab[tuple(sk[ok].T)].tolist()) - {0}
    reach = ndimage.distance_transform_edt(~np.isin(lab, list(ids))) * VOX <= R + 0.02
    keep = {}
    for k, T in groups.items():
        P, tri = pts[k]
        ijk, ok = voxels(P)
        hit = np.zeros(len(P), bool)
        hit[ok] = reach[tuple(ijk[ok].T)]
        kk = np.zeros(len(T), bool)
        np.logical_or.at(kk, tri, hit)
        keep[k] = kk
    print(f'pose {deg}/{hood}: seeded regions {len(ids)} of {lab.max()}; ' + ', '.join(f'{k} {keep[k].sum()}/{len(T)}' for k, T in groups.items()))
    return keep
k1 = flood(posed(17.4, 10), 17.4, 10, 0.76)
k2 = flood(posed(145.8, 41.2), 145.8, 41.2, 0.48)
keep = {k: k1[k] | k2[k] for k in k1}

def simplify(T):
    V = np.round(T.reshape(-1, 3) / GRID).astype(np.int64)
    uniq, inv = np.unique(V, axis=0, return_inverse=True)
    I = inv.reshape(-1, 3)
    I = I[(I[:, 0] != I[:, 1]) & (I[:, 1] != I[:, 2]) & (I[:, 0] != I[:, 2])]
    I = np.unique(np.sort(I, axis=1), axis=0)
    used, I2 = np.unique(I, return_inverse=True)
    return np.round(uniq[used] * GRID * 1000).astype(int), I2.reshape(-1, 3)
P0 = np.array([PIVOT[0], PIVOT[1], 0]); D0 = np.array([DRUM[0], DRUM[1], 0])
meshes = {'body': simplify(BODY[keep['body']]), 'intake': simplify(INTAKE[keep['intake']] - P0), 'slide': simplify(SLIDE[keep['slide']]), 'hood': simplify(HOOD[keep['hood']] - D0)}
res = {k: {'v': v.flatten().tolist(), 'i': i.flatten().tolist()} for k, (v, i) in meshes.items()}
with open(f'{ROOT}/js/mech/path6328.js', 'w') as f:
    f.write('// 6328\'s ball path\'s surfaces from their CAD (generated by tools/path-mesh-6328.py; see js/mech/r6328.js)\n')
    f.write('export default ' + json.dumps(res, separators=(',', ':')) + ';\n')
for k, (v, i) in meshes.items(): print(k, len(v), 'vertices', len(i), 'triangles')
print(os.path.getsize(f'{ROOT}/js/mech/path6328.js') // 1024, 'KB')
