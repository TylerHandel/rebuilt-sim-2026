# 581's ball path as collision geometry, from their CAD (Onshape "581 2026 Rubble", the "2026 Dumper
# Champs Bot" assembly): every surface FUEL could touch from the intake to the drum, simplified
# (vertices snapped to a 1 cm grid) and with the rollers js/mech/r581.js turns cut out. Writes
# js/mech/path581.js: {body, slide, floor, hood: {v, i}}, vertices in mm (int), in the robot's frame
# (x forward, y up, z to its right): the body as it is; the intake that slides out on its racks, as
# exported (all the way out, 0.3047 m), with its kicker plates where they rest pushed back against
# the bumper (the STEP's pose); the roller floor on its pivot (the shaft it's powered through, at the
# shooter tower), as exported (down); the hood on the drum's axle, as exported (their 11 deg, its
# hard stop, the steepest: the AdvantageScope model's zero). Only what a FUEL can reach is kept.
# The assembly's glTF export only carries its shooter, so this reads the document's own STEP (blob
# element 1ca4a779b642bbef0c0597fb, 70 MB), tessellated by tools/step2glb.py:
#   curl -sSfL --location-trusted -u "$ONSHAPE_ACCESS_KEY:$ONSHAPE_SECRET_KEY" -o r581.step \
#     https://cad.onshape.com/api/v6/blobelements/d/766d37b113335d7a60cc2375/w/8b35fc383279c1954cec5b72/e/1ca4a779b642bbef0c0597fb
#   python tools/step2glb.py r581.step r581-step.glb --lin 0.001 --ang 0.4 \
#     --skip "screw|bolt|\bnut\b|nutstrip|locknut|washer|rivet|spacer|bearing|bushing|collar|shim|standoff|BHCS|SHCS|FHCS|zip|grommet"
# then: python3 -I tools/path-mesh-581.py r581-step.glb
# (The STEP's frame is the glTF export's moved 2.5 mm forward and 3 mm down: its drum's axle is at
# (-0.2883, 0.4788), the glTF's mates put it at (-0.2858, 0.4762).)
import json, struct, sys, os, re
import numpy as np
from scipy import ndimage
from scipy.spatial import ConvexHull

ROOT = os.path.join(os.path.dirname(os.path.abspath(__file__)), '..')
GRID = 0.01
SRC = sys.argv[1] if len(sys.argv) > 1 else 'r581-step.glb'

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

# hardware, electronics and drive parts, nothing a FUEL meets in the ball path
SKIP = re.compile(r'screw|bolt|\bnut\b|nutstrip|locknut|washer|rivet|spacer|bearing|bushing|collar|chain|belt|sprocket|gear|pulley|kraken|motor|'
                  r'thunder|flanged|FR6|FR8|hex adapter|hex insert|limelight|\bLL[34]\b|radio|battery|roborio|pdp|pigeon|canivore|breaker|'
                  r'anderson|grommet|candle|vrm|triad|shim|standoff|zip|wire|energy chain|crushblock|origin$|simplified|import for mass|'
                  r'swerve x2|wheel module|wheel assembly|molded wheel|encoder|simpson|spring mount|WCP-0326|WCP-0063|WCP-0189|26D0501|PEM', re.I)
parts = load(SRC)
def robot(T):  # CAD (x back, y right, z up) -> robot (x forward, y up, z right)
    return np.stack([-T[..., 0], T[..., 2], T[..., 1]], -1)
# what of the intake assembly stays put (the deploy's gearboxes and slider blocks) and what slides
INTAKE_FIXED = re.compile(r'Deploy [Pp]late|Slider [Bb]lock|Kraken X44|10T Aluminum Spur|50T 20DP|10T on 12T')
# Flat plates FUEL lean on, full of holes and pockets in the CAD (the hopper's polycarbonate top and
# walls, the floor's side plates, the tower's and hood's side plates, the intake's side panels and
# front plate): each as one sheet over its outline (the convex hull in its plane), which 1 cm
# detail keeps (their many small triangles would vanish)
PLATES = re.compile(r'Part 22$|hopper poly$|Hopper <1> / Part 18$|floor funnel$|Front Intake Hopper$|Side Panels$|Left Plate$|Right Plate$|Hood Plate$|Floor Assembly.* / Part [12]$')
def sheet(T):
    P = T.reshape(-1, 3)
    lo, hi = P.min(0), P.max(0)
    k = int(np.argmin(hi - lo)); u, v = [j for j in range(3) if j != k]
    h = ConvexHull(P[:, [u, v]]).vertices
    Q = np.zeros((len(h), 3)); Q[:, u] = P[h, u]; Q[:, v] = P[h, v]; Q[:, k] = (lo[k] + hi[k]) / 2
    return np.array([[Q[0], Q[i], Q[i + 1]] for i in range(1, len(Q) - 1)])
body, slide, floor, hood = [], [], [], []
for p, T in parts:
    name = ' / '.join(p)
    if any(SKIP.search(x) for x in p[1:]): continue
    if PLATES.search(name): T = sheet(T)
    T = robot(T)
    if 'Champs Intake Assembly' in name: (body if INTAKE_FIXED.search(name) else slide).append(T)
    elif 'Floor Assembly' in name: floor.append(T)
    elif 'Hood Assem' in name: hood.append(T)
    else: body.append(T)
body, slide, floor, hood = (np.vstack(x) for x in (body, slide, floor, hood))

# the rollers the mechanism turns (robot frame), cut out: [center, axis, radius, half length]
def cyl(c, a, r, h): return (np.array(c, float), np.array(a, float) / np.linalg.norm(a), r, h)
def cut(T, cyls):
    c = T.mean(1)
    keep = np.ones(len(T), bool)
    for (p, a, r, h) in cyls:
        d = c - p; s = d @ a
        rad = np.linalg.norm(d - np.outer(s, a), axis=1)
        keep &= ~((np.abs(s) <= h) & (rad <= r))
    return T[keep]
Z = [0, 0, 1]
# (r581.js has the same: the floor's eleven 1in rollers, back to front; the tower's three 1.5in,
# bottom to top; the drum; the hood's two 2in; the intake's 3in and 2in)
FLOOR = [(-0.0701, 0.1105), (-0.0300, 0.1220), (0.0102, 0.1335), (0.0503, 0.1450), (0.0904, 0.1564), (0.1306, 0.1679),
         (0.1707, 0.1794), (0.2123, 0.1823), (0.2540, 0.1852), (0.2938, 0.1975), (0.3237, 0.1684)]
TOWER = [(-0.1142, 0.2789), (-0.1027, 0.3454), (-0.0911, 0.4119)]
DRUM = (-0.2883, 0.4788)
HOOD = [(-0.0820, 0.4615), (-0.0865, 0.5245)]
BODY = cut(body, [cyl([x, y, 0], Z, 0.024, 0.30) for x, y in TOWER] + [cyl([*DRUM, -0.01], Z, 0.056, 0.31)])
FLOOR_T = cut(floor, [cyl([x, y, 0], Z, 0.017, 0.27) for x, y in FLOOR])
HOOD_T = cut(hood, [cyl([x, y, 0], Z, 0.031, 0.30) for x, y in HOOD])
SLIDE = cut(slide, [cyl([0.5794, 0.1781, 0], Z, 0.045, 0.33), cyl([0.5307, 0.2345, 0], Z, 0.034, 0.335)])

def simplify(T):
    V = np.round(T.reshape(-1, 3) / GRID).astype(np.int64)
    uniq, inv = np.unique(V, axis=0, return_inverse=True)
    I = inv.reshape(-1, 3)
    I = I[(I[:, 0] != I[:, 1]) & (I[:, 1] != I[:, 2]) & (I[:, 0] != I[:, 2])]
    I = np.unique(np.sort(I, axis=1), axis=0)
    used, I2 = np.unique(I, return_inverse=True)
    return np.round(uniq[used] * GRID * 1000).astype(int), I2.reshape(-1, 3)

# Only the surfaces a FUEL can get to (as 2910's: tools/path-mesh-2910.py): the space a FUEL's
# center fits in, allowing 1.5 cm of squash, flooded from points along the ball path (everything
# as exported: the intake out, the floor down, the hood at its stop), and the surfaces within a
# FUEL's reach of it. The intake out front is open to the carpet; its sides and the space round the
# robot are walled off at the frame's sides and the intake's front.
VOX, R = 0.01, 0.075
LO, HI = np.array([-0.42, 0.0, -0.42]), np.array([0.74, 0.72, 0.42])
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
groups = {'body': BODY, 'slide': SLIDE, 'floor': FLOOR_T, 'hood': HOOD_T}
pts = {k: samples(T) for k, T in groups.items()}
solid = np.zeros(shape, bool)
for P, _ in pts.values():
    ijk, ok = voxels(P)
    solid[tuple(ijk[ok].T)] = True
# (the rollers too: a FUEL can't be inside one)
g = LO[None, :] + (np.indices(shape).reshape(3, -1).T + 0.5) * VOX
for (x, y), r in [*((p, 0.0127) for p in FLOOR), *((p, 0.019) for p in TOWER), (DRUM, 0.051), *((p, 0.0254) for p in HOOD), ((0.5794, 0.1781), 0.0397), ((0.5307, 0.2345), 0.0286)]:
    solid |= ((np.hypot(g[:, 0] - x, g[:, 1] - y) < r) & (np.abs(g[:, 2]) < 0.3)).reshape(shape)
solid[:, 0, :] = True
# (the outside: beside the frame, behind it, out past the intake's front and over the top)
out = (np.abs(g[:, 2]) > 0.37) | (g[:, 0] > 0.72) | (g[:, 0] < -0.41) | (g[:, 1] > 0.70)
solid |= out.reshape(shape)
free = ndimage.distance_transform_edt(~solid) * VOX >= R - 0.015
lab, _ = ndimage.label(free)
seeds = np.array([[0.1, 0.3, 0], [0.25, 0.3, 0.2], [0.25, 0.3, -0.2], [0.45, 0.35, 0], [0.62, 0.08, 0], [0.62, 0.08, 0.2], [0.62, 0.08, -0.2],
                  [-0.17, 0.22, 0], [-0.19, 0.33, 0], [-0.19, 0.42, 0.15], [-0.2, 0.62, 0], [0.0, 0.45, 0.2]])
sk, ok = voxels(seeds)
ids = set(lab[tuple(sk[ok].T)].tolist()) - {0}
print('seeded regions', len(ids), 'of', lab.max())
reach = ndimage.distance_transform_edt(~np.isin(lab, list(ids))) * VOX <= R + 0.02
def reached(P, tri, n):
    ijk, ok = voxels(P)
    hit = np.zeros(len(P), bool)
    hit[ok] = reach[tuple(ijk[ok].T)]
    k = np.zeros(n, bool)
    np.logical_or.at(k, tri, hit)
    return k
keep = {k: reached(*pts[k], len(T)) for k, T in groups.items()}
for k, T in groups.items(): print(f'reachable {k}: {keep[k].sum()} of {len(T)}')
PIVOT = np.array([FLOOR[0][0], FLOOR[0][1], 0]); AXLE = np.array([*DRUM, 0])
meshes = {'body': simplify(BODY[keep['body']]), 'slide': simplify(SLIDE[keep['slide']]), 'floor': simplify(FLOOR_T[keep['floor']] - PIVOT), 'hood': simplify(HOOD_T[keep['hood']] - AXLE)}
res = {k: {'v': v.flatten().tolist(), 'i': i.flatten().tolist()} for k, (v, i) in meshes.items()}
with open(f'{ROOT}/js/mech/path581.js', 'w') as f:
    f.write('// 581\'s ball path\'s surfaces from their CAD (generated by tools/path-mesh-581.py; see js/mech/r581.js)\n')
    f.write('export default ' + json.dumps(res, separators=(',', ':')) + ';\n')
for k, (v, i) in meshes.items(): print(k, len(v), 'vertices', len(i), 'triangles')
print(os.path.getsize(f'{ROOT}/js/mech/path581.js') // 1024, 'KB')
