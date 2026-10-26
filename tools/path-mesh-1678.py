# 1678's ball path as collision geometry, from their full CAD (Onshape "1678-26c-0000 CAD Release"):
# every surface FUEL could touch from the intake to the drum, simplified (vertices snapped to a 1 cm
# grid) and with the rollers js/mech/r1678.js turns cut out. Writes js/mech/path1678.js:
# {body, intake, hood, slide, lid: {v, i}}, vertices in mm (int), in the robot's frame (x forward,
# y up, z to its right): the body as it is; the intake on its pivot, as exported (folded up inside
# the hopper: their 130 deg); the hood on the drum's axle, as exported (their 17.5 deg, the
# steepest); the extension's side plates and front panel, and the climber's lid, as exported
# (retracted, down). Only the body is pruned to what a FUEL can reach; the moving parts are kept
# whole (they go everywhere).
# The full CAD isn't in the repo (17 MB). Get it with:
#   DID=acdaaf42764a293a9452326d WID=f887ef4c6a254c09c73344c3 tools/onshape-export.sh r1678.glb a9684fa7665096e977b19c44
# then: python3 -I tools/path-mesh-1678.py r1678.glb
import json, struct, sys, os, re
import numpy as np
from scipy import ndimage

ROOT = os.path.join(os.path.dirname(os.path.abspath(__file__)), '..')
GRID = 0.01
SRC = sys.argv[1] if len(sys.argv) > 1 else 'r1678.glb'

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

# hardware and electronics, nothing a FUEL meets in the ball path (and the cameras' view cones)
SKIP = re.compile(r'screw|bolt|\bnut\b|locknut|washer|rivet|spacer|bearing|bushing|collar|chain|belt|sprocket|gear|pulley|pully|kraken|motor|'
                  r'cancoder|canrange|limelight|radio|signal light|battery|roborio|pdp|pigeon|canivore|breaker|anderson|grommet|candle|vrm|'
                  r'shim|retaining|pin\b|standoff|tube plug|adapter|origin cube|reference cube|zip|wire|^fuel\b|bhcs|shcs|fhcs', re.I)
parts = load(SRC)
def robot(T):  # CAD (x across, y back, z up) -> robot (x forward, y up, z right)
    return np.stack([-T[..., 1], T[..., 2], -T[..., 0]], -1)
# the intake's parts that don't swing: the pivot's gearbox and bearing plates, and its tube
INTAKE_FIXED = re.compile(r'1678-26a-0412|1678-26a-0413|1678-25-P-021|1678-26a-0415|1678-26a-0409|1678-26c-1530|1678-26c-1531|WCP-1392|Part 40\b|Part 12\b|Part 24\b|Part 25\b|Part 29\b|Rounded Hex|Part 7\b')
SLIDE = re.compile(r'1678-26c-1529|part 26\b|Part 41\b|part 73\b')
HOOD = re.compile(r'1678-26c-16(09|10|12|13|14|15|17|27|74|75|85)\b|Hood silicone')
body, intake, hood, slide, lid = [], [], [], [], []
for p, T in parts:
    name = ' / '.join(p)
    if SKIP.search(p[-1]) or SKIP.search(p[-2] if len(p) > 1 else ''): continue
    T = robot(T)
    # (a 2in cube at the CAD's origin, on the carpet, came along in the intake's assembly)
    if np.abs(T).max() < 0.03: continue
    if 'Single Roller Intake' in name:
        (body if INTAKE_FIXED.search(name) else slide if SLIDE.search(name) else intake).append(T)
    elif 'Climber' in name:
        # the climber's tubes telescope: what reaches the top (the inner tubes, the lid and its
        # skirts) rises; the outer tubes and the gearbox stay
        (lid if T[..., 1].max() > 0.49 else body).append(T)
    elif 'Shooter+Ball Tunnel' in name and HOOD.search(name): hood.append(T)
    else: body.append(T)
body, intake, hood, slide, lid = (np.vstack(x) for x in (body, intake, hood, slide, lid))

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
FLOOR = [(0.323, 0.215), (0.262, 0.208), (0.214, 0.202), (-0.015, 0.126), (-0.075, 0.115)]     # polycarbonate rollers
FLEX = [(0.159, 0.179), (0.101, 0.160), (0.042, 0.142)]                                          # 2in flex wheel shafts
BACK = [(-0.147, 0.122), (-0.201, 0.154), (-0.247, 0.212), (-0.266, 0.284), (-0.266, 0.346), (-0.266, 0.408)]  # up the tunnel's back
FRONT = [(-0.090, 0.346), (-0.090, 0.408)]
wheels = [cyl([x, y, 0], Z, 0.017, 0.30) for x, y in FLOOR]
wheels += [cyl([x, y, 0], Z, 0.031, 0.27) for x, y in FLEX]
wheels += [cyl([x, y, 0], Z, 0.019, 0.30) for x, y in BACK + FRONT]
wheels += [cyl([-0.082, 0.294, 0], Z, 0.030, 0.30)]           # the feeder's 2in roller
wheels += [cyl([-0.288, 0.477, 0], Z, 0.050, 0.33)]           # the drum, its end discs and hubs
BODY = cut(body, wheels)
INTAKE = cut(intake, [cyl([0.143, 0.368, 0], Z, 0.030, 0.32), cyl([0.253, 0.247, 0], Z, 0.012, 0.335),
                      cyl([0.131, 0.329, 0], Z, 0.012, 0.335), cyl([0.312, 0.322, 0], Z, 0.019, 0.335)])
HOOD_T = cut(hood, [cyl([-0.093, 0.490, 0], Z, 0.025, 0.33), cyl([-0.094, 0.448, 0], Z, 0.025, 0.33), cyl([-0.101, 0.536, 0], Z, 0.025, 0.33)])

def simplify(T):
    V = np.round(T.reshape(-1, 3) / GRID).astype(np.int64)
    uniq, inv = np.unique(V, axis=0, return_inverse=True)
    I = inv.reshape(-1, 3)
    I = I[(I[:, 0] != I[:, 1]) & (I[:, 1] != I[:, 2]) & (I[:, 0] != I[:, 2])]
    I = np.unique(np.sort(I, axis=1), axis=0)
    used, I2 = np.unique(I, return_inverse=True)
    return np.round(uniq[used] * GRID * 1000).astype(int), I2.reshape(-1, 3)

# Only the body's surfaces a FUEL can get to (as 2910's: tools/path-mesh-2910.py): the space a
# FUEL's center fits in, allowing 1.5 cm of squash, flooded from points along the ball path, and
# the surfaces within a FUEL's reach of it. (The moving parts aren't solid here, so it floods
# everywhere they might be.)
VOX, R = 0.01, 0.075
LO, HI = np.array([-0.42, 0.0, -0.42]), np.array([0.72, 0.80, 0.42])
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
pts_b, tri_b = samples(BODY)
seeds = np.array([[0.1, 0.35, 0], [0.25, 0.35, 0.2], [0.25, 0.35, -0.2], [-0.15, 0.25, 0], [-0.17, 0.38, 0], [-0.17, 0.45, 0.15],
                  [0.5, 0.3, 0], [0.55, 0.08, 0], [0.0, 0.3, 0.2], [-0.2, 0.62, 0]])
solid = np.zeros(shape, bool)
ijk, ok = voxels(pts_b)
solid[tuple(ijk[ok].T)] = True
solid[:, 0, :] = True
free = ndimage.distance_transform_edt(~solid) * VOX >= R - 0.015
lab, _ = ndimage.label(free)
sk, ok = voxels(seeds)
ids = set(lab[tuple(sk[ok].T)].tolist()) - {0}
print('seeded regions', len(ids), 'of', lab.max())
reach = ndimage.distance_transform_edt(~np.isin(lab, list(ids))) * VOX <= R + 0.02
ijk, ok = voxels(pts_b)
hit = np.zeros(len(pts_b), bool)
hit[ok] = reach[tuple(ijk[ok].T)]
kb = np.zeros(len(BODY), bool)
np.logical_or.at(kb, tri_b, hit)
print('reachable:', kb.sum(), 'of', len(BODY), 'body')
PIVOT = np.array([0.305, 0.169, 0]); DRUM = np.array([-0.288, 0.477, 0])
meshes = {'body': simplify(BODY[kb]), 'intake': simplify(INTAKE - PIVOT), 'hood': simplify(HOOD_T - DRUM), 'slide': simplify(slide), 'lid': simplify(lid)}
out = {k: {'v': v.flatten().tolist(), 'i': i.flatten().tolist()} for k, (v, i) in meshes.items()}
with open(f'{ROOT}/js/mech/path1678.js', 'w') as f:
    f.write('// 1678\'s ball path\'s surfaces from their CAD (generated by tools/path-mesh-1678.py; see js/mech/r1678.js)\n')
    f.write('export default ' + json.dumps(out, separators=(',', ':')) + ';\n')
for k, (v, i) in meshes.items(): print(k, len(v), 'vertices', len(i), 'triangles')
print(os.path.getsize(f'{ROOT}/js/mech/path1678.js') // 1024, 'KB')
