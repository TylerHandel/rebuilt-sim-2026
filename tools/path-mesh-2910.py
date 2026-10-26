# 2910's ball path as collision geometry, from their full CAD (Onshape "12 - 2026 ReBlitz Top
# Level Assembly", which their CAD release links): every surface FUEL could touch from the intake
# to the shooter (only those a FUEL can reach, found by flooding the space it fits through),
# simplified (vertices snapped to a 1 cm grid) and with the wheels js/mech/r2910.js turns cut out.
# Writes js/mech/path2910.js: {body, intake, hood: {v, i}}, vertices in mm (int), in the robot's
# frame (x forward, y up, z to its right) for the body, on the intake's pivot for the intake (as
# exported, deployed), on the drum's axle for the hood (as exported: their Max_Hood_Angle).
# The full CAD isn't in the repo (15 MB). Get it with:
#   DID=dfb391aac173a4555d00a5b5 WID=3dc64f602735252892b0e47b tools/onshape-export.sh r2910.glb 6c654da4eb6b1710fb0900bd
# (their configuration List_gTJHVM5EzW0JZB=Max_Hood_Angle), then: python3 -I tools/path-mesh-2910.py r2910.glb
import json, struct, sys, os, re
import numpy as np
from scipy import ndimage

ROOT = os.path.join(os.path.dirname(os.path.abspath(__file__)), '..')
GRID = 0.01
IN = 0.0254
SRC = sys.argv[1] if len(sys.argv) > 1 else 'r2910.glb'

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

# hardware and electronics, nothing a FUEL meets in the ball path, and the FUEL their CAD shows
# waiting in the shooter
SKIP = re.compile(r'screw|bolt|\bnut\b|locknut|washer|rivet|spacer|bearing|collar|chain|belt|sprocket|gear|pulley|kraken|motor|'
                  r'cancoder|canrange|limelight|radio|signal light|battery|roborio|pdp|pigeon|canivore|breaker|anderson|grommet|'
                  r'shim|retaining|pin\b|standoff|tube plug|adapter|hub|origin cube|zip|wire|^fuel\b', re.I)
parts = load(SRC)
def robot(T):  # CAD (x across, y back, z up) -> robot (x forward, y up, z right)
    return np.stack([-T[..., 1], T[..., 2], -T[..., 0]], -1)
body, intake, hood = [], [], []
for p, T in parts:
    name = ' / '.join(p)
    if SKIP.search(p[-1]) or SKIP.search(p[-2] if len(p) > 1 else ''): continue
    T = robot(T)
    if 'Pivoting Intake Assembly' in name: intake.append(T)
    elif '32-17 Hood Assembly' in name: hood.append(T)
    else: body.append(T)
body, intake, hood = np.vstack(body), np.vstack(intake), np.vstack(hood)

# the wheels the mechanism turns (robot frame), cut out: [center, axis, radius, half length]
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
FLOOR = [(0.256, 0.195), (0.194, 0.179), (0.132, 0.162), (0.071, 0.146), (0.009, 0.129),   # 1.25in silicone
         (-0.053, 0.113), (-0.161, 0.113), (-0.251, 0.174),                                     # thin aluminum
         (-0.289, 0.261), (-0.278, 0.356)]                                                      # up the back
wheels = [cyl([x, y, 0], Z, 0.032, 0.32) for x, y in FLOOR]
wheels += [cyl([-0.107, 0.289, 0], Z, 0.042, 0.33)]          # the indexer's compliant wheels and shaft
wheels += [cyl([-0.283, 0.470, 0], Z, 0.068, 0.32)]          # the drum, its end wheels
wheels += [cyl([-0.265, 0.373, 0.331], Z, 0.085, 0.02), cyl([-0.265, 0.373, -0.331], Z, 0.085, 0.02)]  # the flywheel
BODY = cut(body, wheels)
INTAKE = cut(intake, [cyl([0.610, 0.161, 0], Z, 0.040, 0.31), cyl([0.448, 0.103, 0], Z, 0.030, 0.32), cyl([0.483, 0.042, 0], Z, 0.030, 0.32)])
HOOD = cut(hood, [cyl([-0.081, 0.461, 0], Z, 0.032, 0.32), cyl([-0.084, 0.508, 0], Z, 0.032, 0.32)])

def simplify(T):
    V = np.round(T.reshape(-1, 3) / GRID).astype(np.int64)
    uniq, inv = np.unique(V, axis=0, return_inverse=True)
    I = inv.reshape(-1, 3)
    I = I[(I[:, 0] != I[:, 1]) & (I[:, 1] != I[:, 2]) & (I[:, 0] != I[:, 2])]
    I = np.unique(np.sort(I, axis=1), axis=0)
    used, I2 = np.unique(I, return_inverse=True)
    return np.round(uniq[used] * GRID * 1000).astype(int), I2.reshape(-1, 3)

# Only what a FUEL can get to (as 8793's: tools/path-mesh.py): the space a FUEL's center fits in,
# allowing 1.5 cm of squash, flooded from points along the ball path, and the surfaces within a
# FUEL's reach of it
VOX, R = 0.01, 0.075
LO, HI = np.array([-0.40, 0.0, -0.40]), np.array([0.72, 0.62, 0.40])
shape = tuple(np.ceil((HI - LO) / VOX).astype(int))
def samples(T, step=0.008):
    a, b, c = T[:, 0], T[:, 1], T[:, 2]
    n = np.clip(np.ceil(np.maximum(np.linalg.norm(b - a, axis=1), np.linalg.norm(c - a, axis=1)) / step).astype(int), 1, 60)
    out, tri = [], []
    for k in range(1, 61):
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
pts_i, tri_i = samples(INTAKE)
pts_h, tri_h = samples(HOOD)
seeds = np.array([[0.0, 0.3, 0], [0.2, 0.3, 0.15], [0.2, 0.3, -0.15], [-0.2, 0.25, 0], [-0.19, 0.33, 0], [-0.19, 0.4, 0.1],
                  [0.3, 0.32, 0], [0.5, 0.2, 0], [0.55, 0.2, 0.2], [0.1, 0.45, 0]])
solid = np.zeros(shape, bool)
for P in (pts_b, pts_i, pts_h):
    ijk, ok = voxels(P)
    solid[tuple(ijk[ok].T)] = True
solid[:, 0, :] = True
g = LO[None, :] + (np.indices(shape).reshape(3, -1).T + 0.5) * VOX
out = (g[:, 0] < -0.37) | (g[:, 0] > 0.70) | (np.abs(g[:, 2]) > 0.37) | (g[:, 1] > 0.60)
solid |= out.reshape(shape)
free = ndimage.distance_transform_edt(~solid) * VOX >= R - 0.015
lab, _ = ndimage.label(free)
sk, ok = voxels(seeds)
ids = set(lab[tuple(sk[ok].T)].tolist()) - {0}
print('seeded regions', len(ids), 'of', lab.max())
reach = ndimage.distance_transform_edt(~np.isin(lab, list(ids))) * VOX <= R + 0.02
def reached(P, tri, n):
    ijk, ok = voxels(P)
    hit = np.zeros(len(P), bool)
    hit[ok] = reach[tuple(ijk[ok].T)]
    keep = np.zeros(n, bool)
    np.logical_or.at(keep, tri, hit)
    return keep
kb, ki, kh = reached(pts_b, tri_b, len(BODY)), reached(pts_i, tri_i, len(INTAKE)), reached(pts_h, tri_h, len(HOOD))
print('reachable:', kb.sum(), 'of', len(BODY), 'body;', ki.sum(), 'of', len(INTAKE), 'intake;', kh.sum(), 'of', len(HOOD), 'hood')
PIVOT = np.array([0.273, 0.17, 0]); DRUM = np.array([-0.283, 0.47, 0])
meshes = {'body': simplify(BODY[kb]), 'intake': simplify(INTAKE[ki] - PIVOT), 'hood': simplify(HOOD[kh] - DRUM)}
out = {k: {'v': v.flatten().tolist(), 'i': i.flatten().tolist()} for k, (v, i) in meshes.items()}
with open(f'{ROOT}/js/mech/path2910.js', 'w') as f:
    f.write('// 2910\'s ball path\'s surfaces from their CAD (generated by tools/path-mesh-2910.py; see js/mech/r2910.js)\n')
    f.write('export default ' + json.dumps(out, separators=(',', ':')) + ';\n')
for k, (v, i) in meshes.items(): print(k, len(v), 'vertices', len(i), 'triangles')
print(os.path.getsize(f'{ROOT}/js/mech/path2910.js') // 1024, 'KB')
