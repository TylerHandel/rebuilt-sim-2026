# 341's ball path as collision geometry, from their full CAD (Onshape "Miss Daisy XXIV",
# 412a0db47e00e2399ebca57e): every surface FUEL could touch from the intake to the shooter,
# simplified (vertices snapped to a 1 cm grid) and with the wheels js/mech/r341.js turns cut out.
# Writes js/mech/path341.js: {body, intake, turret, hood: {v, i}}, vertices in mm (int), in the
# robot's frame (x forward, y up, z to its right): the body as it is; the intake on its pivot (the
# top roller's shaft), as exported (folded up: their 0 deg); the turret on its axis, facing forward
# (as exported); the hood on the flywheel's axle, in the turret's frame, as exported.
# The full CAD isn't in the repo (28 MB). Get it with:
#   DID=412a0db47e00e2399ebca57e WID=b607133da9f21b6baa148170 tools/onshape-export.sh r341.glb e3c2dacd85e82dde2bb28dd9
# then: python3 -I tools/path-mesh-341.py r341.glb
import json, struct, sys, os, re
import numpy as np
from scipy import ndimage

ROOT = os.path.join(os.path.dirname(os.path.abspath(__file__)), '..')
GRID = 0.01
SRC = sys.argv[1] if len(sys.argv) > 1 else 'r341.glb'

def load(f):
    b = open(f, 'rb').read()
    n = struct.unpack('<I', b[12:16])[0]
    j = json.loads(b[20:20 + n])
    off = 20 + n
    bn = struct.unpack('<I', b[off:off + 4])[0]
    bin_ = b[off + 8:off + 8 + bn]
    CT = {5125: ('I', 4), 5123: ('H', 2), 5121: ('B', 1), 5126: ('f', 4)}
    def acc(i):
        a = j['accessors'][i]; bv = j['bufferViews'][a['bufferView']]
        fmt, sz = CT[a['componentType']]
        nc = {'SCALAR': 1, 'VEC2': 2, 'VEC3': 3, 'VEC4': 4}[a['type']]
        stride = bv.get('byteStride', sz * nc)
        start = bv.get('byteOffset', 0) + a.get('byteOffset', 0)
        raw = np.frombuffer(bin_, dtype=np.uint8, count=stride * (a['count'] - 1) + sz * nc, offset=start)
        return np.lib.stride_tricks.as_strided(raw, shape=(a['count'], nc * sz), strides=(stride, 1)).copy().view(np.dtype('<' + fmt)).reshape(a['count'], nc).astype(np.float64)
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

# hardware and electronics, nothing a FUEL meets in the ball path
SKIP = re.compile(r'screw|bolt|\bnut\b|locknut|washer|rivet|spacer|bearing|bushing|collar|chain|belt|sprocket|gear|pulley|kraken|motor|'
                  r'cancoder|encoder|limelight|radio|signal light|battery|roborio|pdh|pigeon|canivore|breaker|grommet|candle|wago|'
                  r'shim|retaining|standoff|tube plug|adapter|zip|wire|swerve|x44 cover|cable|spring|cfs|hex', re.I)
parts = load(SRC)
def robot(T):  # CAD (x across to the left, y back, z up) -> robot (x forward, y up, z right)
    return np.stack([-T[..., 1], T[..., 2], -T[..., 0]], -1)
PIVOT = np.array([0.255, 0.337, 0])          # the intake swings on its top roller's shaft
TURRET = np.array([-0.102, 0, -0.076])        # the turret's axis (their ROBOT_TO_TURRET: -4in, 3in left)
FLY = np.array([0.010, 0.466, -0.076])        # the flywheel's axle (the hood turns on it)
INTAKE_FIXED = re.compile(r'Shoe|Baby brontosaurus|Belt Plate|Hold Plate')
TURRET_PART = re.compile(r'Shooter Mount Plate|Slug|Hood Motor Shooter Plate|Non Hood Motor Shooter Plate|Front Cable Chain Guide|Back Cable Chain Guide')
HOOD_PART = re.compile(r'10DP Hood Plate|Reverser 10DP Hood Plate|10 DP 3DP Rack|Belt Direction Reverser Plate')
body, intake, turret, hood = [], [], [], []
for p, T in parts:
    name = ' / '.join(p)
    leaf = p[-1]
    if SKIP.search(leaf): continue
    T = robot(T)
    if 'Intake Assembly' in name: (body if INTAKE_FIXED.search(name) else intake).append(T)
    elif HOOD_PART.search(leaf): hood.append(T)
    elif TURRET_PART.search(leaf) or ('HOOD ASSEMBLY' in name and not HOOD_PART.search(leaf)): turret.append(T)
    elif 'Drivebase' in name and T[..., 1].max() < 0.06: continue  # (the belly pan's underside)
    else: body.append(T)
body, intake, turret, hood = (np.vstack(x) for x in (body, intake, turret, hood))

def cyl(c, a, r, h): return (np.array(c, float), np.array(a, float) / np.linalg.norm(a), r, h)
def cut(T, cyls):
    c = T.mean(1)
    keep = np.ones(len(T), bool)
    for (p, a, r, h) in cyls:
        d = c - p; s = d @ a
        rad = np.linalg.norm(d - np.outer(s, a), axis=1)
        keep &= ~((np.abs(s) <= h) & (rad <= r))
    return T[keep]
Z, Y = [0, 0, 1], [0, 1, 0]
# the wheels js/mech/r341.js turns (robot frame, as exported)
BODY_CUT = [cyl([0.255, 0.337, 0], Z, 0.032, 0.32),            # the intake's top roller (mecanums, compliants)
            cyl([0.093, 0.337, 0], Z, 0.032, 0.30), cyl([0.168, 0.337, 0], Z, 0.032, 0.30),  # the table's top rollers
            cyl([-0.187, 0.328, -0.077], Z, 0.024, 0.05), cyl([-0.016, 0.328, -0.077], Z, 0.024, 0.05),  # the uptake's
            cyl([-0.044, 0.248, 0.075], Y, 0.078, 0.022), cyl([0.108, 0.248, 0.219], Y, 0.078, 0.022), cyl([0.121, 0.248, -0.229], Y, 0.078, 0.022)]
BODY = cut(body, BODY_CUT)
# the arm's rollers (exported folded up): the two 1.25in silicone rollers and the three 2in plastic ones
INTAKE = cut(intake, [cyl([0.291, 0.680, 0], Z, 0.018, 0.30), cyl([0.119, 0.680, 0], Z, 0.018, 0.30),
                      cyl([0.083, 0.574, 0], Z, 0.027, 0.29), cyl([0.121, 0.468, 0], Z, 0.027, 0.29), cyl([0.192, 0.397, 0], Z, 0.027, 0.29),
                      cyl(PIVOT, Z, 0.032, 0.32)])
TURRET_T = cut(turret, [cyl(FLY, Z, 0.053, 0.06), cyl([-0.028, 0.410, -0.076], Z, 0.015, 0.06)])
HOOD_T = cut(hood, [cyl([-0.192, 0.464, -0.076], Z, 0.028, 0.03), cyl([-0.186, 0.518, -0.076], Z, 0.028, 0.03)])

# Vertices clustered on a 1 cm grid, each cluster's vertex where the CAD's vertices in it are on
# average (not the grid point: snapped to the grid, the serializer's floor, 0.1747 m up, came out
# at 0.170, 5 mm low, and FUEL on it lost a third of their squeeze under the top rollers: the
# last two left in the zipper sat stalled there at the end of most of tools/mechlab341.mjs's path
# runs; here 0.173, the plate's top and bottom averaged). (The turret's and the hood's stay on the
# grid points: the shooter's exit speed and angles were measured against those.)
def simplify(T, mean=True):
    P = T.reshape(-1, 3)
    V = np.round(P / GRID).astype(np.int64)
    uniq, inv = np.unique(V, axis=0, return_inverse=True)
    inv = inv.ravel()
    at = np.zeros((len(uniq), 3)); n = np.bincount(inv, minlength=len(uniq))
    for j in range(3): at[:, j] = np.bincount(inv, weights=P[:, j], minlength=len(uniq))
    at /= np.maximum(n, 1)[:, None]
    I = inv.reshape(-1, 3)
    I = I[(I[:, 0] != I[:, 1]) & (I[:, 1] != I[:, 2]) & (I[:, 0] != I[:, 2])]
    I = np.unique(np.sort(I, axis=1), axis=0)
    used, I2 = np.unique(I, return_inverse=True)
    return np.round((at[used] if mean else uniq[used] * GRID) * 1000).astype(int), I2.reshape(-1, 3)

# Only the body's surfaces a FUEL can get to: the space a FUEL's center fits in (allowing 1.5 cm of
# squash), flooded from points along the ball path, and the surfaces within a FUEL's reach of it
VOX, R = 0.01, 0.075
LO, HI = np.array([-0.45, 0.0, -0.45]), np.array([0.75, 0.80, 0.45])
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
# seeds: on the serializer's floor (0.248 up), in the uptake, in front of the bumper and above the table
seeds = np.array([[0.2, 0.25, 0.0], [0.2, 0.25, 0.22], [0.2, 0.25, -0.22], [0.0, 0.25, 0.1], [-0.1, 0.30, -0.077], [-0.1, 0.40, -0.077],
                  [0.5, 0.10, 0.0], [0.45, 0.25, 0.0], [0.3, 0.25, 0.0]])
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
meshes = {'body': simplify(BODY[kb]), 'intake': simplify(INTAKE - PIVOT), 'turret': simplify(TURRET_T - TURRET, False), 'hood': simplify(HOOD_T - FLY, False)}
out = {k: {'v': v.flatten().tolist(), 'i': i.flatten().tolist()} for k, (v, i) in meshes.items()}
with open(f'{ROOT}/js/mech/path341.js', 'w') as f:
    f.write('// 341\'s ball path\'s surfaces from their CAD (generated by tools/path-mesh-341.py; see js/mech/r341.js)\n')
    f.write('export default ' + json.dumps(out, separators=(',', ':')) + ';\n')
for k, (v, i) in meshes.items(): print(k, len(v), 'vertices', len(i), 'triangles')
print(os.path.getsize(f'{ROOT}/js/mech/path341.js') // 1024, 'KB')
