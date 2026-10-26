# 8793's ball path as collision geometry, from their CAD as the game loads it (cad/robots/8793-*.glb):
# every surface FUEL could touch on its way from the intake to the shooter (and only those: what a
# FUEL can reach, found by flooding the space it fits through), simplified (vertices
# snapped to a 1 cm grid, a FUEL being 150 mm across) and with the wheels js/mech/r8793.js turns cut out
# (they're mechanism wheels, not walls). Writes js/mech/path8793.js: {body: {v, i}, arm: ...,
# turret: ...}, vertices in mm (int), each in its glb's own frame (body: the robot's; arm: on the
# pivot, deployed; turret: on the turntable's axis).
# Run: python3 -I tools/path-mesh.py
import json, struct, sys, os
import numpy as np

ROOT = os.path.join(os.path.dirname(os.path.abspath(__file__)), '..')
GRID = 0.01

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
    T = []
    def walk(i, M):
        nd = j['nodes'][i]; W = M @ local(nd)
        if 'mesh' in nd:
            for pr in j['meshes'][nd['mesh']]['primitives']:
                p = acc(pr['attributes']['POSITION']); p = (W[:3, :3] @ p.T).T + W[:3, 3]
                idx = acc(pr['indices']).astype(int).reshape(-1, 3) if 'indices' in pr else np.arange(len(p)).reshape(-1, 3)
                T.append(p[idx])
        for c in nd.get('children', []): walk(c, W)
    for r in j['scenes'][j.get('scene', 0)]['nodes']: walk(r, np.eye(4))
    return np.vstack(T)

# cylinders to cut out: (center, axis, radius, half length)
def cyl(c, a, r, h): return (np.array(c, float), np.array(a, float) / np.linalg.norm(a), r, h)
def cut(T, cyls):
    c = T.mean(1)
    keep = np.ones(len(T), bool)
    for (p, a, r, h) in cyls:
        d = c - p; s = d @ a
        rad = np.linalg.norm(d - np.outer(s, a), axis=1)
        keep &= ~((np.abs(s) <= h) & (rad <= r))
    return T[keep]
def simplify(T, roi):
    c = T.mean(1)
    T = T[roi(c)]
    V = np.round(T.reshape(-1, 3) / GRID).astype(np.int64)
    uniq, inv = np.unique(V, axis=0, return_inverse=True)
    I = inv.reshape(-1, 3)
    I = I[(I[:, 0] != I[:, 1]) & (I[:, 1] != I[:, 2]) & (I[:, 0] != I[:, 2])]
    I = np.unique(np.sort(I, axis=1), axis=0)
    used, I2 = np.unique(I, return_inverse=True)
    Vmm = np.round(uniq[used] * GRID * 1000).astype(int)
    return Vmm, I2.reshape(-1, 3)

IN = 0.0254
body = load(f'{ROOT}/cad/robots/8793-body.glb')
mount = load(f'{ROOT}/cad/robots/8793-mount.glb')
arm = load(f'{ROOT}/cad/robots/8793-intake.glb')
turret = load(f'{ROOT}/cad/robots/8793-turret.glb')
# the wheels the mechanism turns (robot frame): conveyor shafts, vertical rollers, the indexer's
# shaft and the never-built rear one
wheels = [cyl([x, y, 0], [0, 0, 1], 0.045, 0.34) for x, y in [(0.122, 0.315), (0.217, 0.34), (0.292, 0.336)]]
wheels += [cyl([0.217, 0.241, s * 0.24], [0, 1, 0], 2 * IN + 0.015, 0.05) for s in (1, -1)]
wheels += [cyl([-0.039, 0.226, 0], [0, 0, 1], 1 * IN + 0.015, 0.06), cyl([-0.215, 0.226, 0], [0, 0, 1], 1 * IN + 0.015, 0.06)]
# the indexer's hex shaft, turning with its wheels (z -0.19 to 0.09: a mechanism wheel too), and
# all of the never-built rear shaft
wheels += [cyl([-0.039, 0.226, -0.05], [0, 0, 1], 0.012, 0.14), cyl([-0.215, 0.226, 0], [0, 0, 1], 0.025, 0.175)]
# the vertical-axis omni wheels at the neck (3in at +z, 5in at -z, x 0.065), with their shafts,
# pulleys and collars
wheels += [cyl([0.065, 0.22, 0.119], [0, 1, 0], 1.5 * IN + 0.012, 0.09), cyl([0.065, 0.22, -0.122], [0, 1, 0], 2.5 * IN + 0.012, 0.09)]
BODY = cut(np.vstack([body, mount]), wheels)
# the arm (on its pivot, deployed): its rollers cut out
armRollers = [[0.0835, -0.0059], [0.1834, -0.0152], [0.2593, -0.0803], [0.2764, -0.1823]]
ARM = cut(arm, [cyl([x, y, 0], [0, 0, 1], 1 * IN + 0.015, 0.34) for x, y in armRollers])
# the turret (on its axis): its shooter wheels cut out (the 2in wheels swing with the rack: cut
# the whole arc they sweep)
sw = [cyl([0.127, 0.411, 0], [0, 0, 1], 2 * IN + 0.015, 0.06)]
for e in range(50, 75, 2):
    a = np.radians(e); sw.append(cyl([0.127 - 0.203 * np.sin(a), 0.411 + 0.203 * np.cos(a), 0], [0, 0, 1], 1 * IN + 0.015, 0.06))
TURRET = cut(turret, sw)

# Only what a FUEL can get to: the space a FUEL's center fits in (allowing the foam 1.5 cm of
# squash), flooded from points along the ball path, and the surfaces within a FUEL's reach of it
# (with the turret at every quarter turn). The motors, battery and wiring under the floor go.
from scipy import ndimage
VOX, R = 0.01, 0.075
LO, HI = np.array([-0.38, 0.0, -0.40]), np.array([0.72, 0.72, 0.40])
shape = tuple(np.ceil((HI - LO) / VOX).astype(int))
def samples(T, step=0.008):
    # points over each triangle, about step apart
    a, b, c = T[:, 0], T[:, 1], T[:, 2]
    n = np.clip(np.ceil(np.maximum(np.linalg.norm(b - a, axis=1), np.linalg.norm(c - a, axis=1)) / step).astype(int), 1, 40)
    out, tri = [], []
    for k in range(1, 41):
        m = n == k
        if not m.any(): continue
        u, v = np.meshgrid(np.arange(k + 1), np.arange(k + 1))
        keep = (u + v) <= k
        u, v = u[keep] / k, v[keep] / k
        P = a[m][:, None, :] + (b[m] - a[m])[:, None, :] * u[None, :, None] + (c[m] - a[m])[:, None, :] * v[None, :, None]
        out.append(P.reshape(-1, 3)); tri.append(np.repeat(np.nonzero(m)[0], len(u)))
    return np.vstack(out), np.concatenate(tri)
def yawed(T, yaw, x0=-0.127):
    c, s = np.cos(yaw), np.sin(yaw)
    X, Z = T[..., 0], T[..., 2]
    return np.stack([x0 + X * c + Z * s, T[..., 1], -X * s + Z * c], -1)
def voxels(P):
    ijk = np.floor((P - LO) / VOX).astype(int)
    ok = np.all((ijk >= 0) & (ijk < shape), axis=1)
    return ijk, ok
# the bumper's front, closing the bottom of the way in (a slab across the front)
bx = np.array([[0.349, 0.032], [0.432, 0.032], [0.432, 0.16], [0.349, 0.16]])
BUMPER = []
for z0, z1 in [(-0.43, 0.43)]:
    q = [[x, y, z0] for x, y in bx] + [[x, y, z1] for x, y in bx]
    q = np.array(q)
    for f in [(0, 1, 2), (0, 2, 3), (4, 6, 5), (4, 7, 6), (0, 4, 5), (0, 5, 1), (1, 5, 6), (1, 6, 2), (2, 6, 7), (2, 7, 3)]:
        BUMPER.append(q[list(f)])
BUMPER = np.array(BUMPER)
ARMW = ARM + np.array([0.292, 0.336, 0])
pts_b, tri_b = samples(BODY)
pts_a, tri_a = samples(ARMW)
pts_t, tri_t = samples(TURRET)
pts_k, _ = samples(BUMPER)
seeds = np.array([[-0.06, 0.135, 0], [-0.14, 0.26, 0], [0.066, 0.21, 0], [0.21, 0.255, -0.085], [0.21, 0.255, 0.085],
                  [0.3, 0.25, 0], [0.45, 0.2, 0], [-0.1, 0.45, 0], [-0.127, 0.33, 0]])
reach = np.zeros(shape, bool)
for yaw in np.radians([0, 90, 180, 270]):
    solid = np.zeros(shape, bool)
    for P in (pts_b, pts_a, yawed(pts_t, yaw), pts_k):
        ijk, ok = voxels(P)
        solid[tuple(ijk[ok].T)] = True
    # (the floor of the world: the carpet; and the flood stays inside the robot's envelope, the
    # arm out front: FUEL outside it are the robot's colliders' business)
    solid[:, 0, :] = True
    g = LO[None, :] + (np.indices(shape).reshape(3, -1).T + 0.5) * VOX
    out = (g[:, 0] < -0.36) | (g[:, 0] > 0.68) | (np.abs(g[:, 2]) > 0.36) | (g[:, 1] > 0.68) | ((g[:, 0] > 0.36) & (np.abs(g[:, 2]) > 0.32))
    solid |= out.reshape(shape)
    free = ndimage.distance_transform_edt(~solid) * VOX >= R - 0.015
    lab, _ = ndimage.label(free)
    sk, ok = voxels(seeds)
    ids = set(lab[tuple(sk[ok].T)].tolist()) - {0}
    centers = np.isin(lab, list(ids))
    reach |= ndimage.distance_transform_edt(~centers) * VOX <= R + 0.02
def reached(P, tri, n):
    ijk, ok = voxels(P)
    hit = np.zeros(len(P), bool)
    hit[ok] = reach[tuple(ijk[ok].T)]
    keep = np.zeros(n, bool)
    np.logical_or.at(keep, tri, hit)
    return keep
kb = reached(pts_b, tri_b, len(BODY))
ka = reached(pts_a, tri_a, len(ARMW))
kt = np.zeros(len(TURRET), bool)
for yaw in np.radians([0, 90, 180, 270]):
    kt |= reached(yawed(pts_t, yaw), tri_t, len(TURRET))
print('reachable:', kb.sum(), 'of', len(BODY), 'body;', ka.sum(), 'of', len(ARM), 'arm;', kt.sum(), 'of', len(TURRET), 'turret')
everything = lambda c: np.ones(len(c), bool)
bv, bi = simplify(BODY[kb], everything)
av, ai = simplify(ARM[ka], everything)
tv, ti = simplify(TURRET[kt], everything)
out = {k: {'v': v.flatten().tolist(), 'i': i.flatten().tolist()} for k, (v, i) in {'body': (bv, bi), 'arm': (av, ai), 'turret': (tv, ti)}.items()}
with open(f'{ROOT}/js/mech/path8793.js', 'w') as f:
    f.write('// 8793\'s ball path\'s surfaces from their CAD (generated by tools/path-mesh.py; see js/mech/r8793.js)\n')
    f.write('export default ' + json.dumps(out, separators=(',', ':')) + ';\n')
for k, (v, i) in {'body': (bv, bi), 'arm': (av, ai), 'turret': (tv, ti)}.items(): print(k, len(v), 'vertices', len(i), 'triangles')
print(os.path.getsize(f'{ROOT}/js/mech/path8793.js') // 1024, 'KB')
