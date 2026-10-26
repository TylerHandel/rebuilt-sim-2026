# 8793's ball path as collision geometry, from their CAD as the game loads it (cad/robots/8793-*.glb):
# every surface FUEL could touch on its way from the intake to the shooter, simplified (vertices
# snapped to a 1 cm grid, a FUEL being 150 mm across) and with the wheels js/mech/r8793.js turns cut out
# (they're mechanism wheels, not walls). Writes cad/robots/8793-path.json: {body: {v, i}, arm: ...,
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
# the vertical-axis omni wheels at the neck (3in at +z, 5in at -z, x 0.065), with their shafts,
# pulleys and collars
wheels += [cyl([0.065, 0.22, 0.119], [0, 1, 0], 1.5 * IN + 0.012, 0.09), cyl([0.065, 0.22, -0.122], [0, 1, 0], 2.5 * IN + 0.012, 0.09)]
roi = lambda c: (c[:, 0] > -0.36) & (c[:, 1] > 0.03) & (c[:, 1] < 0.42) & (np.abs(c[:, 2]) < 0.36)
bv, bi = simplify(cut(np.vstack([body, mount]), wheels), roi)
# the arm (on its pivot, deployed): its rollers cut out
armRollers = [[0.0835, -0.0059], [0.1834, -0.0152], [0.2593, -0.0803], [0.2764, -0.1823]]
av, ai = simplify(cut(arm, [cyl([x, y, 0], [0, 0, 1], 1 * IN + 0.015, 0.34) for x, y in armRollers]), lambda c: np.ones(len(c), bool))
# the turret (on its axis): its shooter wheels cut out (the 2in wheels swing with the rack: cut
# the whole arc they sweep)
sw = [cyl([0.127, 0.411, 0], [0, 0, 1], 2 * IN + 0.015, 0.06)]
for e in range(50, 75, 2):
    a = np.radians(e); sw.append(cyl([0.127 - 0.203 * np.sin(a), 0.411 + 0.203 * np.cos(a), 0], [0, 0, 1], 1 * IN + 0.015, 0.06))
tv, ti = simplify(cut(turret, sw), lambda c: c[:, 1] < 0.6)
out = {k: {'v': v.flatten().tolist(), 'i': i.flatten().tolist()} for k, (v, i) in {'body': (bv, bi), 'arm': (av, ai), 'turret': (tv, ti)}.items()}
json.dump(out, open(f'{ROOT}/cad/robots/8793-path.json', 'w'), separators=(',', ':'))
for k, (v, i) in {'body': (bv, bi), 'arm': (av, ai), 'turret': (tv, ti)}.items(): print(k, len(v), 'vertices', len(i), 'triangles')
print(os.path.getsize(f'{ROOT}/cad/robots/8793-path.json') // 1024, 'KB')
