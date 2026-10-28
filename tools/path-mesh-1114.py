# 1114's ball path as collision geometry, from their CAD (Onshape "S26-A000", Simbot Tim): every
# surface FUEL could touch from the intake to the flywheel, simplified (vertices snapped to a 1 cm
# grid) and with the rollers js/mech/r1114.js turns cut out. Writes js/mech/path1114.js:
# {body, slide, kicker, hood: {v, i}}, vertices in mm (int), in the robot's frame (x forward, y up,
# z to its right): the body as it is (the drivetrain's frame and bumper wood, the roller floor and
# its side plates, the ball elevator's tower, the shooter's frame, the box's fixed rack gearboxes;
# its rollers cut out, the passive side rollers at the tower's front among them);
# the hopper box that slides out on its racks (its clear polycarbonate walls, side plates, front
# tubes and guard rods; the intake roller cut out), as exported (all the way out, 0.298 m); the
# kicker bar on its arms, about their pivot pin (0.4513, 0.196) as exported (hanging); the hood on
# the flywheel's shaft, about its axis (-0.0176, 0.5715), as exported. Only what a FUEL can reach
# is kept. Left out: the climber (they took it off for weight), "Part 65" (a solid filling the
# intake's sweep), the blanket over the box (S26-IN-P330: js/mech/r1114.js has it as a soft
# sheet), belts, pulleys, gears, motors and hardware.
# The assembly's glTF export only carries the intake, so this reads a STEP translation of it
# (POST .../assemblies/d/7959d159451177786d52dd9a/w/535bc31b66b0cc442967f1d0/e/a263c1a625ed8cf2484ae82f/translations
# {"formatName":"STEP"}, then .../documents/d/7959d159451177786d52dd9a/externaldata/<id>), with the box
# slid out as cad/robots/1114-*.glb have it, tessellated by tools/step2glb.py (1.2 mm, 0.45 rad):
#   python tools/step2glb.py r1114.step r1114-step.glb --lin 0.0012 --ang 0.45 \
#     --skip "screw|bolt|\bnut\b|nutstrip|locknut|washer|rivet|spacer|bearing|bushing|collar|shim|standoff|BHCS|SHCS|FHCS|zip|grommet"
# then: python3 -I tools/path-mesh-1114.py r1114-step.glb
# (The STEP's frame: x to the robot's left, y back, z up; robot (x, y, z) = (-y, z, -x).)
import json, struct, sys, os, re
import numpy as np
from scipy import ndimage

ROOT = os.path.join(os.path.dirname(os.path.abspath(__file__)), '..')
GRID = 0.01
SRC = sys.argv[1] if len(sys.argv) > 1 else 'r1114-step.glb'

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

# hardware, electronics, drive parts, belts and pulleys: nothing a FUEL meets in the ball path
SKIP = re.compile(r'screw|bolt|\bnut\b|locknut|washer|rivet|spacer|bearing|bushing|snapring|PHCS|pulley|gear|\bHTD\b|^[0-9]+T$|kraken|motor|'
                  r'thunderhex|hex adapter|tube plug|limelight|radio|battery|roborio|\bPDP\b|PDH|cancoder|magnet|swerve|colson|wheel module|'
                  r'upright|flipped|X2c? top plate|WCP-2091|WCP-2092|WCP-0323|id[0-9]+$|^Minimal$|mounting bracket|^Part 65$|S26-IN-P330', re.I)
parts = load(SRC)
def robot(T):  # CAD (x left, y back, z up) -> robot (x forward, y up, z right)
    return np.stack([-T[..., 1], T[..., 2], -T[..., 0]], -1)
# what of the box assembly stays put (its rack gearboxes: cad/robots/1114.json's "mount") and its
# kicker (on its arms, about their pin)
MOUNT = re.compile(r'^S26-IN-P31[89]$|^S26-IN-P32[01]$|^S26-IN-P011$|^Part 47$|^Part [12](-Mirrored)?$|^S26-IN-P018$')
KICKER = re.compile(r'^S26-IN-P311$|^S26-IN-P303$|^Part 4[2-6]$|^Part 52$|^S26-IN-P326$')
HOOD = re.compile(r'^S26-SH-P02[2-478]$|^S26-SH-P03[2-7]$|^S26-SH-P045$|^Part 102$')
body, slide, kicker, hood = [], [], [], []
for p, T in parts:
    name = ' / '.join(p)
    leaf = p[-1]
    if any(SKIP.search(x) for x in p[1:]) or 'S26-CL-A000' in name: continue
    T = robot(T)
    if 'S26-IN-A200' in name:
        if MOUNT.search(leaf): body.append(T)
        elif KICKER.search(leaf): kicker.append(T)
        else: slide.append(T)
    elif 'S26-SH-A000' in name and HOOD.search(leaf) and T[..., 1].mean() > 0.55: hood.append(T)
    else: body.append(T)
body, slide, kicker, hood = (np.vstack(x) for x in (body, slide, kicker, hood))

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
Z, X = [0, 0, 1], [1, 0, 0]
# (r1114.js has the same: the floor's nine 1.25in rollers, front to back; the elevator's back and
# front columns and the roller wall in front of it; the squish wheels; the side rollers; the
# flywheel; the hood's three 1.5in rollers; the intake roller)
FLOOR = [(0.2857, 0.1668), (0.2276, 0.1457), (0.1696, 0.1246), (0.1116, 0.1034), (0.0536, 0.0823), (-0.0326, 0.0719), (-0.1144, 0.1007), (-0.1751, 0.1626), (-0.2022, 0.2451)]
BACK = [(-0.2096, y) for y in (0.3224, 0.3816, 0.4408, 0.5001)]
FRONT = [(-0.0343, y) for y in (0.3224, 0.3816, 0.4408, 0.5001)]
WALL = [(0.0134, 0.3266), (0.0195, 0.4134), (0.0255, 0.5002)]
SQUISH = (-0.0118, 0.2619)
FLY = (-0.0176, 0.5715)
HOODR = [(-0.2024, 0.6137), (-0.1841, 0.6621), (-0.1534, 0.7037)]
INTAKE = (0.5457, 0.1663)
rollers = [cyl([x, y, 0], Z, 0.021, 0.23) for x, y in FLOOR + BACK + FRONT + WALL]
# (and the passive side rollers: two 5/8in rollers standing up either side at the tower's front,
# 0.352 to 0.505 m up, "Part 2" between their "Part 1" bosses)
PASSIVE = [(0.0483, 0.2883), (0.08, 0.3382)]
BODY = cut(body, rollers + [cyl([*SQUISH, 0], Z, 0.042, 0.17), cyl([*FLY, 0], Z, 0.044, 0.215)]
           + [cyl([0.125, 0.2246, s * 0.2725], X, 0.034, 0.1) for s in (-1, 1)]
           + [cyl([x, 0.4286, s * z], [0, 1, 0], 0.011, 0.077) for x, z in PASSIVE for s in (-1, 1)])
HOOD_T = cut(hood, [cyl([x, y, 0], Z, 0.024, 0.215) for x, y in HOODR])
SLIDE = cut(slide, [cyl([*INTAKE, 0], Z, 0.03, 0.33)])

def simplify(T):
    V = np.round(T.reshape(-1, 3) / GRID).astype(np.int64)
    uniq, inv = np.unique(V, axis=0, return_inverse=True)
    I = inv.reshape(-1, 3)
    I = I[(I[:, 0] != I[:, 1]) & (I[:, 1] != I[:, 2]) & (I[:, 0] != I[:, 2])]
    I = np.unique(np.sort(I, axis=1), axis=0)
    used, I2 = np.unique(I, return_inverse=True)
    return np.round(uniq[used] * GRID * 1000).astype(int), I2.reshape(-1, 3)

# Only the surfaces a FUEL can get to (as 581's: tools/path-mesh-581.py): the space a FUEL's center
# fits in, allowing 1.5 cm of squash, flooded from points along the ball path (everything as
# exported: the box out, the kicker hanging, the hood as the CAD has it), and the surfaces within
# a FUEL's reach of it. The box out front is open to the carpet; the space round the robot is
# walled off at the box's sides and front and over its top.
VOX, R = 0.01, 0.075
LO, HI = np.array([-0.36, 0.0, -0.42]), np.array([0.72, 0.80, 0.42])
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
groups = {'body': BODY, 'slide': SLIDE, 'kicker': kicker, 'hood': HOOD_T}
pts = {k: samples(T) for k, T in groups.items()}
solid = np.zeros(shape, bool)
for P, _ in pts.values():
    ijk, ok = voxels(P)
    solid[tuple(ijk[ok].T)] = True
# (the rollers too: a FUEL can't be inside one; and the bumper's foam, 0.329 to 0.393 m out)
g = LO[None, :] + (np.indices(shape).reshape(3, -1).T + 0.5) * VOX
for (x, y), r, hw in [*((p, 0.0167, 0.2) for p in FLOOR + BACK + FRONT + WALL), (SQUISH, 0.0381, 0.165), (FLY, 0.0402, 0.2), *((p, 0.0198, 0.2) for p in HOODR), (INTAKE, 0.026, 0.32)]:
    solid |= ((np.hypot(g[:, 0] - x, g[:, 1] - y) < r) & (np.abs(g[:, 2]) < hw)).reshape(shape)
for s in (-1, 1): solid |= ((np.hypot(g[:, 1] - 0.2246, g[:, 2] - s * 0.2725) < 0.0254) & (g[:, 0] > 0.03) & (g[:, 0] < 0.22)).reshape(shape)
for (x, z) in PASSIVE:
    for s in (-1, 1): solid |= ((np.hypot(g[:, 0] - x, g[:, 2] - s * z) < 0.008) & (g[:, 1] > 0.35) & (g[:, 1] < 0.51)).reshape(shape)
solid |= ((g[:, 0] > 0.329) & (g[:, 0] < 0.393) & (g[:, 1] > 0.047) & (g[:, 1] < 0.172)).reshape(shape)
solid[:, 0, :] = True
# (the outside: beside the box, behind the robot, out past the box's front, over the blanket)
out = (np.abs(g[:, 2]) > 0.38) | (g[:, 0] > 0.70) | (g[:, 0] < -0.35) | (g[:, 1] > 0.78) | ((g[:, 1] > 0.705) & (g[:, 0] > 0.0))
solid |= out.reshape(shape)
free = ndimage.distance_transform_edt(~solid) * VOX >= R - 0.015
lab, _ = ndimage.label(free)
seeds = np.array([[0.1, 0.3, 0], [0.25, 0.4, 0.2], [0.25, 0.4, -0.2], [0.45, 0.4, 0], [0.62, 0.08, 0], [0.62, 0.08, 0.2], [0.62, 0.08, -0.2],
                  [-0.05, 0.17, 0], [-0.12, 0.3, 0.1], [-0.12, 0.42, -0.1], [-0.12, 0.6, 0], [0.1, 0.6, 0.25], [0.2, 0.17, 0]])
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
PIN = np.array([0.4513, 0.196, 0]); AXLE = np.array([*FLY, 0])
meshes = {'body': simplify(BODY[keep['body']]), 'slide': simplify(SLIDE[keep['slide']]), 'kicker': simplify(kicker[keep['kicker']] - PIN), 'hood': simplify(HOOD_T[keep['hood']] - AXLE)}
res = {k: {'v': v.flatten().tolist(), 'i': i.flatten().tolist()} for k, (v, i) in meshes.items()}
with open(f'{ROOT}/js/mech/path1114.js', 'w') as f:
    f.write('// 1114\'s ball path\'s surfaces from their CAD (generated by tools/path-mesh-1114.py; see js/mech/r1114.js)\n')
    f.write('export default ' + json.dumps(res, separators=(',', ':')) + ';\n')
for k, (v, i) in meshes.items(): print(k, len(v), 'vertices', len(i), 'triangles')
print(os.path.getsize(f'{ROOT}/js/mech/path1114.js') // 1024, 'KB')
