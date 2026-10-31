# 7769's ball path as collision geometry, from their CAD (Onshape "Full Robot", CHUNK): every surface
# FUEL could touch from the intake to the shooter, simplified (vertices snapped to a grid: 1 cm for the
# frame and walls, 4 mm where FUEL pass close to the wheels: the ramp up the back, its stationary
# rollers, the fixed hood and the adjustable hood), with the rollers and wheels js/mech/r7769.js
# turns cut out and the belt floor left out (r7769.js has its belts' runs as moving surfaces).
# Writes js/mech/path7769.js: {body, bodyFine, fixedHood, slide, kick, hood: {v, i}}, vertices in mm (int), in
# the robot's frame (x forward, y up, z to its right): the body as it is (bodyFine: the ramp and its
# stationary rollers; fixedHood: the fixed hood, its middle plate and the shaft's support plate; body: the frame and belly pan, the
# hopper's stationary polycarbonate sides and the rack gearboxes, the floor's plates and its center
# fin, the shooter's plates, ramps, stationary rollers, fixed hood and polycarbonate shields, the
# climber's tube); the intake that slides out on its racks (the hopper's front and moving sides, the
# polycarbonate arms and racks, the cross tube, the roller's motors; the 3in roller cut out), as
# exported (all the way out); the kick bar (its 1/2in hex) on its arms, about their pin (0.4873, 0.1915) as exported
# (hanging); the adjustable hood ("Hood": its plates, tube and the two curved Adjust Hood surfaces)
# about the shooter wheels' axis (-0.2286, 0.4953), as exported (down). Only what a FUEL can reach is
# kept (with the static hex bars FUEL ride on: the 4.5in pairs behind the vectored wheels at each side,
# the ramp's standoffs). Left out: hardware, belts, pulleys, gears, chain, motors, the turning shafts,
# electronics, the bumpers (a
# solid Block in r7769.js) and the swerve modules.
# The source is the assembly's glTF export, whose parts sit where the assembly has them (the STEP
# translation puts a few elsewhere):
#   DID=692fc43a78732f76f6a7fc01 WID=68c7f5631809df4361fcb808 tools/onshape-export.sh r7769.glb ae1887b4c2531db24673e5e9
#   python3 -I tools/path-mesh-7769.py r7769.glb
# (The CAD's frame: x to the robot's left, y back, z up, meters; robot (x, y, z) = (-y, z, -x).)
import json, struct, sys, os, re
import numpy as np
from scipy import ndimage

ROOT = os.path.join(os.path.dirname(os.path.abspath(__file__)), '..')
SRC = sys.argv[1] if len(sys.argv) > 1 else 'r7769.glb'

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

# hardware, electronics, drive parts, belts, pulleys, shafts and the wheels: nothing a FUEL meets as a
# fixed surface (the wheels and belts are r7769.js's own)
SKIP = re.compile(r'screw|bolt|\bnut\b|washer|rivet|spacer|bearing|bushing|pulley|gear|sprocket|chain|belt|kraken|motor|maxplanetary|'
                  r'thunderhex \((23|25\.5) in\)|rounded hex \(rev\) \((24|18\.75|28\.5) in\)|^0\.375in hex|wheel|stealth|compliant|intake roller|cat tongue|driven hub|limelight|radio|battery|'
                  r'roborio|canivore|\bPDH\b|pigeon|breaker|sb50|anderson|candle|e18|sensor|\bRSL\b|tensioner|turnbuckle|spool|bumper|mk5|'
                  r'grommet|plug|cable clamp|collar|origin cube|^limits|heat sink|quick-release|knob', re.I)
parts = load(SRC)
def robot(T):  # CAD (x left, y back, z up) -> robot (x forward, y up, z right)
    return np.stack([-T[..., 1], T[..., 2], -T[..., 0]], -1)
FINE = re.compile(r'^Ramp|^Stationary Roller|^Inner Shooter Plate', re.I)
FIXED = re.compile(r'^Fixed Hood|^Mid Hood Plate|^Shaft Support Plate', re.I)
body, fine, fixed, slide, kick, hood = [], [], [], [], [], []
HARDWARE = re.compile(r'screw|bolt|\bnut\b|washer|spacer', re.I)
for p, T in parts:
    names = ' / '.join(p)
    # (the kick bar's 1/2in hex is the bar itself: kept)
    if 'Kick Bar <1>' in names and not HARDWARE.search(p[-1]): kick.append(robot(T)); continue
    if any(SKIP.search(x) for x in p[1:]) or any(x.startswith('Limits') for x in p): continue
    T = robot(T)
    if 'Moving <1>' in names: slide.append(T)
    elif 'Hood <1>' in p: hood.append(T)
    elif FIXED.search(p[-1]): fixed.append(T)
    elif FINE.search(p[-1]): fine.append(T)
    else: body.append(T)
body, fine, fixed, slide, kick, hood = (np.vstack(x) for x in (body, fine, fixed, slide, kick, hood))

# the rollers and wheels the mechanism turns (robot frame), cut out: [center, axis, radius, half length]
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
# (r7769.js has the same: the intake's 3in roller, the feeder's shaft of compliant, ION and vectored
# wheels, the shooter's two stacks of 4in Stealth wheels, and the floor's belt shafts)
INTAKE, FEED, SHOOT = (0.5779, 0.1746), (-0.1061, 0.3175), (-0.2286, 0.4953)
FLOOR = [(0.2045, 0.2030), (-0.0075, 0.1560)]
wheels = [cyl([*FEED, 0], Z, 0.031, 0.31), cyl([*SHOOT, 0], Z, 0.054, 0.2)] + [cyl([*p, 0], Z, 0.02, 0.3) for p in FLOOR]
BODY, FINE_T, FIXED_T = cut(body, wheels), cut(fine, wheels), cut(fixed, wheels)
SLIDE = cut(slide, [cyl([*INTAKE, 0], Z, 0.042, 0.31)])

def simplify(T, grid):
    V = np.round(T.reshape(-1, 3) / grid).astype(np.int64)
    uniq, inv = np.unique(V, axis=0, return_inverse=True)
    I = inv.reshape(-1, 3)
    I = I[(I[:, 0] != I[:, 1]) & (I[:, 1] != I[:, 2]) & (I[:, 0] != I[:, 2])]
    I = np.unique(np.sort(I, axis=1), axis=0)
    used, I2 = np.unique(I, return_inverse=True)
    return np.round(uniq[used] * grid * 1000).astype(int), I2.reshape(-1, 3)

# Only the surfaces a FUEL can get to: the space a FUEL's center fits in, allowing 1.5 cm of squash,
# flooded from points along the ball path (everything as exported: the intake out, the kick bar
# hanging, the hood down), and the surfaces within a FUEL's reach of it. The intake out front is open
# to the carpet; the space round the robot is walled off past the hopper's sides and front, behind the
# shooter and over the hopper's top (where the net is).
VOX, R = 0.01, 0.075
LO, HI = np.array([-0.36, 0.0, -0.40]), np.array([0.70, 0.62, 0.40])
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
groups = {'body': BODY, 'bodyFine': FINE_T, 'fixedHood': FIXED_T, 'slide': SLIDE, 'kick': kick, 'hood': hood}
pts = {k: samples(T) for k, T in groups.items()}
solid = np.zeros(shape, bool)
for P, _ in pts.values():
    ijk, ok = voxels(P)
    solid[tuple(ijk[ok].T)] = True
# (the wheels and belts too: a FUEL can't be inside one; and the bumper's foam, 0.343 to 0.407 m out)
g = LO[None, :] + (np.indices(shape).reshape(3, -1).T + 0.5) * VOX
for (x, y), r, hw in [(INTAKE, 0.039, 0.3), (FEED, 0.0286, 0.3), (SHOOT, 0.0508, 0.17)]:
    solid |= ((np.hypot(g[:, 0] - x, g[:, 1] - y) < r) & (np.abs(g[:, 2]) < hw)).reshape(shape)
(fx, fy), (bx, by) = FLOOR
u = np.clip((g[:, 0] - bx) / (fx - bx), 0, 1)
solid |= ((g[:, 0] > bx - 0.016) & (g[:, 0] < fx + 0.016) & (g[:, 1] < by + u * (fy - by) + 0.016) & (np.abs(g[:, 2]) < 0.275)).reshape(shape)
solid |= ((g[:, 0] > 0.343) & (g[:, 0] < 0.407) & (g[:, 1] > 0.038) & (g[:, 1] < 0.152)).reshape(shape)
solid[:, 0, :] = True
# (the outside: beside the hopper, out past its front, behind the shooter, over the net)
out = (np.abs(g[:, 2]) > 0.352) | (g[:, 0] > 0.63) | (g[:, 0] < -0.33) | (g[:, 1] > 0.60) | ((g[:, 1] > 0.53) & (g[:, 0] > -0.02))
solid |= out.reshape(shape)
free = ndimage.distance_transform_edt(~solid) * VOX >= R - 0.015
lab, _ = ndimage.label(free)
seeds = np.array([[0.1, 0.3, 0], [0.2, 0.4, 0.2], [0.2, 0.4, -0.2], [0.45, 0.35, 0], [0.55, 0.3, 0.2], [0.6, 0.08, 0], [0.6, 0.08, 0.2], [0.6, 0.08, -0.2],
                  [0.1, 0.25, 0.25], [0.1, 0.25, -0.25], [-0.15, 0.25, 0.1], [-0.15, 0.25, -0.1], [-0.2, 0.35, 0.1], [-0.2, 0.35, -0.1],
                  [-0.12, 0.45, 0.1], [-0.12, 0.45, -0.1], [-0.05, 0.25, 0.27], [-0.05, 0.25, -0.27]])
sk, ok = voxels(seeds)
ids = set(lab[tuple(sk[ok].T)].tolist()) - {0}
# (and every free space up the ramp and round the wheels, where FUEL are pinched: in front of the
# ramp, under the hoods)
up = (g[:, 0] > -0.29) & (g[:, 0] < -0.04) & (g[:, 1] > 0.15) & (g[:, 1] < 0.56) & (np.abs(g[:, 2]) < 0.21)
ids |= set(np.unique(lab.reshape(-1)[up]).tolist()) - {0}
print('seeded regions', len(ids), 'of', lab.max(), [int(lab[tuple(s)]) for s in sk[ok]])
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
PIN = np.array([0.4873, 0.1915, 0]); AXLE = np.array([*SHOOT, 0])
meshes = {'body': simplify(BODY[keep['body']], 0.01), 'bodyFine': simplify(FINE_T[keep['bodyFine']], 0.004), 'fixedHood': simplify(FIXED_T[keep['fixedHood']], 0.004), 'slide': simplify(SLIDE[keep['slide']], 0.01),
          'kick': simplify(kick[keep['kick']] - PIN, 0.004), 'hood': simplify(hood[keep['hood']] - AXLE, 0.004)}
res = {k: {'v': v.flatten().tolist(), 'i': i.flatten().tolist()} for k, (v, i) in meshes.items()}
with open(f'{ROOT}/js/mech/path7769.js', 'w') as f:
    f.write('// 7769\'s ball path\'s surfaces from their CAD (generated by tools/path-mesh-7769.py; see js/mech/r7769.js)\n')
    f.write('export default ' + json.dumps(res, separators=(',', ':')) + ';\n')
for k, (v, i) in meshes.items(): print(k, len(v), 'vertices', len(i), 'triangles')
print(os.path.getsize(f'{ROOT}/js/mech/path7769.js') // 1024, 'KB')
