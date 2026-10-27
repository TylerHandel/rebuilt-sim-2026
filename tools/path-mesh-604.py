# 604's ball path as collision geometry, from their CAD (Onshape "Toploader Assembly",
# 0baaf96036e07596972c4d48): every surface FUEL could touch from the intake to the shooter,
# simplified (vertices snapped to a grid: 1 cm for the frame and walls, 5 mm where FUEL pass close
# to them, in the rotor, the turret and the hood) and with the rollers js/mech/r604.js turns cut out.
# Writes js/mech/path604.js: {body, intake, hopper, rotor, turret, hood: {v, i}}, vertices
# in mm (int), in the robot's frame (x forward, y up, z to its right):
#   body    what doesn't move: the chassis, the DPC's plywood bucket (floor and balustrade), the
#           intake's gearbox plates, the superstructure and the hopper's back walls, the motors
#           FUEL can fall onto in the front corners, the bumper's wood
#   (the bumpers' foam is solid for the flooding, not written: js/mech/r604.js draws it as two
#   foam noodles, bumper604)
#   intake  the arm on its pivot (the gearbox's arm shaft, 0.3048 m forward, 0.2064 m up), as
#           exported: their wrist angle 0 (deployed is their -5)
#   hopper  the sliding hopper (its coroplast walls), as exported: all the way out, 0.2921 m
#   rotor   the DPC rotor about its axis (0.0254 m behind the center), as exported: the hook,
#           the ramp, the base and top plates, the tube ("chef's hat") up to the turret, drawn as a
#           clean 32-sided bore (the CAD's 155 mm: 2.5 mm round a FUEL)
#   turret  the turret on the same axis, in its own frame (facing forward: as exported it fires to
#           the robot's right, so it's turned a quarter turn), without the hood
#   hood    the hood on the main roller's axle, in the turret's frame: as exported (their hood
#           angle 35, a 55 deg shot)
# Only what a FUEL can reach is kept (the space a FUEL's center fits in, flooded from points along
# the ball path, and the surfaces within a FUEL's reach of it; the rotor's with it turned to eight
# angles). The assembly's glTF export fails through the API, so this reads a STEP translation
# (POST .../assemblies/d/<did>/w/<wid>/e/<eid>/translations {"formatName":"STEP"}, then the
# external data; cad/README.md), tessellated by tools/step2glb.py:
#   python tools/step2glb.py r604.step r604-step.glb --lin 0.001 --ang 0.4 \
#     --skip "screw|bolt|\bnut\b|nutstrip|locknut|washer|rivet|bearing|bushing|collar|shim|BHCS|SHCS|FHCS|zip|grommet|rivnut|insert"
# then: python3 -I tools/path-mesh-604.py r604-step.glb
import json, struct, sys, os, re
import numpy as np
from scipy import ndimage

ROOT = os.path.join(os.path.dirname(os.path.abspath(__file__)), '..')
SRC = sys.argv[1] if len(sys.argv) > 1 else 'r604-step.glb'

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

# hardware, electronics, drive parts and what moves on its own (belts, chains), nothing a FUEL meets
# in the ball path. (The intake's motors stay: FUEL that fall into the front corners land on them.)
SKIP = re.compile(r'screw|bolt|\bnut\b|nutstrip|locknut|washer|rivet|(?<!hook )spacer|bearing|bushing|collar|chain|belt|sprocket|gear\b|spur gear|bevel|pulley|'
                  r'radio|battery|roborio|pdp|pigeon|canivore|breaker|grommet|shim|standoff|zip|wire|osmo|camera|rsl|encoder|cancoder|'
                  r'swerve x2|1824440000|usbc|overlay|housing|opi mount|id2322|cover$', re.I)
parts = load(SRC)
def robot(T):  # CAD (x to the robot's left, y back, z up) -> robot (x forward, y up, z right)
    return np.stack([-T[..., 1], T[..., 2], -T[..., 0]], -1)

AXIS = np.array([-0.0254, 0.0, 0.0])        # the DPC rotor's and the turret's axis (CAD mates)
PIVOT = np.array([0.3048, 0.2064, 0.0])     # the intake arm's (the arm shaft)
HOOD_PIVOT = np.array([0.0744, 0.5969, 0.0007])  # the main roller's axle, in the turret's frame
# the turret's frame: about its axis, turned so its shot (to the robot's right as exported) is
# forward; local = Ry(90 deg) (p - AXIS): x = the exported z, z = -(exported x - axis)
def turret_frame(T):
    d = T - AXIS
    return np.stack([d[..., 2], d[..., 1], -d[..., 0]], -1)

groups = {k: [] for k in ('body', 'bumper', 'intake', 'hopper', 'rotor', 'turret', 'hood')}
for p, T in parts:
    name = ' / '.join(p)
    if any(SKIP.search(x) for x in p[1:]): continue
    if 'Chefs Hat' in name: continue  # (drawn clean below)
    T = robot(T)
    if 'Bumper Assembly' in name:
        # the foam (Parts 10-25: two 2.5in noodles high round the frame), the wood (plates)
        (groups['bumper'] if re.search(r'/ Part \d+$', name) else groups['body']).append(T)
    elif 'Intake Arm Assembly' in name: groups['intake'].append(T)
    elif 'Hopper Slider Assembly' in name: groups['hopper'].append(T)
    elif 'DPC Rotor Assembly' in name: groups['rotor'].append(T)
    elif 'Turret Hood Assembly' in name: groups['hood'].append(turret_frame(T))
    elif 'Turret Assembly' in name: groups['turret'].append(turret_frame(T))
    else: groups['body'].append(T)
# the tube: a 32-sided bore, 77.5 mm in radius (the CAD's coroplast's inside), from the chef's hat's
# base (0.3461 m) to its top (0.5239)
def bore(r, y0, y1, n=32):
    a = np.linspace(0, 2 * np.pi, n + 1)
    P = np.stack([AXIS[0] + r * np.cos(a), np.zeros_like(a), r * np.sin(a)], -1)
    out = []
    for k in range(n):
        p0, p1 = P[k], P[k + 1]
        q = [p0 + [0, y0, 0], p1 + [0, y0, 0], p1 + [0, y1, 0], p0 + [0, y1, 0]]
        out += [[q[0], q[1], q[2]], [q[0], q[2], q[3]]]
    return np.array(out)
groups['rotor'].append(bore(0.0775, 0.3461, 0.5239))
G = {k: np.vstack(v) for k, v in groups.items()}

# the rollers the mechanism turns (as exported), cut out: [center, axis, radius, half length]
def cyl(c, a, r, h): return (np.array(c, float), np.array(a, float) / np.linalg.norm(a), r, h)
def cut(T, cyls):
    c = T.mean(1)
    keep = np.ones(len(T), bool)
    for (p, a, r, h) in cyls:
        d = c - p; s = d @ a
        rad = np.linalg.norm(d - np.outer(s, a), axis=1)
        keep &= ~((np.abs(s) <= h) & (rad <= r))
    return T[keep]
X, Y, Z = [1, 0, 0], [0, 1, 0], [0, 0, 1]
# (r604.js has the same: the intake's two 2in rollers in silicone tubing; the infeed's three 2in
# omni wheels on the rotor, standing up; the up-feed roller over them; the main roller and the
# hood roller in the turret's frame)
ROLL1, ROLL2 = (0.5826, 0.1524), (0.5553, 0.2381)
OMNI = (-0.1150, -0.0913)
UPFEED = (-0.1159, 0.3159)
MAIN, HOODR = (0.0744, 0.5969), (-0.0686, 0.6971)
G['intake'] = cut(G['intake'], [cyl([*ROLL1, 0], Z, 0.034, 0.33), cyl([*ROLL2, 0], Z, 0.034, 0.33)])
G['rotor'] = cut(G['rotor'], [cyl([OMNI[0], 0.2326, OMNI[1]], Y, 0.027, 0.045), cyl([UPFEED[0], UPFEED[1], 0], Z, 0.027, 0.115)])
G['hood'] = cut(G['hood'], [cyl([*MAIN, 0], Z, 0.031, 0.075), cyl([*HOODR, 0], Z, 0.022, 0.075)])

# Only the surfaces a FUEL can get to (as 2910's and 581's): the space a FUEL's center fits in,
# allowing 1.5 cm of squash (the tube is 2.5 mm round one), flooded from points along the ball
# path, everything as exported (the intake at their 0, the hopper out, the turret firing to the
# right, the hood at 35); and the surfaces within a FUEL's reach of it. The intake out front is
# open to the carpet; the space round the robot is walled off past the bumpers and over the top.
VOX, R = 0.01, 0.075
LO, HI = np.array([-0.46, 0.0, -0.46]), np.array([0.76, 0.80, 0.46])
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
def rot_y(T, a, c=AXIS):  # turn about the vertical through c by a (rad; three.js's sense)
    d = T - c; ca, sa = np.cos(a), np.sin(a)
    return np.stack([d[..., 0] * ca + d[..., 2] * sa, d[..., 1], -d[..., 0] * sa + d[..., 2] * ca], -1) + c
def from_turret(T):  # the turret's frame -> the robot's, as exported
    return np.stack([-T[..., 2], T[..., 1], T[..., 0]], -1) + AXIS
g = LO[None, :] + (np.indices(shape).reshape(3, -1).T + 0.5) * VOX
def rollers(solid, rotor_a):
    # (a FUEL can't be inside a roller: the intake's, the infeed's and up-feed's on the rotor
    # (turned with it), the turret's)
    pts = []
    for (x, y), r in [(ROLL1, 0.027), (ROLL2, 0.027)]:
        solid |= ((np.hypot(g[:, 0] - x, g[:, 1] - y) < r) & (np.abs(g[:, 2]) < 0.27)).reshape(shape)
    o = rot_y(np.array([[OMNI[0], 0.2326, OMNI[1]]]), rotor_a)[0]
    solid |= ((np.hypot(g[:, 0] - o[0], g[:, 2] - o[2]) < 0.025) & (np.abs(g[:, 1] - 0.2326) < 0.042)).reshape(shape)
    u = rot_y(np.array([[UPFEED[0], UPFEED[1], 0.0]]), rotor_a)[0]; ax = rot_y(np.array([[0, 0, 1.0]]), rotor_a, np.zeros(3))[0]
    d = g - u; s = d @ ax; rad = np.linalg.norm(d - np.outer(s, ax), axis=1)
    solid |= ((rad < 0.025) & (np.abs(s) < 0.11)).reshape(shape)
    for (x, y), r in [(MAIN, 0.027), (HOODR, 0.0175)]:
        c = from_turret(np.array([[x, y, 0.0]]))[0]
        solid |= ((np.hypot(g[:, 1] - c[1], g[:, 2] - c[2]) < r) & (np.abs(g[:, 0] - c[0]) < 0.057)).reshape(shape)
    return solid
fixed = {k: G[k] for k in ('body', 'bumper', 'intake', 'hopper')}
fixed['turret'] = from_turret(G['turret']); fixed['hood'] = from_turret(G['hood'])
pts = {k: samples(T) for k, T in fixed.items()}
base = np.zeros(shape, bool)
for P, _ in pts.values():
    ijk, ok = voxels(P)
    base[tuple(ijk[ok].T)] = True
base[:, 0, :] = True
# (the outside: beside the bumpers, behind them, out past the intake's front and over the top)
out = (np.abs(g[:, 2]) > 0.44) | (g[:, 0] > 0.74) | (g[:, 0] < -0.44) | (g[:, 1] > 0.78)
base |= out.reshape(shape)
# the ball path: the carpet in front of the intake, over the bumper, the hopper and its
# extension, the bucket all round, the ramp, the tube, the turret's throat, past the hood
seeds = np.array([[0.68, 0.08, 0], [0.68, 0.08, 0.2], [0.68, 0.08, -0.2], [0.45, 0.33, 0], [0.5, 0.45, 0.2], [0.2, 0.45, 0], [0.0, 0.45, -0.25],
                  [-0.25, 0.45, 0.2], [0.15, 0.24, 0.0], [-0.2, 0.24, 0.12], [0.1, 0.24, -0.2], [-0.025, 0.42, 0], [-0.025, 0.5, 0],
                  [-0.025, 0.6, 0], [-0.025, 0.66, 0.06]])
rotor_pts = samples(G['rotor'])
keep = {k: np.zeros(len(T), bool) for k, T in fixed.items()}
keep_rotor = np.zeros(len(G['rotor']), bool)
def reached(reach, P, tri, n):
    ijk, ok = voxels(P)
    hit = np.zeros(len(P), bool)
    hit[ok] = reach[tuple(ijk[ok].T)]
    k = np.zeros(n, bool)
    np.logical_or.at(k, tri, hit)
    return k
for step in range(8):
    a = -step * np.pi / 4  # (the rotor's eighth turns: it turns clockwise from above)
    solid = base.copy()
    RP = rot_y(rotor_pts[0], a)
    ijk, ok = voxels(RP)
    solid[tuple(ijk[ok].T)] = True
    solid = rollers(solid, a)
    free = ndimage.distance_transform_edt(~solid) * VOX >= R - 0.015
    lab, _ = ndimage.label(free)
    sk, ok = voxels(seeds)
    ids = set(lab[tuple(sk[ok].T)].tolist()) - {0}
    reach = ndimage.distance_transform_edt(~np.isin(lab, list(ids))) * VOX <= R + 0.02
    if step == 0:
        print('seeded regions', len(ids), 'of', lab.max(), [int(lab[tuple(s)]) for s in sk])
        for k, T in fixed.items(): keep[k] = reached(reach, *pts[k], len(T))
    keep_rotor |= reached(reach, RP, rotor_pts[1], len(G['rotor']))
for k, T in fixed.items(): print(f'reachable {k}: {keep[k].sum()} of {len(T)}')
print(f'reachable rotor: {keep_rotor.sum()} of {len(G["rotor"])}')

def simplify(T, grid):
    V = np.round(T.reshape(-1, 3) / grid).astype(np.int64)
    uniq, inv = np.unique(V, axis=0, return_inverse=True)
    I = inv.reshape(-1, 3)
    I = I[(I[:, 0] != I[:, 1]) & (I[:, 1] != I[:, 2]) & (I[:, 0] != I[:, 2])]
    I = np.unique(np.sort(I, axis=1), axis=0)
    used, I2 = np.unique(I, return_inverse=True)
    return np.round(uniq[used] * grid * 1000).astype(int), I2.reshape(-1, 3)
meshes = {
    'body': simplify(G['body'][keep['body']], 0.01),
    'intake': simplify(G['intake'][keep['intake']] - PIVOT, 0.005),
    'hopper': simplify(G['hopper'][keep['hopper']], 0.01),
    'rotor': simplify(G['rotor'][keep_rotor] - AXIS, 0.005),
    'turret': simplify(G['turret'][keep['turret']], 0.005),
    'hood': simplify(G['hood'][keep['hood']] - HOOD_PIVOT, 0.005),
}
res = {k: {'v': v.flatten().tolist(), 'i': i.flatten().tolist()} for k, (v, i) in meshes.items()}
with open(f'{ROOT}/js/mech/path604.js', 'w') as f:
    f.write('// 604\'s ball path\'s surfaces from their CAD (generated by tools/path-mesh-604.py; see js/mech/r604.js)\n')
    f.write('export default ' + json.dumps(res, separators=(',', ':')) + ';\n')
for k, (v, i) in meshes.items(): print(k, len(v), 'vertices', len(i), 'triangles')
print(os.path.getsize(f'{ROOT}/js/mech/path604.js') // 1024, 'KB')
