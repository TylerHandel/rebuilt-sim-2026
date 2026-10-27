# 604's hopper box against the intake arm, from their CAD parts (cad/robots/604-intake.glb, the arm
# on its pivot at their 0 deg; 604-hopper.glb, the box's side walls, exported out): how far out the
# box stays at each wrist angle, -5 to 98 deg. Surgical tubing pulls the box back against the arm,
# so it comes back until its front (0.608 m out, the CAD's retainer contact) meets the arm's
# furthest-forward part between the side walls (|z| under 0.33 m) and within the walls' height.
#   python3 tools/hopper-604.py   (prints HOPPER.ext for js/mech/r604.js)
import json, math, struct
PIVOT, OUT, FRONT, WALL_Y, WALL_Z = (0.3048, 0.2064), 0.2921, 0.608, (0.2603, 0.6961), 0.33
CT = {5120: ('b', 1, 127.0), 5121: ('B', 1, 255.0), 5122: ('h', 2, 32767.0), 5123: ('H', 2, 65535.0), 5125: ('I', 4, None), 5126: ('f', 4, None)}

def points(path):
    b = open(path, 'rb').read(); n = struct.unpack('<I', b[12:16])[0]; J = json.loads(b[20:20 + n]); B = b[20 + n + 8:]
    def acc(i):
        a = J['accessors'][i]; bv = J['bufferViews'][a['bufferView']]; f, sz, nm = CT[a['componentType']]; k = {'SCALAR': 1, 'VEC2': 2, 'VEC3': 3, 'VEC4': 4}[a['type']]
        st = bv.get('byteStride', sz * k); off = bv.get('byteOffset', 0) + a.get('byteOffset', 0)
        for j in range(a['count']):
            v = struct.unpack_from('<' + f * k, B, off + j * st)
            yield tuple(max(x / nm, -1.0) for x in v) if a.get('normalized') and nm else v
    def mat(nd):
        if 'matrix' in nd: m = nd['matrix']; return [[m[c * 4 + r] for c in range(4)] for r in range(4)]
        t = nd.get('translation', [0, 0, 0]); x, y, z, w = nd.get('rotation', [0, 0, 0, 1]); s = nd.get('scale', [1, 1, 1])
        R = [[1 - 2 * (y * y + z * z), 2 * (x * y - z * w), 2 * (x * z + y * w)], [2 * (x * y + z * w), 1 - 2 * (x * x + z * z), 2 * (y * z - x * w)], [2 * (x * z - y * w), 2 * (y * z + x * w), 1 - 2 * (x * x + y * y)]]
        return [[R[r][0] * s[0], R[r][1] * s[1], R[r][2] * s[2], t[r]] for r in range(3)] + [[0, 0, 0, 1]]
    mm = lambda A, C: [[sum(A[r][k] * C[k][c] for k in range(4)) for c in range(4)] for r in range(4)]
    out = []
    def walk(i, M):
        nd = J['nodes'][i]; W = mm(M, mat(nd))
        if 'mesh' in nd:
            for p in J['meshes'][nd['mesh']]['primitives']:
                for v in acc(p['attributes']['POSITION']): out.append([W[r][0] * v[0] + W[r][1] * v[1] + W[r][2] * v[2] + W[r][3] for r in range(3)])
        for c in nd.get('children', []): walk(c, W)
    for r in J['scenes'][J.get('scene', 0)]['nodes']: walk(r, [[1 if r == c else 0 for c in range(4)] for r in range(4)])
    return out

arm = [p for p in points('cad/robots/604-intake.glb') if abs(p[2]) <= WALL_Z]
ext = []
for d in range(-5, 99):
    a = math.radians(d); c, s = math.cos(a), math.sin(a); front = -1
    for x, y, _ in arm:
        X, Y = PIVOT[0] + x * c - y * s, PIVOT[1] + x * s + y * c
        if WALL_Y[0] <= Y <= WALL_Y[1]: front = max(front, X)
    ext.append(round(max(0, min(OUT, front - (FRONT - OUT))), 3))
# (no further out than it is at any higher angle: a part coming up into the walls' height from
# below meets their bottom edge rather than pushing the box out)
ext = [max(ext[i:]) for i in range(len(ext))]
print('ext: [' + ', '.join(f'{e:g}' for e in ext) + '],')
