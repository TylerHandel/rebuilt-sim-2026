#!/usr/bin/env python3
"""Split 8793's cable-carrier track out of cad/robots/8793-turret.glb.

The recipe (cad/robots/8793.json) put the "Cable Carrier Quadrant 1-4" parts in the turret's file, so
they turned with the turret. On the robot the track is fixed: its four printed quadrants sit on the
tower, 1 mm over the 200T turntable gear, held at the back by the two "Part 1" brackets, and the
chain spools inside it (js/echain.js). This finds the turret file's triangles whose corners are the
quadrants' own vertices in the STEP tessellation the file was made from (the simplifier keeps
vertices where they are; two of three is enough, the third may have been welded to a neighbor, so
long as the triangle lies in the track's ring), writes them to their own file (same frame: the
turret's axis, y up, the robot's front +x) and leaves them degenerate in the turret's file (its
buffers keep their size).

  python3 -I tools/split-8793-track.py cad/robots/8793-turret.glb <r8793-step.glb> cad/robots/8793-track.glb

<r8793-step.glb> is tools/step2glb.py's output for the robot's STEP (see cad/README.md).
"""
import json, struct, sys
import numpy as np

CT = {5126: np.float32, 5125: np.uint32, 5123: np.uint16, 5121: np.uint8, 5122: np.int16, 5120: np.int8}
NC = {'SCALAR': 1, 'VEC2': 2, 'VEC3': 3, 'VEC4': 4, 'MAT4': 16}
TURRET_X = -0.127  # the turret file's origin, in the robot's frame (cad/robots/8793.json)


def load(path):
    b = open(path, 'rb').read()
    jl = struct.unpack('<I', b[12:16])[0]
    j = json.loads(b[20:20 + jl])
    off = 20 + jl
    bl = struct.unpack('<I', b[off:off + 4])[0]
    return j, bytearray(b[off + 8:off + 8 + bl]), b


def view(j, i):
    a = j['accessors'][i]
    bv = j['bufferViews'][a['bufferView']]
    return a, bv, bv.get('byteOffset', 0) + a.get('byteOffset', 0)


def acc(j, binb, i):
    a, bv, off = view(j, i)
    dt, n = CT[a['componentType']], NC[a['type']]
    size = n * np.dtype(dt).itemsize
    stride = bv.get('byteStride', 0) or size
    raw = np.frombuffer(bytes(binb), dtype=np.uint8)
    out = np.stack([raw[off + k * stride: off + k * stride + size].view(dt) for k in range(a['count'])]) if stride != size else \
        np.frombuffer(bytes(binb), dtype=dt, count=a['count'] * n, offset=off).reshape(a['count'], n)
    if a.get('normalized'):
        out = np.maximum(out.astype(float) / {np.int16: 32767.0, np.int8: 127.0, np.uint16: 65535.0, np.uint8: 255.0}[dt], -1.0)
    return out


def local(n):
    if 'matrix' in n:
        return np.array(n['matrix'], dtype=float).reshape(4, 4).T
    M = np.eye(4)
    if 'scale' in n:
        M = np.diag(list(n['scale']) + [1]) @ M
    if 'rotation' in n:
        x, y, z, w = n['rotation']
        R = np.eye(4)
        R[:3, :3] = [[1 - 2 * (y * y + z * z), 2 * (x * y - z * w), 2 * (x * z + y * w)],
                     [2 * (x * y + z * w), 1 - 2 * (x * x + z * z), 2 * (y * z - x * w)],
                     [2 * (x * z - y * w), 2 * (y * z + x * w), 1 - 2 * (x * x + y * y)]]
        M = R @ M
    if 'translation' in n:
        T = np.eye(4); T[:3, 3] = n['translation']; M = T @ M
    return M


def step_quadrants(path):
    """the quadrants' vertices, in the turret file's frame"""
    j, binb, _ = load(path)
    par = {c: i for i, n in enumerate(j['nodes']) for c in n.get('children', [])}
    out = []
    for i, n in enumerate(j['nodes']):
        if 'mesh' not in n:
            continue
        names, k, M = [], i, np.eye(4)
        while k is not None:
            names.append(j['nodes'][k].get('name', '')); M = local(j['nodes'][k]) @ M; k = par.get(k)
        if not any('Cable Carrier Quadrant' in s for s in names):
            continue
        for pr in j['meshes'][n['mesh']]['primitives']:
            P = acc(j, binb, pr['attributes']['POSITION']).astype(float)
            P = (M[:3, :3] @ P.T).T + M[:3, 3]
            # CAD Z up, the robot's front -y: model = (-y, z, -x); less the turret file's origin
            out.append(np.stack([-P[:, 1] - TURRET_X, P[:, 2], -P[:, 0]], 1))
    return np.concatenate(out)


def glb_bytes(j, binb):
    js = json.dumps(j, separators=(',', ':')).encode()
    js += b' ' * (-len(js) % 4)
    binb = bytes(binb) + b'\0' * (-len(binb) % 4)
    total = 12 + 8 + len(js) + 8 + len(binb)
    return struct.pack('<III', 0x46546C67, 2, total) + struct.pack('<II', len(js), 0x4E4F534A) + js + struct.pack('<II', len(binb), 0x004E4942) + binb


def main(turret, stepglb, out):
    Q = step_quadrants(stepglb)
    tol = 2e-4
    keys = set()
    for v in np.floor(Q / tol).astype(int):
        for d in ((0, 0, 0), (1, 0, 0), (0, 1, 0), (0, 0, 1), (1, 1, 0), (1, 0, 1), (0, 1, 1), (1, 1, 1)):
            keys.add((v[0] - d[0], v[1] - d[1], v[2] - d[2]))
    j, binb, _ = load(turret)
    P_out, N_out, mat = [], [], None
    moved = 0
    for n in j['nodes']:
        if 'mesh' not in n:
            continue
        M = local(n)
        Rn = M[:3, :3] / np.linalg.norm(M[:3, 0])
        for pr in j['meshes'][n['mesh']]['primitives']:
            P = acc(j, binb, pr['attributes']['POSITION']).astype(float)
            P = (M[:3, :3] @ P.T).T + M[:3, 3]
            N = acc(j, binb, pr['attributes']['NORMAL']).astype(float) if 'NORMAL' in pr['attributes'] else None
            I = acc(j, binb, pr['indices']).astype(np.int64).reshape(-1, 3)
            on = np.array([tuple(k) in keys for k in np.floor(P / tol).astype(int)])
            T = P[I]
            c = T.mean(1)
            r = np.hypot(c[:, 0], c[:, 2])
            ring = (r > 0.119) & (r < 0.2425) & (T[..., 1].min(1) > 0.3105) & (T[..., 1].max(1) < 0.3625)
            sel = ring & (on[I].sum(1) >= 2)
            if not sel.any():
                continue
            mat = j['materials'][pr['material']]
            moved += int(sel.sum())
            for t in I[sel]:
                P_out.append(P[t])
                N_out.append((Rn @ N[t].T).T if N is not None else np.repeat(np.cross(P[t[1]] - P[t[0]], P[t[2]] - P[t[0]])[None], 3, 0))
            # leave them degenerate in the turret's file
            a, bv, off = view(j, pr['indices'])
            dt = CT[a['componentType']]
            Iw = I.copy()
            Iw[sel] = Iw[sel][:, :1]
            binb[off:off + Iw.size * np.dtype(dt).itemsize] = Iw.astype(dt).tobytes()
    P_out = np.concatenate(P_out).astype(np.float32)
    N_out = np.concatenate(N_out)
    N_out = (N_out / np.maximum(1e-9, np.linalg.norm(N_out, axis=1))[:, None]).astype(np.float32)
    print(f'track: {moved} triangles moved out of {turret}')
    open(turret, 'wb').write(glb_bytes(j, binb))
    # the track's own file: one mesh, its material the turret file's (a dark print)
    data = P_out.tobytes() + N_out.tobytes()
    tj = {
        'asset': {'version': '2.0', 'generator': 'tools/split-8793-track.py'},
        'scene': 0, 'scenes': [{'nodes': [0]}],
        'nodes': [{'name': 'Cable Carrier Quadrants 1-4', 'mesh': 0}],
        'meshes': [{'name': 'Cable Carrier Quadrants 1-4', 'primitives': [{'attributes': {'POSITION': 0, 'NORMAL': 1}, 'material': 0}]}],
        'materials': [{k: v for k, v in mat.items() if k in ('name', 'pbrMetallicRoughness', 'doubleSided')}],
        'buffers': [{'byteLength': len(data)}],
        'bufferViews': [{'buffer': 0, 'byteOffset': 0, 'byteLength': P_out.nbytes, 'target': 34962},
                        {'buffer': 0, 'byteOffset': P_out.nbytes, 'byteLength': N_out.nbytes, 'target': 34962}],
        'accessors': [{'bufferView': 0, 'componentType': 5126, 'count': len(P_out), 'type': 'VEC3', 'min': P_out.min(0).tolist(), 'max': P_out.max(0).tolist()},
                      {'bufferView': 1, 'componentType': 5126, 'count': len(N_out), 'type': 'VEC3'}],
    }
    open(out, 'wb').write(glb_bytes(tj, data))
    print(f'wrote {out}: {len(P_out) // 3} triangles, y {P_out[:, 1].min():.4f}..{P_out[:, 1].max():.4f}')


if __name__ == '__main__':
    main(*sys.argv[1:4])
