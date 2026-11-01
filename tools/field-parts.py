"""Measure the official field CAD: list its parts' bounding boxes, or a part's outline.

The raw export of the field ("FE-2026: REBUILT Playing Field", tools/onshape-export.sh, ~2.8M
triangles, before tools/slim-glb.mjs) keeps every part's name in its node path, e.g.
  .../GE-26200: Trench <2>/GE-26201: Trench Hinged Assy <1>_1/GE-26214: Trench Hinged Arm Weldment <1>/...
Don't commit the export. Needs `pip install trimesh numpy scipy`; run with python3 -I on a file
from outside (it's untrusted data).

  python3 -I tools/field-parts.py boxes field-raw.glb [DEPTH] [REGEX]
      bounding boxes in inches of the parts matching REGEX, grouped DEPTH levels below the field
      (hardware left out: screws, bolts, nuts, rivets, PEMs, washers, cable ties)
  python3 -I tools/field-parts.py outline field-raw.glb REGEX AXES
      the convex hull, in inches, of each matching mesh projected on AXES (xy, xz or yz)

CAD frame: inches here (the GLB is meters), Z up, the CENTER LINE at X = 0 and the field's long middle
at Y = 0. cad/manifest.json turns it into the game's world as x = -X, y = Z, z = Y: the blue half is
CAD +X, the scoring table CAD +Y. (The instance numbers and the AprilTag IDs on the parts are copies
on one side: the red OUTPOST's tags are named ID 29 and 30 too. Go by position.)
"""
import re
import sys
from collections import OrderedDict

import numpy as np
import trimesh
from scipy.spatial import ConvexHull

IN = 0.0254
HARDWARE = re.compile(r'screw|bolt|nut|rivet|pem |washer|cable tie', re.I)


def instances(path):
    sc = trimesh.load(path, force='scene')
    g = sc.graph
    par = g.transforms.parents
    for node in g.nodes_geometry:
        names = [node]
        n = node
        while n in par:
            n = par[n]
            names.append(n)
        T, gname = g[node]
        yield '/'.join(reversed(names)), trimesh.transform_points(sc.geometry[gname].vertices, T) / IN


def boxes(path, depth=1, pattern=None):
    rx = re.compile(pattern) if pattern else None
    groups = OrderedDict()
    for p, v in instances(path):
        if (rx and not rx.search(p)) or HARDWARE.search(p.split('/')[-1]):
            continue
        key = '/'.join(p.split('/')[2:2 + depth])
        lo, hi = v.min(0), v.max(0)
        g = groups.setdefault(key, [0, lo, hi])
        g[0] += 1
        g[1] = np.minimum(g[1], lo)
        g[2] = np.maximum(g[2], hi)
    for k, (n, lo, hi) in sorted(groups.items()):
        print(f'{k[:110]:110s} n={n:4d} x[{lo[0]:8.2f},{hi[0]:8.2f}] y[{lo[1]:8.2f},{hi[1]:8.2f}] z[{lo[2]:7.2f},{hi[2]:7.2f}]')


def outline(path, pattern, axes):
    rx = re.compile(pattern)
    ax = ['xyz'.index(c) for c in axes]
    for p, v in instances(path):
        if not rx.search(p):
            continue
        q = v[:, ax]
        h = ConvexHull(q)
        print('##', p[-150:])
        print('  ', ' '.join(f'({a:.2f},{b:.2f})' for a, b in q[h.vertices]))


if __name__ == '__main__':
    cmd = sys.argv[1] if len(sys.argv) > 1 else ''
    if cmd == 'boxes':
        boxes(sys.argv[2], int(sys.argv[3]) if len(sys.argv) > 3 else 1, sys.argv[4] if len(sys.argv) > 4 else None)
    elif cmd == 'outline':
        outline(sys.argv[2], sys.argv[3], sys.argv[4])
    else:
        print(__doc__)
