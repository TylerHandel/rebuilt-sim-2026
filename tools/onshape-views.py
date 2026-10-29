#!/usr/bin/env python3
# Orthographic shaded views (the renders Onshape serves for viewing) of an Onshape element, as PNGs.
# For CAD whose document doesn't allow export (1690's): a view at a known scale and center is
# what the model's dimensions were measured from (cad/README.md).
#   ONSHAPE_ACCESS_KEY=... ONSHAPE_SECRET_KEY=... python3 tools/onshape-views.py <element> <out-prefix>
#       [px m/pixel = 0.001] [size px = 1000] [cx,cy,cz m = 0,0.4,0] [views: R L T F B U, or C:dx,dy,dz:name]
# DID and WID (environment) pick the document; the defaults are 1690's "Kepler". <element> is an
# element id or one of 1690's names below. The image's center is the view's origin c, so a pixel
# (i, j) is c + (i - size/2) * px along the view's right and -(j - size/2) * px along its up.
# Onshape's axes in 1690's CAD: X = the robot's left, Y = up, Z = forward; origin on the carpet
# under the robot's center (sim x = Z, y = Y, z = -X). Its sub-assemblies share those coordinates.
import os, sys, json, ssl, base64, urllib.request, urllib.error

DID = os.environ.get('DID', 'd09a47dbfc1b125f63a78ac7')
WID = os.environ.get('WID', 'd71d2275472653904f9b717a')
NAMES = {  # 1690's elements: the top level, its sub-assemblies and the imported part studio
    'top': '03166dd8363262636c9f6b66', '100': 'ab84ebfd426c80f7950f94af', '200': '57eccef95d2469796f4674b0',
    '300': 'e9d90908e685123751aef046', '400': '4dd07aeb720bc59fcb0836d8', '600': '9bc85ebf99666828b962bd68',
    'bumper': '31da1194e9c699df8559f76b', 'ps': 'fb335bf27ce4b365351755cc',
}
VIEWS = {  # (right, up) in Onshape's coordinates
    'R': ((0, 0, 1), (0, 1, 0)), 'L': ((0, 0, -1), (0, 1, 0)), 'T': ((-1, 0, 0), (0, 0, 1)),
    'F': ((1, 0, 0), (0, 1, 0)), 'B': ((-1, 0, 0), (0, 1, 0)), 'U': ((1, 0, 0), (0, 0, 1)),
}


def get(path):
    # credentials come from the environment only, and are never printed
    tok = base64.b64encode(f"{os.environ['ONSHAPE_ACCESS_KEY']}:{os.environ['ONSHAPE_SECRET_KEY']}".encode()).decode()
    req = urllib.request.Request('https://cad.onshape.com' + path, headers={'Authorization': 'Basic ' + tok, 'Accept': 'application/json'})
    ca = os.environ.get('SSL_CERT_FILE')
    ctx = ssl.create_default_context(cafile=ca) if ca else ssl.create_default_context()
    try:
        with urllib.request.urlopen(req, context=ctx, timeout=300) as r:
            return r.status, json.loads(r.read())
    except urllib.error.HTTPError as e:
        return e.code, e.read()[:400].decode('utf-8', 'replace')


def dot(a, b): return sum(x * y for x, y in zip(a, b))
def cross(a, b): return (a[1] * b[2] - a[2] * b[1], a[2] * b[0] - a[0] * b[2], a[0] * b[1] - a[1] * b[0])
def unit(a):
    n = dot(a, a) ** 0.5
    return tuple(x / n for x in a)


def matrix(r0, r1, c):
    # Onshape's viewMatrix: 3x4, rows the view's right, up and toward-the-camera axes, the last column -R c
    r2 = cross(r0, r1)
    rows = [(*r, -dot(r, c)) for r in (r0, r1, r2)]
    return ','.join(f'{v:.6g}' for row in rows for v in row)


def render(el, r0, r1, c, px, size, out):
    el = NAMES.get(el, el)
    kind = 'partstudios' if el == NAMES['ps'] and DID == 'd09a47dbfc1b125f63a78ac7' else 'assemblies'
    st, v = get(f'/api/v6/{kind}/d/{DID}/w/{WID}/e/{el}/shadedviews?viewMatrix={matrix(r0, r1, c)}'
                f'&outputHeight={size}&outputWidth={size}&pixelSize={px}')
    if st != 200:
        print(st, v)
        return False
    with open(out, 'wb') as f:
        f.write(base64.b64decode(v['images'][0]))
    print('wrote', out)
    return True


if __name__ == '__main__':
    if len(sys.argv) < 3:
        sys.exit(__doc__ or 'usage: onshape-views.py <element> <out-prefix> [px] [size] [cx,cy,cz] [views...]')
    el, pre = sys.argv[1], sys.argv[2]
    px = float(sys.argv[3]) if len(sys.argv) > 3 else 0.001
    size = int(sys.argv[4]) if len(sys.argv) > 4 else 1000
    c = tuple(float(x) for x in sys.argv[5].split(',')) if len(sys.argv) > 5 else (0, 0.4, 0)
    for n in sys.argv[6:] or list(VIEWS):
        if n.startswith('C:'):  # C:dx,dy,dz:name, the direction toward the camera; up as near +Y as it goes
            _, d, name = n.split(':')
            r2 = unit(tuple(float(x) for x in d.split(',')))
            up = (0, 1.0, 0) if abs(r2[1]) < 0.95 else (0, 0, 1.0)
            r1 = unit(tuple(u - r2[i] * dot(r2, up) for i, u in enumerate(up)))
            render(el, cross(r1, r2), r1, c, px, size, f'{pre}_{name}.png')
        else:
            render(el, *VIEWS[n], c, px, size, f'{pre}_{n}.png')
