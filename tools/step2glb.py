"""STEP assembly -> GLB (CAD frame, meters), tessellated by OpenCascade with the surfaces' own
normals, so round parts render round. Keeps the assembly tree (subassembly instances and part
names) for tools/extract-parts.mjs to pick parts from. Each part is meshed once and instanced.

  python3 -m venv /tmp/ocp && /tmp/ocp/bin/pip install cadquery-ocp
  /tmp/ocp/bin/python tools/step2glb.py robot.step robot-step.glb [--lin 0.0004] [--ang 0.3]
      [--only "32 - R2 Shooter" ...] [--skip "screw|nut|washer"]

--lin: max distance from the true surface (m); --ang: max angle between facets (rad).
--only: keep just the top-level subassemblies whose names start with these; --skip: regex of
part names not to mesh (hardware), which saves most of the time on a whole robot.
"""
import argparse, json, re, struct, sys
import numpy as np
from OCP.STEPCAFControl import STEPCAFControl_Reader
from OCP.TDocStd import TDocStd_Document
from OCP.TCollection import TCollection_ExtendedString
from OCP.XCAFDoc import XCAFDoc_DocumentTool, XCAFDoc_ColorType, XCAFDoc_ColorTool
from OCP.TDF import TDF_Label
from OCP.collections import Sequence_TDF_Label as TDF_LabelSequence
from OCP.TDataStd import TDataStd_Name
from OCP.Interface import Interface_Static
from OCP.IFSelect import IFSelect_RetDone
from OCP.BRepMesh import BRepMesh_IncrementalMesh
from OCP.TopExp import TopExp_Explorer
from OCP.TopAbs import TopAbs_FACE, TopAbs_REVERSED
from OCP.TopoDS import TopoDS
from OCP.BRep import BRep_Tool
from OCP.BRepLib import BRepLib_ToolTriangulatedShape
from OCP.TopLoc import TopLoc_Location
from OCP.Quantity import Quantity_ColorRGBA

ap = argparse.ArgumentParser()
ap.add_argument('src'); ap.add_argument('out')
ap.add_argument('--lin', type=float, default=0.0004)
ap.add_argument('--ang', type=float, default=0.3)
ap.add_argument('--only', nargs='*', default=[])
ap.add_argument('--skip', default='')
a = ap.parse_args()
skip = re.compile(a.skip, re.I) if a.skip else None

doc = TDocStd_Document(TCollection_ExtendedString('step'))
rd = STEPCAFControl_Reader()
rd.SetColorMode(True); rd.SetNameMode(True)
Interface_Static.SetCVal_s('xstep.cascade.unit', 'M') # after the reader sets up its defaults
if rd.ReadFile(a.src) != IFSelect_RetDone: sys.exit('could not read ' + a.src)
rd.Transfer(doc)
shapes = XCAFDoc_DocumentTool.ShapeTool_s(doc.Main())
colors = XCAFDoc_DocumentTool.ColorTool_s(doc.Main())

def name_of(label):
    n = TDataStd_Name()
    return n.Get().ToExtString() if label.FindAttribute(TDataStd_Name.GetID_s(), n) else ''

def color_of(label, shape):
    c = Quantity_ColorRGBA()
    for t in (XCAFDoc_ColorType.XCAFDoc_ColorSurf, XCAFDoc_ColorType.XCAFDoc_ColorGen):
        if XCAFDoc_ColorTool.GetColor_s(label, t, c) or colors.GetColor(shape, t, c):
            rgb = c.GetRGB()
            return (rgb.Red(), rgb.Green(), rgb.Blue(), c.Alpha())
    return None

def trsf_matrix(loc):
    t = loc.Transformation()
    m = np.eye(4)
    for r in range(3):
        for c in range(4): m[r, c] = t.Value(r + 1, c + 1)
    return m

meshes = {}  # part label entry -> (positions, normals, indices) in the part's own frame
def mesh_part(label, shape):
    key = label.EntryDumpToString() if hasattr(label, 'EntryDumpToString') else str(label.Tag()) + name_of(label)
    if key in meshes: return meshes[key]
    BRepMesh_IncrementalMesh(shape, a.lin, False, a.ang, True)
    P, N, I = [], [], []
    ex = TopExp_Explorer(shape, TopAbs_FACE)
    while ex.More():
        f = TopoDS.Face(ex.Current())
        loc = TopLoc_Location()
        tri = BRep_Tool.Triangulation_s(f, loc)
        if tri is not None and tri.NbTriangles():
            if not tri.HasNormals(): BRepLib_ToolTriangulatedShape.ComputeNormals_s(f, tri)
            M = trsf_matrix(loc)
            rev = f.Orientation() == TopAbs_REVERSED
            base = sum(len(p) for p in P)
            pts = np.array([[tri.Node(i).X(), tri.Node(i).Y(), tri.Node(i).Z()] for i in range(1, tri.NbNodes() + 1)])
            nrm = np.array([[tri.Normal(i).X(), tri.Normal(i).Y(), tri.Normal(i).Z()] for i in range(1, tri.NbNodes() + 1)])
            pts = pts @ M[:3, :3].T + M[:3, 3]
            nrm = nrm @ M[:3, :3].T
            if rev: nrm = -nrm
            ids = []
            for i in range(1, tri.NbTriangles() + 1):
                n1, n2, n3 = tri.Triangle(i).Get()
                ids.append((n1, n3, n2) if rev else (n1, n2, n3))
            P.append(pts); N.append(nrm); I.append(np.array(ids, dtype=np.int64) - 1 + base)
        ex.Next()
    if not P: meshes[key] = None
    else: meshes[key] = (np.vstack(P).astype(np.float32), np.vstack(N).astype(np.float32), np.vstack(I).astype(np.uint32))
    return meshes[key]

# glTF assembly
nodes, gmeshes, accessors, views, mats, blob = [], [], [], [], [], bytearray()
mat_ids, mesh_ids = {}, {}
def add_view(data, target):
    global blob
    while len(blob) % 4: blob += b'\0'
    views.append({'buffer': 0, 'byteOffset': len(blob), 'byteLength': len(data), 'target': target})
    blob += data
    return len(views) - 1
def material(col):
    k = col or 'none'
    if k not in mat_ids:
        c = list(col) if col else [0.7, 0.7, 0.7, 1]
        m = {'name': 'm%d' % len(mats), 'pbrMetallicRoughness': {'baseColorFactor': c, 'metallicFactor': 0.2, 'roughnessFactor': 0.6}}
        if c[3] < 0.99: m['alphaMode'] = 'BLEND'; m['doubleSided'] = True
        mats.append(m); mat_ids[k] = len(mats) - 1
    return mat_ids[k]
def gl_mesh(key, name, data, col):
    k = (key, col)
    if k in mesh_ids: return mesh_ids[k]
    P, N, I = data
    pv = add_view(P.tobytes(), 34962); nv = add_view(N.tobytes(), 34962); iv = add_view(I.tobytes(), 34963)
    accessors.append({'bufferView': pv, 'componentType': 5126, 'count': len(P), 'type': 'VEC3', 'min': P.min(0).tolist(), 'max': P.max(0).tolist()})
    accessors.append({'bufferView': nv, 'componentType': 5126, 'count': len(N), 'type': 'VEC3'})
    accessors.append({'bufferView': iv, 'componentType': 5125, 'count': I.size, 'type': 'SCALAR'})
    n = len(accessors)
    gmeshes.append({'name': name, 'primitives': [{'attributes': {'POSITION': n - 3, 'NORMAL': n - 2}, 'indices': n - 1, 'material': material(col)}]})
    mesh_ids[k] = len(gmeshes) - 1
    return mesh_ids[k]

tris = 0
def build(label, name, M, col, depth):
    """label: a shape label (prototype); returns a node index or None"""
    global tris
    shape = shapes.GetShape_s(label)
    col = color_of(label, shape) or col
    node = {'name': name, 'matrix': M.T.flatten().tolist()}
    if shapes.IsAssembly_s(label):
        comps = TDF_LabelSequence()
        shapes.GetComponents_s(label, comps, False)
        kids = []
        for i in range(1, comps.Length() + 1):
            c = comps.Value(i)
            ref = TDF_Label()
            if not shapes.GetReferredShape_s(c, ref): continue
            cname = name_of(c) or name_of(ref)
            if depth == 0 and a.only and not any(cname.startswith(o) for o in a.only): continue
            cm = trsf_matrix(shapes.GetShape_s(c).Location())
            k = build(ref, cname, cm, color_of(c, shapes.GetShape_s(c)) or col, depth + 1)
            if k is not None: kids.append(k)
        if not kids: return None
        node['children'] = kids
    else:
        pname = name_of(label) or name
        if skip and (skip.search(pname) or skip.search(name)): return None
        data = mesh_part(label, shape)
        if data is None: return None
        node['name'] = pname
        node['mesh'] = gl_mesh(str(label.Tag()) + pname, pname, data, col)
        tris += len(data[2])
    nodes.append(node)
    return len(nodes) - 1

free = TDF_LabelSequence()
shapes.GetFreeShapes(free)
roots = [k for k in (build(free.Value(i), name_of(free.Value(i)), np.eye(4), None, 0) for i in range(1, free.Length() + 1)) if k is not None]
while len(blob) % 4: blob += b'\0'
gltf = {'asset': {'version': '2.0', 'generator': 'tools/step2glb.py'}, 'scene': 0, 'scenes': [{'nodes': roots}],
        'nodes': nodes, 'meshes': gmeshes, 'accessors': accessors, 'bufferViews': views, 'materials': mats,
        'buffers': [{'byteLength': len(blob)}]}
js = json.dumps(gltf).encode()
while len(js) % 4: js += b' '
with open(a.out, 'wb') as f:
    f.write(struct.pack('<III', 0x46546C67, 2, 12 + 8 + len(js) + 8 + len(blob)))
    f.write(struct.pack('<II', len(js), 0x4E4F534A)); f.write(js)
    f.write(struct.pack('<II', len(blob), 0x004E4942)); f.write(blob)
print(f'wrote {a.out}: {len(meshes)} parts meshed, {tris} triangles placed, {len(blob) / 1e6:.1f} MB')
