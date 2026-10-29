# Every part of a STEP assembly, placed: its path, world bounding box, volume, center of mass and its
# cylindrical faces' axes (the STEP's frame, m): for each distinct axis (direction, a point on it), the
# radii and the area of the cylinders on it. For CAD without mates (1690's X_T import), where the
# rotating parts' axes have to be read off their geometry: a roller's, wheel's or pulley's bore and
# tread share one axis. Needs OpenCascade's Python bindings (cadquery-ocp) and numpy.
#   python -I tools/step-axes.py r1690.step axes.json [skip-regex]
import json, re, sys
import numpy as np
from OCP.STEPCAFControl import STEPCAFControl_Reader
from OCP.TDocStd import TDocStd_Document
from OCP.TCollection import TCollection_ExtendedString
from OCP.XCAFDoc import XCAFDoc_DocumentTool
from OCP.TDF import TDF_Label
from OCP.collections import Sequence_TDF_Label as TDF_LabelSequence
from OCP.TDataStd import TDataStd_Name
from OCP.Interface import Interface_Static
from OCP.IFSelect import IFSelect_RetDone
from OCP.TopExp import TopExp_Explorer
from OCP.TopAbs import TopAbs_FACE
from OCP.TopoDS import TopoDS
from OCP.BRepAdaptor import BRepAdaptor_Surface
from OCP.GeomAbs import GeomAbs_Cylinder
from OCP.GProp import GProp_GProps
from OCP.BRepGProp import BRepGProp
from OCP.Bnd import Bnd_Box
from OCP.BRepBndLib import BRepBndLib

src, out = sys.argv[1:3]
skip = re.compile(sys.argv[3], re.I) if len(sys.argv) > 3 else None
doc = TDocStd_Document(TCollection_ExtendedString('step'))
rd = STEPCAFControl_Reader(); rd.SetNameMode(True)
Interface_Static.SetCVal_s('xstep.cascade.unit', 'M')
if rd.ReadFile(src) != IFSelect_RetDone: sys.exit('could not read')
rd.Transfer(doc)
shapes = XCAFDoc_DocumentTool.ShapeTool_s(doc.Main())


def name_of(label):
    n = TDataStd_Name()
    return n.Get().ToExtString() if label.FindAttribute(TDataStd_Name.GetID_s(), n) else ''


def mat(loc):
    t = loc.Transformation(); m = np.eye(4)
    for r in range(3):
        for c in range(4): m[r, c] = t.Value(r + 1, c + 1)
    return m


cache = {}
def part_axes(label, shape):
    key = label.Tag()
    if key in cache: return cache[key]
    axes = []
    ex = TopExp_Explorer(shape, TopAbs_FACE)
    while ex.More():
        f = TopoDS.Face_s(ex.Current()) if hasattr(TopoDS, 'Face_s') else TopoDS.Face(ex.Current())
        s = BRepAdaptor_Surface(f)
        if s.GetType() == GeomAbs_Cylinder:
            cy = s.Cylinder(); ax = cy.Axis()
            d = ax.Direction(); p = ax.Location()
            g = GProp_GProps(); BRepGProp.SurfaceProperties_s(f, g)
            axes.append(([d.X(), d.Y(), d.Z()], [p.X(), p.Y(), p.Z()], cy.Radius(), g.Mass()))
        ex.Next()
    b = Bnd_Box(); BRepBndLib.Add_s(shape, b)
    box = None
    if not b.IsVoid():
        p0, p1 = b.CornerMin(), b.CornerMax()
        box = (p0.X(), p0.Y(), p0.Z(), p1.X(), p1.Y(), p1.Z())
    g = GProp_GProps(); BRepGProp.VolumeProperties_s(shape, g)
    vol = g.Mass(); cm = g.CentreOfMass()
    cache[key] = (axes, box, vol, [cm.X(), cm.Y(), cm.Z()])
    return cache[key]


rows = []
def walk(label, path, M):
    shape = shapes.GetShape_s(label)
    if shapes.IsAssembly_s(label):
        comps = TDF_LabelSequence(); shapes.GetComponents_s(label, comps, False)
        for i in range(1, comps.Length() + 1):
            c = comps.Value(i); ref = TDF_Label()
            if not shapes.GetReferredShape_s(c, ref): continue
            cname = name_of(c) or name_of(ref)
            walk(ref, path + [cname], M @ mat(shapes.GetShape_s(c).Location()))
        return
    pname = name_of(label) or path[-1]
    if skip and (skip.search(pname) or skip.search(path[-1])): return
    axes, box, vol, cm = part_axes(label, shape)
    R, T = M[:3, :3], M[:3, 3]
    # group the cylinders by axis (direction within 1 deg, the lines within 0.5 mm)
    groups = []
    for d, p, r, a in axes:
        d = R @ np.array(d); p = R @ np.array(p) + T
        if d[np.argmax(np.abs(d))] < 0: d = -d
        for g in groups:
            if abs(abs(g['d'] @ d) - 1) < 1.5e-4 and np.linalg.norm(np.cross(p - g['p'], g['d'])) < 5e-4:
                g['r'].append(r); g['a'] += a; break
        else:
            groups.append({'d': d, 'p': p, 'r': [r], 'a': a})
    corners = None
    if box:
        x0, y0, z0, x1, y1, z1 = box
        cs = np.array([[x, y, z] for x in (x0, x1) for y in (y0, y1) for z in (z0, z1)]) @ R.T + T
        corners = [cs.min(0).round(5).tolist(), cs.max(0).round(5).tolist()]
    rows.append({'path': path, 'name': pname, 'box': corners, 'vol': vol, 'cm': (R @ np.array(cm) + T).round(5).tolist(),
                 'axes': [{'d': g['d'].round(5).tolist(), 'p': g['p'].round(5).tolist(), 'r': sorted(set(round(x, 5) for x in g['r'])), 'a': round(g['a'], 6)}
                          for g in sorted(groups, key=lambda g: -g['a'])[:6]]})


free = TDF_LabelSequence(); shapes.GetFreeShapes(free)
for i in range(1, free.Length() + 1):
    walk(free.Value(i), [name_of(free.Value(i))], np.eye(4))
json.dump(rows, open(out, 'w'))
print(len(rows), 'parts')
