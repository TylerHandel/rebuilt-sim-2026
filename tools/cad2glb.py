#!/usr/bin/env python3
"""Convert STEP CAD to a web-friendly GLB for the game.

    pip install cadquery-ocp trimesh
    python3 tools/cad2glb.py cad/field/field.step cad/field/field.glb [--deflection 2.0]

--deflection is the tessellation tolerance in the STEP file's units (usually mm): bigger =
fewer triangles. Each solid becomes its own mesh node. Then list the .glb in cad/manifest.json
(see cad/README.md). glTF/GLB exported directly from Onshape can skip this step.
"""
import sys
import numpy as np
import trimesh
from OCP.STEPControl import STEPControl_Reader
from OCP.IFSelect import IFSelect_RetDone
from OCP.BRepMesh import BRepMesh_IncrementalMesh
from OCP.TopExp import TopExp_Explorer
from OCP.TopAbs import TopAbs_SOLID, TopAbs_FACE, TopAbs_REVERSED
from OCP.TopoDS import TopoDS
from OCP.BRep import BRep_Tool
from OCP.TopLoc import TopLoc_Location


def solid_mesh(solid):
    verts, faces = [], []
    exp = TopExp_Explorer(solid, TopAbs_FACE)
    while exp.More():
        face = TopoDS.Face(exp.Current())
        loc = TopLoc_Location()
        tri = BRep_Tool.Triangulation_s(face, loc)
        if tri is not None:
            trsf = loc.Transformation()
            base = len(verts)
            for i in range(1, tri.NbNodes() + 1):
                p = tri.Node(i).Transformed(trsf)
                verts.append((p.X(), p.Y(), p.Z()))
            rev = face.Orientation() == TopAbs_REVERSED
            for i in range(1, tri.NbTriangles() + 1):
                a, b, c = tri.Triangle(i).Get()
                faces.append((base + a - 1, base + c - 1, base + b - 1) if rev else (base + a - 1, base + b - 1, base + c - 1))
        exp.Next()
    if not faces:
        return None
    return trimesh.Trimesh(np.array(verts), np.array(faces), process=False)


def main():
    args = [a for a in sys.argv[1:] if not a.startswith('--')]
    if len(args) != 2:
        print(__doc__)
        sys.exit(1)
    src, dst = args
    defl = float(sys.argv[sys.argv.index('--deflection') + 1]) if '--deflection' in sys.argv else 2.0
    reader = STEPControl_Reader()
    if reader.ReadFile(src) != IFSelect_RetDone:
        sys.exit(f'could not read {src}')
    reader.TransferRoots()
    shape = reader.OneShape()
    BRepMesh_IncrementalMesh(shape, defl, False, 0.5, True)
    scene = trimesh.Scene()
    exp = TopExp_Explorer(shape, TopAbs_SOLID)
    n = 0
    while exp.More():
        m = solid_mesh(exp.Current())
        if m is not None:
            m.visual = trimesh.visual.ColorVisuals(m, face_colors=[150, 155, 165, 255])
            scene.add_geometry(m, node_name=f'solid_{n}')
            n += 1
        exp.Next()
    if n == 0:  # surfaces only (no solids): mesh the whole shape
        m = solid_mesh(shape)
        if m is None:
            sys.exit('no geometry found')
        scene.add_geometry(m, node_name='shape')
        n = 1
    scene.export(dst)
    tris = sum(len(g.faces) for g in scene.geometry.values())
    print(f'wrote {dst}: {n} solids, {tris} triangles (deflection {defl}, units as in the STEP file)')


if __name__ == '__main__':
    main()
