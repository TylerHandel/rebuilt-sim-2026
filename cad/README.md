# Real CAD in the game

Put CAD exports in this folder and list them in `manifest.json`. The game loads them at startup.
Physics always uses the colliders built from the game-manual dimensions in `js/field.js`; CAD is visual.

## Getting the files
- **Onshape** (FIRST field, team robots): right-click the assembly tab → Export → **glTF (.glb)**, or STEP.
- **STEP**: convert here with `python3 tools/cad2glb.py cad/field/field.step cad/field/field.glb`
  (`pip install cadquery-ocp trimesh scipy`; add `--deflection 5` for fewer triangles).
- GitHub rejects files over 100 MB: export elements separately or use Git LFS.

## manifest.json
```json
{
  "models": [
    { "file": "field/field.glb", "kind": "field", "scale": 0.001, "rotationDeg": [-90, 0, 0], "position": [0, 0, 0] }
  ]
}
```
- `kind`: `"field"` replaces the drawn field (carpet kept); `"element"` just adds the model.
- `scale`: 0.001 for millimeters, 0.0254 for inches, 1 for meters.
- `rotationDeg`: converts CAD axes to the game's (Y up); `[-90, 0, 0]` suits Z-up CAD.
- `position`: meters; the game's origin is the field center, blue alliance wall at −X.
- `color` (optional): one color for the whole model if the export has none.
