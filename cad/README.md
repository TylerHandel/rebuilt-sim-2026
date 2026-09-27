# Real CAD in the game

Put CAD exports in this folder and list them in `manifest.json`. The game loads them at startup.
Physics always uses the colliders built from the game-manual dimensions in `js/field.js`; CAD is visual.

## The field
`field/field.glb` is the official field CAD (Onshape document 8a691e28680da30504859fce, "FE-2026: REBUILT
Playing Field"). Its GLB export is in meters, Z-up, with the blue alliance on +X. The manifest entry's
`rotationDeg: [-90, 0, 180]` maps that to the game's axes: Y-up, blue at −X. Refresh it with
`tools/onshape-export.sh` and then `tools/slim-glb.mjs` (see the main README).

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
- `enabled` (optional): `false` skips the model.
- Drawn meshes flagged `userData.keepWithCad` (bleachers, HUMAN PLAYERS, HUB lights, CHUTE DOORS) stay visible under a `"field"` model.

## Robot parts
`robots/2910-intake.glb` is 2910's "Pivoting Intake Assembly" from their public Re•Blitz Onshape document (dfb391aac173a4555d00a5b5). It's re-expressed in the robot model's frame with its origin on the intake pivot. `js/robotModels.js` loads it in the browser and swings it about that pivot; the drawn intake stays until it loads, and headless runs never load it.

2910's parts come from a STEP export, tessellated by OpenCascade (`tools/step2glb.py`) with the surfaces' own normals, so wheels, rollers and tubes render round. A glTF export from Onshape comes pre-faceted, and the old pipeline then simplified it to 8% and dropped the normals, which is what made round parts look faceted. STEP carries the exact surfaces (not the parametric history, which stays in Onshape), so the tessellation accuracy is ours to pick: 1.2 mm and 0.45 rad here, then a light simplification that leaves hard edges alone. The STEP export flattens the subassembly tree, so `robots/2910.json` picks the parts of each file by matching name and position against the glTF export (`like`), which also says which panels are see-through. To rebuild them (the STEP translation is an async API job; the Onshape export page works too):
```
DID=dfb391aac173a4555d00a5b5 WID=3dc64f602735252892b0e47b tools/onshape-export.sh /tmp/cad/r2910.glb 6c654da4eb6b1710fb0900bd
# STEP: POST /api/v6/assemblies/d/$DID/w/$WID/e/6c654da4eb6b1710fb0900bd/translations {"formatName":"STEP","storeInDocument":false},
# poll /api/v6/translations/<id> until DONE, then GET /api/v6/documents/d/$DID/externaldata/<resultExternalDataIds[0]>
python3 -m venv /tmp/ocp && /tmp/ocp/bin/pip install cadquery-ocp
/tmp/ocp/bin/python tools/step2glb.py /tmp/cad/r2910.step /tmp/cad/r2910-step.glb --lin 0.0012 --ang 0.45 \
  --only "42 - R2 Intake" "32 - R2 Shooter" "62 - R2 Hopper" \
  --skip "screw|bolt|\bnut\b|washer|rivet|spacer|chain|belt|tensioner|bearing|gear|sprocket|kraken|motor|plug|collar|\bpin\b|hub|insert|pulley|retaining|wcp-0982|^9\d{4}a|^fuel\b|shcs|bhcs|fhcs"
node tools/extract-parts.mjs /tmp/cad/r2910-step.glb cad/robots/2910.json   # finds r2910.glb next to it for "like"
```
`robots/2910-hopper.glb` is their "R2 Hopper": its side, top and front panels sit over the shooter when stowed and slide out along the slotted rails as the intake deploys (0.27 m, `storage.extLen`). The slots allow up to ~0.48 m; the travel isn't in the export.
`robots/2910-shooter.glb` is their "Shooter & Feeder": the powered floor, the roller ramp that indexes FUEL up under the rollered hood, and the drum at the back. It's static (origin at the robot center) and replaces the drawn tower once it loads.


`robots/971-*.glb` and `robots/1678-*.glb` come from those teams' public Onshape CAD through `tools/extract-parts.mjs` and the recipes `robots/971.json` / `robots/1678.json` (sources, which parts go in each file, pivots, what's see-through). 971 (document cabaa0c1c77517916df80783, workspace 48cb057db38a03cc202ff44e, element 96844befd4591dac162d657b): `971-body` (drivetrain, roller floor, separator, kicker, ramps, turret platform, fixed hopper walls), `971-hopper` (the sliding front of the hopper, exported out), `971-intake` (the 4-bar ground intake, exported folded up; origin on the swing pivot), `971-turret` (one shooter, origin on its bearing's axis; the game draws it at both turrets). Its CAD's floor is 0.043 m below its origin, so the parts are lifted that much. 1678 (document acdaaf42764a293a9452326d, workspace f887ef4c6a254c09c73344c3, element a9684fa7665096e977b19c44): `1678-body` (drivetrain and bumper-mounted walls, roller floor, ball tunnel and drum), `1678-intake` (slapdown, exported folded up; origin on its pivot), `1678-slide` (the polycarbonate plates of the horizontal extension), `1678-lift` (climber and lid). The pivots and swing angles were read off the CAD's part positions (the mates aren't in the export). 1690's and 4946's documents allow viewing but not export, so their models are drawn (`build1690` and `build4946` in `js/robotModels.js`) from Onshape's shaded views, which the API does serve for view-only documents (`/api/v10/assemblies/.../shadedviews` with `pixelSize` gives scaled orthographic views to measure from). 971's shooter fires out over its flywheel, away from the hood, so the game draws `971-turret.glb` turned half a turn from the shot direction.
