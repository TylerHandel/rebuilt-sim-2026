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
`robots/2910-intake.glb` is 2910's "Pivoting Intake Assembly" from their public Re•Blitz Onshape document (dfb391aac173a4555d00a5b5). It's re-expressed in the robot model's frame with its origin on the intake pivot. `js/robotModels.js` loads it in the browser and swings it about that pivot; the drawn intake stays until it loads, and headless runs never load it. To rebuild it:
```
DID=dfb391aac173a4555d00a5b5 WID=3dc64f602735252892b0e47b tools/onshape-export.sh /tmp/r2910.glb 6c654da4eb6b1710fb0900bd
node tools/extract-part.mjs /tmp/r2910.glb cad/robots/2910-intake.glb "Pivoting Intake Assembly" -0.273 0.170
node tools/extract-part.mjs /tmp/r2910.glb cad/robots/2910-shooter.glb "32 - R2 Shooter & Feeder" 0 0
node tools/extract-part.mjs /tmp/r2910.glb cad/robots/2910-hopper.glb "62 - R2 Hopper" 0 0
```
`robots/2910-hopper.glb` is their "R2 Hopper": its side, top and front panels sit over the shooter when stowed and slide out along the slotted rails as the intake deploys (0.25 m, `storage.extLen`). The slots allow up to ~0.48 m; the travel isn't in the export.
`robots/2910-shooter.glb` is their "Shooter & Feeder": the powered floor, the roller ramp that indexes FUEL up under the rollered hood, and the drum at the back. It's static (origin at the robot center) and replaces the drawn tower once it loads.


`robots/971-*.glb` and `robots/1678-*.glb` come from those teams' public Onshape CAD through `tools/extract-parts.mjs` and the recipes `robots/971.json` / `robots/1678.json` (sources, which parts go in each file, pivots, what's see-through). 971 (document cabaa0c1c77517916df80783, workspace 48cb057db38a03cc202ff44e, element 96844befd4591dac162d657b): `971-body` (drivetrain, roller floor, separator, kicker, ramps, turret platform, fixed hopper walls), `971-hopper` (the sliding front of the hopper, exported out), `971-intake` (the 4-bar ground intake, exported folded up; origin on the swing pivot), `971-turret` (one shooter, origin on its bearing's axis; the game draws it at both turrets). Its CAD's floor is 0.043 m below its origin, so the parts are lifted that much. 1678 (document acdaaf42764a293a9452326d, workspace f887ef4c6a254c09c73344c3, element a9684fa7665096e977b19c44): `1678-body` (drivetrain and bumper-mounted walls, roller floor, ball tunnel and drum), `1678-intake` (slapdown, exported folded up; origin on its pivot), `1678-slide` (the polycarbonate plates of the horizontal extension), `1678-lift` (climber and lid). The pivots and swing angles were read off the CAD's part positions (the mates aren't in the export). 1690's document allows viewing but not export, so its model is drawn (see `build1690` in `js/robotModels.js`).
