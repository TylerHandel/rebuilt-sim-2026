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


`robots/971-*.glb` and `robots/1678-*.glb` come from those teams' public Onshape CAD through `tools/extract-parts.mjs` and the recipes `robots/971.json` / `robots/1678.json` (sources, which parts go in each file, pivots, what's see-through). 971 (document cabaa0c1c77517916df80783, workspace 48cb057db38a03cc202ff44e, element 96844befd4591dac162d657b): `971-body` (drivetrain, roller floor, separator, kicker, ramps, turret platform, fixed hopper walls), `971-hopper` (the sliding front of the hopper, exported out), `971-intake` (the 4-bar ground intake, exported folded up; origin on the swing pivot), `971-turret` (one shooter, origin on its bearing's axis; the game draws it at both turrets). Its CAD's floor is 0.043 m below its origin, so the parts are lifted that much. 1678 (document acdaaf42764a293a9452326d, workspace f887ef4c6a254c09c73344c3, element a9684fa7665096e977b19c44): `1678-body` (drivetrain and bumper-mounted walls, roller floor, ball tunnel and drum), `1678-intake` (slapdown, exported folded up; origin on its pivot), `1678-slide` (the horizontal extension: polycarbonate sides, corrugated front panel and its corner posts), `1678-lift` (climber and lid). The pivots and swing angles were read off the CAD's part positions (the mates aren't in the export). 1690's and 4946's documents allow viewing but not export, so their models are drawn (`build1690` and `build4946` in `js/robotModels.js`) from Onshape's shaded views, which the API does serve for view-only documents (`/api/v10/assemblies/.../shadedviews` with `pixelSize` gives scaled orthographic views to measure from). 971's shooter fires out over its flywheel, away from the hood, so the game draws `971-turret.glb` turned half a turn from the shot direction.

`robots/8793-*.glb` come from 8793's Onshape CAD ("8793-2026-A-0000 Robot", document e34c8122db6474f80cc7e909, workspace dd037c647c748146eb3a3f31, element 56e3b8b78eff91db02fc8880) the same way as 2910's: a STEP export tessellated by `tools/step2glb.py` (1.2 mm, 0.45 rad, hardware skipped), then `tools/extract-parts.mjs` with `robots/8793.json`, matching parts against the glTF export (the STEP places a few parts wrong). `8793-body` (drivetrain without the swerve modules and bumpers the game draws, Conveyor V2, the fixed turret tower; the PDH's 400 bodies are left out), `8793-mount` (Intake V3's fixed side plates and motors), `8793-intake` (the arm, exported down on the carpet; origin on its pivot, 0.292 m forward and 0.336 m up; it swings up and back 145° to stow, `intake.fold`), `8793-turret` (turntable gear, cable carrier and the shooter; origin on the turntable axis, 0.127 m behind center).

`robots/3928-*.glb`, `robots/341-*.glb` and `robots/4930-*.glb` come from those teams' public Onshape CAD the same way (STEP through `tools/step2glb.py`, then `tools/extract-parts.mjs` with `robots/<team>.json`, matching parts against the glTF export). Hardware, 3D-print adapters and grommets are left out, and the bodies are simplified harder (ratio 0.06). 3928 (document 57c9f32575b97450a6090cdb, workspace 05c87b094e9a536a82f6159e, element 475266d70d4d89294ebea1cd): `3928-body` (drivetrain, walls, spindexer ring and wedges, the tower), `3928-spin` (the spoked wheel and cone; origin on its axis, 0.1125 m forward of center), `3928-intake` (the intake box, exported slid 0.3 m out), `3928-turret` (the shooter, its bearing plate and the curved guard; origin on the turret axis, 0.154 m back and 0.0985 m to the side). 341 (document 412a0db47e00e2399ebca57e, workspace b607133da9f21b6baa148170, element e3c2dacd85e82dde2bb28dd9): `341-body` (drivetrain, serializer, table and uptake, L1 climber), `341-intake` (the slapdown, exported down; origin on its pivot, 0.2545 m forward and 0.3365 m up), `341-turret` (everything on the turret; origin on the 92T gear's axis; as exported it points 150° round from straight ahead, `TURRET_341_YAW`). 4930 (document 83c8fa924839349576d211d0, workspace 9af803d0057b2f9fe7042ded, element 208bd6bbadd59d7f2b97c01f): `4930-body` (drivetrain, belt floor, updexer and the three shooters) and `4930-intake` (exported down; origin on its pivot). 4930's CAD isn't centered on its origin (the frame's middle is at (-0.381, 0.3555) and the carpet at z = -0.145), so its recipe's origins move it there. Its STEP export also carries some placeholder solids (a box and some tubes) that the glTF match drops, and its belts are kept (they are the hopper floor).

`robots/1706-*.glb` come from 1706's public Onshape CAD ("Mirage Public Release", document 23ba2ed5b3893ab97929c8b6, workspace d7012e1e8978e09c9756ee4c, element 70526e43c92b434a946b53a6) the same way, with `robots/1706.json`. Its intake end is the CAD's -x, so the recipe uses `"front": "-x"` (added to `tools/extract-parts.mjs`: model = (-x, z, y)). Parts: `1706-body` (drivetrain, spindexer floor, the outer hopper walls, turret rings), `1706-mount` (the intake's fixed frame plates), `1706-hopper` (the hopper box, exported in; it slides out 0.15 m), `1706-intake` (the arm, exported folded up inside the hopper; origin on its pivot, 0.149 m forward and 0.127 m up; it swings 104° out to deploy, `INTAKE_1706`), `1706-spin-a` / `-b` (the spindexer discs; origins on their axes) and `1706-turret-a` / `-b` (the two shooters; origins on their turret axes; both point 27.8° off straight ahead as exported, `TURRET_1706_YAW`).

1678's parts were re-exported the 2910 way (STEP through `tools/step2glb.py`, parts matched against the glTF export, round parts smooth); `robots/1678.json` has the recipe. The extension's front panel and corner posts (`part 26`, `Part 41`, `part 73`, in the intake assembly in their CAD) are in `1678-slide` now; they used to be in `1678-intake` and swung down into the floor with it.
