# Handoff

For a new Claude Code session. The latest work is on branch `claude/physical-fuel-handling`, which is built on the field CAD work. Don't open a PR unless asked.

## The project in brief
- An FRC 2026 REBUILT single-player browser simulator built on Three.js and Rapier. The libraries load from a CDN, so there's no build step. Run it with `python3 serve.py --open`.
- The README covers features, controls, the AI, training and tuning.
- Headless AI matches: `npm run match -- --a scorer:trained:2910 --b scorer:champs:4414 --seed 3`. The npm scripts run through `tools/node-env.mjs`.
- Physics runs at 120 Hz. The field colliders are built in `js/field.js` from the dimensions in `js/constants.js`. CAD is visual only and must not change the physics.

## What's done
- Robots (`js/robotModels.js`, `js/robot.js`): wrap-around bumpers, detailed models, and hoppers that really extend.
- FUEL rolling fix: lower friction, and no more forced spin.
- HUB exits and net match drawing GE-26300. FUEL leaves an opening 30.13 in off the carpet and 35.56 in wide in the NEUTRAL ZONE face. The net is 58.41 in wide, runs from 49.75 to 120.36 in high, and sits 10.26 in behind the HUB. See the `HUB` block in `js/constants.js`.
- CAD import path:
  - `js/cadModels.js` loads every GLB listed in `cad/manifest.json` at startup. A `kind: "field"` model hides the drawn field but keeps the carpet.
  - `tools/cad2glb.py` converts STEP to GLB and needs `pip install cadquery-ocp trimesh scipy`.
  - `cad/README.md` documents the manifest format.
- The official field drawing is at https://firstfrc.blob.core.windows.net/frc2026/FieldAssets/2026-field-dwg-game-specific.pdf (193 MB, 196 pages). Pages are 0-based as PyMuPDF counts them:

  | Pages | Element |
  |---|---|
  | 0–24 | OUTPOST |
  | 25–39 | BUMP |
  | 40–84 | TRENCH |
  | 85–138 | HUB (overall dimensions on page 88) |
  | 139–161 | FUEL counter |
  | 162–181 | TOWER |
  | 182–187 | DEPOT |

  Render pages with `pip install pymupdf`.

## Done: real field CAD
- `cad/field/field.glb` is the full official field (element f4e47c668796f504844c94a0 in Onshape document 8a691e28680da30504859fce, workspace c6aa636fb23edb3f1e272fb1). The raw export has 2.8M triangles. `tools/slim-glb.mjs` drops FUEL, carpet, tape and hardware, then simplifies and merges the meshes, which leaves 357k triangles in 26 meshes (12 MB).
- To refresh it, run `tools/onshape-export.sh` (needs `ONSHAPE_ACCESS_KEY` and `ONSHAPE_SECRET_KEY`), then `tools/slim-glb.mjs`. The README's "Real CAD" section has the commands.
- Alignment: the GLB is in meters, Z-up, with blue at +X. The manifest uses `scale: 1`, `rotationDeg: [-90, 0, 180]` and `position: [0, 0, 0]`. The HUB, BUMP, TRENCH and alliance walls land on the colliders to within about 2 cm.
- The CAD showed that the TOWER, OUTPOST and DEPOT were misplaced in `js/constants.js`. The TOWER is centered on AprilTag 31, not between 31 and 32, and the OUTPOST on tag 29, not between 29 and 30. Both were 0.22 m off. The DEPOT center is at fy 5.965. All three are fixed.
- Bleachers, HUMAN PLAYERS, HUB lights and CHUTE DOORS are marked `userData.keepWithCad` and are still drawn on top of the CAD.
- Browser check: the game makes 264 draw calls with the CAD, against 474 with the drawn field, and 580k triangles in view against 204k. There were no console errors.
- Playwright in this container: Chromium doesn't trust the proxy CA, so route `cdn.jsdelivr.net` to `node_modules` (`npm i` installs three and rapier) and stub Google Fonts.

## Done: physical FUEL handling (no teleporting)
- **Intake** (`Robot._intake`, `_guideCaptured`, `_settleCaptured`): a ball in the intake zone is "captured". It stays a Rapier body but only collides with STATIC. Each step its velocity is set to carry it up the arm to just over the bumper, then into the hopper. When it gets there it becomes a hopper ball. The intake rate is still capped by `cfg.intake.rate`. The phase is sticky (`cap.over`); without that, 8793's low hopper entry made balls hover.
- **Hopper** (`js/hopper.js`): held balls are position-based particles in the robot frame. Each robot's `cfg.bay` in `js/robotConfigs.js` gives the box, floor (sloped, with a funnel for 4414), top, chamfer, obstacles and mechanism (`floor` / `rotor` / `belt`) and feed path. `robot.stored` still lists the held balls, so the AI and UI are unchanged.
- **Shooter** (`Robot._startFeed`, `_fire`): the same bps timer starts a ball up the feed path, and it launches from the exit with the old shot model. If the shot isn't `ready` when it arrives, it waits at the wheels.
- **Checks:** the shooting rate matched the old code (32.1 / 17.9 / 13.6 bps vs 32.3 / 18.1 / 13.6), with the same HUB accuracy. So did the intake (same counts, first ball about 0.1 s later). Use `tools/intake-test.mjs` and `tools/bench.mjs`.
- 4414's model follows its tech binder CAD renders (`binder_assets/CadPhotos/*.webp` on 2026.team4414.com; its CAD isn't public, and the binder text describes each mechanism). The intake box slides out on racks through pinion strips on the outside of the walls: a front panel with impact guards, an under-roller, a hinged ramp of passive rollers up to the bumper, and a star roller over it. The Dye Rotor is a pocketed spinning rotor with the Dolphin Fin ramp on its rim, a fixed hook of passive rollers, and omni and feeder wheels at the center column. The smoked walls run the full height (`bay.wallTop` 0.53 m) with the teal truss, and a keyhole top plate runs from the back truss to the turret bearing. The A-frame shooter sits down inside the ring with only its hood roller above the plate, flywheel at the back and FUEL out toward the front. The net covers only the top: in the physics the ceiling over its open part is a dome (`bay.dome`: 0.27 m at the middle, pinned at the walls, the top plate and round the turret) that the load can fill, and it's pressed flat under the TRENCH arm (`Hopper.domeScale` from `Robot._headroom`). In the model the net is a sheet draped over whatever FUEL pokes above the walls. The physics follow: the rotor carries FUEL round (`rotor.drag`), the fin pushes from `finR0` out, the box's ramp is the hopper floor out front (`bay.ramp`), and FUEL crosses the bumper at `intake.lip`. 2910's intake is their own CAD part (`cad/robots/2910-intake.glb`, made with `tools/extract-part.mjs`), and its reach comes from that CAD. Its shooter & feeder and its hopper are their CAD parts too. The hopper's panels sit over the shooter when stowed and slide out 0.25 m along slotted rails (an estimate; the slots allow ~0.48 m). The hopper extensions really extend, intake arms reach the rated reach, polycarbonate is tinted, and the HUB lights sit on the real diffusers (`HUB.lightY0/1`).

## Done: best AUTO per robot
- `BEST_AUTOS` in `js/auto.js` holds one plan per robot. Plans are in absolute blue coordinates and consist of trips (neutral-zone sweeps or depot runs). The `best` routine is now the menu default and the AI's AUTO.
- `tools/auto-search.mjs` finds the plans: random plans, then tweaks, each scored with `tools/auto-eval.mjs` against the other robots on both alliances. Rerun it after changing robots or physics.

## Done: tilting robots and real colors
- The robot body is free to pitch and roll (angular damping 1.5). The drive commands only the horizontal velocity and yaw rate, the wheels also collide with FUEL, and the bumper is a rounded cuboid, so robots tilt over BUMPS, DEPOT barriers and FUEL piles. The yaw comes from the full quaternion. The model follows the body's rotation. The hopper solver gets gravity and acceleration in the tilted frame.
- Colors follow the real robots: 2910 in raw aluminum with grey plates and light green drum wheels (their CAD), and 4414 in black and carbon with the teal truss (binder renders). 8793 is unchanged (no source yet). 2910 has clear lids over the fixed hopper and the expanding section.

## Done: capacity from the model
- Capacity is measured from each hopper's geometry (`measureCapacity` in `js/hopper.js`: pour n FUEL in, settle 2 s, largest n under 10 mm of extra squeeze, bisection, cached). `Robot.geoCap` holds retracted/extended; `capacity()` / `maxCapacity()` use it, and so do the UI robot cards, the AI and the AUTO timeouts. `storage.capacity` / `retracted` in the configs are only the teams' stated numbers now.
- Today: 2910 44 → 63, 4414 57 → 111 (walls to 0.53 m, then the net's dome), 8793 12.
- Captured FUEL enters on top of the pile by the entry (`Hopper.dropHeight`), and is pushed in once that spot is up to the top.

## Done: passes, camera
- Passes don't wait for a clean shot: a pass goes as soon as the flywheel is at 75% and the FUEL, launched from where it really leaves with the flywheel, hood and heading as they are right now (and the robot's own motion), would come down on our half, 1.2 m clear of the walls, without dropping into a HUB (`Robot._passLands`). HUB shots keep the strict spin/hood/aim check.
- Right stick click (T) turns the camera around 180° (`CameraRig.flip`), and field-relative drive turns with it.

## Other open items
- The HUB and BUMP sizes were checked against the drawing. All element positions were checked against the field CAD. The TRENCH, TOWER, DEPOT and OUTPOST *sizes* still come from the game manual, so compare them with the drawing pages above or measure them in the CAD. Note that the Block CAD bounding box puts the inside edge of the fixed TRENCH about 8 cm closer to the guardrail than `TRENCH.width` does.
- Optional: a longer AI self-play training run, `npm run train -- --gens 30 --pop 12 --scenarios 6 --resume`.

## Gotchas
- If you use `pkill`, use a bracket pattern such as `pkill -f "trai[n].mjs"` so the command doesn't match and kill its own shell.
- Headless determinism: seed `Math.random` after the world is built. `tools/headless.mjs` already does this.
