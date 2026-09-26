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
- 4414's model follows its tech binder CAD renders (`binder_assets/CadPhotos/*.webp` on 2026.team4414.com; its CAD isn't public): a box intake on racks, the Dye Rotor (a still floor swept by the rotating Dolphin Fin, a center column under the turret, a spiral roller ramp), and smoked walls with the teal truss frame. 2910's intake is their own CAD part (`cad/robots/2910-intake.glb`, made with `tools/extract-part.mjs`), and its reach comes from that CAD. The expanding hopper panels aren't in their CAD export, so they're drawn: they slide 0.2 m out over the deployed intake. The hopper extensions really extend, intake arms reach the rated reach, polycarbonate is tinted, and the HUB lights sit on the real diffusers (`HUB.lightY0/1`).

## Done: best AUTO per robot
- `BEST_AUTOS` in `js/auto.js` holds one plan per robot. Plans are in absolute blue coordinates and consist of trips (neutral-zone sweeps or depot runs). The `best` routine is now the menu default and the AI's AUTO.
- `tools/auto-search.mjs` finds the plans: random plans, then tweaks, each scored with `tools/auto-eval.mjs` against the other robots on both alliances. Rerun it after changing robots or physics.

## Done: tilting robots and real colors
- The robot body is free to pitch and roll (angular damping 1.5). The drive commands only the horizontal velocity and yaw rate, the wheels also collide with FUEL, and the bumper is a rounded cuboid, so robots tilt over BUMPS, DEPOT barriers and FUEL piles. The yaw comes from the full quaternion. The model follows the body's rotation. The hopper solver gets gravity and acceleration in the tilted frame.
- Colors follow the real robots: 2910 in raw aluminum with grey plates and light green drum wheels (their CAD), and 4414 in black and carbon with the teal truss (binder renders). 8793 is unchanged (no source yet). 2910 has clear lids over the fixed hopper and the expanding section.

## Other open items
- The HUB and BUMP sizes were checked against the drawing. All element positions were checked against the field CAD. The TRENCH, TOWER, DEPOT and OUTPOST *sizes* still come from the game manual, so compare them with the drawing pages above or measure them in the CAD. Note that the Block CAD bounding box puts the inside edge of the fixed TRENCH about 8 cm closer to the guardrail than `TRENCH.width` does.
- Optional: a longer AI self-play training run, `npm run train -- --gens 30 --pop 12 --scenarios 6 --resume`.

## Gotchas
- If you use `pkill`, use a bracket pattern such as `pkill -f "trai[n].mjs"` so the command doesn't match and kill its own shell.
- Headless determinism: seed `Math.random` after the world is built. `tools/headless.mjs` already does this.
