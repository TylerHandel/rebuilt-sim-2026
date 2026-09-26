# Handoff: pull the real field CAD from Onshape

For a new Claude Code session. Work on branch `main-aogjl4`, and commit and push there. Don't open a PR unless asked.

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

## The task: real field CAD
**Onshape document** 8a691e28680da30504859fce, workspace c6aa636fb23edb3f1e272fb1. It is public, but model exports need API keys.

| Element | Element id |
|---|---|
| Full field ("FE-2026: REBUILT™ Playing Field") | f4e47c668796f504844c94a0 |
| Simplified ("Block CAD") | 5e2b2310531e01f25fd97afd |
| Carpet part studio | 0dd2e12e8cfc4635c3f8fb96 |

**Keys:** the environment should provide `ONSHAPE_ACCESS_KEY` and `ONSHAPE_SECRET_KEY`. Authenticate with HTTP Basic, `-u "$ONSHAPE_ACCESS_KEY:$ONSHAPE_SECRET_KEY"`. Never print the keys, write them to a file, or commit them. If they are missing, tell the user to add them in the environment settings (cloud environment menu → Edit) and start a new session.

**Steps:**
1. Test access: `GET https://cad.onshape.com/api/v6/assemblies/d/<did>/w/<wid>/e/<eid>` should return 200.
2. Export a GLB. Either way below works:
   - Synchronous glTF: `GET /api/v6/assemblies/d/<did>/w/<wid>/e/<eid>/gltf`. If that isn't offered for assemblies, use the translation route instead.
   - Translation: `POST /api/v6/assemblies/d/<did>/w/<wid>/e/<eid>/translations` with body `{"formatName":"GLTF","storeInDocument":false,"resolution":"medium"}`. Poll `GET /api/translations/<id>` until `requestState` is `DONE`, then download `GET /api/documents/d/<did>/externaldata/<resultExternalDataIds[0]>`. You can use `"formatName":"STEP"` and convert with `tools/cad2glb.py` instead.
3. Start with Block CAD, which is lighter. Try the full field only if it stays under about 50 MB. Otherwise export per element (HUB, TOWER, TRENCH, BUMP, DEPOT, OUTPOST) or lower the resolution. GitHub rejects files over 100 MB.
4. Save the result under `cad/field/` and add an entry to `cad/manifest.json` with `kind: "field"`.
5. Line the model up with the game:
   - The origin is the field center.
   - The blue alliance wall is at −X and Y is up.
   - The blue HUB center is at about x = −3.645, z = 0.
   - Set `scale` to 0.001 for meters (m) or 0.0254 for inches, and `rotationDeg` (Z-up needs [-90, 0, 0]). Set `position` so the HUBs, BUMPs and TRENCHes sit on top of the physics colliders.
6. Verify in the browser with Playwright:
   - Use Chromium at `/opt/pw-browsers/chromium-1194/chrome-linux/chrome`.
   - Serve with `python3 serve.py 8765`.
   - Take screenshots and check that the CAD overlays the colliders and that the console shows no errors.
   - For a debug view, set `__sim.rig.mode = 'free'` and move `__sim.camera`.
7. Check that the frame rate stays reasonable. If the mesh is too heavy, decimate it with trimesh or use Block CAD.
8. Commit, push, and update `README.md` (it has a CAD section).

## Other open items
- Only the HUB and BUMP have been checked against the drawing. The TRENCH, TOWER, DEPOT and OUTPOST sizes in `js/constants.js` come from the game manual. Compare them with the drawing pages above.
- Optional: a longer AI self-play training run, `npm run train -- --gens 30 --pop 12 --scenarios 6 --resume`.

## Gotchas
- If you use `pkill`, use a bracket pattern such as `pkill -f "trai[n].mjs"` so the command doesn't match and kill its own shell.
- Headless determinism: seed `Math.random` after the world is built. `tools/headless.mjs` already does this.
