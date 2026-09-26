# REBUILT Sim 2026

A single-player, single-match FRC simulator for the 2026 game **REBUILT**, in the spirit of MoSimulator / CloSimulator.
It runs in the browser (Three.js rendering + Rapier physics), works with an Xbox controller, and simulates **all 504 FUEL**.
Play solo, or against an AI opponent robot (PvE), and build your own autos in the Auto Editor.
The AI can also drive your robot, so you can watch AI-vs-AI matches. Its strategy can be trained by self-play, or by playing Training matches against it, where it also learns from how you drive. You can run defense drills against it, and fine-tune and export its values from the AI Tuning screen.

## Run it

Requires Python 3 (already on macOS) and a modern browser (Chrome, Edge or Safari). The 3D and physics libraries load from a CDN, so you need an internet connection the first time.

- **Windows:** install Python 3 from python.org (tick "Add python.exe to PATH"), then double-click `start.bat`.
- **macOS:** double-click `start.command`.
- **Any system:** run `python3 serve.py --open` (Windows: `py serve.py --open`) in this folder and it opens `http://localhost:8765/`.

Keep the server window open while you play; close it to stop.

ES modules won't load from `file://`, so always use `serve.py` instead of opening `index.html` directly.

**Controller:** connect an Xbox controller (USB or Bluetooth) and press any button so the browser detects it. The menus, the match, the Auto Editor and the pause/results screens all work from the controller.

## Controls

| Action | Xbox controller | Keyboard |
|---|---|---|
| Drive (field-relative) | Left stick | W A S D |
| Rotate | Right stick X | Q / E or ← → |
| Shoot: auto-aim, shoot on the move | RT | Space |
| Intake | LT | Shift |
| Pass to your ALLIANCE ZONE corner | RB (hold) | R |
| Outtake / eject | LB | F |
| Human player: open/close CHUTE DOOR | X | G |
| Human player: throw FUEL at the HUB | Y (hold) | H |
| Climb / cancel / lower (climber add-on only) | A | C |
| Climb level up / down | D-pad ↑ / ↓ | ↑ / ↓ |
| Camera (Driver Station, Follow, Chase, Overhead, Broadcast) | D-pad ← / → | [ / ] |
| Field- / robot-relative drive | B | B |
| Slow mode (hold) | Left stick click | X |
| Pause | Menu (☰) | Esc |
| Restart match | Hold View (⧉) for 1 s | Hold Backspace |

Flywheels stay spun up for 1.5 s after you release the trigger, so stop-and-go shooting doesn't spin up from zero every time.

**RT is context-aware:**
- With your BUMPERS in your ALLIANCE ZONE, it targets your HUB.
- Anywhere else it lobs FUEL into the nearest corner of your ALLIANCE ZONE, because scoring from outside is a MAJOR FOUL (G407).

Every robot shoots on the move: the solver leads the target by the robot's velocity, including air drag.

## PvE: AI opponent

Set **Opponent (PvE)** in the main menu to put an AI robot on the other alliance. It can drive any of the three robots (**Opponent robot**), scores into its own HUB, and has its own HUMAN PLAYER, who throws when its HUB is active. Its AUTO FUEL counts toward which HUB goes inactive first.

| Strategy | What it does |
|---|---|
| Scorer | Runs its own cycles. It collects FUEL, and **steals** from your ALLIANCE ZONE when it's worth it (every FUEL taken counts twice: one fewer for you, one more for it), then shoots on the move once back in its zone. **Off shifts:** it has one goal, to get as much FUEL onto its side as possible. It collects in the NEUTRAL ZONE and passes everything into its own zone (turrets pass as they intake; the 2910 passes in batches), never over its HUB. Near the end of the shift it fills its hopper and heads home, then works through that stockpile when its HUB turns on. |
| Defense | Tries to **keep you out of your zone**. It guards the BUMP or TRENCH lane you'd use to get in and **rams** you back when you come close. If you get in, it shoves you off your shot. It backs off 72 in before a PIN becomes a foul, and leaves you alone at your TOWER in END GAME. |
| Hybrid | Shift-aware. It defends during the SHIFTS when only your HUB is active and scores the rest of the time. |

How the Scorer handles time and traffic:
- If its own zone has too little FUEL left to be worth picking through, it goes to the NEUTRAL ZONE.
- It measures its own collection rate. If its HUB's active window won't last long enough to fill up, travel and shoot, it scores the partial load it has. It also heads in with any load just before its HUB activates.
- Turret robots (4414, 8793) don't use a fixed shooting spot. Once inside their zone with the HUB active, they shoot from wherever they are while collecting. Outside the zone they head for the nearest point inside it. The 2910 still drives to a shooting spot, because its whole chassis has to turn to aim.
- While shooting or passing on the move, it drives smoothly: capped speed, limited acceleration and turning. This lets the turret, hood and flywheel settle so the shot actually releases (a shot only fires when aim, flywheel speed and hood are all on target).
- If you block it on the way to its zone, it goes around at first. Once it stops gaining ground (you're mirroring it), it drives straight through you.

**Opponent skill** sets its speed, how carefully it collects, how much of its hopper it uses, its shooting accuracy, how long it hesitates between cycles, its reaction time on defense and its pin discipline:
- **Rookie:** slow, small loads, misses more, stops to shoot, and holds pins too long, so it draws G418 fouls.
- **Regional:** a solid district/regional robot.
- **Champs:** full speed, full hoppers, tight cycles and clean defense, using the hand-tuned strategy.
- **Trained (self-play):** Champs-level driving, using the strategy learned by AI-vs-AI self-play (see below).

In AUTO the AI runs a normal routine. Scorer and Hybrid run their robot's *Best for this robot* AUTO. Defense starts beside its HUB, shoots its preload and drives over its BUMP to wait at mid-field, on its side of the CENTER LINE, ready for TELEOP.

### Watch AI vs AI

Set **Your robot driven by** to one of the AI strategies (and **Your AI skill**) to let the AI drive your robot in TELEOP. Your AUTO routine still runs first.
- Pair it with an opponent to watch AI vs AI.
- Cameras and pause still work.

## Training mode: teach the AI by playing it

Set **Match type** to *Training (AI learns)*. You play against **Your trained AI**, a Champs-level robot whose strategy ("brain") is saved in your browser. It learns two ways:

- **From results.** Each Training match, the AI plays a slightly different version of its brain: variation A, then its mirror image, variation B, in the next match. After each pair it moves toward whichever did better against you. The results screen explains what it tried and which values changed. **Exploration** in AI Tuning sets how different the variations are.
- **From your driving.** In every match you drive, the game measures the same decisions the brain makes: where you shoot from, how full you get before a cycle, how long you collect, how early you get back for your HUB, how fast you drive through FUEL, how far away you drop the intake, whether you shuttle FUEL in your off shifts and how much you keep, how long you hold a pin, how fast you shove or ram, where you block and when you engage. When you out-drive the AI (win the match, or beat its average in a defense drill), it copies part of your style. **Copy my style** in AI Tuning sets how much.

### Your role: Score or Defense

**Your role** can be *Score* (win the match) or *Defense (hold the AI down)*.

In Defense, the opponent always plays Scorer, and your goal is to minimize the points it scores. Fouls you commit count as its points.
- The HUD shows the AI's running score and your target: its average across your earlier drills.
- The results show the points you held it to.
- In Training, the AI learns to score through your defense, rewarded by its own points. When you hold it under its average, it copies your defensive style (pin time, shove speed, block position, engage distance) for when it plays defense itself.

### AI Tuning: see, adjust and export the values

**AI TUNING** in the main menu lists every brain value of Your trained AI. Each value is on a slider (controller or mouse), next to three reference values:
- the hand-tuned (Champs) value;
- the shipped **Trained** value;
- what your driving measured, with its sample count.

It also shows the training record and the learning settings. Buttons:
- Blend toward my driving.
- Reset to the shipped or the hand-tuned brain.
- Clear my driving data, or reset the record.
- Export, copy JSON, and import.

**Exporting into the main version.** When the values are where you want them, press *Export trainedBrain.js*. It downloads a drop-in `js/trainedBrain.js`, plus a `.json` copy. Replace the repo's `js/trainedBrain.js` with the download and commit it. The **Trained** skill level then uses your brain for everyone. You can also:
- keep training it headless: `npm run train -- --from rebuilt-ai-brain.json`;
- import it on another browser.

Pick **Your trained AI** as the opponent skill (or as *Your AI skill* in watch mode) to play or watch it without exploration.

## Self-play training

The AI's strategy lives in a small set of numbers, its "brain". The trainer (`tools/train.mjs`) tunes them by having AI robots play full matches against each other. The brain covers:
- how full to get before a cycle, and how long to collect before scoring anyway;
- when to head back and stage for an active HUB, and where to shoot from;
- how fast to drive through FUEL, and which FUEL to go for;
- how long to hold a pin, how hard to shove and where to block;
- when a Hybrid robot is loaded enough to go defend.

The difficulty handicaps (speed, accuracy, reaction time) are separate, so training improves decisions, not raw ability.

The matches are headless: the same code, field, rules and all 504 FUEL as the browser game, run in Node without rendering. Each match gets a fresh physics world and a seeded random stream, so it plays out identically every time.

How training works:
1. Each generation samples candidate brains around the current one (an evolution strategy with mirrored sampling).
2. Every candidate plays the same seeded scenarios, with mixed strategies, robots and alliances. Its opponents come from a league: the hand-tuned brain plus earlier snapshots of the trained brain.
3. Scores are ranked per scenario, and the brain moves toward the best half.
4. At the end, the trained and hand-tuned brains play the same held-out matches, and the paired difference is reported.
5. The result is saved to `js/trainedBrain.js` (the **Trained** skill level) only if it beats the hand-tuned brain. Each run's history is appended to `tools/training-log.json`.

Requires Node 18+:

```
npm install                                   # three + rapier for Node (the browser still uses the CDN)
npm run train                                 # 10 generations, about 45 min on 4 cores
npm run train -- --gens 30 --pop 12 --scenarios 6 --resume   # longer run, continuing from the last result
npm run train -- --quick                      # smoke test, nothing saved
npm run train -- --from rebuilt-ai-brain.json # continue from a brain exported in AI Tuning
npm run match -- --a hybrid:trained:4414 --b defense:champs:2910   # one headless match
```

Each side of `npm run match` is `strategy:skill:robot`. Training options:

| Option | Meaning |
|---|---|
| `--gens` | Number of generations |
| `--pop` | Candidates per generation |
| `--scenarios` | Matches per candidate per generation |
| `--validate` | Held-out validation matches |
| `--workers` | Parallel processes (default: all cores) |
| `--seed` | Random seed |
| `--sigma` | Initial step size |
| `--resume` | Continue from the last trained brain |
| `--from FILE` | Start from an exported brain (`.json` or `trainedBrain.js`) |
| `--no-save` | Don't write the result |
| `--force` | Save even if validation didn't show an improvement |

A match takes about 30 s of CPU, so more cores train faster.

## The robots

| | 2910 Jack in the Bot "Re•Blitz" | 4414 HighTide "RIPCURRENT" | 8793 Pumpkin Bots |
|---|---|---|---|
| Type | Dumper | Dye Rotor | Hopperless |
| Frame | 27.5 × 27 in swerve | 25 × 32 in swerve | 27.5 × 27.5 in swerve |
| Capacity | 58 FUEL | 88 FUEL (extending hopper) | 12 (only the ball path) |
| Shooter | 4-wide drum, adjustable hood, **fixed to the chassis** (whole robot turns to aim) | Single-stream 3" flywheel on a **turret**, adjustable hood | Hooded flywheel on a **turret** |
| Rate | 32 FUEL/s | 18 FUEL/s | 13 FUEL/s |
| Intake | 26 FUEL/s, slap-down (7.8 in reach) | 30 FUEL/s, sliding box (latched out) | 14 FUEL/s |
| Fits under TRENCH | yes | yes | yes |
| Climber | none | none | none |

Sources:
- 2910: the Re•Blitz tech binder and their public Onshape CAD. The intake you see is their "Pivoting Intake Assembly", and its reach (7.8 in past the BUMPER) comes from that CAD.
- 4414: the 2026 tech binder (2026.team4414.com) and its CAD renders: the sliding box intake on racks, the Dye Rotor, the smoked hopper with its teal truss frame.
- 8793: your team CAD (Intake V3, Conveyor V2, Turret, Shooter).

None of the three climbed, so each defaults to *no climber*. The **Climber add-on** option adds a hypothetical Level 1 or Level 1-3 climber if you want to try the TOWER.

## What's simulated

**Field** (2026 Game Manual section 5, the official AprilTag layout and the official field CAD):
- 651.2 × 317.7 in field.
- HUBS: 47 in, with a 41.7 in hex opening at 72 in and a 58.4 in wide net 10.3 in behind them (from the GE-26300 drawing). FUEL leaves through a 35.6 in wide opening in the NEUTRAL ZONE face, 30.1 in off the carpet, and drops onto the field.
- BUMPS: 73 × 44.4 × 6.5 in with 15° ramps.
- TRENCHES: 22.25 in clearance.
- TOWERS: rungs at 27, 45 and 63 in.
- DEPOTS (24 FUEL each) and OUTPOSTS, with the CHUTE (24 FUEL), CHUTE DOOR and CORRAL. The TOWER, OUTPOST and DEPOT positions match the field CAD: the blue TOWER is centered on AprilTag 31 and the blue OUTPOST on 29, and the red ones mirror them.
- 20 in guardrails and the alliance walls.

**FUEL:**
- 504 balls: 400–408 staged in the NEUTRAL ZONE (34 × 12 grid), 48 in the DEPOTS, 48 in the CHUTES, plus up to 8 preloaded.
- Physics uses the real size and mass, quadratic air drag and carpet rolling resistance.
- FUEL that enters a HUB is processed and comes back out the exits into the NEUTRAL ZONE.
- FUEL that leaves the field is returned by "field staff" near where it left.

**Match:**
- AUTO 0:20, then TELEOP 2:20: TRANSITION SHIFT (10 s), SHIFTS 1–4 (25 s each), END GAME (30 s).
- The alliance that scored more FUEL in AUTO has its HUB inactive in SHIFT 1; status alternates each shift, and a tie is broken randomly.
- FUEL in an inactive HUB scores 0.
- FUEL is still counted for 3 s after a HUB deactivates.
- HUB lights follow Table 5-3: active, 3 s warning pulse, white chase during the TRANSITION SHIFT, off.

**Scoring:**
- 1 point per FUEL in an active HUB.
- TOWER: LEVEL 1 is 15 points in AUTO (assessed at 0:00). In TELEOP, LEVEL 1/2/3 is 10/20/30, assessed at the end, one level per robot.
- MINOR fouls give the opponent 5 points; MAJOR fouls give 15.

**Rules enforced:**
- **G303:** starting position with BUMPERS overlapping the ROBOT STARTING LINE and not touching a BUMP. Preload is limited to 8. Robots start in starting configuration (4414's intake latches down and its hopper extends at the start; 2910's hopper deploys with the intake).
- **G402:** no driver control during AUTO; the robot runs the auto routine you pick.
- **G405** (MINOR): launching FUEL out of the field.
- **G407** (MAJOR): launching FUEL into your HUB from outside your ALLIANCE ZONE.
- **G408** (MINOR): catching FUEL released by the HUB before it touches the carpet.
- **G425 / G427:** the human player only enters FUEL through the CHUTE or by throwing from the OUTPOST AREA, and only stores FUEL in the CHUTE and CORRAL.
- **R105 / R106 / R107:** robots stay under 30 in, extend at most 12 in, and extend in only one direction.
- Robots can't enter the OUTPOST openings.

**Robot-to-robot rules** apply when an opponent is on the field. Both robots are held to them, and foul points go to the other alliance:
- **G403** (MAJOR): in AUTO, contacting an opponent while your BUMPERS are fully across the CENTER LINE.
- **G415** (MINOR): a deployed over-the-bumper intake reaching inside the opponent's FRAME PERIMETER, i.e. hitting them intake-first with the intake down.
- Ramming, even at full speed, is legal. High-speed contact isn't called in competition, and robots here can't tip over.
- **G418** (MINOR): PINNING an opponent against a FIELD element for more than 3 s, plus another MINOR for every further 3 s. The count resets when the robots are 72 in apart. The HUD shows the pin count for either robot.
- **G420** (MAJOR): in END GAME, contacting an opponent that is touching its TOWER or climbing.

Robots push each other with realistic traction (mass × acceleration limit), so heavier or faster-accelerating robots win shoving matches.

**FUEL in the robots** (nothing teleports):
- **Intake:** the rollers grab FUEL (still a physics ball) and drag it up the intake arm, over the BUMPER and in through the slot under the hopper wall, at the robot's intake rate. FUEL the rollers let go of before it's over the BUMPER drops back onto the carpet.
- **Hopper:** held FUEL is simulated in the robot's own frame with a lighter solver: gravity, soft ball-to-ball contact (foam squashes), the walls, floors and internal parts, and the robot's own motion, so the load piles up, slides back when you accelerate and sloshes when you spin. Each robot's mechanisms move it:
  - 2910's powered floor rolls FUEL back to the indexer.
  - 4414's Dye Rotor: the Dolphin Fin sweeps round over a still floor, pushing FUEL to the center column. FUEL climbs a spiral of passive rollers to the feeder wheels and the turret on top. Printed "stadium" pieces funnel FUEL onto the floor, and when it isn't feeding the fin turns slowly backward to agitate the load.
  - 8793's conveyor carries FUEL up to its turret and holds it there.
- **Shooter:** FUEL travels the feed path to the flywheel and leaves from the exit at the same rate (BPS) and with the same shot model as before. A FUEL that reaches the wheels while the shot isn't lined up waits there.

## Auto routines

**Best for this robot** runs the highest-scoring AUTO found for the selected robot, with its own starting position (see BEST_AUTOS in `js/auto.js`). `tools/auto-search.mjs` finds these plans: it plays candidate plans headless against the other robots' best AUTOs on both alliances and keeps whichever scores the most AUTO FUEL. BUMPERS may reach past the CENTER LINE but never fully cross it, since G403 is only a foul for contact made fully across it. The AI runs this AUTO too.

The other routines are mirrored automatically for the red alliance and for left/right starting positions:
- Score preload
- Preload + Depot
- Neutral Zone sweep: out through the TRENCH, back over the BUMP, shooting on the move
- Double sweep
- Preload + Climb L1 (needs the climber add-on)
- Preload + defensive position: shoot the preload, then drive over the BUMP to mid-field to start TELEOP on defense

## Auto Editor

**AUTO EDITOR** in the main menu is a simplified PathPlanner. It shows a top-down view of your half of the field (drawn as blue, with the ALLIANCE WALL on the left).
- Drag the START box along the ROBOT STARTING LINE, and place waypoints for the path.
- For each waypoint, set what happens on the way there: intake on/off, shooting (off, shoot on the move once in the ALLIANCE ZONE, or shoot/pass anywhere) and max speed.
- Also set what happens when the robot arrives: drive through, stop, stop and shoot until empty, or wait 1–3 s. Optionally shoot the preload first.
- The panel shows an estimated run time against the 20 s AUTO. Waypoints past the CENTER LINE turn red as a G403 warning.

Autos save automatically in the browser (localStorage) and appear in the **Auto routine** menu marked with ✎. **Test in a match** starts a match with the auto straight away.
- Red alliance runs the path rotated automatically.
- With a custom auto selected, **Starting position** switches to *As drawn* / *Mirrored left ↔ right*.
- **Mirror left ↔ right** in the editor flips the saved path.

| Editor action | Xbox controller | Mouse / keyboard |
|---|---|---|
| Move cursor | Left stick (hold LS click for fine) | Mouse / W A S D |
| Add waypoint / grab / drop | A | Click (drag to move) / Enter |
| Delete waypoint | X | Right-click / G |
| Previous / next waypoint | LB / RB | F / R |
| Shooting on the way | D-pad ◀ ▶ | [ / ] |
| Speed | D-pad ▲ ▼ | ↑ / ↓ |
| Intake on the way | Right stick click | T |
| At-waypoint action | View (⧉) | Backspace |
| Settings panel (auto list, start, mirror, test…) | Y, then D-pad + A | Click / H |
| Done | B or Menu (☰) | Esc |

## Real CAD

The field you see is FIRST's official field CAD (Onshape "FE-2026: REBUILT Playing Field"), in `cad/field/field.glb`. It's slimmed for the browser: the FUEL, carpet, tape and small hardware are dropped, and it has about 360k triangles in 26 meshes. The bleachers, HUMAN PLAYERS, HUB lights and CHUTE DOORS are still drawn by the game. Physics still uses the colliders built from the dimensions in `js/constants.js`, which line up with the CAD. To go back to the drawn field, set `"enabled": false` on the entry in `cad/manifest.json`.

To refresh the field from Onshape (this needs API keys):
```
tools/onshape-export.sh /tmp/field-raw.glb
npm i --no-save @gltf-transform/core @gltf-transform/extensions @gltf-transform/functions meshoptimizer
node tools/slim-glb.mjs /tmp/field-raw.glb cad/field/field.glb
```

Other CAD, such as robots or field elements, can be added the same way. Export glTF (.glb) from Onshape, or convert STEP with `python3 tools/cad2glb.py in.step out.glb`. Then list the file in `cad/manifest.json` (see `cad/README.md`).

## Project layout

```
index.html, css/style.css   UI shell and styles
js/constants.js             field dimensions, timing, points (from the manual)
js/field.js                 field geometry and colliders
js/fuel.js                  all 504 FUEL: physics, drag, HUB processing, returns
js/ballistics.js            drag-aware trajectory model, shot tables, shoot-on-the-move solver
js/robotConfigs.js          the three robots' specs
js/robotModels.js           3D models
js/robot.js                 swerve drive, intake, storage, turret/chassis aiming, shooter, climber
js/hopper.js                FUEL inside a robot: hopper physics, mechanisms, feed paths
js/match.js                 match timing, HUB shifts, scoring, fouls
js/humanPlayer.js           OUTPOST human player
js/auto.js                  autonomous routines and starting positions
js/customAutos.js           saved custom autos (Auto Editor) -> auto steps
js/editor.js                Auto Editor screen
js/opponent.js              robot AI (Scorer / Defense / Hybrid): skill handicaps + trainable brain
js/trainedBrain.js          shipped "Trained" brain (written by tools/train.mjs or exported from AI Tuning)
js/learning.js              Training mode: learning from matches vs you and from your driving
js/tuning.js, js/brainFile.js   AI Tuning screen; trainedBrain.js export/import format
js/game.js                  one match: robots, autos, AIs, rules (shared by browser and trainer)
js/nav.js                   grid A* path planning around field structures
js/rules.js                 robot-to-robot contact rules (G403, G415, G418, G420)
js/input.js                 Xbox controller (Gamepad API) + keyboard
js/cameras.js, js/ui.js     cameras, menus and HUD
js/main.js                  game loop
serve.py                    local server
tools/train.mjs             self-play trainer (Node); tools/match.mjs runs one headless match
tools/headless.mjs          headless match runner; tools/node-env.mjs + resolve-hook.mjs load the game in Node
tools/auto-search.mjs       finds each robot's best AUTO (tools/auto-eval.mjs, auto-worker.mjs)
tools/bench.mjs, tools/intake-test.mjs   regression checks: AI matches, and a straight-line intake run
start.bat / start.command   double-click launchers (Windows / macOS)
```

To tune a robot, edit `js/robotConfigs.js`: speed, capacity, BPS, hood range, exit speed and accuracy.
