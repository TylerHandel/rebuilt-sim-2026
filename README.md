# REBUILT Sim 2026

A single-player, single-match FRC simulator for the 2026 game **REBUILT**, in the spirit of MoSimulator / CloSimulator.
It runs in the browser (Three.js rendering + Rapier physics), works with an Xbox controller, and simulates **all 504 FUEL**.
Play solo, or against an AI opponent robot (PvE), and build your own autos in the Auto Editor.

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

**RT is context-aware:**
- With your BUMPERS in your ALLIANCE ZONE, it targets your HUB.
- Anywhere else it lobs FUEL into the nearest corner of your ALLIANCE ZONE, because scoring from outside is a MAJOR FOUL (G407).

Every robot shoots on the move: the solver leads the target by the robot's velocity, including air drag.

## PvE: AI opponent

Set **Opponent (PvE)** in the main menu to put an AI robot on the other alliance. It can drive any of the three robots (**Opponent robot**), scores into its own HUB, and has its own HUMAN PLAYER, who throws when its HUB is active. Its AUTO FUEL counts toward which HUB goes inactive first.

| Strategy | What it does |
|---|---|
| Scorer | Runs its own cycles. It collects FUEL (avoiding your ALLIANCE ZONE), stages in its ALLIANCE ZONE while its HUB is inactive, then shoots on the move when the HUB turns active. |
| Defense | Blocks the lane between you and your HUB and pushes you while you shoot. It backs off 72 in before a PIN becomes a foul, keeps its intake from reaching into your frame, and leaves you alone at your TOWER in END GAME. |
| Hybrid | Shift-aware. It defends during the SHIFTS when only your HUB is active and scores the rest of the time. |

**Opponent skill** sets its speed, how carefully it collects, how full it gets before a cycle, how long it hesitates between cycles, its reaction time on defense and its pin discipline:
- **Rookie:** slow, stops to shoot, and holds pins too long, so it draws G418 fouls.
- **Regional:** a solid district/regional robot.
- **Champs:** full speed, tight cycles and clean defense.

In AUTO the AI runs a normal routine: a Neutral Zone sweep for Scorer and Hybrid, or preload only for Defense.

## The robots

| | 2910 Jack in the Bot "Re•Blitz" | 4414 HighTide "RIPCURRENT" | 8793 Pumpkin Bots |
|---|---|---|---|
| Type | Dumper | Dye Rotor | Hopperless |
| Frame | 27.5 × 27 in swerve | 25 × 32 in swerve | 27.5 × 27.5 in swerve |
| Capacity | 58 FUEL | 88 FUEL (extending hopper) | 12 (only the ball path) |
| Shooter | 4-wide drum, adjustable hood, **fixed to the chassis** (whole robot turns to aim) | Single-stream 3" flywheel on a **turret**, adjustable hood | Hooded flywheel on a **turret** |
| Rate | 32 FUEL/s | 18 FUEL/s | 13 FUEL/s |
| Intake | 26 FUEL/s | 30 FUEL/s | 14 FUEL/s |
| Fits under TRENCH | yes | yes | yes |
| Climber | none | none | none |

Sources:
- 2910: the Re•Blitz tech binder.
- 4414: the 2026 tech binder (2026.team4414.com).
- 8793: your team CAD (Intake V3, Conveyor V2, Turret, Shooter).

None of the three climbed, so each defaults to *no climber*. The **Climber add-on** option adds a hypothetical Level 1 or Level 1-3 climber if you want to try the TOWER.

## What's simulated

**Field** (2026 Game Manual section 5 plus the official AprilTag layout):
- 651.2 × 317.7 in field.
- HUBS: 47 in, with a 41.7 in hex opening at 72 in, a net in the back, and 4 exits into the NEUTRAL ZONE.
- BUMPS: 73 × 44.4 × 6.5 in with 15° ramps.
- TRENCHES: 22.25 in clearance.
- TOWERS: rungs at 27, 45 and 63 in.
- DEPOTS (24 FUEL each) and OUTPOSTS, with the CHUTE (24 FUEL), CHUTE DOOR and CORRAL.
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
- **G416** (MAJOR): high-speed ramming (over about 3.3 m/s closing speed, most of it yours), treated as a damage risk. Robots here can't tip over, so G417 never triggers.
- **G418** (MINOR): PINNING an opponent against a FIELD element for more than 3 s, plus another MINOR for every further 3 s. The count resets when the robots are 72 in apart. The HUD shows the pin count for either robot.
- **G420** (MAJOR): in END GAME, contacting an opponent that is touching its TOWER or climbing.

Robots push each other with realistic traction (mass × acceleration limit), so heavier or faster-accelerating robots win shoving matches.

## Auto routines

Each routine is mirrored automatically for the red alliance and for left/right starting positions:
- Score preload
- Preload + Depot
- Neutral Zone sweep: out through the TRENCH, back over the BUMP, shooting on the move
- Double sweep
- Preload + Climb L1 (needs the climber add-on)

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
js/match.js                 match timing, HUB shifts, scoring, fouls
js/humanPlayer.js           OUTPOST human player
js/auto.js                  autonomous routines and starting positions
js/customAutos.js           saved custom autos (Auto Editor) -> auto steps
js/editor.js                Auto Editor screen
js/opponent.js              AI opponent (Scorer / Defense / Hybrid)
js/nav.js                   grid A* path planning around field structures
js/rules.js                 robot-to-robot contact rules (G403, G415, G416, G418, G420)
js/input.js                 Xbox controller (Gamepad API) + keyboard
js/cameras.js, js/ui.js     cameras, menus and HUD
js/main.js                  game loop
serve.py                    local server
start.bat / start.command   double-click launchers (Windows / macOS)
```

To tune a robot, edit `js/robotConfigs.js`: speed, capacity, BPS, hood range, exit speed and accuracy.
