# REBUILT Sim 2026

A single-player, single-match FRC simulator for the 2026 game **REBUILT**, in the spirit of MoSimulator / CloSimulator.
It runs in the browser (Three.js rendering + Rapier physics), works with an Xbox controller, and simulates **all 504 FUEL**.

## Run it

Requires Python 3 (already on macOS) and a modern browser (Chrome, Edge or Safari). The 3D and physics libraries load from a CDN, so you need an internet connection the first time.

- **macOS:** double-click `start.command`, or
- run `python3 serve.py --open` in this folder and it opens `http://localhost:8765/`.

ES modules won't load from `file://`, so always use `serve.py` instead of opening `index.html` directly.

**Controller:** connect an Xbox controller (USB or Bluetooth) and press any button so the browser detects it. The menus, the match and the pause/results screens all work from the controller.

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

## The robots

| | 2910 Jack in the Bot "Re•Blitz" | 4414 HighTide "RIPCURRENT" | 8793 Pumpkin Bots |
|---|---|---|---|
| Type | Dumper | Dye Rotor | Hopperless |
| Frame | 27.5 × 27 in swerve | 25 × 32 in swerve | 27.5 × 27.5 in swerve |
| Capacity | 58 FUEL | 88 FUEL (extending hopper) | 12 (only the ball path) |
| Shooter | 4-wide drum, adjustable hood, **fixed to the chassis** (whole robot turns to aim) | Single-stream 3" flywheel on a **turret**, adjustable hood | Hooded flywheel on a **turret** |
| Rate | 32 FUEL/s | 18 FUEL/s | 13 FUEL/s |
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

Robot-to-robot rules (G403, G415–G420) are left out because there is only one robot.

## Auto routines

Each routine is mirrored automatically for the red alliance and for left/right starting positions:
- Score preload
- Preload + Depot
- Neutral Zone sweep: out through the TRENCH, back over the BUMP, shooting on the move
- Double sweep
- Preload + Climb L1 (needs the climber add-on)

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
js/input.js                 Xbox controller (Gamepad API) + keyboard
js/cameras.js, js/ui.js     cameras, menus and HUD
js/main.js                  game loop
serve.py, start.command     local server / launcher
```

To tune a robot, edit `js/robotConfigs.js`: speed, capacity, BPS, hood range, exit speed and accuracy.
