# REBUILT Sim 2026

A single-player, single-match FRC simulator for the 2026 game **REBUILT**, in the spirit of MoSimulator / CloSimulator.
It runs in the browser (Three.js rendering + Rapier physics), works with an Xbox controller, and simulates **all 504 FUEL**.
Play solo, or against an AI opponent robot (PvE), and build your own autos in the Auto Editor.
The AI can also drive your robot, so you can watch AI-vs-AI matches. Its strategy can be trained by self-play, or by playing Training matches against it, where it also learns from how you drive. You can run defense drills against it, and fine-tune and export its values from the AI Tuning screen.

**Play it in your browser: https://tylerhandel.github.io/rebuilt-sim-2026/** (nothing to download or install; any recent Chrome, Edge, Firefox or Safari).

## Run it

The easiest way is the link above. Every push to `main` publishes the game there (`.github/workflows/pages.yml`; one-time setup: the repository's Settings → Pages → Source: "GitHub Actions").

To run it from a copy on your computer instead (to work on it, or offline once the libraries are cached), you need Python 3 (already on macOS) and a modern browser (Chrome, Edge or Safari). The 3D and physics libraries load from a CDN, so you need an internet connection the first time.

- **Windows:** install Python 3 from python.org (tick "Add python.exe to PATH"), then double-click `start.bat`.
- **macOS:** double-click `start.command`.
- **Any system:** run `python3 serve.py --open` (Windows: `py serve.py --open`) in this folder and it opens `http://localhost:8765/`.

Keep the server window open while you play; close it to stop.

ES modules won't load from `file://`, so always use `serve.py` instead of opening `index.html` directly.

**Controller:** connect an Xbox controller (USB or Bluetooth) and press any button so the browser detects it. The menus, the match, the Auto Editor and the pause/results screens all work from the controller.

## Cover art

**COVER RENDER** on the home screen (or add `?cover` to the address) opens the cover renderer.
It plays a 3v3 up to TELEOP, lines three robots up in the blue ALLIANCE ZONE with full hoppers, and has them shoot into the HUB. Then it renders that moment on your computer's GPU, a few seconds on a decent graphics card.
The render uses nicer lighting than the game (reflections, arena spotlights), smooth edges, motion blur on the flying FUEL only, and the title.
- **Export:** **Download PNG** (1920×1080, or 3840×2160 under Output), **GitHub preview 1280×640** (for the repository's social preview image), or **Copy** to the clipboard.
- **Shot:** camera angle, height and zoom; motion blur (a shutter from off to 1/20 s; the default 1/125 s is a slight streak); and the moment (how long they've been shooting).
- **Robots:** any of the eleven robots.
- **Title:** the title text, or no title.
- Camera, blur and title changes re-render the same moment. Changing the robots or the moment replays the match.
- Your settings are remembered in the browser; **Reset settings** goes back to the defaults.

## Controls

| Action | Xbox controller | Keyboard |
|---|---|---|
| Drive (field-relative: up is away from the camera, toward your robot, whichever camera you use) | Left stick | W A S D |
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
| Turn the camera around 180° (the same shot from the other side of your robot; drive turns with it) | Right stick click | T |
| Field- / robot-relative drive | B | B |
| Slow mode (hold) | Left stick click | X |
| Pause | Menu (☰) | Esc |
| Restart match | Hold View (⧉) for 1 s | Hold Backspace |

Flywheels stay spun up for 1.5 s after you release the trigger, so stop-and-go shooting doesn't spin up from zero every time.

**RT is context-aware:**
- With your BUMPERS in your ALLIANCE ZONE, it targets your HUB.
- Anywhere else it lobs FUEL into the nearest corner of your ALLIANCE ZONE, because scoring from outside is a MAJOR FOUL (G407).
- Passes don't wait for a perfect shot: they go as soon as the FUEL would come down on your half of the FIELD, clear of the walls and the HUBS.

Every robot shoots on the move: the solver leads the target by the robot's velocity, including air drag.

## The menu

The home screen has three ways to play, plus the tools:

| | |
|---|---|
| **Practice** | Just your robot, the FUEL and the clock. Pick a robot, its auto, start and preload. |
| **1 v 1** | You against one AI robot on the other ALLIANCE: its robot, strategy and skill. **More options** holds the driver station, Training mode, the defense drill and watch mode. |
| **3 v 3** | Two full ALLIANCES. Each of the six slots (Blue 1–3, Red 1–3) takes a robot (◀ ▶ on the slot), and **Edit** (A) sets who drives it (**You**, **AI Scorer**, **AI Defense**, **AI Hybrid** or **Empty**), its auto, its start and its AI skill. One slot can be you; with none, you watch the AIs play. |
| Auto Editor · AI Tuning · Controls · Settings | Settings has your human player, the climber add-on, the starting camera, the shot preview line and the sound level. |

**Sound:** every motor on each robot is heard: the drive whines with speed, the shooter flywheel gets louder and higher as it spins up, and the intake rollers, indexer, intake pivot, hopper extension and turret run when they move (louder and lower while they strain against FUEL, e.g. rollers stalled on a full hopper or a folding intake packing the load); FUEL thumps and bounces, bumpers hit, the field plays its cues (match start, the TELEOP bells, the endgame whistle with 30 s left, the final buzzer), and the crowd in the stands cheers when a volley of FUEL goes in. It's positional (louder near the camera, left and right follow the view). Browsers only start sound after a click, key or button press. The recordings are CC0 from freesound.org ([sounds/CREDITS.md](sounds/CREDITS.md)). To hear the real FRC field cues instead, use **Settings → Load real match sounds…** and pick your own copies (the official ones are installed with the FRC Game Tools / Driver Station: search its install folder, usually `C:\Program Files (x86)\FRC Driver Station`, for the `.wav` files). They're matched by file name (start / teleop / endgame or warning / end or buzzer), stay in your browser only, and **Use built-in sounds** switches back.

Every page works from the controller: D-pad / stick to move, ◀ ▶ to change, A to select, B to go back (Esc on the keyboard). **Menu (☰)** opens the highlighted mode, or starts the match from its page.

In 3 v 3, robots on an ALLIANCE never start on the same spot: each gets the one it asks for if it's free, a *Best for this robot* auto runs mirrored to the other side if only that side is free, and otherwise it takes the nearest free spot. The AIs play as a team: each one watches every robot on the field, drives around teammates as well as opponents, leaves FUEL a teammate is already near, and a defender picks the opposing robot that's the biggest threat (most FUEL, closest to scoring) and sticks with it. The HUD lists every other robot (what it's doing and how much FUEL it holds), and the results show each robot's launched, passed and intaked FUEL.

## PvE: AI opponent

In **1 v 1** an AI robot plays the other ALLIANCE (in **3 v 3**, up to five of them). It can drive any of the eleven robots, scores into its own HUB, and has its own HUMAN PLAYER, who throws when its HUB is active. Its AUTO FUEL counts toward which HUB goes inactive first.

| Strategy | What it does |
|---|---|
| Scorer | Runs its own cycles. It collects FUEL, and **steals** from your ALLIANCE ZONE when it's worth it (every FUEL taken counts twice: one fewer for you, one more for it), then shoots on the move once back in its zone. **Off shifts:** it has one goal, to get as much FUEL onto its side as possible. It collects in the NEUTRAL ZONE and passes everything into its own zone, never over its HUB. It passes in big batches (it collects a load, spins the flywheel up as the load nears the batch size, then dumps it all at once), so a drum robot turns once and any shooter spins up once per batch instead of once per FUEL. Near the end of the shift it fills its hopper and heads home, then works through that stockpile when its HUB turns on. |
| Defense | Tries to **keep you out of your zone**. It guards the BUMP or TRENCH lane you'd use to get in and **rams** you back when you come close. If you get in, it shoves you off your shot. It backs off 72 in before a PIN becomes a foul, and leaves you alone at your TOWER in END GAME. |
| Hybrid | Shift-aware. It defends during the SHIFTS when only your HUB is active and scores the rest of the time. |

How the Scorer collects:
- It goes where it gets the most FUEL per second of driving. A target counts the FUEL around it (up to what still fits), so a big pile a few meters away beats picking through scraps, like the piles that collect in front of each HUB's exits. A trip out of its zone also pays for the drive back. FUEL it can shoot while picking it up (in its zone, HUB active) counts extra, and so does FUEL stolen from your zone. FUEL behind it costs the turn.
- A turret robot collecting in its zone while shooting stays there only while that's still the best place to collect.
- Its intake goes down on the way to the FUEL and stays down while it collects (and on the way home, except the robots whose intakes fold in to compact their load: 2910, 971, 1678, 4930 and 1706). It comes up only when the hopper is full, when an opponent is about to be inside its reach (allowing for how fast they're closing and how long the intake takes to come up; a latched intake slows the robot instead), or for the moment a FUEL just released by a HUB would drop into it (G408).
- It fills its hopper all the way before a trip: it's full when the intake stalls against the load. (Rookie and Regional make smaller trips: Cycle fill.)

How the Scorer handles time and traffic:
- It heads in with less than a full hopper only when time runs short. While its HUB is active, it keeps collecting until the last moment it can still get back and empty the hopper before the HUB turns off: every FUEL it picks up until then is one more to score. While its HUB is inactive, it heads in early enough to be staged when the HUB turns on. And at the end of the match it brings in whatever it has.
- Turret robots (4414, 8793, 971, 1690, 4946) don't use a fixed shooting spot. Once inside their zone with the HUB active, they shoot from wherever they are while collecting. Outside the zone they head for the nearest point inside it. 2910 and 1678 still drive to a shooting spot, because the whole chassis has to turn to aim; they turn to face their shot over the last few meters, so they arrive aimed.
- While shooting or passing on the move, it drives smoothly: capped speed, limited acceleration and turning (only while it has FUEL to shoot). This lets the turret, hood and flywheel settle so the shot actually releases (a shot only fires when aim, flywheel speed and hood are all on target).
- If you block it on the way to its zone, it goes around at first. Once it stops gaining ground (you're mirroring it), it drives straight through you.

**Skill** sets its speed, how carefully it collects, how much of its hopper it uses, its shooting accuracy, how long it hesitates between cycles, its reaction time on defense and its pin discipline:
- **Rookie:** slow, small loads, misses more, stops to shoot, and holds pins too long, so it draws G418 fouls.
- **Regional:** a solid district/regional robot.
- **Champs:** full speed, full hoppers, tight cycles and clean defense, using the hand-tuned strategy.
- **Trained (self-play):** Champs-level driving, using the strategy learned by AI-vs-AI self-play (see below).

In AUTO the AI runs a normal routine. Scorer and Hybrid run their robot's *Best for this robot* AUTO. Defense starts beside its HUB, shoots its preload and drives over its BUMP to wait at mid-field, on its side of the CENTER LINE, ready for TELEOP.

### Watch AI vs AI

In 1 v 1, set **Your robot driven by** (under More options) to one of the AI strategies (and **Its AI skill**) to let the AI drive your robot in TELEOP. Your AUTO routine still runs first. In 3 v 3, set every slot to an AI to watch two full alliances. Cameras and pause still work.

## Training mode: teach the AI by playing it

In 1 v 1, set **Match type** (under More options) to *Training (AI learns)*. You play against **Your trained AI**, a Champs-level robot whose strategy ("brain") is saved in your browser. It learns two ways:

- **From results.** Each Training match, the AI plays a slightly different version of its brain: variation A, then its mirror image, variation B, in the next match. After each pair it moves toward whichever did better against you. The results screen explains what it tried and which values changed. **Exploration** in AI Tuning sets how different the variations are.
- **From your driving.** In every match you drive, the game measures the same decisions the brain makes: where you shoot from, how full you get before a cycle, how long you collect, how early you get back for your HUB, how fast you drive through FUEL, how far away you drop the intake, whether you shuttle FUEL in your off shifts and how much you keep, how long you hold a pin, how fast you shove or ram, where you block and when you engage. When you out-drive the AI (win the match, or beat its average in a defense drill), it copies part of your style. **Copy my style** in AI Tuning sets how much.

### Your role: Score or Defense

**Your role** (1 v 1, under More options) can be *Score* (win the match) or *Defense drill*.

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

## Neural-net driver (end-to-end)

The self-play trainer above tunes the scripted AI's strategy values. The neural-net driver takes a different approach: a neural network drives the robot directly, like you with a controller. Ten times a second it chooses the movement, rotation, intake, shoot, pass and outtake, for the whole match.

- **Setup (Windows):** run `nn-setup.bat` once. It installs PyTorch for your NVIDIA GPU.
- **Train fast on the GPU:** run `nn-train-gpu.bat`. It plays thousands of simplified matches at once on your graphics card, each decision replayed as a CUDA graph. Ctrl+C saves; `--resume` continues; `--compile` is faster still.
- **Keep training a network you already have:** double-click `nn-continue-gpu.bat` (GPU) or `nn-continue-champs.bat` (full game against the Champs AIs).
- **The strongest recipe:** double-click `nn-train-pro.bat`. It grows a bigger network (1024 × 512) in its own run (`runs\pro`): the new network first learns from your trained one (or from the scripted robot if you don't have one yet), on its own matches, then trains past it in self-play. The trainer's critic sees the whole match and learns normalized returns, and the older versions it keeps playing are the ones that still give it trouble. See `tools/nn/README.md`.
- **Fine-tune in the real game:** run `nn-train.bat --resume --level 3`. It uses every CPU thread for the full simulation and the GPU for learning.
- **Watch progress:** `nn-dashboard.bat` opens a live chart page.
- **Play with it:** in the menu set **Your robot driven by → Neural net (watch)**, or **Opponent → Neural net**. In a 3v3, set any slot to **Neural net**.
- **Watch it learn:** the dashboard shows a live bird's-eye grid of the matches the GPU trainer is playing. It also shows a real-game scoreboard: while the GPU trains, the CPU plays the latest network in full-game matches against the Champs AIs.
- **New robots and old networks:** what the network sees includes which robot each one is. The 11 robots it first learned keep their places; robots added since (581 on) are recognized in a block appended at the end (observation v4, room for 8 more without changing it again). A network trained before plays exactly as it did, and the trainers grow it to the new inputs (with zero weights) when you resume.
- **Share your network:** double-click `nn-share.bat`. Pick a training run, give it a name and a note, then drag the file it prepares into the GitHub page it opens and click "Commit changes". A couple of minutes later anyone on the website can pick it under **Neural net**: to drive their robot (watch), as their 1v1 opponent, or in any 3v3 slot.
- **Overnight:** double-click `nn-overnight.bat`: GPU training, learning from the pre-programmed AIs recorded in the background, and the real-game scoreboard, all at once.
- **Learn from the pre-programmed AIs:** `nn-record-ai.bat` records the Champs AIs playing; `nn-train-from-ai.bat` trains a new network that first copies them, then improves on its own.
- **Start from your own driving:** every full match you drive is recorded for it. Download the recordings from **AI TUNING** and train with `nn-train.bat --bc recordings`.

Details, hardware use and expected training times: [tools/nn/README.md](tools/nn/README.md).

**Updating without losing anything:** double-click **`update.bat`**. It downloads the newest version into the same folder and replaces only the code. Your training (`runs`), the trained network (`js/nn/driver.json`), recordings, and the Python/Node setup stay as they are. Stop the trainer first. If you have training in an older separate copy, double-click **`import-old.bat`** in the new copy and drag the old folder into the window: it copies all of that across.

## The robots

| Robot | Type | Frame | Capacity (modeled hopper: in → out) | Shooter | Rate | Climb |
|---|---|---|---|---|---|---|
| **2910** Jack in the Bot "Re•Blitz" | Dumper | 27.5 × 27 in | 48 → 73 (stated 40 → 58) | 4-wide drum, adjustable hood, **fixed to the chassis** (fires out the back; the robot turns to aim) | 32 FUEL/s | – |
| **4414** HighTide "RIPCURRENT" | Dye Rotor | 25 × 32 in | 66 → 101, the net stretches (stated 58 → 88) | single-stream 3" flywheel on a **turret** | 18 FUEL/s | – |
| **8793** Pumpkin Bots | Hopperless | 27.5 × 27.5 in | 10 → 16 (the ball path, 4 wide down to single file, and the deployed intake) | hooded 4" flywheel on a **turret** | 10 FUEL/s | – |
| **971** Spartan Robotics "Mixtape" | Twin turrets | 24.5 × 29.5 in | 33 → 66 under its net (the intake folds up against the load to compact it) | **two** independent shooters on turrets (4" flywheels, lead-screw hoods, FUEL out over the flywheel). Each turret's ~210° of travel points back and out to its own side, so together they cover everything but straight ahead; the chassis turns when no turret can reach | 20 FUEL/s (both together) | Level 1 |
| **1678** Citrus Circuits "Limestone" | Drum + lift | 27 × 27 in | 37 → 102 with the extension out, the lid lifted and the net bulging (the intake folds up against the load to compact it) | full-width 3.5" drum with three hood rollers, articulating hood, **fixed to the chassis** (fires out the back) | 26 FUEL/s | Level 1 |
| **1690** Orbit "Kepler" | Compact turret | 25 × 29 in | 27 → 52 under its net (stated 50-55 open-topped) | compact gear-driven **turret** on an 8" bearing, shoots on the move while intaking | 12 FUEL/s | – |
| **4946** The Alpha Dogs "Moto Moto" | Round Dye Rotor | 32.75 × 30.3 in, round (a half circle with a 30 in flat front) | 80 → 108 | **turret** on the center of rotation, above a Dye Rotor tray | 20 FUEL/s | – |
| **3928** Team Neutrino | Spindexer tower | 27 × 27 in | 83 → 137 under its net (walls to 29 in, and the intake box) | **turret** on top of a tower in the back corner, fed by a 5-spoke spindexer | 12 FUEL/s (the spindexer delivers ~8.5) | – |
| **341** Miss Daisy XXIV | Serializer | 27 × 27 in | 7 → 14 (one layer under the turret table, and the deployed slapdown) | hooded shooter on a **turret** on the table | 12 FUEL/s | Level 1 |
| **4930** Electric Mayhem "Floyd 2" | Triple shooter | 26.5 × 28.5 in | 31 → 58 (the intake expands the hopper, under a stretchy diagonal net and a net over the top; folding the intake in compacts the load) | **three** hooded flywheels on one shaft, **fixed to the chassis**, firing **forward** over the intake | 18 FUEL/s (3 lanes) | – |
| **1706** Ratchet Rockers "Mirage" | Twin spindexers | 25.2 × 29.5 in | 42 → 78 under its net (the intake pushes the hopper box 0.3 m out; folding it in compacts the load) | **two** turrets, each fed by its own spindexer, each ranging round its own side | 24 FUEL/s (both together) | – |
| **581** Blazing Bulldogs "Rubble" | Dumper | 27 × 28.5 in | 46 → 75 under its net (the linear intake carries the front of the hopper out with it; the roller floor pivots up to compact) | 4" drum with a 35° adjustable hood, **fixed to the chassis** (fires out the back) | 23 FUEL/s (Champs; their alpha bot did 15) | – |
| **1114** Simbotics "Simbot Tim" | Box dumper | 24.5 × 30 in | 47 → 99 (the tall clear box slides out over the intake, under its red "magic blanket") | hooded flywheel at the top of the tower, **fixed to the chassis**, firing **forward** over the hopper | 26 FUEL/s (est.) | – |

4946 (29.5 in tall), 3928 (29.3 in), 1114 (27.6 in) and 4930 (25.8 in) don't fit under the TRENCH: they go over the BUMPS, and the AI and the AUTOs route them that way. The rest fit. 8793 and 341 fit only with their intakes down: stowed, 8793's arm folds up and back and stands 0.69 m tall, and 341's slapdown stands up to 0.72 m, so the AI and the AUTOs lower them on the way under (driving it yourself, deploy it first). Capacity is what the modeled hopper holds (see *FUEL in the robots*). What's too tall meets the TRENCH arm where it really is. 1678's raised lid and 8793's stowed intake are rigid and stop the robot. A load bulging 4414's, 971's or 1690's net is FUEL under netting, and FUEL is soft: the arm presses the FUEL it's over down into the load and back on the robot as hard as the squashed FUEL pushes (it firms up the more it's squashed), plus the net's friction on the arm, so pushed hard enough the load squashes down and goes under (at full drive, all three do when full, slowed by the arm). The AI doesn't try: it takes the BUMP when it's too tall (and stops taking FUEL in near a TRENCH when it's about to be), and an AUTO whose route goes back under a TRENCH keeps its load low enough, or stops and spits FUEL out until it fits. Driving it yourself, you can force it.

Sources:
- 2910: the Re•Blitz tech binder and their public Onshape CAD. The intake, shooter and hopper you see are their "Pivoting Intake Assembly", "Shooter & Feeder" and "R2 Hopper" (its panels slide out along slotted rails as the intake deploys): a roller ramp indexes FUEL up under the rollered hood to the drum, which fires out the back. The intake's reach (7.8 in past the BUMPER) comes from that CAD. Colors follow the CAD: raw aluminum, grey plates, light green drum wheels, and clear lids over the fixed and expanding hopper.
- 4414: the 2026 tech binder (2026.team4414.com) and its CAD renders: the sliding box intake on racks (front panel, under-roller, ramp, star roller, impact guards, pinion strips), the Dye Rotor (pocketed spinning rotor with the Dolphin Fin, hook, feeder wheels, center column), the full-height smoked hopper walls with the teal truss and the keyhole top plate that carries the turret bearing, the A-frame turret shooter sitting down inside its ring (flywheel at the back, FUEL out past the hood roller at the front), and the stretchy net over the top: past the stated 88 the load bulges it up, to about 120 FUEL (full, it's too tall for the TRENCH).
- 8793: the team's Onshape CAD ("8793-2026-A-0000 Robot"). The model is their CAD: the drivetrain, Intake V3 (three silicone rollers on an arm that swings down from a pivot over the front of the frame and folds up and back to stow), Conveyor V2 (overhead wheels with omni wheels on the sides over a polycarbonate floor), the turret indexer (wall plates and J-shaped roller rails under the 200T turntable) and the shooter (4" SDS flywheels under a pivoting hood). The ball path follows it (see *FUEL in the robots*).
- 971: their public Onshape CAD ("2026 971 Robot Mixtape Public Release") and technical documentation. The model is their CAD: the drivetrain, roller floor, powered omni-wheel separator that splits FUEL into two streams, kicker, the two ramps up into the turrets, the turret platform with its two 10" bearings, and the polycarbonate hopper, whose front slides out when the 4-bar ground intake deploys (the intake folds up inside the hopper, as exported). Both turrets are their shooter assembly, each turning on its own. A net covers the open top of the hopper.
- 1678: their public Onshape CAD ("1678-26c-0000 CAD Release"), robot page and code (frc1678/C2026: the intake's pivot, its 130° travel and motion profile, and the extension's 0.303 m, also in the AdvantageScope model in that repo). The model is their CAD: the drivetrain with its polycarbonate walls on the bumper mounts, the roller floor (dead-axle rollers, then flex wheels), the ball tunnel and drum, the slapdown intake, the horizontal extension (polycarbonate sides and a corrugated front; a pin on the intake plate rides a slanted slot in its sides, so the intake's 130° swing down pushes it 0.303 m straight out the first time (their code), its front panel flush with the intake's crash bar, then stays out for the match; folding the intake back in doesn't pull it in), and the climber with the corrugated lid that lifts to make the hopper taller (its side skirts ride over the walls; it comes down with the hopper, but not onto FUEL piled under it, so a full 1678 is too tall for the TRENCH). A net stretches diagonally from the lid's front edge down to the front of the extension, and another over the top of the lid, so FUEL piles up under both and bulges them.
- 4946: their public Onshape CAD ("MOTO MOTO", view only, so drawn from Onshape's scaled renders of it) and their 2026 engineering report: the round drivetrain (a half circle of 16.375 in radius whose sides run on to a 30 in flat front edge; the bumpers, the frame and the physics all follow it), clear hopper walls on the bumpers 1.125 in out (a 35 in round hopper), the Dye Rotor tray (its spinner carries FUEL round to the center), the 11" turret on the center of rotation on its column, the perforated-tube gantry with the beacon, and the intake box that slides out the front. A net stretches diagonally from the top of the hopper's front wall down to the intake box, so FUEL collects in the gap between them too.
- 3928: their public Onshape CAD ("Team 3928 2026 CAD Release"). The model is their CAD: the drivetrain, the tall sheet hopper walls, the spindexer (a 5-spoke wheel under a printed cone, in a ring that the corner wedges slope down into), the tower in the back left corner (a J-shaped ramp at its foot and belts up to the top), the turret on its bearing plate with the curved guard across its front, and the intake box that slides 0.3 m out the front (a 2 in and a 1.5 in roller at its lip; its tall sides are hopper walls). The robot's name isn't published.
- 341: their public Onshape CAD ("Miss Daisy XXIV"). The model is their CAD: the drivetrain, the "brontosaurus" slapdown (two silicone rollers at the bottom of an arm with three plastic rollers over them, pivoting 0.34 m up; stowed it stands up), the serializer (a plate 0.175 m up with three 6 in omni wheels lying flat), the mecanum roller bar along the front edge of the table, the uptake ramp, the table with the turret and its hooded shooter, and the telescoping L1 climber.
- 4930: their public Onshape CAD ("Floyd 2: Whole Robot Assembly") and their Open Alliance build thread (28.5 × 26.5 in drivebase, about 120 lb before they took the climber off). The model is their CAD: the drivetrain, a floor of twenty belts sloping down from the intake to the updexer at the back, whose belts lift FUEL behind three hooded flywheels on one long shaft (the hoods wrap round the back of the flywheels, so FUEL leaves forward over the top), and the intake: polycarbonate side plates, rollers and its own belt floor on an arm pivoting at the front of the frame. Their first robot, Floyd, had a turret; this is the second.
- 1706: their public Onshape CAD ("Mirage Public Release") and their CAD release thread on Chief Delphi ("a mirage ... so seeing double, hence the double turrets"). The model is their CAD: the drivetrain, a deep hopper whose box slides out when the intake deploys, the intake (rollers on an arm that folds up inside the hopper and swings out over the bumper), two spindexers side by side under the hopper, and the two shooters on their turret rings behind it (5 in front wheel, 3 in accelerator wheels). A net over the open top stretches up as the load grows (their CAD doesn't model one). They took their climber off. The team calls the turret end the front; here the intake end is the front, as for every robot. Their public "RS-000 Singularity" document is their 2025 robot.
- 581: their public Onshape CAD ("581 2026 Rubble", the "2026 Dumper Champs Bot" assembly and the document's own STEP), their CAD release thread on Chief Delphi and their code (team581/frc-2026). They rebuilt mid-season from a Dye Rotor turret to this dumper. The model is their CAD, and the moving parts follow the CAD's own mates: the intake slides 0.306 m out along titanium racks tilted 5.5° (its plates, 3 in and 2 in rollers, motors, side panels and the front plate of the hopper go with it; the deploy gearboxes stay), its kicker plates hang from a free pivot and swing up over the bumper as it comes in, the roller floor pivots 7.1° up at the shooter tower when the deploy comes in (mechanically linked, per their spec slides, to feed the shooter faster; the CAD's two poses), and the hood turns its 35° on the drum's shaft (open for longer shots, per their code). Their alpha bot shot 15 FUEL/s; the Champs robot's 23 is scaled from its FUEL per match in the Hopper Division (252, up from 145 at its first event; Statbotics) against 1678's and 2910's at Champs. Their code also gives the drive ratio (7.67:1), that the shooter faces 180° from the front, and the hood and drum tables. A trucker net spans the top from the walls down to the intake's front plate.
- 1114: their public Onshape CAD ("S26-A000", Simbot Tim), their reveal, CAD-release and code-release threads on Chief Delphi, and their code (Simbotics/2026-Simbot-Tim). The model is their CAD (a STEP translation of the assembly), and the moving parts follow its mates: the tall clear hopper box slides 0.298 m out on racks rising 8.7° toward the robot, carrying the intake rollers and their motors on its front (the rack gearboxes stay), the kicker bar in front of the rollers hangs on arms that swing up to 90° over the bumper as the box comes in, and the hood turns its 30° on the flywheel's shaft (their code: 42-74°, steepest at home). The belt floor slopes down to the foot of the tower, where rollers and an active wall take FUEL into the ball elevator up to the flywheel, which fires forward over the hopper (their code turns the intake end at the HUB). The red "magic blanket" over the box is from their reveal (after 1124's 2008 claw). They took their climber off for weight (it's in the CAD; not here). The shot rate isn't published: 26 FUEL/s is scaled from their FUEL per match (Statbotics) against 1678's and 2910's at Champs; their thread gives 95-98% accuracy after they wrapped the rollers in grip tape.
- 1690: Orbit's Onshape document doesn't allow export, so the model is drawn from Onshape's own scaled renders of it, their X_T release, the reveal and CAD-release threads and photos: lattice-sided hopper over a powered roller floor, over-bumper intake held down by surgical tubing, electronics box at the back right, and at the back left a vertical kicker of green compliant wheels feeding the turret (lattice A-frame, green flywheel wheels, hood) on its bearing ringed with printed guide fins. The team said it holds 50-55 open-topped and that FUEL bounces out, and that they'd try netting; the net over the top keeps the load in.

971 and 1678 climb Level 1 as built (the AI heads for the TOWER near the end of the match so it's up before the buzzer, one robot per ALLIANCE). The others didn't climb; the **Climber add-on** option gives your robot a hypothetical Level 1 or Level 1-3 climber if you want to try the TOWER.

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
- FUEL that enters a HUB really falls through it: down the funnel, past the sensors (where it's scored) onto a ramp inside, and out through the exit in the NEUTRAL ZONE face. Inside, FUEL is slick and pressed down a little harder than gravity (`HUB.fuelFriction`, `HUB.extraGravity`), and a roof slopes back from the top of the exit, so the load keeps flowing instead of locking into an arch over the opening: 200 FUEL poured in at 60 a second are all out within about 5 s.
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
- Ramming, even at full speed, is legal. High-speed contact isn't called in competition, and robots here rock but don't tip over.
- **G418** (MINOR): PINNING an opponent against a FIELD element for more than 3 s, plus another MINOR for every further 3 s. The count resets when the robots are 72 in apart. The HUD shows the pin count for either robot.
- **G420** (MAJOR): in END GAME, contacting an opponent that is touching its TOWER or climbing.

Driving comes from each robot's drivetrain (`js/drivetrain.js`): four Kraken X60s through its swerve modules' gear ratio (2910, 971 and 1678: MK5n R1, 7.03:1; 4414: 7.67:1; the others from their stated free speeds) to 4 in wheels. The robot's mass includes bumpers and battery (58-66 kg).
- **Off the line:** the robots accelerate as hard as the tires grip (about 1 g), but the push falls off with speed. The motors' current limits (assumed 80 A stator and 70 A supply each, CTRE's usual settings), their back-EMF and the battery sagging under four motors' current all take force away.
- **Top speed:** they top out where what's left meets drag, about 88% of the free speed their gearing is quoted at (2910: 12.7 ft/s of 14.4 free). 5 m from rest takes about 1.55 s.
- **Turning:** driving and turning share the wheels (swerve desaturation), so turning hard at full speed costs speed.
- **Robot cards:** show the real top speed next to the free speed.

Robots push each other with realistic traction (mass × acceleration limit), so heavier or faster-accelerating robots win shoving matches. Robots ride on their wheels and can pitch and roll: they tilt going up a BUMP, over a DEPOT barrier or onto a jammed pile of FUEL, and the FUEL in the hopper feels the tilt. They only drive with wheels on the floor: a short ray down from each wheel finds the carpet, a BUMP or a DEPOT barrier, and the drive gets that share of its grip, so a robot lifted onto FUEL or another robot coasts instead of climbing. Bumpers follow each frame's shape (4414's cut corners, 4946's round back) and are straight-sided (1.25 in to 6.3 in off the carpet, chamfered underneath and a little along the top), so robots meet bumper to bumper and push level. Everything above is solid too: the frame and superstructure, an extended hopper, a raised lid and the FUEL bulging a net meet other robots and the field (a robot coming off a BUMP into another lands against it, not through it), and intakes meet walls and robots as they come out (one deployed against a wall pushes its robot back). Most of the weight is low (drivetrain, battery), and pitching and rolling are capped and damped the way tires and frame flex soak up a hit, so a crash or a DEPOT barrier at full speed rocks a robot (under 30°) but doesn't flip it.

**FUEL in the robots** (nothing teleports):
- **Intake:** the rollers grab FUEL (still a physics ball) and drag it up the intake arm, over the BUMPER and in through the slot under the hopper wall, at the robot's intake rate. FUEL the rollers let go of before it's over the BUMPER drops back onto the carpet.
- **Capacity:** a robot holds as much as its modeled hopper physically does, packed the way its intake packs it, not the team's stated number. FUEL comes in one at a time at the front, on top of what's there, and the intake roller shoves each one into the load, so the load fills out against every wall, the floor and the nets; the hopper is full when a FUEL stays pressed into the load harder than the roller pushes (`Hopper.pack`, `measureCapacity` in `js/hopper.js`). How hard a roller pushes comes from how much it squeezes a FUEL (`intake.squeeze`: teams report 3/4-1 in with compliant wheels, 1/2-5/8 in with rigid rollers, [Chief Delphi](https://www.chiefdelphi.com/t/what-is-the-optimal-compression-for-fuel-between-a-2-inch-roller-and-35a-compliant-wheels-2-inch-diameter/511867)), the foam's spring and rubber-on-foam grip: about 60 N (13 lbf) at 3/4 in (`intakePush`). That packing measure (precomputed: `tools/capacity.mjs` writes `js/capacities.js`; a hopper that's changed since is measured live) is only the robot's rated capacity, for the cards, the HUD and the AI's plans. In play nothing counts: the intake stalls. FUEL goes in at the bottom of the pile, on the hopper floor at the front (at the front of a hopper that's out over the intake), not on top of it. A FUEL goes in only once the one before it has moved out of the way; until then the rollers hold it in the mouth and push the FUEL in its way back and up, as hard as they grip it (the same push), so they have to lift the pile over it: its weight (and the FUEL's grip on each other) pushes back. When the load gives way it goes in; the fuller the hopper, the longer that takes, so intaking slows down as it fills. When a FUEL has waited 0.35 s the rollers have stalled: they stop grabbing FUEL, the load pushes the waiting one back down into the stalled roller at the front of the intake (it sticks out a little past the intake), and after 0.75 s stalled the intake is full (the HUD says FULL; the number after the slash is only the rating). It takes no more FUEL until the load gives (it settles, the robot shoots, a folding intake compacts it). Drive into a wall with a FUEL stuck in the roller and the wall squashes it: that adds to the rollers' push (as hard as the FUEL is squashed, up to a few hundred N, and it pushes the robot back as hard), so the load gets packed tighter than the rollers alone would pack it (4414, loaded and driven into the guardrail: 87 instead of 83). Filling from empty in the NEUTRAL ZONE at 1 m/s, intakes take 30-35 FUEL/s at first and stall at about 70-100% of the rated capacity, depending on how easily each pile lifts (2910 at 68 of 73, 971 55 of 66, 1678 93 of 102, 4414 100 of 101). 8793's and 341's ball paths aren't packed by pushing, so they stop at their measured count. 8793 has no hopper, so its ball path is still poured. On 8793 and 341 the deployed intake holds FUEL too (the ball path or tray runs out over its rollers), so the intake stays down while it's holding FUEL there.
- **Hopper:** held FUEL is simulated in the robot's own frame with a lighter solver: gravity, soft ball-to-ball contact (FUEL is a full 5.91 in foam ball that gives softly at first and firms up exponentially as it's squashed: 12 lb/in to start, 20.6 lb at 1 in, 42 lb at 1.5 in, 77 lb at 2 in, set so the first inch averages the 20 lb/in nominal spring rate in AndyMark's [2026 Scoring Element Testing Report](https://community.firstinspires.org/hubfs/blog/frc/2026-scoring-element-whitepaper-am.pdf); it's drawn flattened where it presses on other FUEL or the walls), friction between touching FUEL (foam on foam, μ 0.5: a load holds together and bridges instead of flowing like marbles), the walls, floors and internal parts, and the robot's own motion, so the load piles up, slides back when you accelerate and sloshes when you spin. Each robot's mechanisms move it:
  - 2910's powered floor rolls FUEL back to the indexer, and while it shoots its intake retracts slowly and pushes the load back into the indexer (it stalls against the FUEL until there's room). Its hopper slides out with the first intake and stays out for the match.
  - Folding intakes that compact (2910, 971, 1678, 4930, 1706): the intake's roller end is a moving wall of the hopper, so as it folds in it really touches the FUEL out over it and sweeps it back into the hopper. It keeps folding while the FUEL still fits behind it (packed as hard as the fold motor pushes, `compactPush`), and stalls against the load when it doesn't, so a full robot's intake stays part way out holding the overflow.
  - 4414's Dye Rotor: the pocketed rotor spins under the load and carries it round, and the Dolphin Fin ramp on its rim sweeps it along. A fixed hook of passive rollers steers FUEL in to the feeder at the center column, where the omni and feeder wheels lift it up a ramp into the turret. Printed "stadium" pieces funnel FUEL onto the rotor, and when it isn't feeding the rotor turns slowly backward to agitate the load. FUEL comes in under the intake box's front roller, up its hinged ramp and over the front bumper (a star roller drives it); the box is part of the hopper, so FUEL also collects in it.
  - 8793 has a ball path, no hopper: FUEL comes in 4 wide over the front of the frame (the outer two ride over the swerve covers), and the conveyor's wheels carry it back down its floor, 2 wide, to the middle of the robot, where the indexer's walls close in to single file. Two omni wheels on opposite sides of that throat spin FUEL against each other, so two arriving abreast roll round each other and go in one at a time instead of wedging. Single file, it runs back along the J-shaped rails under the turntable, up their curve and up through the turret into the shooter, then out forward over the flywheel under the hood (`bay.taper`, `bay.floor.pts` in `js/robotConfigs.js`). It holds what fits in the path (10) and moves 12-13 FUEL/s through it, more than the shooter's 10.
  - 971's roller floor carries FUEL back to the separator, which splits it into two streams: each goes up its own ramp into its turret, one turret after the other. Its net lets the load pile up over the walls.
  - 1678's roller floor carries FUEL back to the ball tunnel, which lifts it to the drum. Its lid rises with the hopper, so the load can stack higher; it can't come down onto FUEL. Its extension slides out with the first intake and stays out; folding the intake in is what compacts the load.
  - 1690's roller floor carries FUEL back to the vertical kicker in front of its turret. Its net lets the load pile up over the walls.
  - 4946's Dye Rotor tray carries FUEL round to the turret's column, where it goes up into the shooter. The diagonal net over the gap to its intake box is the ceiling there.
  - 3928's spindexer: five spokes turn under the load in the middle of the hopper (the floor slopes down into it from the corners) and carry FUEL round to the mouth of the tower, which only takes a FUEL that has got there; the tower lifts it single file to the turret. The spokes deliver about 8.5 FUEL/s.
  - 341's serializer: one layer of FUEL on the plate under the table; the three omni wheels drive it to the uptake at the back (`feed.pull`), which curls up into the turret. It holds 7, so it shoots about as fast as it intakes.
  - 4930's belt floor carries FUEL back and down to the updexer, which lifts it into whichever of the three shooters is next. Its intake expands the hopper out over it, under a net stretched from the shooters' side plates down to the intake.
  - 3928's net covers its open top (with a hole round the turret), so the load can bulge it up.
  - 1706's two spindexers each carry FUEL round to its back edge (the floor slopes down into them from the corners), where a ramp takes it up into the turret behind it. Above the spindexers the back of the hopper is a polycarbonate wall in front of the turrets (`bay.obstacles` boxes).
  - 1678's lid and 4946's and 4930's nets: over the part in front of the hopper, the ceiling slopes down with the diagonal net to the front of the extension or intake box (`bay.slope`).
- The hopper walls, floors and tops were checked against each robot's CAD (sliced at several heights with the hopper drawn over it): 2910's walls are 0.69 m apart and the front of its top panel bevels down (`bay.bevel`), 971's hopper is the frame's full width and its roller floor climbs over the front swerve modules (`floor.pts`), 1678's ceiling is the lid itself (0.545 m), and 1706 has its upper back wall. The rest were within 2 cm.
- **Shooter:** FUEL travels the feed path to the flywheel and leaves from the exit at the same rate (BPS) and with the same shot model as before. The multi-lane dumpers (2910, 1678, 4930) don't fire their lanes in a fixed order: each FUEL goes to a random other lane, after a random gap (half to one and a half times the average), so the stream comes out ragged like a real one at the same average rate. A FUEL that reaches the wheels while the shot isn't lined up waits there.

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

**AUTO EDITOR** in the main menu is a simplified PathPlanner. The field (your half, ALLIANCE WALL on the left) is on the left; the settings are on the right; a bar along the bottom always shows the controls for what you're doing (for the controller or for the keyboard and mouse, whichever you're using).

- **Field:** drag the START box along the ROBOT STARTING LINE and place waypoints for the path. A label at the cursor says what A / Enter / a click does there (add waypoint 3, pick up waypoint 2, move START…). Arrows on the path show the direction; a legend under the field explains the colors (drive, intake, shoot, both, dashed = shoots or passes anywhere, thicker = faster). Waypoints past the CENTER LINE turn red as a G403 warning.
- **Settings, grouped:** *Auto* (which auto you're editing, the robot, New / Copy / Mirror / Rename / Delete), *Start* (starting spot, shoot the preload first), *Waypoint N* (on the way here: intake, shoot, speed; when it gets here: keep driving, stop, stop and shoot it all, wait 1–3 s), and *Path*, a list of every waypoint to jump to. The focused setting's description shows below them.
- **Time:** a bar at the top shows the estimated run time against the 20 s AUTO. **Test ▶** starts a match with the auto straight away; **Done** goes back.

Stick for the field, D-pad for the settings: whichever you touched last is active (outlined). Autos save automatically in the browser (localStorage) and appear in every **Auto routine** menu marked with ✎.
- Red alliance runs the path rotated automatically.
- With a custom auto selected, **Starting position** switches to *As drawn* / *Mirrored left ↔ right*.

| Editor action | Xbox controller | Keyboard | Mouse |
|---|---|---|---|
| Move the cursor | Left stick (hold LS click for fine) | W A S D | — |
| Add a waypoint / pick up / drop | A | Enter | Click, drag to move |
| Delete the waypoint | X | Delete | Right-click |
| Previous / next waypoint | LB / RB | F / R | Click it in *Path* |
| Choose a setting / change it | D-pad ▲▼ / ◀▶ | ↑↓ / ←→ | Click its ◀ / ▶ side |
| Press the focused button | A | Enter | Click |
| Test in a match | Y | H | Test ▶ |
| Back to the field / done | B · Menu (☰) | Esc | Done |

## Real CAD

The field you see is FIRST's official field CAD (Onshape "FE-2026: REBUILT Playing Field"), in `cad/field/field.glb`. It's slimmed for the browser: the FUEL, carpet, tape and small hardware are dropped, and it has about 360k triangles in 26 meshes. The bleachers, HUMAN PLAYERS, HUB lights and CHUTE DOORS are still drawn by the game. Physics still uses the colliders built from the dimensions in `js/constants.js`, which line up with the CAD. To go back to the drawn field, set `"enabled": false` on the entry in `cad/manifest.json`.

To refresh the field from Onshape (this needs API keys):
```
tools/onshape-export.sh /tmp/field-raw.glb
npm i --no-save @gltf-transform/core @gltf-transform/extensions @gltf-transform/functions meshoptimizer
node tools/slim-glb.mjs /tmp/field-raw.glb cad/field/field.glb
```

The robots with public CAD (2910, 971, 1678) are built from it: `tools/extract-parts.mjs` takes a robot's Onshape GLB export and a recipe (`cad/robots/971.json`, `1678.json`) and writes each moving part (body, intake, sliding hopper, turret, lid) as its own light GLB in the robot's frame, with its origin on its pivot or axis. The game loads them in the browser and moves them; the drawn stand-ins show until they arrive, and headless runs never load them. To rebuild 971's (needs the API keys; 1678's is the same with its document):
```
DID=cabaa0c1c77517916df80783 WID=48cb057db38a03cc202ff44e tools/onshape-export.sh /tmp/r971.glb 96844befd4591dac162d657b
node tools/extract-parts.mjs /tmp/r971.glb cad/robots/971.json
```

Other CAD, such as robots or field elements, can be added the same way. Export glTF (.glb) from Onshape, or convert STEP with `python3 tools/cad2glb.py in.step out.glb`. Then list the file in `cad/manifest.json` (see `cad/README.md`).

## Project layout

```
index.html, css/style.css   UI shell and styles
js/constants.js             field dimensions, timing, points (from the manual)
js/field.js                 field geometry and colliders
js/fuel.js                  all 504 FUEL: physics, drag, HUB scoring and exits, returns
js/ballistics.js            drag-aware trajectory model, shot tables, shoot-on-the-move solver
js/robotConfigs.js          the robots' specs
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
tools/ai-trips.mjs          how full the scripted Scorers are when they head in, and their points
js/nn/                      neural-net driver: observations, policy (plain-JS MLP), in-game driver, recorder
tools/nn/                   neural-net trainer (train.py, PyTorch) and its simulation worker (worker.mjs)
nn.html                     neural-net training dashboard
start.bat / start.command   double-click launchers (Windows / macOS)
nn-setup.bat, nn-train-gpu.bat, nn-train.bat, nn-dashboard.bat   neural-net setup / GPU training / real-game training / dashboard (Windows)
```

To tune a robot, edit `js/robotConfigs.js`: speed, BPS, hood range, exit speed and accuracy. How much a robot holds comes from its hopper (`bay`): change the hopper and the capacity follows.
