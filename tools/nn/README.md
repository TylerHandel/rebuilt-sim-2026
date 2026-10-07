# Neural-network driver (end-to-end)

A neural network that drives the robot directly, the way you do with a controller. Ten times a second it chooses:
- how fast to move and in which direction,
- how fast to turn,
- whether to hold the intake, shoot, pass, or spit FUEL out (outtake).

It drives the whole match, AUTO included. Nothing is scripted for it. Aiming works the same as when you hold the trigger: the turret, or the chassis on the 2910, aims itself.

It learns by playing hundreds of thousands of matches with the game's own physics. It can start by copying your recorded driving, then improves against the scripted AIs and against older copies of itself.

## How it uses your computer
| Part | Job |
|---|---|
| **CPU, all threads** | One simulation worker per hardware thread, minus one for the trainer. On an i7-11700F (8 cores / 16 threads) that's **15 workers**, each playing full matches back to back. Each worker runs the network itself, so no worker ever waits on another. |
| **GPU (CUDA)** | PyTorch updates the network on your graphics card (an RTX 3070 here) while the workers keep playing. The new weights reach the workers between their chunks of play. |
| **RAM** | About 260 MB per worker, so roughly 4 GB with 15 workers. |

The workers run at below-normal priority, so the PC stays usable while training. Pass `--full-priority` to squeeze out the last few percent.

**What sets the speed:** the physics simulation of 504 FUEL balls on the CPU. The network math is tiny next to it, so the GPU spends most of its time waiting; that's normal for this kind of training.
- On 3 cores of a cloud machine: about 15–35 decisions per second per worker with an opponent robot, more when it plays alone. (The detailed robot and FUEL physics cost more than they used to.)
- On your i7, expect very roughly **300–800 decisions per second** in total, which is about 700–1,800 matches per hour. Check the dashboard for the real number.

## How long it takes
These are rough expectations, not measurements on your PC. End-to-end learning is slow compared with the scripted AI, which already knows how to play.
- **Lifting its score** past the human-player points takes a few million decisions: a few hours.
- **Learning to collect and score** on its own takes tens of millions of decisions: one or more nights.
- **Beating the Champs and Trained scripted AIs** is likely to take several nights.

Starting from your recorded driving (`--bc`) saves a lot of that.

The dashboard shows whether it's improving. If points per match stay flat for many hours, see Tips below.

## Fast training on the GPU (recommended start)
`nn-train-gpu.bat` runs `tools/nn/train_gpu.py`, which trains in a **simplified copy of the game that runs on the graphics card**. It plays 4,096 matches at once as tensor math, so the RTX 3070 does both the simulation and the learning. The full game, by contrast, needs one CPU thread per match.

**Match sizes:** 1v1, 2v2 and 3v3 (by default 25% / 15% / 60%). Teammates are the same network, so it learns to play together: splitting up, staying out of each other's way, and taking defense when that helps the alliance.

**How it's simplified** (`tools/nn/gpusim.py`):
- **Robots:** the real speed, acceleration and turning limits, and the real sizes, intake width, capacity, shot rate, flywheel spin-up and aiming. Each robot's numbers are exported from the real game into `tools/nn/gpusim-params.json` by `tools/nn/gpusim-export.mjs`.
- **Collisions:** robots are boxes. They collide with the walls, HUBs, TRENCH posts, TOWERs and each other, so they can push, block and pin. Robots too tall for the TRENCH are stopped by its arm.
- **Intake:** matched to the real robots (`tools/intake-test.mjs`):
  - up to about 70 FUEL/s driving through a pile;
  - the rollers grip about a row at a time;
  - packing hoppers jam at about 80% of their listed capacity, as they do in the real game.
- **FUEL:**
  - rolls and slows down, and bounces off walls, field elements and bumpers;
  - pushed FUEL takes momentum from the robot;
  - FUEL can't stack: crowded FUEL is pushed apart, so a pile spreads instead of being swallowed in one spot.
  - It's still simpler than the real game, which simulates every FUEL-FUEL contact.
- **Shots and passes:** timed flights with a hit chance instead of full ballistics. The hit chance per robot and speed is measured in the full game by `tools/nn/shoot-test.mjs` (saved in `tools/nn/shoot-calib.json`). Turret robots score nearly every shot even at full speed; robots that aim with the chassis (2910, 1678) drop to about 60–70% at full speed. Scored FUEL goes through the HUB and comes back out the exit opening into the NEUTRAL ZONE.
- **Match rules:** the real ones: AUTO, the gap, the TELEOP shifts with the active-HUB rules, the 3 s scoring grace, and human players throwing from the CHUTE. Fouls, the TOWER and BUMP slopes aren't modeled.
- **Domain randomization:** each robot's speed, acceleration, intake and accuracy, and the carpet's rolling resistance, vary a little from match to match. That way it learns habits that still work when the real game differs a bit. `--no-randomize` turns this off.

**Same network:** the observation is built exactly like the real game's. All 482 numbers match real-game snapshots in 1v1 and 3v3; the only difference is the score when fouls happened. The controls are the same too. So `js/nn/driver.json` from the GPU trainer drives in the real game as is.

**Opponents:**
- **To start:** scripted bots (80%, some of them defending) and nobody (20%). The scripted bots fill their hoppers before a trip and head in with less only when time runs short: at the last moment they can still empty the hopper before their HUB turns off, or in time to be staged when it turns on. (They used to head in with a few FUEL whenever their HUB was active; filling up more than doubled their score, from about 150 to 350–380 points per alliance in 1v1.)
- **Self-play:** once it beats the bots 60% of the time, it switches to:
  - 35% of matches against itself, with both alliances learning;
  - 35% against older versions of itself (a new one is saved every 50 updates; it keeps the last 16);
  - 25% against the bots;
  - 5% alone.
- **Older versions it loses to come up more often** (prioritized self-play). It keeps a win rate against each one and picks the ones it struggles with. Every 25 updates the console lists them as `v<version> <its win rate>/<share of those matches>`. `--no-pfsp` plays them all equally.

**Team critic:** the value network (the critic, which judges how well things are going during training) sees the robot's own view and both teammates' views. It can judge the alliance as a whole, so credit for teamwork lands where it belongs. Only the driving network goes into the game; it still sees just its own view.

**Winning, not just points:** while it learns the game, the reward is the points margin. Once self-play is on, the reward shifts over 50M decisions (`--win-ramp`) toward winning:
- a squashed margin, so closing a 10-point gap in a close match counts far more than adding 10 to a blowout;
- a bonus for winning, or a penalty for losing, at the final buzzer.

The console shows the current `win-weight` (0 = points only, 1 = mostly winning). `--win-weight 1` sets it straight away. As the win weight rises, it also looks further ahead: `gamma` goes from 0.995 to 0.998, which is about 20 s to 50 s of match time.

**Real-game scoreboard:** while the GPU trains, the CPU has little to do. So the trainer runs 2 full-game matches at a time there, at low priority: the latest network against the Scorer, Defense and Hybrid AIs at Champs skill, 1v1 and 3v3, with every robot on its alliance driven by the network. The dashboard shows its real-game win rate over time and by opponent, and a table comparing the simulator with the real game: FUEL picked up, shots, how much it turns, and how often it flips its turn direction. A big difference there points to what the simulator gets wrong. A full match takes a minute or two of CPU. `--eval 0` turns this off; `--eval 4` plays more at once.

**The dashboard** (`nn-dashboard.bat`, the page `nn.html`) follows a run as it trains. It opens the run updated most recently (pick another in the Run menu) and says whether it's training right now. The Range and Across controls scope every chart (all of it or the last 24 / 6 / 1 hours; by training hours or decisions), every chart has a Table button, and the settings stick in your browser.
- **Headline numbers:** points per match, win rates against the scripted bots, its older versions and (real game) the Champs AIs, speed, training so far, each with its change over the last hour of training.
- **What's going on:** plain-language notes read from the log: improving or flat, the stage it's in (self-play, win weight, teacher), how it does against its older versions, how well its critic predicts, warnings (stopped, exploration nearly gone, slower than earlier, wasted FUEL, a big gap between the simulator and the real game) and its best and weakest robot.
- **Is it getting better:** points per match (its alliance and the opponents), win rates over time, where its points come from (AUTO / TELEOP), runs side by side (tick runs to compare), win rate by match size and opponent, and its win rate against each older version it keeps.
- **How it plays:** FUEL picked up, shot and passed per match, where it drives (own zone / NEUTRAL ZONE / their zone), FUEL carried, distance, FUEL put into an inactive HUB, and a table by robot (GPU simulator and real game).
- **Real game, learning health and speed:** the full-game record against the Champs AIs, the simulator-vs-real table, critic accuracy, step size, exploration, losses, the win weight and teacher pulls, decisions per second, and where the time goes (playing vs learning).

It reads only what the logs gained since its last look, so a log of several nights stays quick. If the dashboard's port is taken (an older dashboard still running, maybe from another copy of the project), it uses the next free one, and it shows which folder's runs it's reading.

**Live view:** the dashboard shows a grid of the matches the trainer is playing right now, as a bird's-eye view: robots in alliance colors, learning robots outlined, FUEL, HUB lights, score and clock. It shows 4 matches by default; `--live 0` turns it off and `--live 9` shows more, at a small cost in speed.

**Your earlier network carries over.** The observation grew for 3v3: teammates and the 2nd and 3rd opponents were added at the end. When you `--resume` a run trained before that, it's upgraded automatically with zero weights on the new inputs, so it plays exactly as before until it learns to use them. The same goes for the team critic. Older driving recordings still load too.

**How it uses the GPU:**
- Every match has the same array sizes at every step, and no step waits for the CPU. So the trainer records a whole decision once as a **CUDA graph** and replays it: the networks, the physics, the resets and the observations for all 4,096 matches in one go, without Python in between.
- `--compile` also fuses the simulator's math into a few large GPU kernels. The first start with `--compile` takes a few minutes; later starts reuse the cache.
- The rollout is stored in half precision, and only one minibatch at a time is normalized. That uses about half the memory, which leaves room for more matches (`--envs 8192`).
- If the graph can't be recorded on your setup, the trainer says so and trains without it.

**Then fine-tune in the real game.** The two trainers save the same checkpoint format, so `nn-train.bat --resume --level 3` continues the same run (`runs/driver`) in the full game. It plays 1v1 and 3v3 there (`--teams`), with its teammates driven by the network too.

**Learning from the pre-programmed AIs** (imitation, then training on top). Recordings made before the AIs learned to fill their hoppers (October 2026) show them heading in with a few FUEL: delete the `recordings-ai` folder and record them again.
1. `nn-record-ai.bat` records the Champs-level AIs (mostly Scorer, some Hybrid and Defense) playing full matches in the real game, 1v1 and 3v3: what each robot saw and did, every 0.1 s. It uses every CPU thread and saves 1,000,000 decisions to `recordings-ai` (about 15–40 minutes). Running it again adds more, up to `--samples`.
2. `nn-train-from-ai.bat` starts a new network in its own run (`runs\from-ai`), so your current one stays as it is. It first copies the AIs' driving, so it starts out playing like them instead of twitching randomly. Then it trains on the GPU as usual, imitating them a little less as it goes (`--bc-weight 0.5`, fading over `--bc-steps` 300M decisions). Pick `from-ai` on the dashboard to compare the two runs' real-game records.
3. To have your current network learn from them too, add `--bc recordings-ai` to its usual command. It doesn't start over, it just gets pulled toward the AIs' habits while it keeps training. `--bc-pretrain` copies them outright first, which overwrites much of what it learned.

`nn-train.bat --bc recordings-ai` works for the real-game trainer as well.

**All of it at once (overnight):** `nn-overnight.bat` continues `runs\driver` on the GPU (8,192 matches, `--compile`) while 10 CPU threads record the AIs in the background (`--record-ai 10`, into `recordings-ai`, up to `--bc-max` 2,000,000 decisions) and 2 play scoreboard matches. It picks up new recordings every 20 minutes (`--bc-reload`) and keeps imitating them with weight 0.3, fading out over 2 billion decisions (a few hours). Ctrl+C saves; double-click again to continue.

**Continuing a trained network** (double-click):
- `nn-continue-gpu.bat`: resumes `runs\driver` on the GPU with `--compile` and a mix of mostly older versions of itself, itself, and the bots. Add options after it, for example `nn-continue-gpu.bat --envs 8192`.
- `nn-overnight.bat`: the same, plus learning from the pre-programmed AIs recorded in the background (see below).
- `nn-train-pro.bat`: the strongest recipe (see below), in its own run `runs\pro`.
- `nn-continue-champs.bat`: backs up `runs\driver` to `runs\driver-gpu`, then trains in the full game against the Champs AIs, 1v1 and 3v3. It's slower, but it learns the real physics.

**The strongest recipe** (`nn-train-pro.bat`, its own run `runs\pro`): a bigger network (1024 × 512 instead of 512 × 256) that starts by copying a teacher and then trains past it in self-play.
- If you already have a trained network (`runs\driver`), that's the teacher. The new network copies it first (`--teacher-warmup`, 30 updates), so nothing it learned is lost, then keeps improving with more room to learn. It writes `js/nn/driver.json` like the others.
- Otherwise the teacher is the GPU simulator's scripted robot, so it starts out playing sensibly instead of twitching at random.
- Running it again continues where it left off, teacher included.

**Learning from a teacher** (`--teacher`): each decision, the teacher says what it would do in the very situation the network is in, and the network is pulled toward that on top of its own trial and error. This is different from copying recordings (`--bc`): recordings only show the teacher's own matches, so a network that drifts somewhere the teacher never went has nothing to go on. Here it learns from its own mistakes too. The pull fades out over `--teacher-steps` (200M decisions), so it can go past the teacher.
- `--teacher bot`: the GPU simulator's scripted robot.
- `--teacher runs\driver` (a run folder, its `ckpt.pt`, or a policy `.json`, such as a shared network): an earlier network. With a new `--run` and a bigger `--hidden`, this grows a bigger network from a trained one.
- A new teacher starts with `--teacher-warmup` updates of copying only. `--teacher none` stops learning from it.

**How it learns (these are on by default):**
- **The critic sees the whole match.** Besides the three teammates' views, it gets every robot's load, flywheel and readiness, how fast and accurate each robot is this match, who's driving the other alliance, the FUEL in flight (and whether it will score), in the HUBs and in the CHUTES, and the score by period. It never goes into the game, so it may see what the driving network can't. The better it judges a situation, the clearer the lesson from each decision. `--no-priv` turns it off.
- **The critic learns normalized returns** (PopArt). The reward changes scale as the win bonus ramps in, and the critic's targets would change with it; normalizing them keeps its learning steady and keeps its gradients from crowding out the driving network's in their shared gradient clip. `--no-popart` turns it off.
- **Updates stop early** once the network has moved far enough from the one that played the matches (`--target-kl`, 0.02): more passes over the same matches would only overfit them, and they cost time.
- **The older versions it keeps** are the ones that still give it trouble: when the pool is full, a new version replaces the one it beats most easily (of those it has played enough, keeping the 4 newest), not the oldest. So it can't forget how to beat a strategy it once struggled with. `--pool-evict oldest` goes back to dropping the oldest.
- The console shows `critic` (how much of the outcome the critic explains: 1 is all of it, 0 no better than the average) and `epochs` (update passes it made).

Earlier networks carry over: `--resume` adds the critic's new inputs with zero weights, so it judges exactly as before until it learns to use them. `nn-train.bat --resume` (the real-game trainer) still continues a GPU run: it folds the normalization into the critic and drops the inputs it can't fill.

**Options** (`tools/nn/train_gpu.py --help`):

| Option | Default | |
|---|---|---|
| `--envs` | 4096 | matches at once. More means more matches per hour, until the GPU is full. If you get "CUDA out of memory", lower it. |
| `--teams` | 1v1 25%, 2v2 15%, 3v3 60% | match sizes; repeat a size to weight it. `--teams 3` is 3v3 only; `--teams 1 3 3` is one-third 1v1, two-thirds 3v3. |
| `--substeps` | 4 | physics steps per 0.1 s decision. 2 is faster and a bit coarser. |
| `--rollout` | 32 | decisions per match between updates |
| `--robots` | all 11 | robots it learns to drive |
| `--selfplay` | | turn self-play on right away instead of waiting until it beats the bots |
| `--mix A B O I` | 0.05 0.25 0.35 0.35 | during self-play, the share of matches alone / vs the bots / vs older versions / vs itself. `--mix 0 0.1 0.45 0.45` is almost all self-play. |
| `--pool-size` | 16 | older versions kept to play against |
| `--no-pfsp` | | play all older versions equally, instead of the ones it loses to more often |
| `--win-ramp` | 50M | decisions over which the reward shifts to winning once self-play is on |
| `--win-weight` | | fix the win weight (0 to 1) instead |
| `--gamma` | 0.995 → 0.998 | how far ahead it looks; by default it rises with the win weight |
| `--smooth` | 0.005 | cost of changing the drive or turn command between decisions, so it holds a heading instead of twitching back and forth. `--smooth 0` turns it off. |
| `--spin` | 0.003 | cost of turning, so it turns when it needs to rather than all the time |
| `--live` | 4 | matches shown on the dashboard. 0 turns it off. |
| `--eval` | 2 | full-game matches played at a time on the CPU for the real-game scoreboard. 0 turns it off. |
| `--compile` | off | `torch.compile` fuses the simulator into a few GPU kernels. On Windows it needs `.venv\Scripts\pip install triton-windows` first; without it the trainer says so and carries on normally. |
| `--no-graph` | | don't record the decision as a CUDA graph (only to rule it out if something looks wrong) |
| `--no-amp` | | run the critic in full precision instead of bf16 on the tensor cores |
| `--teacher` | | learn from a teacher on its own matches: `bot`, a run folder, its `ckpt.pt`, or a policy `.json` (see above) |
| `--teacher-weight` | 1.0 | how strongly it's pulled toward the teacher |
| `--teacher-steps` | 200M | decisions over which that pull fades out |
| `--teacher-warmup` | 30 | updates at the start of a new teacher where it only copies it |
| `--hidden` | 512 256 | network size for a new run (`nn-train-pro.bat` uses 1024 512) |
| `--no-priv` | | the critic sees only the teammates' views, not the whole match |
| `--no-popart` | | the critic learns plain returns |
| `--target-kl` | 0.02 | stop the update passes early once the network has moved this far (0: always all of them) |
| `--pool-evict` | easiest | which older version a new one replaces: the one it beats most easily, or the `oldest` |

The console prints decisions per second and roughly how many matches per hour that is. On this project's 4-thread cloud CPU, with no GPU, it runs about 1,000–2,500 robot decisions/s with a few dozen matches; the full game manages about 20–50. On an RTX 3070, before the CUDA graph, it ran about 45,000–50,000 decisions/s (about 30,000 matches an hour).

## Setup (Windows)
1. Install **Node.js LTS** (https://nodejs.org) and **Python 3.11+** (https://python.org). In the Python installer, tick "Add python.exe to PATH".
2. Update your **NVIDIA driver** (GeForce Experience or nvidia.com).
3. Double-click **`nn-setup.bat`** in the repo folder. It installs the Node packages and creates a Python environment (`.venv`) with PyTorch for CUDA. Then it prints whether it found your GPU and how many workers it will use.

On macOS or Linux, run the same steps by hand:
```
npm install
python3 -m venv .venv && .venv/bin/pip install numpy torch
.venv/bin/python tools/nn/train.py
```

## Training
| Do this | How |
|---|---|
| Start training | `nn-train.bat` |
| Stop | **Ctrl+C**. It saves `runs/driver/ckpt.pt` and the current network, then exits. Press Ctrl+C twice to quit immediately. |
| Continue later | `nn-train.bat --resume` |
| Watch progress | `nn-dashboard.bat` opens the training dashboard (see below) |
| Watch it play | In the game menu set **Your robot driven by → Neural net (watch)**, or **Opponent → Neural net** to play against it |

While training runs, the newest network is written to `js/nn/driver.json` after every update. The game reloads it after each match, so you can play against it as it learns. Keep the file you like: it's the one the game ships with, the same way `js/trainedBrain.js` is for the scripted AI.

### Learn from your driving first (recommended)
1. In the game, play full matches yourself. Every finished match you drive is recorded for the network: what it would have seen, and what you did, 10 times a second. AUTO is recorded too, as your auto routine drove it.
2. Open **AI TUNING → Download my driving for the neural net** and save the file into a `recordings` folder in the repo.
3. Run `nn-train.bat --bc recordings`. It first learns to copy you, using the GPU, which takes seconds. Then it keeps practicing on its own, and keeps imitating you a little for the first 10 million decisions.
4. To see only the clone of your driving: `nn-train.bat --bc recordings --bc-only`, then pick Neural net in the game.

More matches give a better clone; 10 or more is a good start. Play the way you want it to play.

### Opponent curriculum
It starts alone on the field and moves up a level once it wins at least 55% of matches against everything in the level before it:

| Level | Opponents |
|---|---|
| 0 | Solo (no opponent). Passed at 100 points per match. |
| 1 | Scorer, Rookie |
| 2 | Defense and Hybrid, Rookie |
| 3 | Scorer, Defense and Hybrid, Regional. From here, older copies of itself join the opponent pool (self-play). |
| 4 | Champs |
| 5 | Trained |

Opponents it doesn't beat yet get picked more often. Use `--level N` to start at a higher level.

## Options (`tools/nn/train.py --help`)
| Option | Default | |
|---|---|---|
| `--run NAME` | driver | separate runs live in `runs/NAME/` |
| `--workers N` | CPU threads − 1 | simulation workers |
| `--device` | auto | `cuda`, `cpu` or `auto` |
| `--robots` | 2910 4414 8793 | robots it learns to drive. One network drives them all, because it sees which robot it's in. Add any of the 11 robots by team number. Train on one robot for faster learning. |
| `--teams` | 1 3 | match sizes: half 1v1, half 3v3 (its teammates are driven by the network too). 3v3 matches take about 3× the CPU. |
| `--win-weight` | 0.3 / 0.6 / 1.0 | how much the reward is about winning rather than points, at levels 3 / 4 / 5 |
| `--batch` | 16384 | decisions per PPO update |
| `--hidden` | 512 256 | network size (this is what runs in the browser) |
| `--critic` | 1024 512 256 | value network (GPU only, never exported) |
| `--shaping-steps` | 20M | a small bonus for picking up FUEL that fades out over this many decisions |
| `--no-publish` | | don't overwrite `js/nn/driver.json` |

## Files
| File | What it is |
|---|---|
| `js/nn/obs.js` | What the network sees: its robot, both HUBs, the shift clock and score, the other robot, 16 distance rays, and FUEL around it and over the whole field. It also converts the network's output into a robot command and back. |
| `js/nn/policy.js` | The network run in plain JavaScript, used by the browser and by the workers. |
| `js/nn/driver.js` | Lets the network drive in the game (Neural net option). |
| `js/nn/recorder.js` | Records your driving and handles the download. |
| `tools/nn/worker.mjs` | A simulation worker. |
| `tools/nn/gpusim.py` | The simplified game on the GPU (PyTorch). |
| `tools/nn/gpusim-export.mjs` → `gpusim-params.json` | Robot specs, field and FUEL layout taken from the real game. Re-run the export after changing robots or the field. |
| `tools/nn/train_gpu.py` | The GPU trainer (`nn-train-gpu.bat`). |
| `tools/nn/shoot-test.mjs` → `shoot-calib.json` | Measures each robot's shooting accuracy at different speeds in the full game, for the GPU simulator. |
| `tools/nn/record-ai.mjs` | Records the pre-programmed AIs playing full matches, for imitation (`nn-record-ai.bat`). |
| `tools/nn/share.py` (`nn-share.bat`) | Prepares a trained network for sharing and opens the GitHub upload page. Shared networks live in `js/nn/shared/`; the website lists them (`tools/nn/share-index.mjs` writes the list when it's deployed) under **Neural net** in the menu. |
| `tools/nn/eval.mjs` | The real-game scoreboard: full-game matches of the latest network against the Champs AIs, started by the GPU trainer. Results go to `runs/<run>/eval.jsonl`. |
| `tools/nn/train.py` | The trainer: PPO with GAE, observation normalization, the curriculum, self-play, imitation and checkpoints. |
| `nn.html` | The training dashboard. |

## Tips
- **Changing `js/nn/obs.js`** changes what the network sees, so old networks and recordings stop matching. Bump `OBS_VERSION` there and in `tools/nn/train.py`, then start a new run.
- **Score flat for hours:**
  - Train on one robot, for example `--robots 4414` (the turret is easiest).
  - Start from your recordings with `--bc`.
  - Give it longer: the first gains (picking up FUEL, shooting in the ALLIANCE ZONE while the HUB is active) usually show within a few million decisions.
- **Climbing** isn't part of what the network controls. These robots don't climb.
- **3v3 in the game:** in a 3v3 match, set any slot's *Driven by* to **Neural net**.
