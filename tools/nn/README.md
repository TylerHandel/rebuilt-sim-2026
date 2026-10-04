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
- **Shots and passes:** timed flights with a hit chance, lower while moving fast, instead of full ballistics. Scored FUEL goes through the HUB and comes back out the exit opening into the NEUTRAL ZONE.
- **Match rules:** the real ones: AUTO, the gap, the TELEOP shifts with the active-HUB rules, the 3 s scoring grace, and human players throwing from the CHUTE. Fouls, the TOWER and BUMP slopes aren't modeled.
- **Domain randomization:** each robot's speed, acceleration, intake and accuracy, and the carpet's rolling resistance, vary a little from match to match. That way it learns habits that still work when the real game differs a bit. `--no-randomize` turns this off.

**Same network:** the observation is built exactly like the real game's. All 482 numbers match real-game snapshots in 1v1 and 3v3; the only difference is the score when fouls happened. The controls are the same too. So `js/nn/driver.json` from the GPU trainer drives in the real game as is.

**Opponents:**
- **To start:** scripted bots (80%, some of them defending) and nobody (20%).
- **Self-play:** once it beats the bots 60% of the time, it switches to:
  - 35% of matches against itself, with both alliances learning;
  - 35% against older snapshots of itself;
  - 25% against the bots;
  - 5% alone.

**Winning, not just points:** while it learns the game, the reward is the points margin. Once self-play is on, the reward shifts over 50M decisions (`--win-ramp`) toward winning:
- a squashed margin, so closing a 10-point gap in a close match counts far more than adding 10 to a blowout;
- a bonus for winning, or a penalty for losing, at the final buzzer.

The console shows the current `win-weight` (0 = points only, 1 = mostly winning). `--win-weight 1` sets it straight away.

**Live view:** the dashboard (`nn-dashboard.bat`) shows a grid of the matches the trainer is playing right now, as a bird's-eye view: robots in alliance colors, learning robots outlined, FUEL, HUB lights, score and clock. It shows 4 matches by default; `--live 0` turns it off and `--live 9` shows more, at a small cost in speed.

**Your earlier network carries over.** The observation grew for 3v3: teammates and the 2nd and 3rd opponents were added at the end. When you `--resume` a run trained before that, it's upgraded automatically with zero weights on the new inputs, so it plays exactly as before until it learns to use them. Older driving recordings still load too.

**Then fine-tune in the real game.** The two trainers save the same checkpoint format, so `nn-train.bat --resume --level 3` continues the same run (`runs/driver`) in the full game. It plays 1v1 and 3v3 there (`--teams`), with its teammates driven by the network too.

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
| `--win-ramp` | 50M | decisions over which the reward shifts to winning once self-play is on |
| `--win-weight` | | fix the win weight (0 to 1) instead |
| `--live` | 4 | matches shown on the dashboard. Each one costs a little speed; 0 turns it off. |
| `--amp` | on | the critic (the bigger network) runs in bf16 on the tensor cores. `--no-amp` turns it off. |
| `--compile` | off | `torch.compile` fuses the observation code into a few GPU kernels. On Windows it needs `.venv\Scripts\pip install triton-windows` first; without it the trainer says so and carries on normally. |

The console prints decisions per second and roughly how many matches per hour that is. On this project's 4-thread cloud CPU, with no GPU, it runs about 1,000–2,500 robot decisions/s with a few dozen matches; the full game manages about 20–50. It hasn't been measured on a GPU yet.

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
| Watch progress | `nn-dashboard.bat` opens a live page with points per match, speed, and margin and win rate against each opponent |
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
