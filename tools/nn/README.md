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
| `tools/nn/train.py` | The trainer: PPO with GAE, observation normalization, the curriculum, self-play, imitation and checkpoints. |
| `nn.html` | The training dashboard. |

## Tips
- **Changing `js/nn/obs.js`** changes what the network sees, so old networks and recordings stop matching. Bump `OBS_VERSION` there and in `tools/nn/train.py`, then start a new run.
- **Score flat for hours:**
  - Train on one robot, for example `--robots 4414` (the turret is easiest).
  - Start from your recordings with `--bc`.
  - Give it longer: the first gains (picking up FUEL, shooting in the ALLIANCE ZONE while the HUB is active) usually show within a few million decisions.
- **Climbing** isn't part of what the network controls. These robots don't climb.
