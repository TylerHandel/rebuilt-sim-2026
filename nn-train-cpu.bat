@echo off
rem The network trained on real matches only: the full game with its real physics on every CPU
rem thread (the GPU just does the learning), in its own run (runs\cpu). Nothing it learns comes
rem from the simplified GPU simulator, so what it learns is what works in the real game.
rem   1. Generations (first time only, an hour or two): the scripted AI's strategy evolves in 3v3
rem      real matches. Each generation plays variations of it and moves toward the ones that won.
rem      The result is kept in runs\brain.json (the hand-tuned strategy if nothing beat it).
rem   2. A new attention + memory network learns from that AI: the AI rides along with every robot
rem      the network drives and says what it would do there. The network copies it first, then
rem      improves on it by trial and error while the AI's pull fades out (over 40M decisions).
rem   3. It keeps training against the Champs AIs and its own older versions. Every decision is
rem      judged by what it does to the rest of the match (it looks about a minute ahead).
rem Running it again continues. Ctrl+C saves and stops. Extra options go to the trainer
rem (tools\nn\train.py), for example --size large for a bigger network.
cd /d "%~dp0"
if not exist .venv\Scripts\python.exe (echo Run nn-setup.bat first. & pause & exit /b 1)
if not exist runs mkdir runs
if not exist runs\brain.json (
  echo === Step 1: evolving the scripted AI in 3v3 real matches ^(generations^) ===
  node --import ./tools/node-env.mjs tools/train.mjs --teams 3 --gens 20 --pop 10 --scenarios 6 --validate 24 --out runs\brain.json || (echo Evolving the AI failed. & pause & exit /b 1)
)
echo === Training the network on real matches ===
set START=--resume
if not exist runs\cpu\ckpt.pt set START=--arch attn-gru
.venv\Scripts\python tools\nn\train.py --run cpu %START% --teacher scorer --teacher-brain runs\brain.json --level 4 --teams 1 3 3 --robots all --gamma 0.999 %*
pause
