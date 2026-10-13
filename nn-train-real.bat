@echo off
rem Train on real matches: the full game with its real physics, every CPU thread playing matches
rem and the GPU learning. Much slower than the GPU simulator (a match takes a minute or two of a
rem CPU thread), but it learns exactly what the game rewards. Its own run (runs\real):
rem   * the first time, it starts from your best network trained in the GPU simulator (runs\memory-large,
rem     else runs\memory, runs\pro or runs\driver), so it begins from everything that learned;
rem   * then it plays the Champs AIs (Scorer, Defense, Hybrid), 1v1 and 3v3, with every robot, and
rem     its own older versions. Every robot of its ALLIANCE is the network, and it learns from each.
rem Running it again continues. Ctrl+C saves and stops. Extra options go after the name.
cd /d "%~dp0"
if not exist .venv\Scripts\python.exe (echo Run nn-setup.bat first. & pause & exit /b 1)
set FROM=
if not exist runs\real\ckpt.pt (
  if exist runs\driver\ckpt.pt set FROM=driver
  if exist runs\pro\ckpt.pt set FROM=pro
  if exist runs\memory\ckpt.pt set FROM=memory
  if exist runs\memory-large\ckpt.pt set FROM=memory-large
)
set START=--resume
if not exist runs\real\ckpt.pt (
  if defined FROM (
    echo Starting from runs\%FROM% ^(trained in the GPU simulator^) ...
    robocopy runs\%FROM% runs\real ckpt.pt /NFL /NDL /NJH /NJS /NP >nul
  ) else (
    echo No trained network to start from: a new attention + memory network learns from scratch on real matches.
    echo That is very slow; training one in the GPU simulator first ^(nn-train-memory.bat^) is much faster.
    set START=--arch attn-gru
  )
)
.venv\Scripts\python tools\nn\train.py --run real %START% --level 4 --teams 1 3 3 --robots all %*
pause
