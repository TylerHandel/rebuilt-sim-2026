@echo off
rem The large attention + memory network (about 3.1 million weights, 4x the compute of
rem nn-train-memory.bat's), in its own run (runs\memory-large), trained in the GPU simulator. It
rem starts by learning from your best trained network (runs\memory, runs\pro or runs\driver; the
rem scripted robot if you have none), then trains past it in self-play. Afterwards, nn-train-real.bat
rem continues it on real matches. Running it again continues. Ctrl+C saves and stops.
cd /d "%~dp0"
if not exist .venv\Scripts\python.exe (echo Run nn-setup.bat first. & pause & exit /b 1)
set START=--resume --selfplay
if not exist runs\memory-large\ckpt.pt (
  set START=--arch attn-gru --size large --teacher bot
  if exist runs\driver\ckpt.pt set START=--arch attn-gru --size large --teacher runs\driver --selfplay
  if exist runs\pro\ckpt.pt set START=--arch attn-gru --size large --teacher runs\pro --selfplay
  if exist runs\memory\ckpt.pt set START=--arch attn-gru --size large --teacher runs\memory --selfplay
)
.venv\Scripts\python tools\nn\train_gpu.py --run memory-large %START% --compile --envs 2048 --mix 0 0.25 0.45 0.3 %*
pause
