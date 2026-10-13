@echo off
rem A network with attention and memory, in its own run (runs\memory). It looks at each robot and
rem each patch of FUEL on the field and picks out what matters right now, and it remembers what it
rem saw over the last seconds (which way an opponent is heading, whether a defender keeps following
rem it). It starts by learning from your best trained network (runs\pro, else runs\driver; the GPU
rem simulator's scripted robot if you have none), then trains past it in self-play on the GPU.
rem Running it again continues. Ctrl+C saves and stops. Extra options go after the name.
cd /d "%~dp0"
if not exist .venv\Scripts\python.exe (echo Run nn-setup.bat first. & pause & exit /b 1)
set START=--resume --selfplay
if not exist runs\memory\ckpt.pt (
  set START=--arch attn-gru --teacher bot
  if exist runs\driver\ckpt.pt set START=--arch attn-gru --teacher runs\driver --selfplay
  if exist runs\pro\ckpt.pt set START=--arch attn-gru --teacher runs\pro --selfplay
)
.venv\Scripts\python tools\nn\train_gpu.py --run memory %START% --compile --envs 4096 --mix 0 0.25 0.45 0.3 %*
pause
