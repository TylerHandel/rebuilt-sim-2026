@echo off
rem The strongest recipe, in its own run (runs\pro): a bigger network (1024 x 512) that starts by
rem copying a teacher, then trains past it in self-play on the GPU (8192 matches, --compile).
rem   * If you have a trained network (runs\driver), it is the teacher: the new network copies it
rem     first, so nothing it learned is lost, then keeps improving with more room to learn.
rem   * Otherwise the teacher is the GPU simulator's scripted robot.
rem Running it again continues (teacher included). Ctrl+C saves and stops. Extra options go after
rem the name, e.g. nn-train-pro.bat --envs 4096
cd /d "%~dp0"
if not exist .venv\Scripts\python.exe (echo Run nn-setup.bat first. & pause & exit /b 1)
set START=--resume --selfplay
if not exist runs\pro\ckpt.pt (
  if exist runs\driver\ckpt.pt (set START=--teacher runs\driver --hidden 1024 512 --selfplay) else (set START=--teacher bot --hidden 1024 512)
)
.venv\Scripts\python tools\nn\train_gpu.py --run pro %START% --compile --envs 8192 --mix 0 0.25 0.45 0.3 %*
pause
