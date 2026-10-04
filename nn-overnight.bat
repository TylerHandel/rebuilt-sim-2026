@echo off
rem Overnight training in one double-click: continues your network (runs\driver) on the GPU while
rem the CPU records the pre-programmed Champs AIs playing (recordings-ai) and plays scoreboard
rem matches. The network keeps imitating the recorded AIs a little (picking up new recordings
rem every 20 min) on top of its normal training; the imitation fades out over ~2 billion
rem decisions (a few hours). Ctrl+C saves and stops. Extra options go after the name.
cd /d "%~dp0"
if not exist .venv\Scripts\python.exe (echo Run nn-setup.bat first. & pause & exit /b 1)
set RESUME=
if exist runs\driver\ckpt.pt set RESUME=--resume --selfplay
.venv\Scripts\python tools\nn\train_gpu.py %RESUME% --compile --envs 8192 --mix 0 0.25 0.45 0.3 --bc recordings-ai --record-ai 10 --bc-weight 0.3 --bc-steps 2e9 %*
pause
