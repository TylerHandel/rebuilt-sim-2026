@echo off
rem Update this folder to the newest version from GitHub. Only the code is replaced: your training
rem (runs), the trained network (js\nn\driver.json), recordings and the Python/Node setup
rem (.venv, node_modules) are kept. Close the trainer and the game first.
setlocal
cd /d "%~dp0"
set ZIP=%TEMP%\rebuilt-sim-update.zip
set TMPDIR=%TEMP%\rebuilt-sim-update
echo Downloading the newest version ...
powershell -NoProfile -Command "$ProgressPreference='SilentlyContinue'; Invoke-WebRequest -Uri 'https://github.com/TylerHandel/rebuilt-sim-2026/archive/refs/heads/main.zip' -OutFile '%ZIP%'" || (echo Download failed. Check your internet connection. & pause & exit /b 1)
if exist "%TMPDIR%" rmdir /s /q "%TMPDIR%"
powershell -NoProfile -Command "Expand-Archive -Path '%ZIP%' -DestinationPath '%TMPDIR%' -Force" || (echo Could not unzip the download. & pause & exit /b 1)
echo Copying the new code over this folder (your data stays) ...
robocopy "%TMPDIR%\rebuilt-sim-2026-main" "%CD%" /E /NFL /NDL /NJH /NJS /NP /XD runs recordings recordings-ai recordings-ai-2910 .venv node_modules /XF driver.json update.bat >nul
if errorlevel 8 (echo Copy failed. & pause & exit /b 1)
rmdir /s /q "%TMPDIR%" & del "%ZIP%"
where node >nul 2>nul && (echo Updating Node packages ... & call npm install --no-audit --no-fund >nul)
echo.
echo Updated. Your training, network and recordings were kept.
echo Continue training with: nn-train-gpu.bat --resume
pause
