@echo off
rem Copy your training data from an older copy of this project into this one: the training
rem (runs), the trained network (js\nn\driver.json), recordings, and the Python / Node setup
rem (.venv, node_modules) if this copy doesn't have them yet. Nothing in the old copy changes.
setlocal
cd /d "%~dp0"
echo Drag the OLD project folder into this window (or type its path), then press Enter:
set /p OLD=
set OLD=%OLD:"=%
if not exist "%OLD%\tools\nn" (echo "%OLD%" doesn't look like a copy of this project. & pause & exit /b 1)
if /i "%OLD%"=="%CD%" (echo That's this folder. Pick the old one. & pause & exit /b 1)
if exist "%OLD%\runs" (echo Copying training progress ^(runs^) ... & robocopy "%OLD%\runs" "%CD%\runs" /E /NFL /NDL /NJH /NJS /NP >nul)
if exist "%OLD%\recordings" (echo Copying recordings ... & robocopy "%OLD%\recordings" "%CD%\recordings" /E /NFL /NDL /NJH /NJS /NP >nul)
if exist "%OLD%\js\nn\driver.json" (echo Copying the trained network ... & copy /y "%OLD%\js\nn\driver.json" "%CD%\js\nn\driver.json" >nul)
if not exist ".venv\Scripts\python.exe" if exist "%OLD%\.venv\Scripts\python.exe" (echo Copying the Python setup ^(.venv^), this can take a minute ... & robocopy "%OLD%\.venv" "%CD%\.venv" /E /NFL /NDL /NJH /NJS /NP >nul)
if not exist "node_modules" if exist "%OLD%\node_modules" (echo Copying Node packages ... & robocopy "%OLD%\node_modules" "%CD%\node_modules" /E /NFL /NDL /NJH /NJS /NP >nul)
echo.
if exist "runs\driver\ckpt.pt" (echo Done. Continue training with: nn-train-gpu.bat --resume) else (echo Done, but no runs\driver\ckpt.pt was found in the old copy.)
pause
