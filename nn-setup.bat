@echo off
rem One-time setup for neural-net training on Windows: Node packages + a Python environment
rem with PyTorch for NVIDIA GPUs (CUDA). Double-click it, or run it from a terminal.
cd /d "%~dp0"
echo === Checking Node.js ===
where node >nul 2>nul || (echo Node.js was not found. Install the LTS version from https://nodejs.org and run this again. & pause & exit /b 1)
node -e "process.exit(+process.versions.node.split('.')[0] >= 20 ? 0 : 1)" || (echo Node.js 20 or newer is needed. Update it from https://nodejs.org & pause & exit /b 1)
node --version
echo === Installing Node packages ===
call npm install || (echo npm install failed & pause & exit /b 1)
echo === Creating the Python environment (.venv) ===
set PY=
where py >nul 2>nul && set PY=py -3
if not defined PY where python >nul 2>nul && set PY=python
if not defined PY (echo Python 3 was not found. Install it from https://www.python.org/downloads/ and tick "Add python.exe to PATH". & pause & exit /b 1)
if not exist .venv\Scripts\python.exe %PY% -m venv .venv || (echo Could not create .venv & pause & exit /b 1)
.venv\Scripts\python -m pip install --upgrade pip
.venv\Scripts\python -m pip install numpy
echo === Installing PyTorch with CUDA (about 3 GB, takes a while) ===
.venv\Scripts\python -m pip install torch --index-url https://download.pytorch.org/whl/cu126 || (echo PyTorch install failed & pause & exit /b 1)
echo === Checking the GPU ===
.venv\Scripts\python -c "import torch, os; ok = torch.cuda.is_available(); print('CUDA GPU:', torch.cuda.get_device_name(0) if ok else 'NOT FOUND - update your NVIDIA driver'); print('CPU threads:', os.cpu_count(), '-> training workers:', os.cpu_count() - 1)"
echo.
echo Setup done. Train on the GPU with nn-train-gpu.bat (fastest), in the real game with nn-train.bat, and watch with nn-dashboard.bat.
pause
