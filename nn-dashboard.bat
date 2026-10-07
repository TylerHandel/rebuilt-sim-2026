@echo off
rem Opens the training dashboard in your browser (the run updated most recently; pick another there).
cd /d "%~dp0"
where py >nul 2>nul && (py -3 serve.py 8766 --nn & goto end)
python serve.py 8766 --nn
:end
pause
