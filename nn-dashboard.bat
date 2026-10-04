@echo off
rem Opens the live training dashboard (progress of nn-train.bat) in your browser.
cd /d "%~dp0"
where py >nul 2>nul && (py -3 serve.py 8766 --nn & goto end)
python serve.py 8766 --nn
:end
pause
