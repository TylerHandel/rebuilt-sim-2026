@echo off
rem Double-click to launch the REBUILT simulator in your default browser (Windows).
cd /d "%~dp0"
where py >nul 2>nul && goto usepy
where python >nul 2>nul && goto usepython
echo Python 3 was not found.
echo Install it from https://www.python.org/downloads/ and tick "Add python.exe to PATH", then try again.
pause
exit /b 1
:usepy
py -3 serve.py 8765 --open
goto end
:usepython
python serve.py 8765 --open
:end
pause
