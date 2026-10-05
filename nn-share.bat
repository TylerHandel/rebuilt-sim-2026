@echo off
rem Share a trained neural network so anyone can play as it or against it on the website:
rem prepares the file, then opens the GitHub upload page and the folder with the file in it.
cd /d "%~dp0"
if not exist .venv\Scripts\python.exe (echo Run nn-setup.bat first. & pause & exit /b 1)
.venv\Scripts\python tools\nn\share.py
pause
