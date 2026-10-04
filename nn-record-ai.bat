@echo off
rem Record the pre-programmed AIs (Champs Scorer, Defense, Hybrid) playing full matches, 1v1 and
rem 3v3, for the neural network to learn from. Uses every CPU thread; takes roughly 15-40 minutes
rem for the default 1,000,000 decisions (Ctrl+C stops early; what's saved stays, and running it
rem again continues). Saved in recordings-ai. More: nn-record-ai.bat --samples 2000000
cd /d "%~dp0"
where node >nul 2>nul || (echo Node.js was not found. Run nn-setup.bat first. & pause & exit /b 1)
node --import ./tools/node-env.mjs tools/nn/record-ai.mjs --out recordings-ai %*
echo.
echo Next: nn-train-from-ai.bat (a new network that starts out playing like the AIs), or add
echo --bc recordings-ai to your usual training to have the current one learn from them too.
pause
