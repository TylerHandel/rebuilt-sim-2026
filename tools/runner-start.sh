#!/usr/bin/env bash
# Start tools/runner.mjs and keep it running: pull the latest code, run it, and if it ever stops,
# wait half a minute and start it again. Only one runs at a time (a second start exits at once).
# Meant for a login task (Windows Task Scheduler running it in WSL, see README), or by hand.
cd "$(dirname "$0")/.." || exit 1
exec 9>/tmp/sim-runner.lock
flock -n 9 || { echo "a runner is already running"; exit 0; }
export NVM_DIR="$HOME/.nvm"
[ -s "$NVM_DIR/nvm.sh" ] && . "$NVM_DIR/nvm.sh"
mkdir -p results/runner-logs
while true; do
  git pull -q --ff-only || echo "git pull failed; running the code that's here"
  node tools/runner.mjs "$@" 2>&1 | tee -a results/runner-logs/runner.log
  echo "runner stopped; starting again in 30 s"
  sleep 30
done
