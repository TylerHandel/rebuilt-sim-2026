#!/bin/bash
# Double-click to launch the REBUILT simulator in your default browser.
cd "$(dirname "$0")"
python3 serve.py 8765 --open
