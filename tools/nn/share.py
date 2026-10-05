#!/usr/bin/env python3
"""Share a trained neural network with everyone who plays the simulator (nn-share.bat).

Prepares the network of one of your training runs (runs/<run>/policy/current.json) as a file in
the share folder, with a name and a note, copies it into js/nn/shared (so your own game lists it
right away), then opens the project's GitHub upload page and the share folder: drag the file in
and click "Commit changes". A couple of minutes later the website has it under Neural net.
"""
import json
import os
import re
import shutil
import subprocess
import sys
import time
import webbrowser
from pathlib import Path

ROOT = Path(__file__).resolve().parents[2]
REPO = 'TylerHandel/rebuilt-sim-2026'
UPLOAD = f'https://github.com/{REPO}/upload/main/js/nn/shared'


def ask(q, default=''):
    a = input(f'{q}' + (f' [{default}]' if default else '') + ': ').strip()
    return a or default


def main():
    runs = sorted((p.parent.parent.name for p in (ROOT / 'runs').glob('*/policy/current.json')), key=lambda r: -(ROOT / 'runs' / r / 'policy' / 'current.json').stat().st_mtime)
    if not runs:
        sys.exit('No trained network found (runs\\<run>\\policy\\current.json). Train one first.')
    print('Your training runs, newest first:')
    for r in runs:
        info = json.loads((ROOT / 'runs' / r / 'policy' / 'current.json').read_text()).get('info', {})
        print(f'  {r:16s} {info.get("hours", 0):6.1f} h of training, {info.get("steps", 0) / 1e6:8.1f}M decisions, saved {info.get("date", "?")}'
              + (f', robots {" ".join(info["robots"])}' if info.get('robots') else ''))
    run = ask('Which run to share', runs[0])
    src = ROOT / 'runs' / run / 'policy' / 'current.json'
    if not src.exists():
        sys.exit(f'{src} does not exist')
    d = json.loads(src.read_text())
    name = ask('Name people will see in the game', f'{run} ({time.strftime("%b %d")})')
    note = ask('A short note (what it does well, which robot; optional)')
    by = ask('Your name or team (optional)')
    info = d.get('info', {})
    d['share'] = {'name': name, 'note': note, 'by': by, 'date': time.strftime('%Y-%m-%d %H:%M'), 'robots': info.get('robots')}
    slug = re.sub(r'[^a-z0-9]+', '-', name.lower()).strip('-') or run
    out_dir = ROOT / 'share'
    out_dir.mkdir(exist_ok=True)
    out = out_dir / f'{slug}.json'
    out.write_text(json.dumps(d))
    local = ROOT / 'js' / 'nn' / 'shared' / out.name
    local.parent.mkdir(parents=True, exist_ok=True)
    shutil.copyfile(out, local)
    print(f'\nReady: {out}  ({out.stat().st_size / 1e6:.1f} MB). Your own game lists it now under Neural net.')
    print('\nTo share it with everyone:')
    print('  1. The GitHub upload page opens in your browser (sign in if it asks).')
    print(f'  2. Drag {out.name} from the folder that opens into the page.')
    print('  3. Click "Commit changes". In about 2 minutes the website has it:')
    print(f'     https://{REPO.split("/")[0].lower()}.github.io/{REPO.split("/")[1]}/  ->  Neural net')
    print('Uploading a file with the same name later replaces it (an improved version).')
    webbrowser.open(UPLOAD)
    try:
        if os.name == 'nt':
            subprocess.Popen(['explorer', '/select,', str(out)])
    except OSError:
        pass


if __name__ == '__main__':
    main()
