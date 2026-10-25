#!/usr/bin/env python3
"""Local web server for the REBUILT simulator (ES modules must be served over http).

Usage:  python3 serve.py [port] [--open] [--nn]   (--nn opens the neural-net training dashboard)

If the port is taken by this same folder's server, it just opens that one. If another copy of the
project (or another program) has it, it uses the next free port and says which copy is in the way,
so the page always comes from this folder. (On Windows a second server could otherwise start on a
port that's in use and quietly share it: the browser then gets some files from one copy and some
from the other, such as an older copy's robot models.)

For the training dashboard (nn.html) it also answers:
  * Range requests (bytes=N-): the dashboard reads only what a training log gained since its
    last look, instead of the whole file every time;
  * /nn-api/runs: the training runs in runs/, most recently updated first, and this folder.
"""
import http.server
import json
import os
import socket
import sys
import threading
import time
import urllib.error
import urllib.request
import webbrowser

args = [a for a in sys.argv[1:] if not a.startswith("--")]
PORT = int(args[0]) if args else 8765
ROOT = os.path.dirname(os.path.abspath(__file__))
os.chdir(ROOT)


def runs_info():
    out = []
    base = os.path.join(ROOT, "runs")
    if os.path.isdir(base):
        for name in os.listdir(base):
            d = os.path.join(base, name)
            if not os.path.isdir(d) or name.startswith("."):
                continue
            files = {f: os.path.join(d, f) for f in ("progress.jsonl", "eval.jsonl", "live.json", "ckpt.pt")}
            have = {f: os.path.isfile(p) for f, p in files.items()}
            if not any(have.values()):
                continue
            updated = max((os.path.getmtime(p) for f, p in files.items() if have[f]), default=0)
            out.append({
                "name": name, "updated": round(updated),
                "progress": os.path.getsize(files["progress.jsonl"]) if have["progress.jsonl"] else 0,
                "eval": have["eval.jsonl"], "live": have["live.json"],
                "liveUpdated": round(os.path.getmtime(files["live.json"])) if have["live.json"] else 0,
            })
    out.sort(key=lambda r: -r["updated"])
    return {"folder": ROOT, "now": round(time.time()), "runs": out}


class Handler(http.server.SimpleHTTPRequestHandler):
    extensions_map = {**http.server.SimpleHTTPRequestHandler.extensions_map, ".js": "text/javascript", ".jsonl": "text/plain"}

    def end_headers(self):
        # always serve the latest files while developing
        self.send_header("Cache-Control", "no-store")
        super().end_headers()

    def log_message(self, fmt, *a):
        pass

    def do_GET(self):
        if self.path.split("?")[0] == "/nn-api/runs":
            body = json.dumps(runs_info()).encode()
            self.send_response(200)
            self.send_header("Content-Type", "application/json")
            self.send_header("Content-Length", str(len(body)))
            self.end_headers()
            self.wfile.write(body)
            return
        rng = self.headers.get("Range", "")
        if rng.startswith("bytes="):
            path = self.translate_path(self.path)
            if os.path.isfile(path):
                return self.send_range(path, rng[6:])
        super().do_GET()

    def send_range(self, path, spec):
        """Part of a file (a growing training log): bytes=start- or bytes=start-end."""
        try:
            a, _, b = spec.partition("-")
            start = int(a)
        except ValueError:
            return super().do_GET()
        with open(path, "rb") as f:
            size = os.fstat(f.fileno()).st_size
            if start >= size:
                self.send_response(416)
                self.send_header("Content-Range", f"bytes */{size}")
                self.send_header("Content-Length", "0")
                self.end_headers()
                return
            end = min(size - 1, int(b)) if b.strip() else size - 1
            f.seek(start)
            data = f.read(end - start + 1)
        self.send_response(206)
        self.send_header("Content-Type", self.guess_type(path))
        self.send_header("Content-Range", f"bytes {start}-{start + len(data) - 1}/{size}")
        self.send_header("Content-Length", str(len(data)))
        self.end_headers()
        self.wfile.write(data)


class Server(http.server.ThreadingHTTPServer):
    # Windows' SO_REUSEADDR lets a socket bind a port another one is listening on (both then get
    # connections). Ask for the port exclusively there, so a taken port fails to bind.
    allow_reuse_address = os.name != "nt"

    def server_bind(self):
        if os.name == "nt" and hasattr(socket, "SO_EXCLUSIVEADDRUSE"):
            self.socket.setsockopt(socket.SOL_SOCKET, socket.SO_EXCLUSIVEADDRUSE, 1)
        super().server_bind()


def who_has(port):
    """The folder the server on this port serves, if it's a copy of this project new enough to say
    ('' if something else answers, None if nothing does)."""
    opener = urllib.request.build_opener(urllib.request.ProxyHandler({}))
    try:
        with opener.open(f"http://127.0.0.1:{port}/nn-api/runs", timeout=2) as r:
            return json.loads(r.read().decode()).get("folder", "")
    except Exception as e:
        return "" if isinstance(e, urllib.error.HTTPError) else None


def same(a, b):
    return os.path.normcase(os.path.abspath(a)) == os.path.normcase(os.path.abspath(b))


def bind(port):
    for p in range(port, port + 20):
        try:
            return Server(("127.0.0.1", p), Handler), p
        except OSError:
            other = who_has(p)
            if other and same(other, ROOT):
                return None, p  # this folder is already being served there
            if p == port:
                print(f"Port {port} is in use by " + (f"another copy of the project ({other})." if other else
                      "another program, or an older copy of the project (an old start.bat or nn-dashboard.bat window?).")
                      + " Its pages are not this folder's: close that window if it's an old copy.")
    raise SystemExit(f"No free port between {port} and {port + 19}")


server, port = bind(PORT)
url = f"http://localhost:{port}/"
page = url + ("nn.html" if "--nn" in sys.argv else "")
if server is None:
    print(f"This folder is already being served at {url} (another window): opening that.")
    if "--open" in sys.argv or "--nn" in sys.argv:
        webbrowser.open(page)
    sys.exit(0)
if port != PORT:
    print(f"Using port {port} instead: {url}")
print(f"REBUILT Sim running at {url}  (serving {ROOT}; Ctrl+C to stop)")
if "--open" in sys.argv or "--nn" in sys.argv:
    threading.Timer(0.5, lambda: webbrowser.open(page)).start()
try:
    server.serve_forever()
except KeyboardInterrupt:
    pass
