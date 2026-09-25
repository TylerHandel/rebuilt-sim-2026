#!/usr/bin/env python3
"""Local web server for the REBUILT simulator (ES modules must be served over http).

Usage:  python3 serve.py [port] [--open]
"""
import http.server
import os
import sys
import threading
import webbrowser

args = [a for a in sys.argv[1:] if not a.startswith("--")]
PORT = int(args[0]) if args else 8765
os.chdir(os.path.dirname(os.path.abspath(__file__)))


class Handler(http.server.SimpleHTTPRequestHandler):
    extensions_map = {**http.server.SimpleHTTPRequestHandler.extensions_map, ".js": "text/javascript"}

    def end_headers(self):
        # always serve the latest files while developing
        self.send_header("Cache-Control", "no-store")
        super().end_headers()

    def log_message(self, fmt, *a):
        pass


server = http.server.ThreadingHTTPServer(("127.0.0.1", PORT), Handler)
url = f"http://localhost:{PORT}/"
print(f"REBUILT Sim running at {url}  (Ctrl+C to stop)")
if "--open" in sys.argv:
    threading.Timer(0.5, lambda: webbrowser.open(url)).start()
try:
    server.serve_forever()
except KeyboardInterrupt:
    pass
