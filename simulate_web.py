#!/usr/bin/env python3
"""
SWARM-X Web Remote Simulator
Extracts the embedded web dashboard from src/main.cpp and serves it locally
with a mock /drive endpoint for browser testing.
"""

import http.server
import re
import socketserver
from urllib.parse import parse_qs, urlparse

PORT = 8080

def get_html_from_main():
    with open("src/main.cpp", "r", encoding="utf-8") as f:
        content = f.read()
    match = re.search(r'const char PAGE\[\] PROGMEM = R"SWARMHTML\((.*?)\)SWARMHTML";', content, re.DOTALL)
    if match:
        return match.group(1).strip()
    return "<h1>Error: HTML template not found in src/main.cpp</h1>"

HTML_CONTENT = get_html_from_main()

class CarSimulatorHandler(http.server.BaseHTTPRequestHandler):
    def do_GET(self):
        parsed = urlparse(self.path)
        if parsed.path == "/" or parsed.path == "/index.html":
            self.send_response(200)
            self.send_header("Content-Type", "text/html; charset=utf-8")
            self.end_headers()
            self.wfile.write(HTML_CONTENT.encode("utf-8"))
        elif parsed.path == "/drive":
            params = parse_qs(parsed.query)
            left = params.get("l", ["0"])[0]
            right = params.get("r", ["0"])[0]
            print(f"[ESP32 Sim] HTTP drive L:{left} R:{right}")
            self.send_response(200)
            self.send_header("Content-Type", "text/plain")
            self.end_headers()
            self.wfile.write(b"OK")
        else:
            self.send_response(404)
            self.send_header("Content-Type", "text/plain")
            self.end_headers()
            self.wfile.write(b"Not found")

    def log_message(self, format, *args):
        # Suppress default request logging to keep output clean for motor logs
        return

if __name__ == "__main__":
    print(f"Starting SWARM-X Web Remote Simulator on http://localhost:{PORT}")
    print("Press Ctrl+C to stop.")
    with socketserver.TCPServer(("", PORT), CarSimulatorHandler) as httpd:
        try:
            httpd.serve_forever()
        except KeyboardInterrupt:
            print("\nShutting down simulator.")
