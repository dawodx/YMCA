#!/usr/bin/env python3
"""YMCA Robot Rave - Launcher for Go1 Dance Demos"""

import http.server
import json
import os
import subprocess
import threading
import webbrowser
from pathlib import Path

PORT = 8889
PROJECT_ROOT = Path(__file__).parent

DEMOS = {
    "go1_keyboard": {
        "name": "Go1 Simulation (MuJoCo)",
        "script": "mo_simulation/run_go1_keyboard.py",
        "icon": "🎮",
        "description": "MuJoCo simulation - Arrows walk, 1-4 YMCA poses (needs mjpython)",
        "use_mjpython": True
    },
    "go1_unlock": {
        "name": "UNLOCK SDK Mode (Run First!)",
        "script": "mo_simulation/go1_unlock.py",
        "icon": "🔓",
        "description": "SSH into Go1 and unlock Dance/Special moves. Run this BEFORE SDK Control!",
        "use_mjpython": False
    },
    "go1_sdk": {
        "name": "Go1 SDK Control (Standard Moves)",
        "script": "mo_simulation/go1_sdk_control.py",
        "icon": "🎛️",
        "description": "Dance1, Dance2, JumpYaw, body poses & more! Run UNLOCK first for special moves.",
        "use_mjpython": False
    },
    "go1_real": {
        "name": "Go1 YMCA Dance Control",
        "script": "mo_simulation/go1_real_control.py",
        "icon": "🐕",
        "description": "Custom YMCA poses! Connect to robot WiFi first. 1-4 YMCA, 5 full dance!",
        "use_mjpython": False
    },
    "go1_dashboard": {
        "name": "Go1 Visual Dashboard",
        "script": "mo_simulation/go1_dashboard.py",
        "icon": "📊",
        "description": "Web dashboard with all commands, timing stats, LED control & action log!",
        "use_mjpython": False
    },
    "go1_sim_server": {
        "name": "Go1 Simulation Server (MuJoCo)",
        "script": "mo_simulation/go1_sim_server.py",
        "icon": "🎬",
        "description": "MuJoCo simulation with dashboard API. Run this, then use Dashboard in SIM mode!",
        "use_mjpython": True
    },
}

HTML = """<!DOCTYPE html>
<html lang="en">
<head>
    <meta charset="UTF-8">
    <meta name="viewport" content="width=device-width, initial-scale=1.0">
    <title>Team YMCA - Robot Rave</title>
    <style>
        * { margin: 0; padding: 0; box-sizing: border-box; }
        body {
            font-family: 'Segoe UI', sans-serif;
            background: linear-gradient(135deg, #1a1a2e 0%, #16213e 100%);
            min-height: 100vh;
            display: flex;
            justify-content: center;
            align-items: center;
            color: white;
        }
        .container { text-align: center; padding: 40px; max-width: 600px; }
        .logo { font-size: 72px; margin-bottom: 20px; }
        h1 { font-size: 42px; color: #ffd700; margin-bottom: 10px; }
        .tagline { font-size: 18px; color: #fff; margin-bottom: 50px; }
        .demos { display: flex; flex-direction: column; gap: 15px; }
        .demo-card {
            background: rgba(22, 33, 62, 0.8);
            border: 1px solid rgba(255, 215, 0, 0.3);
            border-radius: 12px;
            padding: 25px;
            cursor: pointer;
            transition: all 0.3s ease;
            text-align: left;
        }
        .demo-card:hover {
            background: rgba(31, 43, 71, 0.9);
            border-color: #ffd700;
            transform: translateY(-2px);
        }
        .demo-card h3 { color: #ffd700; font-size: 18px; margin-bottom: 8px; }
        .demo-card p { color: #aaa; font-size: 14px; }
        .demo-card .icon { float: right; font-size: 32px; }
        .demo-card .status { font-size: 12px; color: #4ade80; margin-top: 8px; }
        .team { margin-top: 40px; font-size: 12px; color: #666; }
    </style>
</head>
<body>
    <div class="container">
        <div class="logo">🐕🎵</div>
        <h1>Team YMCA</h1>
        <p class="tagline">Making Go1 dance like Trump at Robot Rave 2026!</p>
        <div class="demos">
            {demo_cards}
        </div>
        <p class="team">Mo | Pawit | Alex</p>
    </div>
    <script>
        async function runDemo(demoId, element) {
            document.querySelectorAll('.demo-card').forEach(card => {
                card.classList.remove('running');
                const status = card.querySelector('.status');
                if (status) status.remove();
            });
            const statusEl = document.createElement('div');
            statusEl.className = 'status';
            statusEl.textContent = 'Launching...';
            element.appendChild(statusEl);
            try {
                const response = await fetch('/run/' + demoId, { method: 'POST' });
                const data = await response.json();
                statusEl.textContent = data.success ? 'Running - check MuJoCo window!' : 'Error: ' + data.error;
            } catch (err) {
                statusEl.textContent = 'Failed to connect';
            }
        }
    </script>
</body>
</html>
"""


def generate_demo_cards():
    cards = []
    for demo_id, demo in DEMOS.items():
        card = f'''<div class="demo-card" onclick="runDemo('{demo_id}', this)">
                <span class="icon">{demo["icon"]}</span>
                <h3>{demo["name"]}</h3>
                <p>{demo["description"]}</p>
            </div>'''
        cards.append(card)
    return "\n            ".join(cards)


class Handler(http.server.BaseHTTPRequestHandler):
    def log_message(self, format, *args):
        pass

    def send_cors_headers(self):
        self.send_header("Access-Control-Allow-Origin", "*")
        self.send_header("Access-Control-Allow-Methods", "GET, POST, OPTIONS")
        self.send_header("Access-Control-Allow-Headers", "Content-Type")

    def do_OPTIONS(self):
        self.send_response(200)
        self.send_cors_headers()
        self.end_headers()

    def do_GET(self):
        if self.path == "/" or self.path == "/index.html":
            # Serve the disco launcher.html
            html_path = PROJECT_ROOT / "launcher.html"
            if html_path.exists():
                html = html_path.read_text()
                # Fix the fetch URL to be relative
                html = html.replace("http://localhost:8889/run/", "/run/")
            else:
                html = HTML.replace("{demo_cards}", generate_demo_cards())
            self.send_response(200)
            self.send_header("Content-type", "text/html")
            self.send_cors_headers()
            self.end_headers()
            self.wfile.write(html.encode())
        elif self.path.startswith("/assets/"):
            # Serve static assets (images, gifs)
            asset_path = PROJECT_ROOT / self.path[1:]  # Remove leading /
            if asset_path.exists():
                self.send_response(200)
                if self.path.endswith(".gif"):
                    self.send_header("Content-type", "image/gif")
                elif self.path.endswith(".png"):
                    self.send_header("Content-type", "image/png")
                elif self.path.endswith(".jpg") or self.path.endswith(".jpeg"):
                    self.send_header("Content-type", "image/jpeg")
                self.send_cors_headers()
                self.end_headers()
                self.wfile.write(asset_path.read_bytes())
            else:
                self.send_response(404)
                self.end_headers()
        else:
            self.send_response(404)
            self.end_headers()

    def do_POST(self):
        if self.path.startswith("/run/"):
            demo_id = self.path[5:]
            if demo_id in DEMOS:
                self.run_demo(demo_id)
            else:
                self.send_json({"success": False, "error": "Unknown demo"})
        else:
            self.send_response(404)
            self.end_headers()

    def run_demo(self, demo_id):
        demo = DEMOS[demo_id]
        script_path = PROJECT_ROOT / demo["script"]
        use_mjpython = demo.get("use_mjpython", True)

        if not script_path.exists():
            self.send_json({"success": False, "error": f"Script not found"})
            return

        if use_mjpython:
            # MuJoCo simulation - needs mjpython
            mjpython = PROJECT_ROOT / ".venv" / "bin" / "mjpython"
            if not mjpython.exists():
                self.send_json({"success": False, "error": "mjpython not found - run from ARA-Robotic venv"})
                return
            python_cmd = str(mjpython)
        else:
            # Regular Python script - runs in terminal
            python_cmd = str(PROJECT_ROOT / ".venv" / "bin" / "python")

        try:
            def run():
                if use_mjpython:
                    # Run MuJoCo in background
                    subprocess.Popen(
                        [python_cmd, str(script_path)],
                        cwd=str(PROJECT_ROOT),
                    )
                else:
                    # Open in new Terminal window for keyboard input
                    apple_script = f'''
                    tell application "Terminal"
                        activate
                        do script "cd {PROJECT_ROOT} && {python_cmd} {script_path}"
                    end tell
                    '''
                    subprocess.Popen(["osascript", "-e", apple_script])

            threading.Thread(target=run, daemon=True).start()
            self.send_json({"success": True})
        except Exception as e:
            self.send_json({"success": False, "error": str(e)})

    def send_json(self, data):
        self.send_response(200)
        self.send_header("Content-type", "application/json")
        self.send_cors_headers()
        self.end_headers()
        self.wfile.write(json.dumps(data).encode())


def main():
    os.chdir(PROJECT_ROOT)
    server = http.server.HTTPServer(("", PORT), Handler)
    print(f"\n  Team YMCA - Robot Rave Launcher")
    print(f"  ================================")
    print(f"  Running at: http://localhost:{PORT}")
    print(f"  Press Ctrl+C to stop\n")
    webbrowser.open(f"http://localhost:{PORT}")
    try:
        server.serve_forever()
    except KeyboardInterrupt:
        print("\n  Shutting down...")
        server.shutdown()


if __name__ == "__main__":
    main()
