#!/usr/bin/env python3
"""Live view of what the robot is doing and thinking, for the Mac (./run_dash.sh). Runs on the Jetson HOST
(no ROS, no container, Python 3.6 standard library only), so it survives restarts of the robot stack.

tools/mission_run.py writes /tmp/nontok_live.json five times a second (stage, step, what it is thinking,
its pose, the route it is about to drive, where it last located itself on station tags) and the room once
(/tmp/nontok_live_layout.json). This server streams each new snapshot to the browser as Server-Sent Events:
about 1-2 KB five times a second, no pictures, so it stays light over the Tailscale relay.

    python3 tools/live_dash.py [--port 8095]
    /          the page (tools/live_dash.html)
    /stream    the snapshots as Server-Sent Events
    /layout    the room
"""
import argparse
import json
import os
import socketserver
import time
from http.server import BaseHTTPRequestHandler, HTTPServer

HERE = os.path.dirname(os.path.abspath(__file__))
LIVE_FILE = '/tmp/nontok_live.json'
LAYOUT_FILE = '/tmp/nontok_live_layout.json'
MISSION_LOG = '/tmp/mission.log'


def read(path):
    try:
        with open(path, 'rb') as handle:
            return handle.read()
    except OSError:
        return None


def log_tail(lines=6):
    """The newest mission log lines (for a mission_run.py too old to write the live file)."""
    try:
        with open(MISSION_LOG, 'rb') as handle:
            handle.seek(0, os.SEEK_END)
            handle.seek(max(0, handle.tell() - 6000))
            text = handle.read().decode('utf-8', 'replace').splitlines()[1:]
    except OSError:
        return []
    return [line.strip() for line in text if line.strip() and 'HW ok' not in line][-lines:]


class Handler(BaseHTTPRequestHandler):
    def send_body(self, body, kind, status=200):
        self.send_response(status)
        self.send_header('Content-Type', kind)
        self.send_header('Content-Length', str(len(body)))
        self.send_header('Cache-Control', 'no-store')
        self.end_headers()
        self.wfile.write(body)

    def do_GET(self):
        path = self.path.split('?')[0]
        try:
            if path in ('/', '/index.html'):
                self.send_body(read(os.path.join(HERE, 'live_dash.html')) or b'page missing', 'text/html; charset=utf-8')
            elif path == '/layout':
                self.send_body(read(LAYOUT_FILE) or b'{}', 'application/json')
            elif path == '/stream':
                self.stream()
            else:
                self.send_body(b'not found', 'text/plain', 404)
        except (BrokenPipeError, ConnectionResetError):
            pass

    def stream(self):
        self.send_response(200)
        self.send_header('Content-Type', 'text/event-stream')
        self.send_header('Cache-Control', 'no-store')
        self.send_header('X-Accel-Buffering', 'no')
        self.end_headers()
        last_mtime, last_sent = 0.0, 0.0
        while True:
            try:
                mtime = os.path.getmtime(LIVE_FILE)
            except OSError:
                mtime = 0.0
            now = time.time()
            if mtime != last_mtime and mtime:
                body = read(LIVE_FILE)
                if body:
                    self.wfile.write(b'data: ' + body.replace(b'\n', b' ') + b'\n\n')
                    self.wfile.flush()
                    last_mtime, last_sent = mtime, now
            elif now - last_sent > 2.0:
                # nothing new: the mission is not writing (stopped, or too old a version) - say so
                stale = {'stale': True, 'age_s': round(now - mtime, 1) if mtime else None, 'log': log_tail()}
                self.wfile.write(b'data: ' + json.dumps(stale).encode() + b'\n\n')
                self.wfile.flush()
                last_sent = now
            time.sleep(0.1)

    def log_message(self, *args):
        pass


class Server(socketserver.ThreadingMixIn, HTTPServer):
    daemon_threads = True
    allow_reuse_address = True


def main():
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument('--port', type=int, default=8095)
    args = ap.parse_args()
    print('live view on 127.0.0.1:%d' % args.port, flush=True)
    Server(('127.0.0.1', args.port), Handler).serve_forever()


if __name__ == '__main__':
    main()
