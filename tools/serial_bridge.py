"""Make the Jetson's Mega look like a local serial port on the Mac, so Mac tools
(the teammate's probe dashboard) can use it while the Mega is plugged into the Jetson.

  python3 tools/serial_bridge.py [nvidia@10.68.194.30]          (on the Mac, from this repo)
prints a local port such as /dev/ttys012 and writes it to /tmp/mega_bridge_port; then, in ../Nontok:
  ./tools/start_console.sh --port $(cat /tmp/mega_bridge_port)   (probe dashboard, http://127.0.0.1:8766)
The Mega must run the probe firmware for that dashboard (Jetson: tools/mega_firmware.sh probe).
Needs an SSH master connection first (password login):
  ssh -fN -o ControlMaster=yes -o ControlPath=~/.ssh/cm/%C -o ControlPersist=4h nvidia@10.68.194.30

Relays bytes over SSH (reuses ~/.ssh/cm ControlPath). The Jetson side opens /dev/nontok_mega at
115200, which resets the Mega, like plugging USB into the Mac. Ctrl-C ends it and frees the port.
Baud rate and DTR set by the Mac program are ignored (the link is fixed at 115200).
"""
import os
import pty
import select
import subprocess
import sys
import tty

HOST = sys.argv[1] if len(sys.argv) > 1 else "nvidia@10.68.194.30"
REMOTE = r'''
import os, select, sys, serial
s = serial.Serial("/dev/nontok_mega", 115200, timeout=0)
out, inp = sys.stdout.buffer, sys.stdin.buffer.fileno()
while True:
    r, _, _ = select.select([s.fileno(), inp], [], [], 0.5)
    if s.fileno() in r:
        d = s.read(4096)
        if d: out.write(d); out.flush()
    if inp in r:
        d = os.read(inp, 4096)
        if not d: break
        s.write(d)
'''

master, slave = pty.openpty()
tty.setraw(slave)
name = os.ttyname(slave)
with open("/tmp/mega_bridge_port", "w") as f:
    f.write(name)
ssh = subprocess.Popen(
    ["ssh", "-T", "-o", "ControlPath=" + os.path.expanduser("~/.ssh/cm/%C"), HOST,
     "python3 -u -c '%s'" % REMOTE.replace("'", "'\\''")],
    stdin=subprocess.PIPE, stdout=subprocess.PIPE, bufsize=0)
print("Mega bridge on", name, "(Ctrl-C to stop)", flush=True)
try:
    while ssh.poll() is None:
        r, _, _ = select.select([master, ssh.stdout], [], [], 0.5)
        if master in r:
            try:
                d = os.read(master, 4096)
            except OSError:   # no program has the port open right now
                d = b""
            if d:
                ssh.stdin.write(d)
        if ssh.stdout in r:
            d = os.read(ssh.stdout.fileno(), 4096)
            if not d:
                break
            os.write(master, d)
except KeyboardInterrupt:
    pass
finally:
    ssh.terminate()
    print("bridge closed", flush=True)
