#!/usr/bin/env python3
"""
Unitree A2 Wi-Fi teleop: LAPTOP side. Window with the live front camera, combined keys and speed levels.

It starts the robot side (a2_teleop_server.py, on PC2) over SSH with a tunnel, so it works on the
robot's Wi-Fi (A2-4G / Unitree AP) and over Tailscale. The robot side runs DETACHED on PC2: if the
link drops it keeps guarding the robot, and this window reconnects to it by itself.

Run in an a2py terminal (never with sudo), laptop with a route to 192.168.123.0/24:
    a2py
    python3 ~/unitree_a2/teleop/a2_teleop_wifi_gui.py --dry-run   # window + camera + alarm, NO motion commands
    python3 ~/unitree_a2/teleop/a2_teleop_wifi_gui.py             # real teleop
    python3 ~/unitree_a2/teleop/a2_teleop_wifi_gui.py --reset     # end an old robot-side session first (it lies down)

Controls (click the window once so it has keyboard focus). Keys can be COMBINED:
    W / S   forward / backward
    A / D   sideways left / right
    Q / E   turn left / right          e.g. W+Q = forward while turning, W+A = diagonal
    1 - 5   speed level (2 = yesterday's speeds; 5 = fastest allowed)
    Space   stop now; motion stays blocked until you release all movement keys
    T       hold to talk: your laptop mic plays on the robot's speaker (release = stop)
    L       robot microphone on/off on your laptop speakers (on at start; muted while T is held)
    R       re-arm driving after "LINK RECOVERED" (release all keys first)
    ESC     stop, lie down (StandDown) and exit. Closing the window or Ctrl+C does the same.

LINK GUARD (enforced on PC2): short glitches only pause the robot (commands missing > 0.5 s; it carries
on when they return). A problem lasting MORE THAN 1.6 s (no heartbeat reply, no commands, or latency
> 2 x normal + 100 ms, ignoring good dips <= 0.3 s) is a LOSS: the robot stops, stands still and loops
the "Danger loosing connection" alarm. When latency is back to its pre-loss level for 3 s the alarm
stops; press R to drive again. No stable link for 5 min -> the robot lies down (alarm continues).

Talk and listen run on their own SSH links (a2_talk*.py, a2_listen*.py), separate from motion, so an
audio problem never affects driving. --no-audio / --no-listen turn them off; --volume / --gain /
--listen-gain tune them.
"""

import argparse
import json
import math
import os
import socket
import struct
import subprocess
import sys
import threading
import time

os.environ.setdefault("PYGAME_HIDE_SUPPORT_PROMPT", "1")
try:
    import cv2
    import numpy as np
    import pygame
except ImportError as e:
    print(f"ERROR: {e}\nRun 'a2py' first so the SDK virtualenv is active (it has pygame and cv2).")
    sys.exit(1)
from a2_talk import TalkLink   # same folder: laptop mic -> robot speaker
from a2_listen import ListenLink   # same folder: robot mic -> laptop speakers

# ============================================================================
# CONFIGURATION
# ============================================================================
SSH_HOST = "a2-pc2"
REMOTE_DIR = "~/a2_wireless"
REMOTE_PORT = 9500           # a2_teleop_server.py listens here on PC2 (127.0.0.1 only)
LOCAL_PORT = 9501            # laptop end of the SSH tunnel

# Speed levels: (vx m/s, vy m/s, vyaw rad/s). PC2 caps at 0.50 / 0.30 / 0.80 whatever is sent.
SPEED_LEVELS = {1: (0.10, 0.08, 0.20),
                2: (0.20, 0.15, 0.30),     # = a2_teleop.py first-test values
                3: (0.30, 0.20, 0.45),
                4: (0.40, 0.25, 0.60),
                5: (0.50, 0.30, 0.80)}
DEFAULT_LEVEL = 2

SEND_RATE_HZ = 20            # "vel" messages to PC2 (its watchdog stops the robot after 0.5 s without one)
LINK_WARN = 1.0              # s without any message from PC2 -> "LINK LOST" on screen
LINK_DEAD = 4.0              # s without any message -> drop the connection and reconnect

WIN_W, WIN_H = 960, 540
ZERO = (0.0, 0.0, 0.0)
RED, YELLOW, PINK = (255, 60, 60), (255, 210, 0), (255, 80, 200)


# ============================================================================
# Pure helpers
# ============================================================================
def compute_target(held, level):
    """Held keys + speed level -> (vx, vy, vyaw). Opposite keys cancel; diagonals not faster."""
    mx, my, myaw = SPEED_LEVELS[level]
    vx = mx * (int(held["W"]) - int(held["S"]))
    vy = my * (int(held["A"]) - int(held["D"]))
    vyaw = myaw * (int(held["Q"]) - int(held["E"]))
    lin = math.hypot(vx, vy)
    if lin > mx:
        k = mx / lin
        vx, vy = vx * k, vy * k
    return (vx, vy, vyaw)


# ============================================================================
# Link to PC2
# ============================================================================
class Shared:
    def __init__(self):
        self.lock = threading.Lock()
        self.stop = threading.Event()        # window is closing for good
        self.status = {}
        self.status_time = 0.0
        self.frame = None            # RGB numpy array, WIN_W x WIN_H
        self.frame_count = 0
        self.first_status = threading.Event()
        self.link_down = False       # laptop-side: connection to PC2 lost, reconnecting
        self.reconnect_msg = ""
        self.session_gone = False    # PC2 side no longer running


class Conn:
    """The current socket to PC2 (replaced on reconnect). Sends never raise."""

    def __init__(self):
        self.lock = threading.Lock()
        self.sock = None

    def set(self, sock):
        with self.lock:
            old, self.sock = self.sock, sock
        if old is not None:
            try:
                old.close()
            except OSError:
                pass

    def send(self, msg):
        with self.lock:
            if self.sock is None:
                return False
            try:
                self.sock.sendall((json.dumps(msg) + "\n").encode())
                return True
            except OSError:
                return False

    def close(self):
        self.set(None)


def recv_exact(sock, n):
    buf = b""
    while len(buf) < n:
        chunk = sock.recv(n - len(buf))
        if not chunk:
            raise ConnectionError("PC2 closed the connection")
        buf += chunk
    return buf


def receiver_loop(sock, conn: Conn, sh: Shared):
    try:
        while not sh.stop.is_set():
            kind = recv_exact(sock, 1)
            (length,) = struct.unpack(">I", recv_exact(sock, 4))
            payload = recv_exact(sock, length)
            now = time.monotonic()
            if kind == b"J":
                msg = json.loads(payload)
                if "hb" in msg:                          # heartbeat: echo at once (PC2 measures RTT)
                    conn.send({"t": "hb", "id": msg["hb"]})
                    with sh.lock:
                        sh.status_time = now
                    continue
                with sh.lock:
                    sh.status.update(msg)
                    sh.status_time = now
                sh.first_status.set()
            elif kind == b"F":
                img = cv2.imdecode(np.frombuffer(payload, np.uint8), cv2.IMREAD_COLOR)
                if img is not None:
                    if img.shape[1] != WIN_W or img.shape[0] != WIN_H:
                        img = cv2.resize(img, (WIN_W, WIN_H))
                    rgb = cv2.cvtColor(img, cv2.COLOR_BGR2RGB)
                    with sh.lock:
                        sh.frame = rgb
                        sh.frame_count += 1
                        sh.status_time = now
    except (OSError, ConnectionError, ValueError) as e:
        with sh.lock:
            if not sh.stop.is_set():
                sh.link_down = True
                sh.reconnect_msg = f"connection lost ({e})"


def start_robot_side(dry_run=False, no_video=False, reset=False, attach_only=False, brake_args=""):
    """Start (or attach to) a2_teleop_server.py on PC2, with an SSH tunnel.

    Returns (process, mode) with mode "new", "attach", or None/"none" on failure / no session.
    The server runs detached (setsid) with its log in ~/a2_wireless/teleop_server.log;
    this SSH session only carries the tunnel and shows the log.
    """
    args = " --dry-run" if dry_run else ""
    args += " --no-video" if no_video else ""
    args += brake_args
    # The name is split (S=..._server) so pgrep never matches this shell's own command line.
    script = f"""cd {REMOTE_DIR} || exit 3
S=a2_teleop; P="python3 -u [a]2_teleop_server"
if [ {int(reset)} = 1 ] && pgrep -f "$P" >/dev/null; then
  echo "RESET: ending the old robot-side session (it lies the robot down)"; pkill -TERM -f "$P"
  for i in $(seq 1 150); do pgrep -f "$P" >/dev/null || break; sleep 0.1; done
fi
if pgrep -f "$P" >/dev/null; then echo "ATTACH"; exec tail -n 3 -F teleop_server.log; fi
if [ {int(attach_only)} = 1 ]; then echo "NO_SESSION"; exit 0; fi
: > teleop_server.log
PYTHONPATH={REMOTE_DIR} setsid nohup python3 -u ${{S}}_server.py{args} > teleop_server.log 2>&1 < /dev/null &
echo "STARTED"; exec tail -n +1 -F teleop_server.log"""
    cmd = ["ssh", "-o", "BatchMode=yes", "-o", "ExitOnForwardFailure=yes", "-o", "ConnectTimeout=6",
           "-o", "ServerAliveInterval=2", "-o", "ServerAliveCountMax=3",
           "-L", f"{LOCAL_PORT}:127.0.0.1:{REMOTE_PORT}", SSH_HOST, script]
    proc = subprocess.Popen(cmd, stdin=subprocess.DEVNULL, stdout=subprocess.PIPE, stderr=subprocess.STDOUT,
                            text=True, bufsize=1,   # DEVNULL: ssh must not eat the typed "stand"
                            start_new_session=True)  # Ctrl+C reaches only this window, which then quits cleanly
    mode = None
    t0 = time.monotonic()
    while time.monotonic() - t0 < 30:
        line = proc.stdout.readline()
        if not line:
            break
        line = line.rstrip()
        if line == "NO_SESSION":
            mode = "none"
            break
        if line == "ATTACH":
            mode = "attach"
            break
        print(f"[PC2] {line}")
        if "READY" in line:
            mode = "new"
            break
    if mode in ("new", "attach"):
        threading.Thread(target=lambda: [print(f"[PC2] {l.rstrip()}") for l in proc.stdout],
                         daemon=True).start()
        return proc, mode
    try:
        proc.terminate()
    except OSError:
        pass
    return None, mode


def connect_tunnel():
    for _ in range(50):
        try:
            sock = socket.create_connection(("127.0.0.1", LOCAL_PORT), timeout=2)
            sock.settimeout(None)
            sock.setsockopt(socket.IPPROTO_TCP, socket.TCP_NODELAY, 1)
            return sock
        except OSError:
            time.sleep(0.1)
    return None


class Reconnector:
    """Keeps the link to the (detached) PC2 side alive: reattaches after drops."""

    def __init__(self, conn: Conn, sh: Shared, proc):
        self.conn, self.sh, self.proc = conn, sh, proc
        self.tries = 0
        threading.Thread(target=self._run, daemon=True).start()

    def _kill_proc(self):
        if self.proc is not None:
            try:
                self.proc.terminate()
                self.proc.wait(timeout=3)
            except (OSError, subprocess.TimeoutExpired):
                pass
            self.proc = None

    def _run(self):
        sh = self.sh
        while not sh.stop.is_set():
            with sh.lock:
                down = sh.link_down
            if not down:
                time.sleep(0.2)
                continue
            self.conn.close()
            self._kill_proc()
            self.tries += 1
            with sh.lock:
                sh.reconnect_msg = f"reconnecting to PC2 (try {self.tries})..."
            print(f"Link to PC2 lost - reconnecting (try {self.tries})...")
            proc, mode = start_robot_side(attach_only=True)
            if mode == "none":
                with sh.lock:
                    sh.session_gone = True
                    sh.reconnect_msg = "robot-side session has ended"
                print("The robot-side session has ended (nothing to reconnect to).")
                return
            if proc is None:
                time.sleep(2.0)
                continue
            sock = connect_tunnel()
            if sock is None:
                try:
                    proc.terminate()
                except OSError:
                    pass
                time.sleep(2.0)
                continue
            self.proc = proc
            self.conn.set(sock)
            with sh.lock:
                sh.link_down = False
                sh.reconnect_msg = ""
                sh.status_time = time.monotonic()
            threading.Thread(target=receiver_loop, args=(sock, self.conn, sh), daemon=True).start()
            print(f"Reconnected to PC2 (try {self.tries}).")

    def close(self):
        self._kill_proc()


# ============================================================================
# Window
# ============================================================================
def guard_lines(st, big, small, link_down, reconnect_msg):
    """Banners for the link guard (empty list when everything is normal)."""
    phase = st.get("phase", "")
    rtt, normal = st.get("rtt_ms"), st.get("normal_ms")
    lat = f"latency {'--' if rtt is None else rtt} ms (normal {normal} ms)"
    out = []
    if link_down:
        out.append((big, "CONNECTION LOST - robot stopped, alarm on (PC2 guard)", RED))
        out.append((small, reconnect_msg or "reconnecting...", RED))
    elif phase == "unstable":
        left = max(0, int(st.get("backstop_s", 300) - st.get("unstable_s", 0)))
        out.append((big, "LINK UNSTABLE - robot stopped, alarm on", RED))
        out.append((small, f"{st.get('reason', '')} | {lat} | stable {st.get('stable_for', 0):.1f}/3.0 s"
                           f" | lies down in {left // 60}:{left % 60:02d}", RED))
    elif phase == "backstop":
        out.append((big, "NO STABLE LINK FOR 5 MIN - robot lying down, alarm on", RED))
        out.append((small, f"{lat} | stable {st.get('stable_for', 0):.1f}/3.0 s", RED))
    elif phase == "recovered":
        out.append((big, "LINK RECOVERED - release all keys, press R to drive", YELLOW))
    elif phase == "done" and st.get("reason"):
        out.append((big, "SESSION ENDED - robot lying down. Press ESC.", YELLOW))
    return out


def lidar_lines(st, big, small):
    """LiDAR brake status from the robot side (empty if the server is too old to report it)."""
    li = st.get("lidar")
    if not isinstance(li, dict):
        return []
    if not li.get("on"):
        return [(small, "LiDAR brake OFF - nothing stops you driving into obstacles", YELLOW)]
    front = li.get("front")
    dist = "--" if front is None else ("clear" if front >= 9 else f"{front:.2f} m")
    state = li.get("state")
    if state == "no data":
        return [(big, "NO LIDAR DATA - ALL motion frozen", RED),
                (small, "start obstacle_sectors.py on PC2 (or use --no-lidar-brake)", RED)]
    if state == "freeze":
        where = li.get("where") or "near"
        names = {"fwd": "forward (W)", "back": "back (S)", "left": "left (A)", "right": "right (D)",
                 "turn_left": "turn left (Q)", "turn_right": "turn right (E)"}
        blocked = [names.get(m, m) for m in li.get("blocked", [])]
        if len(blocked) >= 6:
            return [(big, f"PERIMETER - object {where}, ALL motion blocked", RED),
                    (small, "move the object away (or use the remote)", RED)]
        return [(big, f"PERIMETER - object {where}", RED),
                (small, ("blocked: " + ", ".join(blocked) if blocked else "nothing blocked")
                 + " - drive away from it", RED)]
    if li.get("release_keys"):
        return [(big, "PERIMETER CLEAR - release all keys to drive again", YELLOW)]
    if state == "stop":
        return [(big, f"LIDAR BRAKE - obstacle {dist} ahead, forward blocked", RED),
                (small, "turn (A/D, Q/E) or back up (S) to get clear", RED)]
    if state == "slow":
        return [(small, f"LiDAR: obstacle {dist} ahead - forward slowed"
                        + (" (braking)" if li.get("active") else ""), YELLOW)]
    return [(small, f"LiDAR brake on - front {dist}", (0, 210, 0))]


def draw(screen, fonts, sh: Shared, held, focused, latched, level, fps, dry_run, talk, listen=None):
    big, small = fonts
    now = time.monotonic()
    with sh.lock:
        frame = sh.frame
        st = dict(sh.status)
        age = now - sh.status_time
        link_down, reconnect_msg = sh.link_down, sh.reconnect_msg
    if frame is not None:
        screen.blit(pygame.image.frombuffer(frame.tobytes(), (WIN_W, WIN_H), "RGB"), (0, 0))
    else:
        screen.fill((20, 20, 20))

    vx, vy, vyaw = st.get("vel", ZERO)
    moving = bool(st.get("moving"))
    color = (255, 165, 0) if moving else (0, 210, 0)
    status = "MOVING" if moving else "STOPPED"
    if dry_run:
        status += "  [DRY RUN - no motion sent]"
    mx, my, myaw = SPEED_LEVELS[level]
    rtt, normal = st.get("rtt_ms"), st.get("normal_ms")
    lines = [(big, status, color),
             (small, f"speed level {level}: max {mx:.2f} m/s fwd, {my:.2f} side, {myaw:.2f} rad/s turn", color),
             (small, f"robot cmd: vx {vx:+.2f}  vy {vy:+.2f}  yaw {vyaw:+.2f}   cam {fps:4.1f} fps   "
                     f"latency {'--' if rtt is None else rtt} ms (normal {normal})   link {age:3.1f} s", color)]
    lines += guard_lines(st, big, small, link_down, reconnect_msg)
    lines += lidar_lines(st, big, small)
    if age > LINK_WARN and not link_down and st.get("phase") == "teleop":
        lines.append((big, "LINK SLOW - robot stops by itself if it gets worse", RED))
    if not focused:
        lines.append((big, "CLICK THIS WINDOW TO CONTROL (no focus = stopped)", RED))
    if latched:
        lines.append((small, "STOP latched: release all movement keys to drive again", RED))
    for key in ("fault", "video_error"):
        if st.get(key):
            lines.append((small, str(st[key]), RED))
    if talk is not None and talk.active:
        lines.append((big, "MIC ON - robot speaking", PINK))
    elif talk is not None and not talk.alive:
        lines.append((small, "audio link down (driving not affected)", RED))
    if listen is not None and not listen.receiving:
        lines.append((small, "robot mic: no audio (driving not affected)", RED))
    y = 10
    for font, text, col in lines:
        screen.blit(font.render(text, True, col), (12, y))
        y += font.get_linesize() + 2

    x = 12
    for name in "WSADQE":
        col = (255, 165, 0) if held[name] else (90, 90, 90)
        screen.blit(big.render(name, True, col), (x, WIN_H - 70))
        x += 34
    if talk is not None:
        screen.blit(big.render("T", True, PINK if talk.active else (90, 90, 90)), (x + 20, WIN_H - 70))
    if listen is not None:
        on = not listen.muted
        screen.blit(big.render("L", True, (0, 190, 255) if on else (90, 90, 90)), (x + 54, WIN_H - 70))
        screen.blit(small.render("robot mic ON" if on else "robot mic off", True,
                                 (0, 190, 255) if on else (150, 150, 150)), (x + 80, WIN_H - 64))
    help_text = "Hold & combine W/S A/D Q/E | 1-5 speed | Space stop | R re-arm | ESC lie down + exit"
    if talk is not None:
        help_text += " | hold T talk"
    if listen is not None:
        help_text += " | L mic"
    screen.blit(small.render(help_text, True, (255, 255, 255)), (12, WIN_H - 30))
    pygame.display.flip()


def run_window(conn: Conn, sh: Shared, dry_run, talk=None, listen=None):
    pygame.init()
    screen = pygame.display.set_mode((WIN_W, WIN_H))
    pygame.display.set_caption("A2 Wi-Fi teleop - click here, then hold keys")
    fonts = (pygame.font.Font(None, 34), pygame.font.Font(None, 24))
    clock = pygame.time.Clock()
    latched = False
    level = DEFAULT_LEVEL
    fps, last_count, last_t = 0.0, 0, time.monotonic()
    last_send = 0.0
    listen_on = listen is not None       # what the L key chose (talk mutes temporarily on top of this)
    keymap = {"W": pygame.K_w, "S": pygame.K_s, "A": pygame.K_a,
              "D": pygame.K_d, "Q": pygame.K_q, "E": pygame.K_e}
    levelkeys = {pygame.K_1: 1, pygame.K_2: 2, pygame.K_3: 3, pygame.K_4: 4, pygame.K_5: 5,
                 pygame.K_KP1: 1, pygame.K_KP2: 2, pygame.K_KP3: 3, pygame.K_KP4: 4, pygame.K_KP5: 5}
    try:
        while not sh.stop.is_set():
            pressed = pygame.key.get_pressed()
            focused = pygame.key.get_focused()
            held = {n: bool(pressed[k]) and focused for n, k in keymap.items()}
            for ev in pygame.event.get():
                if ev.type == pygame.QUIT:
                    print("Window closed - stopping and lying down")
                    return
                if ev.type == pygame.KEYDOWN:
                    if ev.key == pygame.K_ESCAPE:
                        print("ESC - stopping and lying down")
                        return
                    if ev.key == pygame.K_SPACE:
                        latched = True
                        conn.send({"t": "stop"})
                        print("STOP")
                    if ev.key in levelkeys:
                        level = levelkeys[ev.key]
                        print(f"Speed level {level}: {SPEED_LEVELS[level]}")
                    if ev.key == pygame.K_l and listen is not None:
                        listen_on = not listen_on
                        print("Robot mic " + ("ON" if listen_on else "off"))
                    if ev.key == pygame.K_r:
                        with sh.lock:
                            phase = sh.status.get("phase")
                        if phase != "recovered":
                            print(f"R ignored (link guard phase: {phase})")
                        elif any(held.values()):
                            print("R ignored: release all movement keys first")
                        else:
                            conn.send({"t": "arm"})
                            print("Re-arm sent (R)")

            with sh.lock:
                session_gone = sh.session_gone
                age = time.monotonic() - sh.status_time
                link_down = sh.link_down
                phase = sh.status.get("phase")
            if session_gone:
                print("Robot-side session ended - closing the window.")
                return
            if phase == "done":
                print("PC2 ended the session (robot lying down) - closing the window.")
                time.sleep(2.0)
                return
            if age > LINK_DEAD and not link_down:          # silent dead link: force a reconnect
                with sh.lock:
                    sh.link_down = True
                    sh.reconnect_msg = f"no data from PC2 for {age:.0f} s"

            if latched and not any(held.values()):
                latched = False
            target = compute_target(held, level) if (focused and not latched) else ZERO

            if talk is not None:                 # push-to-talk; never touches the motion link
                want_talk = focused and bool(pressed[pygame.K_t])
                if want_talk and not talk.active:
                    talk.talk_on()
                elif not want_talk and talk.active:
                    talk.talk_off()
            if listen is not None:               # never play the robot mic while talking (no feedback howl)
                want_muted = not listen_on or (talk is not None and talk.active)
                if want_muted != listen.muted:
                    listen.set_muted(want_muted)

            now = time.monotonic()
            if now - last_send >= 1.0 / SEND_RATE_HZ:      # always sent: PC2 also uses it as a link check
                conn.send({"t": "vel", "vx": target[0], "vy": target[1], "vyaw": target[2]})
                last_send = now
            with sh.lock:
                count = sh.frame_count
            if now - last_t >= 1.0:
                fps, last_count, last_t = (count - last_count) / (now - last_t), count, now
            draw(screen, fonts, sh, held, focused, latched, level, fps, dry_run, talk, listen)
            clock.tick(30)
    finally:
        if talk is not None:
            talk.talk_off()
        pygame.quit()


def main():
    ap = argparse.ArgumentParser(description="A2 Wi-Fi teleop window (robot side runs on PC2)")
    ap.add_argument("--dry-run", action="store_true", help="window + camera + alarm only, NO motion commands")
    ap.add_argument("--no-video", action="store_true", help="don't fetch camera frames")
    ap.add_argument("--reset", action="store_true", help="end an old robot-side session first (it lies down)")
    ap.add_argument("--no-audio", action="store_true", help="no push-to-talk (T key)")
    ap.add_argument("--volume", type=int, default=70, help="robot speaker volume for talk, 0-100 (default 70)")
    ap.add_argument("--gain", type=float, default=2.0, help="mic boost for talk (default 2.0)")
    ap.add_argument("--no-listen", action="store_true", help="don't play the robot microphone")
    ap.add_argument("--listen-gain", type=float, default=1.0, help="robot mic playback boost (default 1.0)")
    ap.add_argument("--brake-stop", type=float, help="LiDAR brake: block forward at this front distance, m (PC2 default 0.25)")
    ap.add_argument("--brake-slow", type=float, help="LiDAR brake: start slowing at this front distance, m (PC2 default 0.45)")
    ap.add_argument("--no-lidar-brake", action="store_true", help="drive WITHOUT the LiDAR brake")
    args = ap.parse_args()

    print("=" * 70)
    print("Unitree A2 Wi-Fi teleop" + ("  [DRY RUN - no motion commands]" if args.dry_run else ""))
    print("Speed levels (vx, vy m/s; yaw rad/s): " + "  ".join(f"{k}={v}" for k, v in SPEED_LEVELS.items()))
    print("=" * 70)

    print("Starting the robot side on PC2 over SSH...")
    brake_args = (f" --brake-stop {args.brake_stop:g}" if args.brake_stop is not None else "") \
        + (f" --brake-slow {args.brake_slow:g}" if args.brake_slow is not None else "") \
        + (" --no-lidar-brake" if args.no_lidar_brake else "")
    proc, mode = start_robot_side(args.dry_run, args.no_video, reset=args.reset, brake_args=brake_args)
    if proc is None:
        print("ERROR: PC2 side did not start. Route to 192.168.123.0/24 (robot Wi-Fi or Tailscale)?"
              " ~/a2_wireless on PC2? See ~/a2_wireless/teleop_server.log on PC2.")
        sys.exit(1)

    conn = Conn()
    sock = connect_tunnel()
    if sock is None:
        print("ERROR: could not connect through the SSH tunnel.")
        proc.terminate()
        sys.exit(1)
    conn.set(sock)
    sh = Shared()
    threading.Thread(target=receiver_loop, args=(sock, conn, sh), daemon=True).start()
    sh.first_status.wait(timeout=5)
    with sh.lock:
        fsm = sh.status.get("robot_state", "?")
        phase = sh.status.get("phase", "?")
        remote_dry = sh.status.get("dry_run")
    reconnector = Reconnector(conn, sh, proc)

    def shutdown_link():
        sh.stop.set()
        reconnector.close()
        conn.close()

    if mode == "attach":
        print(f"Attached to an EXISTING robot-side session (phase: {phase}, dry run: {remote_dry}).")
        if remote_dry is not None and bool(remote_dry) != args.dry_run:
            print("!! That session's dry-run setting differs from what you asked for. Not attaching.\n"
                  "   Use --reset to end it (the robot lies down) and start fresh.")
            shutdown_link()
            sys.exit(1)

    talk = None
    if not args.no_audio:
        print("Starting push-to-talk audio on PC2 (separate link)...")
        talk = TalkLink(args.volume, args.gain)
        if not talk.start():
            print("WARNING: audio did not start. Driving works; the T key is disabled.")
            talk.close()
            talk = None

    listen = None
    if not args.no_listen:
        print("Connecting to the robot microphone (separate link)...")
        listen = ListenLink(args.listen_gain)
        if not listen.start():
            print("WARNING: no audio from the robot mic. Driving works; the L key is disabled.")
            listen.close()
            listen = None

    def close_audio():
        for a in (talk, listen):
            if a is not None:
                a.close()

    if not (mode == "attach" and phase in ("teleop", "unstable", "recovered", "backstop")):
        print(f"Robot state: {fsm}")
        print("""
PRE-MOTION CHECKLIST
  [ ] 2 m clear around the robot, flat floor, nobody within reach
  [ ] Remote controller ON and in your hand (it is the e-stop)
  [ ] You can SEE the robot, not only the camera (Wi-Fi video lags)
  [ ] You know: ESC = stop + lie down; release keys = stop; Space = stop
  [ ] Link guard: link problem > 1.6 s -> robot stops + alarm; R re-arms after recovery
""")
        if fsm != "STAND_DOWN":
            print(f"NOTE: robot state is {fsm}, not STAND_DOWN. It will be sent StandUp/BalanceStand anyway.")
        try:
            answer = input("Type 'stand' and press Enter to stand the robot up (anything else quits): ")
        except (KeyboardInterrupt, EOFError):
            answer = ""
        if answer.strip().lower() != "stand":
            print("Quit before standing. Nothing was sent.")
            close_audio()
            conn.send({"t": "quit"})
            time.sleep(0.5)
            shutdown_link()
            return

        conn.send({"t": "stand"})
        print("Standing up... (remote in hand)")
        t0 = time.monotonic()
        while time.monotonic() - t0 < 15:
            with sh.lock:
                phase = sh.status.get("phase", "")
            if phase == "teleop" or phase.startswith("stand failed") or sh.link_down:
                break
            time.sleep(0.1)
        print(f"Robot side: {phase}")
        if phase != "teleop":
            print("Not in teleop. PC2 lies the robot down and exits.")
            close_audio()
            time.sleep(2.0)
            shutdown_link()
            return

    print("Teleop running. Click the window, then hold (and combine) keys. 1-5 = speed. R = re-arm."
          + (" Hold T to talk." if talk is not None else "") + (" L = robot mic on/off." if listen else ""))
    try:
        run_window(conn, sh, args.dry_run, talk, listen)
    except KeyboardInterrupt:
        print("Ctrl+C - stopping and lying down")
    finally:
        with sh.lock:
            link_down = sh.link_down
            phase = sh.status.get("phase", "")
        delivered = conn.send({"t": "quit"}) if not link_down else False   # motion first: lie down
        close_audio()
        if delivered:
            print("Waiting for PC2 to stop and lie the robot down...")
            t0 = time.monotonic()
            while time.monotonic() - t0 < 15:
                with sh.lock:
                    phase = sh.status.get("phase", "")
                if phase == "done" or sh.link_down:
                    break
                time.sleep(0.1)
        if phase != "done":
            print("!! PC2 did not confirm the lie-down. If the link is down, the robot stays STOPPED with the\n"
                  "   alarm, and lies down by itself after 5 min. Run this program again to reconnect, or use\n"
                  "   the REMOTE to lie it down now.")
        shutdown_link()


if __name__ == "__main__":
    main()
