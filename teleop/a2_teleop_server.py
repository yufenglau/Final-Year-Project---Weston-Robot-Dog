#!/usr/bin/env python3
"""
Unitree A2 Wi-Fi teleop: ROBOT side. Runs ON PC2 (.162); the laptop window (a2_teleop_wifi_gui.py)
starts it over SSH and talks to it through an SSH tunnel (it listens on 127.0.0.1 only).

Why on PC2: over Wi-Fi/Tailscale only SSH reaches the robot network reliably, and DDS from the
laptop does not work through NAT. This program talks DDS on PC2's wired eth0.

The laptop starts it DETACHED (setsid, output in ~/a2_wireless/teleop_server.log), so it keeps
guarding the robot if the SSH session or Wi-Fi drops, and the laptop can reconnect to it.

Protocol (TCP; a new laptop connection replaces the old one):
    laptop -> PC2: JSON lines  {"t":"stand"} | {"t":"vel","vx":..,"vy":..,"vyaw":..} | {"t":"stop"}
                               {"t":"hb","id":n} (heartbeat echo) | {"t":"arm"} (R key) | {"t":"quit"}
    PC2 -> laptop: frames      1 byte type (b"J" status JSON / b"F" JPEG) + 4 byte big-endian length + payload
                               heartbeats are J frames {"hb": n}, echoed straight back by the laptop

Guard rails (enforced HERE, whatever the laptop sends):
    * Absolute speed caps ABS_MAX_*; combined linear speed capped at ABS_MAX_LINEAR.
    * Speed-ups ramped; slowing down and stopping immediate; reversing stops first.
    * Watchdog: no "vel" message for WATCHDOG (0.5) s -> robot pauses until commands return.
    * LINK GUARD (LinkMonitor): heartbeat round-trip time (RTT) every 0.2 s. "Normal" latency =
      median RTT over the last 10 s of stable driving, frozen when the link goes bad.
        (any gap in "vel" > 0.5 s already pauses the robot; it resumes when commands return, no alarm)
        unstable  = a problem lasting MORE THAN 1.6 s: no "vel" for 1.6 s, OR no heartbeat reply for 1.6 s,
                    OR RTT > 2 x normal + 100 ms for 1.6 s (good dips <= 0.3 s inside it don't reset
                    the count), OR the laptop disconnected
                    -> robot stops and STANDS STILL, alarm WAV loops on the robot speaker
        stable    = for 3 s in a row: median RTT <= normal + 20 % (at least + 20 ms), heartbeats and
                    "vel" arriving -> alarm stops, motion stays locked until the laptop sends "arm"
                    (R key), and a zero "vel" must arrive before any motion
        backstop  = no stable link for 5 min -> StandDown (alarm continues until the link is back)
    * LIDAR BRAKE (lidar_brake): reads the clearances obstacle_sectors.py sends to UDP 127.0.0.1:9600.
      Only FORWARD speed is touched: full speed when front > BRAKE_SLOW; between BRAKE_SLOW and
      BRAKE_STOP scaled down linearly, but never below BRAKE_CRAWL (slower commands don't make the A2
      walk, it would just stand); 0 at/below BRAKE_STOP, and it stays blocked until front >
      BRAKE_STOP + BRAKE_HYST. No LiDAR data for LIDAR_STALE s (or a LiDAR missing) -> forward
      blocked. Turning, sideways and reversing are not changed by the brake. --brake-stop /
      --brake-slow tune it, --no-lidar-brake switches it off. It never adds motion.
    * SAFETY OVAL (same function): obstacle_sectors.py counts points inside a rounded oval 0.20 m beyond
      the body in front, 0.10 m behind, 0.20 m at the sides ("oval_n"; 7 Oct). >= "oval_min" points
      inside -> only the motions that would bring the body CLOSER to it are blocked ("oval_block", per
      direction: fwd/back/left/right/turn_left/turn_right); you can always drive away (user, 7 Oct;
      replaces the 6 Oct full freeze). Released with a 5 cm bigger oval ("oval_n_release" /
      "oval_block_release"). No direction info (old obstacle_sectors) -> full freeze. No LiDAR data -> frozen.
    * Every SDK call's return code is checked; repeated Move failures lock out motion + StopMove.
    * Deliberate quit ("quit", Ctrl+C/SIGTERM on PC2) -> StopMove -> StandDown, no alarm.
      Damp() is never called. The remote controller stays your hardware e-stop.
"""

import argparse
import json
import math
import os
import signal
import socket
import statistics
import struct
import sys
import threading
import time
import wave
from collections import deque

# ============================================================================
# CONFIGURATION
# ============================================================================
NETWORK_INTERFACE = "eth0"   # PC2's wired port to the robot network (192.168.123.162/24)
PORT = 9500                  # listens on 127.0.0.1 only (reached through the SSH tunnel)

ABS_MAX_VX = 0.50            # m/s   hard caps, whatever the laptop asks for
ABS_MAX_VY = 0.30            # m/s
ABS_MAX_VYAW = 0.80          # rad/s
ABS_MAX_LINEAR = 0.50        # m/s cap on combined forward+sideways speed

ACCEL_LINEAR = 0.5           # m/s^2 ramp-up limit. Slowing down is never ramped.
ACCEL_YAW = 1.0              # rad/s^2 ramp-up limit

WATCHDOG = 0.5               # s without a "vel" message -> robot pauses (resumes when commands return)
COMMAND_RATE_HZ = 20
MAX_FAILED_CALLS = 3

# Link guard (see LinkMonitor). User's rule (2026-10-05): a problem lasting MORE than 1.6 s is a loss.
LOSS_TIME = 1.6              # s
HB_PERIOD = 0.2              # s between heartbeats
HB_TIMEOUT = LOSS_TIME       # s without a heartbeat reply -> loss
VEL_LOSS = LOSS_TIME         # s without a "vel" message -> loss (the 0.5 s WATCHDOG already paused the robot)
HB_OK_AGE = 0.6              # recovery needs a heartbeat reply within this long
BASELINE_WINDOW = 10.0       # s of stable RTT samples that define "normal" latency
BASELINE_MIN_SAMPLES = 5
HIGH_FACTOR, HIGH_ADD = 2.0, 0.100   # unstable if RTT > 2 x normal + 100 ms ...
HIGH_FOR = LOSS_TIME                 # ... continuously for 1.6 s ...
DIP_TOLERANCE = 0.3                  # ... where good moments up to 0.3 s inside the bad stretch don't reset it
OK_FACTOR, OK_MIN_ADD = 1.2, 0.020   # stable if median RTT <= max(1.2 x normal, normal + 20 ms) ...
OK_WINDOW = 1.0                      # ... (median over the last 1 s) ...
OK_FOR = 3.0                         # ... continuously for 3 s
DEFAULT_BASELINE = 0.100     # s, used until enough stable samples exist
BACKSTOP = 300.0             # s without a stable link -> StandDown (alarm continues)
FIRST_VEL_GRACE = 3.0        # s after teleop starts before "no vel" counts (window opening)

ALARM_WAV = "alarm_lost_connection.wav"   # 16 kHz mono 16-bit, in the program's folder
ALARM_VOLUME = 100
ALARM_APP = "a2_link_alarm"

VIDEO_FPS = 12               # frames sent to the laptop per second (max)
VIDEO_SIZE = (960, 540)
VIDEO_QUALITY = 70           # JPEG quality of the re-encoded frames (~40 KB each)
VIDEO_HOLD = 0.4             # s: skip frames while a heartbeat is unanswered this long (don't flood a weak link)

RPC_TIMEOUT_MOTION = 1.0     # s, per Move/StopMove call
RPC_TIMEOUT_POSTURE = 5.0    # s, for StandUp / BalanceStand / StandDown / GetState
VIDEO_TIMEOUT = 1.0          # s
SEND_TIMEOUT = 3.0           # s; a send blocked longer than this drops the connection
ACCEPT_TIMEOUT = 60.0        # s to wait for the laptop's first connection

# LiDAR brake (see lidar_brake). Distances are obstacle_sectors.py's "front": metres from the front
# edge of its body box (0.55 m from the body centre, a few cm beyond the real nose).
LIDAR_PORT = 9600            # obstacle_sectors.py sends JSON here (127.0.0.1)
BRAKE_SLOW = 0.45            # m: forward speed starts being scaled down below this (short zone: reaction time)
BRAKE_STOP = 0.25            # m: forward speed 0 at/below this: just OUTSIDE the 0.20 m safety oval, so driving
                             #    up to something stops short instead of freezing (you can still back away)
BRAKE_HYST = 0.05            # m: once stopped, stay stopped until front > BRAKE_STOP + this (no flicker)
BRAKE_CRAWL = 0.10           # m/s: slowest forward speed in the slow zone (level 1; walks, 6 Oct)
LIDAR_STALE = 0.5            # s: older LiDAR data (or "ok": false) -> forward blocked

ZERO = (0.0, 0.0, 0.0)


def log(msg):
    print(f"{time.strftime('%H:%M:%S')} {msg}", flush=True)


# ============================================================================
# Pure helpers (unit-testable without a robot)
# ============================================================================
def clamp_target(vx, vy, vyaw):
    """Laptop request -> capped (vx, vy, vyaw). Non-numbers become 0."""
    def num(v):
        return float(v) if isinstance(v, (int, float)) and math.isfinite(v) else 0.0
    vx = max(-ABS_MAX_VX, min(ABS_MAX_VX, num(vx)))
    vy = max(-ABS_MAX_VY, min(ABS_MAX_VY, num(vy)))
    vyaw = max(-ABS_MAX_VYAW, min(ABS_MAX_VYAW, num(vyaw)))
    lin = math.hypot(vx, vy)
    if lin > ABS_MAX_LINEAR:
        k = ABS_MAX_LINEAR / lin
        vx, vy = vx * k, vy * k
    return (vx, vy, vyaw)


def _num(d, key):
    v = d.get(key) if isinstance(d, dict) else None
    return float(v) if isinstance(v, (int, float)) and not isinstance(v, bool) and math.isfinite(v) else None


OVAL_MOTIONS = ("fwd", "back", "left", "right", "turn_left", "turn_right")


def _oval_mask(target, blk):
    """Zero only the parts of (vx, vy, vyaw) that move toward the object (blk: motion -> bool)."""
    vx, vy, vyaw = target
    if (vx > 0 and blk["fwd"]) or (vx < 0 and blk["back"]):
        vx = 0.0
    if (vy > 0 and blk["left"]) or (vy < 0 and blk["right"]):
        vy = 0.0
    if (vyaw > 0 and blk["turn_left"]) or (vyaw < 0 and blk["turn_right"]):
        vyaw = 0.0
    return (vx, vy, vyaw)


def lidar_brake(target, lidar, age, prev_state="clear"):
    """Apply the safety oval + forward brake to a (vx, vy, vyaw) target. Never adds motion.

    lidar: last JSON from obstacle_sectors.py (or None), age: seconds since it arrived,
    prev_state: the state returned last time (for the hysteresis).
    Returns (target, state, front) with state in "clear" | "slow" | "stop" | "freeze" | "no data".
      freeze -> axes toward the object 0 (all 0 if no direction info).   no data -> ALL axes 0.
      stop -> forward 0.   slow -> forward scaled. "freeze" also applies the forward brake.
    """
    vx, vy, vyaw = target
    front = _num(lidar, "front")
    oval_n, oval_rel, oval_min = _num(lidar, "oval_n"), _num(lidar, "oval_n_release"), _num(lidar, "oval_min")
    if (front is None or oval_n is None or oval_rel is None or not oval_min or not lidar.get("ok")
            or age is None or age > LIDAR_STALE):
        return (ZERO, "no data", front)
    in_oval = (oval_rel if prev_state == "freeze" else oval_n) >= oval_min
    if in_oval:
        blk = lidar.get("oval_block_release" if prev_state == "freeze" else "oval_block")
        if not isinstance(blk, dict) or not all(isinstance(blk.get(m), bool) for m in OVAL_MOTIONS):
            return (ZERO, "freeze", front)               # no direction info: full freeze (safe)
        target = _oval_mask(target, blk)
        vx, vy, vyaw = target
    if front <= BRAKE_STOP or (prev_state == "stop" and front <= BRAKE_STOP + BRAKE_HYST):
        return ((min(vx, 0.0), vy, vyaw), "freeze" if in_oval else "stop", front)
    if front < BRAKE_SLOW:
        k = (front - BRAKE_STOP) / (BRAKE_SLOW - BRAKE_STOP)
        slowed = min(vx, max(vx * k, BRAKE_CRAWL))     # never faster than asked, never below a crawl
        return ((slowed if vx > 0 else vx, vy, vyaw), "freeze" if in_oval else "slow", front)
    return (target, "freeze" if in_oval else "clear", front)


def _ramp_axis(prev, target, max_step):
    if target == 0.0:
        return 0.0                       # stopping: immediate
    if prev != 0.0 and (prev > 0) != (target > 0):
        return 0.0                       # reversing: stop first, then ramp the other way
    if abs(target) <= abs(prev):
        return target                    # slowing down: immediate
    step = min(max_step, abs(target) - abs(prev))
    return prev + math.copysign(step, target)


def ramp(prev, target, dt):
    """Limit how fast speeds increase; decreases and stops pass straight through."""
    lin_step, yaw_step = ACCEL_LINEAR * dt, ACCEL_YAW * dt
    return (_ramp_axis(prev[0], target[0], lin_step),
            _ramp_axis(prev[1], target[1], lin_step),
            _ramp_axis(prev[2], target[2], yaw_step))


class LinkMonitor:
    """Decides 'unstable' / 'stable again' from heartbeat RTTs and command arrival. Times in seconds.

    Not thread-safe by itself: the caller holds a lock.
    """

    def __init__(self):
        self.recent = deque()          # (t, rtt) of the last few seconds
        self.stable = deque()          # (t, rtt) collected while the link was stable -> baseline
        self.frozen = False            # baseline frozen while unstable
        self.last_reply = None
        self.high_since = None
        self.last_high = None
        self.good_since = None

    # --- samples -----------------------------------------------------------
    def add_rtt(self, t, rtt):
        self.last_reply = t
        self.recent.append((t, rtt))
        while self.recent and t - self.recent[0][0] > max(OK_WINDOW, HIGH_FOR) + 1.0:
            self.recent.popleft()
        if not self.frozen:
            self.stable.append((t, rtt))
            while self.stable and t - self.stable[0][0] > BASELINE_WINDOW:
                self.stable.popleft()
        if rtt > self.high_threshold():
            self.last_high = t
            if self.high_since is None:
                self.high_since = t
        elif self.high_since is not None and t - self.last_high > DIP_TOLERANCE:
            self.high_since = None           # good for longer than a short dip: the bad stretch is over

    def baseline(self):
        if len(self.stable) >= BASELINE_MIN_SAMPLES:
            return statistics.median(r for _, r in self.stable)
        return DEFAULT_BASELINE

    def high_threshold(self):
        return HIGH_FACTOR * self.baseline() + HIGH_ADD

    def ok_threshold(self):
        b = self.baseline()
        return max(OK_FACTOR * b, b + OK_MIN_ADD)

    def median_rtt(self, now, window=OK_WINDOW):
        vals = [r for t, r in self.recent if now - t <= window]
        return statistics.median(vals) if vals else None

    def last_rtt(self):
        return self.recent[-1][1] if self.recent else None

    # --- decisions ---------------------------------------------------------
    def unstable_reason(self, now, connected, vel_age):
        """None if fine, else a short human-readable reason. vel_age=None: don't check commands."""
        if not connected:
            return "laptop disconnected"
        if vel_age is not None and vel_age > VEL_LOSS:
            return f"no drive commands for {vel_age:.1f} s"
        if self.last_reply is not None and now - self.last_reply > HB_TIMEOUT:
            return f"no heartbeat reply for {now - self.last_reply:.1f} s"
        if self.high_since is not None and now - self.high_since >= HIGH_FOR:
            return (f"latency {self.last_rtt() * 1000:.0f} ms > limit {self.high_threshold() * 1000:.0f} ms "
                    f"for {now - self.high_since:.1f} s")
        return None

    def freeze(self):
        """Link went bad: keep the pre-loss 'normal' latency."""
        self.frozen = True
        self.good_since = None

    def unfreeze(self):
        self.frozen = False

    def stable_for(self, now, connected, vel_age):
        """Seconds the recovery condition has held continuously (0 if it doesn't hold now)."""
        med = self.median_rtt(now)
        ok = (connected
              and (vel_age is not None and vel_age <= WATCHDOG)
              and self.last_reply is not None and now - self.last_reply <= HB_OK_AGE
              and med is not None and med <= self.ok_threshold())
        if not ok:
            self.good_since = None
            return 0.0
        if self.good_since is None:
            self.good_since = now
        return now - self.good_since


def load_pcm16k(path):
    """WAV file -> raw 16-bit mono 16 kHz bytes, or None if missing / wrong format."""
    try:
        with wave.open(path) as w:
            if w.getframerate() != 16000 or w.getnchannels() != 1 or w.getsampwidth() != 2:
                log(f"ALARM WAV {path} must be 16 kHz mono 16-bit (is {w.getframerate()} Hz, "
                    f"{w.getnchannels()} ch, {w.getsampwidth() * 8} bit)")
                return None
            return w.readframes(w.getnframes())
    except (OSError, wave.Error) as e:
        log(f"ALARM WAV not usable ({e}); the guard still stops the robot, but silently")
        return None


# ============================================================================
# Robot side
# ============================================================================
class DryRunSport:
    """Stands in for SportClient in --dry-run: no motion commands are sent."""

    def SetTimeout(self, t):
        pass

    def _log(self, name, *args):
        log(f"[dry-run] would send {name}{args}")
        return 0

    def StandUp(self):
        return self._log("StandUp")

    def BalanceStand(self):
        return self._log("BalanceStand")

    def StandDown(self):
        return self._log("StandDown")

    def StopMove(self):
        return 0

    def Move(self, vx, vy, vyaw):
        return 0


class Shared:
    def __init__(self):
        self.lock = threading.Lock()
        self.sport_lock = threading.Lock()   # one SDK motion call at a time (control loop vs guard)
        self.stop = threading.Event()
        self.target = ZERO          # capped laptop request
        self.target_time = 0.0      # when the last "vel" arrived
        self.first_vel_seen = False
        self.need_zero = False      # after re-arm: ignore non-zero vel until a zero vel arrives
        self.stop_request = False
        self.sent_vel = ZERO        # what was actually commanded (after ramping)
        self.moving = False
        self.fault = ""
        self.frame = None           # latest re-encoded JPEG bytes
        self.frame_id = 0
        self.video_error = ""
        self.phase = "connected"    # connected -> teleop <-> unstable -> recovered -> teleop; backstop; done
        self.teleop_start = 0.0
        self.reason = ""
        self.unstable_since = 0.0
        self.alarm = False
        self.link = None            # current Link (None while the laptop is disconnected)
        self.gen = 0                # connection generation
        self.ever_connected = False
        self.mon = LinkMonitor()
        self.hb_id = 0
        self.hb_sent = {}           # id -> send time
        self.robot_state = "?"
        self.dry_run = False
        self.metrics = None         # open CSV file: t,kind,value (see metric())
        self.last_vel = None
        self.brake_on = True        # --no-lidar-brake sets False
        self.lidar = None           # last JSON from obstacle_sectors.py
        self.lidar_time = None      # monotonic time it arrived
        self.brake_state = "starting"  # until the first control tick (so start-up isn't a "cleared" event)
        self.brake_front = None
        self.brake_active = False   # True while the brake is cutting the driver's request
        self.brake_where = ""       # side of the object inside the safety oval
        self.brake_blocked = []     # motions blocked by the oval right now


def metric(sh, kind, value):
    """One line per heartbeat RTT / command gap / guard event, for graphing the link. Never raises."""
    f = sh.metrics
    if f is not None:
        try:
            f.write(f"{time.monotonic():.3f},{kind},{str(value).replace(',', ';')}\n")
        except (OSError, ValueError):
            pass


class Link:
    """Thread-safe framed sender for one laptop connection."""

    def __init__(self, conn):
        self.conn = conn
        self.lock = threading.Lock()

    def send(self, kind, payload):
        with self.lock:
            self.conn.sendall(kind + struct.pack(">I", len(payload)) + payload)

    def close(self):
        try:
            self.conn.shutdown(socket.SHUT_RDWR)
        except OSError:
            pass
        try:
            self.conn.close()
        except OSError:
            pass


def status_msg(sh: Shared, **extra):
    now = time.monotonic()
    with sh.lock:
        mon = sh.mon
        rtt = mon.median_rtt(now, 0.6)
        msg = {"phase": sh.phase, "vel": sh.sent_vel, "moving": sh.moving,
               "fault": sh.fault, "video_error": sh.video_error,
               "robot_state": sh.robot_state, "dry_run": sh.dry_run,
               "rtt_ms": None if rtt is None else round(rtt * 1000),
               "normal_ms": round(mon.baseline() * 1000),
               "frozen": mon.frozen, "reason": sh.reason, "alarm": sh.alarm,
               "stable_for": round(mon.good_since and now - mon.good_since or 0.0, 1),
               "unstable_s": round(now - sh.unstable_since, 1) if sh.phase in ("unstable", "backstop") else 0,
               "backstop_s": BACKSTOP,
               "lidar": {"on": sh.brake_on, "state": sh.brake_state, "front": sh.brake_front,
                         "active": sh.brake_active, "stop_m": BRAKE_STOP, "slow_m": BRAKE_SLOW,
                         "where": sh.brake_where, "blocked": sh.brake_blocked, "release_keys": sh.need_zero}}
    msg.update(extra)
    return json.dumps(msg).encode()


def drop_link(sh: Shared, gen, why):
    """Mark the laptop connection `gen` as gone (no-op if a newer one already replaced it)."""
    with sh.lock:
        if gen != sh.gen or sh.link is None:
            return
        link, sh.link = sh.link, None
        sh.hb_sent.clear()
    link.close()
    metric(sh, "link_down", why)
    log(f"Laptop link down ({why}).")


def reader_loop(link: Link, gen, sh: Shared, stand_event):
    """Reads JSON lines from one laptop connection."""
    buf = b""
    why = "laptop disconnected"
    try:
        while not sh.stop.is_set():
            try:
                data = link.conn.recv(4096)
            except socket.timeout:
                continue
            if not data:
                break
            buf += data
            while b"\n" in buf:
                line, buf = buf.split(b"\n", 1)
                try:
                    msg = json.loads(line)
                except ValueError:
                    continue
                t = msg.get("t")
                now = time.monotonic()
                if t == "vel":
                    target = clamp_target(msg.get("vx"), msg.get("vy"), msg.get("vyaw"))
                    with sh.lock:
                        if sh.last_vel is not None and now - sh.last_vel > 0.1:
                            metric(sh, "vel_gap", f"{now - sh.last_vel:.3f}")
                        sh.last_vel = now
                        if sh.need_zero and target == ZERO:
                            sh.need_zero = False
                        sh.target = ZERO if sh.need_zero else target
                        sh.target_time = now
                        sh.first_vel_seen = True
                elif t == "hb":
                    with sh.lock:
                        sent = sh.hb_sent.pop(msg.get("id"), None)
                        if sent is not None:
                            sh.mon.add_rtt(now, now - sent)
                            metric(sh, "rtt", f"{now - sent:.4f}")
                elif t == "stop":
                    with sh.lock:
                        sh.stop_request = True
                        sh.target = ZERO
                elif t == "arm":
                    with sh.lock:
                        if sh.phase == "recovered":
                            sh.phase = "teleop"
                            sh.need_zero = True
                            sh.target = ZERO
                            sh.reason = ""
                            metric(sh, "armed", "R")
                            log("Re-armed by the laptop (R). Waiting for released keys before any motion.")
                elif t == "stand":
                    stand_event.set()
                elif t == "quit":
                    log("Laptop asked to quit.")
                    sh.stop.set()
                    return
    except OSError as e:
        why = f"connection error: {e}"
    if sh.stop.is_set():
        return                    # session ending: main sends the final status, then closes
    drop_link(sh, gen, why)


def accept_loop(srv, sh: Shared, stand_event):
    """Accepts laptop connections for the whole session; a new one replaces the old one."""
    srv.settimeout(1.0)
    while not sh.stop.is_set():
        try:
            conn, _ = srv.accept()
        except socket.timeout:
            continue
        except OSError:
            break
        conn.setsockopt(socket.IPPROTO_TCP, socket.TCP_NODELAY, 1)
        conn.settimeout(SEND_TIMEOUT)
        link = Link(conn)
        with sh.lock:
            old = sh.link
            sh.gen += 1
            gen = sh.gen
            sh.link = link
            sh.hb_sent.clear()
            sh.ever_connected = True
            phase = sh.phase
        if old is not None:
            old.close()
        metric(sh, "connected", gen)
        log(f"Laptop connected (connection #{gen}, phase {phase}).")
        threading.Thread(target=reader_loop, args=(link, gen, sh, stand_event), daemon=True).start()
        try:
            link.send(b"J", status_msg(sh, caps=[ABS_MAX_VX, ABS_MAX_VY, ABS_MAX_VYAW]))
        except OSError as e:
            drop_link(sh, gen, f"send failed: {e}")


def control_loop(sport, sh: Shared):
    """The only code that sends Move/StopMove while teleop runs."""
    period = 1.0 / COMMAND_RATE_HZ
    cmd = ZERO
    moving = False
    failures = 0
    sport.SetTimeout(RPC_TIMEOUT_MOTION)
    while not sh.stop.is_set():
        t0 = time.monotonic()
        with sh.lock:
            target = sh.target
            stale = (t0 - sh.target_time) > WATCHDOG
            stop_now = sh.stop_request
            sh.stop_request = False
            locked = bool(sh.fault) or sh.phase != "teleop" or sh.need_zero
            lidar = sh.lidar
            lidar_age = None if sh.lidar_time is None else t0 - sh.lidar_time
            brake_on = sh.brake_on

        if stale or stop_now or locked:
            target = ZERO
        if brake_on:
            with sh.lock:
                prev_state = sh.brake_state
            braked, bstate, front = lidar_brake(target, lidar, lidar_age, prev_state)
            active = braked != target
            target = braked
            with sh.lock:
                sh.brake_where = lidar.get("oval_where", "") if bstate == "freeze" and isinstance(lidar, dict) else ""
                blk = (lidar.get("oval_block_release" if prev_state == "freeze" else "oval_block")
                       if bstate == "freeze" and isinstance(lidar, dict) else None)
                sh.brake_blocked = ([m for m in OVAL_MOTIONS if blk.get(m)] if isinstance(blk, dict)
                                    else (list(OVAL_MOTIONS) if bstate in ("freeze", "no data") else []))
                if prev_state in ("freeze", "no data") and bstate not in ("freeze", "no data"):
                    log("Safety oval clear again.")
                if bstate != sh.brake_state:
                    metric(sh, "brake", f"{bstate} front={front}")
                    if bstate in ("stop", "freeze", "no data") or sh.brake_state in ("stop", "freeze", "no data"):
                        log(f"LiDAR brake: {sh.brake_state} -> {bstate}"
                            + ("" if front is None else f" (front {front:.2f} m)"))
                sh.brake_state, sh.brake_front, sh.brake_active = bstate, front, active
        cmd = ramp(cmd, target, period)

        with sh.sport_lock:
            if cmd != ZERO:
                code = sport.Move(*cmd)
                if code == 0:
                    failures = 0
                    moving = True
                else:
                    failures += 1
                    log(f"Move{tuple(round(v, 3) for v in cmd)} failed, code {code} ({failures}/{MAX_FAILED_CALLS})")
                    if failures >= MAX_FAILED_CALLS:
                        with sh.lock:
                            sh.fault = f"Move failed {failures}x (code {code}) - motion locked, press ESC"
                        moving = True  # replies may have been lost while moving: send StopMove next tick
            elif moving or stop_now:
                code = sport.StopMove()
                if code == 0:
                    moving = False
                else:
                    log(f"StopMove failed, code {code}; retrying")

        with sh.lock:
            sh.sent_vel = cmd
            sh.moving = moving
        time.sleep(max(0.0, period - (time.monotonic() - t0)))

    # Leaving teleop: make sure walking has stopped (main then stands the robot down).
    for _ in range(3):
        with sh.sport_lock:
            if sport.StopMove() == 0:
                break
        time.sleep(0.2)


def lidar_loop(sh: Shared, port):
    """Receives obstacle_sectors.py's clearances (UDP JSON on 127.0.0.1). Never touches motion."""
    sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    try:
        sock.bind(("127.0.0.1", port))
    except OSError as e:
        log(f"!! LiDAR brake cannot listen on UDP {port} ({e}): forward driving stays BLOCKED. "
            f"Is avoid_drive.py running? Stop it, then restart teleop.")
        return
    sock.settimeout(0.5)
    while not sh.stop.is_set():
        try:
            data = sock.recv(4096)
        except socket.timeout:
            continue
        except OSError:
            break
        try:
            msg = json.loads(data)
        except ValueError:
            continue
        with sh.lock:
            sh.lidar = msg
            sh.lidar_time = time.monotonic()
    sock.close()


def guard_loop(sport, sh: Shared):
    """Link guard state machine: teleop -> unstable (stop + alarm) -> recovered (wait for R) -> teleop."""
    backstop_done = False
    while not sh.stop.is_set():
        now = time.monotonic()
        do_backstop = False
        with sh.lock:
            phase = sh.phase
            connected = sh.link is not None
            vel_age = (now - sh.target_time) if sh.first_vel_seen else None
            if vel_age is None and now - sh.teleop_start > FIRST_VEL_GRACE:
                vel_age = now - sh.teleop_start      # window never started sending
            mon = sh.mon
            if phase in ("teleop", "recovered"):
                reason = mon.unstable_reason(now, connected, vel_age)
                if reason:
                    mon.freeze()
                    sh.phase = "unstable"
                    sh.reason = reason
                    sh.unstable_since = now
                    sh.alarm = True
                    sh.target = ZERO
                    metric(sh, "unstable", reason)
                    log(f"LINK UNSTABLE: {reason}. Robot stopped (standing), alarm on. "
                        f"Normal latency {mon.baseline() * 1000:.0f} ms.")
            elif phase in ("unstable", "backstop"):
                held = mon.stable_for(now, connected, vel_age)
                if held >= OK_FOR:
                    sh.alarm = False
                    mon.unfreeze()
                    if phase == "backstop":
                        sh.phase = "done"
                        sh.reason = "link back after the 5 min backstop; robot is lying down"
                        log("Link stable again after the backstop. Alarm off; robot stays lying down. Exiting.")
                        sh.stop.set()
                    else:
                        sh.phase = "recovered"
                        sh.reason = ""
                        metric(sh, "recovered", f"{held:.1f}")
                        log(f"Link stable for {held:.1f} s. Alarm off. Waiting for R (re-arm) on the laptop.")
                elif phase == "unstable" and not backstop_done and now - sh.unstable_since >= BACKSTOP:
                    backstop_done = True
                    do_backstop = True
                    sh.phase = "backstop"
                    sh.reason = f"no stable link for {BACKSTOP / 60:.0f} min - lying down"
        if do_backstop:
            log(f"BACKSTOP: no stable link for {BACKSTOP:.0f} s. StandDown, alarm continues.")
            with sh.sport_lock:
                sport.SetTimeout(RPC_TIMEOUT_POSTURE)
                sport.StopMove()
                time.sleep(0.5)
                code = sport.StandDown()
                sport.SetTimeout(RPC_TIMEOUT_MOTION)
            log("Robot lying down (backstop)." if code == 0 else
                f"!! Backstop StandDown failed (code {code}). Use the REMOTE.")
        time.sleep(0.1)


def alarm_loop(audio, pcm, volume, sh: Shared):
    """Loops the alarm WAV on the robot speaker while sh.alarm is set. Restores the volume after."""
    playing = False
    old_volume = None
    duration = len(pcm) / 2 / 16000 if pcm else 0.0
    next_start = 0.0
    while not sh.stop.is_set() or playing:
        with sh.lock:
            want = sh.alarm and not sh.stop.is_set()
        now = time.monotonic()
        if want and not playing:
            playing = True
            next_start = now
            try:
                code, v = audio.GetVolume()
                old_volume = v.get("volume") if code == 0 and isinstance(v, dict) else None
                audio.SetVolume(volume)
            except Exception as e:
                log(f"alarm volume error: {e}")
        if want and playing and pcm and now >= next_start:
            stream_id = str(int(time.time() * 1000))
            for i in range(0, len(pcm), 96000):          # same chunking as Unitree's example
                code, _ = audio.PlayStream(ALARM_APP, stream_id, pcm[i:i + 96000])
                if code != 0:
                    log(f"alarm PlayStream error code {code}")
                    break
            next_start = now + duration + 0.3
        if not want and playing:
            playing = False
            try:
                audio.PlayStop(ALARM_APP)
                if old_volume is not None:
                    audio.SetVolume(int(old_volume))
            except Exception as e:
                log(f"alarm stop error: {e}")
        time.sleep(0.1)


def grab_loop(video, sh: Shared):
    """Fetches camera frames and re-encodes them small for Wi-Fi."""
    import cv2
    import numpy as np
    period = 1.0 / VIDEO_FPS
    while not sh.stop.is_set():
        t0 = time.monotonic()
        try:
            code, data = video.GetImageSample()
            if code == 0 and data:
                frame = cv2.imdecode(np.frombuffer(bytes(data), np.uint8), cv2.IMREAD_COLOR)
                if frame is not None:
                    ok, jpg = cv2.imencode(".jpg", cv2.resize(frame, VIDEO_SIZE),
                                           [cv2.IMWRITE_JPEG_QUALITY, VIDEO_QUALITY])
                    if ok:
                        with sh.lock:
                            sh.frame = jpg.tobytes()
                            sh.frame_id += 1
                            sh.video_error = ""
            else:
                with sh.lock:
                    sh.video_error = f"camera code {code}"
        except Exception as e:  # keep teleop alive even if video breaks
            with sh.lock:
                sh.video_error = f"camera error: {e}"
        time.sleep(max(0.0, period - (time.monotonic() - t0)))


def sender_loop(sh: Shared):
    """Sends heartbeats, the newest frame and status to the current laptop. Never touches motion."""
    period = 1.0 / VIDEO_FPS
    last_id = -1
    last_hb = 0.0
    while not sh.stop.is_set():
        t0 = time.monotonic()
        with sh.lock:
            link, gen = sh.link, sh.gen
            frame, fid = sh.frame, sh.frame_id
            oldest_hb = min(sh.hb_sent.values()) if sh.hb_sent else None
        if link is not None:
            try:
                if t0 - last_hb >= HB_PERIOD:
                    with sh.lock:
                        sh.hb_id += 1
                        hb = sh.hb_id
                        sh.hb_sent[hb] = time.monotonic()
                        for k in [k for k, v in sh.hb_sent.items() if t0 - v > 5.0]:
                            del sh.hb_sent[k]
                    link.send(b"J", json.dumps({"hb": hb}).encode())
                    last_hb = t0
                weak = oldest_hb is not None and t0 - oldest_hb > VIDEO_HOLD
                if frame is not None and fid != last_id and not weak:
                    link.send(b"F", frame)
                    last_id = fid
                link.send(b"J", status_msg(sh))
            except OSError as e:
                drop_link(sh, gen, f"send failed: {e}")
        time.sleep(max(0.0, period - (time.monotonic() - t0)))


def main():
    global BACKSTOP, BRAKE_STOP, BRAKE_SLOW
    ap = argparse.ArgumentParser(description="A2 Wi-Fi teleop, robot side (runs on PC2)")
    ap.add_argument("--dry-run", action="store_true", help="no motion commands; video and alarm still run")
    ap.add_argument("--no-video", action="store_true", help="don't fetch camera frames")
    ap.add_argument("--mute-alarm", action="store_true", help="guard works but plays no sound (testing)")
    ap.add_argument("--alarm-volume", type=int, default=ALARM_VOLUME)
    ap.add_argument("--backstop", type=float, default=BACKSTOP, help="s without a stable link before lying down")
    ap.add_argument("--metrics", default="link_metrics.csv",
                    help="per-heartbeat link log (t,kind,value), in the program folder; '' = off")
    ap.add_argument("--iface", default=NETWORK_INTERFACE)
    ap.add_argument("--port", type=int, default=PORT)
    ap.add_argument("--no-lidar-brake", action="store_true",
                    help="drive WITHOUT the LiDAR brake (forward never limited by obstacles)")
    ap.add_argument("--lidar-port", type=int, default=LIDAR_PORT)
    ap.add_argument("--brake-stop", type=float, default=BRAKE_STOP, help="m: forward blocked at/below this front distance")
    ap.add_argument("--brake-slow", type=float, default=BRAKE_SLOW, help="m: forward slowed below this front distance")
    args = ap.parse_args()
    BACKSTOP = args.backstop
    if not 0.10 <= args.brake_stop < args.brake_slow <= 3.0:
        log(f"ERROR: need 0.10 <= --brake-stop ({args.brake_stop}) < --brake-slow ({args.brake_slow}) <= 3.0")
        sys.exit(2)
    BRAKE_STOP, BRAKE_SLOW = args.brake_stop, args.brake_slow

    try:
        from unitree_sdk2py.core.channel import ChannelFactoryInitialize
        from unitree_sdk2py.a2.sport.sport_client import SportClient
        from unitree_sdk2py.a2.audio.audio_client import AudioClient
        from unitree_sdk2py.go2.video.video_client import VideoClient  # no A2 video client; this one works on the A2
    except ImportError as e:
        log(f"ERROR: {e}\nRun with PYTHONPATH=~/a2_wireless so the copied SDK is found.")
        sys.exit(1)

    sh = Shared()
    sh.dry_run = args.dry_run
    sh.brake_on = not args.no_lidar_brake
    if sh.brake_on:
        threading.Thread(target=lidar_loop, args=(sh, args.lidar_port), daemon=True).start()
    if args.metrics:
        path = os.path.join(os.path.dirname(os.path.abspath(__file__)), args.metrics)
        sh.metrics = open(path, "w", buffering=1)
        sh.metrics.write("t,kind,value\n")
    for sig in (signal.SIGINT, signal.SIGTERM):
        signal.signal(sig, lambda *_: sh.stop.set())   # deliberate stop on PC2 -> lie down
    signal.signal(signal.SIGHUP, signal.SIG_IGN)       # SSH dropping must NOT end the guard

    ChannelFactoryInitialize(0, args.iface)
    real_sport = SportClient()
    real_sport.SetTimeout(RPC_TIMEOUT_POSTURE)
    real_sport.Init()
    code, state = 0, {}
    code = real_sport.GetState(state)
    sh.robot_state = state.get("fsm_name", "?") if code == 0 else f"GetState failed (code {code})"
    sport = DryRunSport() if args.dry_run else real_sport

    audio = AudioClient()
    audio.SetTimeout(3.0)
    audio.Init()
    pcm = None if args.mute_alarm else load_pcm16k(os.path.join(os.path.dirname(os.path.abspath(__file__)),
                                                                 ALARM_WAV))

    srv = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
    srv.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
    srv.bind(("127.0.0.1", args.port))
    srv.listen(2)
    stand_event = threading.Event()
    threading.Thread(target=accept_loop, args=(srv, sh, stand_event), daemon=True).start()
    threading.Thread(target=sender_loop, args=(sh,), daemon=True).start()
    log(f"READY port {args.port} | robot state {sh.robot_state}" + (" | DRY RUN" if args.dry_run else "")
        + (" | alarm MUTED" if pcm is None else f" | alarm {args.alarm_volume}%")
        + (f" | LiDAR brake ON (stop {BRAKE_STOP} m, slow {BRAKE_SLOW} m, UDP {args.lidar_port})"
           if sh.brake_on else " | LiDAR brake OFF"))

    # Wait for the laptop's first connection, then its "stand" (the person confirms the checklist there).
    t0 = time.monotonic()
    while not sh.stop.is_set() and not stand_event.is_set():
        with sh.lock:
            ever, connected = sh.ever_connected, sh.link is not None
        if not ever and time.monotonic() - t0 > ACCEPT_TIMEOUT:
            log("No laptop connected within 60 s. Nothing was sent.")
            sh.stop.set()
        elif ever and not connected:
            log("Laptop left before standing. Nothing was sent.")
            sh.stop.set()
        time.sleep(0.05)
    if not stand_event.is_set():
        srv.close()
        return

    stood = False
    threads = []
    try:
        log("Standing up...")
        stood = True               # from here on, a deliberate exit sends StandDown
        sport.SetTimeout(RPC_TIMEOUT_POSTURE)
        code = sport.StandUp()
        if code == 0:
            time.sleep(3.0)
            code = sport.BalanceStand()
        if code != 0:
            with sh.lock:
                sh.phase = f"stand failed (code {code})"
            time.sleep(1.0)        # let the sender report it
            return
        time.sleep(1.5)

        with sh.lock:
            sh.phase = "teleop"
            sh.teleop_start = time.monotonic()
            sh.target_time = 0.0       # watchdog: no motion until the first "vel"
        threads = [threading.Thread(target=control_loop, args=(sport, sh), daemon=True),
                   threading.Thread(target=guard_loop, args=(sport, sh), daemon=True),
                   threading.Thread(target=alarm_loop, args=(audio, pcm, args.alarm_volume, sh), daemon=True)]
        if not args.no_video:
            video = VideoClient()
            video.SetTimeout(VIDEO_TIMEOUT)
            video.Init()
            threads.append(threading.Thread(target=grab_loop, args=(video, sh), daemon=True))
        for t in threads:
            t.start()
        log("Teleop running. Link guard active.")
        while not sh.stop.is_set():
            time.sleep(0.05)
    finally:
        sh.stop.set()
        with sh.lock:
            sh.alarm = False
        for t in threads:
            t.join(timeout=4.0)
        with sh.lock:
            already_down = sh.phase == "done"
        sport.SetTimeout(RPC_TIMEOUT_POSTURE)
        sport.StopMove()
        if stood and not already_down:
            time.sleep(0.5)
            code = sport.StandDown()
            msg = "Robot lying down (StandDown)." if code == 0 else \
                f"!! StandDown failed (code {code}). Use the REMOTE to lie the robot down."
            log(msg)
            with sh.lock:
                sh.phase = "done" if code == 0 else msg
        with sh.lock:
            link = sh.link
        if link is not None:
            try:
                link.send(b"J", status_msg(sh))
            except OSError:
                pass
            link.close()
        srv.close()
        log("Robot side finished.")


if __name__ == "__main__":
    main()
