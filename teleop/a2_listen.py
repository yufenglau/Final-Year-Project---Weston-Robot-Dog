#!/usr/bin/env python3
"""
Unitree A2 live microphone: LAPTOP side. Hear the robot's microphone on the laptop speakers.

    python3 ~/unitree_a2/teleop/a2_listen.py              # listen until Ctrl+C
    python3 ~/unitree_a2/teleop/a2_listen.py --gain 2     # louder

The teleop window (a2_teleop_wifi_gui.py) uses the same ListenLink class: on by default, L toggles it,
and it mutes itself while you hold T (talk) so the two don't feed back into each other.

Robot side: a2_listen_server.py on PC2 (~/a2_wireless), started here over its own SSH link.
Latency is kept low by dropping old audio instead of letting a backlog build up.
Read-only: nothing is sent to the robot.
"""
import argparse
import array
import collections
import subprocess
import sys
import threading
import time

SSH_HOST = "a2-pc2"
REMOTE_DIR = "~/a2_wireless"
APLAY = ["aplay", "-q", "-t", "raw", "-f", "S16_LE", "-r", "16000", "-c", "1", "--buffer-time=200000"]
MAX_BACKLOG_S = 0.5          # drop audio older than this (keeps it "live")
BYTES_PER_S = 32000


class ListenLink:
    """Robot mic -> PC2 -> SSH -> laptop speakers. Separate from the motion link."""

    def __init__(self, gain=1.0, prefix="[PC2 mic]"):
        self.gain, self.prefix = gain, prefix
        self.muted = False
        self.ssh = None
        self.play = None
        self.queue = collections.deque()
        self.queued = 0
        self.lock = threading.Condition()
        self.dropped_s = 0.0
        self.last_audio = 0.0
        self.stopping = False

    @property
    def alive(self):
        return self.ssh is not None and self.ssh.poll() is None

    @property
    def receiving(self):
        return self.alive and time.monotonic() - self.last_audio < 1.0

    def start(self, timeout=10):
        self.ssh = subprocess.Popen(["ssh", "-o", "BatchMode=yes", "-o", "ConnectTimeout=8",
                                     "-o", "ServerAliveInterval=5", SSH_HOST,
                                     f"cd {REMOTE_DIR} && exec python3 -u a2_listen_server.py"],
                                    stdin=subprocess.DEVNULL, stdout=subprocess.PIPE, stderr=subprocess.PIPE,
                                    start_new_session=True)
        self.play = subprocess.Popen(APLAY, stdin=subprocess.PIPE, stderr=subprocess.DEVNULL)
        threading.Thread(target=self._show_pc2, daemon=True).start()
        threading.Thread(target=self._receive, daemon=True).start()
        threading.Thread(target=self._playback, daemon=True).start()
        t0 = time.monotonic()
        while time.monotonic() - t0 < timeout and self.alive and not self.last_audio:
            time.sleep(0.1)
        return self.receiving

    def _show_pc2(self):
        for line in iter(self.ssh.stderr.readline, b""):
            text = line.decode(errors="replace").rstrip()
            if text and "Warning: Permanently added" not in text:
                print(f"{self.prefix} {text.removeprefix('[PC2] ')}", flush=True)

    def _receive(self):
        for block in iter(lambda: self.ssh.stdout.read(5120), b""):   # one 160 ms robot packet
            self.last_audio = time.monotonic()
            with self.lock:
                self.queue.append(block)
                self.queued += len(block)
                while self.queued > MAX_BACKLOG_S * BYTES_PER_S:    # fell behind: drop the oldest audio
                    old = self.queue.popleft()
                    self.queued -= len(old)
                    self.dropped_s += len(old) / BYTES_PER_S
                self.lock.notify()
        with self.lock:
            self.stopping = True
            self.lock.notify()

    def _playback(self):
        while True:
            with self.lock:
                while not self.queue and not self.stopping:
                    self.lock.wait(0.5)
                if self.stopping and not self.queue:
                    return
                block = self.queue.popleft()
                self.queued -= len(block)
                muted, gain = self.muted, self.gain
            if muted:
                block = bytes(len(block))                  # keep timing, play silence
            elif gain != 1.0:
                a = array.array("h", block[: len(block) // 2 * 2])
                block = array.array("h", (max(-32768, min(32767, int(v * gain))) for v in a)).tobytes()
            try:
                self.play.stdin.write(block)
                self.play.stdin.flush()
            except (BrokenPipeError, OSError):
                return

    def set_muted(self, muted):
        with self.lock:
            self.muted = muted
            if muted:                                      # don't replay what was said while muted
                self.queue.clear()
                self.queued = 0

    def close(self):
        for p in (self.ssh, self.play):
            if p is not None:
                try:
                    p.terminate()
                    p.wait(timeout=3)
                except (OSError, subprocess.TimeoutExpired):
                    p.kill()


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--gain", type=float, default=1.0, help="playback boost (default 1.0)")
    args = ap.parse_args()
    lst = ListenLink(args.gain, prefix="[PC2]")
    print("Connecting to the robot microphone...")
    if not lst.start():
        print("ERROR: no audio from the robot mic. On the robot network / Tailscale? "
              "a2_listen_server.py copied to ~/a2_wireless on PC2?")
        lst.close()
        sys.exit(1)
    print("Listening live. Ctrl+C to stop.")
    try:
        while lst.alive:
            time.sleep(5)
            if lst.dropped_s:
                print(f"(dropped {lst.dropped_s:.1f} s of late audio to stay live)")
                lst.dropped_s = 0.0
    except KeyboardInterrupt:
        pass
    finally:
        lst.close()


if __name__ == "__main__":
    main()
