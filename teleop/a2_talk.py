#!/usr/bin/env python3
"""
Unitree A2 walkie-talkie: LAPTOP side. Speak into the laptop, the robot plays it on its speaker.

    python3 ~/unitree_a2/teleop/a2_talk.py               # volume 70
    python3 ~/unitree_a2/teleop/a2_talk.py --volume 50   # quieter
    python3 ~/unitree_a2/teleop/a2_talk.py --check       # connection test only, no sound

Press Enter to start talking, Enter again to stop. q + Enter quits.
The teleop window (a2_teleop_wifi_gui.py) uses the same TalkLink class: hold T to talk.

Laptop on the robot Wi-Fi (A2-4G or the Unitree AP) with the 192.168.123.0/24 route, or on the cable.
The robot side (a2_talk_server.py) runs on PC2 in ~/a2_wireless, started here over SSH.
Audio only: no motion commands. Keep the laptop away from the robot (or use a headset) to avoid echo.
"""
import argparse
import subprocess
import sys
import threading

SSH_HOST = "a2-pc2"
REMOTE_DIR = "~/a2_wireless"
ARECORD = ["arecord", "-q", "-f", "S16_LE", "-r", "16000", "-c", "1", "-t", "raw"]


class TalkLink:
    """Laptop mic -> SSH -> a2_talk_server.py on PC2 -> robot speaker. Separate from the motion link."""

    def __init__(self, volume=70, gain=2.0, prefix="[PC2 audio]"):
        self.volume, self.gain, self.prefix = volume, gain, prefix
        self.ssh = None
        self.rec = None
        self.pump = None
        self.ready = threading.Event()

    @property
    def active(self):
        return self.rec is not None

    @property
    def alive(self):
        return self.ssh is not None and self.ssh.poll() is None

    def start(self, check=False, timeout=20):
        """Start the PC2 side. Returns True when it reports READY (or, with check=True, when the check passed)."""
        remote = (f"cd {REMOTE_DIR} && PYTHONPATH={REMOTE_DIR} exec python3 -u a2_talk_server.py "
                  f"--volume {self.volume} --gain {self.gain}" + (" --check" if check else ""))
        self.ssh = subprocess.Popen(["ssh", "-o", "BatchMode=yes", "-o", "ConnectTimeout=8",
                                     "-o", "ServerAliveInterval=5", SSH_HOST, remote],
                                    stdin=subprocess.PIPE, stdout=subprocess.PIPE, stderr=subprocess.STDOUT,
                                    start_new_session=True)   # Ctrl+C must not kill ssh mid-sentence
        threading.Thread(target=self._show_pc2, daemon=True).start()
        if check:
            self.ssh.wait(timeout=30)
            return self.ssh.returncode == 0
        self.ready.wait(timeout=timeout)
        return self.alive and self.ready.is_set()

    def _show_pc2(self):
        for line in iter(self.ssh.stdout.readline, b""):
            text = line.decode(errors="replace").rstrip()
            print(f"{self.prefix} {text.removeprefix('[PC2] ')}", flush=True)
            if "READY" in text:
                self.ready.set()
        self.ready.set()

    def talk_on(self):
        if self.active or not self.alive:
            return
        rec = subprocess.Popen(ARECORD, stdout=subprocess.PIPE, stderr=subprocess.DEVNULL)

        def copy():
            for block in iter(lambda: rec.stdout.read(3200), b""):   # 0.1 s blocks
                try:
                    self.ssh.stdin.write(block)
                    self.ssh.stdin.flush()
                except (BrokenPipeError, ValueError, OSError):
                    break
        self.rec = rec
        self.pump = threading.Thread(target=copy, daemon=True)
        self.pump.start()

    def talk_off(self):
        if self.rec:
            self.rec.terminate()
            self.rec.wait()
            self.pump.join(timeout=1)
            self.rec = None

    def close(self):
        self.talk_off()
        if self.ssh:
            try:
                self.ssh.stdin.close()
            except Exception:
                pass
            try:
                self.ssh.wait(timeout=10)
            except subprocess.TimeoutExpired:
                self.ssh.kill()


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--volume", type=int, default=70, help="robot speaker volume 0-100 (default 70)")
    ap.add_argument("--gain", type=float, default=2.0, help="mic boost on PC2 (default 2.0)")
    ap.add_argument("--check", action="store_true", help="connection test only, no sound")
    args = ap.parse_args()

    talk = TalkLink(args.volume, args.gain, prefix="[PC2]")
    if args.check:
        sys.exit(0 if talk.start(check=True) else 1)
    if not talk.start():
        print("ERROR: PC2 side did not start. On the robot Wi-Fi? Route to 192.168.123.0/24? "
              "a2_talk_server.py copied to ~/a2_wireless on PC2?")
        talk.close()
        sys.exit(1)

    print("\nEnter = start talking | Enter again = stop | q + Enter = quit")
    try:
        while True:
            cmd = input("" if talk.active else "> ").strip().lower()
            if cmd == "q":
                break
            if talk.active:
                talk.talk_off()
                print("   (stopped; the robot finishes playing)")
            else:
                talk.talk_on()
                print("   TALKING... press Enter to stop")
    except (KeyboardInterrupt, EOFError):
        pass
    finally:
        talk.close()


if __name__ == "__main__":
    main()
