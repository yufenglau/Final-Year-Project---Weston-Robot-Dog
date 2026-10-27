#!/usr/bin/env python3
"""
Unitree A2 Pro keyboard teleop with multi-axis control and live front-camera view.

Corrected version of ~/Downloads/unitree_a2_teleop.py (that file is left unchanged).

Run in an a2py terminal (never with sudo, never with system python):
    a2py
    python3 ~/unitree_a2/teleop/a2_teleop.py --dry-run   # keys + camera only, NO motion commands
    python3 ~/unitree_a2/teleop/a2_teleop.py             # real teleop

Controls (click the window once so it has keyboard focus). Keys can be COMBINED:
    W / S   forward / backward
    A / D   sideways left / right
    Q / E   turn left / right
    e.g. W+A = diagonal, W+Q = walk forward while turning, W+A+Q = all three axes.
    Space   stop now; motion stays blocked until you release all movement keys
    ESC     stop, lie down (StandDown) and exit. Ctrl+C or closing the window does the same.

Guard rails:
    * A2 SDK sport client. Checks the robot state first and asks before standing up.
    * Real key-held state (pygame): releasing a key stops that axis on the next tick (50 ms).
    * Window loses focus (you click elsewhere) -> all keys count as released -> robot stops.
    * GUI frozen for more than GUI_STALE_TIMEOUT -> robot stops.
    * Per-axis speed limits, plus a cap on combined linear speed (diagonals are not faster).
    * Speed-ups are ramped (no jerky starts); slowing down and stopping are immediate.
    * Opposite keys cancel (W+S = no forward motion).
    * Every SDK call's return code is checked; repeated failures lock out motion and send StopMove.
    * One control thread owns all motion calls; camera frames are fetched in their own thread.
    * Exit always goes StopMove -> StandDown. Damp() is never called (it would drop a standing robot).
    * The remote controller stays your hardware e-stop. Whether the A2 keeps walking if Move
      commands stop arriving is UNKNOWN, so keep the remote in hand.
"""

import argparse
import math
import os
import signal
import sys
import threading
import time

import cv2
import numpy as np

os.environ.setdefault("PYGAME_HIDE_SUPPORT_PROMPT", "1")
try:
    import pygame
    from unitree_sdk2py.core.channel import ChannelFactoryInitialize
    from unitree_sdk2py.a2.sport.sport_client import SportClient
    from unitree_sdk2py.go2.video.video_client import VideoClient  # no A2 video client; this one works on the A2
except ImportError as e:
    print(f"ERROR: {e}\nRun 'a2py' first so the SDK virtualenv is active (it has pygame, cv2, unitree_sdk2py).")
    sys.exit(1)

# ============================================================================
# CONFIGURATION
# ============================================================================
NETWORK_INTERFACE = "enp2s0"   # laptop port wired to the robot (192.168.123.222/24)

MAX_VX = 0.20       # m/s forward/backward   (first-test values; raise only after testing)
MAX_VY = 0.15       # m/s sideways
MAX_VYAW = 0.30     # rad/s turning
MAX_LINEAR = 0.20   # m/s cap on combined forward+sideways speed (W+A must not exceed this)

ACCEL_LINEAR = 0.5  # m/s^2 ramp-up limit (0 -> 0.2 m/s in 0.4 s). Slowing down is never ramped.
ACCEL_YAW = 1.0     # rad/s^2 ramp-up limit

COMMAND_RATE_HZ = 20       # Move() rate while keys are held
GUI_STALE_TIMEOUT = 0.3    # s; if the window loop stops updating key state, the robot is stopped
MAX_FAILED_CALLS = 3       # consecutive failed Move() calls before motion is locked out

RPC_TIMEOUT_MOTION = 1.0   # s, per SDK call during teleop
RPC_TIMEOUT_POSTURE = 5.0  # s, for StandUp / BalanceStand / StandDown / GetState
VIDEO_TIMEOUT = 1.0        # s

WIN_W, WIN_H = 960, 540
ZERO = (0.0, 0.0, 0.0)


# ============================================================================
# Pure helpers (unit-testable without a robot or a window)
# ============================================================================
def compute_target(w, s, a, d, q, e):
    """Held keys -> (vx, vy, vyaw). Opposite keys cancel; combined linear speed is capped."""
    vx = MAX_VX * (int(w) - int(s))
    vy = MAX_VY * (int(a) - int(d))
    vyaw = MAX_VYAW * (int(q) - int(e))
    lin = math.hypot(vx, vy)
    if lin > MAX_LINEAR:
        k = MAX_LINEAR / lin
        vx, vy = vx * k, vy * k
    return (vx, vy, vyaw)


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


# ============================================================================
# Robot side
# ============================================================================
class DryRunSport:
    """Stands in for SportClient in --dry-run: prints instead of sending motion commands."""

    def SetTimeout(self, t):
        pass

    def _log(self, name, *args):
        print(f"[dry-run] would send {name}{args}")
        return 0

    def StandUp(self):
        return self._log("StandUp")

    def BalanceStand(self):
        return self._log("BalanceStand")

    def StandDown(self):
        return self._log("StandDown")

    def StopMove(self):
        return self._log("StopMove")

    def Move(self, vx, vy, vyaw):
        return 0  # too frequent to print; the window shows the values


class Shared:
    """State shared between the window (main) thread, control thread and video thread."""

    def __init__(self):
        self.lock = threading.Lock()
        self.stop = threading.Event()
        self.key_target = ZERO      # what the held keys ask for (already capped)
        self.gui_time = 0.0         # last time the window loop refreshed key_target
        self.stop_request = False
        self.sent_vel = ZERO        # what was actually commanded (after ramping)
        self.fault = ""
        self.frame = None
        self.frame_count = 0
        self.video_error = ""


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
            target = sh.key_target
            stale = (t0 - sh.gui_time) > GUI_STALE_TIMEOUT
            stop_now = sh.stop_request
            sh.stop_request = False
            locked = bool(sh.fault)

        if stale or stop_now or locked:
            target = ZERO
        cmd = ramp(cmd, target, period)

        if cmd != ZERO:
            code = sport.Move(*cmd)
            if code == 0:
                failures = 0
                moving = True
            else:
                failures += 1
                print(f"Move{tuple(round(v, 3) for v in cmd)} failed, code {code} ({failures}/{MAX_FAILED_CALLS})")
                if failures >= MAX_FAILED_CALLS:
                    with sh.lock:
                        sh.fault = f"Move failed {failures}x (code {code}) - motion locked, press ESC"
                    print("!! " + sh.fault)
                    moving = True  # replies may have been lost while the robot moved: send StopMove next tick
        elif moving or stop_now:
            code = sport.StopMove()
            if code == 0:
                moving = False
            else:
                print(f"StopMove failed, code {code}; retrying")

        with sh.lock:
            sh.sent_vel = cmd
        time.sleep(max(0.0, period - (time.monotonic() - t0)))

    # Leaving teleop: make sure walking has stopped (main thread then stands the robot down).
    for _ in range(3):
        if sport.StopMove() == 0:
            break
        time.sleep(0.2)


def video_loop(video, sh: Shared):
    while not sh.stop.is_set():
        try:
            code, data = video.GetImageSample()
            if code == 0 and data:
                frame = cv2.imdecode(np.frombuffer(bytes(data), np.uint8), cv2.IMREAD_COLOR)
                if frame is not None:
                    rgb = cv2.cvtColor(cv2.resize(frame, (WIN_W, WIN_H)), cv2.COLOR_BGR2RGB)
                    with sh.lock:
                        sh.frame = rgb
                        sh.frame_count += 1
                        sh.video_error = ""
                    continue
            with sh.lock:
                sh.video_error = f"camera code {code}"
        except Exception as e:  # keep teleop alive even if video breaks
            with sh.lock:
                sh.video_error = f"camera error: {e}"
        time.sleep(0.2)


def get_state(client):
    """Read-only A2 GetState (API 1034). Returns (code, dict)."""
    state = {}
    code = client.GetState(state)
    return code, state


# ============================================================================
# Window (main thread)
# ============================================================================
def draw(screen, fonts, sh: Shared, held, focused, latched, fps, dry_run):
    big, small = fonts
    with sh.lock:
        frame = sh.frame
        vx, vy, vyaw = sh.sent_vel
        fault, verr = sh.fault, sh.video_error
    if frame is not None:
        screen.blit(pygame.image.frombuffer(frame.tobytes(), (WIN_W, WIN_H), "RGB"), (0, 0))
    else:
        screen.fill((20, 20, 20))

    moving = (vx, vy, vyaw) != ZERO
    color = (255, 165, 0) if moving else (0, 210, 0)
    status = "MOVING" if moving else "STOPPED"
    if dry_run:
        status += "  [DRY RUN - no motion sent]"
    lines = [(big, status, color),
             (small, f"vx {vx:+.2f} m/s   vy {vy:+.2f} m/s   yaw {vyaw:+.2f} rad/s   cam {fps:4.1f} fps", color)]
    if not focused:
        lines.append((big, "CLICK THIS WINDOW TO CONTROL (no focus = stopped)", (255, 60, 60)))
    if latched:
        lines.append((small, "STOP latched: release all movement keys to drive again", (255, 60, 60)))
    if fault:
        lines.append((small, fault, (255, 60, 60)))
    if verr:
        lines.append((small, verr, (255, 60, 60)))
    y = 10
    for font, text, col in lines:
        screen.blit(font.render(text, True, col), (12, y))
        y += font.get_linesize() + 2

    # Key indicators: held keys light up.
    x = 12
    for name in "WSADQE":
        col = (255, 165, 0) if held[name] else (90, 90, 90)
        screen.blit(big.render(name, True, col), (x, WIN_H - 70))
        x += 34
    screen.blit(small.render("Hold & combine W/S A/D Q/E | Space stop | ESC lie down + exit",
                             True, (255, 255, 255)), (12, WIN_H - 30))
    pygame.display.flip()


def run_window(sh: Shared, dry_run):
    pygame.init()
    screen = pygame.display.set_mode((WIN_W, WIN_H))
    pygame.display.set_caption("A2 teleop - click here, then hold keys")
    fonts = (pygame.font.Font(None, 34), pygame.font.Font(None, 24))
    clock = pygame.time.Clock()
    latched = False
    fps, last_count, last_t = 0.0, 0, time.monotonic()
    keymap = {"W": pygame.K_w, "S": pygame.K_s, "A": pygame.K_a,
              "D": pygame.K_d, "Q": pygame.K_q, "E": pygame.K_e}
    try:
        while not sh.stop.is_set():
            for ev in pygame.event.get():
                if ev.type == pygame.QUIT:
                    print("Window closed - stopping and lying down")
                    return
                if ev.type == pygame.KEYDOWN and ev.key == pygame.K_ESCAPE:
                    print("ESC - stopping and lying down")
                    return
                if ev.type == pygame.KEYDOWN and ev.key == pygame.K_SPACE:
                    latched = True
                    with sh.lock:
                        sh.stop_request = True
                    print("STOP")

            focused = pygame.key.get_focused()
            pressed = pygame.key.get_pressed()
            held = {n: bool(pressed[k]) and focused for n, k in keymap.items()}
            if latched and not any(held.values()):
                latched = False
            if focused and not latched:
                target = compute_target(held["W"], held["S"], held["A"], held["D"], held["Q"], held["E"])
            else:
                target = ZERO
            with sh.lock:
                sh.key_target = target
                sh.gui_time = time.monotonic()
                count = sh.frame_count

            now = time.monotonic()
            if now - last_t >= 1.0:
                fps, last_count, last_t = (count - last_count) / (now - last_t), count, now
            draw(screen, fonts, sh, held, focused, latched, fps, dry_run)
            clock.tick(30)
    finally:
        with sh.lock:
            sh.key_target = ZERO
        pygame.quit()


def main():
    ap = argparse.ArgumentParser(description="Unitree A2 multi-axis keyboard teleop")
    ap.add_argument("--dry-run", action="store_true",
                    help="no motion commands: test keys, window and camera only")
    ap.add_argument("--no-video", action="store_true", help="don't fetch camera frames")
    ap.add_argument("--iface", default=NETWORK_INTERFACE, help="network interface to the robot")
    args = ap.parse_args()

    print("=" * 70)
    print("Unitree A2 teleop (multi-axis)" + ("  [DRY RUN - no motion commands]" if args.dry_run else ""))
    print(f"Interface {args.iface} | limits vx {MAX_VX} m/s, vy {MAX_VY} m/s, yaw {MAX_VYAW} rad/s,"
          f" combined linear {MAX_LINEAR} m/s")
    print("=" * 70)

    ChannelFactoryInitialize(0, args.iface)

    real_sport = SportClient()
    real_sport.SetTimeout(RPC_TIMEOUT_POSTURE)
    real_sport.Init()

    code, state = get_state(real_sport)
    if code != 0:
        print(f"ERROR: robot did not answer GetState (code {code}).")
        print("  Check: robot on, 'ping 192.168.123.161', interface name, a2py env.")
        sys.exit(1)
    fsm = state.get("fsm_name", "?")
    print(f"Robot state: {fsm} (fsm_id {state.get('fsm_id')}, speed_level {state.get('speed_level')})")

    sport = DryRunSport() if args.dry_run else real_sport

    print("""
PRE-MOTION CHECKLIST
  [ ] 2 m clear around the robot, flat floor, nobody within reach
  [ ] Remote controller ON and in your hand (it is the e-stop)
  [ ] You know: ESC = stop + lie down; remote takes over if anything looks wrong
""")
    if fsm != "STAND_DOWN":
        print(f"NOTE: the robot is not lying down (state {fsm}). It will be sent StandUp/BalanceStand anyway.")
    try:
        answer = input("Type 'stand' and press Enter to stand the robot up (anything else quits): ")
    except (KeyboardInterrupt, EOFError):
        print("\nQuit before standing. Nothing was sent.")
        return
    if answer.strip().lower() != "stand":
        print("Quit before standing. Nothing was sent.")
        return

    sh = Shared()
    signal.signal(signal.SIGINT, lambda *_: sh.stop.set())  # Ctrl+C -> same clean exit as ESC

    print("Standing up... (remote in hand)")
    sport.SetTimeout(RPC_TIMEOUT_POSTURE)
    code = sport.StandUp()
    if code != 0:
        print(f"StandUp failed (code {code}). Sending StandDown and exiting.")
        sport.StandDown()
        return
    time.sleep(3.0)
    code = sport.BalanceStand()
    if code != 0:
        print(f"BalanceStand failed (code {code}). Sending StandDown and exiting.")
        sport.StandDown()
        return
    time.sleep(1.5)
    code, state = get_state(real_sport)
    print(f"Robot state now: {state.get('fsm_name', '?')} (code {code})")

    threads = [threading.Thread(target=control_loop, args=(sport, sh), daemon=True)]
    if not args.no_video:
        video = VideoClient()
        video.SetTimeout(VIDEO_TIMEOUT)
        video.Init()
        threads.append(threading.Thread(target=video_loop, args=(video, sh), daemon=True))
    for t in threads:
        t.start()

    print("Teleop running. Click the window, then hold (and combine) keys.")
    try:
        run_window(sh, args.dry_run)
    finally:
        sh.stop.set()
        for t in threads:
            t.join(timeout=3.0)
        print("Stopping and lying down (StandDown)...")
        sport.SetTimeout(RPC_TIMEOUT_POSTURE)
        sport.StopMove()
        time.sleep(0.5)
        code = sport.StandDown()
        if code != 0:
            print(f"!! StandDown failed (code {code}). Use the REMOTE to lie the robot down.")
        else:
            time.sleep(3.0)
            print("Robot is lying down. Motors were NOT damped. Use the remote or power off as usual.")


if __name__ == "__main__":
    main()
