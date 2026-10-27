#!/usr/bin/env python3
"""
Fake A2 battery + body-state publisher (a stand-in for the real robot).

Publishes the SAME topics, message types and units as our A2 PRO, so the
battery logger can be tested on the laptop before robot day:

    /lf/bmsstate            unitree_hg/BmsState      20 Hz   (pack 1)
    /lf/secondary_bmsstate  unitree_hg/BmsState      20 Hz   (pack 2)
    /lf/sportmodestate      unitree_go/SportModeState 20 Hz  (body state)

Units copied from the real robot (ROBOT_FACTS.md, 1 Oct 2026):
    bmsvoltage[0] in mV, current in mA with NEGATIVE = discharge (inferred,
    still to be confirmed in T7), cell_vol[0..13] in mV, soc in whole %.

The robot "does" a scripted list of activities, e.g. lie 30 s, stand 30 s,
walk 60 s ... and the battery drains according to a simple model:

    pack power  = activity power / 2           (two packs share the load)
    pack OCV    = 14 x cell voltage(SoC)       (curve fitted to OUR readings)
    pack V      = OCV - I x R                  (internal resistance sag)
    pack I      = pack power / pack V
    energy     -= pack power x dt              ->  new SoC

Only "lie" power (87 W) is measured. The others are GUESSES, to be replaced
by real numbers after the data campaign (T10/T14).

Run (after `a2local`):
    python3 fake_bms_publisher.py
    python3 fake_bms_publisher.py --schedule lie:10,walk:20,charge:10
    python3 fake_bms_publisher.py --gap-at 15 --gap-len 2.5   # test gap detection
"""

import argparse
import math
import random
import time

import rclpy
from rclpy.node import Node
from unitree_go.msg import SportModeState
from unitree_hg.msg import BmsState

PACK_CAPACITY_WH = 453.6   # 9 Ah x 50.4 V per pack (product page)
PACK_R_OHM = 0.06          # internal resistance per pack: GUESS
N_CELLS = 14

# Total robot power per activity, in watts (positive = drawn from battery).
# "charge" is negative: the pad pushes energy INTO the battery.
ACTIVITY_POWER_W = {
    "lie": 87.0,      # MEASURED on our robot, 1-2 Oct (computers on, lying)
    "stand": 160.0,   # guess
    "walk": 300.0,    # guess, about 0.3 m/s
    "charge": -250.0, # guess: charger in (about 5 A x 57 V) minus the 87 W the computers use
}

# Body state per activity. mode 1 = lying, 3 = standing (both seen on our
# robot). Which mode number the A2 reports while WALKING is not known yet.
ACTIVITY_BODY = {
    "lie":    dict(mode=1, height=0.06, speed=0.0),
    "stand":  dict(mode=3, height=0.45, speed=0.0),
    "walk":   dict(mode=3, height=0.45, speed=0.30),
    "charge": dict(mode=1, height=0.06, speed=0.0),
}

# Cell voltage vs SoC, from OUR robot's readings (SoC %, volts per cell).
# Outside this range we extend the straight line and clamp to 3.3-4.2 V.
OCV_POINTS = [(56, 3.781), (74, 3.948), (78, 3.993), (95, 4.092)]


def cell_ocv(soc):
    """Open-circuit cell voltage for a SoC, by straight-line interpolation."""
    pts = OCV_POINTS
    if soc <= pts[0][0]:
        (s0, v0), (s1, v1) = pts[0], pts[1]
    elif soc >= pts[-1][0]:
        (s0, v0), (s1, v1) = pts[-2], pts[-1]
    else:
        for (s0, v0), (s1, v1) in zip(pts, pts[1:]):
            if s0 <= soc <= s1:
                break
    v = v0 + (soc - s0) * (v1 - v0) / (s1 - s0)
    return min(4.2, max(3.3, v))


class FakePack:
    """One battery pack: keeps its own energy, works out V and I."""

    def __init__(self, start_soc, soh, cycle, temp_offset):
        self.energy_wh = PACK_CAPACITY_WH * start_soc / 100.0
        self.soh, self.cycle = soh, cycle
        self.temp_c = 25.0 + temp_offset
        self.current_a = 0.0   # positive = discharging (inside this class only)

    @property
    def soc(self):
        return 100.0 * self.energy_wh / PACK_CAPACITY_WH

    def step(self, power_w, dt):
        ocv = N_CELLS * cell_ocv(self.soc)
        volts = ocv - self.current_a * PACK_R_OHM      # sag when discharging
        self.current_a = power_w / volts
        self.energy_wh -= power_w * dt / 3600.0
        self.energy_wh = min(PACK_CAPACITY_WH, max(0.0, self.energy_wh))
        # Slowly warm up with load, cool down toward 25 C (very rough).
        self.temp_c += (25.0 + abs(power_w) / 40.0 - self.temp_c) * dt / 300.0
        return volts

    def to_msg(self, volts):
        msg = BmsState()
        cell_mv = volts / N_CELLS * 1000.0
        cells = [int(cell_mv + random.uniform(-2, 2)) for _ in range(N_CELLS)]
        msg.cell_vol = cells + [0] * (40 - N_CELLS)          # rest unused, as on the robot
        msg.bmsvoltage = [int(volts * 1000), int(volts * 1000) - 20, 0]
        msg.current = int(round(-self.current_a * 1000))     # robot style: mA, negative = discharge
        msg.soc = int(round(self.soc))                       # robot reports whole %
        msg.soh = self.soh
        msg.cycle = self.cycle
        t = int(round(self.temp_c))
        msg.temperature = [t, t + 2, t + 4, t + 1] + [0] * 8  # which slots the A2 fills: unknown
        msg.bmsstate = [560, 0, 0, 0, 14, ]                  # copied from the real robot
        return msg


def parse_schedule(text):
    plan = []
    for item in text.split(","):
        name, secs = item.split(":")
        if name not in ACTIVITY_POWER_W:
            raise SystemExit(f"unknown activity '{name}', use one of {list(ACTIVITY_POWER_W)}")
        plan.append((name, float(secs)))
    return plan


class FakeRobot(Node):
    def __init__(self, args):
        super().__init__("fake_a2_battery")
        self.args = args
        self.plan = parse_schedule(args.schedule)
        self.packs = [FakePack(args.start_soc, 95, 2, 0.0), FakePack(args.start_soc + 1, 97, 2, 1.0)]
        self.pub_bms = [self.create_publisher(BmsState, "/lf/bmsstate", 10),
                        self.create_publisher(BmsState, "/lf/secondary_bmsstate", 10)]
        self.pub_sport = self.create_publisher(SportModeState, "/lf/sportmodestate", 10)
        self.t0 = time.monotonic()
        self.t_last = self.t0
        self.x = 0.0
        self.last_activity = None
        self.create_timer(1.0 / 20.0, self.tick)   # 20 Hz like the real robot

    def activity_at(self, t):
        total = sum(s for _, s in self.plan)
        if t >= total and not self.args.loop:
            return None
        t = t % total
        for name, secs in self.plan:
            if t < secs:
                return name
            t -= secs
        return self.plan[-1][0]

    def tick(self):
        now = time.monotonic()
        dt, self.t_last = now - self.t_last, now
        t = now - self.t0
        act = self.activity_at(t)
        if act is None:
            self.get_logger().info("schedule finished, stopping")
            raise SystemExit
        if act != self.last_activity:
            self.get_logger().info(f"t={t:6.1f} s  activity -> {act}")
            self.last_activity = act

        # Split the load unevenly (52/48), like our two packs' slightly different currents.
        total_w = ACTIVITY_POWER_W[act] + random.gauss(0, 3.0)
        for pack, share, pub in zip(self.packs, (0.52, 0.48), self.pub_bms):
            volts = pack.step(total_w * share, dt)
            # Simulated dropout: pack 2 goes silent for a while (tests gap detection).
            gap = (self.args.gap_at is not None and pub is self.pub_bms[1]
                   and self.args.gap_at <= t < self.args.gap_at + self.args.gap_len)
            if not gap:
                pub.publish(pack.to_msg(volts))

        body = ACTIVITY_BODY[act]
        speed = body["speed"]
        if speed == 0.0 and body["mode"] == 3:
            speed = random.uniform(0.05, 0.11)   # the real robot's estimator drifts while standing
        self.x += speed * dt
        msg = SportModeState()
        msg.mode = body["mode"]
        msg.velocity = [speed + random.gauss(0, 0.01), random.gauss(0, 0.01), 0.0]
        msg.position = [self.x, 0.0, body["height"]]
        msg.body_height = 0.0                    # our firmware leaves this at 0
        self.pub_sport.publish(msg)


def main():
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--schedule", default="lie:30,stand:30,walk:60,stand:20,lie:30,charge:30",
                    help="activity:seconds list (activities: %s)" % ", ".join(ACTIVITY_POWER_W))
    ap.add_argument("--start-soc", type=float, default=80.0)
    ap.add_argument("--loop", action="store_true", help="repeat the schedule forever")
    ap.add_argument("--gap-at", type=float, default=None, help="seconds: pack 2 goes silent here")
    ap.add_argument("--gap-len", type=float, default=2.0, help="seconds of silence")
    args = ap.parse_args()

    rclpy.init()
    node = FakeRobot(args)
    node.get_logger().info("fake A2 publishing /lf/bmsstate, /lf/secondary_bmsstate, /lf/sportmodestate at 20 Hz")
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, SystemExit):
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
