#!/usr/bin/env python3
"""
A2 PRO battery logger (T7 / objective O1).

Listens to the robot's battery and body-state topics and writes one CSV row
every 0.1 s (10 Hz), so each row says: "at this moment, the two packs had
these volts, amps and SoC, and the robot was doing this".

    /lf/bmsstate            pack 1   (unitree_hg/BmsState, 20 Hz)
    /lf/secondary_bmsstate  pack 2   (unitree_hg/BmsState, 20 Hz)
    /lf/sportmodestate      body     (unitree_go/SportModeState, 20 Hz)

What it does besides copying numbers:
  * ENERGY: adds up V x I x dt for every battery message, because the SoC
    only moves in 1 % steps (about 4.5 Wh per pack). energy_used_wh is the
    total taken out of both packs since the logger started.
  * GAPS: warns when a topic is silent for more than 1 s (target in T7:
    "no gap > 1 s"), and never adds energy across a gap (we don't know what
    happened during it).
  * SUMMARY: on Ctrl+C, writes message counts, rates and gaps to a .txt
    next to the CSV.

Same code for the fake publisher (laptop) and the real robot (PC2): it only
knows topic names, not who publishes them.

Units on our robot (ROBOT_FACTS.md): bmsvoltage[0] mV, current mA with
NEGATIVE = discharge (still to be CONFIRMED in T7). If the check shows the
opposite, run with --flip-current; nothing else changes.

Run (after `a2local` on the laptop, or the ROS env on PC2):
    python3 battery_logger.py
    python3 battery_logger.py --out-dir ~/a2_battery/logs
"""

import argparse
import csv
import math
import os
import time
from datetime import datetime

import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from unitree_go.msg import SportModeState
from unitree_hg.msg import BmsState

GAP_LIMIT_S = 1.0        # a topic silent for longer than this counts as a gap
ROW_PERIOD_S = 0.1       # CSV rows at 10 Hz
N_CELLS = 14

# Provisional activity label from the body state. Raw mode and speed are
# saved too, so labels can be recomputed later with a better rule.
STANDING_DRIFT = 0.05    # m/s: 19 Oct real data: standing still reads 0.002-0.008, slow walking
                         # (teleop max 0.2) averages about 0.11. Lying shows a frozen value (0.05-0.06),
                         # so mode 1 is checked first.
TURNING_RATE = 0.10      # rad/s: teleop turns at up to 0.30. yaw_speed is NOT filled by this
                         # firmware (0 even while turning, 19 Oct), so the IMU gyroscope z is used.


def label_activity(mode, speed, turn_rate, pack_current_a):
    if pack_current_a is not None and pack_current_a > 0.5:
        return "charging"          # energy flowing INTO the packs
    if mode == 1:
        return "lying"
    if speed > STANDING_DRIFT:
        return "walking"
    if abs(turn_rate) > TURNING_RATE:
        return "turning"           # turning on the spot
    return "standing"


def yaw_deg(q):
    """Heading from the IMU quaternion. Our robot sends it w-first: [w, x, y, z]."""
    w, x, y, z = q
    return math.degrees(math.atan2(2.0 * (w * z + x * y), 1.0 - 2.0 * (y * y + z * z)))


class TopicWatch:
    """Remembers the latest message on one topic and watches for gaps."""

    def __init__(self, name):
        self.name = name
        self.msg = None
        self.first_rx = None     # time.monotonic() of the first message
        self.last_rx = None      # time.monotonic() of the last message
        self.count = 0
        self.gaps = []           # list of gap lengths in seconds
        self.in_gap = False

    def received(self, msg, now):
        if self.in_gap:
            length = now - self.last_rx
            self.gaps.append(length)
            self.in_gap = False
            print(f"[logger] {self.name}: back after a {length:.2f} s gap")
        if self.first_rx is None:
            self.first_rx = now
        self.msg, self.last_rx = msg, now
        self.count += 1

    def rate_hz(self):
        # Rate while the topic was alive (first to last message), gaps included.
        if self.count < 2:
            return 0.0
        return (self.count - 1) / (self.last_rx - self.first_rx)

    def check_gap(self, now):
        if self.last_rx is not None and not self.in_gap and now - self.last_rx > GAP_LIMIT_S:
            self.in_gap = True
            print(f"[logger] WARNING {self.name}: no message for > {GAP_LIMIT_S:.0f} s")

    def age_ms(self, now):
        return None if self.last_rx is None else round((now - self.last_rx) * 1000)


class BatteryLogger(Node):
    def __init__(self, args):
        super().__init__("a2_battery_logger")
        self.sign = -1.0 if args.flip_current else 1.0
        self.packs = [TopicWatch("/lf/bmsstate"), TopicWatch("/lf/secondary_bmsstate")]
        self.sport = TopicWatch("/lf/sportmodestate")
        self.energy_wh = [0.0, 0.0]   # energy taken OUT of each pack (negative while charging)
        self.t_start = time.monotonic()

        # Best-effort QoS: matches the robot's publishers whatever their setting.
        for i, w in enumerate(self.packs):
            self.create_subscription(BmsState, w.name,
                                     lambda m, i=i: self.on_bms(i, m), qos_profile_sensor_data)
        self.create_subscription(SportModeState, self.sport.name,
                                 lambda m: self.sport.received(m, time.monotonic()),
                                 qos_profile_sensor_data)

        os.makedirs(args.out_dir, exist_ok=True)
        stamp = datetime.now().strftime("%Y%m%d_%H%M%S")
        self.csv_path = os.path.join(args.out_dir, f"battery_{stamp}.csv")
        self.csv_file = open(self.csv_path, "w", newline="", buffering=1)  # line-buffered: a crash loses at most 1 row
        self.writer = csv.writer(self.csv_file)
        self.writer.writerow(self.header())
        self.rows = 0
        self.create_timer(ROW_PERIOD_S, self.write_row)
        self.get_logger().info(f"logging to {self.csv_path}")

    # ---------- unit conversion: the only place that knows the robot's units ----------
    def pack_values(self, msg):
        volts = msg.bmsvoltage[0] / 1000.0                 # mV -> V
        amps = self.sign * msg.current / 1000.0            # mA -> A, negative = discharge
        cells = [c / 1000.0 for c in msg.cell_vol[:N_CELLS]]
        temps = [t for t in msg.temperature if t != 0]     # unused slots read 0
        return volts, amps, cells, temps

    def on_bms(self, i, msg):
        now = time.monotonic()
        w = self.packs[i]
        # Energy: power x time since the previous message on this topic.
        # Skip the very first message and any step across a gap.
        if w.last_rx is not None and not w.in_gap:
            dt = now - w.last_rx
            if dt <= GAP_LIMIT_S:
                volts, amps, _, _ = self.pack_values(msg)
                load_w = -volts * amps                     # positive while discharging
                self.energy_wh[i] += load_w * dt / 3600.0
        w.received(msg, now)

    # ---------- CSV ----------
    def header(self):
        cols = ["time_iso", "t_s"]
        for p in ("p1", "p2"):
            cols += [f"{p}_soc", f"{p}_soh", f"{p}_v", f"{p}_v_cells", f"{p}_i_a",
                     f"{p}_cell_min_v", f"{p}_cell_max_v", f"{p}_temp_max_c", f"{p}_age_ms"]
        cols += ["load_w", "energy_used_wh", "mode", "speed_mps", "yaw_speed", "gyro_z", "yaw_deg",
                 "pos_x", "pos_y", "pos_z",
                 "sport_age_ms", "activity", "gap"]
        return cols

    def write_row(self):
        now = time.monotonic()
        for w in self.packs + [self.sport]:
            w.check_gap(now)
        if any(w.msg is None for w in self.packs):
            return                                          # wait until both packs have spoken

        row = [datetime.now().isoformat(timespec="milliseconds"), round(now - self.t_start, 3)]
        load_w, currents = 0.0, []
        for w in self.packs:
            volts, amps, cells, temps = self.pack_values(w.msg)
            load_w += -volts * amps
            currents.append(amps)
            row += [w.msg.soc, w.msg.soh, round(volts, 3), round(sum(cells), 3), round(amps, 3),
                    round(min(cells), 3), round(max(cells), 3),
                    max(temps) if temps else "", w.age_ms(now)]

        s = self.sport.msg
        if s is not None:
            speed = math.hypot(s.velocity[0], s.velocity[1])
            gyro_z = s.imu_state.gyroscope[2]               # rad/s around the vertical axis
            row += [round(load_w, 1), round(sum(self.energy_wh), 4), s.mode, round(speed, 3),
                    round(s.yaw_speed, 3), round(gyro_z, 3), round(yaw_deg(s.imu_state.quaternion), 1),
                    round(s.position[0], 3), round(s.position[1], 3),
                    round(s.position[2], 3), self.sport.age_ms(now),
                    label_activity(s.mode, speed, gyro_z, sum(currents))]
        else:
            row += [round(load_w, 1), round(sum(self.energy_wh), 4)] + [""] * 10
        row.append(int(any(w.in_gap for w in self.packs + [self.sport])))
        self.writer.writerow(row)
        self.rows += 1
        if self.rows % 100 == 0:                            # a short status line every 10 s
            self.get_logger().info(
                f"{self.rows} rows | load {load_w:6.1f} W | used {sum(self.energy_wh):7.3f} Wh | "
                f"SoC {self.packs[0].msg.soc}/{self.packs[1].msg.soc} % | {row[-2]}")

    # ---------- end of session ----------
    def summary(self):
        dur = time.monotonic() - self.t_start
        lines = [f"Battery log summary  {self.csv_path}",
                 f"duration {dur:.1f} s, {self.rows} CSV rows, energy used {sum(self.energy_wh):.3f} Wh "
                 f"(pack 1 {self.energy_wh[0]:.3f}, pack 2 {self.energy_wh[1]:.3f})"]
        ok = True
        for w in self.packs + [self.sport]:
            rate = w.rate_hz()
            longest = max(w.gaps) if w.gaps else 0.0
            if w.in_gap:
                lines.append(f"{w.name}: still silent at the end of the log")
                ok = False
            ok = ok and not w.gaps and w.count > 0
            lines.append(f"{w.name}: {w.count} msgs, {rate:.1f} Hz, {len(w.gaps)} gaps > {GAP_LIMIT_S:.0f} s"
                         + (f", longest {longest:.2f} s" if w.gaps else ""))
        lines.append("T7 gap check: PASS (no gap > 1 s)" if ok else "T7 gap check: FAIL (see gaps above)")
        text = "\n".join(lines)
        with open(self.csv_path.replace(".csv", "_summary.txt"), "w") as f:
            f.write(text + "\n")
        print(text)


def main():
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--out-dir", default=os.path.expanduser("~/unitree_a2/battery/logs"))
    ap.add_argument("--flip-current", action="store_true",
                    help="use if T7 shows the robot reports discharge as POSITIVE")
    args = ap.parse_args()

    rclpy.init()
    node = BatteryLogger(args)
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.summary()
        node.csv_file.close()
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
