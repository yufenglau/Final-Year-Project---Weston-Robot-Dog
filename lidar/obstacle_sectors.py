#!/usr/bin/env python3
"""
A2 obstacle sectors: PERCEPTION ONLY (reads LiDAR, never moves the robot). Runs on PC2.

Subscribes to the two Hesai point clouds (from hesai_ros_driver with a2_hesai_config.yaml),
moves every point into the robot's BODY frame, throws away the floor, the robot itself and
anything too high, then reports three clearances:

    front  = distance from the robot's nose to the nearest point in its walking corridor
    left   = nearest point on the left side  (decides which way to turn)
    right  = nearest point on the right side

Body frame (ROS convention): x forward, y left, z up, origin at the body centre.

Output:
    * a status line on the terminal (2 per second)
    * a small JSON packet over UDP to 127.0.0.1:9600 every 0.1 s, for avoid_drive.py:
        {"t": <unix time>, "ok": true, "front": 1.42, "left": 0.85, "right": 2.10,
         "fresh": {"/lidar_a/points": true, ...}}
      "ok" is false when any LiDAR's data is older than --stale seconds.

Modes:
    (default)  sectors + UDP
    --stats    per-LiDAR point counts, nearest point, and floor height per LiDAR:
               use it for the cover test (front vs rear) and to check the mounting transforms

Floor: a tilted PLANE z = a*x + b*y + c is fitted every tick to the low points in a 0.6-2.0 m
ring, and obstacle height = height above that plane. So body pitch/roll (standing, walking) doesn't
turn the far floor into "obstacles". Works lying down and standing. --fixed-floor --floor-z -0.45
switches that off (flat, level floor at that height).

Mounting transforms (--tf-a / --tf-b): "x y z roll pitch yaw" in metres and RADIANS, same
convention as the Hesai driver (R = Rz(yaw) Ry(pitch) Rx(roll), then add x y z).
Take the values from Unitree's own config (guide step 5). Identity until then.

Run (PC2, after sourcing ROS and the driver workspace):
    python3 obstacle_sectors.py --stats
    python3 obstacle_sectors.py --tf-a "0.35 0 0.10 0 0 0" --tf-b "-0.35 0 -0.10 0 3.14159 0"
"""

import argparse
import json
import math
import socket
import time

import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import PointCloud2

# ============================================================================
# CONFIGURATION (all overridable on the command line)
# ============================================================================
TOPIC_A = "/lidar_a/points"     # 192.168.124.20
TOPIC_B = "/lidar_b/points"     # 192.168.124.21
UDP_TARGET = ("127.0.0.1", 9600)

BODY_HALF_LENGTH = 0.55   # m, body centre to nose/tail incl. margin (A2 is about 1 m long: check on the robot)
BODY_HALF_WIDTH = 0.35    # m, body centre to side incl. legs and margin
CORRIDOR_MARGIN = 0.15    # m, extra width either side of the body for the "front" corridor
FLOOR_Z = -0.45           # m, floor height in the body frame when standing (used with --fixed-floor)
FLOOR_CLEARANCE = 0.10    # m, ignore points lower than floor + this (floor, small bumps, body pitch)
MAX_OBSTACLE_HEIGHT = 0.60  # m above the floor; higher points can't be bumped (tables are, check!)
MAX_RANGE = 3.0           # m, ignore points further away (far floor "rises" when the body pitches)
FLOOR_RING = (0.6, 2.0)   # m, horizontal distance band used to estimate the floor height
FLOOR_PERCENTILE = 30     # floor fit starts from the lowest 30 % of points in the ring
FLOOR_FIT_TOL = 0.04      # m, points within this of the plane count as floor in the refit
FLOOR_MAX_SLOPE = 0.20    # |a|, |b| limit (about 11 deg): steeper fits are rejected (walls, clutter)
NO_OBSTACLE = 9.9         # value reported when a sector is empty
PERIM_FRONT = 0.20        # m: safety oval reaches this far beyond the body box at the front ...
PERIM_BACK = 0.10         # m: ... at the back (7 Oct: 0.20 froze it whenever the person following came close) ...
PERIM_SIDE = 0.20         # m: ... and at the sides (7 Oct: 0.30 froze it in narrow corridors). Inside -> teleop freezes
PERIM_EXP = 4.0           # oval shape: |x/ax|^n + |y/ay|^n < 1; n = 4 = rounded oval that keeps a margin at the corners
PERIM_HYST = 0.05         # m: "oval_n_release" counts points in an oval this much bigger (unfreeze only when clear)
MIN_POINTS = 10           # a sector distance is the MIN_POINTS-th nearest point, so a few stray
                          # floor/noise points can't trigger a stop (a real object gives dozens to
                          # hundreds; 6 Oct live speckle: up to 7 faint points 0.3-0.5 m ahead)
STALE_AFTER = 0.3         # s, a cloud older than this makes "ok" false
RATE_HZ = 10.0


def kth_nearest(dist, k):
    """k-th smallest distance (k >= 1), or NO_OBSTACLE if fewer than k points."""
    if len(dist) < k:
        return NO_OBSTACLE
    return float(np.partition(dist, k - 1)[k - 1])


def oval_axes(a):
    """Safety-oval half sizes (front, back, side) in metres from the body centre."""
    return (a.body_half_length + a.perim_front, a.body_half_length + a.perim_back, a.body_half_width + a.perim_side)


def oval_scale(x, y, a):
    """Normalised oval radius: < 1 inside the safety oval around the body box (rounded oval / superellipse,
    centre = body centre, its own reach in front, behind and at the sides)."""
    fx, bx, ay = oval_axes(a)
    ax = np.where(x >= 0, fx, bx)
    n = PERIM_EXP
    return (np.abs(x / ax) ** n + np.abs(y / ay) ** n) ** (1.0 / n)


MOTIONS = ("fwd", "back", "left", "right", "turn_left", "turn_right")


TURN_STEPS_DEG = (5, 10, 15, 20, 25, 30)   # turn check: rotate the body this far each way ...
TURN_CLOSER = 0.01                          # ... blocked if the gap to a point shrinks by more than 1 cm


def _gap(x, y, bl, bw):
    """Distance from points to the body box (0 inside)."""
    return np.hypot(np.maximum(np.abs(x) - bl, 0.0), np.maximum(np.abs(y) - bw, 0.0))


def approaching(x, y, a, min_points):
    """Which of the six teleop motions would bring the body box closer to these points?

    Walking (fwd/back/left/right): the gap's gradient (unit vector away from the box) against the
    point's motion relative to the robot (opposite to the robot's). Negative dot product = gap shrinks.
    Turning on the spot: the body is actually rotated 5..30 deg each way and the smallest gap during
    the turn compared with now, so swinging corners count (they reach hypot(L, W) from the centre)
    but an object diagonally off a corner, further out than the corners reach, doesn't block a turn.
    A motion is blocked when >= min_points points would get closer. Returns {motion: True/False}.
    """
    if len(x) == 0:
        return {m: False for m in MOTIONS}
    bl, bw = a.body_half_length, a.body_half_width
    dx = np.where(np.abs(x) > bl, x - np.sign(x) * bl, 0.0)       # vector from the nearest box point
    dy = np.where(np.abs(y) > bw, y - np.sign(y) * bw, 0.0)
    norm = np.hypot(dx, dy)
    norm[norm < 1e-6] = 1e-6
    gx, gy = dx / norm, dy / norm
    out = {}
    for m, (vx, vy) in (("fwd", (-1.0, 0.0)), ("back", (1.0, 0.0)), ("left", (0.0, -1.0)), ("right", (0.0, 1.0))):
        out[m] = bool(np.count_nonzero(gx * vx + gy * vy < -1e-3) >= min_points)
    g0 = _gap(x, y, bl, bw)
    for m, sign in (("turn_left", 1.0), ("turn_right", -1.0)):
        closest = g0.copy()
        for deg in TURN_STEPS_DEG:
            t = np.radians(deg) * sign                 # robot turns by t -> points turn by -t around the centre
            xr = x * np.cos(t) + y * np.sin(t)
            yr = -x * np.sin(t) + y * np.cos(t)
            closest = np.minimum(closest, _gap(xr, yr, bl, bw))
        out[m] = bool(np.count_nonzero(closest < g0 - TURN_CLOSER) >= min_points)
    return out


def oval_where(x, y):
    """Direction of a body-frame point as a word: front / front-left / left / ... """
    ang = math.degrees(math.atan2(y, x))
    names = ["front", "front-left", "left", "back-left", "back", "back-right", "right", "front-right"]
    return names[int(((ang + 22.5) % 360) // 45)]


def make_tf(text):
    """'x y z roll pitch yaw' -> (3x3 rotation, translation). Hesai convention."""
    x, y, z, roll, pitch, yaw = (float(v) for v in text.split())
    ca, sa = math.cos(roll), math.sin(roll)
    cb, sb = math.cos(pitch), math.sin(pitch)
    cc, sc = math.cos(yaw), math.sin(yaw)
    rot = np.array([
        [cb * cc, sa * sb * cc - ca * sc, sa * sc + ca * sb * cc],
        [cb * sc, ca * cc + sa * sb * sc, ca * sb * sc - sa * cc],
        [-sb,     sa * cb,                ca * cb],
    ], dtype=np.float32)
    return rot, np.array([x, y, z], dtype=np.float32)


def cloud_to_xyz(msg: PointCloud2) -> np.ndarray:
    """PointCloud2 -> Nx3 float32 array, without copying point by point."""
    offs = {f.name: f.offset for f in msg.fields}
    dtype = np.dtype({"names": ["x", "y", "z"], "formats": ["<f4"] * 3,
                      "offsets": [offs["x"], offs["y"], offs["z"]], "itemsize": msg.point_step})
    pts = np.frombuffer(msg.data, dtype=dtype, count=msg.width * msg.height)
    xyz = np.stack([pts["x"], pts["y"], pts["z"]], axis=1)
    return xyz[np.isfinite(xyz).all(axis=1)]


class ObstacleSectors(Node):
    def __init__(self, args):
        super().__init__("a2_obstacle_sectors")
        self.args = args
        self.tf = {args.topic_a: make_tf(args.tf_a), args.topic_b: make_tf(args.tf_b)}
        self.latest = {t: None for t in self.tf}      # topic -> (receive time, Nx3 body-frame points)
        self.raw_stats = {t: None for t in self.tf}
        self.counter = {t: 0 for t in self.tf}
        for topic in self.tf:
            self.create_subscription(PointCloud2, topic, lambda m, t=topic: self.on_cloud(t, m),
                                     qos_profile_sensor_data)
        self.sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        self.last_print = 0.0
        self.floor = args.floor_z                     # floor height under the body centre (= plane c)
        self.plane = np.array([0.0, 0.0, args.floor_z])  # floor z = a*x + b*y + c; refit every tick
        self.plane_fitted = False                     # first good fit replaces the start value at once
        self.oval_n = self.oval_n_release = 0          # points inside the safety oval (see sectors)
        self.oval_where = ""
        self.oval_block = self.oval_block_release = {}
        self.create_timer(1.0 / RATE_HZ, self.tick)
        self.get_logger().info(f"listening to {list(self.tf)}; UDP -> {args.udp_host}:{args.udp_port}"
                               + (" | STATS mode" if args.stats else ""))

    def on_cloud(self, topic, msg):
        self.counter[topic] += 1
        if self.counter[topic] % self.args.every:      # optional decimation if PC2 gets busy
            return
        xyz = cloud_to_xyz(msg)
        if self.args.stats:
            self.raw_stats[topic] = xyz                # sensor frame, for the cover test
        rot, trans = self.tf[topic]
        self.latest[topic] = (time.monotonic(), xyz @ rot.T + trans)

    def update_floor(self, pts):
        """Fit the floor plane z = a*x + b*y + c to low points in a ring around the robot.

        Start from the lowest points, then refit twice using only points close to the plane.
        A plane (not one height) absorbs body pitch/roll, so the far floor stays "floor".
        """
        r = np.hypot(pts[:, 0], pts[:, 1])
        ring = pts[(r > FLOOR_RING[0]) & (r < FLOOR_RING[1])].astype(np.float64)
        if len(ring) < 200:
            return
        sel = ring[ring[:, 2] <= np.percentile(ring[:, 2], FLOOR_PERCENTILE)]
        coef = None
        for _ in range(3):
            if len(sel) < 100:
                return
            A = np.c_[sel[:, 0], sel[:, 1], np.ones(len(sel))]
            coef = np.linalg.lstsq(A, sel[:, 2], rcond=None)[0]
            resid = ring[:, 2] - ring[:, :2] @ coef[:2] - coef[2]
            sel = ring[np.abs(resid) < FLOOR_FIT_TOL]
        if coef is None or abs(coef[0]) > FLOOR_MAX_SLOPE or abs(coef[1]) > FLOOR_MAX_SLOPE:
            return                                    # implausible tilt: keep the previous plane
        self.plane = 0.7 * self.plane + 0.3 * coef if self.plane_fitted else coef
        self.plane_fitted = True
        self.floor = float(self.plane[2])

    def height_above_floor(self, pts):
        a, b, c = self.plane
        return pts[:, 2] - (a * pts[:, 0] + b * pts[:, 1] + c)

    def sectors(self, pts):
        a = self.args
        if not a.fixed_floor:
            self.update_floor(pts)
        x, y = pts[:, 0], pts[:, 1]
        h = self.height_above_floor(pts)
        keep = ((h > a.floor_clearance) & (h < a.max_height)
                & (np.hypot(x, y) < a.max_range))
        inside_body = (np.abs(x) < a.body_half_length) & (np.abs(y) < a.body_half_width)
        keep &= ~inside_body
        x, y = x[keep], y[keep]

        # Safety oval (teleop freezes when >= min_points are inside it)
        r = oval_scale(x, y, a)
        grow = 1.0 + a.perim_hyst / max(a.body_half_width + a.perim_side, 1e-6)
        inside = r < 1.0
        self.oval_n = int(inside.sum())
        self.oval_n_release = int((r < grow).sum())
        self.oval_where = oval_where(float(np.median(x[inside])), float(np.median(y[inside]))) if inside.any() else ""
        # which motions would close the gap (teleop blocks only those; inner oval / release oval)
        self.oval_block = approaching(x[inside], y[inside], a, a.min_points)
        rel_in = r < grow
        self.oval_block_release = approaching(x[rel_in], y[rel_in], a, a.min_points)

        half_corridor = a.body_half_width + a.corridor_margin
        ahead = (x > a.body_half_length) & (np.abs(y) < half_corridor)
        front = kth_nearest(x[ahead] - a.body_half_length, a.min_points)
        # Sides: points beside or ahead of the robot, measured from the body edge.
        side_zone = x > -a.body_half_length
        left_sel = side_zone & (y >= half_corridor * 0.5)
        right_sel = side_zone & (y <= -half_corridor * 0.5)
        left = kth_nearest(np.hypot(np.maximum(x[left_sel] - a.body_half_length, 0),
                                    y[left_sel] - a.body_half_width), a.min_points)
        right = kth_nearest(np.hypot(np.maximum(x[right_sel] - a.body_half_length, 0),
                                     -y[right_sel] - a.body_half_width), a.min_points)
        return round(front, 2), round(left, 2), round(right, 2), int(keep.sum())

    def tick(self):
        now = time.monotonic()
        fresh = {t: v is not None and now - v[0] < self.args.stale for t, v in self.latest.items()}
        clouds = [v[1] for t, v in self.latest.items() if fresh[t]]
        ok = all(fresh.values())
        if clouds:
            front, left, right, used = self.sectors(np.concatenate(clouds))
        else:
            front = left = right = 0.0
            used = 0
        msg = {"t": time.time(), "ok": ok, "front": front, "left": left, "right": right,
               "oval_n": self.oval_n if clouds else 0, "oval_n_release": self.oval_n_release if clouds else 0,
               "oval_where": self.oval_where if clouds else "", "oval_min": self.args.min_points,
               "oval_block": self.oval_block if clouds else {}, "oval_block_release": self.oval_block_release if clouds else {},
               "floor": round(self.floor, 2), "fresh": fresh}
        self.sock.sendto(json.dumps(msg).encode(), (self.args.udp_host, self.args.udp_port))

        if now - self.last_print >= 0.5:
            self.last_print = now
            if self.args.stats:
                self.print_stats()
            else:
                flag = "OK " if ok else "STALE"
                print(f"[{flag}] front {front:4.2f} m | left {left:4.2f} m | right {right:4.2f} m"
                      f" | oval {self.oval_n:4d} pts{' ' + self.oval_where if self.oval_n >= self.args.min_points else ''}"
                      f" | floor {self.floor:+.2f} m | {used} obstacle pts | fresh {[int(f) for f in fresh.values()]}",
                      flush=True)

    def print_stats(self):
        for topic, xyz in self.raw_stats.items():
            if xyz is None or len(xyz) == 0:
                print(f"{topic}: no data yet", flush=True)
                continue
            r = np.linalg.norm(xyz, axis=1)
            near = xyz[r < 1.5]
            zp = np.percentile(near[:, 2], [2, 50]) if len(near) else (float("nan"),) * 2
            print(f"{topic}: {len(xyz):6d} pts | nearest {r.min():4.2f} m | pts<0.3 m {int((r < 0.3).sum()):5d}"
                  f" | z(sensor) 2%={zp[0]:+.2f} 50%={zp[1]:+.2f} within 1.5 m", flush=True)
        for topic, v in self.latest.items():
            if v is None:
                continue
            r = np.hypot(v[1][:, 0], v[1][:, 1])
            ring = v[1][(r > FLOOR_RING[0]) & (r < FLOOR_RING[1]), 2]
            if len(ring) > 200:
                print(f"  {topic} after --tf: floor z {np.percentile(ring, 10):+.2f} m "
                      f"(both LiDARs should agree within a few cm)", flush=True)
        print("-" * 60, flush=True)


def main():
    ap = argparse.ArgumentParser(description="A2 LiDAR obstacle sectors (perception only)")
    ap.add_argument("--topic-a", default=TOPIC_A)
    ap.add_argument("--topic-b", default=TOPIC_B)
    ap.add_argument("--tf-a", default="0 0 0 0 0 0", help='"x y z roll pitch yaw" (m, rad)')
    ap.add_argument("--tf-b", default="0 0 0 0 0 0", help='"x y z roll pitch yaw" (m, rad)')
    ap.add_argument("--floor-z", type=float, default=FLOOR_Z, help="start value / value for --fixed-floor")
    ap.add_argument("--fixed-floor", action="store_true", help="don't estimate the floor, use --floor-z")
    ap.add_argument("--floor-clearance", type=float, default=FLOOR_CLEARANCE)
    ap.add_argument("--max-height", type=float, default=MAX_OBSTACLE_HEIGHT)
    ap.add_argument("--max-range", type=float, default=MAX_RANGE)
    ap.add_argument("--body-half-length", type=float, default=BODY_HALF_LENGTH)
    ap.add_argument("--body-half-width", type=float, default=BODY_HALF_WIDTH)
    ap.add_argument("--corridor-margin", type=float, default=CORRIDOR_MARGIN)
    ap.add_argument("--perim-front", type=float, default=PERIM_FRONT, help="m: safety oval beyond the body at the front")
    ap.add_argument("--perim-back", type=float, default=PERIM_BACK, help="m: safety oval beyond the body at the back")
    ap.add_argument("--perim-side", type=float, default=PERIM_SIDE, help="m: safety oval beyond the body at the sides")
    ap.add_argument("--perim-hyst", type=float, default=PERIM_HYST)
    ap.add_argument("--min-points", type=int, default=MIN_POINTS,
                    help="points needed before a sector reports an obstacle (1 = nearest point)")
    ap.add_argument("--stale", type=float, default=STALE_AFTER)
    ap.add_argument("--every", type=int, default=1, help="use every Nth cloud (lower CPU)")
    ap.add_argument("--udp-host", default=UDP_TARGET[0])
    ap.add_argument("--udp-port", type=int, default=UDP_TARGET[1])
    ap.add_argument("--stats", action="store_true", help="cover test / floor height helper")
    args = ap.parse_args()

    rclpy.init()
    node = ObstacleSectors(args)
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == "__main__":
    main()
