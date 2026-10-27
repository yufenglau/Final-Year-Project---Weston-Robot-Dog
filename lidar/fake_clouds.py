#!/usr/bin/env python3
"""
Offline test helper (laptop): publishes FAKE point clouds on /lidar_a/points and /lidar_b/points
so obstacle_sectors.py and avoid_drive.py can be tested without the robot.

Scene (body frame, identity mounting): floor at z = -0.45 m, a box 0.4 m wide in front that
moves from 3 m to 0.2 m and back over 30 s, and a wall 0.9 m to the right.

    python3 fake_clouds.py
    python3 fake_clouds.py --drop-b-after 20     # lidar_b goes silent after 20 s (stale test)
"""

import argparse
import time

import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import PointCloud2, PointField

FIELDS = [PointField(name="x", offset=0, datatype=PointField.FLOAT32, count=1),
          PointField(name="y", offset=4, datatype=PointField.FLOAT32, count=1),
          PointField(name="z", offset=8, datatype=PointField.FLOAT32, count=1),
          PointField(name="intensity", offset=12, datatype=PointField.FLOAT32, count=1),
          PointField(name="ring", offset=16, datatype=PointField.UINT16, count=1),
          PointField(name="timestamp", offset=18, datatype=PointField.FLOAT64, count=1)]
DTYPE = np.dtype({"names": ["x", "y", "z", "intensity", "ring", "timestamp"],
                  "formats": ["<f4", "<f4", "<f4", "<f4", "<u2", "<f8"],
                  "offsets": [0, 4, 8, 12, 16, 18], "itemsize": 26})


def scene(t, fz=-0.45):
    rng = np.random.default_rng()
    floor = np.column_stack([rng.uniform(-4, 4, 20000), rng.uniform(-4, 4, 20000), np.full(20000, fz)])
    d = 0.2 + 2.8 * abs(((t / 15.0) % 2) - 1)            # 3.0 -> 0.2 -> 3.0 m from the nose
    x0 = 0.55 + d
    box = np.column_stack([np.full(3000, x0), rng.uniform(-0.2, 0.2, 3000), rng.uniform(fz, fz + 0.65, 3000)])
    wall = np.column_stack([rng.uniform(-2, 4, 4000), np.full(4000, -1.25), rng.uniform(fz, fz + 1.25, 4000)])
    body = np.column_stack([rng.uniform(-0.4, 0.4, 2000), rng.uniform(-0.2, 0.2, 2000), rng.uniform(-0.4, 0.1, 2000)])
    return np.vstack([floor, box, wall, body]).astype(np.float32), d


class Fake(Node):
    def __init__(self, args):
        super().__init__("fake_clouds")
        self.args = args
        self.pubs = {t: self.create_publisher(PointCloud2, t, qos_profile_sensor_data)
                     for t in ("/lidar_a/points", "/lidar_b/points")}
        self.t0 = time.monotonic()
        self.create_timer(0.1, self.tick)

    def tick(self):
        t = time.monotonic() - self.t0
        pts, d = scene(t, self.args.floor_z)
        half = len(pts) // 2
        for i, (topic, pub) in enumerate(self.pubs.items()):
            if topic == "/lidar_b/points" and self.args.drop_b_after and t > self.args.drop_b_after:
                continue
            part = pts[:half] if i == 0 else pts[half:]
            arr = np.zeros(len(part), dtype=DTYPE)
            arr["x"], arr["y"], arr["z"] = part[:, 0], part[:, 1], part[:, 2]
            msg = PointCloud2()
            msg.header.stamp = self.get_clock().now().to_msg()
            msg.header.frame_id = topic.split("/")[1]
            msg.height, msg.width = 1, len(part)
            msg.fields, msg.is_bigendian, msg.point_step = FIELDS, False, DTYPE.itemsize
            msg.row_step, msg.is_dense = DTYPE.itemsize * len(part), True
            msg.data = arr.tobytes()
            pub.publish(msg)
        if int(t * 10) % 20 == 0:
            print(f"t={t:5.1f}s  box {d:4.2f} m ahead of the nose", flush=True)


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--drop-b-after", type=float, default=0.0)
    ap.add_argument("--floor-z", type=float, default=-0.45, help="-0.10 ~ robot lying down")
    args = ap.parse_args()
    rclpy.init()
    node = Fake(args)
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    rclpy.try_shutdown()


if __name__ == "__main__":
    main()
