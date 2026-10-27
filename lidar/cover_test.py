"""Cover test: which LiDAR is front? Prints, once a second, how much of each LiDAR's view is blocked.

Cover ONE LiDAR with your hand (or a box) and watch which line jumps. Read-only. Ctrl+C to stop.
Run on PC2:  source ~/a2_lidar/env.sh && python3 ~/a2_lidar/cover_test.py
"""
import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import PointCloud2

NEAR = 0.30  # m: anything closer than this counts as "blocked"


def ranges(msg):
    offs = {f.name: f.offset for f in msg.fields}
    dt = np.dtype({"names": ["x", "y", "z"], "formats": ["<f4"] * 3,
                   "offsets": [offs["x"], offs["y"], offs["z"]], "itemsize": msg.point_step})
    p = np.frombuffer(msg.data, dtype=dt, count=msg.width * msg.height)
    r = np.sqrt(p["x"].astype(np.float64) ** 2 + p["y"] ** 2 + p["z"] ** 2)
    return r[np.isfinite(r)]


class Cover(Node):
    def __init__(self):
        super().__init__("a2_cover_test")
        self.last = {}
        for n in ("lidar_a", "lidar_b"):
            self.create_subscription(PointCloud2, f"/{n}/points",
                                     lambda m, n=n: self.last.__setitem__(n, m), qos_profile_sensor_data)
        self.create_timer(1.0, self.show)

    def show(self):
        line = []
        for n in ("lidar_a", "lidar_b"):
            m = self.last.get(n)
            if m is None:
                line.append(f"{n}: no data")
                continue
            r = ranges(m)
            valid = r[r > 0.01]
            blocked = 100.0 * np.mean(valid < NEAR) if len(valid) else 0.0
            returns = 100.0 * len(valid) / max(len(r), 1)
            median = np.median(valid) if len(valid) else 0.0
            line.append(f"{n}: blocked {blocked:5.1f}%  returns {returns:5.1f}%  median {median:4.2f} m")
        print("   |   ".join(line), flush=True)


rclpy.init()
try:
    rclpy.spin(Cover())
except KeyboardInterrupt:
    pass
