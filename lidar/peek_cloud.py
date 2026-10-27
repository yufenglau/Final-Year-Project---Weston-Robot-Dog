"""Read one point cloud from each A2 LiDAR and print a summary (read-only).

Run on PC2:  source ~/a2_lidar/env.sh && python3 ~/a2_lidar/peek_cloud.py
"""
import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import PointCloud2
from sensor_msgs_py import point_cloud2


class Peek(Node):
    def __init__(self):
        super().__init__("a2_peek_cloud")
        self.done = {}
        for name in ("lidar_a", "lidar_b"):
            self.create_subscription(PointCloud2, f"/{name}/points",
                                     lambda msg, n=name: self.on_cloud(n, msg),
                                     qos_profile_sensor_data)

    def on_cloud(self, name, msg):
        if name in self.done:
            return
        pts = point_cloud2.read_points(msg, field_names=("x", "y", "z"), skip_nans=True)
        xyz = np.stack([pts["x"], pts["y"], pts["z"]], axis=1).astype(np.float64)
        r = np.linalg.norm(xyz, axis=1)
        r = r[r > 0.05]
        print(f"{name}: frame={msg.header.frame_id} fields={[(f.name, f.datatype) for f in msg.fields]}")
        print(f"   points={len(xyz)}  range min/median/max = {r.min():.2f} / {np.median(r):.2f} / {r.max():.2f} m")
        self.done[name] = True


rclpy.init()
node = Peek()
while rclpy.ok() and len(node.done) < 2:
    rclpy.spin_once(node, timeout_sec=1.0)
node.destroy_node()
rclpy.shutdown()
