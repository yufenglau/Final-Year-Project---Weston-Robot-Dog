"""Run on PC2: source ~/a2_lidar/env.sh && python3 ~/a2_lidar/check_mounting.py  (needs about 1.5 m of open floor around the robot).
Apply candidate mounting transforms and check the floor comes out flat and level for both LiDARs."""
import sys, math, numpy as np, rclpy
sys.path.insert(0, "/home/unitree/a2_lidar")
from obstacle_sectors import make_tf, cloud_to_xyz
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import PointCloud2

TF = {"lidar_a": "0.31 0 0 1.5708 0 1.5708", "lidar_b": "-0.31 0 0 1.5708 0 -1.5708"}

def plane(a, it=500, tol=0.02, rng=np.random.default_rng(1)):
    best = (0, None, None)
    for _ in range(it):
        s = a[rng.choice(len(a), 3, replace=False)]
        n = np.cross(s[1]-s[0], s[2]-s[0]); nn = np.linalg.norm(n)
        if nn < 1e-9: continue
        n = n / nn
        if n[2] < 0: n = -n
        d = n @ s[0]
        k = int((np.abs(a @ n - d) < tol).sum())
        if k > best[0]: best = (k, n, d)
    return best

class N(Node):
    def __init__(s):
        super().__init__("a2_check_tf"); s.got = {}
        for t in TF:
            s.create_subscription(PointCloud2, f"/{t}/points", lambda m, t=t: s.got.setdefault(t, m), qos_profile_sensor_data)

rclpy.init(); n = N()
while len(n.got) < 2: rclpy.spin_once(n, timeout_sec=1.0)
for t, txt in TF.items():
    rot, tr = make_tf(txt)
    a = cloud_to_xyz(n.got[t]).astype(np.float64)
    a = a[np.linalg.norm(a, axis=1) > 0.35] @ rot.T.astype(np.float64) + tr
    low = a[a[:, 2] < 0.0]
    k, nrm, d = plane(low)
    tilt = math.degrees(math.acos(min(1.0, nrm[2])))
    horiz = np.hypot(a[:, 0], a[:, 1])
    print(f"== {t} tf=\"{txt}\"")
    print(f"   floor: z = {d:.3f} m, normal [{nrm[0]:.2f} {nrm[1]:.2f} {nrm[2]:.2f}] (tilt {tilt:.1f} deg), {k} pts")
    print(f"   points ahead (x>0): {100*np.mean(a[:,0] > 0):.0f}%   behind: {100*np.mean(a[:,0] < 0):.0f}%")
    print(f"   z percentiles p1/p50/p99: {np.percentile(a[:,2],1):.2f} / {np.percentile(a[:,2],50):.2f} / {np.percentile(a[:,2],99):.2f}")
n.destroy_node(); rclpy.shutdown()
