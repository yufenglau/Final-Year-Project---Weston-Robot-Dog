#!/usr/bin/env python3
"""
A2 LiDAR live 3D view: both LiDARs as a rotatable 3D point cloud in your browser. READ-ONLY.

Runs on PC2. Subscribes to /lidar_a/points and /lidar_b/points, moves the points into the robot's
body frame (same transforms, floor plane and safety oval as obstacle_sectors.py), thins them out on
a voxel grid, packs them small (int16 cm + 1 byte class, zlib) and serves the newest frame over HTTP.
The web page (three.js) draws them in 3D: drag = rotate, right-drag/shift-drag = pan, wheel = zoom.

Bandwidth (shown live on the page): about 0.5-1 Mbit/s at the defaults. Over 4G, don't watch it
WHILE driving (the 2D map at 5 Mbit/s made the teleop link guard trip, 7 Oct).

Colours: grey = floor, orange -> red = obstacle band (what the brake/oval count; red = taller),
blue = above the band, bright red = inside the safety oval.

Run (PC2):
    source ~/a2_lidar/env.sh && python3 ~/a2_lidar/lidar_view3d.py
Laptop:
    ssh -N -L 8096:127.0.0.1:8096 a2-pc2        # leave running
    open http://localhost:8096
"""
import argparse
import json
import os
import struct
import threading
import time
import types
import zlib
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer

import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import PointCloud2

from obstacle_sectors import (BODY_HALF_LENGTH, BODY_HALF_WIDTH, CORRIDOR_MARGIN, FLOOR_CLEARANCE,
                              FLOOR_Z, MAX_OBSTACLE_HEIGHT, MAX_RANGE, MIN_POINTS, PERIM_BACK, PERIM_EXP, PERIM_FRONT,
                              PERIM_HYST, PERIM_SIDE, ObstacleSectors, cloud_to_xyz, make_tf, oval_axes,
                              oval_scale)

TF_A = "0.31 0 0 1.5708 0 1.5708"     # front LiDAR (192.168.124.20), verified 2026-10-06
TF_B = "-0.31 0 0 1.5708 0 -1.5708"   # rear LiDAR (192.168.124.21), verified 2026-10-06
HERE = os.path.dirname(os.path.abspath(__file__))

# point classes sent to the browser
FLOOR, OBST, HIGH, OVAL = 0, 1, 2, 3


def voxel_pick(pts, size):
    """One point per voxel of `size` metres (the first one found). Returns indices."""
    if len(pts) == 0:
        return np.zeros(0, np.int64)
    keys = np.floor(pts / size).astype(np.int64)
    keys -= keys.min(axis=0)
    span = keys.max(axis=0) + 1
    flat = (keys[:, 0] * span[1] + keys[:, 1]) * span[2] + keys[:, 2]
    _, idx = np.unique(flat, return_index=True)
    return idx


class View3D(Node):
    def __init__(self, args):
        super().__init__("a2_lidar_view3d")
        self.args = args
        self.tf = {"/lidar_a/points": make_tf(args.tf_a), "/lidar_b/points": make_tf(args.tf_b)}
        self.latest = {t: None for t in self.tf}
        for topic in self.tf:
            self.create_subscription(PointCloud2, topic, lambda m, t=topic: self.on_cloud(t, m),
                                     qos_profile_sensor_data)
        # obstacle_sectors.py's own floor plane, sector and oval maths, so everything matches the brake
        self.sec = types.SimpleNamespace(args=args, floor=args.floor_z, plane=np.array([0.0, 0.0, args.floor_z]),
                                         plane_fitted=False, oval_n=0, oval_n_release=0, oval_where="")
        for name in ("update_floor", "height_above_floor"):
            setattr(self.sec, name, types.MethodType(getattr(ObstacleSectors, name), self.sec))
        self.lock = threading.Lock()
        self.frame = None            # compressed bytes of the newest frame
        self.frame_id = 0
        self.create_timer(1.0 / args.fps, self.build)

    def on_cloud(self, topic, msg):
        xyz = cloud_to_xyz(msg)
        xyz = xyz[np.einsum("ij,ij->i", xyz, xyz) > 1e-4]
        rot, trans = self.tf[topic]
        self.latest[topic] = (time.monotonic(), xyz @ rot.T + trans)

    def build(self):
        a = self.args
        now = time.monotonic()
        fresh = {t[1:8]: v is not None and now - v[0] < a.stale for t, v in self.latest.items()}
        clouds = [v[1] for v in self.latest.values() if v is not None]
        if not clouds:
            return
        pts = np.concatenate(clouds)
        front, left, right, _ = ObstacleSectors.sectors(self.sec, pts)
        pts = pts[np.hypot(pts[:, 0], pts[:, 1]) < a.range]
        h = self.sec.height_above_floor(pts)
        body = (np.abs(pts[:, 0]) < a.body_half_length) & (np.abs(pts[:, 1]) < a.body_half_width)
        floor = h <= a.floor_clearance
        high = h >= a.max_height
        band = ~floor & ~high
        oval = band & ~body & (oval_scale(pts[:, 0], pts[:, 1], a) < 1.0)

        cls = np.full(len(pts), OBST, np.uint8)
        cls[floor], cls[high], cls[oval] = FLOOR, HIGH, OVAL
        keep = []
        for sel, size in ((floor, a.voxel_floor), (high, a.voxel_floor), (band, a.voxel)):
            i = np.where(sel)[0]
            keep.append(i[voxel_pick(pts[i], size)])
        idx = np.concatenate(keep)
        if len(idx) > a.max_points:                      # keep obstacles first, thin the rest
            pri = np.where(cls[idx] >= OBST, 0, 1)
            idx = idx[np.argsort(pri, kind="stable")][:a.max_points]

        p = np.clip(np.round(pts[idx] * 100), -32767, 32767).astype("<i2")       # cm
        hcm = np.clip(np.round(h[idx] * 100), 0, 255).astype(np.uint8)            # cm above floor (colour)
        status = {"front": front, "left": left, "right": right, "oval_n": self.sec.oval_n,
                  "oval_where": self.sec.oval_where, "freeze": self.sec.oval_n >= a.min_points,
                  "floor": round(self.sec.floor, 3),
                  "tilt": [round(float(np.degrees(np.arctan(self.sec.plane[i]))), 1) for i in (0, 1)],
                  "fresh": fresh, "n": int(len(idx)), "t": time.strftime("%H:%M:%S"),
                  "body": [a.body_half_length, a.body_half_width],
                  "oval": list(oval_axes(a)), "oval_exp": PERIM_EXP,
                  "band": [a.floor_clearance, a.max_height], "fps": a.fps,
                  "lidars": [[float(v[1][0]), float(v[1][1]), float(v[1][2])] for v in self.tf.values()]}
        js = json.dumps(status).encode()
        raw = struct.pack("<II", len(js), len(idx)) + js + p.tobytes() + cls[idx].tobytes() + hcm.tobytes()
        comp = zlib.compress(raw, 6)
        with self.lock:
            self.frame = comp
            self.frame_id += 1


def serve(node, port):
    page = open(os.path.join(HERE, "web", "view3d.html"), "rb").read()
    three = os.path.join(HERE, "web", "three.min.js")

    class Handler(BaseHTTPRequestHandler):
        def log_message(self, *_):
            pass

        def send(self, code, ctype, body):
            self.send_response(code)
            self.send_header("Content-Type", ctype)
            self.send_header("Cache-Control", "no-store")
            self.send_header("Content-Length", str(len(body)))
            self.end_headers()
            self.wfile.write(body)

        def do_GET(self):
            if self.path in ("/", "/index.html"):
                self.send(200, "text/html; charset=utf-8", page)
            elif self.path == "/three.min.js":
                with open(three, "rb") as f:
                    self.send(200, "application/javascript", f.read())
            elif self.path.startswith("/frame"):
                after = -1
                if "after=" in self.path:
                    try:
                        after = int(self.path.split("after=")[1].split("&")[0])
                    except ValueError:
                        pass
                deadline = time.monotonic() + 2.0           # long-poll: wait for a newer frame
                while True:
                    with node.lock:
                        fid, data = node.frame_id, node.frame
                    if data is not None and fid > after:
                        self.send_response(200)
                        self.send_header("Content-Type", "application/octet-stream")
                        self.send_header("Cache-Control", "no-store")
                        self.send_header("X-Frame-Id", str(fid))
                        self.send_header("Content-Length", str(len(data)))
                        self.end_headers()
                        self.wfile.write(data)
                        return
                    if time.monotonic() > deadline:
                        self.send(204, "text/plain", b"")
                        return
                    time.sleep(0.05)
            else:
                self.send(404, "text/plain", b"not found")

    httpd = ThreadingHTTPServer(("127.0.0.1", port), Handler)   # localhost only: reach it via SSH tunnel
    httpd.daemon_threads = True
    threading.Thread(target=httpd.serve_forever, daemon=True).start()


def main():
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--tf-a", default=TF_A)
    ap.add_argument("--tf-b", default=TF_B)
    ap.add_argument("--port", type=int, default=8096)
    ap.add_argument("--fps", type=float, default=2.0, help="frames per second sent (bandwidth!)")
    ap.add_argument("--range", type=float, default=6.0, help="m: points further away (horizontally) are dropped")
    ap.add_argument("--voxel", type=float, default=0.05, help="m: grid for obstacle points")
    ap.add_argument("--voxel-floor", type=float, default=0.15, help="m: grid for floor and high points")
    ap.add_argument("--max-points", type=int, default=15000)
    ap.add_argument("--stale", type=float, default=0.5)
    # same settings as obstacle_sectors.py
    ap.add_argument("--floor-z", type=float, default=FLOOR_Z)
    ap.add_argument("--fixed-floor", action="store_true")
    ap.add_argument("--floor-clearance", type=float, default=FLOOR_CLEARANCE)
    ap.add_argument("--max-height", type=float, default=MAX_OBSTACLE_HEIGHT)
    ap.add_argument("--max-range", type=float, default=MAX_RANGE)
    ap.add_argument("--body-half-length", type=float, default=BODY_HALF_LENGTH)
    ap.add_argument("--body-half-width", type=float, default=BODY_HALF_WIDTH)
    ap.add_argument("--corridor-margin", type=float, default=CORRIDOR_MARGIN)
    ap.add_argument("--min-points", type=int, default=MIN_POINTS)
    ap.add_argument("--perim-front", type=float, default=PERIM_FRONT)
    ap.add_argument("--perim-back", type=float, default=PERIM_BACK)
    ap.add_argument("--perim-side", type=float, default=PERIM_SIDE)
    ap.add_argument("--perim-hyst", type=float, default=PERIM_HYST)
    args = ap.parse_args()

    rclpy.init()
    node = View3D(args)
    serve(node, args.port)
    node.get_logger().info(f"3D view on http://127.0.0.1:{args.port} (PC2 only; tunnel it), {args.fps} fps")
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass


if __name__ == "__main__":
    main()
