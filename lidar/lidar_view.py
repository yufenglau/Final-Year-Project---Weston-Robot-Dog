#!/usr/bin/env python3
"""
A2 LiDAR live view: top-down map of both LiDARs in the robot's frame, in your browser. READ-ONLY.

Runs on PC2. Subscribes to /lidar_a/points and /lidar_b/points, moves the points into the
robot's body frame with the same transforms as obstacle_sectors.py, and serves a live picture:

    * robot body box (same size obstacle_sectors.py uses), nose arrow, front walking corridor
    * points coloured by height above the floor:
        grey   = floor (ignored)
        orange/red = obstacle band (what obstacle_sectors.py counts), red = taller
        blue   = above the obstacle band (ignored, e.g. table tops, walls high up)
    * distance rings every 0.5 m (from the body centre), front/left/right clearances

Robot forward is UP on the screen, robot left is LEFT on the screen.

Run (PC2):
    source ~/a2_lidar/env.sh && python3 ~/a2_lidar/lidar_view.py
Laptop:
    ssh -N -L 8095:127.0.0.1:8095 a2-pc2        # leave running
    open http://localhost:8095                  # live view; /snapshot.jpg = one frame
"""
import argparse
import threading
import time
import types
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer

import cv2
import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import PointCloud2

from obstacle_sectors import (BODY_HALF_LENGTH, BODY_HALF_WIDTH, CORRIDOR_MARGIN, FLOOR_CLEARANCE,
                              FLOOR_Z, MAX_OBSTACLE_HEIGHT, MAX_RANGE, MIN_POINTS, PERIM_BACK, PERIM_EXP, PERIM_FRONT,
                              PERIM_HYST, PERIM_SIDE, ObstacleSectors, cloud_to_xyz, make_tf, oval_axes)

TF_A = "0.31 0 0 1.5708 0 1.5708"     # front LiDAR (192.168.124.20), verified 2026-10-06
TF_B = "-0.31 0 0 1.5708 0 -1.5708"   # rear LiDAR (192.168.124.21), verified 2026-10-06

PAGE = b"""<!doctype html><html><head><meta charset="utf-8"><title>A2 LiDAR view</title>
<style>body{margin:0;background:#111;color:#ddd;font:14px sans-serif;text-align:center}
img{max-width:100vw;max-height:100vh}</style></head>
<body><img src="/stream.mjpg" alt="LiDAR stream"></body></html>"""


class LidarView(Node):
    def __init__(self, args):
        super().__init__("a2_lidar_view")
        self.args = args
        self.tf = {"/lidar_a/points": make_tf(args.tf_a), "/lidar_b/points": make_tf(args.tf_b)}
        self.latest = {t: None for t in self.tf}       # topic -> (receive time, Nx3 body-frame points)
        for topic in self.tf:
            self.create_subscription(PointCloud2, topic, lambda m, t=topic: self.on_cloud(t, m),
                                     qos_profile_sensor_data)
        # Reuse obstacle_sectors.py's own floor + sector maths, so the numbers match it exactly.
        self.sec = types.SimpleNamespace(args=args, floor=args.floor_z, plane=np.array([0.0, 0.0, args.floor_z]),
                                         plane_fitted=False, oval_n=0, oval_n_release=0, oval_where="")
        for name in ("update_floor", "height_above_floor"):
            setattr(self.sec, name, types.MethodType(getattr(ObstacleSectors, name), self.sec))
        self.jpeg = None
        self.lock = threading.Lock()
        self.create_timer(1.0 / args.fps, self.render)

    def on_cloud(self, topic, msg):
        xyz = cloud_to_xyz(msg)
        xyz = xyz[np.einsum("ij,ij->i", xyz, xyz) > 1e-4]   # drop "no return" points (0,0,0)
        rot, trans = self.tf[topic]
        self.latest[topic] = (time.monotonic(), xyz @ rot.T + trans)

    def to_px(self, x, y):
        """Robot frame (x fwd, y left) -> image pixel (u right, v down). Forward = up."""
        a = self.args
        c = a.size // 2
        return (c - y * a.scale).astype(np.int32), (c - x * a.scale).astype(np.int32)

    def render(self):
        a = self.args
        now = time.monotonic()
        fresh = {t: v is not None and now - v[0] < a.stale for t, v in self.latest.items()}
        clouds = [v[1] for t, v in self.latest.items() if v is not None]
        img = np.full((a.size, a.size, 3), 20, np.uint8)
        c = a.size // 2

        # distance rings + axes
        for r in np.arange(0.5, a.range + 0.01, 0.5):
            col = (70, 70, 70) if r % 1 else (110, 110, 110)
            cv2.circle(img, (c, c), int(r * a.scale), col, 1, cv2.LINE_AA)
            if r % 1 == 0:
                cv2.putText(img, f"{r:.0f} m", (c + 4, c - int(r * a.scale) + 14),
                            cv2.FONT_HERSHEY_SIMPLEX, 0.4, (140, 140, 140), 1, cv2.LINE_AA)

        front = left = right = None
        if clouds:
            pts = np.concatenate(clouds)
            front, left, right, _ = ObstacleSectors.sectors(self.sec, pts)
            h = self.sec.height_above_floor(pts)                       # height above the fitted floor plane
            u, v = self.to_px(pts[:, 0], pts[:, 1])
            on = (u >= 0) & (u < a.size) & (v >= 0) & (v < a.size)
            low = on & (h <= a.floor_clearance)
            band = on & (h > a.floor_clearance) & (h < a.max_height)
            high = on & (h >= a.max_height)
            img[v[high], u[high]] = (90, 55, 20)                         # dim blue (BGR), drawn first
            img[v[low], u[low]] = (110, 110, 110)
            t = np.clip((h[band] - a.floor_clearance) / (a.max_height - a.floor_clearance), 0, 1)
            band_col = np.stack([np.zeros_like(t), 200 * (1 - t) + 30, np.full_like(t, 255)], 1)
            img[v[band], u[band]] = band_col.astype(np.uint8)            # orange -> red with height
            # thicken obstacle points so small objects are visible
            if a.dot > 1:
                mask = np.zeros((a.size, a.size), np.uint8)
                mask[v[band], u[band]] = 255
                fat = cv2.dilate(mask, np.ones((a.dot, a.dot), np.uint8)) > 0
                img[fat & (mask == 0)] = (0, 140, 255)

        # safety oval (red when something is inside it -> teleop freezes all motion)
        frozen = clouds and self.sec.oval_n >= a.min_points
        fx, bx, ay = oval_axes(a)
        t = np.linspace(0, 2 * np.pi, 180)
        ct, st_ = np.cos(t), np.sin(t)
        ox = np.sign(ct) * np.abs(ct) ** (2 / PERIM_EXP) * np.where(ct >= 0, fx, bx)
        oy = np.sign(st_) * np.abs(st_) ** (2 / PERIM_EXP) * ay
        ou, ov = self.to_px(ox, oy)
        cv2.polylines(img, [np.c_[ou, ov].astype(np.int32)], True, (60, 60, 255) if frozen else (200, 200, 0),
                      3 if frozen else 1, cv2.LINE_AA)

        # robot body box, corridor, nose arrow, LiDAR positions
        bl, bw = a.body_half_length, a.body_half_width
        hc = bw + a.corridor_margin
        (u0, u1), (v0, v1) = self.to_px(np.array([0.0, 0.0]), np.array([bw, -bw]))[0], \
            self.to_px(np.array([bl, -bl]), np.array([0.0, 0.0]))[1]
        cv2.rectangle(img, (int(u0), int(v0)), (int(u1), int(v1)), (255, 255, 255), 2)
        cu = self.to_px(np.array([0.0, 0.0]), np.array([hc, -hc]))[0]
        vtop = int(c - a.range * a.scale)
        for uu in cu:
            cv2.line(img, (int(uu), int(v0)), (int(uu), max(vtop, 0)), (60, 160, 60), 1, cv2.LINE_AA)
        cv2.arrowedLine(img, (c, c), (c, int(v0) - 25), (255, 255, 255), 2, cv2.LINE_AA, tipLength=0.3)
        for name, (rot, tr) in zip(("A front", "B rear"), self.tf.values()):
            pu, pv = self.to_px(np.array([tr[0]]), np.array([tr[1]]))
            cv2.circle(img, (int(pu[0]), int(pv[0])), 5, (0, 255, 255), -1)
            cv2.putText(img, name, (int(pu[0]) + 8, int(pv[0]) + 4), cv2.FONT_HERSHEY_SIMPLEX, 0.4,
                        (0, 255, 255), 1, cv2.LINE_AA)
        if front is not None and front < 9:
            vf = int(v0 - front * a.scale)
            cv2.line(img, (int(cu[0]), vf), (int(cu[1]), vf), (0, 255, 0), 2)
        cv2.putText(img, "LEFT", (8, c), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (200, 200, 200), 1, cv2.LINE_AA)
        cv2.putText(img, "RIGHT", (a.size - 62, c), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (200, 200, 200), 1,
                    cv2.LINE_AA)

        # text panel
        ok = all(fresh.values())
        fmt = lambda d: "--" if d is None else ("clear" if d >= 9 else f"{d:.2f} m")
        oval = (f"   OVAL: {self.sec.oval_n} pts {self.sec.oval_where} -> FREEZE" if clouds and self.sec.oval_n >= a.min_points
                else f"   oval clear ({self.sec.oval_n if clouds else 0} pts)")
        lines = [f"front {fmt(front)}   left {fmt(left)}   right {fmt(right)}" + oval,
                 f"floor {self.sec.floor:+.2f} m  tilt {np.degrees(np.arctan(self.sec.plane[0])):+.1f}/{np.degrees(np.arctan(self.sec.plane[1])):+.1f} deg   LiDAR A {'OK' if fresh['/lidar_a/points'] else 'STALE'}"
                 f"   LiDAR B {'OK' if fresh['/lidar_b/points'] else 'STALE'}   {time.strftime('%H:%M:%S')}",
                 "grey floor | orange/red obstacle | blue too high | green = front | oval: cyan clear, RED = freeze"]
        cv2.rectangle(img, (0, 0), (a.size, 70), (0, 0, 0), -1)
        for i, s in enumerate(lines):
            cv2.putText(img, s, (10, 20 + 22 * i), cv2.FONT_HERSHEY_SIMPLEX, 0.5 if i < 2 else 0.42,
                        (255, 255, 255) if ok or i else (0, 0, 255), 1, cv2.LINE_AA)
        good, buf = cv2.imencode(".jpg", img, [cv2.IMWRITE_JPEG_QUALITY, 80])
        if good:
            with self.lock:
                self.jpeg = buf.tobytes()


def serve(node, port):
    class Handler(BaseHTTPRequestHandler):
        def log_message(self, *_):
            pass

        def frame(self):
            with node.lock:
                return node.jpeg

        def do_GET(self):
            if self.path in ("/", "/index.html"):
                self.send_response(200)
                self.send_header("Content-Type", "text/html")
                self.end_headers()
                self.wfile.write(PAGE)
            elif self.path.startswith("/snapshot.jpg"):
                j = self.frame()
                self.send_response(200 if j else 503)
                self.send_header("Content-Type", "image/jpeg")
                self.end_headers()
                self.wfile.write(j or b"")
            elif self.path.startswith("/stream.mjpg"):
                self.send_response(200)
                self.send_header("Content-Type", "multipart/x-mixed-replace; boundary=frame")
                self.end_headers()
                try:
                    while True:
                        j = self.frame()
                        if j:
                            self.wfile.write(b"--frame\r\nContent-Type: image/jpeg\r\n\r\n" + j + b"\r\n")
                        time.sleep(1.0 / node.args.fps)
                except (BrokenPipeError, ConnectionResetError):
                    pass
            else:
                self.send_response(404)
                self.end_headers()

    httpd = ThreadingHTTPServer(("127.0.0.1", port), Handler)   # localhost only: reach it via SSH tunnel
    httpd.daemon_threads = True
    threading.Thread(target=httpd.serve_forever, daemon=True).start()
    return httpd


def main():
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--tf-a", default=TF_A)
    ap.add_argument("--tf-b", default=TF_B)
    ap.add_argument("--port", type=int, default=8095)
    ap.add_argument("--fps", type=float, default=5.0)
    ap.add_argument("--range", type=float, default=4.0, help="metres shown from the centre")
    ap.add_argument("--size", type=int, default=800, help="image size in pixels")
    ap.add_argument("--dot", type=int, default=3, help="obstacle dot size in pixels")
    ap.add_argument("--stale", type=float, default=0.5)
    # same sector settings as obstacle_sectors.py
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
    args.scale = (args.size / 2 - 10) / args.range

    rclpy.init()
    node = LidarView(args)
    serve(node, args.port)
    node.get_logger().info(f"live view on http://127.0.0.1:{args.port} (PC2 only; tunnel it to the laptop)")
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass


if __name__ == "__main__":
    main()
