#!/usr/bin/env python3
"""Offline test of obstacle_sectors.py's floor-plane fit and sectors (no ROS traffic, no robot).

Synthetic scenes in the body frame: floor at lying (-0.15) / standing (-0.45 / -0.54) height,
tilted by body pitch/roll, a 0.4 m wide x 0.5 m tall box at a known distance ahead, a wall 1.5 m
to the right. Checks that the tilted floor is NOT reported as an obstacle and the box distance is right.

    a2local && python3 ~/unitree_a2/lidar/test_floor_plane.py      (or on PC2 with env.sh)
"""
import math
import types

import numpy as np

from obstacle_sectors import BODY_HALF_LENGTH, BODY_HALF_WIDTH, ObstacleSectors

rng = np.random.default_rng(0)


def scene(floor_z, pitch_deg, roll_deg, box_front, strays=0, post=None):
    """Points in body frame. Positive pitch = nose down (the floor ahead appears higher)."""
    g = np.mgrid[-3:3:0.03, -3:3:0.03].reshape(2, -1).T
    g = g[np.hypot(g[:, 0], g[:, 1]) > 0.4]
    fz = floor_z + math.tan(math.radians(pitch_deg)) * g[:, 0] - math.tan(math.radians(roll_deg)) * g[:, 1]
    floor = np.c_[g, fz + rng.normal(0, 0.01, len(g))]
    pts = [floor]
    if box_front is not None:
        x0 = BODY_HALF_LENGTH + box_front
        yy, hh = np.mgrid[-0.2:0.2:0.02, 0.02:0.5:0.02].reshape(2, -1)
        base = floor_z + math.tan(math.radians(pitch_deg)) * x0
        pts.append(np.c_[np.full(yy.size, x0), yy, base + hh])
    if strays:  # a few faint points just over the 10 cm threshold, about 1 m ahead (seen live 6 Oct)
        x0 = BODY_HALF_LENGTH + 0.45
        pts.append(np.c_[np.full(strays, x0), np.linspace(0.1, 0.2, strays),
                         floor_z + math.tan(math.radians(pitch_deg)) * x0 + 0.11 + np.zeros(strays)])
    if post is not None:  # small real object: 5 cm wide x 15 cm tall post, JT128-like spacing at that range
        x0 = BODY_HALF_LENGTH + post
        rng_ = x0 - 0.31
        step_h, step_v = rng_ * math.radians(0.4), rng_ * math.radians(0.74)
        yy, hh = np.mgrid[-0.025:0.025:step_h, 0.0:0.15:step_v].reshape(2, -1)
        base = floor_z + math.tan(math.radians(pitch_deg)) * x0
        pts.append(np.c_[np.full(yy.size, x0), yy, base + hh])
    xx, hh = np.mgrid[-2.5:2.5:0.03, 0.02:1.5:0.03].reshape(2, -1)
    pts.append(np.c_[xx, np.full(xx.size, -1.5), floor_z + hh])
    return np.concatenate(pts).astype(np.float32)


def make_sec(floor_start):
    args = types.SimpleNamespace(fixed_floor=False, floor_clearance=0.10, max_height=0.60, max_range=3.0,
                                 body_half_length=BODY_HALF_LENGTH, body_half_width=BODY_HALF_WIDTH,
                                 corridor_margin=0.15, min_points=10, perim_front=0.20, perim_back=0.10,
                                 perim_side=0.20, perim_hyst=0.05)
    s = types.SimpleNamespace(args=args, floor=floor_start, plane=np.array([0.0, 0.0, floor_start]),
                              plane_fitted=False, oval_n=0, oval_n_release=0, oval_where="")
    for name in ("update_floor", "height_above_floor"):
        setattr(s, name, types.MethodType(getattr(ObstacleSectors, name), s))
    return s


cases = [  # (name, floor_z, pitch, roll, box distance from nose or None[, stray points])
    ("lying, level, no box", -0.15, 0.0, 0.0, None),
    ("lying, level, box 1.00 m", -0.15, 0.0, 0.0, 1.00),
    ("standing, level, box 1.00 m", -0.45, 0.0, 0.0, 1.00),
    ("standing, nose-down 1.8 deg (today), no box", -0.54, 1.8, 0.0, None),
    ("standing, nose-down 1.8 deg, box 1.00 m", -0.54, 1.8, 0.0, 1.00),
    ("walking rock: nose-down 4 deg + roll 3 deg, box 0.60 m", -0.45, 4.0, 3.0, 0.60),
    ("nose-up 3 deg, box 2.00 m", -0.45, -3.0, 0.0, 2.00),
    ("standing 1.8 deg, 4 stray noise points 0.45 m ahead", -0.54, 1.8, 0.0, None, 4),
    ("standing 1.8 deg, strays + box 1.00 m", -0.54, 1.8, 0.0, 1.00, 4),
    ("standing 1.8 deg, 7 stray points (worst seen live)", -0.54, 1.8, 0.0, None, 7),
    ("standing, small post 5x15 cm at 0.60 m (must be seen)", -0.54, 1.8, 0.0, None, 0, 0.60),
    ("standing, small post 5x15 cm at 1.50 m (must be seen)", -0.54, 1.8, 0.0, None, 0, 1.50),
]
fails = 0
for name, fz, pitch, roll, box, *extra in cases:
    s = make_sec(-0.45)
    pts = scene(fz, pitch, roll, box, *extra)
    for _ in range(5):                                   # a few ticks, like the live node
        front, left, right, used = ObstacleSectors.sectors(s, pts)
    post = extra[1] if len(extra) > 1 else None
    want = box if box is not None else (post if post is not None else 9.9)
    ok = abs(front - want) < 0.06 and right > 0.9
    fails += not ok
    print(f"{'PASS' if ok else 'FAIL'}  {name:55s} front {front:4.2f} (want {want:4.2f}) | left {left:4.2f} "
          f"right {right:4.2f} | floor {s.floor:+.2f} tilt {math.degrees(math.atan(s.plane[0])):+.1f}/"
          f"{math.degrees(math.atan(s.plane[1])):+.1f} deg | {used} pts")

# --- safety oval
def block(cx, cy, w=0.10, h=0.30):
    """A 10 cm x 10 cm x 30 cm object centred at (cx, cy), on the floor at -0.54."""
    xx, yy, hh = np.mgrid[cx - w / 2:cx + w / 2:0.02, cy - w / 2:cy + w / 2:0.02, 0.02:h:0.03].reshape(3, -1)
    return np.c_[xx, yy, -0.54 + hh].astype(np.float32)

oval_cases = [  # name, object centre (x, y), expect freeze, expect release-count >= min
    # oval (7 Oct): 0.20 m beyond the body box in front, 0.10 m behind, 0.20 m at the sides; body box 0.55 x 0.35
    ("object 0.10 m in front of the nose",         (0.55 + 0.10, 0.0),   True,  True),
    ("object face 0.30 m in front (outside)",      (0.55 + 0.35, 0.0),   False, False),
    ("object face 0.12 m from the left side",      (0.0, 0.35 + 0.17),   True,  True),
    ("object face 0.22 m from the left side",      (0.0, 0.35 + 0.27),   False, True),
    ("object face 0.40 m from the left side",      (0.0, 0.35 + 0.45),   False, False),
    ("object face 0.10 m from the right side",     (0.0, -(0.35 + 0.15)), True, True),
    ("object face 0.03 m behind the tail",         (-(0.55 + 0.08), 0.0), True,  True),
    ("person following, face 0.15 m behind",       (-(0.55 + 0.20), 0.0), False, True),
    ("person following, face 0.30 m behind",       (-(0.55 + 0.35), 0.0), False, False),
    ("object at the front-left corner, close",     (0.62, 0.42),         True,  True),
    ("object at the rear-left corner, close",      (-0.60, 0.40),        True,  True),
]
for name, (cx, cy), want_in, want_rel in oval_cases:
    sec = make_sec(-0.45)
    pts = np.concatenate([scene(-0.54, 1.8, 0.0, None), block(cx, cy)])
    for _ in range(5):
        ObstacleSectors.sectors(sec, pts)
    got_in = sec.oval_n >= 10
    got_rel = sec.oval_n_release >= 10
    ok = got_in == want_in and got_rel == want_rel
    fails += not ok
    print(f"{'PASS' if ok else 'FAIL'}  oval: {name:42s} inside {sec.oval_n:4d} pts ({sec.oval_where or '-'}) "
          f"| release-oval {sec.oval_n_release:4d} -> {'FREEZE' if got_in else 'clear'}")

# --- direction blocking: an object in the oval blocks only the motions that close the gap
B = {"fwd", "back", "left", "right", "turn_left", "turn_right"}
dir_cases = [  # name, object centre, motions that MUST be blocked, motions that MUST stay allowed
    # corner objects (further from the centre than the corners reach): only the two walking directions toward them
    # (the front-corner test box is only 2 cm ahead of the nose line: turning TOWARD it swings the front edge
    #  under it, so that turn may be blocked; turning AWAY must stay allowed)
    ("front-left corner",  (0.62, 0.42),   {"fwd", "left"},   {"back", "right", "turn_right"}),
    ("front-right corner", (0.62, -0.42),  {"fwd", "right"},  {"back", "left", "turn_left"}),
    ("back-left corner",   (-0.62, 0.42),  {"back", "left"},  {"fwd", "right", "turn_left", "turn_right"}),
    ("back-right corner",  (-0.62, -0.42), {"back", "right"}, {"fwd", "left", "turn_left", "turn_right"}),
    # objects the swinging corners would hit: turning blocked too, but you can always walk away
    ("straight ahead",     (0.65, 0.0),    {"fwd", "turn_left", "turn_right"},  {"back", "left", "right"}),
    ("straight behind",    (-0.63, 0.0),   {"back", "turn_left", "turn_right"}, {"fwd", "left", "right"}),
    ("left side, middle",  (0.0, 0.47),    {"left", "turn_left", "turn_right"}, {"fwd", "back", "right"}),
    ("right side, middle", (0.0, -0.47),   {"right", "turn_left", "turn_right"}, {"fwd", "back", "left"}),
    # object beside the front half on the left: nose swings left into it -> only turn left blocked
    ("left side, front half", (0.40, 0.47), {"left", "turn_left"}, {"fwd", "back", "right", "turn_right"}),
]
for name, (cx, cy), must_block, must_allow in dir_cases:
    sec = make_sec(-0.45)
    pts = np.concatenate([scene(-0.54, 1.8, 0.0, None), block(cx, cy)])
    for _ in range(5):
        ObstacleSectors.sectors(sec, pts)
    blk = {m for m, v in sec.oval_block.items() if v}
    ok = sec.oval_n >= 10 and must_block <= blk and not (must_allow & blk)
    fails += not ok
    print(f"{'PASS' if ok else 'FAIL'}  direction: object {name:20s} blocked {sorted(blk)} | allowed {sorted(B - blk)}")
print("ALL PASSED" if not fails else f"{fails} FAILED")
