#!/usr/bin/env python3
"""Offline tests for the teleop LiDAR brake + safety oval (a2_teleop_server.lidar_brake). No robot, no network.

    python3 ~/unitree_a2/teleop/test_lidar_brake.py
"""
import itertools
import math

import a2_teleop_server as srv

Z = (0.0, 0.0, 0.0)


def L(front, oval_n=0, oval_rel=None, ok=True, oval_min=10, block=None, block_rel=None):
    d = {"ok": ok, "front": front, "left": 9.9, "right": 9.9, "oval_n": oval_n,
         "oval_n_release": oval_n if oval_rel is None else oval_rel, "oval_where": "front", "oval_min": oval_min}
    if block is not None:
        d["oval_block"] = {m: m in block for m in srv.OVAL_MOTIONS}
        d["oval_block_release"] = {m: m in (block if block_rel is None else block_rel) for m in srv.OVAL_MOTIONS}
    return d


FL = {"fwd", "left", "turn_left"}        # what an object at the front-left corner blocks
BR = {"back", "right"}                   # ... at the back-right corner


FWD = (0.40, 0.0, 0.0)
ALL = (0.30, 0.10, 0.50)        # forward + sideways + turning at once
S, B = srv.BRAKE_STOP, srv.BRAKE_SLOW
cases = [
    # name, target, lidar msg, age, expected target out, expected state[, prev_state]
    ("clear road: full speed",                    ALL, L(9.9), 0.1, ALL, "clear"),
    ("front at slow edge: full forward",          FWD, L(B), 0.1, FWD, "clear"),
    ("mid slow zone: half forward",               FWD, L((S + B) / 2), 0.1, (0.20, 0.0, 0.0), "slow"),
    ("just above stop: crawl, not below",         FWD, L(S + 0.02), 0.1, (0.10, 0.0, 0.0), "slow"),
    ("asked slower than crawl: not raised",       (0.05, 0.0, 0.0), L((S + B) / 2), 0.1, (0.05, 0.0, 0.0), "slow"),
    ("front at stop: forward blocked",            FWD, L(S), 0.1, Z, "stop"),
    ("stopped, wobble +3 cm: stays stop",         FWD, L(S + 0.03), 0.1, Z, "stop", "stop"),
    ("forward stop: reverse still allowed",       (-0.2, 0.0, 0.0), L(S - 0.05), 0.1, (-0.2, 0.0, 0.0), "stop"),
    ("forward stop: turn + sideways allowed",     (0.0, 0.1, 0.5), L(S - 0.05), 0.1, (0.0, 0.1, 0.5), "stop"),
    ("OVAL no direction info -> ALL frozen",      ALL, L(9.9, oval_n=10), 0.1, Z, "freeze"),
    ("OVAL no direction info: reverse frozen",    (-0.3, 0.0, 0.0), L(9.9, oval_n=50), 0.1, Z, "freeze"),
    ("OVAL front-left: fwd+left+turnL cut",       (0.3, 0.1, 0.5), L(9.9, oval_n=50, block=FL), 0.1, Z, "freeze"),
    ("OVAL front-left: back allowed",             (-0.3, 0.0, 0.0), L(9.9, oval_n=50, block=FL), 0.1, (-0.3, 0.0, 0.0), "freeze"),
    ("OVAL front-left: right + turn right ok",    (0.0, -0.1, -0.5), L(9.9, oval_n=50, block=FL), 0.1, (0.0, -0.1, -0.5), "freeze"),
    ("OVAL front-left: W+D -> only right kept",   (0.3, -0.1, 0.0), L(9.9, oval_n=50, block=FL), 0.1, (0.0, -0.1, 0.0), "freeze"),
    ("OVAL back-right: forward allowed",          (0.3, 0.0, 0.0), L(9.9, oval_n=50, block=BR), 0.1, (0.3, 0.0, 0.0), "freeze"),
    ("OVAL back-right: back cut",                 (-0.3, 0.0, 0.0), L(9.9, oval_n=50, block=BR), 0.1, Z, "freeze"),
    ("OVAL + front brake: fwd still braked",      (0.4, 0.0, 0.0), L(S - 0.05, oval_n=50, block=BR), 0.1, Z, "freeze"),
    ("OVAL held by release band (dir. kept)",     (0.3, 0.0, 0.0), L(9.9, oval_n=0, oval_rel=40, block=set(), block_rel={"fwd"}), 0.1, Z, "freeze", "freeze"),
    ("OVAL: 9 pts (speckle) -> not frozen",       ALL, L(9.9, oval_n=9), 0.1, ALL, "clear"),
    ("OVAL frozen, object in release band: stays", ALL, L(9.9, oval_n=0, oval_rel=40), 0.1, Z, "freeze", "freeze"),
    ("bad direction info (not bool): ALL frozen", ALL, {**L(9.9, oval_n=50), "oval_block": {m: 1 for m in srv.OVAL_MOTIONS}}, 0.1, Z, "freeze"),
    ("OVAL frozen, object gone: released",        ALL, L(9.9, oval_n=0, oval_rel=0), 0.1, ALL, "clear", "freeze"),
    ("not frozen, object only in release band",   ALL, L(9.9, oval_n=0, oval_rel=40), 0.1, ALL, "clear", "clear"),
    ("no message yet: ALL frozen",                ALL, None, None, Z, "no data"),
    ("stale data 0.6 s: ALL frozen",              ALL, L(9.9), 0.6, Z, "no data"),
    ("a LiDAR missing (ok false): ALL frozen",    ALL, L(9.9, ok=False), 0.1, Z, "no data"),
    ("old obstacle_sectors (no oval): ALL frozen", ALL, {"ok": True, "front": 9.9}, 0.1, Z, "no data"),
    ("garbage front: ALL frozen",                 ALL, L("x"), 0.1, Z, "no data"),
    ("NaN oval count: ALL frozen",                ALL, L(9.9, oval_n=float("nan")), 0.1, Z, "no data"),
    ("bool oval count: ALL frozen",               ALL, L(9.9, oval_n=True), 0.1, Z, "no data"),
]
print(f"settings: stop {S} m, slow {B} m, hysteresis {srv.BRAKE_HYST} m, crawl {srv.BRAKE_CRAWL} m/s")
fails = 0
for name, target, msg, age, want, want_state, *prev in cases:
    out, state, front = srv.lidar_brake(target, msg, age, *prev)
    ok = all(abs(a - b) < 1e-9 for a, b in zip(out, want)) and state == want_state
    fails += not ok
    print(f"{'PASS' if ok else 'FAIL'}  {name:44s} {tuple(round(v, 2) for v in target)} -> "
          f"{tuple(round(v, 2) for v in out)} | {state}")

# never ADDS motion: every axis keeps its sign and never grows
combos = list(itertools.product([-0.5, -0.1, 0.0, 0.05, 0.1, 0.5], [0.0, S - 0.01, S, S + 0.03, S + 0.06, B, 1.0, 9.9],
                                [0, 9, 10, 200], [0, 10], [0.0, 0.49, 0.51, None], [True, False],
                                ["starting", "clear", "slow", "stop", "freeze", "no data"],
                                [None, set(), FL, BR, set(srv.OVAL_MOTIONS)], [(0.2, -0.3), (-0.2, 0.3)]))
for vx, front, on, orel, age, okf, prev, blk, (vy0, w0) in combos:
    tgt = (vx, vy0, w0)
    out, _, _ = srv.lidar_brake(tgt, L(front, oval_n=on, oval_rel=orel, ok=okf, block=blk), age, prev)
    for a, b in zip(out, tgt):
        if abs(a) > abs(b) + 1e-12 or (a != 0 and math.copysign(1, a) != math.copysign(1, b)):
            fails += 1
            print(f"FAIL  never-adds-motion: {tgt} front {front} oval {on}/{orel} age {age} ok {okf} prev {prev} -> {out}")
            break
print(f"never-adds-motion sweep: {len(combos)} combinations checked")
print("ALL PASSED" if not fails else f"{fails} FAILED")
