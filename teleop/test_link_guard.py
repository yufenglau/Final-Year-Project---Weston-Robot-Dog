#!/usr/bin/env python3
"""Offline tests for the link guard (LinkMonitor) in a2_teleop_server.py. No robot, no SDK needed.

    python3 ~/unitree_a2/teleop/test_link_guard.py
"""
import sys

# The server imports the SDK only inside main(); nothing to stub for LinkMonitor.
sys.path.insert(0, __file__.rsplit("/", 1)[0])
import a2_teleop_server as s  # noqa: E402

failures = 0


def check(name, cond, detail=""):
    global failures
    print(("PASS " if cond else "FAIL ") + name + (f"  ({detail})" if detail else ""))
    failures += 0 if cond else 1


def feed(mon, t0, t1, rtt, step=0.2):
    """Heartbeat replies every `step` s from t0 to t1 with constant rtt. Returns end time."""
    t = t0
    while t < t1 - 1e-9:
        mon.add_rtt(t, rtt)
        t += step
    return t


# 1. Normal latency = median of the stable period
m = s.LinkMonitor()
t = feed(m, 0, 10, 0.040)
check("baseline = 40 ms after 10 s at 40 ms", abs(m.baseline() - 0.040) < 1e-9, f"{m.baseline()*1000:.0f} ms")
check("stable at 40 ms -> no reason", m.unstable_reason(t, True, 0.05) is None)

# 2. Each trigger
check("disconnected -> unstable", m.unstable_reason(t, False, 0.05) == "laptop disconnected")
check("no vel 1.5 s -> fine (under 1.6 s)", m.unstable_reason(t, True, 1.5) is None)
check("no vel 1.7 s -> loss", "no drive commands" in (m.unstable_reason(t, True, 1.7) or ""))
lr = m.last_reply
check("no hb reply 1.7 s -> loss", "no heartbeat" in (m.unstable_reason(lr + 1.7, True, 0.05) or ""))
check("no hb reply 1.5 s -> fine", m.unstable_reason(lr + 1.5, True, 0.05) is None)

m2 = s.LinkMonitor()
t = feed(m2, 0, 10, 0.040)                       # limit = 2*40+100 = 180 ms
t = feed(m2, t, t + 1.4, 0.300)                  # 1.4 s of 300 ms: not yet 1.6 s
check("300 ms for 1.4 s -> still fine", m2.unstable_reason(m2.last_reply, True, 0.05) is None)
t = feed(m2, t, t + 0.4, 0.300)
r = m2.unstable_reason(m2.last_reply + 0.05, True, 0.05)
check("300 ms for > 1.6 s -> loss", r is not None and "latency" in r, r)
t3 = feed(m2, t, t + 0.6, 0.150)                 # under the 180 ms limit for > 0.3 s resets the timer
check("good for > 0.3 s -> timer resets", m2.high_since is None)

m6 = s.LinkMonitor()                              # the #13 shape: bad, a short dip, bad again
t = feed(m6, 0, 10, 0.040)
t = feed(m6, t, t + 1.0, 0.500)
t = feed(m6, t, t + 0.2, 0.060)                   # one good heartbeat (0.2 s dip)
t = feed(m6, t, t + 0.8, 0.500)
r = m6.unstable_reason(m6.last_reply + 0.05, True, 0.05)
check("bad 1.0 s + 0.2 s dip + bad 0.8 s -> loss", r is not None and "latency" in r, r)
m7 = s.LinkMonitor()
t = feed(m7, 0, 10, 0.040)
t = feed(m7, t, t + 1.0, 0.500)
t = feed(m7, t, t + 0.6, 0.060)                   # 0.6 s good: a real recovery
t = feed(m7, t, t + 0.8, 0.500)
check("bad 1.0 s + 0.6 s good + bad 0.8 s -> not yet", m7.unstable_reason(m7.last_reply + 0.05, True, 0.05) is None)

# 3. Baseline frozen while unstable; recovery needs <= max(1.2x, +20 ms) for 3 s
m3 = s.LinkMonitor()
t = feed(m3, 0, 10, 0.100)                       # normal 100 ms -> ok limit 120 ms
m3.freeze()
t = feed(m3, t, t + 5, 0.500)                    # bad period must not change "normal"
check("baseline frozen at 100 ms during bad link", abs(m3.baseline() - 0.100) < 1e-9, f"{m3.baseline()*1000:.0f} ms")
t = feed(m3, t, t + 2.0, 0.130)                  # 130 > 120: not recovered
held = m3.stable_for(t - 0.2, True, 0.05)
check("130 ms (> 120 limit) -> not stable", held == 0.0)
t_start = t
while t < t_start + 3.8:   # 110 ms <= 120: stable; the 1-s median still holds 130 ms samples for ~0.5 s
    m3.add_rtt(t, 0.110)
    held = m3.stable_for(t, True, 0.05)
    t += 0.1
check("110 ms for 3 s -> stable >= 3 s", held >= 3.0, f"held {held:.1f} s")
m3.add_rtt(t, 0.110)
check("a vel gap resets the stable timer", m3.stable_for(t, True, 0.7) == 0.0)

# 4. Small baseline: +20 ms floor makes recovery possible next to the robot
m4 = s.LinkMonitor()
feed(m4, 0, 10, 0.003)
check("3 ms normal -> ok limit 23 ms (not 3.6 ms)", abs(m4.ok_threshold() - 0.023) < 1e-9,
      f"{m4.ok_threshold()*1000:.1f} ms")

# 5. Before enough samples, defaults apply
m5 = s.LinkMonitor()
check("no samples -> default normal 100 ms", m5.baseline() == s.DEFAULT_BASELINE)
check("no reply ever -> heartbeat rule not applied", m5.unstable_reason(5.0, True, 0.05) is None)

# 6. Speed caps / ramps unchanged
check("caps unchanged", s.clamp_target(9, 0, 9) == (0.5, 0.0, 0.8))
check("stop is immediate", s.ramp((0.4, 0.0, 0.5), s.ZERO, 0.05) == s.ZERO)

print(f"\n{'ALL PASSED' if failures == 0 else f'{failures} FAILED'}")
sys.exit(1 if failures else 0)
