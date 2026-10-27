# AI-Based Battery Management and Autonomous Docking for a Quadruped Inspection Robot

Final-year individual project (BEng Robotics and AI, University of Hertfordshire / PSB Academy), on the **Unitree A2 PRO** quadruped from Weston Robot.

The robot carries a 360° camera around construction sites. This project covers the part that keeps it running: **measuring its battery, predicting how long it can keep working, deciding when it must go back to charge, and lining it up on its charging pad.**

*Last updated: 27 Oct 2026 (end of Week 7). Work in progress.*

---

## 1. The idea

A fixed rule like "go back at 25 %" ignores how far away the charger is and what the robot is doing. The plan is instead:

```
go back when:  energy left  ≤  energy needed to reach the pad  +  safety margin
```

To make that work, the robot needs:

1. **Real battery data** with what the robot was doing at each moment (lying, standing, walking, turning).
2. **An energy model**: how many watts each activity and each sensor costs.
3. **An AI model** that predicts remaining working time from live data, compared against simple baselines.
4. **A decision (state machine)**: MISSION → RETURNING → DOCKING → CHARGING.
5. **Docking**: the robot lies down on Unitree's **Contact Charging Board**, so two electrodes under its body touch two plates on the pad. Unitree only documents placing the robot by hand, so **automating this alignment is the main engineering contribution.**

Navigation around the site, sound-based safety alarms and the 360° camera are teammates' projects on the same robot. This project uses their outputs (route to the pad, alarm events, camera power draw).

---

## 2. Progress so far

| Stage | Status | Result |
|---|---|---|
| Robot set-up, interfaces, risk assessment (30 hazards, signed) | ✅ Done | Battery, joints, IMU, camera, microphone, speaker, 4G and both LiDARs read from the laptop and the robot's own computer (PC2) |
| LiDAR safety layer for driving | ✅ Done | Two Hesai JT128 LiDARs on PC2; a LiDAR brake and safety perimeter inside the teleop. Shown at the 7 Oct exhibition |
| **Battery logger (T7)** | ✅ **Passed 20 Oct** | 1 hour, **0 gaps**, 98.6 Wh measured. 11 % SoC drop → about 9 Wh per 1 % → about 900 Wh pack, matching Unitree's 907 Wh spec. Runs on PC2 since 22 Oct |
| Current sign check | ✅ Done | Negative current = discharging (more negative when standing and walking) |
| **Data campaign 1 (T10)** | 🟡 In progress | About 6 h logged, mostly lying. Walking and turning blocks still needed |
| **Energy model (T14)** | 🟡 First numbers | See the table below |
| AI prediction model (T17) | ⏳ Planned (Nov) | Will be trained on real logs only |
| Return decision, "decision-only" demo | ⏳ Planned (Nov) | Decides when to return; does not walk the robot yet |
| Docking on the pad | ⏳ Part B (Feb–Apr 2027) | **The charging pad has not arrived yet.** Lie-down drift test on a taped pad-sized box comes first |

### First energy numbers (both packs together, real robot)

| Robot state | Power |
|---|---|
| Lying, nothing extra running | about 86–89 W |
| Standing still | about 110–123 W |
| Walking slowly (about 0.1 m/s) | about 175–180 W |
| Standing up | short peaks up to about 300 W |
| + LiDAR driver | **+7 W** |
| + obstacle detection | +1 W |
| + teleop camera stream | about +6 W |
| + battery logger on PC2 | **about +3 W** (on/off test, 27 Oct) |

Rough runtime while lying still: 900 Wh ÷ 90 W ≈ **10 hours**. Walking costs about twice the lying power, so walking runtime will be much shorter. Measuring that properly is the job of T10.

---

## 3. What's in this repository

```
battery/   battery logger, power report, fake battery for laptop tests
lidar/     LiDAR obstacle detection, safety perimeter, set-up checks, live maps
teleop/    Wi-Fi teleop window with camera, LiDAR brake, link-loss guard, talk/listen
docs/      team manual and obstacle-detection guide
```

| File | What it does |
|---|---|
| `battery/battery_logger.py` | Records both battery packs and the robot's state 10 times a second to CSV. Adds up energy (V × I over time), labels each row (lying / standing / walking / turning / charging), checks for gaps over 1 s, writes a PASS/FAIL summary |
| `battery/power_report.py` | Average power per activity or per time block from a log, with an optional power + SoC plot |
| `battery/fake_bms_publisher.py` | A pretend robot on the laptop: same topics, message types and units as the A2, for testing without the robot |
| `lidar/obstacle_sectors.py` | Reads both LiDAR point clouds, removes the floor, finds the nearest obstacle in each direction and checks the safety perimeter around the body. Sends the result to the teleop |
| `lidar/a2_hesai_config.yaml` | Driver settings for the two JT128 LiDARs (front `.20`, rear `.21`) |
| `lidar/check_mounting.py`, `lidar/cover_test.py` | Checks after any repair: LiDAR tilt and direction, and which LiDAR is the front one |
| `lidar/lidar_view.py`, `lidar/lidar_view3d.py`, `lidar/web/` | Live 2D / 3D LiDAR map in a browser |
| `lidar/test_floor_plane.py`, `lidar/fake_clouds.py`, `lidar/peek_cloud.py` | Tests and debugging helpers |
| `teleop/a2_teleop_wifi_gui.py` + `teleop/a2_teleop_server.py` | Laptop window (camera, speed levels, combined keys) and the robot-side server on PC2. The server enforces speed caps, the **LiDAR brake** and the **link-loss guard** |
| `teleop/a2_teleop.py` | Simple cable-only keyboard teleop (no PC2 needed) |
| `teleop/a2_talk*.py`, `teleop/a2_listen*.py` | Talk through the robot's speaker; listen to its microphone |
| `teleop/test_lidar_brake.py`, `teleop/test_link_guard.py` | Offline tests: the brake can only slow or stop the robot, never add motion |
| `docs/A2_TEAM_MANUAL.md` | Step-by-step team manual: connecting, reading data, LiDARs, teleop, battery logger, safety rules |
| `docs/OBSTACLE_AVOIDANCE_GUIDE.md` | How the LiDAR obstacle detection works |

The Hesai LiDAR driver itself is not included; it is built from Hesai's own repository (see References).

### Earlier simulation code (removed)

Before the robot was available, the repository held four simulation scripts (simulated battery, Wh-per-metre energy model, state machine, and a Random Forest trained on made-up data). They were removed on 27 Oct 2026 because real robot data has replaced their assumptions; for example, the real idle load is about 90 W, not the 40 W they assumed. They remain in the git history.

---

## 4. Safety features

The LiDAR safety layer runs on the robot (PC2), so a laptop or Wi-Fi problem cannot make the robot run away. It can only **slow or stop** motion, never add any.

| Situation | What happens |
|---|---|
| Object 0.45 → 0.25 m in front | Forward speed slowed |
| Object ≤ 0.25 m in front | Forward blocked; turning and reversing still work |
| Object inside the safety perimeter (0.20 m front, 0.10 m back, 0.20 m sides) | Only motions **toward** the object are blocked, so you can always drive away |
| No LiDAR data | All motion frozen |
| Connection problem over 1.6 s | Robot stops and an alarm plays; after 5 min it lies down |

Limits: the LiDARs cannot see under the body, there is a small blind strip at each flank, and drops (stairs going down) are probably not detected. No autonomous walking is done yet; every robot test has a spotter holding the remote as the emergency stop.

---

## 5. How to run

**Platform:** Ubuntu 22.04, ROS 2 Humble, CycloneDDS, Python 3.10. Robot messages (`unitree_go`, `unitree_hg`) from Unitree's `unitree_ros2`. Full set-up and every command are in `docs/A2_TEAM_MANUAL.md`.

**Battery logger, laptop test without the robot:**

```bash
python3 battery/fake_bms_publisher.py
```

```bash
python3 battery/battery_logger.py
```

**Battery logger on the robot** (PC2, or the laptop on the robot cable): run `battery_logger.py` with the robot's ROS 2 environment loaded. Stop with Ctrl+C to get the summary. Then:

```bash
python3 battery/power_report.py logs/battery_<date>_<time>.csv --plot power.png
```

**LiDAR safety layer** (PC2, after building the Hesai driver): start the driver with `lidar/a2_hesai_config.yaml`, then `lidar/obstacle_sectors.py` with the mounting transforms. See `docs/A2_TEAM_MANUAL.md` §11.

**Teleop** (laptop; dry run sends no motion):

```bash
python3 teleop/a2_teleop_wifi_gui.py --dry-run
```

The robot is shared with other groups, so all work on PC2 stays in our own folders, with no system-wide installs or package changes.

---

## 6. Plan

| When | Work |
|---|---|
| Week 7–8 (to 6 Nov) | Walking and turning data (T10); lie-down drift test on a taped 457 × 330 mm box; energy model per activity and per sensor (T14); Logbook 3 |
| Week 9–11 (to 27 Nov) | First AI models on real data (T17); return-decision demo; Mid-360 3D LiDAR power cost; Logbook 4 and interim demo |
| Part B (Feb–Apr 2027) | Docking on the real pad with charge detection; full mission → return → dock → resume; test campaigns; report and viva |

**Open items:** charging pad not delivered; Livox Mid-360 3D LiDAR arrived but not yet fitted (team decision); Unitree's own SLAM/navigation service is not available on PC2, so localisation will use another method together with the navigation teammate.

---

## 7. References

- Unitree A2 SDK Development Guide: https://support.unitree.com/home/en/A2_SDK_Development_Guide
- Unitree Contact Charging Board: https://support.unitree.com/home/en/A2_SDK_Development_Guide/a2_charge_pad
- Unitree ROS 2 messages: https://github.com/unitreerobotics/unitree_ros2
- Unitree SDK2 Python: https://github.com/unitreerobotics/unitree_sdk2_python
- Hesai LiDAR ROS 2 driver: https://github.com/HesaiTechnology/HesaiLidar_ROS_2.0
- ROS 2 Humble: https://docs.ros.org/en/humble/
- Three.js (bundled in `lidar/web/`, MIT licence): https://threejs.org
