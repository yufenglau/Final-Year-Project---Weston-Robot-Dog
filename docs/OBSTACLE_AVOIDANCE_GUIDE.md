# Unitree A2 PRO: LiDAR Obstacle Avoidance Guide (for Tue 6 Oct, demo Wed 7 Oct)

Written 5 Oct 2026, **without the robot**. Everything here has been built and tested **on the laptop with fake LiDAR data**. Nothing has run on the robot yet. Tomorrow's job is to repeat it on PC2, one step at a time.

**Every command has its own box: paste one box at a time.** Each step says **where** it runs and what you **should see**.

- 💻 **Laptop**: a normal terminal on Yufeng's laptop.
- 🤖 **PC2**: first log in with `ssh a2-pc2`. The prompt then shows `unitree@unitree-a2-pc2`.

---

## Contents
1. [Where we are: progress so far](#1-where-we-are-progress-so-far)
2. [Can we use the LiDAR safely? (the short answer)](#2-can-we-use-the-lidar-safely)
3. [How the demo works](#3-how-the-demo-works)
4. [Before you start: checklist](#4-before-you-start-checklist)
5. [Step-by-step for tomorrow](#5-step-by-step-for-tomorrow)
6. [If something goes wrong](#6-if-something-goes-wrong)
7. [Exhibition day (7 Oct)](#7-exhibition-day-7-oct)
8. [Limits: what this demo cannot do](#8-limits-what-this-demo-cannot-do)
9. [How to undo everything](#9-how-to-undo-everything)
10. [Files](#10-files)

---

## 1. Where we are: progress so far

| Day | What we did |
|---|---|
| **29 Sep** | First connection over Ethernet. About 100 DDS topics visible, sport state streaming, IMU and odometry read. |
| **30 Sep** | Found that the robot's programs are plain DDS apps (no ROS 2 nodes). Found the **two Hesai JT128 LiDARs** (`192.168.124.20` → UDP 2368, `.21` → 2369) streaming to PC2, with **nothing reading them**. C++ SDK built. |
| **1 Oct** | Battery (two 14-cell packs) and joint data read. Full PC2 inventory: i7, no GPU, ROS 2 Humble, **Unitree `slam_nav` broken** (files missing). Third computer `.164` = **Jetson Orin NX** with a Quectel 4G/GNSS modem. **First keyboard teleop: robot walked.** |
| **2 Oct** | Unitree AP hotspot joined. **Wi-Fi teleop v1/v2 on PC2** (speed caps, 0.5 s watchdog, live camera). The real run worked. |
| **3 Oct** | Project Outline and Gantt (to the viva, about 27 Apr 2027). Cloud-integration findings for the IT team. Risk assessment signed. |
| **4 Oct** | Research: navigation vs obstacle avoidance. Full SLAM/Nav2 ruled out for 7 Oct. LiDAR obstacle-avoidance plan made. |
| **5 Oct** | **4G SIM** in the Jetson. **`A2-4G` hotspot** (robot's own internet). **Talk** (T), **Listen** (L), **Tailscale** remote access, **link-loss guard** with alarm, Wi-Fi range ≈ 15 m. *(See `DAY_2026-10-05_MANUAL.md`.)* **Evening:** Hesai driver downloaded and test-built on the laptop. Obstacle-avoidance programs written and tested offline (this guide). |

**What already works and is proven on the robot:** walking from code (`SportClient.Move`), safe stop/lie-down, Wi-Fi teleop with link guard.
**What is new tomorrow:** reading the LiDARs, then letting the robot walk by itself.

---

## 2. Can we use the LiDAR safely?

**Not through the Unitree SDK.** We checked this on 30 Sep. The A2 SDK (Python and C++) only has `sport` and `audio` clients, with no LiDAR or obstacle-avoid client. The Go2 obstacle-avoid example fails (error 3102, no server). Unitree's own LiDAR topics (`/utlidar/*`, `/uslam/*`) have **0 publishers**, because the program that should fill them (`slam_nav`) is missing on PC2.

**Yes, by listening to the LiDARs directly.** This is the safe way. The LiDARs already send their data to PC2 (about 10 MB/s), and nothing on PC2 reads it. We run **Hesai's official ROS 2 driver** (`HesaiLidar_ROS_2.0`, which lists JT128 as supported) in **our own folder** on PC2. It turns those packets into ROS 2 point clouds.

Why this is safe:

| Concern | Answer |
|---|---|
| Could it change LiDAR settings? | **No**, as long as `standby_mode: -1` and `speed: -1` stay in the config. We checked the driver source (`libhesai/Lidar/lidar.h`): it only writes settings when these are not −1. Otherwise it **listens** to UDP and **reads** the calibration file. |
| Could it disturb the robot? | No. It only opens UDP ports 2368/2369, which nothing else uses. It publishes on **new** topic names (`/lidar_a/points`, `/lidar_b/points`). We **never** publish to `/utlidar/*` or `/unitree/slam_lidar/*`, because robot-side programs listen there. |
| PC2 is shared: does it install anything? | No. It's built from source in `~/a2_lidar`, and no apt/pip installs are needed if the build packages are already there (step 2 checks). Undo = delete the folder. |
| How does the robot move? | With **exactly the same SDK calls as our proven teleop**: `StandUp`, `BalanceStand`, `Move`, `StopMove`, `StandDown`. No low-level commands, nothing on PC1. |
| Hardware e-stop? | The **Unitree remote**, always in someone's hand. |

---

## 3. How the demo works

```
 Hesai JT128 x2 ──UDP──► hesai_ros_driver ──/lidar_a/points, /lidar_b/points──► obstacle_sectors.py
 (192.168.124.20/.21)    (PC2, our folder)          (ROS 2 point clouds)          (PC2, PERCEPTION only)
                                                                                         │ UDP 127.0.0.1:9600
                                                                                         │ {"front":1.4,"left":0.8,"right":2.1}
                                                                                         ▼
                                                     Unitree SportClient ◄──Move── avoid_drive.py
                                                                                   (PC2, DECISION + MOTION,
                                                                                    dry run unless --drive)
```

Three programs, each with **one job**. That makes each easy to test on its own:

1. **Driver**: packets → 3D points.
2. **`obstacle_sectors.py`**: points → three numbers.
   - Rotates each LiDAR's points into the robot's frame (x forward, y left, z up). One LiDAR is mounted upside-down.
   - Finds the floor automatically, so it works lying down and standing.
   - Throws away the floor, the robot's own body, and anything above 0.6 m.
   - Measures **front** (nearest thing in the walking corridor, from the nose), **left** and **right**.
   - It **never moves the robot**.
3. **`avoid_drive.py`**: three numbers → walking, using a small state machine:

```
FORWARD ── front < 0.6 m ──► PAUSE (0.6 s) ──► TURN toward the more open side
   ▲                                              │
   └──────────── front > 1.0 m ◄──────────────────┘     (0.6 / 1.0 = hysteresis, no jitter)

Anything < 0.3 m in front (someone steps in) ──► HOLD: no motion until clear for 1 s
LiDAR data late (> 0.3 s) or a LiDAR missing  ──► StopMove at once
```

Speed: up to **0.25 m/s** forward, slowing down near obstacles. Turning: **0.4 rad/s**. Hard limits in code: 0.35 m/s and 0.6 rad/s. Every run is logged to `avoid_log_<time>.csv`, which is good material for the logbook.

> **Offline test result (5 Oct, laptop, fake clouds):** a box moving toward the robot gave FORWARD → PAUSE → TURN → HOLD → PAUSE → TURN → FORWARD. Silencing one fake LiDAR gave an instant stop. The floor was found correctly at both lying (−0.10 m) and standing (−0.45 m) heights.

---

## 4. Before you start: checklist

- [ ] Both batteries charged (they drain about 10 %/h even lying down).
- [ ] Open area of **at least 5 × 5 m**, flat, **no stairs, stage edges or drops** (the robot can't see drops, see §8).
- [ ] Cardboard boxes as obstacles. **No people as obstacles until the HOLD test passes.**
- [ ] One person holds the **Unitree remote** the whole time and knows how to make the robot lie down/stop.
- [ ] Laptop can reach PC2 (cable, `A2-4G` Wi-Fi, or Tailscale). Teleop window **closed**.
- [ ] Teammate from the navigation project invited: this is a team demo.

---

## 5. Step-by-step for tomorrow

### Step 1: Log in to PC2 and check it's ready

💻 Laptop: log in to PC2.

```bash
ssh a2-pc2
```

✅ Prompt shows `unitree@unitree-a2-pc2`.

<br>

🤖 PC2: check ROS 2 is loaded.

```bash
echo $ROS_DISTRO
```

✅ `humble`

<br>

🤖 PC2: check nothing else is already using the LiDAR ports (another group's driver, for example).

```bash
ss -ulpn | grep -E ":236[89]"
```

✅ **No output.** If something is listed, **stop** and find out whose it is. Two programs can't share these ports.

---

### Step 2: Check the build packages are already on PC2 (no installing)

🤖 PC2:

```bash
dpkg -l libyaml-cpp-dev libboost-thread-dev libssl-dev | grep ^ii
```

✅ Three lines starting with `ii`. Note any that are missing:
- **All 3 present** → go to step 3.
- **Only `libssl-dev` missing** → step 3, but use the **"no SSL" build** command shown there.
- **`libyaml-cpp-dev` or `libboost-thread-dev` missing** → don't install anything. Use **Plan B** in §6 (build on the laptop and copy).

---

### Step 3: Copy the driver and our programs to PC2, then build

🤖 PC2: make our own folder.

```bash
mkdir -p ~/a2_lidar/ws/src
```

<br>

💻 Laptop (**new terminal**, not the PC2 one): copy the driver source.

```bash
scp -r ~/unitree_a2/lidar/src/HesaiLidar_ROS_2.0 a2-pc2:~/a2_lidar/ws/src/
```

✅ Many file names scroll past, with no errors.

<br>

💻 Laptop: copy the config and our two programs.

```bash
scp ~/unitree_a2/lidar/a2_hesai_config.yaml ~/unitree_a2/lidar/obstacle_sectors.py ~/unitree_a2/lidar/avoid_drive.py a2-pc2:~/a2_lidar/
```

<br>

🤖 PC2: go to the workspace.

```bash
cd ~/a2_lidar/ws
```

<br>

🤖 PC2: build (about 1 minute).

```bash
colcon build --cmake-args -DCMAKE_BUILD_TYPE=Release
```

✅ `Summary: 1 package finished`. (A "stderr output" note is normal: it's the driver printing "ROS2 Found".)

*Only if `libssl-dev` was missing in step 2*, use this build command instead:

```bash
colcon build --cmake-args -DCMAKE_BUILD_TYPE=Release -DWITH_PTCS_USE=OFF
```

---

### Step 4: Start the LiDAR driver (Terminal 1)

🤖 PC2: load our workspace.

```bash
source ~/a2_lidar/ws/install/setup.bash
```

<br>

🤖 PC2: start the driver. Leave this terminal running.

```bash
ros2 run hesai_ros_driver hesai_ros_driver_node --ros-args -p config_path:=$HOME/a2_lidar/a2_hesai_config.yaml
```

✅ Lines like `PtcClient::PtcClient() 192.168.124.20 9347` and `... .21 9347`, `Lidar::Receive Udp Thread start to run`, and **no repeating `[ERROR]`**.
❌ Repeating errors about *correction* / *angle* → see §6 "Calibration file".

---

### Step 5: Check the point clouds arrive (Terminal 2)

💻 Laptop: open a **second** PC2 terminal.

```bash
ssh a2-pc2
```

<br>

🤖 PC2: is LiDAR A publishing?

```bash
ros2 topic hz /lidar_a/points
```

✅ `average rate: 10.0` (about 10 Hz). Press **Ctrl+C** after a few lines.

<br>

🤖 PC2: is LiDAR B publishing?

```bash
ros2 topic hz /lidar_b/points
```

✅ About 10 Hz. **Ctrl+C**.

> 🎉 If both show about 10 Hz, **we are reading the A2's LiDARs**, for the first time on this robot. Note the time for the logbook.

---

### Step 6: Which LiDAR is front? (cover test)

🤖 PC2 (Terminal 2): start the stats view (no motion, read-only).

```bash
python3 ~/a2_lidar/obstacle_sectors.py --stats
```

✅ Every 0.5 s, one line per LiDAR: `pts`, `nearest`, `pts<0.3 m`, `z(sensor)`.

Now **cover the FRONT LiDAR** with a hand or a cardboard box, about 5 cm away (JT128 lasers are eye-safe class 1).

✅ For one topic, `nearest` drops to about 0.05 m and `pts<0.3 m` jumps to thousands. **That topic is the FRONT LiDAR.** Write it down:

- `/lidar_a/points` (192.168.124.20) = **front / rear** ← circle one
- `/lidar_b/points` (192.168.124.21) = **front / rear**

Leave the stats view running for step 7, or stop it with **Ctrl+C**.

---

### Step 7: Get the mounting positions from Unitree's own config (read-only)

The program needs to know where each LiDAR sits on the body and how it's rotated. Unitree's (uninstalled) navigation package has these numbers. We only **read** it.

💻 Laptop: open a **third** PC2 terminal (or press Ctrl+C in Terminal 2).

```bash
ssh a2-pc2
```

<br>

🤖 PC2: list Unitree's JT128 config files.

```bash
find /unitree/ota/backup/module/slam_nav -path "*hs_lidar_jt128*"
```

✅ A folder with `config.yaml`, and maybe calibration files (`.csv`/`.dat`). Note them.

<br>

🤖 PC2: show the config.

```bash
cat $(find /unitree/ota/backup/module/slam_nav -path "*hs_lidar_jt128*" -name config.yaml | head -1)
```

Look for each LiDAR's **x, y, z, roll, pitch, yaw** (we already know LiDAR 2 has `pitch: 3.14159`, i.e. flipped, in **radians**). **Take a photo / copy it into the chat with Claude.** Then turn the numbers into two strings, in the order `"x y z roll pitch yaw"`:

- `--tf-a "..."` = numbers for `192.168.124.20`
- `--tf-b "..."` = numbers for `192.168.124.21`

> 💡 If one LiDAR is all zeros, Unitree measured everything **from that LiDAR**, not from the body centre. That's fine for now: the box test in step 9 corrects the forward offset.

---

### Step 8: Measure the robot (tape measure, 2 minutes)

With the robot **standing** (use the remote), measure:
- nose-to-tail length: half of it, plus 5 cm → `--body-half-length` (default 0.55)
- widest width incl. legs: half of it, plus 5 cm → `--body-half-width` (default 0.35)

Then lay it down again.

---

### Step 9: Sector test with a box (still no motion)

🤖 PC2 (Terminal 2): run the sectors with **your** numbers from steps 7–8. The values below are an **example only**: replace each quoted string and number with yours.

```bash
python3 ~/a2_lidar/obstacle_sectors.py --tf-a "0 0 0 0 0 0" --tf-b "0 0 0 0 3.14159 0" --body-half-length 0.55 --body-half-width 0.35
```

✅ Twice a second: `[OK ] front ... | left ... | right ... | floor ... | fresh [1, 1]`.

Tests (robot lying or standing, motors not walking):

| Put a box… | Expect |
|---|---|
| nothing nearby | `front`, `left`, `right` large (or 9.90 = nothing seen) |
| **1.00 m in front of the nose** (tape) | `front` ≈ **1.00** (±0.1) |
| 0.5 m to the **left** side | `left` ≈ 0.5, `front` unchanged |
| 0.5 m to the **right** side | `right` ≈ 0.5 |
| behind the robot | `front` unchanged |

Fixes:
- `front` is off by a constant (e.g. reads 1.40 for a 1.00 m box) → change the **x** value in **both** tf strings by that difference (here −0.40) and repeat.
- The box in front shows up as `left`/`right` → that LiDAR's **yaw** is off by 90°: add `1.5708` or `-1.5708` to its yaw.
- `left` and `right` are swapped → yaw off by 180°: add `3.14159` to the yaw.
- `floor` is very different between lying and standing: that's normal (about −0.1 vs −0.45). With `--stats`, the two LiDARs' floor values should agree within a few cm. If not, a tf is wrong.
- `front` is small with **nothing** there → the robot sees itself: increase `--body-half-length` / `--body-half-width` by 0.05.

**Write the final command down**: it's the one you'll use from now on. Leave it running.

---

### Step 10: Decision test, dry run (robot does NOT move)

💻 Laptop: run the brain in dry-run mode. It prints what it *would* send.

```bash
ssh -t a2-pc2 "cd ~/a2_lidar && python3 avoid_drive.py"
```

✅ `LiDAR OK: front ...`, then `DRY RUN would send: StandUp`, `BalanceStand`, then `PAUSE -> FORWARD` and `Move(vx=...)` lines.

Now walk a box towards the front of the robot:

✅ The speed shrinks, then `FORWARD -> PAUSE`, `turning LEFT/RIGHT (...)` toward the open side, `PAUSE -> TURN`. Box very close (< 0.3 m) → `HOLD`. Box removed → back to `FORWARD`.

Stop with **Ctrl+C**. ✅ `DRY RUN would send: StopMove`, `StandDown`, `Done. Log: avoid_log_....csv`.

Also test the **stale-data stop**: while it runs, press **Ctrl+C in Terminal 1** (the driver). ✅ Within 0.3 s: `LiDAR data stale or a LiDAR missing -> stop`. Restart the driver (step 4) afterwards.

---

### Step 11: First real walk: slow, short, boxes only

Checklist §4 done? Remote in someone's hand? Area clear of people?

💻 Laptop: real run, **0.15 m/s, 30 seconds**.

```bash
ssh -t a2-pc2 "cd ~/a2_lidar && PYTHONPATH=~/a2_wireless python3 avoid_drive.py --drive --max-vx 0.15 --duration 30"
```

✅ `LiDAR OK ...`, then `Remote in someone's hand? Area clear? Press Enter to STAND UP and start`. Press **Enter**. The robot stands, waits about 4 s, then walks slowly, stops before the box, turns away, and walks on. After 30 s it stops and lies down.

**Ctrl+C at any time** → StopMove → lies down. **Remote** = hardware e-stop.

> Why `ssh -t "…"` from the laptop and not inside a PC2 terminal? This is exactly how the teleop server is started (`PYTHONPATH=~/a2_wireless`, no ROS loaded), which we know works with the Unitree SDK on PC2.

---

### Step 12: Normal speed, longer

Only after step 11 looked good twice.

💻 Laptop:

```bash
ssh -t a2-pc2 "cd ~/a2_lidar && PYTHONPATH=~/a2_wireless python3 avoid_drive.py --drive --max-vx 0.25 --duration 120"
```

Then try the **HOLD** test: a person, holding the remote, steps in front of the robot. ✅ It stops (`HOLD`) and waits until the person leaves.

---

### Step 13: Finish

- Terminal with `avoid_drive`: already ended (or **Ctrl+C**).
- Terminal 2 (`obstacle_sectors.py`): **Ctrl+C**.
- Terminal 1 (driver): **Ctrl+C**.

Nothing keeps running: we installed **no services**.

<br>

💻 Laptop: copy today's logs back for the logbook.

```bash
scp "a2-pc2:~/a2_lidar/avoid_log_*.csv" ~/unitree_a2/logs/
```

---

## 6. If something goes wrong

| Problem | What to do |
|---|---|
| **Plan B: build packages missing on PC2** | Build on the laptop, copy the result (same Ubuntu 22.04 / Humble / x86_64). See the boxes below. |
| **Calibration file** errors in the driver | The driver couldn't read the angle calibration over PTC. If step 7's folder has calibration files, copy their paths into `correction_file_path` (and `firetimes_path`) in `~/a2_lidar/a2_hesai_config.yaml`. **Don't change anything else**, especially not `standby_mode`/`speed`. |
| Cloud looks scrambled / jumps | Possibly the small 99-byte packets both LiDARs send to port 2369. Tell Claude. The fix is the `device_udp_src_port` filter in the config. |
| `hz` shows nothing | Is the driver still running? `ss -ulpn \| grep 236` should now show `hesai_ros_driver`. Is `net1` up (`ip -br addr`)? |
| PC2 slow (others use it too) | Run the sectors with `--every 2` (uses every 2nd cloud, 5 Hz). Check with `top`. |
| `avoid_drive` says teleop is running | Close the teleop window first: only one program may drive. |
| `No good LiDAR data within 15 s` | `obstacle_sectors.py` isn't running, or one LiDAR shows `fresh [1, 0]`. |
| Phantom obstacles far away while walking | The body pitches when walking, so the far floor "rises". Add `--floor-clearance 0.15` or `--max-range 2.5`. |
| Robot stops for no visible reason | Look at the `obstacle_sectors.py` line: which sector is small? Often it's its own leg: increase `--body-half-width`. |
| `Move failed, code ...` | Same as teleop: is the robot in a normal standing mode? Is something else driving (app, remote)? |

**Plan B boxes** (only if step 2 showed missing packages):

💻 Laptop: go to the laptop workspace.

```bash
cd ~/unitree_a2/lidar
```

<br>

💻 Laptop: remove the earlier test build.

```bash
rm -rf build install log
```

<br>

💻 Laptop: build a copyable version.

```bash
colcon build --cmake-args -DCMAKE_BUILD_TYPE=Release
```

<br>

💻 Laptop: copy the built driver to PC2.

```bash
scp -r ~/unitree_a2/lidar/install a2-pc2:~/a2_lidar/ws/
```

<br>

🤖 PC2: check every library it needs is there.

```bash
ldd ~/a2_lidar/ws/install/hesai_ros_driver/lib/hesai_ros_driver/hesai_ros_driver_node | grep "not found"
```

✅ **No output** → continue at step 4. If something is "not found", stop: that would need an install on the shared PC2, so ask first.

---

## 7. Exhibition day (7 Oct)

- Use the **final commands** from steps 9 and 12. Keep `--max-vx 0.25` or lower.
- **Rope off or tape** an area. Visitors stay outside, and one person with the remote stays inside.
- Plan B if the LiDAR misbehaves on the day: **Wi-Fi teleop v2** (proven).
- What to tell visitors: *"Two 3D LiDARs give the robot tens of thousands of 3D distance points, 10 times a second. We remove the floor and the robot's own body, measure the free space ahead, left and right, and a state machine decides: walk, pause, turn or hold."*

---

## 8. Limits: what this demo cannot do

Be honest about these with visitors and the supervisor. They're also good "future work" lines for the report.

- **It is obstacle avoidance, not navigation.** No map, no goal, no "go to the kitchen". That needs SLAM + a planner (the navigation teammate's project, or Unitree's `slam_nav` once restored by Weston Robot/Unitree).
- **No drop/stair detection.** It ignores everything near floor height, so it **will walk off a stage edge or down stairs**. Keep it on flat ground away from edges.
- **Low objects (< 10 cm) are ignored** on purpose (the floor filter). Overhangs above 0.6 m are ignored (a table top could hit a raised head/payload).
- **Glass, mirrors, very dark materials** may return few or no points.
- **Never walks backwards or sideways**, and turns in place only.
- Wander behaviour can **loop** in a dead end (it gives up turning after 8 s, pauses, and re-picks a side).

---

## 9. How to undo everything

🤖 PC2: delete our folder (driver, programs, logs: copy logs first, see step 13).

```bash
rm -rf ~/a2_lidar
```

That's all: no packages, services or settings were changed on PC2, the LiDARs or the robot.

---

## 10. Files

| File (laptop) | What |
|---|---|
| `~/unitree_a2/OBSTACLE_AVOIDANCE_GUIDE.md` | This guide |
| `~/unitree_a2/lidar/src/HesaiLidar_ROS_2.0/` | Hesai's official driver (v2.0.12, cloned 5 Oct 2026), test-built on the laptop |
| `~/unitree_a2/lidar/a2_hesai_config.yaml` | Driver config: both LiDARs, safe settings (`standby_mode`/`speed` = −1) |
| `~/unitree_a2/lidar/obstacle_sectors.py` | Perception: clouds → front/left/right (+ `--stats` cover test) |
| `~/unitree_a2/lidar/avoid_drive.py` | Decision + motion state machine (dry run unless `--drive`) |
| `~/unitree_a2/lidar/fake_clouds.py` | Fake LiDAR data for testing on the laptop without the robot |

**Practise tonight without the robot** (laptop only, three terminals, each starts with `source /opt/ros/humble/setup.bash`):

💻 Laptop, terminal A: fake LiDARs.

```bash
python3 ~/unitree_a2/lidar/fake_clouds.py
```

<br>

💻 Laptop, terminal B: perception.

```bash
python3 ~/unitree_a2/lidar/obstacle_sectors.py
```

<br>

💻 Laptop, terminal C: brain in dry run (watch FORWARD → PAUSE → TURN → HOLD as the fake box comes closer).

```bash
python3 ~/unitree_a2/lidar/avoid_drive.py --duration 60
```
