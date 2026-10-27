# Unitree A2 PRO — Team Manual (what works so far)

> **Public copy.** Passwords and private network addresses are removed. The day manuals, `ROBOT_FACTS.md`, `CONNECT_GUIDE.md` and the backup it mentions are the team's private notes and are not in this repository. Paths like `~/unitree_a2/battery/` and `~/unitree_a2/teleop/` match this repository's `battery/` and `teleop/` folders; the `lidar/` files live on PC2 in `~/a2_lidar/`.

*Last updated: 25 Oct 2026.* This manual only lists things we have **actually done and seen working**. The basics (setup, cable, reading data) date from 29–30 Sep 2026. Each robot session has its own day manual (below) with the full steps; this manual gives the short version and points to them.
Each step says **what it does**, gives the **command**, and shows **what you should see**. **Every command has its own box**: paste one box at a time.

**Where to look:**
- This folder, **`~/ROBOT_MANUAL/`**, holds this manual and every day manual:

| Day manual | What's in it |
|---|---|
| `DAY_2026-10-01_MANUAL.md` | Battery, joints, the three computers (.164 = Jetson), GNSS location, the **keyboard teleop** (`teleop/a2_teleop.py`) |
| `DAY_2026-10-02_MANUAL.md` | **Wireless (Unitree AP mode)**, phone tethering, the **Wi-Fi teleop window** (camera, speed levels, combined keys), fun-move scripts |
| `DAY_2026-10-05_MANUAL.md` | **4G SIM**, the **`A2-4G` hotspot**, **Tailscale**, the **link-loss guard**, **talk (T)** and **listen (L)**, Wi-Fi range |
| `DAY_2026-10-06_MANUAL.md` | **Hesai LiDAR driver on PC2**, which LiDAR is front, mounting, live browser map, **LiDAR brake + safety oval** in the teleop, Tailscale fix |
| `DAY_2026-10-16_MANUAL.md` | First check after the Weston repair: **PC2 replaced/re-imaged**, SSH key re-added |
| `DAY_2026-10-19_MANUAL.md` | **Battery logger** first real run, current-sign check, Mid-360 wiring notes |
| `DAY_2026-10-20_MANUAL.md` | **T7 PASS** (1 h, 0 gaps), **PC2 restored from backup**, LiDAR re-checks, lie-down drift test method |
| `DAY_2026-10-22_MANUAL.md` | Standby power test, **battery logger on PC2**, power report and plots |

- In the project folder **`~/unitree_a2/`**:
  - **`ROBOT_FACTS.md`**: every verified fact about *this* robot, with a **"Current state" summary at the top**. Check it before investigating anything.
  - `CONNECT_GUIDE.md`: cable connection guide and troubleshooting.
  - `OBSTACLE_AVOIDANCE_GUIDE.md`: how the LiDAR obstacle detection works.
  - `battery/README.md`: the battery logger.
  - `LIDAR_FINDINGS.txt`: first LiDAR/network findings (note: it still calls `.164` "unknown"; it is the Jetson).
  - `CLOUD_INTEGRATION_FINDINGS.md`: what Unitree's docs say about cloud/camera interfaces.
- **`~/A2_PRO_BACKUP/`**: full backup of the laptop project, PC2 and the Jetson, plus `RESTORE_AFTER_FACTORY_RESET.md` (§14).

> 💡 `enp2s0` is the Ethernet port name on **Yufeng's laptop**. On your laptop it may be different, for example `enp0s31f6`.
> Find yours with `ip -br addr`, and use your name wherever this manual says `enp2s0`.
>
> **💻 Laptop** = a normal terminal on the laptop. **🤖 PC2** = after `ssh a2-pc2` (prompt `unitree@unitree-a2-pc2`).

---

## Contents
1. [Our setup at a glance](#1-our-setup-at-a-glance)
2. [One-time laptop setup (already done on Yufeng's laptop)](#2-one-time-laptop-setup)
3. [Setting up a second laptop (done on a teammate's laptop)](#3-setting-up-a-second-laptop)
4. [Daily shortcuts](#4-daily-shortcuts)
5. [Connect to the robot (cable)](#5-connect-to-the-robot)
6. [Other ways to connect (wireless and remote)](#6-other-ways-to-connect)
7. [Read robot data with ROS 2](#7-read-robot-data-with-ros-2)
8. [Read robot data with the Python SDK](#8-read-robot-data-with-the-python-sdk)
9. [C++ SDK: robot state and audio recording](#9-c-sdk-robot-state-and-audio-recording)
10. [Log into PC2 and the Jetson](#10-log-into-pc2-and-the-jetson)
11. [LiDARs on PC2: start, check, stop](#11-lidars-on-pc2-start-check-stop)
12. [Teleop: drive, talk, listen, LiDAR brake](#12-teleop-drive-talk-listen-lidar-brake)
13. [Battery logger](#13-battery-logger)
14. [Backup and restore](#14-backup-and-restore)
15. [ROS MCP + rosbridge (lets Claude read robot data)](#15-ros-mcp--rosbridge)
16. [Setup checker script](#16-setup-checker-script)
17. [GPS programs (tested on the laptop with a fake GPS)](#17-gps-programs-tested-with-a-fake-gps)
18. [Problems we hit and how we fixed them](#18-problems-we-hit-and-how-we-fixed-them)
19. [Safe shutdown](#19-safe-shutdown)
20. [Safety rules](#20-safety-rules)

---

## 1. Our setup at a glance

| Thing | Value |
|---|---|
| Robot | Unitree **A2 PRO** (from Weston Robot; repaired by Weston 8–14 Oct 2026) |
| Laptop OS | **Ubuntu 22.04.5 LTS** |
| Robot software | **ROS 2 Humble** + **CycloneDDS** (the robot sends all its data as DDS "topics") |
| Project folder | `~/unitree_a2/` · manuals in `~/ROBOT_MANUAL/` · backup in `~/A2_PRO_BACKUP/` |
| Laptop address on the robot network (cable) | `192.168.123.222` (Yufeng) · `192.168.123.223` (second laptop) |
| PC1 (motion control, **don't touch**) | `192.168.123.161` |
| PC2 (the PRO's extra computer, shared with other groups) | `192.168.123.162`, Intel i7, Ubuntu 22.04 + ROS 2 Humble, no internet. **Replaced or re-imaged by Weston** (found 16 Oct); our folders restored 20–22 Oct |
| **Jetson Orin NX (docking hub)** | `192.168.123.164`: **4G modem + SIM**, GNSS, USB Wi-Fi (`A2-4G` hotspot), Tailscale (logged out after the repair, re-check) |
| LiDARs (2× Hesai JT128, only reachable from PC2) | **Front `192.168.124.20`** (`/lidar_a/points`) · **rear `192.168.124.21`** (`/lidar_b/points`), 10 Hz |
| Main robot-state topic | `/lf/sportmodestate` |
| Battery topics | `/lf/bmsstate`, `/lf/secondary_bmsstate` (20 Hz each). **Negative current = discharging** |
| Battery | 2 packs, 907 Wh total, about 9 Wh per 1 %. Lying about 87–95 W, standing about 110–123 W, slow walking about 175–180 W |
| New hardware | **Livox Mid-360** 3D LiDAR arrived 19 Oct, **not fitted yet** (needs 9–27 V, never the ~50 V battery rail) |
| Charging pad | Unitree Contact Charging Board: **not delivered yet** |

---

## 2. One-time laptop setup

*Already done on Yufeng's laptop on 29 Sep 2026 (all 14 steps OK). The full step-by-step plan is `~/Desktop/plan.md`, and the results are in `~/unitree_a2/SETUP_LOG.md`.*
Summary of what was installed and the main commands:

| # | What it does | Command |
|---|---|---|
| 1 | Installs basic build tools | `sudo apt-get install -y git cmake build-essential curl python3-pip python3-venv python3-dev libyaml-cpp-dev net-tools` |
| 2 | Installs ROS 2 Humble and the CycloneDDS middleware | `sudo apt-get install -y ros-humble-desktop ros-dev-tools python3-colcon-common-extensions python3-rosdep ros-humble-rmw-cyclonedds-cpp ros-humble-rosidl-generator-dds-idl` |
| 3 | Downloads Unitree's code | `git clone https://github.com/unitreerobotics/unitree_ros2.git` and `git clone https://github.com/unitreerobotics/unitree_sdk2_python.git` (inside `~/unitree_a2`) |
| 4 | Builds the Unitree message types so ROS 2 can understand the robot's data | `colcon build --packages-select unitree_api unitree_go unitree_hg` (in `~/unitree_a2/unitree_ros2/cyclonedds_ws`, after `source /opt/ros/humble/setup.bash`) |
| 5 | Builds CycloneDDS 0.10.x (needed by the Python SDK) | `git clone -b releases/0.10.x https://github.com/eclipse-cyclonedds/cyclonedds.git`, then build with `cmake` into `~/unitree_a2/cyclonedds/install` |
| 6 | Creates the Python environment and installs the Unitree Python SDK | `python3 -m venv --system-site-packages venv`, activate it, then `pip install -e unitree_sdk2_python` |
| 7 | Gives the laptop a fixed address on the robot network. Wi-Fi stays your internet | `sudo nmcli con add type ethernet con-name unitree-a2 ifname enp2s0 ipv4.method manual ipv4.addresses 192.168.123.222/24 ipv4.never-default yes ipv6.method disabled` |
| 8 | Adds the shortcuts `a2ros`, `a2py`, `a2local` and `a2cd` | see [section 4](#4-daily-shortcuts) |

✅ Check it worked:
```bash
ls /opt/ros
```
✅ Shows `humble`.

---

## 3. Setting up a second laptop

*Done and working on a teammate's laptop on 30 Sep 2026: it read live robot data.*
This is the **minimum** needed to read robot data with ROS 2, on a laptop that already has ROS 2 Humble.

**Step 1: find your Ethernet port name.**
```bash
ip -br addr
```
Look for the name starting with `enp` or `eth`, for example `enp0s31f6`. Use lowercase, exactly as shown.

&nbsp;

**Step 2: give the laptop a fixed IPv4 address on the robot network.** Use your own port name, and a free address (not .222, which Yufeng's laptop uses).
```bash
sudo nmcli con add type ethernet con-name unitree-a2 ifname enp0s31f6 ipv4.method manual ipv4.addresses 192.168.123.223/24 ipv6.method disabled
```

&nbsp;

```bash
sudo nmcli con up unitree-a2
```

&nbsp;

```bash
ip -br addr show enp0s31f6
```
✅ Shows `UP 192.168.123.223/24`.

> Why: the robot doesn't hand out addresses. Without this step, the laptop only shows an IPv6 `fe80::…` address and can't reach the robot.

**Step 3: install the ROS extras and build the Unitree messages.**
```bash
sudo apt-get install -y ros-humble-rmw-cyclonedds-cpp ros-humble-rosidl-generator-dds-idl git
```

&nbsp;

```bash
mkdir -p ~/unitree_a2
```

&nbsp;

```bash
cd ~/unitree_a2
```

&nbsp;

```bash
git clone https://github.com/unitreerobotics/unitree_ros2.git
```

&nbsp;

```bash
cd unitree_ros2/cyclonedds_ws
```

&nbsp;

```bash
source /opt/ros/humble/setup.bash
```

&nbsp;

```bash
colcon build --packages-select unitree_api unitree_go unitree_hg
```
✅ The build ends with `Summary: 3 packages finished`.

&nbsp;

**Step 4: create the `a2ros` shortcut.** Copy `~/unitree_a2/setup_robot.sh` from Yufeng's laptop, then run:
```bash
echo "alias a2ros='A2_IFACE=enp0s31f6 source ~/unitree_a2/setup_robot.sh'" >> ~/.bashrc
```

&nbsp;

```bash
source ~/.bashrc
```

**Step 5: test.** Continue with [section 5](#5-connect-to-the-robot) and [section 7](#7-read-robot-data-with-ros-2).

---

## 4. Daily shortcuts

Run one of these **first**, in every new terminal:

| Shortcut | What it does | You should see |
|---|---|---|
| `a2ros` | Sets up the terminal for **ROS 2 over the robot cable** (also needed for the battery logger) | `[a2] ROS 2 humble \| RMW=rmw_cyclonedds_cpp \| iface=enp2s0` |
| `a2py` | Switches to the **Python SDK** environment (also needed for the teleop window) | Prompt starts with `(venv)` |
| `a2local` | ROS 2 **without the robot** (offline practice) | `iface=lo (offline)` |
| `a2cd` | Goes to the workspace folder | `~/unitree_a2` |

⚠️ **Use `a2ros` and `a2py` in separate terminals.** Never use both in the same one.

---

## 5. Connect to the robot

1. **Robot lying down** on the floor, remote controller on and in someone's hand.
2. **Power on** the robot and wait about **1–2 minutes** for it to boot.
3. **Plug the Ethernet cable** into the robot's **Ethernet 1** port and into your laptop.
4. **Turn on the laptop's robot connection:**
```bash
nmcli con up unitree-a2
```
✅ `Connection successfully activated`.

&nbsp;

5. **Check that the laptop joined the robot network:**
```bash
ip -br addr show enp2s0
```
✅ `enp2s0  UP  192.168.123.222/24`

&nbsp;

6. **Ping the robot's computers:**
```bash
ping -c 3 192.168.123.161
```

&nbsp;

```bash
ping -c 3 192.168.123.162
```

&nbsp;

```bash
ping -c 3 192.168.123.164
```
✅ Replies like `64 bytes from 192.168.123.161 ...`
If ping fails, nothing else will work. Fix this first.

---

## 6. Other ways to connect

| Way | What you get | What doesn't work | How |
|---|---|---|---|
| **Cable** (§5) | Everything, incl. ROS 2/SDK on the laptop | — | §5 |
| **Unitree AP** (`A2_11707_*`, set in the Unitree app) | SSH to PC2/Jetson, teleop window, camera | ROS 2/SDK on the laptop (PC1 NAT); no internet (use phone USB tethering) | `DAY_2026-10-02_MANUAL.md` §6–8, `DAY_2026-10-05_MANUAL.md` §2 |
| **Jetson hotspot `A2-4G`** | Same as the AP **plus internet through the robot's 4G** (no phone needed) | ROS 2/SDK on the laptop; range about 15 m; must be started by hand after each robot boot | `DAY_2026-10-05_MANUAL.md` §4–5 |
| **Tailscale** (any internet) | SSH to the Jetson and PC2, teleop window, from anywhere | ROS 2/SDK on the laptop; laggy; robot 4G upload only about 0.65 Mbit/s, so use `--no-video` over 4G | `DAY_2026-10-05_MANUAL.md` §9, `DAY_2026-10-06_MANUAL.md` §11 |

Over Wi-Fi/Tailscale, read ROS 2 data **on PC2** (`ssh a2-pc2`, then `ros2 topic echo …`).

**Switch between them with one command** (7 Oct, `tools/a2_net.sh`). 💻 Laptop:

```bash
bash ~/unitree_a2/tools/a2_net.sh status
```
✅ Shows the current Wi-Fi, Tailscale on/off and whether PC2 is reachable.

| Command | What it does |
|---|---|
| `bash ~/unitree_a2/tools/a2_net.sh ap` | Join the robot's own Unitree Wi-Fi, Tailscale off |
| `bash ~/unitree_a2/tools/a2_net.sh a2-4g` | Join the Jetson hotspot `A2-4G`, Tailscale off (the hotspot must be on first: Jetson `sudo nmcli con up A2-4G`) |
| `bash ~/unitree_a2/tools/a2_net.sh tailscale` | Join the phone hotspot, Tailscale on with the robot route |

⚠️ **Cable and Tailscale don't mix:** on the cable, the laptop's Tailscale `accept-routes` must be **off**; for Tailscale it must be **on** (`DAY_2026-10-06_MANUAL.md` §11).

---

## 7. Read robot data with ROS 2

**Read the robot's state once.** This is read-only; the robot doesn't move. (Cable only.)
```bash
a2ros
```

&nbsp;

```bash
ros2 daemon stop
```

&nbsp;

```bash
ros2 topic echo /lf/sportmodestate unitree_go/msg/SportModeState --once --no-daemon
```
✅ You get a block of text starting with `stamp:` and `imu_state:`. What we saw with the robot lying down:

| Field | Our value | Meaning |
|---|---|---|
| `accelerometer` | about `[0, 0, 9.65]` | Gravity on the Z axis, so the body is level |
| `quaternion` | about `[1, 0, 0, 0]` | No tilt |
| `mode` | `1` | Lying down (standing and walking both show `3`) |
| `position` | about `[0.01, 0.24, 0.06]` m | Position measured from where the robot was switched on (not GPS) |
| `stamp`, `rpy`, `body_height`, `yaw_speed` | `0` | Not filled by this firmware. This is normal. For turning, use `imu_state.gyroscope` |

**Other commands that worked:**

| What it does | Command |
|---|---|
| Lists every topic the robot publishes (we found **111**) | `ros2 topic list --no-daemon --spin-time 5` |
| Saves the list with message types to a file | `ros2 topic list -t --no-daemon --spin-time 5 > ~/unitree_a2/topics_full.txt` |
| Shows who publishes and subscribes to a topic | `ros2 topic info -v /lf/sportmodestate` |
| Lists nodes. It returns **nothing**, which is normal: the robot's programs are plain DDS apps, not ROS 2 nodes | `ros2 node list --no-daemon --spin-time 6` |
| Reads a battery | `ros2 topic echo /lf/bmsstate unitree_hg/msg/BmsState --once` |

Useful topics to read: `/lf/sportmodestate` (pose, IMU, speed), `/lowstate` (joints and motors, type `unitree_hg/msg/LowState`), `/lf/bmsstate` and `/lf/secondary_bmsstate` (the two batteries), `/lf/secondary_imu`, `/uwbstate` (the UWB follow-me companion module). The full list with types is in `~/unitree_a2/topics_full.txt`.

---

## 8. Read robot data with the Python SDK

This prints the robot's state continuously. It is read-only. Open a **new terminal** (not the `a2ros` one):
```bash
a2py
```

&nbsp;

```bash
cd ~/unitree_a2/unitree_sdk2_python/example/a2/sport
```

&nbsp;

```bash
python3 a2_sport_state.py enp2s0
```
✅ Robot state keeps scrolling. Stop it with **Ctrl+C**.

Check you're using the right Python: `which python3` should print `~/unitree_a2/venv/bin/python3`.

> The A2 part of the Python SDK (`unitree_sdk2py/a2`) only has **sport** (movement and state) and **audio**.
> Examples in the `go2/`, `obstacles_avoid/` and LiDAR folders are for the Go2 robot and don't work on the A2. Exception: the Go2 **`VideoClient`** works on the A2 for camera frames.

---

## 9. C++ SDK: robot state and audio recording

*Built on 30 Sep 2026 in `~/unitree_a2/unitree_sdk2` (laptop). The full guide is `~/unitree_a2/CPP_SDK_GUIDE.md`.*

⚠️ Use a **plain new terminal**. Do **not** run `a2ros` in it.

**Read the robot state (read-only):**
```bash
cd ~/unitree_a2/unitree_sdk2/build/bin
```

&nbsp;

```bash
./a2_sport_state enp2s0
```
✅ It keeps printing `Mode: STAND_DOWN` along with state values. Stop it with Ctrl+C.

&nbsp;

**Audio example.** The robot speaks, flashes its LEDs and records **10 seconds** from its microphone. We changed the recording from 5 s to 10 s. Run it from `build/`, **not** `build/bin`:
```bash
cd ~/unitree_a2/unitree_sdk2/build
```

&nbsp;

```bash
./bin/a2_audio_client_example enp2s0
```
✅ What you'll see:
- `SetVolume to 100% , API ret:0`. The volume goes to 100%, so it's loud.
- The robot speaks.
- `start record!`: start talking now.
- About 10 seconds later, `record finish! save to record.wav`.
- Press **Ctrl+C**, because the program doesn't exit by itself.

`ret:0` means the command worked. `{"play_state":1}` / `0` means the robot started / finished speaking.

> How the microphone works: PC1 sends it continuously as raw 16 kHz mono audio on UDP multicast `239.168.123.161:5555`. The listen tool (§12) uses the same stream.

**How we changed the recording length:** edit line 16 of `example/a2/audio/a2_audio_client_example.cpp` to `#define WAV_SECOND 10`, then rebuild (in `~/unitree_a2/unitree_sdk2/build`):
```bash
make -j$(nproc) a2_audio_client_example
```

---

## 10. Log into PC2 and the Jetson

PC2 is the PRO's extra computer (Ubuntu 22.04, ROS 2 Humble), **shared with other groups**. The Jetson (`.164`) is the docking hub with 4G, GNSS and Wi-Fi.

**PC2 with the shortcut** (key-based; all teleop/talk/listen tools and Claude use it):
```bash
ssh a2-pc2
```
✅ Logs in without a password.

&nbsp;

**PC2 with a password:**
```bash
ssh unitree@192.168.123.162
```
Ask the team for the password (it is not stored in this repository). The factory default `123` does **not** work. Nothing appears while you type it; that's normal.

&nbsp;

**The Jetson:**
```bash
ssh unitree@192.168.123.164
```
✅ The prompt shows `unitree@tegra-ubuntu`. Its sudo password is **not** PC2's (ask Yufeng).

&nbsp;

**Password-free login on a new laptop, or after PC2 was reset/replaced** (one-time, cable plugged in).

Create a key if you don't have one (press Enter at every question):
```bash
ssh-keygen -t ed25519
```

&nbsp;

Add the `a2-pc2` shortcut that the teleop tools use:
```bash
printf '\nHost a2-pc2\n  HostName 192.168.123.162\n  User unitree\n  ServerAliveInterval 15\n' >> ~/.ssh/config
```

&nbsp;

Copy the key to PC2 and the Jetson (each asks for that computer's password once):
```bash
ssh-copy-id a2-pc2
```
✅ `Number of key(s) added: …`.

&nbsp;

```bash
ssh-copy-id unitree@192.168.123.164
```
If `ssh-copy-id` fails, log in with the password and add the contents of your `~/.ssh/*.pub` to `~/.ssh/authorized_keys` by hand. If SSH warns **"host key changed"**, follow `~/A2_PRO_BACKUP/RESTORE_AFTER_FACTORY_RESET.md` step B2.

To leave, type `exit`.

**Shared-PC2 rules:** work only in our own folders (`~/a2_lidar`, `~/a2_wireless`, `~/a2_battery`); don't install, update or remove system packages; never remove Anaconda; don't touch `slam_config`, `dist` or `graph_pid_ws`.

---

## 11. LiDARs on PC2: start, check, stop

*Driver installed 6 Oct, restored after the repair 20 Oct. Full details: `DAY_2026-10-06_MANUAL.md` §3–8 and §12, `DAY_2026-10-20_MANUAL.md` §4–5.*

**What it is:** the two Hesai JT128 LiDARs send raw data to PC2 only. Our **Hesai ROS 2 driver** in `~/a2_lidar` turns it into point clouds (`/lidar_a/points` front, `/lidar_b/points` rear, 10 Hz, 115,200 points each). `obstacle_sectors.py` reads those clouds, finds the floor, and sends the nearest obstacles and the safety-oval status to the teleop (UDP 9600). **Nothing starts at boot**: start it each session.

**Start** (🤖 PC2):
```bash
source ~/a2_lidar/env.sh
```

&nbsp;

1. LiDAR driver:
```bash
setsid nohup ros2 run hesai_ros_driver hesai_ros_driver_node --ros-args -p config_path:=$HOME/a2_lidar/a2_hesai_config.yaml > ~/a2_lidar/driver.log 2>&1 < /dev/null &
```

&nbsp;

```bash
grep -E "ptc connect|correction file" ~/a2_lidar/driver.log
```
✅ `ptc connect success` twice and `Read correction file from lidar success` twice.

&nbsp;

```bash
timeout 6 ros2 topic hz /lidar_a/points
```
✅ `average rate: 10.0`. Same for `/lidar_b/points`.

&nbsp;

2. Obstacle detection + safety oval (feeds the teleop brake, about 27 % of one CPU core):
```bash
setsid nohup python3 -u ~/a2_lidar/obstacle_sectors.py --tf-a "0.31 0 0 1.5708 0 1.5708" --tf-b "-0.31 0 0 1.5708 0 -1.5708" > ~/a2_lidar/obstacle_sectors.log 2>&1 < /dev/null &
```

&nbsp;

```bash
tail -2 ~/a2_lidar/obstacle_sectors.log
```
✅ `[OK ] front … m | left … | oval … | fresh [1, 1]`.

&nbsp;

**Live 3D map in the browser** (optional, 7 Oct). 💻 Laptop:
```bash
bash ~/unitree_a2/tools/a2_view3d.sh on 2
```
✅ Opens http://localhost:8096. Use 5 fps on the AP / `A2-4G`, 1–2 fps over Tailscale. Turn it off with `bash ~/unitree_a2/tools/a2_view3d.sh off`. It doesn't touch the brake or oval.

**Checks after any repair or hardware change** (🤖 PC2, driver running):

| Check | Command | Pass |
|---|---|---|
| Which LiDAR is front (cover front, then rear, 15 s each) | `timeout -s INT 75 python3 -u ~/a2_lidar/cover_test.py` | Front covered → `lidar_a` 100 % blocked |
| Mounting (open floor front and back) | `timeout -s INT 40 python3 -u ~/a2_lidar/check_mounting.py` | 100 % of points on the correct side; floor tilt about 2° or less (20 Oct: front 0.8°, rear 2.6° → re-check on flat floor) |

**Stop** (🤖 PC2). ⚠️ Stop by **name**; `driver.pid` holds the launcher, not the driver:
```bash
pkill -INT -f "[o]bstacle_sectors.py"
```

&nbsp;

```bash
pkill -INT -f "[h]esai_ros_driver_node"
```

&nbsp;

```bash
pgrep -fa "[h]esai_ros_driver_node|[o]bstacle_sectors.py"
```
✅ Prints nothing.

> 💡 The `[h]`/`[o]` brackets stop `pkill`/`pgrep` from matching their own command line (otherwise, inside `ssh '…'`, they kill the SSH session).

**Limits:** the LiDARs can't see under the body, and there's a blind strip about 16 cm long at the middle of each flank, 0.2–0.3 m out. Drops and stairs going down are probably **not** detected. Running the LiDAR driver costs about **7 W** of battery.

---

## 12. Teleop: drive, talk, listen, LiDAR brake

*Real-run tested (2 Oct; talk/listen/link guard 5 Oct; brake + oval 6–7 Oct). Dry run re-tested after the PC2 restore on 20 and 22 Oct. Full details: `DAY_2026-10-02_MANUAL.md` §10, `DAY_2026-10-05_MANUAL.md` §8–13, `DAY_2026-10-06_MANUAL.md` §10.*

**What it is:** a window on the laptop with the **live front camera**. The robot side runs **on PC2** and enforces the speed caps and safety stops, so a laptop or Wi-Fi problem can't make the robot run away. Works over the cable, the Unitree AP, `A2-4G` or Tailscale. **For the LiDAR brake, the driver and `obstacle_sectors.py` must be running on PC2 (§11).**

**Dry run first** (window, camera, banners, but **no motion is sent**):
```bash
a2py
```

&nbsp;

```bash
python3 ~/unitree_a2/teleop/a2_teleop_wifi_gui.py --dry-run
```

&nbsp;

**Real run** (robot lying down, 2 m clear, remote in hand, spotter):
```bash
python3 ~/unitree_a2/teleop/a2_teleop_wifi_gui.py
```
Type `stand`, click the window, start at speed level 1.

| Key | Action |
|---|---|
| W/S, A/D, Q/E | hold and combine: forward/back, sideways, turn |
| 1–5 | speed level (5 = PC2 hard cap 0.5 m/s, 0.8 rad/s) |
| Space | stop now |
| **T** (hold) | talk: your voice on the robot's speaker |
| **L** | listen: robot microphone on/off (muted while T is held) |
| **R** | re-arm driving after `LINK RECOVERED` |
| ESC | stop, lie down, exit |

**LiDAR brake and safety perimeter** (they can only slow or stop the robot, never add motion):

| Situation | Effect | Window shows |
|---|---|---|
| Nothing near | Normal driving | green `LiDAR brake on – front …` |
| Something **0.45 → 0.25 m** in front | Forward slowed (never below 0.10 m/s) | yellow `forward slowed` |
| Something **≤ 0.25 m** in front | Forward blocked; turning, sideways and reverse still work | red `LIDAR BRAKE – forward blocked` |
| Object inside the **perimeter** (0.20 m front, 0.10 m back, 0.20 m sides) | Only motions **toward** the object are blocked; you can always drive away | red `PERIMETER - object <side>` + `blocked: …` |
| No LiDAR data | **All motion frozen** | red `NO LIDAR DATA` |

Options: `--brake-stop 0.3 --brake-slow 0.6` (tune distances), `--no-lidar-brake` (drive without it, e.g. LiDAR broken; extra care), `--no-video` (over 4G).

**Link-loss guard:** a connection problem lasting **more than 1.6 s** stops the robot (standing) and loops the **"Danger loosing connection"** alarm. When the latency is back to normal for 3 s, the alarm stops; press **R** to drive again. **5 min** without a stable link → it lies down. Shorter glitches only pause the robot.

The robot side keeps running on PC2 if the link drops, and the window reconnects by itself. If an **old session** is still running, the window attaches to it. To end it (the robot lies down) and start fresh:
```bash
python3 ~/unitree_a2/teleop/a2_teleop_wifi_gui.py --reset
```

**Cable-only keyboard teleop** (no PC2 needed, so it works even if PC2 is wiped, but **no LiDAR brake**; top speed 0.20 m/s):
```bash
python3 ~/unitree_a2/teleop/a2_teleop.py --no-video
```
It asks before standing up. W/S forward/back, Q/E turn, Space stop, ESC lie down and exit.

Standalone tools: `teleop/a2_talk.py` (talk), `teleop/a2_listen.py` (listen).

**No autonomous walking** for now (team decision, 6 Oct). Any autonomous run needs the supervisor's OK and the risk-assessment check (gate G0) first.

---

## 13. Battery logger

*Built 17 Oct, first real run 19 Oct, **T7 PASS 20 Oct** (62 min, 0 gaps), running on PC2 since 22 Oct. Full details: `DAY_2026-10-19_MANUAL.md`, `DAY_2026-10-20_MANUAL.md` §3, `DAY_2026-10-22_MANUAL.md` §3–6, `~/unitree_a2/battery/README.md`.*

**What it does:** records both battery packs and the robot's state (mode, speed, turning) into a CSV, 10 rows a second. It adds up the energy used (V × I over time), labels each row (lying / standing / walking / turning / charging) and checks for gaps longer than 1 s. On Ctrl+C it writes a `_summary.txt` with **PASS/FAIL**. This data is what the battery AI model will learn from, so **start it first at every session**, whatever else the team is doing.

**On the laptop (cable):**
```bash
a2ros
```

&nbsp;

```bash
python3 ~/unitree_a2/battery/battery_logger.py
```
✅ Every 10 s: `... rows | load  87.0 W | used ... Wh | SoC 56/56 % | lying`. Logs go to `~/unitree_a2/battery/logs/`.

&nbsp;

**On PC2 (the robot logs itself, no laptop needed).** 🤖 PC2:
```bash
source ~/a2_battery/env.sh
```

&nbsp;

```bash
cd ~/a2_battery && setsid nohup python3 -u battery_logger.py --out-dir ~/a2_battery/logs > ~/a2_battery/logger.log 2>&1 < /dev/null &
```

&nbsp;

```bash
tail -2 ~/a2_battery/logger.log
```
✅ `... rows | load ... W | ... | lying`.

&nbsp;

**Stop it** (sends Ctrl+C, so it writes its summary). 🤖 PC2:
```bash
pkill -INT -f "[b]attery_logger.py"
```

&nbsp;

**Copy PC2 logs to the laptop.** 💻 Laptop:
```bash
scp 'a2-pc2:~/a2_battery/logs/battery_*' ~/unitree_a2/battery/logs/pc2/
```

> ⚠️ **Stop the logger before switching the robot off.** Otherwise all topics stop at once and the summary says FAIL (19 Oct).
> ⚠️ PC2's clock was **about 54 s behind** the laptop (22 Oct). Measure it at each session before comparing logs.

**Power report.** Average power per activity, or per named time block, with an optional plot. No ROS needed. 💻 Laptop:
```bash
python3 ~/unitree_a2/battery/power_report.py ~/unitree_a2/battery/logs/battery_20261020_150544.csv
```
✅ A table: `group  minutes  mean W  sd W  median W  SoC start-end  max degC`. Add `--blocks "A=15:21-15:50,B=15:50-16:15"` for time blocks, `--plot ~/Downloads/power.png` for a chart. (Fixed 25 Oct: "minutes" now counts only time spent in each activity. Copy the fixed file to PC2 `~/a2_battery/` at the next session.)

**Charging data:** not possible yet. Our charger only charges packs **outside** the robot; the first in-robot charging data will come with the Contact Charging Board.

---

## 14. Backup and restore

*First full backup 8 Oct (before the repair). It saved us: Weston replaced/re-imaged PC2 and we restored everything from it on 20 Oct. Latest: `2026-10-22_1741`.*

**Take a backup** at the end of every robot session (read-only on the robot, a few minutes, cable plugged in). 💻 Laptop:
```bash
bash ~/unitree_a2/tools/a2_backup.sh
```
✅ Ends with `laptop parts OK`, `PC2 OK`, `Jetson OK`.

What it saves in `~/A2_PRO_BACKUP/`: the laptop project folder, these manuals, project documents, our PC2 folders (`a2_lidar`, `a2_wireless`, `a2_gps`, `a2_battery`), Jetson config, and a dated PC2 archive each run. **No passwords or private keys.** It skips PC2 `*.log` files (they change while being read).

Also copy `~/A2_PRO_BACKUP/A2_PRO_essentials.zip` to Google Drive or a USB stick now and then, so a copy exists off the laptop.

**Restore after a reset or repair:** follow `~/A2_PRO_BACKUP/RESTORE_AFTER_FACTORY_RESET.md` Part B (look first → host keys → key login → put our PC2 programs back → re-check hardware → Jetson → fresh backup). The short version used on 20 Oct is in `DAY_2026-10-20_MANUAL.md` §4. If PC2 lost `~/unitree_ros2`, the battery messages for PC2 are rebuilt in `~/a2_battery/ws` (`DAY_2026-10-22_MANUAL.md` §5).

---

## 15. ROS MCP + rosbridge

*Installed and tested on 30 Sep 2026.* This lets the Claude Code assistant read robot topics. **Cable only** (rosbridge runs on the laptop and needs DDS).

**Installed once (already done):** `ros-humble-rosbridge-server` (apt), `uv`, and `claude mcp add --scope user ros-mcp -- uvx ros-mcp --transport=stdio`.
```bash
claude mcp list
```
✅ Shows `ros-mcp` as connected.

&nbsp;

**Each session:** start rosbridge in its own terminal and leave it open:
```bash
a2ros
```

&nbsp;

```bash
ros2 launch rosbridge_server rosbridge_websocket_launch.xml
```
✅ It prints `Rosbridge WebSocket server started on port 9090`. Claude then connects to `127.0.0.1:9090`. Press Ctrl+C to stop it.

If rosbridge isn't running, Claude can still read topics with plain `ros2 topic echo` on the laptop.

---

## 16. Setup checker script

This checks everything in order: laptop software → cable → ping → robot data → rosbridge.
It prints ✅ / ⚠️ / ❌ for each check, with a fix for every problem, and it only reads, never changes anything.

**Laptop only, no robot needed:**
```bash
bash ~/.claude/skills/a2-robot-setup/scripts/check_setup.sh --laptop
```

&nbsp;

**Laptop + robot (cable):**
```bash
bash ~/.claude/skills/a2-robot-setup/scripts/check_setup.sh
```
**Fix the first ❌ first.** Later checks depend on the earlier ones.

---

## 17. GPS programs (tested with a fake GPS)

*Tested on the laptop only, with a simulated GPS. Not yet tested on the robot.*

> ⚠️ `~/unitree_a2/GPS_GUIDE.md` is **out of date**: it assumes the GPS is on PC2. It is in the **Quectel EG25-G 4G/GNSS modem on the Jetson** (`.164`), NMEA on `/dev/ttyUSB1`, which the `unitree` user can't read yet (not in group `dialout`). See `ROBOT_FACTS.md` (2026-10-01).

**Terminal 1** (prints e.g. `FAKE GPS on /dev/pts/5`):
```bash
source ~/unitree_a2/gps/venv/bin/activate
```

&nbsp;

```bash
cd ~/unitree_a2/gps
```

&nbsp;

```bash
python3 fake_gps.py
```

&nbsp;

**Terminal 2** (same two setup boxes first, then; use the port terminal 1 printed):
```bash
python3 a2_gps_reader.py --port /dev/pts/5
```
✅ Shows fix, lat/lon, satellites. Add `--csv gps_log.csv` to save the data to a file.

---

## 18. Problems we hit and how we fixed them

| Problem you see | Why it happens | Fix |
|---|---|---|
| `topic [/lf/sportmodestate] does not appear to be published yet` | An old ROS 2 background "daemon" is using old network settings | `ros2 daemon stop`, then retry, or add `--no-daemon` |
| `The daemon is not running` after `ros2 daemon stop` | It was already stopped | Nothing to fix; carry on |
| `ModuleNotFoundError: No module named 'unitree_sdk2py'` | The Python environment isn't active | Run `a2py`; the prompt must show `(venv)`. Don't use `sudo python3` |
| `a2ros: command not found` | The terminal was opened before the shortcut existed, or it's a new laptop | `source ~/.bashrc`, or add the alias (section 3, step 4) |
| Laptop only shows an IPv6 `fe80::` address | The robot doesn't give out addresses | Set a fixed IPv4 address (section 3, step 2) |
| `ipv4... unrecognised` when running `nmcli` | The command was split over two lines when pasted | Put the whole command on **one line** |
| `ddsi_udp_conn_write ... failed` and the interface shows `DOWN` | The cable link dropped | Reseat the cable in **Ethernet 1**; check the robot is on; `ip -br addr` must show `UP` |
| Build killed (out of memory) | Too many compile jobs at once | Build with `MAKEFLAGS=-j2` |
| rosbridge crashes: `rclpy._rclpy_pybind11` missing | Anaconda's Python clashed with ROS (which needs Python 3.10) | Fixed: Anaconda was removed from the **laptop** (never from PC2). `a2ros` also switches conda off |
| Ubuntu popup "ros2 has crashed" | A `ros2` command was piped into `head` | Harmless. Dismiss it |
| C++ audio: `audio file format error` | The program was started from `build/bin` and couldn't find `test.wav` | Run it from `~/unitree_a2/unitree_sdk2/build` as `./bin/a2_audio_client_example enp2s0` |
| SSH `Permission denied` | Wrong password for this firmware | Ask the team for the right password (not `123`) |
| `ssh a2-pc2` suddenly asks for a password | PC2 was reset or replaced, so our key is gone | `ssh-copy-id a2-pc2` (§10), then check what's missing (§14) |
| `~/a2_lidar` / `~/a2_wireless` missing on PC2 | Repair/reset wiped PC2 | Restore from `~/A2_PRO_BACKUP` (§14) |
| PC2 can't decode battery messages | `~/unitree_ros2` gone after the reset | Use `~/a2_battery/env.sh` (own build of the messages) |
| PC1/PC2 no ping right after power-on | Robot still booting | Wait 1–2 minutes |
| ROS 2 topics missing on the laptop over Wi-Fi | NAT between the laptop and the robot | Read data on PC2 (`ssh a2-pc2`) |
| 4G down, "SIM missing" | The modem reset itself, or the SIM is out | Power-cycle the robot; check the SIM |
| Teleop alarm / `LINK UNSTABLE` | Link problem over 1.6 s | Get closer / better signal; press **R** after `LINK RECOVERED` |
| Teleop shows `NO LIDAR DATA`, nothing moves | Driver or `obstacle_sectors.py` not running on PC2 | Start them (§11) |
| `kill $(cat ~/a2_lidar/driver.pid)` doesn't stop the driver | The `.pid` holds the launcher, not the driver | `pkill -INT -f "[h]esai_ros_driver_node"` (§11) |
| `pkill -f …` inside `ssh '…'` ends the SSH session | The pattern matches the remote shell's own command | Use the bracket trick `"[h]esai…"` |
| Backup: PC2 part fails, "file changed as we read it" | A running program writes its `.log` | Fixed: the backup skips PC2 `*.log` |
| Battery summary says FAIL, all topics silent at the end | Robot switched off before the logger was stopped | Not a bug. Stop the logger first |
| Slow walking or turning not labelled in the battery log | Firmware leaves `yaw_speed` at 0 | Fixed 19 Oct: logger uses the IMU's `gyro_z` and a 0.05 m/s walking threshold |

More: the problem tables in each day manual.

---

## 19. Safe shutdown

Do it in this order: **software first → robot → cable.**
1. In the teleop window press **ESC** and wait for `Robot lying down`. Press **Ctrl+C** in every other laptop terminal that's running something (battery logger: Ctrl+C **before** the robot goes off).
2. 🤖 PC2: stop our programs by name (§11 and §13):
```bash
pkill -INT -f "[b]attery_logger.py"
```

&nbsp;

```bash
pkill -INT -f "[o]bstacle_sectors.py"
```

&nbsp;

```bash
pkill -INT -f "[h]esai_ros_driver_node"
```

&nbsp;

3. 💻 Laptop: copy the PC2 battery logs (§13), then take a backup (§14):
```bash
bash ~/unitree_a2/tools/a2_backup.sh
```

&nbsp;

4. If you used ROS 2:
```bash
ros2 daemon stop
```

&nbsp;

5. `exit` from PC2 / the Jetson if you're logged in.
6. Make sure the robot is **lying down**.
7. **Power off** the robot with the battery button (see the A2 manual).
8. **Unplug** the Ethernet cable. If you used `A2-4G`, switch the laptop back to normal Wi-Fi:
```bash
nmcli con down a2-4g
```
Nothing else needs undoing on the laptop.

---

## 20. Safety rules

- ✅ **Safe any time (read-only):** `ros2 topic echo`, `ros2 topic list`, `a2_sport_state` (Python and C++), `a2_listen.py`, the battery logger, the LiDAR driver, `obstacle_sectors.py`, `cover_test.py`, `check_mounting.py`, the backup script, and the teleop window with `--dry-run`.
- 🔊 **Makes sound but doesn't move:** the audio examples, `a2_talk.py`, and the link-guard alarm (100 % volume). They can be loud.
- ⚠️ **Moves the robot:** the teleop window (without `--dry-run`), `a2_teleop.py`, `a2_sport_client` (Python or C++), the fun-move scripts, anything with `sport_client` or `low_level`, and anything that publishes to `/api/...` or `/lowcmd`.
  - Needs 2 m of clear space, a flat floor, **2 people** (one is the spotter), and the **remote in hand as the emergency stop**. The signed risk assessment requires a spotter whenever the robot stands or walks.
  - **Check with the lab engineer first.** When a class is running, the robot must stay lying down (22 Oct).
  - Use the Wi-Fi teleop with the **LiDAR brake on** whenever PC2's LiDAR programs are available. `a2_teleop.py` has no brake: walk slowly.
  - On the cable, keep it slack and away from the legs.
  - Keep the robot **in direct sight**: the camera lags.
  - **Remote driving (Tailscale)** only with a person next to the robot holding the remote, and with the supervisor's OK.
  - Don't drive with the remote and the laptop at the same time.
  - **No autonomous walking** until the supervisor approves it and the risk assessment covers it.
- ❌ **Never:**
  - Change anything on PC1 (`.161`).
  - Install, update or remove system packages on PC2, or remove Anaconda there (shared PC).
  - Reformat the Jetson (it links the robot network and holds 4G, the hotspot and Tailscale).
  - Publish to `/lowcmd`, `/wirelesscontroller` or `/api/bashrunner`.
  - Run examples from the `go2/`, `b2/`, `g1/` or `h1/` folders on the A2 (the Go2 `VideoClient` for camera frames is the one tested exception).
  - Send modem configuration commands (`AT+…`) to the Jetson's 4G modem.
  - Power the Mid-360 (or any 9–27 V device) from the battery rail (about 50 V).
