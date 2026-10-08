# Autonomous Return-to-Base and Battery Charging for a Quadruped Inspection Robot (Updating...)

Power management for an autonomous site-documentation robot. The robot carries a 360-degree camera around a construction site, and this repository handles the part that keeps it running: monitoring its batteries, predicting remaining range, deciding when it must stop work, navigating back to its charging station, and docking.

Target platform: **Unitree A2 Pro**.

---

## 1. What This Project Does

Site documentation is currently done by walking a site manually with a camera, which is slow, inconsistent between visits, and produces incomplete records. An autonomous quadruped robot can repeat the same route reliably — but only if it can manage its own energy without someone watching it.

This work package covers:

- Reading battery telemetry from the robot (state of charge per pack, voltage, current, temperature).
- Building an energy model that relates distance travelled to energy consumed.
- Predicting remaining range with a trained machine-learning model rather than a fixed consumption rate.
- Deciding when the robot must abandon its mission and return to base.
- Navigating back to a staging pose in front of the charging pad.
- Aligning precisely onto the pad using LiDAR, and confirming that charging has started.

Out of scope: inspection route planning, general navigation/obstacle-avoidance demos, alarm detection, camera mount design, and hardware fabrication. These are handled separately; the interfaces to them are listed in Section 7.

---

## 2. Target Platform

| Parameter | Value |
|---|---|
| Battery | Dual hot-swappable packs, 2 × 9000 mAh |
| Total capacity | 907.2 Wh |
| Nominal voltage | ≈ 50.4 V *(derived — verify against pack label)* |
| Charge time | Approximately 1 hour |
| Endurance | Up to 5 h / 20 km unloaded; approximately 3 h / 12.5 km at 25 kg payload |
| Mass | Approximately 37–42 kg *(sources disagree — verify)* |
| Perception | Dual LiDAR (front and rear), HD camera, GPS (expanded feature on Pro) |
| Connectivity | Wi-Fi 6, Bluetooth 5.2, Gigabit Ethernet |
| Auxiliary power | 12 V / 24 V / battery rail |

### 2.1 Onboard computers and network

| Unit | IP | Role |
|---|---|---|
| PC1 — motion control unit | `192.168.123.161` | Runs Unitree's locomotion controller. Receives SDK2 commands. Not user-accessible; updated only via OTA. |
| PC2 — user development unit (Intel i7, Ubuntu 22.04) | `192.168.123.162` / `192.168.124.162` | Where this project's code runs. SSH as user `unitree`. Also hosts Unitree's SLAM/navigation service. |
| LiDARs (front / rear) | `192.168.124.20` / `.21` | Connected to PC2. |

- **The 123 subnet** (Switch 1) is the only one that supports SDK control.
- **The 124 subnet** (Switch 2) reaches PC2 and the LiDARs only, and is not supported for SDK development.

### 2.2 Charging hardware

Unitree's official **Contact Charging Board** is used: a flat pad powered by a lithium battery charger box. The robot lies down on the pad so that two electrode contacts on its underside meet two contacts on the pad. Charging stops automatically when the battery is full.

As documented by Unitree, positioning onto the pad is **manual** — an operator aligns the robot to a printed sticker by eye using the remote controller. Automating this alignment is the core engineering contribution of this project.

| Route | Approach | Status |
|---|---|---|
| A | Automate alignment onto the official pad using the robot's own LiDAR | **Primary** |
| B | Custom dock with dedicated alignment features | Not pursued (out of scope — no hardware fabrication) |
| C | Return to base for a supervised manual battery hot-swap | Guaranteed fallback |

**Why not GPS:** GPS accuracy is metres at best and degrades indoors or near structures, while electrode alignment needs millimetre-to-centimetre precision. GPS may be used only for coarse "near the dock" awareness on large outdoor sites.

---

## 3. System Architecture

```
 battery % (rt/slam_info)          energy model + ML range prediction
        │                                       │
        └──────────────► state machine ◄────────┘
                              │
          MISSION ──► RETURNING ──► DOCKING ──► CHARGING
                          │             │
          SLAM nav API 1102 to     LiDAR alignment
          pre-dock pose (~1 m      onto contact pad
          in front of charger)     (this project)
```

1. **Trigger** — battery percentage for both packs is read from `rt/slam_info` (`batteryPower`). The energy model and ML predictor decide whether the remaining charge still covers the distance back to the dock plus a safety margin.
2. **Return** — PC2 already runs Unitree's SLAM/navigation service (`slam_operate`). The state machine calls API `1102` to walk the robot to a stored pre-dock pose; arrival is reported on `rt/slam_key_info` (`is_arrived`).
3. **Dock** — this project's LiDAR alignment code takes over for the final approach, then commands the robot to lie down on the pad.
4. **Confirm** — charging state is checked; on failure the robot retries or enters FAULT.

### Unitree SLAM/navigation service interface

| API ID | Function |
|---|---|
| `1801` / `1802` | Start mapping / stop mapping and save `.pcd` |
| `1804` | Set initial pose on a saved map (relocalisation) |
| `1102` | Navigate to target pose (≤ 30 m, straight-line; obstacles must be ≥ 20 cm high) |
| `1201` / `1202` / `1901` | Pause / resume / shut down SLAM |

Requirements: JT128 LiDAR, updated LiDAR firmware, `slam_nav` above v1.0.0 via OTA. Works best indoors, on flat ground, in areas under 45 m × 45 m. *(Firmware and OTA versions to be confirmed with the supplier.)*

---

## 4. Repository Contents

| File | Description |
|---|---|
| `battery_monitor.py` | Battery telemetry logging to CSV. Currently simulated; the hardware interface is isolated in one function marked `# REPLACE WITH REAL SDK CALL`, to be replaced by a subscriber to `rt/slam_info`. |
| `energy_model.py` | Derives energy consumption in Wh per metre from logged data and evaluates whether the robot can still reach the dock. |
| `state_machine.py` | MISSION → RETURNING → DOCKING → CHARGING → FAULT logic. Independent of ROS 2 so it can be tested in isolation. Integration points marked `# ROS 2 HOOK`. |
| `battery_prediction_model.py` | Trains a Random Forest regressor to predict remaining range from live telemetry (SOC, voltage, current, temperature, recent discharge rate), compared against a Linear Regression baseline. Split by whole run to avoid data leakage. Saves the model with `joblib`. |
| `nav2_docking_params_example.yaml` | Configuration template for the Nav2 Docking Server (reference only; the robot's own navigation service is now the primary path). |
| `resources.md` | Reference list — SDK, ROS 2 and Nav2 documentation, battery safety, relevant literature. |

All modules use the confirmed platform capacity of **907.2 Wh / 50.4 V**.

The Python modules were written before hardware was available, so the decision logic could be developed and tested without the robot. All hardware-dependent behaviour sits in clearly marked functions, so the simulated data source can be swapped for live robot data without touching the energy model, predictor, or state machine.

### ML model results (simulated data)

| Model | Mean Absolute Error | RMSE |
|---|---|---|
| Linear Regression (baseline) | 1134 m | 1208 m |
| Random Forest (main) | 842 m | 1037 m |

Test runs covered distances up to ~25.8 km. The model will be retrained on real A2 Pro logs once collected; only the data-loading function changes.

---

## 5. Environment and Setup

| Component | Version |
|---|---|
| Robot PC2 OS | Ubuntu 22.04 LTS |
| Middleware | ROS 2 Humble |
| Robot SDK | `unitree_sdk2` (C++); `unitree_sdk2_python` also available |
| Simulation | Gazebo Classic 11 with TurtleBot3 as a stand-in platform |
| Languages | Python 3.10, C++ |

### Connecting to the robot

**Wired (most reliable):** connect to the Switch 1 aviation Ethernet port (left side, under the T45 cover), set the laptop to `192.168.123.222/24`, then:

```bash
ping 192.168.123.161        # PC1
ssh unitree@192.168.123.162 # PC2
```

**Wireless (AP mode):**
1. Switch the A2 to AP mode in the Unitree app; join the `A2_xxxxx` hotspot.
2. Disconnect all other networks on the laptop.
3. Set a manual IP: `192.168.12.10/24`, gateway `192.168.12.1`.
4. Run Unitree's static-route script once (`unitree_slam` repo, branch `unitree_slam_A2`, folder `AP_Remote_Connection`).
5. `ssh unitree@192.168.123.162`

Notes for wireless development:
- Use high-level commands only over Wi-Fi; keep low-level control on a wired link.
- Run long jobs inside `tmux` so a dropped connection does not kill them.
- A virtual machine is not recommended by Unitree. Use the laptop/VM as an SSH terminal and build and run on PC2.
- The robot must fail safe (sit/stop) on loss of connection; the physical emergency stop is always kept in hand.

### Building the SDK on PC2

```bash
sudo apt-get install -y cmake g++ build-essential libyaml-cpp-dev libeigen3-dev libboost-all-dev libfmt-dev
git clone https://github.com/unitreerobotics/unitree_sdk2.git
cd unitree_sdk2 && mkdir build && cd build
cmake .. && make
./bin/a2_sport_client <interface>    # enter 3 = lie down, 4 = stand
```

Do not run `sudo make install` on the shared PC2 without agreement from the other groups.

### Running the simulation modules

No ROS 2 installation required — these run on any system with Python 3:

```bash
python3 battery_monitor.py            # Simulates a patrol cycle; writes battery_log.csv
python3 energy_model.py               # Derives Wh/m and evaluates the return-to-dock decision
python3 state_machine.py              # Exercises the state machine against scripted telemetry
python3 battery_prediction_model.py   # Trains and evaluates the range predictor
```

### Shared PC2 rules

The robot is shared with other project groups:
- Do not modify `/home/unitree/slam_config`, `/home/unitree/dist` or `/home/unitree/graph_pid_ws`.
- Do not update or uninstall existing packages; installing new ones is fine. Use Docker for complex dependencies.
- There is no recovery image. One agreed Timeshift backup is taken before development begins.
- Keep all work for this project in its own folder on PC2.

---

## 6. Design

The work is structured as six capability levels. Each is a self-contained deliverable, so slippage at a later level does not invalidate earlier work.

| Level | Capability | Status |
|---|---|---|
| 1 | Per-pack telemetry logged and displayed | Done in simulation; real source identified (`rt/slam_info`) |
| 2 | Measured consumption (Wh/m) and ML range prediction | Done in simulation |
| 3 | Return decision based on remaining energy versus energy required to reach the dock | Done in simulation |
| 4 | Navigation to a pre-dock pose beside the charger | Next — via SLAM service API `1102` |
| 5 | Precision LiDAR docking, charging confirmed, retry on misalignment | Not started |
| 6 | Fault handling — dock unreachable, over-temperature, emergency signal, lost connection | Designed in state machine; not yet on hardware |

### Return-to-base policy

The return trigger is deliberately not a fixed state-of-charge threshold. A fixed threshold ignores how far the robot is from the dock: it strands the robot when far away and cuts missions short when close. The policy is instead:

```
return when:  remaining energy ≤ (distance to dock × Wh per metre) + safety margin
```

The Wh-per-metre term can be supplied either by the measured average or by the ML predictor. The margin covers navigation inefficiency, detours around obstacles, and the reduced accuracy of state-of-charge estimation at low charge.

---

## 7. Interfaces

| Dependency | Interface |
|---|---|
| Navigation | Unitree SLAM/navigation service on PC2 (API `1102` for pre-dock pose; shared map built with APIs `1801`/`1802`) |
| General navigation demo | Owned by another team member on the same robot |
| Safety response | An emergency alarm must be able to interrupt or suspend an in-progress return or docking sequence |
| Camera payload | Draws auxiliary power from the robot; consumption must be measured and included in the energy model |

---

## 8. Status

| Item | Status |
|---|---|
| Platform confirmed | Done — Unitree A2 Pro |
| Ubuntu 22.04 + ROS 2 Humble environment | Done |
| Gazebo simulation (TurtleBot3 stand-in) | Done — world launches and runs |
| Simulation modules | Done — capacity updated to 907.2 Wh |
| ML range predictor | Done on simulated data |
| Risk assessment | Done — 30 hazards, signed |
| Physical robot access | Done |
| SDK documentation | Done — official A2 SDK Development Guide |
| Wireless connection to PC2 | In progress |
| SDK2 build on PC2 and first motion test | Next |
| Charging route | Route A (LiDAR alignment) primary; Route C fallback |

### Open questions

1. LiDAR firmware version and `slam_nav` OTA version — must meet the SLAM service requirements (confirm with supplier).
2. Electrode contact tolerance: how much positioning error the pad accepts before contact fails. This sets the accuracy LiDAR alignment must achieve.
3. Return trigger threshold and safety margin values, to be set from real discharge data.
4. Timing of the shared Timeshift backup, agreed with the other groups.
5. Verified mass of the robot as configured.

---

## 9. Safety

A formal risk assessment covering thirty hazards has been prepared and signed. The operational rules derived from it:

- No person within reach of the robot while it is powered and moving; 2 m minimum clearance during walking tests.
- New control or navigation software is validated in simulation before running on the robot; first hardware runs start with the robot at rest on the ground, then at reduced speed inside a cordoned area.
- The emergency stop is verified at the start of every session, with a second person present as a spotter during powered operation.
- The robot must sit or stop on loss of communication — critical now that development is over Wi-Fi.
- Battery charging happens at a designated point with fire extinguishing equipment available, is never left unattended, and stops if a pack becomes hot or swollen.
- Both packs are removed before any mechanical work on the robot.

---

## 10. References

**Platform**
- Unitree A2 SDK Development Guide — https://support.unitree.com/home/en/A2_SDK_Development_Guide
- User Development Unit — https://support.unitree.com/home/en/A2_SDK_Development_Guide/develop_module
- Quick Development — https://support.unitree.com/home/en/A2_SDK_Development_Guide/quick_development
- SLAM and Navigation Service Interface — https://support.unitree.com/home/en/A2_SDK_Development_Guide/slam_and_navigation_service_interface
- Contact Charging Board — https://support.unitree.com/home/en/A2_SDK_Development_Guide/a2_charge_pad
- Unitree SDK2 — https://github.com/unitreerobotics/unitree_sdk2
- Unitree SDK2 Python — https://github.com/unitreerobotics/unitree_sdk2_python
- Unitree SLAM (A2 AP-mode route scripts) — https://github.com/unitreerobotics/unitree_slam

**Middleware and navigation**
- ROS 2 Humble — https://docs.ros.org/en/humble/
- Nav2 — https://docs.nav2.org

**Comparable deployments**
- OpenSpace, 360-degree documentation with a quadruped — https://www.openspace.ai/blog/construction-360-photo-documentation-easy-for-spot-the-robot/
- Boston Dynamics, autonomous charging and site documentation — https://bostondynamics.com/industry/construction/

**Battery safety**
- Battery University — https://batteryuniversity.com

Literature search terms: *autonomous recharging mobile robot*; *docking station quadruped robot*; *LiDAR-based docking*; *energy-aware path planning*; *battery state of charge estimation*; *remaining useful range prediction*.
