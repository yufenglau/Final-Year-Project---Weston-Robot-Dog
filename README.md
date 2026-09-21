# Autonomous Return-to-Base and Battery Charging for a Quadruped Inspection Robot

Power management for an autonomous site-documentation robot. The robot carries a 360-degree camera around a construction site, and this repository handles the part that keeps it running: monitoring its batteries, deciding when it must stop work, navigating back to its charging station, and docking.

Target platform: **Unitree A2 Pro**.

---

## 1. What This Project Does

Site documentation is currently done by walking a site manually with a camera, which is slow, inconsistent between visits, and produces incomplete records. An autonomous quadruped robot can repeat the same route reliably — but only if it can manage its own energy without someone watching it.

This work package covers:

- Reading battery telemetry from the robot (state of charge, voltage, current, temperature).
- Building an energy model that relates distance travelled to energy consumed.
- Deciding when the robot must abandon its mission and return to base.
- Navigating back to the charging station and docking accurately.
- Confirming that charging has started, and behaving safely when it has not.

Out of scope: inspection route planning, alarm detection, and camera mount design. These are handled separately, and the interfaces to them are listed in Section 6.

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
| Perception | Dual LiDAR (front and rear), HD camera |
| Onboard computing | 8-core CPU platform plus Intel Core i7 for user development |
| Auxiliary power | 12 V / 24 V / battery rail |

Two consequences of the dual-battery design:

1. **Telemetry is per pack.** Each pack is monitored independently, with an alarm on significant imbalance between them.
2. **Hot-swap is a viable fallback.** Because packs can be exchanged without powering down, "return to base for a supervised battery exchange" is a legitimate deliverable if autonomous docking proves infeasible.

### 2.1 Charging route — undecided

No official charging dock for the A2 Pro has been identified. Three routes are under consideration:

| Route | Approach | Risk |
|---|---|---|
| A | Integrate an official dock, if one exists and is procured | Low — depends on availability |
| B | Build a custom dock with charging contacts and AprilTag alignment | High — adds a mechanical and electrical sub-project |
| C | Return to base for a supervised manual battery exchange | Low — less autonomous, but achievable |

Current plan: implement Route C as a guaranteed deliverable, then extend to A or B if hardware and schedule allow.

---

## 3. Repository Contents

| File | Description |
|---|---|
| `battery_monitor.py` | Reads battery telemetry and logs it to CSV. Currently simulated; the hardware interface is isolated in one function marked `# REPLACE WITH REAL SDK CALL`. |
| `energy_model.py` | Derives energy consumption in Wh per metre from logged data and evaluates whether the robot can still reach the dock. |
| `state_machine.py` | MISSION → RETURNING → DOCKING → CHARGING → FAULT logic. Independent of ROS 2 so it can be tested in isolation. Integration points marked `# ROS 2 HOOK`. |
| `nav2_docking_params_example.yaml` | Configuration template for the Nav2 Docking Server, annotated with the parameters needing measurement against the physical dock. |
| `resources.md` | Reference list — SDK, ROS 2 and Nav2 documentation, battery safety, relevant literature. |

The Python modules were written before hardware was available, so the decision logic could be developed and tested without the robot. All hardware-dependent behaviour sits in clearly marked functions, meaning the simulated data source can be swapped for live SDK calls without touching the energy model or state machine.

> **Outstanding:** the capacity constant is still set to a 500 Wh placeholder from before the platform was confirmed. Update it to **907.2 Wh** in all three modules.

---

## 4. Environment

| Component | Version |
|---|---|
| OS | Ubuntu 22.04 LTS |
| Middleware | ROS 2 Humble |
| Navigation | Nav2, including the built-in Docking Server |
| Simulation | Gazebo |
| Languages | Python 3.10, C++ |

A VirtualBox VM running Ubuntu 22.04 is adequate for ROS 2 basics and the plain-Python modules here. A native dual-boot install is recommended before Nav2 and Gazebo work begins, since 3D simulation plus navigation planning exceeds what a VM handles comfortably.

Nav2 installs separately when needed:

```bash
sudo apt install ros-humble-navigation2 ros-humble-nav2-bringup -y
```

### Running the simulation modules

No ROS 2 installation required — these run on any system with Python 3:

```bash
python3 battery_monitor.py   # Simulates a patrol cycle; writes battery_log.csv
python3 energy_model.py      # Derives Wh/m and evaluates the return-to-dock decision
python3 state_machine.py     # Exercises the state machine against scripted telemetry
```

---

## 5. Design

The work is structured as six capability levels. Each is a self-contained deliverable, so slippage at a later level does not invalidate earlier work.

| Level | Capability |
|---|---|
| 1 | Per-pack telemetry logged and displayed |
| 2 | Measured consumption (Wh/m) standing, walking at varying speeds, and with the camera payload active |
| 3 | Return decision based on remaining energy versus energy required to reach the dock |
| 4 | Navigation to a staging pose beside the dock |
| 5 | Precision docking, with charging confirmed and retry on misalignment |
| 6 | Fault handling — dock unreachable, battery over-temperature, emergency signal received |

### Return-to-base policy

The return trigger is deliberately not a fixed state-of-charge threshold. A fixed threshold ignores how far the robot is from the dock: it strands the robot when far away and cuts missions short when close. The policy is instead:

```
return when:  remaining energy ≤ (distance to dock × measured Wh per metre) + safety margin
```

The margin covers navigation inefficiency, detours around unmapped obstacles, and the reduced accuracy of state-of-charge estimation at low charge.

---

## 6. Interfaces

| Dependency | Interface |
|---|---|
| Navigation | This package issues a goal to return the robot to the dock staging pose — contract to be agreed |
| Safety response | An emergency alarm must be able to interrupt or suspend an in-progress return or docking sequence |
| Camera payload | Draws auxiliary power from the robot; consumption must be measured and included in the energy model |

---

## 7. Status

| Item | Status |
|---|---|
| Platform confirmed | Done — Unitree A2 Pro |
| Ubuntu 22.04 environment | Done — VM verified |
| Native dual-boot install | Planned |
| ROS 2 Humble install | In progress |
| Simulation modules | Done — capacity constant still needs updating |
| Risk assessment | Drafted, awaiting signatures |
| SDK access | Pending |
| Charging route decision | Pending — see Section 2.1 |
| Hardware access | Pending |

### Open questions

Several of these determine the technical approach and cannot be deferred:

1. Does an official A2 charging dock exist, and will one be available?
2. What is the charger specification — voltage, current, connector? Needed for any custom dock design.
3. Access to the A2 SDK Development Guide and developer credentials.
4. What battery data does the SDK expose per pack — charge, voltage, current, temperature, charging state?
5. Is the A2 supported by `unitree_ros2`? The official repository lists Go2, B2, H1 and G1 only; a bridge may be required.
6. Access to the onboard i7 development computer, and its OS and ROS version.
7. Verified mass of the robot as configured.

---

## 8. Safety

A formal risk assessment covering twelve hazards has been prepared separately and requires signature before practical work begins. The operational rules derived from it:

- No person within reach of the robot while it is powered and moving; 2 m minimum clearance during walking tests.
- New control or navigation software is validated in simulation before running on the robot; first hardware runs are at reduced speed inside a cordoned area.
- The emergency stop is verified at the start of every session, with a second person present as a spotter during powered operation.
- Battery charging happens at a designated point with fire extinguishing equipment available, is never left unattended, and stops if a pack becomes hot or swollen.
- Both packs are removed before any mechanical work on the robot.
- Any custom dock is approved before being energised, and supplied through a fused, RCD-protected circuit with shielded contacts.
- Software includes a fault timeout that commands the robot to sit on loss of communication or fault detection.

---

## 9. References

**Platform**
- Unitree developer support — https://support.unitree.com
- Unitree SDK2 — https://github.com/unitreerobotics/unitree_sdk2
- Unitree ROS 2 — https://github.com/unitreerobotics/unitree_ros2

**Middleware and navigation**
- ROS 2 Humble — https://docs.ros.org/en/humble/
- Nav2 — https://docs.nav2.org
- Nav2 Docking Server — https://docs.nav2.org/configuration/packages/configuring-docking-server.html
- `opennav_docking` — https://github.com/open-navigation/opennav_docking

**Comparable deployments**
- OpenSpace, 360-degree documentation with a quadruped — https://www.openspace.ai/blog/construction-360-photo-documentation-easy-for-spot-the-robot/
- Boston Dynamics, autonomous charging and site documentation — https://bostondynamics.com/industry/construction/

**Battery safety**
- Battery University — https://batteryuniversity.com

Literature search terms: *autonomous recharging mobile robot*; *docking station quadruped robot*; *energy-aware path planning*; *battery state of charge estimation*.
