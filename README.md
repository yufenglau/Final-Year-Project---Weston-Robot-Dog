# Final-Year-Project---Weston-Robot-Dog

# FYP — Power Management: Autonomous Return-to-Base & Charging

Challenge 3 of the AlphaMini/OpenSpace-Weston Robot FYP. This starter kit lets you
build and test the power-management logic **before** you ever touch the physical
robot dog, using simulated battery data. Every module is written so the "fake"
parts are isolated in one place — when the robot and SDK arrive, you swap those
functions for real calls and the rest of your code doesn't change.

## What's in this folder

| File | Purpose |
|---|---|
| `battery_monitor.py` | Reads battery telemetry (SOC, voltage, current, temp) and logs it to CSV. Currently simulated — see the `# REPLACE WITH REAL SDK CALL` marker. |
| `energy_model.py` | Turns raw logs into Wh-per-metre and predicts whether the robot can still reach the dock. |
| `state_machine.py` | The MISSION → RETURN → DOCK → CHARGING → FAULT logic. Runs standalone so you can test it with fake inputs. |
| `nav2_docking_params_example.yaml` | Template config for Nav2's built-in Docking Server (so you don't write docking control from scratch). |
| `resources.md` | Curated links for every stage — SDK docs, ROS 2/Nav2 tutorials, papers, dock hardware. |

## Week-by-week starting plan (now → mid-August)

**Week 1 — environment & fundamentals**
- Install Ubuntu 22.04 (dual boot or VM) + ROS 2 Humble. Do NOT try to learn ROS 2 on Windows — it will fight you the whole way.
- Work through the official ROS 2 "Beginner: CLI tools" tutorials (link in resources.md).
- Run `battery_monitor.py` and `energy_model.py` in this folder (pure Python, no ROS needed) to get comfortable with the data shape you'll be working with.

**Week 2 — battery & energy fundamentals**
- Read up on Li-ion/LiPo SOC estimation, C-rates, and safe charging temperature ranges (resources.md has beginner-friendly sources — this is safety-critical, not optional).
- Extend `energy_model.py` with real numbers once you get manufacturer battery specs (capacity in Wh, nominal voltage) from your lab engineer.

**Week 3 — ROS 2 + Nav2 basics**
- Nav2 "Getting Started" tutorial in simulation (Gazebo, TurtleBot3 example — not your real robot, just to learn the framework).
- Read the Nav2 Docking Server docs and the AprilTag docking tutorial (resources.md). This is the exact framework you'll adapt for the Unitree dock.

**Week 4 — state machine & integration prep**
- Get `state_machine.py` running standalone with simulated telemetry.
- Confirm from your lab engineer: exact Unitree model/variant, dock hardware ownership, SDK access. This determines whether autonomous docking is even physically supported (only Go2 EDU Plus + Mid-360/XT16 LiDAR supports auto-docking with the official board).
- Draft your interface contract with the navigation-challenge teammate (what service/topic you call to say "take me home").

**Mid-August onward (robot in hand)**
- Swap the simulated read in `battery_monitor.py` for the real Unitree SDK battery state message.
- Wire `state_machine.py` into a real ROS 2 node.
- Configure `nav2_docking_params_example.yaml` for your actual dock (official board or AprilTag fallback).
- Start with docking tests on a bench/tabletop mockup before full-speed lab tests.

## How to run the starter code now

```bash
python3 battery_monitor.py      # simulates a 20-minute patrol, writes battery_log.csv
python3 energy_model.py         # reads battery_log.csv, prints Wh/m and a return-or-not decision
python3 state_machine.py        # runs the state machine against simulated telemetry, prints transitions
```

No ROS 2 install needed for these three — they're plain Python so you can start today on any laptop.

## Safety checklist to start drafting now (for your report + your junior handover)

- [ ] Dock only on level, dry ground, away from foot/vehicle traffic
- [ ] RCD/GFCI-protected charging circuit
- [ ] Battery temperature checked before charge starts, monitored during charge
- [ ] Charging aborts and alerts operator if temperature or voltage is out of safe range
- [ ] Manual override / e-stop always reachable regardless of autonomy state
- [ ] Return-to-base threshold includes a safety margin, not just "reach the dock with 0% left"
- [ ] Emergency alarm signal (from teammate's safety-responsiveness module) can interrupt docking/charging
