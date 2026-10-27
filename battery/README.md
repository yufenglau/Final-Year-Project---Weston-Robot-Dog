# Battery logger (T7, objective O1)

Records the A2's battery data, plus what the robot was doing, into a CSV file 10 times a second.

| File | What it is |
|---|---|
| `battery_logger.py` | **The real logger.** Same code on the laptop and on PC2 |
| `fake_bms_publisher.py` | A pretend robot for laptop tests: same topics, types and units as the A2 |
| `logs/` | Output: `battery_<date>_<time>.csv` + `_summary.txt` |

## How it works, in four steps

1. **Listen.** It subscribes to `/lf/bmsstate` (pack 1), `/lf/secondary_bmsstate` (pack 2) and `/lf/sportmodestate` (body). Each sends 20 messages per second.
2. **Convert units.** mV becomes V and mA becomes A, all in one function (`pack_values`). If T7 shows the current sign is the other way round, add `--flip-current`.
3. **Add up energy.** For every battery message: energy += V × I × time since the last message. This is needed because SoC only changes in 1 % steps. It never adds energy across a gap.
4. **Write and check.** One CSV row every 0.1 s. It warns if any topic goes quiet for more than 1 s, and on Ctrl+C it writes a summary that says **PASS** or **FAIL** for the T7 gap target.

## Test on the laptop (no robot)

Terminal 1, the pretend robot:

```bash
a2local
```

```bash
python3 ~/unitree_a2/battery/fake_bms_publisher.py
```

Terminal 2, the logger:

```bash
a2local
```

```bash
python3 ~/unitree_a2/battery/battery_logger.py
```

Stop the logger with **Ctrl+C** and read the summary it prints.

To test gap detection, start the pretend robot with a 2.5 s dropout on pack 2:

```bash
python3 ~/unitree_a2/battery/fake_bms_publisher.py --gap-at 15 --gap-len 2.5
```

## Main CSV columns

| Column | Meaning |
|---|---|
| `p1_v`, `p1_v_cells` | Pack voltage from the BMS, and the sum of the 14 cells (should agree) |
| `p1_i_a` | Pack current in A, negative = discharging (sign still to be confirmed) |
| `load_w` | Total power taken from both packs (negative while charging) |
| `energy_used_wh` | Energy taken out since the logger started |
| `mode`, `speed_mps` | Raw body state (kept so labels can be redone later). `mode` is 3 for both standing and walking |
| `gyro_z`, `yaw_deg` | Turning rate (rad/s) and heading (°) from the IMU. `yaw_speed` is always 0 on this firmware, so these are used instead |
| `activity` | Provisional label: lying / standing / walking / turning / charging |
| `p1_age_ms`, `gap` | How old the latest message was; 1 = a topic is currently silent |

## Not done yet

- Run on PC2 (copy to `~/a2_battery`, ROS env from PC2's `.bashrc`). Also check it with the real robot.
- T7 sign check on the robot: does `p1_i_a` get more negative when the robot stands up and walks?
- Also record a rosbag (raw data, for replay), and summarise `/lowstate` leg power.
- Start it automatically at every session.
- In the fake publisher, only the "lie" power (87 W) is measured. Stand, walk and charge are guesses.
