"""
Battery telemetry simulation for pre-hardware development.

Simulates a walking patrol and logs per-timestep battery state (SOC, voltage,
current, temperature, distance travelled) to CSV, in the same shape real
Unitree A2 Pro SDK telemetry will eventually take. Only one function
(`read_battery_state`) touches the "hardware" -- swap that one function for a
real SDK call later and everything downstream (energy_model.py,
state_machine.py, battery_prediction_model.py) keeps working unchanged.

Run directly:
    python3 battery_monitor.py
"""

import csv
from dataclasses import dataclass


@dataclass
class BatteryState:
    timestamp: float
    soc_percent: float
    voltage_v: float
    current_a: float
    temperature_c: float
    distance_traveled_m: float


class SimulatedBattery:
    """
    Stand-in for the physical A2 Pro dual-pack battery until SDK access is
    available. Capacity and nominal voltage match the confirmed platform.
    """

    def __init__(self, capacity_wh: float = 907.2, nominal_voltage: float = 50.4):
        self.capacity_wh = capacity_wh
        self.nominal_voltage = nominal_voltage
        self.soc_percent = 100.0
        self.temperature_c = 25.0
        self.distance_traveled_m = 0.0

    def step(self, dt_s: float, walking_speed_mps: float, docked: bool = False):
        if docked:
            # Charging: roughly full in ~1 hour, per platform spec.
            charge_rate_percent_per_s = 100.0 / 3600.0
            self.soc_percent = min(100.0, self.soc_percent + charge_rate_percent_per_s * dt_s)
            power_w = 0.0
            current_a = 0.0
        else:
            base_w = 40.0                      # hotel load: compute, sensors, comms
            motion_w = 180.0 * (walking_speed_mps ** 1.6)
            power_w = base_w + motion_w
            energy_wh = power_w * dt_s / 3600.0
            self.soc_percent = max(0.0, self.soc_percent - 100.0 * energy_wh / self.capacity_wh)
            self.distance_traveled_m += walking_speed_mps * dt_s
            current_a = power_w / self.nominal_voltage

        self.temperature_c += (25.0 - self.temperature_c) * 0.01
        return power_w, current_a

    def read_battery_state(self, t: float) -> BatteryState:
        # REPLACE WITH REAL SDK CALL
        # Once SDK access is available, this function should instead query the
        # A2 Pro's actual battery telemetry (per-pack SOC, voltage, current,
        # temperature) and return the same BatteryState shape, so nothing
        # downstream needs to change.
        voltage_v = self.nominal_voltage - (100 - self.soc_percent) * 0.02
        current_a = 0.0
        return BatteryState(
            timestamp=t,
            soc_percent=self.soc_percent,
            voltage_v=voltage_v,
            current_a=current_a,
            temperature_c=self.temperature_c,
            distance_traveled_m=self.distance_traveled_m,
        )


def run_simulated_patrol(duration_s: float = 1200, dt_s: float = 5.0,
                          walking_speed_mps: float = 0.7,
                          log_path: str = "battery_log.csv"):
    battery = SimulatedBattery()
    rows = []
    t = 0.0
    while t < duration_s and battery.soc_percent > 0.0:
        power_w, current_a = battery.step(dt_s, walking_speed_mps)
        state = battery.read_battery_state(t)
        state.current_a = current_a
        rows.append(state)
        t += dt_s

    with open(log_path, "w", newline="") as f:
        writer = csv.writer(f)
        writer.writerow(["timestamp", "soc_percent", "voltage_v", "current_a",
                          "temperature_c", "distance_traveled_m"])
        for s in rows:
            writer.writerow([s.timestamp, f"{s.soc_percent:.4f}", f"{s.voltage_v:.3f}",
                              f"{s.current_a:.3f}", f"{s.temperature_c:.2f}",
                              f"{s.distance_traveled_m:.2f}"])

    print(f"Logged {len(rows)} samples to {log_path}")
    print(f"Final SOC: {rows[-1].soc_percent:.2f}% | "
          f"Distance traveled: {rows[-1].distance_traveled_m:.1f} m")
    return rows


if __name__ == "__main__":
    run_simulated_patrol()
