"""
Deterministic energy model: turns logged battery data into a Wh/m consumption
rate, and evaluates the return-to-dock decision:

    return when:  remaining_energy <= (distance_to_dock * measured_Wh_per_m) + margin

Consumes the CSV produced by battery_monitor.py. Capacity matches the
confirmed A2 Pro platform (907.2 Wh, dual hot-swappable packs).
"""

import csv


def load_log(path: str):
    rows = []
    with open(path, newline="") as f:
        reader = csv.DictReader(f)
        for r in reader:
            rows.append({
                "timestamp": float(r["timestamp"]),
                "soc_percent": float(r["soc_percent"]),
                "voltage_v": float(r["voltage_v"]),
                "current_a": float(r["current_a"]),
                "temperature_c": float(r["temperature_c"]),
                "distance_traveled_m": float(r["distance_traveled_m"]),
            })
    return rows


def estimate_wh_per_metre(rows, capacity_wh: float = 907.2) -> float:
    """Average Wh consumed per metre travelled, from a logged run."""
    if len(rows) < 2:
        raise ValueError("Need at least two samples to estimate a rate.")
    first, last = rows[0], rows[-1]
    soc_used_percent = first["soc_percent"] - last["soc_percent"]
    distance_m = last["distance_traveled_m"] - first["distance_traveled_m"]
    if distance_m <= 0:
        raise ValueError("No distance travelled in log -- cannot estimate rate.")
    wh_used = capacity_wh * soc_used_percent / 100.0
    return wh_used / distance_m


def predict_remaining_range(current_soc_percent: float, wh_per_metre: float,
                             capacity_wh: float = 907.2) -> float:
    """How much further distance the remaining charge can sustain, in metres."""
    remaining_wh = capacity_wh * current_soc_percent / 100.0
    if wh_per_metre <= 0:
        return float("inf")
    return remaining_wh / wh_per_metre


def should_return_to_dock(current_soc_percent: float, distance_to_dock_m: float,
                           wh_per_metre: float, capacity_wh: float = 907.2,
                           safety_margin_percent: float = 15.0):
    """
    Core return-to-base policy (see README section 5). Deliberately not a
    fixed SOC threshold -- it accounts for how far the robot actually is
    from the dock right now.
    """
    energy_to_dock_wh = distance_to_dock_m * wh_per_metre
    margin_wh = capacity_wh * safety_margin_percent / 100.0
    remaining_wh = capacity_wh * current_soc_percent / 100.0
    threshold_soc_percent = 100.0 * (energy_to_dock_wh + margin_wh) / capacity_wh

    must_return = remaining_wh <= (energy_to_dock_wh + margin_wh)
    return must_return, threshold_soc_percent


if __name__ == "__main__":
    rows = load_log("battery_log.csv")
    wh_per_m = estimate_wh_per_metre(rows)
    current_soc = rows[-1]["soc_percent"]
    remaining_range_m = predict_remaining_range(current_soc, wh_per_m)

    distance_to_dock_m = 40.0
    must_return, threshold_soc = should_return_to_dock(current_soc, distance_to_dock_m, wh_per_m)

    print(f"Estimated consumption: {wh_per_m:.4f} Wh/m")
    print(f"Current SOC: {current_soc:.2f}% -> estimated remaining range: {remaining_range_m:.1f} m")
    print(f"Distance to dock: {distance_to_dock_m:.1f} m | "
          f"Return trigger threshold: {threshold_soc:.1f}% SOC")
    print(f"DECISION: {'return to dock' if must_return else 'continue mission'}")
