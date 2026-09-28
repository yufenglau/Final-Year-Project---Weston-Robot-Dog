"""
AI-based battery-capability prediction for the FYP's power management system.

WHAT THIS ADDS (vs. the deterministic energy_model.py formula):
    The original return-to-dock trigger was a hand-derived formula:
        return when: remaining_energy <= (distance_to_dock * measured_Wh_per_m) + margin
    That's solid engineering, but it's not "AI-based prediction" -- it assumes a
    CONSTANT Wh/m rate. Real discharge isn't constant: it depends on how hard the
    battery has already been worked, temperature, and subtle non-linearities in
    how lithium cells behave near empty. A trained model can pick up on patterns
    like this that a single fixed number can't.

    This script trains a small regression model that looks at the robot's CURRENT
    telemetry (SOC, voltage, current draw, temperature, recent discharge trend)
    and predicts how much further distance the battery can sustain -- instead of
    assuming a fixed rate. The deterministic formula in energy_model.py can then
    consume this model's prediction as its input, instead of a raw linear estimate.

HOW TO USE THIS FILE:
    1. Run it directly: `python3 battery_prediction_model.py`
       This simulates several varied patrol runs (different speeds, payloads,
       conditions), trains the model, evaluates it, and saves it to disk.
    2. Once you have REAL logged data from the physical A2 Pro (same CSV shape as
       battery_monitor.py already produces), replace `generate_training_runs()`
       with a function that loads your real log files instead. The rest of the
       pipeline (features, training, evaluation) does not need to change.
    3. Import `predict_remaining_distance()` from state_machine.py / energy_model.py
       to use the trained model at decision time.
"""

import numpy as np
import pandas as pd
from dataclasses import dataclass
from sklearn.ensemble import RandomForestRegressor
from sklearn.linear_model import LinearRegression
from sklearn.metrics import mean_absolute_error, root_mean_squared_error
import joblib

MODEL_PATH = "battery_prediction_model.joblib"


# ---------------------------------------------------------------------------
# STEP 1: Training data. Until real A2 Pro logs exist, we simulate several
# DIFFERENT patrol runs (varying speed, payload, ambient temperature) so the
# model sees a range of discharge behaviour, not just one fixed pattern.
# ---------------------------------------------------------------------------

@dataclass
class SimBattery:
    capacity_wh: float = 907.2       # A2 Pro dual-pack total capacity
    nominal_voltage: float = 50.4    # derived -- verify against real pack label
    soc: float = 100.0
    temperature_c: float = 25.0

    def step(self, dt_s, walking_speed_mps, payload_draw_w, ambient_c):
        # crude but varied discharge model: base hotel load + speed-dependent
        # draw + payload draw + a small nonlinearity as SOC gets low (real cells
        # sag faster near empty -- this is exactly the kind of pattern a fixed
        # Wh/m formula can't represent but a trained model can pick up on)
        base_w = 40.0
        motion_w = 180.0 * (walking_speed_mps ** 1.6)
        low_soc_penalty = 1.0 + max(0.0, (30.0 - self.soc) / 30.0) * 0.35
        power_w = (base_w + motion_w + payload_draw_w) * low_soc_penalty
        energy_wh = power_w * dt_s / 3600.0
        self.soc = max(0.0, self.soc - 100.0 * energy_wh / self.capacity_wh)
        self.temperature_c += (ambient_c - self.temperature_c) * 0.01 + motion_w * 0.0003
        current_a = power_w / self.nominal_voltage
        distance_m = walking_speed_mps * dt_s
        return power_w, current_a, distance_m


def simulate_one_run(run_id, speed_mps, payload_w, ambient_c, dt_s=5.0, seed=0):
    rng = np.random.default_rng(seed)
    batt = SimBattery(temperature_c=ambient_c)
    rows = []
    distance_total = 0.0
    t = 0.0
    while batt.soc > 2.0:
        speed = max(0.05, speed_mps + rng.normal(0, 0.05))
        power_w, current_a, dist_m = batt.step(dt_s, speed, payload_w, ambient_c)
        distance_total += dist_m
        rows.append({
            "run_id": run_id, "t_s": t, "soc_percent": batt.soc,
            "voltage_v": batt.nominal_voltage - (100 - batt.soc) * 0.02,
            "current_a": current_a, "temperature_c": batt.temperature_c,
            "distance_traveled_m": distance_total,
        })
        t += dt_s
    df = pd.DataFrame(rows)
    # label: how much further did the robot actually go from each point onward?
    df["remaining_distance_m"] = distance_total - df["distance_traveled_m"]
    return df


def generate_training_runs():
    """Several varied runs -- swap this out for real logged CSVs later."""
    configs = [
        dict(speed_mps=0.6, payload_w=15, ambient_c=25),
        dict(speed_mps=0.9, payload_w=15, ambient_c=25),
        dict(speed_mps=0.6, payload_w=35, ambient_c=30),   # camera payload, hot day
        dict(speed_mps=1.2, payload_w=15, ambient_c=20),   # fast walk, cool day
        dict(speed_mps=0.8, payload_w=25, ambient_c=35),   # hot construction site
        dict(speed_mps=0.5, payload_w=40, ambient_c=28),   # slow, heavy payload
    ]
    runs = [simulate_one_run(i, seed=i, **cfg) for i, cfg in enumerate(configs)]
    return pd.concat(runs, ignore_index=True)


# ---------------------------------------------------------------------------
# STEP 2: Features. A rolling discharge-rate feature lets the model see recent
# TREND, not just a single instant -- this is what actually makes it different
# from reading raw SOC.
# ---------------------------------------------------------------------------

def add_features(df):
    df = df.sort_values(["run_id", "t_s"]).copy()
    df["recent_discharge_rate"] = (
        df.groupby("run_id")["soc_percent"].diff(3).fillna(0) / 3.0 * -1.0
    )
    return df


FEATURES = ["soc_percent", "voltage_v", "current_a", "temperature_c", "recent_discharge_rate"]
TARGET = "remaining_distance_m"


# ---------------------------------------------------------------------------
# STEP 3: Train / evaluate. Split by RUN, not by row -- rows from the same
# patrol are correlated, so splitting rows randomly would leak information
# and make the model look better than it really is.
# ---------------------------------------------------------------------------

def train_and_evaluate():
    data = add_features(generate_training_runs())

    run_ids = data["run_id"].unique()
    test_runs = set(run_ids[-2:])          # hold out 2 whole runs for testing
    train_df = data[~data["run_id"].isin(test_runs)]
    test_df = data[data["run_id"].isin(test_runs)]

    X_train, y_train = train_df[FEATURES], train_df[TARGET]
    X_test, y_test = test_df[FEATURES], test_df[TARGET]

    models = {
        "Linear Regression (baseline)": LinearRegression(),
        "Random Forest (main model)": RandomForestRegressor(
            n_estimators=200, max_depth=8, random_state=0
        ),
    }

    print(f"Training rows: {len(X_train)} | Test rows: {len(X_test)} "
          f"(held out {len(test_runs)} full runs)\n")

    best_model, best_mae = None, float("inf")
    for name, model in models.items():
        model.fit(X_train, y_train)
        preds = model.predict(X_test)
        mae = mean_absolute_error(y_test, preds)
        rmse = root_mean_squared_error(y_test, preds)
        print(f"{name}")
        print(f"  Mean Absolute Error:  {mae:8.1f} m")
        print(f"  Root Mean Sq. Error:  {rmse:8.1f} m")
        print(f"  (for reference, test-run distances ranged up to "
              f"{y_test.max() + test_df['distance_traveled_m'].max():.0f} m)\n")
        if mae < best_mae:
            best_model, best_mae = model, mae

    joblib.dump({"model": best_model, "features": FEATURES}, MODEL_PATH)
    print(f"Saved best model to {MODEL_PATH} (MAE = {best_mae:.1f} m)")
    return best_model


# ---------------------------------------------------------------------------
# STEP 4: Inference -- this is what energy_model.py / state_machine.py calls
# at decision time, in place of the fixed Wh/m projection.
# ---------------------------------------------------------------------------

def predict_remaining_distance(soc_percent, voltage_v, current_a, temperature_c,
                                recent_discharge_rate, model_path=MODEL_PATH):
    bundle = joblib.load(model_path)
    model, features = bundle["model"], bundle["features"]
    row = pd.DataFrame([{
        "soc_percent": soc_percent, "voltage_v": voltage_v, "current_a": current_a,
        "temperature_c": temperature_c, "recent_discharge_rate": recent_discharge_rate,
    }])[features]
    return float(model.predict(row)[0])


if __name__ == "__main__":
    trained = train_and_evaluate()

    print("\n--- Example inference call (what the state machine would do) ---")
    example_pred = predict_remaining_distance(
        soc_percent=35.0, voltage_v=49.0, current_a=3.2,
        temperature_c=31.0, recent_discharge_rate=0.4,
    )
    print(f"At 35% SOC, model predicts ~{example_pred:.0f} m of range remaining.")
