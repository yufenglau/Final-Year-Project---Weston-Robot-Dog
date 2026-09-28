"""
Standalone power-management state machine: MISSION -> RETURNING -> DOCKING ->
CHARGING -> FAULT.

Kept independent of ROS 2 so the decision logic can be written and tested
before Nav2/hardware are available. Every point where a real ROS 2 action
call (Nav2 goal, DockRobot action, e-stop subscription, etc.) will eventually
plug in is marked "# ROS 2 HOOK".

Run directly:
    python3 state_machine.py
"""

from dataclasses import dataclass
from enum import Enum, auto


class State(Enum):
    MISSION = auto()
    RETURNING = auto()
    DOCKING = auto()
    CHARGING = auto()
    FAULT = auto()


@dataclass
class Telemetry:
    soc_percent: float
    temperature_c: float
    distance_to_dock_m: float
    emergency_alarm: bool = False
    dock_reachable: bool = True


class PowerManager:
    def __init__(self, wh_per_metre: float = 0.7, capacity_wh: float = 907.2,
                 safety_margin_percent: float = 15.0, resume_soc_percent: float = 90.0,
                 max_safe_temp_c: float = 45.0):
        self.wh_per_metre = wh_per_metre
        self.capacity_wh = capacity_wh
        self.safety_margin_percent = safety_margin_percent
        self.resume_soc_percent = resume_soc_percent
        self.max_safe_temp_c = max_safe_temp_c
        self.state = State.MISSION

    def _return_threshold(self, distance_to_dock_m: float) -> float:
        """SOC% below which the robot must start returning, given distance to dock."""
        energy_to_dock_wh = distance_to_dock_m * self.wh_per_metre
        margin_wh = self.capacity_wh * self.safety_margin_percent / 100.0
        return 100.0 * (energy_to_dock_wh + margin_wh) / self.capacity_wh

    def step(self, telem: Telemetry) -> State:
        prev_state = self.state

        # Faults take priority over everything else.
        if telem.emergency_alarm or telem.temperature_c > self.max_safe_temp_c:
            self.state = State.FAULT

        elif self.state == State.MISSION:
            threshold = self._return_threshold(telem.distance_to_dock_m)
            if telem.soc_percent <= threshold:
                self.state = State.RETURNING

        elif self.state == State.RETURNING:
            if not telem.dock_reachable:
                self.state = State.FAULT
            elif telem.distance_to_dock_m <= 1.0:
                self.state = State.DOCKING

        elif self.state == State.DOCKING:
            # ROS 2 HOOK: this transition should be driven by the Nav2
            # Docking Server's DockRobot action result (success/failure/
            # retry), not just proximity.
            self.state = State.CHARGING

        elif self.state == State.CHARGING:
            if telem.soc_percent >= self.resume_soc_percent:
                self.state = State.MISSION

        elif self.state == State.FAULT:
            # ROS 2 HOOK: recovery from FAULT should require an explicit
            # operator acknowledgement / clear-fault service call, not an
            # automatic transition.
            if not telem.emergency_alarm and telem.temperature_c <= self.max_safe_temp_c:
                pass  # stays in FAULT until an operator clears it

        if self.state != prev_state:
            self._on_transition(prev_state, self.state)
        return self.state

    def _on_transition(self, old_state: State, new_state: State):
        print(f"[state change] {old_state.name} -> {new_state.name}")
        # ROS 2 HOOK: publish state transitions, e.g. to a /power_manager/state
        # topic, and trigger the corresponding Nav2 goal (send return-to-dock
        # goal on RETURNING, send DockRobot action on DOCKING, etc.).


def _demo():
    pm = PowerManager()
    scenario = [
        Telemetry(soc_percent=90.0, temperature_c=28.0, distance_to_dock_m=200.0),
        Telemetry(soc_percent=40.0, temperature_c=30.0, distance_to_dock_m=150.0),
        Telemetry(soc_percent=20.0, temperature_c=31.0, distance_to_dock_m=100.0),  # crosses threshold -> RETURNING
        Telemetry(soc_percent=18.0, temperature_c=31.0, distance_to_dock_m=50.0),
        Telemetry(soc_percent=16.0, temperature_c=31.0, distance_to_dock_m=0.5),    # close enough -> DOCKING
        Telemetry(soc_percent=16.0, temperature_c=31.0, distance_to_dock_m=0.0),    # docked -> CHARGING
        Telemetry(soc_percent=60.0, temperature_c=27.0, distance_to_dock_m=0.0),
        Telemetry(soc_percent=91.0, temperature_c=25.0, distance_to_dock_m=0.0),    # charged -> MISSION
    ]
    for telem in scenario:
        pm.step(telem)

    print(f"Final state: {pm.state.name}")


if __name__ == "__main__":
    _demo()
