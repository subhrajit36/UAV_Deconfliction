from dataclasses import dataclass
from typing import List, Tuple

@dataclass
class Waypoint:
    x: float
    y: float
    z: float = 0.0  # Include z for 3D support

    def to_tuple(self) -> Tuple[float, float, float]:
        return (self.x, self.y, self.z)

@dataclass
class Mission:
    waypoints: List[Waypoint]
    t_start: float
    t_end: float

    def validate(self) -> bool:
        """Ensure the mission is valid."""
        return len(self.waypoints) > 0 and self.t_start < self.t_end

def generate_sample_schedules() -> List[Mission]:
    """Generate sample flight schedules with 3D waypoints."""
    schedules = [
        Mission(
            waypoints=[
                Waypoint(0, 0, 100),
                Waypoint(10, 10, 120),
                Waypoint(20, 0, 110)
            ],
            t_start=0,
            t_end=60
        ),
        Mission(
            waypoints=[
                Waypoint(5, 5, 115),
                Waypoint(15, 15, 125),
                Waypoint(25, 5, 105)
            ],
            t_start=30,
            t_end=90
        )
    ]
    return schedules