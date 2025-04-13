from dataclasses import dataclass
from typing import List, Tuple

@dataclass
class Waypoint:

    '''
    Variable Name: x_coordinate: The x-axis position of the waypoint, expected range: any float value.
    Variable Name: y_coordinate: The y-axis position of the waypoint, expected range: any float value.
    Variable Name: z_altitude: The z-axis altitude of the waypoint (0.0 by default for 2D support), expected range: any float value.
    '''
    
    x: float
    y: float
    z_altitude: float = 0.0  # Include z_altitude for 3D support

    '''
    * Function Name: to_tuple
    * Input:
    * None: Uses instance attributes (x, y, z_altitude).
    * Output:
    * Tuple[float, float, float]: A tuple containing the 3D coordinates (x, y, z).
    * Logic: Returns the waypoint coordinates as a tuple for easy processing.
    * Example Call:
    * coords = my_waypoint.to_tuple()

    '''

    def to_tuple(self) -> Tuple[float, float, float]:
        """Convert waypoint coordinates to a tuple."""
        return (self.x, self.y, self.z_altitude)

@dataclass
class Mission:

    '''
    Variable Name: waypoint_list: List of Waypoint objects defining the mission path, expected range: non-empty list.
    Variable Name: mission_start_time: The start time of the mission, expected range: any float value.
    Variable Name: mission_end_time: The end time of the mission, expected range: greater than mission_start_time.
    '''

    waypoints: List[Waypoint]
    t_start: float
    t_end: float

    '''
    * Function Name: validate
    * Input:
    * None: Uses instance attributes (waypoints, t_start, t_end).
    * Output:
    * bool: True if the mission is valid (non-empty waypoints and t_start < t_end), False otherwise.
    * Logic: Checks if the mission has at least one waypoint and a valid time window.
    * Example Call:
    * is_valid = my_mission.validate()
    '''

    def validate(self) -> bool:
        """Ensure the mission is valid."""
        return len(self.waypoints) > 0 and self.t_start < self.t_end

    '''
    * Function Name: generate_sample_schedules
    * Input:
    * None: Generates hardcoded sample data.
    * Output:
    * List[Mission]: A list of Mission objects representing sample flight schedules.
    * Logic: Creates two predefined missions with 3D waypoints and time windows for testing.
    * Example Call:
    * schedules = generate_sample_schedules()
    '''

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

    # Returns the list of schedules after populating with hardcoded waypoint and time data.
    return schedules