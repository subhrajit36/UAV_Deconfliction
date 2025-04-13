'''
Author List: Subhrajit Mahana
 Filename: deconfliction.py
 Theme: UAV Deconfliction - FlytBase Robotics Assignment 2025
 Functions: __init__, segment_distance, interpolate_position, check_spatial_conflict, check_temporal_conflict, check_mission
 Global Variables: None
 
 '''

from typing import List, Tuple, Optional
from utils import Waypoint, Mission
import numpy as np

class DeconflictionSystem:
    def __init__(self, safety_buffer_distance: float = 5.0):
        # Variable Name: safety_buffer_distance: The minimum distance threshold (in units) for detecting conflicts, expected range: 0.0 to infinity.
        # Variable Name: detected_conflicts_list: A list to store detected conflict details, initialized empty.
        self.safety_buffer_distance = safety_buffer_distance
        self.detected_conflicts_list = []

    """
    * Function Name: segment_distance
    * Input:
    * start_waypoint_segment_one: First waypoint of the first line segment (Waypoint object).
    * end_waypoint_segment_one: Second waypoint of the first line segment (Waypoint object).
    * start_waypoint_segment_two: First waypoint of the second line segment (Waypoint object).
    * end_waypoint_segment_two: Second waypoint of the second line segment (Waypoint object).
    * Output:
    * float: The shortest distance between the two line segments in 3D space.
    * Logic: Uses vector mathematics to compute the closest distance between two 3D line segments.
    * Employs dot product and normalization to find the closest points, with a fallback for point cases.
    * Example Call:
    * distance = deconfliction.segment_distance(wp1, wp2, wp3, wp4)

    """

    def segment_distance(self, start_waypoint_segment_one: Waypoint, end_waypoint_segment_one: Waypoint, start_waypoint_segment_two: Waypoint, end_waypoint_segment_two: Waypoint) -> float:
        """Calculate the closest distance between two line segments in 3D."""
        def dot_product(u, v): return u[0] * v[0] + u[1] * v[1] + u[2] * v[2]
        def vector_norm(v): return np.sqrt(dot_product(v, v))

        # Variable Name: point_a_coords: NumPy array of start_waypoint_segment_one coordinates, expected range: any 3D coordinates.
        # Variable Name: point_b_coords: NumPy array of end_waypoint_segment_one coordinates, expected range: any 3D coordinates.
        # Variable Name: point_c_coords: NumPy array of start_waypoint_segment_two coordinates, expected range: any 3D coordinates.
        # Variable Name: point_d_coords: NumPy array of end_waypoint_segment_two coordinates, expected range: any 3D coordinates.

        point_a_coords = np.array(start_waypoint_segment_one.to_tuple())
        point_b_coords = np.array(end_waypoint_segment_one.to_tuple())
        point_c_coords = np.array(start_waypoint_segment_two.to_tuple())
        point_d_coords = np.array(end_waypoint_segment_two.to_tuple())

        vector_ab = point_b_coords - point_a_coords
        vector_cd = point_d_coords - point_c_coords
        vector_ac = point_c_coords - point_a_coords

        denominator = vector_norm(vector_ab) * vector_norm(vector_cd)
        if denominator == 0:  # One or both segments are points
            # Describe what the code below is doing
            # Handles the case where one or both segments degenerate into points by computing minimum distances between endpoints.
            return np.min([np.linalg.norm(point_a_coords - point_c_coords), np.linalg.norm(point_a_coords - point_d_coords),
                           np.linalg.norm(point_b_coords - point_c_coords), np.linalg.norm(point_b_coords - point_d_coords)])

        parameter_s = np.clip(dot_product(vector_ac, vector_ab) / (vector_norm(vector_ab) ** 2), 0, 1)
        parameter_t = np.clip(-dot_product(vector_ac, vector_cd) / (vector_norm(vector_cd) ** 2), 0, 1)

        closest_point_segment_one = point_a_coords + parameter_s * vector_ab
        closest_point_segment_two = point_c_coords + parameter_t * vector_cd

        return np.linalg.norm(closest_point_segment_one - closest_point_segment_two)

    '''
    * Function Name: interpolate_position
    * Input:
    * drone_mission: The Mission object containing waypoints and time window (Mission object).
    * current_time: The time at which to interpolate the position (float).
    * Output:
    * Optional[Tuple[float, float, float]]: The interpolated 3D position as a tuple (x, y, z), or None if out of range.
    * Logic: Calculates the position at current_time by determining the segment index and interpolating between waypoints linearly.
    * Prints debug information for validation.
    * Example Call:
    * position = deconfliction.interpolate_position(my_mission, 45.1)

    '''

    def interpolate_position(self, drone_mission: Mission, current_time: float) -> Optional[Tuple[float, float, float]]:
        """Interpolate the drone's position at current_time in 3D."""
        if current_time < drone_mission.t_start or current_time > drone_mission.t_end or not drone_mission.validate():
            return None

        total_mission_time = drone_mission.t_end - drone_mission.t_start
        # Variable Name: segment_time_duration: The time duration for each waypoint segment, expected range: 0.0 to total_mission_time.
        segment_time_duration = total_mission_time / (len(drone_mission.waypoints) - 1)

        segment_index = int((current_time - drone_mission.t_start) / segment_time_duration)
        if segment_index >= len(drone_mission.waypoints) - 1:
            return drone_mission.waypoints[-1].to_tuple()

        time_within_segment = (current_time - drone_mission.t_start) % segment_time_duration
        interpolation_factor = time_within_segment / segment_time_duration

        start_waypoint = drone_mission.waypoints[segment_index]
        end_waypoint = drone_mission.waypoints[segment_index + 1]

        x_coord = start_waypoint.x + interpolation_factor * (end_waypoint.x - start_waypoint.x)
        y_coord = start_waypoint.y + interpolation_factor * (end_waypoint.y - start_waypoint.y)
        z_coord = start_waypoint.z_altitude + interpolation_factor * (end_waypoint.z_altitude - start_waypoint.z_altitude)  # Changed from z to z_altitude
        
        position = (x_coord, y_coord, z_coord)
        print(f"Debug: t={current_time}, Interpolated position: {position}")
        return position
    
    '''
    * Function Name: check_spatial_conflict
    * Input:
    * primary_mission: The primary Mission object to check (Mission object).
    * schedule_list: List of Mission objects representing other drone schedules (List[Mission]).
    * Output:
    * List[dict]: List of dictionaries containing conflict details (type, location, distance, flight_id).
    * Logic: Iterates over all segment pairs between the primary_mission and schedule_list, computing distances using segment_distance.
    * Records conflicts if distance < safety_buffer_distance.
    * Example Call:
    * conflicts = deconfliction.check_spatial_conflict(my_mission, [sched1, sched2])

    '''
    def check_spatial_conflict(self, primary_mission: Mission, schedule_list: List[Mission]) -> List[dict]:
        """Check for spatial conflicts in 3D."""
        self.detected_conflicts_list = []
        for flight_index, current_schedule in enumerate(schedule_list):
            for segment_index in range(len(primary_mission.waypoints) - 1):
                start_point_one, end_point_one = primary_mission.waypoints[segment_index], primary_mission.waypoints[segment_index + 1]
                for segment_index_two in range(len(current_schedule.waypoints) - 1):
                    start_point_two, end_point_two = current_schedule.waypoints[segment_index_two], current_schedule.waypoints[segment_index_two + 1]
                    distance = self.segment_distance(start_point_one, end_point_one, start_point_two, end_point_two)
                    if distance < self.safety_buffer_distance:
                        self.detected_conflicts_list.append({
                            "type": "spatial",
                            "location": (start_point_one.to_tuple(), end_point_one.to_tuple()),
                            "distance": distance,
                            "flight_id": flight_index
                        })
        return self.detected_conflicts_list

    '''
    * Function Name: check_temporal_conflict
    * Input:
    * primary_mission: The primary Mission object to check (Mission object).
    * schedule_list: List of Mission objects representing other drone schedules (List[Mission]).
    * Output:
    * List[dict]: List of dictionaries containing conflict details (type, time, location, distance, flight_id).
    * Logic: Interpolates positions at discrete time steps within overlapping windows, checking distances.
    * Records temporal conflicts if distance < safety_buffer_distance.
    * Example Call:
    * conflicts = deconfliction.check_temporal_conflict(my_mission, [sched1, sched2])

    '''
    def check_temporal_conflict(self, primary_mission: Mission, schedule_list: List[Mission]) -> List[dict]:
        """Check for spatiotemporal conflicts in 3D."""
        self.detected_conflicts_list = []
        # Variable Name: start_time_overlap: The earliest start time among overlapping missions, expected range: 0.0 to infinity.
        # Variable Name: end_time_overlap: The latest end time among overlapping missions, expected range: 0.0 to infinity.
        start_time_overlap = max(primary_mission.t_start, min(s.t_start for s in schedule_list))
        end_time_overlap = min(primary_mission.t_end, max(s.t_end for s in schedule_list))

        if start_time_overlap >= end_time_overlap:
            return self.detected_conflicts_list

        # Variable Name: time_step_increment: Time step increment for interpolation, expected range: 0.0 to 1.0.
        time_step_increment = 0.1
        for current_time in np.arange(start_time_overlap, end_time_overlap, time_step_increment):
            position_one = self.interpolate_position(primary_mission, current_time)
            if position_one is None:
                continue
            for flight_index, current_schedule in enumerate(schedule_list):
                position_two = self.interpolate_position(current_schedule, current_time)
                if position_two is None:
                    continue
                distance = np.linalg.norm(np.array(position_one) - np.array(position_two))
                if distance < self.safety_buffer_distance:
                    self.detected_conflicts_list.append({
                        "type": "temporal",
                        "time": current_time,
                        "location": position_one,
                        "distance": distance,
                        "flight_id": flight_index
                    })
        return self.detected_conflicts_list

    '''
    * Function Name: check_mission
    * Input:
    * primary_mission: The primary Mission object to check (Mission object).
    * schedule_list: List of Mission objects representing other drone schedules (List[Mission]).
    * Output:
    * Tuple[str, List[dict]]: A tuple containing the status ("clear" or "conflict detected") and list of conflicts.
    * Logic: Validates the mission, then checks for both spatial and temporal conflicts, combining results.
    * Returns status based on presence of conflicts.
    * Example Call:
    * status, conflicts = deconfliction.check_mission(my_mission, [sched1, sched2])

    '''

    def check_mission(self, primary_mission: Mission, schedule_list: List[Mission]) -> Tuple[str, List[dict]]:
        """Check the mission for conflicts."""
        if not primary_mission.validate():
            return "invalid mission", []

        spatial_conflicts = self.check_spatial_conflict(primary_mission, schedule_list)
        temporal_conflicts = self.check_temporal_conflict(primary_mission, schedule_list)

        all_conflicts = spatial_conflicts + temporal_conflicts
        mission_status = "clear" if not all_conflicts else "conflict detected"
        return mission_status, all_conflicts