'''
Author List: Subhrajit Mahana
Filename: visualization.py
Theme: UAV Deconfliction - FlytBase Robotics Assignment 2025
Functions: plot_2d_trajectories, plot_3d_trajectories, simulate_scenarios
Global Variables: None
'''

import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D
import numpy as np
from typing import List
from utils import Mission, Waypoint
from deconfliction import DeconflictionSystem

'''
* Function Name: plot_2d_trajectories
* Input:
* primary_mission: The primary Mission object to plot (Mission object).
* schedule_list: List of Mission objects representing other drone schedules (List[Mission]).
* conflict_data: List of dictionaries containing conflict details (List[dict]).
* output_file_path: String path to save the 2D plot (str).
* Output:
* None: Saves the plot to the specified file path.
* Logic: Plots 2D trajectories of the primary mission and schedules, marking spatial and temporal conflicts.
* Uses Matplotlib to create the visualization and saves it as a PNG file.
* Example Call:
* plot_2d_trajectories(my_mission, [sched1, sched2], conflicts, "output.png")
'''

def plot_2d_trajectories(primary_mission: Mission, schedule_list: List[Mission], conflict_data: List[dict], output_file_path: str):
    """Plot drone trajectories in 2D for reference."""
    plt.figure(figsize=(10, 8))

    # Variable Name: x_coordinates_mission: List of x coordinates from the primary mission waypoints, expected range: any float values.
    # Variable Name: y_coordinates_mission: List of y coordinates from the primary mission waypoints, expected range: any float values.
    x_coordinates_mission = [wp.x for wp in primary_mission.waypoints]
    y_coordinates_mission = [wp.y for wp in primary_mission.waypoints]
    plt.plot(x_coordinates_mission, y_coordinates_mission, 'bo-', label='Primary Mission', linewidth=2)
    for i, current_schedule in enumerate(schedule_list):
        x_coordinates_schedule = [wp.x for wp in current_schedule.waypoints]
        y_coordinates_schedule = [wp.y for wp in current_schedule.waypoints]
        plt.plot(x_coordinates_schedule, y_coordinates_schedule, 'ro--', label=f'Schedule {i}', alpha=0.5)
    for conflict in conflict_data:
        if conflict['type'] == 'spatial':
            (x1, y1, _), (x2, y2, _) = conflict['location']
            plt.plot([x1, x2], [y1, y2], 'gx-', label='Spatial Conflict', markersize=10)
        elif conflict['type'] == 'temporal':
            x, y, _ = conflict['location']
            plt.scatter(x, y, c='purple', s=100, label='Temporal Conflict', marker='x')
    plt.xlabel('X Coordinate')
    plt.ylabel('Y Coordinate')
    plt.title('2D Drone Trajectories and Conflicts')
    plt.legend()
    plt.grid(True)
    # Saves the 2D plot to the specified output file path and closes the figure to free memory.
    plt.savefig(output_file_path)
    plt.close()

'''
* Function Name: plot_3d_trajectories
* Input:
* primary_mission: The primary Mission object to plot (Mission object).
* schedule_list: List of Mission objects representing other drone schedules (List[Mission]).
* conflict_data: List of dictionaries containing conflict details (List[dict]).
* output_file_path: String path to save the 3D plot (str).
* Output:
* None: Saves the plot to the specified file path.
* Logic: Plots 3D trajectories of the primary mission and schedules, marking spatial and temporal conflicts.
* Uses Matplotlib with 3D projection and saves it as a PNG file.
* Example Call:
* plot_3d_trajectories(my_mission, [sched1, sched2], conflicts, "output.png")
'''

def plot_3d_trajectories(primary_mission: Mission, schedule_list: List[Mission], conflict_data: List[dict], output_file_path: str):
    """Plot drone trajectories and conflicts in 3D."""
    fig = plt.figure(figsize=(10, 8))
    ax = fig.add_subplot(111, projection='3d')

    # Plot primary mission
    ax.plot(
        [wp.x for wp in primary_mission.waypoints],
        [wp.y for wp in primary_mission.waypoints],
        [wp.z for wp in primary_mission.waypoints],
        'bo-', label='Primary Mission', linewidth=2, markersize=8
    )

    # Plot schedules
    for i, current_schedule in enumerate(schedule_list):
        ax.plot(
            [wp.x for wp in current_schedule.waypoints],
            [wp.y for wp in current_schedule.waypoints],
            [wp.z for wp in current_schedule.waypoints],
            'ro--', label=f'Schedule {i}', alpha=0.5, markersize=6
        )

    # Highlight conflicts
    for conflict in conflict_data:
        if conflict['type'] == 'spatial':
            (x1, y1, z1), (x2, y2, z2) = conflict['location']
            ax.plot([x1, x2], [y1, y2], [z1, z2], 'gx-', label='Spatial Conflict', markersize=10)
        elif conflict['type'] == 'temporal':
            x, y, z = conflict['location']
            ax.scatter([x], [y], [z], c='purple', s=100, label='Temporal Conflict', marker='x')

    ax.set_xlabel('X Coordinate')
    ax.set_ylabel('Y Coordinate')
    ax.set_zlabel('Z Coordinate (Altitude)')
    ax.set_title('3D Drone Trajectories and Conflicts')
    ax.legend()
    ax.grid(True)

    # Saves the 3D plot to the specified output file path and closes the figure to free memory.
    plt.savefig(output_file_path)
    plt.close()

'''
* Function Name: simulate_scenarios
* Input:
* None: Uses hardcoded mission and schedule data for simulation.
* Output:
* None: Prints simulation results and generates plots.
* Logic: Creates two scenarios (conflict-free and conflict-present) using DeconflictionSystem.
* Checks for conflicts and plots 2D and 3D trajectories for each scenario.
* Example Call:
* simulate_scenarios()
'''

def simulate_scenarios():
    """Simulate conflict-free and conflict-present scenarios in 3D."""
    deconfliction_system = DeconflictionSystem(safety_buffer=5.0)

    # Scenario 1: Conflict-free
    conflict_free_mission = Mission(
        waypoints=[
            Waypoint(0, 0, 100),
            Waypoint(10, 0, 110),
            Waypoint(20, 0, 120)
        ],
        t_start=0,
        t_end=60
    )
    schedule_list = [
        Mission(
            waypoints=[
                Waypoint(0, 10, 130),
                Waypoint(10, 10, 140),
                Waypoint(20, 10, 150)
            ],
            t_start=0,
            t_end=60
        )
    ]
    status, conflicts = deconfliction_system.check_mission(conflict_free_mission, schedule_list)
    print(f"Scenario 1: {status}, Conflicts: {conflicts}")
    plot_2d_trajectories(conflict_free_mission, schedule_list, conflicts, "outputs/conflict_free_2d.png")
    plot_3d_trajectories(conflict_free_mission, schedule_list, conflicts, "outputs/conflict_free_3d.png")

    # Scenario 2: Conflict-present with temporal overlap
    conflict_present_mission = Mission(
        waypoints=[
            Waypoint(0, 0, 100),          # t=0
            Waypoint(5, 5, 105),          # t=15
            Waypoint(10, 10, 110),        # t=30
            Waypoint(10.1, 10.1, 110.1),  # t=45
            Waypoint(10.15, 10.15, 110.15), # t=50
            Waypoint(10.2, 10.2, 110.2),  # t=55
            Waypoint(10.25, 10.25, 110.25), # t=60
            Waypoint(20, 0, 120)          # t=60+
        ],
        t_start=0,
        t_end=60
    )
    schedule_list = [
        Mission(
            waypoints=[
                Waypoint(5, 5, 105),        # t=30
                Waypoint(10, 10, 110),      # t=45
                Waypoint(10.15, 10.15, 110.15), # t=50
                Waypoint(10.2, 10.2, 110.2), # t=55
                Waypoint(10.25, 10.25, 110.25), # t=60
                Waypoint(15, 5, 115)        # t=90
            ],
            t_start=30,
            t_end=90
        )
    ]
    # Check for conflicts
    status, conflicts = deconfliction_system.check_mission(conflict_present_mission, schedule_list)
    print(f"Scenario 2: {status}, Conflicts: {conflicts}")
    plot_2d_trajectories(conflict_present_mission, schedule_list, conflicts, "outputs/conflict_present_2d.png")
    plot_3d_trajectories(conflict_present_mission, schedule_list, conflicts, "outputs/conflict_present_3d.png")

if __name__ == "__main__":
    simulate_scenarios()
