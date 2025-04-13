import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D
import numpy as np
from typing import List
from utils import Mission, Waypoint
from deconfliction import DeconflictionSystem

def plot_2d_trajectories(mission: Mission, schedules: List[Mission], conflicts: List[dict], output_path: str):
    """Plot drone trajectories in 2D for reference."""
    plt.figure(figsize=(10, 8))
    x = [wp.x for wp in mission.waypoints]
    y = [wp.y for wp in mission.waypoints]
    plt.plot(x, y, 'bo-', label='Primary Mission', linewidth=2)
    for i, sched in enumerate(schedules):
        x = [wp.x for wp in sched.waypoints]
        y = [wp.y for wp in sched.waypoints]
        plt.plot(x, y, 'ro--', label=f'Schedule {i}', alpha=0.5)
    for conflict in conflicts:
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
    plt.savefig(output_path)
    plt.close()

def plot_3d_trajectories(mission: Mission, schedules: List[Mission], conflicts: List[dict], output_path: str):
    """Plot drone trajectories and conflicts in 3D."""
    fig = plt.figure(figsize=(10, 8))
    ax = fig.add_subplot(111, projection='3d')

    # Plot primary mission
    ax.plot(
        [wp.x for wp in mission.waypoints],
        [wp.y for wp in mission.waypoints],
        [wp.z for wp in mission.waypoints],
        'bo-', label='Primary Mission', linewidth=2, markersize=8
    )

    # Plot schedules
    for i, sched in enumerate(schedules):
        ax.plot(
            [wp.x for wp in sched.waypoints],
            [wp.y for wp in sched.waypoints],
            [wp.z for wp in sched.waypoints],
            'ro--', label=f'Schedule {i}', alpha=0.5, markersize=6
        )

    # Highlight conflicts
    for conflict in conflicts:
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
    plt.savefig(output_path)
    plt.close()

def simulate_scenarios():
    """Simulate conflict-free and conflict-present scenarios in 3D."""
    system = DeconflictionSystem(safety_buffer=5.0)

    # Scenario 1: Conflict-free
    mission1 = Mission(
        waypoints=[
            Waypoint(0, 0, 100),
            Waypoint(10, 0, 110),
            Waypoint(20, 0, 120)
        ],
        t_start=0,
        t_end=60
    )
    schedules = [
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
    status, conflicts = system.check_mission(mission1, schedules)
    print(f"Scenario 1: {status}, Conflicts: {conflicts}")
    plot_2d_trajectories(mission1, schedules, conflicts, "outputs/conflict_free_2d.png")
    plot_3d_trajectories(mission1, schedules, conflicts, "outputs/conflict_free_3d.png")

    # Scenario 2: Conflict-present with temporal overlap
    mission2 = Mission(
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
    schedules = [
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
    status, conflicts = system.check_mission(mission2, schedules)
    print(f"Scenario 2: {status}, Conflicts: {conflicts}")
    plot_2d_trajectories(mission2, schedules, conflicts, "outputs/conflict_present_2d.png")
    plot_3d_trajectories(mission2, schedules, conflicts, "outputs/conflict_present_3d.png")

if __name__ == "__main__":
    simulate_scenarios()