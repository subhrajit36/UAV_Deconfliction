# UAV Deconfliction System

This repository contains a Python-based solution for the FlytBase Robotics Assignment 2025, implementing a strategic deconfliction system for UAVs in shared airspace. The system checks for spatial and temporal conflicts between a primary drone mission and simulated flight schedules, with visualization of trajectories.

## Overview
- **Objective**: Design a system to verify the safety of a drone's waypoint mission by detecting conflicts in 3D space and time.
- **Features**:
  - Spatial conflict detection with a 0.5-unit safety buffer.
  - Temporal conflict checking within mission windows.
  - Visualization of drone trajectories.
  - Iterative waypoint refinement for conflict resolution.

## Repository Structure
- `__init__.py`: Package initialization file.
- `deconfliction.py`: Core logic for spatial and temporal conflict checks.
- `utils.py`: Utility functions (e.g., math calculations, helpers).
- `visualization.py`: Code for plotting drone trajectories and generating debug output.
- `full_output.txt`: Debug file with conflict analysis data.
- `README.md`: This file with setup and usage instructions.

## Setup
### Prerequisites
- Python 3.7 or higher.
- Required libraries: `matplotlib` and `numpy`.

### Installation
1. Clone the repository:
   ```bash
   git clone https://github.com/subhrajit36/UAV_Deconfliction.git
   cd UAV_Deconfliction