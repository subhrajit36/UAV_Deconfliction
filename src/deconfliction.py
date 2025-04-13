from typing import List, Tuple, Optional
from utils import Waypoint, Mission
import numpy as np

class DeconflictionSystem:
    def __init__(self, safety_buffer: float = 5.0):
        self.safety_buffer = safety_buffer
        self.conflicts = []

    def segment_distance(self, p1: Waypoint, p2: Waypoint, q1: Waypoint, q2: Waypoint) -> float:
        """Calculate the closest distance between two line segments in 3D."""
        def dot(u, v): return u[0] * v[0] + u[1] * v[1] + u[2] * v[2]
        def norm(v): return np.sqrt(dot(v, v))

        A = np.array(p1.to_tuple())
        B = np.array(p2.to_tuple())
        C = np.array(q1.to_tuple())
        D = np.array(q2.to_tuple())

        AB = B - A
        CD = D - C
        AC = C - A

        denom = norm(AB) * norm(CD)
        if denom == 0:  # One or both segments are points
            return np.min([np.linalg.norm(A - C), np.linalg.norm(A - D),
                           np.linalg.norm(B - C), np.linalg.norm(B - D)])

        s = np.clip(dot(AC, AB) / (norm(AB) ** 2), 0, 1)
        t = np.clip(-dot(AC, CD) / (norm(CD) ** 2), 0, 1)

        closest_point_AB = A + s * AB
        closest_point_CD = C + t * CD

        return np.linalg.norm(closest_point_AB - closest_point_CD)

    def interpolate_position(self, mission: Mission, t: float) -> Optional[Tuple[float, float, float]]:
        """Interpolate the drone's position at time t in 3D."""
        if t < mission.t_start or t > mission.t_end or not mission.validate():
            return None

        total_time = mission.t_end - mission.t_start
        segment_duration = total_time / (len(mission.waypoints) - 1)

        segment_index = int((t - mission.t_start) / segment_duration)
        if segment_index >= len(mission.waypoints) - 1:
            return mission.waypoints[-1].to_tuple()

        t_segment = (t - mission.t_start) % segment_duration
        alpha = t_segment / segment_duration

        p1 = mission.waypoints[segment_index]
        p2 = mission.waypoints[segment_index + 1]

        x = p1.x + alpha * (p2.x - p1.x)
        y = p1.y + alpha * (p2.y - p1.y)
        z = p1.z + alpha * (p2.z - p1.z)
        
        pos = (x, y, z)
        print(f"Debug: t={t}, Interpolated position: {pos}")
        return pos
    
    def check_spatial_conflict(self, mission: Mission, schedules: List[Mission]) -> List[dict]:
        """Check for spatial conflicts in 3D."""
        self.conflicts = []
        for i, sched in enumerate(schedules):
            for j in range(len(mission.waypoints) - 1):
                p1, p2 = mission.waypoints[j], mission.waypoints[j + 1]
                for k in range(len(sched.waypoints) - 1):
                    q1, q2 = sched.waypoints[k], sched.waypoints[k + 1]
                    dist = self.segment_distance(p1, p2, q1, q2)
                    if dist < self.safety_buffer:
                        self.conflicts.append({
                            "type": "spatial",
                            "location": (p1.to_tuple(), p2.to_tuple()),
                            "distance": dist,
                            "flight_id": i
                        })
        return self.conflicts

    def check_temporal_conflict(self, mission: Mission, schedules: List[Mission]) -> List[dict]:
        """Check for spatiotemporal conflicts in 3D."""
        self.conflicts = []
        t_start = max(mission.t_start, min(s.t_start for s in schedules))
        t_end = min(mission.t_end, max(s.t_end for s in schedules))

        if t_start >= t_end:
            return self.conflicts

        dt = 0.1
        for t in np.arange(t_start, t_end, dt):
            pos1 = self.interpolate_position(mission, t)
            if pos1 is None:
                continue
            for i, sched in enumerate(schedules):
                pos2 = self.interpolate_position(sched, t)
                if pos2 is None:
                    continue
                dist = np.linalg.norm(np.array(pos1) - np.array(pos2))
                if dist < self.safety_buffer:
                    self.conflicts.append({
                        "type": "temporal",
                        "time": t,
                        "location": pos1,
                        "distance": dist,
                        "flight_id": i
                    })
        return self.conflicts

    def check_mission(self, mission: Mission, schedules: List[Mission]) -> Tuple[str, List[dict]]:
        """Check the mission for conflicts."""
        if not mission.validate():
            return "invalid mission", []

        spatial_conflicts = self.check_spatial_conflict(mission, schedules)
        temporal_conflicts = self.check_temporal_conflict(mission, schedules)

        all_conflicts = spatial_conflicts + temporal_conflicts
        status = "clear" if not all_conflicts else "conflict detected"
        return status, all_conflicts