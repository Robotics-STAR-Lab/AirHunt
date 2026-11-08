import json
import numpy as np
from typing import List, Tuple, Dict


def load_airsim_navigation_data(data_path: str) -> Dict:
    with open(data_path, "r") as f:
        data = json.load(f)
    return data


def prepare_airsim_navigation_goals(
    scene_name: str, episode: Dict, all_navigation_goals: List[Dict]
) -> Tuple[List[str], List[List[Dict]]]:
    goals = episode.get("goals", [])
    all_subtask_goal_types = []
    all_subtask_goals = []
    for goal in goals:
        goal_type = "object"
        if goal.get("task_type") == "object_navigation":
            goal_type = "object"
        subtask_goal = [
            {
                "object_id": goal.get("object_id", ""),
                "object_category": goal.get("object_class", ""),
                "query": goal.get("query", ""),
                "position": [0.0, 0.0, -2.0],
                "view_points": [
                    {
                        "agent_state": {
                            "position": [0.0, 0.0, -2.0],
                            "rotation": [0.0, 0.0, 0.0, 1.0],
                        }
                    }
                ],
            }
        ]
        all_subtask_goal_types.append(goal_type)
        all_subtask_goals.append(subtask_goal)
    return (all_subtask_goal_types, all_subtask_goals)


def get_pts_angle_airsim(
    start_position: List[float], start_rotation: List[float]
) -> Tuple[np.ndarray, float]:
    pts = np.array(start_position, dtype=np.float32)
    if len(start_rotation) == 4:
        w, x, y, z = (
            start_rotation[3],
            start_rotation[0],
            start_rotation[1],
            start_rotation[2],
        )
        yaw = np.arctan2(2.0 * (w * z + x * y), 1.0 - 2.0 * (y * y + z * z))
    else:
        yaw = start_rotation[0] if len(start_rotation) > 0 else 0.0
    return (pts, yaw)


class AirsimDataLoader:

    def __init__(self, data_path: str):
        self.data_path = data_path
        self.data = load_airsim_navigation_data(data_path)

    def get_scene_list(self) -> List[str]:
        return list(self.data["scenes"].keys())

    def get_scene_data(self, scene_name: str) -> Dict:
        return self.data["scenes"].get(scene_name, {})

    def get_episodes(self, scene_name: str) -> List[Dict]:
        scene_data = self.get_scene_data(scene_name)
        return scene_data.get("episodes", [])

    def get_total_episodes(self) -> int:
        total = 0
        for scene_name in self.get_scene_list():
            total += len(self.get_episodes(scene_name))
        return total
