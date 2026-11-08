from typing import Tuple, Dict, List
import numpy as np
from dataclasses import dataclass, field


@dataclass
class SnapShot:
    image: str
    depth: str
    angle: float
    color: Tuple[float, float, float]
    full_obj_list: Dict[int, float] = field(default_factory=dict)
    cluster: List[int] = field(default_factory=list)
    pos: np.ndarray = None
    queried: bool = False
    pose: np.ndarray = None

    def __eq__(self, other):
        raise NotImplementedError("Cannot compare SnapShot objects.")
