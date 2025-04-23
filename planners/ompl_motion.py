from typing import List, Optional
import numpy as np
from planners.pddlstream_interface import cfree_config

# Type aliases
Config = List[float]
Path = List[Config]

def plan_rrt_connect(q1: Config,
                     q2: Config,
                     timeout: float = 1.0
) -> Optional[Path]:
    """
    A very simple “free-space RRT-Connect” stub:
    We just linearly interpolate and check collisions.
    """
    num_steps = 20
    weights = np.linspace(0, 1, num_steps)
    path: Path = []
    for w in weights:
        q = tuple((1 - w)*a + w*b for a, b in zip(q1, q2))
        if not cfree_config(q):
            return None
        path.append(q)
    return path

def motion_stream(q1: Config,
                  q2: Config
) -> List[tuple]:
    """
    Try straight-line interpolation first; if that collides, fall back
    to plan_rrt_connect (above).
    Yields exactly one collision-free path as an Iterator[(path,)].
    """
    # 1) Linear interpolation
    num_steps = 20
    weights = np.linspace(0, 1, num_steps)
    straight = [tuple((1 - w)*a + w*b for a, b in zip(q1, q2))
                for w in weights]
    if all(cfree_config(q) for q in straight):
        yield (straight,)
        return

    # 2) Fallback
    rrt_path = plan_rrt_connect(list(q1), list(q2), timeout)
    if rrt_path:
        yield (rrt_path,)

