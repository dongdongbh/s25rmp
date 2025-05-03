from typing import List, Optional
import numpy as np
from planners.pddlstream_interface import cfree_config
import math
from ompl import base as ob
from ompl import geometric as og

# Type aliases
Config = List[float]
Path = List[Config]

def ompl_state_to_config(state: ob.State, space: ob.StateSpace) -> Config:
    """Transfer OMPL State to Config (tuple)"""
    if isinstance(space, ob.RealVectorStateSpace):
        return tuple(state[i] for i in range(space.getDimension()))
    else:
        try:
            real_state = space.allocState().real()
            space.copyState(real_state, state)
            return tuple(real_state[i] for i in range(space.getDimension()))
        except AttributeError:
            raise TypeError(f"Cannot convert state of type {type(space)} to Config.")

def ompl_path_to_list_config(path: og.PathGeometric, si: ob.SpaceInformation) -> Path:
    space = si.getStateSpace()
    return [list(ompl_state_to_config(path.getState(i), space)) for i in range(path.getStateCount())]

space_bounds = [(-2*math.pi, 2*math.pi)] * 6


def plan_rrt_connect(q1: Config,
                     q2: Config,
                     seed: Optional[int] = None,
                     timeout: float = 1.0,
                     goal_threshold: float = 1e-3,
                     simplify_solution: bool = False
                     ) -> Optional[Path]:
    """
    A very simple “free-space RRT-Connect” stub:
    We just linearly interpolate and check collisions.
    """
    # num_steps = 20
    # weights = np.linspace(0, 1, num_steps)
    # path: Path = []
    # for w in weights:
    #     q = tuple((1 - w)*a + w*b for a, b in zip(q1, q2))
    #     if not cfree_config(q):
    #         return None
    #     path.append(q)
    # Setting the seed
    if seed is not None:
        ob.RNG.setSeed(seed)

    dim = len(q1)
    if dim != len(q2):
        raise ValueError("The dimensions of the two configurations do not match.")

    space = ob.RealVectorStateSpace(dim)
    bounds = ob.RealVectorBounds(dim)
    for i in range(dim):
        bounds.setLow(i, space_bounds[i][0])
        bounds.setHigh(i, space_bounds[i][1])
    space.setBounds(bounds)
    ss = og.SimpleSetup(space)

    def isStateValid(state: ob.State) -> bool:
        config = ompl_state_to_config(state, space)
        return cfree_config(config)

    ss.setStateValidityChecker(ob.StateValidityCheckerFn(isStateValid))

    # Setting start and goal
    start = ob.State(space)
    goal = ob.State(space)
    for i in range(dim):
        start[i] = q1[i]
        goal[i] = q2[i]
    ss.setStartAndGoalStates(start, goal, goal_threshold)

    # Set RRTConnect planner
    planner = og.RRTConnect(ss.getSpaceInformation())
    ss.setPlanner(planner)

    # Solve the path
    solved = ss.solve(timeout)
    path: Optional[Path] = None

    if solved:
        if ss.haveSolutionPath():
            if simplify_solution:
                ss.simplifySolution(5.0)

            path_geometric = ss.getSolutionPath()
            path = ompl_path_to_list_config(path_geometric, ss.getSpaceInformation())
            print(f"The path includes {len(path)} points")
        else:
            print("Warning: The problem is solved but no solution path is found.")
    else:
        print(f"No solution found after {timeout} seconds.")

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
    timeout = 0.5
    num_steps = 20
    weights = np.linspace(0, 1, num_steps)
    straight = [tuple((1 - w)*a + w*b for a, b in zip(q1, q2))
                for w in weights]
    if all(cfree_config(q) for q in straight):
        yield (straight,)
        return

    # 2) Fallback
    max_rrt_attempts = 5
    for seed in range(max_rrt_attempts):
        rrt_path = plan_rrt_connect(list(q1), list(q2), seed=seed, timeout=timeout)
        if not rrt_path:
            continue
        yield (rrt_path,)

    print(f"All {max_rrt_attempts} RRT attempts failed for {q1} -> {q2}")
