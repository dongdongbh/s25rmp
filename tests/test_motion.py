import pytest
from typing import Tuple, List
from planners.ompl_motion import plan_rrt_connect, motion_stream
from simulation import SimulationEnvironment

def test_motion_interpolation_and_rrt(tmp_path):
    env = SimulationEnvironment(show=False)
    # get two distinct but nearby configs
    start_dict = env.get_current_angles()
    start = tuple(start_dict.values())
    # make a small perturbation for target
    target = list(start)
    target[0] += 5  # bump joint 1 by 5°
    target = tuple(target)

    # motion_stream should yield at least one path
    paths = [p for (p,) in motion_stream(start, target)]
    assert paths, "motion_stream must yield at least one path"
    path = paths[0]
    # path should start and end at the correct configs
    assert path[0] == start
    assert path[-1] == target

    # and plan_rrt_connect should also succeed for free‐space
    rrt_path = plan_rrt_connect(list(start), list(target), timeout=0.5)
    assert rrt_path is not None
    assert rrt_path[0] == list(start)
    assert rrt_path[-1] == list(target)
    env.close()

def test_motion_interpolation_and_rrt2(tmp_path):
    env = SimulationEnvironment(show=False)
    # get two distinct but nearby configs
    start_dict = env.get_current_angles()
    start = tuple(start_dict.values())
    # make a small perturbation for target
    target = list(start)
    target[0] += 5  # bump joint 1 by 5°
    target[1] += 3.5
    target[2] -= 2.5
    target = tuple(target)

    # motion_stream should yield at least one path
    paths = [p for (p,) in motion_stream(start, target)]
    assert paths, "motion_stream must yield at least one path"
    path = paths[0]
    # path should start and end at the correct configs
    assert path[0] == start
    assert path[-1] == target

    # and plan_rrt_connect should also succeed for free‐space
    rrt_path = plan_rrt_connect(list(start), list(target), timeout=0.5)
    assert rrt_path is not None
    assert rrt_path[0] == list(start)
    assert rrt_path[-1] == list(target)
    env.close()

