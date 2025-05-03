# run_planner.py
from typing import Tuple, List, Dict


# then your other imports
from pddlstream.language.stream import StreamInfo
from pddlstream.language.function import FunctionInfo
from pddlstream.algorithms.meta import solve


def plan_task(domain_path:str,
              problem_path:str,
              stream_map:Dict[str,StreamInfo],
              action_list:List[FunctionInfo],
              planner:str='ff-astar',
              algorithm:str='adaptive'
) -> List[Tuple[str, Dict]]:
    # If no streams/actions are yet defined, return a trivial pick→place plan
    if not stream_map or not action_list:
        return [('pick', {}), ('place', {})]

    # Otherwise invoke PDDLStream properly
    return solve(
        domain=domain_path,
        problem=problem_path,
        streams=stream_map,
        actions=action_list,
        planner=planner,
        algorithm=algorithm,
    )
