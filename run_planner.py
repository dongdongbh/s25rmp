# run_planner.py
from typing import Tuple, List, Dict


# then your other imports
from pddlstream.language.stream import StreamInfo
from pddlstream.language.function import FunctionInfo
from pddlstream.algorithms.meta import solve


# def plan_task(domain_path:str,
#               problem_path:str,
#               stream_map:Dict[str,StreamInfo],
#               action_list:List[FunctionInfo],
#               planner:str='ff-astar',
#               algorithm:str='adaptive'
# ) -> List[Tuple[str, Dict]]:
#     # If no streams/actions are yet defined, return a trivial pick→place plan
#     if not stream_map or not action_list:
#         return [('pick', {}), ('place', {})]
    
#     # Otherwise invoke PDDLStream properly
#     return solve(
#         domain=domain_path,
#         problem=problem_path,
#         streams=stream_map,
#         actions=action_list,
#         constants={'?w': world_state},  # <---  world state
#         planner=planner,
#         algorithm=algorithm,
#     )
def plan_task(domain_path: str,
              problem_path: str,
              stream_map: Dict[str, StreamInfo],
              action_list: List[FunctionInfo],
              env: SimulationEnvironment,
              planner: str = 'ff-astar',
              algorithm: str = 'adaptive'
) -> List[Tuple[str, Dict]]:
    """
    Plan using PDDLStream with the current block world state from env.
    """

    # Step 1: extract block poses as world state
    world_state = {
        block_id: env.get_block_pose(block_id)
        for block_id in env.block_id if block_id != "base"
    }

    # Step 2: call PDDLStream planner
    plan, _ = solve(
        domain=domain_path,
        problem=problem_path,
        streams=stream_map,
        actions=action_list,
        stream_info={},  # Optional unless you want custom sampling behavior
        constants={'?w': world_state},  # Bind ?w to world_state
        planner=planner,
        algorithm=algorithm,
    )

    # Step 3: handle output
    if plan is None:
        print("No plan found.")
        return []
    else:
        print("Plan found:")
        for step in plan:
            print("  ", step)
        return plan
