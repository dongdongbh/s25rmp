# test_submission_run.py

import pprint
import numpy as np
from pddlstream.language.constants import And

# --- MONKEY‑PATCH: disable optimistic combination logic in Instantiator ---
from pddlstream.algorithms.instantiation import Instantiator

from pddlstream.language.stream import StreamInstance


# no-op out the two combination methods
Instantiator._add_combinations_relation = lambda self, stream, atoms: None
Instantiator._add_combinations          = lambda self, stream, atoms: None
StreamInstance.next_optimistic = lambda self: iter(())

# --- end patch ---

from pddlstream.algorithms.meta import solve

from submission import Controller
from simulation import SimulationEnvironment
from planners.pddlstream_interface import STREAMS as stream_info


def main():
    # 1) Spin up a DIRECT‐mode sim, add 2 blocks on two different bases:
    env = SimulationEnvironment(show=False)
    # b0 is the static platform, add two dynamic cubes:
    b1 = env._add_block((0.2, 0.0, 0.05), (0, 0, 0, 1), mass=1, side=0.01905)
    b2 = env._add_block((-0.2, 0.0, 0.05), (0, 0, 0, 1), mass=1, side=0.01905)
    env.settle(0.5)

    ctrl = Controller()

    # 2) Ask Controller to infer symbolic map
    symbolic_map = ctrl._infer_symbolic_map_from_env(env)
    print("Symbolic map:", symbolic_map)

    from planners.pddlstream_interface import extract_physical_world
    world = extract_physical_world(env)

    # 3) Pick a simple “goal” that just swaps those two blocks’ locations:
    #    i.e. send b1 to b2’s loc, and b2 to b1’s loc
    goal_poses = {
        b1: (symbolic_map[b2], None),
        b2: (symbolic_map[b1], None),
    }

     # 4) Build the PDDLProblem (now passing in `world`)
    pddl_prob = ctrl._make_pddlstream_problem(symbolic_map,
                                              goal_poses,
                                              world)

    # 5) Solve it! Make sure to pass in your STREAMS‐info dictionary.
    #    If your actions have no costs, you may want unit_costs=True as well.
    print("\nCalling solver...")
    solutions = solve(
        pddl_prob,
        algorithm='adaptive',
        planner='ff-astar',
        stream_info=stream_info,
        unit_costs=True,
        max_planner_time=10,
        verbose=True,
        debug=True,
    )

    solution = solutions[0]

    print("\n=== SOLUTION SKELETON ===")
    pprint.pprint(solution)

    env.close()


if __name__ == "__main__":
    main()

