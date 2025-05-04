# test_build_problem.py

import pprint
import sys, os
# Ensure project root is on PYTHONPATH
sys.path.append(os.getcwd())

from submission import Controller

from planners.pddlstream_interface import extract_physical_world
from simulation import SimulationEnvironment

def main():
    ctrl = Controller()

    # Fake symbolic map: two blocks on two different bases/levels
    symbolic_map = {
        'b1': 'baseA_loc0',
        'b2': 'baseB_loc0',
        'b3': 'baseB_loc1',
    }
    # Fake goal poses: only the location symbol is used
    goal_poses = {
        'b1': ('baseB_loc1', None),
        'b2': ('baseB_loc0', None),
        'b3': ('baseA_loc0', None),
    }

    env = SimulationEnvironment(show=False)
    world = extract_physical_world(env)
    env.close()

# now pass `world` into the problem builder
    pddl_prob = ctrl._make_pddlstream_problem(symbolic_map, goal_poses, world)

    # Unpack the 6‐tuple returned by PDDLProblem
    domain_pddl, constant_map, stream_pddl, stream_map, init_list, goal_formula = pddl_prob

    print("=== domain.pddl path ===")
    print(domain_pddl)

    print("\n=== stream.pddl path ===")
    print(stream_pddl)

    print("\n=== constant_map ===")
    pprint.pprint(constant_map)

    print("\n=== init list ===")
    for atom in init_list:
        print(atom)

    print("\n=== goal formula ===")
    print(goal_formula)

if __name__ == "__main__":
    main()

