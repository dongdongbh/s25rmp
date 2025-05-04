# test_validate_and_execute.py

import numpy as np
from submission import Controller
from simulation import SimulationEnvironment
from planners.pddlstream_interface import extract_physical_world

def main():
    ctrl = Controller()
    env = SimulationEnvironment(show=False)
    try:
        # Add one dynamic block
        label = env._add_block((0.1, 0.0, 0.05),
                               (0.0, 0.0, 0.0, 1.0),
                               mass=1, side=0.02)
        env.settle(0.5)

        # Get the full 8‑joint reading, then select only the 6 actuated joints
        full_q = tuple(env._get_position())  # length 8
        # env.joint_fixed[j] is True for fixed joints (the two finger tips).
        actuated_q = tuple(
            full_q[j]
            for j, fixed in enumerate(env.joint_fixed)
            if not fixed
        )  # now length 6

        # Build a trivial “move” plan using 6‑DOF configs
        move_plan = [
            ('move', {'?q1': actuated_q, '?q2': actuated_q, '?t': [actuated_q]})
        ]
        ok_move = all(
            ctrl._validate_action(extract_physical_world(env), a, p)
            for a, p in move_plan
        )
        print("Validate trivial move:", ok_move)

        # Build a trivial pick→place plan
        symbolic_map = ctrl._infer_symbolic_map_from_env(env)
        print("Symbolic map returned:", symbolic_map)
        if label not in symbolic_map:
            raise RuntimeError(f"Expected block `{label}` in symbolic_map but got: {symbolic_map}")
        pick_plan = [
            ('pick',  {'?b': label, '?l': symbolic_map[label], '?q': actuated_q}),
            ('place', {'?b': label, '?l': symbolic_map[label], '?q': actuated_q}),
        ]
        ok_pp = all(
            ctrl._validate_action(extract_physical_world(env), a, p)
            for a, p in pick_plan
        )
        print("Validate pick/place:", ok_pp)

        print(f'"actuated_q: {actuated_q}"')

        # Test validate_and_execute (now driving real 6‑DOF motions)
        combined = move_plan + pick_plan
        ok_full = ctrl._validate_and_execute(env, combined)
        print("_validate_and_execute(combined):", ok_full)

    finally:
        env.close()

if __name__ == "__main__":
    main()

