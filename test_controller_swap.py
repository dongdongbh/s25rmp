#!/usr/bin/env python3

import sys
import numpy as np
from simulation import SimulationEnvironment
from submission import Controller
from evaluation import sample_trial, evaluate

def test_two_block_swap():
    """
    Create a 2‑block tower with one swap, call Controller.run (which will invoke solve),
    and then check that both blocks are placed at their goal locations.
    """
    print("Running test_two_block_swap…", end=" ", flush=True)
    # 1) Sample an evaluation trial with 2 blocks and 1 swap
    env, goal_poses = sample_trial(num_blocks=2, num_swaps=1, show=False)

    try:
        # 2) Run your real planner‐controller end‐to‐end
        Controller().run(env, goal_poses)

        # 3) Evaluate the final state
        accuracy, loc_errors, rot_errors = evaluate(env, goal_poses)

        # We expect both blocks to be within tolerance of their swapped goals
        if accuracy == 1.0:
            print("PASSED")
        else:
            print("FAILED")
            print(f"  Expected accuracy 1.0, got {accuracy:.3f}")
            print(f"  loc errors: {loc_errors}")
            print(f"  rot errors: {rot_errors}")
            sys.exit(1)

    except Exception as e:
        print("FAILED")
        print("  Error during planning/execution:", e)
        sys.exit(1)

    finally:
        env.close()

def main():
    test_two_block_swap()
    print("🎉 Planner‐controller swap test passed!")

if __name__ == "__main__":
    main()

