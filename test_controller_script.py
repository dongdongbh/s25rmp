#!/usr/bin/env python3

import sys
from simulation import SimulationEnvironment
from submission import Controller
from evaluation import sample_trial, evaluate

def test_no_blocks():
    """
    Runs Controller on an environment with only the static platform (b0)
    and an empty goal map. Should exit without error.
    """
    print("Running test_no_blocks…", end=" ", flush=True)
    env = SimulationEnvironment(show=False)
    try:
        Controller().run(env, {})   # empty goal_poses
        print("PASSED")
    except Exception as e:
        print("FAILED")
        print("  Error:", e)
        sys.exit(1)
    finally:
        env.close()

def test_single_block_no_move():
    """
    Samples a one-block trial with num_swaps=0 (so goal=initial).
    Controller should recognize it's already solved and exit cleanly,
    and accuracy should be 1.0.
    """
    print("Running test_single_block_no_move…", end=" ", flush=True)
    env, goal_poses = sample_trial(num_blocks=1, num_swaps=0, show=False)
    try:
        Controller().run(env, goal_poses)
        accuracy, loc_errors, rot_errors = evaluate(env, goal_poses)
        if abs(accuracy - 1.0) < 1e-6:
            print("PASSED")
        else:
            print("FAILED")
            print(f"  Expected accuracy 1.0, got {accuracy:.3f}")
            sys.exit(1)
    except Exception as e:
        print("FAILED")
        print("  Error:", e)
        sys.exit(1)
    finally:
        env.close()

def main():
    test_no_blocks()
    test_single_block_no_move()
    print("🎉 All controller tests passed!")

if __name__ == "__main__":
    main()

