# test_infer_mapping.py

import sys
import os
import pprint

# Add project root to Python path so we can import modules
sys.path.append(os.path.dirname(__file__))

from evaluation import sample_trial
from submission import Controller
from simulation import SimulationEnvironment

def main():
    # 1) Create a sample trial (5 blocks, no swaps) without GUI
    env, goal_poses = sample_trial(num_blocks=20, num_swaps=0, show=False)

    # 2) Instantiate your Controller
    ctrl = Controller()

    # 3) Call the helper to infer the symbolic map
    symbolic_map = ctrl._infer_symbolic_map_from_env(env)

    # 4) Print out the mapping
    print("Inferred {block: location_symbol} map:")
    pprint.pprint(symbolic_map)

    # 5) Clean up the simulation
    env.close()

if __name__ == "__main__":
    main()

