# test_pddlstream_interface.py

import sys, os, pprint, numpy as np
# Ensure project root is on PYTHONPATH
sys.path.append(os.getcwd())

from simulation import SimulationEnvironment
from planners.pddlstream_interface import (
    extract_physical_world,
    ik_stream,
    cfree_config,
    motion_stream,
    traj_free,
    gen_loc_stream,
)

def main():
    # Create a DIRECT-mode sim and add two test blocks
    env = SimulationEnvironment(show=False)
    CUBE_SIDE = 0.01905
    base_z = -0.5 * CUBE_SIDE
    quat = (1,0,0,0)
    # Block near baseA
    b1 = env._add_block((0.0, -0.14, base_z + CUBE_SIDE), quat, mass=2, side=CUBE_SIDE)
    # Block near baseB
    b2 = env._add_block((0.0,  0.14, base_z + 2*CUBE_SIDE), quat, mass=2, side=CUBE_SIDE)
    env.settle(1.0)

    # 1) Snapshot world
    world = extract_physical_world(env)
    print("WorldState:")
    pprint.pprint(world)

    # 2) Test ik_stream
    print("\n[ik_stream]")
    ik_out = list(ik_stream(world, b1, "baseA_loc0", (0.5, 0.2, 0.1, 0.0, 1.57, 0.0)))
    print("  yields:", ik_out)

    # 3) Test cfree_config
    if ik_out:
        q = ik_out[0][0]
        print("\n[cfree_config]")
        print("  cfree_config(world, q) ->", cfree_config(world, q))

    # 4) Test motion_stream
    print("\n[motion_stream]")
    mot_out = list(motion_stream(world, q, q))
    print("  yields:", mot_out)

    # 5) Test traj_free
    print("\n[traj_free]")
    # use 'nil' as the held-block placeholder
    traj_ok = list(traj_free(world, mot_out[0][0], 'nil'))
    print("  yields:", traj_ok)

    # 6) Test gen_loc_stream
    print("\n[gen_loc_stream]")
    symbolic = {b1: "baseA_loc0", b2: "baseB_loc1"}
    print("  baseA ->", list(gen_loc_stream(symbolic, "baseA")))
    print("  baseB ->", list(gen_loc_stream(symbolic, "baseB")))

    env.close()

if __name__ == "__main__":
    main()

