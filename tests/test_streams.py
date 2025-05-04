# import pytest
# from typing import Tuple
# from planners.pddlstream_interface import ik_stream, cfree_config, traj_free

# # a dummy block pose and grasp for testing
# DUMMY_BLOCK = 'b0'
# DUMMY_GRASP = (0.0, 0.0, 0.05, 0, 0, 0)  # x,y,z,roll,pitch,yaw

# def test_ik_stream_and_collision(tmp_path):
#     # assume SimulationEnvironment has been imported and used to create block b0
#     from simulation import SimulationEnvironment
#     env = SimulationEnvironment(show=False)
#     # add a test block at known pose
#     label = env._add_block((.1, .0, .05), (1,0,0,0))
#     assert label == DUMMY_BLOCK  # for a fresh env

#     # IK: should yield at least one configuration
#     configs = list(ik_stream(label, DUMMY_GRASP))
#     assert len(configs) > 0
#     q = configs[0]
#     # collision‐check the returned config
#     assert cfree_config(q), "IK solution should be collision‐free"

#     # traj_free: a trivial path of one step should be free
#     assert list(traj_free([q], label)) == [()]

#     env.close()

import pytest
from typing import Tuple
from planners.pddlstream_interface import ik_stream, cfree_config, traj_free
from simulation import SimulationEnvironment

# dummy test constants
DUMMY_BLOCK = 'b0'
DUMMY_GRASP = (0.1, 0.0, 0.05, 0, 0, 0)  # x,y,z,roll,pitch,yaw

def test_ik_stream_and_collision():
    env = SimulationEnvironment(show=False)

    # Step 1: add test block with label 'b0'
    label = env._add_block((0.1, 0.0, 0.05), (1, 0, 0, 0))
    assert label == DUMMY_BLOCK

    # Step 2: get current world state
    world_state = {
        b_id: env.get_block_pose(b_id)
        for b_id in env.block_id if b_id != "base"
    }

    # Step 3: test ik_stream
    configs = list(ik_stream(world_state, label, DUMMY_GRASP))
    assert len(configs) > 0, "IK stream should yield at least one config"
    q = configs[0]

    # Step 4: test collision-free config
    assert cfree_config(q, world_state), "IK config should be collision-free"

    # Step 5: test traj_free
    assert list(traj_free([q], label, world_state)) == [()], "Trivial trajectory should be free"

    env.close()
