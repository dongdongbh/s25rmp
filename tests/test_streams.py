import pytest
from typing import Tuple
from planners.pddlstream_interface import ik_stream, cfree_config, traj_free

# a dummy block pose and grasp for testing
DUMMY_BLOCK = 'b0'
DUMMY_GRASP = (0.0, 0.0, 0.05, 0, 0, 0)  # x,y,z,roll,pitch,yaw

def test_ik_stream_and_collision(tmp_path):
    # assume SimulationEnvironment has been imported and used to create block b0
    from simulation import SimulationEnvironment
    env = SimulationEnvironment(show=False)
    # add a test block at known pose
    label = env._add_block((.1, .0, .05), (1,0,0,0))
    assert label == DUMMY_BLOCK  # for a fresh env

    # IK: should yield at least one configuration
    configs = list(ik_stream(label, DUMMY_GRASP))
    assert len(configs) > 0
    q = configs[0]
    # collision‐check the returned config
    assert cfree_config(q), "IK solution should be collision‐free"

    # traj_free: a trivial path of one step should be free
    assert list(traj_free([q], label)) == [()]

    env.close()

