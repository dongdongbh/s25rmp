from typing import Tuple, List, Iterator
import pybullet as pb
import numpy as np
from simulation import SimulationEnvironment

# Type aliases for clarity
Config = Tuple[float, ...]
Path = List[Config]

# STREAM samplers for PDDLStream

def grasp_stream(block_id: str) -> Iterator[Tuple[Tuple[float, ...],]]:
    """Generate candidate grasp poses (x,y,z,roll,pitch,yaw) for a given block."""
    # Example: yield a fixed top-down grasp
    env = SimulationEnvironment(show=False)
    loc, quat = env.get_block_pose(block_id)
    # Simple grasp at block center with fixed orientation
    grasp = (loc[0], loc[1], loc[2] + 0.02, 0.0, 0.0, 0.0)
    yield (grasp,)
    env.close()


def ik_stream(block_id: str,
              grasp_pose: Tuple[float, float, float, float, float, float]
) -> Iterator[Tuple[Config,]]:
    """Yield joint configurations that achieve the given grasp on block."""
    env = SimulationEnvironment(show=False)
    # Replace with your HW2 IK solver. Here we use PyBullet's IK as placeholder.
    pos = grasp_pose[:3]
    orn = pb.getQuaternionFromEuler(grasp_pose[3:])
    joint_angles = pb.calculateInverseKinematics(env.robot_id,
                                                 env.joint_index['m6'],
                                                 pos, orn)
    config = tuple(joint_angles)
    yield (config,)
    env.close()


def cfree_config(config: Config) -> bool:
    """Return True if the robot at `config` is collision-free."""
    env = SimulationEnvironment(show=False)
    # Apply config
    for i, angle in enumerate(config):
        pb.resetJointState(env.robot_id, i, angle)
    # Check for self-collisions and collisions with blocks
    contacts = pb.getContactPoints(env.robot_id)
    env.close()
    return len(contacts) == 0

def traj_free(
    path: List[Tuple[float, ...]],
    block_id: str
) -> Iterator[tuple]:
    """
    Test that carrying `block_id` along the joint‐space path incurs no collision.
    Yields exactly one empty tuple if the entire path is safe; otherwise yields nothing.
    """
    # You need access to your collision checker for the held block.
    # Here we use `cfree_config` to check the robot+block at each config.
    for q in path:
        if not cfree_config(q, carried_block=block_id):
            # collision detected; abort
            return
    # if we reach here, no collisions
    yield ()



# Placeholder STREAMS and ACTIONS for planner integration
STREAMS = {}
ACTIONS = []
