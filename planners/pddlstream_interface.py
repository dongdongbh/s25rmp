# planners/pddlstream_interface.py

from typing import Tuple, Iterator, List, Dict
import pybullet as pb
from simulation import SimulationEnvironment, BASE_Z
from pddlstream.language.stream import StreamInfo
from pddlstream.language.generator import from_fn
import numpy as np

# Cube side length (m)
CUBE_SIDE = 0.01905

# Type aliases
WorldState    = Dict[str, Tuple[Tuple[float,float,float], Tuple[float,float,float,float]]]
SymbolicWorld = Dict[str, str]
Config        = Tuple[float, ...]
Path          = List[Config]




def _make_env(world: WorldState) -> SimulationEnvironment:
    """
    Build a fresh DIRECT‑mode SimulationEnvironment from scratch,
    then replay the contents of `world` (your block snapshot).
    """
    # 1) start a brand‑new direct‑mode sim
    env = SimulationEnvironment(show=False)

    # 2) remove any dynamic blocks (SimulationEnvironment.__init__
    #    always adds plane, robot, and a static 'b0' platform)
    for label in list(env.block_id):
        if label != 'b0':
            pb.removeBody(env.block_id[label], physicsClientId=env.client_id)
            del env.block_id[label]

    # 3) re‑spawn all dynamic blocks from the world snapshot
    for label, (pos, quat) in world.items():
        if label == 'b0':
            continue
        env._add_block(pos, quat, mass=2, side=CUBE_SIDE)

    return env



def extract_physical_world(env: SimulationEnvironment) -> WorldState:
    """
    Read all block poses from `env`.  On failure, fall back to the last‑good read.
    """
    state: WorldState = {}
    for b in env.block_id:
        pose = env.get_block_pose(b)
        state[b] = pose
    return state

def ik_stream(world: WorldState, b: str, l: str, grasp) -> Iterator[tuple]:
    """
    Yield collision-free IK solutions for grasp_pose using PyBullet.
    Tries multiple rest poses to find collision-free IK.
    """
    if len(grasp) < 6:
        raise ValueError(
            f"Invalid grasp input: Expected at least 6 elements (x, y, z, roll, pitch, yaw), got {len(grasp)}")

    env = _make_env(world)
    pos = grasp[:3]
    orn = pb.getQuaternionFromEuler(grasp[3:])
    num_joints = len(env.joint_index)

    for _ in range(100):  # Try 10 samples
        rest_pose = [np.random.uniform(-np.pi, np.pi) for _ in range(num_joints)]
        ik_solution = pb.calculateInverseKinematics(
            env.robot_id, env.joint_index['m6'], pos, orn,
            restPoses=rest_pose,
            jointDamping=[0.1] * num_joints
        )
        config = tuple(ik_solution[:num_joints])

        # Collision check
        for i, angle in enumerate(config):
            pb.resetJointState(env.robot_id, i, angle)
        pb.stepSimulation()

        if len(pb.getContactPoints(env.robot_id)) == 0:
            yield (config,)

def cfree_config(world: WorldState,
                 q: Config
) -> bool:
    """
    True if moving the robot to q causes no collisions.
    """
    env = _make_env(world)
    for j_idx, angle in enumerate(q):
        pb.resetJointState(env.robot_id, j_idx, angle)
    pb.stepSimulation()
    contacts = pb.getContactPoints(bodyA=env.robot_id)
    return len(contacts) == 0
# def ik_stream(world: WorldState, b: str, l: str, grasp) -> Iterator[tuple]:
#     """Dummy IK: return zero‑vector of length 6."""
#     q = (0.0,) * 6
#     yield (q,)
#
# def cfree_config(world: WorldState, q: Config) -> bool:
#     """Dummy collision check: always succeed."""
#     return True

def motion_stream(world: WorldState, q1: Config, q2: Config) -> Iterator[tuple]:
    """Straight‑line path between two 6‑d configs."""
    num_steps = 20
    weights = np.linspace(0, 1, num_steps)
    straight = [tuple((1 - w)*a + w*b for a, b in zip(q1, q2))
                for w in weights]
    if all(cfree_config(world, q) for q in straight):
        yield (straight,)
        return
    from .ompl_motion import plan_rrt_connect
    from ompl import util as ou
    max_rrt_attempts = 5
    timeout = 0.5
    ou.RNG.setSeed(123)
    for seed in range(max_rrt_attempts):
        rrt_path = plan_rrt_connect(list(q1), list(q2), seed=seed, timeout=timeout, world=world)
        if not rrt_path:
            continue
        yield (rrt_path,)
    # yield ([q1, q2],)

def traj_free(world: WorldState, path: Path, b: str) -> Iterator[tuple]:
    """Dummy trajectory‑free: always succeed."""
    yield ()

def gen_loc_stream(symbolic: SymbolicWorld, base: str) -> Iterator[Tuple[int,str]]:
    """
    Lazily propose the next free location on `base`.
    """
    used = [int(loc.split('_loc',1)[1])
            for loc in symbolic.values()
            if loc.startswith(f"{base}_loc")]
    lvl = max(used)+1 if used else 0
    yield (lvl, f"{base}_loc{lvl}")


# Register streams for PDDLStream
STREAMS: Dict[str, StreamInfo] = {
    'ik':           StreamInfo(from_fn(ik_stream)),
    'cfree_config': StreamInfo(from_fn(cfree_config)),
    'motion':       StreamInfo(from_fn(motion_stream)),
    'traj_free':    StreamInfo(from_fn(traj_free)),
    'gen-loc':      StreamInfo(from_fn(gen_loc_stream)),
}

