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

def generate_grasp_from_block(world, block):
    # block_position, block_orientation = pb.getBasePositionAndOrientation(block)
    env = _make_env(world)
    b_id = env.block_id[block]
    aabb_min, aabb_max = pb.getAABB(b_id)

    center_x = (aabb_min[0] + aabb_max[0]) / 2
    center_y = (aabb_min[1] + aabb_max[1]) / 2
    top_z = aabb_max[2]

    grasp = (center_x, center_y, top_z + 0.5, 0, 0, 0)

    return grasp

def ik_stream(world: WorldState, b: str, l: str, grasp) -> Iterator[tuple]:
    """
    Yield collision-free IK solutions for grasp_pose using PyBullet.
    Tries multiple rest poses to find collision-free IK.
    """
    if grasp is None:
        grasp = generate_grasp_from_block(world, b)

    if len(grasp) < 6:
        raise ValueError(
            f"Invalid grasp input: Expected at least 6 elements (x, y, z, roll, pitch, yaw), got {len(grasp)}")

    env = _make_env(world)
    print(f"Grasp: {grasp}")
    pos = grasp[:3]
    orn = pb.getQuaternionFromEuler(grasp[3:])
    # print(f"Target Position: {pos}, Target Orientation (Quaternion): {orn}")
    # print(f"joint_index={env.joint_index}")
    num_joints = len(env.joint_index)
    # robot_pos, robot_orn = pb.getBasePositionAndOrientation(env.robot_id)
    # block_pos, block_orn = pb.getBasePositionAndOrientation(env.block_id[b])
    #
    # print("Robot Position:", robot_pos)
    # print("Block Position:", block_pos)
    # robot_collision_shape_info = pb.getCollisionShapeData(env.robot_id, -1)
    # block_collision_shape_info = pb.getCollisionShapeData(env.block_id[b], -1)
    #
    # print("Robot Collision Shape:", robot_collision_shape_info)
    # print("Block Collision Shape:", block_collision_shape_info)

    for attempt in range(1):  # Try 10 samples
        current_state = [pb.getJointState(env.robot_id, i)[0] for i in range(num_joints)]
        angle_range = np.pi / max(1e-3, (4 - attempt / 2))
        rest_pose = [np.random.uniform(max(-np.pi, c - angle_range),
                                       min(np.pi, c + angle_range))
                     for c in current_state]

        ik_solution = pb.calculateInverseKinematics(
            env.robot_id, env.joint_index['m6'], pos, orn,
            restPoses=rest_pose,
            jointDamping=[0.1] * num_joints
        )
        config = tuple(ik_solution[:num_joints])

        # Collision check
        if cfree_config(world, config):
            yield (config,)
        # for i, angle in enumerate(config):
        #     pb.resetJointState(env.robot_id, i, angle)
        # for _ in range(10):
        #     pb.stepSimulation()
        #
        # # b_id = env.block_id[b]
        # # print(b_id)
        # collision_detected = len(pb.getContactPoints(bodyA=env.robot_id)) > 0
        # print(f"Collision detected: {len(pb.getContactPoints(bodyA=env.robot_id))}")

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
    # print(env.block_id)
    # print(pb.getBasePositionAndOrientation(env.block_id['b0']))
    # for _ in range(10):
    #     pb.stepSimulation()

    contacts = (len(pb.getContactPoints(bodyA=env.robot_id, bodyB=b)) == 0 for b in env.block_id.values())
    return all(contacts)
    #
    # if any(c[2] in env.block_id for c in contacts):
    #     return False
    #
    # return True


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
    # num_steps = 20
    # weights = np.linspace(0, 1, num_steps)
    # straight = [tuple((1 - w)*a + w*b for a, b in zip(q1, q2))
    #             for w in weights]
    # if all(cfree_config(world, q) for q in straight):
    #     yield (straight,)
    #     return
    from .ompl_motion import plan_rrt_connect
    from ompl import util as ou
    import math
    max_rrt_attempts = 5
    timeout = 0.5
    space_bounds = [(-math.pi,  math.pi)] * 6
    ou.RNG().setSeed(1)
    for seed in range(max_rrt_attempts):
        rrt_path = plan_rrt_connect(list(q1), list(q2), seed=seed, timeout=timeout, world=world, space_bounds=space_bounds)
        if rrt_path is not None:
            yield (rrt_path,)
            return
    # yield ([q1, q2],)

def traj_free(world: WorldState, path: Path, b: str) -> Iterator[tuple]:
    """Dummy trajectory‑free: always succeed."""
    env     = _make_env(world)
    held_id = env.block_id.get(b, None)

    for q in path:
        for j_idx, angle in enumerate(q):
            pb.resetJointState(env.robot_id, j_idx, angle)
        pb.stepSimulation()

    # contacts = (len(pb.getContactPoints(bodyA=env.robot_id, bodyB=b)) == 0 for b in env.block_id.values() if b!=held_id)
    # filter out any contact with the held block
    if held_id is not None:
        # contacts = [c for c in contacts if c[2] != held_id and c[2] in env.block_id]
        contacts = [pb.getContactPoints(bodyA=env.robot_id, bodyB=b) for b in env.block_id.values() if
                    b != held_id]
        print(contacts)
        yield (contacts,)

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

