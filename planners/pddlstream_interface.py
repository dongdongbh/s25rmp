# planners/pddlstream_interface.py

from typing import Tuple, Iterator, List, Dict
import pybullet as pb
from pddlstream.language.stream import StreamInfo, PartialInputs
from pddlstream.language.function import FunctionInfo
from pddlstream.language.generator import from_fn
from simulation import SimulationEnvironment

# --- Type aliases ---------------------------------------------------------
PhysicalPose  = Tuple[Tuple[float, float, float], Tuple[float, float, float, float]]
WorldState    = Dict[str, PhysicalPose]
SymbolicWorld = Dict[str, str]

# --- Shared DIRECT-mode environment --------------------------------------
# Create one persistent DIRECT-mode env and load robot & floor
MASTER_ENV = SimulationEnvironment(show=False)

# --- Helper: reset and populate MASTER_ENV -------------------------------
def _make_env_from_world(world: WorldState) -> SimulationEnvironment:
    """
    Reset the shared DIRECT-mode env to its base state and add all blocks
    according to `world`.
    """
    MASTER_ENV._reset()
    for pos, quat in world.values():
        MASTER_ENV._add_block(pos, quat)
    return MASTER_ENV

# --- Stream implementations -----------------------------------------------
Config = Tuple[float, ...]
Path   = List[Config]

# def ik_stream(world: WorldState,
#               b: str,
#               l: str,
#               grasp: Tuple[float, ...]
# ) -> Iterator[Tuple[Config,]]:
#     """
#     Dummy IK: return the current robot joint angles as a valid config.
#     """
#     env = _make_env_from_world(world)
#     q   = tuple(env._get_position())
#     yield (q,)

def ik_stream(world_state: WorldState,
              block_id: str,
              grasp_pose: Tuple[float, float, float, float, float, float]
) -> Iterator[Tuple[Config,]]:
    """
    Yield collision-free IK solutions for grasp_pose using PyBullet.
    Tries multiple rest poses to find collision-free IK.
    """
    env = _make_env_from_world(world)
    pos = grasp_pose[:3]
    orn = pb.getQuaternionFromEuler(grasp_pose[3:])
    num_joints = len(env.joint_index)

    for _ in range(10):  # Try 10 samples
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
    env = _make_env_from_world(world)
    for j_idx, angle in enumerate(q):
        pb.resetJointState(env.robot_id, j_idx, angle)
    pb.stepSimulation()
    contacts = pb.getContactPoints(bodyA=env.robot_id)
    return len(contacts) == 0

def motion_stream(world: WorldState,
                  q1: Config,
                  q2: Config
) -> Iterator[Tuple[Path,]]:
    """
    Simple straight-line joint-space path.
    """
    yield ([q1, q2],)

def traj_free(world: WorldState,
              path: Path,
              b: str
) -> Iterator[()]:
    """
    True if executing `path` causes no robot-to-environment collisions,
    excluding any contact with the held block `b`.
    """
    env     = _make_env_from_world(world)
    held_id = env.block_id.get(b, None)

    for q in path:
        for j_idx, angle in enumerate(q):
            pb.resetJointState(env.robot_id, j_idx, angle)
        pb.stepSimulation()

    contacts = pb.getContactPoints(bodyA=env.robot_id)
    # filter out any contact with the held block
    if held_id is not None:
        contacts = [c for c in contacts if c[2] != held_id]
    if not contacts:
        yield ()

def gen_loc_stream(symbolic: SymbolicWorld,
                   base: str
) -> Iterator[Tuple[int, str]]:
    """
    Lazily propose the next free location on `base`, given the current symbolic map.
    """
    used   = [loc for loc in symbolic.values() if loc.startswith(f"{base}_loc")]
    levels = []
    for loc in used:
        try:
            levels.append(int(loc.split("_loc",1)[1]))
        except ValueError:
            pass
    next_level = max(levels) + 1 if levels else 0
    new_loc     = f"{base}_loc{next_level}"
    yield (next_level, new_loc)

# --- Register streams & actions ------------------------------------------
STREAMS: Dict[str, StreamInfo] = {
    'ik': StreamInfo(
        inputs    = ['?w','?b','?l','?g'],
        domain    = ['(Block ?b)','(Location ?l)'],
        outputs   = ['?q'],
        certified = ['(Kin ?b ?l ?q)'],
        fn        = from_fn(ik_stream),
        partial   = PartialInputs(share=True),
    ),
    'cfree_config': StreamInfo(
        inputs    = ['?w','?q'],
        domain    = ['(Config ?q)'],
        outputs   = [],
        certified = ['(CFreeConf ?q)'],
        fn        = from_fn(cfree_config),
        partial   = PartialInputs(share=True),
    ),
    'motion': StreamInfo(
        inputs    = ['?w','?q1','?q2'],
        domain    = ['(Config ?q1)','(Config ?q2)'],
        outputs   = ['?t'],
        certified = ['(Motion ?q1 ?t ?q2)'],
        fn        = from_fn(motion_stream),
        partial   = PartialInputs(share=True),
    ),
    'traj_free': StreamInfo(
        inputs    = ['?w','?t','?b'],
        domain    = ['(Motion ?q1 ?t ?q2)','(Holding ?b)'],
        outputs   = [],
        certified = ['(CFreeTraj ?t ?b)'],
        fn        = from_fn(traj_free),
        partial   = PartialInputs(share=True),
    ),
    'gen-loc': StreamInfo(
        inputs    = ['?w','?base'],
        domain    = ['(Base ?base)'],
        outputs   = ['?level','?loc'],
        certified = ['(Location ?loc)'],
        fn        = from_fn(gen_loc_stream),
        partial   = PartialInputs(share=True),
    ),
}

ACTIONS: List[FunctionInfo] = [
    FunctionInfo('pick',  inputs=['?b','?l','?g','?q'], outputs=[], costs=0),
    FunctionInfo('move',  inputs=['?q1','?t','?q2'],      outputs=[], costs=0),
    FunctionInfo('place', inputs=['?b','?l','?g','?q'], outputs=[], costs=0),
]

