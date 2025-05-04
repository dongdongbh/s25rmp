# planners/pddlstream_interface.py

from typing import Tuple, Iterator, List, Dict
import pybullet as pb
from simulation import SimulationEnvironment, BASE_Z
from pddlstream.language.stream import StreamInfo
from pddlstream.language.generator import from_fn

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
    """Dummy IK: return zero‑vector of length 6."""
    q = (0.0,) * 6
    yield (q,)

def cfree_config(world: WorldState, q: Config) -> bool:
    """Dummy collision check: always succeed."""
    return True

def motion_stream(world: WorldState, q1: Config, q2: Config) -> Iterator[tuple]:
    """Straight‑line path between two 6‑d configs."""
    yield ([q1, q2],)

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

