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

SYMBOLIC_MAP = {}   # type: Dict[str,str]

def set_symbolic_map(symbolic_map: Dict[str,str]) -> None:
    """Call this once, right after you infer your symbolic_map."""
    SYMBOLIC_MAP.clear()
    SYMBOLIC_MAP.update(symbolic_map)

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


def ik_stream(world: WorldState, b: str, l) -> List[Tuple[Config]]:
    q: Config = (0.0,) * 6
    return [(q,)]

def cfree_config(world: WorldState, q: Config) -> bool:
    return True

def motion_stream(world: WorldState, q1: Config, q2: Config) -> List[Tuple[Path]]:
    path: Path = (q1, q2)
    return [(path,)]

def traj_free(world: WorldState, path: Path, b: str) -> List[Tuple]:
    return [()]

def gen_loc_stream(_world: WorldState, base: str):
    """
    Returns the next free location on `base` using the already-captured
    SYMBOLIC_MAP. This function now returns a single location string instead of level and location.
    It also updates the SYMBOLIC_MAP with the newly generated location.
    """
    used = []
    prefix = f"{base}_loc"
    
    # Iterate over the symbolic map to determine used levels
    for loc in SYMBOLIC_MAP.values():
        if loc.startswith(prefix):
            try:
                lvl = int(loc.split(prefix, 1)[1])
                used.append(lvl)
            except ValueError:
                pass
    
    # Determine the next available level
    new_lvl = max(used) + 1 if used else 0
    new_loc = f"{base}_loc{new_lvl}"

    # Update the SYMBOLIC_MAP with the new location
    SYMBOLIC_MAP[f"b{len(SYMBOLIC_MAP)}"] = new_loc  # Add a new block entry to the symbolic map

    # Return the location string only, not the level
    return [new_loc]  # Return only the new location as a list


STREAMS: Dict[str, StreamInfo] = {
    'ik'           : StreamInfo(from_fn(ik_stream)),
    'cfree_config' : StreamInfo(from_fn(cfree_config)),
    'motion'       : StreamInfo(from_fn(motion_stream)),
    'traj_free'    : StreamInfo(from_fn(traj_free)),
    'gen-loc'      : StreamInfo(from_fn(gen_loc_stream)),
}
