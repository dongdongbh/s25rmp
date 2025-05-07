import contextlib
import pybullet as pb
import numpy as np

from pddlstream.language.generator import from_gen_fn
from simulation import SimulationEnvironment
from pybullet_planning.pybullet_tools.utils import (
    get_movable_joints,
    get_collision_fn,
)

# -----------------------------------------------------------------------------
# GLOBAL INITIALIZATION
# -----------------------------------------------------------------------------
_env = None
_robot = None
_movable_joints = None
_ee_link = None
_collision_fn = None


def _extract_pair(x):
    """Recursively find a 2‑tuple (np.array, quat) inside nested lists/tuples."""
    if isinstance(x, (list, tuple)):
        if len(x) == 2 and isinstance(x[0], np.ndarray) and isinstance(x[1], tuple):
            return x[0], x[1]
        for elt in x:
            try:
                return _extract_pair(elt)
            except ValueError:
                continue
    raise ValueError(f"Could not extract (pos,ori) from {x!r}")



def init_pybullet(show=False):
    global _env, _robot, _movable_joints, _ee_link, _collision_fn
    if _env is not None:
        pb.disconnect()
    _env = SimulationEnvironment(show=show)
    _robot = _env.robot_id
    _ee_link = _env.joint_index['t7m']
    _movable_joints = get_movable_joints(_robot)
    obstacles = list(_env.block_id.values())[1:]
    _collision_fn = get_collision_fn(_robot, _movable_joints,
                                     obstacles=obstacles)

# -----------------------------------------------------------------------------
# CONTEXT‐MANAGER (optional, used in execute_plan)
# -----------------------------------------------------------------------------
@contextlib.contextmanager
def attach(robot_id, link_index, block_id, raw_grasp):
    """
    Temporarily weld `block_id` onto robot `robot_id` at `link_index`
    using the (possibly nested) grasp_pose.
    """
    # peel off the true (pos, ori) regardless of nesting
    pos, ori = _extract_pair(raw_grasp)
    cid = pb.createConstraint(
        parentBodyUniqueId     = robot_id,
        parentLinkIndex        = link_index,
        childBodyUniqueId      = block_id,
        childLinkIndex         = -1,
        jointType              = pb.JOINT_FIXED,
        jointAxis              = (0,0,0),
        parentFramePosition    = pos,
        childFramePosition     = (0,0,0),
        parentFrameOrientation = ori,
        childFrameOrientation  = (0,0,0,1),
    )
    try:
        yield
    finally:
        pb.removeConstraint(cid)

# -----------------------------------------------------------------------------
# STREAM: sample-grasp  (1 output: ?g)
# -----------------------------------------------------------------------------
def _sample_grasp_fn(block):
    """Inputs: ?o  Outputs: one grasp object"""
    grasp = (np.zeros(3), (0.0, 0.0, 0.0, 1.0))
    # yield a 1‐tuple containing that grasp
    yield (grasp,)
sample_grasp = from_gen_fn(_sample_grasp_fn)

# -----------------------------------------------------------------------------
# STREAM: sample-pose   (1 output: ?p)
# -----------------------------------------------------------------------------
def _sample_pose_fn(block):
    """Inputs: ?o  Outputs: one pose object"""
    loc, quat = _env.get_block_pose(block)
    for _ in range(10):
        dx, dy = np.random.uniform(-0.1, 0.1, size=2)
        x, y = loc[0] + dx, loc[1] + dy
        if np.hypot(x, y) > 0.2:
            continue
        pose = (np.array([x, y, loc[2]]), quat)
        yield (pose,)
sample_pose = from_gen_fn(_sample_pose_fn)

# -----------------------------------------------------------------------------
# STREAM: inverse-kin   (2 outputs: ?q ?t)
# -----------------------------------------------------------------------------
def _inverse_kin_fn(block, raw_pose, raw_grasp):
    """
    Inputs: ?o ?p ?g
    raw_pose may be nested; raw_grasp likewise.
    """
    # peel off the real (pos, ori) pair
    pos, ori   = _extract_pair(raw_pose)
    grasp_vec  = _extract_pair(raw_grasp)  # if you ever need grasp details

    # lift target above block
    CUBE_SIDE  = 0.01905
    target_pos = (pos[0], pos[1], pos[2] + CUBE_SIDE/2 + 0.05)
    from simulation import _pb_to_quat, _quat_to_pb
    ori = _quat_to_pb(ori)
    # seed with current joints
    q0 = [pb.getJointState(_robot, j)[0] for j in _movable_joints]
    sol = pb.calculateInverseKinematics(
        _robot, _ee_link,
        target_pos, ori,
        restPoses=q0,
        jointDamping=[0.1]*len(_movable_joints)
    )
    q_sol = tuple(sol)
    # only yield if collision‑free
    if _collision_fn(q_sol):
        traj = (tuple(q0), q_sol)
        # exactly two outputs: (?q, ?t)
        yield (q_sol, traj)

inverse_kin = from_gen_fn(_inverse_kin_fn)

# -----------------------------------------------------------------------------
# STREAM: plan-free   (1 output: ?t)
# -----------------------------------------------------------------------------
def _plan_free_fn(q1, q2):
    """Inputs: ?q1 ?q2  Outputs: one trajectory"""
    traj = [q1, q2]
    yield (traj,)
plan_free = from_gen_fn(_plan_free_fn)

# -----------------------------------------------------------------------------
# STREAM: plan-hold   (1 output: ?t)
# -----------------------------------------------------------------------------
def _plan_hold_fn(q1, q2, block, grasp):
    """Inputs: ?q1 ?q2 ?o ?g  Outputs: one trajectory"""
    traj = [q1, q2]
    yield (traj,)
plan_hold = from_gen_fn(_plan_hold_fn)

# -----------------------------------------------------------------------------
# STREAM MAP
# -----------------------------------------------------------------------------
STREAM_MAP = {
    'sample-grasp': sample_grasp,
    'sample-pose':  sample_pose,
    'inverse-kin':  inverse_kin,
    'plan-free':    plan_free,
    'plan-hold':    plan_hold,
}

