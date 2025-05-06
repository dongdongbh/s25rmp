#!/usr/bin/env python3
import sys, os
import time

# Point at your local pybullet-planning folder
sys.path.insert(0, os.path.join(os.path.dirname(__file__), 'pybullet-planning'))

import stream
from simulation import SimulationEnvironment
from pybullet_planning.pybullet_tools.utils import get_movable_joints, get_collision_fn

def replay_trajectory(env, joints, trajectory, duration=2.0):
    """
    Linearly execute each waypoint in `trajectory` over `duration` seconds.
    """
    n = len(trajectory)
    if n == 0:
        return
    dt = duration / n
    for q in trajectory:
        # unwrap one level of nesting if needed
        if isinstance(q, (list, tuple)) and len(q) == 1 and isinstance(q[0], (list, tuple)):
            q = q[0]
        # now q should be a flat tuple of floats
        target = {
            env.joint_name[j]: q[i] * 180.0 / 3.14159
            for i, j in enumerate(joints)
        }
        env.goto_position(target, duration=dt)
        time.sleep(dt)

def flatten_traj(raw):
    """
    Given raw output from next(stream.plan_*), return a flat list of joint‐tuples.
    E.g. ((q1,q2),)  → [q1, q2]
          (((q1,q2),),) → [q1, q2]
    """
    traj = raw
    # keep unwrapping single‐element sequences
    while isinstance(traj, (list, tuple)) and len(traj) == 1:
        traj = traj[0]
    # now traj is either a list/tuple of q‐tuples, or a single q‐tuple
    if isinstance(traj[0], (list, tuple)):
        return list(traj)
    else:
        # maybe it was just one configuration?
        return [traj]

def main():
    # 1) Create a GUI env
    env = SimulationEnvironment(show=True)
    robot = env.robot_id

    # 2) Spawn one movable block in front of the arm
    block = env._add_block((0.2, 0.0, 0.01), (1,0,0,0), mass=1, side=0.01905)
    env.settle(0.5)

    # 3) Rewire stream internals to use this env
    stream._env = env
    stream._robot = robot
    joints = get_movable_joints(robot)
    stream._movable_joints = joints
    stream._ee_link = env.joint_index['t7m']
    obstacles = list(env.block_id.values())[1:]  # exclude the static base
    stream._collision_fn = get_collision_fn(robot, joints, obstacles=obstacles)

    input("Press Enter to sample grasp and pose...")

    CERT = []

    # 4) Sample a grasp and pose
    grasp = next(stream.sample_grasp((block,), CERT))[0]
    pose  = next(stream.sample_pose((block,), CERT))[0]
    print("Grasp:", grasp)
    print("Pose :", pose)
    input("Press Enter to solve IK and move to the pre-grasp pose...")

    # 6) Inverse‐kin: solve and move
    print("\n=== inverse-kin ===")
    try:
        raw = next(stream.inverse_kin((block, pose, grasp), CERT))
    except StopIteration:
        print("[FAIL] inverse-kin produced no outputs")
        return

    # Robust unpacking of PDDLStream’s nested return
    if isinstance(raw, tuple) and len(raw) == 2:
        q_sol, traj = raw
    elif isinstance(raw, (list, tuple)) and len(raw) == 1 \
         and isinstance(raw[0], tuple) and len(raw[0]) == 2:
        q_sol, traj = raw[0]
    else:
        print("Unexpected inverse-kin output:", raw)
        return

    print(f"[OK] inverse-kin -> q_sol={q_sol}, traj={traj}")
    # replay the two‐step trajectory
    replay_trajectory(env, joints, traj, duration=2.0)
    q0 = traj[0]

    # 6) plan-free (trivial teleport)
    gen_free = stream.plan_free((q_sol, q_sol), CERT)
    try:
        raw_free  = next(gen_free)
        plan_free = flatten_traj(raw_free)
        print("[OK] plan-free waypoints:", plan_free)
        replay_trajectory(env, joints, plan_free, duration=2.0)
    except StopIteration:
        print("[FAIL] plan-free produced no outputs")
        env.close()
        return

# 7) plan-hold (trivial teleport)
    gen_hold = stream.plan_hold((q_sol, q0, block, grasp), CERT)
    try:
        raw_hold  = next(gen_hold)
        plan_hold = flatten_traj(raw_hold)
        print("[OK] plan-hold waypoints:", plan_hold)
        replay_trajectory(env, joints, plan_hold, duration=2.0)
    except StopIteration:
        print("[FAIL] plan-hold produced no outputs")
        env.close()
        return

    print("[OK] plan-hold waypoints:", plan_hold)
    replay_trajectory(env, joints, plan_hold, duration=2.0)
    input("Done. Press Enter to exit.")

    env.close()


if __name__ == "__main__":
    main()

