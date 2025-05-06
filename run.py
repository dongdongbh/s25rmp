import time
from pddlstream.utils import read, get_file_path
from pddlstream.language.constants import PDDLProblem, print_solution
from pddlstream.algorithms.meta import solve
import numpy as np
from simulation import SimulationEnvironment

import stream
from stream import register_environment, attach, STREAM_MAP



def execute_plan(env, plan):
    attach_ctx = None
    movable_joints = stream._movable_joints   # grab it *after* init_pybullet()
    for action, args in plan:
        if action == 'pick':
            block, pose_obj, grasp, q0, traj = args

            # 1) replay approach trajectory
            for q_conf in traj:
                full_q = env._get_position().copy()
                for i, joint_index in enumerate(movable_joints):
                    full_q[joint_index] = q_conf[i]
                angle_dict = env._angle_dict_from(full_q, convert=True)
                env.goto_position(angle_dict, duration=1.0)

            # 2) close gripper
            env.goto_position({'m6': -20}, duration=0.5)

            # 3) weld block
            attach_ctx = attach(
              env.robot_id, env.joint_index['t7m'],
              env.block_id[block], grasp)
            attach_ctx.__enter__()
            env.settle(1.0)
            
            input("Press Enter to continue…")

        elif action == 'place':
            block, pose_obj, grasp, q0, traj = args

            # 1) replay holding motion
            for q_conf in traj:
                full_q = env._get_position().copy()
                for i, joint_index in enumerate(movable_joints):
                    full_q[joint_index] = q_conf[i]
                angle_dict = env._angle_dict_from(full_q, convert=True)
                env.goto_position(angle_dict, duration=1.0)

            # 2) open gripper
            env.goto_position({'m6': 0}, duration=0.5)
            if attach_ctx:
                attach_ctx.__exit__(None, None, None)
                attach_ctx = None
            env.settle(1.0)

    if attach_ctx:
        attach_ctx.__exit__(None, None, None)



def main():
    # 1) spin up bullet and your SimEnvironment
    env = SimulationEnvironment(show=True)

    # 2) tell your streams about it
    register_environment(env)


    robot = env.robot_id
    import pybullet as pb
    nj = pb.getNumJoints(robot)
    for link in range(nj):
        pb.setCollisionFilterPair(robot, robot, -1, link, enableCollision=0)
    for i in range(nj):
        for j in range(i+1, nj):
            pb.setCollisionFilterPair(robot, robot, i, j, 1)


    # 3) now you can do your normal block‐adding, stream calls, PDDLProblem, solve, etc.
    block = env._add_block((0.2,0,0.01), (1,0,0,0), mass=1, side=0.01905)
    env.settle(1.0)

    input("Press Enter to continue…")
    init = [
      ('HandEmpty',),
      ('Graspable', block),
      ('Poseable',  block),
    ]

    # sample a grasp & pose for INIT
    g  = next(stream.sample_grasp(block))
    p0 = next(stream.sample_pose(block))
    init += [
      ('Grasp',   block, g),
      ('Pose',    block, p0),
      ('AtPose',  block, p0),
    ]
    goal = ('Holding', block)

    # build & solve
    domain_pddl  = read(get_file_path(__file__, 'domain.pddl'))
    stream_pddl  = read(get_file_path(__file__, 'stream.pddl'))
    problem      = PDDLProblem(domain_pddl, {}, stream_pddl,
                               STREAM_MAP, init, goal)
    plan, cost, _ = solve(problem,
                          algorithm='adaptive',
                          planner='ff-astar',
                          max_planner_time=10,
                          unit_costs=True)

    print("Plan: ", plan)
    input("Press Enter to continue…")

    if plan:
        execute_plan(env, plan)

    env.close()


if __name__ == '__main__':
    main()

