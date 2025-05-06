import time
from pddlstream.utils import read, get_file_path
from pddlstream.language.constants import PDDLProblem, print_solution
from pddlstream.algorithms.meta import solve
import numpy as np

import stream
from stream import attach
from stream import STREAM_MAP



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
    stream.init_pybullet(show=True)
    env = stream._env

    block = env._add_block((0.2,0,0.01), (1,0,0,0), mass=1, side=0.01905)
    env.settle(1.0)

    init = [
      ('HandEmpty',),
      ('Graspable', block),
      ('Poseable',  block),
    ]

    input("Press Enter to start…")
    # sample a grasp and pose for the INIT state
    g = next(stream.sample_grasp(block))
    init.append(('Grasp', block, g))

    p0 = next(stream.sample_pose(block))
    init += [
      ('Pose',   block, p0),
      ('AtPose', block, p0),
    ]

    # <<< THIS is the only goal we use for now >>>
    goal = ('Holding', block)

    domain_pddl = read(get_file_path(__file__, 'domain.pddl'))
    stream_pddl = read(get_file_path(__file__, 'stream.pddl'))
    problem = PDDLProblem(domain_pddl, {}, stream_pddl,
                          STREAM_MAP, init, goal)

    # debug print to confirm types:
    print("INIT:", init)
    print("GOAL:", goal, type(goal))

    plan, cost, _ = solve(problem,
                          algorithm='adaptive',
                          planner='ff-astar',
                          max_planner_time=10,
                          unit_costs=True)

    print_solution((plan, cost, []))
    if plan:
        for action, args in plan:
            print("→", action, args)
            time.sleep(0.5)

    if plan:
        execute_plan(env, plan)

    input("Done. Press Enter to exit…")
    env.close()

if __name__ == '__main__':
    main()

