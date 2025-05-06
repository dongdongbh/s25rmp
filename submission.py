"""
See example.py for examples
Implement Controller class with your motion planner
"""
import numpy as np
import pybullet as pb


CUBE_SIDE = 0.01905

class Controller:
    def __init__(self):
        # load any optimized model data here
        pass

    def run(self, env, goal_poses):
        # run the controller in the environment to achieve the goal
        # Only one block in evaluation_simple
        block, = goal_poses.keys()
        goal_pos, goal_quat = goal_poses[block]
        # 1) Get initial pose
        init_pos, init_quat = env.get_block_pose(block)

        # 2) Define two waypoints: above block and above goal
        lift_height = CUBE_SIDE/2 + 0.05
        pre_grasp = (np.array(init_pos)   + np.array([0,0,lift_height]), init_quat)
        pre_place = (np.array(goal_pos)   + np.array([0,0,lift_height]), goal_quat)

        # Helper to solve IK and move there
        def goto(pose):
            pos, quat = pose
            # Get the robot's base position and orientation using PyBullet API
            robot_base_pos, robot_base_quat = pb.getBasePositionAndOrientation(env.robot_id)

            # Convert the target position from world coordinates to the robot's base frame
            pos_in_robot_frame = np.array(pos) - np.array(robot_base_pos)

            # Convert the robot's base quaternion to a rotation matrix
            robot_base_rot = pb.getMatrixFromQuaternion(robot_base_quat)

            # Convert the target quaternion to a rotation matrix
            quat_rot = pb.getMatrixFromQuaternion(quat)

            # To transform the target orientation to the robot's base frame,
            # we multiply the target rotation matrix by the inverse of the robot's base rotation matrix.
            # We multiply the rotation matrices and then convert it back to a quaternion.
            transformed_rot = np.dot(np.linalg.inv(np.array(robot_base_rot).reshape(3, 3)), np.array(quat_rot).reshape(3, 3))

            # Convert the transformed rotation matrix back to a quaternion
            transformed_quat = pb.getQuaternionFromMatrix(transformed_rot)

            # Use current robot state as rest pose
            movable = env.joint_index
            q0 = [pb.getJointState(env.robot_id, j)[0] for j in movable.values()]

            # Now compute the inverse kinematics with the position and transformed orientation in the robot's base frame
            sol = pb.calculateInverseKinematics(
                env.robot_id,
                env.joint_index['t7m'],
                pos_in_robot_frame,  # Use the corrected position here
                transformed_quat,     # Use the transformed quaternion here
                jointDamping=[0.1] * len(q0)
            )

            # Build dict { joint_name: angle_in_degrees }
            angle_dict = {
                name: sol[idx] * (180 / np.pi)
                for name, idx in env.joint_index.items()
                if idx < len(sol)
            }
            
            # Move the robot to the calculated joint positions
            env.goto_position(angle_dict, duration=3.0)


        input("Press Enter to start move above block...")
        # 3) Move above block
        goto(pre_grasp)

        print(f'Moving to pre-grasp position: {pre_grasp}')

        link_pos, link_ori = pb.getLinkState(env.robot_id, env.joint_index['t7m'])[:2]
        print("Link position relative to robot base:", link_pos)
        print("Link orientation:", link_ori)


        input("Press Enter to grasp...")
        # 4) Descend to grasp
        goto((init_pos, init_quat))
        input("Press Enter to lift...")
        # 6) Lift
        goto(pre_grasp)
        input("Press Enter to above goal...")
        # 7) Move above goal
        goto(pre_place)
        input("Press Enter to place...")
        # 8) Descend to place
        goto((goal_pos, goal_quat))
        input("Press Enter to start the controller...")
        # 9) Detach
        # 10) Lift away
        goto(pre_place)

if __name__ == "__main__":

    # you can edit this part for informal testing
    pass

