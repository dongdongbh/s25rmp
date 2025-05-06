import itertools as it
import numpy as np
import pybullet as pb
from simulation import SimulationEnvironment, _pb_to_quat
from submission import Controller

CUBE_SIDE = 0.01905  # .75 inches


def get_tower_base_poses(half_spots=4):
    radii = (-.14, -.19)
    alpha = 0.5 * 1.57
    thetas = (3.14 - alpha) / 2 + alpha * np.arange(half_spots) / (half_spots - 1)

    bases = ()
    for (r, theta) in it.product(radii, thetas):
        pos = (r * np.cos(theta), r * np.sin(theta), -0.5 * CUBE_SIDE)
        quat = _pb_to_quat(pb.getQuaternionFromEuler((0, 0, theta)))
        bases += ((pos, quat),)

    return bases


def sample_trial(num_blocks, num_swaps, show=True):
    """
    Simplified trial: only supports num_blocks=1 and num_swaps=0.
    Blocks are placed at the first tower base, goal is the second base.
    """
    assert num_blocks == 1 and num_swaps == 0, \
        "This simple trial only supports one block and zero swaps."

    env = SimulationEnvironment(show=show)

    # get two candidate bases
    bases = get_tower_base_poses(half_spots=2)
    init_pos, init_quat = bases[0]
    goal_pos, goal_quat = bases[1]

    # instantiate block at initial base
    label = env._add_block(init_pos, init_quat, mass=2, side=CUBE_SIDE)
    env.settle(1.)

    goal_poses = {label: (goal_pos, goal_quat)}
    return env, goal_poses


def evaluate(env, goal_poses):
    env.settle(1.)
    num_correct = 0
    loc_errors, rot_errors = [], []
    for label, (goal_loc, goal_quat) in goal_poses.items():
        loc, quat = map(np.array, env.get_block_pose(label))
        if (np.fabs(loc - goal_loc) < CUBE_SIDE / 2).all():
            num_correct += 1
        loc_errors.append(np.linalg.norm(loc - goal_loc))
        rot_errors.append(2 * np.arccos(min(1., np.fabs(quat @ goal_quat))))
    accuracy = num_correct / len(goal_poses)
    return accuracy, loc_errors, rot_errors


if __name__ == "__main__":
    controller = Controller()
    env, goal_poses = sample_trial(num_blocks=1, num_swaps=0, show=True)

    # run controller
    controller.run(env, goal_poses)
    input("Press Enter to continue…")


    # evaluate and print
    accuracy, loc_errors, rot_errors = evaluate(env, goal_poses)
    env.close()

    print(f"\n{int(100 * accuracy)}% of blocks at goal positions")
    print(f"loc error: mean={np.mean(loc_errors):.3f}, max={np.max(loc_errors):.3f}")
    print(f"rot error: mean={np.mean(rot_errors):.3f}, max={np.max(rot_errors):.3f}")
