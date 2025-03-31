import itertools as it
import numpy as np
import pybullet as pb
from simulation import SimulationEnvironment
from submission import Controller

CUBE_SIDE = .02

def get_base_positions(half_spots=4):
    radii = (-.14, -.19) #-.16
    alpha = 0.75*1.57
    thetas = (3.14 - alpha)/2 + alpha * np.arange(half_spots) / (half_spots-1)

    bases = ()
    for (r, theta) in it.product(radii, thetas):
        pos = (r*np.cos(theta), r*np.sin(theta), -0.5*CUBE_SIDE)
        quat = pb.getQuaternionFromEuler((0,0,theta))
        bases += ((pos, quat),)

    return bases

def sample_trial(show=True):
    env = SimulationEnvironment(show=show)

    # sample initial block positions
    bases = get_base_positions()

    # stack some random blocks
    towers = [[base] for base in bases]
    block_labels = []
    for block in range(4):

        # sample support of next block
        tower_idx = np.random.choice(len(towers))
        pos, quat = towers[tower_idx][-1]

        # position block on top of support
        new_pos = pos[:2] + (pos[2] + .0201,)

        # instantiate block
        label = env.add_block(new_pos, quat, mass=2, side=CUBE_SIDE)
        block_labels.append(label)

        # update new top of tower
        towers[tower_idx].append( (new_pos, quat) )

    # let blocks settle
    env.settle(1.)

    # setup goal poses
    goal_poses = {}
    for label in block_labels:
        goal_poses[label] = env.get_block_pose(label)

    # swap some poses to create a goal
    for _ in range(3):
        a, b = np.random.choice(block_labels, size=2)
        goal_poses[a], goal_poses[b] = goal_poses[b], goal_poses[a]

    return env, goal_poses

def evaluate(env, goal_poses):
    return 0., 1

if __name__ == "__main__":

    # initialize controller class
    controller = Controller()

    # sample a validation trial
    env, goal_poses = sample_trial()

    for label, pose in goal_poses.items():
        print(label, pose)
    input('.')

    # run the controller on the trial
    controller.run(env, goal_poses)

    # evaluate success
    accuracy, residual = evaluate(env, goal_poses)

    env.close()

    print(f"\n{int(100*accuracy)}% of blocks at correct goal positions, residual pose error = {residual}\n")
    
