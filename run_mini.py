from pddlstream.language.constants import PDDLProblem
from pddlstream.utils import read, get_file_path
from pddlstream.language.generator import from_fn, from_test, from_gen_fn
from pddlstream.algorithms.incremental import solve_incremental
from pddlstream.language.converters import convert_plan

# your pybullet-planning wrappers:
from examples.pybullet.utils.pybullet_tools.kuka_primitives import get_grasp_gen, get_stable_gen, get_ik_fn
from examples.pybullet.utils.pybullet_tools.utils import connect, load_model, get_bodies, get_configuration

def make_problem(robot, block):
    # load PDDL
    domain = read(get_file_path(__file__, 'domain.pddl'))
    stream_pddl = read(get_file_path(__file__, 'stream.pddl'))

    # initial conf & pose
    from examples.pybullet.utils.pybullet_tools.kuka_primitives import BodyConf, BodyPose
    q0 = BodyConf(robot, get_configuration(robot))
    p0 = BodyPose(block, None)  # will be filled by sample-pose

    init = [
      ('HandEmpty',),
      ('AtConf', q0),
      ('AtPose', block, p0),
    ]
    goal = ('AtPose', block, BodyPose(block, None))  # goal will come from sample-pose too

    # streams
    stream_map = {
      'sample-grasp': from_gen_fn(get_grasp_gen(robot)),
      'sample-pose': from_gen_fn(get_stable_gen(get_bodies())),
      'inverse-kin':   from_fn(get_ik_fn(robot, get_bodies(), teleport=False)),
      'plan-free':     from_fn(lambda inputs, certified: ...),  # wrap your free‐motion planner
      'plan-hold':     from_fn(lambda inputs, certified: ...),  # wrap holding‐motion
    }

    return PDDLProblem(domain, {}, stream_pddl, stream_map, init, goal)

def main():
    connect(use_gui=True)
    # load your PyBullet world exactly as in evaluation_simple.py
    from simulation import SimulationEnvironment
    env = SimulationEnvironment(show=True)
    # create one block
    bases = [(pos, quat) for pos, quat in env.get_tower_base_poses(half_spots=2)]
    block = env._add_block(*bases[0], mass=2, side=0.01905)

    # build & solve
    problem = make_problem(env.robot_id, block)
    solution = solve_incremental(*problem)
    plan, _ = convert_plan(solution)

    print("Plan:", plan)
    # translate each PDDL action into env.goto_position(...) calls in Controller.run

if __name__ == '__main__':
    main()

