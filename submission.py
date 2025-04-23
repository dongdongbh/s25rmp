from planners.pddlstream_interface import STREAMS, ACTIONS
from run_planner import plan_task
from planners.ompl_motion import plan_rrt_connect

class Controller:
    def __init__(self):
        # load any optimized model data here
        pass

    def run(self, env, goal_poses):
        # 1) dump env → PDDLProblem, call plan_task
        plan = plan_task('pddl/domain.pddl','pddl/problem.pddl',STREAMS,ACTIONS)
        # 2) for each (action, params) in plan:
        #     if action=='pick': env.goto_position(params['q'], duration)
        #     elif action=='move': ...
        #     elif action=='place': ...
        # 3) optionally call env._add_block, env.settle(), etc.

if __name__ == "__main__":

    # you can edit this part for informal testing
    pass

