"""
submission.py

Controller using PDDLStream for planning and reusing stream functions for validation.
"""
import pybullet as pb
from pddlstream.language.constants import PDDLProblem, And, Atom
from pddlstream.algorithms.meta import solve
from planners.pddlstream_interface import (
    STREAMS, ACTIONS,
    extract_physical_world,
    ik_stream, cfree_config,
    motion_stream, traj_free
)
from simulation import SimulationEnvironment

class Controller:
    def __init__(self,
                 domain_path: str = 'pddl/domain.pddl'):
        self.domain_path = domain_path

    def run(self,
            real_env: SimulationEnvironment,
            goal_poses: Dict[str, tuple]):
        """
        Plan–validate–execute loop:
        1) Build and solve in-memory PDDLProblem
        2) Validate each action using stream functions
        3) Execute actions in real_env, replan on failure
        """
        while True:
            # 1) Snapshot current symbolic map
            symbolic_map = self._infer_symbolic_map_from_env(real_env)
            # 2) Build in-memory problem
            problem = self._build_problem(symbolic_map, goal_poses)
            # 3) Solve once via PDDLStream
            result = solve(
                domain=self.domain_path,
                problem=problem,
                streams=STREAMS,
                actions=ACTIONS,
                planner='ff-astar',
                algorithm='adaptive'
            )
            # result is list of (action_name, params)
            if self._validate_and_execute(real_env, result):
                break

    def _infer_symbolic_map_from_env(self, env: SimulationEnvironment) -> Dict[str, str]:
        """
        Build {block: location_symbol} from real_env block poses.
        Must align block positions to declared location symbols.
        """
        # Placeholder: implement mapping from env.get_block_pose to symbolic locations
        raise NotImplementedError

    def _build_problem(self,
                       symbolic_map: Dict[str, str],
                       goal_poses: Dict[str, tuple]) -> PDDLProblem:
        """
        Construct a PDDLProblem with current At/Clear/Empty/Base/Location facts.
        """
        atoms = []
        # Dynamic At facts
        for b, loc in symbolic_map.items():
            atoms.append(Atom('At', [b, loc]))
        # Clear facts for all declared locations not occupied
        # (user should collect all locations and add Clear accordingly)
        atoms.append(Atom('Empty', []))
        # Goal atoms
        goal_atoms = [Atom('At', [b, l]) for b, l in goal_poses.items()]
        return PDDLProblem(
            domain=self.domain_path,
            initial=And(*atoms),
            goal=And(*goal_atoms)
        )

    def _validate_and_execute(self,
                              env: SimulationEnvironment,
                              plan: List[tuple]) -> bool:
        """
        Validate each step using stream functions; execute on success.
        """
        # snapshot physical world once (will refresh after each execution)
        for action, params in plan:
            world_state = extract_physical_world(env)
            if not self._validate_action(world_state, action, params):
                return False
            self._execute_action(env, action, params)
        return True

    def _validate_action(self,
                         world_state: Dict[str, tuple],
                         action: str,
                         params: Dict) -> bool:
        """
        Reuse PDDLStream streams to test feasibility in forked state.
        """
        if action in ('pick', 'place'):
            # test collision-free configuration
            q = params['?q']
            return cfree_config(world_state, q)
        if action == 'move':
            q1 = params['?q1']
            q2 = params['?q2']
            # check motion → path candidates
            for (path,) in motion_stream(world_state, q1, q2):
                # check each config
                if not all(cfree_config(world_state, q) for q in path):
                    continue
                # check trajectory free
                if list(traj_free(world_state, path, 'nil')):
                    return True
            return False
        return True

    def _execute_action(self,
                        env: SimulationEnvironment,
                        action: str,
                        params: Dict):
        """
        Execute pick/move/place via simple joint commands.
        """
        if action in ('pick', 'place'):
            q = params['?q']
            env._step(q)
        elif action == 'move':
            for q in params['?t']:
                env._step(q)

if __name__ == '__main__':
    env = SimulationEnvironment(show=True)
    goal = {}  # user‐provided goal map
    ctrl = Controller()
    ctrl.run(env, goal)
    env.close()

