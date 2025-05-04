# submission.py

import pybullet as pb
import numpy as np
from typing import Dict, List, Tuple

# --- MONKEY‑PATCH: disable optimistic combination logic in Instantiator ---
from pddlstream.algorithms.instantiation import Instantiator
from pddlstream.language.stream import StreamInstance


# no-op out the two combination methods
Instantiator._add_combinations_relation = lambda self, stream, atoms: None
Instantiator._add_combinations          = lambda self, stream, atoms: None
StreamInstance.next_optimistic = lambda self: iter(())

# --- end patch ---

from pddlstream.language.constants import PDDLProblem, And, Atom
from pddlstream.algorithms.meta import solve

from simulation import SimulationEnvironment
from planners.pddlstream_interface import (
    STREAMS   as stream_info,
    extract_physical_world,
    ik_stream, cfree_config,
    motion_stream, traj_free, gen_loc_stream,
    WorldState,
)

# No Python‐side ACTIONS list—our PDDL actions come from domain.pddl
ACTIONS: List = []

# Cube side for height discretization
CUBE_SIDE = 0.01905


class Controller:
    def __init__(self,
                 domain_pddl: str = 'pddl/domain.pddl',
                 stream_pddl: str = 'pddl/streams.pddl',
                 base_names: List[str] = None,
                 base_poses: List[Tuple[Tuple[float,float,float],
                                        Tuple[float,float,float,float]]] = None):
        self.domain_pddl = domain_pddl
        self.stream_pddl = stream_pddl

        # default tower bases if none provided
        if base_names is None or base_poses is None:
            from evaluation import get_tower_base_poses
            poses = get_tower_base_poses()
            names = [f"base{chr(ord('A')+i)}" for i in range(len(poses))]
            self.base_pose_map = dict(zip(names, poses))
        else:
            self.base_pose_map = dict(zip(base_names, base_poses))

    def run(self,
            real_env: SimulationEnvironment,
            goal_poses: Dict[str, Tuple[Tuple[float,float,float],
                                         Tuple[float,float,float,float]]]):
        """
        1) Infer symbolic state from real_env
        2) Build a PDDLProblem
        3) solve() once
        4) validate via streams + execute; replan on any failure
        """
        while True:
            # (1) Symbolic map block→location
            symbolic_map = self._infer_symbolic_map_from_env(real_env)


            # EARLY EXIT: if every block is physically already at its goal, do nothing
            all_at_goal = True
            for b, (goal_loc, goal_quat) in goal_poses.items():
                # get actual pose
                actual_loc, actual_quat = real_env.get_block_pose(b)
                # check positional tolerance
                if any(abs(actual_loc[i] - goal_loc[i]) > CUBE_SIDE/2 for i in range(3)):
                    all_at_goal = False
                    break
                # check rotational tolerance (quaternion inner product)
                dot = sum(a*q for a,q in zip(actual_quat, goal_quat))
                if abs(abs(dot) - 1.0) > 1e-3:
                    all_at_goal = False
                    break
            if all_at_goal:
                return


            # grab the real‐world snapshot for streaming
            world = extract_physical_world(real_env)

            pddl_prob = self._make_pddlstream_problem(symbolic_map,
                                           goal_poses,
                                           world)
            # Now call solve with our StreamInfo metadata
            solutions = solve(
                pddl_prob,
                algorithm='adaptive',
                planner='ff-astar',
                stream_info=stream_info,   
                max_planner_time=10,
                verbose=False,
                debug=False,
            )
            solution = solutions[0]
            # (4) Validate & execute
            if self._validate_and_execute(real_env, solution):
                break

    def _make_pddlstream_problem(self,
                                 symbolic_map: Dict[str,str],
                                 goal_poses: Dict[str,Tuple],
                                 world: WorldState
                                 ) -> PDDLProblem:

        """
        Constructs and returns a PDDLProblem(domain, consts, stream_pddl,
                                            stream_map, init_list, goal_formula).
        """
        # Map the PDDL constants to Python objects:
        #  - 'w0' is the world snapshot for streams
        #  - 'nil' is our “no‐block” placeholder
        #  - each baseX constant just maps to its own name string
        constant_map = {
            'w0': world,
            'nil': None,
        }
        # bind all of your base names
        for base in self.base_pose_map:
            constant_map[base.lower()] = base

        # (a) Build init list
        init: List[Tuple] = []
        # At(...)
        for b, loc in symbolic_map.items():
            init.append(('At', b, loc))
            
            # # --- BOOTSTRAP STREAM FACTS FOR THIS AT(...) pair ---
            # # 1) Inverse kinematics: get one q
            # q, = ik_stream(world, b, loc, None)
            # init.append(('Kin', b, loc, q))
            # # 2) Collision‐free config
            # if cfree_config(world, q):
            #     init.append(('CFreeConf', q))
            # # 3) A trivial motion from q to itself
            # t0 = f"traj_{b}"
            # init.append(('Motion', q, t0, q))
            # init.append(('CFreeTraj', t0, b))

        # Clear(...) and Location(...)
        occupied = set(symbolic_map.values())
        for base in self.base_pose_map:
            # 1) Gather levels already in use on this base
            prefix = f"{base}_loc"
            levels = [
                int(loc.split(prefix, 1)[1])
                for loc in symbolic_map.values()
                if loc.startswith(prefix)
            ]
            # 2) Compute the highest level (or -1 if none)
            max_lvl = max(levels) if levels else -1

            # 3) Seed Above relations for every existing level, plus one extra link
            #    This makes both loc0→loc1 when max_lvl=-1,
            #    and locN→locN+1 for N up to the real max.
            for lvl in range(max_lvl + 1):
                lower = f"{base}_loc{lvl}"
                upper = f"{base}_loc{lvl+1}"
                init.append(('Above', lower, upper))

            # 4) Declare each spot as a valid Location and, if unoccupied, Clear
            for lvl in range(max_lvl + 2):
                loc_sym = f"{base}_loc{lvl}"
                init.append(('Location', loc_sym))
                if loc_sym not in occupied:
                    init.append(('Clear', loc_sym))

            # 5) Table‐support for level‐0
            init.append(('Base', base))
            init.append(('OnFloor', f"{base}_loc0"))


        init.append(('World', 'w0'))
        # Empty
        init.append(('Empty',))
        init.append(('Holding', 'nil'))

        # (b) Build goal formula
        goals: List[Tuple] = []
        for b, (loc, _) in goal_poses.items():
            goals.append(('At', b, loc))
        goal_formula = And(*goals)

        # load the PDDL text
        with open(self.domain_pddl) as f:
            domain_pddl_str = f.read()
        with open(self.stream_pddl) as f:
            stream_pddl_str = f.read()

    # 1) generator map: name → Python callable
        gen_map = {
            'ik'           : ik_stream,
            'cfree_config' : cfree_config,
            'motion'       : motion_stream,
            'traj_free'    : traj_free,
            'gen-loc'      : gen_loc_stream,
        }

        # build and return PDDLProblem from the actual PDDL text
        return PDDLProblem(
            domain_pddl_str,    # <-- now the file’s contents
            constant_map,
            stream_pddl_str,    # <-- also contents here
            gen_map,
            init,
            goal_formula,
        )
    def _infer_symbolic_map_from_env(self,
                                     env: SimulationEnvironment
                                     ) -> Dict[str,str]:
        """
        Map each block (except 'b0') to the nearest base_locN.
        """
        state = extract_physical_world(env)
        symbolic_map: Dict[str,str] = {}
        for b, (pos, _) in state.items():
            if b == 'b0':
                continue
            x,y,z = pos
            # pick nearest base in XY
            best_base, best_dist = None, float('inf')
            for base, (bpos, _) in self.base_pose_map.items():
                dx, dy = x - bpos[0], y - bpos[1]
                d = np.hypot(dx, dy)
                if d < best_dist:
                    best_base, best_dist = base, d
            # compute level
            base_z = self.base_pose_map[best_base][0][2]
            level = max(0, int(round((z - base_z)/CUBE_SIDE)))
            symbolic_map[b] = f"{best_base}_loc{level}"
        return symbolic_map

    def _validate_and_execute(self,
                              env: SimulationEnvironment,
                              plan: List[Tuple[str,Dict]]
                              ) -> bool:
        """
        Runs through each (action,params).  Uses our streams for
        feasibility checks.  Executes on success; returns False on any failure.
        """
        for action, params in plan:
            world = extract_physical_world(env)
            if not self._validate_action(world, action, params):
                return False
            self._execute_action(env, action, params)
        return True

    def _validate_action(self,
                         world: Dict[str,Tuple],
                         action: str,
                         params: Dict
                         ) -> bool:
        if action in ('pick','place'):
            return cfree_config(world, params['?q'])
        if action == 'move':
            # read the block you’re carrying directly from the PDDL binding
            b   = params['?b']
            q1  = params['?q1']
            q2  = params['?q2']

            # sample motion paths, motion_stream returns a list of paths
            for (path,) in motion_stream(world, q1, q2):
                # check that every waypoint is a collision-free robot configuration
                if not all(cfree_config(world, q) for q in path):
                    continue
                # check that this path is collision-free *with* block b
                if list(traj_free(world, path, b)):
                    return True

            return False
        return True

    def _execute_action(self,
                        env: SimulationEnvironment,
                        action: str,
                        params: Dict):
        """
        Execute pick/move/place via joint commands.
        Expects params['?q'] (or '?q2') to be a 6‑tuple of joint angles (radians)
        for the motors m1–m6.
        """
        # 1) Pick out the correct ?q
        if action == 'move':
            q = params['?q2']
        else:
            q = params['?q']

        # 2) Identify the actuated joints (non-fixed ones)
        motor_names = [
            name for (name, fixed) 
            in zip(env.joint_name, env.joint_fixed) 
            if not fixed
        ]
        assert len(q) == len(motor_names), \
            f"Expected {len(motor_names)} angles but got {len(q)}"

        # 3) Build the joint→angle(degrees) map
        angle_dict = {
            motor_names[i]: q[i] * (180.0/np.pi) 
            for i in range(len(q))
        }

        print(f"[EXECUTE] {action} → angles (deg): {angle_dict}")

        # 4) Command the arm
        env.goto_position(angle_dict, duration=1.0)



if __name__ == '__main__':
    env = SimulationEnvironment(show=True)
    # the user must supply a real goal map
    goal = {}
    ctrl = Controller()
    ctrl.run(env, goal)
    env.close()

