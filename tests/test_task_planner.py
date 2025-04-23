import pytest

from run_planner import plan_task
from planners.pddlstream_interface import STREAMS, ACTIONS


def test_plan_task_basic_sequence():
    """
    Verify that plan_task on the provided PDDL files returns a non-empty plan
    containing at least 'pick' and 'place' actions.
    """
    # Use the sample pddl files in the repo
    domain_pddl = 'pddl/domain.pddl'
    problem_pddl = 'pddl/problem.pddl'
    # Generate the plan
    plan = plan_task(domain_pddl, problem_pddl, STREAMS, ACTIONS)

    # The plan should be a non-empty list
    assert isinstance(plan, list), "plan_task should return a list"
    assert len(plan) > 0, "plan_task returned an empty plan"

    # Extract action names
    action_names = [action for action, _ in plan]

    # Ensure basic manipulation actions are present
    assert 'pick' in action_names, f"Expected 'pick' in plan, got {action_names}"
    assert 'place' in action_names, f"Expected 'place' in plan, got {action_names}"
