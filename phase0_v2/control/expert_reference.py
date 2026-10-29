"""Explicit unavailable expert: never relabel measured or commanded force as truth."""
from .interface import AssistTorqueReference


def expert_reference(timestamp):
    return AssistTorqueReference(timestamp, None, False,
        'No validated object-wrench allocator/tracker; expert target not generated.')


EXPERT_INPUT_CONTRACT = {
    'privileged_allowed': ['human_goal', 'human_target_trajectory', 'human_applied_wrench',
                          'external_disturbance_wrench', 'object_dynamics', 'contact_limits'],
    'proposed_objective': 'goal tracking + human effort + robot effort + torque smoothness',
    'implementation_status': 'interface only; weights, optimality and reachable references unvalidated',
}
