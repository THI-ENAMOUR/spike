import unittest

from core.model.action.atomic.no_op_action import NoOpAction
from core.model.action.group.action_group import ActionGroup
from core.validation.action_group_validator import ActionGroupValidator
from error.validation_error import ValidationError
from tests.app_test import AppTestCases


class TestActionGroupValidator(AppTestCases.AppTest):
    def test_validate_action_group_start_time_sorting(self):
        # Positive testing
        ActionGroupValidator.validate_action_group_start_time_sorting([])  # No error expected
        actions = [
            NoOpAction(start_ms=100),
            NoOpAction(start_ms=300),
            NoOpAction(start_ms=300),
            NoOpAction(start_ms=1000),
        ]
        ActionGroupValidator.validate_action_group_start_time_sorting(actions)  # No error expected

        # Negative testing
        actions = [NoOpAction(start_ms=200), NoOpAction(start_ms=100)]
        self.assertRaises(ValidationError, ActionGroupValidator.validate_action_group_start_time_sorting, actions)
        action_group = ActionGroup(actions=actions)
        # No error expected, since the action group should automatically sort actions by start time
        ActionGroupValidator.validate_action_group_start_time_sorting(action_group.actions)

        # Complex positive testing
        actions = [
            NoOpAction(start_ms=100),
            NoOpAction(start_ms=100),
            ActionGroup(
                start_ms=130,
                actions=[
                    NoOpAction(start_ms=30),
                    ActionGroup(start_ms=40, actions=[NoOpAction(start_ms=0)]),
                    NoOpAction(start_ms=50),
                ],
            ),
        ]
        ActionGroupValidator.validate_action_group_start_time_sorting(actions)  # No error expected

        # Complex negative testing
        actions = [
            NoOpAction(start_ms=100),
            NoOpAction(start_ms=100),
            ActionGroup(
                start_ms=80,
                actions=[
                    NoOpAction(start_ms=30),
                    ActionGroup(start_ms=20, actions=[NoOpAction(start_ms=0)]),
                    NoOpAction(start_ms=10),
                ],
            ),
        ]
        action_group = ActionGroup(actions=actions)
        # No error expected, since the action group should automatically sort actions by start time
        ActionGroupValidator.validate_action_group_start_time_sorting(action_group.actions)


if __name__ == "__main__":
    unittest.main()
