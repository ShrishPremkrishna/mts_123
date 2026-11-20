"""Control graph checks through the same strict/zero sequence as a disarmed runner."""
from unittest.mock import patch

from final_project.behaviors import combined_fc_runner as runner
from final_project.behaviors.test_combined_fc_runner import path_fixture
from final_project.pupper_fc.model import Command


def test_idle_graph_checks_are_bounded_and_recheck_conflicting_ownership():
    path, now, published, pubs, _, endpoint = path_fixture()
    with patch.object(runner.evidence, "guard_publishers_problem",
                      wraps=runner.evidence.guard_publishers_problem) as scans:
        for tick in range(50):
            now[0] = 10. + .02 * tick
            path.check(False)
            path.write(Command(expires_at=now[0] + .15))
        assert 10 <= scans.call_count <= 20
        assert len(published) == 50

        pubs[runner.protocol.REQUEST_TOPIC].append(endpoint("competitor"))
        now[0] += .101
        assert path.check(False)[0] == "authority"
        path.write(Command(expires_at=now[0] + .15))
        assert len(published) == 50
