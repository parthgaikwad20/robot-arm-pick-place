import math

import pytest

from arm_pick_place.kinematics import forward, inverse, UnreachableError


@pytest.mark.parametrize('target', [
    (0.30, 0.10, 0.02), (0.10, -0.30, 0.15), (0.20, 0.0, 0.30), (0.40, 0.0, 0.02),
])
def test_ik_fk_roundtrip(target):
    q = inverse(*target)
    assert all(abs(a - b) < 1e-9 for a, b in zip(forward(q), target))
    # gripper must point straight down
    assert math.isclose(q[1] + q[2] + q[3], math.pi)


def test_out_of_reach():
    with pytest.raises(UnreachableError):
        inverse(1.0, 0.0, 0.0)
