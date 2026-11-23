"""Forward / inverse kinematics for the simple 4-DOF arm in urdf/arm.urdf.

The arm is a base yaw joint followed by three pitch joints (shoulder, elbow,
wrist) that all bend in the same vertical plane. At zero angles the arm points
straight up. A positive pitch tilts the arm towards the direction the base is
facing.

For pick-and-place we keep the gripper pointing straight down, which turns the
problem into a planar 2-link IK plus a base rotation -- solvable in closed form.
No ROS imports here, so this file can be unit-tested with plain Python.
"""
import math

# Must match urdf/arm.urdf
H = 0.10   # base -> shoulder axis height
L1 = 0.25  # shoulder -> elbow
L2 = 0.20  # elbow -> wrist
L3 = 0.10  # wrist -> grasp point (tool_link)


class UnreachableError(ValueError):
    pass


def forward(q):
    """Joint angles [q1..q4] -> tool position (x, y, z) in base_link."""
    q1, q2, q3, q4 = q
    a2, a3, a4 = q2, q2 + q3, q2 + q3 + q4
    r = L1 * math.sin(a2) + L2 * math.sin(a3) + L3 * math.sin(a4)
    z = H + L1 * math.cos(a2) + L2 * math.cos(a3) + L3 * math.cos(a4)
    return (r * math.cos(q1), r * math.sin(q1), z)


def inverse(x, y, z, elbow_up=True):
    """Tool position (gripper pointing down) -> joint angles [q1..q4]."""
    q1 = math.atan2(y, x)
    r = math.hypot(x, y)

    # wrist centre sits L3 above the grasp point because the tool points down
    a = r              # horizontal distance shoulder -> wrist
    b = z + L3 - H     # vertical distance shoulder -> wrist
    d2 = a * a + b * b
    c3 = (d2 - L1 * L1 - L2 * L2) / (2 * L1 * L2)
    if not -1.0 <= c3 <= 1.0:
        raise UnreachableError(
            f"target ({x:.3f}, {y:.3f}, {z:.3f}) is out of reach")

    q3 = math.acos(c3) * (1 if elbow_up else -1)
    alpha = math.atan2(a, b)  # angle of shoulder->wrist line from vertical
    q2 = alpha - math.atan2(L2 * math.sin(q3), L1 + L2 * math.cos(q3))
    q4 = math.pi - q2 - q3    # total pitch = 180 deg -> gripper points down
    return [q1, q2, q3, q4]
