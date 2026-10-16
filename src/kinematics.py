import numpy as np

L1, L2 = 1.0, 1.0


def forward_kinematics(t1, t2, l1=L1, l2=L2):
    """Return elbow and end-effector positions (angles in radians)."""
    x1, y1 = l1 * np.cos(t1), l1 * np.sin(t1)
    x2, y2 = x1 + l2 * np.cos(t1 + t2), y1 + l2 * np.sin(t1 + t2)
    return (x1, y1), (x2, y2)


def inverse_kinematics(x, y, l1=L1, l2=L2, elbow_up=True):
    """Return joint angles (radians) that place the tip at (x, y)."""
    c2 = (x**2 + y**2 - l1**2 - l2**2) / (2 * l1 * l2)
    if abs(c2) > 1:
        raise ValueError("Target is outside the workspace")
    t2 = np.arccos(c2) if elbow_up else -np.arccos(c2)
    t1 = np.arctan2(y, x) - np.arctan2(l2 * np.sin(t2), l1 + l2 * np.cos(t2))
    return t1, t2
