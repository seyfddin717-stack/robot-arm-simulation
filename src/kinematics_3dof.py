import numpy as np

from kinematics import inverse_kinematics

L1, L2, L3 = 1.0, 1.0, 0.5


def forward_kinematics_3dof(t1, t2, t3, l1=L1, l2=L2, l3=L3):
    """Return elbow, wrist and tool-tip positions (angles in radians)."""
    x1, y1 = l1 * np.cos(t1), l1 * np.sin(t1)
    x2, y2 = x1 + l2 * np.cos(t1 + t2), y1 + l2 * np.sin(t1 + t2)
    x3, y3 = x2 + l3 * np.cos(t1 + t2 + t3), y2 + l3 * np.sin(t1 + t2 + t3)
    return (x1, y1), (x2, y2), (x3, y3)


def inverse_kinematics_3dof(x, y, phi, l1=L1, l2=L2, l3=L3, elbow_up=True):
    """Return joint angles (radians) for tip position (x, y) and tool angle phi.

    The tool orientation phi (angle of the last link from the x-axis) is chosen
    by the user. This removes the redundancy: the wrist point is found first,
    then the 2-link solution is reused for the first two joints.
    """
    xw = x - l3 * np.cos(phi)
    yw = y - l3 * np.sin(phi)
    t1, t2 = inverse_kinematics(xw, yw, l1, l2, elbow_up)
    t3 = phi - t1 - t2
    return t1, t2, t3
