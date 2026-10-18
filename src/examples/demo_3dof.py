import sys
import os
import numpy as np
import matplotlib.pyplot as plt
from matplotlib.animation import FuncAnimation

sys.path.append(os.path.join(os.path.dirname(__file__), ".."))
from kinematics_3dof import forward_kinematics_3dof, inverse_kinematics_3dof

# The tool tip moves along a horizontal line while the tool keeps pointing down.
PHI = np.radians(-90)
N = 80
xs_target = np.linspace(-1.0, 1.0, N)
y_target = 1.2

angles = np.array([inverse_kinematics_3dof(x, y_target, PHI) for x in xs_target])
t1s, t2s, t3s = np.unwrap(angles, axis=0).T

fig, ax = plt.subplots(figsize=(6, 6))
ax.set_xlim(-2.5, 2.5)
ax.set_ylim(-1.0, 2.5)
ax.set_aspect("equal")
ax.grid(True)
ax.plot([-1.0, 1.0], [y_target, y_target], "r--", lw=1, label="target line")
ax.legend(loc="lower left")
ax.set_title("3-DOF arm: tool always points down")
arm, = ax.plot([], [], "o-", lw=4)
tool, = ax.plot([], [], "-", lw=6, color="tab:orange")


def update(i):
    (x1, y1), (x2, y2), (x3, y3) = forward_kinematics_3dof(t1s[i], t2s[i], t3s[i])
    arm.set_data([0, x1, x2], [0, y1, y2])
    tool.set_data([x2, x3], [y2, y3])
    return arm, tool


anim = FuncAnimation(fig, update, frames=N, interval=50)
anim.save("arm3.gif", writer="pillow", fps=20)
plt.show()
