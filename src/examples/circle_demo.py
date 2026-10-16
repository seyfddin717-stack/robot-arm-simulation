import sys
import os
import numpy as np
import matplotlib.pyplot as plt
from matplotlib.animation import FuncAnimation

sys.path.append(os.path.join(os.path.dirname(__file__), "..", "src"))
from kinematics import forward_kinematics, inverse_kinematics

fig, ax = plt.subplots()
ax.set_xlim(-2.2, 2.2)
ax.set_ylim(-2.2, 2.2)
ax.set_aspect("equal")
ax.grid(True)
arm, = ax.plot([], [], "o-", lw=4)
trail, = ax.plot([], [], "r-", lw=1)
xs, ys = [], []


def update(i):
    a = np.radians(i * 4)
    x, y = 1.0 + 0.5 * np.cos(a), 0.5 + 0.5 * np.sin(a)
    t1, t2 = inverse_kinematics(x, y)
    (x1, y1), (x2, y2) = forward_kinematics(t1, t2)
    arm.set_data([0, x1, x2], [0, y1, y2])
    xs.append(x2)
    ys.append(y2)
    trail.set_data(xs, ys)
    return arm, trail


anim = FuncAnimation(fig, update, frames=90, interval=50)
anim.save("arm.gif", writer="pillow", fps=20)
plt.show()
