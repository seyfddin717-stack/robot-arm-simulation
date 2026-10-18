import sys
import os
import numpy as np
import matplotlib.pyplot as plt
from matplotlib.animation import FuncAnimation

sys.path.append(os.path.join(os.path.dirname(__file__), ".."))
from kinematics import forward_kinematics, inverse_kinematics

# Start and end points of the straight line
start = np.array([1.5, 0.5])
end = np.array([-0.5, 1.2])
N = 60
s = np.linspace(0, 1, N)
points = start[None, :] + s[:, None] * (end - start)[None, :]

# Joint angles along the path
angles = np.array([inverse_kinematics(x, y) for x, y in points])
t1s = np.unwrap(angles[:, 0])
t2s = np.unwrap(angles[:, 1])
deg1, deg2 = np.degrees(t1s), np.degrees(t2s)

fig, (ax, ax2) = plt.subplots(1, 2, figsize=(10, 4.5))

ax.set_xlim(-2.2, 2.2)
ax.set_ylim(-0.5, 2.2)
ax.set_aspect("equal")
ax.grid(True)
ax.plot(*zip(start, end), "r--", lw=1, label="target line")
ax.legend(loc="lower left")
arm, = ax.plot([], [], "o-", lw=4)
ax.set_title("Straight-line motion")

ax2.plot(range(N), deg1, label="theta1")
ax2.plot(range(N), deg2, label="theta2")
ax2.set_xlabel("step")
ax2.set_ylabel("angle (deg)")
ax2.grid(True)
ax2.legend()
ax2.set_title("Joint angles")
m1, = ax2.plot([], [], "ko")
m2, = ax2.plot([], [], "ko")


def update(i):
    (x1, y1), (x2, y2) = forward_kinematics(t1s[i], t2s[i])
    arm.set_data([0, x1, x2], [0, y1, y2])
    m1.set_data([i], [deg1[i]])
    m2.set_data([i], [deg2[i]])
    return arm, m1, m2


anim = FuncAnimation(fig, update, frames=N, interval=50)
anim.save("line.gif", writer="pillow", fps=20)
plt.show()
