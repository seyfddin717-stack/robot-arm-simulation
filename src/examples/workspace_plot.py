import numpy as np
import matplotlib.pyplot as plt

L1, L2 = 1.2, 0.8


def forward_kinematics(t1, t2, l1=L1, l2=L2):
    x1, y1 = l1 * np.cos(t1), l1 * np.sin(t1)
    x2, y2 = x1 + l2 * np.cos(t1 + t2), y1 + l2 * np.sin(t1 + t2)
    return (x1, y1), (x2, y2)


t1, t2 = np.meshgrid(np.radians(np.arange(0, 360, 2)),
                     np.radians(np.arange(-180, 180, 2)))
_, (x, y) = forward_kinematics(t1, t2)

fig, ax = plt.subplots(figsize=(6, 6))
ax.scatter(x, y, s=1, alpha=0.3)

a = np.linspace(0, 2 * np.pi, 200)
for r, label in [(L1 + L2, "max reach"), (abs(L1 - L2), "min reach")]:
    ax.plot(r * np.cos(a), r * np.sin(a), "r--", lw=1, label=f"{label} = {r:.1f}")

ax.set_aspect("equal")
ax.grid(True)
ax.legend()
ax.set_title("Workspace of 2-DOF arm")
plt.savefig("workspace.png", dpi=150)
