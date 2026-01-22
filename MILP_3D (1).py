import pulp
import numpy as np
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D

# ------------------ GRID SETUP ------------------
grid_size = 5
drones = [0, 1]

obstacles = [
    (1, 2, 1), (3, 3, 2),
    (1, 2, 0), (1, 2, 2),
    (3, 3, 1), (3, 3, 0)
]

start = {0: (0, 0, 0), 1: (0, 4, 0)}
end   = {0: (4, 4, 4), 1: (4, 0, 4)}

# ------------------ PROBLEM ------------------
prob = pulp.LpProblem("Multi_UAV_3D_Path_Planning", pulp.LpMinimize)

# ------------------ VARIABLES ------------------
p = pulp.LpVariable.dicts(
    "p",
    ((d, i, j, k, ni, nj, nk)
     for d in drones
     for i in range(grid_size)
     for j in range(grid_size)
     for k in range(grid_size)
     for ni in range(grid_size)
     for nj in range(grid_size)
     for nk in range(grid_size)),
    cat="Binary"
)

u = pulp.LpVariable.dicts(
    "u",
    ((d, i, j, k)
     for d in drones
     for i in range(grid_size)
     for j in range(grid_size)
     for k in range(grid_size)),
    cat="Binary"
)

# ------------------ OBJECTIVE ------------------
prob += pulp.lpSum(p[d, i, j, k, ni, nj, nk] for d, i, j, k, ni, nj, nk in p)

# ------------------ ADJACENCY ------------------
def is_adjacent(a, b, c, x, y, z):
    return abs(a - x) + abs(b - y) + abs(c - z) == 1

# ------------------ OBSTACLE AVOIDANCE ------------------
for d in drones:
    for (i, j, k) in obstacles:
        for ni in range(grid_size):
            for nj in range(grid_size):
                for nk in range(grid_size):
                    if is_adjacent(i, j, k, ni, nj, nk):
                        prob += p[d, i, j, k, ni, nj, nk] == 0
                        prob += p[d, ni, nj, nk, i, j, k] == 0
        prob += u[d, i, j, k] == 0

# ------------------ FLOW CONSTRAINTS ------------------
for d in drones:
    si, sj, sk = start[d]
    ei, ej, ek = end[d]

    for i in range(grid_size):
        for j in range(grid_size):
            for k in range(grid_size):

                if (i, j, k) in obstacles:
                    continue

                outgoing = pulp.lpSum(
                    p[d, i, j, k, ni, nj, nk]
                    for ni in range(grid_size)
                    for nj in range(grid_size)
                    for nk in range(grid_size)
                    if is_adjacent(i, j, k, ni, nj, nk)
                )

                incoming = pulp.lpSum(
                    p[d, ni, nj, nk, i, j, k]
                    for ni in range(grid_size)
                    for nj in range(grid_size)
                    for nk in range(grid_size)
                    if is_adjacent(ni, nj, nk, i, j, k)
                )

                if (i, j, k) == (si, sj, sk):
                    prob += outgoing == 1
                    prob += incoming == 0
                elif (i, j, k) == (ei, ej, ek):
                    prob += outgoing == 0
                    prob += incoming == 1
                else:
                    prob += outgoing == u[d, i, j, k]
                    prob += incoming == u[d, i, j, k]

# ------------------ INTER-DRONE NODE CONFLICT ------------------
for i in range(grid_size):
    for j in range(grid_size):
        for k in range(grid_size):
            if (i, j, k) in obstacles:
                continue
            prob += pulp.lpSum(u[d, i, j, k] for d in drones) <= 1

# ------------------ INTER-DRONE EDGE CONFLICT ------------------
for d1 in drones:
    for d2 in drones:
        if d1 >= d2:
            continue
        for i in range(grid_size):
            for j in range(grid_size):
                for k in range(grid_size):
                    for ni in range(grid_size):
                        for nj in range(grid_size):
                            for nk in range(grid_size):
                                if is_adjacent(i, j, k, ni, nj, nk):
                                    prob += (
                                        p[d1, i, j, k, ni, nj, nk]
                                        + p[d2, ni, nj, nk, i, j, k]
                                        <= 1
                                    )

# ------------------ SOLVE ------------------
prob.solve(pulp.PULP_CBC_CMD(msg=0))

# ------------------ PATH EXTRACTION ------------------
paths = {}

for d in drones:
    current = start[d]
    path = [current]

    while current != end[d]:
        i, j, k = current
        found = False
        for ni in range(grid_size):
            for nj in range(grid_size):
                for nk in range(grid_size):
                    if is_adjacent(i, j, k, ni, nj, nk):
                        if pulp.value(p[d, i, j, k, ni, nj, nk]) == 1:
                            current = (ni, nj, nk)
                            path.append(current)
                            found = True
                            break
                if found:
                    break
            if found:
                break
    paths[d] = path

# ------------------ PLOTTING ------------------
fig = plt.figure()
ax = fig.add_subplot(111, projection="3d")

# Obstacles
for (i, j, k) in obstacles:
    ax.bar3d(j, i, k, 1, 1, 1, color="red", alpha=0.5)

colors = ["blue", "cyan"]

for d in drones:
    xs = [p[1] + 0.5 for p in paths[d]]
    ys = [p[0] + 0.5 for p in paths[d]]
    zs = [p[2] + 0.5 for p in paths[d]]

    ax.plot(xs, ys, zs, marker="o", linewidth=2,
            color=colors[d], label=f"Drone {d}")

ax.set_xlim(0, grid_size)
ax.set_ylim(0, grid_size)
ax.set_zlim(0, grid_size)
ax.set_xlabel("X")
ax.set_ylabel("Y")
ax.set_zlabel("Z")
ax.set_title("Collision-Free 3D Multi-Drone Path Planning")
ax.legend()
plt.show()
