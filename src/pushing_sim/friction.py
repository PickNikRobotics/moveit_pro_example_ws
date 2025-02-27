import cvxpy as cp
import numpy as np

T = np.array([
    [1, 1, 1, 1, 1, 1],  # 2.5
    [1, 1, 1, 1, 1, 1],
    [0, 0, 1, 1, 0, 0],  # ~0.8
    [0, 0, 1, 1, 0, 0],
    [0, 0, 1, 1, 0, 0],
    [0, 0, 1, 1, 0, 0],  # -2.5
])


def get_points(T):
    points = []
    com = np.zeros(2, )
    for y_ind in range(T.shape[0]):
        for x_ind in range(T.shape[1]):
            if T[y_ind, x_ind] > 0:
                arr = np.array([x_ind - T.shape[1] / 2 + .5, -(y_ind - T.shape[0] / 2) - .5])
                points.append(arr)
                com += arr

    return points, com / len(points)


points, com = get_points(T)

def get_eqs(point, com):
    diff = np.hstack([point - com, 0.0])
    rot_axis = np.array([0, 0, 1])
    rot_vel = np.cross(diff, rot_axis)
    row_x = [1, 0, rot_vel[0]]  # (p-com)w_x + xd
    row_y = [0, 1, rot_vel[1]]  # (p-com)w_x + xd
    return row_x, row_y


rows = []
for point in points:
    row_x, row_y = get_eqs(point, com)
    rows.append(row_x)
    rows.append(row_y)



# Define M (linear transformation matrix) and a (target vector)
M = np.vstack(rows)  # Example M matrix (3x2)
a = np.zeros(len(rows))  # Example target vector (size 3)

print("M", M)

# Compute Q and c for the QP formulation
Q = 2 * (M.T @ M)  # Q = 2 * M^T M
c = -2 * (M.T @ a)  # c = -2 * M^T a


con_x, con_y = get_eqs(np.array([0.5, -2.5]), com)
# A = np.vstack([con_x, con_y])  # Constraint coefficients
# b = np.array([[-1.0], [0]])  # Constraint bounds
A = np.vstack([con_x])  # Constraint coefficients
b = np.array([[-1.0]])  # Constraint bounds

print("A", A)

# Define variables
# vars to estimate xd, yd, thetad
x = cp.Variable(3)

# Define objective function (quadratic)
objective = cp.Minimize(0.5 * cp.quad_form(x, Q) + c @ x)

# Define constraints
constraints = [A @ x <= b, A @ x >= b]  # x should also be non-negative

# Solve the problem
prob = cp.Problem(objective, constraints)
result = prob.solve()

# Output results
print("Optimal value:", result)
print("Optimal x:", x.value)

print(A @ x.value)
