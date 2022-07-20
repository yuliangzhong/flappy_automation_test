import numpy as np
import math
from qpsolvers import solve_qp


# system constants
DELTA_T = 1/30
AX_MAX = 3
AY_MAX = 35 # m/s2

LASER_NUM = 9
LASER_RANGE = 3.549 # m
ANGLES = np.linspace(-45,45,9)/180*math.pi # radius

OBSTACLE_WIDTH_SUP = 0.60 # m
OBSTACLE_GAP = 1.92 # m
GATE_HEIGHT = 0.50 # m
UP_LOW_FENCE = 0.7 # m

T_REACH_MAX = 0.65 # s

# y-axis MPC control constants
N = 15
px = 5.63
vx = 2.59
r = 5.63+1.05

# x_+ = Ax + Bu
A = np.array([[1, DELTA_T], [0, 1]])
B = np.array([[DELTA_T**2/2],[DELTA_T]])
# Ax*x_i <= bx; Au*u_i <= bu; Af*x_N <= bf
Ax = np.array([[0, -1]])
bx = np.array([[0]])
Au = np.array([[1], [-1]])
bu = np.array([[AX_MAX], [AX_MAX]])
Af = np.array([[1, 0],[0, -1]])
bf = np.array([[0], [0]])
# Loss = |xN|_P + sum(|xi|_Q + |ui|_R)
P = np.array([[1, 0], [0, 0]])
Q = np.zeros((2,2))
R = np.array([[0]])

# with substitution
# Sx = [[I], [A], [A^2], ..., [A^N]]
# Su = [[0, ..., 0], [B, 0, ..., 0], [AB, B, 0, ..., 0], ..., [A^(N-1)B, ..., AB, B]]
Sx = np.zeros((2*N+2, 2))
Su = np.zeros((2*N+2, N))
for i in range(N+1):
    Sx[2*i:2*i+2, :] = np.linalg.matrix_power(A, i)
    for j in range(i):
        Su[2*i:2*i+2, j:j+1] = np.dot(np.linalg.matrix_power(A, i-j-1), B)

# Q_bar = diag(Q, Q, ..., Q, P)
# R_bar = diag(R, R, ..., R)
Q_bar = np.diag(np.zeros(2*N+2))
Q_bar[-2, -2] = 1
R_bar = np.diag(np.zeros(N))

# constraints: G*x <= w + E*x(0)
Gu = np.zeros((2*N, N))
for i in range(N):
    Gu[2*i:2*i+2, i:i+1] = Au
AX = np.zeros((N, 2*N))
for i in range(N):
    AX[i:i+1, 2*i:2*i+2] = Ax
Gx = np.dot(AX, Su[0:2*N, :])
Gf = np.dot(Af, Su[2*N:2*N+2, :])
G = np.vstack((Gu, Gx, Gf))

wu = AX_MAX*np.ones((2*N, 1))
wx = np.zeros((N, 1))
wf = np.zeros((2,1))
w = np.vstack((wu, wx, wf))

Eu = np.zeros((2*N, 2))
Ex = -np.dot(AX, Sx[0:2*N, :])
Ef = -np.dot(Af, Sx[2*N:2*N+2, :])
E = np.vstack((Eu, Ex, Ef))

H = np.dot(np.dot(Su.T, Q_bar), Su) + R_bar
F = np.dot(np.dot(Sx.T, Q_bar), Su)

x0 = np.array([[px-r], [vx]])
h = w + np.dot(E, x0)
q = np.dot(F.T, x0)
u = solve_qp(H, q, G, h, solver="osqp")

u = u.reshape(N,1)
xs = np.dot(Sx,x0) + np.dot(Su,u)

print(u)
print(xs)
