import rospy
import numpy as np
from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import Vector3

import math
from qpsolvers import solve_qp
import time

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

# y-axis MPC control constants
N = 20
A = np.array([[1, -DELTA_T], [0, 1]])
B = np.array([[-DELTA_T**2/2],[DELTA_T]])

Sx = np.zeros((2*N+2, 2))
Su = np.zeros((2*N+2, N))
for iSx in range(N+1):
    Sx[2*iSx:2*iSx+2, :] = np.linalg.matrix_power(A, iSx)
    for iSu in range(iSx):
        Su[2*iSx:2*iSx+2, iSu:iSu+1] = np.dot(np.linalg.matrix_power(A, iSx-1-iSu), B)

Q_bar = np.diag(np.zeros(2*N+2))
for iQ_bar in range(N+1):
    Q_bar[2*iQ_bar:2*iQ_bar+2, 2*iQ_bar:2*iQ_bar+2] = np.array([[iQ_bar,0],[0,0.01]])
R_bar = np.diag(np.zeros(N))

h = AY_MAX*np.ones((2*N,1))
G = np.zeros((2*N, N))
for iG in range(N):
    G[2*iG:2*iG+2, iG:iG+1] = np.array([[1], [-1]])

H = np.dot(np.dot(Su.T, Q_bar), Su) + R_bar
F = np.dot(np.dot(Sx.T, Q_bar), Su)

# solve y-axis acceleration by MPC --> QP

goal_y = -0.01
pos_y =  -0.01
Vy =  0.00

Dy = goal_y - pos_y
x0 = np.array([[Dy],[Vy]])
q = np.dot(F.T, x0)
u = solve_qp(H, q, G, h, solver="osqp")
ay = u[0]
u = u.reshape(N,1)
xs = np.dot(Sx,x0) + np.dot(Su,u)
xs = xs.reshape(2*N+2)[::2]
try:
    n_time_gaps = N - np.where(np.abs(xs)[::-1]>0.02)[0][0] + 1
    t_approx = n_time_gaps*DELTA_T
except:
    t_approx = 0.0

print("u = ")
print(u)

print("y = ")
print(goal_y - xs)

print("a = ,t = ")
print(ay, t_approx)