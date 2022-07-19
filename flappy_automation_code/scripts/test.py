# from numpy import array, dot
from qpsolvers import solve_qp

# M = array([[1., 2., 0.], [-8., 3., 2.], [0., 1., 1.]])
# P = dot(M.T, M)  # this is a positive definite matrix
# q = dot(array([3., 2., 3.]), M)
# G = array([[1., 2., 1.], [2., 0., 1.], [-1., 2., -1.]])
# h = array([3., 2., -2.])
# A = array([1., 1., 1.])
# b = array([1.])
# # print(qpsolvers.available_solvers)
# x = solve_qp(P, q, G, h, A, b, solver="osqp")
# print("QP solution: x = {}".format(x))

import numpy as np

DELTA_T = 1/30
AX_MAX = 3
AY_MAX = 35 # m/s2

# MPC constants
N = 20
A = np.array([[1, -DELTA_T], [0, 1]])
B = np.array([[-DELTA_T**2/2],[DELTA_T]])

Sx = np.zeros((2*N+2, 2))
Su = np.zeros((2*N+2, N))
for iSx in range(N+1):
    Sx[2*iSx:2*iSx+2, :] = np.linalg.matrix_power(A, iSx)
    for iSu in range(iSx):
        Su[2*iSx:2*iSx+2, iSu:iSu+1] = np.dot(np.linalg.matrix_power(A, iSx-1-iSu), B)

# Q_bar = np.diag(100*np.ones(2*N+2))
Q_bar = np.diag(np.zeros(2*N+2))
for iQ_bar in range(N+1):
    Q_bar[2*iQ_bar:2*iQ_bar+2, 2*iQ_bar:2*iQ_bar+2] = np.array([[iQ_bar,0],[0,0.01]])
Q_bar[-1,-1] = N
R_bar = np.diag(np.zeros(N))

h = AY_MAX*np.ones((2*N,1))
G = np.zeros((2*N, N))
for iG in range(N):
    G[2*iG:2*iG+2, iG:iG+1] = np.array([[1], [-1]])

H = np.dot(np.dot(Su.T, Q_bar), Su) + R_bar
F = np.dot(np.dot(Sx.T, Q_bar), Su)
# Dy = 1.4805565196861985 
# Vy = 1.1666088188280848
Dy = 1.4222263857099602
Vy = 2.3331992197462146

x0 = np.array([[Dy],[Vy]])
q = np.dot(F.T, x0)

# AA = np.array([1., 1., 1.])
# bb = np.array([1.])
u = solve_qp(H, q, G, h, None, None, solver="osqp")
u = u.reshape(N,1)
print(u)
xs = np.dot(Sx,x0) + np.dot(Su,u)
xs = xs.reshape(2*N+2)[::2]
try:
    n_time_gaps = np.where(np.abs(xs)<=0.02)[0][0]
    t_approx = n_time_gaps*DELTA_T
except:
    t_approx = 1.5*N*DELTA_T
# print("QP solution: u = {}".format(u))
print(1.5-xs)
print(t_approx)
# print(type(u))
# print(u.shape)