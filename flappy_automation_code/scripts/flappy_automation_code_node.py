#!/usr/bin/env python3
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

OBSTACLE_GAP = 1.92 # m
GATE_HEIGHT = 0.50 # m

# my constants
OBSTACLE_WIDTH_SUP = 0.60 # m
UP_LOW_FENCE = 0.75 # m
L_BACK = 0.20 # m
L_FORWARD = 0.15 # m

T_REACH_MAX = 17 * DELTA_T # s
FREE_X = 1.15 # m
VX_SAFE = FREE_X / T_REACH_MAX + AX_MAX * T_REACH_MAX / 2
VX_DANGER = 4.70 # m/s

# y-axis MPC control constants
N = 25
# x_+ = Ax + Bu
A = np.array([[1, -DELTA_T], [0, 1]])
B = np.array([[-DELTA_T**2/2],[DELTA_T]])

# Ax*x_i <= bx; Au*u_i <= bu; Af*x_N <= bf
Au = np.array([[1], [-1]])
# Loss = |xN|_P + sum(|xi|_Q + |ui|_R)

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
for i in range(N+1):
    Q_bar[2*i:2*i+2, 2*i:2*i+2] = np.array([[i,0],[0,0.01]])
R_bar = np.diag(np.zeros(N))

# constraints: G*x <= w + E*x(0)
h = AY_MAX*np.ones((2*N,1))
G = np.zeros((2*N, N))
for i in range(N):
    G[2*i:2*i+2, i:i+1] = Au

H = np.dot(np.dot(Su.T, Q_bar), Su) + R_bar
F = np.dot(np.dot(Sx.T, Q_bar), Su)


class Agent(object):
    def __init__(self):
        # Publisher for sending acceleration commands to flappy bird
        self.pub_acc_cmd = rospy.Publisher('/flappy_acc', Vector3, queue_size=1)
        # Subscribe to topics for velocity and laser scan from Flappy Bird game
        self.vel_sub = rospy.Subscriber("/flappy_vel", Vector3, self.velCallback)
        self.laser_sub = rospy.Subscriber("/flappy_laser_scan", LaserScan, self.laserScanCallback)

        # calculate the bird position by velocity integral
        self.pos_x = 0.0
        self.pos_y = 0.0
        self.dx = 0.0
        self.dy = 0.0
        self.vel_buffer = Vector3()

        # define the y-axis goal
        self.goal_y = 0.0
        self.goal_y_reach_count = 0

        # position up bound and low bound
        self.pos_up = 0.0
        self.pos_up_flag = 0 # 0: not set; 1: set
        self.pos_low = 0.0
        self.pos_low_flag = 0 # 0: not set; 1: set

        # obstacle memory
        # ----- self.pos_up -----
        # -min-|forward|-max-       -min-|backward|-max-
        # ----- self.pos_low -----
        # 0-free; 1-stone
        self.resolution = 300
        self.obstacleMemoryReset('f') # 'f': forward
        self.obstacleMemoryReset('b') # 'b': backward

        # the obstacle in front of the bird, or the bird is going through
        self.front_obstacle_update_flag = 1
        self.front_obstacle_max_x = 0
        self.front_obstacle_min_x = 999

        # the bird status
        self.status = 'init' # 'init', 'free', 'constrained', 'forecast'

        # forecast count
        self.forecast_count = 0.0

        # print_out
        self.time_count = 0
        self.max_count = 0
        self.max_running_time = 0.0


    def velCallback(self, msg):
        self.vel_buffer = msg


    def laserScanCallback(self, msg):
        
        start_time = time.time()
        
        # position integral
        self.pos_x += self.dx
        self.pos_y += self.dy
        
        # set tunnel up and low bound
        if self.pos_up_flag==0 or self.pos_low_flag==0:
            self.setUpLowBound(msg)

        else:
            # update obstacle memory
            self.obstacleUpdate(msg)

            # update bird status
            self.setBirdStatus()

        # take actions based on current bird status
        self.setGoalY()
        self.setAcc()

        end_time = time.time()
        dt = end_time - start_time
        self.max_running_time = max(self.max_running_time, dt)
        print("Max total time cost: %.7f" % (self.max_running_time)) # < 10ms
        print("----------------------------------------")

    def obstacleMemoryReset(self, flag):
        if flag == 'f':
            self.forward_obstacles = np.zeros(self.resolution)
            self.forward_max_x = 0
            self.forward_min_x = 999
        elif flag == 'b':
            self.backward_obstacles = np.zeros(self.resolution)
            self.backward_max_x = 0
            self.backward_min_x = 999


    def setUpLowBound(self, msg):
        if self.pos_low_flag==0 and msg.ranges[0] < LASER_RANGE: 
            self.pos_low = msg.ranges[0]*math.sin(-math.pi/4) + self.pos_y + UP_LOW_FENCE
            self.pos_low_flag = 1

        if self.pos_up_flag==0 and msg.ranges[LASER_NUM-1] < LASER_RANGE:
            self.pos_up = msg.ranges[LASER_NUM-1]*math.sin(math.pi/4) + self.pos_y - UP_LOW_FENCE
            self.pos_up_flag = 1

        if self.pos_low_flag==0 and msg.ranges[0] >= LASER_RANGE: 
            self.goal_y -= 0.1
            return
        
        if self.pos_up_flag==0 and msg.ranges[LASER_NUM-1] >= LASER_RANGE:
            self.goal_y += 0.1
            return


    def obstacleUpdate(self, msg):

        # measurement temp storage
        x_temp = []
        y_temp_ind = [] # index of obstacle memory

        # scan data raw processing
        for i in range(LASER_NUM):
            if msg.ranges[i] < LASER_RANGE: # obstacles
                x = self.pos_x + msg.ranges[i]*math.cos(ANGLES[i])
                y = self.pos_y + msg.ranges[i]*math.sin(ANGLES[i])
                # reject laser points in up and low fence
                if y > self.pos_low and y < self.pos_up:
                    y_ind = int((y - self.pos_low) / (self.pos_up - self.pos_low) * self.resolution)
                    x_temp.append(x)
                    y_temp_ind.append(y_ind)

        # data final processing
        if len(x_temp) > 0:
            x_temp = np.array(x_temp)
            y_temp_ind = np.array(y_temp_ind)

            # clustering measurement points
            x_range = max(x_temp) - min(x_temp)

            if x_range < OBSTACLE_WIDTH_SUP:
                # refresh memory if necessary
                if min(x_temp) > self.backward_min_x - 0.2:
                    self.forward_obstacles = self.backward_obstacles
                    self.forward_max_x = self.backward_max_x
                    self.forward_min_x = self.backward_min_x
                    self.obstacleMemoryReset('b')
                    self.front_obstacle_update_flag = 0
                # update forward obstacle memory
                self.forward_max_x = max(max(x_temp), self.forward_max_x)
                self.forward_min_x = min(min(x_temp), self.forward_min_x)
                self.forward_obstacles[y_temp_ind] = 1

            else:
                # forward obstacles & backward obstacles
                x_border = (max(x_temp) + min(x_temp)) / 2
                forward_index = np.where(x_temp < x_border)
                backward_index = np.where(x_temp >= x_border)

                x_forward = x_temp[forward_index]
                self.forward_max_x = max(max(x_forward), self.forward_max_x)
                self.forward_min_x = min(min(x_forward), self.forward_min_x)
                y_ind_forward = y_temp_ind[forward_index]
                self.forward_obstacles[y_ind_forward] = 1

                x_backward = x_temp[backward_index]
                self.backward_max_x = max([max(x_backward), self.backward_max_x])
                self.backward_min_x = min([min(x_backward), self.backward_min_x])
                y_ind_backward = y_temp_ind[backward_index]
                self.backward_obstacles[y_ind_backward] = 1
        
        print("posx: %.2f, front:[%.2f, %.2f], forward:[%.2f, %.2f], backward:[%.2f, %.2f]" % (self.pos_x, self.front_obstacle_min_x, self.front_obstacle_max_x, self.forward_min_x, self.forward_max_x, self.backward_min_x, self.backward_max_x))        


    def setBirdStatus(self):
        # update front obstacle
        if self.front_obstacle_update_flag == 1:
            self.front_obstacle_max_x = self.forward_max_x + L_BACK
            self.front_obstacle_min_x = self.forward_min_x - L_FORWARD

        # update bird status
        if self.forward_min_x == 999:
            self.status = 'init'
        elif self.pos_x < self.front_obstacle_min_x:
            self.status = 'free'
        elif self.pos_x >= self.front_obstacle_min_x and self.pos_x <= self.front_obstacle_max_x and self.front_obstacle_update_flag == 1:
            self.status = 'constrained'
        elif self.pos_x >= self.front_obstacle_min_x and self.pos_x <= self.front_obstacle_max_x and self.front_obstacle_update_flag == 0:
            self.status = 'forecast'
        elif self.pos_x > self.front_obstacle_max_x:
            self.status = 'free'
            self.front_obstacle_update_flag = 1
            self.front_obstacle_max_x = self.forward_max_x + L_BACK
            self.front_obstacle_min_x = self.forward_min_x - L_FORWARD
        
        print("The bird status is", self.status)


    def setGoalY(self):

        if self.status == 'init':
            return
        elif self.status == 'free' or self.status == 'forecast':

            # calculate goal_y from forward obstacle memory: 0-free, 1-stone
            goal_y = 0.0
            signs = np.diff(np.concatenate((np.array([1]), self.forward_obstacles, np.array([1])), axis=0))
            zero_starts = np.where(signs==-1)[0]
            zero_ends = np.where(signs==1)[0]

            zero_count = zero_ends - zero_starts
            zero_best = round(GATE_HEIGHT / (self.pos_up - self.pos_low) * self.resolution)
            
            zero_select = np.abs(zero_count - zero_best)
            zero_select_ind = np.where(zero_select < 0.100*zero_best)

            if len(zero_select_ind[0]) == 1:
                # find only one possible gate
                index = np.argmin(zero_select)
                goal_y = (zero_starts[index] + zero_ends[index]) / 2 / self.resolution * (self.pos_up - self.pos_low) + self.pos_low

            elif len(zero_select_ind[0]) > 1:
                # find multiple possible gates, choose the closest one
                goals = (zero_starts[zero_select_ind] + zero_ends[zero_select_ind]) / 2 / self.resolution * (self.pos_up - self.pos_low) + self.pos_low
                goal_ind = np.argmin(np.abs(goals - self.pos_y))
                goal_y = goals[goal_ind]
            else:
                # don't find possible gate, to explore unknown areas
                index = np.argmax(zero_count)
                goal_y = (zero_starts[index] + zero_ends[index]) / 2 / self.resolution * (self.pos_up - self.pos_low) + self.pos_low
  
            print(zero_count, zero_best)

            if self.status == 'free' or self.status == 'constrained':
                self.goal_y = goal_y
            if self.status == 'forecast':
                _, self.forecast_count = self.calculateAccY(self.vel_buffer.y, goal_y)
                print("forcast count %.3f, goal_y %.3f" %(self.forecast_count, goal_y))
            

    def setAcc(self):

        # setup accelerations
        # acc_x = -0.5
        # acc_y = 0.0
        acc_y, self.goal_y_reach_count = self.calculateAccY(self.vel_buffer.y, self.goal_y)
        self.max_count = max(self.max_count, self.goal_y_reach_count)
        acc_x = self.calculateAccX()
        self.pub_acc_cmd.publish(Vector3(acc_x, acc_y, 0))
        
        # log dx dy in this time step
        # acc_x, acc_y should satisfy constraints
        acc_x = max(min(acc_x, AX_MAX), -AX_MAX)
        acc_y = max(min(acc_y, AY_MAX), -AY_MAX)
        # Vx >= 0
        Vx_p = max(self.vel_buffer.x + acc_x*DELTA_T, 0.0)
        self.dx = (self.vel_buffer.x + Vx_p) * DELTA_T / 2
        self.dy = self.vel_buffer.y*DELTA_T + acc_y*DELTA_T*DELTA_T/2

        t = self.time_count * DELTA_T
        print("t: %.3f, x: %.2f, y: %.2f, Vx: %.2f, Vy: %.2f, ax: %.2f, ay: %.2f, goal_y: %.2f, t_reach: %.3f" % (t, self.pos_x, self.pos_y, self.vel_buffer.x, self.vel_buffer.y, acc_x, acc_y, self.goal_y, self.goal_y_reach_count*DELTA_T))
        print("Max count: %.1f"%self.max_count)
        self.time_count += 1


    def calculateAccY(self, Vy, goal_y):
        # solve y-axis acceleration by MPC --> QP

        # starttime = time.time()
        Dy = goal_y - self.pos_y
        x0 = np.array([[Dy],[Vy]])
        q = np.dot(F.T, x0)
        u = solve_qp(H, q, G, h, solver="osqp")
        ay = u[0]
        u = u.reshape(N,1)
        xs = np.dot(Sx,x0) + np.dot(Su,u)
        xs = xs.reshape(2*N+2)[::2]
        try:
            n_time_gaps = N - np.where(np.abs(xs)[::-1]>0.015)[0][0] + 1
        except:
            n_time_gaps = 0
        # end_time = time.time()
        # print("QP time cost %.7f" % ((end_time - starttime))) # about 0.8ms
        return ay, n_time_gaps


    def calculateAccX(self):

        if self.status == 'init':
            if self.vel_buffer.x < 3.0:
                ax = 1.5
            else:
                ax = 0.0

        elif self.status == 'free':
            if self.goal_y_reach_count > 0:
                ax = self.onlineMpcSolver(self.pos_x, self.vel_buffer.x, self.goal_y_reach_count, self.front_obstacle_min_x)
            else:
                ax = AX_MAX
            if self.vel_buffer.x + ax*DELTA_T > VX_DANGER:
                ax = 0.0
        
        elif self.status == 'constrained':
            Dv = VX_SAFE - self.vel_buffer.x
            ax = max(min(Dv / DELTA_T, AX_MAX), -AX_MAX)

        elif self.status == 'forecast':
            if self.forecast_count > 0:
                t = self.forecast_count*DELTA_T
                Vx_desire = min(FREE_X/t + AX_MAX*t/2, VX_DANGER)
                print("Vx desire = %.2f" % Vx_desire)
                Dv = Vx_desire - self.vel_buffer.x
                ax = max(min(Dv / DELTA_T, AX_MAX), -AX_MAX)
            else:
                ax = AX_MAX

        return ax


    def onlineMpcSolver(self, px, vx, N, r):

        # x_+ = Ax + Bu
        A = np.array([[1, DELTA_T], [0, 1]])
        B = np.array([[DELTA_T**2/2],[DELTA_T]])
        # Ax*x_i <= bx; Au*u_i <= bu; Af*x_N <= bf
        Ax = np.array([[0, -1]])
        # bx = np.array([[0]])
        Au = np.array([[1], [-1]])
        # bu = np.array([[AX_MAX], [AX_MAX]])
        Af = np.array([[1, 0],[0, -1]])
        # bf = np.array([[0], [0]])
        # Loss = |xN|_P + sum(|xi|_Q + |ui|_R)
        # P = np.array([[1, 0], [0, 0]])
        # Q = np.zeros((2,2))
        # R = np.array([[0]])

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
        
        print("solution: u = {}".format(u))
        try:
            ax = u[0]
        except:
            ax = -AX_MAX
        return ax


if __name__ == '__main__':
    try:
        rospy.init_node('flappy_automation_code', anonymous=True)
        agent = Agent()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass
