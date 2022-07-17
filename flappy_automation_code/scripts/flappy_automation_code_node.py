#!/usr/bin/env python3
import rospy
import numpy as np
from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import Vector3

import math
import time

DELTA_T = 1/30
AY_MAX = 35 # m/s2

LASER_NUM = 9
LASER_RANGE = 3.549 # m
ANGLES = np.linspace(-45,45,9)/180*math.pi # radius

STONE_WIDTH = 0.80 # m
GATE_HEIGHT = 0.50 # m
UP_LOW_FENCE = 0.7 # m

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

        # define the y-axis goal
        self.goal_y = 0.0

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
        self.resolution = 400
        self.obstacleMemoryReset('f') # 'f': forward
        self.obstacleMemoryReset('b') # 'b': backward

        # the obstacle in front of the bird 
        self.front_obstacle_max_x = 0
        self.front_obstacle_min_x = 999

        self.time_count = 0

    def velCallback(self, msg):

        # position integral
        self.pos_x += self.dx
        self.pos_y += self.dy

        # setup accelerations
        acc_x = -0.5
        # acc_y = 0.0
        # acc_x = self.calculateAccX(msg.x)
        acc_y = self.calculateAccY(msg.y)
        self.pub_acc_cmd.publish(Vector3(acc_x, acc_y, 0))
        
        # log dx dy in this time step
        Vx_p = max([msg.x + acc_x*DELTA_T, 0.0])
        self.dx = (msg.x + Vx_p) * DELTA_T / 2
        self.dy = msg.y*DELTA_T + acc_y*DELTA_T*DELTA_T/2

        t = self.time_count * DELTA_T
        # print("t: %.3f, x: %.5f, y: %.3f, Vx: %.5f, Vy: %.3f, ay: %.3f" % (t, self.pos_x, self.pos_y, msg.x, msg.y, acc_y))
        self.time_count += 1

    def laserScanCallback(self, msg):
        
        ##### Main Logic for Each Scan #####

        # Step 1: set tunnel up and low bound
        if self.pos_up_flag==0 or self.pos_low_flag==0:
            self.setUpLowBound(msg)

        else:
            start_time = time.time()
            # Step 2: update obstacle memory
            self.obstacleUpdate(msg)

            # Step 3: set y-axis goal
            self.setGoalY()
            end_time = time.time()
            # print("time cost: %.5f" %(end_time - start_time))
        return


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
        y_temp_ind = [] # index in obstacle memory

        # process scan data
        for i in range(LASER_NUM):
            if msg.ranges[i] < LASER_RANGE: # obstacles
                x = self.pos_x + msg.ranges[i]*math.cos(ANGLES[i])
                y = self.pos_y + msg.ranges[i]*math.sin(ANGLES[i])
                # reject up and low bound
                if y > self.pos_low and y < self.pos_up:
                    y_ind = int((y - self.pos_low) / (self.pos_up - self.pos_low) * self.resolution)
                    x_temp.append(x)
                    y_temp_ind.append(y_ind)

        # arrange scan data
        if len(x_temp) > 0:
            x_temp = np.array(x_temp)
            y_temp_ind = np.array(y_temp_ind)
            x_range = max(x_temp) - min(x_temp)
            if x_range < STONE_WIDTH:
                self.forward_max_x = max([max(x_temp), self.forward_max_x])
                self.forward_min_x = min([min(x_temp), self.forward_min_x])
                self.forward_obstacles[y_temp_ind] = 1
            else:
                x_middle = (max(x_temp) + min(x_temp)) / 2
                forward_index = np.where(x_temp < x_middle)
                backward_index = np.where(x_temp >= x_middle)

                x_forward = x_temp[forward_index]
                self.forward_max_x = max([max(x_forward), self.forward_max_x])
                self.forward_min_x = min([min(x_forward), self.forward_min_x])
                y_num_forward = y_temp_ind[forward_index]
                self.forward_obstacles[y_num_forward] = 1

                x_backward = x_temp[backward_index]
                self.backward_max_x = max([max(x_backward), self.backward_max_x])
                self.backward_min_x = min([min(x_backward), self.backward_min_x])
                y_num_backward = y_temp_ind[backward_index]
                self.backward_obstacles[y_num_backward] = 1
            
            if abs(self.forward_max_x - self.backward_max_x) < 0.2:
                self.forward_obstacles = self.backward_obstacles
                self.forward_max_x = self.backward_max_x
                self.forward_min_x = self.backward_min_x
                self.obstacleMemoryReset('b')
        # print("-----------------------")
        # print("posx: %.2f, forward:[%.2f, %.2f], backward:[%.2f, %.2f]" % (self.pos_x, self.forward_min_x, self.forward_max_x, self.backward_min_x, self.backward_max_x))
        # print(self.forward_obstacles)
        # print(self.backward_obstacles)


    def setGoalY(self):

        # update obstacle x ahead of the bird
        if self.pos_x > self.forward_min_x - 1.0:
            self.front_obstacle_max_x = self.forward_max_x
            self.front_obstacle_min_x = self.forward_min_x

        # do not change y-goal when pass the gate
        if self.pos_x > self.front_obstacle_min_x - 0.15 and self.pos_x < self.front_obstacle_max_x + 0.25:
            # print("posx: %.2f, ahead:[%.2f, %.2f], forward:[%.2f, %.2f], backward:[%.2f, %.2f]" % (self.pos_x, self.front_obstacle_min_x, self.front_obstacle_max_x, self.forward_min_x, self.forward_max_x, self.backward_min_x, self.backward_max_x))
            return
        else:
            signs = np.diff(np.concatenate((np.array([1]), self.forward_obstacles, np.array([1])), axis=0))
            zero_starts = np.where(signs==-1)[0]
            zero_ends = np.where(signs==1)[0]

            zero_count = zero_ends - zero_starts
            zero_best = round(GATE_HEIGHT / (self.pos_up - self.pos_low) * self.resolution)
            
            zero_select = np.abs(zero_count - zero_best)
            zero_select_ind = np.where(zero_select < 0.2*zero_best)

            if len(zero_select_ind[0]) == 1:
                # find only one possible gate
                index = np.argmin(zero_select)
                self.goal_y = (zero_starts[index] + zero_ends[index]) / 2 / self.resolution * (self.pos_up - self.pos_low) + self.pos_low

            elif len(zero_select_ind[0]) > 1:
                # find multiple possible gates, choose the closest one
                goals = (zero_starts[zero_select_ind] + zero_ends[zero_select_ind]) / 2 / self.resolution * (self.pos_up - self.pos_low) + self.pos_low
                goal_ind = np.argmin(np.abs(goals - self.pos_y))
                self.goal_y = goals[goal_ind]
            else:
                # don't find possible gate, to explore unknown areas
                index = np.argmax(zero_count)
                self.goal_y = (zero_starts[index] + zero_ends[index]) / 2 / self.resolution * (self.pos_up - self.pos_low) + self.pos_low

            
            print(zero_count, zero_best)

            # self.goal_y = (zero_starts[index] + zero_ends[index]) / 2 / self.resolution * (self.pos_up - self.pos_low) + self.pos_low
            # print("goal y: %.2f" % self.goal_y)
            # print("posx: %.2f, ahead:[%.2f, %.2f], forward:[%.2f, %.2f], backward:[%.2f, %.2f]" % (self.pos_x, self.front_obstacle_min_x, self.front_obstacle_max_x, self.forward_min_x, self.forward_max_x, self.backward_min_x, self.backward_max_x))


    def obstacleMemoryReset(self, flag):
        if flag == 'f':
            self.forward_obstacles = np.zeros(self.resolution)
            self.forward_max_x = 0
            self.forward_min_x = 999
        elif flag == 'b':
            self.backward_obstacles = np.zeros(self.resolution)
            self.backward_max_x = 0
            self.backward_min_x = 999
        else:
            print("Wrong flag received in function obstacleMemoryReset")
            

    def calculateAccY(self, Vy):
        Dy = self.goal_y - self.pos_y
        
        if Vy >= 0:
            # S = (Vy+AY_MAX*DELTA_T)**2/2/AY_MAX # > 0
            pd_ctrl_range = Vy*DELTA_T + AY_MAX*DELTA_T*DELTA_T/2 # [m]
        else:
            # S = (Vy-AY_MAX*DELTA_T)**2/2/AY_MAX # > 0
            pd_ctrl_range = -Vy*DELTA_T + AY_MAX*DELTA_T*DELTA_T/2 # [m]
        
        Kp = 50
        Kd = 30

        # if very close: pd control
        if abs(Dy) < pd_ctrl_range:
            ay = Kp * Dy + Kd * (0 - Vy)
        else:
        # bang - bang control
            if Dy >= 0 and Vy >= 0:
                S = Vy**2/2/AY_MAX + pd_ctrl_range
                if Dy > S:
                    ay = AY_MAX
                else:
                    ay = -AY_MAX
            if Dy >= 0 and Vy < 0:
                ay = AY_MAX
            if Dy < 0 and Vy >= 0:
                ay = -AY_MAX
            if Dy < 0 and Vy < 0:
                S = Vy**2/2/AY_MAX + pd_ctrl_range
                if Dy < -S:
                    ay = -AY_MAX
                else:
                    ay = AY_MAX
        return ay

    def calculateAccX(self, Vx):
        pass

if __name__ == '__main__':
    try:
        rospy.init_node('flappy_automation_code', anonymous=True)
        agent = Agent()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass
