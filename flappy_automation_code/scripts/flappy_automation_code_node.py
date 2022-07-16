#!/usr/bin/env python3
import rospy
import numpy as np
from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import Vector3

import math
import time

LASER_NUM = 9
R = 3.549 # m
AY_MAX = 35 # m/s2
ANGLES = np.linspace(-45,45,9)/180*math.pi # radius
STONE_WIDTH = 0.80 # m
GATE_HEIGHT = 0.60 # m

class Agent(object):
    def __init__(self):
        # Publisher for sending acceleration commands to flappy bird
        self.pub_acc_cmd = rospy.Publisher('/flappy_acc', Vector3, queue_size=1)
        # Subscribe to topics for velocity and laser scan from Flappy Bird game
        self.vel_sub = rospy.Subscriber("/flappy_vel", Vector3, self.velCallback)
        self.laser_sub = rospy.Subscriber("/flappy_laser_scan", LaserScan, self.laserScanCallback)

        # calculate the bird position by velocity integral
        self.pos_x = 0
        self.pos_y = 0

        # define y-axis goal
        self.goal_y = 0

        # position up bound and low bound
        self.pos_up = 0
        self.pos_up_flag = 0 # 0: not set; 1: set
        self.pos_low = 0
        self.pos_low_flag = 0 # 0: not set; 1: set

        # obstacle memory
        # -min-|forward|-max-       -min-|backward|-max-
        # 0-free; 1-stone
        self.resolution = 25
        self.forward_obstacles = np.zeros(self.resolution)
        self.forward_max_x = 0
        self.forward_min_x = 999
        self.backward_obstacles = np.zeros(self.resolution)
        self.backward_max_x = 0
        self.backward_min_x = 999

        # set y-axis goal
        self.ahead_obstacle_max_x = 0
        self.ahead_obstacle_min_x = 999

    def velCallback(self, msg):

        # forward Euler position integral
        self.pos_x += msg.x*1/30
        self.pos_y += msg.y*1/30
        
        acc_x = -0.5
        acc_y = self.calculateAccY(msg.y)
        # acc_y = 0.0
        self.pub_acc_cmd.publish(Vector3(acc_x, acc_y, 0))
        print("x: %.2f, y: %.2f" % (self.pos_x, self.pos_y))


    def laserScanCallback(self, msg):

        ##### Main Logic for Each Scan #####

        # Step 1: set tunnel up and low bound
        if self.pos_up_flag==0 or self.pos_low_flag==0:
            self.setUpLowBound(msg)
        else:
            # Step 2: update obstacle memory
            self.obstacleUpdate(msg)

            # Step 3: set y-axis goal
            starttime = time.time()
            self.setGoalY()
            endtime = time.time()
            # print("set goal time cost: %.15f s" % (endtime-starttime))


    def setUpLowBound(self, msg):
        if self.pos_low_flag==0 and msg.ranges[0] < R: 
            self.pos_low = msg.ranges[0]*math.sin(-math.pi/4) + self.pos_y
            self.pos_low_flag = 1

        if self.pos_up_flag==0 and msg.ranges[8] < R:
            self.pos_up = msg.ranges[8]*math.sin(math.pi/4) + self.pos_y
            self.pos_up_flag = 1

        if self.pos_low_flag==0 and msg.ranges[0] >= R: 
            self.goal_y -= 0.1
            return
        
        if self.pos_up_flag==0 and msg.ranges[8] >= R:
            self.goal_y += 0.1
            return


    def obstacleUpdate(self, msg):
        # measurement temp storage
        x_temp = []
        y_num_temp = []

        # process scan data
        for i in range(LASER_NUM):
            if msg.ranges[i] < R: # obstacles
                x = self.pos_x + msg.ranges[i]*math.cos(ANGLES[i])
                y = self.pos_y + msg.ranges[i]*math.sin(ANGLES[i])
                # reject up and low bound
                if abs(y - self.pos_low) > 0.02 and abs(y - self.pos_up) > 0.02:
                    y_num = int((y - self.pos_low) / (self.pos_up - self.pos_low) * self.resolution)
                    x_temp.append(x)
                    y_num_temp.append(y_num)
        # arrange scan data
        if len(x_temp) > 0:
            x_temp = np.array(x_temp)
            y_num_temp = np.array(y_num_temp)
            x_range = max(x_temp) - min(x_temp)
            if x_range < STONE_WIDTH:
                self.forward_max_x = max([max(x_temp), self.forward_max_x])
                self.forward_min_x = min([min(x_temp), self.forward_min_x])
                self.forward_obstacles[y_num_temp] = 1
            else:
                x_middle = (max(x_temp) + min(x_temp)) / 2
                forward_index = np.where(x_temp < x_middle)
                backward_index = np.where(x_temp >= x_middle)

                x_forward = x_temp[forward_index]
                self.forward_max_x = max([max(x_forward), self.forward_max_x])
                self.forward_min_x = min([min(x_forward), self.forward_min_x])
                y_num_forward = y_num_temp[forward_index]
                self.forward_obstacles[y_num_forward] = 1

                x_backward = x_temp[backward_index]
                self.backward_max_x = max([max(x_backward), self.backward_max_x])
                self.backward_min_x = min([min(x_backward), self.backward_min_x])
                y_num_backward = y_num_temp[backward_index]
                self.backward_obstacles[y_num_backward] = 1
            
            if abs(self.forward_max_x - self.backward_max_x) < 0.2:
                self.forward_obstacles = self.backward_obstacles
                self.forward_max_x = self.backward_max_x
                self.forward_min_x = self.backward_min_x
                self.backward_obstacles = np.zeros(self.resolution)
                self.backward_max_x = 0
                self.backward_min_x = 999
        # print("-----------------------")
        # print("posx: %.2f, forward:[%.2f, %.2f], backward:[%.2f, %.2f]" % (self.pos_x, self.forward_min_x, self.forward_max_x, self.backward_min_x, self.backward_max_x))
        # print(self.forward_obstacles)
        # print(self.backward_obstacles)


    def setGoalY(self):

        # update obstacle x ahead of the bird
        if self.pos_x > self.forward_min_x - 1.0:
            self.ahead_obstacle_max_x = self.forward_max_x
            self.ahead_obstacle_min_x = self.forward_min_x

        # do not change y-goal when pass the gate
        if self.pos_x > self.ahead_obstacle_min_x - 0.15 and self.pos_x < self.ahead_obstacle_max_x + 0.30:
            return
            # print("[LOCKED], posx: %.2f, ahead:[%.2f, %.2f], forward:[%.2f, %.2f], backward:[%.2f, %.2f]" % (self.pos_x, self.ahead_obstacle_min_x, self.ahead_obstacle_max_x, self.forward_min_x, self.forward_max_x, self.backward_min_x, self.backward_max_x))
        else:
            signs = np.diff(np.concatenate((np.array([1]), self.forward_obstacles, np.array([1])), axis=0))
            zero_starts = np.where(signs==-1)[0]
            zero_ends = np.where(signs==1)[0]
            # zero_best = int(GATE_HEIGHT / (self.pos_up - self.pos_low) * self.resolution)
            
            # zero_count = zero_ends - zero_starts
            # zero_count[np.where(zero_count<zero_best)] = 999 # can't be the gate
            # index = np.argmin(zero_count)
            index = np.argmax(zero_ends - zero_starts)
            self.goal_y = (zero_starts[index] + zero_ends[index]) / 2 / self.resolution * (self.pos_up - self.pos_low) + self.pos_low
            self.goal_y = max(min(self.goal_y, self.pos_up-0.2), self.pos_low+0.2)
            # print("posx: %.2f, ahead:[%.2f, %.2f], forward:[%.2f, %.2f], backward:[%.2f, %.2f]" % (self.pos_x, self.ahead_obstacle_min_x, self.ahead_obstacle_max_x, self.forward_min_x, self.forward_max_x, self.backward_min_x, self.backward_max_x))

        # if sum(self.forward_obstacles) > 0.1*self.resolution:
        #     signs = np.diff(np.concatenate((np.array([1]), self.forward_obstacles, np.array([1])), axis=0))
        #     zero_starts = np.where(signs==-1)[0]
        #     zero_ends = np.where(signs==1)[0]
        #     index = np.argmax(zero_ends - zero_starts)
        #     self.goal_y = (zero_starts[index] + zero_ends[index]) / 2 / self.resolution * (self.pos_up - self.pos_low) + self.pos_low

        #     self.goal_y = max(min(self.goal_y, self.pos_up-0.5), self.pos_low+0.5)


            

    def calculateAccY(self, Vy):
        Dy = self.goal_y - self.pos_y
        pd_ctrl_range = 0.3 # [m]
        Kp = 50
        Kd = 20

        # if very close: pd control
        if abs(Dy) < pd_ctrl_range:
            ay = Kp * Dy + Kd * (0 - Vy)
        else:
        # bang - bang control
            if Dy >= 0 and Vy >= 0:
                S1 = Vy**2/2/AY_MAX + pd_ctrl_range
                if Dy > S1:
                    ay = AY_MAX
                else:
                    ay = -AY_MAX
            if Dy >= 0 and Vy < 0:
                ay = AY_MAX
            if Dy < 0 and Vy >= 0:
                ay = -AY_MAX
            if Dy < 0 and Vy < 0:
                S1 = Vy**2/2/AY_MAX + pd_ctrl_range
                if Dy < -S1:
                    ay = -AY_MAX
                else:
                    ay = AY_MAX
        return ay
                




if __name__ == '__main__':
    try:
        rospy.init_node('flappy_automation_code', anonymous=True)
        agent = Agent()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass
