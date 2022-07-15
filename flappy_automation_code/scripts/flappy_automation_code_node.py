#!/usr/bin/env python3
import rospy
import numpy as np
from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import Vector3

import math

R = 3.549 # m
AY_MAX = 35/10 # m/s2
ANGLES = np.linspace(-45,45,9)/180*math.pi # radius
STONE_WIDTH = 1 # m

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
        self.y_goal = 0

        # position up bound and low bound
        self.pos_up = 0
        self.pos_up_flag = 0 # 0: not set; 1: set
        self.pos_low = 0
        self.pos_low_flag = 0 # 0: not set; 1: set

        # obstacle memory
        self.resolution = 25
        self.forward_obstacles = np.zeros(self.resolution)
        self.forward_x = 0 # maximum forward obstacle x;    0: not set
        self.backward_obstacles = np.zeros(self.resolution)
        self.backward_x = 0 # maximum backward obstacle x;  0: not set
        
        # set y-axis goal
        self.ahead_obstacle_x = 0

    def velCallback(self, msg):

        # forward Euler position integral
        self.pos_x += msg.x*1/30
        self.pos_y += msg.y*1/30
        
        acc_x = -0.5
        acc_y = self.calculateAccY(msg.y)
        self.pub_acc_cmd.publish(Vector3(acc_x, acc_y, 0))


    def laserScanCallback(self, msg):

        ##### Main Logic for Each Scan #####

        # Step 1: set tunnel up and low bound
        if self.pos_up_flag==0 or self.pos_low_flag==0:
            self.setUpLowBound(msg)
        else:
            # Step 2: update obstacle memory
            self.obstacleUpdate(msg)

            # Step 3: set y-axis goal
            self.setGoalY()



    def setUpLowBound(self, msg):
        if self.pos_low_flag==0 and msg.ranges[0] < R: 
            self.pos_low = msg.ranges[0]*math.sin(-math.pi/4) + self.pos_y
            self.pos_low_flag = 1

        if self.pos_up_flag==0 and msg.ranges[8] < R:
            self.pos_up = msg.ranges[8]*math.sin(math.pi/4) + self.pos_y
            self.pos_up_flag = 1

        if self.pos_low_flag==0 and msg.ranges[0] >= R: 
            self.y_goal -= 0.1
            return
        
        if self.pos_up_flag==0 and msg.ranges[8] >= R:
            self.y_goal += 0.1
            return


    def obstacleUpdate(self, msg):
        x_temp = []
        y_num_temp = []
        # process scan data
        for i in range(9):
            if msg.ranges[i] < R:
                x = self.pos_x + msg.ranges[i]*math.cos(ANGLES[i])
                y = self.pos_y + msg.ranges[i]*math.sin(ANGLES[i])
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
                self.forward_x = max([max(x_temp), self.forward_x])
                for i in y_num_temp:
                    self.forward_obstacles[i] = 1
            else:
                x_middle = (max(x_temp) + min(x_temp)) / 2
                forward_index = np.where(x_temp < x_middle)
                backward_index = np.where(x_temp >= x_middle)

                x_forward = x_temp[forward_index]
                self.forward_x = max([max(x_forward), self.forward_x])
                y_num_forward = y_num_temp[forward_index]
                self.forward_obstacles[y_num_forward] = 1

                x_backward = x_temp[backward_index]
                self.backward_x = max([max(x_backward), self.backward_x])
                y_num_backward = y_num_temp[backward_index]
                self.backward_obstacles[y_num_backward] = 1
            
            if abs(self.forward_x - self.backward_x) < 0.2:
                self.forward_obstacles = self.backward_obstacles
                self.forward_x = self.backward_x
                self.backward_obstacles = np.zeros(self.resolution)
                self.backward_x = 0
        # print("-----------------------")
        # print("forward_x: %.3f, backward_x: %.3f" % (self.forward_x, self.backward_x))
        # print(self.forward_obstacles)
        # print(self.backward_obstacles)


    def setGoalY(self):
        if self.pos_x > self.ahead_obstacle_x - 0.35 and self.pos_x < self.ahead_obstacle_x + 0.6:
            print("[LOCKED], %.2f, pos x: %.2f, ahead x: %.2f, forward x: %.2f" % (self.pos_x - self.ahead_obstacle_x, self.pos_x, self.ahead_obstacle_x, self.forward_x))
            return
        
        print("pos x: %.2f, ahead x: %.2f, forward x: %.2f" % (self.pos_x, self.ahead_obstacle_x, self.forward_x))

        if self.pos_x >= self.ahead_obstacle_x + 0.6:
            self.ahead_obstacle_x = self.forward_x

        if sum(self.forward_obstacles) > 0.1*self.resolution:
            signs = np.diff(np.concatenate((np.array([1]), self.forward_obstacles, np.array([1])), axis=0))
            zero_starts = np.where(signs==-1)[0]
            zero_ends = np.where(signs==1)[0]
            index = np.argmax(zero_ends - zero_starts)
            self.y_goal = (zero_starts[index] + zero_ends[index]) / 2 / self.resolution * (self.pos_up - self.pos_low) + self.pos_low

            self.y_goal = max(min(self.y_goal, self.pos_up-0.5), self.pos_low+0.5)


            

    def calculateAccY(self, Vy):
        Dy = self.y_goal - self.pos_y
        ay = 0

        # if very close: pd control
        if abs(Dy) < 0.25:
            ay = 20 * Dy + 20 * (0 - Vy)
        else:
        # bang - bang control
            if Dy >= 0 and Vy >= 0:
                S1 = Vy**2/2/AY_MAX
                if Dy > S1:
                    ay = AY_MAX
                else:
                    ay = -AY_MAX
            if Dy >= 0 and Vy < 0:
                ay = AY_MAX
            if Dy < 0 and Vy >= 0:
                ay = -AY_MAX
            if Dy < 0 and Vy < 0:
                S1 = Vy**2/2/AY_MAX
                if Dy < S1:
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
