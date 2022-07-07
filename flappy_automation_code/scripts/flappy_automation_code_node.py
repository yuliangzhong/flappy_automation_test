#!/usr/bin/env python3
from types import DynamicClassAttribute
import rospy
import numpy as np
from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import Vector3

import math

R = 3.549 # m
Vy_m = 10 # m/s
ay_m = 35 # m/s2

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

        # # position upper bound and lower bound
        # self.pos_up = 0
        # self.pos_up_flag = 0 # not set
        # self.pos_low = 0
        # self.pos_low_flag = 0 # not set

        # # target pos y
        # self.y_goal = 0
        # self.y_goal_flag = 0

        # self.init_flag = 1

    def velCallback(self, msg):

        # forward Euler position integral
        self.pos_x += msg.x*1/30
        self.pos_y += msg.y*1/30

        # msg has the format of geometry_msgs::Vector3
        # Example of publishing acceleration command on velocity velCallback
        # x = -1
        # y = 0
        # if msg.y>3:
        #     y = -1
        # elif msg.y<-3:
        #     y = -1
        
        
        self.pub_acc_cmd.publish(Vector3(0,self.accY(1,msg.y),0))
        # print("Vx: %.4f; Vy: %.4f; x: %.4f" %(msg.x, msg.y, self.mark_x))
        print("x_pos: %.2f; y_pos: %.2f" %(self.pos_x, self.pos_y))



    def laserScanCallback(self, msg):

        # if self.pos_up_flag==0 or self.pos_low_flag==0:
        #     self.setUpLowBound(msg)
        #     return

        # if msg.ranges[0] < R: 
        #     self.pos_low = msg.ranges[0]*math.sin(-math.pi/4) + self.pos_y
        # if msg.ranges[8] < R:
        #     self.pos_up = msg.ranges[8]*math.sin(math.pi/4) + self.pos_y
        
        # print("upper: %.3f; lower: %.4f" % (self.pos_up, self.pos_low))



        # message = np.array(msg.ranges)

        # angles = np.linspace(-45,45,9)/180*math.pi
        # distances = np.multiply(message,np.cos(angles))
        # # msg has the format of sensor_msgs::LaserScan
        # # print laser angle and range
        # # print("Laser range: %.4f, angle: %.4f" %(msg.ranges[0], msg.angle_min/math.pi*180))
        # print(msg.ranges)
        pass

    # def setUpLowBound(self, msg):
    #     if self.pos_low_flag==0 and msg.ranges[0] < R: 
    #         self.pos_low = msg.ranges[0]*math.sin(-math.pi/4) + self.pos_y
    #         self.pos_low_flag = 1
    #         print("low bound set: %.3f" % self.pos_low)
    #     if self.pos_up_flag==0 and msg.ranges[8] < R:
    #         self.pos_up = msg.ranges[8]*math.sin(math.pi/4) + self.pos_y
    #         self.pos_up_flag = 1
    #         print("up bound set: %.3f" % self.pos_up)
    
    def accY(self, yd, Vy):
        Dy = yd - self.pos_y
        ay = 0

        # if very close: pd control
        if abs(Dy) < 0.05:
            ay = 10 * Dy + 10 * (0 - Vy)
        else:
        # bang - bang control
            if Dy >= 0 and Vy >= 0:
                S1 = Vy**2/2/ay_m
                if Dy > S1:
                    ay = ay_m
                else:
                    ay = -ay_m
            if Dy >= 0 and Vy < 0:
                ay = ay_m
            if Dy < 0 and Vy >= 0:
                ay = -ay_m
            if Dy < 0 and Vy < 0:
                S1 = Vy**2/2/ay_m
                if Dy < S1:
                    ay = -ay_m
                else:
                    ay = ay_m
        return ay
                




if __name__ == '__main__':
    try:
        rospy.init_node('flappy_automation_code', anonymous=True)
        agent = Agent()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass
