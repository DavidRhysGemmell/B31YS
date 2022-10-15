#! /usr/bin/env python

import rospy
from geometry_msgs.msg import Twist
from sensor_msgs.msg import LaserScan
from nav_msgs.msg import Odometry
from tf.transformations import euler_from_quaternion
from nav_msgs.msg import OccupancyGrid
import numpy as np
import math
class Drive_at_block:

    def __init__(self):
        self.grid=np.zeros((100,100))
        print(self.grid)
        self.closest_object_angle=0       
        self.pub = rospy.Publisher('/cmd_vel', Twist, queue_size=1)
        self.sub = rospy.Subscriber('/scan', LaserScan, self.callback_laser)
        self.vel = Twist()
        self.sub_odom = rospy.Subscriber("/odom", Odometry, self.callback_function)
        self.real_yaw = 0
        rospy.loginfo("subscriber node is active...")       
        self.real_angle= np.zeros(720)
        self.linearx=0
        self.lineary=0
        self.robot_position=([[0],[0]])


    def callback_laser(self, LaserMsg):
        self.Laser_scan_array = LaserMsg
        
        
        for i in range(720):
            # self.real_angle[i] = (i/2) + self.real_yaw #angle of laser relative to world frame
            # if self.real_angle[i] >= 360:
            #     self.real_angle[i] = self.real_angle[i] - 360
            # self.real_angle[i]= self.real_angle[i]*180/math.pi #convert it to radians for sin cos you fucking idiot
            theta= (i/2) *180/math.pi



            if self.Laser_scan_array.ranges[i] <2: #self.Laser_scan_array.range_max:
                #print(self.Laser_scan_array.ranges[i])
                if self.Laser_scan_array.ranges[i] > self.Laser_scan_array.range_min:
                    bodyframe=([[(self.Laser_scan_array.ranges[i]*math.cos(theta))], [self.Laser_scan_array.ranges[i]*math.sin(theta)]])
                    #print(bodyframe)
                    R=np.array([[math.cos(self.real_yaw),-1*math.sin(self.real_yaw)],[math.sin(self.real_yaw),math.cos(self.real_yaw)]])
                    ob= (self.robot_position)+(np.dot(R,bodyframe))
                    #print(ob)
                    #ob_y= (self.lineary + self.Laser_scan_array.ranges[i]*math.sin(self.real_angle[i]))*10 +7 #y coordinate of an obstacle
                    #ob_x= (self.linearx + self.Laser_scan_array.ranges[i]*math.cos(self.real_angle[i]))*10 +7#x coordinate of an obstacle
                    
                    self.grid[(int(ob[0]*10))+49, int(ob[1]*10)+49]=1 #sets the coordinates of obstacle on grid as 1
                    print((int(ob[0]*10))+49, int(ob[1]*10)+49)
                    # print("---")
                    # print(ob_y)
                    # print(ob_x)
                    
                    #print(f"range is {self.Laser_scan_array.ranges[i]:.2f}")
                    #print(f"range is {self.Laser_scan_array.ranges[i]:.2f}, angle is{self.real_angle:.2f}")
                    #print(self.lineary) 
        #print(self.grid)                   
        #print(self.real_angle[0])           
        # closest_object_range=min(self.Laser_scan_array.ranges)
        # #print(closest_object_range)
        # self.closest_object_angle=self.Laser_scan_array.ranges.index(min(self.Laser_scan_array.ranges))
        # #print(self.closest_object_angle) 
    
        # if self.closest_object_angle <= 5:
        #     self.vel.angular.z=0
        #     self.pub.publish(self.vel)
        #     #print("Target Aquired") 
        #     self.vel.linear.x=1
        #     self.pub.publish(self.vel)          
        # else:
        #     if self.closest_object_angle < 360:
        #         self.vel.angular.z=0.5
        #         self.pub.publish(self.vel)
        #         #print("Turning left")
        #     else:
        #         if self.closest_object_angle >= 715:
        #             self.vel.angular.z=0
        #             self.pub.publish(self.vel)
        #             #print("Target Aquired")
        #             self.vel.linear.x=1
        #             self.pub.publish(self.vel)                         
        #         else:
        #             self.vel.angular.z=-0.5
        #             self.pub.publish(self.vel)
        #             #print("Turning right")


        # for i in range(1,2):
        #     current = self.Laser_scan_array.ranges(i)
        #     print(current)

        # true_laser_angle= self.yaw + self.Laser_scan_array.ranges.index()/2
        # print(f"True Laser Angle is {true_laser_angle:.2f}")

    def callback_function(self, odom_data):
        self.linearx=odom_data.pose.pose.position.x
        self.lineary=odom_data.pose.pose.position.y
        self.robot_position=([[self.linearx], [self.lineary]]) #position on grid
        #print(self.robot_position)
        linearz=odom_data.pose.pose.position.z
        quaternion_orientation= odom_data.pose.pose.orientation
        (roll, pitch, yaw) = euler_from_quaternion([quaternion_orientation.x, quaternion_orientation.y, quaternion_orientation.z, quaternion_orientation.w])
        if yaw < 0:
            self.real_yaw =yaw+2*math.pi
        else:
            self.real_yaw = yaw #angle of robot relative to world frame

        #print(self.real_yaw)
        #print(f"x is {self.linearx:.2f}, y is {self.lineary:.2f}, z is {linearz:.2f}, Yaw is {self.real_yaw:.2f}")
    



       






if __name__ == '__main__':
    rospy.init_node('move_service_server')
    rospy.loginfo('its working dipshit')
    Drive_at_block()
    rospy.spin()

