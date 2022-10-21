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
        rospy.init_node('go_robot_go')
        # self.x_goal=int(rospy.get_param("/go_robot_go/x_goal"))
        # self.y_goal=int(rospy.get_param("/go_robot_goal/y_goal"))
        # self.goal=(self.x_goal,self.y_goal)
        self.goal=(80,70)
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
        self.q_goal=(80,30)
        self.q_0=(0,0)
        self.robot_position_grid=([[49],[49]])
        self.max_speed=1
        self.ctrl_c=False
        rospy.on_shutdown(self.shutdown) 
        self.on_target=False
    
 
        
    def shutdown(self):
        self.vel.angular.z=0
        self.vel.linear.x=0
        self.pub.publish(self.vel)
        print("shutting down")

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
                    #print((int(ob[0]*10))+49, int(ob[1]*10)+49)
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
        self.potential_field()
    def potential_field(self):
        p_0=2 #barrier around object in meters
        F_rep=0
        k_att=1
        k_rep=1
        q=self.robot_position_grid #robot position on grid
        p_goal=math.sqrt((self.q_goal[0]-self.robot_position_grid[0])^2+(self.q_goal[1]-self.robot_position_grid[1])^2) #distance from robot to goal
        #Attractive
        U_att=1/2 *k_att*p_goal^2
        F_att=-k_att*(q-self.q_goal)
        
        for i in range(-20,20): #takes a 2m range around the robot, based on the map though
            for j in range(-20,20):
                if self.grid[self.robot_position_grid[0]+i,self.robot_position_grid[1]+j] == 1:
                    p_q=math.sqrt((self.robot_position_grid[0]+i)^2+(self.robot_position_grid[1]+j)^2)
                        #Repulsive
                    if p_q<=p_0:
                        U_rep=1/2 * k_rep*(((1/p_q)-(1/p_0))^2)
                        F_rep=F_rep+k_rep*((1/p_q)-(1/p_0))*(1/p_q^2)*((q-self.q_0)/p_q)
                    else:
                        U_rep=0
                        F_rep=F_rep
        potential_vector= (F_att[1]+F_rep[1],F_att[0]+F_rep[0])
        print(potential_vector)
        self.potential_angle=np.arctan2(potential_vector[1],potential_vector[0])
        self.potential_force= self.max_speed/math.sqrt(potential_vector[0]^2,potential_vector[1])
        self.turn()

    def turn(self):
        if abs(self.real_yaw-self.potential_angle) <= 2:
             self.vel.angular.z=0
             self.pub.publish(self.vel)
             self.on_target=True  
             print("on target")
             self.go_forward()
        else:
            self.on_target=False
            self.vel.linear.x=0
            self.vel.angular.z=0.5
            self.pub.publish(self.vel)
            print("turning")

                 
    def go_forward(self):
        if self.on_target==True:
            self.vel.linear.x=self.potential_force
            self.pub.publish(self.vel)
        else:
            self.vel.linear.x=0
            self.pub.publish(self.vel)


    def found_goal(self):
        if self.robot_position_grid==self.goal:
            print("found goal, you finally succeeded at something")
            self.shutdown()



    def callback_function(self, odom_data):
        self.linearx=odom_data.pose.pose.position.x
        self.lineary=odom_data.pose.pose.position.y
        self.robot_position=([[self.linearx], [self.lineary]]) #position on not on array
        self.robot_position_grid=([[self.linearx*10+49], [self.lineary*10+49]])
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
    
    rospy.loginfo('its working dipshit')
    Drive_at_block()
    rospy.spin()

