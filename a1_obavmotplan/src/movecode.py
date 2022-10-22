#! /usr/bin/env python

import rospy
from geometry_msgs.msg import Twist
from sensor_msgs.msg import LaserScan
from nav_msgs.msg import Odometry
from tf.transformations import euler_from_quaternion
from nav_msgs.msg import OccupancyGrid
import numpy as np
import math
from matplotlib import pyplot as plt
import sys

class Drive_at_block:

    def __init__(self):
        rospy.init_node('go_robot_go')
        # self.x_goal=int(rospy.get_param("/go_robot_go/x_goal"))
        # self.y_goal=int(rospy.get_param("/go_robot_goal/y_goal"))
        # self.goal=(self.x_goal,self.y_goal)
        self.goal=(100,0)
        print(self.goal)
        self.grid_size=200
        self.grid=np.zeros((self.grid_size,self.grid_size))
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
        self.q_0=(0,0)
        self.robot_position_grid=([[self.grid/2],[self.grid_size/2]])
        self.max_speed=1
        self.ctrl_c=False
        rospy.on_shutdown(self.shutdown) 
        self.on_target=False

 
        
    def shutdown(self):
        self.vel.angular.z=0
        self.vel.linear.x=0
        self.pub.publish(self.vel)
        # plt.imshow(self.grid)
        # plt.show()
        self.vel.angular.z=0
        self.vel.linear.x=0
        self.pub.publish(self.vel)
        print("shutting down")
        sys.exit()

    def callback_laser(self, LaserMsg):
        self.Laser_scan_array = LaserMsg
        for i in range(720):
            theta= (i/2) *math.pi/180

            if self.Laser_scan_array.ranges[i] <3: #self.Laser_scan_array.range_max:
                #print(self.Laser_scan_array.ranges[i])
                if self.Laser_scan_array.ranges[i] > self.Laser_scan_array.range_min:
                    bodyframe=([[(self.Laser_scan_array.ranges[i]*math.cos(theta))], [self.Laser_scan_array.ranges[i]*math.sin(theta)]])
                    #print(bodyframe)
                    R=np.array([[math.cos(self.real_yaw),-1*math.sin(self.real_yaw)],[math.sin(self.real_yaw),math.cos(self.real_yaw)]])
                    ob= (self.robot_position)+(np.dot(R,bodyframe))
                    #print(ob)
                    #ob_y= (self.lineary + self.Laser_scan_array.ranges[i]*math.sin(self.real_angle[i]))*10 +7 #y coordinate of an obstacle
                    #ob_x= (self.linearx + self.Laser_scan_array.ranges[i]*math.cos(self.real_angle[i]))*10 +7#x coordinate of an obstacle
                    
                    self.grid[(int(ob[0]*10))+int(self.grid_size/2), int(ob[1]*10)+int(self.grid_size/2)]=1 #sets the coordinates of obstacle on grid as 1

        self.potential_field()

    def potential_field(self):
        p_0=30 #barrier around object in grid squares
        q_goal=self.goal
        F_rep=(0,0)
        k_att=1
        k_rep=10
        q=self.robot_position_grid #robot position on grid
        p_goal=math.sqrt(pow(q_goal[0]-self.robot_position_grid[0],2)+pow(q_goal[1]-self.robot_position_grid[1],2)) #distance from robot to goal
        #print(p_goal)
        if  p_goal <= 2:
            self.found_goal()
        #Attractive
        #U_att=1/2 *k_att*pow(p_goal,2)
        
        F_att=(-k_att*(q[0]-q_goal[0]),-k_att*(q[1]-q_goal[1]))
        #print(f"F_att is{F_att}")
        
        for i in range(-p_0,p_0): #takes a p_0m range around the robot, based on the map though
            for j in range(-20,20):
                if self.grid[int(self.robot_position_grid[0])+i,int(self.robot_position_grid[1])+j] == 1: #takes all points on the map that have an obstacle
                    p_q=math.sqrt(pow(i,2)+pow(j,2)) # distance to obstacle
                        #Repulsive
                    if p_q<=p_0:
                        #U_rep=1/2 * k_rep*(pow((1/p_q)-(1/p_0),2))
                        F_rep=(F_rep[0]+(k_rep*((1/p_q)-(1/p_0))*(1/pow(p_q,2))*(q[0]-self.q_0[0])/p_q),F_rep[1]+(k_rep*((1/p_q)-(1/p_0))*(1/pow(p_q,2))*((q[1]-self.q_0[1])/p_q)))
                    else:
                        U_rep=0
        #print(f"F_att is {F_att}, F_rep is {F_rep}") 

                       
        potential_vector= (F_att[0]+F_rep[0],F_att[1]+F_rep[1])
        #print(potential_vector)
        self.potential_angle=np.arctan2(potential_vector[1],potential_vector[0])+math.pi


        self.potential_force= 10*self.max_speed/math.sqrt(pow(potential_vector[0],2)+pow(potential_vector[1],2))
        
        #print(f" robot velocity would be{self.potential_force}")
                
        self.turn()

    def turn(self):
        print(self.potential_angle)
        if abs((self.real_yaw-self.potential_angle)*180/math.pi) <= 4:
             self.vel.angular.z=0
             self.pub.publish(self.vel)
             self.on_target=True  
             print("on target")
             self.go_forward()
        elif ((self.real_yaw-self.potential_angle)*180/math.pi) <4:
            self.on_target=False
            self.vel.linear.x=0
            self.vel.angular.z=0.5
            self.pub.publish(self.vel)
            print("turning left")
        elif ((self.real_yaw-self.potential_angle)*180/math.pi) >-4:
            self.on_target=False
            self.vel.linear.x=0
            self.vel.angular.z=-0.5
            self.pub.publish(self.vel)
            print("turning right")
                 
    def go_forward(self):
        if self.on_target==True:
            self.vel.linear.x=self.potential_force
            self.pub.publish(self.vel)
        else:
            self.vel.linear.x=0
            self.pub.publish(self.vel)
        self.found_goal()
        

    def found_goal(self):

        if math.sqrt(pow(self.robot_position_grid[0]-self.goal[0],2)+pow(self.robot_position_grid[1]-self.goal[1],2))<=2:
            print("found goal, you finally succeeded at something")
            self.vel.angular.z=0
            self.vel.linear.x=0
            self.pub.publish(self.vel)            
            self.shutdown()



    def callback_function(self, odom_data):
        self.linearx=odom_data.pose.pose.position.x
        self.lineary=odom_data.pose.pose.position.y
        self.robot_position=([[self.linearx], [self.lineary]]) #position on not on array
        self.robot_position_grid=(self.linearx*10+(self.grid_size/2), self.lineary*10+(self.grid_size/2))
        print(f" robot position is {self.robot_position_grid}")
        linearz=odom_data.pose.pose.position.z
        quaternion_orientation= odom_data.pose.pose.orientation
        (roll, pitch, yaw) = euler_from_quaternion([quaternion_orientation.x, quaternion_orientation.y, quaternion_orientation.z, quaternion_orientation.w])
        if yaw < 0:
            self.real_yaw =yaw+2*math.pi
        else:
            self.real_yaw = yaw #angle of robot relative to world frame
        
        if self.robot_position_grid[0] >=self.grid_size-30 or self.robot_position_grid[1] >= self.grid_size-30 or self.robot_position_grid[0]<=30 or self.robot_position_grid[1]<=30:
            print("Warning, robot close to edge of map, will fall off into void soon...")
        if max(self.robot_position_grid)>=self.grid_size:
            print("fell into the void, shutting down")
            self.vel.angular.z=0
            self.vel.linear.x=0
            self.pub.publish(self.vel) 
            self.shutdown()           
        #print(self.real_yaw)
        #print(f"x is {self.linearx:.2f}, y is {self.lineary:.2f}, z is {linearz:.2f}, Yaw is {self.real_yaw:.2f}")
    



if __name__ == '__main__':
    
    rospy.loginfo('its working dipshit')
    Drive_at_block()
    rospy.spin()
