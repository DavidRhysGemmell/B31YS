#! /usr/bin/env python

import rospy
from geometry_msgs.msg import Twist
from sensor_msgs.msg import LaserScan
from nav_msgs.msg import Odometry
from tf.transformations import euler_from_quaternion
import numpy as np
import math
from matplotlib import pyplot as plt
import sys

class Drive_at_block:

    def __init__(self): #initialise variables, subscribers, publishers etc.
        rospy.init_node('go_robot_go')
        self.x_goal=int(rospy.get_param("/go_robot_go/x_goal"))
        self.y_goal=int(rospy.get_param("go_robot_go/y_goal"))
        self.goal=(self.x_goal,self.y_goal)
        #self.goal=(140,140)
        print(self.goal)
        self.grid_size=200 #20*20m world split into 10cm grid squares
        self.grid=np.zeros((self.grid_size,self.grid_size))
        self.closest_object_angle=0       
        self.pub = rospy.Publisher('/cmd_vel', Twist, queue_size=1)
        self.sub = rospy.Subscriber('/scan', LaserScan, self.callback_laser)
        self.vel = Twist()
        self.sub_odom = rospy.Subscriber("/odom", Odometry, self.callback_function)
        self.real_yaw = 0
        rospy.loginfo("subscriber node is active...")       
        self.linearx=0
        self.lineary=0
        self.robot_position=([[0],[0]])
        self.q_0=(0,0)
        self.robot_position_grid=([[self.grid/2],[self.grid_size/2]])
        self.max_speed=0.5
        self.ctrl_c=False
        rospy.on_shutdown(self.shutdown) 
        self.on_target=False

 
        
    def shutdown(self):
        self.vel.angular.z=0
        self.vel.linear.x=0
        self.pub.publish(self.vel)
        # plt.imshow(self.grid) #uncomment to show map when ctrl_c
        # plt.show()            #uncomment to show map when ctrl_c
        self.vel.angular.z=0
        self.vel.linear.x=0
        self.pub.publish(self.vel)
        print("shutting down")
        sys.exit()

    def callback_laser(self, LaserMsg):
        self.Laser_scan_array = LaserMsg
        for i in range(720): #essentially index to match angles with laser ranges
            theta= (i/2) *math.pi/180 
            if self.Laser_scan_array.ranges[i] <5: #scans for obstacles within 5 meters and the minimum laser range.
                if self.Laser_scan_array.ranges[i] > self.Laser_scan_array.range_min: 
                    bodyframe=([[(self.Laser_scan_array.ranges[i]*math.cos(theta))], [self.Laser_scan_array.ranges[i]*math.sin(theta)]]) 
                    R=np.array([[math.cos(self.real_yaw),-1*math.sin(self.real_yaw)],[math.sin(self.real_yaw),math.cos(self.real_yaw)]])
                    ob= (self.robot_position)+(np.dot(R,bodyframe))                  
                    self.grid[(int(ob[0]*10))+int(self.grid_size/2), int(ob[1]*10)+int(self.grid_size/2)]=1 #sets the coordinates of obstacle on grid as 1
        for i in range(-1,1): #if the goal is set too close/in an obstacle, it stops.
            for j in range(-1,1):                    
                if self.grid[self.goal[i],self.goal[j]] == 1:
                    print("Goal is occupied by obstacle. Shutting down")
                    self.shutdown()
        self.potential_field()

    def potential_field(self):
        p_0=15 #range around object in grid squares that effects the robot
        q_goal=self.goal
        F_rep=(0,0)
        k_att=3
        k_rep=5
        q=self.robot_position_grid #robot position on grid
        p_goal=math.sqrt(pow(q_goal[0]-self.robot_position_grid[0],2)+pow(q_goal[1]-self.robot_position_grid[1],2)) #distance from robot to goal
        #print(p_goal)
        if  p_goal <= 2:
            self.found_goal() #if range to goal is close enough, calls found_goal and stops.
        #Attractive
        F_att=(-k_att*(q[0]-q_goal[0]),-k_att*(q[1]-q_goal[1])) #the attractive force pulling robot towards goal (vector form)
        
        for i in range(-p_0,p_0): #takes a p_0m range around the robot, based on the map though
            for j in range(-p_0,p_0):
                if self.grid[int(self.robot_position_grid[0])+i,int(self.robot_position_grid[1])+j] == 1: #takes all points on the map that have an obstacle
                    p_q=math.sqrt(pow(i,2)+pow(j,2)) # distance to obstacle
                        #Repulsive
                    if p_q<=1.4:
                        F_rep = (10000,10000)    #stops divide by 0, rep goes high
                    elif p_q<=p_0 and p_q>1.4:
                        F_rep=(F_rep[0]+(k_rep*((1/p_q)-(1/p_0))*(1/pow(p_q,2))*(q[0]-self.q_0[0])/p_q),F_rep[1]+(k_rep*((1/p_q)-(1/p_0))*(1/pow(p_q,2))*((q[1]-self.q_0[1])/p_q))) #the repulsive force, pushing robot away from obstacles. (vector form)
              
        potential_vector= (F_att[0]+F_rep[0],F_att[1]+F_rep[1]) # repulsive + attractive vectors gives overall vector
        self.potential_angle=np.arctan2(potential_vector[1],potential_vector[0]) #angle which the potential_vector gives
        if self.potential_angle<0:
            self.potential_angle=self.potential_angle + 2*math.pi #normally from -pi to pi. I need 0 tp 2pi

        self.potential_force= math.sqrt(pow(potential_vector[0],2)+pow(potential_vector[1],2))/100 #the size of the potential_vector, will be converted to give the speed of the robot. It tends to 0 when close to goal.
        if self.potential_force > self.max_speed: #keeps our max speed to the maximum desired speed of the robot. 
            self.potential_force = self.max_speed
                
        self.turn()

    def turn(self):
        print(f"angle is {(self.real_yaw-self.potential_angle)*180/math.pi}")
        if abs((self.real_yaw-self.potential_angle)*180/math.pi) <= 4: #if within 4 degrees of required angle (in world frame), the robot will move forward.
             self.vel.angular.z=0
             self.pub.publish(self.vel)
             self.on_target=True  
             print("on target")
             self.go_forward()
        elif ((self.real_yaw-self.potential_angle)*180/math.pi) <-4 or ((self.real_yaw-self.potential_angle)*180/math.pi)>180: # if difference in angle of robot to desired angle is less than -4 degrees, turn left
            self.on_target=False
            self.vel.linear.x=0
            self.vel.angular.z=0.5
            self.pub.publish(self.vel)
            print("turning left")
        elif ((self.real_yaw-self.potential_angle)*180/math.pi) >4 or ((self.real_yaw-self.potential_angle)*180/math.pi) <=180 : # if difference in angle of robot to desired angle is greater than 4 degrees, turn right
            self.on_target=False
            self.vel.linear.x=0
            self.vel.angular.z=-0.5
            self.pub.publish(self.vel)
            print("turning right")
                 
    def go_forward(self):
        if self.on_target==True: #checks if on target, goes forward
            self.vel.linear.x=self.potential_force
            self.pub.publish(self.vel)
        else:
            self.vel.linear.x=0
            self.pub.publish(self.vel)
        self.found_goal() #checks if found goal
        

    def found_goal(self): #termination criteria

        if math.sqrt(pow(self.robot_position_grid[0]-self.goal[0],2)+pow(self.robot_position_grid[1]-self.goal[1],2))<=2: #if robot within 20cm of goal
            print("found goal")
            self.vel.angular.z=0
            self.vel.linear.x=0
            self.pub.publish(self.vel)            
            self.shutdown()



    def callback_function(self, odom_data): #this is odometry subrsciber
        self.linearx=odom_data.pose.pose.position.x #x and y coordinates. z does not matter
        self.lineary=odom_data.pose.pose.position.y
        self.robot_position=([[self.linearx], [self.lineary]]) #position on not on array
        self.robot_position_grid=(self.linearx*10+(self.grid_size/2), self.lineary*10+(self.grid_size/2)) #robot position on grid
        print(f" robot position is {self.robot_position_grid}") # prints to let us know where it is
        quaternion_orientation= odom_data.pose.pose.orientation #orientation of robot
        (roll, pitch, yaw) = euler_from_quaternion([quaternion_orientation.x, quaternion_orientation.y, quaternion_orientation.z, quaternion_orientation.w]) #orientation converted from quarternion to euler angles
        if yaw < 0:
            self.real_yaw =yaw+2*math.pi #I'm using all angles from 0-360 or 0-2pi
        else:
            self.real_yaw = yaw #angle of robot relative to world frame
        
        if self.robot_position_grid[0] >=self.grid_size-30 or self.robot_position_grid[1] >= self.grid_size-30 or self.robot_position_grid[0]<=30 or self.robot_position_grid[1]<=30: #soon will get errors as plotting off the grid size
            print("Warning, robot close to edge of map, will fall off into void soon...")
        if max(self.robot_position_grid)>=self.grid_size:
            print("Fell into the void, shutting down")
            self.vel.angular.z=0
            self.vel.linear.x=0
            self.pub.publish(self.vel) 
            self.shutdown()           
    
if __name__ == '__main__':
    
    rospy.loginfo('Code is running')
    Drive_at_block()
    rospy.spin()
