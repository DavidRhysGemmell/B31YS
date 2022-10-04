#! /usr/bin/env python

import rospy
from geometry_msgs.msg import Twist
from srv_examples.srv import TimedMovement, SetBoolResponse
from sensor_msgs.msg import LaserScan


class Drive_at_block:

    def __init__(self):
        self.closest_object_angle=0       
        self.pub = rospy.Publisher('/cmd_vel', Twist, queue_size=1)
        self.sub = rospy.Subscriber('/scan', LaserScan, self.callback_laser)
        self.vel = Twist()

    def callback_laser(self, LaserMsg):
        Laser_scan_array = LaserMsg
        closest_object_range=min(Laser_scan_array.ranges)
        #print(closest_object_range)
        self.closest_object_angle=Laser_scan_array.ranges.index(min(Laser_scan_array.ranges))
        print(self.closest_object_angle) 
        if self.closest_object_angle <= 5:
            self.vel.angular.z=0
            self.pub.publish(self.vel)
            print("Target Aquired") 
            self.vel.linear.x=1
            self.pub.publish(self.vel)          
        else:
            if self.closest_object_angle < 360:
                self.vel.angular.z=0.5
                self.pub.publish(self.vel)
                print("Turning left")
            else:
                if self.closest_object_angle >= 715:
                    self.vel.angular.z=0
                    self.pub.publish(self.vel)
                    print("Target Aquired")
                    self.vel.linear.x=1
                    self.pub.publish(self.vel)                         
                else:
                    self.vel.angular.z=-0.5
                    self.pub.publish(self.vel)
                    print("Turning right")                

        


    #def callback_move(self, service_request):
        #print(self.closest_object_angle)







rospy.init_node('move_service_server')
rospy.loginfo('its working dipshit')
Drive_at_block()
rospy.spin()



#laser_scan_array=ranges
#closest_object=min(laser_scan_array)
#position_of_closest_object_in_array=laser_scan_array.index(closest_object)
#turn_angle=angle_increment *
#[position_of_closest_object_in_array*angle_increment]
