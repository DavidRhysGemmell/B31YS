#! /usr/bin/env python

import rospy
from geometry_msgs.msg import Twist
from srv_examples.srv import TimedMovement, SetBoolResponse
from sensor_msgs.msg import LaserScan


class Drive_at_block:

    def __init__(self):       
        self.pub = rospy.Publisher('/cmd_vel', Twist, queue_size=1)
        self.sub = rospy.Subscriber('/scan', LaserScan, self.callback_laser)
        vel = Twist()

    def callback_laser(self, LaserMsg):
        self.Laser_scan_array = LaserMsg
        self.closest_object_range=min(self.Laser_scan_array.ranges)
        #print(closest_object_range)
        self.closest_object_angle=self.Laser_scan_array.ranges.index(min(self.Laser_scan_array.ranges))
        #print(closest_object_angle)   


    def callback_move(self, service_request):
        print(self.closest_object_angle)







rospy.init_node('move_service_server')
rospy.loginfo('its working dipshit')
Drive_at_block()
rospy.spin()



#laser_scan_array=ranges
#closest_object=min(laser_scan_array)
#position_of_closest_object_in_array=laser_scan_array.index(closest_object)
#turn_angle=angle_increment *
#[position_of_closest_object_in_array*angle_increment]
