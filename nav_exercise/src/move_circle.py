#!/usr/bin/env python
# A simple ROS publisher node in Python

import rospy
from geometry_msgs.msg import Twist

class Publisher:
    
    def __init__(self):
        self.pub = rospy.Publisher('/cmd_vel', Twist, queue_size=10)
        rospy.init_node('publisher_node', anonymous=True)
        self.rate = rospy.Rate(10) # hz    
        self.ctrl_c = False
        rospy.on_shutdown(self.shutdownhook) 
        rospy.loginfo("publisher node is active...")

    def shutdownhook(self):
        self.shutdown_function()
        self.ctrl_c = True

    def shutdown_function(self):
        print("stopping publisher node at: {}".format(rospy.get_time()))
        vel_cmd = Twist()
        #vel_cmd.linear.x = 0.0 # m/s
        vel_cmd.angular.z = 0.0 # rad/s
        self.pub.publish(vel_cmd)

    def main_loop(self):
        while not self.ctrl_c:
            vel_cmd = Twist()
            vel_cmd.linear.x = 1.0 # m/s
            vel_cmd.angular.z = 1.820 # rad/s
            self.pub.publish(vel_cmd)
            self.rate.sleep()

if __name__ == '__main__':
    publisher_instance = Publisher()
    try:
        publisher_instance.main_loop()
    except rospy.ROSInterruptException:
        pass