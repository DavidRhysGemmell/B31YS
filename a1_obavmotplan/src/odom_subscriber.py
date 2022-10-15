#!/usr/bin/env python
# A simple ROS subscriber node in Python

import rospy
from nav_msgs.msg import Odometry
from tf.transformations import euler_from_quaternion
import math
class Subscriber:

    def __init__(self):
        rospy.init_node('subscriber_node', anonymous=True)
        self.sub = rospy.Subscriber("/odom", Odometry, self.callback_function)
        rospy.loginfo("subscriber node is active...")

    def callback_function(self, odom_data):
        linearx=odom_data.pose.pose.position.x
        lineary=odom_data.pose.pose.position.y
        #robot_position=([[linearx+7], [lineary+7]])
        #print(self.robot_position)
        linearz=odom_data.pose.pose.position.z
        quaternion_orientation= odom_data.pose.pose.orientation
        (roll, pitch, yaw) = euler_from_quaternion([quaternion_orientation.x, quaternion_orientation.y, quaternion_orientation.z, quaternion_orientation.w])
        if yaw < 0:
            real_yaw = (yaw*(180/math.pi))+360
        else:
            real_yaw = yaw*(180/math.pi) #angle of robot relative to world frame     
        print(f"x is {linearx:.2f}, y is {lineary:.2f}, z is {linearz:.2f}, Yaw is {real_yaw:.2f}")
    def main_loop(self):
        rospy.spin()

if __name__ == '__main__':
    subscriber_instance = Subscriber()
    subscriber_instance.main_loop()
