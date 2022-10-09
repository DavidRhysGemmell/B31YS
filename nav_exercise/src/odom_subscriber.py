#!/usr/bin/env python
# A simple ROS subscriber node in Python

import rospy
from nav_msgs.msg import Odometry
from tf.transformations import euler_from_quaternion
class Subscriber:

    def __init__(self):
        rospy.init_node('subscriber_node', anonymous=True)
        self.sub = rospy.Subscriber("/odom", Odometry, self.callback_function)
        rospy.loginfo("subscriber node is active...")

    def callback_function(self, odom_data):
        linearx=odom_data.pose.pose.position.x
        lineary=odom_data.pose.pose.position.y
        linearz=odom_data.pose.pose.position.z
        thetaw=odom_data.pose.pose.orientation.w
        (roll, pitch, yaw) = euler_from_quaternion([linearx, lineary, linearz, thetaw])
        print(f"x is {linearx:.2f}, y is {lineary:.2f}, z is {linearz:.2f}, Yaw is {yaw:.2f}")
    
    def main_loop(self):
        rospy.spin()

if __name__ == '__main__':
    subscriber_instance = Subscriber()
    subscriber_instance.main_loop()