#! /usr/bin/env python

import rospy
from geometry_msgs.msg import Twist
from srv_examples.srv import SetBool, SetBoolResponse

pub = rospy.Publisher('/cmd_vel', Twist, queue_size=1)
vel = Twist()

def callback_function(service_request):

    service_response = SetBoolResponse()

    if service_request.boolean_request == True:
        print('the move_service server recieved a "True" request, the robot will now move for 5 seconds...')

        StartTime = rospy.get_rostime()

        vel.linear.x = 0.1
        pub.publish(vel)

        rospy.loginfo('Published the velocity command to /cmd_vel')
        while rospy.get_rostime().secs - StartTime.secs < 5:
            continue

        rospy.loginfo('5 seconds have elapsed, stopping the robot...')

        vel.linear.x = 0.0
        pub.publish(vel)

        service_response.boolean_response = True
        service_response.response_message = 'Request complete.'
    else:
        service_response.boolean_response = False
        service_response.response_message = 'Nothing happened, set boolean_request to true next time.'
    return service_response

rospy.init_node('move_service_server')
my_service = rospy.Service('/move_service', SetBool, callback_function)
rospy.loginfo('the move_service server is ready to be called...')
rospy.spin()