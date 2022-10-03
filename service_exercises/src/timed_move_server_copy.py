#! /usr/bin/env python

import rospy
from geometry_msgs.msg import Twist
from srv_examples.srv import TimedMovement, SetBoolResponse

pub = rospy.Publisher('/cmd_vel', Twist, queue_size=1)
vel = Twist()

def callback_function(service_request):

    service_response = TimedMovement()

    if service_request.movement_request == "w":
        print('the move_service server recieved a "w" request, the robot will now move for ? seconds...')

        StartTime = rospy.get_rostime()

        vel.linear.x = 0.5
        pub.publish(vel)

        rospy.loginfo('Published the velocity command to /cmd_vel')
        while rospy.get_rostime().secs - StartTime.secs < service_request.duration:
            continue

        rospy.loginfo('? seconds have elapsed, stopping the robot...')

        vel.linear.x = 0.0
        pub.publish(vel)

        service_response.success = True
        service_response.response_message = 'Request complete. Robot go BRRRRRRRRRRRRRRRRRRRRRRRRR'
    if service_request.movement_request == "a":
        print('the move_service server recieved a "a" request, the robot will now turn left for ? seconds...')

        StartTime = rospy.get_rostime()

        vel.angular.z = 0.5
        pub.publish(vel)

        rospy.loginfo('Published the velocity command to /cmd_vel')
        while rospy.get_rostime().secs - StartTime.secs < service_request.duration:
            continue

        rospy.loginfo('? seconds have elapsed, stopping the robot...')

        vel.angular.z = 0.0
        pub.publish(vel)

        service_response.success = True
        service_response.response_message = 'Request complete. Robot go BRRRRRRRRRRRRRRRRRRRRRRRRR'
    if service_request.movement_request == "d":
        print('the move_service server recieved a "d" request, the robot will now turn right for ? seconds...')

        StartTime = rospy.get_rostime()

        vel.angular.z = -0.5
        pub.publish(vel)

        rospy.loginfo('Published the velocity command to /cmd_vel')
        while rospy.get_rostime().secs - StartTime.secs < service_request.duration:
            continue

        rospy.loginfo('? seconds have elapsed, stopping the robot...')

        vel.angular.z = 0.0
        pub.publish(vel)

        service_response.success = True
        service_response.response_message = 'Request complete. Robot go BRRRRRRRRRRRRRRRRRRRRRRRRR'
    if service_request.movement_request == "s":
        print('the move_service server recieved a "s" request, the robot will now move backwards for ? seconds...')

        StartTime = rospy.get_rostime()

        vel.linear.x = -0.5
        pub.publish(vel)

        rospy.loginfo('Published the velocity command to /cmd_vel')
        while rospy.get_rostime().secs - StartTime.secs < service_request.duration:
            continue

        rospy.loginfo('? seconds have elapsed, stopping the robot...')

        vel.linear.x = 0.0
        pub.publish(vel)

        service_response.TimedMovement.success = True
        service_response.TimedMovement.response_message = 'Request complete. Robot go BRRRRRRRRRRRRRRRRRRRRRRRRR'        
    else:
        service_response.success = False
        service_response.response_message = 'Nothing happened, you are not pressing anything or have done something wrong'
    return service_response

rospy.init_node('move_service_server')
my_service = rospy.Service('/move_service', TimedMovement, callback_function)
rospy.loginfo('the move_service server is ready to be called...')
rospy.spin()