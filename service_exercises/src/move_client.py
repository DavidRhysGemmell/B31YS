#! /usr/bin/env python

import rospy
from srv_examples.srv import SetBool, SetBoolRequest
import sys

service_name = '/move_service'

rospy.init_node('move_service_client')

rospy.wait_for_service(service_name)

service = rospy.ServiceProxy(service_name, SetBool)

service_request = SetBoolRequest()
service_request.boolean_request = True

service_response = service(service_request)
print (service_response)