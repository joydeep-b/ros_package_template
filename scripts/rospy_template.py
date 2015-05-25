#!/usr/bin/env python
# -*- coding: utf-8 -*-
import roslib; roslib.load_manifest('ros_package_template')
import rospy
import sys
import time
from ros_package_template.msg import *
from ros_package_template.srv import *

def service_callback(req):
    print("Service request: '%s'" % req.my_command)
    return "My response"

def topic_callback(data):
    print("Received: '%s'" % data.data)

def talker():
    pub = rospy.Publisher('MyPythonTopic', CustomMessageType, queue_size=10)
    rospy.init_node('ros_package_template_python')
    rospy.Subscriber("MyCppTopic", CustomMessageType, topic_callback)
    my_service = rospy.Service('example_rospy_service', CustomService,
            service_callback)
    rate = rospy.Rate(10) # 10hz
    while not rospy.is_shutdown():
        hello_str = "hello world %s" % rospy.get_time()
        print("Sent: '%s'" % hello_str)
        pub.publish(hello_str)
        rate.sleep()

if __name__ == '__main__':
    try:
        talker()
    except rospy.ROSInterruptException:
        pass
