#!/usr/bin/env python
# -*- coding: utf-8 -*-
import roslib; roslib.load_manifest('ros_package_template')
import rospy
import sys
import time
from ros_package_template.msg import *

def topic_callback(data):
    print("Received: '%s'" % data.data)

def talker():
    pub = rospy.Publisher('MyPythonTopic', CustomMessageType, queue_size=10)
    rospy.init_node('ros_package_template_python')
    rospy.Subscriber("MyCppTopic", CustomMessageType, topic_callback)
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
