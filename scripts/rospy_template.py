#!/usr/bin/env python
# The line above tells the OS what interpreter to use for this script.

# Import roslib, and load manifest of the specified package
# (ros_package_template) to include all dependencies.
import roslib; roslib.load_manifest('ros_package_template')
import rospy
import sys
import time

# Load all message definitions from the ROS package ros_package_template.
from ros_package_template.msg import *
# Load all service definitions from the ROS package ros_package_template.
from ros_package_template.srv import *

# Define a callback function to handle service calls to the
# ROS service "/MyPythonService".
def my_service_callback(req):
    # In the following statement, "my_command" is a member of the service type.
    # See srv/MyServiceType.srv for the definition.
    print("Service request: '%s'" % req.my_command)

    # Return a response, which in this case happens to be a simple string.
    # Again, see srv/MyServiceType.srv for the definition.
    return "My response"

# Define a callback function to handle new messages received on the
# ROS topic "/MyCppTopic".
def my_topic_callback(data):
    print("Received: '%s'" % data.data)

def main():
    # Initialize a ROS node named "my_python_node".
    rospy.init_node('my_python_node')

    # Advertise a ROS topic called "/MyPythonTopic", of type
    # "ros_package_template/MyMessageType".
    my_publisher = rospy.Publisher(
            'MyPythonTopic',
            MyMessageType,
            queue_size=10)

    # Subscribe to the ROS topic called "MyCppTopic", of type
    # "ros_package_template/MyMessageType", and tell ROS that the function
    # "my_topic_callback" will handle new messages received on the topic.
    rospy.Subscriber("MyCppTopic", MyMessageType, my_topic_callback)

    # Advertise a ROS service named "/MyPythonService", of type
    # "ros_package_template/MyServiceType", and tell ROS that the function
    # "my_service_callback" will handle calls to this service.
    # You can manually call this service from the command line by exeuting:
    # rosservice call /MyPythonService "Hello service"
    my_service = rospy.Service(
            'MyPythonService',
            MyServiceType,
            my_service_callback)

    # Create a new ROS rate limiting timer, and set it to 10Hz.
    rate = rospy.Rate(10)

    # Execute indefinitely until ROS tells the node to shut down.
    while not rospy.is_shutdown():
        # This is what the program will publish.
        hello_str = "Rospy hello world %s" % rospy.get_time()

        # Publish the above-constucted string with the publisher that we defined
        # earlier.
        my_publisher.publish(hello_str)
        print("Sent: '%s'" % hello_str)

        # Sleep appropriately so as to limit the execution rate to the
        # pre-defined value.
        rate.sleep()

if __name__ == '__main__':
    try:
        main()
    except rospy.ROSInterruptException:
        pass
