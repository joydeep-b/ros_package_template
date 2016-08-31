#include "ros/ros.h"
#include "std_msgs/String.h"
#include "ros_package_template/MyMessageType.h"
#include "ros_package_template/MyServiceType.h"

#include <iostream>
#include <sstream>

using std::cout;
using std::endl;
using std::stringstream;

// Define a callback function to handle service calls to the
// ROS service "/MyCppService".
bool MyServiceCallback(ros_package_template::MyServiceType::Request& req,
                       ros_package_template::MyServiceType::Response& res) {
  // In the following statement, "my_command" is a member of the service type.
  // See srv/MyServiceType.srv for the definition.
  cout << "Service request: " << req.my_command << endl;

  // Return a response, which in this case happens to be a simple string.
  // Again, see srv/MyServiceType.srv for the definition.
  res.my_response =  "My response";

  return true;
}


// Define a callback function to handle new messages received on the
// ROS topic "/MyCppTopic".
void MyTopicCallback(const ros_package_template::MyMessageType& msg) {
  cout << "Received: " << msg.data << endl;
}

int main(int argc, char **argv) {
  // Initialize a ROS node named "my_cpp_node".
  ros::init(argc, argv, "my_cpp_node");

  // Instantiate a Roscpp Node.
  ros::NodeHandle n;

  // Advertise a ROS topic called "/MyCppTopic", of type
  // "ros_package_template/MyMessageType", with a queue size of 10 message.
  ros::Publisher my_publisher =
      n.advertise<ros_package_template::MyMessageType>("MyCppTopic", 10);

  // Subscribe to the ROS topic called "MyCppTopic", of type
  // "ros_package_template/MyMessageType", and tell ROS that the function
  // "my_topic_callback" will handle new messages received on the topic.
  ros::Subscriber my_subscriber =
      n.subscribe("MyPythonTopic",
                  10,
                  MyTopicCallback);

  // Create a new ROS rate limiting timer, and set it to 10Hz.
  ros::Rate loop_rate(10);

  while (ros::ok()) {
    // This is what the program will publish.
    std::stringstream ss;
    ss << "Roscpp hello world " << ros::Time::now();

    // Publish the above-constucted string with the publisher that we defined
    // earlier.
    ros_package_template::MyMessageType msg;
    msg.data = ss.str();
    my_publisher.publish(msg);
    cout << "Sent: '" << msg.data << "'\n";

    // Sleep appropriately so as to limit the execution rate to the
    // pre-defined value.
    ros::spinOnce();
    loop_rate.sleep();
  }

  return 0;
}
