#include "ros/ros.h"
#include "std_msgs/String.h"
#include <sstream>
int main(int argc, char **argv) {
    ros::init(argc, argv, "talker"); // initialize with the name of the node
    ros::NodeHandle n; // create the instance of the nodehandler
    ros::Publisher chatter_pub = 
        n.advertise<std_msgs::String>("chatter", 1000); // "chatter" is the name of the topic, 1000 is the queue size
    ros::Rate loop_rate(10);
    int count = 0;
    while (ros::ok()) {
        std_msgs::String msg;
        std::stringstream ss;
        ss << "hello world " << count;
        msg.data = ss.str();
        ROS_INFO("%s", msg.data.c_str());
        
        chatter_pub.publish(msg); // publishes the message
        
        ros::spinOnce();
        loop_rate.sleep();
        ++count;
    }
    return 0;
}
