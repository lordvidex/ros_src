#include "ros/ros.h"
#include "geometry_msgs/Twist.h"
#include "turtlesim/Pose.h"
#include <iostream>
using namespace std;

void poseCallback(const turtlesim::PoseConstPtr& msg) // Topic messages callback
{
  ROS_INFO("x: %.2f, y: %.2f", msg->x, msg->y);
}

int main(int argc, char ** argv)
{
    const double FORWARD_SPEED_MPS = 0.5;
    string robot_name = string(argv[1]);

    ros::init(argc, argv, "move_turtle");   //Initialize the node
    ros::NodeHandle node;

    // A publisher for the movement data
    ros::Publisher pub = node.advertise<geometry_msgs::Twist>(robot_name + "/cmd_vel", 10);

    // A listener for pose
    ros::Subscriber sub = node.subscribe(robot_name + "/pose", 10, poseCallback);

    // Drive forward at a given speed. The robot points up the x-axis.
    // The default constuctor will set all commands to 0.
    geometry_msgs::Twist msg;
    msg.linear.x = FORWARD_SPEED_MPS;

    // Loop at 10Hz, publishing movement conmmands until we shut down.
    ros::Rate rate(10);
    ROS_INFO("Starting to move forward");

    while (ros::ok())
    {
      pub.publish(msg);
      ros::spinOnce(); // Allow processing of incoming messages
      rate.sleep();
    }

    return 0;
}
