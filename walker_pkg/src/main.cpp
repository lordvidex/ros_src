#include "Walker.h"

int main(int argc, char** argv)
{
    // initialize a node
    ros::init(argc, argv, "walker_node");
    
    // creates a new walker object
    Walker walker;

    // starts the robot loop
    walker.startMoving();
    ROS_INFO("Walker stopping");

    return 0;
}