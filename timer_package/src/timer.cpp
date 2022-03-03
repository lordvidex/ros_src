#include "ros/ros.h"

std::string stampToString(const ros::Time& stamp, const std::string format="%H:%M %d.%m.%Y");

int main(int argc, char **argv)
{
    ros::init(argc, argv, "timer_node");
    ros::NodeHandle n;
    ros::Rate loop_rate(2);

    int count = 0;
    
    while(ros::ok()) {
        ros::Time ti = ros::Time::now();

        ROS_INFO_STREAM(stampToString(ti));
        ros::spinOnce();
        loop_rate.sleep();
        count++;
    }
    return 0;
}

std::string stampToString(const ros::Time& stamp, const std::string format) {
    const int output_size = 100;
    char output[output_size];

    std::time_t raw_time = static_cast<time_t>(stamp.sec);
    struct tm* timeinfo = localtime(&raw_time);
    std::strftime(output, output_size, format.c_str(), timeinfo);
    
    return std::string(output);
}
