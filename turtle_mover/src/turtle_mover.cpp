#include <ros/ros.h>
#include <geometry_msgs/Twist.h>
#include <turtlesim/Pose.h>

ros::Publisher pub;
ros::Subscriber sub;
double initial_x = -1;
double initial_y = -1;
double final_x = -1;
double final_y = -1;

const double METERS_FORWARD = 1;
bool printPos = false;

// method to move the robot by 1m
void move(double speed, double distance);

// method to rotate the robot by 45 degs
void rotate(double speed, double deg);

/**
 * @brief Subscriber callback for /turtle1/pose
 * 
 * @param pose Turtlesim pose object
 */
void callback(turtlesim::PosePtr const &pose);




int main(int argc, char **argv)
{
    ros::init(argc, argv, "turtle_mover");    
    ros::NodeHandle n;
    pub = n.advertise<geometry_msgs::Twist>("turtle1/cmd_vel", 10);
    sub = n.subscribe("/turtle1/pose",1000, callback);
    ros::Rate sleeper(0.5);
    sleeper.sleep();
    
    
    
    move(0.2,1);
    rotate(5,45);
    printPos = true;
    ros::spinOnce();
    
    return 0;
}

void move(double speed, double distance) {
    ROS_INFO("Moving robot 1m");
    geometry_msgs::Twist vel_msg;
    vel_msg.linear.x = speed;
    vel_msg.linear.y = 0;
    vel_msg.linear.z = 0;
    vel_msg.angular.x = 0;
    vel_msg.angular.y = 0;
    vel_msg.angular.z = 0;
    
    // get the current time
    double t1 = ros::Time::now().toSec();
    double current_distance = 0;

    ros::Rate loop(30);
    do {
        pub.publish(vel_msg);
        double t2 = ros::Time::now().toSec();
        current_distance = (t2-t1) * speed;
        ROS_INFO("current distance %f", current_distance);
        ros::spinOnce();
        loop.sleep();
        } while(current_distance < METERS_FORWARD);

        // stop the robot
        vel_msg.linear.x = 0;
        pub.publish(vel_msg);
}

void rotate(double speed, double angle) {
    ROS_INFO("Rotating robot 45 degs");
    double angular_speed = speed * 2 * M_PI / 360;
    geometry_msgs::Twist vel_msg;
    vel_msg.linear.x = 0;
    vel_msg.linear.y = 0;
    vel_msg.linear.z = 0;
    vel_msg.angular.x = 0;
    vel_msg.angular.y = 0;
    vel_msg.angular.z = angular_speed;

    // M_PI
    double t1 = ros::Time::now().toSec();
    double current_angle = 0;
    double final_angle = angle * 2 * M_PI / 360;
    ros::Rate loop(30);
    do {
        pub.publish(vel_msg);
        double t2 = ros::Time::now().toSec();
        current_angle = (t2-t1) * angular_speed;
        ros::spinOnce();
        loop.sleep();
    } while(current_angle < final_angle);
    // stop
    vel_msg.angular.z = 0;
    pub.publish(vel_msg);
}


void callback(turtlesim::PosePtr const &pose) {
    if (initial_x == -1) {
        initial_x = pose->x;
        initial_y = pose->y;
    }
    if (printPos) {
        ROS_INFO("The initial position is x: %f, y: %f and the final position is x: %f, y: %f",
        initial_x,
        initial_y,
        pose->x,
        pose->y);
        printPos = false;
    }
}