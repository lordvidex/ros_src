#include <ros/ros.h>
#include <geometry_msgs/Twist.h>
#include <turtlesim/Pose.h>

ros::Publisher pub;
ros::Subscriber sub;
double initial_x = -1;
double initial_y = -1;
double initial_z = -1;
double final_x;
double final_y;
double final_z;
const double LINEAR_DISTANCE = 1;
const double ANGULAR_DISTANCE = 45;

bool printPos = false;

// method to move the robot by 1m
void move(double speed);

// method to rotate the robot by 45 degs
void rotate(double speed);

/**
 * @brief Subscriber callback for /turtle1/pose
 *
 * @param pose Turtlesim pose object
 */
void callback(turtlesim::PosePtr const &pose);

double deg2rad(double degreeValue);

int main(int argc, char **argv)
{
    ros::init(argc, argv, "turtle_mover");
    ros::NodeHandle n;
    pub = n.advertise<geometry_msgs::Twist>("turtle1/cmd_vel", 10);
    sub = n.subscribe("/turtle1/pose", 1000, callback);

    while (ros::ok() && !printPos)
    {
        ros::spinOnce();
    }

    return 0;
}

void move(double speed)
{
    ROS_INFO("Moving robot 1m");
    geometry_msgs::Twist vel_msg;
    vel_msg.linear.x = speed;
    vel_msg.linear.y = 0;
    vel_msg.linear.z = 0;
    vel_msg.angular.x = 0;
    vel_msg.angular.y = 0;
    vel_msg.angular.z = 0;
    pub.publish(vel_msg);
}

void rotate(double speedInDegrees)
{
    ROS_INFO("Rotating robot 45 degs");
    double angular_speed = speedInDegrees * 2 * M_PI / 360;
    geometry_msgs::Twist vel_msg;
    vel_msg.linear.x = 0;
    vel_msg.linear.y = 0;
    vel_msg.linear.z = 0;
    vel_msg.angular.x = 0;
    vel_msg.angular.y = 0;
    vel_msg.angular.z = angular_speed;
    pub.publish(vel_msg);
}

void callback(turtlesim::PosePtr const &pose)
{
    // get initial position and initialize movement
    if (initial_x == -1)
    {
        initial_x = pose->x;
        initial_y = pose->y;
        initial_z = pose->theta;
        final_x = initial_x + LINEAR_DISTANCE;
        final_y = initial_y;
        final_z = initial_z + deg2rad(ANGULAR_DISTANCE);
    }

    if (pose->x < final_x) {
        move(1);
    } else if (pose->theta < final_z){
        rotate(10);
    } else {
        ROS_INFO("The initial position is x: %f, y: %f, z: %f, and the final position is x: %f, y: %f, z: %f",
                 initial_x,
                 initial_y,
                 initial_z,
                 pose->x,
                 pose->y,
                 pose->theta
                 );
        printPos = true; // stops the program
    }
}

double deg2rad(double degreeValue) {
    return degreeValue * 2 * M_PI / 360;
}