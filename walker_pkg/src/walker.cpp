#include "Walker.h"
#include <tf/tf.h>
#include <geometry_msgs/Twist.h>

Walker::Walker()
{
    isMoving = true;
    isTurning = false;
    turnMultiplier = -2;
    startAngle = -1;
    currentAngle = -1;
    // publisher
    pub = nh.advertise<geometry_msgs::Twist>("cmd_vel", 10);
    // subscribe to laser scans
    scanSub = nh.subscribe("scan", 10, &Walker::scanCallback, this);
}

void Walker::scanCallback(const sensor_msgs::LaserScanConstPtr &scan)
{
    bool isObstacleInFront = false;

    // Find the closest range between the defined minimum and maximum angles
    int minIndex = ceil((MIN_SCAN_ANGLE - scan->angle_min) / scan->angle_increment);
    int maxIndex = floor((MAX_SCAN_ANGLE - scan->angle_min) / scan->angle_increment);

    for (int currIndex = minIndex + 1; currIndex <= maxIndex; currIndex++)
    {
        if (scan->ranges[currIndex] < MIN_DIST_FROM_OBSTACLE)
        {
            isObstacleInFront = true;
            break;
        }
    }
    geometry_msgs::Twist msg;
    if (isObstacleInFront)
    {
        if (!isTurning)
        {
            // choose to turn left or right
            Direction right(180 - SCAN_RANGE, SCAN_RANGE / 2);
            Direction left(180 + SCAN_RANGE, SCAN_RANGE / 2);

            // turn left if the average range is greater in left
            turnMultiplier = left.averageRangeInDirection(scan) > right.averageRangeInDirection(scan) ? 1 : -1;
            isTurning = true;
            // The default constructor will set all commands to 0
        }
        ROS_INFO("Turn %s!", turnMultiplier == 1 ? "Left" : "Right");
        msg.angular.z = ROTATE_SPEED * turnMultiplier;
        msg.linear.x = 0;
        pub.publish(msg);
    }
    else
    {
        isTurning = false;
        ROS_INFO("Keep moving");
        msg.linear.x = -1 * FORWARD_SPEED;
        msg.angular.z = 0;
        pub.publish(msg);
    }
}
void Walker::move(double x)
{
    geometry_msgs::Twist msg;
    msg.angular.z = 0;
    msg.linear.x = -1 * x;
    pub.publish(msg);
}
void Walker::move()
{
    move(FORWARD_SPEED);
}

void Walker::rotateTo(Direction &direction)
{
    ROS_INFO("started turning");
    geometry_msgs::Twist twist;
    twist.angular.z = ROTATE_SPEED;
    bool endCondition = false;
    double finishAngle = startAngle + (direction.center - currentDirection.center);
    if (finishAngle < 0)
    {
        finishAngle = finishAngle + 2 * M_PI;
    }
    while (!Direction::near(finishAngle, currentAngle))
    {
        ROS_INFO("Turning");
        pub.publish(twist);
        ros::spinOnce();
    }
    ROS_INFO("Angle reached, reseting movement");
    twist.angular.z = 0;
    pub.publish(twist); // stop rotating

    isTurning = false;
    isMoving = true; // start moving
    startAngle = -1;
    currentAngle = -1;
    currentDirection = directions[4];
}

void Walker::odomCallback(const nav_msgs::OdometryConstPtr &msg)
{
    tf::Quaternion q(
        msg->pose.pose.orientation.x,
        msg->pose.pose.orientation.y,
        msg->pose.pose.orientation.z,
        msg->pose.pose.orientation.w);
    tf::Matrix3x3 m(q);
    double roll, pitch, yaw;
    m.getRPY(roll, pitch, yaw);
    // convert negative yaw to positive
    if (yaw < 0)
    {
        yaw = yaw + 2 * M_PI;
    }
    ROS_INFO("Angle is %f", yaw);
    if (!isMoving)
    {
        if (startAngle == -1)
        {
            startAngle = yaw;
            ROS_INFO("setting startAngle to %f", yaw);
        }
        else
        {
            currentAngle = yaw;
            ROS_INFO("currentAngle is %f but startAngle is %f", currentAngle, startAngle);
        }
    }
}

void Walker::startMoving()
{
    ros::spin();
}

bool Direction::hasObstacleInFront(const sensor_msgs::LaserScanConstPtr &msg)
{
    bool isObstacleInFront = false;

    // Find the closest range between the defined minimum and maximum angles
    int minIndex = ceil((left - msg->angle_min) / msg->angle_increment);
    int maxIndex = floor((right - msg->angle_min) / msg->angle_increment);

    for (int currIndex = minIndex + 1; currIndex <= maxIndex; currIndex++)
    {
        if (!std::isnan(msg->ranges[currIndex]) &&
            msg->ranges[currIndex] < Walker::MIN_DIST_FROM_OBSTACLE)
        {
            isObstacleInFront = true;
            break;
        }
    }
    return isObstacleInFront;
}

double Direction::averageRangeInDirection(const sensor_msgs::LaserScanConstPtr &msg)
{
    double sum = 0;
    int minIndex = ceil((left - msg->angle_min) / msg->angle_increment);
    int maxIndex = floor((right - msg->angle_min) / msg->angle_increment);

    for (int currIndex = minIndex + 1; currIndex <= maxIndex; currIndex++)
    {
        if (!std::isnan(msg->ranges[currIndex]))
        {
            sum += msg->ranges[currIndex];
        }
    }
    return sum / (maxIndex - minIndex);
}
