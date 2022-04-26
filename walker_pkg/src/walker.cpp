#include "Walker.h"
#include <tf/tf.h>
#include <geometry_msgs/Twist.h>

Walker::Walker()
{
    isMoving = true;
    isTurning = false;
    startAngle = -1;
    currentAngle = -1;
    // publisher
    pub = nh.advertise<geometry_msgs::Twist>("cmd_vel", 1);
    // subscribe to laser scans
    scanSub = nh.subscribe("scan", 1, &Walker::scanCallback, this);
    // subscribe to odom scans
    odomSub = nh.subscribe("odom", 1, &Walker::odomCallback, this);

    // add to directions and set current direction
    directions.push_back(Direction(0));
    directions.push_back(Direction(45));
    directions.push_back(Direction(90));
    directions.push_back(Direction(135));
    directions.push_back(Direction(180));
    directions.push_back(Direction(225));
    directions.push_back(Direction(270));
    directions.push_back(Direction(315));
    currentDirection = directions[4];
}

void Walker::scanCallback(const sensor_msgs::LaserScanConstPtr &msg)
{
    if (!isMoving)
        return;
    if (isTurning)
        return;
    if (currentDirection.hasObstacleInFront(msg))
    {
        ROS_INFO("STOPEEEEEDDDDDD");
        move(0);
        isMoving = false;
        while (startAngle == -1)
        { // wait for odom subscriber to set the angle
            // then we can find the best angle to turn to
            ros::spinOnce();
        }
        ROS_INFO("Gotteen a start angle");
        Direction &maxDirection = currentDirection;
        double maxAverage = -1;
        for (Direction &dir : directions)
        {
            if (dir != currentDirection && !dir.hasObstacleInFront(msg))
            {
                ROS_INFO("The direction %f does not have an obstacle", dir.center);
                double temp = dir.averageRangeInDirection(msg);
                if (temp > maxAverage)
                {
                    maxDirection = dir;
                    maxAverage = temp;
                }
            }
        }
        ROS_INFO("The best direction is %f", Direction::rad2Deg(maxDirection.center));
        isTurning = true;
        ROS_INFO("this should turn");
        rotateTo(maxDirection);
    }
    else
    {
        move();
    }
}
void Walker::move(double x)
{
    geometry_msgs::Twist msg;
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
    if (finishAngle < 0) {
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
    ros::Rate rate(30);

    while (ros::ok())
    {
        ros::spinOnce();
        rate.sleep();
    }
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
