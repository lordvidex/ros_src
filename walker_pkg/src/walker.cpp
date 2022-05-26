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
    for (int i = 0; i < 12; i++)
    {
        Direction direction(i*30);
        directions.push_back(direction);
        ROS_INFO("Direction's center: %f left: %f, right: %f", direction.center, direction.left, direction.right);
    }
    currentDirection = directions[6];
    
}

void Walker::scanCallback(const sensor_msgs::LaserScanConstPtr &scan)
{
    if(isTurning) {
        ROS_INFO("Scan callback returning");
        return ;
    }
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
            // create directions
            Direction* maxDirection;
            int maxInfs;
            int maxAverage;
            for (int i = 0; i < directions.size(); i++)
            {
                if(!directions[i].hasObstacleInFront(scan)) {
                    ROS_INFO("Direction %d - no obstacle", i);
                    int infs = directions[i].numberOfInfs(scan);
                    int average = directions[i].averageRangeInDirection(scan);
                    if (infs > maxInfs) {
                        maxDirection = &directions[i];
                        maxInfs = infs;
                        maxAverage = average;
                    } else if (infs == maxInfs && average > maxAverage) {
                        maxAverage = average;
                        maxDirection = &directions[i];
                    }
                }
            }
            isTurning = true;
            rotateTo(*maxDirection);
            // The default constructor will set all commands to 0
        }
    }
    else
    {
        isTurning = false;
        move();
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
    bool endCondition = false;
    ROS_INFO("Best dir=%f, Current dir=%f", direction.center, currentDirection.center);
    double turnAngle = direction.center - currentDirection.center;
    ros::Time startTime = ros::Time::now();
    ros::Time endTime = startTime + ros::Duration(abs(turnAngle) / ROTATE_SPEED);
    ROS_INFO("Start time is %f and end time is %f", startTime , endTime);
    ROS_INFO("Turning");
    while (ros::Time::now() < endTime)
    {
        twist.angular.z = turnAngle > 0 ? 1 : -1 * ROTATE_SPEED;
        pub.publish(twist);
        ros::spinOnce();
    }
    ROS_INFO("Angle reached, reseting movement");
    twist.angular.z = 0;
    pub.publish(twist); // stop rotating

    isTurning = false;
    isMoving = true; // start moving
    currentDirection = directions[6];
}

void Walker::odomCallback(const nav_msgs::OdometryConstPtr &msg)
{
    if (isTurning) {
        ROS_INFO("Odom callback returning");
        return ;
    }
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
    currentAngle = yaw;
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
        if (std::isnan(msg->ranges[currIndex])) {
            sum+=msg->range_max;
        }
        else {
            sum += msg->ranges[currIndex];
        }
    }
    return sum / (maxIndex - minIndex);
}

int Direction::numberOfInfs(const sensor_msgs::LaserScanConstPtr& msg) {
    int sum = 0;
    int minIndex = ceil((left - msg->angle_min) / msg->angle_increment);
    int maxIndex = floor((right - msg->angle_min) / msg->angle_increment);

    for (int currIndex = minIndex + 1; currIndex <= maxIndex; currIndex++)
    {
        if (std::isnan(msg->ranges[currIndex])) {
            sum++;
        }
    }
    return sum;
}
