#ifndef WALKER_H
#define WALKER_H

#include <ros/ros.h>
#include <sensor_msgs/LaserScan.h>
#include <nav_msgs/Odometry.h>


struct Direction
{
    double center;
    double left;
    double right;

    // constructors should be in degrees for convenience
    Direction(double center = 180, double width = 15)
    {
        this->center = deg2Radians(center);
        left = deg2Radians(center - width);
        right = deg2Radians(center + width);
    };

    bool equals(Direction& other) const 
    {
         if(this == &other || center == other.center) return true;
        return false;
    }

    bool operator==(Direction& other) const
    {
       return equals(other);
    }

    bool operator!=(Direction& other) const
    {
        return !equals(other);
    }
    
    static double deg2Radians(double degrees)
    {
        return (degrees / 180) * M_PI;
    };

    static bool near(double rad1, double rad2) {
        return abs(rad1-rad2) <= deg2Radians(5);
    }

    static double rad2Deg(double radians) 
    {
        return (radians / M_PI) * 180;
    };


    bool hasObstacleInFront(const sensor_msgs::LaserScanConstPtr &msg);
    double averageRangeInDirection(const sensor_msgs::LaserScanConstPtr &msg);
    
};

class Walker
{
public:
    // Tunable parameters
    static constexpr double FORWARD_SPEED = 0.4;
    static constexpr double ROTATE_SPEED = 20 * M_PI / 180;
    static constexpr float MIN_DIST_FROM_OBSTACLE = 0.6f;
    static constexpr float SCAN_RANGE = 45;
    static constexpr double MIN_SCAN_ANGLE = M_PI+(-(SCAN_RANGE/2)/180*M_PI);
    static constexpr double MAX_SCAN_ANGLE = M_PI+((SCAN_RANGE/2)/180*M_PI);
    void startMoving();
    Walker();

private:
    ros::NodeHandle nh;
    ros::Publisher pub;
    ros::Subscriber scanSub;
    ros::Subscriber odomSub;

    /* subscriber callbacks */
    void scanCallback(const sensor_msgs::LaserScan::ConstPtr &scan);
    void odomCallback(const nav_msgs::Odometry::ConstPtr &odom);

    /* control parameters */
    bool isMoving;
    bool isTurning;
    int turnMultiplier;
    double startAngle; // angle from which turning is started
    double currentAngle;
    std::vector<Direction> directions;
    Direction currentDirection;

    /* functions */
    void rotateTo(Direction& direction);
    void move(double x);
    void move();
}; 

#endif
