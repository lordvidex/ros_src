import math
import rospy
from geometry_msgs.msg import Twist
from sensor_msgs.msg import LaserScan

class Direction:
    def __init__(self, center, width):
        self.center = self.deg2rad(center)
        self.left = self.deg2rad(center-width)
        self.right = self.deg2rad(center+width)
    
    def hasObstacle(self, scan: LaserScan) -> bool:
        obstacle = False
        minIndex = math.ceil((self.left-scan.angle_min) / scan.angle_increment)
        maxIndex = math.floor((self.right - scan.angle_min) / scan.angle_increment)

        for i in range(minIndex+1,maxIndex+1):
            if scan.ranges[i] != float('nan') and scan.ranges[i] < Walker.MIN_DIST_FROM_OBSTACLE:
                obstacle = True
                break
        return obstacle

    def infsAndAverageRange(self, scan: LaserScan):
        infs = 0
        average: float = 0
        minIndex = math.ceil((self.left-scan.angle_min) / scan.angle_increment)
        maxIndex = math.floor((self.right - scan.angle_min) / scan.angle_increment)
        for i in range(minIndex+1,maxIndex+1):
            if scan.ranges[i] == float('nan'):
                infs += 1
                average += scan.range_max
            else:
                average += scan.ranges[i]
        return infs, average / (maxIndex - minIndex)
            

    # helpers
    def deg2rad(self, degrees):
        return (degrees/180) * math.pi
    def rad2deg(self, rad):
        return (rad/math.pi) *180

class Walker:
    ROTATE_SPEED = 45 * math.pi / 180
    FORWARD_SPEED = 0.4
    MIN_DIST_FROM_OBSTACLE = 0.6
    
    def __init__(self) -> None:
        # init ros
        rospy.init_node('walker_node', anonymous=True)
        
        # set directions
        self.directions = []
        for i in range(12):
            self.directions.append(Direction(i*30,15))
        self.currentDirection = Direction(180, 15)
        
        # set params
        self._is_moving = True
        self.is_turning = False
        self.pub = rospy.Publisher('cmd_vel',Twist,queue_size=1)
        self.sub = rospy.Subscriber('scan',LaserScan, self.scan_callback, queue_size=10)


    def start_walking(self):
        while not rospy.is_shutdown():
            rospy.spin()


    def rotateTo(self, direction: Direction):
        twist = Twist()
        turnAngle = direction.center - self.currentDirection.center
        startTime = rospy.Time.now()
        endTime = startTime + rospy.Duration(abs(turnAngle) / self.ROTATE_SPEED)
        while rospy.Time.now() < endTime:
            twist.angular.z = (1 if turnAngle > 0 else -1) * self.ROTATE_SPEED
            self.pub.publish(twist)
        twist.angular.z = 0
        self.pub.publish(twist)
        self.is_turning = False
        self.is_moving = True

    def scan_callback(self, scan:LaserScan):
        if self.is_turning:
            rospy.loginfo("Scan callback returning")
            return
        hasObstacle = self.currentDirection.hasObstacle(scan)
        msg = Twist()
        if hasObstacle:
            maxDirection = None
            maxInfs = 0
            maxAverage = 0
            for d in self.directions:
                if not d.hasObstacle(scan):
                    infs, average = d.infsAndAverageRange(scan)
                    if infs > maxInfs:
                        maxInfs = infs
                        maxDirection = d
                        maxAverage = average
                    elif infs == maxInfs and average > maxAverage:
                        maxAverage = average
                        maxDirection = d
            self.is_turning = True
            self.rotateTo(maxDirection)
        else:
            self.is_turning = False
            msg.linear.x = -1 * self.FORWARD_SPEED
            msg.angular.z = 0
            self.pub.publish(msg)

if __name__ == '__main__':
    try:
        walker = Walker()
        walker.start_walking()
        
    except rospy.ROSInterruptException:
        pass
