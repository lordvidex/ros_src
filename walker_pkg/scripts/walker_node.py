import math
import rospy
from geometry_msgs.msg import Twist
from sensor_msgs.msg import LaserScan


class Walker:
    ROTATE_SPEED = 45 * math.pi / 180
    FORWARD_SPEED = 0.4
    MIN_DIST_FROM_OBSTACLE = 0.6
    MIN_SCAN_ANGLE = math.pi + (-22.5 / 180 * math.pi)
    MAX_SCAN_ANGLE = math.pi + (22.5 / 180 * math.pi)
    def __init__(self) -> None:
        # init ros
        rospy.init_node('walker_node', anonymous=True)
        
        # set params
        self._is_moving = True
        self.is_turning = False
        self.pub = rospy.Publisher('cmd_vel',Twist,queue_size=1)
        self.sub = rospy.Subscriber('scan',LaserScan, self.scan_callback, queue_size=1)
    
    def start_walking(self):
        while not rospy.is_shutdown():
            rospy.spin()
    def turn(self, angle:float):
        twist = Twist()
        turn_time = angle / self.ROTATE_SPEED
        start_time = rospy.get_time()
        end_time = start_time + turn_time
        while rospy.get_time() < end_time:
            twist.angular.z = self.ROTATE_SPEED
            self.pub.publish(twist)
        twist.angular.z = 0
        twist.linear.x = 0
        self.pub.publish(twist)
    
    # def analyze_scan(scan: LaserScan):
    #     for range in scan.ranges:
            
    def scan_callback(self, scan:LaserScan):
        blocked = False
        minIndex = math.ceil((self.MIN_SCAN_ANGLE - scan.angle_min) / scan.angle_increment)
        maxIndex = math.floor((self.MAX_SCAN_ANGLE - scan.angle_min) / scan.angle_increment)

        currIndex = minIndex + 1
        while currIndex <= maxIndex:
            if scan.ranges[int(currIndex)] < self.MIN_DIST_FROM_OBSTACLE:
                blocked = True
                break
            currIndex = currIndex + 1

        if blocked:
            rospy.loginfo("Turn away robot!!")
            self.turn()
        else:
            rospy.loginfo("Keep moving robot!!")
            twist = Twist()
            twist.angular.z = 0
            twist.linear.x = -1 * self.FORWARD_SPEED
            # publish message
            self.pub.publish(twist)
        

if __name__ == '__main__':
    try:
        walker = Walker()
        walker.start_walking()
        
    except rospy.ROSInterruptException:
        pass
