import math
import rospy
from geometry_msgs.msg import Twist
from turtlesim.msg import Pose 

# publisher
pub = rospy.Publisher('turtle1/cmd_vel', Twist, queue_size=10)

# robot starting point
initial_x = -1 
initial_y = -1
print_pos = False # print result


def move(distance:float, speed: float):
    vel_msg = Twist()
    vel_msg.linear.x = speed
    current_x = 0
    start_time = rospy.Time.now().to_sec()
    rate = rospy.Rate(30)

    while current_x < distance:
        pub.publish(vel_msg) #publish message
        current_time = rospy.Time.now().to_sec()
        current_x = (current_time - start_time) * speed
        rospy.loginfo("Current x position: %f", current_x)
        rate.sleep()

    # stop the robot
    vel_msg.linear.x = 0
    pub.publish(vel_msg)
    
def rotate(angle: float, speed: float):
    vel_msg = Twist()
    angular_velocity = speed * 2 * math.pi / 360
    vel_msg.angular.z = angular_velocity
    current_angular_dist = 0
    final_angular_dist = angle * 2 * math.pi / 360
    start_time = rospy.Time.now().to_sec()

    while current_angular_dist < final_angular_dist:
        pub.publish(vel_msg)
        time = rospy.Time.now().to_sec()
        current_angular_dist = (time - start_time) * angular_velocity
    
    # stop the robot
    vel_msg.angular.z = 0
    pub.publish(vel_msg)

def pose_changed(data:Pose):
    global initial_x, initial_y, print_pos
    if initial_x == -1:
        initial_x = data.x
        initial_y = data.y
    if print_pos:
        rospy.loginfo("The starting position is x: %f, y: %f and the ending position is x: %f, y: %f",
        initial_x,
        initial_y,
        data.x,
        data.y)
        print_pos = False

def main():
    rospy.init_node('turtle_mover', anonymous=True)
    rospy.Subscriber('turtle1/pose', Pose, pose_changed)
    rospy.sleep(3) # wait for 3 seconds

    move(1, 0.2)
    rotate(45,5)
    global print_pos
    print_pos = True
    rospy.sleep(2) 
    # sleep for 2 seconds
    # so that subscribers can listen one more time to print_pos

if __name__ == "__main__":
    main()