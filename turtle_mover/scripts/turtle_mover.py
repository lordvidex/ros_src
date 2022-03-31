import math
from tokenize import Double
import rospy
from geometry_msgs.msg import Twist
from turtlesim.msg import Pose 

# publisher
pub = rospy.Publisher('turtle1/cmd_vel', Twist, queue_size=10)

# robot starting point
initial_x = initial_y = initial_z = \
final_x = final_y = final_z = -1

# variables
LINEAR_DISTANCE = 1
ANGULAR_DISTANCE = 45
print_pos = False # print result


def move(speed: float):
    vel_msg = Twist()
    vel_msg.linear.x = speed

    pub.publish(vel_msg) #publish message
    
def rotate(speed: float):
    vel_msg = Twist()
    angular_velocity = speed * 2 * math.pi / 360
    vel_msg.angular.z = angular_velocity
    pub.publish(vel_msg)
    
def pose_changed(data:Pose):
    global initial_x, initial_y,initial_z 
    global final_x, final_y, final_z, print_pos
    
    # program just started, set the goals
    if initial_x == -1:
        initial_x = data.x
        initial_y = data.y
        initial_z = data.theta
        final_x = initial_x + LINEAR_DISTANCE
        final_y = initial_y
        final_z = initial_z + ANGULAR_DISTANCE * math.pi / 180

    # move linearly first, then move angularly, then print distance and stop
    if data.x < final_x:
        move(1)
    elif data.theta < final_z:
        rotate(10)
    else:
        rospy.loginfo("The starting position is x: %f, y: %f, z: %f, and the ending position is x: %f, y: %f, z: %f",
        initial_x,
        initial_y,
        initial_z,
        data.x,
        data.y,
        data.theta
        )
        print_pos = True

def main():
    rospy.init_node('turtle_mover', anonymous=True)
    rospy.Subscriber('turtle1/pose', Pose, pose_changed)
    rate = rospy.Rate(60) 
    while not print_pos:
        rate.sleep()

if __name__ == "__main__":
    main()