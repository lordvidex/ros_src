#!/usr/bin/env python
import rospy
import datetime

def run():
    rospy.init_node("timer_node", anonymous=True)
    rate = rospy.Rate(2)

    while not rospy.is_shutdown():
        
        time = rospy.Time.now().to_time()
        rospy.loginfo(time_to_str(time))
        
        rate.sleep()

def time_to_str(time: float):
    return datetime.datetime.fromtimestamp(time).strftime("%H:%M %d.%m.%y")
    
    
if __name__ == "__main__":
    try:
        run()
    except rospy.ROSInterruptException:
        pass
