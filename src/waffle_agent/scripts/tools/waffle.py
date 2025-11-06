#!/usr/bin/env python3
import rospy
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from sensor_msgs.msg import Image
#from cv_bridge import CvBridge
#import cv2
#import base64
#from io import BytesIO
from langchain.agents import tool
from langchain.llms import OpenAI  # or whatever your LLM class is


#bridge = CvBridge()
cmd_vel_pub = rospy.Publisher("/cmd_vel", Twist, queue_size=10)


@tool
def move_waffle(linear: float = 0.2, angular: float = 0.0, duration: float = 2.0):
    """Move the TurtleBot3 Waffle with given linear and angular velocities."""
    vel = Twist()
    vel.linear.x = linear
    vel.angular.z = angular
    start_time = rospy.Time.now()
    rate = rospy.Rate(10)
    while (rospy.Time.now() - start_time).to_sec() < duration:
        cmd_vel_pub.publish(vel)
        rate.sleep()
    cmd_vel_pub.publish(Twist())  # stop
    return f"Moved with linear={linear}, angular={angular} for {duration} sec."


@tool
def get_waffle_pose():
    """Get the robot's current position from /odom."""
    try:
        msg = rospy.wait_for_message("/odom", Odometry, timeout=5)
        pos = msg.pose.pose.position
        return f"Robot is at x={pos.x:.2f}, y={pos.y:.2f}."
    except rospy.ROSException:
        return "Odometry data not available."


