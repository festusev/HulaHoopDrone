from djitellopy import Tello
import rospy
from geometry_msgs.msg import TransformStamped, PoseStamped
import numpy as np



def drone_callback(data):


if __name__ == "__main__":
    rospy.init_node("debug_orientation")
    tello = Tello()
    tello.connect()
    # tello.streamon()
    tello.takeoff()
    rospy.Subscriber("/vicon/b_tello/b_tello", TransformStamped, drone_callback)

