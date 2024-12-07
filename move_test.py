from djitellopy import Tello
import rospy
from geometry_msgs.msg import TransformStamped, PoseStamped
import numpy as np
import time


if __name__ == "__main__":
    tello = Tello()
    tello.connect()
    tello.takeoff()

    # tello.send_rc_control(0, 100, 0, 0)
    # time.sleep(3)
    # tello.go_xyz_speed(200, 200, 0, 100)
    tello.land()

