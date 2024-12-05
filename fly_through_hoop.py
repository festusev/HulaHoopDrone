from djitellopy import Tello
import cv2, math, time
import rospy
from std_msgs.msg import String
from geometry_msgs.msg import TransformStamped
import numpy as np

tello = Tello()
tello.connect()

tello.streamon()
frame_read = tello.get_frame_read()

tello.takeoff()

hoop_cm = []
drone_cm = []

capturing = False

def hoop_callback(data):
    rospy.loginfo(rospy.get_caller_id() + "I heard %s", data.data)

    if not capturing:
        return

    t = data.header.stamp
    x = data.translation.x
    y = data.translation.y
    z = data.translation.z
    hoop_cm.append([x, y, z, t])

    if len(hoop_cm) > 5:
        params = lsq(hoop_cm)
        breakpoint()

def lsq(cms):
    cms = np.array(cms)
    X = cms[:, :-1]
    t = cms[:, -1]
    Y = np.hstack([t**2 / 2, t, 1])

    params = np.linalg.inv(X.T @ X) @ X.T @ Y
    return params

def hoop_listener():
    rospy.Subscriber("chatter", TransformStamped, hoop_callback)
    rospy.spin()

def drone_callback(data):
    rospy.loginfo(rospy.get_caller_id() + "I heard %s", data.data)

def drone_listener():
    rospy.Subscriber("chatter", TransformStamped, drone_callback)

if __name__ == '__main__':
    rospy.init_node('flyer', anonymous=True)
    hoop_listener()
    drone_listener()

    start = input("Type 'Y' to start").lower()
    if start == "y" or start == "yes":
        capturing = True
