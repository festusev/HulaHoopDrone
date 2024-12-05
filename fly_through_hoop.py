from djitellopy import Tello
import cv2, math, time
import rospy
from std_msgs.msg import String

tello = Tello()
tello.connect()

tello.streamon()
frame_read = tello.get_frame_read()

tello.takeoff()

hoop_cm = []
drone_cm = []

def hoop_callback(data):
    rospy.loginfo(rospy.get_caller_id() + "I heard %s", data.data)

def hoop_listener():
    rospy.init_node('hoop_listener', anonymous=True)
    rospy.Subscriber("chatter", String, hoop_callback)
    rospy.spin()

    cm = [0, 0, 0]

    hoop_cm.append(cm)

def drone_callback(data):
    rospy.loginfo(rospy.get_caller_id() + "I heard %s", data.data)

def drone_listener():
    rospy.init_node('drone_listener', anonymous=True)
    rospy.Subscriber("chatter", String, drone_callback)

if __name__ == '__main__':
    hoop_listener()
    drone_listener()

    tello.go_xyz_speed(200, 0, 0, 50)
    # tello.curve_xyz_speed(100, 100, 0, 200, 0, 0, 50)

    tello.land()
