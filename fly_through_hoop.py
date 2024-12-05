import cv2, math, time
import rospy
from std_msgs.msg import String
from geometry_msgs.msg import TransformStamped
import numpy as np

# import pyqtgraph as pg
# import pyqtgraph.opengl as gl
# from pyqtgraph.Qt import QtWidgets

hoop_cm = []
drone_cm = []

capturing = False

# app = QtWidgets.QApplication([])
# view = gl.GLViewWidget()
# view.show()

# true_params = np.array(
#     [
#         [0.5, 0.3, 0.1],
#         [0.7, 0.11, 12],
#         [5, 13, 0.6]
#     ]
# )

# t = 0
def hoop_callback(data):
    global hoop_cm
    if not capturing:
        return

    print("Hoop callback")

    t = data.header.stamp.to_sec()
    x = data.transform.translation.x
    y = data.transform.translation.y
    z = data.transform.translation.z
    # 


    # x = (1/2) * true_params[0][0] * t**2 + true_params[0][1] * t + true_params[0][2]
    # y = (1/2) * true_params[1][0] * t**2 + true_params[1][1] * t + true_params[1][2]
    # z = (1/2) * true_params[2][0] * t**2 + true_params[2][1] * t + true_params[2][2]
    

    hoop_cm.append([x, y, z, t])
    
    
    # t += 1

    if len(hoop_cm) > 10:
        params = lsq(hoop_cm)
        print(params)
        # breakpoint()
        hoop_cm = hoop_cm[1:]

def lsq(cms):
    cms = np.array(cms)
    Y = cms[:, :-1]
    t = cms[:, -1]
    t = t - t[0]
    X = np.vstack([t**2 / 2, t, np.ones_like(t)]).T

    # breakpoint()

    params = np.linalg.inv(X.T @ X) @ X.T @ Y
    return params.T

def hoop_listener():
    rospy.Subscriber("/vicon/hoop_real/hoop_real", TransformStamped, hoop_callback)

def drone_callback(data):
    pass

def drone_listener():
    rospy.Subscriber("/vicon/b_tello/b_tello", TransformStamped, drone_callback)

if __name__ == '__main__':
    rospy.init_node('flyer', anonymous=True)
    hoop_listener()
    drone_listener()

    # time.sleep(5)
    capturing = True

    rospy.spin()

    # start = input("Type 'Y' to start").lower()
    # if start == "y" or start == "yes":
    #     capturing = True
