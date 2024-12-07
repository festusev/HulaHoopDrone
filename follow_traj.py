from djitellopy import Tello
import cv2, math, time
import rospy
from geometry_msgs.msg import TransformStamped, PoseStamped
import numpy as np

# tello = Tello()
# tello.connect()

# get_roll, get_pitch, get_yaw
# get_acceleration_x, get_acceleration_y, get_acceleration_z
# get_barometer


class TrajFollower:
    def __init__(self, traj):
        rospy.init_node('flyer', anonymous=True)

        self.tello = Tello()
        self.tello.connect()
        self.tello.takeoff()

        self.xyz_state = np.array([0, 0, 0], dtype=np.float64)
        self.prev_error = np.array([0, 0, 0], dtype=np.float64)
        self.desired_state = None
        self.total_error = np.array([0, 0, 0], dtype=np.float64)
        self.time = time.time()
        self.start_time = time.time()
        self.traj = traj
        self.run = False
        self.start_xyz = None

        rospy.Subscriber("/vicon/b_tello/b_tello", TransformStamped, self.drone_callback)

        self.vis_pub = rospy.Publisher("/drone", PoseStamped, 40)

        # Timer Event
        # rospy.Timer(rospy.Duration(0.5), self.timer_callback)

        

    # PID Controller based on error of current state and desired state
    def pid_controller(self, desired_state):
        kp = 1
        ki = 1
        kd = 1

        error = desired_state - self.xyz_state

        diff_error = error - self.prev_error

        dt = time.time() - self.time

        self.total_error += error * dt

        # Proportional
        p = kp * error

        # Integral
        i = ki * error

        # Derivative
        d = kd * diff_error / dt

        self.prev_error = error
        self.time = time.time()

        return p

    def timer_callback(self, event):

        if time.time() - self.start_time > 2:
            self.run = True
            self.start_xyz = self.xyz_state

        if self.run:
            if self.traj == []:
                self.tello.land()
                self.tello.end()
                return
            self.desired_state = self.traj.pop(0) + self.start_xyz
            if self.desired_state is not None:
                pid_output = self.pid_controller(self.desired_state)
                self.tello.send_rc_control(int(pid_output[0]), int(pid_output[1]), int(pid_output[2]), 0)





    def drone_callback(self, data):

        print("drone callback")

        self.xyz_state[0] = data.transform.translation.x
        self.xyz_state[1] = data.transform.translation.y
        self.xyz_state[2] = data.transform.translation.z

        if self.desired_state is not None:
            if np.linalg.norm(self.xyz_state - self.desired_state) < 5:
                self.desired_state = None

        # msg = PoseStamped()
        # msg.header = data.header
        # msg.header.frame_id = "map"

        # msg.pose.position.x = data.transform.translation.x
        # msg.pose.position.y = data.transform.translation.y
        # msg.pose.position.z = data.transform.translation.z

        # self.vis_pub.publish(msg)
            

# Signal Handler



if __name__ == '__main__':
    
    traj = [[100, 0, 0], [0, 100, 0], [-100, 0, 0], [0, -100, 0]]
    tf = TrajFollower(traj)

    rospy.spin()


