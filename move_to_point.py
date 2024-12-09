from djitellopy import Tello
import cv2, math, time
import rospy
import tf2_ros
import numpy as np
import tf2_geometry_msgs
from geometry_msgs.msg import TransformStamped, PoseStamped, PointStamped
from std_msgs.msg import Bool

tello = Tello()
tello.connect()

fly = True
# tello.streamon()
# frame_read = tello.get_frame_read()

if fly:
    t0 = time.time()
    tello.takeoff()
    t1 = time.time()
    print("Took Off: ", t1 - t0)
    tello.send_rc_control(0, 100, 0, 0)
    t2 = time.time()
    print("Warmed Up, ", t2 - t1)

# tello.curve_xyz_speed(100, 100, 0, 200, 0, 0, 50)


fixed_target = [1, 2, 1]
ran_once = False
let_it_rip = False

def tello_callback(event):
    global ran_once
    if (ran_once):
        return
    if time.time() - t2 > 5:
        print("No command detected, abandoning")
        tello.land()
        exit()
    # ran_once = True
    
    try:
        # pose = tfBuffer.lookup_transform("vicon/world", "vicon/b_tello/b_tello", rospy.Time())
        pose = tfBuffer.lookup_transform("vicon/b_tello/b_tello", "vicon/world", rospy.Time())
    except (tf2_ros.LookupException, tf2_ros.ConnectivityException, tf2_ros.ExtrapolationException):
        # print("LOOKUP EXCEPTION")
        return
    
    orientation = pose.transform.rotation
    orientation = [orientation.x, orientation.y, orientation.z, orientation.w]

    # position = pose.transform.translation
    # position = [position.x, position.y, position.z]

    # diff = [fixed_target[0] - position[0], fixed_target[1] - position[1], fixed_target[2] - position[2]]
    # diff = np.array(diff)
    # diff *= 100

    target_pose_msg = PoseStamped()
    target_pose_msg.header.frame_id = "vicon/world"
    target_pose_msg.pose.position.x = fixed_target[0]
    target_pose_msg.pose.position.y = fixed_target[1]
    target_pose_msg.pose.position.z = fixed_target[2]
    target_pose_msg.pose.orientation.x = orientation[0]
    target_pose_msg.pose.orientation.y = orientation[1]
    target_pose_msg.pose.orientation.z = orientation[2]
    target_pose_msg.pose.orientation.w = orientation[3]


    transformed_pose = tf2_geometry_msgs.do_transform_pose(target_pose_msg, pose)

    # if not fly: 
    #     print("Transformed pose: ", end="")
    #     print(-pose.transform.translation.x, -pose.transform.translation.y, -pose.transform.translation.z)

    transformed_pose = np.array([transformed_pose.pose.position.x, transformed_pose.pose.position.y, transformed_pose.pose.position.z])
    transformed_pose *= 100
    transformed_pose *= 1.4

    transformed_pose = transformed_pose.astype(int)
    # transformed_pose = np.clip(transformed_pose, 20, 500)


    # tello.go_xyz_speed(-100, 0, 0, 10)
    print(f"Fixed Target: {fixed_target}")
    if fly and let_it_rip:
        print(fixed_target)
        print("READY, flying to", fixed_target)
        tello.go_xyz_speed(int(transformed_pose[1]), -int(transformed_pose[0]), int(transformed_pose[2]), 100)
        t3 = time.time()
        print("Sent XYZ: ", t3 - t2)
        ran_once = True
        tello.land()
    # exit(0)
    
    
def let_it_rip_callback(data):
    print("data: ", data.data)
    global let_it_rip
    if data.data == True:
        let_it_rip = True
    

def point_callback(data):
    global fixed_target
    fixed_target = [data.point.x, data.point.y, data.point.z]




rospy.init_node('flyer', anonymous=True)
rospy.Timer(rospy.Duration(1.0/100.0), tello_callback)
rospy.Subscriber("/let_it_rip", Bool, let_it_rip_callback)
rospy.Subscriber("/target_position", PointStamped, point_callback)
tfBuffer = tf2_ros.Buffer()
listener = tf2_ros.TransformListener(tfBuffer)
# tello_callback(None)

rospy.spin()

# tello.land()
