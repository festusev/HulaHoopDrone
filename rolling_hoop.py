import rospy
from geometry_msgs.msg import TransformStamped, Point, PointStamped
from visualization_msgs.msg import Marker
from std_msgs.msg import Float64, Bool

import matplotlib.pyplot as plt
# from mpl_toolkits.mplot3d import Axes3D
from matplotlib.animation import FuncAnimation
import numpy as np
import threading
import time
import tf2_ros

LSQ = False
LSQ_POINTS = 6
HOOP_R = 0.345
PRED_GRANULARITY = 0.001 # Down to a millisecond
g = -9.81 # -9.81e3 # VERIFIED THAT THIS IS CORRECT FOR VICOM

real_position_viz_pub = None
pred_position_viz_pub = None
target_position_viz_pub = None

list_points = []

def hoop_callback(event):

    try:
        pose = tfBuffer.lookup_transform("vicon/world", "vicon/hoop_real/hoop_real", rospy.Time())
    except (tf2_ros.LookupException, tf2_ros.ConnectivityException, tf2_ros.ExtrapolationException):
        # print("LOOKUP EXCEPTION")
        return
    # print("No exception")
    
    data = pose


    global pred_hoop_cm, flythru_cm, hoop_cm, list_points 
    if not capturing:
        return

    # print("Hoop callback")
    #
    t = data.header.stamp.to_sec()
    # t = time.time()
    x = data.transform.translation.x
    y = data.transform.translation.y
    z = data.transform.translation.z

    marker = Marker()
    marker.header.frame_id = "vicon/world"
    marker.header.stamp = rospy.Time.now()
    marker.ns = "hoop"
    marker.id = 0
    marker.type = Marker.LINE_STRIP
    marker.action = Marker.ADD
    marker.pose.orientation.w = 1.0
    marker.scale.x = 0.01
    marker.color.r = 1.0
    marker.color.a = 1.0
    marker.points = []

    point = Point()
    point.x = data.transform.translation.x
    point.y = data.transform.translation.y
    point.z = data.transform.translation.z

    list_points.append(point)
    if len(list_points) > 1000:
        list_points = list_points[1:]

    marker.points = list_points
    real_position_viz_pub.publish(marker)

    # Free Fall Detector
    # if len(hoop_cm) > 2 and (not (abs(t-hoop_cm[-1][3]) == 0)) and (not (abs(hoop_cm[-1][3]-hoop_cm[-2][3]) == 0.0)):
    #     prev_vz = (hoop_cm[-1][2] - hoop_cm[-2][2]) / (hoop_cm[-1][3] - hoop_cm[-2][3] +1e-6)
    #     vz = (z - hoop_cm[-1][2]) / (t - hoop_cm[-1][3] +1e-6)
    #     az = (vz - prev_vz) / (t - hoop_cm[-1][3] +1e-6)

    #     # print("Az:", az)
    #     msg = Float64()
    #     msg.data = az
    #     debug_pub.publish(msg)
    #     # Approx 9.81 m/s^2
    #     if az < -9.7 and az > -9.9:
    #         print("Freefall detected")
            
        

   

    # print(f"{t}: {x}, {y}, {z}")

    # t = time.time() - t0
    # t = t/10
    # noise = 0.01
    # x = (1/2) * true_params[0][0] * t**2 + true_params[0][1] * t + true_params[0][2] + np.random.normal(scale=noise)
    # y = (1/2) * true_params[1][0] * t**2 + true_params[1][1] * t + true_params[1][2] + np.random.normal(scale=noise)
    #
    # # This code assumes that the z axis is up and down
    # z = (1/2) * true_params[2][0] * t**2 + true_params[2][1] * t + true_params[2][2] + np.random.normal(scale=noise)
    #
    # print("Time:", t, "t0: ", t0, "cur time:", t + t0)
    hoop_cm.append([x, y, z, t])
    
    # if len(hoop_cm) > 2:
    #     print(hoop_cm[-1][3] - hoop_cm[-2][3])
    #     hp = np.array(hoop_cm)
        # print(np.mean(hp[1:, 3] - hp[:-1, 3]))

    # if LSQ:
    #     if len(hoop_cm) > LSQ_POINTS:
    #         filtered_points = filter_points(np.array(hoop_cm))       

    #         if filtered_points is not None:
    #             filtered_points[:, -1] -= filtered_points[0, -1] # Subtract the initial timestep
    #             # filter_points = filtered_points[-20:]
    #             params = lsq(filtered_points)
                    
    #             pred_hoop_cm = pred_cm(params, np.arange(0, 10, PRED_GRANULARITY))
    #             valid_hoop_cm_idx = np.argwhere(pred_hoop_cm[:, 2] > HOOP_R)

    #             # Get the center of mass that is 1 second before hitting the ground
    #             steps_before_impact = int(0.1 / PRED_GRANULARITY)
    #             if valid_hoop_cm_idx.size > steps_before_impact:
    #                 flythru_cm = pred_hoop_cm[valid_hoop_cm_idx[-steps_before_impact]][0]

    #                 point = Point()
    #                 point.x = flythru_cm[0]
    #                 point.y = flythru_cm[1]
    #                 point.z = flythru_cm[2]
    #                 pt_stamped = PointStamped(point=point)
    #                 pt_stamped.header.frame_id = "vicon/world"
    #                 pt_stamped.header.stamp = rospy.Time.now()
    #                 target_position_viz_pub.publish(pt_stamped)
    #                 print("Sending Target Position: ", point.x, point.y, point.z)

    #             marker = Marker()
    #             marker.header.frame_id = "vicon/world"
    #             marker.header.stamp = rospy.Time.now()
    #             marker.ns = "hoop_pred"
    #             marker.id = 1
    #             marker.type = Marker.LINE_STRIP
    #             marker.action = Marker.ADD
    #             marker.pose.orientation.w = 1.0

    #             marker.scale.x = 0.01
    #             marker.color.g = 1.0
    #             marker.color.a = 1.0
    #             marker.points = []

    #             for cm in pred_hoop_cm:
    #                 point = Point()
    #                 point.x = cm[0]
    #                 point.y = cm[1]
    #                 point.z = cm[2]
    #                 # print(f"Pred hoop cm: {cm}")
    #                 marker.points.append(point)
                
    #             pred_position_viz_pub.publish(marker)




def pred_cm(params, ts, pt):
    cms = []
    for t in ts:
        x = pt[0]+params[0]*t
        y = pt[1]+params[1]*t
        z = pt[2]
        cms.append([x, y, z, t])
    return np.array(cms)

def filter_points(cms):
    # print("Number of out of bounds", (cms[-20:, -1] - cms[-21:-1,-1] == 0).sum())
    # if (cms[-20:, -1] - cms[-21:-1, -1] == 0).all():
    #     return None
    nonzero = cms[np.nonzero(cms[1:, -1] - cms[:-1, -1])]
    # print("Deleted:", cms.shape[0] - nonzero.shape[0])
    # if cms.shape[0] - nonzero.shape[0] > 12:
    #     breakpoint()
    return nonzero[-LSQ_POINTS:]

    # global freefall
    # # print("Freefall", freefall)
    # if freefall >= 0:
    #     return cms[freefall:]

    # len_cms = cms.shape[0]
    # cms = cms[-LSQ_POINTS:]

    # print(f"cms: {cms}")
    # pos = cms[:, :-1]
    # t = cms[:, -1][:, None]
    # dt = t[1:] - t[:-1]
    # dp = pos[1:] - pos[:-1]
    # a = (dp[1:]/(dt[1:]**2) - dp[:-1]/(dt[1:]*dt[:-1]))

    # print(f"pos diff: {dp}")
    # print(f"t: {t}")
    # print(f"t diff: {dt}")
    # print(f"Acceleration: {a}")

    # if np.isclose(a, np.mean(a, axis=0)[None], atol=0.4).all():
    #     freefall = len_cms - LSQ_POINTS
    #     print("Freefall idx:", freefall)
    #     return cms[freefall:]
    # else:
    #     return None


def lsq(cms):
    cms = np.array(cms)
    x = cms[:, 0]
    y = cms[:, 1]
    Y = cms[:, :-1]
    t = cms[:, -1]
    X = np.vstack([t, np.ones_like(t)]).T
    
    # Y[:, -1] -= 1/2 * g * t**2    
    params_x = np.linalg.lstsq(X, x, rcond=None)[0]  # Fit x positions
    params_y = np.linalg.lstsq(X, y, rcond=None)[0]  # Fit y positions

    #Using last 2 pts
    # prev = cms[-2,:]
    # curr = cms[-1,:]
    # v_x = (curr[0] - prev[0])/(curr[3]-prev[3])
    # v_x = (curr[1] - prev[1])/(curr[3]-prev[3])

    v_x = params_x[1]
    v_y = params_y[1] 
    return v_x, v_y

def create_animation():
    # Update function for FuncAnimation
    def update_animation(frame):
        """Update the scatter plot with new data."""

        if flythru_cm is not None:
            scat_flythru.zorder = 10
            scat_flythru._offsets3d = ([flythru_cm[0].item()], [flythru_cm[1].item()], [flythru_cm[2].item()])

        # Update data with the latest points
        if len(hoop_cm) == 0:
            return

        new_offsets = np.array(hoop_cm)
        scat_hoop.zorder = 2
        scat_hoop._offsets3d = (new_offsets[:, 0], new_offsets[:, 1], new_offsets[:, 2])

        # print("Predicting:", len(pred_hoop_cm), pred_hoop_cm[-1])
        if len(pred_hoop_cm) == 0:
            return
        scat_hoop_pred.zorder = 1
        scat_hoop_pred._offsets3d = (pred_hoop_cm[:, 0], pred_hoop_cm[:, 1], pred_hoop_cm[:, 2])

    # Initialize the plot
    fig = plt.figure()
    ax = fig.add_subplot(111, projection='3d')
    scat_flythru = ax.scatter([0], [0], [0], c='green', zorder=10, s=60)
    scat_hoop = ax.scatter([0], [0], [0], c='blue', zorder=2, s=20)
    scat_hoop_pred = ax.scatter([0], [0], [0], c='red', zorder=1, s=10)

    # Set plot limits
    ax.set_xlim([-10, 10])
    ax.set_ylim([-10, 10])
    ax.set_zlim([0, 10])

    # Animation
    ani = FuncAnimation(fig, update_animation, interval=0.01)
    plt.show()

def let_it_rip_callback(data):
    # print("data: ", data.data)
    global let_it_rip
    if data.data == True:
        # point = Point()
        # point.x = hoop_cm[-1][0]
        # point.y = hoop_cm[-1][1]
        # point.z = hoop_cm[-1][2]
        # pt_stamped = PointStamped(point=point)
        # pt_stamped.header.frame_id = "vicon/world"
        # pt_stamped.header.stamp = rospy.Time.now()
        # target_position_viz_pub.publish(pt_stamped)
        # return

        if len(hoop_cm) > LSQ_POINTS:
            filtered_points = filter_points(np.array(hoop_cm))       

            if filtered_points is not None:
                filtered_points[:, -1] -= filtered_points[0, -1] # Subtract the initial timestep
                # filter_points = filtered_points[-20:]
                x_obs, y_obs = filtered_points[:, 0], filtered_points[:, 1]
                # Fit a linear model to the observed points
                coefficients = np.polyfit(x_obs, y_obs, 1)
                slope, intercept = coefficients
                pred_hoop_cm = pred_cm(slope, intercept, np.arange(0.1, 10, PRED_GRANULARITY))
                valid_hoop_cm_idx = np.argwhere(pred_hoop_cm[:, 2] > HOOP_R)

                # Get the center of mass that is 0,1 second before hitting the ground
                steps_before_impact = int(0.1 / PRED_GRANULARITY)
                if valid_hoop_cm_idx.size > steps_before_impact:
                    flythru_cm = pred_hoop_cm[valid_hoop_cm_idx[-steps_before_impact]][0]

                    point = Point()
                    point.x = flythru_cm[0]
                    point.y = flythru_cm[1]
                    point.z = flythru_cm[2]
                    pt_stamped = PointStamped(point=point)
                    pt_stamped.header.frame_id = "vicon/world"
                    pt_stamped.header.stamp = rospy.Time.now()
                    target_position_viz_pub.publish(pt_stamped)
                    # print("Sending Target Position: ", point.x, point.y, point.z)

                marker = Marker()
                marker.header.frame_id = "vicon/world"
                marker.header.stamp = rospy.Time.now()
                marker.ns = "hoop_pred"
                marker.id = 1
                marker.type = Marker.LINE_STRIP
                marker.action = Marker.ADD
                marker.pose.orientation.w = 1.0

                marker.scale.x = 0.01
                marker.color.g = 1.0
                marker.color.a = 1.0
                marker.points = []

                for cm in pred_hoop_cm:
                    point = Point()
                    point.x = cm[0]
                    point.y = cm[1]
                    point.z = cm[2]
                    # print(f"Pred hoop cm: {cm}")
                    marker.points.append(point)
                
                pred_position_viz_pub.publish(marker)
    

if __name__ == '__main__':
    hoop_cm = []
    pred_hoop_cm = np.array([])
    drone_cm = []

    capturing = False

    # app = QtWidgets.QApplication([])
    # view = gl.GLViewWidget()
    # view.show()

    true_params = np.array(
            [
                [0, 1, 3],
                [0, 1, 3],
                [-9.81, 10, 2]
            ]
    )

    t0 = time.time()
    flythru_cm = None  # Center that we will fly through
    freefall = -1

    rospy.init_node('flyer', anonymous=True)
    tfBuffer = tf2_ros.Buffer()
    listener = tf2_ros.TransformListener(tfBuffer)
    rospy.Timer(rospy.Duration(1.0/125.0), hoop_callback)
    real_position_viz_pub = rospy.Publisher("/hoop_viz", Marker, queue_size=10)
    pred_position_viz_pub = rospy.Publisher("/hoop_pred_viz", Marker, queue_size=10)
    target_position_viz_pub = rospy.Publisher("/target_position", PointStamped, queue_size=10)
    debug_pub = rospy.Publisher("/debug", Float64, queue_size=10)
    rospy.Subscriber("/let_it_rip", Bool, let_it_rip_callback)

    # rospy.Subscriber("/vicon/hoop_real/hoop_real", TransformStamped, hoop_callback)
    # rospy.Subscriber("/vicon/b_tello/b_tello", TransformStamped, drone_callback)

    capturing = True

    # create_animation()

    # t1 = threading.Thread(target=create_animation)
    # t1.start()

    # plt.show(block=False)
    rospy.spin()

    # start = input("Type 'Y' to start").lower()
    # if start == "y" or start == "yes":
    #     capturing = True


    # Display the plot
    # t1.join()
