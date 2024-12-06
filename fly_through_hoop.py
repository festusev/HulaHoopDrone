import rospy
from geometry_msgs.msg import TransformStamped

import matplotlib.pyplot as plt
# from mpl_toolkits.mplot3d import Axes3D
from matplotlib.animation import FuncAnimation
import numpy as np
import threading
import time

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

LSQ_POINTS = 15
HOOP_R = 0.5
PRED_GRANULARITY = 0.001 # Down to a millisecond
g = -9.81 # -9.81e3 # VERIFY THAT THIS IS CORRECT FOR VICOM

def hoop_callback(data):
    global pred_hoop_cm, flythru_cm
    if not capturing:
        return

    # print("Hoop callback")
    #
    t = data.header.stamp.to_sec()
    x = data.transform.translation.x
    y = data.transform.translation.y
    z = data.transform.translation.z

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

    if len(hoop_cm) > LSQ_POINTS:
        filtered_points = np.array(hoop_cm) #filter_points(np.array(hoop_cm))
        filtered_points[:, -1] -= filtered_points[0, -1] # Subtract the initial timestep

        if filtered_points is not None:
            params = lsq(filtered_points)
            pred_hoop_cm = pred_cm(params, np.arange(0, 20, PRED_GRANULARITY))
            valid_hoop_cm_idx = np.argwhere(pred_hoop_cm[:, 2] > HOOP_R)

            # Get the center of mass that is 1 second before hitting the ground
            steps_before_impact = int(0.1 / PRED_GRANULARITY)
            if valid_hoop_cm_idx.size > steps_before_impact:
                flythru_cm = pred_hoop_cm[valid_hoop_cm_idx[-steps_before_impact]][0]

        # print(params)
    time.sleep(0.01)

def pred_cm(params, ts):
    cms = []
    for t in ts:
        x = (1/2) * params[0][0] * t**2 + params[0][1] * t + params[0][2]
        y = (1/2) * params[1][0] * t**2 + params[1][1] * t + params[1][2]
        z = (1/2) * params[2][0] * t**2 + params[2][1] * t + params[2][2]

        cms.append([x, y, z, t])
    return np.array(cms)

def filter_points(cms):
    global freefall
    # print("Freefall", freefall)
    if freefall >= 0:
        return cms[freefall:]

    len_cms = cms.shape[0]
    cms = cms[-LSQ_POINTS:]

    print(f"cms: {cms}")
    pos = cms[:, :-1]
    t = cms[:, -1][:, None]
    dt = t[1:] - t[:-1]
    dp = pos[1:] - pos[:-1]
    a = (dp[1:]/(dt[1:]**2) - dp[:-1]/(dt[1:]*dt[:-1]))

    print(f"pos diff: {dp}")
    print(f"t: {t}")
    print(f"t diff: {dt}")
    print(f"Acceleration: {a}")

    if np.isclose(a, np.mean(a, axis=0)[None], atol=0.4).all():
        freefall = len_cms - LSQ_POINTS
        print("Freefall idx:", freefall)
        return cms[freefall:]
    else:
        return None


def lsq(cms):
    cms = np.array(cms)
    Y = cms[:, :-1]
    t = cms[:, -1]
    X = np.vstack([t, np.ones_like(t)]).T
    Y[:, -1] -= 1/2 * g * t**2

    # breakpoint()
    params = np.linalg.inv(X.T @ X) @ X.T @ Y
    params = params.T
    params = np.hstack([np.array([[0, 0, g]]).T, params])
    print("Params:", params)
    return params

def drone_callback(data):
    pass


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

    # Initialize the plot
    fig = plt.figure()
    ax = fig.add_subplot(111, projection='3d')
    scat_flythru = ax.scatter([0], [0], [0], c='green', zorder=10, s=60)
    scat_hoop = ax.scatter([0], [0], [0], c='blue', zorder=2, s=20)
    scat_hoop_pred = ax.scatter([0], [0], [0], c='red', zorder=1, s=10)

    # Set plot limits
    ax.set_xlim([0, 10])
    ax.set_ylim([0, 10])
    ax.set_zlim([0, 10])

    # Animation
    ani = FuncAnimation(fig, update_animation, interval=0.1)

    rospy.init_node('flyer', anonymous=True)
    rospy.Subscriber("/vicon/hoop_real/hoop_real", TransformStamped, hoop_callback)
    rospy.Subscriber("/vicon/b_tello/b_tello", TransformStamped, drone_callback)


    capturing = True

    t1 = threading.Thread(target=plt.show)
    t1.start()

    # plt.show(block=False)
    rospy.spin()

    # start = input("Type 'Y' to start").lower()
    # if start == "y" or start == "yes":
    #     capturing = True


    # Display the plot
    t1.join()
