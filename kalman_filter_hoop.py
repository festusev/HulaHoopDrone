import rospy
import numpy as np
from geometry_msgs.msg import PoseStamped, Point
import tf2_ros
from visualization_msgs.msg import Marker


class KalmanFilterNode:
    def __init__(self):
        # Initialize Kalman Filter parameters
        self.x = np.zeros(9)  # State vector: [x, y, z, vx, vy, vz, ax, ay, az]
        self.P = np.eye(9)    # State covariance
        self.Q = np.eye(9) * 0.01  # Process noise covariance
        self.R = np.eye(3) * 0.1   # Measurement noise covariance
        self.H = np.array([
            [1, 0, 0, 0, 0, 0, 0, 0, 0],
            [0, 1, 0, 0, 0, 0, 0, 0, 0],
            [0, 0, 1, 0, 0, 0, 0, 0, 0]
        ])  # Measurement matrix (we measure position only)

        # ROS setup
        rospy.init_node('kalman_filter_node', anonymous=True)
        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer)
        self.timer = rospy.Timer(rospy.Duration(1.0/100.0), self.callback)
        # self.sub = rospy.Subscriber('/vicon/hoop_position', PoseStamped, self.callback)
        self.pub_filtered_viz = rospy.Publisher("/pub_filtered_viz", Marker, queue_size=10)
        self.pub_forecasted_viz = rospy.Publisher("/pub_forecasted_viz", Marker, queue_size=10)
        self.pub_filtered = rospy.Publisher('/hoop_filtered_position', PoseStamped, queue_size=10)
        self.pub_forecasted = rospy.Publisher('/hoop_forecasted_position', PoseStamped, queue_size=10)

        self.list_filtered = []
        self.list_forecasted = []
        # Time tracking
        self.previous_time = None

        # Future forecast time (seconds)
        self.future_time = rospy.get_param('~future_time', 1.5)  # Default to 0.5 seconds in the future

    @staticmethod
    def compute_F(delta_t):
        """Compute the state transition matrix F based on delta_t."""
        F = np.eye(9)
        F[:3, 3:6] = np.eye(3) * delta_t
        F[:3, 6:9] = np.eye(3) * (delta_t**2 / 2)
        F[3:6, 6:9] = np.eye(3) * delta_t
        return F

    def predict(self, F):
        """Prediction step of the Kalman Filter."""
        self.x = F @ self.x
        self.P = F @ self.P @ F.T + self.Q

    def update(self, z):
        """Update step of the Kalman Filter."""
        y = z - self.H @ self.x  # Innovation
        S = self.H @ self.P @ self.H.T + self.R  # Innovation Covariance
        K = self.P @ self.H.T @ np.linalg.inv(S)  # Kalman Gain
        self.x += K @ y
        self.P = (np.eye(len(self.P)) - K @ self.H) @ self.P

    def forecast(self, delta_t_future):
        """Forecast the state at a future time."""
        F_future = self.compute_F(delta_t_future)
        return F_future @ self.x

    def callback(self, event):
        # Extract position measurement
        try:
            pose = self.tf_buffer.lookup_transform("vicon/world", "vicon/hoop_real/hoop_real", rospy.Time())
        except (tf2_ros.LookupException, tf2_ros.ConnectivityException, tf2_ros.ExtrapolationException):
            # print("LOOKUP EXCEPTION")
            return


        z = np.array([pose.transform.translation.x,
                      pose.transform.translation.y,
                      pose.transform.translation.z])

        # Time calculations
        current_time = pose.header.stamp.to_sec()
        if self.previous_time is None:
            self.previous_time = current_time
            return

        delta_t = current_time - self.previous_time
        self.previous_time = current_time

        # Compute F for current time step
        F = self.compute_F(delta_t)

        # Kalman Filter: Predict and Update
        self.predict(F)
        self.update(z)

        # Forecast future state
        delta_t_future = self.future_time

        # Forcast at 0.1 s intervals till future_time
        interval = 0.2
        time_ = interval
        self.list_forecasted = []
        while time_ <= delta_t_future:
            x_forecast = self.forecast(time_)
            point = Point()
            point.x = x_forecast[0]
            point.y = x_forecast[1]
            point.z = x_forecast[2]
            self.list_forecasted.append(point)
            time_ += interval
        

        # Publish filtered state
        filtered_msg = PoseStamped()
        filtered_msg.header.stamp = rospy.Time.now()
        filtered_msg.header.frame_id = "vicon/world"
        filtered_msg.pose.position.x = self.x[0]
        filtered_msg.pose.position.y = self.x[1]
        filtered_msg.pose.position.z = self.x[2]
        self.pub_filtered.publish(filtered_msg)

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
        point.x = self.x[0]
        point.y = self.x[1]
        point.z = self.x[2]
        self.list_filtered.append(point)
        marker.points = self.list_filtered
        if len(self.list_filtered) > 1:
            self.pub_filtered_viz.publish(marker)

        if len(self.list_filtered) > 100:
            self.list_filtered.pop(0)
        

        marker = Marker()
        marker.header.frame_id = "vicon/world"
        marker.header.stamp = rospy.Time.now()
        marker.ns = "hoop"
        marker.id = 1
        marker.type = Marker.LINE_STRIP
        marker.action = Marker.ADD
        marker.pose.orientation.w = 1.0
        marker.scale.x = 0.01
        marker.color.g = 1.0
        marker.color.a = 1.0
        marker.points = []
        marker.points = self.list_forecasted
        self.pub_forecasted_viz.publish(marker)

        # Publish forecasted state
        forecasted_msg = PoseStamped()
        forecasted_msg.header.stamp = rospy.Time.now()
        forecasted_msg.header.frame_id = "vicon/world"
        forecasted_msg.pose.position.x = self.list_forecasted[-1].x
        forecasted_msg.pose.position.y = self.list_forecasted[-1].y
        forecasted_msg.pose.position.z = self.list_forecasted[-1].z
        self.pub_forecasted.publish(forecasted_msg)

if __name__ == '__main__':
    try:
        node = KalmanFilterNode()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass
