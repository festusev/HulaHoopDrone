import numpy as np
import rospy
from geometry_msgs.msg import Twist

# Constants
TIME_STEP = 0.1  # Time increment for prediction (seconds)
LOOKAHEAD_TIME = 3.0  # Total lookahead time for trajectory prediction (seconds)
MAX_SPEED = 1.0  # Maximum drone speed (m/s)
MAX_HEIGHT = 10.0  # Maximum height the drone can fly (meters)

# Weights for feasibility scoring
WEIGHTS = {
    "apex": 0.4,
    "align": 0.3,
    "time": 0.2,
    "risk": 0.1,
}

# Helper function to compute feasibility score
def feasibility_score(point, apex_height, drone_position, drone_velocity, max_speed, max_height):
    """Compute the feasibility score for a candidate point."""
    required_velocity = (point - drone_position) / np.linalg.norm(point - drone_position)
    required_speed = np.linalg.norm(required_velocity)

    # Height constraint
    if point[2] > max_height or point[2] < 0:
        return -np.inf  # Reject points outside the height constraint

    # Speed constraint
    if required_speed > max_speed:
        return -np.inf  # Reject points requiring speed > max_speed

    # Distance to apex
    apex_score = 1 - abs(point[2] - apex_height) / apex_height

    # Alignment score
    align_score = np.dot(required_velocity, drone_velocity / np.linalg.norm(drone_velocity))

    # Time score
    time_score = 1 / np.linalg.norm(point - drone_position)

    # Risk score (simple example: no obstacles for now)
    risk_score = 1.0

    # Total weighted score
    total_score = (
        WEIGHTS["apex"] * apex_score +
        WEIGHTS["align"] * align_score +
        WEIGHTS["time"] * time_score +
        WEIGHTS["risk"] * risk_score
    )
    return total_score

# Predict the hoop's trajectory
def predict_hoop_trajectory(current_pos, current_vel, max_height):
    """Predict the hoop's trajectory considering maximum height constraint."""
    trajectory = []
    g = np.array([0, 0, -9.81])  # Gravity vector (m/s^2)
    for t in np.arange(0, LOOKAHEAD_TIME, TIME_STEP):
        pos = current_pos + current_vel * t + 0.5 * g * t**2
        if pos[2] <= max_height and pos[2] > 0:  # Respect height constraints
            trajectory.append(pos)
    return trajectory

# Find the best feasible point
def find_best_point(trajectory, apex_height, drone_position, drone_velocity):
    """Find the best feasible point on the hoop's trajectory."""
    best_score = -np.inf
    best_point = None
    for point in trajectory:
        score = feasibility_score(
            point,
            apex_height,
            drone_position,
            drone_velocity,
            max_speed=MAX_SPEED,
            max_height=MAX_HEIGHT
        )
        if score > best_score:
            best_score = score
            best_point = point
    return best_point

# Main control loop
def main():
    rospy.init_node("drone_hoop_navigation")

    # Publishers
    cmd_vel_pub = rospy.Publisher("/cmd_vel", Twist, queue_size=10)

    # Simulation parameters (replace these with real sensor data)
    hoop_position = np.array([0.0, 0.0, 5.0])  # Initial hoop position (x, y, z)
    hoop_velocity = np.array([0.0, 0.0, 2.0])  # Initial hoop velocity (x, y, z)
    drone_position = np.array([0.0, 0.0, 0.0])  # Initial drone position (x, y, z)
    drone_velocity = np.array([0.0, 0.0, 0.0])  # Initial drone velocity (x, y, z)

    rate = rospy.Rate(10)  # Control loop frequency (10 Hz)

    while not rospy.is_shutdown():
        # Predict hoop trajectory
        hoop_apex_height = hoop_position[2] + (hoop_velocity[2]**2) / (2 * 9.81)
        hoop_trajectory = predict_hoop_trajectory(hoop_position, hoop_velocity, MAX_HEIGHT)

        # Find the best feasible point
        best_point = find_best_point(hoop_trajectory, hoop_apex_height, drone_position, drone_velocity)

        # Compute velocity command to reach the best point
        if best_point is not None:
            desired_velocity = (best_point - drone_position) / np.linalg.norm(best_point - drone_position)

            # Constrain velocity magnitude
            if np.linalg.norm(desired_velocity) > MAX_SPEED:
                desired_velocity = desired_velocity / np.linalg.norm(desired_velocity) * MAX_SPEED

            # Publish velocity command
            cmd_msg = Twist()
            cmd_msg.linear.x = desired_velocity[0]
            cmd_msg.linear.y = desired_velocity[1]
            cmd_msg.linear.z = desired_velocity[2]
            cmd_vel_pub.publish(cmd_msg)

        # Update drone position for simulation purposes
        drone_position += drone_velocity * TIME_STEP

        rate.sleep()

if __name__ == "__main__":
    try:
        main()
    except rospy.ROSInterruptException:
        pass
