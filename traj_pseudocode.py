import numpy as np
import rospy
from geometry_msgs.msg import Twist
from djitellopy import Tello
from sklearn.preprocessing import PolynomialFeatures
from sklearn.linear_model import LinearRegression
import time
import numpy as np
import time

# Constants
TIME_STEP = 0.1  # Time increment for prediction (seconds)
LOOKAHEAD_TIME = 3.0  # Total lookahead time for trajectory prediction (seconds)
MAX_SPEED = 1.0  # Maximum drone speed (m/s)
MAX_HEIGHT = 10.0  # Maximum height the drone can fly (meters)
PAST_POINTS_WINDOW = 20  # Number of past points for fitting the parabola

# Weights for feasibility scoring
WEIGHTS = {
    "apex": 0.4,
    "align": 0.3,
    "time": 0.2,
    "risk": 0.1,
}

# Buffers for past hoop positions
past_positions = []

def fit_3d_parabola(past_positions, past_times):
    """
    Fit a 3D parabola to observed positions and times.

    Parameters:
    - past_positions: List of observed positions [(x, y, z)]
    - past_times: Corresponding times [t1, t2, ...]

    Returns:
    - coeff_x: Coefficients [a_x, b_x, c_x] for x(t)
    - coeff_y: Coefficients [a_y, b_y, c_y] for y(t)
    - coeff_z: Coefficients [a_z, b_z, c_z] for z(t)
    """
    # Prepare the time matrix for a quadratic fit: [t^2, t, 1]
    A = np.vstack([np.power(past_times, 2), past_times, np.ones(len(past_times))]).T

    # Extract x, y, z from past_positions
    past_positions = np.array(past_positions)
    x_data, y_data, z_data = past_positions[:, 0], past_positions[:, 1], past_positions[:, 2]

    # Fit each dimension
    coeff_x = np.linalg.lstsq(A, x_data, rcond=None)[0]  # [a_x, b_x, c_x]
    coeff_y = np.linalg.lstsq(A, y_data, rcond=None)[0]  # [a_y, b_y, c_y]
    coeff_z = np.linalg.lstsq(A, z_data, rcond=None)[0]  # [a_z, b_z, c_z]

    return coeff_x, coeff_y, coeff_z


def predict_3d_trajectory(coeff_x, coeff_y, coeff_z, max_height):
    """
    Predict the hoop's trajectory using fitted 3D parabola coefficients.

    Parameters:
    - coeff_x, coeff_y, coeff_z: Coefficients for x(t), y(t), z(t)
    - max_height: Maximum height constraint

    Returns:
    - trajectory: List of predicted positions [(x, y, z)]
    """
    trajectory = []

    for t in np.arange(0, LOOKAHEAD_TIME, TIME_STEP):
        x = coeff_x[0] * t**2 + coeff_x[1] * t + coeff_x[2]
        y = coeff_y[0] * t**2 + coeff_y[1] * t + coeff_y[2]
        z = coeff_z[0] * t**2 + coeff_z[1] * t + coeff_z[2]

        # Respect height constraints
        if 0 <= z <= max_height:
            trajectory.append(np.array([x, y, z]))
    
    return trajectory



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

def update_past_positions(past_positions, new_position):
    """Update the buffer of past hoop positions."""
    if len(past_positions) >= PAST_POINTS_WINDOW:
        past_positions.pop(0)
    past_positions.append(new_position)

def passed_hoop():
    pass

# Main control loop
def main():
    rospy.init_node("drone_hoop_navigation")

    # Publishers
    # Simulation parameters (replace these with real sensor data)
    hoop_position = np.array([0.0, 0.0, 5.0])  # Initial hoop position (x, y, z)
    hoop_velocity = np.array([0.0, 0.0, 2.0])  # Initial hoop velocity (x, y, z)
    drone_position = np.array([0.0, 0.0, 0.0])  # Initial drone position (x, y, z)
    drone_velocity = np.array([0.0, 0.0, 0.0])  # Initial drone velocity (x, y, z)
    last_time = time.time()

    rate = rospy.Rate(10)  # Control loop frequency (10 Hz)

    tello = Tello()
    tello.connect()

    tello.takeoff()
    while not rospy.is_shutdown():
        # Simulate time increment
        current_time = time.time()
        time_step = current_time - last_time
        last_time = current_time

        # Update past positions buffer
        update_past_positions(past_positions, hoop_position, current_time)

        # Predict hoop trajectory using fitted parabola
        coeffs = fit_3d_parabola(past_positions, current_time)
        hoop_trajectory = predict_3d_trajectory(coeffs[0], coeffs[1], coeffs[2], 100.0)

        # Find the best feasible point
        hoop_apex_height = max(pos[2] for pos in hoop_trajectory) if hoop_trajectory else 5.0
        best_point = find_best_point(hoop_trajectory, hoop_apex_height, drone_position, drone_velocity)

        # Compute velocity command to reach the best point
        if best_point is not None:
            desired_velocity = (best_point - drone_position) / np.linalg.norm(best_point - drone_position)

            # Constrain velocity magnitude
            if np.linalg.norm(desired_velocity) > MAX_SPEED:
                desired_velocity = desired_velocity / np.linalg.norm(desired_velocity) * MAX_SPEED

            # Send velocity command
            tello.send_rc_control(int(desired_velocity[0]*100), int(desired_velocity[1]*100), int(desired_velocity[2]*100), 0)

        # Update drone position for simulation purposes
        drone_position += desired_velocity * time_step
        rate.sleep()
    
        # Check if the drone has passed the hoop
        if passed_hoop(drone_position):
            break

    tello.land()

if __name__ == "__main__":
    try:
        main()
    except rospy.ROSInterruptException:
        pass
