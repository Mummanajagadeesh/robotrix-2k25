import sys
import traceback
import time
import os
import math
from coppeliasim_zmqremoteapi_client import RemoteAPIClient
import numpy as np
import cv2
import random

# Constants and parameters
ball_color_lower = np.array([5, 100, 150])  # Lower HSV range (adjust saturation/value if necessary)
ball_color_upper = np.array([20, 255, 255])  # Upper HSV range (adjust hue to match the ball's color)

# Adjusted PID parameters for smoother and slower movement
pid_x = {'kp': 0.5, 'ki': 0.002, 'kd': 0.02, 'prev_error': 0, 'integral': 0}
pid_y = {'kp': 0.5, 'ki': 0.002, 'kd': 0.02, 'prev_error': 0, 'integral': 0}
pid_z = {'kp': 0.5, 'ki': 0.002, 'kd': 0.02, 'prev_error': 0, 'integral': 0}

def detect_ball_position(img_r, img_l):
    """
    Detects the ball's 3D position using stereo vision from two camera images.
    """
    # Process right and left images
    pos_r = detect_ball_position_single(img_r)
    pos_l = detect_ball_position_single(img_l)

    if pos_r is None or pos_l is None:
        print("Ball position could not be detected in one or both images.")
        return None

    # Stereo vision parameters
    image_width = 512
    fov = 80 * (math.pi / 180)  # Convert degrees to radians
    focal_length = image_width / (2 * math.tan(fov / 2))  # Calculate focal length in pixels
    baseline = 0.7  # Distance between cameras (meters)
    
    # Compute disparity
    disparity = abs(pos_l[0] - pos_r[0])
    
    if disparity <= 1e-5:  # Avoid division by zero
        print("Disparity is too small; depth cannot be calculated.")
        return None

    # Calculate depth using the formula: Z = (f * B) / disparity
    z = (focal_length * baseline) / disparity

    # Convert image pixel coordinates to world coordinates (X, Y)
    # Assume the image plane's origin is at the center of the image
    cx, cy = image_width / 2, image_width / 2  # Image center
    x = ((pos_l[0] - cx) * z) / focal_length  # Convert x-coordinate
    y = ((pos_l[1] - cy) * z) / focal_length  # Convert y-coordinate

    print(f"Detected Ball Position (x, y, z): ({x:.3f}, {y:.3f}, {z:.3f})")
    return (x, y, z)


def detect_ball_position_single(image):
    """
    Detects the ball's 2D position in a single image using a bounding circle on the masked area.
    """
    hsv_image = cv2.cvtColor(image, cv2.COLOR_BGR2HSV)
    
    # Create the mask
    mask = cv2.inRange(hsv_image, ball_color_lower, ball_color_upper)
    
    # Apply Gaussian blur to reduce noise
    mask = cv2.GaussianBlur(mask, (5, 5), 0)

    # Morphological closing to ensure the ball's region is continuous
    mask = cv2.morphologyEx(mask, cv2.MORPH_CLOSE, np.ones((7, 7), np.uint8))

    # Find contours in the mask
    contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
    
    if contours:
        # Find the largest contour (assumes the ball is the largest white object)
        largest_contour = max(contours, key=cv2.contourArea)

        # Fit a minimum enclosing circle around the largest contour
        (x, y), radius = cv2.minEnclosingCircle(largest_contour)
        center = (int(x), int(y))
        radius = int(radius)

        # Draw the circle on the original image (for debugging)
        output_image = image.copy()
        cv2.circle(output_image, center, radius, (0, 255, 0), 2)  # Green circle
        cv2.imshow("Detected Ball with Bounding Circle", output_image)
        cv2.waitKey(1)

        # Only return the center if the radius is above a threshold (to avoid noise)
        if radius > 2:  # Adjust this threshold as needed
            print(f"Ball Center Detected: {center}, Radius: {radius}")
            return center
        else:
            print("Detected contour is too small to be the ball.")
            return None
    else:
        print("No contours found in mask.")
        return None



def calculate_velocity(error, pid):
    """
    Calculate velocity using PID control.
    """
    pid['integral'] += error
    derivative = error - pid['prev_error']
    velocity = pid['kp'] * error + pid['ki'] * pid['integral'] + pid['kd'] * derivative
    pid['prev_error'] = error
    return velocity

MAX_VELOCITY = 5  # Adjust this value based on your system's response

# Global variables to track previous positions
prev_ball_position = None
prev_timestamp = None
prediction_horizon = 1  # Predict the position 1 second ahead

def predict_ball_position(current_position, timestamp):
    """
    Predicts the ball's future position based on its velocity.
    """
    global prev_ball_position, prev_timestamp

    if prev_ball_position is None or prev_timestamp is None:
        # Can't predict without previous data, return current position
        prev_ball_position = current_position
        prev_timestamp = timestamp
        return current_position

    # Calculate velocity: change in position over time
    delta_position = (current_position[0] - prev_ball_position[0], 
                      current_position[1] - prev_ball_position[1], 
                      current_position[2] - prev_ball_position[2])
    delta_time = timestamp - prev_timestamp
    
    # Compute the predicted future position
    predicted_position = (
		current_position[0] + delta_position[0] * prediction_horizon / delta_time,
		current_position[1] + delta_position[1] * prediction_horizon / delta_time,
		current_position[2] + delta_position[2] * prediction_horizon / delta_time
	)


    # Update previous data for the next prediction
    prev_ball_position = current_position
    prev_timestamp = timestamp

    return predicted_position

# Global variables to track reset times
last_ball_reset_time = None
hoop_reset_position = [0.260, -3.695, 1.9355]  # Example reset position of the hoop (adjust as needed)
ball_reset_threshold = 0.15
# Threshold distance to determine if the ball is reset

start_t = None  # Ball throw start time
end_t = None    # Ball throw end time

def reset_hoop_position(sim, hoop_odom, initial_hoop_position, actuators, tolerance=0.01, max_velocity=0.5):
    """
    Resets the hoop's position to its initial state using joint velocities.

    Parameters:
    - sim: Simulation object for controlling actuators and obtaining state.
    - hoop_odom: Object representing the hoop's current odometry data.
    - initial_hoop_position: The initial [x, y, z] position of the hoop.
    - actuators: A dictionary containing the actuators for x, y, z axes.
      e.g., {'x': actuator_x, 'y': actuator_y, 'z': actuator_z}.
    - tolerance: The acceptable margin of error for alignment (in meters).
    - max_velocity: The maximum velocity to apply to the actuators.
    """
    # Get the current position of the hoop
    current_position = sim.getObjectPosition(hoop_odom, -1)

    # Calculate the difference along each axis
    delta_x = initial_hoop_position[0] - current_position[0]
    delta_y = initial_hoop_position[1] - current_position[1]
    delta_z = initial_hoop_position[2] - current_position[2]

    # Adjust velocities for each actuator based on the required movement
    velocity_x = max(-max_velocity, min(max_velocity, delta_x))
    velocity_y = max(-max_velocity, min(max_velocity, delta_y))
    velocity_z = max(-max_velocity, min(max_velocity, delta_z))

    # Set actuator velocities until the hoop is within the tolerance
    while (
        abs(delta_x) > tolerance
        or abs(delta_y) > tolerance
        or abs(delta_z) > tolerance
    ):
        sim.setJointTargetVelocity(actuators['x'], velocity_x)
        sim.setJointTargetVelocity(actuators['y'], velocity_y)
        sim.setJointTargetVelocity(actuators['z'], velocity_z)

        # Update current position and calculate deltas again
        current_position = sim.getObjectPosition(hoop_odom, -1)
        delta_x = initial_hoop_position[0] - current_position[0]
        delta_y = initial_hoop_position[1] - current_position[1]
        delta_z = initial_hoop_position[2] - current_position[2]

        # Recalculate velocities
        velocity_x = max(-max_velocity, min(max_velocity, delta_x))
        velocity_y = max(-max_velocity, min(max_velocity, delta_y))
        velocity_z = max(-max_velocity, min(max_velocity, delta_z))

    # Stop the actuators once the hoop is aligned
    sim.setJointTargetVelocity(actuators[0], 0)
    sim.setJointTargetVelocity(actuators[1], 0)
    sim.setJointTargetVelocity(actuators[2], 0)

    print(f"Hoop position reset to: {initial_hoop_position}")

def detect_ball_reset(ball_position, stand_position):
    """
    Detect if the ball has returned to the top of the stand.
    Returns True if the ball is reset, False otherwise.
    """
    distance = np.linalg.norm(np.array(ball_position) - np.array(stand_position))
    return distance < ball_reset_threshold

def control_logic(sim):
    global last_ball_reset_time, start_t, end_t, hoop_reset_position

    # Actuator Handles
    actuator_x = sim.getObject('/basket_bot/actuator_x')
    actuator_y = sim.getObject('/basket_bot/actuator_y')
    actuator_z = sim.getObject('/basket_bot/actuator_z')

    # Camera Handles
    cam_r = sim.getObject('/basket_bot/cam_r')
    cam_l = sim.getObject('/basket_bot/cam_l')

    # Hoop Odom Handle
    hoop_odom = sim.getObject('/basket_bot/hoop_odom')

    # Ball Stand Position (for reset detection)
    ball_stand_position = [0, 3.5, 0.3]  # Adjust based on the actual position of the stand

    # Record the initial hoop position (using hoop_odom as reference)
    initial_hoop_position = sim.getObjectPosition(hoop_odom, -1)
    print(f"Initial Hoop Position Recorded: {initial_hoop_position}")

    # Define the specific initial position values
    hoop_initial_position = [0.260, -3.695, 1.9355]  # When no ball is detected

    while sim.getSimulationState() != sim.simulation_advancing_abouttostop:
        try:
            # Get images from cameras
            img_r, res_r = sim.getVisionSensorImg(cam_r)
            img_l, res_l = sim.getVisionSensorImg(cam_l)

            if res_r != [512, 512] or res_l != [512, 512]:
                print("Error: Camera resolution mismatch.")
                continue

            # Unpack the image byte buffer using sim.unpackUInt8Table
            img_r = sim.unpackUInt8Table(img_r)
            img_l = sim.unpackUInt8Table(img_l)

            img_r = cv2.cvtColor(cv2.flip(np.array(img_r, dtype=np.uint8).reshape((512, 512, 3)), 0), cv2.COLOR_BGR2RGB)
            img_l = cv2.cvtColor(cv2.flip(np.array(img_l, dtype=np.uint8).reshape((512, 512, 3)), 0), cv2.COLOR_BGR2RGB)
            
            # Detect ball position
            ball_position = detect_ball_position(img_r, img_l)

            if ball_position is None:
                print("Ball not detected.")
                
                # Align the hoop_odom with the initial position if no ball is detected
                print(f"Aligning hoop to initial position: {hoop_initial_position}")
                sim.setObjectPosition(hoop_odom, -1, hoop_initial_position)
                
                # Stop the actuators to prevent unnecessary movement
                sim.setJointTargetVelocity(actuator_x, 0)
                sim.setJointTargetVelocity(actuator_y, 0)
                sim.setJointTargetVelocity(actuator_z, 0)
                
                time.sleep(0.05)
                continue
            # Get the current timestamp
            timestamp = time.time()

            # Use the time from the random throw logic for synchronization
            if start_t and end_t and (end_t - start_t) >= 5:  # Check if 5 seconds have passed
                # Ball has completed its throw, reset hoop position
                reset_hoop_position(sim, hoop_odom, initial_hoop_position,[actuator_x, actuator_y, actuator_z])
                last_ball_reset_time = timestamp  # Update reset time
                print("Ball throw complete, hoop reset.")

            # Check if the ball has reset
            if detect_ball_reset(ball_position, ball_stand_position):
                if last_ball_reset_time is None or (timestamp - last_ball_reset_time) > 1:  # Ensure at least 1 second between resets
                    # Reset hoop position
                    reset_hoop_position(sim, hoop_odom, initial_hoop_position)
                    last_ball_reset_time = timestamp  # Update reset time
                    print("Ball reset detected, hoop reset.")

            # Predict future ball position if the ball is detected
            predicted_ball_position = predict_ball_position(ball_position, timestamp)

            # Calculate errors and velocities based on the predicted position
            hoop_position = sim.getObjectPosition(hoop_odom, -1)
            x_error = predicted_ball_position[0] - hoop_position[0]
            y_error = predicted_ball_position[1] - hoop_position[1]
            z_error = predicted_ball_position[2] - hoop_position[2]

            x_velocity = calculate_velocity(x_error, pid_x)
            y_velocity = calculate_velocity(y_error, pid_y)
            z_velocity = calculate_velocity(z_error, pid_z)

            # Limit velocities to avoid too fast movement
            x_velocity = max(-MAX_VELOCITY, min(MAX_VELOCITY, x_velocity))
            y_velocity = max(-MAX_VELOCITY, min(MAX_VELOCITY, y_velocity))
            z_velocity = max(-MAX_VELOCITY, min(MAX_VELOCITY, z_velocity))

            # Send velocities to actuators
            sim.setJointTargetVelocity(actuator_x, x_velocity)
            sim.setJointTargetVelocity(actuator_y, y_velocity)
            sim.setJointTargetVelocity(actuator_z, z_velocity)

            time.sleep(0.05)

        except Exception as e:
            print(f"Error processing images: {e}")
            time.sleep(0.05)
            continue

    return None


######### YOU ARE NOT ALLOWED TO MAKE CHANGES TO THE MAIN CODE BELOW #########

if __name__ == "__main__":
    client = RemoteAPIClient()
    sim = client.getObject('sim')    

    try:
        # Start the simulation
        return_code = sim.startSimulation()
        if sim.getSimulationState() != sim.simulation_stopped:
            print('\nSimulation started correctly in CoppeliaSim.')
        else:
            print('\nSimulation could not be started correctly in CoppeliaSim.')
            sys.exit()

        # Runs the control logic
        control_logic(sim)

        # Stop the simulation
        return_code = sim.stopSimulation()
        time.sleep(0.5)
        if sim.getSimulationState() == sim.simulation_stopped:
            print('\nSimulation stopped correctly in CoppeliaSim.')
        else:
            print('\nSimulation could not be stopped correctly in CoppeliaSim.')
            sys.exit()

    except Exception as e:
        print(f"Error occurred: {e}")
        traceback.print_exc()
        sys.exit()
