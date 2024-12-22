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
pid_x = {'kp': 0.05, 'ki': 0.002, 'kd': 0.02, 'prev_error': 0, 'integral': 0}
pid_y = {'kp': 0.05, 'ki': 0.002, 'kd': 0.02, 'prev_error': 0, 'integral': 0}
pid_z = {'kp': 0.05, 'ki': 0.002, 'kd': 0.02, 'prev_error': 0, 'integral': 0}



def detect_ball_position(img_r, img_l):
    """
    Detects the ball's 3D position using stereo vision from two camera images.
    """
    # Process right and left images
    pos_r = detect_ball_position_single(img_r)
    pos_l = detect_ball_position_single(img_l)

    if pos_r is None or pos_l is None:
        return None

    # Stereo vision: compute depth (z-axis) based on disparity
    focal_length = 512 / 2  # Camera focal length (pixels)
    baseline = 0.7  # Distance between cameras (meters)
    
    disparity = pos_l[0] - pos_r[0]
    if disparity == 0:
        return None

    z = (focal_length * baseline) / disparity
    x = (pos_l[0] + pos_r[0]) / 2  # Average x-coordinates
    y = (pos_l[1] + pos_r[1]) / 2  # Average y-coordinates

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
        if radius > 5:  # Adjust this threshold as needed
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

import time

# Global variables to track reset times
last_ball_reset_time = None
hoop_reset_position = [0, 0, 2]  # Example reset position of the hoop (adjust as needed)
ball_reset_threshold = 0.15
  # Threshold distance to determine if the ball is reset

# Adding necessary variables from the provided code
start_t = None  # Ball throw start time
end_t = None    # Ball throw end time

def detect_ball_reset(ball_position, stand_position):
    """
    Detect if the ball has returned to the top of the stand.
    Returns True if the ball is reset, False otherwise.
    """
    distance = np.linalg.norm(np.array(ball_position) - np.array(stand_position))
    return distance < ball_reset_threshold

def reset_hoop_position(sim, hoop_odom):
    """
    Resets the hoop's position after each shot.
    """
    sim.setObjectPosition(hoop_odom, -1, hoop_reset_position)
    print("Hoop position reset to:", hoop_reset_position)

def control_logic(sim):
    global last_ball_reset_time, start_t, end_t

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
    ball_stand_position = [0, 0, 0.4]  # Adjust based on the actual position of the stand

    while sim.getSimulationState() != sim.simulation_advancing_abouttostop:
        # Get hoop position
        hoop_position = sim.getObjectPosition(hoop_odom, -1)

        # Get images from cameras
        img_r, res_r = sim.getVisionSensorImg(cam_r)
        img_l, res_l = sim.getVisionSensorImg(cam_l)

        if res_r != [512, 512] or res_l != [512, 512]:
            print("Error: Camera resolution mismatch.")
            continue

        # Unpack the image byte buffer using sim.unpackUInt8Table
        img_r = sim.unpackUInt8Table(img_r)
        img_l = sim.unpackUInt8Table(img_l)

        try:
            img_r = cv2.cvtColor(cv2.flip(np.array(img_r, dtype=np.uint8).reshape((512, 512, 3)), 0), cv2.COLOR_BGR2RGB)
            img_l = cv2.cvtColor(cv2.flip(np.array(img_l, dtype=np.uint8).reshape((512, 512, 3)), 0), cv2.COLOR_BGR2RGB)
            
            # Detect ball position (using bounding circle technique)
            ball_position = detect_ball_position(img_r, img_l)

            if ball_position is None:
                print("Ball not detected.")
                time.sleep(0.05)
                continue

            # Get the current timestamp (simulation time or frame count)
            timestamp = time.time()

            # Use the time from the random throw logic for synchronization
            if start_t and end_t and (end_t - start_t) >= 5:  # Check if 5 seconds have passed
                # Ball has completed its throw, reset hoop position
                reset_hoop_position(sim, hoop_odom)
                last_ball_reset_time = timestamp  # Update reset time
                print("Ball throw complete, hoop reset.")

            # Check if the ball has reset
            if detect_ball_reset(ball_position, ball_stand_position):
                if last_ball_reset_time is None or (timestamp - last_ball_reset_time) > 1:  # Ensure at least 1 second between resets
                    # Reset hoop position
                    reset_hoop_position(sim, hoop_odom)
                    last_ball_reset_time = timestamp  # Update reset time
                    print("Ball reset detected, hoop reset.")

            # Predict future ball position if the ball is detected
            predicted_ball_position = predict_ball_position(ball_position, timestamp)

            # Calculate errors and velocities based on the predicted position
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

        ## Start the simulation using ZeroMQ RemoteAPI
        try:
            return_code = sim.startSimulation()
            if sim.getSimulationState() != sim.simulation_stopped:
                print('\nSimulation started correctly in CoppeliaSim.')
            else:
                print('\nSimulation could not be started correctly in CoppeliaSim.')
                sys.exit()

        except Exception:
            print('\n[ERROR] Simulation could not be started !!')
            traceback.print_exc(file=sys.stdout)
            sys.exit()

        ## Runs the control logic written by participants
        try:
            control_logic(sim)

        except Exception:
            print('\n[ERROR] Your control_logic function threw an Exception, kindly debug your code!')
            print('Stop the CoppeliaSim simulation manually if required.\n')
            traceback.print_exc(file=sys.stdout)
            print()
            sys.exit()

        
        ## Stop the simulation
        try:
            return_code = sim.stopSimulation()
            time.sleep(0.5)
            if sim.getSimulationState() == sim.simulation_stopped:
                print('\nSimulation stopped correctly in CoppeliaSim.')
            else:
                print('\nSimulation could not be stopped correctly in CoppeliaSim.')
                sys.exit()

        except Exception:
            print('\n[ERROR] Simulation could not be stopped !!')
            traceback.print_exc(file=sys.stdout)
            sys.exit()

    except Exception:
        print('[ERROR] ZeroMQ RemoteAPI connection was lost or there was some error in starting the simulation.')
        traceback.print_exc(file=sys.stdout)
        sys.exit()
