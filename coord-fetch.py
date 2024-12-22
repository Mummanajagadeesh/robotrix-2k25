import numpy as np
import cv2
import math
import time

# Constants
RESOLUTION = (512, 512)  # Camera resolution
FOV = 80  # Camera field of view in degrees
BASELINE = 0.7  # Distance between the two cameras (meters)
FOCAL_LENGTH = RESOLUTION[0] / (2 * math.tan(math.radians(FOV / 2)))  # Approximation

# Ball Detection Logic (Your method)
def detect_ball(image):
    """Detect ball in the given image using user-defined logic."""
    gray = cv2.cvtColor(image, cv2.COLOR_BGR2GRAY)
    _, binary = cv2.threshold(gray, 200, 255, cv2.THRESH_BINARY)
    contours, _ = cv2.findContours(binary, cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)
    if contours:
        largest_contour = max(contours, key=cv2.contourArea)
        x, y, w, h = cv2.boundingRect(largest_contour)
        return (int(x + w / 2), int(y + h / 2))  # Return the center of the bounding box
    return None

# Calculate 3D Coordinates Using Stereo Vision
def calculate_3d_coordinates(pos_l, pos_r):
    """Calculate 3D coordinates using stereo vision."""
    disparity = pos_l[0] - pos_r[0]
    if disparity == 0:  # Avoid division by zero
        return None

    z = (FOCAL_LENGTH * BASELINE) / disparity
    x = (pos_l[0] - RESOLUTION[0] / 2) * z / FOCAL_LENGTH
    y = (pos_l[1] - RESOLUTION[1] / 2) * z / FOCAL_LENGTH
    return x, y, z

# Fetch Initial Ball Coordinates
def fetch_initial_ball_coordinates(sim):
    """Fetch the initial 3D coordinates of the ball."""
    # Get camera handles
    cam_r = sim.getObject('/basket_bot/cam_r')
    cam_l = sim.getObject('/basket_bot/cam_l')

    while True:
        # Fetch images from cameras
        img_r, res_r = sim.getVisionSensorImg(cam_r)
        img_l, res_l = sim.getVisionSensorImg(cam_l)

        # Ensure valid image resolution
        if res_r != [512, 512] or res_l != [512, 512]:
            print("Error: Camera resolution mismatch.")
            continue

        # Process images
        img_r = np.array(sim.unpackUInt8Table(img_r), dtype=np.uint8).reshape((512, 512, 3))
        img_l = np.array(sim.unpackUInt8Table(img_l), dtype=np.uint8).reshape((512, 512, 3))

        img_r = cv2.flip(img_r, 0)  # Flip vertically
        img_l = cv2.flip(img_l, 0)  # Flip vertically

        # Detect ball position
        pos_r = detect_ball(img_r)
        pos_l = detect_ball(img_l)

        if pos_r is not None and pos_l is not None:
            # Calculate 3D coordinates
            coords_3d = calculate_3d_coordinates(pos_l, pos_r)
            coords_3d_modified = list(coords_3d)
            coords_3d_modified[0] += 7.61245
            coords_3d_modified[1] += -11.285 + 4.5 
            coords_3d_modified[2] += -26.69528 + 0.605
            # coords_3d_modified[0] = int(coords_3d_modified[0])
            # coords_3d_modified[1] = int(coords_3d_modified[1])
            # coords_3d_modified[2] = int(coords_3d_modified[2])

            print(f"Initial Ball 3D Coordinates: {coords_3d_modified}")
            return coords_3d

        # Exit loop if no ball detected within 5 seconds
        time.sleep(0.05)

# Main Function
if __name__ == "__main__":
    from coppeliasim_zmqremoteapi_client import RemoteAPIClient

    client = RemoteAPIClient()
    sim = client.getObject('sim')

    # Start simulation
    sim.startSimulation()
    time.sleep(1)  # Allow time for the simulation to start

    try:
        coords_3d = fetch_initial_ball_coordinates(sim)
        print(f"Ball Initial 3D Coordinates: {coords_3d}")
    finally:
        sim.stopSimulation()
        print("Simulation stopped.")
