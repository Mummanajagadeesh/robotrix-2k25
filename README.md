# Robotrix-2k25

## Useful Resources
| Resource Name                                  | Link                                                                 |
|-----------------------------------------------|----------------------------------------------------------------------|
| **Unstop Registration**                       | [Robotrix-2025 on Unstop](https://unstop.com/hackathons/robotrix-2025-diodexcelerate-2025-ieee-nitk-1265503) |
| **OpenCV Learning Resources**                | [OpenCV Session Materials](https://drive.google.com/drive/folders/1I3onFJ9BmKhO2ySJHbKbqaqb6ShACZez)       |
| **Getting Started with CoppeliaSim**         | [CoppeliaSim Guide](https://docs.google.com/document/d/16T91vF_IcL98n5h6RFddoQVI-fivpbWYlDm_yUQMZ3E/edit?tab=t.0) |
| **Robotrix 2025 Problem Statement**          | [Problem Statement Document](https://docs.google.com/document/d/1vff6XrevOeJcRv-ABr8Lx3oOjOZTUBogC9AZYqRhWjU/edit?tab=t.0#heading=h.bn1tbf9oj3ut) |
| **CoppeliaSim Technical Session**            | [Session Video](https://drive.google.com/file/d/1xbQdN7m91ATBxBm8FzqdT0GL89Kq6rRG/view)                    |

## Problem Statement

### Robotrix 2024-25 Final Hackathon Problem Statement

#### Basket the Ball!!!

**Objective:**  
The provided CoppeliaSim simulation scene contains an arena featuring a robot, a basketball hoop, and a basketball shooter. The robot can move the hoop along any of the three axes. The task is to successfully dunk the basketball into the hoop by adjusting its position to align with the trajectory of the basketball, which is shot automatically by the basketball shooter. Two cameras are mounted on the backboard to assist in this task

#### Instructions:
1. **Download Resources:**
   - [CoppeliaSim world file](#)
   - [Python script template](#)
2. **No Modifications to Arena:** Do not edit the provided world file
3. **Python Script:** Edit the script template to achieve the stated objective
4. **Coding Standards:** Ensure your code is neat, structured, and well-documented

#### Specifications:
- **Arena Dimensions:** 10000 mm x 10000 mm
- **Hoop Diameter:** 320 mm
- **Backboard Dimensions:** 960 mm x 720 mm
- **Height of Ball Shooter:** 400 mm
- **Vision Sensors:**
  - Resolution: 512 x 512 px
  - Field of View: 80°
  - Distance Between Cameras: 700 mm
  - Vertical Distance Between Cameras and Hoop: 390 mm
- **Position Data:** Use the dummy element `hoop_odom` to find the hoop’s position (the `getObjectPosition()` function is restricted to this purpose only)
- **Control Mechanism:** All sliding joints can be controlled using velocity data.

#### Scoring:
Each run will have 10 shots. The simulation stops automatically after 10 shots. Teams score 10 points for every successful dunk, with a maximum score of 100 points


---
# My Approach 

### Ball Detection and Tracking Approach

Since we do not have access to `getObjectPosition` for the `/ball`, we need to calculate its 3D position using the cameras. The ball is the only object in orange, so I used its color properties from VREP, which are $$\( \text{RGB} = (1.00, 0.411, 0.08) \)$$. I converted this into the HSV space because it's easier to create a mask for this color range.

### Color Masking and Preprocessing

I applied a mask using some offsets to account for different lighting conditions. This means that pixels within the given HSV range are shown as white, while others are shown as black.

Next, I applied a **Gaussian blur** to reduce noise and used **morphological closing** (`morphologyEx`) to ensure that the detected region is continuous. 

#### Gaussian Blur

Gaussian blur is used to smooth the image by averaging surrounding pixels. This helps reduce noise and makes the contours easier to detect by softening sharp edges.

#### Morphological Closing

Morphological closing is a combination of dilation followed by erosion. It helps in closing small holes and gaps in the detected regions (white areas in the mask). This ensures that the contours are solid and continuous.

### Contour Detection

After preprocessing, I found the **contours** in the image. Contours are simply the boundaries or outlines of objects. Once the contours are detected, I selected the largest one (using a set threshold) and bounded it using a circle with the function `minEnclosingCircle`.

Since the image is in black and white, I used the following approach:
1. Traverse each horizontal row from left to right.
2. Whenever there’s a sudden change in pixel intensity (e.g., from black to white), that indicates the boundary of the ball.
3. For example, if the pixel intensity increases sharply between the 4th and 5th pixels, then we know that this part is likely the ball.

By detecting such changes, we calculate the area of the detected ball indirectly by counting the number of pixels forming the contour. The largest contour will be considered the ball, while smaller contours are ignored. 

### Circle Bounding and Center Calculation

Once the largest contour is identified, we calculate the maximum difference in both horizontal and vertical directions to get the radius of the circle. The center of the ball is computed as the mean of the pixel positions at the maximum horizontal and vertical distances.

At this stage, we have the center of the ball (from the left and right cameras) and their radii.

### Stereo Vision and Depth Calculation

Now, we use **stereo vision**, similar to how human eyes work. By taking the common point (center of the ball) from both camera views, we can calculate the **disparity** between the two images. Disparity is the difference in position of the ball in both camera images. From this disparity, we can calculate the depth $$\( Z \)$$ of the ball using the following formula:

$$
Z = \frac{f \cdot B}{d}
$$

Where:
- $$\( f \)$$ is the focal length of the camera
- $$\( B \)$$ is the baseline (distance between the cameras)
- $$\( d \)$$ is the disparity (difference in position)

### Coordinate Mapping and Ball Tracking

We map the ball's 3D position to real-world coordinates and return that information. To predict the ball's future position, we estimate its trajectory using its initial position and current location. The predicted position is then used to move the hoop to that location.

However, since we cannot use `setObjectPosition` or `getObjectPosition`, we use **PID controllers** to control the actuators. We start with an initial set of values for all three actuators and adjust them using PID control to minimize the error between the predicted and actual ball positions. The error is calculated as the difference between the predicted position and the current position of the ball, measured from `/hoop_odom` (which is an odometer fixed at the center of the hoop).

### Addressing Camera Field of View (FOV) Limitations

One issue is that the camera’s **field of view (FOV)** is very limited. If the ball is out of the FOV or if the hoop has moved too far due to a previous throw, the ball might not be visible. To solve this, I proposed two potential solutions:

1. **Backtracking the hoop's position**: Similar to how the ball's position is tracked, we can backtrack the hoop’s movement after a certain time interval to bring the ball into the camera’s FOV. Accuracy isn't the main goal here; the priority is just to bring the ball back into view.
2. **Choosing optimal points on the floor**: Another idea is to choose several points on the floor where the hoop is usually placed. We can calculate the distance from each of these points to the current hoop position and select the shortest distance to travel. Once the ball is in the FOV, the ball tracking and PID control will proceed as usual.

### Further Improvements and Code Details

For more details, check the `main3.py` in the code folder. The remaining tasks include:
- Changing the reference frame: We get the ball’s coordinates relative to the camera's midpoint, but the `/hoop_odom` values are given in the world frame. 
- Tuning the PID parameters for all three actuators to ensure quick and accurate movement of the hoop.


