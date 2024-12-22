# Robotrix-2k25

## Useful Resources
| Resource                               | Link                                                                 |
|-----------------------------------------------|----------------------------------------------------------------------|
| **Unstop Registration**                       | [Robotrix-2025 on Unstop](https://unstop.com/hackathons/robotrix-2025-diodexcelerate-2025-ieee-nitk-1265503) |
| **OpenCV Learning Resources**                | [OpenCV Session Materials](https://drive.google.com/drive/folders/1I3onFJ9BmKhO2ySJHbKbqaqb6ShACZez)       |
| **Getting Started with CoppeliaSim**         | [CoppeliaSim Guide](https://docs.google.com/document/d/16T91vF_IcL98n5h6RFddoQVI-fivpbWYlDm_yUQMZ3E/edit?tab=t.0) |
| **Robotrix 2025 Problem Statement**          | [Problem Statement Document](https://docs.google.com/document/d/1vff6XrevOeJcRv-ABr8Lx3oOjOZTUBogC9AZYqRhWjU/edit?tab=t.0#heading=h.bn1tbf9oj3ut) |
| **CoppeliaSim Technical Session**            | [Session Video](https://drive.google.com/file/d/1xbQdN7m91ATBxBm8FzqdT0GL89Kq6rRG/view)                    |
| **How to Setup Stereo Vision and Depth Estimation** | [Video Link](https://www.youtube.com/watch?v=c0f0s_o2qho)           |

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

## Ball Detection and Tracking Approach

Since the `getObjectPosition()` function is not available for the `/ball`, its 3D position must be estimated using stereo cameras. The ball's unique **orange color** with RGB values $$(1.00, 0.411, 0.08)$$ is used for detection. This RGB value is converted to the **HSV color space**, as HSV makes it easier to apply color-based segmentation under varying lighting conditions.

### Color Masking and Preprocessing

To detect the ball in the camera images:

1. **HSV Conversion:** Convert the captured RGB images from the cameras to HSV using the `cv2.cvtColor()` function.
2. **Color Mask:** A mask is created by defining a range of HSV values around the ball's color. This ensures flexibility under different lighting conditions. For instance:

   $$
   \\ \text{lower\_hsv} = [h_\text{min}, s_\text{min}, v_\text{min}] \\
   \text{upper\_hsv} = [h_\text{max}, s_\text{max}, v_\text{max}]
   $$
   
   The range is tuned experimentally to detect only the ball.
4. **Preprocessing Steps:**
   - **Gaussian Blur:** Smooth the image to reduce noise and improve contour detection.
     ```python
     blurred = cv2.GaussianBlur(mask, (5, 5), 0)
     ```
   - **Morphological Closing:** Apply `cv2.morphologyEx()` with the `cv2.MORPH_CLOSE` operation to close small gaps in the mask and create a solid region for the ball.

### Contour Detection

After preprocessing:

- Extract **contours** using `cv2.findContours()`. Contours represent the boundaries of detected objects in the binary mask.
- Select the largest contour by area, assuming it corresponds to the ball. This is done using:
  ```python
  max_contour = max(contours, key=cv2.contourArea)
  ```
- Approximate the ball as a circle using `cv2.minEnclosingCircle()`, which provides the ball's **center** (in pixel coordinates) and its **radius**.

---

## Stereo Vision and Depth Calculation

To estimate the ball's 3D position, stereo cameras are used. Stereo vision leverages the disparity between the same object's positions in the left and right camera images to calculate depth.

### Disparity Calculation

The disparity ($$d$$) is the horizontal difference in the ball's position between the two images:

$$
d = x_l - x_r
$$

Where:
- $$x_l$$ and $$x_r$$ are the horizontal pixel coordinates of the ball's center in the left and right camera images.

### Depth Estimation

The depth ($$Z$$) of the ball is calculated using:

$$
Z = \frac{f \cdot B}{d}
$$

Where:
- $$f$$ = Focal length of the cameras (in pixels).
- $$B$$ = Baseline distance between the two cameras (700 mm).
- $$d$$ = Disparity (in pixels).

### Real-World 3D Coordinates

Using the depth ($$Z$$), the ball's real-world coordinates $$X$$ and $$Y$$ are calculated as:

$$
X = \frac{(x - c_x) \cdot Z}{f}
$$

$$
Y = \frac{(y - c_y) \cdot Z}{f}
$$

Where:
- $$x, y$$ = Ball's center in the image (pixels).
- $$c_x, c_y$$ = Camera's optical center (principal point).
- $$f$$ = Focal length.

---

## Projectile Trajectory Estimation and Position Prediction

To move the hoop accurately, the ball's trajectory must be predicted based on its current motion. This involves:

### Projectile Motion Equations

The ball's motion is governed by the physics of projectile motion:

$$
x(t) = v_x \cdot t + x_0
$$

$$
y(t) = v_y \cdot t + y_0
$$

$$
z(t) = v_z \cdot t + z_0 - \frac{1}{2} g t^2
$$

Where:
- $$v_x, v_y, v_z$$ = Initial velocities in the $$x, y, z$$ directions.
- $$x_0, y_0, z_0$$ = Initial position of the ball.
- $$g$$ = Gravitational acceleration ($$9.81 \ \text{m/s}^2$$).
- $$t$$ = Time.

### Velocity Estimation

The ball's velocity components are estimated from its positions in consecutive frames:

$$
v_x = \frac{x_2 - x_1}{\Delta t}, \quad v_y = \frac{y_2 - y_1}{\Delta t}, \quad v_z = \frac{z_2 - z_1}{\Delta t}
$$

Where $$\Delta t$$ is the time between two frames.

### Future Position Prediction

Using the projectile motion equations, the ball's position at a future time $$t$$ is predicted. This predicted position is where the hoop must be moved.

---

## PID Controllers for Hoop Movement

The hoop is controlled using **PID controllers** for each axis (X, Y, Z). These controllers minimize the error between the desired (predicted) position and the hoop's current position.

### PID Control Formula

$$
u(t) = K_p \cdot e(t) + K_i \cdot \int e(t) \, dt + K_d \cdot \frac{de(t)}{dt}
$$

Where:
- $$e(t)$$ = Error at time $$t$$ (difference between target and current position).
- $$K_p, K_i, K_d$$ = Proportional, Integral, and Derivative gains.

### Hoop Movement

The error for each axis is calculated as:

$$
e_x = x_\text{predicted} - x_\text{hoop}, \quad e_y = y_\text{predicted} - y_\text{hoop}, \quad e_z = z_\text{predicted} - z_\text{hoop}
$$

The actuators (`actuator_x`, `actuator_y`, `actuator_z`) are controlled using the PID outputs to minimize these errors.

### PID Tuning

The PID parameters ($$K_p, K_i, K_d$$) are tuned experimentally to balance:
1. **Responsiveness**: Ensuring the hoop quickly moves to the predicted position.
2. **Stability**: Avoiding oscillations or overshooting.

---

## Addressing Camera Field of View (FOV) Limitations

If the ball moves out of the cameras' FOV:

1. **Reactive Hoop Movement:** Move the hoop toward the last known position of the ball to bring it back into view.
2. **Predefined Waypoints:** Use a set of predefined positions to reset the hoop's position near the center, ensuring the ball remains in the cameras' FOV.

---

## Functions Used

1. **`getVisionSensorImage()`**: Captures images from the stereo cameras.
2. **`cv2.GaussianBlur()`**: Applies Gaussian blur for noise reduction.
3. **`cv2.inRange()`**: Creates a binary mask based on HSV color thresholds.
4. **`cv2.morphologyEx()`**: Performs morphological transformations like closing.
5. **`cv2.findContours()`**: Detects contours in binary images.
6. **`cv2.minEnclosingCircle()`**: Fits a circle around the largest contour.
7. **`cv2.calcOpticalFlowPyrLK()`** (Optional): Estimates ball motion between frames.

---

### To-Do:
- **PID Optimization:** Test and adjust PID parameters to balance responsiveness and stability
- **Edge Cases:** Handle scenarios where the ball is partially occluded or moves out of the cameras' FOV





