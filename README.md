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

