# ICP Scan-to-Scan Localization Validation

## Overview
This repository contains the experimental validation of scan-to-scan localization using the Iterative Closest Point (ICP) algorithm with 2D LiDAR data.

The objective of this work is to evaluate the accuracy and repeatability of pure ICP-based motion estimation in an indoor corridor environment.

At this stage, localization is performed exclusively using LiDAR scan matching, without wheel encoders, motion models, or sensor fusion.

---

## Scope of This Work
<p align="center">
  <img src="images/localization_architecture.png" width="450">
</p>

<p align="center">
  <em>Figure: LiDAR–Encoder based localization architecture</em>
</p>
``

### Included
- 2D LiDAR scan-to-scan ICP
- Relative motion estimation between consecutive scans
- Accumulated displacement estimation
- Quantitative accuracy evaluation
- RMSE-based performance analysis

### Not Included
- Wheel encoder odometry  
- Kalman Filter / EKF  
- Sensor fusion  

This repository represents a **baseline evaluation** before introducing additional sensors or filtering techniques.

---

## Experimental Setup

- **Environment:** Indoor corridor  
- **Path length:** 5 meters (measured using a measuring tape)  
- **Motion:** Straight-line traversal  
- **Number of trials:** 5  
- **Ground truth:** Physical measurement of distance  

The experiment was repeated multiple times to ensure consistency and reliability of the results.

---

## Localization Method

### Scan-to-Scan ICP
- Consecutive 2D LiDAR scans are aligned using the ICP algorithm  
- Each scan alignment produces a relative displacement  
- Relative displacements are accumulated to estimate total travelled distance  
- No odometry, motion prior, or external correction is applied  

Localization accuracy is evaluated solely by comparing ICP-estimated distance with the known ground-truth distance.

---

## Error Calculation

For each trial:

$$
Error = d_{ICP} - d_{true}
$$

Where:
- $d_{true} = 5.0 \, m$
- $d_{ICP}$ = distance estimated by ICP

## Root Mean Square Error (RMSE)

\[
RMSE = \sqrt{\frac{1}{N}\sum_{i=1}^{N}(d_{ICP,i} - d_{true})^2}
\]

---

## Experimental Results

| Trial | True Distance (m) | ICP Distance (m) | Error (m) | Squared Error (m²) |
|------|------------------|------------------|----------|--------------------|
| 1    | 5.00             | 4.89             | -0.11    | 0.0121             |
| 2    | 5.00             | 4.85             | -0.15    | 0.0225             |
| 3    | 5.00             | 4.80             | -0.20    | 0.0400             |
| 4    | 5.00             | 4.71             | -0.29    | 0.0841             |
| 5    | 5.00             | 4.74             | -0.26    | 0.0676             |

- **Mean Error:** -0.045 m  
- **RMSE:** 0.2128 m  

---

## Interpretation

- ICP estimates are consistent across trials  
- All errors share the same sign → indicates **systematic underestimation**  
- Average deviation ≈ **21.3 cm (~4.26%)**  

This behavior is typical in corridor environments where **parallel walls provide weak longitudinal constraints** for scan matching.

---

## Conclusion

This validation demonstrates that:

- Scan-to-scan ICP provides **stable and repeatable motion estimates**
- Pure ICP alone exhibits a **systematic bias in straight corridors**
- Bias correction or additional sensing is required for higher accuracy  

The results establish a clear **baseline for future localization improvements**.

---

## Future Work

Planned extensions include:

- Integration of axle-mounted encoder data  
- Motion prior incorporation  
- Kalman Filter / EKF-based correction  
- Long-term trajectory evaluation  
- Bias reduction and drift mitigation  

---

## License
(Optional: Add your license here, e.g., MIT License)


## ROS Interfaces

<p align="center">
    <img src="localization.png" width="500"> 
</p>`

## Inputs

| Topic Name | Message Type | Description |
|-----------|------------|------------|
| /static_map | nav_msgs/OccupancyGrid.msg | Static hospital layout in map frame(ENU-coordinates) |
| /ackermann_drive_feedback | ackermann_msgs/AckermannDrive.msg | Robot motion feedback in base_link frame (vehicle frame): speed and steering angle for motion prediction using Ackermann kinematics. |
| /scan | sensor_msgs/LaserScan.msg | Raw LiDAR measurements use to estimate object boundaries and distance information.  |

## Outputs

| Topic Name | Message Type | Description |
|-----------|------------|------------|
| /odom | nav_msgs/Odometry.msg | Estimate relative robot pose and velocity . Used by Path Planning and Trajectory Planning. |


- `ROS 2` (Humble or later)  
- `Python` 3.10+  
kjnfidfb

`Required packages`:
- `rclpy` — ROS 2 Python client library  
- `nav_msgs` — Standard navigation message types (`Odometry`, `OccupancyGrid`)  
- `mocap_msgs` — Motion capture data messages (`RigidBodies`)  
- `sensor_msgs` — Standard sensor messages (`LaserScan`)  
- `ackermann_msgs` — Ackermann drive feedback messages for motion prediction  

### Installation
```bash
cd ~/ros2_ws/src
# Navigate to the sterilink workspace
colcon build --packages-select sterilink_localization
source install/setup.bash
```

### Running the Node
Launch the trajectory planning node:
```bash
ros2 run sterilink_localization localization_node.py
```