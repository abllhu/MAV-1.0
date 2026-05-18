# MAV-1.0

An advanced mobile robot simulation environment focused on autonomous navigation, state estimation under uncertainty, and reactive safety control.

---

## Project Upgrades (MAV-1.0 vs Previous Version)

This version introduces major upgrades to improve robot localization, state estimation accuracy, and collision avoidance:

### 1. Probabilistic Odometry Motion Model
Standard deterministic odometry accumulates kinematic errors over time. To address this, a probabilistic motion model has been implemented using a particle filter approach:
* **Particle-Based Estimation:** The model utilizes a set of particles to represent the probability distribution of the robot's possible poses, predicting the exact state at any given time step.
* **Error Propagation Mitigation:** By modeling motion uncertainties stochastically, the system handles sensor noise and cumulative errors, resulting in highly accurate robot localization.

**Mathematical Foundation & Algorithm:**

The algorithm estimates the new position by computing motion parameters from odometry, adding zero-mean noise, and updating the pose based on the following transitions:

1. **Compute Odometry Motion Parameters:**

$$\delta_{rot1} = \operatorname{atan2}(\bar{y}' - \bar{y}, \bar{x}' - \bar{x}) - \bar{\theta}$$

$$\delta_{trans} = \sqrt{(\bar{x} - \bar{x}')^2 + (\bar{y} - \bar{y}')^2}$$

$$\delta_{rot2} = \bar{\theta}' - \bar{\theta} - \delta_{rot1}$$

*(Where x-bar represents the internal odometry reading and x represents the true world coordinates).*

2. **Sample True Parameters with Noise:**

$$\hat{\delta}_{rot1} = \delta_{rot1} - \operatorname{sample}(\alpha_1 \delta_{rot1}^2 + \alpha_2 \delta_{trans}^2)$$

$$\hat{\delta}_{trans} = \delta_{trans} - \operatorname{sample}(\alpha_3 \delta_{trans}^2 + \alpha_4 \delta_{rot1}^2 + \alpha_4 \delta_{rot2}^2)$$

$$\hat{\delta}_{rot2} = \delta_{rot2} - \operatorname{sample}(\alpha_1 \delta_{rot2}^2 + \alpha_2 \delta_{trans}^2)$$

3. **Update Robot Pose:**

$$x' = x + \hat{\delta}_{trans} \cos(\theta + \hat{\delta}_{rot1})$$

$$y' = y + \hat{\delta}_{trans} \sin(\theta + \hat{\delta}_{rot1})$$

$$\theta' = \theta + \hat{\delta}_{rot1} + \hat{\delta}_{rot2}$$

![Odometry Motion Model Demo](gifs/odom_motion_model.gif)

---

### 2. Speed Monitoring and Collision Avoidance Logic
To prevent hardware and simulation collisions, a safety system has been integrated into the velocity command pipeline using LiDAR sensor data and the `twist_mux` package. The surroundings of the robot are monitored through defined safety zones:

* **Warning Zone (Yellow):** When an obstacle enters the outer threshold, the robot's linear and angular velocities are automatically decreased to ensure safe maneuvering.
* **Danger Zone (Red):** When an obstacle breaches the critical inner radius, a ROS 2 Action Node is triggered immediately to execute an emergency stop and preempt lower-priority velocity commands.

![Safety Stop Demo](gifs/safty_stop.gif)

---


