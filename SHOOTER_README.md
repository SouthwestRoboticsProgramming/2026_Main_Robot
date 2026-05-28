# 🎯 Shooter Aiming & Trajectory Logic

This module (`AimCalc.java`) handles the physics-based calculations required to hit the hub from any distance while the robot is stationary or in motion.

---

## 🏗️ The Math: How It Works

### 1. The Virtual Target (Moving Shots)
Because the ball takes time to reach the goal (**Time of Flight**), aiming directly at the hub while driving would cause the ball to miss. 
* The code calculates a **Virtual Target** by looking at the robot's current field velocity and the expected time the ball will be in the air.
* It "leads" the target, ensuring the ball and the hoop meet at the same coordinate in space regardless of robot speed.

### 2. Trajectory Lookup & Interpolation
Instead of a single fixed shot, we use `InterpolatingDoubleTreeMap` to provide a smooth curve of RPS and Hood angles across the entire field.
* **Low Range:** Steep angles for "Lob" shots.
* **High Range:** Flat, high-velocity shots for speed.

### 3. Obstacle Avoidance (Hub Clearance)
When in **Passing Mode**, the code doesn't just aim for a point; it checks for the **72" Hub Obstacle**.
* It projects the hub's physical footprint onto the ball's flight path.
* If a collision is predicted, the code iterates through higher hood angles to find a "clearing" trajectory that stays within our physical limits (23° - 48°).

---

## ⚙️ Physical Constants
| Variable | Value | Description |
| :--- | :--- | :--- |
| **Efficiency** | 85% | Energy transfer from wheel to ball. |
| **Wheel Diameter** | 4.0" | Physical size of the flywheels. |
| **Hood Range** | 23° - 48° | Mechanical hard-stops for the tilt. |
| **Turret Range** | ±270° | Physical soft-limits for wire management. |

---

## 🛠️ Turret "Unwinding"
The turret logic includes an automatic "wrapping" feature. If the target angle requires the turret to spin past its physical limit, it automatically calculates the shortest path in the opposite direction to "unwind" while maintaining the lock.

---

<p align="center">
  <font color="#8A2BE2"><b>SW ROBOTICS | PROJECT ULTRAVIOLET</b></font>
</p>