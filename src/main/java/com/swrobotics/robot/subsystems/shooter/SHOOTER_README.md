# 🎯 Shooter Aiming & Trajectory Logic

This module handles the physics-based calculations required to hit the hub from any distance while the robot is stationary or in motion.

---

## 🏗️ The Math: How It Works

Imagine trying to throw a basketball into a hoop while sprinting; that’s exactly the physics this code is solving! To hit a target consistently, the code balances three main pillars: **Motion Compensation**, **Discrete Gear Shifting**, and **Parabolic Trajectories**.



### 1. The Virtual Target (Moving Shots)
Because the ball takes time to reach the goal (**Time of Flight**), aiming directly at the hub while driving would cause the ball to miss. 
* The code calculates a **Virtual Target** by looking at the robot's current field velocity and the expected time the ball will be in the air.
* It "leads" the target, much like a quarterback throwing to a moving receiver, ensuring the ball and the hoop meet at the same coordinate in space.

### 2. "Gear Shifting" Logic
Instead of a simple lookup table, the code treats Shooter Wheel speeds (RPS) like gears in a car to prioritize hood movement:

1. **Test a Gear:** It starts at a low RPS and calculates the required launch angle needed to hit the target using the trajectory formula:
   $$y = x \tan(\theta) - \frac{gx^2}{2v^2\cos^2(\theta)}$$
2. **Check the Hood:** Our hood is physically limited between **23° and 48°**. The code checks if the calculated angle is "legal" for that gear.
3. **Shift Up:** If the robot is too far for the current RPS to reach the goal (or if the angle exceeds 48°), the code "shifts" to a higher RPS gear and resets the hood to a lower angle.
4. **The Result:** This creates a "sawtooth" behavior. As you drive away, the hood tilts up until it hits the limit; then, the RPS jumps up, and the hood resets low to start the climb again.

---

## ⚙️ Physical Constants
| Constant | Value | Description |
| :--- | :--- | :--- |
| **Efficiency** | 85% | Energy transfer from wheel to ball. |
| **Wheel Diameter** | 4.0" | Physical size of the shooter flywheels. |
| **Hood Range** | 23° - 48° | The mechanical hard-stops of the tilting hood. |
| **Target Depth** | 0.25m | Aiming for the center of the hub, not the rim. |

---

## 🛠️ Implementation Notes
The `AimCalc.java` class uses an iterative search to find the lowest possible RPS gear that satisfies the hood constraints. This ensures we use the minimum energy required for every shot, which improves recovery time and battery life.

> [!TIP]
> To update the diagram below, replace `shooter_diagram.png` in your `/images` folder.

![Shooting Physics Diagram](images/shooter_diagram.png)