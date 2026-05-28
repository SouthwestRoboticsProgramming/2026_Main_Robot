# <img src="https://raw.githubusercontent.com/Tarik_K/Tarik_K/main/assets/purple_sparkle.gif" width="30"> Project Ultraviolet | Team 2129

<p align="center">
  <img src="https://img.shields.io/badge/Team-2129-8A2BE2?style=for-the-badge&logo=target" />
  <img src="https://img.shields.io/badge/Main_Language-Java-purple?style=for-the-badge&logo=java" />
</p>

## 💜 Overview
Welcome to the official repository for **Ultraviolet**, Team 2129's 2026 FRC robot. This codebase is built for high-speed precision, featuring advanced trajectory math, holonomic movement, and a fully automated intake-to-shooter pipeline.

### ⚙️ Robot Features
* **Swerve Drive:** 4-module independent drive using CTRE Phoenix 6 and Pigeon 2.0.
* **Smart Indexer:** Time-of-Flight (CANrange) controlled belt system for zero-jam indexing.
* **Dynamic Shooter:** Automated hood adjustment and RPS "gear shifting" based on distance.
* **arm pivot Subsystem:** Smooth vertical positioning using **Motion Magic** for consistent scoring heights.

---

## 🚀 Subsystems & Architecture

| Subsystem | Key Components | Control Strategy |
| :--- | :--- | :--- |
| **Swerve** | x60(Drive), Falcon(Turn), CANcoder | FOC Field-Oriented / PathPlanner |
| **arm pivot** | x44 | Motion Magic (Cruise: 50 rps) |
| **Indexer** | x60(rollers), x44(feeder), Falcon(belts), Minion(vertical feeder), CANrange(ball inspection) | State-based Logic (ToF Triggered) |
| **Intake** | x44 | High-torque Voltage Control |

---

## 🎯 Part choices
We leverage **Limelight** vision data fused with our internal Odometry to provide a 360-degree field awareness.
We also chose this year to use exclusively **CTRE** motors and sensors in our robot. We found the **X44 krakens** were particularly great and thatt the **Minion** was exceptionaly powerful for being so small.

> [!IMPORTANT]
> ### 🔍 Technical Deep Dive
> For the "Heavy Stuff"—including the custom physics, virtual target leading, and the math behind our shooter—please visit the **[SHOOTER_README.md](./SHOOTER_README.md)**.

## 🛠️ Calibration
To calibrate the swerve module offsets, use the **Dashboard**:
1. Align all wheels to face "Forward" (Bolts facing inside).
2. Toggle the `Drive/Modules/Calibrate` entry in NetworkTables.
3. The code will automatically calculate the difference and update the CANcoder magnet offsets.

<p align="center">
  <img src="https://capsule-render.vercel.app/api?type=waving&color=8A2BE2&height=100&section=footer" width="100%"/>
</p>