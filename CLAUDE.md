# CLAUDE.md

This file provides guidance to Claude Code (claude.ai/code) when working with code in this repository.

## Project Overview

FRC Team 2129 (Southwest Robotics) 2026 competition robot code. Java 17, WPILib 2026.2.1, command-based framework.

## Build Commands

```bash
./gradlew build              # Compile and package
./gradlew deploy             # Deploy to RoboRIO
./gradlew simulateJava       # Run robot simulation (use simulationDebug for debug mode)
./gradlew test               # Run unit tests
./gradlew clean              # Clean build artifacts
```

## Architecture

### Package Structure

- `com.swrobotics.robot` — Robot code (entry point: `Main.java` → `Robot.java` → `RobotContainer.java`)
- `com.swrobotics.robot.subsystems` — Subsystems: swerve drive, shooter, hood, intake, indexer, expansion, vision
- `com.swrobotics.robot.commands` — Autonomous commands, drive commands, characterization
- `com.swrobotics.robot.config` — `Constants.java` (tunable values), `IOAllocation.java` (CAN IDs), `FieldPositions.java`
- `com.swrobotics.robot.control` — Controller mappings (`ControlBoard.java`), aim calculations, drive filtering
- `com.swrobotics.robot.logging` — Telemetry, field visualization, robot state visualization
- `com.swrobotics.lib` — Reusable utility library (NetworkTable wrappers, CTRE config helpers, field geometry, math)

### Key Patterns

- **Subsystem state machines**: Each subsystem uses an enum `State` with a `periodic()` method that switches on current state. State is set via public methods called by commands.
- **Tunable constants**: `NTEntry<T>` (NTDouble, NTBoolean, etc.) wraps NetworkTables for runtime-adjustable parameters. Defined in `Constants.java`.
- **CTRE config helpers**: `TalonFXConfigHelper` / `TalonFXSConfigHelper` in `com.swrobotics.lib.ctre` wrap Phoenix 6 motor configuration with retry logic and apply patterns.
- **IOAllocation.CanId**: Stores device IDs with bus name; has factory methods like `.talonFX()`, `.talonFXS()`, `.cancoder()`, `.canrange()` to create devices directly.
- **Vision fusion**: `VisionSubsystem` manages 3 Limelight cameras feeding MegaTag2 pose estimates into swerve drive odometry, gated by robot speed.

### Hardware

- **Drive**: CTRE swerve (4x Kraken X60 FOC modules + Pigeon2 IMU) on "Gerald" CANivore bus
- **Shooter**: Dual Kraken motors (leader/follower), hood with position control
- **Intake/Indexer**: 5 motors total, CANrange ball sensor
- **Vision**: 3x Limelight cameras (front, right, back)

### Autonomous

PathPlanner-based. Auto routines in `src/main/deploy/pathplanner/autos/`, paths in `paths/`. Named commands registered in `AutonomousCommands.java`. Auto mode selected via SmartDashboard chooser.

### Dependencies

- WPILib 2026.2.1 (command-based framework)
- CTRE Phoenix 6 (motor control, swerve)
- PathPlanner (autonomous trajectories)
- Limelight (vision/AprilTag localization)
- JAMA 1.0.3 (matrix math)
