# CLAUDE.md

This file provides guidance to Claude Code (claude.ai/code) when working with code in this repository.

## Project Overview

FRC Team 2129 (SW Robotics) competition robot code for the 2026 "Rebuilt" game season. Java-based WPILib command-based robot using GradleRIO 2026.2.1 and Java 17.

## Build Commands

- **Build:** `./gradlew build`
- **Deploy to robot:** `./gradlew deploy`
- **Run tests:** `./gradlew test` (JUnit 5 / JUnit Platform)
- **Simulate:** `./gradlew simulateJava`
- **Clean:** `./gradlew clean`

## Architecture

### Package Structure

- `com.swrobotics.lib` - Reusable library code (not robot-specific)
  - `lib.ctre` - CTRE Phoenix 6 config helpers (`TalonFXConfigHelper`, `TalonFXSConfigHelper`) that wrap `TalonFXConfiguration` and add retry logic + live-tunable NT bindings
  - `lib.net` - NetworkTables wrapper types (`NTDouble`, `NTBoolean`, `NTEntry<T>`) for tunable values published via NT
  - `lib.field` - Field dimensions and alliance symmetry helpers
  - `lib.utils` - Math utilities, polynomial regression, debug graphics
- `com.swrobotics.robot` - Robot application code
  - `robot.config` - Constants, IOAllocation (CAN IDs), FieldPositions
  - `robot.control` - ControlBoard (button bindings), AimCalc (shooting calculations), DriveAccelFilter
  - `robot.commands` - DriveCommands, AutonomousCommands, RumblePatternCommands
  - `robot.subsystems` - Subsystem implementations
  - `robot.logging` - FieldView, RobotView, Telemetry

### Key Patterns

**Subsystem state machines:** Each subsystem (Intake, Indexer, Shooter, Hood, Expansion) follows a state-enum pattern. States are set via `commandSetState(State)` which returns a `Command`. The `periodic()` method reads the current state and applies the appropriate motor output.

**Tunable constants via NetworkTables:** Parameters that need runtime tuning use `NTEntry<T>` (e.g., `NTDouble`, `NTBoolean`) with `.setPersistent()`. These show up in SmartDashboard/Shuffleboard for live adjustment. Use plain `static final` constants for values that are finalized and shouldn't change at runtime.

**CTRE config helpers:** `TalonFXConfigHelper` extends `TalonFXConfiguration` and adds `apply(TalonFX...)` for retry-safe config application. It supports `addTunable(TunableConfig)` to auto-reapply configs when NT values change.

**CAN ID management:** All device IDs and CAN bus assignments are centralized in `IOAllocation`. The `CanId` class handles simulation workarounds (CTRE sim doesn't support CANivore, so IDs are offset by 32 in sim). Use `CanId.createTalonFX()` / `createTalonFXS()` / `createCANcoder()` factory methods.

### Subsystems

- **SwerveDriveSubsystem** - CTRE Phoenix 6 swerve (4 Kraken X60 FOC modules on "Gerald" CANivore). Uses `SwerveDrivetrain` with PathPlanner AutoBuilder integration. Pigeon 2 IMU ("Josh", CAN ID 13).
- **ShooterSubsystem** - Dual TalonFX flywheel (left + right, right follows left opposed). Velocity control with FOC.
- **HoodSubsystem** - Single TalonFX with 24:1 gear ratio. Position control with gravity FF. Homes on startup by driving into hard stop and zeroing. Hood angle range is stored as rotations (negative values: kMinAngleRot to kMaxAngleRot).
- **IndexerSubsystem** - 4 motors (floor, belt, shooter-feeder TalonFX + kicker TalonFXS Minion). CANrange sensor for ball detection.
- **IntakeSubsystem** - Single TalonFX, voltage control.
- **ExpansionSubsystem** - Single TalonFX with CANcoder, position control for intake extension/retraction.
- **VisionSubsystem** - Limelight cameras for AprilTag pose estimation. Feeds MegaTag 2 measurements to swerve pose estimator. Switches between MT1/MT2 based on robot speed.

### Autonomous

PathPlanner-based. Autos are stored in `src/main/deploy/pathplanner/`. Named commands registered in `RobotContainer`: "Shoot", "Intake", "Expand", "Retract". Auto selection via SmartDashboard `SendableChooser`.

### AimCalc

Singleton (`AimCalc.getInstance()`) that computes hood angle and drive aiming using `InterpolatingDoubleTreeMap` for distance-based lookup. Compensates for robot velocity using time-of-flight prediction. Updated every periodic cycle by `SwerveDriveSubsystem`.

### Controls

Two Xbox controllers: driver (port 0) and operator (port 1). Bindings are defined in `ControlBoard.configureControls()`. Driver handles driving + primary actions, operator handles manual overrides and hood/shooter nudges.

## Vendor Dependencies

- CTRE Phoenix 6 (TalonFX, TalonFXS, CANcoder, CANrange, Pigeon 2)
- CTRE Phoenix 5 (legacy support)
- PathPlannerLib (autonomous path following)
- WPILib New Commands
- JAMA (matrix math, used by PolynomialRegression)
