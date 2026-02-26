package com.swrobotics.robot.control;

import com.swrobotics.robot.config.Constants;
import com.swrobotics.robot.config.FieldPositions;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;

public class AimCalc {
    private static AimCalc instance;
    private static AimCalc instance2; // for testing purposes, to avoid conflicts with the main instance
    private Rotation2d angleToHub;
    private double distanceToHub;
    private Rotation2d angleToLob;
    private double distanceToLob;

    private double speedMultiplier = 1.0;

    public enum HoodControlMode {
        AUTO,
        MANUAL
    }

    /*
    * Calculate hood angle using distance.
    * Calculate RPS, for not just using a constant
    * Caluclate angle for the drivebase to aim
    * calculate distance to hub.
    * find points on regression parabola to get hood angle from distance.
    * Recieve updates from the drivebase as to WHERE the robot is
    */

    public static AimCalc getInstance() {
        if (instance == null) {
            instance = new AimCalc();
        }
        return instance;
    }
    public static AimCalc getInstance2() {
        if (instance2 == null) {
            instance2 = new AimCalc();
        }
        return instance2;
    }

    // used for auto aiming, takes in the robot's current pose and calculates the angle and distance to the hub
    public void update(Pose2d estimatedPose) {
        Pose2d hubPose = FieldPositions.getAllianceHubPose(DriverStation.getAlliance().orElse(Alliance.Red));
        Translation2d vectorToHub = hubPose.getTranslation().minus(estimatedPose.getTranslation());
        angleToHub = vectorToHub.getAngle();
        distanceToHub = vectorToHub.getNorm();
    }
    public void update2(Pose2d estimatedPose){
        Pose2d lobPose = FieldPositions.getAllianceLobPose(DriverStation.getAlliance().orElse(Alliance.Red));
        Translation2d vectorToLob = lobPose.getTranslation().minus(estimatedPose.getTranslation());
        angleToLob = vectorToLob.getAngle();
        distanceToLob = vectorToLob.getNorm();
    }

    public Rotation2d getHoodAngle() {
        double targetAngleDeg = 1 * Math.pow(distanceToHub, 2) + 1 * distanceToHub + 1; // TODO: Program in actual constants
        return Rotation2d.fromDegrees(targetAngleDeg);
    }

    public double getShooterRPS() {
        return Constants.kShooterRPS.get() * speedMultiplier;
    }

    public Rotation2d getDrivebaseAimAngle() {
        return angleToHub;
    }
    public Rotation2d getDrivebaseLobAngle() {
        return angleToLob;
    }

    /** Sets the percentage to adjust the shooter RPS. Based on how far the trigger is pulled */
    public void setSpeedMultiplier(double multiplier) {
        // Clamp the value between 0.0 and 1.0 to prevent unexpected behavior
        speedMultiplier = Math.max(0.0, Math.min(1.0, multiplier));
    }

    // @ Override
    // public void periodic() {
    



    // }
}
