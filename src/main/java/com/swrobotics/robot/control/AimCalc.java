package com.swrobotics.robot.control;

import java.lang.annotation.Target;

import com.swrobotics.robot.config.Constants;
import com.swrobotics.robot.config.FieldPositions;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;

public class AimCalc {
    private static final AimCalc instance = new AimCalc();

    private Rotation2d angleToHub = new Rotation2d();
    private double distanceToHub = 0.0;
    

    // Hood angle management
    private Rotation2d autoTargetAngle = new Rotation2d();
    private Rotation2d manualAngle = new Rotation2d();
    private boolean useManual = false;

    private double speedMultiplier = 1.0;


    // Updated polynomial fit for hood tracking
    // (Assuming tuned from testing or simulation)
    private double a = 1.95;   // curvature term
    private double b = -9.5;   // linear term
    private double c = 33.2;   // offset term

    // Shooter speed constant (RPS)
    private static final double BASE_SHOOTER_RPS = Constants.kShooterRPS.get(); //rps is (26/(2 * π * wheel_radius))/3 because 3 is the gear ratio

    private AimCalc() {}

    public static AimCalc getInstance() {
        return instance;
    }

    public void update(Pose2d estimatedPose) {
        Pose2d hubPose = FieldPositions.getAllianceHubPose(DriverStation.getAlliance().orElse(Alliance.Red));
        Translation2d vectorToHub = hubPose.getTranslation().minus(estimatedPose.getTranslation());
        angleToHub = vectorToHub.getAngle();
        distanceToHub = vectorToHub.getNorm();

        double d = Math.max(0.25, Math.min(6.0, distanceToHub));
        double targetAngleDeg = a * d * d + b * d + c;
        autoTargetAngle = Rotation2d.fromDegrees(targetAngleDeg);
    }


    public Rotation2d getHoodAngle() {
        return useManual ? manualAngle : autoTargetAngle;
    }

    public void setManualHoodAngle(Rotation2d angle) {
        manualAngle = angle;
        useManual = true;
    }

    public void disableManualOverride() {
        useManual = false;
    }

    public boolean isManualOverride() {
        return useManual;
    }

    public double getShooterRPS() {
        return Constants.kShooterRPS.get() * speedMultiplier;
    }

    public Rotation2d getDrivebaseAimAngle() {
        return angleToHub;
    }

    public void setSpeedMultiplier(double multiplier) {
        speedMultiplier = Math.max(0.0, Math.min(1.0, multiplier));
    }

    public void setHoodPolynomial(double a, double b, double c) {
        this.a = a;
        this.b = b;
        this.c = c;
    }
}
