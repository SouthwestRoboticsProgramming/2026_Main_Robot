package com.swrobotics.robot.control;

import com.swrobotics.robot.config.FieldPositions;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.DriverStation;

public class AimCalc {
    private static final AimCalc instance = new AimCalc();
    private Rotation2d drivebaseAimAngle = new Rotation2d();
    private Rotation2d hoodAngle = new Rotation2d();
    private double shooterRPS = 0;
    private double virtualDistance = 0;

    private AimCalc() {}
    public static AimCalc getInstance() { return instance; }

    public void update(Pose2d robotPose, double vx, double vy) {
        var alliance = DriverStation.getAlliance().orElse(DriverStation.Alliance.Blue);
        Translation2d hubTarget = FieldPositions.getAllianceHubPose(alliance).getTranslation();
        Translation2d robotPos = robotPose.getTranslation();

        double staticDist = robotPos.getDistance(hubTarget);
        
        double shotTime = staticDist * 0.12; 

        double virtualX = hubTarget.getX() - (vx * shotTime);
        double virtualY = hubTarget.getY() - (vy * shotTime);
        Translation2d virtualTarget = new Translation2d(virtualX, virtualY);

        Translation2d delta = virtualTarget.minus(robotPos);
        drivebaseAimAngle = delta.getAngle();
        virtualDistance = delta.getNorm();

        double d = virtualDistance;
        shooterRPS = 40.0 + (d * 4.5); // Example linear ramp
        double hoodDeg = 1.95 * d * d - 9.5 * d + 33.2; // Example curve
        hoodAngle = Rotation2d.fromDegrees(hoodDeg);
    }

    public Rotation2d getDrivebaseAimAngle() { return drivebaseAimAngle; }
    public Rotation2d getHoodAngle() { return hoodAngle; }
    public double getShooterRPS() { return shooterRPS; }
}