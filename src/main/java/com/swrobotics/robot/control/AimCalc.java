package com.swrobotics.robot.control;

import com.swrobotics.robot.config.FieldPositions;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.DriverStation;

public class AimCalc {
    private static final AimCalc instance = new AimCalc();

    private Rotation2d drivebaseAimAngle = new Rotation2d();
    private Rotation2d turretAimAngle = new Rotation2d();
    private Rotation2d hoodAngle = new Rotation2d();
    private double shooterRPS = 0;
    private double virtualDistance = 0;

    private AimCalc() {}
    public static AimCalc getInstance() { return instance; }

    public void update(Pose2d robotPose, double vx, double vy) {
        var alliance = DriverStation.getAlliance().orElse(DriverStation.Alliance.Blue);
        Translation2d hubTarget = FieldPositions.getAllianceHubPose(alliance).getTranslation();
        Translation2d robotPos = robotPose.getTranslation();

        // Lead compensation: Projectile travel time estimate
        double staticDist = robotPos.getDistance(hubTarget);
        double shotTime = staticDist * 0.012; 

        // Calculate Virtual Target (Where to aim to hit the moving/relative hub)
        double virtualX = hubTarget.getX() - (vx * shotTime);
        double virtualY = hubTarget.getY() - (vy * shotTime);
        Translation2d virtualTarget = new Translation2d(virtualX, virtualY);

        Translation2d delta = virtualTarget.minus(robotPos);
        virtualDistance = delta.getNorm();

        drivebaseAimAngle = delta.getAngle();
        turretAimAngle = drivebaseAimAngle.minus(robotPose.getRotation());

        // Regression Lookups
        shooterRPS = 40.0 + (virtualDistance * 4.5);
        double hoodDeg = 1.95 * Math.pow(virtualDistance, 2) - 9.5 * virtualDistance + 33.2;
        hoodAngle = Rotation2d.fromDegrees(hoodDeg);
    }

    public Rotation2d getDrivebaseAimAngle() { return drivebaseAimAngle; }
    public Rotation2d getTurretAimAngle() { return turretAimAngle; }
    public Rotation2d getHoodAngle() { return hoodAngle; }
    public double getShooterRPS() { return shooterRPS; }
    public double getVirtualDistance() { return virtualDistance; }
}