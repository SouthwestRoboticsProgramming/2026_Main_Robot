package com.swrobotics.robot.control;

import com.swrobotics.robot.config.Constants;
import com.swrobotics.robot.config.FieldPositions;
import edu.wpi.first.math.geometry.*;
import edu.wpi.first.math.interpolation.InterpolatingDoubleTreeMap;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.DriverStation;

public class AimCalc {    
    private static final InterpolatingDoubleTreeMap hoodAngleMap = new InterpolatingDoubleTreeMap();
    private static final InterpolatingDoubleTreeMap passingHoodAngleMap = new InterpolatingDoubleTreeMap();
    private final InterpolatingDoubleTreeMap rpsMap = new InterpolatingDoubleTreeMap();
    private static final AimCalc instance = new AimCalc();
    public static AimCalc getInstance() { return instance; }

    private static final double kMinAngleDeg = 22.73;
    private static final double kMaxAngleDeg = 45.0;




    private Rotation2d driveAim = new Rotation2d();
    private double lastDist = 0.0;
    private double lastVirtualDist = 0.0; // Storing the "compensated" distance

    private AimCalc() {
        // --- INITIAL SCORING MAP ---
        hoodAngleMap.put(0.96, 10.0);
        hoodAngleMap.put(1.16, 12.0);
        hoodAngleMap.put(1.58, 14.0);
        hoodAngleMap.put(2.07, 18.5);
        hoodAngleMap.put(2.37, 22.0);
        hoodAngleMap.put(2.47, 23.0);
        hoodAngleMap.put(2.70, 24.0);
        hoodAngleMap.put(2.94, 25.0);
        hoodAngleMap.put(3.48, 27.0);
        hoodAngleMap.put(3.92, 32.0);
        hoodAngleMap.put(4.35, 34.0);
        hoodAngleMap.put(4.84, 38.0);

        rpsMap.put(0.0, 20.0);
        rpsMap.put(40.0, 20.0);

        // --- PASSING MAP ---
        passingHoodAngleMap.put(5.46, 38.0);
        passingHoodAngleMap.put(6.62, 38.0);
        passingHoodAngleMap.put(7.80, 38.0);
        passingHoodAngleMap.put(17.16, 38.0);
    }

    public void update(Pose2d robotPose, ChassisSpeeds fieldSpeeds) {
        Translation2d hub = FieldPositions.getAllianceHubPose(
            DriverStation.getAlliance().orElse(DriverStation.Alliance.Blue)
        ).getTranslation();

        // 1. Where are we now?
        Translation2d shooterPos = robotPose.getTranslation().plus(
            new Translation2d(Constants.kShooterOffsetX, Constants.kShooterOffsetY)
                .rotateBy(robotPose.getRotation())
        );

        lastDist = shooterPos.getDistance(hub);


        double tof = 0.4 + (0.1 * lastDist); 
        
        Translation2d virtualTarget = hub.minus(new Translation2d(
            fieldSpeeds.vxMetersPerSecond * tof,
            fieldSpeeds.vyMetersPerSecond * tof
        ));
        
        // The distance the hood/flywheels should actually care about
        lastVirtualDist = virtualTarget.getDistance(shooterPos);
        driveAim = virtualTarget.minus(shooterPos).getAngle();
    }

    public Rotation2d getHoodAngle(boolean passing) {
        // Use the virtual distance so we compensate for robot velocity!
        if (passing) {
            double targetDegrees = passingHoodAngleMap.get(lastVirtualDist);
            targetDegrees = Math.max(kMinAngleDeg, Math.min(kMaxAngleDeg, targetDegrees));
            return Rotation2d.fromDegrees(targetDegrees);
        } else {
            Double targetAngle = hoodAngleMap.get(lastVirtualDist);
            double targetDegrees = Math.max(kMinAngleDeg, Math.min(kMaxAngleDeg, targetAngle));
            return Rotation2d.fromDegrees(targetDegrees);
        }
    }
    public void saveCurrentShot(double dist, double ang, double rps) {
        hoodAngleMap.put(dist, ang);
        rpsMap.put(dist, rps);

        System.out.println("SAVED POINT: Dist: " + dist + " | Ang: " + ang + " | RPS: " + rps);
    }

    

    public boolean isInRange() {
        return lastVirtualDist > 0.5 && lastVirtualDist < 6.0;
    }

    public double getLastVirtualDistance() {
        return lastVirtualDist;
    }

    public Rotation2d getDrivebaseAimAngle() { return driveAim; }
    public double getLastDistance() { return lastDist; }
    public double getShooterRPS() {
        return rpsMap.get(lastVirtualDist);
    }
}