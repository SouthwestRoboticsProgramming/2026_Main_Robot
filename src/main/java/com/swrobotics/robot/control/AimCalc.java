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
    private double passDist = 0.0;
    private double lastVirtualDist = 0.0;

    private AimCalc() {
       hoodAngleMap.put(0.25, 44.0);
        hoodAngleMap.put(0.50, 39.5);
        hoodAngleMap.put(0.75, 34.0);
        rpsMap.put(0.25, 40.0);
        rpsMap.put(0.50, 40.0);
        rpsMap.put(0.75, 40.0);

        // 2. Medium-Paced Longer Shots (60 RPS)
        hoodAngleMap.put(1.50, 31.5);
        hoodAngleMap.put(1.75, 29.0);
        hoodAngleMap.put(2.00, 26.5);
        hoodAngleMap.put(2.50, 23.5); 
        rpsMap.put(1.50, 60.0);
        rpsMap.put(1.75, 60.0);
        rpsMap.put(2.00, 60.0);
        rpsMap.put(2.50, 60.0);
        
        // 3. Fast-Paced Long Passing Shots (90 RPS)
        passingHoodAngleMap.put(5.00, 32.0);
        passingHoodAngleMap.put(7.50, 36.5);
        passingHoodAngleMap.put(10.00, 41.0);
        rpsMap.put(5.00, 90.0);
        rpsMap.put(7.50, 90.0);
        rpsMap.put(10.00, 90.0);
    }

    public void update(Pose2d robotPose, ChassisSpeeds fieldSpeeds) {
        Translation2d hub = FieldPositions.getAllianceHubPose(
            DriverStation.getAlliance().orElse(DriverStation.Alliance.Blue)
        ).getTranslation();
        Translation2d pass = FieldPositions.getAlliancePassPose(
            DriverStation.getAlliance().orElse(DriverStation.Alliance.Blue)
        ).getTranslation();
        // 1. Where are we now?
        Translation2d shooterPos = robotPose.getTranslation().plus(
            new Translation2d(Constants.kShooterOffsetX, Constants.kShooterOffsetY)
                .rotateBy(robotPose.getRotation())
        );

        lastDist = shooterPos.getDistance(hub);
        passDist = shooterPos.getDistance(pass);


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
            double targetDegrees = passingHoodAngleMap.get(passDist);
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
    public double getShooterRPS(boolean passing) {
        if (passing) {
            return rpsMap.get(passDist);
        }else{
        return rpsMap.get(lastVirtualDist);
    }
    }
}