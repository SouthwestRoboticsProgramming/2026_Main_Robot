package com.swrobotics.robot.control;

import com.swrobotics.lib.utils.MathUtil;
import com.swrobotics.robot.config.Constants;
import com.swrobotics.robot.config.FieldPositions;
import edu.wpi.first.math.geometry.*;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.DriverStation;

public class AimCalc {

    private static final double kHubHeightMetersOffset = 0.43;
    private static final double kMinAngleDeg = 23.0;
    private static final double kMaxAngleDeg = 48.0;

    // Trajectory Constants 
    private static final double kShooterHeightMeters = Units.inchesToMeters(20); 
    private static final double kHubHeightMeters = 1.257;     
    private static final double kTargetDepthOffset = 0.25;  
    private static final double kWheelDiameterMeters = Units.inchesToMeters(4.0); 
    private static final double kShooterEfficiency = 0.85; 
    
    // Max achievable wheel RPS (assuming 1:3 step-up from Kraken/Falcon)
    private static final double kMaxWheelRPS = 150.0; 

    private static final AimCalc instance = new AimCalc();
    public static AimCalc getInstance() { return instance; }

    private Rotation2d driveAim = new Rotation2d();
    private double lastDist = 0.0;
    private double lastVirtualDist = 0.0;

    // Dynamically calculated optimal states
    private Rotation2d targetHoodAngle = Rotation2d.fromDegrees(kMaxAngleDeg);
    private double targetWheelRPS = 0.0;

    private AimCalc() {}

    private void calculateOptimalTrajectory(double distance) {
        double x = distance + kTargetDepthOffset;
        double y = kHubHeightMeters - kShooterHeightMeters + kHubHeightMetersOffset; 
        double g = 9.81;

        // 1. Start by attempting the shot at maximum possible velocity to minimize Time of Flight
        double vWheelMax = kMaxWheelRPS * Math.PI * kWheelDiameterMeters;
        double vBallMax = (vWheelMax / 2.0) * kShooterEfficiency;
        
        // Solve the ballistic trajectory quadratic for tan(theta)
        double a = (g * x * x) / (2 * vBallMax * vBallMax);
        double b = -x;
        double c = y + a;

        double discriminant = (b * b) - (4 * a * c);
        double launchAngleRad;
        double requiredVBall = vBallMax;

        if (discriminant >= 0) {
            // Pick the smaller tan(theta) root for the lowest/fastest arc
            double u = (-b - Math.sqrt(discriminant)) / (2 * a);
            launchAngleRad = Math.atan(u);
        } else {
            // Fallback if max velocity somehow can't reach (out of range)
            launchAngleRad = Math.toRadians(45.0); 
        }

        // 2. Map theoretical launch angle to physical hood angle
        // Lower launch angle = higher physical hood angle
        double hoodAngleDeg = 90.0 - Math.toDegrees(launchAngleRad);

        // 3. Clamp and throttle RPS if angle exceeds mechanical limits
        if (hoodAngleDeg > kMaxAngleDeg) {
            hoodAngleDeg = kMaxAngleDeg;
            double clampedLaunchRad = Math.toRadians(90.0 - kMaxAngleDeg);
            
            // Recalculate the lower RPM needed since we are forced to shoot at a steeper arc
            double vBallSq = (g * x * x) / (2 * Math.pow(Math.cos(clampedLaunchRad), 2) * (x * Math.tan(clampedLaunchRad) - y));
            if (vBallSq > 0) requiredVBall = Math.sqrt(vBallSq);

        } else if (hoodAngleDeg < kMinAngleDeg) {
            hoodAngleDeg = kMinAngleDeg;
            double clampedLaunchRad = Math.toRadians(90.0 - kMinAngleDeg);
            
            double vBallSq = (g * x * x) / (2 * Math.pow(Math.cos(clampedLaunchRad), 2) * (x * Math.tan(clampedLaunchRad) - y));
            if (vBallSq > 0) requiredVBall = Math.sqrt(vBallSq);
        }

        // Convert the required ball velocity back to Wheel RPS
        double requiredVWheel = (requiredVBall / kShooterEfficiency) * 2.0;
        this.targetWheelRPS = MathUtil.clamp(requiredVWheel / (Math.PI * kWheelDiameterMeters), 0, kMaxWheelRPS);
        this.targetHoodAngle = Rotation2d.fromDegrees(MathUtil.clamp(hoodAngleDeg, kMinAngleDeg, kMaxAngleDeg));
    }
    
    public void update(Pose2d robotPose, ChassisSpeeds fieldSpeeds) {
        Translation2d hub = FieldPositions.getAllianceHubPose(
            DriverStation.getAlliance().orElse(DriverStation.Alliance.Blue)
        ).getTranslation();
        
        Translation2d shooterPos = robotPose.getTranslation().plus(
            new Translation2d(Constants.kShooterOffsetX, Constants.kShooterOffsetY)
                .rotateBy(robotPose.getRotation())
        );

        lastDist = shooterPos.getDistance(hub);
        
        // Iterative approximation for moving target ToF
        double estimatedToF = lastDist / 18.0; 
        Translation2d virtualTarget = hub.minus(new Translation2d(
            fieldSpeeds.vxMetersPerSecond * estimatedToF,
            fieldSpeeds.vyMetersPerSecond * estimatedToF
        ));
        
        lastVirtualDist = virtualTarget.getDistance(shooterPos);
        driveAim = virtualTarget.minus(shooterPos).getAngle();

        // Calculate synchronized hood and RPS
        calculateOptimalTrajectory(lastVirtualDist);
    }

    public Rotation2d getHoodAngle(boolean passing) {
        return passing ? Rotation2d.fromDegrees(kMaxAngleDeg) : targetHoodAngle;
    }

    public double getShooterRPS(boolean passing) {
        return passing ? targetWheelRPS + 10.0 : targetWheelRPS;
    }

    public Rotation2d getDrivebaseAimAngle() { return driveAim; }
    public double getLastVirtualDistance() { return lastDist; }
}