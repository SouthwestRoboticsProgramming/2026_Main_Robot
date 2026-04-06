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
 
    private static final double kShooterHeightMeters = Units.inchesToMeters(20); 
    private static final double kHubHeightMeters = 1.257;      
    private static final double kTargetDepthOffset = 0.25;  
    private static final double kWheelDiameterMeters = Units.inchesToMeters(4.0); 
    private static final double kShooterEfficiency = 0.85; 
    
    private static final double kMaxWheelRPS = 150.0;
    private static final double kRPSStep = 5.0; 

    private static final AimCalc instance = new AimCalc();
    public static AimCalc getInstance() { return instance; }

    private Rotation2d driveAim = new Rotation2d();
    private double lastDist = 0.0;
    private double lastVirtualDist = 0.0;
    private double lastToF = 0.0; 

    private Rotation2d targetHoodAngle = Rotation2d.fromDegrees(kMaxAngleDeg);
    private double targetWheelRPS = 0.0;

    private AimCalc() {}

    private void calculateOptimalTrajectory(double distance) {
        double x = distance + kTargetDepthOffset;
        double y = kHubHeightMeters - kShooterHeightMeters + kHubHeightMetersOffset; 
        double g = 9.81;

        boolean foundGear = false;

        for (double rps = 30.0; rps <= kMaxWheelRPS; rps += kRPSStep) {
            double vWheel = rps * Math.PI * kWheelDiameterMeters;
            double vBall = (vWheel / 2.0) * kShooterEfficiency;

            double k = (g * x * x) / (2 * vBall * vBall);
            double discriminant = (x * x) - (4 * k * (y + k));

            if (discriminant >= 0) {
                double tanTheta = (x + Math.sqrt(discriminant)) / (2 * k);
                double thetaRad = Math.atan(tanTheta);
                double hoodDeg = 90.0 - Math.toDegrees(thetaRad);

                // If this gear produces a valid hood angle, we lock it in and stop searching.
                if (hoodDeg >= kMinAngleDeg && hoodDeg <= kMaxAngleDeg) {
                    this.targetWheelRPS = rps;
                    this.targetHoodAngle = Rotation2d.fromDegrees(hoodDeg);
                    updateToF(x, rps, hoodDeg);
                    foundGear = true;
                    break;
                }
            }
        }

        // Fallback: If no gear perfectly matched (either too close or too far)
        if (!foundGear) {
            double rpsForMinAngle = calculateExactRPS(x, y, kMinAngleDeg);
            double rpsForMaxAngle = calculateExactRPS(x, y, kMaxAngleDeg);

            // If a very low RPS works for the minimum angle, we are extremely close to the hub.
            if (rpsForMinAngle > 0 && rpsForMinAngle < 80.0) {
                this.targetWheelRPS = MathUtil.clamp(rpsForMinAngle, 0, kMaxWheelRPS);
                this.targetHoodAngle = Rotation2d.fromDegrees(kMinAngleDeg);
                updateToF(x, this.targetWheelRPS, kMinAngleDeg);
            } else {
                // Otherwise, we are too far. Default to max angle and max out the RPS.
                this.targetWheelRPS = rpsForMaxAngle > 0 ? MathUtil.clamp(rpsForMaxAngle, 0, kMaxWheelRPS) : kMaxWheelRPS;
                this.targetHoodAngle = Rotation2d.fromDegrees(kMaxAngleDeg);
                updateToF(x, this.targetWheelRPS, kMaxAngleDeg);
            }
        }
    }

    private double calculateExactRPS(double x, double y, double hoodAngleDeg) {
        double launchRad = Math.toRadians(90.0 - hoodAngleDeg);
        double cosSq = Math.pow(Math.cos(launchRad), 2);
        double tan = Math.tan(launchRad);
        
        double vBallSq = (9.81 * x * x) / (2 * cosSq * (x * tan - y));
        if (vBallSq <= 0) return -1;
        
        double requiredVBall = Math.sqrt(vBallSq);
        double requiredVWheel = (requiredVBall / kShooterEfficiency) * 2.0;
        return requiredVWheel / (Math.PI * kWheelDiameterMeters);
    }

    private void updateToF(double x, double rps, double hoodAngleDeg) {
        double vWheel = rps * Math.PI * kWheelDiameterMeters;
        double vBall = (vWheel / 2.0) * kShooterEfficiency;
        double launchRad = Math.toRadians(90.0 - hoodAngleDeg);
        
        if (vBall > 0) {
            this.lastToF = x / (vBall * Math.cos(launchRad));
        } else {
            this.lastToF = x / 18.0; 
        }
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
        
        double estimatedToF = (lastToF > 0) ? lastToF : (lastDist / 18.0); 
        Translation2d virtualTarget = hub.minus(new Translation2d(
            fieldSpeeds.vxMetersPerSecond * estimatedToF,
            fieldSpeeds.vyMetersPerSecond * estimatedToF
        ));
        
        lastVirtualDist = virtualTarget.getDistance(shooterPos);
        driveAim = virtualTarget.minus(shooterPos).getAngle();

        calculateOptimalTrajectory(lastVirtualDist);
    }

    public Rotation2d getHoodAngle(boolean passing) {
        return passing ? Rotation2d.fromDegrees(kMaxAngleDeg) : targetHoodAngle;
    }

    public double getShooterRPS(boolean passing) {
        return passing ? targetWheelRPS + 10.0 : targetWheelRPS;
    }

    public Rotation2d getDrivebaseAimAngle() { return driveAim; }
    
    public double getLastVirtualDistance() { return lastVirtualDist; } 
}