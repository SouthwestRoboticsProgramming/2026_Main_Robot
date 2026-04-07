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
    private static final double kMinAngleDeg = 23.0; // Steepest shot (lob)
    private static final double kMaxAngleDeg = 48.0; // Flattest shot (direct)

    private static final double kShooterHeightMeters = Units.inchesToMeters(20); 
    private static final double kHubTargetHeightMeters = 1.257;      
    private static final double kTargetDepthOffset = 0.25;  
    private static final double kWheelDiameterMeters = Units.inchesToMeters(4.0); 
    private static final double kShooterEfficiency = 0.85; 
    
    private static final double kMaxWheelRPS = 150.0;
    private static final double kRPSStep = 5.0; 

    // Passing Constraints
    private static final double kHubObstacleHeightMeters = Units.inchesToMeters(72.0);
    private static final double kHubObstacleRadiusMeters = Units.inchesToMeters(41.0 / 2.0);
    private static final double kPassTargetHeightMeters = Units.inchesToMeters(10.0); // Height of receiving robot
    
    // TUNE THESE: The coordinates of the corner in front of the HP station
    private static final Translation2d kBlueHumanPlayerCorner = new Translation2d(15.5, 0.5);
    private static final Translation2d kRedHumanPlayerCorner = new Translation2d(1.0, 7.5);

    private static final AimCalc instance = new AimCalc();
    public static AimCalc getInstance() { return instance; }

    private Rotation2d driveAim = new Rotation2d();
    private double lastDist = 0.0;
    private double lastVirtualDist = 0.0;
    private double lastToF = 0.0; 

    private Rotation2d targetHoodAngle = Rotation2d.fromDegrees(kMaxAngleDeg);
    private double targetWheelRPS = 0.0;
    private boolean passingMode = false;

    private AimCalc() {}

    public void setPassingMode(boolean isPassing) {
        this.passingMode = isPassing;
    }

    private void calculateOptimalTrajectory(double distance) {
        double x = distance + kTargetDepthOffset;
        double y = kHubTargetHeightMeters - kShooterHeightMeters + kHubHeightMetersOffset; 
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

                if (hoodDeg >= kMinAngleDeg && hoodDeg <= kMaxAngleDeg) {
                    this.targetWheelRPS = rps;
                    this.targetHoodAngle = Rotation2d.fromDegrees(hoodDeg);
                    updateToF(x, rps, hoodDeg);
                    foundGear = true;
                    break;
                }
            }
        }

        if (!foundGear) {
            double rpsForMinAngle = calculateExactRPS(x, y, kMinAngleDeg);
            double rpsForMaxAngle = calculateExactRPS(x, y, kMaxAngleDeg);

            if (rpsForMinAngle > 0 && rpsForMinAngle < 80.0) {
                this.targetWheelRPS = MathUtil.clamp(rpsForMinAngle, 0, kMaxWheelRPS);
                this.targetHoodAngle = Rotation2d.fromDegrees(kMinAngleDeg);
                updateToF(x, this.targetWheelRPS, kMinAngleDeg);
            } else {
                this.targetWheelRPS = rpsForMaxAngle > 0 ? MathUtil.clamp(rpsForMaxAngle, 0, kMaxWheelRPS) : kMaxWheelRPS;
                this.targetHoodAngle = Rotation2d.fromDegrees(kMaxAngleDeg);
                updateToF(x, this.targetWheelRPS, kMaxAngleDeg);
            }
        }
    }

    private void calculatePassingTrajectory(Translation2d shooterPos, Translation2d targetPos, Translation2d hubPos) {
        double xTarget = shooterPos.getDistance(targetPos);
        double yTarget = kPassTargetHeightMeters - kShooterHeightMeters; 

        Translation2d shootToTarget = targetPos.minus(shooterPos);
        Translation2d shootToHub = hubPos.minus(shooterPos);
        
        // Project the hub onto the passing path to see if it's in the way
        double angleDiffRad = shootToHub.getAngle().minus(shootToTarget.getAngle()).getRadians();
        double xHubCenter = shootToHub.getNorm() * Math.cos(angleDiffRad);
        double yHubPerp = Math.abs(shootToHub.getNorm() * Math.sin(angleDiffRad));
        
        // Hub is in the way if the perpendicular distance is less than the hub radius + ball clearance (0.2m)
        boolean pathIntersectsHub = yHubPerp < (kHubObstacleRadiusMeters + 0.2);
        
        // Check front lip of hub for clearance
        double xObstacle = xHubCenter - kHubObstacleRadiusMeters; 
        double yObstacle = kHubObstacleHeightMeters - kShooterHeightMeters + 0.15; // + 15cm safety margin
        
        boolean foundPass = false;

        // Iterate from flattest (Max Deg) to steepest lob (Min Deg)
        // Note: launchRad = 90 - hoodDeg, so lower hoodDeg = higher launch arc
        for (double hoodDeg = kMaxAngleDeg; hoodDeg >= kMinAngleDeg; hoodDeg -= 1.0) {
            double launchRad = Math.toRadians(90.0 - hoodDeg);
            double cosSq = Math.pow(Math.cos(launchRad), 2);
            double tan = Math.tan(launchRad);
            
            // Calculate required V-Ball to hit the HP Corner
            double vBallSq = (9.81 * xTarget * xTarget) / (2 * cosSq * (xTarget * tan - yTarget));
            if (vBallSq <= 0) continue;
            
            double requiredVBall = Math.sqrt(vBallSq);
            
            // Check 72" clearance if hub is in the way
            boolean clearsObstacle = true;
            if (pathIntersectsHub && xObstacle > 0.5 && xObstacle < xTarget) {
                double heightAtObstacle = xObstacle * tan - (9.81 * xObstacle * xObstacle) / (2 * vBallSq * cosSq);
                if (heightAtObstacle < yObstacle) {
                    clearsObstacle = false;
                }
            }
            
            if (clearsObstacle) {
                double requiredVWheel = (requiredVBall / kShooterEfficiency) * 2.0;
                double rps = requiredVWheel / (Math.PI * kWheelDiameterMeters);
                
                if (rps <= kMaxWheelRPS) {
                    this.targetWheelRPS = rps;
                    this.targetHoodAngle = Rotation2d.fromDegrees(hoodDeg);
                    updateToF(xTarget, rps, hoodDeg);
                    foundPass = true;
                    break;
                }
            }
        }
        
        // Fallback if no safe path clears the 72" height
        if (!foundPass) {
            this.targetWheelRPS = kMaxWheelRPS;
            this.targetHoodAngle = Rotation2d.fromDegrees(kMinAngleDeg); // Max lob as panic
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
        DriverStation.Alliance alliance = DriverStation.getAlliance().orElse(DriverStation.Alliance.Blue);
        Translation2d hub = FieldPositions.getAllianceHubPose(alliance).getTranslation();
        
        Translation2d target = passingMode ? 
            (alliance == DriverStation.Alliance.Blue ? kBlueHumanPlayerCorner : kRedHumanPlayerCorner) 
            : hub;
        
        Translation2d shooterPos = robotPose.getTranslation().plus(
            new Translation2d(Constants.kShooterOffsetX, Constants.kShooterOffsetY)
                .rotateBy(robotPose.getRotation())
        );

        lastDist = shooterPos.getDistance(target);
        
        double estimatedToF = (lastToF > 0) ? lastToF : (lastDist / 18.0); 
        Translation2d virtualTarget = target.minus(new Translation2d(
            fieldSpeeds.vxMetersPerSecond * estimatedToF,
            fieldSpeeds.vyMetersPerSecond * estimatedToF
        ));
        
        lastVirtualDist = virtualTarget.getDistance(shooterPos);
        driveAim = virtualTarget.minus(shooterPos).getAngle();

        if (passingMode) {
            calculatePassingTrajectory(shooterPos, virtualTarget, hub);
        } else {
            calculateOptimalTrajectory(lastVirtualDist);
        }
    }

    public Rotation2d getHoodAngle() {
        return targetHoodAngle;
    }

    public double getShooterRPS() {
        return targetWheelRPS;
    }

    //     public double getShooterRPS(boolean passing) {
    //     return passing ? targetWheelRPS + 10.0 : targetWheelRPS;
    // }

    public Rotation2d getDrivebaseAimAngle() { return driveAim; }
    
    public double getLastVirtualDistance() { return lastVirtualDist; } 
}