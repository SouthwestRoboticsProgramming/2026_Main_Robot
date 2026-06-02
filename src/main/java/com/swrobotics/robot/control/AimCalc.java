package com.swrobotics.robot.control;

import com.swrobotics.lib.utils.MathUtil;
import com.swrobotics.robot.config.Constants;
import com.swrobotics.robot.config.FieldPositions;
import edu.wpi.first.math.geometry.*;
import edu.wpi.first.math.interpolation.InterpolatingDoubleTreeMap;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.DriverStation;

public class AimCalc {
    private static final double kFixedExitAngleDeg = 34.0; 

    private static final double kShooterHeightMeters = Units.inchesToMeters(20); 
    private static final double kTargetDepthOffset = 0;  
    private static final double kWheelDiameterMeters = Units.inchesToMeters(4.0); 
    private static final double kShooterEfficiency = 0.85; 
    
    private static final double kMaxWheelRPS = 150.0;
    private static final double kHubObstacleHeightMeters = Units.inchesToMeters(72.0);
    private static final double kHubObstacleRadiusMeters = Units.inchesToMeters(41.0);
    
    private static final Translation2d kBlueHumanPlayerRightCorner = new Translation2d(1.6, 1.45);
    private static final Translation2d kRedHumanPlayerRightCorner = new Translation2d(15, 1.45);

    private static final AimCalc instance = new AimCalc();
    public static AimCalc getInstance() { return instance; }

    // --- INTERPOLATING MAP ---
    // Brought back exclusively for RPS tuning
    private final InterpolatingDoubleTreeMap distanceToRPSMap = new InterpolatingDoubleTreeMap();

    private Rotation2d driveAim = new Rotation2d();
    private double lastVirtualDist = 0.0;
    private double lastToF = 0.0; 

    private double targetWheelRPS = 0.0;
    private boolean passingMode = false;

    private AimCalc() {
        // TUNE THESE: Empirical data (Distance in Meters -> Target Wheel RPS)
        // Go on the field, place the robot at these distances, find the perfect RPS, and log it here.
        distanceToRPSMap.put(1.5, 40.0);
        distanceToRPSMap.put(3.0, 55.0); 
        distanceToRPSMap.put(5.0, 75.0); 
        distanceToRPSMap.put(7.0, 95.0); 
    }

    public void setPassingMode(boolean isPassing) {
        this.passingMode = isPassing;
    }

    private void calculateOptimalTrajectory(double distance) {
        double x = distance + kTargetDepthOffset;
        
        // Lowkey the best way to handle FRC foam balls - pure lookup
        double rawRPS = distanceToRPSMap.get(x);
        
        this.targetWheelRPS = MathUtil.clamp(rawRPS, 0, kMaxWheelRPS);
        updateToF(x, this.targetWheelRPS);
    }

    private void calculatePassingTrajectory(Translation2d shooterPos, Translation2d targetPos, Translation2d hubPos) {
        double xTarget = shooterPos.getDistance(targetPos);
        double yTarget = -kShooterHeightMeters; 

        Translation2d shootToTarget = targetPos.minus(shooterPos);
        Translation2d shootToHub = hubPos.minus(shooterPos);
        
        double angleDiffRad = shootToHub.getAngle().minus(shootToTarget.getAngle()).getRadians();
        double xHubCenter = shootToHub.getNorm() * Math.cos(angleDiffRad);
        double yHubPerp = Math.abs(shootToHub.getNorm() * Math.sin(angleDiffRad));
        
        boolean pathIntersectsHub = yHubPerp < (kHubObstacleRadiusMeters + 0.2);
        double xObstacle = xHubCenter - kHubObstacleRadiusMeters; 
        double yObstacle = kHubObstacleHeightMeters - kShooterHeightMeters + 0.15; 
        
        // For passing to the floor, we can still use math since pinpoint accuracy matters slightly less, 
        // OR you can create a second distanceToPassingRPSMap if you want to dial this in empirically too.
        double rps = calculateExactRPS(xTarget, yTarget);
        boolean clearsObstacle = true;

        if (rps > 0 && pathIntersectsHub && xObstacle > 0.5 && xObstacle < xTarget) {
            double launchRad = Math.toRadians(kFixedExitAngleDeg);
            double cosSq = Math.pow(Math.cos(launchRad), 2);
            double vWheel = rps * Math.PI * kWheelDiameterMeters;
            double vBallSq = Math.pow((vWheel / 2.0) * kShooterEfficiency, 2);

            double heightAtObstacle = xObstacle * Math.tan(launchRad) - (9.81 * xObstacle * xObstacle) / (2 * vBallSq * cosSq);
            if (heightAtObstacle < yObstacle) {
                clearsObstacle = false;
            }
        }

        if (clearsObstacle && rps > 0 && rps <= kMaxWheelRPS) {
            this.targetWheelRPS = rps;
            updateToF(xTarget, rps);
        } else {
            this.targetWheelRPS = kMaxWheelRPS; 
        }
    }

    private double calculateExactRPS(double x, double y) {
        double launchRad = Math.toRadians(kFixedExitAngleDeg);
        double cosSq = Math.pow(Math.cos(launchRad), 2);
        double tan = Math.tan(launchRad);
        
        double denominator = 2 * cosSq * (x * tan - y);
        if (denominator <= 0) return -1; 
        
        double vBallSq = (9.81 * x * x) / denominator;
        if (vBallSq <= 0) return -1;
        
        double requiredVBall = Math.sqrt(vBallSq);
        double requiredVWheel = (requiredVBall / kShooterEfficiency) * 2.0;
        return requiredVWheel / (Math.PI * kWheelDiameterMeters);
    }

    private void updateToF(double x, double rps) {
        double vWheel = rps * Math.PI * kWheelDiameterMeters;
        double vBall = (vWheel / 2.0) * kShooterEfficiency;
        double launchRad = Math.toRadians(kFixedExitAngleDeg);
        
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
            (alliance == DriverStation.Alliance.Blue ? kBlueHumanPlayerRightCorner : kRedHumanPlayerRightCorner) 
            : hub;
        
        Translation2d shooterPos = robotPose.getTranslation().plus(
            new Translation2d(Constants.kShooterOffsetX, Constants.kShooterOffsetY).rotateBy(robotPose.getRotation())
        );

        double dist = shooterPos.getDistance(target);
        double estimatedToF = (lastToF > 0) ? lastToF : (dist / 18.0); 
        
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

    public Rotation2d getHoodAngle() { return Rotation2d.fromDegrees(kFixedExitAngleDeg); }
    public double getShooterRPS() { return targetWheelRPS; }
    public Rotation2d getDrivebaseAimAngle() { return driveAim; }
    public double getLastVirtualDistance() { return lastVirtualDist; } 
}