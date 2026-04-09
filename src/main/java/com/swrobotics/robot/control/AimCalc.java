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

    private static final double kHubHeightMetersOffset = 0.43;
    private static final double kMinAngleDeg = 23.0; // Steepest shot (lob)
    private static final double kMaxAngleDeg = 48.0; // Flattest shot (direct)

    private static final double kShooterHeightMeters = Units.inchesToMeters(20); 
    private static final double kHubTargetHeightMeters = 1.257;      
    private static final double kTargetDepthOffset = 0;  
    private static final double kWheelDiameterMeters = Units.inchesToMeters(4.0); 
    private static final double kShooterEfficiency = 0.85; 
    
    private static final double kMaxWheelRPS = 150.0;
    private static final double kRPSStep = 5.0; 

    private static final double kHubObstacleHeightMeters = Units.inchesToMeters(72.0);
    private static final double kHubObstacleRadiusMeters = Units.inchesToMeters(41.0);
    
    // TUNE THESE: The coordinates of the corner in front of the HP station
    private static final Translation2d kBlueHumanPlayerRightCorner = new Translation2d(1.6, 1.45);
    private static final Translation2d kBlueHumanPlayerLeftCorner = new Translation2d(1.6, 6.6);
    
    private static final Translation2d kRedHumanPlayerLeftCorner = new Translation2d(15, 1.45);
    private static final Translation2d kRedHumanPlayerRightCorner = new Translation2d(15, 6.6);

    // TUNE THESE: Turret physical soft limits to prevent infinite spinning/wire damage
    private static final double kTurretMinAngleDeg = -270.0; 
    private static final double kTurretMaxAngleDeg = 270.0;  

    private static final AimCalc instance = new AimCalc();
    public static AimCalc getInstance() { return instance; }

    // --- INTERPOLATING MAPS ---
    private final InterpolatingDoubleTreeMap distanceToRPSMap = new InterpolatingDoubleTreeMap();
    private final InterpolatingDoubleTreeMap distanceToAngleMap = new InterpolatingDoubleTreeMap();

    private Rotation2d driveAim = new Rotation2d();
    private Rotation2d targetTurretAngle = new Rotation2d();
    private double lastDist = 0.0;
    private double lastVirtualDist = 0.0;
    private double lastToF = 0.0; 

    private Rotation2d targetHoodAngle = Rotation2d.fromDegrees(kMaxAngleDeg);
    private double targetWheelRPS = 0.0;
    private boolean passingMode = false;

    private AimCalc() {
        // TUNE THESE: (Distance in Meters, Target Value)
        distanceToRPSMap.put(1.5, 40.0);
        distanceToAngleMap.put(1.5, 48.0);

        distanceToRPSMap.put(3.0, 60.0);
        distanceToAngleMap.put(3.0, 40.0); 

        distanceToRPSMap.put(5.0, 85.0);
        distanceToAngleMap.put(5.0, 32.0); 
        
        distanceToRPSMap.put(7.0, 110.0);
        distanceToAngleMap.put(7.0, 25.0); 
    }

    public void setPassingMode(boolean isPassing) {
        this.passingMode = isPassing;
    }

    private void calculateOptimalTrajectory(double distance) {
        double x = distance + kTargetDepthOffset;
        
        // O(1) Lookup replacing the heavy math loop
        double rawRPS = distanceToRPSMap.get(distance);
        double rawHoodDeg = distanceToAngleMap.get(distance);
        
        // Clamp constraints to ensure extrapolation doesn't exceed physical limits
        this.targetWheelRPS = MathUtil.clamp(rawRPS, 0, kMaxWheelRPS);
        double hoodDeg = MathUtil.clamp(rawHoodDeg, kMinAngleDeg, kMaxAngleDeg);
        
        this.targetHoodAngle = Rotation2d.fromDegrees(hoodDeg);
        
        // Feed into your existing ToF logic
        updateToF(x, this.targetWheelRPS, hoodDeg);
    }

    private void calculatePassingTrajectory(Translation2d shooterPos, Translation2d targetPos, Translation2d hubPos) {
        double xTarget = shooterPos.getDistance(targetPos);
        double yTarget =  - kShooterHeightMeters; 

        Translation2d shootToTarget = targetPos.minus(shooterPos);
        Translation2d shootToHub = hubPos.minus(shooterPos);
        
        // Project the hub onto the passing path to see if it's in the way
        double angleDiffRad = shootToHub.getAngle().minus(shootToTarget.getAngle()).getRadians();
        double xHubCenter = shootToHub.getNorm() * Math.cos(angleDiffRad);
        double yHubPerp = Math.abs(shootToHub.getNorm() * Math.sin(angleDiffRad));
        
        // Hub is in the way if the perpendicular distance is less than the hub radius + ball clearance (0.2m)
        boolean pathIntersectsHub = yHubPerp < (kHubObstacleRadiusMeters + 0.2);
        
        double xObstacle = xHubCenter - kHubObstacleRadiusMeters; 
        double yObstacle = kHubObstacleHeightMeters - kShooterHeightMeters + 0.15; // + 15cm safety margin
        
        boolean foundPass = false;

        for (double hoodDeg = kMaxAngleDeg; hoodDeg >= kMinAngleDeg; hoodDeg -= 1.0) {
            double launchRad = Math.toRadians(90.0 - hoodDeg);
            double cosSq = Math.pow(Math.cos(launchRad), 2);
            double tan = Math.tan(launchRad);
            
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

    /**
     * Determines the optimal turret angle, ensuring it unwinds correctly if it hits physical bounds.
     */
    private Rotation2d wrapTurretAngle(Rotation2d rawTargetAngle, Rotation2d currentAngle) {
        // Normalize target angle to a standard [-180, 180) range
        double normalizedTarget = rawTargetAngle.getDegrees() % 360.0;
        if (normalizedTarget <= -180.0) normalizedTarget += 360.0;
        if (normalizedTarget > 180.0) normalizedTarget -= 360.0;

        double currentDeg = currentAngle.getDegrees();

        double[] possibleAngles = {
            normalizedTarget - 360.0,
            normalizedTarget,
            normalizedTarget + 360.0
        };

        double bestAngle = currentDeg;
        double minError = Double.MAX_VALUE;

        for (double angle : possibleAngles) {
            // Check if this equivalent angle is within our physical soft limits
            if (angle >= kTurretMinAngleDeg && angle <= kTurretMaxAngleDeg) {
                double error = Math.abs(angle - currentDeg);
                if (error < minError) {
                    minError = error;
                    bestAngle = angle;
                }
            }
        }

        if (minError == Double.MAX_VALUE) {
            bestAngle = MathUtil.clamp(normalizedTarget, kTurretMinAngleDeg, kTurretMaxAngleDeg);
        }

        return Rotation2d.fromDegrees(bestAngle);
    }

    public void update(Pose2d robotPose, ChassisSpeeds fieldSpeeds) {
        update(robotPose, fieldSpeeds, new Rotation2d());
    }

    public void update(Pose2d robotPose, ChassisSpeeds fieldSpeeds, Rotation2d currentTurretAngle) {
        DriverStation.Alliance alliance = DriverStation.getAlliance().orElse(DriverStation.Alliance.Blue);
        Translation2d hub = FieldPositions.getAllianceHubPose(alliance).getTranslation();
        
        Translation2d target = passingMode ? 
            (alliance == DriverStation.Alliance.Blue ? kBlueHumanPlayerRightCorner : kRedHumanPlayerRightCorner) 
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

        // Calculate Turret Angle with physical wrapping constraints
        Rotation2d rawTurretAim = driveAim.minus(robotPose.getRotation());
        targetTurretAngle = wrapTurretAngle(rawTurretAim, currentTurretAngle);

        if (passingMode) {
            calculatePassingTrajectory(shooterPos, virtualTarget, hub);
        } else {
            calculateOptimalTrajectory(lastVirtualDist);
        }
    }

    public Rotation2d getHoodAngle() { return targetHoodAngle; }
    public double getShooterRPS() { return targetWheelRPS; }
    public Rotation2d getDrivebaseAimAngle() { return driveAim; }
    public Rotation2d getTurretAimAngle() { return targetTurretAngle; }
    public double getLastVirtualDistance() { return lastVirtualDist; } 
}