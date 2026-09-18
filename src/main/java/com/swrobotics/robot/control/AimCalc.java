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

    // --- SHOOTER KINEMATICS CONSTANTS ---
    private static final double kMinAngleDeg = 23.0; // Steepest lob
    private static final double kMaxAngleDeg = 48.0; // Flattest direct shot
    private static final double kShooterHeightMeters = Units.inchesToMeters(20); 
    private static final double kTargetDepthOffset = 0;  
    private static final double kWheelDiameterMeters = Units.inchesToMeters(4.0); 
    private static final double kShooterEfficiency = 0.85; 
    private static final double kMaxWheelRPS = 150.0;
    
    // --- FIELD CONSTANTS ---
    private static final double kHubObstacleHeightMeters = Units.inchesToMeters(72.0);
    private static final double kHubObstacleRadiusMeters = Units.inchesToMeters(41.0);

    private static final AimCalc instance = new AimCalc();
    public static AimCalc getInstance() { return instance; }

    private final InterpolatingDoubleTreeMap distanceToRPSMap = new InterpolatingDoubleTreeMap();
    private final InterpolatingDoubleTreeMap distanceToAngleMap = new InterpolatingDoubleTreeMap();

    private Rotation2d driveAim = new Rotation2d();
    private Rotation2d targetHoodAngle = Rotation2d.fromDegrees(kMaxAngleDeg);
    private double targetWheelRPS = 0.0;
    private double lastDist = 0.0;
    private double lastVirtualDist = 0.0;
    private double lastToF = 0.0; // Time of Flight
    private boolean passingMode = false;

    private AimCalc() {
        distanceToRPSMap.put(1.5, 40.0);   distanceToAngleMap.put(1.5, 48.0);
        distanceToRPSMap.put(3.0, 60.0);   distanceToAngleMap.put(3.0, 40.0); 
        distanceToRPSMap.put(5.0, 85.0);   distanceToAngleMap.put(5.0, 32.0); 
        distanceToRPSMap.put(7.0, 110.0);  distanceToAngleMap.put(7.0, 25.0); 
    }

    public void setPassingMode(boolean isPassing) {
        this.passingMode = isPassing;
    }

    /** Calculates the standard shot for hitting the hub using interpolation tables */
    private void calculateOptimalTrajectory(double distance) {
        double x = distance + kTargetDepthOffset;
        
        this.targetWheelRPS = MathUtil.clamp(distanceToRPSMap.get(distance), 0, kMaxWheelRPS);
        double hoodDeg = MathUtil.clamp(distanceToAngleMap.get(distance), kMinAngleDeg, kMaxAngleDeg);
        this.targetHoodAngle = Rotation2d.fromDegrees(hoodDeg);
        
        updateToF(x, this.targetWheelRPS, hoodDeg);
    }

    /** Calculates projectile motion to pass over the central hub without hitting it */
    private void calculatePassingTrajectory(Translation2d shooterPos, Translation2d targetPos, Translation2d hubPos) {
        double xTarget = shooterPos.getDistance(targetPos);
        double yTarget = -kShooterHeightMeters; 

        // Geometric relationship between Robot, Hub, and Target
        Translation2d shootToTarget = targetPos.minus(shooterPos);
        Translation2d shootToHub = hubPos.minus(shooterPos);
        
        double angleDiffRad = shootToHub.getAngle().minus(shootToTarget.getAngle()).getRadians();
        double xHubCenter = shootToHub.getNorm() * Math.cos(angleDiffRad);
        double yHubPerp = Math.abs(shootToHub.getNorm() * Math.sin(angleDiffRad));
        
        boolean pathIntersectsHub = yHubPerp < (kHubObstacleRadiusMeters + 0.2);
        
        double xObstacle = xHubCenter - kHubObstacleRadiusMeters; 
        double yObstacle = kHubObstacleHeightMeters - kShooterHeightMeters + 0.15; // 15cm safety margin
        
        boolean foundPass = false;

        // Iterate through possible hood angles, finding the flattest shot that clears the hub
        for (double hoodDeg = kMaxAngleDeg; hoodDeg >= kMinAngleDeg; hoodDeg -= 1.0) {
            double launchRad = Math.toRadians(90.0 - hoodDeg);
            double cosSq = Math.pow(Math.cos(launchRad), 2);
            double tan = Math.tan(launchRad);
            
            // Required ball velocity squared derived from standard 2D kinematic trajectory formula
            double vBallSq = (9.81 * xTarget * xTarget) / (2 * cosSq * (xTarget * tan - yTarget));
            if (vBallSq <= 0) continue;
            
            double requiredVBall = Math.sqrt(vBallSq);
            boolean clearsObstacle = true;
            
            // Check clearance if path crosses the hub area
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
        
        // Failsafe: Max lob
        if (!foundPass) {
            this.targetWheelRPS = kMaxWheelRPS;
            this.targetHoodAngle = Rotation2d.fromDegrees(kMinAngleDeg); 
        }
    }

    /** Updates estimated Time of Flight for moving target compensation */
    private void updateToF(double x, double rps, double hoodAngleDeg) {
        double vWheel = rps * Math.PI * kWheelDiameterMeters;
        double vBall = (vWheel / 2.0) * kShooterEfficiency;
        double launchRad = Math.toRadians(90.0 - hoodAngleDeg);
        
        this.lastToF = (vBall > 0) ? (x / (vBall * Math.cos(launchRad))) : (x / 18.0);
    }

    public void update(Pose2d robotPose, ChassisSpeeds fieldSpeeds) {
        DriverStation.Alliance alliance = DriverStation.getAlliance().orElse(DriverStation.Alliance.Blue);
        Translation2d hub = FieldPositions.getAllianceHubPose(alliance).getTranslation();
        
        // Adjust for shooter position relative to robot center
        Translation2d shooterPos = robotPose.getTranslation().plus(
            new Translation2d(Constants.kShooterOffsetX, Constants.kShooterOffsetY).rotateBy(robotPose.getRotation())
        );

        // Determine actual target (Hub vs closest Pass Pose)
        Translation2d target;
        if (passingMode) {
            Pose2d[] passPoses = FieldPositions.getAlliancePassPoses(alliance);
            target = shooterPos.getDistance(passPoses[0].getTranslation()) < shooterPos.getDistance(passPoses[1].getTranslation()) 
                     ? passPoses[0].getTranslation() : passPoses[1].getTranslation();
        } else {
            target = hub;
        }

        lastDist = shooterPos.getDistance(target);
        
        // Lead the target by predicting where we will be during the Time of Flight
        double estimatedToF = (lastToF > 0) ? lastToF : (lastDist / 18.0); 
        Translation2d virtualTarget = target.minus(new Translation2d(
            fieldSpeeds.vxMetersPerSecond * estimatedToF,
            fieldSpeeds.vyMetersPerSecond * estimatedToF
        ));
        
        lastVirtualDist = virtualTarget.getDistance(shooterPos);
        driveAim = virtualTarget.minus(shooterPos).getAngle();

        if (passingMode) calculatePassingTrajectory(shooterPos, virtualTarget, hub);
        else calculateOptimalTrajectory(lastVirtualDist);
    }

    public Rotation2d getHoodAngle() { return targetHoodAngle; }
    public double getShooterRPS() { return targetWheelRPS; }
    public Rotation2d getDrivebaseAimAngle() { return driveAim; }
    public double getLastVirtualDistance() { return lastVirtualDist; } 
}