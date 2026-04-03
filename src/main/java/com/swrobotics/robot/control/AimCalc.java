package com.swrobotics.robot.control;

import com.swrobotics.lib.utils.MathUtil;
import com.swrobotics.robot.config.Constants;
import com.swrobotics.robot.config.FieldPositions;
import edu.wpi.first.math.geometry.*;
import edu.wpi.first.math.interpolation.InterpolatingDoubleTreeMap;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.DriverStation;

public class AimCalc {    
    private static final InterpolatingDoubleTreeMap hoodAngleMap = new InterpolatingDoubleTreeMap();
    private final InterpolatingDoubleTreeMap rpsMap = new InterpolatingDoubleTreeMap();
    private static final AimCalc instance = new AimCalc();
    public static AimCalc getInstance() { return instance; }

    // --- HOOD GEOMETRY ---
    // 23 degrees: Hood is retracted. Ball pops UP more (67 deg launch). Best for close shots.
    // 48 degrees: Hood is pushed down. Ball shoots FLATTER (42 deg launch). Best for far shots.
    private static final double kMinAngleDeg = 23.0;
    private static final double kMaxAngleDeg = 48.0;

    // Trajectory Constants 
    private static final double kShooterHeightMeters = 0.6; 
    private static final double kHubHeightMeters = 2.0;     
    private static final double kTargetDepthOffset = 0.15;  
    private static final double kWheelDiameterMeters = 4.0 * 0.0254; 
    private static final double kShooterEfficiency = 0.85;  

    private Rotation2d driveAim = new Rotation2d();
    private double lastDist = 0.0;
    private double lastVirtualDist = 0.0;

    private AimCalc() {
        // RPS Mapping: 40 up to 0.75m, then 60 for general field scoring.
        rpsMap.put(0.0, 40.0);
        rpsMap.put(1.25, 40.0);
        rpsMap.put(1.251, 60.0); 
        rpsMap.put(16.5, 60.0); // Out to driver station wall

        // Pre-calculate angles based on physics out to field edge
        double[] testDistances = {0.0, 0.5, 0.75, 1.0, 1.5, 2.0, 3.0, 5.0, 7.5, 10.0, 13.0, 16.5};
        for (double d : testDistances) {
            hoodAngleMap.put(d, calculateLaunchAngle(d, rpsMap.get(d)));
        }
    }

   private double calculateLaunchAngle(double distance, double wheelRps) {
        double vWheel = wheelRps * Math.PI * kWheelDiameterMeters;
        double vBall = (vWheel / 2.0) * kShooterEfficiency; 
        
        double x = distance + kTargetDepthOffset;
        double y = kHubHeightMeters - kShooterHeightMeters; 
        double g = 9.81;
        double v2 = vBall * vBall;

        // Step 1: Check the physical limits of the hood.
        // A minimum hood angle of 23 degrees creates our steepest launch angle (67 degrees)
        double maxLaunchAngleRad = Math.toRadians(90.0 - kMinAngleDeg);
        
        // Trajectory equation to find the height of the ball at distance x with our steepest shot
        double yAtMinHoodAngle = (x * Math.tan(maxLaunchAngleRad)) 
                               - ((g * x * x) / (2 * v2 * Math.pow(Math.cos(maxLaunchAngleRad), 2)));

        // If the ball's height at distance 'x' clears the target height 'y' using the steepest shot, 
        // 23 degrees is plenty to make the shot (we are very close to the hub).
        if (yAtMinHoodAngle >= y) {
            return kMinAngleDeg;
        }

        // Step 2: If we are further out, calculate the exact required flatter trajectory.
        double discriminant = Math.pow(v2, 2) - g * (g * x * x + 2 * y * v2);

        // If discriminant < 0, the target is out of physical range for the current RPS.
        // Default to the flattest possible angle to maximize distance.
        if (discriminant < 0) return kMaxAngleDeg; 

        double launchAngleRad = Math.atan((v2 - Math.sqrt(discriminant)) / (g * x));
        double launchAngleDeg = Math.toDegrees(launchAngleRad);

        double physicalHoodAngle = 90.0 - launchAngleDeg; 

        return MathUtil.clamp(physicalHoodAngle, kMinAngleDeg, kMaxAngleDeg);
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
        double tof = 0.4 + (0.1 * lastDist); 
        
        Translation2d virtualTarget = hub.minus(new Translation2d(
            fieldSpeeds.vxMetersPerSecond * tof,
            fieldSpeeds.vyMetersPerSecond * tof
        ));
        
        lastVirtualDist = virtualTarget.getDistance(shooterPos);
        driveAim = virtualTarget.minus(shooterPos).getAngle();
    }

    public Rotation2d getHoodAngle(boolean passing) {
        // Passing gets kMaxAngleDeg (48 degrees) to smash the ball down into a flat trajectory
        Double targetDegrees = passing ? kMaxAngleDeg : hoodAngleMap.get(lastVirtualDist);
        return Rotation2d.fromDegrees(Math.max(kMinAngleDeg, Math.min(kMaxAngleDeg, targetDegrees)));
    }

    public double getShooterRPS(boolean passing) {
        return passing ? 90.0 : rpsMap.get(lastVirtualDist);
    }

    public Rotation2d getDrivebaseAimAngle() { return driveAim; }
    public double getLastVirtualDistance() { return lastVirtualDist; }
}