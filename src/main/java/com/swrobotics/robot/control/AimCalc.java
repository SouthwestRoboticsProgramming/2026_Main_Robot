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
    private static final InterpolatingDoubleTreeMap passingHoodAngleMap = new InterpolatingDoubleTreeMap();
    private final InterpolatingDoubleTreeMap rpsMap = new InterpolatingDoubleTreeMap();
    private static final AimCalc instance = new AimCalc();
    public static AimCalc getInstance() { return instance; }

    // Sync with HoodSubsystem constraints
    private static final double kMinAngleDeg = 26.0;
    private static final double kMaxAngleDeg = 50.0;

    // Trajectory Constants (Tune these based on physical measurements)
    private static final double kShooterHeightMeters = 0.6; // Floor to ball exit
    private static final double kHubHeightMeters = 2.0;     // Floor to Hub rim
    private static final double kTargetDepthOffset = 0.15;  // Aim slightly behind the front rim
    private static final double kWheelDiameterMeters = 4.0 * 0.0254; 
    private static final double kShooterEfficiency = 0.85;  // Adjust for energy loss/slip

    private Rotation2d driveAim = new Rotation2d();
    private double lastDist = 0.0;
    private double passDist = 0.0;
    private double lastVirtualDist = 0.0;

    private AimCalc() {
        // RPS Mapping based on distance (meters)
        rpsMap.put(0.0, 40.0);
        rpsMap.put(1.0, 40.0);
        rpsMap.put(2.0, 60.0);
        rpsMap.put(5.0, 90.0);
        rpsMap.put(10.0, 90.0);

        // Pre-calculate angles based on physics
        double[] testDistances = {0.5, 1.0, 1.5, 2.0, 2.5, 3.0, 4.0, 5.0, 7.5, 10.0};
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
        double discriminant = Math.pow(v2, 2) - g * (g * x * x + 2 * y * v2);

        if (discriminant < 0) return kMaxAngleDeg;

        // This gives angle relative to floor (e.g., 45 degrees)
        double launchAngleRad = Math.atan((v2 - Math.sqrt(discriminant)) / (g * x));
        double launchAngleDeg = Math.toDegrees(launchAngleRad);

        // FLIP THE LOGIC FOR YOUR HOOD:
        // If your hood at 26 degrees points "Up" (90 deg launch) 
        // and 50 degrees points "Forward" (0 deg launch), we map it:
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
        Double targetDegrees = passing ? 45.0 : hoodAngleMap.get(lastVirtualDist);
        return Rotation2d.fromDegrees(Math.max(kMinAngleDeg, Math.min(kMaxAngleDeg, targetDegrees)));
    }

    public double getShooterRPS(boolean passing) {
        return passing ? 90.0 : rpsMap.get(lastVirtualDist);
    }

    public Rotation2d getDrivebaseAimAngle() { return driveAim; }
    public double getLastVirtualDistance() { return lastVirtualDist; }
}