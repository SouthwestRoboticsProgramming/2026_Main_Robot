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
    private static final InterpolatingDoubleTreeMap hoodAngleMap = new InterpolatingDoubleTreeMap();
    
    

    // --- HOOD GEOMETRY ---
    // 23 degrees: Hood is retracted. Ball pops UP more (67 deg launch). Best for close shots.
    // 48 degrees: Hood is pushed down. Ball shoots FLATTER (42 deg launch). Best for far shots.
    private static final double kMinAngleDeg = 23.0;
    private static final double kMaxAngleDeg = 48.0;

    // Trajectory Constants 
    private static final double kShooterHeightMeters = Units.inchesToMeters(20); 
    private static final double kHubHeightMeters = 1.257;     
    private static final double kTargetDepthOffset = 0.15;  
    private static final double kWheelDiameterMeters = Units.inchesToMeters(4.0); 
    private static final double kShooterEfficiency = 0.85;  
    private static final AimCalc instance = new AimCalc();
    public static AimCalc getInstance() { return instance; }
    private final InterpolatingDoubleTreeMap rpsMap = new InterpolatingDoubleTreeMap();
    private Rotation2d driveAim = new Rotation2d();
    private double lastDist = 0.0;
    private double lastVirtualDist = 0.0;

    private AimCalc() {
        
        rpsMap.put(.75, 40.0); 
        rpsMap.put(1.0, 40.0);
        rpsMap.put(1.25, 40.0);
        rpsMap.put(1.5, 40.0); 
        rpsMap.put(1.75, 60.0);
        rpsMap.put(2.0, 60.0);
        rpsMap.put(2.25, 60.0);
        rpsMap.put(2.5, 60.0);
        rpsMap.put(2.51, 60.0); 
        rpsMap.put(16.5, 60.0);

        double[] testDistances = {0.75, 1.0, 1.25, 1.5, 1.75, 2.0, 2.25,  2.5, 2.51, 16.5};
        for (double d : testDistances) {
            hoodAngleMap.put(d, calculateLaunchAngle(d, rpsMap.get(d)));
        }
    }

   private double calculateLaunchAngle(double distance, double wheelRps) {
        double vWheel = wheelRps * Math.PI * kWheelDiameterMeters;
        double vBall = (vWheel / 2.0) * kShooterEfficiency; 
        
        double x = 3; //distance + kTargetDepthOffset;
        double y = kHubHeightMeters - kShooterHeightMeters; 
        double g = 9.81;
        double v2 = vBall * vBall;

        double maxLaunchAngleRad = Math.toRadians(90.0 - kMinAngleDeg);
        
        double yAtMinHoodAngle = (x * Math.tan(maxLaunchAngleRad)) 
                               - ((g * x * x) / (2 * v2 * Math.pow(Math.cos(maxLaunchAngleRad), 2)));

                               System.out.println(x);
                               System.out.println(y);
                               System.out.println(v2);
        if (yAtMinHoodAngle >= y) {
            return kMinAngleDeg;
        }

        double discriminant = Math.pow(v2, 2) - g * (g * x * x + 2 * y * v2);

        if (discriminant < 0.0) return kMaxAngleDeg; 

        double launchAngleRad = Math.atan((v2 + Math.sqrt(discriminant)) / (g * x));
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
        Double targetDegrees = passing ? kMaxAngleDeg :hoodAngleMap.get(lastDist);
        return Rotation2d.fromDegrees(Math.max(kMinAngleDeg, Math.min(kMaxAngleDeg, targetDegrees)));
    }

    public double getShooterRPS(boolean passing) {
        return passing ? 90.0 : rpsMap.get(lastVirtualDist);
    }

    public Rotation2d getDrivebaseAimAngle() { return driveAim; }
    public double getLastVirtualDistance() { return lastDist; }

}