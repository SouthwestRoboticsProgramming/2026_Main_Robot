// package com.swrobotics.robot.control;

// import java.util.TreeMap;

// import com.swrobotics.robot.config.Constants;
// import com.swrobotics.robot.config.FieldPositions;

// import edu.wpi.first.math.geometry.Pose2d;
// import edu.wpi.first.math.geometry.Rotation2d;
// import edu.wpi.first.math.geometry.Translation2d;
// import edu.wpi.first.wpilibj.DriverStation;

// public class AimCalc {
//     private static final AimCalc instance = new AimCalc();
//     public static AimCalc getInstance() { return instance; }


//     public static double kConstantShooterRPS = 20.0;

//     public static double kBaseFlightTime = 0.4;
//     public static double kFlightTimePerMeter = 0.10;

//     private final TreeMap<Double, Double> hoodMap = new TreeMap<>();

//     private Rotation2d drivebaseAimAngle = new Rotation2d();
//     private Rotation2d hoodAngle = new Rotation2d();
//     private double shooterRPS = kConstantShooterRPS;
//     private double lastDistanceToHub = 0.0;

//     private AimCalc() {

//         hoodMap.put(1.0,  30.0);
//         hoodMap.put(1.25, 35.0);
//         hoodMap.put(1.5,  40.0);
//         hoodMap.put(1.75, 45.0);
//     }

//     public void update(Pose2d robotPose, double fieldVx, double fieldVy) {
//         Translation2d hubTarget = FieldPositions.getAllianceHubPose(
//             DriverStation.getAlliance().orElse(DriverStation.Alliance.Blue)
//         ).getTranslation();

//         Translation2d shooterOffset = new Translation2d(
//             Constants.kShooterOffsetX,
//             Constants.kShooterOffsetY
//         ).rotateBy(robotPose.getRotation());

//         Translation2d shooterFieldPos = robotPose.getTranslation().plus(shooterOffset);

//         double actualDist = shooterFieldPos.getDistance(hubTarget);
//         lastDistanceToHub = actualDist;

//         double shotTime = estimateFlightTime(actualDist);

//         Translation2d robotMotionDuringShot = new Translation2d(
//             fieldVx * shotTime,
//             fieldVy * shotTime
//         );
//         Translation2d virtualTarget = hubTarget.minus(robotMotionDuringShot);

//         Translation2d delta = virtualTarget.minus(shooterFieldPos);

//         drivebaseAimAngle = delta.getAngle();
//         double vDist = delta.getNorm();

//         shooterRPS = kConstantShooterRPS;       
//         hoodAngle = Rotation2d.fromDegrees(getInterpolatedHoodAngle(vDist));
//     }

//     private double estimateFlightTime(double distanceMeters) {
//         return kBaseFlightTime + kFlightTimePerMeter * distanceMeters;
//     }

//     private double getInterpolatedHoodAngle(double distance) {
//         double minKey = hoodMap.firstKey();
//         double maxKey = hoodMap.lastKey();
//         double clamped = Math.max(minKey, Math.min(maxKey, distance));

//         Double lowKey = hoodMap.floorKey(clamped);
//         Double highKey = hoodMap.ceilingKey(clamped);

//         if (lowKey == null && highKey == null) {
//             return 0.0;
//         }
//         if (lowKey == null) return hoodMap.get(highKey);
//         if (highKey == null) return hoodMap.get(lowKey);
//         if (lowKey.equals(highKey)) return hoodMap.get(lowKey);

//         double lowVal = hoodMap.get(lowKey);
//         double highVal = hoodMap.get(highKey);
//         double t = (clamped - lowKey) / (highKey - lowKey);

//         return lowVal + t * (highVal - lowVal);
//     }

//     public Rotation2d getDrivebaseAimAngle() { return drivebaseAimAngle; }
//     public Rotation2d getHoodAngle() { return hoodAngle; }
//     public double getShooterRPS() { return shooterRPS; }
//     public double getLastDistanceToHub() { return lastDistanceToHub; }
// }
package com.swrobotics.robot.control;

import com.swrobotics.robot.config.Constants;
import com.swrobotics.robot.config.FieldPositions;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.DriverStation;

public class AimCalc {
    private static final AimCalc instance = new AimCalc();
    public static AimCalc getInstance() { return instance; }

    public static double kConstantShooterRPS = 20.0;

    // Base flight time offset (seconds)
    public static double kBaseFlightTime = 0.4;
    public static double kFlightTimePerMeter = 0.10;


    public static double kHoodCurveA = 0.0;  // (Curve steepness)
    public static double kHoodCurveB = 5.0;  //(Degrees added per meter)
    public static double kHoodCurveC = 22.7; //(Base angle at 0 meters)

    private Rotation2d drivebaseAimAngle = new Rotation2d();
    private Rotation2d hoodAngle = new Rotation2d();
    private double shooterRPS = kConstantShooterRPS;
    private double lastDistanceToHub = 0.0;

    private AimCalc() {
        // Constructor is now empty since we don't need to build a map!
    }

    public void update(Pose2d robotPose, double fieldVx, double fieldVy) {
        // Alliance-relative hub position
        Translation2d hubTarget = FieldPositions.getAllianceHubPose(
            DriverStation.getAlliance().orElse(DriverStation.Alliance.Blue)
        ).getTranslation();

        // Shooter offset from robot center, rotated into field frame
        Translation2d shooterOffset = new Translation2d(
            Constants.kShooterOffsetX,
            Constants.kShooterOffsetY
        ).rotateBy(robotPose.getRotation());

        Translation2d shooterFieldPos = robotPose.getTranslation().plus(shooterOffset);

        // Actual distance to hub
        double actualDist = shooterFieldPos.getDistance(hubTarget);
        lastDistanceToHub = actualDist;

        // Estimate flight time based on actual distance
        double shotTime = estimateFlightTime(actualDist);

        // Compensate for robot motion: virtual target
        Translation2d robotMotionDuringShot = new Translation2d(
            fieldVx * shotTime,
            fieldVy * shotTime
        );
        Translation2d virtualTarget = hubTarget.minus(robotMotionDuringShot);

        Translation2d delta = virtualTarget.minus(shooterFieldPos);

        drivebaseAimAngle = delta.getAngle();
        double vDist = delta.getNorm(); // Virtual distance for shoot-on-the-move

        // 1. Lock the shooter to the constant RPS
        shooterRPS = kConstantShooterRPS;
        
        // 2. Calculate the hood angle using the polynomial curve
        hoodAngle = Rotation2d.fromDegrees(calculateHoodAngle(vDist));
    }

    private double estimateFlightTime(double distanceMeters) {
        // TUNING: adjust kBaseFlightTime and kFlightTimePerMeter
        return kBaseFlightTime + kFlightTimePerMeter * distanceMeters;
    }

    private double calculateHoodAngle(double distance) {
        // Quadratic equation to calculate the exact angle needed
        // y = ax^2 + bx + c
        double targetDegrees = (kHoodCurveA * (distance * distance)) 
                             + (kHoodCurveB * distance) 
                             + kHoodCurveC;

        return targetDegrees;
    }

    public Rotation2d getDrivebaseAimAngle() { return drivebaseAimAngle; }
    public Rotation2d getHoodAngle() { return hoodAngle; }
    public double getShooterRPS() { return shooterRPS; }
    public double getLastDistanceToHub() { return lastDistanceToHub; }
}