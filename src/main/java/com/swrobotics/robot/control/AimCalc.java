package com.swrobotics.robot.control;

import java.util.TreeMap;

import com.swrobotics.robot.config.Constants;
import com.swrobotics.robot.config.FieldPositions;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.DriverStation;

public class AimCalc {
    private static final AimCalc instance = new AimCalc();
    public static AimCalc getInstance() { return instance; }


    // Base flight time offset (seconds)
    public static double kBaseFlightTime = 0.4;
    // Additional flight time per meter of distance (seconds per meter)
    public static double kFlightTimePerMeter = 0.10;

    private final TreeMap<Double, ShotParams> shotMap = new TreeMap<>();

    private Rotation2d drivebaseAimAngle = new Rotation2d();
    private Rotation2d hoodAngle = new Rotation2d();
    private double shooterRPS = 0.0;
    private double lastDistanceToHub = 0.0;

    private record ShotParams(double rps, double hoodDegrees) {}

    private AimCalc() {
        //TODO: Tune TS boy
        shotMap.put(1.0,  new ShotParams(20, 30));
        shotMap.put(1.25, new ShotParams(20, 50));
        shotMap.put(1.5,  new ShotParams(20, 20));
        shotMap.put(1.75, new ShotParams(20, 20));
    }

    public void update(Pose2d robotPose, double fieldVx, double fieldVy) {
        // Alliance-relative hub position
        Translation2d hubTarget = FieldPositions.getAllianceHubPose(
            DriverStation.getAlliance().orElse(DriverStation.Alliance.Blue)
        ).getTranslation();

        // Shooter offset from robot center
        Translation2d shooterOffset = new Translation2d(
            Constants.kShooterOffsetX,
            Constants.kShooterOffsetY
        ).rotateBy(robotPose.getRotation());

        Translation2d shooterFieldPos = robotPose.getTranslation().plus(shooterOffset);

        double staticDist = shooterFieldPos.getDistance(hubTarget);
        lastDistanceToHub = staticDist;

        double shotTime = estimateFlightTime(staticDist);

        // Compensate for robot motion: virtual target
        Translation2d robotMotionDuringShot = new Translation2d(
            fieldVx * shotTime,
            fieldVy * shotTime
        );
        Translation2d virtualTarget = hubTarget.minus(robotMotionDuringShot);

        Translation2d delta = virtualTarget.minus(shooterFieldPos);

        drivebaseAimAngle = delta.getAngle();
        double vDist = delta.getNorm();

        ShotParams params = getInterpolatedParams(vDist);
        shooterRPS = params.rps;
        hoodAngle = Rotation2d.fromDegrees(params.hoodDegrees);
    }

    private double estimateFlightTime(double distanceMeters) {
        // TUNING: adjust kBaseFlightTime and kFlightTimePerMeter
        return kBaseFlightTime + kFlightTimePerMeter * distanceMeters;
    }

    private ShotParams getInterpolatedParams(double distance) {
        double minKey = shotMap.firstKey();
        double maxKey = shotMap.lastKey();
        double clamped = Math.max(minKey, Math.min(maxKey, distance));

        Double lowKey = shotMap.floorKey(clamped);
        Double highKey = shotMap.ceilingKey(clamped);

        if (lowKey == null && highKey == null) {
            // Should not happen if map not empty
            return new ShotParams(0.0, 0.0);
        }
        if (lowKey == null) return shotMap.get(highKey);
        if (highKey == null) return shotMap.get(lowKey);
        if (lowKey.equals(highKey)) return shotMap.get(lowKey);

        ShotParams low = shotMap.get(lowKey);
        ShotParams high = shotMap.get(highKey);
        double t = (clamped - lowKey) / (highKey - lowKey);

        double rps = low.rps + t * (high.rps - low.rps);
        double hoodDeg = low.hoodDegrees + t * (high.hoodDegrees - low.hoodDegrees);
        return new ShotParams(rps, hoodDeg);
    }

    public Rotation2d getDrivebaseAimAngle() { return drivebaseAimAngle; }
    public Rotation2d getHoodAngle() { return hoodAngle; }
    public double getShooterRPS() { return shooterRPS; }
    public double getLastDistanceToHub() { return lastDistanceToHub; }
}