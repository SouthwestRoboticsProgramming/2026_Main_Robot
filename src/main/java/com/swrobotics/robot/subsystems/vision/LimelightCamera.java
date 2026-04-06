package com.swrobotics.robot.subsystems.vision;

import com.swrobotics.lib.utils.MathUtil;
import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.networktables.*;
import edu.wpi.first.wpilibj.DriverStation;

import java.util.List;

public final class LimelightCamera {
    private static final String MEGATAG_1_NAME = "botpose_wpiblue";
    private static final String MEGATAG_2_NAME = "botpose_orb_wpiblue";
    private static final String ORIENTATION_NAME = "robot_orientation_set";
    private static final String LOCATION_NAME = "camerapose_robotspace_set";

    public record MountingLocation(
            double forward, double right, double up,
            double roll, double pitch, double yaw) {}

    public record Config(
            double mt1MaxDistance,
            double xyStdDevCoeffMT1,
            double thetaStdDevCoeffMT1,
            double xyStdDevCoeffMT2) {}

    public record Update(Pose2d pose, double timestamp, Matrix<N3, N1> stdDevs) {}

    private record PoseEstimate(Pose2d pose, double timestamp, int tagCount, double avgTagDist) {}

    private final Config config;
    private final DoubleArraySubscriber mt1EstimateSub;
    private final DoubleArraySubscriber mt2EstimateSub;
    private final DoubleArrayPublisher robotOrientationPub;
    private final DoubleArrayPublisher mountingLocationPub;

    private double prevUpdateTimestamp;

    public LimelightCamera(String name, MountingLocation location, Config config) {
        this.config = config;

        NetworkTable table = NetworkTableInstance.getDefault().getTable(name);

        mt1EstimateSub = table.getDoubleArrayTopic(MEGATAG_1_NAME).subscribe(new double[0]);
        mt2EstimateSub = table.getDoubleArrayTopic(MEGATAG_2_NAME).subscribe(new double[0]);

        robotOrientationPub = table.getDoubleArrayTopic(ORIENTATION_NAME).publish();

        mountingLocationPub = table.getDoubleArrayTopic(LOCATION_NAME).publish();
        mountingLocationPub.set(new double[] {
            location.forward(), location.right(), location.up(),
            location.roll(), location.pitch(), location.yaw()
        });

        prevUpdateTimestamp = Double.NaN;
    }

    public void updateRobotState(double yawAngle, double yawRate) {
        robotOrientationPub.set(new double[] {
            yawAngle, yawRate, 0, 0, 0, 0
        });
    }

    public void getNewUpdates(List<Update> updatesOut, boolean useMegaTag2) {
        PoseEstimate mt1 = decodeEstimate(mt1EstimateSub);
        PoseEstimate mt2 = decodeEstimate(mt2EstimateSub);

        processEstimate(updatesOut, mt1, mt2, useMegaTag2);
    }

    private PoseEstimate decodeEstimate(DoubleArraySubscriber sub) {
        TimestampedDoubleArray estimate = sub.getAtomic();
        long timestamp = estimate.timestamp;
        double[] data = estimate.value;

        if (data.length < 10)
            return null;

        Pose2d pose = new Pose2d(
            new Translation2d(data[0], data[1]),
            Rotation2d.fromDegrees(data[5]));
        double latency = data[6];
        int tagCount = (int) data[7];
        double avgTagDist = data[9];

        if (tagCount <= 0 || (pose.getX() == 0 && pose.getY() == 0))
            return null;

        double correctedTimestamp = (timestamp / 1000000.0) - (latency / 1000.0);

        return new PoseEstimate(pose, correctedTimestamp, tagCount, avgTagDist);
    }

    private void processEstimate(List<Update> updatesOut, PoseEstimate mt1, PoseEstimate mt2, boolean useMegaTag2) {
        PoseEstimate est = useMegaTag2 ? mt2 : mt1;

        if (est == null || est.timestamp == prevUpdateTimestamp)
            return;

        if (!useMegaTag2 && est.avgTagDist > config.mt1MaxDistance && DriverStation.isEnabled()) {
            processEstimate(updatesOut, mt1, mt2, true);
            return;
        }

        prevUpdateTimestamp = est.timestamp;

        double baseStdDev = MathUtil.square(est.avgTagDist) / est.tagCount;

        double xyStdDev;
        double thetaStdDev;
        if (useMegaTag2) {
            xyStdDev = baseStdDev * config.xyStdDevCoeffMT2;
            thetaStdDev = 999999999999999.0;
        } else {
            xyStdDev = baseStdDev * config.xyStdDevCoeffMT1;
            thetaStdDev = baseStdDev * config.thetaStdDevCoeffMT1;
        }
        

        updatesOut.add(new Update(
                est.pose,
                est.timestamp,
                VecBuilder.fill(xyStdDev, xyStdDev, thetaStdDev)
        ));
    }
}
