package com.swrobotics.robot.subsystems.vision;

import edu.wpi.first.math.geometry.Pose2d;
import org.littletonrobotics.junction.AutoLog;

public interface VisionIO {
    @AutoLog
    public static class VisionIOInputs {
        public boolean connected = false;
        public double[] estimatedPoseXs = new double[0];
        public double[] estimatedPoseYs = new double[0];
        public double[] estimatedPoseThetas = new double[0];
        public double[] timestamps = new double[0];
        public double[] stdDevXs = new double[0];
        public double[] stdDevYs = new double[0];
        public double[] stdDevThetas = new double[0];
    }

    public default void updateInputs(VisionIOInputs inputs, double yaw, double yawRate, boolean useMegaTag2) {}
}
