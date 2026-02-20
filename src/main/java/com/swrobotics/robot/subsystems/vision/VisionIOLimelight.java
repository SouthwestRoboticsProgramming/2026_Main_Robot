package com.swrobotics.robot.subsystems.vision;

import edu.wpi.first.math.geometry.Pose2d;

import java.util.ArrayList;
import java.util.List;

public class VisionIOLimelight implements VisionIO {
    private final List<LimelightCamera> cameras;

    public VisionIOLimelight(List<LimelightCamera> cameras) {
        this.cameras = cameras;
    }

    @Override
    public void updateInputs(VisionIOInputs inputs, double yaw, double yawRate, boolean useMegaTag2) {
        inputs.connected = !cameras.isEmpty();

        for (LimelightCamera camera : cameras) {
            camera.updateRobotState(yaw, yawRate);
        }

        List<LimelightCamera.Update> updates = new ArrayList<>();
        for (LimelightCamera camera : cameras) {
            camera.getNewUpdates(updates, useMegaTag2);
        }

        inputs.estimatedPoseXs = new double[updates.size()];
        inputs.estimatedPoseYs = new double[updates.size()];
        inputs.estimatedPoseThetas = new double[updates.size()];
        inputs.timestamps = new double[updates.size()];
        inputs.stdDevXs = new double[updates.size()];
        inputs.stdDevYs = new double[updates.size()];
        inputs.stdDevThetas = new double[updates.size()];

        for (int i = 0; i < updates.size(); i++) {
            LimelightCamera.Update update = updates.get(i);
            Pose2d pose = update.pose();
            inputs.estimatedPoseXs[i] = pose.getX();
            inputs.estimatedPoseYs[i] = pose.getY();
            inputs.estimatedPoseThetas[i] = pose.getRotation().getDegrees();
            inputs.timestamps[i] = update.timestamp();
            inputs.stdDevXs[i] = update.stdDevs().get(0, 0);
            inputs.stdDevYs[i] = update.stdDevs().get(1, 0);
            inputs.stdDevThetas[i] = update.stdDevs().get(2, 0);
        }
    }
}
