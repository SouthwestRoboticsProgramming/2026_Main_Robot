package com.swrobotics.robot.subsystems.vision;

import com.swrobotics.robot.config.Constants;
import com.swrobotics.robot.logging.FieldView;
import com.swrobotics.robot.subsystems.swerve.SwerveDriveSubsystem;

import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import org.littletonrobotics.junction.Logger;

public final class VisionSubsystem extends SubsystemBase {
    private final SwerveDriveSubsystem drive;
    private final VisionIO io;
    private final VisionIOInputsAutoLogged inputs = new VisionIOInputsAutoLogged();

    private boolean ignoreUpdates;

    public VisionSubsystem(SwerveDriveSubsystem drive, VisionIO io) {
        this.drive = drive;
        this.io = io;

        ignoreUpdates = false;
        setDefaultCommand(Commands.run(() -> ignoreUpdates = false, this));
    }

    public Command commandIgnoreUpdates() {
        return Commands.run(() -> ignoreUpdates = true, this);
    }

    @Override
    public void periodic() {
        Pose2d currentPose = drive.getEstimatedPose();
        ChassisSpeeds currentSpeeds = drive.getRobotRelativeSpeeds();

        double yaw = currentPose.getRotation().getDegrees();
        double yawRate = Math.toDegrees(currentSpeeds.omegaRadiansPerSecond);

        boolean useMegaTag2 = Math.hypot(currentSpeeds.vxMetersPerSecond, currentSpeeds.vyMetersPerSecond)
                > Constants.kVisionMT2SpeedThreshold;

        io.updateInputs(inputs, yaw, yawRate, useMegaTag2);
        Logger.processInputs("Vision", inputs);

        // Reconstruct poses for FieldView
        Pose2d[] poses = new Pose2d[inputs.estimatedPoseXs.length];
        for (int i = 0; i < poses.length; i++) {
            poses[i] = new Pose2d(
                    inputs.estimatedPoseXs[i],
                    inputs.estimatedPoseYs[i],
                    Rotation2d.fromDegrees(inputs.estimatedPoseThetas[i]));
        }
        FieldView.visionEstimates.setPoses(poses);

        if (ignoreUpdates)
            return;

        for (int i = 0; i < inputs.estimatedPoseXs.length; i++) {
            Pose2d pose = new Pose2d(
                    inputs.estimatedPoseXs[i],
                    inputs.estimatedPoseYs[i],
                    Rotation2d.fromDegrees(inputs.estimatedPoseThetas[i]));
            drive.addVisionMeasurement(
                    pose,
                    inputs.timestamps[i],
                    VecBuilder.fill(inputs.stdDevXs[i], inputs.stdDevYs[i], inputs.stdDevThetas[i]));
        }
    }
}
