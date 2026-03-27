package com.swrobotics.robot.logging;

import com.swrobotics.robot.control.AimCalc;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj.smartdashboard.Mechanism2d;
import edu.wpi.first.wpilibj.smartdashboard.MechanismLigament2d;
import edu.wpi.first.wpilibj.smartdashboard.MechanismRoot2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj.util.Color8Bit;

public final class RobotView {
    // 3x3 grid for the Mechanism2d viewer
    private static final Mechanism2d mechanism = new Mechanism2d(3, 3);

    // --- Top-Down Aiming Visualization ---
    // Root at the center of the canvas (1.5, 1.5)
    private static final MechanismRoot2d topDownRoot = mechanism.getRoot("Drivebase", 1.5, 1.5);
    
    // Red line representing the robot's current heading
    private static final MechanismLigament2d currentHeadingLigament = topDownRoot.append(
            new MechanismLigament2d("Current Heading", 0.5, 0, 6, new Color8Bit(Color.kRed))
    );
    
    // Green line representing the target angle to the hub
    private static final MechanismLigament2d targetAimLigament = topDownRoot.append(
            new MechanismLigament2d("Hub Target Aim", 1.25, 0, 4, new Color8Bit(Color.kGreen))
    );

    // --- Side-Profile Shooter Visualization ---
    // Root in the bottom left corner
    private static final MechanismRoot2d shooterRoot = mechanism.getRoot("Shooter Side View", 0.5, 0.2);
    
    // Blue line representing the interpolated hood angle
    private static final MechanismLigament2d hoodLigament = shooterRoot.append(
            new MechanismLigament2d("Hood Angle", 0.6, 30, 8, new Color8Bit(Color.kBlue))
    );

    /**
     * Publishes the Mechanism2d to SmartDashboard/AdvantageScope
     */
    public static void publish() {
        SmartDashboard.putData("Robot View", mechanism);
    }

    /**
     * Updates the Mechanism2d ligaments with live data.
     * Call this in your robot's periodic loop (e.g., RobotPeriodic).
     * * @param currentPose The current estimated pose of the robot.
     */
    public static void update(Pose2d currentPose) {
        // 1. Update the robot's actual rotation
        currentHeadingLigament.setAngle(currentPose.getRotation().getDegrees());

        // 2. Update the target vector calculated by AimCalc
        targetAimLigament.setAngle(AimCalc.getInstance().getDrivebaseAimAngle().getDegrees());

        // 3. Update the shooter hood angle
        hoodLigament.setAngle(AimCalc.getInstance().getHoodAngle().getDegrees());
    }
}