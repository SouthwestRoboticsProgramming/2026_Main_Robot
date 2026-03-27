package com.swrobotics.robot.subsystems.shooter.hood;

import com.ctre.phoenix6.controls.PositionVoltage;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import com.swrobotics.lib.ctre.TalonFXConfigHelper;
import com.swrobotics.robot.config.Constants;
import com.swrobotics.robot.config.IOAllocation;
import com.swrobotics.robot.control.AimCalc;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class HoodSubsystem extends SubsystemBase {
    private final TalonFX motor;
    private final PositionVoltage positionControl = new PositionVoltage(0.0).withEnableFOC(true);

    public enum HoodMode { AUTO_TRACK, MANUAL }

    private HoodMode mode = HoodMode.MANUAL;
    private double targetRotations = 0.0;

    public HoodSubsystem() {
        motor = IOAllocation.CAN.kHoodMotor.createTalonFX();

        TalonFXConfigHelper config = new TalonFXConfigHelper();
        config.MotorOutput.Inverted = Constants.kHoodInverted.get()
                ? InvertedValue.CounterClockwise_Positive
                : InvertedValue.Clockwise_Positive;
        config.MotorOutput.NeutralMode = NeutralModeValue.Brake;
        config.CurrentLimits.StatorCurrentLimit = 20.0;
        config.CurrentLimits.StatorCurrentLimitEnable = true;

        // config.Feedback.SensorToMechanismRatio = ...; // TUNE: set your gear ratio here

        config.addTunable(Constants.kHoodPID);
        config.apply(motor);
    }

    @Override
    public void periodic() {
        if (mode == HoodMode.AUTO_TRACK) {
            double maxRotations = Constants.kHoodMaxAngle.get() / 360.0;
            double minRotations = Constants.kHoodMinAngle.get() / 360.0;

            // AimCalc hood angle is in degrees; convert to rotations
            double hoodRot = AimCalc.getInstance().getHoodAngle().getRotations();
            targetRotations = Math.max(minRotations, Math.min(maxRotations, hoodRot));
        }

        motor.setControl(positionControl.withPosition(targetRotations));
        SmartDashboard.putBoolean("Shooter/Hood At Target", isAtTarget());
        SmartDashboard.putNumber("Shooter/Hood Target Rot", targetRotations);
        SmartDashboard.putNumber("Shooter/Hood Actual Rot", motor.getPosition().getValueAsDouble());
    }

    public boolean isAtTarget() {
        // Tolerance: within 0.5 degrees
        double toleranceRot = 0.5 / 360.0;
        return Math.abs(motor.getPosition().getValueAsDouble() - targetRotations) < toleranceRot;
    }

    // Manual absolute set
    public Command setManualPosition(double rotations) {
        return Commands.runOnce(() -> {
            targetRotations = rotations;
            mode = HoodMode.MANUAL;
        }, this);
    }

    public Command setMode(HoodMode newMode) {
        return Commands.runOnce(() -> mode = newMode, this);
    }

    // ------------- MANUAL NUDGE (±2°) -------------
    public void nudgeAngleDegrees(double deltaDeg) {
        double deltaRot = deltaDeg / 360.0;
        targetRotations += deltaRot;

        double maxRotations = Constants.kHoodMaxAngle.get() / 360.0;
        double minRotations = Constants.kHoodMinAngle.get() / 360.0;
        targetRotations = Math.max(minRotations, Math.min(maxRotations, targetRotations));

        mode = HoodMode.MANUAL;
    }

    public Command commandNudgeAngleDegrees(double deltaDeg) {
        return Commands.runOnce(() -> nudgeAngleDegrees(deltaDeg), this);
    }
}