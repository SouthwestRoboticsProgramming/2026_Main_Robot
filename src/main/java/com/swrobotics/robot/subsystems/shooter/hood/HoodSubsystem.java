package com.swrobotics.robot.subsystems.shooter.hood;

import com.ctre.phoenix6.controls.PositionVoltage;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import com.swrobotics.lib.ctre.TalonFXConfigHelper;
import com.swrobotics.robot.config.Constants;
import com.swrobotics.robot.config.IOAllocation;
import com.swrobotics.robot.control.AimCalc;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class HoodSubsystem extends SubsystemBase {
    private final TalonFX motor;
    private final PositionVoltage positionControl = new PositionVoltage(0.0);

    // State machine for control mode
    public enum HoodMode {
        AUTO_TRACK,
        MANUAL
    }

    private HoodMode mode = HoodMode.MANUAL;
    private double manualTargetRotations = 0.0;

    public HoodSubsystem() {
        motor = IOAllocation.CAN.kHoodMotor.createTalonFX();

        TalonFXConfigHelper config = new TalonFXConfigHelper();
        config.MotorOutput.Inverted =
                Constants.kHoodInverted.get()
                        ? InvertedValue.CounterClockwise_Positive
                        : InvertedValue.Clockwise_Positive;
        config.MotorOutput.NeutralMode = NeutralModeValue.Brake;
        config.CurrentLimits.StatorCurrentLimit = 20.0;
        config.CurrentLimits.StatorCurrentLimitEnable = true;
        config.addTunable(Constants.kHoodPID);

        config.apply(motor);
    }

    @Override
    public void periodic() {
        switch (mode) {
            case AUTO_TRACK -> runAutoTracking();
            case MANUAL -> runManualControl();
        }
    }

    private void runAutoTracking() {
        double maxAngle = Constants.kHoodMaxAngle.get();
        double minAngle = Constants.kHoodMinAngle.get();
        double maxRotations = maxAngle / 360.0;
        double minRotations = minAngle / 360.0;

        // Use AimCalc to get target angle, clamp to allowed range
        double targetRotations = AimCalc.getInstance().getHoodAngle().getRotations();
        targetRotations = Math.max(minRotations, Math.min(maxRotations, targetRotations));

        motor.setControl(positionControl.withPosition(targetRotations));
    }

    private void runManualControl() {
        motor.setControl(positionControl.withPosition(manualTargetRotations));
    }

    // External controls
    public Command setManualPosition(double rotations) {
        return Commands.runOnce(() -> {
            manualTargetRotations = rotations;
        });
    }

    public Command setMode(HoodMode newMode) {
        return Commands.runOnce(() -> {
            mode = newMode;
        });
    }

    public HoodMode getMode() {
        return mode;
    }
}
