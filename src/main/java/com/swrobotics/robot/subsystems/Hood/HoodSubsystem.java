package com.swrobotics.robot.subsystems.hood;

import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.controls.MotionMagicVoltage;
import com.ctre.phoenix6.controls.PositionVoltage;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import com.ctre.phoenix6.signals.SensorDirectionValue;

import com.swrobotics.lib.ctre.TalonFXConfigHelper;
import com.swrobotics.robot.config.Constants;
import com.swrobotics.robot.config.IOAllocation;
import com.swrobotics.robot.control.AimCalc;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.util.Units;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class HoodSubsystem extends SubsystemBase {
    private final TalonFX motor;
    private final PositionVoltage positionControl = new PositionVoltage(0.0);

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
        motor.setPosition(0);
    }

    @Override
    public void periodic() {

        double maxAngle = Constants.kHoodMaxAngle.get();
        double minAngle = Constants.kHoodMinAngle.get();
        double maxRotations = maxAngle / 360.0;
        double minRotations = minAngle / 360.0;

        double targetRotations = AimCalc.getInstance().getHoodAngle().getRotations();
        targetRotations = Math.max(minRotations, Math.min(maxRotations, targetRotations));

        motor.setControl(positionControl.withPosition(targetRotations));
    }
}