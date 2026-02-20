package com.swrobotics.robot.subsystems.climber;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.NeutralOut;
import com.ctre.phoenix6.controls.PositionVoltage;
import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import com.swrobotics.robot.config.IOAllocation;
import edu.wpi.first.units.measure.*;

public class ClimberIOTalonFX implements ClimberIO {
    private final TalonFX motor;

    private final StatusSignal<Angle> positionSignal;
    private final StatusSignal<AngularVelocity> velocitySignal;
    private final StatusSignal<Voltage> appliedVoltsSignal;
    private final StatusSignal<Current> supplyCurrentSignal;
    private final StatusSignal<Current> statorCurrentSignal;
    private final StatusSignal<Temperature> tempSignal;

    public ClimberIOTalonFX() {
        motor = IOAllocation.CAN.kClimberMotor.createTalonFX();

        TalonFXConfiguration config = new TalonFXConfiguration();
        config.MotorOutput.Inverted = InvertedValue.Clockwise_Positive;
        config.MotorOutput.NeutralMode = NeutralModeValue.Brake;
        config.Slot0.kP = 0;
        config.Slot0.kI = 0;
        config.Slot0.kD = 0;

        motor.getConfigurator().apply(config);

        positionSignal = motor.getPosition();
        velocitySignal = motor.getVelocity();
        appliedVoltsSignal = motor.getMotorVoltage();
        supplyCurrentSignal = motor.getSupplyCurrent();
        statorCurrentSignal = motor.getStatorCurrent();
        tempSignal = motor.getDeviceTemp();

        BaseStatusSignal.setUpdateFrequencyForAll(
                50.0,
                positionSignal, velocitySignal, appliedVoltsSignal,
                supplyCurrentSignal, statorCurrentSignal, tempSignal);
    }

    @Override
    public void updateInputs(ClimberIOInputs inputs) {
        BaseStatusSignal.refreshAll(
                positionSignal, velocitySignal, appliedVoltsSignal,
                supplyCurrentSignal, statorCurrentSignal, tempSignal);

        inputs.positionRotations = positionSignal.getValueAsDouble();
        inputs.velocityRPS = velocitySignal.getValueAsDouble();
        inputs.appliedVolts = appliedVoltsSignal.getValueAsDouble();
        inputs.supplyCurrentAmps = supplyCurrentSignal.getValueAsDouble();
        inputs.statorCurrentAmps = statorCurrentSignal.getValueAsDouble();
        inputs.tempCelsius = tempSignal.getValueAsDouble();
    }

    @Override
    public void setPosition(double rotations) {
        motor.setControl(new PositionVoltage(rotations));
    }

    @Override
    public void setVoltage(double volts) {
        motor.setControl(new VoltageOut(volts));
    }

    @Override
    public void stop() {
        motor.setControl(new NeutralOut());
    }

    @Override
    public void zeroPosition(double position) {
        motor.setPosition(position);
    }
}
