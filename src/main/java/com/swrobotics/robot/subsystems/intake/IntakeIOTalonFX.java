package com.swrobotics.robot.subsystems.intake;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.controls.NeutralOut;
import com.ctre.phoenix6.controls.VelocityVoltage;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import com.swrobotics.lib.ctre.TalonFXConfigHelper;
import com.swrobotics.robot.config.IOAllocation;
import edu.wpi.first.units.measure.*;

public class IntakeIOTalonFX implements IntakeIO {
    private final TalonFX motor;
    private final VelocityVoltage velocityControl = new VelocityVoltage(0);

    private final StatusSignal<AngularVelocity> velocitySignal;
    private final StatusSignal<Voltage> appliedVoltsSignal;
    private final StatusSignal<Current> supplyCurrentSignal;
    private final StatusSignal<Current> statorCurrentSignal;
    private final StatusSignal<Temperature> tempSignal;

    public IntakeIOTalonFX() {
        motor = IOAllocation.CAN.kIntakeMotor.createTalonFX();

        TalonFXConfigHelper config = new TalonFXConfigHelper();
        config.MotorOutput.Inverted = InvertedValue.Clockwise_Positive;
        config.MotorOutput.NeutralMode = NeutralModeValue.Brake;

        config.Slot0.kP = 0.1;
        config.Slot0.kI = 0.0;
        config.Slot0.kD = 0.001;
        config.Slot0.kV = 0.12;

        config.apply(motor);

        velocitySignal = motor.getVelocity();
        appliedVoltsSignal = motor.getMotorVoltage();
        supplyCurrentSignal = motor.getSupplyCurrent();
        statorCurrentSignal = motor.getStatorCurrent();
        tempSignal = motor.getDeviceTemp();

        BaseStatusSignal.setUpdateFrequencyForAll(
                50.0,
                velocitySignal, appliedVoltsSignal, supplyCurrentSignal,
                statorCurrentSignal, tempSignal);
    }

    @Override
    public void updateInputs(IntakeIOInputs inputs) {
        BaseStatusSignal.refreshAll(
                velocitySignal, appliedVoltsSignal, supplyCurrentSignal,
                statorCurrentSignal, tempSignal);

        inputs.velocityRPS = velocitySignal.getValueAsDouble();
        inputs.appliedVolts = appliedVoltsSignal.getValueAsDouble();
        inputs.supplyCurrentAmps = supplyCurrentSignal.getValueAsDouble();
        inputs.statorCurrentAmps = statorCurrentSignal.getValueAsDouble();
        inputs.tempCelsius = tempSignal.getValueAsDouble();
    }

    @Override
    public void setVelocity(double rps) {
        motor.setControl(velocityControl.withVelocity(rps));
    }

    @Override
    public void stop() {
        motor.setControl(new NeutralOut());
    }
}
