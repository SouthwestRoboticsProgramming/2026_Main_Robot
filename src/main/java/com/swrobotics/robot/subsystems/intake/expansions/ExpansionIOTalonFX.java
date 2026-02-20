package com.swrobotics.robot.subsystems.intake.expansions;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.MotionMagicConfigs;
import com.ctre.phoenix6.controls.MotionMagicVoltage;
import com.ctre.phoenix6.controls.NeutralOut;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import com.swrobotics.lib.ctre.TalonFXConfigHelper;
import com.swrobotics.robot.config.Constants;
import com.swrobotics.robot.config.IOAllocation;
import edu.wpi.first.units.measure.*;

public class ExpansionIOTalonFX implements ExpansionIO {
    private final TalonFX motor;
    private final MotionMagicVoltage motionMagic = new MotionMagicVoltage(0).withSlot(0);

    private final StatusSignal<Angle> positionSignal;
    private final StatusSignal<AngularVelocity> velocitySignal;
    private final StatusSignal<Voltage> appliedVoltsSignal;
    private final StatusSignal<Current> supplyCurrentSignal;
    private final StatusSignal<Current> statorCurrentSignal;
    private final StatusSignal<Temperature> tempSignal;

    public ExpansionIOTalonFX() {
        motor = IOAllocation.CAN.kExpansionMotor.createTalonFX();

        TalonFXConfigHelper config = new TalonFXConfigHelper();
        config.MotorOutput.Inverted = InvertedValue.Clockwise_Positive;
        config.MotorOutput.NeutralMode = NeutralModeValue.Brake;

        config.Slot0.kP = 0.1;
        config.Slot0.kI = 0.0;
        config.Slot0.kD = 0.001;

        MotionMagicConfigs mm = new MotionMagicConfigs();
        mm.MotionMagicCruiseVelocity = Constants.kExpansionCruiseVelocity.get();
        mm.MotionMagicAcceleration = Constants.kExpansionAcceleration.get();
        config.MotionMagic = mm;

        config.apply(motor);

        // Zero the position at startup
        motor.setPosition(0);

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
    public void updateInputs(ExpansionIOInputs inputs) {
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
        motor.setControl(motionMagic.withPosition(rotations));
    }

    @Override
    public void stop() {
        motor.setControl(new NeutralOut());
    }

    @Override
    public void zeroPosition() {
        motor.setPosition(0);
    }
}
