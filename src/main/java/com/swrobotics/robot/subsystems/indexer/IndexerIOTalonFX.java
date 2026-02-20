package com.swrobotics.robot.subsystems.indexer;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.controls.NeutralOut;
import com.ctre.phoenix6.controls.VelocityVoltage;
import com.ctre.phoenix6.hardware.CANrange;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import com.swrobotics.lib.ctre.TalonFXConfigHelper;
import com.swrobotics.robot.config.IOAllocation;
import edu.wpi.first.units.measure.*;

public class IndexerIOTalonFX implements IndexerIO {
    private final TalonFX motor1;
    private final TalonFX motor2;
    private final CANrange canrange;
    private final VelocityVoltage velocityControl = new VelocityVoltage(0);

    private final StatusSignal<AngularVelocity> motor1Velocity;
    private final StatusSignal<AngularVelocity> motor2Velocity;
    private final StatusSignal<Voltage> motor1AppliedVolts;
    private final StatusSignal<Voltage> motor2AppliedVolts;
    private final StatusSignal<Current> motor1Current;
    private final StatusSignal<Current> motor2Current;
    private final StatusSignal<Temperature> motor1Temp;
    private final StatusSignal<Temperature> motor2Temp;

    public IndexerIOTalonFX() {
        motor1 = IOAllocation.CAN.kIndexerMotor.createTalonFX();
        motor2 = IOAllocation.CAN.kIndexerMotor2.createTalonFX();
        canrange = IOAllocation.CAN.kIndexerCANrange.createCANrange();

        TalonFXConfigHelper config = new TalonFXConfigHelper();
        config.MotorOutput.Inverted = InvertedValue.Clockwise_Positive;
        config.MotorOutput.NeutralMode = NeutralModeValue.Brake;

        config.Slot0.kP = 0.1;
        config.Slot0.kI = 0.0;
        config.Slot0.kD = 0.001;
        config.Slot0.kV = 0.12;

        config.apply(motor1, motor2);

        motor1Velocity = motor1.getVelocity();
        motor2Velocity = motor2.getVelocity();
        motor1AppliedVolts = motor1.getMotorVoltage();
        motor2AppliedVolts = motor2.getMotorVoltage();
        motor1Current = motor1.getSupplyCurrent();
        motor2Current = motor2.getSupplyCurrent();
        motor1Temp = motor1.getDeviceTemp();
        motor2Temp = motor2.getDeviceTemp();

        BaseStatusSignal.setUpdateFrequencyForAll(
                50.0,
                motor1Velocity, motor2Velocity,
                motor1AppliedVolts, motor2AppliedVolts,
                motor1Current, motor2Current,
                motor1Temp, motor2Temp);
    }

    @Override
    public void updateInputs(IndexerIOInputs inputs) {
        BaseStatusSignal.refreshAll(
                motor1Velocity, motor2Velocity,
                motor1AppliedVolts, motor2AppliedVolts,
                motor1Current, motor2Current,
                motor1Temp, motor2Temp);

        inputs.motor1VelocityRPS = motor1Velocity.getValueAsDouble();
        inputs.motor2VelocityRPS = motor2Velocity.getValueAsDouble();
        inputs.motor1AppliedVolts = motor1AppliedVolts.getValueAsDouble();
        inputs.motor2AppliedVolts = motor2AppliedVolts.getValueAsDouble();
        inputs.motor1CurrentAmps = motor1Current.getValueAsDouble();
        inputs.motor2CurrentAmps = motor2Current.getValueAsDouble();
        inputs.motor1TempCelsius = motor1Temp.getValueAsDouble();
        inputs.motor2TempCelsius = motor2Temp.getValueAsDouble();
        inputs.ballDetected = canrange.getIsDetected().getValue();
    }

    @Override
    public void setMotor1Velocity(double rps) {
        motor1.setControl(velocityControl.withVelocity(rps));
    }

    @Override
    public void setMotor2Velocity(double rps) {
        motor2.setControl(velocityControl.withVelocity(rps));
    }

    @Override
    public void stop() {
        motor1.setControl(new NeutralOut());
        motor2.setControl(new NeutralOut());
    }
}
