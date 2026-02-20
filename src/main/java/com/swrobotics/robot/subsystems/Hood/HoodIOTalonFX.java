package com.swrobotics.robot.subsystems.Hood;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.CANcoderConfiguration;
import com.ctre.phoenix6.configs.MotionMagicConfigs;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.controls.MotionMagicVoltage;
import com.ctre.phoenix6.controls.NeutralOut;
import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import com.ctre.phoenix6.signals.SensorDirectionValue;
import com.swrobotics.lib.ctre.TalonFXConfigHelper;
import com.swrobotics.robot.config.Constants;
import com.swrobotics.robot.config.IOAllocation;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.units.measure.*;

public class HoodIOTalonFX implements HoodIO {
    private final TalonFX motor;
    private final CANcoder encoder;
    private final MotionMagicVoltage motionMagic = new MotionMagicVoltage(0).withSlot(0);

    private final StatusSignal<Angle> positionSignal;
    private final StatusSignal<AngularVelocity> velocitySignal;
    private final StatusSignal<Voltage> appliedVoltsSignal;
    private final StatusSignal<Current> supplyCurrentSignal;
    private final StatusSignal<Current> statorCurrentSignal;
    private final StatusSignal<Temperature> tempSignal;
    private final StatusSignal<Angle> absolutePositionSignal;

    public HoodIOTalonFX() {
        motor = IOAllocation.CAN.kHoodMotor.createTalonFX();
        encoder = IOAllocation.CAN.kHoodCANcoder.createCANcoder();

        TalonFXConfigHelper config = new TalonFXConfigHelper();
        CANcoderConfiguration encoderConfig = new CANcoderConfiguration();
        config.MotorOutput.Inverted =
                Constants.kHoodInverted.get()
                        ? InvertedValue.CounterClockwise_Positive
                        : InvertedValue.Clockwise_Positive;
        config.MotorOutput.NeutralMode = NeutralModeValue.Brake;

        encoderConfig.MagnetSensor.MagnetOffset = 0;
        encoderConfig.MagnetSensor.SensorDirection = SensorDirectionValue.CounterClockwise_Positive;

        config.Feedback.SensorToMechanismRatio = 1 / 1;

        Slot0Configs gains = new Slot0Configs();
        gains.withKP(6.0);
        gains.withKI(0.0);
        gains.withKD(0.8);
        gains.withKV(0.1);
        config.Slot0 = gains;

        MotionMagicConfigs mmConfig = new MotionMagicConfigs();
        mmConfig.MotionMagicCruiseVelocity = Constants.kHoodCruiseVelocity.get();
        mmConfig.MotionMagicAcceleration = Constants.kHoodAcceleration.get();
        mmConfig.MotionMagicJerk = 2000.0;
        config.MotionMagic = mmConfig;

        config.apply(motor);
        encoder.getConfigurator().apply(encoderConfig);

        // Seed motor position from absolute encoder
        double absolutePos = encoder.getAbsolutePosition().getValueAsDouble();
        motor.setPosition(absolutePos);

        positionSignal = motor.getPosition();
        velocitySignal = motor.getVelocity();
        appliedVoltsSignal = motor.getMotorVoltage();
        supplyCurrentSignal = motor.getSupplyCurrent();
        statorCurrentSignal = motor.getStatorCurrent();
        tempSignal = motor.getDeviceTemp();
        absolutePositionSignal = encoder.getAbsolutePosition();

        BaseStatusSignal.setUpdateFrequencyForAll(
                50.0,
                positionSignal, velocitySignal, appliedVoltsSignal,
                supplyCurrentSignal, statorCurrentSignal, tempSignal,
                absolutePositionSignal);
    }

    @Override
    public void updateInputs(HoodIOInputs inputs) {
        BaseStatusSignal.refreshAll(
                positionSignal, velocitySignal, appliedVoltsSignal,
                supplyCurrentSignal, statorCurrentSignal, tempSignal,
                absolutePositionSignal);

        inputs.positionDeg = Units.rotationsToDegrees(positionSignal.getValueAsDouble());
        inputs.velocityDegPerSec = Units.rotationsToDegrees(velocitySignal.getValueAsDouble());
        inputs.appliedVolts = appliedVoltsSignal.getValueAsDouble();
        inputs.supplyCurrentAmps = supplyCurrentSignal.getValueAsDouble();
        inputs.statorCurrentAmps = statorCurrentSignal.getValueAsDouble();
        inputs.tempCelsius = tempSignal.getValueAsDouble();
        inputs.absolutePositionDeg = Units.rotationsToDegrees(absolutePositionSignal.getValueAsDouble());
    }

    @Override
    public void setPosition(double motorRotations) {
        motor.setControl(motionMagic.withPosition(motorRotations));
    }

    @Override
    public void stop() {
        motor.setControl(new NeutralOut());
    }

    @Override
    public void seedPosition(double rotations) {
        motor.setPosition(rotations);
    }
}
