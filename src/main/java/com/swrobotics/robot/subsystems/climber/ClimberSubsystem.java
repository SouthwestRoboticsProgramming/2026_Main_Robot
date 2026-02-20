package com.swrobotics.robot.subsystems.climber;

import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.PositionVoltage;
import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import com.swrobotics.lib.net.NTBoolean;
import com.swrobotics.lib.net.NTEntry;
import com.swrobotics.robot.config.Constants;
import com.swrobotics.robot.config.IOAllocation;

import edu.wpi.first.math.filter.Debouncer;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import org.littletonrobotics.junction.Logger;

public final class ClimberSubsystem extends SubsystemBase {
    public enum State {
        RETRACTED,
        EXTENDED
    }

    private final TalonFX motor;
    private final StatusSignal<AngularVelocity> motorVelocity;

    private boolean hasCalibrated;
    private Debouncer calibrationDebounce;
    private State targetState;

    private final StatusSignal<Angle> motorPosition;

    private double targetPos;
    private double manualAdjust;

    private NTEntry<Boolean> calibrating = new NTBoolean("Climber/Calibrating?",true);

    public ClimberSubsystem() {
        TalonFXConfiguration config = new TalonFXConfiguration();
        config.MotorOutput.Inverted = InvertedValue.Clockwise_Positive;
        config.MotorOutput.NeutralMode = NeutralModeValue.Brake;
        config.Slot0.kP = 0;
        config.Slot0.kI = 0;
        config.Slot0.kD = 0;


        motor = IOAllocation.CAN.kClimberMotor.createTalonFX();
        motor.getConfigurator().apply(config);
        motorVelocity = motor.getVelocity();

        hasCalibrated = RobotBase.isSimulation();
        calibrationDebounce = null;
        targetState = State.RETRACTED;

        motorPosition = motor.getPosition();

        targetPos = 0;
        manualAdjust = 0;
    }

    public void setState(State state) {
        targetState = state;
        if (!hasCalibrated)
            return;

        double position = state == State.EXTENDED
                ? Constants.kClimberTall.get()
                : 0;

        if (state == State.EXTENDED)
            position += manualAdjust;

        motor.setControl(new PositionVoltage(position));
        targetPos = position;
    }

    @Override
    public void periodic() {
        motorPosition.refresh();

        if (DriverStation.isDisabled())
            return;

        if (!hasCalibrated) {
            if (calibrationDebounce == null) {
                calibrationDebounce = new Debouncer(
                        Constants.kClimberCalibrationTime.get(),
                        Debouncer.DebounceType.kBoth
                );
            }

            motorVelocity.refresh();
            double velocity = motorVelocity.getValueAsDouble();
            boolean reachedHardStop = Math.abs(velocity) < Constants.kClimberCalibrationVelocity.get();

            if (calibrationDebounce.calculate(reachedHardStop)) {
                motor.setPosition(Constants.kClimberCalibrationPosition.get());
                hasCalibrated = true;
                calibrating.set(false);

                setState(targetState);
            } else {
                calibrating.set(true);

                motor.setControl(new VoltageOut(-Constants.kClimberCalibrationVoltage.get()));
            }
        }

        Logger.recordOutput("Climber/State", targetState.name());
        Logger.recordOutput("Climber/IsCalibrating", !hasCalibrated);
        Logger.recordOutput("Climber/TargetPosition", targetPos);
        Logger.recordOutput("Climber/PositionRotations", motorPosition.getValueAsDouble());
        Logger.recordOutput("Climber/VelocityRPS", motorVelocity.getValueAsDouble());
    }

    public void recalibrate() {
        hasCalibrated = false;
        calibrationDebounce = null;
    }

    public TalonFX getMotor() {
        return motor;
    }

    public boolean isAtPosition() {
        // Intentionally big tolerance
        return Math.abs(motorPosition.getValueAsDouble() - targetPos) < 6;
    }

    public void applyManualAdjust(double adjust) {
        if (targetState == State.EXTENDED) {
            manualAdjust += adjust;
        }
    }

    public Command commandSetState(State targetState) {
        return Commands.run(() -> setState(targetState), this);
    }
}