package com.swrobotics.robot.subsystems.climber;

import com.swrobotics.robot.config.Constants;
import edu.wpi.first.math.filter.Debouncer;
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

    private final ClimberIO io;
    private final ClimberIOInputsAutoLogged inputs = new ClimberIOInputsAutoLogged();

    private boolean hasCalibrated;
    private Debouncer calibrationDebounce;
    private State targetState;

    private double targetPos;
    private double manualAdjust;

    public ClimberSubsystem(ClimberIO io) {
        this.io = io;

        hasCalibrated = RobotBase.isSimulation();
        calibrationDebounce = null;
        targetState = State.RETRACTED;

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

        io.setPosition(position);
        targetPos = position;
    }

    @Override
    public void periodic() {
        io.updateInputs(inputs);
        Logger.processInputs("Climber", inputs);

        if (DriverStation.isDisabled())
            return;

        if (!hasCalibrated) {
            if (calibrationDebounce == null) {
                calibrationDebounce = new Debouncer(
                        Constants.kClimberCalibrationTime.get(),
                        Debouncer.DebounceType.kBoth
                );
            }

            double velocity = inputs.velocityRPS;
            boolean reachedHardStop = Math.abs(velocity) < Constants.kClimberCalibrationVelocity.get();

            if (calibrationDebounce.calculate(reachedHardStop)) {
                io.zeroPosition(Constants.kClimberCalibrationPosition.get());
                hasCalibrated = true;

                setState(targetState);
            } else {
                io.setVoltage(-Constants.kClimberCalibrationVoltage.get());
            }
        }

        Logger.recordOutput("Climber/State", targetState.name());
        Logger.recordOutput("Climber/IsCalibrating", !hasCalibrated);
        Logger.recordOutput("Climber/TargetPosition", targetPos);
    }

    public void recalibrate() {
        hasCalibrated = false;
        calibrationDebounce = null;
    }

    public boolean isAtPosition() {
        return Math.abs(inputs.positionRotations - targetPos) < 6;
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
