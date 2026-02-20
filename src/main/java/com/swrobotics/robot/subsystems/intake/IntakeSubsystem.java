package com.swrobotics.robot.subsystems.intake;

import com.swrobotics.robot.config.Constants;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import org.littletonrobotics.junction.Logger;

public class IntakeSubsystem extends SubsystemBase {

    public enum State {
        IDLE,
        INTAKE
    }

    private final IntakeIO io;
    private final IntakeIOInputsAutoLogged inputs = new IntakeIOInputsAutoLogged();
    private State targetState;

    public IntakeSubsystem(IntakeIO io) {
        this.io = io;
        targetState = State.IDLE;
    }

    @Override
    public void periodic() {
        io.updateInputs(inputs);
        Logger.processInputs("Intake", inputs);

        double targetRPS = 0;
        switch (targetState) {
            case INTAKE -> targetRPS = Constants.kIntakeRPS.get();
            case IDLE -> targetRPS = Constants.kIntakeIdleRPS.get();
        }

        io.setVelocity(targetRPS);

        Logger.recordOutput("Intake/State", targetState.name());
        Logger.recordOutput("Intake/TargetRPS", targetRPS);
    }

    public void setTargetState(State targetState) {
        this.targetState = targetState;
    }

    public Command commandSetState(State targetState) {
        return Commands.run(() -> setTargetState(targetState), this);
    }
}
