package com.swrobotics.robot.subsystems.intake.expansions;

import com.swrobotics.robot.config.Constants;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import org.littletonrobotics.junction.Logger;

public class ExpansionSubsystem extends SubsystemBase {

    public enum State {
        RETRACTED,
        EXTENDED
    }

    private final ExpansionIO io;
    private final ExpansionIOInputsAutoLogged inputs = new ExpansionIOInputsAutoLogged();
    private State targetState;

    public ExpansionSubsystem(ExpansionIO io) {
        this.io = io;
        targetState = State.RETRACTED;
    }

    @Override
    public void periodic() {
        io.updateInputs(inputs);
        Logger.processInputs("Expansion", inputs);

        double targetRotations;
        switch (targetState) {
            case EXTENDED:
                targetRotations = Constants.kExpansionExtendedRotations.get();
                break;
            case RETRACTED:
            default:
                targetRotations = Constants.kExpansionRetractedRotations.get();
                break;
        }

        io.setPosition(targetRotations);

        Logger.recordOutput("Expansion/State", targetState.name());
        Logger.recordOutput("Expansion/TargetRotations", targetRotations);
    }

    public void setTargetState(State targetState) {
        this.targetState = targetState;
    }

    public Command commandSetState(State targetState) {
        return Commands.runOnce(() -> setTargetState(targetState), this);
    }
}
