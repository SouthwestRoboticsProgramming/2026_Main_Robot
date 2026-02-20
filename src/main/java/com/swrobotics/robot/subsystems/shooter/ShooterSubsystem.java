package com.swrobotics.robot.subsystems.shooter;

import com.swrobotics.robot.config.Constants;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import org.littletonrobotics.junction.Logger;

public class ShooterSubsystem extends SubsystemBase {

    public enum State {
        IDLE,
        SHOOT
    }

    private final ShooterIO io;
    private final ShooterIOInputsAutoLogged inputs = new ShooterIOInputsAutoLogged();
    private State targetState;

    public ShooterSubsystem(ShooterIO io) {
        this.io = io;
        targetState = State.IDLE;
    }

    @Override
    public void periodic() {
        io.updateInputs(inputs);
        Logger.processInputs("Shooter", inputs);

        double targetRPS = 0;
        switch (targetState) {
            case SHOOT -> targetRPS = Constants.kShooterRPS.get();
            case IDLE -> targetRPS = Constants.kShooterIdleRPS.get();
        }

        io.setVelocity(targetRPS);

        Logger.recordOutput("Shooter/State", targetState.name());
        Logger.recordOutput("Shooter/TargetRPS", targetRPS);
    }

    public void setTargetState(State targetState) {
        this.targetState = targetState;
    }

    public Command commandSetState(State targetState) {
        return Commands.run(() -> setTargetState(targetState), this);
    }
}
