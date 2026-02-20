package com.swrobotics.robot.subsystems.indexer;

import com.swrobotics.robot.config.Constants;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import org.littletonrobotics.junction.Logger;

public class IndexerSubsystem extends SubsystemBase {

    public enum State {
        IDLE,
        INTAKE,
        HOLDING_BALL
    }

    private final IndexerIO io;
    private final IndexerIOInputsAutoLogged inputs = new IndexerIOInputsAutoLogged();

    private boolean ballAtTop = false;
    private State targetState;

    public IndexerSubsystem(IndexerIO io) {
        this.io = io;
        targetState = State.IDLE;
    }

    @Override
    public void periodic() {
        io.updateInputs(inputs);
        Logger.processInputs("Indexer", inputs);

        ballAtTop = inputs.ballDetected;

        // If we're intaking and now see a ball, go to HOLDING_BALL
        if (targetState == State.INTAKE && ballAtTop) {
            targetState = State.HOLDING_BALL;
        }

        double motor1RPS = 0.0;
        double motor2RPS = 0.0;

        switch (targetState) {
            case INTAKE -> {
                motor1RPS = Constants.kIndexerRollRPS.get();
                motor2RPS = Constants.kIndexerRollRPS.get();
            }
            case HOLDING_BALL -> {
                motor1RPS = Constants.kIndexerHoldRPS.get();
                motor2RPS = Constants.kIndexerIdleRPS.get();
            }
            case IDLE -> {
                motor1RPS = Constants.kIndexerIdleRPS.get();
                motor2RPS = Constants.kIndexerIdleRPS.get();
            }
        }

        io.setMotor1Velocity(motor1RPS);
        io.setMotor2Velocity(motor2RPS);

        Logger.recordOutput("Indexer/State", targetState.name());
        Logger.recordOutput("Indexer/Motor1TargetRPS", motor1RPS);
        Logger.recordOutput("Indexer/Motor2TargetRPS", motor2RPS);
        Logger.recordOutput("Indexer/BallAtTop", ballAtTop);
    }

    public void setTargetState(State targetState) {
        this.targetState = targetState;
    }

    public Command commandSetState(State state) {
        return Commands.runOnce(() -> setTargetState(state), this);
    }

    public boolean isBallAtTop() {
        return ballAtTop;
    }

    public Command commandIntakeUntilBall() {
        return Commands.run(
                () -> setTargetState(State.INTAKE),
                this
        ).until(() -> ballAtTop)
         .finallyDo(interrupted -> setTargetState(State.HOLDING_BALL));
    }
}
