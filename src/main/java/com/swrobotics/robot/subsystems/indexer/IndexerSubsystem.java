package com.swrobotics.robot.subsystems.indexer;

// CTRE Phoenix 6 imports
import com.ctre.phoenix6.controls.VelocityVoltage;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.hardware.CANrange;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;

// SW Robotics imports
import com.swrobotics.lib.ctre.TalonFXConfigHelper;
import com.swrobotics.robot.config.Constants;
import com.swrobotics.robot.config.IOAllocation;

// WPILib imports
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import org.littletonrobotics.junction.Logger;

public class IndexerSubsystem extends SubsystemBase {

    // States for the indexer
    public enum State {
        IDLE,
        INTAKE,
        HOLDING_BALL // ball at top, upper stage stopped
    }

    // Motors and control
    private final TalonFX motor;   // lower stage (keeps creeping)
    private final TalonFX motor2;  // upper stage (stops when ball present)
    private final VelocityVoltage velocityControl = new VelocityVoltage(0);

    // CANrange sensor at top of tube
    private final CANrange canrange;

    // Ball detection flag
    private boolean ballAtTop = false;

    // Current state
    private State targetState;

    // Constructor
    public IndexerSubsystem() {
        // Initialize motors
        motor = IOAllocation.CAN.kIndexerMotor.createTalonFX();
        motor2 = IOAllocation.CAN.kIndexerMotor2.createTalonFX();

        // Configure motors
        TalonFXConfigHelper config = new TalonFXConfigHelper();

        config.MotorOutput.Inverted = InvertedValue.Clockwise_Positive;
        config.MotorOutput.NeutralMode = NeutralModeValue.Brake;

        // PID/FF gains (tune these)
        config.Slot0.kP = 0.1;
        config.Slot0.kI = 0.0;
        config.Slot0.kD = 0.001;
        config.Slot0.kV = 0.12;

        // Apply configuration to both motors
        config.apply(motor, motor2);

        // Create CANrange (adjust to your actual IO allocation)
        canrange = IOAllocation.CAN.kIndexerCANrange.createCANrange();

        // Initial state
        targetState = State.IDLE;
    }

    @Override
    public void periodic() {
        // Read CANrange: true when a ball is at the top
        boolean detected = canrange.getIsDetected().getValue();
        ballAtTop = detected;

        // If we’re intaking and now see a ball, go to HOLDING_BALL
        if (targetState == State.INTAKE && ballAtTop) {
            targetState = State.HOLDING_BALL;
        }

        // Decide RPS for each motor independently
        double motor1RPS = 0.0; // lower-stage
        double motor2RPS = 0.0; // upper-stage

        switch (targetState) {
            case INTAKE -> {
                // Normal intake speed for both motors
                motor1RPS = Constants.kIndexerRollRPS.get();
                motor2RPS = Constants.kIndexerRollRPS.get();
            }
            case HOLDING_BALL -> {
                // Lower motor creeps slowly to keep feeding / manage queue
                motor1RPS = Constants.kIndexerHoldRPS.get(); // slower, new constant
                // Upper motor stopped (or small idle)
                motor2RPS = Constants.kIndexerIdleRPS.get(); // usually 0
            }
            case IDLE -> {
                // Both motors stopped/idle
                motor1RPS = Constants.kIndexerIdleRPS.get();
                motor2RPS = Constants.kIndexerIdleRPS.get();
            }
        }

        // Apply controls
        motor.setControl(velocityControl.withVelocity(motor1RPS));
        motor2.setControl(velocityControl.withVelocity(motor2RPS));

        Logger.recordOutput("Indexer/State", targetState.name());
        Logger.recordOutput("Indexer/Motor1TargetRPS", motor1RPS);
        Logger.recordOutput("Indexer/Motor2TargetRPS", motor2RPS);
        Logger.recordOutput("Indexer/Motor1VelocityRPS", motor.getVelocity().getValueAsDouble());
        Logger.recordOutput("Indexer/Motor2VelocityRPS", motor2.getVelocity().getValueAsDouble());
        Logger.recordOutput("Indexer/BallDetected", ballAtTop);
    }

    // State setter
    public void setTargetState(State targetState) {
        this.targetState = targetState;
    }

    // Command to set a state once
    public Command commandSetState(State state) {
        return Commands.runOnce(() -> setTargetState(state), this);
    }

    // For shooter or higher-level logic to query
    public boolean isBallAtTop() {
        return ballAtTop;
    }

    // Convenience command: intake until a ball is detected, then hold
    public Command commandIntakeUntilBall() {
        return Commands.run(
                () -> setTargetState(State.INTAKE),
                this
        ).until(() -> ballAtTop)
         .finallyDo(interrupted -> setTargetState(State.HOLDING_BALL));
    }
}
