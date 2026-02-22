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

public class IndexerSubsystem extends SubsystemBase {

    // States for the indexer
    public enum State {
        IDLE,
        INTAKE,
        HOLDING_BALL, // ball at top, upper stage stopped
        IGNORE,
        RINDEX // optional: run in reverse to clear jams, etc.
    }

    // Motors and control
    private final TalonFX FloorMotor;   // lower stage (keeps creeping)
    private final TalonFX ShooterFeederMotor;  // upper stage (stops when ball present)
    private final TalonFX BeltMotor;
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
        FloorMotor = IOAllocation.CAN.kIndexerFloor.createTalonFX();
        ShooterFeederMotor = IOAllocation.CAN.kIndexerShooter.createTalonFX();
        BeltMotor = IOAllocation.CAN.kIndexerBelt.createTalonFX();

        // Configure motors
        TalonFXConfigHelper config = new TalonFXConfigHelper();

        TalonFXConfigHelper config2 = new TalonFXConfigHelper();

        config.MotorOutput.Inverted = InvertedValue.Clockwise_Positive;
        config.MotorOutput.NeutralMode = NeutralModeValue.Coast;

        config2.MotorOutput.Inverted = InvertedValue.Clockwise_Positive;
        config2.MotorOutput.NeutralMode = NeutralModeValue.Coast;

        // PID/FF gains (tune these)
        config.Slot0.kP = 0.1; // Start with a moderate kP for responsive control, but not too high to avoid oscillations.
        config.Slot0.kI = 0.0; // This is Usually never used so leave at 0.
        config.Slot0.kD = 0.001; // kD can help with overshoot and settling time, but for this leaving it low is fine.
        config.Slot0.kV = 0.12; // kV is crucial for velocity control, Adjust as needed based on performance.
        config.Slot0.kA = 0.0; // kA can be used to add feedforward based on acceleration, but for a simple velocity control, it can be left at 0.

        config2.Slot0.kP = 0.1; 
        config2.Slot0.kI = 0.0;
        config2.Slot0.kD = 0.001;
        config2.Slot0.kV = 0.12;
        config2.Slot0.kA = 0.0;

        // Apply configuration to both motors
        config.apply(FloorMotor, BeltMotor);
        config2.apply( ShooterFeederMotor);

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
        double FloorMotorsRPS = 0.0; // Floor And belt motor (lower-stage, keeps creeping)
        double FeederRPS = 0.0; // upper-stage 

        switch (targetState) {
            case INTAKE -> {
                // Normal intake speed for both motors
                FloorMotorsRPS = Constants.kIndexerRollRPS.get();
                FeederRPS = Constants.kIndexerRollRPS.get();
            }
            case HOLDING_BALL -> {
                // Lower motor creeps slowly to keep feeding / manage queue
                FloorMotorsRPS = Constants.kIndexerHoldRPS.get(); // slower, new constant
                FeederRPS = Constants.kIndexerIdleRPS.get(); // usually 0

            }
            case IDLE -> {
                // Both motors stopped/idle
                FloorMotorsRPS = Constants.kIndexerIdleRPS.get();
                FeederRPS = Constants.kIndexerIdleRPS.get();
            }
            case IGNORE -> {
                // Both motors run at intake speed, ignoring ball presence
                FloorMotorsRPS = Constants.kIndexerIdleRPS.get();
                FeederRPS = Constants.kIndexerRollRPS.get();
            }
            case RINDEX -> {
                // Both motors run in reverse to clear jams
                FloorMotorsRPS = -Constants.kIndexerRollRPS.get();
                FeederRPS = -Constants.kIndexerRollRPS.get();
            }
        }

        // Apply controls
        FloorMotor.setControl(velocityControl.withVelocity(FloorMotorsRPS));
        ShooterFeederMotor.setControl(velocityControl.withVelocity(FeederRPS));
        BeltMotor.setControl(velocityControl.withVelocity(FloorMotorsRPS)); 
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

    // Command that conditionally intakes based on ball presence
    public Command ConditionalIntake(){
        return Commands.run(
            () -> {
                if (ballAtTop) {
                    setTargetState(State.IGNORE);
                } else {
                    setTargetState(State.INTAKE);
                }
            },
            this
        );
    }
}
