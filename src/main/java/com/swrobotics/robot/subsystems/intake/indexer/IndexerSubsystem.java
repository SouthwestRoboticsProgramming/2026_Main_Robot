package com.swrobotics.robot.subsystems.intake.indexer;

import com.ctre.phoenix6.controls.VelocityVoltage;
// CTRE Phoenix 6 imports
import com.ctre.phoenix6.controls.VoltageOut;
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
        FEED,
        RINDEX // optional: run in reverse to clear jams, etc.
    }

    // Motors and control
    private final TalonFX FloorMotor;   // lower stage (keeps creeping)
    private final TalonFX ShooterFeederMotor;  // upper stage (stops when ball present)
    private final TalonFX BeltMotor;
    //private final VelocityVoltage velocityControl = new VelocityVoltage(0).withEnableFOC(true);
    private final VoltageOut voltageControl = new VoltageOut(0).withEnableFOC(true);

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

        config.MotorOutput.Inverted = InvertedValue.CounterClockwise_Positive;
        config.MotorOutput.NeutralMode = NeutralModeValue.Coast;
        config.CurrentLimits.StatorCurrentLimit = 60.0;
        config.CurrentLimits.StatorCurrentLimitEnable = true; 

        config2.MotorOutput.Inverted = InvertedValue.Clockwise_Positive;
        config2.MotorOutput.NeutralMode = NeutralModeValue.Coast;
        config2.CurrentLimits.StatorCurrentLimit = 40.0;
        config2.CurrentLimits.StatorCurrentLimitEnable = true; 

        // Apply configuration to both motors
        config.apply(FloorMotor, BeltMotor);
        config2.apply( ShooterFeederMotor);

        // Create CANrange (adjust to your actual IO allocation)
        canrange = IOAllocation.CAN.kIndexerCANrange.createCANrange();
        
        targetState = State.IDLE;
        setDefaultCommand(commandSetState(IndexerSubsystem.State.IDLE));
    }

    @Override
    public void periodic() {
        // Read CANrange: true when a ball is at the top
        boolean detected = canrange.getIsDetected().getValue();
        ballAtTop = detected;

        // If we’re intaking and now see a ball, go to HOLDING_BALL
        // if (targetState == State.INTAKE && ballAtTop) {
        //     targetState = State.HOLDING_BALL;
        // }

        // Decide Voltage for each motor independently
        double FloorMotorsVoltage = 0.0; // Floor And belt motor (lower-stage, keeps creeping)
        double FeederVoltage = 0.0; // upper-stage 

        switch (targetState) {
            case INTAKE : {
                // Normal intake speed for both motors
                FloorMotorsVoltage = 10.0;
                FeederVoltage = 0;
                break;
            }
            case IDLE : {
                // Both motors stopped/idle
                FloorMotorsVoltage = Constants.kIndexerIdleVoltage.get();
                FeederVoltage =  Constants.kIndexerIdleVoltage.get();
                break;
            }
            case FEED : {
                // Both motors run at intake speed, ignoring ball presence
                FloorMotorsVoltage = 10.0;
                FeederVoltage = 12.0;
                break;
            }
            case RINDEX : {
                // Both motors run in reverse to clear jams
                FloorMotorsVoltage = -10.0;
                FeederVoltage = -12.0;
                break;
                
            }
        }

        // Apply controls
        FloorMotor.setControl(voltageControl.withOutput(0));
        ShooterFeederMotor.setControl(voltageControl.withOutput(FeederVoltage));
        BeltMotor.setControl(voltageControl.withOutput(FeederVoltage)); 
    }

    // State setter
    public void setTargetState(State targetState) {
        this.targetState = targetState;
    }

    // Command to set a state once
    public Command commandSetState(State state) {
        return Commands.run(() -> setTargetState(state), this);
    }

    // For shooter or higher-level logic to query
    public boolean isBallAtTop() {
        return ballAtTop;
    }



    // Command that conditionally intakes based on ball presence
    public Command ConditionalIntake() {
        return Commands.run(
            () -> {
                if (ballAtTop) {
                    setTargetState(State.FEED);
                } else {
                    setTargetState(State.INTAKE);
                }
            },
            this
        );
    }
}
