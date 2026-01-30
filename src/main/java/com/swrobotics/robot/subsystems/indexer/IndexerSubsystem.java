package com.swrobotics.robot.subsystems.indexer;

// CTRE Phoenix6 imports

import com.ctre.phoenix6.controls.VelocityVoltage; // Switched to Velocity
import com.ctre.phoenix6.hardware.TalonFX;
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
        INTAKE
    }
    // Motor and control objects
    private final TalonFX motor;
    private final TalonFX motor2;
    private final VelocityVoltage velocityControl = new VelocityVoltage(0);
    private State targetState;
    // Constructor
    public IndexerSubsystem() {

        // Initialize motor

        motor = IOAllocation.CAN.kIndexerMotor.createTalonFX();
        motor2 = IOAllocation.CAN.kIndexerMotor2.createTalonFX();

        // Configure motor settings

        TalonFXConfigHelper config = new TalonFXConfigHelper();

        // Motor output settings inverted and neutral

        config.MotorOutput.Inverted = InvertedValue.Clockwise_Positive;
        config.MotorOutput.NeutralMode = NeutralModeValue.Brake;

        /* --- PID GAINS --- */
        //TODO: These values need tuning! Start with kP at 0.1 and kV at 0.12 DO NOT SET kI or kD YET, as they are not always necessary and can cause instability if set too high
        
        config.Slot0.kP = 0.1; // kP is for proportional control
        config.Slot0.kI = 0.0; // DO NOT TOUCH THE kI or I will kI'll you
        config.Slot0.kD = 0.001; // kD is for damping oscillations. that big D, am I right?
        config.Slot0.kV = 0.12; // kV is vital for velocity control

        // Apply configuration to motor

        config.apply(motor, motor2);

        // Set initial target state which in this case is IDLE

        targetState = State.IDLE;
    }
    // Periodic method called regularly
    @Override
    public void periodic() {

        // Refresh PID gains from NetworkTables if they changed

        // Determine target RPS based on current state
        double targetRPS = 0;
        switch (targetState) {
            case INTAKE -> targetRPS = Constants.kIndexerRollRPS.get();
            case IDLE -> targetRPS = Constants.kIndexerIdleRPS.get();
        }

        // Apply control
        motor.setControl(velocityControl.withVelocity(targetRPS));
        motor2.setControl(velocityControl.withVelocity(targetRPS));
    }

    // Method to set the target state

    public void setTargetState(State targetState) {
        this.targetState = targetState;
    }

    // Command to set the state

    public Command commandSetState(State shoot) {
        return Commands.run(() -> setTargetState(shoot), this);
    }
}