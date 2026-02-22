package com.swrobotics.robot.subsystems.intake;

// Ctre imports
import com.ctre.phoenix6.controls.VelocityVoltage; // Switched to Velocity
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;

// Sw Robotics imports
import com.swrobotics.lib.ctre.TalonFXConfigHelper;
import com.swrobotics.robot.config.Constants;
import com.swrobotics.robot.config.IOAllocation;

// WPILib imports
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class IntakeSubsystem extends SubsystemBase {

    //Intake States
    public enum State {
        IDLE,
        INTAKE
    }

    // The motor controlling the intake mechanism
    private final TalonFX motor;
    private final VelocityVoltage velocityControl = new VelocityVoltage(0);
    private State targetState;

    public IntakeSubsystem() {

        // Initialize the motor
        motor = IOAllocation.CAN.kIntakeMotor.createTalonFX();
        
        // Configure the motor with appropriate settings for velocity control
        TalonFXConfigHelper config = new TalonFXConfigHelper();
        config.MotorOutput.Inverted = InvertedValue.Clockwise_Positive;
        config.MotorOutput.NeutralMode = NeutralModeValue.Coast;

        // --- PID GAINS --- //
        //TODO: These values need tuning! Start with kP at 0.1 and kV at 0.12 DO NOT SET kI or kD YET, as they are not always necessary and can cause instability if set too high
        config.Slot0.kP = 0.1; // Start with a moderate kP for responsive control, but not too high to avoid oscillations.
        config.Slot0.kI = 0.0; // I. do not touch
        config.Slot0.kD = 0.001; // D. Start with a small value to help dampen any oscillations, but not too high to cause instability.
        config.Slot0.kV = 0.12; // kV is vital for velocity control

        // apply the configuration to the motor
        config.apply(motor);
        targetState = State.IDLE;
    }

    @Override
    public void periodic() {
        
        // set RPS based on target state
        double targetRPS = 0;
        switch (targetState) {
            case INTAKE -> targetRPS = Constants.kIntakeRPS.get(); // Intake speed is positive to pull balls in, but this may need to be reversed based on the actual motor orientation on the robot. If the intake runs backwards, simply change this to negative or set to counterclockwise
            case IDLE -> targetRPS = Constants.kIntakeIdleRPS.get(); //idle speed is 0.
        }

        // Apply control
        motor.setControl(velocityControl.withVelocity(targetRPS));
    }

    // Method to change the target state of the intake. 
    public void setTargetState(State targetState) {
        this.targetState = targetState;
    }

    // Command to set State of the Intake.
    public Command commandSetState(State targetState) {
        return Commands.run(() -> setTargetState(targetState), this);
    }
}