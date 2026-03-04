package com.swrobotics.robot.subsystems.intake;

// Ctre imports
import com.ctre.phoenix6.controls.VelocityVoltage; // Switched to Velocity
import com.ctre.phoenix6.controls.VoltageOut;
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
    //private final VelocityVoltage velocityControl = new VelocityVoltage(0);
    private final VoltageOut voltageControl = new VoltageOut(0);
    private State targetState;

    public IntakeSubsystem() {

        // Initialize the motor
        motor = IOAllocation.CAN.kIntakeMotor.createTalonFX();
        
        // Configure the motor with appropriate settings for velocity control
        TalonFXConfigHelper config = new TalonFXConfigHelper();
        config.MotorOutput.Inverted = InvertedValue.Clockwise_Positive;
        config.MotorOutput.NeutralMode = NeutralModeValue.Coast;
        voltageControl.withEnableFOC(true);
        config.addTunable(Constants.kIntakePID);

        // apply the configuration to the motor
        config.apply(motor);
        targetState = State.IDLE;

        setDefaultCommand(commandSetState(IntakeSubsystem.State.IDLE));
    }

    @Override
    public void periodic() {
        double targetVoltage = 0;
        switch (targetState) {
            case INTAKE: targetVoltage = Constants.kIntakeVoltage.get();
            break;
            case IDLE: targetVoltage = Constants.kIntakeIdleVoltage.get();
            break;
        }

        // Apply control
        motor.setControl(voltageControl.withOutput(targetVoltage)); 
    }

    public void setTargetState(State targetState) {
        this.targetState = targetState;
    }

    // Command to set State of the Intake.
    public Command commandSetState(State targetState) {
        return Commands.run(() -> setTargetState(targetState), this);
    }
}