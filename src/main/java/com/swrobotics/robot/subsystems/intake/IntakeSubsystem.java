package com.swrobotics.robot.subsystems.intake;

import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;

// Sw Robotics imports
import com.swrobotics.lib.ctre.TalonFXConfigHelper;
import com.swrobotics.robot.config.IOAllocation;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
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
    private final VoltageOut voltageControl = new VoltageOut(0);
    private State targetState;

    public IntakeSubsystem() {

        // Initialize the motor
        motor = IOAllocation.CAN.kIntakeMotor.createTalonFX();
        
        TalonFXConfigHelper config = new TalonFXConfigHelper();
        config.MotorOutput.Inverted = InvertedValue.Clockwise_Positive;
        config.MotorOutput.NeutralMode = NeutralModeValue.Coast;

        // apply the configuration to the motor
        config.apply(motor);
        targetState = State.IDLE;

        setDefaultCommand(commandSetState(IntakeSubsystem.State.IDLE));
    }

    @Override
    public void periodic() {
        double targetVoltage = 0;
        switch (targetState) {
            case INTAKE: targetVoltage = 12.0; // Set to a positive voltage to intake balls. Adjust as needed for optimal performance.
            break;
            case IDLE: targetVoltage = 0.0;
            break;
        }

        // Apply control
        motor.setControl(voltageControl.withOutput(targetVoltage)); 

        SmartDashboard.putString("Intake/State", targetState.name());
        SmartDashboard.putNumber("Intake/Voltage", targetVoltage);
        SmartDashboard.putNumber("Intake/Current", motor.getSupplyCurrent().getValueAsDouble());

    }

    public void setTargetState(State targetState) {
        this.targetState = targetState;
    }

    // Command to set State of the Intake.
    public Command commandSetState(State targetState) {
        return Commands.run(() -> setTargetState(targetState), this);
    }
}