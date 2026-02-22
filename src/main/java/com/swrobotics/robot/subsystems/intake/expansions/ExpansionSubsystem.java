package com.swrobotics.robot.subsystems.intake.expansions;

// Ctre imports
import com.ctre.phoenix6.configs.MotionMagicConfigs;
import com.ctre.phoenix6.controls.MotionMagicVoltage;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;

// SW Robotics imports
import com.swrobotics.lib.ctre.TalonFXConfigHelper;
import com.swrobotics.robot.config.IOAllocation;
import com.swrobotics.robot.config.Constants;

// WPILib imports
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class ExpansionSubsystem extends SubsystemBase {

    // States for the expansion mechanism
    public enum State {
        RETRACTED,
        EXTENDED
    }

    // The motor controlling the expansion mechanism
    private final TalonFX motor;
    private final MotionMagicVoltage m_motionMagic = new MotionMagicVoltage(0).withSlot(0);
    private State targetState;

    public ExpansionSubsystem() {

        // Initialize the motor
        motor = IOAllocation.CAN.kExpansionMotor.createTalonFX();

        // Configure the motor with appropriate settings for position control
        TalonFXConfigHelper config = new TalonFXConfigHelper();
        config.MotorOutput.Inverted = InvertedValue.Clockwise_Positive;
        config.MotorOutput.NeutralMode = NeutralModeValue.Brake;

        // PID gains (slot 0) - these values will need tuning on the actual robot, but this is a reasonable starting point for position control
        config.Slot0.kP = 0.1; // Start with a moderate kP for responsive control, but not too high to avoid oscillations.
        config.Slot0.kI = 0.0; // I. do not touch
        config.Slot0.kD = 0.001; // D. Start with a small value to help dampen any oscillations, but not too high to cause instability.
        config.Slot0.kS = 0.0; // kS is not always necessary for position control, but can help overcome static friction if the mechanism is heavy or has a lot of stiction. Start with 0 and increase if you see sluggish response at low speeds.
        config.Slot0.kG = 0.0; // kG is not always necessary, but can help compensate for gravity if the mechanism is affected by it. Start with 0 and increase if you see consistent error in one direction due to gravity.

        // Motion Magic profile using constants
        MotionMagicConfigs mm = new MotionMagicConfigs();
        mm.MotionMagicCruiseVelocity = Constants.kExpansionCruiseVelocity.get();
        mm.MotionMagicAcceleration   = Constants.kExpansionAcceleration.get();
        config.MotionMagic = mm;

        // Apply the configuration to the motor
        config.apply(motor);

        // Zero the position at startup so rotations are relative to home and set initial state to retracted. always start with intake up before matches.
        motor.setPosition(0);
        targetState = State.RETRACTED;
    }

    // In periodic, we will check the target state and set the motor's Motion Magic target position accordingly. This allows us to simply change the target state and let the control loop handle moving to the correct position.
    @Override
    public void periodic() {

        // Determine the target position in rotations based on the desired state.  
        double targetRotations;

        switch (targetState) {

            // Extends bucket out to intake balls. This should be the position where the bucket is fully extended and can intake balls from the floor.
            case EXTENDED:
                // Must be in ROTATIONS, not sensor counts
                targetRotations = Constants.kExpansionExtendedRotations.get();
                break;

            // Retracts bucket up to stow. This should be the position where the bucket is fully retracted and stowed away, for driving around the field without hitting anything.
            case RETRACTED:
            default:
                targetRotations = Constants.kExpansionRetractedRotations.get();
                break;
        }

        // MotionMagicVoltage expects rotations as the unit for Position
        motor.setControl(m_motionMagic.withPosition(targetRotations));
    }

    // Simple setter for the target state. 
    public void setTargetState(State targetState) {
        this.targetState = targetState;
    }

    // Command version of the setter, for easy use in button bindings and autonomous sequences. This will run the setter once when the command is executed, and then finish immediately.
    public Command commandSetState(State targetState) {
        return Commands.runOnce(() -> setTargetState(targetState), this);
    }

}
