package com.swrobotics.robot.subsystems.intake.expansions;

import com.ctre.phoenix6.configs.MotionMagicConfigs;
import com.ctre.phoenix6.controls.MotionMagicVoltage;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import com.swrobotics.lib.ctre.TalonFXConfigHelper;
import com.swrobotics.robot.config.IOAllocation;
import com.swrobotics.robot.config.Constants;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class ExpansionSubsystem extends SubsystemBase {

    public enum State {
        RETRACTED,
        EXTENDED
    }

    private final TalonFX motor;
    private final MotionMagicVoltage m_motionMagic = new MotionMagicVoltage(0).withSlot(0);
    private State targetState;

    public ExpansionSubsystem() {
        motor = IOAllocation.CAN.kExpansionMotor.createTalonFX();

        TalonFXConfigHelper config = new TalonFXConfigHelper();
        config.MotorOutput.Inverted = InvertedValue.Clockwise_Positive;
        config.MotorOutput.NeutralMode = NeutralModeValue.Brake;

        // PID gains (slot 0) – tune later
        config.Slot0.kP = 0.1;
        config.Slot0.kI = 0.0;
        config.Slot0.kD = 0.001;

        // Motion Magic profile using constants
        MotionMagicConfigs mm = new MotionMagicConfigs();
        mm.MotionMagicCruiseVelocity = Constants.kExpansionCruiseVelocity.get();
        mm.MotionMagicAcceleration   = Constants.kExpansionAcceleration.get();
        config.MotionMagic = mm;

        config.apply(motor);

        // Zero the position at startup so rotations are relative to home
        motor.setPosition(0);

        targetState = State.RETRACTED;
    }

    @Override
    public void periodic() {
        double targetRotations;

        switch (targetState) {
            case EXTENDED:
                // Must be in ROTATIONS, not sensor counts
                targetRotations = Constants.kExpansionExtendedRotations.get();
                break;
            case RETRACTED:
            default:
                targetRotations = Constants.kExpansionRetractedRotations.get();
                break;
        }

        // MotionMagicVoltage expects rotations as the unit for Position
        motor.setControl(m_motionMagic.withPosition(targetRotations));
    }

    public void setTargetState(State targetState) {
        this.targetState = targetState;
    }


    public Command commandSetState(State targetState) {
        return Commands.runOnce(() -> setTargetState(targetState), this);
    }

}
