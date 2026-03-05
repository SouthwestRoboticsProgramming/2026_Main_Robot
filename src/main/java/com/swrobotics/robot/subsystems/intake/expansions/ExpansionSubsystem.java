package com.swrobotics.robot.subsystems.intake.expansions;

import com.ctre.phoenix6.controls.PositionVoltage;
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
    private final PositionVoltage positionControl = new PositionVoltage(0).withEnableFOC(true);
    private State targetState;

    public ExpansionSubsystem() {

        motor = IOAllocation.CAN.kExpansionMotor.createTalonFX();
        TalonFXConfigHelper config = new TalonFXConfigHelper();
        config.MotorOutput.Inverted = InvertedValue.CounterClockwise_Positive;
        config.MotorOutput.NeutralMode = NeutralModeValue.Brake;
        config.CurrentLimits.SupplyCurrentLimit = 40;
        config.CurrentLimits.SupplyCurrentLimitEnable = true;
        config.Slot0.kP = 0.5;
        config.Slot0.kI = 0;
        config.Slot0.kD = 0;
        config.Slot0.kG = 0;

        config.apply(motor);
        
        motor.setPosition(0);
        targetState = State.RETRACTED;

        setDefaultCommand(commandSetState(State.EXTENDED));
    }

    @Override
    public void periodic() {
        double targetRotations;

        switch (targetState) {
            case EXTENDED: targetRotations = Constants.kExpansionExtendedRotations.get();
            break;
            case RETRACTED: targetRotations = 0;
            break;
            default: targetRotations = 0;
        }

        motor.setControl(positionControl.withPosition(targetRotations));
    }

    public void setTargetState(State targetState) {
        this.targetState = targetState;
    }

    public Command commandSetState(State targetState) {
        return Commands.runOnce(() -> setTargetState(targetState), this);
    }

}
