package com.swrobotics.robot.subsystems.intake.expansion;

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
        EXTENDED,
    }

    private final TalonFX motor;
    private final PositionVoltage positionControl = new PositionVoltage(0).withEnableFOC(false);
    private State targetState;

    public ExpansionSubsystem() {

        motor = IOAllocation.CAN.kExpansionMotor.createTalonFX();
        TalonFXConfigHelper config = new TalonFXConfigHelper();
        config.MotorOutput.Inverted = InvertedValue.Clockwise_Positive;
        config.MotorOutput.NeutralMode = NeutralModeValue.Coast;
        config.CurrentLimits.SupplyCurrentLimit = 80;
        config.CurrentLimits.SupplyCurrentLimitEnable = true;
        config.Slot0.kP = .2;
        config.Slot0.kI = 0;
        config.Slot0.kD = 0.00;
        config.Slot0.kG = 0.1;

        config.apply(motor);
        
        motor.setPosition(0);
        targetState = State.RETRACTED;

        setDefaultCommand(commandSetState(State.RETRACTED));
    }

    @Override
    public void periodic() {
        double targetRotations;
        targetRotations = 0;

        switch (targetState) {
            case EXTENDED: targetRotations = 28.0;
            break;
            case RETRACTED: targetRotations = 0;
            break;
            
        }

        motor.setControl(positionControl.withPosition(targetRotations));
    }

    public void setTargetState(State targetState) {
        this.targetState = targetState;
    }

    public Command commandSetState(State targetState) {
        return Commands.run(() -> setTargetState(targetState), this);
    }

    


    

}
