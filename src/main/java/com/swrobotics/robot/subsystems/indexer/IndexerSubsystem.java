package com.swrobotics.robot.subsystems.indexer;

import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;

import com.swrobotics.lib.ctre.TalonFXConfigHelper;
import com.swrobotics.robot.config.Constants;
import com.swrobotics.robot.config.IOAllocation;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class IndexerSubsystem extends SubsystemBase {

    public enum State {
        IDLE,
        FEED,
        RINDEX 
    }

    private final TalonFX floorMotor; 
    private final TalonFX shooterFeederMotor; 

    private final VoltageOut voltageControl = new VoltageOut(0);
    private boolean ballAtTop = false;
    private State targetState;

    public IndexerSubsystem() {
        floorMotor = IOAllocation.CAN.kIndexerFloor.createTalonFX();
        shooterFeederMotor = IOAllocation.CAN.kIndexerShooter.createTalonFX();

        TalonFXConfigHelper config = new TalonFXConfigHelper();
        config.MotorOutput.Inverted = InvertedValue.Clockwise_Positive;
        config.MotorOutput.NeutralMode = NeutralModeValue.Coast;
        
        // Pulled from Tunable Robot Constants Matrix
        config.CurrentLimits.StatorCurrentLimit = Constants.kIndexerStatorCurrentLimit.get();
        config.CurrentLimits.StatorCurrentLimitEnable = true; 
        config.Slot0.kP = Constants.kIndexerKp.get();
        config.Slot0.kV = Constants.kIndexerKv.get();

        config.apply(shooterFeederMotor, floorMotor);
        
        targetState = State.IDLE;
        setDefaultCommand(commandSetState(IndexerSubsystem.State.IDLE));
    }

    @Override
    public void periodic() {
        double RollerVolts = 0.0;
        double feederVolts = 0.0; 

        switch (targetState) {
            case IDLE:
                RollerVolts = Constants.kIndexerIdleVoltage.get();
                feederVolts = Constants.kIndexerIdleVoltage.get();
                break;
            case FEED:
                RollerVolts = Constants.kIndexerFeedVoltage.get();
                feederVolts = Constants.kIndexerFeedVoltage.get();
                break;
            case RINDEX:
                RollerVolts = Constants.kIndexerReverseVoltage.get();
                feederVolts = Constants.kIndexerReverseVoltage.get();
                break;
        }

        floorMotor.setControl(voltageControl.withOutput(-RollerVolts));
        shooterFeederMotor.setControl(voltageControl.withOutput(feederVolts));

        SmartDashboard.putString("Indexer/State", targetState.name());
        SmartDashboard.putNumber("Indexer/Roller Voltage", RollerVolts);
        SmartDashboard.putNumber("Indexer/Feeder Voltage", feederVolts);
        SmartDashboard.putNumber("Indexer/Roller Current", floorMotor.getSupplyCurrent().getValueAsDouble());
        SmartDashboard.putNumber("Indexer/Feeder Current", shooterFeederMotor.getSupplyCurrent().getValueAsDouble());
    }

    public void setTargetState(State targetState) {
        this.targetState = targetState;
    }

    public Command commandSetState(State state) {
        return Commands.run(() -> setTargetState(state), this);
    }

    public boolean isBallAtTop() {
        return ballAtTop;
    }
}