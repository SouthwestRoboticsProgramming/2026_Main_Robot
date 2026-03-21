package com.swrobotics.robot.subsystems.intake.indexer;


import com.ctre.phoenix6.controls.VelocityVoltage;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.hardware.TalonFXS;
import com.ctre.phoenix6.hardware.CANrange;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;

import com.swrobotics.lib.ctre.TalonFXSConfigHelper;
import com.swrobotics.lib.ctre.TalonFXConfigHelper;
import com.swrobotics.robot.config.Constants;
import com.swrobotics.robot.config.IOAllocation;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class IndexerSubsystem extends SubsystemBase {

    public enum State {
        IDLE,
        INTAKE,
        HOLD,
        FEED,
        RINDEX // optional: run in reverse to clear jams, etc.
    }

    private final TalonFX FloorMotor; //Rollers
    private final TalonFX ShooterFeederMotor; //Top belt and Feeder
    private final TalonFX BeltMotor; //Bottom belt
    private final TalonFXS KickerMotor; //Kicker 
    private final VelocityVoltage velocityControl = new VelocityVoltage(0).withEnableFOC(false);
    private final CANrange canrange;
    private boolean ballAtTop = false;
    private State targetState;

    public IndexerSubsystem() {
        FloorMotor = IOAllocation.CAN.kIndexerFloor.createTalonFX();
        ShooterFeederMotor = IOAllocation.CAN.kIndexerShooter.createTalonFX();
        BeltMotor = IOAllocation.CAN.kIndexerBelt.createTalonFX();
        KickerMotor = IOAllocation.CAN.kIndexerKicker.createTalonFXS();
        canrange = IOAllocation.CAN.kIndexerCANrange.createCANrange();

        TalonFXConfigHelper config = new TalonFXConfigHelper();
        TalonFXConfigHelper config2 = new TalonFXConfigHelper();
        TalonFXSConfigHelper config3 = new TalonFXSConfigHelper();
        
        //Current limits might be nice for these motors.
        config.MotorOutput.Inverted = InvertedValue.CounterClockwise_Positive;
        config.MotorOutput.NeutralMode = NeutralModeValue.Coast;
        config.CurrentLimits.StatorCurrentLimit = 60.0;
        config.CurrentLimits.StatorCurrentLimitEnable = true;
        config.Slot0.kP = 0.2;
        config.Slot0.kV = 0.13;


        config2.MotorOutput.Inverted = InvertedValue.Clockwise_Positive;
        config2.MotorOutput.NeutralMode = NeutralModeValue.Coast;
        config2.CurrentLimits.StatorCurrentLimit = 60.0;
        config2.CurrentLimits.StatorCurrentLimitEnable = true; 
        config2.Slot0.kP = 0.2;
        config2.Slot0.kV = 0.13;

        config3.MotorOutput.Inverted = InvertedValue.Clockwise_Positive;
        config3.MotorOutput.NeutralMode = NeutralModeValue.Coast;
        config3.CurrentLimits.StatorCurrentLimit = 60.0;
        config3.CurrentLimits.StatorCurrentLimitEnable = true; 
        config3.Slot0.kP = 0.2;
        config3.Slot0.kV = 0.13;

        config.apply(FloorMotor, BeltMotor);
        config2.apply( ShooterFeederMotor);
        config3.apply(KickerMotor);
        
        targetState = State.IDLE;
        setDefaultCommand(commandSetState(IndexerSubsystem.State.IDLE));
    }

    @Override
    public void periodic() {

        boolean detected = canrange.getIsDetected().getValue();
        ballAtTop = detected;

        // Decide Voltage for each motor independently
        double FloorMotorsRPS = 0.0; // Floor motor (rollers)
        double FeederRPS = 0.0; // upper-stage (belts, kicker, and feeder) 

        switch (targetState) {
            case INTAKE : {
                FloorMotorsRPS = 20.0;
                FeederRPS = 0.0;
                break;
            }
            case IDLE : {
                FloorMotorsRPS = Constants.kIndexerIdleVoltage.get();
                FeederRPS =  Constants.kIndexerIdleVoltage.get();
                break;
            }
            case HOLD : {
                FloorMotorsRPS = 10.0;
                FeederRPS = 0.0;
                break;
            }
            case FEED : {
                FloorMotorsRPS = 20.0;
                FeederRPS = 20.0;
                break;
            }
            case RINDEX : {
                // Both motors run in reverse to clear jams
                FloorMotorsRPS = -20.0;
                FeederRPS = -25.0;
                break;
                
            }
        }

        FloorMotor.setControl(velocityControl.withVelocity(FloorMotorsRPS));
        KickerMotor.setControl(velocityControl.withVelocity(FeederRPS));
        ShooterFeederMotor.setControl(velocityControl.withVelocity(FeederRPS));
        BeltMotor.setControl(velocityControl.withVelocity(FeederRPS)); 
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

    public Command intakeUntilCommand() {
        return Commands.run(() -> setTargetState(State.INTAKE), this)
                .until(this::isBallAtTop)
                .finallyDo((interrupted) -> setTargetState(State.HOLD));
    }
}
