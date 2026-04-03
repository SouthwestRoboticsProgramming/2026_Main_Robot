package com.swrobotics.robot.subsystems.intake.indexer;

import com.ctre.phoenix6.configs.CANrangeConfiguration;
import com.ctre.phoenix6.controls.Follower;
import com.ctre.phoenix6.controls.VelocityVoltage;
import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.hardware.CANrange;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.hardware.TalonFXS;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.MotorAlignmentValue;
import com.ctre.phoenix6.signals.MotorArrangementValue; // Added for Minion config
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
        FEED,
        RINDEX 
    }

    private final TalonFX floorMotor; 
    private final TalonFX shooterFeederMotor; 
    private final TalonFX beltMotor; 
    private final TalonFXS kickerMotor; // Minion Motor
    private final CANrange canrange;

    private final VelocityVoltage velocityControl = new VelocityVoltage(0).withEnableFOC(false);
    private final VoltageOut voltageControl = new VoltageOut(0);
    private boolean ballAtTop = false;
    private State targetState;

    public IndexerSubsystem() {
        floorMotor = IOAllocation.CAN.kIndexerFloor.createTalonFX();
        shooterFeederMotor = IOAllocation.CAN.kIndexerShooter.createTalonFX();
        beltMotor = IOAllocation.CAN.kIndexerBelt.createTalonFX();
        kickerMotor = IOAllocation.CAN.kIndexerKicker.createTalonFXS();
        canrange = IOAllocation.CAN.kIndexerCANrange.createCANrange();

        TalonFXConfigHelper config = new TalonFXConfigHelper();
        config.MotorOutput.Inverted = InvertedValue.CounterClockwise_Positive;
        config.MotorOutput.NeutralMode = NeutralModeValue.Coast;
        config.CurrentLimits.StatorCurrentLimit = 60.0;
        config.CurrentLimits.StatorCurrentLimitEnable = true;
        config.Slot0.kP = 0.3;
        config.Slot0.kV = 0.13;

        TalonFXConfigHelper config2 = new TalonFXConfigHelper();
        config2.MotorOutput.Inverted = InvertedValue.Clockwise_Positive;
        config2.MotorOutput.NeutralMode = NeutralModeValue.Coast;
        config2.CurrentLimits.StatorCurrentLimit = 60.0;
        config2.CurrentLimits.StatorCurrentLimitEnable = true; 
        config2.Slot0.kP = 0.3;
        config2.Slot0.kV = 0.13;

        TalonFXSConfigHelper config3 = new TalonFXSConfigHelper();
        config3.Commutation.MotorArrangement = MotorArrangementValue.Minion_JST;
        config3.MotorOutput.NeutralMode = NeutralModeValue.Coast;
        config3.CurrentLimits.StatorCurrentLimit = 40.0;
        config3.CurrentLimits.StatorCurrentLimitEnable = true; 
        TalonFXConfigHelper config4 = new TalonFXConfigHelper();
        config4.MotorOutput.Inverted = InvertedValue.CounterClockwise_Positive;
        config4.MotorOutput.NeutralMode = NeutralModeValue.Coast;
        config4.CurrentLimits.StatorCurrentLimit = 80.0;
        config4.CurrentLimits.StatorCurrentLimitEnable = true; 
        

        config.apply(beltMotor);
        config2.apply(shooterFeederMotor);
        config3.apply(kickerMotor);
        config4.apply(floorMotor);
        
        // --- CANrange Detector Configuration ---
        CANrangeConfiguration rangeConfig = new CANrangeConfiguration();

        canrange.getConfigurator().apply(rangeConfig);
        
        targetState = State.IDLE;
        setDefaultCommand(commandSetState(IndexerSubsystem.State.IDLE));
    }

    @Override
    public void periodic() {
        // Read the CANrange detection state
        ballAtTop = canrange.getIsDetected().getValue();

        double floorRPS = 0.0;
        double beltRPS = 0.0; // Belt will follow the shooter feeder motor
        double feederRPS = 0.0; 
        double kickerRPS = 0.0;

        switch (targetState) {
            case INTAKE:
                floorRPS = 0.0;
                beltRPS = 0.0;
                feederRPS = 0.0;
                kickerRPS = 0.0;
                break;
            case IDLE:
                floorRPS = Constants.kIndexerIdleVoltage.get();
                beltRPS = Constants.kIndexerIdleVoltage.get();
                feederRPS = Constants.kIndexerIdleVoltage.get();
                kickerRPS = Constants.kIndexerIdleVoltage.get();
                break;
            case FEED:
                floorRPS = 6.0;
                beltRPS = 20.0;
                feederRPS = 20.0;
                kickerRPS = 4.0;
                break;
            case RINDEX:
                floorRPS = -6.0;
                beltRPS = -20.0;
                feederRPS = -25.0;
                kickerRPS = -10.0;
                break;
        }

        // Apply calculated velocities
        floorMotor.setControl(voltageControl.withOutput(floorRPS));
        beltMotor.setControl(velocityControl.withVelocity(beltRPS)); 
        shooterFeederMotor.setControl(velocityControl.withVelocity(feederRPS));
        kickerMotor.setControl(voltageControl.withOutput(kickerRPS));
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
        return Commands.sequence(
            // Phase 1: Run INTAKE until sensor sees the ball
            commandSetState(State.INTAKE).until(this::isBallAtTop),
            
            // Phase 2: Switch to HOLD and stay there until the command is finished 
            // (usually because the user released the button)
            commandSetState(State.IDLE)
        );
    }
}