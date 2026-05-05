package com.swrobotics.robot.subsystems.shooter;

import com.ctre.phoenix6.controls.NeutralOut;
import com.ctre.phoenix6.controls.VelocityVoltage;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import com.swrobotics.lib.ctre.TalonFXConfigHelper;
import com.swrobotics.robot.config.IOAllocation;
import com.swrobotics.robot.control.AimCalc;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class ShooterSubsystem extends SubsystemBase {
    public enum State { IDLE, 
        SHOOT, WARM, RINDEX, AUTO, PASS }

    private final TalonFX ShooterX44;
    // Updated frequency to 50Hz for faster PID loop response during ball contact
    private final VelocityVoltage velocityControl = new VelocityVoltage(0).withEnableFOC(true).withUpdateFreqHz(50);
    private final NeutralOut neutralControl = new NeutralOut();
    private State targetState = State.AUTO;
    private double currentMotorTargetRPS = 0.0;

    public ShooterSubsystem() {
        ShooterX44 = IOAllocation.CAN.kShooterMotor.createTalonFX();

        TalonFXConfigHelper config = new TalonFXConfigHelper();
        config.MotorOutput.Inverted = InvertedValue.CounterClockwise_Positive;
        config.MotorOutput.NeutralMode = NeutralModeValue.Coast;
        config.CurrentLimits.StatorCurrentLimit = 120;
        config.CurrentLimits.StatorCurrentLimitEnable = true;

        config.Slot0.kS = 0.0125;
        config.Slot0.kV = 0.0103;
        config.Slot0.kA = 0.001; 
        config.Slot0.kP = 0.11;  
        config.Slot0.kD = 0.001; 

        config.apply(ShooterX44);
        setDefaultCommand(commandSetState(State.IDLE));
    }

    @Override
    public void periodic() {
        
        double wheelRps = 0;
        switch (targetState) {
            case IDLE: ShooterX44.setControl(neutralControl);
            wheelRps = 0; break;
            case SHOOT: wheelRps = 60; break;
            case WARM: wheelRps = 20; break;
            case RINDEX: wheelRps = -30; break;
            case AUTO: wheelRps = AimCalc.getInstance().getShooterRPS(); break;
            case PASS: wheelRps = AimCalc.getInstance().getShooterRPS(); break;
        }

        currentMotorTargetRPS  = wheelRps / 3.0;
        ShooterX44.setControl(velocityControl.withVelocity(currentMotorTargetRPS));

        SmartDashboard.putNumber("Shooter/Wheel Target RPS", wheelRps);
        SmartDashboard.putNumber("Shooter/Motor Target RPS", currentMotorTargetRPS);
        SmartDashboard.putNumber("Shooter/Motor Velocity RPS", ShooterX44.getVelocity().getValueAsDouble());
        SmartDashboard.putNumber("Shooter/Wheel Actual RPS", ShooterX44.getVelocity().getValueAsDouble() * 3.0);
        SmartDashboard.putBoolean("Shooter/At Speed", isAtTargetRPS());
    }

    public boolean isAtTargetRPS() {
        if (targetState != State.SHOOT && targetState != State.AUTO && targetState != State.PASS) return false;
        // Opened tolerance slightly to 1.5 RPS to prevent feeder hesitation on consecutive shots
        return Math.abs(ShooterX44.getVelocity().getValueAsDouble() - currentMotorTargetRPS) < 1.5;
    }

    public Command commandSetState(State state) { return run(() -> this.targetState = state); }
}