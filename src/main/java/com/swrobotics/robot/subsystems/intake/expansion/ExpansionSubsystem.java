package com.swrobotics.robot.subsystems.intake.expansion;

import java.beans.Encoder;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.MotionMagicVoltage;
import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import com.swrobotics.robot.config.IOAllocation;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class ExpansionSubsystem extends SubsystemBase {

    public enum State {
        RETRACTED(0.0),
        STOWED(6.0),
        EXTENDED(26.0), 
        SHOOT(0.0); 

        public final double position;
        State(double position) { this.position = position; }
    }

    private final TalonFX motor;
    private final MotionMagicVoltage request = new MotionMagicVoltage(0);
    
    private State targetState = State.RETRACTED;
    private final double kOscillationSpeed = 7.5;
    private final double kOscillationAmp = 4.0;
    private final double kOscillationCenter = 10.0;

    public ExpansionSubsystem() {
        motor = IOAllocation.CAN.kExpansionMotor.createTalonFX();

        TalonFXConfiguration config = new TalonFXConfiguration();
        
        config.MotorOutput.Inverted = InvertedValue.Clockwise_Positive;
        config.MotorOutput.NeutralMode = NeutralModeValue.Brake;

        config.CurrentLimits.SupplyCurrentLimit = 60; 
        config.CurrentLimits.SupplyCurrentLimitEnable = true;

        config.Slot0.kP = 2.0; 
        config.Slot0.kI = 0;
        config.Slot0.kD = 0.1;
        config.MotionMagic.MotionMagicCruiseVelocity = 50; // rotations per sec
        config.MotionMagic.MotionMagicAcceleration = 100;   // rotations per sec^2
        config.MotionMagic.MotionMagicJerk = 400;           // Smooths out starts/stops

        config.SoftwareLimitSwitch.ForwardSoftLimitThreshold = 30.0;
        config.SoftwareLimitSwitch.ForwardSoftLimitEnable = true;
        config.SoftwareLimitSwitch.ReverseSoftLimitThreshold = -2.0;
        config.SoftwareLimitSwitch.ReverseSoftLimitEnable = true;

        motor.getConfigurator().apply(config);
        motor.setPosition(0);

        this.setDefaultCommand(run(() -> setTargetState(State.STOWED)));
    }

    @Override
    public void periodic() {
        double targetRotations;

        if (targetState == State.SHOOT) {
            targetRotations = kOscillationCenter + (kOscillationAmp * Math.sin(Timer.getFPGATimestamp() * kOscillationSpeed));
        } else {
            targetRotations = targetState.position;
        }

        // Apply control
        motor.setControl(request.withPosition(targetRotations));

        // Telemetry
        SmartDashboard.putString("Expansion/State", targetState.name());
        SmartDashboard.putNumber("Expansion/Motor Rotations", motor.getPosition().getValueAsDouble());
        SmartDashboard.putNumber("Expansion/Target Rotations", targetRotations);
    }

    public void setTargetState(State state) {
        this.targetState = state;
    }

    public Command commandSetState(State state) {
        return run(() -> setTargetState(state)).withName("SetExpansion:" + state.name());
    }
    
    public Command commandShoot() {
        return Commands.run(() -> setTargetState(State.SHOOT), this)
                .finallyDo((interrupted) -> setTargetState(State.EXTENDED));
    }
}