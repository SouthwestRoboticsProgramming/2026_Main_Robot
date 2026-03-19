package com.swrobotics.robot.subsystems.shooter.hood;

import com.ctre.phoenix6.controls.PositionVoltage;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import com.swrobotics.lib.ctre.TalonFXConfigHelper;
import com.swrobotics.robot.config.Constants;
import com.swrobotics.robot.config.IOAllocation;
import com.swrobotics.robot.control.AimCalc;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class HoodSubsystem extends SubsystemBase {
    // ADJUST THIS: If your motor spins 60 times for 1 hood rotation, this is 60.0
    private static final double HOOD_GEAR_RATIO = 60.0; 

    private final TalonFX motor;
    private final PositionVoltage positionControl = new PositionVoltage(0.0).withEnableFOC(true);

    public enum HoodMode { AUTO_TRACK, MANUAL }
    private HoodMode mode = HoodMode.MANUAL;
    private double manualTargetMotorRotations = 0.0;

    public HoodSubsystem() {
        motor = IOAllocation.CAN.kHoodMotor.createTalonFX();

        TalonFXConfigHelper config = new TalonFXConfigHelper();
        config.MotorOutput.Inverted = Constants.kHoodInverted.get() 
            ? InvertedValue.CounterClockwise_Positive 
            : InvertedValue.Clockwise_Positive;
        config.MotorOutput.NeutralMode = NeutralModeValue.Brake;
        
        config.CurrentLimits.StatorCurrentLimit = 20.0;
        config.CurrentLimits.StatorCurrentLimitEnable = true;
        
        // P needs to be high because we are multiplying the error by the gear ratio
        config.Slot0.kP = 15.0; 
        config.Slot0.kD = 0.2;
        
        config.apply(motor);
    }

    @Override
    public void periodic() {
        switch (mode) {
            case AUTO_TRACK -> runAutoTracking();
            case MANUAL -> runManualControl();
        }
        
        SmartDashboard.putNumber("Hood/Motor Rotations", motor.getPosition().getValueAsDouble());
        SmartDashboard.putNumber("Hood/Actual Degrees", getPhysicalDegrees());
    }

    private void runAutoTracking() {
        // 1. Get physical target rotations (0 to 1) from AimCalc
        double targetPhysicalRotations = AimCalc.getInstance().getHoodAngle().getRotations();
        
        // 2. Convert to Motor Rotations
        double targetMotorRotations = targetPhysicalRotations * HOOD_GEAR_RATIO;
        
        // 3. Clamp based on physical motor limits
        double minMotorRot = (Constants.kHoodMinAngle.get() / 360.0) * HOOD_GEAR_RATIO;
        double maxMotorRot = (Constants.kHoodMaxAngle.get() / 360.0) * HOOD_GEAR_RATIO;
        
        motor.setControl(positionControl.withPosition(
            MathUtil.clamp(targetMotorRotations, minMotorRot, maxMotorRot)
        ));
    }

    private void runManualControl() {
        motor.setControl(positionControl.withPosition(manualTargetMotorRotations));
    }

    public double getPhysicalDegrees() {
        // Motor Rotations -> Physical Rotations -> Degrees
        return (motor.getPosition().getValueAsDouble() / HOOD_GEAR_RATIO) * 360.0;
    }

    public boolean isAtTarget() {
        double currentMotorRot = motor.getPosition().getValueAsDouble();
        double targetMotorRot = (mode == HoodMode.AUTO_TRACK) 
            ? AimCalc.getInstance().getHoodAngle().getRotations() * HOOD_GEAR_RATIO 
            : manualTargetMotorRotations;
        
        // Tolerance of 0.2 motor rotations is usually < 1 degree physically
        return Math.abs(currentMotorRot - targetMotorRot) < 0.2;
    }

    public Command setManualTargetMotorRot(double motorRotations) {
        return Commands.runOnce(() -> {
            mode = HoodMode.MANUAL;
            manualTargetMotorRotations = motorRotations;
        }, this);
    }

    public Command cmdAutoTrack() {
        return Commands.runOnce(() -> mode = HoodMode.AUTO_TRACK, this);
    }
    
}