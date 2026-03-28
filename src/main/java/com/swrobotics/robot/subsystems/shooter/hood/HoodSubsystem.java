package com.swrobotics.robot.subsystems.shooter.hood;

import java.lang.Thread.State;

import com.ctre.phoenix6.controls.PositionVoltage;
import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import com.swrobotics.lib.ctre.TalonFXConfigHelper;
import com.swrobotics.robot.config.Constants;
import com.swrobotics.robot.config.IOAllocation;
import com.swrobotics.robot.control.AimCalc;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class HoodSubsystem extends SubsystemBase {
    // --- HOOD CONFIGURATIONS ---
    public static final double kSensorToMechRatio = 1.0; // TUNE: Enter your gear ratio here
    public static final double kMaxAngleDeg = 45.0;      // TUNE: Maximum safe hood angle
    public static final double kMinAngleDeg = 0.0;       // TUNE: Minimum safe hood angle
    public static final double kHomingVoltage = -2.0;    // TUNE: Voltage to drive hood down (- for down)
    public static final double kHomingCurrentLimit = 15.0; // TUNE: Stator current spike (Amps) to detect hard stop

    private final TalonFX motor;
    private final PositionVoltage positionControl = new PositionVoltage(0.0).withEnableFOC(true);
    private final VoltageOut voltageControl = new VoltageOut(0.0);

    public enum HoodMode { HOMING, IDLE, AUTO_TRACK, MANUAL }

    // Start in HOMING mode so it zeroes itself on boot
    private HoodMode mode = HoodMode.HOMING;
    private double targetRotations = 0.0;

    public HoodSubsystem() {
        motor = IOAllocation.CAN.kHoodMotor.createTalonFX();

        TalonFXConfigHelper config = new TalonFXConfigHelper();
        config.MotorOutput.Inverted = Constants.kHoodInverted.get()
                ? InvertedValue.CounterClockwise_Positive
                : InvertedValue.Clockwise_Positive;
        config.MotorOutput.NeutralMode = NeutralModeValue.Brake;
        config.CurrentLimits.StatorCurrentLimit = 20.0;
        config.CurrentLimits.StatorCurrentLimitEnable = true;

<<<<<<< HEAD
        // Apply your configured gear ratio
        config.Feedback.SensorToMechanismRatio = kSensorToMechRatio;
=======
        config.Feedback.SensorToMechanismRatio = 24; // TUNE: set your gear ratio here
>>>>>>> 11aed943826ef022f7018a6acb5636d129805d4c

        config.addTunable(Constants.kHoodPID);
        config.apply(motor);
    }

    @Override
    public void periodic() {
        if (mode == HoodMode.HOMING) {
            motor.setControl(voltageControl.withOutput(kHomingVoltage));
            
            if (motor.getStatorCurrent().getValueAsDouble() > kHomingCurrentLimit) {
                motor.setPosition(0.0);
                targetRotations = 0.0;
                mode = HoodMode.IDLE;   
            }
            return;
        } else if (mode == HoodMode.IDLE) {
            targetRotations = 0.0;
        } else if (mode == HoodMode.AUTO_TRACK) {
            targetRotations = AimCalc.getInstance().getHoodAngle().getRotations();
        }

        // 2. Enforce Min/Max Limits constraints on ALL position modes
        double maxRotations = kMaxAngleDeg / 360.0;
        double minRotations = kMinAngleDeg / 360.0;
        targetRotations = Math.max(minRotations, Math.min(maxRotations, targetRotations));

        // 3. Command the Motor
        motor.setControl(positionControl.withPosition(targetRotations));

        // 4. Telemetry
        SmartDashboard.putString("Shooter/Hood Mode", mode.name());
        SmartDashboard.putBoolean("Shooter/Hood At Target", isAtTarget());
        SmartDashboard.putNumber("Shooter/Hood Target Rot", targetRotations);
        SmartDashboard.putNumber("Shooter/Hood Actual Rot", motor.getPosition().getValueAsDouble());
        SmartDashboard.putNumber("Shooter/Hood Current", motor.getStatorCurrent().getValueAsDouble());
    }

    public boolean isAtTarget() {
        
        double toleranceRot = 0.5 / 360.0;
        return Math.abs(motor.getPosition().getValueAsDouble() - targetRotations) < toleranceRot;
    }

    public HoodMode getMode() {
        return mode;
    }

    // Manual absolute set
    public Command setManualPosition(double rotations) {
        return Commands.runOnce(() -> {
            targetRotations = rotations;
            mode = HoodMode.MANUAL;
        }, this);
    }

    public Command setMode(HoodMode newMode) {
        return Commands.runOnce(() -> mode = newMode, this);
    }

    // ------------- MANUAL NUDGE (±2°) -------------
    public void nudgeAngleDegrees(double deltaDeg) {
        double deltaRot = deltaDeg / 360.0;
        targetRotations += deltaRot;
        mode = HoodMode.MANUAL;
    }

    public Command commandNudgeAngleDegrees(double deltaDeg) {
        return Commands.runOnce(() -> nudgeAngleDegrees(deltaDeg), this);
    }
}