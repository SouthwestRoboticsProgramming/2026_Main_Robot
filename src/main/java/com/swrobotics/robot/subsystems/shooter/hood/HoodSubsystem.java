package com.swrobotics.robot.subsystems.shooter.hood;

import com.ctre.phoenix6.controls.PositionVoltage;
import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import com.swrobotics.lib.ctre.TalonFXConfigHelper;
import com.swrobotics.robot.config.Constants;
import com.swrobotics.robot.config.IOAllocation;
import com.swrobotics.robot.control.AimCalc;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class HoodSubsystem extends SubsystemBase {

    public static final double kMinAngleRot = 64 / 360.0;
    public static final double kMaxAngleRot = 40 / 360.0;
    public static final double kHoodGearRatio = 24; 
    

    private final TalonFX motor;
    private final PositionVoltage positionControl = new PositionVoltage(0).withEnableFOC(true);
    private final VoltageOut voltageControl = new VoltageOut(0);

    public enum HoodState { HOMING, IDLE, AUTO_TRACK, PASSING, MANUAL }
    private HoodState state = HoodState.HOMING;
    
    private double targetRotations = kMinAngleRot;
    private final Timer homingTimer = new Timer();

    public HoodSubsystem() {
        motor = IOAllocation.CAN.kHoodMotor.createTalonFX();
        TalonFXConfigHelper config = new TalonFXConfigHelper();
        
        config.MotorOutput.Inverted = Constants.kHoodInverted.get() 
            ? InvertedValue.Clockwise_Positive : InvertedValue.CounterClockwise_Positive;
        config.MotorOutput.NeutralMode = NeutralModeValue.Brake;
        
        // This MUST match your physical gear ratio so 1 "Rotation" = 1 Mechanism Rotation
        config.Feedback.SensorToMechanismRatio = kHoodGearRatio; 
        
        config.CurrentLimits.StatorCurrentLimit = 60.0;
        config.CurrentLimits.StatorCurrentLimitEnable = true;

        config.Slot0.kP = 8;
        config.Slot0.kD = 0;
        config.Slot0.kG = .75;

        config.apply(motor);

        homingTimer.start();
    }

    @Override
    public void periodic() {
        switch (state) {
            case HOMING:
                motor.setControl(voltageControl.withOutput(-2.0)); // Drive down
                
                double current = motor.getStatorCurrent().getValueAsDouble();

                // Wait 0.5s to bypass inrush current, then look for true stall
                if (homingTimer.hasElapsed(0.5) || current > 30.0) {
                    motor.setPosition(kMinAngleRot); 
                    state = HoodState.IDLE;
                }
                break;

            case IDLE:
                // Safely rest at the physical bottom
                targetRotations = kMinAngleRot;
                applyPositionControl();
                break;

            case AUTO_TRACK:
                targetRotations = AimCalc.getInstance().getHoodAngle(false).getRotations();
                applyPositionControl();
                break;

            case PASSING:
                targetRotations = AimCalc.getInstance().getHoodAngle(true).getRotations();
                applyPositionControl();
                break;

            case MANUAL:
                applyPositionControl();

                break;
        }
        updateTelemetry();
    }

    private void applyPositionControl() {
        // Clamp prevents the motor from ever trying to break past 45 deg or dig into the 22.73 hard stop
        targetRotations = MathUtil.clamp(targetRotations, kMinAngleRot, kMaxAngleRot);
        
        motor.setControl(positionControl
            .withPosition(targetRotations)
        );
    }

    public Command setMode(HoodState newState) {
        return run(() -> {
            state = newState;
            if (newState == HoodState.HOMING) homingTimer.restart();
        });
    }

    public Command manualNudge(double degrees) {
        return run(() -> {
            
            targetRotations = motor.getPosition().getValueAsDouble();           
            targetRotations += (degrees / 360.0);
            targetRotations= MathUtil.clamp(targetRotations, -800, 800);
        });
    }

    public boolean isAtTarget() {
        return Math.abs(motor.getPosition().getValueAsDouble() - targetRotations) < (0.5 / 360.0);
    }

    private void updateTelemetry() {
        SmartDashboard.putString("Hood/State", state.name());
        // Multiply by 360 to read out actual degrees on the dashboard for easy debugging
        SmartDashboard.putNumber("Hood/Target Deg", targetRotations * 360.0);
        SmartDashboard.putNumber("Hood/Actual Deg", motor.getPosition().getValueAsDouble() * 360.0);
        SmartDashboard.putBoolean("Hood/At Target", isAtTarget());
    }

    public HoodState getState() {
        return state;
    }
}