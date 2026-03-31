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


                boolean bypassedInrush = homingTimer.hasElapsed(0.1);
                boolean isStalled = current > 30.0;
                boolean isTimedOut = homingTimer.hasElapsed(0.5);

                if ((bypassedInrush && isStalled) || isTimedOut) {
                    applyPositionControl();
                    motor.setPosition(kMinAngleRot); 
                    targetRotations = kMinAngleRot; // Ensure target matches actual
                    state = HoodState.IDLE;
                }
                break;

            case IDLE:
                // Safely rest at the physical bottom
                applyPositionControl();
                targetRotations = kMinAngleRot;
                break;

            case AUTO_TRACK:
                applyPositionControl();
                targetRotations = AimCalc.getInstance().getHoodAngle(false).getRotations();
                break;

            case PASSING:
                applyPositionControl();
                targetRotations = AimCalc.getInstance().getHoodAngle(true).getRotations();
                break;

            case MANUAL:
                applyPositionControl();
                break;
        }
        updateTelemetry();
    }

    private void applyPositionControl() {
        double safeLowerBound = Math.min(kMinAngleRot, kMaxAngleRot);
        double safeUpperBound = Math.max(kMinAngleRot, kMaxAngleRot);
        
        targetRotations = MathUtil.clamp(targetRotations, safeLowerBound, safeUpperBound);
        
        motor.setControl(positionControl.withPosition(targetRotations));
    }

    public Command setMode(HoodState newState) {
        return runOnce(() -> { // Changed to runOnce so it triggers correctly on button press
            state = newState;
            if (newState == HoodState.HOMING) {
                homingTimer.restart();
            }
        });
    }

    public Command manualNudge(double degrees) {
        // Changed to runOnce so one button click = exactly one nudge
        return runOnce(() -> {
            state = HoodState.MANUAL; 
            targetRotations += (degrees / 360.0);
            
        });
    }

    public boolean isAtTarget() {
        return Math.abs(motor.getPosition().getValueAsDouble() - targetRotations) < (0.5 / 360.0);
    }

    private void updateTelemetry() {
        SmartDashboard.putString("Hood/State", state.name());
        SmartDashboard.putNumber("Hood/Target Deg", targetRotations * 360.0);
        SmartDashboard.putNumber("Hood/Actual Deg", motor.getPosition().getValueAsDouble() * 360.0);
        SmartDashboard.putBoolean("Hood/At Target", isAtTarget());
        SmartDashboard.putNumber("Hood/Homing Timer", homingTimer.get());
        SmartDashboard.putNumber("Hood/Stator Current", motor.getStatorCurrent().getValueAsDouble());
    }

    public HoodState getState() {
        return state;
    }
}