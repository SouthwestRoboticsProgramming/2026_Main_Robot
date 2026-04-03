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
    // 26 degrees is bottom (hard stop), 50 degrees is top
    public static final double kMinAngleRot = 23.0 / 360.0; 
    public static final double kMaxAngleRot = 48.0 / 360.0;
    public static final double kHoodGearRatio = 24.0; 

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
        
        // FIX 1: Set Clockwise to Positive as requested
        config.MotorOutput.Inverted = InvertedValue.Clockwise_Positive;
        config.MotorOutput.NeutralMode = NeutralModeValue.Brake;
        
        config.Feedback.SensorToMechanismRatio = kHoodGearRatio; 
        
        // config.CurrentLimits.StatorCurrentLimit = 80.0;
        // config.CurrentLimits.StatorCurrentLimitEnable = true;

        // PID Gains (Ensure kG is positive to lift UP against gravity)
        config.Slot0.kP = 50.0; 
        config.Slot0.kG = 0;

        config.apply(motor);
        homingTimer.start();
    }

    @Override
    public void periodic() {
        switch (state) {
            case HOMING:
                motor.setControl(voltageControl.withOutput(-3.0)); 
                
                double current = motor.getStatorCurrent().getValueAsDouble();
                boolean bypassedInrush = homingTimer.hasElapsed(0.75);
                boolean isStalled = current > 20.0; 

                if (bypassedInrush && isStalled) {

                    motor.setPosition(kMinAngleRot); // Declare this is 26 deg
                    targetRotations = kMinAngleRot;
                    state = HoodState.IDLE;
                }
                break;

            case IDLE:
                targetRotations = kMinAngleRot;       
                setDefaultCommand(setMode(HoodState.IDLE));
                applyPositionControl();
                break;

            case AUTO_TRACK:
                targetRotations = AimCalc.getInstance().getHoodAngle(false).getRotations(); // Subtract 26 degrees to convert to rotation units
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
        // Constraints: 26 to 50 degrees
        targetRotations = MathUtil.clamp(targetRotations, kMinAngleRot, kMaxAngleRot);
        motor.setControl(positionControl.withPosition(targetRotations));
    }

    public Command manualNudge(double degrees) {
        return runOnce(() -> {
            // FIX 3: Immediate feedback by pulling current position before adding nudge
            this.state = HoodState.MANUAL;
            double currentPos = motor.getPosition().getValueAsDouble();
            this.targetRotations = currentPos + (degrees / 360.0);
        });
    }

    public Command setMode(HoodState newState) {
        return run(
            () -> {
            this.state = newState;
            if (newState == HoodState.HOMING) 
            homingTimer.restart();
        }
        );
    }
    

    private void updateTelemetry() {
        SmartDashboard.putString("Hood/State", state.name());
        SmartDashboard.putNumber("Hood/Actual Deg", motor.getPosition().getValueAsDouble() * 360.0);
        SmartDashboard.putNumber("Hood/Target Deg", targetRotations * 360.0 );
    }

    // Add this inside your HoodSubsystem class
public double getMeasurementDegrees() {
    // Converts the TalonFX rotations (0-1) to degrees (0-360)
    return motor.getPosition().getValueAsDouble() * 360.0;
}
}