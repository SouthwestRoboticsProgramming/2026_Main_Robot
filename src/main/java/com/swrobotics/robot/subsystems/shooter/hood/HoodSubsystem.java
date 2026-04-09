package com.swrobotics.robot.subsystems.shooter.hood;

import com.ctre.phoenix6.controls.PositionVoltage;
import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import com.swrobotics.lib.ctre.TalonFXConfigHelper;
import com.swrobotics.robot.config.IOAllocation;
import com.swrobotics.robot.control.AimCalc;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class HoodSubsystem extends SubsystemBase {
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
        
        config.MotorOutput.Inverted = InvertedValue.Clockwise_Positive;
        config.MotorOutput.NeutralMode = NeutralModeValue.Brake;
        
        config.Feedback.SensorToMechanismRatio = kHoodGearRatio; 
        
        config.CurrentLimits.StatorCurrentLimit = 40.0;
        config.CurrentLimits.StatorCurrentLimitEnable = true;

        config.Slot0.kP = 60.0; 
        config.Slot0.kD = 3.5;
        config.Slot0.kG = 0;

        config.apply(motor);
        homingTimer.start();
    }

    @Override
    public void periodic() {
        // Feed the current mode state directly to AimCalc so it can resolve targets & trajectories
        AimCalc.getInstance().setPassingMode(state == HoodState.PASSING);

        switch (state) {
            case HOMING:
                motor.setControl(voltageControl.withOutput(-3.0)); 
                
                double current = motor.getStatorCurrent().getValueAsDouble();
                boolean bypassedInrush = homingTimer.hasElapsed(0.75);
                boolean isStalled = current > 20.0; 

                if (bypassedInrush && isStalled) {
                    motor.setPosition(kMinAngleRot); 
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
            case PASSING:
                targetRotations = AimCalc.getInstance().getHoodAngle().getRotations(); 
                applyPositionControl();
                break;

            case MANUAL:
                applyPositionControl();
                break;
        }
        updateTelemetry();
    }

    private void applyPositionControl() {
        targetRotations = MathUtil.clamp(targetRotations, kMinAngleRot, kMaxAngleRot);
        motor.setControl(positionControl.withPosition(targetRotations));
    }

    public Command manualNudge(double degrees) {
        return runOnce(() -> {
            this.state = HoodState.MANUAL;
            double currentPos = motor.getPosition().getValueAsDouble();
            this.targetRotations = currentPos + (degrees / 360.0);
        });
    }

    public Command setMode(HoodState newState) {
        return run(() -> {
            this.state = newState;
            if (newState == HoodState.HOMING) homingTimer.restart();
        });
    }

    public boolean isAtTarget() {
        return Math.abs(motor.getPosition().getValueAsDouble() - targetRotations) < (0.5 / 360.0);
    }
    
    private void updateTelemetry() {
        SmartDashboard.putString("Hood/State", state.name());
        SmartDashboard.putNumber("Hood/Actual Deg", getMeasurementDegrees());
        SmartDashboard.putNumber("Hood/Target Deg", targetRotations * 360.0);
        SmartDashboard.putBoolean("Hood/At Target", isAtTarget());
        SmartDashboard.putNumber("Hood/lastVirtualDist", AimCalc.getInstance().getLastVirtualDistance());
        SmartDashboard.putNumber("Hood/Turret-targetdegrees", AimCalc.getInstance().getTurretAimAngle().getDegrees());
        
    }

    public double getMeasurementDegrees() {
        return motor.getPosition().getValueAsDouble() * 360.0;
    }
}