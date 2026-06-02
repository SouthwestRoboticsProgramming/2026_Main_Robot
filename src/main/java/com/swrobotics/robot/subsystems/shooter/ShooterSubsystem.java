package com.swrobotics.robot.subsystems.shooter;

import com.ctre.phoenix6.controls.Follower;
import com.ctre.phoenix6.controls.VelocityVoltage;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.MotorAlignmentValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import com.swrobotics.lib.ctre.TalonFXConfigHelper;
import com.swrobotics.robot.config.IOAllocation;
import com.swrobotics.robot.control.AimCalc;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class ShooterSubsystem extends SubsystemBase {
    public enum State { 
        IDLE, 
        SHOOT, 
        WARM, 
        RINDEX, 
        AUTO, 
        PASS 
    }

    private final TalonFX x60L1;
    private final TalonFX x60L2;
    private final TalonFX x60R;
    
    private final VelocityVoltage velocityControl = new VelocityVoltage(0).withEnableFOC(true).withUpdateFreqHz(50);
    private State targetState = State.IDLE;
    private double currentMotorTargetRPS = 0.0;

    public ShooterSubsystem() {
        x60L1 = IOAllocation.CAN.kShooterL.createTalonFX();
        x60L2 = IOAllocation.CAN.kShooterL2.createTalonFX();
        x60R = IOAllocation.CAN.kShooterR.createTalonFX();

        TalonFXConfigHelper config = new TalonFXConfigHelper();
        config.MotorOutput.Inverted = InvertedValue.CounterClockwise_Positive;
        config.MotorOutput.NeutralMode = NeutralModeValue.Coast;
        config.CurrentLimits.StatorCurrentLimit = 120; // Good for Krakens, keeps torque high
        config.CurrentLimits.StatorCurrentLimitEnable = true;
        
        // TUNE THESE: 3.9 lbs drum will need heavy feedforward
        config.Slot0.kS = 0.3;
        config.Slot0.kV = 0.12; 
        config.Slot0.kA = 0.05; // Raised slightly to help punch through the heavy inertia 
        config.Slot0.kP = 0.4; 
        config.Slot0.kD = 0.01;

        config.apply(x60L1, x60L2, x60R);
        x60R.setControl(new Follower(x60L1.getDeviceID(), MotorAlignmentValue.Opposed));
        x60L2.setControl(new Follower(x60L1.getDeviceID(), MotorAlignmentValue.Aligned));

        setDefaultCommand(commandSetState(State.IDLE));
    }

    @Override
    public void periodic() {
        double wheelRps = 0;
        switch (targetState) {
            case IDLE: wheelRps = 0; break;
            case SHOOT: wheelRps = 40; break;
            case WARM: wheelRps = 30; break;
            case RINDEX: wheelRps = -20; break;
            case AUTO: 
                AimCalc.getInstance().setPassingMode(false);
                wheelRps = AimCalc.getInstance().getShooterRPS(); 
                break;
            case PASS: 
                AimCalc.getInstance().setPassingMode(true);
                wheelRps = AimCalc.getInstance().getShooterRPS(); 
                break;
        }

        // 1 to 2 step up: Motor spins 0.5 times for every 1 wheel rotation
        currentMotorTargetRPS = wheelRps / 2.0;
        x60L1.setControl(velocityControl.withVelocity(currentMotorTargetRPS));

        SmartDashboard.putNumber("Shooter/Wheel Target RPS", wheelRps);
        SmartDashboard.putNumber("Shooter/Motor Target RPS", currentMotorTargetRPS);
        SmartDashboard.putNumber("Shooter/Motor Velocity RPS", x60L1.getVelocity().getValueAsDouble());
        SmartDashboard.putNumber("Shooter/Wheel Actual RPS", x60L1.getVelocity().getValueAsDouble() * 2.0);
        SmartDashboard.putBoolean("Shooter/At Speed", isAtTargetRPS());
        SmartDashboard.putString("Shooter/State", targetState.name());
    }

    public boolean isAtTargetRPS() {
        if (targetState != State.SHOOT && targetState != State.AUTO && targetState != State.PASS) return false;
        // Require it to be within 2 RPS of the motor target
        return Math.abs(x60L1.getVelocity().getValueAsDouble() - currentMotorTargetRPS) < 2.0;
    }

    public Command commandSetState(State state) { return run(() -> this.targetState = state); }
}