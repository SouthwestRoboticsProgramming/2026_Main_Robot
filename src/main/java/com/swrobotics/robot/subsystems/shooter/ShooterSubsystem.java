package com.swrobotics.robot.subsystems.shooter;

import com.ctre.phoenix6.controls.Follower;
import com.ctre.phoenix6.controls.VelocityVoltage;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.MotorAlignmentValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import com.swrobotics.lib.ctre.TalonFXConfigHelper;
import com.swrobotics.robot.config.IOAllocation;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class ShooterSubsystem extends SubsystemBase {
    public enum State {
        IDLE(0), SHOOT(60), WARM(15), RINDEX(-10); 
        public final double rps;
        State(double rps) { this.rps = rps; }
    }
    
    private final TalonFX motorL;
    private final TalonFX motorR;
    private final VelocityVoltage velocityControl = new VelocityVoltage(0);
    private State targetState = State.IDLE;
    
    private double dynamicRPS = 0;
    private boolean useDynamic = false;

    public ShooterSubsystem() {
        motorL = IOAllocation.CAN.kShooterL.createTalonFX();
        motorR = IOAllocation.CAN.kShooterR.createTalonFX();
        TalonFXConfigHelper config = new TalonFXConfigHelper();
        config.Feedback.SensorToMechanismRatio = 1.0 / 3.0; 
        config.MotorOutput.Inverted = InvertedValue.CounterClockwise_Positive;
        config.MotorOutput.NeutralMode = NeutralModeValue.Coast;
        config.Slot0.kP = 0.5; 
        config.Slot0.kV = 0.13; 
        config.apply(motorL, motorR);
        motorR.setControl(new Follower(motorL.getDeviceID(), MotorAlignmentValue.Opposed));
    }

    @Override
    public void periodic() {
        double setpoint = useDynamic ? dynamicRPS : targetState.rps;
        motorL.setControl(velocityControl.withVelocity(setpoint));
    }

    public void setDynamicRPS(double rps) {
        this.dynamicRPS = rps;
        this.useDynamic = true;
    }

    public void stopDynamic() { this.useDynamic = false; }

    public boolean isAtTargetRPS() {
        double target = useDynamic ? dynamicRPS : targetState.rps;
        if (target <= 0) return false;
        return Math.abs(motorL.getVelocity().getValueAsDouble() - target) < 2.0;
    }

    public void setTargetState(State state) { this.targetState = state; }
    public Command commandSetState(State state) { return run(() -> setTargetState(state)); }
}