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
    public enum State { IDLE, SHOOT, WARM, RINDEX, AUTO, PASS }

    private final TalonFX motorL;
    private final TalonFX motorR;
    private final VelocityVoltage velocityControl = new VelocityVoltage(0).withEnableFOC(true);

    private State targetState = State.IDLE;
    private double currentMotorTargetRPS = 0.0;

    public ShooterSubsystem() {
        motorL = IOAllocation.CAN.kShooterL.createTalonFX();
        motorR = IOAllocation.CAN.kShooterR.createTalonFX();

        TalonFXConfigHelper config = new TalonFXConfigHelper();
        config.MotorOutput.Inverted = InvertedValue.CounterClockwise_Positive;
        config.MotorOutput.NeutralMode = NeutralModeValue.Coast;
        config.CurrentLimits.StatorCurrentLimit = 80;
        config.CurrentLimits.StatorCurrentLimitEnable = true;
        config.Slot0.kP = 0.2;
        config.Slot0.kV = 0.13;

        config.apply(motorL, motorR);
        motorR.setControl(new Follower(motorL.getDeviceID(), MotorAlignmentValue.Opposed));
        setDefaultCommand(commandSetState(State.WARM));
    }

    @Override
    public void periodic() {
        double wheelRps = 0;
        switch (targetState) {
            case IDLE: wheelRps = 0; break;
            case SHOOT: wheelRps = 20; break;
            case WARM: wheelRps = 7.5; break;
            case RINDEX: wheelRps = -10; break;
            case AUTO: wheelRps = AimCalc.getInstance().getShooterRPS(false)/3; break;
            case PASS: wheelRps = AimCalc.getInstance().getShooterRPS(true)/3; break;
        }

        currentMotorTargetRPS = wheelRps ;
        motorL.setControl(velocityControl.withVelocity(currentMotorTargetRPS));

        SmartDashboard.putNumber("Shooter/Wheel Target RPS", wheelRps * 3);
        SmartDashboard.putBoolean("Shooter/At Speed", isAtTargetRPS());
    }

    public boolean isAtTargetRPS() {
        if (targetState != State.SHOOT && targetState != State.AUTO && targetState != State.PASS) return false;
        return Math.abs(motorL.getVelocity().getValueAsDouble() - currentMotorTargetRPS) < 1.0;
    }

    public Command commandSetState(State state) { return run(() -> this.targetState = state); }
}