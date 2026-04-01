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
    public static final double kGearRatio = 3.0; // 3:1 Motor to Wheel

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
        config.Slot0.kP = 0.2;
        config.Slot0.kV = 0.13;

        config.apply(motorL, motorR);
        motorR.setControl(new Follower(motorL.getDeviceID(), MotorAlignmentValue.Opposed));
        setDefaultCommand(commandSetState(State.IDLE));
    }

    @Override
    public void periodic() {
        double wheelRps = 0;
        switch (targetState) {
            case IDLE: wheelRps = 0; break;
            case SHOOT: wheelRps = 60; break;
            case WARM: wheelRps = 30; break;
            case RINDEX: wheelRps = -20; break;
            case AUTO: wheelRps = AimCalc.getInstance().getShooterRPS(false); break;
            case PASS: wheelRps = AimCalc.getInstance().getShooterRPS(true); break;
        }

        // Apply 3:1 ratio (Motor needs to spin 1/3 speed of wheel output)
        currentMotorTargetRPS = wheelRps / kGearRatio;
        motorL.setControl(velocityControl.withVelocity(currentMotorTargetRPS));

        SmartDashboard.putNumber("Shooter/Wheel Target RPS", wheelRps);
        SmartDashboard.putBoolean("Shooter/At Speed", isAtTargetRPS());
    }

    public boolean isAtTargetRPS() {
        // FIXED BUG: Changed || to &&. Now correctly checks if we are in a shooting state.
        if (targetState != State.SHOOT && targetState != State.AUTO && targetState != State.PASS) return false;
        return Math.abs(motorL.getVelocity().getValueAsDouble() - currentMotorTargetRPS) < 1.0;
    }

    public Command commandSetState(State state) { return run(() -> this.targetState = state); }
}