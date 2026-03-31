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
    public enum State { IDLE, SHOOT, WARM, RINDEX, AUTO }

    private final TalonFX motorL;
    private final TalonFX motorR;
    private final VelocityVoltage velocityControl = new VelocityVoltage(0).withEnableFOC(true);

    private State targetState = State.IDLE;
    private double currentTargetRPS = 0.0;

    // Manual shooter RPS setpoint (TUNING: starting value, step size)
    private double manualRps = 20.0;
    public static double kManualRpsStep = 2.0;

    public ShooterSubsystem() {
        motorL = IOAllocation.CAN.kShooterL.createTalonFX();
        motorR = IOAllocation.CAN.kShooterR.createTalonFX();

        TalonFXConfigHelper config = new TalonFXConfigHelper();
        config.MotorOutput.Inverted = InvertedValue.CounterClockwise_Positive;
        config.MotorOutput.NeutralMode = NeutralModeValue.Coast;

        // TUNING: PID/FF constants
        config.Slot0.kP = 0.2;
        config.Slot0.kV = 0.13;

        config.apply(motorL, motorR);

        motorR.setControl(new Follower(motorL.getDeviceID(), MotorAlignmentValue.Opposed));
        setDefaultCommand(commandSetState(State.IDLE));
    }

    @Override
    public void periodic() {
        switch (targetState) {
            case IDLE:
                currentTargetRPS = 0.0;
                break;

            case SHOOT:
                currentTargetRPS = 20;
                break;
            case WARM:
                currentTargetRPS = 10.0;
                break;
            case RINDEX:
                currentTargetRPS = -10.0; 
                break;
            case AUTO:
                currentTargetRPS = AimCalc.getInstance().getShooterRPS()/3.0;
                break;
        }

        motorL.setControl(velocityControl.withVelocity(currentTargetRPS));

        SmartDashboard.putBoolean("Shooter/Flywheel At Target", isAtTargetRPS());
        SmartDashboard.putNumber("Shooter/Flywheel Target RPS", currentTargetRPS);
        SmartDashboard.putNumber("Shooter/Flywheel Actual RPS", motorL.getVelocity().getValueAsDouble());
    }

    public boolean isAtTargetRPS() {
        if (targetState != State.SHOOT) return false;
        return Math.abs(motorL.getVelocity().getValueAsDouble() - currentTargetRPS) < 1.0;
    }

    public void setTargetState(State state) {
        this.targetState = state;
    }

    public Command commandSetState(State state) {
        return run(() -> setTargetState(state));
    }

    public Command manualNudgeRPS(double rpsDelta) {
    return runOnce(() -> {
        this.targetState = State.AUTO; // Ensure you added a MANUAL state to your Enum
        this.manualRps += rpsDelta;
    });
}

}