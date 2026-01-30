package com.swrobotics.robot.subsystems.Hood;

import com.ctre.phoenix6.configs.MotionMagicConfigs;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.controls.MotionMagicVoltage;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import com.swrobotics.lib.ctre.TalonFXConfigHelper;
import com.swrobotics.robot.config.Constants;
import com.swrobotics.robot.config.IOAllocation;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class HoodSubsystem extends SubsystemBase {
    public enum State {
        GO_TO_START,
        AUTO_TRACKING,
        MANUAL_MODE
    }

    private final TalonFX motor;
    private final MotionMagicVoltage motionMagic = new MotionMagicVoltage(0).withSlot(0);
    private State targetState = State.GO_TO_START;

    private double manualTargetRotations = 0.0; // Motor rotations
    private double currentHoodAngle = 0.0;      // Degrees (approx)
    private Pose2d robotPose;
    private Rotation2d hubAngle;

    // Shooter speed the hood model will use
    private double shooterTargetRPS = Constants.kShooterRPS.get();

    public HoodSubsystem() {
        motor = IOAllocation.CAN.kHoodMotor.createTalonFX();

        TalonFXConfigHelper config = new TalonFXConfigHelper();
        config.MotorOutput.Inverted =
                Constants.kHoodInverted.get()
                        ? InvertedValue.CounterClockwise_Positive
                        : InvertedValue.Clockwise_Positive;
        config.MotorOutput.NeutralMode = NeutralModeValue.Brake;

        // Hood-specific PID gains. we will need to tune these
        Slot0Configs gains = new Slot0Configs();
        gains.withKP(6.0);
        gains.withKI(0.0);
        gains.withKD(0.8);
        gains.withKV(0.1);
        config.Slot0 = gains;

        // Motion Magic configuration for smooth movement
        MotionMagicConfigs mmConfig = new MotionMagicConfigs();
        mmConfig.MotionMagicCruiseVelocity = Constants.kHoodCruiseVelocity.get();
        mmConfig.MotionMagicAcceleration = Constants.kHoodAcceleration.get();
        mmConfig.MotionMagicJerk = 2000.0;
        config.MotionMagic = mmConfig;

        config.apply(motor);
        motor.setPosition(0); // Zero at origin but we should use cancoders to set an absolute position and not hope we are at 0 on boot
    }

    @Override
    public void periodic() {
        // assuming 1 to 1 gear ratio. will need to be changed if not
        currentHoodAngle = Math.toDegrees(motor.getPosition().getValueAsDouble());

        double maxAngle = Constants.kHoodMaxAngle.get();
        double minAngle = Constants.kHoodMinAngle.get();
        double maxRotations = maxAngle / 360.0;
        double minRotations = minAngle / 360.0;

        switch (targetState) {
            case GO_TO_START:
                manualTargetRotations = 0.0; // Home to center
                break;

            case AUTO_TRACKING://TODO: cameron put your auto hood code logic here.
                // Auto compute hood angle from field-relative pose + shooter RPS
                if (robotPose != null) {
                    Pose2d hubPose = Constants.kHubPose;
                    double dx = hubPose.getX() - robotPose.getX();
                    double dy = hubPose.getY() - robotPose.getY();
                    double distance = Math.hypot(dx, dy); // meters

                    // Store angle to hub for other subsystems if needed
                    hubAngle = new Rotation2d(Math.atan2(dy, dx));

                    double a0 = Constants.kA0.get();
                    double a1 = Constants.kA1.get();
                    double a2 = Constants.kA2.get();
                    double aRps = Constants.kA_Rps.get();

                    double desiredAngleDeg =
                            a0
                                    + a1 * distance
                                    + a2 * distance * distance
                                    + aRps * shooterTargetRPS;

                    // Clamp to mechanical limits
                    desiredAngleDeg =
                            Math.max(minAngle, Math.min(maxAngle, desiredAngleDeg));

                    // Convert to rotations for TalonFX
                    double desiredRotations = desiredAngleDeg / 360.0;

                    manualTargetRotations =
                            Math.max(minRotations,
                                    Math.min(maxRotations, desiredRotations));
                }
                break;

            case MANUAL_MODE:
                manualTargetRotations =
                        Math.max(minRotations,
                                Math.min(maxRotations, manualTargetRotations));
                break;
        }

        motor.setControl(motionMagic.withPosition(manualTargetRotations));
    }

    // ===== Public API =====

    public void setTargetState(State state) {
        this.targetState = state;
    }

    public State getCurrentState() {
        return targetState;
    }

    public double getCurrentAngleDegrees() {
        return currentHoodAngle;
    }

    public Rotation2d getHubAngle() {
        return hubAngle;
    }

    public void setRobotPose(Pose2d pose) {
        this.robotPose = pose;
    }

    // Call this from your shooter subsystem or container to synchronize with kShooterRPS
    public void setShooterTargetRPS(double rps) {
        this.shooterTargetRPS = rps;
    }

    public void setManualPosition(double rotations) {
        this.manualTargetRotations = rotations;
    }

    // ===== Commands =====

    public Command commandSetState(State state) {
        return Commands.runOnce(() -> setTargetState(state), this);
    }

    public Command commandHome() {
        return Commands.runOnce(() -> setTargetState(State.GO_TO_START), this);
    }

    public Command commandToggleAutoManual() {
        return Commands.runOnce(() -> {
            if (targetState == State.AUTO_TRACKING) {
                setTargetState(State.MANUAL_MODE);
            } else {
                setTargetState(State.AUTO_TRACKING);
            }
        }, this);
    }

    public Command commandManualLeft() {
        return Commands.runOnce(() -> {
            setTargetState(State.MANUAL_MODE);
            double newAngle = getCurrentAngleDegrees() - 5.0;
            double newPos = Math.max(
                    Constants.kHoodMinAngle.get() / 360.0,
                    newAngle / 360.0);
            setManualPosition(newPos);
        }, this);
    }

    public Command commandManualRight() {
        return Commands.runOnce(() -> {
            setTargetState(State.MANUAL_MODE);
            double newAngle = getCurrentAngleDegrees() + 5.0;
            double newPos = Math.min(
                    Constants.kHoodMaxAngle.get() / 360.0,
                    newAngle / 360.0);
            setManualPosition(newPos);
        }, this);
    }

    public Command commandManualJog(double degrees) {
        return Commands.runOnce(() -> {
            setTargetState(State.MANUAL_MODE);
            double newAngle = getCurrentAngleDegrees() + degrees;
            double minAngle = Constants.kHoodMinAngle.get();
            double maxAngle = Constants.kHoodMaxAngle.get();
            newAngle = Math.max(minAngle, Math.min(maxAngle, newAngle));
            double newPos = newAngle / 360.0;
            setManualPosition(newPos);
        }, this);
    }
}
